from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

import cv2
import numpy as np

from grbl import Grbl


@dataclass
class WorkflowMetrics:
    started_at: float = 0.0
    finished_at: float = 0.0
    points_visited: int = 0
    mean_error_mm: float = 0.0
    last_error_mm: float = 0.0
    running: bool = False
    last_message: str = ""

    @property
    def elapsed_s(self) -> float:
        if not self.started_at:
            return 0.0
        end = self.finished_at if self.finished_at else time.time()
        return max(0.0, end - self.started_at)


@dataclass
class WorkflowState:
    start_pos: Optional[Tuple[float, float, float]] = None
    visited: List[Tuple[float, float]] = field(default_factory=list)
    decoded_chain: List[str] = field(default_factory=list)
    metrics: WorkflowMetrics = field(default_factory=WorkflowMetrics)


def parse_pos(text: str) -> Optional[Tuple[float, float]]:
    """Parse 'xxx,xx:yyy,yy' (comma decimal allowed)."""
    raw = text.strip()
    if raw.upper() == "END":
        return None

    if ":" not in raw:
        raise ValueError("Expected format xxx,xx:yyy,yy")

    xs, ys = raw.split(":", 1)
    xs = xs.strip().replace(",", ".")
    ys = ys.strip().replace(",", ".")
    return float(xs), float(ys)


def decode_qr(gray: np.ndarray) -> str:
    text, _, _ = _QR_DETECTOR.detectAndDecode(gray)
    return (text or "").strip()


_QR_DETECTOR = cv2.QRCodeDetector()


def _encode_jpeg(frame_bgr: np.ndarray, quality: int = 80) -> bytes:
    ok, buf = cv2.imencode(".jpg", frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        return b""
    return buf.tobytes()


def _to_gray(frame: np.ndarray) -> np.ndarray:
    if frame.ndim == 2:
        return frame
    return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


def _focus_score_laplacian_var(frame: np.ndarray) -> float:
    """Fast sharpness metric (higher => sharper).

    Center ROI + optional downsample + variance of Laplacian.
    """
    if frame is None:
        return 0.0
    gray = _to_gray(frame)
    h, w = gray.shape[:2]
    if h < 10 or w < 10:
        return 0.0

    x0 = int(w * 0.25)
    x1 = int(w * 0.75)
    y0 = int(h * 0.25)
    y1 = int(h * 0.75)
    roi = gray[y0:y1, x0:x1]
    if roi.size == 0:
        roi = gray

    rh, rw = roi.shape[:2]
    target_w = 360
    if rw > target_w:
        scale = target_w / float(rw)
        roi = cv2.resize(roi, (target_w, max(1, int(rh * scale))), interpolation=cv2.INTER_AREA)

    lap = cv2.Laplacian(roi, cv2.CV_64F)
    return float(lap.var())


class QrWorkflow:
    def __init__(
        self,
        grbl: Grbl,
        get_frame: Callable[[], Optional[np.ndarray]],
        qr_retry_seconds: float = 8.0,
    ):
        self._grbl = grbl
        self._get_frame = get_frame
        self._qr_retry_seconds = float(qr_retry_seconds)

        self._state = WorkflowState()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self._err_sum = 0.0
        self._err_n = 0

        self._capture_lock = threading.Lock()
        self._last_capture_jpeg: bytes = b""
        self._capture_id: int = 0

        self._focus_lock = threading.Lock()
        self._focus_threshold: float = 0.0  # 0 => auto
        self._focus_ref_score: float = 0.0
        self._last_good_z: Optional[float] = None

    def get_last_capture_jpeg(self) -> tuple[bytes, int]:
        with self._capture_lock:
            return self._last_capture_jpeg, self._capture_id

    def set_focus_threshold(self, threshold: float) -> None:
        t = float(threshold)
        if not (t >= 0.0 and t < 1e12):
            raise ValueError("focus threshold must be >= 0")
        with self._focus_lock:
            self._focus_threshold = t

    def get_focus_threshold(self) -> float:
        with self._focus_lock:
            return float(self._focus_threshold)

    def _set_last_capture(self, frame: np.ndarray) -> None:
        if frame is None:
            return
        if frame.ndim == 2:
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            frame_bgr = frame
        jpeg = _encode_jpeg(frame_bgr, quality=80)
        if not jpeg:
            return
        with self._capture_lock:
            self._last_capture_jpeg = jpeg
            self._capture_id += 1

    @property
    def state(self) -> WorkflowState:
        return self._state

    def set_start_pos_from_machine(self) -> Tuple[float, float]:
        pos = self._grbl.get_position_mpos()
        if pos is None:
            raise RuntimeError("No GRBL position available")
        self._state.start_pos = (pos[0], pos[1], pos[2])
        return pos[0], pos[1]

    def run_async(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _set_msg(self, msg: str) -> None:
        self._state.metrics.last_message = msg

    def _run(self) -> None:
        st = self._state
        st.metrics = WorkflowMetrics(started_at=time.time(), running=True, last_message="Starting")
        st.visited.clear()
        st.decoded_chain.clear()
        self._err_sum = 0.0
        self._err_n = 0

        try:
            if st.start_pos is None:
                raise RuntimeError("Startposition ist nicht gesetzt")

            start_x, start_y, start_z = st.start_pos

            self._set_msg("Homing")
            self._grbl.homing()

            self._set_msg("Fahre zur Startposition")
            self._grbl.move_abs_xyz(start_x, start_y, start_z)
            self._grbl.wait_until_idle(timeout=60.0)

            last_code = ""

            while not self._stop.is_set():
                self._set_msg("QR lesen")
                code = self._read_qr_with_focus_retry(exclude_code=last_code)
                if code == "":
                    raise RuntimeError("Kein QR-Code erkannt")

                last_code = code

                st.decoded_chain.append(code)

                if code.strip().upper() == "END":
                    self._set_msg("END erkannt")
                    break

                target = parse_pos(code)
                if target is None:
                    break

                x, y = target
                self._set_msg(f"Fahre zu {x:.2f}, {y:.2f}")
                self._grbl.move_abs_xyz(x, y, start_z)
                self._grbl.wait_until_idle(timeout=90.0)

                # error metric
                pos = self._grbl.get_position_xy()
                if pos is not None:
                    err = math.hypot(pos[0] - x, pos[1] - y)
                    st.metrics.last_error_mm = err
                    self._err_sum += err
                    self._err_n += 1
                    st.metrics.mean_error_mm = self._err_sum / max(1, self._err_n)

                st.visited.append((x, y))
                st.metrics.points_visited = len(st.visited)

                # at new position read next QR in next loop

            self._set_msg("Homing (Ende)")
            self._grbl.homing()

        except Exception as exc:
            self._set_msg(f"Fehler: {exc}")
        finally:
            st.metrics.running = False
            st.metrics.finished_at = time.time()

    def _wait_until_idle_fast(self, timeout: float = 2.0) -> None:
        deadline = time.time() + float(timeout)
        while time.time() < deadline and not self._stop.is_set():
            try:
                st = self._grbl.query_status()
                if (st.state or "").lower() == "idle":
                    return
            except Exception:
                pass
            time.sleep(0.03)

    def _try_decode_once(self, exclude: str) -> str:
        frame = self._get_frame()
        if frame is None:
            return ""

        try:
            code = decode_qr(_to_gray(frame))
        except Exception:
            return ""

        if not code:
            return ""
        if exclude and code.strip() == exclude:
            return ""

        try:
            self._set_last_capture(frame)
        except Exception:
            pass

        try:
            score = _focus_score_laplacian_var(frame)
        except Exception:
            score = 0.0

        try:
            mpos = self._grbl.get_position_mpos()
        except Exception:
            mpos = None

        with self._focus_lock:
            if self._focus_ref_score <= 0.0 and score > 0.0:
                self._focus_ref_score = score
            if mpos is not None and len(mpos) >= 3:
                self._last_good_z = float(mpos[2])

        return code

    def _read_qr_with_focus_retry(self, exclude_code: str = "") -> str:
        """Read QR with minimal overhead, focusing only when necessary."""
        deadline = time.time() + self._qr_retry_seconds
        exclude = (exclude_code or "").strip()

        # Fast path: decode without any Z motion.
        code = self._try_decode_once(exclude)
        if code:
            return code

        # Only consider focusing if a threshold is configured (absolute or auto with reference).
        with self._focus_lock:
            threshold = float(self._focus_threshold)
            ref = float(self._focus_ref_score)
            last_good_z = self._last_good_z

        if threshold <= 0.0 and ref > 0.0:
            threshold = 0.65 * ref

        if threshold > 0.0:
            try:
                mpos = self._grbl.get_position_mpos()
            except Exception:
                mpos = None

            if mpos is not None and len(mpos) >= 3:
                x, y, z = float(mpos[0]), float(mpos[1]), float(mpos[2])
                z_center = float(last_good_z) if last_good_z is not None else z

                try:
                    frame0 = self._get_frame()
                    score0 = _focus_score_laplacian_var(frame0) if frame0 is not None else 0.0
                except Exception:
                    score0 = 0.0

                if score0 < threshold:
                    code2 = self._focus_hill_climb_and_decode(
                        x=x,
                        y=y,
                        z_center=z_center,
                        threshold=threshold,
                        exclude=exclude,
                        deadline=deadline,
                    )
                    if code2:
                        return code2

        # Keep trying (no more Z moves).
        while time.time() < deadline and not self._stop.is_set():
            code = self._try_decode_once(exclude)
            if code:
                return code
            time.sleep(0.12)
        return ""

    def _focus_hill_climb_and_decode(
        self,
        x: float,
        y: float,
        z_center: float,
        threshold: float,
        exclude: str,
        deadline: float,
    ) -> str:
        """Very small Z search: few steps, early exit when QR decodes."""
        z_range = 0.6
        dz_coarse = 0.20
        dz_fine = 0.05
        max_moves = 5
        settle_s = 0.08

        z_min = float(z_center) - z_range
        z_max = float(z_center) + z_range

        def clamp(zv: float) -> float:
            return max(z_min, min(z_max, float(zv)))

        def goto(zv: float) -> float:
            zt = clamp(zv)
            self._grbl.move_abs_xyz(x, y, zt)
            self._wait_until_idle_fast(timeout=2.0)
            time.sleep(settle_s)
            return zt

        def eval_at_current() -> tuple[float, str]:
            if time.time() >= deadline or self._stop.is_set():
                return 0.0, ""
            frame = self._get_frame()
            if frame is None:
                return 0.0, ""

            # Decode first (fastest success criterion)
            try:
                code = decode_qr(_to_gray(frame))
            except Exception:
                code = ""

            if code:
                if exclude and code.strip() == exclude:
                    code = ""
                else:
                    try:
                        self._set_last_capture(frame)
                    except Exception:
                        pass
                    try:
                        score = _focus_score_laplacian_var(frame)
                    except Exception:
                        score = 0.0
                    with self._focus_lock:
                        if self._focus_ref_score <= 0.0 and score > 0.0:
                            self._focus_ref_score = score
                        self._last_good_z = None
                        try:
                            mp = self._grbl.get_position_mpos()
                            if mp is not None and len(mp) >= 3:
                                self._last_good_z = float(mp[2])
                        except Exception:
                            self._last_good_z = None
                    return score, code

            try:
                score = _focus_score_laplacian_var(frame)
            except Exception:
                score = 0.0
            return score, ""

        # Evaluate center
        try:
            z0 = goto(z_center)
        except Exception:
            z0 = clamp(z_center)
        best_z = z0
        best_s, code0 = eval_at_current()
        if code0:
            return code0
        if best_s >= threshold:
            with self._focus_lock:
                self._last_good_z = best_z
                if self._focus_ref_score <= 0.0 and best_s > 0.0:
                    self._focus_ref_score = best_s
            return ""

        # Probe +/- coarse step
        sp = sm = 0.0
        try:
            zp = goto(z0 + dz_coarse)
            sp, cp = eval_at_current()
            if cp:
                return cp
            if sp > best_s:
                best_s, best_z = sp, zp
        except Exception:
            pass

        try:
            zm = goto(z0 - dz_coarse)
            sm, cm = eval_at_current()
            if cm:
                return cm
            if sm > best_s:
                best_s, best_z = sm, zm
        except Exception:
            pass

        if best_s >= threshold:
            try:
                goto(best_z)
            except Exception:
                pass
            with self._focus_lock:
                self._last_good_z = best_z
                if self._focus_ref_score <= 0.0 and best_s > 0.0:
                    self._focus_ref_score = best_s
            return ""

        direction = 1.0 if sp >= sm else -1.0
        z = best_z
        moves = 0
        while moves < max_moves and time.time() < deadline and not self._stop.is_set():
            z_next = clamp(z + direction * dz_coarse)
            if abs(z_next - z) < 1e-6:
                break
            try:
                z_next = goto(z_next)
            except Exception:
                break
            s, c = eval_at_current()
            if c:
                return c
            moves += 1
            if s > best_s + 1e-6:
                best_s, best_z = s, z_next
                z = z_next
                if best_s >= threshold:
                    break
            else:
                break

        # Fine adjustment around best
        for z_try in (best_z + dz_fine, best_z - dz_fine):
            if time.time() >= deadline or self._stop.is_set():
                break
            try:
                zt = goto(z_try)
            except Exception:
                continue
            s, c = eval_at_current()
            if c:
                return c
            if s > best_s:
                best_s, best_z = s, zt

        try:
            goto(best_z)
        except Exception:
            pass
        with self._focus_lock:
            self._last_good_z = best_z
            if self._focus_ref_score <= 0.0 and best_s > 0.0:
                self._focus_ref_score = best_s
        return ""

    def _read_qr_with_retry(self, exclude_code: str = "") -> str:
        deadline = time.time() + self._qr_retry_seconds
        exclude = (exclude_code or "").strip()
        while time.time() < deadline and not self._stop.is_set():
            frame = self._get_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            gray = _to_gray(frame)

            code = decode_qr(gray)
            if code:
                if exclude and code.strip() == exclude:
                    time.sleep(0.05)
                    continue
                # Store the captured image that produced this QR for UI display during runs.
                try:
                    self._set_last_capture(frame)
                except Exception:
                    pass
                return code
            time.sleep(0.15)
        return ""
