from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

import random
import math

import cv2
import numpy as np

from grbl import GrblStatus


@dataclass
class DemoCameraInfo:
    model: str = "DemoCam"
    serial: str = "DEMO"
    pixel_format: str = "BGR8"


class DemoScript:
    """Holds a deterministic QR chain for the demo."""

    def __init__(
        self,
        points: int = 12,
        span_x_mm: float = 60.0,
        span_y_mm: float = 40.0,
        tol_mm: float = 0.5,
        seed: Optional[int] = None,
    ):
        self._points = int(points)
        self._span_x = float(span_x_mm)
        self._span_y = float(span_y_mm)
        self._tol = float(tol_mm)

        self._rng = random.Random(seed if seed is not None else time.time_ns())

        self._codes: List[str] = ["END"]
        self._idx = 0
        self._lock = threading.Lock()
        self._start_pos: Optional[Tuple[float, float, float]] = None

    def reset(self) -> None:
        with self._lock:
            self._idx = 0

    def set_span(self, span_x_mm: float, span_y_mm: float) -> None:
        """Update demo work area (used for random point generation)."""
        with self._lock:
            self._span_x = float(span_x_mm)
            self._span_y = float(span_y_mm)
            if self._start_pos is not None:
                self._idx = 0
                self._codes = self._generate_codes(self._start_pos)

    def set_start_pos(self, pos: Tuple[float, float, float]) -> None:
        with self._lock:
            self._start_pos = pos
            self._idx = 0
            self._codes = self._generate_codes(pos)

    def _fmt_num(self, v: float) -> str:
        # QR spec uses comma decimal in the main app
        return f"{v:.2f}".replace(".", ",")

    def _generate_codes(self, start: Tuple[float, float, float]) -> List[str]:
        _sx, _sy, _sz = start
        codes: List[str] = []
        # generate points inside the configured work area
        for _ in range(max(1, self._points)):
            x = self._rng.uniform(0.0, max(0.1, self._span_x))
            y = self._rng.uniform(0.0, max(0.1, self._span_y))
            codes.append(f"{self._fmt_num(x)}:{self._fmt_num(y)}")
        codes.append("END")
        return codes

    def _parse_xy(self, code: str) -> Optional[Tuple[float, float]]:
        c = code.strip().upper()
        if c == "END":
            return None
        if ":" not in code:
            return None
        xs, ys = code.split(":", 1)
        xs = xs.strip().replace(",", ".")
        ys = ys.strip().replace(",", ".")
        try:
            return float(xs), float(ys)
        except Exception:
            return None

    def current_code(self) -> str:
        with self._lock:
            return self._codes[min(self._idx, len(self._codes) - 1)]

    def on_move_completed(self, new_pos: Tuple[float, float, float]) -> None:
        """Advance QR code when the current target is reached (within tolerance)."""
        with self._lock:
            if self._idx >= len(self._codes) - 1:
                return

            target = self._parse_xy(self._codes[self._idx])
            if target is None:
                return

            x, y, _z = new_pos
            tx, ty = target
            if math.hypot(x - tx, y - ty) <= self._tol:
                self._idx += 1


class DemoCamera:
    """Synthetic camera that renders a QR code into the frame."""

    def __init__(self, fps: float = 10.0, code_provider: Optional[Callable[[], str]] = None):
        self._fps = float(fps)
        self._code_provider = code_provider or (lambda: "DEMO")

        self._info = DemoCameraInfo()
        self._connected = True
        self._last_error = ""
        self._last_frame_at = 0.0

        self._lock = threading.Lock()
        self._latest: Optional[np.ndarray] = None
        self._frame_id: int = 0

        self._qr_cache_lock = threading.Lock()
        self._qr_cache: dict[str, np.ndarray] = {}

        self._feature_lock = threading.Lock()
        # A small set of pseudo "neoAPI-like" features for the UI.
        self._features = {
            "ExposureTime": {
                "interface": "IFloat",
                "value": 10000.0,
                "min": 50.0,
                "max": 50000.0,
                "inc": 50.0,
                "unit": "us",
                "enum_values": None,
                "description": "Demo: beeinflusst Helligkeit",
            },
            "Gain": {
                "interface": "IFloat",
                "value": 1.0,
                "min": 0.0,
                "max": 8.0,
                "inc": 0.05,
                "unit": "",
                "enum_values": None,
                "description": "Demo: verstärkt Helligkeit",
            },
            "Gamma": {
                "interface": "IFloat",
                "value": 1.0,
                "min": 0.2,
                "max": 3.0,
                "inc": 0.05,
                "unit": "",
                "enum_values": None,
                "description": "Demo: Gamma-Korrektur",
            },
            "ReverseX": {
                "interface": "IBoolean",
                "value": False,
                "min": 0,
                "max": 1,
                "inc": 1,
                "unit": "",
                "enum_values": None,
                "description": "Demo: horizontal spiegeln",
            },
            "ReverseY": {
                "interface": "IBoolean",
                "value": False,
                "min": 0,
                "max": 1,
                "inc": 1,
                "unit": "",
                "enum_values": None,
                "description": "Demo: vertikal spiegeln",
            },
            "AcquisitionFrameRate": {
                "interface": "IFloat",
                "value": float(self._fps),
                "min": 1.0,
                "max": 30.0,
                "inc": 0.5,
                "unit": "fps",
                "enum_values": None,
                "description": "Demo: Bildrate",
            },
        }

        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @property
    def info(self) -> DemoCameraInfo:
        return self._info

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def last_error(self) -> str:
        return self._last_error

    @property
    def last_frame_at(self) -> float:
        return self._last_frame_at

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self._thread = None

    def get_latest(self):
        with self._lock:
            if self._latest is None:
                return None, 0.0
            return self._latest.copy(), time.time()

    def get_latest_ref(self):
        with self._lock:
            return self._latest, self._last_frame_at, self._frame_id

    def _loop(self) -> None:
        interval = 1.0 / max(self._fps, 0.1)
        while not self._stop.is_set():
            try:
                code = self._code_provider()
                frame = self._render_frame(code)
                with self._lock:
                    self._latest = frame
                    self._last_frame_at = time.time()
                    self._frame_id += 1
            except Exception as exc:
                self._last_error = str(exc)
            with self._feature_lock:
                try:
                    self._fps = float(self._features["AcquisitionFrameRate"]["value"])
                except Exception:
                    pass
                interval = 1.0 / max(self._fps, 0.1)
            time.sleep(interval)

    def _render_frame(self, code: str) -> np.ndarray:
        h, w = 480, 640
        frame = np.full((h, w, 3), 245, dtype=np.uint8)

        # generate QR (cached)
        qr_bgr: Optional[np.ndarray] = None
        with self._qr_cache_lock:
            qr_bgr = self._qr_cache.get(code)

        if qr_bgr is None:
            try:
                import qrcode  # type: ignore

                qr = qrcode.QRCode(
                    version=2,
                    error_correction=qrcode.constants.ERROR_CORRECT_M,
                    box_size=8,
                    border=2,
                )
                qr.add_data(code)
                qr.make(fit=True)
                img = qr.make_image(fill_color="black", back_color="white")
                qr_np = np.array(img.convert("RGB"))
                qr_bgr = cv2.cvtColor(qr_np, cv2.COLOR_RGB2BGR)

                # resize once for our fixed frame size
                qh, qw = qr_bgr.shape[:2]
                scale = min((w - 40) / qw, (h - 120) / qh, 1.0)
                if scale != 1.0:
                    qr_bgr = cv2.resize(
                        qr_bgr,
                        (max(1, int(qw * scale)), max(1, int(qh * scale))),
                        interpolation=cv2.INTER_NEAREST,
                    )

                with self._qr_cache_lock:
                    if len(self._qr_cache) > 64:
                        self._qr_cache.clear()
                    self._qr_cache[code] = qr_bgr
            except Exception:
                qr_bgr = None

        if isinstance(qr_bgr, np.ndarray):
            # paste centered
            qh, qw = qr_bgr.shape[:2]
            x0 = (w - qw) // 2
            y0 = (h - qh) // 2
            frame[y0 : y0 + qh, x0 : x0 + qw] = qr_bgr

        cv2.putText(frame, "DEMO MODE", (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (20, 20, 20), 2)
        cv2.putText(frame, f"QR: {code}", (14, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (20, 20, 20), 2)
        cv2.putText(frame, time.strftime("%H:%M:%S"), (14, h - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (20, 20, 20), 2)

        # Apply demo "feature" effects so changes are visible live.
        with self._feature_lock:
            exposure = float(self._features["ExposureTime"]["value"])
            gain = float(self._features["Gain"]["value"])
            gamma = float(self._features["Gamma"]["value"])
            revx = bool(self._features["ReverseX"]["value"])
            revy = bool(self._features["ReverseY"]["value"])

        # exposure maps to brightness scale around 10ms baseline
        scale = (exposure / 10000.0) * max(0.01, gain)
        if gamma <= 0:
            gamma = 1.0

        f = frame.astype(np.float32) / 255.0
        f = np.clip(f * scale, 0.0, 1.0)
        f = np.power(f, 1.0 / gamma)
        frame = (np.clip(f, 0.0, 1.0) * 255.0).astype(np.uint8)

        if revx:
            frame = cv2.flip(frame, 1)
        if revy:
            frame = cv2.flip(frame, 0)
        return frame

    def get_settable_features(self) -> list[dict]:
        with self._feature_lock:
            out = []
            for name, meta in self._features.items():
                out.append({"name": name, **meta})
            out.sort(key=lambda f: str(f.get("name", "")))
            return out

    def set_feature(self, name: str, value, execute: bool = False):
        if execute:
            raise RuntimeError("demo camera has no command features")
        key = str(name)
        with self._feature_lock:
            if key not in self._features:
                raise RuntimeError("unknown feature")
            iface = self._features[key]["interface"]
            if iface == "IBoolean":
                self._features[key]["value"] = bool(value)
            elif iface in {"IFloat"}:
                self._features[key]["value"] = float(value)
            else:
                self._features[key]["value"] = value
            return {"name": key, "interface": iface, "value": self._features[key]["value"]}


class DemoGrbl:
    """In-memory GRBL simulator with MPos and jogging."""

    def __init__(self):
        self._lock = threading.Lock()
        self._state = "Idle"
        self._pos = [0.0, 0.0, 0.0]

        self._stop = threading.Event()
        self._jog_dir: Optional[str] = None
        self._jog_step_mm = 0.25
        self._jog_feed = 300.0
        self._jog_thread = threading.Thread(target=self._jog_loop, daemon=True)
        self._jog_thread.start()

        self._on_move_completed: Optional[Callable[[Tuple[float, float, float]], None]] = None

    def set_jog_params(self, step_mm: float, feed_mm_min: float) -> None:
        self._jog_step_mm = float(step_mm)
        self._jog_feed = float(feed_mm_min)

    def set_on_move_completed(self, cb: Callable[[Tuple[float, float, float]], None]) -> None:
        self._on_move_completed = cb

    def connect(self) -> None:
        return

    def close(self) -> None:
        self._stop.set()

    def query_status(self) -> GrblStatus:
        with self._lock:
            return GrblStatus(state=self._state, mpos=tuple(self._pos), wpos=None, raw="<Demo>", updated_at=time.time())

    def get_position_mpos(self) -> Tuple[float, float, float]:
        return self.query_status().mpos  # type: ignore[return-value]

    def get_position_xy(self):
        p = self.get_position_mpos()
        return p[0], p[1]

    def wait_until_idle(self, timeout: float = 30.0) -> None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.query_status().state == "Idle":
                return
            time.sleep(0.05)
        raise TimeoutError("DemoGrbl did not become Idle")

    def get_settings(self, timeout: float = 1.0) -> dict[str, dict]:
        # Demo: no real $$ settings, return empty for UI.
        return {}

    def set_setting(self, key: str, value, timeout: float = 1.0) -> None:
        raise RuntimeError("DemoGrbl does not support $$ settings")

    def homing(self, timeout: float = 5.0) -> None:
        with self._lock:
            self._state = "Home"
        time.sleep(0.2)
        with self._lock:
            self._pos = [0.0, 0.0, 0.0]
            self._state = "Idle"
        if self._on_move_completed:
            self._on_move_completed(tuple(self._pos))

    def move_abs_xy(self, x: float, y: float, feed: Optional[float] = None) -> None:
        self.move_abs_xyz(x, y, None, feed=feed)

    def move_abs_xyz(self, x: float, y: float, z: Optional[float], feed: Optional[float] = None) -> None:
        with self._lock:
            self._state = "Run"
        time.sleep(0.25)
        with self._lock:
            self._pos[0] = float(x)
            self._pos[1] = float(y)
            if z is not None:
                self._pos[2] = float(z)
            self._state = "Idle"
        if self._on_move_completed:
            self._on_move_completed(tuple(self._pos))

    def start_jog(self, direction: str) -> None:
        if direction not in {"up", "down", "left", "right", "zup", "zdown"}:
            raise ValueError("invalid direction")
        self._jog_dir = direction

    def stop_jog(self) -> None:
        self._jog_dir = None

    def _jog_loop(self) -> None:
        while not self._stop.is_set():
            d = self._jog_dir
            if d is None:
                time.sleep(0.02)
                continue

            dx = dy = dz = 0.0
            step = self._jog_step_mm
            if d == "up":
                dy = step
            elif d == "down":
                dy = -step
            elif d == "left":
                dx = -step
            elif d == "right":
                dx = step
            elif d == "zup":
                dz = step
            elif d == "zdown":
                dz = -step

            with self._lock:
                self._state = "Jog"
                self._pos[0] += dx
                self._pos[1] += dy
                self._pos[2] += dz
            time.sleep(0.08)
            with self._lock:
                if self._jog_dir is None:
                    self._state = "Idle"


def build_demo_devices(
    fps: float = 10.0,
    points: int = 12,
    span_x_mm: float = 60.0,
    span_y_mm: float = 40.0,
    tol_mm: float = 0.5,
    seed: Optional[int] = None,
):
    script = DemoScript(points=points, span_x_mm=span_x_mm, span_y_mm=span_y_mm, tol_mm=tol_mm, seed=seed)
    grbl = DemoGrbl()

    cam = DemoCamera(fps=fps, code_provider=script.current_code)
    grbl.set_on_move_completed(script.on_move_completed)

    return cam, grbl, script
