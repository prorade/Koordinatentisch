from __future__ import annotations

import queue
import re
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import serial


_STATUS_RE = re.compile(r"^<(?P<state>[^|>]+)\|(?P<body>.*)>$")
_WPOS_RE = re.compile(r"WPos:(?P<x>-?\d+\.\d+),(?P<y>-?\d+\.\d+),(?P<z>-?\d+\.\d+)")
_MPOS_RE = re.compile(r"MPos:(?P<x>-?\d+\.\d+),(?P<y>-?\d+\.\d+),(?P<z>-?\d+\.\d+)")


@dataclass
class GrblStatus:
    state: str = "Unknown"
    wpos: Optional[Tuple[float, float, float]] = None
    mpos: Optional[Tuple[float, float, float]] = None
    raw: str = ""
    updated_at: float = 0.0


class Grbl:
    """Minimal GRBL serial controller (GRBL 1.1 compatible).

    Features:
    - Background reader thread
    - send() with ok-wait
    - status query '?' parsing (WPos/MPos)
    - continuous jog loop controlled by start_jog/stop_jog
    """

    def __init__(self, port: str, baud: int = 115200):
        self._port = port
        self._baud = baud

        self._ser: Optional[serial.Serial] = None
        self._reader: Optional[threading.Thread] = None
        self._stop = threading.Event()

        self._lines: "queue.Queue[str]" = queue.Queue()
        self._status_lock = threading.Lock()
        self._status = GrblStatus()

        self._write_lock = threading.Lock()

        self._status_poller: Optional[threading.Thread] = None
        self._status_poll_interval_s: float = 0.2

        self._jog_lock = threading.Lock()
        self._jog_dir: Optional[str] = None
        self._jog_thread: Optional[threading.Thread] = None
        self._jog_step_mm = 0.25
        self._jog_feed = 300.0

    def connect(self) -> None:
        if self._ser and self._ser.is_open:
            return

        self._ser = serial.Serial(self._port, self._baud, timeout=0.1, write_timeout=0.5)
        time.sleep(2.0)  # allow Arduino reset

        self._stop.clear()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        self._status_poller = threading.Thread(target=self._status_poll_loop, daemon=True)
        self._status_poller.start()

        # wake up and clear
        self.write_raw("\r\n")
        self.flush_input()

    def close(self) -> None:
        self._stop.set()
        self.stop_jog()
        if self._reader:
            self._reader.join(timeout=1.0)
        self._reader = None

        if self._status_poller:
            self._status_poller.join(timeout=1.0)
        self._status_poller = None

        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None

    def set_jog_params(self, step_mm: float, feed_mm_min: float) -> None:
        self._jog_step_mm = float(step_mm)
        self._jog_feed = float(feed_mm_min)

    def write_raw(self, data: str) -> None:
        if not self._ser:
            raise RuntimeError("GRBL not connected")
        with self._write_lock:
            self._ser.write(data.encode("ascii", errors="ignore"))

    def flush_input(self) -> None:
        if not self._ser:
            return
        with self._write_lock:
            self._ser.reset_input_buffer()

    def send(self, line: str, wait_ok: bool = True, timeout: float = 10.0) -> None:
        """Send a gcode/settings line (newline auto-added)."""
        if not self._ser:
            raise RuntimeError("GRBL not connected")

        clean = line.strip()
        if clean == "":
            return

        self.write_raw(clean + "\n")
        if not wait_ok:
            return

        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                resp = self._lines.get(timeout=0.1)
            except queue.Empty:
                continue

            if resp.lower().startswith("ok"):
                return
            if resp.lower().startswith("error"):
                raise RuntimeError(resp)
        raise TimeoutError(f"Timeout waiting for ok: {line}")

    def command(self, line: str, timeout: float = 10.0) -> list[str]:
        """Send a command and return all response lines until 'ok'/'error'.

        Useful for multi-line commands like '$$'.
        """
        if not self._ser:
            raise RuntimeError("GRBL not connected")

        clean = line.strip()
        if clean == "":
            return []

        # Drain any previous buffered lines to keep parsing clean.
        drained: list[str] = []
        try:
            while True:
                drained.append(self._lines.get_nowait())
        except queue.Empty:
            pass

        self.write_raw(clean + "\n")
        out: list[str] = []
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                resp = self._lines.get(timeout=0.1)
            except queue.Empty:
                continue

            low = resp.lower()
            if low.startswith("ok"):
                return out
            if low.startswith("error"):
                raise RuntimeError(resp)
            out.append(resp)

        raise TimeoutError(f"Timeout waiting for response: {line}")

    def get_settings(self, timeout: float = 10.0) -> dict[str, dict]:
        """Return GRBL '$$' settings as a dict.

        Keys are like '$0', '$110', etc.
        Values contain 'value' and optional 'description' (text in parentheses).
        """
        lines = self.command("$$", timeout=timeout)
        settings: dict[str, dict] = {}
        for ln in lines:
            s = ln.strip()
            if not s.startswith("$"):
                continue
            # Example: $0=10 (step pulse, usec)
            m = re.match(r"^\$(?P<num>\d+)=?(?P<val>[^\s(]+)?\s*(?:\((?P<desc>.*)\))?$", s)
            if not m:
                continue
            key = f"${m.group('num')}"
            val = m.group("val")
            desc = m.group("desc")
            settings[key] = {
                "value": val,
                "description": desc,
                "raw": s,
            }
        return settings

    def set_setting(self, key: str, value, timeout: float = 10.0) -> None:
        k = str(key).strip()
        if not k.startswith("$"):
            k = "$" + k
        self.send(f"{k}={value}", wait_ok=True, timeout=timeout)

    def query_status(self) -> GrblStatus:
        # Non-blocking: return cached status. A background poller keeps it fresh.
        now = time.time()
        with self._status_lock:
            st = self._status

        if self._ser is not None and (now - (st.updated_at or 0.0)) > 1.0:
            try:
                self.write_raw("?")
            except Exception:
                pass
        return st

    def _status_poll_loop(self) -> None:
        # Periodically ask GRBL for status ('?') so _read_loop receives <...> lines.
        while not self._stop.is_set():
            try:
                if self._ser is not None:
                    self.write_raw("?")
            except Exception:
                pass
            time.sleep(self._status_poll_interval_s)

    def get_position_mpos(self) -> Optional[Tuple[float, float, float]]:
        st = self.query_status()
        return st.mpos

    def get_position_wpos(self) -> Optional[Tuple[float, float, float]]:
        st = self.query_status()
        return st.wpos

    def get_position_xy(self) -> Optional[Tuple[float, float]]:
        """Return XY in machine coordinates (MPos).

        This matches setups where homing defines machine zero (0,0,0) and
        all commanded positions are absolute from there.
        """
        pos = self.get_position_mpos()
        if pos is None:
            return None
        return pos[0], pos[1]

    def wait_until_idle(self, timeout: float = 30.0) -> None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            st = self.query_status()
            if st.state.lower() == "idle":
                return
            time.sleep(0.1)
        raise TimeoutError("GRBL did not become Idle")

    def homing(self, timeout: float = 120.0) -> None:
        self.send("$H", wait_ok=True, timeout=timeout)
        self.wait_until_idle(timeout=timeout)

    def move_abs_xy(self, x: float, y: float, feed: Optional[float] = None) -> None:
        self.move_abs_xyz(x=x, y=y, z=None, feed=feed)

    def move_abs_xyz(self, x: float, y: float, z: Optional[float], feed: Optional[float] = None) -> None:
        """Move to absolute XY(Z) in machine coordinates.

        Uses G53 (machine coordinate system) + G90.
        """
        parts = ["G90", "G53"]
        parts.append("G0" if feed is None else "G1")
        parts.append(f"X{x:.3f}")
        parts.append(f"Y{y:.3f}")
        if z is not None:
            parts.append(f"Z{float(z):.3f}")
        if feed is not None:
            parts.append(f"F{float(feed):.1f}")
        self.send(" ".join(parts))

    def start_jog(self, direction: str) -> None:
        """Start continuous jog.

        Supported directions: up/down/left/right/zup/zdown.
        """
        if direction not in {"up", "down", "left", "right", "zup", "zdown"}:
            raise ValueError("invalid direction")

        with self._jog_lock:
            self._jog_dir = direction
            if self._jog_thread and self._jog_thread.is_alive():
                return
            self._jog_thread = threading.Thread(target=self._jog_loop, daemon=True)
            self._jog_thread.start()

    def stop_jog(self) -> None:
        with self._jog_lock:
            self._jog_dir = None

        # Jog cancel (GRBL 1.1)
        try:
            self.write_raw("\x85")
        except Exception:
            pass

    def _jog_loop(self) -> None:
        while not self._stop.is_set():
            with self._jog_lock:
                direction = self._jog_dir
            if direction is None:
                time.sleep(0.02)
                continue

            dx = 0.0
            dy = 0.0
            dz = 0.0
            step = self._jog_step_mm
            if direction == "up":
                dy = step
            elif direction == "down":
                dy = -step
            elif direction == "left":
                dx = -step
            elif direction == "right":
                dx = step
            elif direction == "zup":
                dz = step
            elif direction == "zdown":
                dz = -step

            # Send short jog increment repeatedly to emulate continuous motion.
            try:
                # $J= jog uses G91 incremental.
                self.send(
                    f"$J=G91 X{dx:.3f} Y{dy:.3f} Z{dz:.3f} F{self._jog_feed:.1f}",
                    wait_ok=True,
                    timeout=2.0,
                )
            except Exception:
                time.sleep(0.1)

            time.sleep(0.08)

    def _read_loop(self) -> None:
        assert self._ser is not None
        buffer = b""
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(256)
            except Exception:
                time.sleep(0.05)
                continue

            if not chunk:
                continue

            buffer += chunk
            while b"\n" in buffer:
                raw, buffer = buffer.split(b"\n", 1)
                line = raw.decode("ascii", errors="ignore").strip()
                if not line:
                    continue

                if line.startswith("<") and line.endswith(">"):
                    self._handle_status(line)
                else:
                    self._lines.put(line)

    def _handle_status(self, line: str) -> None:
        m = _STATUS_RE.match(line)
        if not m:
            return
        state = m.group("state")
        body = m.group("body")

        wpos = None
        mpos = None

        mw = _WPOS_RE.search(body)
        if mw:
            wpos = (float(mw.group("x")), float(mw.group("y")), float(mw.group("z")))
        mm = _MPOS_RE.search(body)
        if mm:
            mpos = (float(mm.group("x")), float(mm.group("y")), float(mm.group("z")))

        with self._status_lock:
            self._status = GrblStatus(state=state, wpos=wpos, mpos=mpos, raw=line, updated_at=time.time())
