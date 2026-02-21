from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any, Callable, Optional, Tuple

import config
from workflow import QrWorkflow


@dataclass
class Devices:
    demo_mode: bool
    camera: Any
    grbl: Any
    workflow: QrWorkflow
    demo_script: Any = None


class DeviceManager:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._devices: Optional[Devices] = None
        self._work_area_x_mm: float = float(config.WORK_AREA_X_MM)
        self._work_area_y_mm: float = float(config.WORK_AREA_Y_MM)
        self._focus_threshold: float = float(config.FOCUS_THRESHOLD)

    def get_focus_threshold(self) -> float:
        with self._lock:
            return float(self._focus_threshold)

    def set_focus_threshold(self, threshold: float) -> float:
        t = float(threshold)
        if not (t >= 0.0 and t < 1e12):
            raise ValueError("focus threshold must be >= 0")

        with self._lock:
            self._focus_threshold = t
            cur = self._devices
            if cur is not None:
                try:
                    cur.workflow.set_focus_threshold(t)
                except Exception:
                    pass
            return float(self._focus_threshold)

    def get_work_area(self) -> Tuple[float, float]:
        with self._lock:
            return self._work_area_x_mm, self._work_area_y_mm

    def set_work_area(self, x_mm: float, y_mm: float) -> Tuple[float, float]:
        x = float(x_mm)
        y = float(y_mm)
        if not (x > 0.0 and y > 0.0):
            raise ValueError("work area must be > 0")

        with self._lock:
            self._work_area_x_mm = x
            self._work_area_y_mm = y

            cur = self._devices
            if cur is not None and cur.demo_script is not None:
                try:
                    # keep old public API name span_x/span_y
                    if hasattr(cur.demo_script, "set_span"):
                        cur.demo_script.set_span(x, y)
                    elif hasattr(cur.demo_script, "set_work_area"):
                        cur.demo_script.set_work_area(x, y)
                except Exception:
                    pass

            return self._work_area_x_mm, self._work_area_y_mm

    def get(self) -> Devices:
        with self._lock:
            if self._devices is None:
                self._devices = self._create(demo_mode=config.DEMO_MODE)
            return self._devices

    def set_demo_mode(self, demo_mode: bool) -> Devices:
        with self._lock:
            cur = self._devices
            if cur is not None and cur.demo_mode == bool(demo_mode):
                return cur

            # stop old
            if cur is not None:
                try:
                    cur.workflow.stop()
                except Exception:
                    pass
                try:
                    cur.camera.stop()
                except Exception:
                    pass
                try:
                    cur.grbl.close()
                except Exception:
                    pass

            self._devices = self._create(demo_mode=bool(demo_mode))
            return self._devices

    def _create(self, demo_mode: bool) -> Devices:
        demo_script = None

        if demo_mode:
            from demo_devices import build_demo_devices

            seed = None
            if config.DEMO_SEED:
                try:
                    seed = int(config.DEMO_SEED)
                except Exception:
                    seed = None

            camera, grbl, demo_script = build_demo_devices(
                fps=config.CAMERA_FPS,
                points=config.DEMO_POINTS,
                span_x_mm=self._work_area_x_mm,
                span_y_mm=self._work_area_y_mm,
                tol_mm=config.DEMO_TOL_MM,
                seed=seed,
            )
        else:
            from camera_baumer import BaumerCamera
            from grbl import Grbl

            camera = BaumerCamera(
                exposure_us=config.CAMERA_EXPOSURE_US,
                fps=config.CAMERA_FPS,
                serial=config.CAMERA_SERIAL,
            )
            grbl = Grbl(port=config.GRBL_PORT, baud=config.GRBL_BAUD)

        def _get_frame():
            if hasattr(camera, "get_latest_ref"):
                try:
                    return camera.get_latest_ref()[0]
                except Exception:
                    return camera.get_latest()[0]
            return camera.get_latest()[0]

        workflow = QrWorkflow(grbl=grbl, get_frame=_get_frame, qr_retry_seconds=config.QR_RETRY_SECONDS)
        try:
            workflow.set_focus_threshold(self._focus_threshold)
        except Exception:
            pass
        return Devices(demo_mode=demo_mode, camera=camera, grbl=grbl, workflow=workflow, demo_script=demo_script)
