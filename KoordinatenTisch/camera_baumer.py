from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class CameraInfo:
    model: str = ""
    serial: str = ""
    pixel_format: str = ""


class BaumerCamera:
    """Minimal neoAPI camera wrapper.

    Provides a background grab loop and the latest frame as a NumPy array.

    Notes:
    - Requires Baumer neoAPI Python module: `import neoapi`.
    - Pixel format is set to BGR8 if available, otherwise Mono8.
    """

    def __init__(self, exposure_us: int = 10000, fps: float = 10.0, serial: str = ""):
        self._exposure_us = exposure_us
        self._fps = fps
        self._serial = serial

        self._camera = None
        self._neoapi = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

        self._cam_lock = threading.Lock()

        self._lock = threading.Lock()
        self._latest: Optional[np.ndarray] = None
        self._info = CameraInfo()

        self._frame_id: int = 0

        self._connected = False
        self._last_error: str = ""
        self._last_frame_at: float = 0.0

    @property
    def info(self) -> CameraInfo:
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

    def connect(self) -> None:
        try:
            import neoapi  # type: ignore

            self._neoapi = neoapi
            cam = neoapi.Cam()

            # If a specific serial is desired, try to select it.
            # neoapi selection API can differ by version; keep minimal + robust.
            cam.Connect()
        except Exception as exc:
            self._connected = False
            self._last_error = str(exc)
            self._camera = None
            return

        # PixelFormat
        is_color = False
        try:
            if cam.f.PixelFormat.GetEnumValueList().IsReadable("BGR8"):
                cam.f.PixelFormat.SetString("BGR8")
                is_color = True
                self._info.pixel_format = "BGR8"
            elif cam.f.PixelFormat.GetEnumValueList().IsReadable("Mono8"):
                cam.f.PixelFormat.SetString("Mono8")
                self._info.pixel_format = "Mono8"
            else:
                self._info.pixel_format = "unknown"
        except Exception:
            self._info.pixel_format = "unknown"

        # Exposure + FPS
        try:
            cam.f.ExposureTime.Set(int(self._exposure_us))
        except Exception:
            pass

        try:
            cam.f.AcquisitionFrameRateEnable.value = True
            cam.f.AcquisitionFrameRate.value = float(self._fps)
        except Exception:
            pass

        # Best-effort info
        try:
            self._info.model = str(cam.f.DeviceModelName.value)
        except Exception:
            pass
        try:
            self._info.serial = str(cam.f.DeviceSerialNumber.value)
        except Exception:
            pass

        self._camera = cam
        self._connected = True
        self._last_error = ""

    def start(self) -> None:
        if self._camera is None:
            self.connect()
        if self._camera is None:
            return
        if self._thread and self._thread.is_alive():
            return

        self._stop.clear()
        self._thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self._thread = None

        cam = self._camera
        self._camera = None
        self._connected = False
        if cam is not None:
            try:
                cam.Disconnect()
            except Exception:
                pass

    def get_latest(self) -> Tuple[Optional[np.ndarray], float]:
        """Returns (frame, timestamp)."""
        with self._lock:
            if self._latest is None:
                return None, 0.0
            return self._latest.copy(), time.time()

    def get_latest_ref(self) -> Tuple[Optional[np.ndarray], float, int]:
        """Returns (frame_ref, last_frame_at, frame_id) without copying.

        The returned array must be treated as read-only.
        """
        with self._lock:
            return self._latest, self._last_frame_at, self._frame_id

    def _grab_loop(self) -> None:
        assert self._camera is not None
        frame_interval = 1.0 / max(self._fps, 0.1)
        while not self._stop.is_set():
            try:
                with self._cam_lock:
                    img = self._camera.GetImage()
                if img is None or img.IsEmpty():
                    time.sleep(0.01)
                    continue

                arr = img.GetNPArray()
                if isinstance(arr, np.ndarray):
                    with self._lock:
                        self._latest = arr
                        self._last_frame_at = time.time()
                        self._frame_id += 1
            except Exception:
                self._last_error = "frame grab failed"
                time.sleep(0.05)

            time.sleep(frame_interval)

    def get_settable_features(self) -> list[dict]:
        """Return all features that are currently IsAvailable() and IsWritable().

        Notes:
        - The concrete feature set is camera-model and state dependent.
        - This method is intended for UI introspection.
        """
        cam = self._camera
        if cam is None:
            return []

        features: list[dict] = []
        with self._cam_lock:
            feat_list = cam.GetFeatureList()
            for it in feat_list:
                name = None
                feat = None
                # Depending on binding details, the iterator may yield either
                # a FeatureListIterator (with __ref__) or a Feature directly.
                try:
                    if hasattr(it, "__ref__"):
                        name = str(it.GetName())
                        feat = it.__ref__()
                    else:
                        feat = it
                        try:
                            name = str(feat.GetName())
                        except Exception:
                            name = None
                except Exception:
                    name = None
                    feat = None
                if not name or feat is None:
                    continue

                try:
                    if not (feat.IsAvailable() and feat.IsWritable()):
                        continue
                except Exception:
                    continue

                iface = ""
                try:
                    iface = str(feat.GetInterface())
                except Exception:
                    iface = ""

                value = None
                try:
                    if iface == "IBoolean":
                        value = bool(feat.GetBool())
                    elif iface == "IFloat":
                        value = float(feat.GetDouble())
                    elif iface in {"IInteger", "IEnumeration"}:
                        value = int(feat.GetInt())
                    elif iface == "IString":
                        value = str(feat.GetString())
                    elif iface == "ICommand":
                        value = None
                    else:
                        value = feat.value
                except Exception:
                    try:
                        value = feat.value
                    except Exception:
                        value = None

                unit = None
                try:
                    unit = str(feat.GetUnit())
                except Exception:
                    unit = None

                minimum = None
                maximum = None
                inc = None
                try:
                    if iface == "IFloat":
                        minimum = float(feat.GetDoubleMin())
                        maximum = float(feat.GetDoubleMax())
                        inc = float(feat.GetDoubleInc())
                    elif iface in {"IInteger", "IEnumeration", "IBoolean"}:
                        minimum = int(feat.GetIntMin())
                        maximum = int(feat.GetIntMax())
                        inc = int(feat.GetIntInc())
                except Exception:
                    pass

                enum_values = None
                if iface == "IEnumeration":
                    try:
                        values: list[str] = []
                        for vit in feat.GetEnumValueList():
                            try:
                                values.append(str(vit.GetName()))
                            except Exception:
                                pass
                        enum_values = values
                    except Exception:
                        enum_values = None

                desc = None
                try:
                    desc = str(feat.GetToolTip())
                except Exception:
                    desc = None

                features.append(
                    {
                        "name": name,
                        "interface": iface,
                        "value": value,
                        "min": minimum,
                        "max": maximum,
                        "inc": inc,
                        "unit": unit,
                        "enum_values": enum_values,
                        "description": desc,
                    }
                )

        features.sort(key=lambda f: str(f.get("name", "")))
        return features

    def set_feature(self, name: str, value, execute: bool = False):
        cam = self._camera
        if cam is None:
            raise RuntimeError("camera not connected")

        with self._cam_lock:
            feat = cam.GetFeature(str(name))
            if not (feat.IsAvailable() and feat.IsWritable()):
                raise RuntimeError("feature not available/writable")

            iface = ""
            try:
                iface = str(feat.GetInterface())
            except Exception:
                iface = ""

            if iface == "ICommand":
                if not execute:
                    raise RuntimeError("command feature requires execute=true")
                feat.Execute()
                return {"name": name, "interface": iface, "value": None}

            # Best-effort type handling
            try:
                if iface == "IBoolean":
                    feat.SetBool(bool(value))
                elif iface == "IString":
                    feat.SetString(str(value))
                elif iface == "IFloat":
                    feat.SetDouble(float(value))
                elif iface in {"IInteger"}:
                    feat.SetInt(int(value))
                elif iface == "IEnumeration":
                    if isinstance(value, str):
                        # Many GenICam enum features accept string entry names.
                        try:
                            feat.SetString(value)
                        except Exception:
                            feat.value = value
                    else:
                        feat.SetInt(int(value))
                else:
                    feat.value = value
            except Exception:
                # final fallback
                feat.value = value

            new_value = None
            try:
                if iface == "IBoolean":
                    new_value = bool(feat.GetBool())
                elif iface == "IFloat":
                    new_value = float(feat.GetDouble())
                elif iface in {"IInteger", "IEnumeration"}:
                    new_value = int(feat.GetInt())
                elif iface == "IString":
                    new_value = str(feat.GetString())
                else:
                    new_value = feat.value
            except Exception:
                try:
                    new_value = feat.value
                except Exception:
                    new_value = None
            return {"name": name, "interface": iface, "value": new_value}
