from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Optional, Tuple, Any

import numpy as np

from harvesters.core import Harvester

from core.camera.base import BaseCamera, CameraConfig
from core.camera.decoder import FrameDecoder
from core.camera.geni import GenIcam


@dataclass
class CameraInfo:
    model: Optional[str] = None
    serial: Optional[str] = None
    pixel_format: Optional[str] = None
    width: Optional[int] = None
    height: Optional[int] = None


class BaumerHarvesterCamera(BaseCamera):

    def __init__(self, cfg: CameraConfig) -> None:
        super().__init__(cfg)
        self._h: Optional[Harvester] = None
        self._ia = None
        self._nm = None

        self._exp_node_name: str = "ExposureTime"
        self._gain_node_name: str = "Gain"

        self._decoder = FrameDecoder()
        self.info = CameraInfo()

    # ---------- Lifecycle ----------
    def open(self) -> None:
        self._ensure_paths()

        h = Harvester()
        h.add_file(self.cfg.cti_path)
        h.update()

        if not h.device_info_list:
            h.reset()
            raise RuntimeError("Keine GenTL-Geräte gefunden (Harvester.device_info_list ist leer).")

        device_index = self._select_device_index(h)
        ia = h.create(device_index)

        # Buffer count
        try:
            ia.num_buffers = 4
        except Exception:
            pass

        nm = ia.remote_device.node_map
        self._h = h
        self._ia = ia
        self._nm = nm

        # Basiskonfig
        self._exp_node_name, self._gain_node_name = GenIcam.configure_camera_basics(
            nm,
            start_exposure_us=self.cfg.start_exposure_us,
            start_gain_db=self.cfg.start_gain_db,
            start_fps=self.cfg.start_fps,
            want_packet_size=self.cfg.want_packet_size,
            want_throughput=self.cfg.want_throughput,
        )

        # ROI
        GenIcam.set_roi_full(nm)

        # Kamerainfo
        self.info.model = GenIcam.get_val(nm, "DeviceModelName")
        self.info.serial = GenIcam.get_val(nm, "DeviceSerialNumber")
        self.info.pixel_format = str(GenIcam.get_val(nm, "PixelFormat") or "Mono8")

    def start(self) -> None:
        if not self._ia:
            raise RuntimeError("Camera not opened.")
        self._ia.start()

        # width/height aktualisieren
        self.info.width = int(GenIcam.get_val(self._nm, "Width") or 0)
        self.info.height = int(GenIcam.get_val(self._nm, "Height") or 0)

    def stop(self) -> None:
        if self._ia:
            try:
                self._ia.stop()
            except Exception:
                pass

    def close(self) -> None:
        # Harvester resources freigeben
        if self._ia:
            try:
                self._ia.destroy()
            except Exception:
                pass
            self._ia = None
            self._nm = None

        if self._h:
            try:
                self._h.reset()
            except Exception:
                pass
            self._h = None

    # ---------- Public API ----------
    def get_frame(self, timeout_ms: int = 1500) -> Tuple[Any, dict]:
        if not self._ia:
            raise RuntimeError("Camera not started.")

        pf_str = self.info.pixel_format or "Mono8"

        with self._ia.fetch(timeout=timeout_ms) as buf:
            comp = buf.payload.components[0]
            frame_u8, decode_label = self._decoder.to_display_u8_copy(comp, pf_str)

        meta = {
            "decode_label": decode_label,
            "pixel_format": pf_str,
            "model": self.info.model,
            "serial": self.info.serial,
            "width": int(getattr(comp, "width", 0)),
            "height": int(getattr(comp, "height", 0)),
        }
        return frame_u8, meta

    def set_exposure_us(self, exposure_us: float) -> None:
        if not self._nm:
            raise RuntimeError("Camera not opened.")
        GenIcam.set_node(self._nm, "ExposureAuto", "Off")
        GenIcam.set_float_clamped(self._nm, self._exp_node_name, float(exposure_us))

    def set_gain_db(self, gain_db: float) -> None:
        if not self._nm:
            raise RuntimeError("Camera not opened.")
        gname = "Gain" if GenIcam.supports(self._nm, "Gain") else "GainRaw"
        GenIcam.set_node(self._nm, "GainAuto", "Off")
        GenIcam.set_float_clamped(self._nm, gname, float(gain_db))

    def set_fps(self, fps: float) -> None:
        if not self._nm:
            raise RuntimeError("Camera not opened.")
        GenIcam.set_node(self._nm, "AcquisitionFrameRateEnable", True)
        GenIcam.set_float_clamped(self._nm, "AcquisitionFrameRate", float(fps))

    def set_full_roi(self) -> None:
        if not self._nm:
            raise RuntimeError("Camera not opened.")
        GenIcam.set_roi_full(self._nm)
        self.info.width = int(GenIcam.get_val(self._nm, "Width") or 0)
        self.info.height = int(GenIcam.get_val(self._nm, "Height") or 0)

    # ---------- Helpers ----------
    def _ensure_paths(self) -> None:
        if self.cfg.bin_dir:
            if hasattr(os, "add_dll_directory"):
                os.add_dll_directory(self.cfg.bin_dir)
            os.environ["PATH"] = self.cfg.bin_dir + os.pathsep + os.environ.get("PATH", "")

        if not os.path.exists(self.cfg.cti_path):
            raise FileNotFoundError(f"CTI nicht gefunden: {self.cfg.cti_path}")

    def _select_device_index(self, h: Harvester) -> int:
        if self.cfg.target_ip:
            for i, d in enumerate(h.device_info_list):
                if self.cfg.target_ip in str(d):
                    return i
        return 0
    
    def get_live_values(self) -> dict:
        """
        Best-effort Werte aus der NodeMap auslesen.
        Entspricht dem, was im Script im Overlay angezeigt wird.
        """
        if not self._nm:
            return {}

        nm = self._nm

        def _f(name: str):
            try:
                return float(GenIcam.get_val(nm, name))
            except Exception:
                return None

        def _i(name: str):
            try:
                v = GenIcam.get_val(nm, name)
                return int(v) if v is not None else None
            except Exception:
                return None

        # Exposure
        exp = _f(self._exp_node_name)

        # Gain (Gain oder GainRaw)
        gname = "Gain" if GenIcam.supports(nm, "Gain") else "GainRaw"
        gain = _f(gname)

        # FPS
        fps = _f("AcquisitionFrameRate")

        # Scaling (Binning/Decimation)
        b_h = _i("BinningHorizontal") if GenIcam.supports(nm, "BinningHorizontal") else 1
        b_v = _i("BinningVertical") if GenIcam.supports(nm, "BinningVertical") else 1
        d_h = _i("DecimationHorizontal") if GenIcam.supports(nm, "DecimationHorizontal") else 1
        d_v = _i("DecimationVertical") if GenIcam.supports(nm, "DecimationVertical") else 1

        # Auflösung
        width = _i("Width")
        height = _i("Height")

        # PixelFormat
        pf = str(GenIcam.get_val(nm, "PixelFormat") or (self.info.pixel_format or "Mono8"))

        return {
            "model": self.info.model,
            "serial": self.info.serial,
            "width": width,
            "height": height,
            "pixel_format": pf,
            "exposure_us": exp,
            "gain_db": gain,
            "camera_fps": fps,
            "bin_h": b_h or 1,
            "bin_v": b_v or 1,
            "dec_h": d_h or 1,
            "dec_v": d_v or 1,
        }
