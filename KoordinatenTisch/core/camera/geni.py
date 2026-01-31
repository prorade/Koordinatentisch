from __future__ import annotations

from typing import Any, Optional, Tuple, List


class GenIcam:
    @staticmethod
    def set_node(nm, name: str, value: Any) -> bool:
        if not name:
            return False
        try:
            nm.get_node(name).value = value
            return True
        except Exception:
            return False

    @staticmethod
    def get_val(nm, name: str) -> Any:
        try:
            return nm.get_node(name).value
        except Exception:
            return None

    @staticmethod
    def supports(nm, node_name: str) -> bool:
        try:
            nm.get_node(node_name)
            return True
        except Exception:
            return False

    @staticmethod
    def get_limits(nm, name: str) -> Tuple[Optional[float], Optional[float]]:
        try:
            n = nm.get_node(name)
            return float(getattr(n, "min")), float(getattr(n, "max"))
        except Exception:
            return None, None

    @staticmethod
    def get_inc(nm, name: str, fallback: int = 1) -> int:
        try:
            return int(getattr(nm.get_node(name), "inc"))
        except Exception:
            return fallback

    @staticmethod
    def enum_symbolics(nm, enum_name: str) -> List[str]:
        try:
            return list(nm.get_node(enum_name).symbolics)
        except Exception:
            return []

    @staticmethod
    def clamp(v: float, mn: Optional[float], mx: Optional[float]) -> float:
        if mn is not None:
            v = max(v, mn)
        if mx is not None:
            v = min(v, mx)
        return v

    @staticmethod
    def set_float_clamped(nm, name: str, value: float) -> bool:
        mn, mx = GenIcam.get_limits(nm, name)
        return GenIcam.set_node(nm, name, float(GenIcam.clamp(value, mn, mx)))

    @staticmethod
    def configure_camera_basics(
        nm,
        start_exposure_us: float,
        start_gain_db: float,
        start_fps: float,
        want_packet_size: int,
        want_throughput: int,
    ) -> Tuple[str, str]:
        # Mode/Trigger
        for k, v in [
            ("TriggerMode", "Off"),
            ("ExposureMode", "Timed"),
            ("AcquisitionMode", "Continuous"),
        ]:
            GenIcam.set_node(nm, k, v)

        # PixelFormat (best effort Mono8)
        pf_list = GenIcam.enum_symbolics(nm, "PixelFormat")
        if "Mono8" in pf_list:
            GenIcam.set_node(nm, "PixelFormat", "Mono8")

        # Auto off
        GenIcam.set_node(nm, "ExposureAuto", "Off")
        GenIcam.set_node(nm, "GainAuto", "Off")
        GenIcam.set_node(nm, "GammaEnable", False)
        GenIcam.set_node(nm, "LUTEnable", False)

        # BlackLevel if available
        bl_min, bl_max = GenIcam.get_limits(nm, "BlackLevel")
        if bl_min is not None and bl_max is not None:
            GenIcam.set_node(nm, "BlackLevel", 0.0)

        # Exposure node name differs on some cameras
        exp_name = "ExposureTime" if GenIcam.get_limits(nm, "ExposureTime") != (None, None) else "ExposureTimeAbs"
        GenIcam.set_float_clamped(nm, exp_name, float(start_exposure_us))

        # Gain node name differs
        gain_name = "Gain" if GenIcam.get_limits(nm, "Gain") != (None, None) else "GainRaw"
        GenIcam.set_float_clamped(nm, gain_name, float(start_gain_db))

        # FPS
        GenIcam.set_node(nm, "AcquisitionFrameRateEnable", True)
        fmin, fmax = GenIcam.get_limits(nm, "AcquisitionFrameRate")
        if fmin is not None or fmax is not None:
            GenIcam.set_float_clamped(nm, "AcquisitionFrameRate", float(start_fps))

        # Throughput
        GenIcam.set_node(nm, "DeviceLinkThroughputLimitMode", "Off")
        if want_throughput and want_throughput > 0:
            GenIcam.set_node(nm, "DeviceLinkThroughputLimitMode", "On")
            GenIcam.set_node(nm, "DeviceLinkThroughputLimit", int(want_throughput))

        # Packet size try list
        for p in [want_packet_size, 9000, 8192, 3000, 1500]:
            if p and GenIcam.set_node(nm, "GevSCPSPacketSize", int(p)):
                break

        return exp_name, gain_name

    @staticmethod
    def set_roi_full(nm) -> None:
        try:
            wmax = int(GenIcam.get_val(nm, "WidthMax"))
            hmax = int(GenIcam.get_val(nm, "HeightMax"))
        except Exception:
            return

        w_inc = GenIcam.get_inc(nm, "Width", 2)
        h_inc = GenIcam.get_inc(nm, "Height", 2)

        GenIcam.set_node(nm, "OffsetX", 0)
        GenIcam.set_node(nm, "OffsetY", 0)
        GenIcam.set_node(nm, "Width", (wmax // w_inc) * w_inc)
        GenIcam.set_node(nm, "Height", (hmax // h_inc) * h_inc)
