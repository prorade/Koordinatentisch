import os


def _env(name: str, default: str) -> str:
    value = os.environ.get(name)
    return default if value is None or value == "" else value


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    return float(raw)


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    return int(raw)


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    return raw.strip().lower() in {"1", "true", "yes", "y", "on"}


# Camera
CAMERA_EXPOSURE_US = _env_int("CAMERA_EXPOSURE_US", 10000)
CAMERA_FPS = _env_float("CAMERA_FPS", 10.0)

# If you have multiple cameras you can set an optional serial.
CAMERA_SERIAL = _env("CAMERA_SERIAL", "")

# GRBL
GRBL_PORT = _env("GRBL_PORT", "COM3")
GRBL_BAUD = _env_int("GRBL_BAUD", 115200)

# Motion
JOG_STEP_MM = _env_float("JOG_STEP_MM", 0.25)
JOG_FEED_MM_MIN = _env_float("JOG_FEED_MM_MIN", 300.0)
RAPID_FEED_MM_MIN = _env_float("RAPID_FEED_MM_MIN", 1200.0)

# Workflow
QR_RETRY_SECONDS = _env_float("QR_RETRY_SECONDS", 8.0)

# Focus (sharpness check)
# 0 => auto (threshold derived from first successful QR read)
FOCUS_THRESHOLD = _env_float("FOCUS_THRESHOLD", 0.0)

# Work area (machine coordinates, mm). Used for canvas scaling and demo point generation.
WORK_AREA_X_MM = _env_float("WORK_AREA_X_MM", 200.0)
WORK_AREA_Y_MM = _env_float("WORK_AREA_Y_MM", 150.0)

# Demo
DEMO_MODE = _env_bool("DEMO_MODE", False)
DEMO_POINTS = _env_int("DEMO_POINTS", 12)
DEMO_SPAN_X_MM = _env_float("DEMO_SPAN_X_MM", 60.0)
DEMO_SPAN_Y_MM = _env_float("DEMO_SPAN_Y_MM", 40.0)
DEMO_TOL_MM = _env_float("DEMO_TOL_MM", 0.5)
DEMO_SEED = _env("DEMO_SEED", "")
