from __future__ import annotations

import os
from dataclasses import dataclass


@dataclass(frozen=True)
class BaseConfig:
    SECRET_KEY: str = os.getenv("SECRET_KEY", "dev-secret-key-change-me")

    # Flask run params (für __main__)
    DEBUG: bool = False
    HOST: str = os.getenv("FLASK_HOST", "127.0.0.1")
    PORT: int = int(os.getenv("FLASK_PORT", "5000"))

    # App-spezifische Defaults
    GRBL_BAUDRATE: int = int(os.getenv("GRBL_BAUDRATE", "115200"))
    GRBL_DEFAULT_PORT: str = os.getenv("GRBL_DEFAULT_PORT", "")
    CAMERA_DEVICE_INDEX: int = int(os.getenv("CAMERA_DEVICE_INDEX", "0"))
    BAUMER_BIN_DIR: str = os.getenv("BAUMER_BIN_DIR", r"H:\Baumer_neoAPI_1.5.0_win_x86_64_cpp\bin")
    BAUMER_CTI_PATH: str = os.getenv("BAUMER_CTI_PATH", r"H:\Baumer_neoAPI_1.5.0_win_x86_64_cpp\bin\bgapi2_gige.cti")
    BAUMER_TARGET_IP: str = os.getenv("BAUMER_TARGET_IP", "192.168.178.158")

    CAM_START_EXPOSURE_US: float = float(os.getenv("CAM_START_EXPOSURE_US", "5000"))
    CAM_START_GAIN_DB: float = float(os.getenv("CAM_START_GAIN_DB", "6"))
    CAM_START_FPS: float = float(os.getenv("CAM_START_FPS", "60"))
    CAM_WANT_PACKET: int = int(os.getenv("CAM_WANT_PACKET", "9000"))
    CAM_WANT_THROUGHPUT: int = int(os.getenv("CAM_WANT_THROUGHPUT", "0"))
    CAM_DISPLAY_STRETCH: bool = os.getenv("CAM_DISPLAY_STRETCH", "0") == "1"



@dataclass(frozen=True)
class DevConfig(BaseConfig):
    DEBUG: bool = True


@dataclass(frozen=True)
class ProdConfig(BaseConfig):
    DEBUG: bool = False


@dataclass(frozen=True)
class TestConfig(BaseConfig):
    DEBUG: bool = True
    TESTING: bool = True


def get_config():
    env = os.getenv("FLASK_ENV", "dev").lower()
    if env in ("prod", "production"):
        return ProdConfig
    if env in ("test", "testing"):
        return TestConfig
    return DevConfig
