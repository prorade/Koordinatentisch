from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Tuple, Any


@dataclass
class CameraConfig:
    # Settings/UI konfigurierbar:
    cti_path: str
    bin_dir: Optional[str] = None
    target_ip: Optional[str] = None

    # Defaults:
    start_exposure_us: float = 5000.0
    start_gain_db: float = 6.0
    start_fps: float = 60.0

    want_packet_size: int = 9000
    want_throughput: int = 0

    display_stretch: bool = False


class BaseCamera(ABC):

    def __init__(self, cfg: CameraConfig) -> None:
        self.cfg = cfg

    @abstractmethod
    def open(self) -> None:
        """Initialisiert Ressourcen"""

    @abstractmethod
    def start(self) -> None:
        """Startet Streaming/Acquisition."""

    @abstractmethod
    def stop(self) -> None:
        """Stoppt Streaming/Acquisition."""

    @abstractmethod
    def close(self) -> None:
        """Gibt Ressourcen frei."""

    @abstractmethod
    def get_frame(self, timeout_ms: int = 1500) -> Tuple[Any, dict]:
        """
        Liefert (frame, meta).
        frame: typischerweise np.ndarray (uint8) für Anzeige/Operatoren
        meta: dict mit Infos (pixel_format, width, height, decode_label, etc.)
        """

    def __enter__(self):
        self.open()
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            self.stop()
        finally:
            self.close()
