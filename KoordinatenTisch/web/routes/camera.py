from __future__ import annotations

import time
from typing import Generator, Optional

import cv2
from flask import Blueprint, Response, current_app, jsonify, request

from core.camera.base import CameraConfig
from core.camera.baumer_harvester import BaumerHarvesterCamera

bp = Blueprint("camera", __name__, url_prefix="/camera")


def _get_camera() -> BaumerHarvesterCamera:
    cam = current_app.extensions.get("camera")
    if cam is None:
        cfg = CameraConfig(
            bin_dir=current_app.config.get("BAUMER_BIN_DIR"),
            cti_path=current_app.config["BAUMER_CTI_PATH"],
            target_ip=current_app.config.get("BAUMER_TARGET_IP"),
            start_exposure_us=float(current_app.config.get("CAM_START_EXPOSURE_US", 5000.0)),
            start_gain_db=float(current_app.config.get("CAM_START_GAIN_DB", 6.0)),
            start_fps=float(current_app.config.get("CAM_START_FPS", 60.0)),
            want_packet_size=int(current_app.config.get("CAM_WANT_PACKET", 9000)),
            want_throughput=int(current_app.config.get("CAM_WANT_THROUGHPUT", 0)),
            display_stretch=bool(current_app.config.get("CAM_DISPLAY_STRETCH", False)),
        )
        cam = BaumerHarvesterCamera(cfg)
        current_app.extensions["camera"] = cam
        current_app.extensions["camera_opened"] = False
        current_app.extensions["camera_started"] = False
        current_app.extensions["camera_last_meta"] = {}
        current_app.extensions["camera_fps"] = 0.0
    return cam


def _ensure_started() -> None:
    cam = _get_camera()
    if not current_app.extensions.get("camera_opened", False):
        cam.open()
        current_app.extensions["camera_opened"] = True

    if not current_app.extensions.get("camera_started", False):
        cam.start()
        current_app.extensions["camera_started"] = True


@bp.post("/start")
def start():
    _ensure_started()
    return jsonify({"ok": True, "started": True})


@bp.post("/stop")
def stop():
    cam = _get_camera()
    try:
        if current_app.extensions.get("camera_started", False):
            cam.stop()
            current_app.extensions["camera_started"] = False
    finally:
        pass
    return jsonify({"ok": True, "started": False})


@bp.get("/status")
def status():
    cam = _get_camera()
    opened = current_app.extensions.get("camera_opened", False)
    started = current_app.extensions.get("camera_started", False)

    meta = dict(current_app.extensions.get("camera_last_meta", {}) or {})
    meta["opened"] = opened
    meta["started"] = started
    meta["stream_fps"] = float(current_app.extensions.get("camera_fps", 0.0))

    if opened:
        try:
            meta.update(cam.get_live_values())
        except Exception:
            pass

    return jsonify(meta)


@bp.get("/stream")
def stream():
    """
    MJPEG Stream für <img src="/camera/stream">
    """
    _ensure_started()
    cam = _get_camera()

    def gen() -> Generator[bytes, None, None]:
        last_t = time.time()
        frames = 0

        while True:
            frame, meta = cam.get_frame(timeout_ms=1500)

            frames += 1
            now = time.time()
            if now - last_t >= 1.0:
                current_app.extensions["camera_fps"] = frames / (now - last_t)
                frames = 0
                last_t = now

            current_app.extensions["camera_last_meta"] = meta

            ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ok:
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n"
            )

    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")
