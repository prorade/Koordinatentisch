from __future__ import annotations

import time
from flask import Flask, Response, jsonify, render_template, request

import cv2
import numpy as np

import config
from device_manager import DeviceManager


app = Flask(__name__)


_manager = DeviceManager()


def _ensure_connected() -> None:
    dev = _manager.get()
    # camera
    try:
        dev.camera.start()
    except Exception:
        pass

    # grbl
    try:
        if not dev.demo_mode:
            dev.grbl.connect()
        dev.grbl.set_jog_params(step_mm=config.JOG_STEP_MM, feed_mm_min=config.JOG_FEED_MM_MIN)
    except Exception:
        pass


def _devices():
    return _manager.get()


@app.get("/")
def index():
    _ensure_connected()
    return render_template("index.html")


def _encode_jpeg(frame: np.ndarray) -> bytes:
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    if not ok:
        return b""
    return buf.tobytes()


@app.get("/video_feed")
def video_feed():
    _ensure_connected()

    def gen():
        last_mode: str = ""
        last_id: int = -1
        last_jpeg: bytes = b""
        last_sent_at: float = 0.0
        min_interval_s = 0.1  # ~10 fps max
        while True:
            dev = _devices()
            running = bool(getattr(dev.workflow.state.metrics, "running", False))

            now = time.time()
            if (now - last_sent_at) < min_interval_s:
                time.sleep(0.01)
                continue

            if running:
                mode = "capture"
                try:
                    jpeg, cap_id = dev.workflow.get_last_capture_jpeg()
                except Exception:
                    jpeg, cap_id = b"", -1

                if not jpeg:
                    placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                    jpeg = _encode_jpeg(placeholder)
                    cap_id = -1

                if mode != last_mode or cap_id != last_id:
                    last_mode = mode
                    last_id = int(cap_id)
                    last_jpeg = jpeg
            else:
                mode = "live"
                cam = dev.camera

                frame = None
                frame_id = -1
                if hasattr(cam, "get_latest_ref"):
                    try:
                        frame, _ts, frame_id = cam.get_latest_ref()
                    except Exception:
                        frame, _ts = cam.get_latest()
                        frame_id = -1
                else:
                    frame, _ts = cam.get_latest()

                if frame is None:
                    frame_bgr = np.zeros((480, 640, 3), dtype=np.uint8)
                elif frame.ndim == 2:
                    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                else:
                    frame_bgr = frame

                if mode != last_mode or frame_id != last_id or not last_jpeg:
                    jpeg = _encode_jpeg(frame_bgr)
                    if jpeg:
                        last_mode = mode
                        last_id = int(frame_id)
                        last_jpeg = jpeg

            if last_jpeg:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + last_jpeg + b"\r\n"
                last_sent_at = now
            else:
                time.sleep(0.02)

    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.get("/api/status")
def api_status():
    _ensure_connected()

    dev = _devices()

    grbl_status = None
    try:
        s = dev.grbl.query_status()
        grbl_status = {
            "state": s.state,
            "wpos": s.wpos,
            "mpos": s.mpos,
            "raw": s.raw,
            "updated_at": s.updated_at,
        }
    except Exception as exc:
        grbl_status = {"error": str(exc)}

    wf = dev.workflow.state
    start_pos = wf.start_pos

    cam_status = {
        "connected": dev.camera.connected,
        "model": dev.camera.info.model,
        "serial": dev.camera.info.serial,
        "pixel_format": dev.camera.info.pixel_format,
        "last_error": dev.camera.last_error,
        "last_frame_at": dev.camera.last_frame_at,
    }

    current_mpos = None
    try:
        current_mpos = grbl_status.get("mpos") if isinstance(grbl_status, dict) else None
    except Exception:
        current_mpos = None

    wa_x, wa_y = _manager.get_work_area()

    return jsonify(
        {
            "demo_mode": dev.demo_mode,
            "work_area": {"x_mm": wa_x, "y_mm": wa_y},
            "camera": cam_status,
            "grbl": grbl_status,
            "current_mpos": current_mpos,
            "start_pos": start_pos,
            "visited": wf.visited,
            "decoded_chain": wf.decoded_chain,
            "metrics": {
                "running": wf.metrics.running,
                "elapsed_s": wf.metrics.elapsed_s,
                "points_visited": wf.metrics.points_visited,
                "mean_error_mm": wf.metrics.mean_error_mm,
                "last_error_mm": wf.metrics.last_error_mm,
                "last_message": wf.metrics.last_message,
            },
        }
    )


@app.get("/api/settings")
def api_settings_get():
    dev = _devices()
    wa_x, wa_y = _manager.get_work_area()
    return jsonify(
        {
            "demo_mode": dev.demo_mode,
            "work_area": {"x_mm": wa_x, "y_mm": wa_y},
            "focus_threshold": _manager.get_focus_threshold(),
        }
    )


@app.post("/api/settings")
def api_settings_set():
    data = request.get_json(silent=True) or {}

    # demo mode (optional)
    demo_mode = data.get("demo_mode", None)
    if demo_mode is not None:
        dev = _manager.set_demo_mode(bool(demo_mode))
    else:
        dev = _devices()

    # work area (optional)
    wa = data.get("work_area") or {}
    x_mm = wa.get("x_mm", None)
    y_mm = wa.get("y_mm", None)
    if x_mm is not None and y_mm is not None:
        try:
            _manager.set_work_area(float(x_mm), float(y_mm))
        except Exception as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400

    # focus threshold (optional)
    if "focus_threshold" in data:
        try:
            _manager.set_focus_threshold(float(data.get("focus_threshold") or 0.0))
        except Exception as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400

    _ensure_connected()
    wa_x, wa_y = _manager.get_work_area()
    return jsonify(
        {
            "ok": True,
            "demo_mode": dev.demo_mode,
            "work_area": {"x_mm": wa_x, "y_mm": wa_y},
            "focus_threshold": _manager.get_focus_threshold(),
        }
    )


@app.post("/api/jog/start")
def api_jog_start():
    _ensure_connected()
    dev = _devices()
    data = request.get_json(silent=True) or {}
    direction = str(data.get("direction", ""))
    try:
        dev.grbl.start_jog(direction)
        return jsonify({"ok": True})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400


@app.post("/api/jog/stop")
def api_jog_stop():
    _ensure_connected()
    dev = _devices()
    try:
        dev.grbl.stop_jog()
        return jsonify({"ok": True})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400


@app.post("/api/set_start")
def api_set_start():
    _ensure_connected()
    dev = _devices()
    try:
        pos = dev.workflow.set_start_pos_from_machine()
        if dev.demo_script is not None:
            try:
                mpos = dev.grbl.get_position_mpos()
                if mpos is not None:
                    dev.demo_script.set_start_pos(mpos)
            except Exception:
                pass
        return jsonify({"ok": True, "start_pos": pos})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400


@app.post("/api/run_program")
def api_run_program():
    _ensure_connected()
    dev = _devices()
    try:
        if dev.demo_script is not None:
            # Regenerate random points for each run (if a start pos exists)
            sp = dev.workflow.state.start_pos
            if sp is not None:
                try:
                    dev.demo_script.set_start_pos((sp[0], sp[1], sp[2]))
                except Exception:
                    pass
        dev.workflow.run_async()
        return jsonify({"ok": True})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400


@app.post("/api/stop_program")
def api_stop_program():
    try:
        _devices().workflow.stop()
        return jsonify({"ok": True})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400


@app.post("/api/move_to")
def api_move_to():
    _ensure_connected()
    dev = _devices()
    data = request.get_json(silent=True) or {}
    x = data.get("x")
    y = data.get("y")
    z = data.get("z")
    if x is None or y is None:
        return jsonify({"ok": False, "error": "x/y required"}), 400

    try:
        if z is None:
            dev.grbl.move_abs_xy(float(x), float(y), feed=config.RAPID_FEED_MM_MIN)
        else:
            dev.grbl.move_abs_xyz(float(x), float(y), float(z), feed=config.RAPID_FEED_MM_MIN)
        return jsonify({"ok": True})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400


@app.get("/api/camera/features")
def api_camera_features():
    _ensure_connected()
    dev = _devices()
    try:
        if not hasattr(dev.camera, "get_settable_features"):
            return jsonify({"ok": True, "features": []})
        feats = dev.camera.get_settable_features()
        return jsonify({"ok": True, "features": feats})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.post("/api/camera/features/<path:name>")
def api_camera_feature_set(name: str):
    _ensure_connected()
    dev = _devices()
    data = request.get_json(silent=True) or {}
    execute = bool(data.get("execute", False))
    if "value" not in data and not execute:
        return jsonify({"ok": False, "error": "value or execute required"}), 400

    try:
        if not hasattr(dev.camera, "set_feature"):
            return jsonify({"ok": False, "error": "camera does not support feature setting"}), 400
        res = dev.camera.set_feature(name=str(name), value=data.get("value"), execute=execute)
        return jsonify({"ok": True, "result": res})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400


@app.get("/api/grbl/settings")
def api_grbl_settings():
    _ensure_connected()
    dev = _devices()
    try:
        if not hasattr(dev.grbl, "get_settings"):
            return jsonify({"ok": True, "settings": {}})
        settings = dev.grbl.get_settings(timeout=10.0)
        return jsonify({"ok": True, "settings": settings})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.post("/api/grbl/settings/<path:key>")
def api_grbl_setting_set(key: str):
    _ensure_connected()
    dev = _devices()
    data = request.get_json(silent=True) or {}
    if "value" not in data:
        return jsonify({"ok": False, "error": "value required"}), 400
    try:
        if not hasattr(dev.grbl, "set_setting"):
            return jsonify({"ok": False, "error": "grbl does not support setting changes"}), 400
        dev.grbl.set_setting(key=str(key), value=data.get("value"), timeout=10.0)
        return jsonify({"ok": True})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400


if __name__ == "__main__":
    app.run(host="127.0.0.1", port=5000, debug=True, threaded=True)
