from __future__ import annotations

from flask import Flask

from .main import bp as main_bp
from .health import bp as health_bp
from .camera import bp as camera_bp


def register_blueprints(app: Flask) -> None:
    app.register_blueprint(main_bp)
    app.register_blueprint(health_bp)
    app.register_blueprint(camera_bp)
