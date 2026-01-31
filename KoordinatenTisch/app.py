from __future__ import annotations

import os
from flask import Flask

from config import get_config
from web.routes import register_blueprints


def create_app() -> Flask:
    app = Flask(
        __name__,
        template_folder="web/templates",
        static_folder="web/static",
    )

    # Config laden (DEV/PROD/TEST)
    app.config.from_object(get_config())

    # Blueprints registrieren (Routen ausgelagert)
    register_blueprints(app)

    return app


if __name__ == "__main__":
    app = create_app()
    app.run(
        host=app.config.get("HOST", "127.0.0.1"),
        port=int(app.config.get("PORT", 5000)),
        debug=bool(app.config.get("DEBUG", True)),
    )
