"""Standalone, host-independent web observer and operations interface (TRAX UI).

Built purely on Python's stdlib ``http.server`` with zero external web dependencies.
Consumes any ``SceneHostDriver`` read-only for perception/cameras and delegates all
orchestration through TRAX APIs.
"""
from __future__ import annotations

from .studio import (
    STUDIO_HTML as UI_HTML,
    TraxStudioApp as TraxUIApp,
    create_studio_server as create_ui_server,
)

__all__ = ["UI_HTML", "TraxUIApp", "create_ui_server"]
