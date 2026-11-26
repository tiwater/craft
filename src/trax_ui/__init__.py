"""Optional, detachable web research console for TRAX (`trax-ui`).

Strictly decoupled from headless `trax` core: `trax` has zero imports of `trax_ui`,
while `trax_ui` consumes public `trax` APIs and MCP endpoints read-only/via governance.
"""
from __future__ import annotations

from .server import TraxUIApp, create_ui_server
from .template import UI_HTML

__all__ = ["UI_HTML", "TraxUIApp", "create_ui_server"]
