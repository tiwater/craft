"""Optional standalone web observer and MCP operations console (`trax-ui`).

Strictly one-way dependency on `trax` public APIs (enforced by `make check-arch`).
Core `trax` modules never import `trax.ui`.
"""
from __future__ import annotations

from .server import TraxUIApp, create_ui_server

__all__ = ["TraxUIApp", "create_ui_server"]
