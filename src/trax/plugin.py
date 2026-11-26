"""Embodiment backend plugin ABI, entry-point discovery, and safe URDF/mesh asset helpers.

Pure Python stdlib (zero third-party dependencies). Implements strict dependency inversion:
- Unknown/external/physical robots plug in out-of-process via MCP (`--mcp-url` or `--mcp-stdio`)
  with zero Python code changes in TRAX core.
- Built-in and third-party Python backends register via standard Python entry points
  (`group="trax.backends"`) and register their own MCP tools, resources, and URDF assets
  through ``register_mcp(server, lock)``.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from importlib import metadata as importlib_metadata
import mimetypes
from pathlib import Path
import re
import threading
from typing import TYPE_CHECKING, Any, Callable, Mapping, Protocol, runtime_checkable

from .config import SceneConfig
from .contracts import digest

if TYPE_CHECKING:
    from .mcp_protocol import MCPServer


TRAX_BACKEND_ENTRYPOINT_GROUP = "trax.backends"
LEGACY_DRIVER_ENTRYPOINT_GROUP = "craft.drivers"

ALLOWED_ROBOT_ASSET_EXTS = frozenset(
    {".urdf", ".xml", ".stl", ".dae", ".obj", ".glb", ".gltf", ".png", ".jpg", ".jpeg"}
)

_ASSET_MIME_MAP: dict[str, str] = {
    ".urdf": "application/xml; charset=utf-8",
    ".xml": "application/xml; charset=utf-8",
    ".stl": "model/stl",
    ".dae": "model/vnd.collada+xml",
    ".obj": "text/plain; charset=utf-8",
    ".glb": "model/gltf-binary",
    ".gltf": "model/gltf+json",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
}


@dataclass(frozen=True)
class EmbodimentManifest:
    """Self-describing hardware/simulator metadata owned by an embodiment backend."""

    embodiment_id: str
    display_name: str
    evidence_source: str  # fixture | simulator | recorded_data | physical_robot
    execution_authority: str
    perception_owner: str = "host"
    urdf_uri: str | None = None
    urdf_sha256: str = ""
    mesh_base_url: str | None = None
    kinematics_notes: str = ""
    physics_approximations: tuple[str, ...] = ()
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "embodiment_id": self.embodiment_id,
            "display_name": self.display_name,
            "evidence_source": self.evidence_source,
            "execution_authority": self.execution_authority,
            "perception_owner": self.perception_owner,
            "urdf_uri": self.urdf_uri,
            "urdf_sha256": self.urdf_sha256,
            "mesh_base_url": self.mesh_base_url,
            "kinematics_notes": self.kinematics_notes,
            "physics_approximations": list(self.physics_approximations),
        }
        if self.extra:
            out["extra"] = dict(self.extra)
        return out


@runtime_checkable
class EmbodimentBackendPlugin(Protocol):
    """Optional plugin extension interface implemented by MCP-native embodiment backends."""

    driver_name: str
    evidence_source: str

    def embodiment_manifest(self) -> EmbodimentManifest: ...

    def register_mcp(self, server: MCPServer, lock: threading.RLock) -> None: ...


BackendPluginFactory = Callable[[SceneConfig], Any]

_RUNTIME_BACKEND_PLUGINS: dict[str, BackendPluginFactory] = {}


def register_backend_plugin(name: str, factory: BackendPluginFactory) -> None:
    """Register an in-process embodiment backend plugin dynamically at runtime."""
    key = (name or "").strip()
    if not key:
        raise ValueError("Backend plugin name must be non-empty")
    _RUNTIME_BACKEND_PLUGINS[key] = factory


def discover_entrypoint_backends() -> dict[str, BackendPluginFactory]:
    """Discover installed Python backend plugins from `trax.backends` (and legacy `craft.drivers`)."""
    discovered: dict[str, BackendPluginFactory] = {}
    for group in (LEGACY_DRIVER_ENTRYPOINT_GROUP, TRAX_BACKEND_ENTRYPOINT_GROUP):
        try:
            eps = importlib_metadata.entry_points(group=group)
        except TypeError:
            eps = importlib_metadata.entry_points().get(group, ())  # type: ignore[arg-type]
        for ep in eps:
            try:
                discovered[ep.name] = ep.load()
            except Exception:  # noqa: BLE001
                continue
    return discovered


def list_backend_plugins() -> tuple[str, ...]:
    """List all runtime-registered and entry-point-discovered embodiment backend plugins."""
    names = set(_RUNTIME_BACKEND_PLUGINS.keys()) | set(discover_entrypoint_backends().keys())
    return tuple(sorted(names))


def resolve_backend_factory(name: str) -> BackendPluginFactory | None:
    """Resolve a backend plugin factory by name (runtime registrations override entry points)."""
    key = (name or "").strip()
    if key in _RUNTIME_BACKEND_PLUGINS:
        return _RUNTIME_BACKEND_PLUGINS[key]
    ep_map = discover_entrypoint_backends()
    return ep_map.get(key)


def asset_content_type(path: Path) -> str:
    ext = path.suffix.lower()
    if ext in _ASSET_MIME_MAP:
        return _ASSET_MIME_MAP[ext]
    guessed, _ = mimetypes.guess_type(path.name)
    return guessed or "application/octet-stream"


def resolve_safe_asset_file(asset_root: Path, rel_path: str) -> tuple[Path, str] | None:
    """Safely resolve a relative robot asset path under `asset_root` without path traversal.

    Rejects absolute paths, `..` traversal, symlink escapes outside `asset_root`,
    and any file extension outside `ALLOWED_ROBOT_ASSET_EXTS`.
    """
    cleaned = (rel_path or "").strip().lstrip("/")
    if not cleaned or "\x00" in cleaned:
        return None
    parts = Path(cleaned).parts
    if any(p in ("..", "") for p in parts):
        return None
    root = asset_root.resolve()
    candidate = (root / cleaned).resolve()
    try:
        candidate.relative_to(root)
    except ValueError:
        return None
    if not candidate.is_file():
        return None
    if candidate.suffix.lower() not in ALLOWED_ROBOT_ASSET_EXTS:
        return None
    return candidate, asset_content_type(candidate)


def read_safe_robot_asset(asset_root: Path, rel_path: str) -> tuple[bytes, str, str] | None:
    """Read a validated robot asset and return `(data, content_type, etag)`."""
    resolved = resolve_safe_asset_file(asset_root, rel_path)
    if resolved is None:
        return None
    path, mime = resolved
    data = path.read_bytes()
    etag = f'"{digest({"path": path.name, "size": len(data), "head": data[:256].hex()})[:16]}"'
    return data, mime, etag


_MESH_FILENAME_RE = re.compile(r'(<mesh\b[^>]*\bfilename\s*=\s*)(["\'])([^"\']+)(\2)', re.IGNORECASE)


def rewrite_urdf_mesh_uris(urdf_xml: str, *, asset_url_prefix: str = "/assets/robot") -> str:
    """Rewrite ROS `package://<pkg>/...` and relative mesh paths in URDF XML for browser URDFLoader."""
    prefix = asset_url_prefix.rstrip("/")

    def _replace(match: re.Match[str]) -> str:
        head, quote, raw_uri, tail = match.groups()
        uri = raw_uri.strip()
        if uri.startswith(("http://", "https://", "data:", "/")):
            return match.group(0)
        if uri.startswith("package://"):
            rest = uri[len("package://") :]
            # Strip package name prefix: package://pkg_name/meshes/foo.stl -> meshes/foo.stl
            if "/" in rest:
                _, rel = rest.split("/", 1)
            else:
                rel = rest
            rel = rel.lstrip("/")
        else:
            rel = uri.lstrip("./")
        return f"{head}{quote}{prefix}/{rel}{tail}"

    return _MESH_FILENAME_RE.sub(_replace, urdf_xml)
