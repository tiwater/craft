"""Pure-stdlib Model Context Protocol (MCP) JSON-RPC 2.0 server, persistent client, and inspector.

Zero third-party dependencies and zero hardcoded robot-model or domain assumptions.
- ``MCPServer`` implements JSON-RPC 2.0 (`initialize`, `tools/list`, `tools/call`,
  `resources/list`, `resources/read`) and can be served over HTTP (`/mcp`) or `stdio`.
- ``build_driver_mcp_server`` uses strict dependency inversion: any backend implementing
  ``register_mcp(server, lock)`` registers its own tools, resources, manifest, and URDF.
- ``MCPClient`` supports Streamable HTTP (`/mcp`), **persistent bidirectional `stdio`**
  subprocesses (for arbitrary external/unknown robots), and in-process servers.
"""
from __future__ import annotations

import atexit
import base64
from dataclasses import dataclass, field
import json
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
from typing import Any, Callable, Mapping, Sequence
import urllib.request

from .host_driver import SceneHostDriver
from .journal import Journal

MCP_PROTOCOL_VERSION = "2025-03-26"


def _utc_hms() -> str:
    return time.strftime("%H:%M:%S", time.gmtime())


@dataclass(frozen=True)
class MCPToolSpec:
    """MCP tool descriptor with JSON Schema and transactional annotations."""

    name: str
    description: str
    input_schema: dict[str, Any]
    read_only: bool = False
    idempotent: bool = True
    destructive: bool = False
    requires_reconciliation: bool = False

    def to_mcp_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "description": self.description,
            "inputSchema": self.input_schema,
            "annotations": {
                "readOnlyHint": self.read_only,
                "idempotentHint": self.idempotent,
                "destructiveHint": self.destructive,
                "openWorldHint": False,
                "traxReconciliationRequired": self.requires_reconciliation,
            },
        }


@dataclass(frozen=True)
class MCPResourceSpec:
    """MCP resource descriptor (e.g. embodiment manifest, URDF, live state, cameras)."""

    uri: str
    name: str
    description: str
    mime_type: str = "application/json"

    def to_mcp_dict(self) -> dict[str, Any]:
        return {
            "uri": self.uri,
            "name": self.name,
            "description": self.description,
            "mimeType": self.mime_type,
        }


class MCPServer:
    """Pure-stdlib JSON-RPC 2.0 MCP server for embodiment tools and resources."""

    def __init__(
        self,
        name: str,
        version: str = "0.3.0",
        *,
        embodiment_meta: Mapping[str, Any] | None = None,
    ) -> None:
        self.name = name
        self.version = version
        self.embodiment_meta = dict(embodiment_meta or {})
        self._tools: dict[str, tuple[MCPToolSpec, Callable[[dict[str, Any]], Any]]] = {}
        self._resources: dict[str, tuple[MCPResourceSpec, Callable[[], tuple[str, bytes | str]]]] = {}

    def set_embodiment_meta(self, meta: Mapping[str, Any]) -> None:
        self.embodiment_meta.update(dict(meta))

    def register_tool(self, spec: MCPToolSpec, handler: Callable[[dict[str, Any]], Any]) -> None:
        self._tools[spec.name] = (spec, handler)

    def register_resource(
        self,
        spec: MCPResourceSpec,
        reader: Callable[[], tuple[str, bytes | str]],
    ) -> None:
        self._resources[spec.uri] = (spec, reader)

    def server_info(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "version": self.version,
            "protocolVersion": MCP_PROTOCOL_VERSION,
            **self.embodiment_meta,
        }

    def list_tools_dicts(self) -> list[dict[str, Any]]:
        return [spec.to_mcp_dict() for spec, _ in self._tools.values()]

    def list_resources_dicts(self) -> list[dict[str, Any]]:
        return [spec.to_mcp_dict() for spec, _ in self._resources.values()]

    def handle_rpc(self, req: Mapping[str, Any]) -> dict[str, Any]:
        req_id = req.get("id")
        method = str(req.get("method") or "")
        params = req.get("params") or {}
        if not isinstance(params, Mapping):
            return {
                "jsonrpc": "2.0",
                "id": req_id,
                "error": {"code": -32602, "message": "Invalid params (expected object)"},
            }

        try:
            if method == "initialize":
                return {
                    "jsonrpc": "2.0",
                    "id": req_id,
                    "result": {
                        "protocolVersion": MCP_PROTOCOL_VERSION,
                        "capabilities": {
                            "tools": {"listChanged": False},
                            "resources": {"subscribe": False, "listChanged": False},
                        },
                        "serverInfo": self.server_info(),
                    },
                }

            if method in ("notifications/initialized", "ping"):
                return {"jsonrpc": "2.0", "id": req_id, "result": {}}

            if method == "tools/list":
                return {
                    "jsonrpc": "2.0",
                    "id": req_id,
                    "result": {"tools": self.list_tools_dicts()},
                }

            if method == "tools/call":
                tool_name = str(params.get("name") or "")
                arguments = dict(params.get("arguments") or {})
                if tool_name not in self._tools:
                    return {
                        "jsonrpc": "2.0",
                        "id": req_id,
                        "error": {"code": -32602, "message": f"Unknown MCP tool: {tool_name}"},
                    }
                _, handler = self._tools[tool_name]
                out = handler(arguments)
                return {
                    "jsonrpc": "2.0",
                    "id": req_id,
                    "result": {
                        "content": [{"type": "text", "text": json.dumps(out, ensure_ascii=False)}],
                        "structuredContent": out,
                        "isError": False,
                    },
                }

            if method == "resources/list":
                return {
                    "jsonrpc": "2.0",
                    "id": req_id,
                    "result": {"resources": self.list_resources_dicts()},
                }

            if method == "resources/read":
                uri = str(params.get("uri") or "")
                if uri not in self._resources:
                    return {
                        "jsonrpc": "2.0",
                        "id": req_id,
                        "error": {"code": -32602, "message": f"Unknown MCP resource: {uri}"},
                    }
                _, reader = self._resources[uri]
                mime, payload = reader()
                if isinstance(payload, bytes):
                    content_item = {
                        "uri": uri,
                        "mimeType": mime,
                        "blob": base64.b64encode(payload).decode("ascii"),
                    }
                else:
                    content_item = {
                        "uri": uri,
                        "mimeType": mime,
                        "text": str(payload),
                    }
                return {
                    "jsonrpc": "2.0",
                    "id": req_id,
                    "result": {"contents": [content_item]},
                }

            return {
                "jsonrpc": "2.0",
                "id": req_id,
                "error": {"code": -32601, "message": f"Method not found: {method}"},
            }
        except Exception as exc:  # noqa: BLE001
            return {
                "jsonrpc": "2.0",
                "id": req_id,
                "error": {"code": -32000, "message": str(exc)},
            }

    def serve_stdio(self) -> None:
        """Run a persistent newline-delimited JSON-RPC 2.0 loop over stdin/stdout."""
        for raw_line in sys.stdin:
            line = raw_line.strip()
            if not line:
                continue
            try:
                req = json.loads(line)
            except ValueError as exc:
                resp = {
                    "jsonrpc": "2.0",
                    "id": None,
                    "error": {"code": -32700, "message": f"Parse error: {exc}"},
                }
            else:
                if not isinstance(req, Mapping):
                    resp = {
                        "jsonrpc": "2.0",
                        "id": None,
                        "error": {"code": -32600, "message": "Invalid JSON-RPC request"},
                    }
                else:
                    resp = self.handle_rpc(req)
                    # JSON-RPC 2.0 notifications omit id and expect no reply
                    if "id" not in req:
                        continue
            sys.stdout.write(json.dumps(resp, ensure_ascii=False) + "\n")
            sys.stdout.flush()


def _locked_call(lock: threading.RLock | threading.Lock, fn: Callable[[], Any]) -> Any:
    with lock:
        return fn()


def _register_camera_resources(server: MCPServer, driver: SceneHostDriver, authority: str) -> None:
    for cam in driver.list_cameras():
        cam_name = str(cam)

        def _make_cam_reader(c: str) -> Callable[[], tuple[str, bytes | str]]:
            def _read() -> tuple[str, bytes | str]:
                img = driver.fetch_camera_image(c)
                if img is None:
                    return ("text/plain", f"Camera {c} unavailable")
                return (img[1], img[0])

            return _read

        server.register_resource(
            MCPResourceSpec(
                uri=f"trax://cameras/{cam_name}",
                name=f"Camera: {cam_name}",
                description=f"Live rendered camera frame from {authority} ({cam_name}).",
                mime_type="image/jpeg",
            ),
            _make_cam_reader(cam_name),
        )


def build_driver_mcp_server(
    driver: SceneHostDriver,
    lock: threading.RLock | threading.Lock | None = None,
) -> MCPServer:
    """Expose any TRAX `SceneHostDriver` as an MCP server via dependency inversion.

    Any backend plugin implementing ``register_mcp(server, lock)`` owns and registers
    its own tools, resources, embodiment manifest, and URDF. Legacy drivers without
    ``register_mcp`` receive generic contract-reflective discovery & scene tools only.
    """
    guard = lock if lock is not None else threading.RLock()
    cap = None
    try:
        cap = driver.discover_capabilities(allow_busy=True)
    except Exception:  # noqa: BLE001
        cap = None
    disc = cap.discovered if cap else None
    authority = disc.execution_authority if disc else driver.driver_name

    srv = MCPServer(
        name=f"trax-mcp-{authority}",
        version="0.3.0",
        embodiment_meta={
            "execution_authority": authority,
            "perception_owner": disc.perception_owner if disc else "host",
            "evidence_source": driver.evidence_source,
            "driver": driver.driver_name,
            "sdk_contract_sha256": (disc.sdk_contract_sha256 if disc else "")[:16],
        },
    )

    # Preferred plugin path: backend registers its own tools, resources, manifest, and URDF.
    if hasattr(driver, "register_mcp") and callable(getattr(driver, "register_mcp")):
        driver.register_mcp(srv, guard)  # type: ignore[attr-defined]
        if hasattr(driver, "embodiment_manifest") and callable(getattr(driver, "embodiment_manifest")):
            manifest = driver.embodiment_manifest()  # type: ignore[attr-defined]
            manifest_dict = manifest.to_dict() if hasattr(manifest, "to_dict") else dict(manifest)
            srv.set_embodiment_meta(
                {
                    "embodiment_id": manifest_dict.get("embodiment_id", authority),
                    "urdf_uri": manifest_dict.get("urdf_uri"),
                    "urdf_sha256": manifest_dict.get("urdf_sha256", ""),
                    "mesh_base_url": manifest_dict.get("mesh_base_url"),
                }
            )
            if "trax://embodiment/manifest" not in srv._resources:
                srv.register_resource(
                    MCPResourceSpec(
                        uri="trax://embodiment/manifest",
                        name="Embodiment Manifest",
                        description="Authoritative backend-owned embodiment metadata, URDF digest, and physics notes.",
                        mime_type="application/json",
                    ),
                    lambda m=manifest_dict: ("application/json", json.dumps(m, indent=2)),
                )
        _register_camera_resources(srv, driver, authority)
        return srv

    # Generic fallback for legacy drivers that only implement SceneHostDriver
    srv.register_tool(
        MCPToolSpec(
            name="observe_scene",
            description="Read authoritative host-owned scene observation snapshot.",
            input_schema={"type": "object", "properties": {}, "additionalProperties": False},
            read_only=True,
            idempotent=True,
        ),
        lambda args: driver.observe_scene().raw_snapshot or driver.observe_scene().to_dict(),
    )
    srv.register_tool(
        MCPToolSpec(
            name="discover_capabilities",
            description="Query host execution authority, perception owner, allowed methods, and contract digest.",
            input_schema={"type": "object", "properties": {}, "additionalProperties": False},
            read_only=True,
            idempotent=True,
        ),
        lambda args: driver.discover_capabilities(allow_busy=True).to_dict(),
    )
    srv.register_tool(
        MCPToolSpec(
            name="reset_scenario",
            description="Reset host environment to a named scenario preset.",
            input_schema={
                "type": "object",
                "properties": {"preset": {"type": "string", "default": "default"}},
            },
            read_only=False,
            idempotent=True,
        ),
        lambda args: _locked_call(guard, lambda: driver.reset_scenario({"preset": str(args.get("preset") or "default")})),
    )
    srv.register_resource(
        MCPResourceSpec(
            uri="trax://scene/state",
            name="Live Authoritative Scene Snapshot",
            description="Current host-owned scene observation JSON.",
            mime_type="application/json",
        ),
        lambda: ("application/json", json.dumps(driver.observe_scene().raw_snapshot or driver.observe_scene().to_dict(), indent=2)),
    )
    srv.register_resource(
        MCPResourceSpec(
            uri="trax://capabilities/contract",
            name="Host Capability Contract",
            description="Authoritative capability discovery receipt and SHA-256 contract digest.",
            mime_type="application/json",
        ),
        lambda: ("application/json", json.dumps(driver.discover_capabilities(allow_busy=True).to_dict(), indent=2)),
    )
    _register_camera_resources(srv, driver, authority)
    return srv


def format_mcp_tools_for_prompt(tools: Sequence[Mapping[str, Any]]) -> str:
    """Format dynamically discovered MCP tools/list schemas into a compact LLM grounding contract."""
    lines = ["DYNAMICALLY DISCOVERED MCP TOOLS (`tools/list`):"]
    for t in tools:
        ann = t.get("annotations") or {}
        tags = []
        if ann.get("readOnlyHint"):
            tags.append("read-only")
        if ann.get("idempotentHint") is False:
            tags.append("NON-IDEMPOTENT/JOURNALED")
        if ann.get("destructiveHint"):
            tags.append("destructive")
        tag_str = f" [{', '.join(tags)}]" if tags else ""
        schema = json.dumps(t.get("inputSchema") or {}, ensure_ascii=False)
        lines.append(f"- `{t.get('name')}`{tag_str}: {t.get('description', '')} | schema={schema}")
    return "\n".join(lines)


class _PersistentStdioSession:
    """Persistent bidirectional newline-delimited JSON-RPC 2.0 subprocess session."""

    def __init__(self, command: Sequence[str]) -> None:
        self.command = [str(c) for c in command]
        self._proc: subprocess.Popen[str] | None = None
        self._lock = threading.Lock()
        self._stdout_q: queue.Queue[str | None] = queue.Queue()
        self._stderr_lines: list[str] = []
        self._reader_thread: threading.Thread | None = None
        self._stderr_thread: threading.Thread | None = None

    def _ensure_started(self) -> subprocess.Popen[str]:
        if self._proc is not None and self._proc.poll() is None:
            return self._proc
        self.close()
        self._stdout_q = queue.Queue()
        self._stderr_lines = []
        self._proc = subprocess.Popen(
            self.command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )

        proc = self._proc

        def _read_stdout() -> None:
            assert proc.stdout is not None
            try:
                for line in proc.stdout:
                    self._stdout_q.put(line)
            finally:
                self._stdout_q.put(None)

        def _read_stderr() -> None:
            assert proc.stderr is not None
            for line in proc.stderr:
                self._stderr_lines.append(line.rstrip())
                if len(self._stderr_lines) > 40:
                    self._stderr_lines = self._stderr_lines[-40:]

        self._reader_thread = threading.Thread(target=_read_stdout, daemon=True)
        self._stderr_thread = threading.Thread(target=_read_stderr, daemon=True)
        self._reader_thread.start()
        self._stderr_thread.start()
        return proc

    def send_rpc(self, req_obj: Mapping[str, Any], *, timeout_s: float) -> dict[str, Any]:
        req_id = req_obj.get("id")
        with self._lock:
            proc = self._ensure_started()
            assert proc.stdin is not None
            try:
                proc.stdin.write(json.dumps(req_obj, ensure_ascii=False) + "\n")
                proc.stdin.flush()
            except BrokenPipeError as exc:
                err_tail = "\n".join(self._stderr_lines[-8:])
                self.close()
                raise RuntimeError(f"Stdio MCP process terminated unexpectedly: {err_tail}") from exc

            deadline = time.monotonic() + max(0.5, float(timeout_s))
            while True:
                rem = deadline - time.monotonic()
                if rem <= 0:
                    err_tail = "\n".join(self._stderr_lines[-8:])
                    raise TimeoutError(
                        f"Stdio MCP RPC `{req_obj.get('method')}` timed out after {timeout_s:.1f}s. stderr={err_tail}"
                    )
                try:
                    line = self._stdout_q.get(timeout=min(rem, 0.25))
                except queue.Empty:
                    if proc.poll() is not None:
                        err_tail = "\n".join(self._stderr_lines[-8:])
                        rc = proc.returncode
                        self.close()
                        raise RuntimeError(f"Stdio MCP server exited (rc={rc}): {err_tail}")
                    continue

                if line is None:
                    err_tail = "\n".join(self._stderr_lines[-8:])
                    rc = proc.poll()
                    self.close()
                    raise RuntimeError(f"Stdio MCP server closed stdout (rc={rc}): {err_tail}")

                stripped = line.strip()
                if not stripped:
                    continue
                try:
                    msg = json.loads(stripped)
                except ValueError:
                    # Ignore non-JSON diagnostic log lines emitted on stdout
                    continue
                if isinstance(msg, dict) and (req_id is None or msg.get("id") == req_id):
                    return msg

    def close(self) -> None:
        proc = self._proc
        self._proc = None
        if proc is None:
            return
        try:
            if proc.stdin and not proc.stdin.closed:
                proc.stdin.close()
        except Exception:  # noqa: BLE001
            pass
        if proc.poll() is None:
            try:
                proc.terminate()
                proc.wait(timeout=1.5)
            except Exception:  # noqa: BLE001
                try:
                    proc.kill()
                    proc.wait(timeout=1.0)
                except Exception:  # noqa: BLE001
                    pass


class MCPClient:
    """Pure-stdlib MCP JSON-RPC 2.0 client supporting HTTP (`/mcp`), persistent `stdio`, and local servers."""

    def __init__(
        self,
        *,
        transport: str = "http",
        url: str | None = None,
        command: Sequence[str] | None = None,
        local_server: MCPServer | None = None,
        timeout_s: float = 15.0,
    ) -> None:
        self.transport = transport
        self.url = (url or "").strip()
        self.command = list(command or [])
        self.local_server = local_server
        self.timeout_s = timeout_s
        self._seq = 0
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self._stdio_session: _PersistentStdioSession | None = (
            _PersistentStdioSession(self.command) if transport == "stdio" and self.command else None
        )
        if self._stdio_session is not None:
            atexit.register(self.close)

    def __enter__(self) -> MCPClient:
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self.close()

    def close(self) -> None:
        if self._stdio_session is not None:
            self._stdio_session.close()

    def _next_id(self) -> int:
        self._seq += 1
        return self._seq

    def rpc(self, method: str, params: Mapping[str, Any] | None = None) -> tuple[dict[str, Any], dict[str, Any], float]:
        """Send one JSON-RPC 2.0 request and return (raw_request, raw_response, latency_ms)."""
        req_obj: dict[str, Any] = {"jsonrpc": "2.0", "id": self._next_id(), "method": method}
        if params is not None:
            req_obj["params"] = dict(params)
        t0 = time.monotonic()

        if self.transport == "local":
            if self.local_server is None:
                raise RuntimeError("Local MCP server is not bound")
            resp_obj = self.local_server.handle_rpc(req_obj)
        elif self.transport == "http":
            if not self.url:
                raise ValueError("HTTP MCP transport requires a target URL (e.g. http://127.0.0.1:8880/mcp)")
            raw = json.dumps(req_obj).encode("utf-8")
            http_req = urllib.request.Request(
                self.url,
                data=raw,
                headers={
                    "Content-Type": "application/json",
                    "Accept": "application/json, text/event-stream",
                },
                method="POST",
            )
            with self._opener.open(http_req, timeout=self.timeout_s) as resp:
                body = resp.read().decode("utf-8", errors="replace").strip()
            if body.startswith("data:") or "\ndata:" in body:
                data_lines = [ln[5:].strip() for ln in body.splitlines() if ln.startswith("data:")]
                body = data_lines[-1] if data_lines else "{}"
            resp_obj = json.loads(body) if body else {}
        elif self.transport == "stdio":
            if not self.command:
                raise ValueError("Stdio MCP transport requires a command list")
            if self._stdio_session is None:
                self._stdio_session = _PersistentStdioSession(self.command)
            resp_obj = self._stdio_session.send_rpc(req_obj, timeout_s=self.timeout_s)
        else:
            raise ValueError(f"Unsupported MCP transport: {self.transport}")

        ms = round((time.monotonic() - t0) * 1000.0, 2)
        return req_obj, resp_obj, ms


class MCPInspectorController:
    """Stateful MCP discovery & live tool-invocation workbench."""

    def __init__(self, workspace_dir: Path, get_active_driver: Callable[[], SceneHostDriver], lock: threading.Lock) -> None:
        self.workspace_dir = workspace_dir
        self._get_active_driver = get_active_driver
        self._driver_lock = lock
        self._state_lock = threading.RLock()
        runtime_dir = self.workspace_dir / "runtime"
        runtime_dir.mkdir(parents=True, exist_ok=True)
        self._journal_path = runtime_dir / "mcp_journal.sqlite"

        self._endpoint_label: str = "Active Host (Auto)"
        self._transport: str = "http"
        self._url: str = ""
        self._command: list[str] = []
        self._persistent_stdio_client: MCPClient | None = None
        self._server_info: dict[str, Any] | None = None
        self._tools: list[dict[str, Any]] = []
        self._resources: list[dict[str, Any]] = []
        self._wire_log: list[dict[str, Any]] = []
        self._last_error: str | None = None

    def close(self) -> None:
        if self._persistent_stdio_client is not None:
            self._persistent_stdio_client.close()
            self._persistent_stdio_client = None

    def _resolve_client(self, body: Mapping[str, Any] | None = None) -> tuple[MCPClient, str, str, str, list[str]]:
        b = dict(body or {})
        transport = str(b.get("transport") or "auto").strip().lower()
        url = str(b.get("url") or "").strip()
        cmd = b.get("command") or []
        if isinstance(cmd, str):
            cmd = [tok for tok in cmd.split() if tok]
        cmd_list = [str(x) for x in cmd]

        if transport == "auto" or (transport == "http" and not url):
            drv = self._get_active_driver()
            if drv.driver_name == "http" and hasattr(drv, "config"):
                base = drv.config.host.normalized_base_url  # type: ignore[attr-defined]
                mcp_url = f"{base}/mcp"
                return (
                    MCPClient(transport="http", url=mcp_url, timeout_s=15.0),
                    f"Active HTTP Host ({mcp_url})",
                    "http",
                    mcp_url,
                    [],
                )
            local_srv = build_driver_mcp_server(drv, self._driver_lock)
            return (
                MCPClient(transport="local", local_server=local_srv),
                f"In-Process Driver ({drv.driver_name})",
                "local",
                "local://active-driver",
                [],
            )

        if transport == "http":
            return MCPClient(transport="http", url=url, timeout_s=15.0), url, "http", url, []
        if transport == "stdio":
            with self._state_lock:
                if self._persistent_stdio_client is not None and self._persistent_stdio_client.command == cmd_list:
                    client = self._persistent_stdio_client
                else:
                    if self._persistent_stdio_client is not None:
                        self._persistent_stdio_client.close()
                    client = MCPClient(transport="stdio", command=cmd_list, timeout_s=15.0)
                    self._persistent_stdio_client = client
            return client, " ".join(cmd_list), "stdio", "", cmd_list
        if transport == "local":
            drv = self._get_active_driver()
            local_srv = build_driver_mcp_server(drv, self._driver_lock)
            return (
                MCPClient(transport="local", local_server=local_srv),
                f"In-Process Driver ({drv.driver_name})",
                "local",
                "local://active-driver",
                [],
            )
        raise ValueError(f"Unsupported transport: {transport}")

    def _append_wire(
        self,
        *,
        method: str,
        request: dict[str, Any],
        response: dict[str, Any],
        latency_ms: float,
        journal_id: str | None = None,
    ) -> None:
        resp_copy = json.loads(json.dumps(response))
        contents = (resp_copy.get("result") or {}).get("contents")
        if isinstance(contents, list):
            for item in contents:
                if isinstance(item, dict) and isinstance(item.get("blob"), str) and len(item["blob"]) > 240:
                    item["blob"] = f"<base64 {len(item['blob'])} chars omitted from wire log>"
        self._wire_log.append(
            {
                "t": _utc_hms(),
                "method": method,
                "latency_ms": latency_ms,
                "ok": "error" not in response,
                "journal_id": journal_id,
                "request": request,
                "response": resp_copy,
            }
        )
        if len(self._wire_log) > 60:
            self._wire_log = self._wire_log[-60:]

    def inspect(self, body: Mapping[str, Any] | None = None) -> dict[str, Any]:
        client, label, transport, url, cmd = self._resolve_client(body)
        try:
            init_req, init_resp, init_ms = client.rpc(
                "initialize",
                {
                    "protocolVersion": MCP_PROTOCOL_VERSION,
                    "capabilities": {},
                    "clientInfo": {"name": "trax-ui-inspector", "version": "0.3.0"},
                },
            )
            tools_req, tools_resp, tools_ms = client.rpc("tools/list", {})
            res_req, res_resp, res_ms = client.rpc("resources/list", {})

            if "error" in init_resp:
                raise RuntimeError(init_resp["error"].get("message", "initialize failed"))
            if "error" in tools_resp:
                raise RuntimeError(tools_resp["error"].get("message", "tools/list failed"))

            server_info = (init_resp.get("result") or {}).get("serverInfo") or {}
            tools = (tools_resp.get("result") or {}).get("tools") or []
            resources = (res_resp.get("result") or {}).get("resources") or []

            with self._state_lock:
                self._endpoint_label = label
                self._transport = transport
                self._url = url
                self._command = cmd
                self._server_info = server_info
                self._tools = list(tools)
                self._resources = list(resources)
                self._last_error = None
                self._append_wire(method="initialize", request=init_req, response=init_resp, latency_ms=init_ms)
                self._append_wire(method="tools/list", request=tools_req, response=tools_resp, latency_ms=tools_ms)
                self._append_wire(method="resources/list", request=res_req, response=res_resp, latency_ms=res_ms)
        except Exception as exc:  # noqa: BLE001
            with self._state_lock:
                self._last_error = str(exc)
            raise
        return self.get_state()

    def call_tool(self, body: Mapping[str, Any]) -> dict[str, Any]:
        tool_name = str(body.get("tool_name") or "").strip()
        if not tool_name:
            raise ValueError("tool_name is required")
        arguments = dict(body.get("arguments") or {})
        use_journal = bool(body.get("use_journal", True))

        with self._state_lock:
            tool_meta = next((t for t in self._tools if t.get("name") == tool_name), None)
            default_body = {
                "transport": body.get("transport") or self._transport or "auto",
                "url": body.get("url") or self._url,
                "command": body.get("command") or self._command,
            }
        client, _, _, _, _ = self._resolve_client(default_body)

        ann = (tool_meta or {}).get("annotations") or {}
        is_non_idempotent = (ann.get("idempotentHint") is False) or bool(ann.get("destructiveHint"))
        journal_id: str | None = None

        if use_journal and is_non_idempotent:
            journal_id = f"mcp-{tool_name}-{time.time_ns()}"
            with Journal(self._journal_path) as journal:
                journal.claim(journal_id, {"action": f"mcp:{tool_name}", "arguments": arguments})

        req_obj, resp_obj, ms = client.rpc("tools/call", {"name": tool_name, "arguments": arguments})

        if journal_id is not None:
            receipt = {"status": "error" if "error" in resp_obj else "done", "latency_ms": ms}
            with Journal(self._journal_path) as journal:
                journal.record(journal_id, receipt)

        with self._state_lock:
            self._append_wire(
                method=f"tools/call:{tool_name}",
                request=req_obj,
                response=resp_obj,
                latency_ms=ms,
                journal_id=journal_id,
            )

        return {
            "tool_name": tool_name,
            "latency_ms": ms,
            "journal_id": journal_id,
            "non_idempotent": is_non_idempotent,
            "request": req_obj,
            "response": resp_obj,
            "state": self.get_state(),
        }

    def read_resource(self, body: Mapping[str, Any]) -> dict[str, Any]:
        uri = str(body.get("uri") or "").strip()
        if not uri:
            raise ValueError("uri is required")
        with self._state_lock:
            default_body = {
                "transport": body.get("transport") or self._transport or "auto",
                "url": body.get("url") or self._url,
                "command": body.get("command") or self._command,
            }
        client, _, _, _, _ = self._resolve_client(default_body)
        req_obj, resp_obj, ms = client.rpc("resources/read", {"uri": uri})
        with self._state_lock:
            self._append_wire(method=f"resources/read:{uri}", request=req_obj, response=resp_obj, latency_ms=ms)
        return {
            "uri": uri,
            "latency_ms": ms,
            "request": req_obj,
            "response": resp_obj,
            "state": self.get_state(),
        }

    def record_external_wire(self, entry: Mapping[str, Any]) -> None:
        with self._state_lock:
            self._append_wire(
                method=str(entry.get("method") or "rpc"),
                request=dict(entry.get("request") or {}),
                response=dict(entry.get("response") or {}),
                latency_ms=float(entry.get("latency_ms") or 0.0),
                journal_id=entry.get("journal_id"),
            )

    def get_state(self) -> dict[str, Any]:
        with self._state_lock:
            return {
                "endpoint_label": self._endpoint_label,
                "transport": self._transport,
                "url": self._url,
                "command": list(self._command),
                "server_info": self._server_info,
                "tools": list(self._tools),
                "resources": list(self._resources),
                "prompt_contract_preview": format_mcp_tools_for_prompt(self._tools) if self._tools else "",
                "wire_log": list(self._wire_log),
                "last_error": self._last_error,
            }
