"""Standalone HTTP Simulation Host Server (`craft serve-sim-host`).

Wraps any local `SceneHostDriver` (e.g., `mujoco_cafe` or `fixture`) behind the
exact standard HTTP host contract consumed by `HttpHostDriver` / `EdgeHostAdapter`
and external control planes (`ticos` / `ticos-edge`):
- GET  /edge-scene
- GET  /cap/state
- POST /cap/execute
- POST /cap/reset
- POST /cap/confirm
- GET  /cameras/<name>.jpg
- POST /v1/sim/testbed/setup
- GET  /v1/sim/testbed/oracle

Single simulation kernel, two execution modes:
1. In-process (`--driver mujoco_cafe`) for fast offline benchmarks & standalone demos
2. Standalone HTTP service (`craft serve-sim-host` + `--driver http`) exercising the
   exact production HTTP contract, Journal intent-before-dispatch, and reconciliation path.
"""
from __future__ import annotations

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
from typing import Any, Mapping
from urllib.parse import urlparse

from .host_driver import SceneHostDriver
from .mcp_protocol import build_driver_mcp_server


class StandaloneSimHostApp:
    """Thread-safe HTTP + MCP JSON-RPC 2.0 facade over a local `SceneHostDriver`."""

    def __init__(self, driver: SceneHostDriver) -> None:
        self.driver = driver
        self._lock = threading.RLock()
        self.mcp_server = build_driver_mcp_server(driver, self._lock)
        self._last_runtime: dict[str, Any] = {
            "status": "idle",
            "commandId": None,
            "steps": [],
            "auditRecord": {"plan": []},
            "result": {},
        }

    def get_edge_scene(self) -> dict[str, Any]:
        obs = self.driver.observe_scene()
        snap = dict(obs.raw_snapshot or {})
        # Advance step_seq monotonically on read when idle so reset fresh-snapshot checks never stall
        if hasattr(self.driver, "_step_seq"):
            self.driver._step_seq = int(getattr(self.driver, "_step_seq", 0)) + 1  # type: ignore[attr-defined]
            snap["step_seq"] = getattr(self.driver, "_step_seq")
        elif hasattr(self.driver, "_state") and isinstance(getattr(self.driver, "_state"), dict):
            st = getattr(self.driver, "_state")
            st["step_seq"] = int(st.get("step_seq", 0)) + 1
            snap["step_seq"] = st["step_seq"]
        snap["evidence_source"] = self.driver.evidence_source
        snap["cameras_available"] = list(self.driver.list_cameras())
        return snap

    def get_cap_state(self) -> dict[str, Any]:
        caps = self.driver.discover_capabilities(allow_busy=True)
        d = caps.discovered
        return {
            "executionAuthority": d.execution_authority if d else self.driver.driver_name,
            "perceptionOwner": d.perception_owner if d else "unknown",
            "sdkVersion": d.sdk_version if d else "1.0",
            "sdkAllowedMethods": list(d.allowed_methods) if d else [],
            "sdkContractSha256": d.sdk_contract_sha256 if d else "",
            "deviceBundleId": d.device_bundle_id if d else self.driver.driver_name,
            "deviceBundleSha256": d.device_bundle_sha256 if d else "",
            "evidenceSource": self.driver.evidence_source,
            "cameras": list(self.driver.list_cameras()),
            "runtime": dict(self._last_runtime),
        }

    def post_cap_reset(self) -> dict[str, Any]:
        with self._lock:
            self._last_runtime = {
                "status": "idle",
                "commandId": None,
                "steps": [],
                "auditRecord": {"plan": []},
                "result": {},
            }
            return {"status": "reset", "driver": self.driver.driver_name}

    def post_cap_confirm(self, body: Mapping[str, Any]) -> dict[str, Any]:
        cup_id = str(body.get("cupId") or body.get("object_id") or "")
        return self.driver.confirm_object(cup_id)

    def post_testbed_setup(self, body: Mapping[str, Any]) -> dict[str, Any]:
        scenario: dict[str, Any] = {"preset": str(body.get("preset", "default"))}
        if body.get("cupAXy") is not None:
            scenario["cup_a_xy"] = list(body["cupAXy"])
        if body.get("cupBXy") is not None:
            scenario["cup_b_xy"] = list(body["cupBXy"])
        with self._lock:
            res = self.driver.reset_scenario(scenario)
            self._last_runtime["status"] = "idle"
            self._last_runtime["steps"] = []
            return res

    def get_testbed_oracle(self) -> dict[str, Any]:
        obs = self.driver.observe_scene()
        snap = obs.raw_snapshot or {}
        cup_pos = [0.0, 0.0, 0.0]
        cup_tilt = 0.0
        for obj in snap.get("objects") or []:
            if obj.get("object_id") == "obj-cup-a" or obj.get("id") == "obj-cup-a":
                cup_pos = list(obj.get("center_world_m") or obj.get("world") or [0.0, 0.0, 0.0])
                cup_tilt = float(obj.get("tilt_deg", 0.0))
                break
        cm = snap.get("coffee_machine") or {}
        return {
            "evidence_source": self.driver.evidence_source,
            "simulator_engine": snap.get("simulator_engine", self.driver.driver_name),
            "cup": cup_pos,
            "cup_tilt_deg": cup_tilt,
            "coffee_machine": {
                "status": cm.get("status", snap.get("brew_status", "idle")),
                "cup_filled": bool(cm.get("cup_filled", snap.get("cup_filled", False))),
            },
        }

    def post_cap_execute(self, body: Mapping[str, Any]) -> dict[str, Any]:
        command_id = str(body.get("commandId") or "sim-cmd")
        policy_code = str(body.get("policyCode") or "")
        scenario: dict[str, Any] = {
            "order_id": str(body.get("orderId") or "A042"),
            "target_slot": str(body.get("targetSlot") or "$SLOT_PICKUP_1"),
            "simulate_timeout": bool(body.get("simulateTimeout", False)),
            "fault_inject_slip_xy_m": body.get("injectPlaceSlipXyM"),
            "expect_status": "completed",
        }
        with self._lock:
            res = self.driver.run_policy_scenario(
                policy_code,
                command_id=command_id,
                scenario=scenario,
                journal=None,
            )
            plan = [
                {"skill_kind": name, "execution_count": int(cnt)}
                for name, cnt in (res.get("non_idempotent_counts") or {}).items()
            ]
            raw_steps = list(res.get("step_records") or [])
            if not raw_steps and res.get("methods"):
                raw_steps = [{"method": str(m)} for m in res.get("methods") or []]
            rt = {
                "status": str(res.get("host_runtime_status") or res.get("status") or "unknown"),
                "commandId": command_id,
                "steps": raw_steps,
                "auditRecord": {"plan": plan},
                "result": {
                    "sandbox_isolation": {"mode": res.get("sandbox_mode") or "host_sandbox"},
                    "detail": res.get("detail", ""),
                },
            }
            self._last_runtime = rt
            return {"ok": True, "runtime": rt}


def create_sim_host_server(
    app: StandaloneSimHostApp,
    *,
    host: str = "127.0.0.1",
    port: int = 8880,
) -> ThreadingHTTPServer:
    """Create a multi-threaded HTTP server exposing `StandaloneSimHostApp`."""

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args: Any) -> None:  # noqa: A002
            return

        def _send_json(self, payload: Mapping[str, Any], status: int = 200) -> None:
            raw = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def _send_bytes(self, data: bytes, content_type: str, status: int = 200) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _read_json(self) -> dict[str, Any]:
            length = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(length) if length > 0 else b"{}"
            return json.loads(raw.decode("utf-8")) if raw.strip() else {}

        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path.rstrip("/") or "/"
            if path == "/edge-scene":
                self._send_json(app.get_edge_scene())
                return
            if path == "/cap/state":
                self._send_json(app.get_cap_state())
                return
            if path == "/v1/sim/testbed/oracle":
                self._send_json(app.get_testbed_oracle())
                return
            if path.startswith("/cameras/"):
                cam = path[len("/cameras/") :]
                if cam.endswith(".jpg"):
                    cam = cam[:-4]
                img = app.driver.fetch_camera_image(cam)
                if img is None:
                    self._send_json({"error": f"Unknown camera: {cam}"}, HTTPStatus.NOT_FOUND)
                else:
                    self._send_bytes(img[0], img[1])
                return
            if path.startswith("/assets/robot/"):
                rel = path[len("/assets/robot/") :]
                resolver = getattr(app.driver, "resolve_robot_asset", None)
                asset = resolver(rel) if callable(resolver) else None
                if asset is None:
                    self._send_json({"error": f"Robot asset not found: {rel}"}, HTTPStatus.NOT_FOUND)
                else:
                    data, content_type, etag = asset
                    if self.headers.get("If-None-Match") == etag:
                        self.send_response(HTTPStatus.NOT_MODIFIED)
                        self.send_header("ETag", etag)
                        self.end_headers()
                        return
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", content_type)
                    self.send_header("ETag", etag)
                    self.send_header("Cache-Control", "public, max-age=3600")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                return
            if path == "/health":
                self._send_json({"status": "ok", "driver": app.driver.driver_name, "evidence_source": app.driver.evidence_source})
                return
            if path == "/sensors/rgbd":
                fn = getattr(app.driver, "capture_rgbd_bundle", None)
                if callable(fn):
                    self._send_json(fn())
                else:
                    self._send_json({"error": "Active driver does not expose calibrated RGB-D sensor stream"}, HTTPStatus.NOT_FOUND)
                return
            if path == "/robot/kinematics":
                fn = getattr(app.driver, "get_robot_kinematics", None)
                if callable(fn):
                    self._send_json(fn())
                else:
                    self._send_json({"error": "Active driver does not expose authoritative robot kinematics"}, HTTPStatus.NOT_FOUND)
                return
            if path == "/scene/reconstruction":
                fn = getattr(app.driver, "reconstruct_scene_evidencescene", None)
                if callable(fn):
                    self._send_json(fn())
                else:
                    self._send_json({"error": "Active driver does not expose RGB-D scene reconstruction"}, HTTPStatus.NOT_FOUND)
                return
            if path == "/scene/snapshot":
                fn = getattr(app.driver, "get_spatial_snapshot_dict", None)
                if callable(fn):
                    self._send_json(fn())
                else:
                    self._send_json({"error": "Active driver does not expose SpatialSnapshot v1"}, HTTPStatus.NOT_FOUND)
                return
            if path == "/mcp":
                self._send_json(
                    {
                        "endpoint": "/mcp",
                        "transport": "streamable-http-jsonrpc-2.0",
                        "serverInfo": app.mcp_server.server_info(),
                        "toolsCount": len(app.mcp_server.list_tools_dicts()),
                        "resourcesCount": len(app.mcp_server.list_resources_dicts()),
                    }
                )
                return
            self._send_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)

        def do_POST(self) -> None:  # noqa: N802
            path = urlparse(self.path).path.rstrip("/") or "/"
            try:
                body = self._read_json()
                if path == "/mcp":
                    self._send_json(app.mcp_server.handle_rpc(body))
                    return
                if path == "/cap/execute":
                    self._send_json(app.post_cap_execute(body))
                    return
                if path == "/cap/reset":
                    self._send_json(app.post_cap_reset())
                    return
                if path == "/cap/confirm":
                    self._send_json(app.post_cap_confirm(body))
                    return
                if path == "/scene/select_surface":
                    fn = getattr(app.driver, "low_level_select_surface_target", None)
                    if callable(fn):
                        self._send_json(
                            fn(
                                entity_id=str(body.get("entity_id") or "auto"),
                                point_index=int(body["point_index"]) if body.get("point_index") is not None else None,
                                target_xyz_m=body.get("target_xyz_m"),
                                reviewer_id=str(body.get("reviewer_id") or "operator"),
                                reason=str(body.get("reason") or "3d_twin_surface_selection"),
                            )
                        )
                    else:
                        self._send_json({"error": "Active driver does not support surface target selection"}, HTTPStatus.NOT_FOUND)
                    return
                if path == "/v1/sim/testbed/setup":
                    self._send_json(app.post_testbed_setup(body))
                    return
                self._send_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)
            except Exception as exc:  # noqa: BLE001
                self._send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)

    return ThreadingHTTPServer((host, port), Handler)
