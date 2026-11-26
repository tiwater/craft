"""Pure-stdlib Model Context Protocol (MCP) JSON-RPC 2.0 server, client, and driver bridge.

Zero external dependencies: implements MCP streamable HTTP (`/mcp`) and stdio JSON-RPC 2.0
(`initialize`, `tools/list`, `tools/call`, `resources/list`, `resources/read`) so heterogeneous
robot embodiments (single-arm, bimanual, dexterous hand, mobile, humanoid; sim or physical) can
expose their own native tool schemas and safety annotations (`readOnlyHint`, `idempotentHint`,
`destructiveHint`) for dynamic discovery and TRAX transactional governance.
"""
from __future__ import annotations

import base64
from dataclasses import dataclass, field
import json
from pathlib import Path
import subprocess
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
    """Standard MCP tool descriptor with JSON Schema and safety/governance annotations."""

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
                "traxRequiresReconciliation": self.requires_reconciliation,
            },
        }


@dataclass(frozen=True)
class MCPResourceSpec:
    """Standard MCP resource descriptor (scene state, contracts, live cameras)."""

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


@dataclass
class MCPServer:
    """Pure-stdlib JSON-RPC 2.0 MCP server dispatcher."""

    name: str
    version: str = "0.2.0"
    embodiment_meta: dict[str, Any] = field(default_factory=dict)
    _tools: dict[str, tuple[MCPToolSpec, Callable[[dict[str, Any]], Any]]] = field(default_factory=dict)
    _resources: dict[str, tuple[MCPResourceSpec, Callable[[], tuple[str, bytes | str]]]] = field(default_factory=dict)

    def register_tool(
        self,
        spec: MCPToolSpec,
        handler: Callable[[dict[str, Any]], Any],
    ) -> None:
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
        params = dict(req.get("params") or {})

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

            if method in ("notifications/initialized", "initialized"):
                return {"jsonrpc": "2.0", "id": req_id, "result": {"acknowledged": True}}

            if method == "ping":
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


def _driver_exec_single_sdk_call(
    driver: SceneHostDriver,
    lock: threading.RLock | threading.Lock,
    sdk_expr: str,
    command_id: str,
    scenario: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Execute a single public-SDK call inside the host driver's AST sandbox."""
    src = f"def run_policy(sdk, params):\n    res = {sdk_expr}\n    return {{'status': 'completed', 'result': res}}\n"
    sc = dict(scenario or {"order_id": "A042", "target_slot": "$SLOT_PICKUP_1", "expect_status": "completed"})
    with lock:
        return driver.run_policy_scenario(src, command_id=command_id, scenario=sc, journal=None)


def build_driver_mcp_server(
    driver: SceneHostDriver,
    lock: threading.RLock | threading.Lock | None = None,
) -> MCPServer:
    """Expose any TRAX `SceneHostDriver` as a native MCP server with dynamic tool schemas & resources."""
    guard = lock or threading.RLock()
    cap = None
    try:
        cap = driver.discover_capabilities(allow_busy=True)
    except Exception:  # noqa: BLE001
        cap = None
    disc = cap.discovered if cap else None
    authority = disc.execution_authority if disc else driver.driver_name
    allowed = set(disc.allowed_methods) if disc else set()

    srv = MCPServer(
        name=f"trax-mcp-{authority}",
        version="0.2.0",
        embodiment_meta={
            "execution_authority": authority,
            "perception_owner": disc.perception_owner if disc else "host",
            "evidence_source": driver.evidence_source,
            "driver": driver.driver_name,
            "sdk_contract_sha256": (disc.sdk_contract_sha256 if disc else "")[:16],
            "native_low_level": hasattr(driver, "low_level_get_robot_state"),
        },
    )

    # Generic Low-Level Robotics Embodiment Backend (e.g. LeRobot SO-101 6-DoF)
    if hasattr(driver, "low_level_get_robot_state"):
        srv.register_tool(
            MCPToolSpec(
                name="get_robot_state",
                description="Read current 5-DoF joint angles (rad), joint limits, 3D end-effector pose (m), gripper open ratio, grasped object, and workspace bounds.",
                input_schema={"type": "object", "properties": {}, "additionalProperties": False},
                read_only=True,
                idempotent=True,
            ),
            lambda args: driver.low_level_get_robot_state(),  # type: ignore[attr-defined]
        )
        srv.register_tool(
            MCPToolSpec(
                name="get_scene_objects",
                description="Read authoritative tabletop objects (`id`, `world_m`, `height_above_table_m`, `grasped`) and named target zones (`target_zone_a`, `target_zone_b`, `station_pad`).",
                input_schema={"type": "object", "properties": {}, "additionalProperties": False},
                read_only=True,
                idempotent=True,
            ),
            lambda args: driver.low_level_get_scene_objects(),  # type: ignore[attr-defined]
        )
        srv.register_tool(
            MCPToolSpec(
                name="move_ee",
                description="Move robot end-effector smoothly to Cartesian world pose [x, y, z] in meters via IK. Enforces workspace bounds and returns structured `limit_refusal` BEFORE moving if out of bounds.",
                input_schema={
                    "type": "object",
                    "required": ["x", "y", "z"],
                    "properties": {
                        "x": {"type": "number", "default": 0.33, "description": "World X in meters"},
                        "y": {"type": "number", "default": -0.02, "description": "World Y in meters"},
                        "z": {"type": "number", "default": 1.15, "description": "World Z in meters (tabletop is ~1.00 m)"},
                        "steps": {"type": "integer", "default": 36, "description": "Simulation integration steps (12..80)"},
                    },
                },
                read_only=False,
                idempotent=True,
            ),
            lambda args: driver.low_level_move_ee(  # type: ignore[attr-defined]
                float(args.get("x", 0.33)),
                float(args.get("y", -0.02)),
                float(args.get("z", 1.15)),
                steps=int(args.get("steps", 36)),
            ),
        )
        srv.register_tool(
            MCPToolSpec(
                name="set_gripper",
                description="Control SO-101 moving jaw gripper: open_ratio=1.0 (fully open / release) or 0.0 (closed / grasp nearby object within contact radius).",
                input_schema={
                    "type": "object",
                    "required": ["open_ratio"],
                    "properties": {
                        "open_ratio": {"type": "number", "minimum": 0.0, "maximum": 1.0, "default": 0.0},
                    },
                },
                read_only=False,
                idempotent=True,
            ),
            lambda args: driver.low_level_set_gripper(float(args.get("open_ratio", 0.0))),  # type: ignore[attr-defined]
        )
        srv.register_tool(
            MCPToolSpec(
                name="move_joints",
                description="Command 5-DoF arm joint angles [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll] in radians. Refuses out-of-range joints with `limit_refusal`.",
                input_schema={
                    "type": "object",
                    "required": ["q_rad"],
                    "properties": {
                        "q_rad": {"type": "array", "items": {"type": "number"}, "minItems": 5, "maxItems": 5, "default": [0.0, -0.35, 0.55, 0.95, 0.0]},
                        "steps": {"type": "integer", "default": 32},
                    },
                },
                read_only=False,
                idempotent=True,
            ),
            lambda args: driver.low_level_move_joints(list(args.get("q_rad") or [0.0, -0.35, 0.55, 0.95, 0.0]), steps=int(args.get("steps", 32))),  # type: ignore[attr-defined]
        )
        srv.register_tool(
            MCPToolSpec(
                name="go_home",
                description="Retract the SO-101 arm safely to its neutral home joint configuration.",
                input_schema={"type": "object", "properties": {}, "additionalProperties": False},
                read_only=False,
                idempotent=True,
            ),
            lambda args: driver.low_level_go_home(),  # type: ignore[attr-defined]
        )
        srv.register_tool(
            MCPToolSpec(
                name="reset_task",
                description="Reset the tabletop environment for a specified task_id (`pick_and_place_zone`, `lift_red_object`, `place_and_activate_station`) and seed.",
                input_schema={
                    "type": "object",
                    "properties": {
                        "task_id": {
                            "type": "string",
                            "default": "pick_and_place_zone",
                            "enum": ["pick_and_place_zone", "lift_red_object", "place_and_activate_station"],
                        },
                        "seed": {"type": "integer", "default": 0},
                    },
                },
                read_only=False,
                idempotent=True,
            ),
            lambda args: driver.low_level_reset_task(str(args.get("task_id") or "pick_and_place_zone"), seed=int(args.get("seed", 0))),  # type: ignore[attr-defined]
        )
        srv.register_tool(
            MCPToolSpec(
                name="activate_workstation",
                description="NON-IDEMPOTENT: Trigger external workstation actuator once. Must pass a stable `command_id` and reconcile via `query_command_status` if receipt is `unknown`.",
                input_schema={
                    "type": "object",
                    "required": ["command_id"],
                    "properties": {
                        "command_id": {"type": "string", "default": "station-cmd-1"},
                    },
                },
                read_only=False,
                idempotent=False,
                destructive=True,
                requires_reconciliation=True,
            ),
            lambda args: driver.low_level_activate_workstation(str(args.get("command_id") or "station-cmd-1")),  # type: ignore[attr-defined]
        )
        srv.register_tool(
            MCPToolSpec(
                name="query_command_status",
                description="Query host ledger for a prior non-idempotent `command_id` to reconcile unknown receipts before any retry.",
                input_schema={
                    "type": "object",
                    "required": ["command_id"],
                    "properties": {"command_id": {"type": "string", "default": "station-cmd-1"}},
                },
                read_only=True,
                idempotent=True,
            ),
            lambda args: driver.low_level_query_command_status(str(args.get("command_id") or "station-cmd-1")),  # type: ignore[attr-defined]
        )
        srv.register_tool(
            MCPToolSpec(
                name="verify_task",
                description="Run independent physics-based verification for the active tabletop task.",
                input_schema={
                    "type": "object",
                    "properties": {"task_id": {"type": "string", "default": "pick_and_place_zone"}},
                },
                read_only=True,
                idempotent=True,
            ),
            lambda args: driver.low_level_verify_task(args.get("task_id")),  # type: ignore[attr-defined]
        )
        srv.register_resource(
            MCPResourceSpec(
                uri="trax://scene/state",
                name="Live Robot & Tabletop Scene State",
                description="Authoritative SO-101 joint/EE state and tabletop object poses.",
                mime_type="application/json",
            ),
            lambda: (
                "application/json",
                json.dumps(
                    {
                        "robot": driver.low_level_get_robot_state(),  # type: ignore[attr-defined]
                        "scene": driver.low_level_get_scene_objects(),  # type: ignore[attr-defined]
                        "tasks": driver.low_level_task_catalog(),  # type: ignore[attr-defined]
                    },
                    indent=2,
                ),
            ),
        )
        for cam in driver.list_cameras():
            cam_name = str(cam)

            def _make_ll_cam_reader(c: str) -> Callable[[], tuple[str, bytes | str]]:
                def _read() -> tuple[str, bytes | str]:
                    img = driver.fetch_camera_image(c)
                    return ("text/plain", f"Camera {c} unavailable") if img is None else (img[1], img[0])

                return _read

            srv.register_resource(
                MCPResourceSpec(
                    uri=f"trax://cameras/{cam_name}",
                    name=f"Camera: {cam_name}",
                    description=f"Live rendered camera frame from {authority} ({cam_name}).",
                    mime_type="image/jpeg",
                ),
                _make_ll_cam_reader(cam_name),
            )
        return srv

    # 1. Core perception & read-only tools (always exposed)
    srv.register_tool(
        MCPToolSpec(
            name="observe_scene",
            description="Read authoritative host-owned scene observation (objects, 3D world poses, L2 pouring safety, brewer state).",
            input_schema={"type": "object", "properties": {}, "additionalProperties": False},
            read_only=True,
            idempotent=True,
        ),
        lambda args: driver.observe_scene().raw_snapshot or driver.observe_scene().to_dict(),
    )

    srv.register_tool(
        MCPToolSpec(
            name="discover_capabilities",
            description="Query host execution authority, perception owner, allowed SDK methods, and contract digest.",
            input_schema={"type": "object", "properties": {}, "additionalProperties": False},
            read_only=True,
            idempotent=True,
        ),
        lambda args: driver.discover_capabilities(allow_busy=True).to_dict(),
    )

    srv.register_tool(
        MCPToolSpec(
            name="reset_scenario",
            description="Reset simulation environment to a named scenario preset (default, sim_pose_offset, sim_dual_cup).",
            input_schema={
                "type": "object",
                "properties": {
                    "preset": {"type": "string", "default": "default", "enum": ["default", "sim_pose_offset", "sim_dual_cup"]},
                },
            },
            read_only=False,
            idempotent=True,
        ),
        lambda args: _locked_call(guard, lambda: driver.reset_scenario({"preset": str(args.get("preset") or "default")})),
    )

    # 2. Dynamically register actuation & domain tools according to discovered host capabilities
    if not allowed or "move_to_target" in allowed:
        srv.register_tool(
            MCPToolSpec(
                name="move_to_target",
                description="Move robot end-effector to a 3D world coordinate [x, y, z] in meters via host IK controller.",
                input_schema={
                    "type": "object",
                    "required": ["xyz_m"],
                    "properties": {
                        "xyz_m": {
                            "type": "array",
                            "items": {"type": "number"},
                            "minItems": 3,
                            "maxItems": 3,
                            "default": [0.22, -0.14, 0.16],
                            "description": "Target 3D world position [x, y, z] in meters.",
                        },
                        "command_id": {"type": "string", "default": "mcp-move-1"},
                    },
                },
                read_only=False,
                idempotent=True,
            ),
            lambda args: _driver_exec_single_sdk_call(
                driver,
                guard,
                f"sdk.move_to_target({json.dumps([float(v) for v in (args.get('xyz_m') or [0.22, -0.14, 0.16])[:3]])}, command_id={json.dumps(str(args.get('command_id') or 'mcp-move-1'))})",
                str(args.get("command_id") or f"mcp-move-{time.time_ns()}"),
            ),
        )

    if not allowed or "retract_safe_pose" in allowed:
        srv.register_tool(
            MCPToolSpec(
                name="retract_safe_pose",
                description="Retract manipulator arm to its host-defined safe home pose.",
                input_schema={
                    "type": "object",
                    "properties": {"command_id": {"type": "string", "default": "mcp-retract-1"}},
                },
                read_only=False,
                idempotent=True,
            ),
            lambda args: _driver_exec_single_sdk_call(
                driver,
                guard,
                f"sdk.retract_safe_pose(command_id={json.dumps(str(args.get('command_id') or 'mcp-retract-1'))})",
                str(args.get("command_id") or f"mcp-retract-{time.time_ns()}"),
            ),
        )

    if not allowed or "place_cup_under_machine" in allowed:
        srv.register_tool(
            MCPToolSpec(
                name="place_cup_under_machine",
                description="Pick target cup by object_id and place it onto the espresso machine drip tray.",
                input_schema={
                    "type": "object",
                    "required": ["cup_id"],
                    "properties": {
                        "cup_id": {"type": "string", "default": "obj-cup-a"},
                        "command_id": {"type": "string", "default": "mcp-place-1"},
                    },
                },
                read_only=False,
                idempotent=False,
            ),
            lambda args: _driver_exec_single_sdk_call(
                driver,
                guard,
                f"sdk.place_cup_under_machine({json.dumps(str(args.get('cup_id') or 'obj-cup-a'))}, command_id={json.dumps(str(args.get('command_id') or 'mcp-place-1'))})",
                str(args.get("command_id") or f"mcp-place-{time.time_ns()}"),
            ),
        )

    if not allowed or "recenter_cup_under_spout" in allowed:
        srv.register_tool(
            MCPToolSpec(
                name="recenter_cup_under_spout",
                description="Re-center cup under espresso spout to satisfy L2 pouring safety offset tolerance.",
                input_schema={
                    "type": "object",
                    "required": ["cup_id"],
                    "properties": {
                        "cup_id": {"type": "string", "default": "obj-cup-a"},
                        "command_id": {"type": "string", "default": "mcp-recenter-1"},
                    },
                },
                read_only=False,
                idempotent=True,
            ),
            lambda args: _driver_exec_single_sdk_call(
                driver,
                guard,
                f"sdk.recenter_cup_under_spout({json.dumps(str(args.get('cup_id') or 'obj-cup-a'))}, command_id={json.dumps(str(args.get('command_id') or 'mcp-recenter-1'))})",
                str(args.get("command_id") or f"mcp-recenter-{time.time_ns()}"),
            ),
        )

    if not allowed or "start_coffee_machine" in allowed:
        srv.register_tool(
            MCPToolSpec(
                name="start_coffee_machine",
                description="NON-IDEMPOTENT: Trigger espresso brewer cycle. Requires TRAX intent-before-dispatch journal & query-first reconciliation on unknown receipt.",
                input_schema={
                    "type": "object",
                    "properties": {
                        "command_id": {"type": "string", "default": "mcp-brew-1"},
                    },
                },
                read_only=False,
                idempotent=False,
                destructive=True,
                requires_reconciliation=True,
            ),
            lambda args: _driver_exec_single_sdk_call(
                driver,
                guard,
                f"sdk.start_coffee_machine(command_id={json.dumps(str(args.get('command_id') or 'mcp-brew-1'))})",
                str(args.get("command_id") or f"mcp-brew-{time.time_ns()}"),
            ),
        )

    if not allowed or "query_command_status" in allowed:
        srv.register_tool(
            MCPToolSpec(
                name="query_command_status",
                description="Reconcile an unknown/timed-out non-idempotent command by command_id before any retry.",
                input_schema={
                    "type": "object",
                    "required": ["command_id"],
                    "properties": {"command_id": {"type": "string", "default": "mcp-brew-1"}},
                },
                read_only=True,
                idempotent=True,
            ),
            lambda args: driver.reconcile_command(str(args.get("command_id") or "mcp-brew-1")).to_dict(),
        )

    srv.register_tool(
        MCPToolSpec(
            name="execute_policy_sandbox",
            description="Execute a full Python `run_policy(sdk, params)` program inside the host AST sandbox.",
            input_schema={
                "type": "object",
                "required": ["source"],
                "properties": {
                    "source": {"type": "string", "default": "def run_policy(sdk, params):\n    return {'scene': sdk.observe_scene()}\n"},
                    "command_id": {"type": "string", "default": "mcp-policy-1"},
                    "simulate_timeout": {"type": "boolean", "default": False},
                },
            },
            read_only=False,
            idempotent=False,
            requires_reconciliation=True,
        ),
        lambda args: _locked_call(
            guard,
            lambda: driver.run_policy_scenario(
                str(args.get("source") or ""),
                command_id=str(args.get("command_id") or f"mcp-pol-{time.time_ns()}"),
                scenario={
                    "order_id": "A042",
                    "target_slot": "$SLOT_PICKUP_1",
                    "simulate_timeout": bool(args.get("simulate_timeout", False)),
                    "expect_status": "completed",
                },
                journal=None,
            ),
        ),
    )

    # 3. MCP Resources: live scene state, capability contract, and every camera feed
    srv.register_resource(
        MCPResourceSpec(
            uri="trax://scene/state",
            name="Live Authoritative Scene Snapshot",
            description="Current host-owned scene observation JSON (objects, poses, L2 safety, brewer).",
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
    for cam in driver.list_cameras():
        cam_name = str(cam)

        def _make_cam_reader(c: str) -> Callable[[], tuple[str, bytes | str]]:
            def _read() -> tuple[str, bytes | str]:
                img = driver.fetch_camera_image(c)
                if img is None:
                    return ("text/plain", f"Camera {c} unavailable")
                return (img[1], img[0])

            return _read

        srv.register_resource(
            MCPResourceSpec(
                uri=f"trax://cameras/{cam_name}",
                name=f"Camera: {cam_name}",
                description=f"Live rendered camera frame from {authority} ({cam_name}).",
                mime_type="image/jpeg",
            ),
            _make_cam_reader(cam_name),
        )

    return srv


def _locked_call(lock: threading.RLock | threading.Lock, fn: Callable[[], Any]) -> Any:
    with lock:
        return fn()


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


class MCPClient:
    """Pure-stdlib MCP JSON-RPC 2.0 client supporting HTTP (`/mcp`), stdio, and in-process servers."""

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
            # Handle both plain JSON and SSE-framed Streamable HTTP responses
            if body.startswith("data:") or "\ndata:" in body:
                data_lines = [ln[5:].strip() for ln in body.splitlines() if ln.startswith("data:")]
                body = data_lines[-1] if data_lines else "{}"
            resp_obj = json.loads(body) if body else {}
        elif self.transport == "stdio":
            if not self.command:
                raise ValueError("Stdio MCP transport requires a command list")
            proc = subprocess.run(
                self.command,
                input=json.dumps(req_obj) + "\n",
                capture_output=True,
                text=True,
                timeout=self.timeout_s,
                check=False,
            )
            lines = [ln.strip() for ln in proc.stdout.splitlines() if ln.strip()]
            if not lines:
                raise RuntimeError(f"Stdio MCP server produced no JSON-RPC output (rc={proc.returncode}): {proc.stderr[:300]}")
            resp_obj = json.loads(lines[-1])
        else:
            raise ValueError(f"Unsupported MCP transport: {self.transport}")

        ms = round((time.monotonic() - t0) * 1000.0, 2)
        return req_obj, resp_obj, ms


class MCPInspectorController:
    """Stateful MCP discovery & live tool-invocation workbench for TRAX UI."""

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
        self._server_info: dict[str, Any] | None = None
        self._tools: list[dict[str, Any]] = []
        self._resources: list[dict[str, Any]] = []
        self._wire_log: list[dict[str, Any]] = []
        self._last_error: str | None = None

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
            return MCPClient(transport="stdio", command=cmd_list, timeout_s=15.0), " ".join(cmd_list), "stdio", "", cmd_list
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
        # Avoid bloating wire log with huge base64 camera blobs
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
                    "clientInfo": {"name": "trax-ui-inspector", "version": "0.2.0"},
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
        """Append a live JSON-RPC exchange emitted by TraxAgentEngine into the inspector wire log."""
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
