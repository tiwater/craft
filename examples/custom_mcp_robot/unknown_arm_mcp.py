#!/usr/bin/env python3
"""Minimal standalone external MCP robot backend for an unknown custom robot model.

Demonstrates how ANY new/unknown robot (C++, Rust, ROS 2, or Python) plugs into TRAX
out-of-process over persistent stdio JSON-RPC 2.0 with ZERO changes to TRAX core.

Run directly via TRAX:
    python examples/custom_mcp_robot/unknown_arm_mcp.py
"""
from __future__ import annotations

import json
import sys
from typing import Any

from trax.mcp_protocol import MCPResourceSpec, MCPServer, MCPToolSpec


def build_unknown_gantry_server() -> MCPServer:
    """Create a stateful custom 2-axis gantry robot never mentioned in TRAX core."""
    state: dict[str, Any] = {
        "x_mm": 0.0,
        "y_mm": 0.0,
        "valve_fired": 0,
        "ledger": {},
    }

    srv = MCPServer(
        name="custom-xy-gantry-v1",
        version="1.0.0",
        embodiment_meta={
            "embodiment_id": "custom-xy-gantry-v1",
            "execution_authority": "custom-xy-gantry-v1",
            "perception_owner": "gantry-encoder",
            "evidence_source": "fixture",
            "urdf_uri": "trax://robot/urdf",
        },
    )

    def _read_pose(_args: dict[str, Any]) -> dict[str, Any]:
        return {"x_mm": state["x_mm"], "y_mm": state["y_mm"], "valve_fired": state["valve_fired"]}

    def _move_gantry(args: dict[str, Any]) -> dict[str, Any]:
        x = float(args.get("x_mm", 0.0))
        y = float(args.get("y_mm", 0.0))
        if not (-200.0 <= x <= 200.0 and -200.0 <= y <= 200.0):
            return {
                "ok": False,
                "status": "limit_refusal",
                "violated_limit": f"gantry target ({x}, {y}) outside [-200, 200]mm",
            }
        state["x_mm"] = x
        state["y_mm"] = y
        return {"ok": True, "status": "done", "x_mm": x, "y_mm": y}

    def _fire_dispense_valve(args: dict[str, Any]) -> dict[str, Any]:
        cid = str(args.get("command_id") or "valve-1")
        if cid in state["ledger"]:
            return dict(state["ledger"][cid])
        state["valve_fired"] += 1
        rec = {"command_id": cid, "receipt": "done", "valve_fired": state["valve_fired"]}
        state["ledger"][cid] = rec
        return dict(rec)

    def _query_valve(args: dict[str, Any]) -> dict[str, Any]:
        cid = str(args.get("command_id") or "")
        return state["ledger"].get(cid, {"command_id": cid, "receipt": "not_found"})

    def _verify(_args: dict[str, Any]) -> dict[str, Any]:
        ok = abs(state["x_mm"] - 120.0) <= 5.0 and abs(state["y_mm"] - 80.0) <= 5.0 and state["valve_fired"] == 1
        return {
            "passed": ok,
            "detail": f"pos=({state['x_mm']},{state['y_mm']})mm, valve_fired={state['valve_fired']}",
            "metrics": {"x_mm": state["x_mm"], "y_mm": state["y_mm"], "valve_fired": state["valve_fired"]},
        }

    srv.register_tool(
        MCPToolSpec(
            name="read_gantry_pose",
            description="Read current custom XY gantry encoder coordinates (mm) and dispense count.",
            input_schema={"type": "object", "properties": {}, "additionalProperties": False},
            read_only=True,
            idempotent=True,
        ),
        _read_pose,
    )
    srv.register_tool(
        MCPToolSpec(
            name="move_gantry_xy",
            description="Move XY gantry to target [x_mm, y_mm] within [-200, 200] mm.",
            input_schema={
                "type": "object",
                "required": ["x_mm", "y_mm"],
                "properties": {"x_mm": {"type": "number"}, "y_mm": {"type": "number"}},
            },
            read_only=False,
            idempotent=True,
        ),
        _move_gantry,
    )
    srv.register_tool(
        MCPToolSpec(
            name="fire_dispense_valve",
            description="NON-IDEMPOTENT: Fire fluid dispense valve once with a stable command_id.",
            input_schema={"type": "object", "required": ["command_id"], "properties": {"command_id": {"type": "string"}}},
            read_only=False,
            idempotent=False,
            destructive=True,
            requires_reconciliation=True,
        ),
        _fire_dispense_valve,
    )
    srv.register_tool(
        MCPToolSpec(
            name="query_command_status",
            description="Query gantry valve ledger for command_id reconciliation.",
            input_schema={"type": "object", "required": ["command_id"], "properties": {"command_id": {"type": "string"}}},
            read_only=True,
            idempotent=True,
        ),
        _query_valve,
    )
    srv.register_tool(
        MCPToolSpec(
            name="verify_task",
            description="Verify gantry reached (120, 80) mm and fired dispense valve exactly once.",
            input_schema={"type": "object", "properties": {}},
            read_only=True,
            idempotent=True,
        ),
        _verify,
    )
    srv.register_resource(
        MCPResourceSpec(
            uri="trax://robot/urdf",
            name="Custom XY Gantry URDF",
            description="Minimal URDF for the unknown custom XY gantry.",
            mime_type="application/xml",
        ),
        lambda: (
            "application/xml",
            '<?xml version="1.0"?><robot name="custom_xy_gantry"><link name="base_link"/></robot>',
        ),
    )
    srv.register_resource(
        MCPResourceSpec(
            uri="trax://scene/state",
            name="Gantry State",
            description="Current XY gantry state JSON.",
            mime_type="application/json",
        ),
        lambda: ("application/json", json.dumps(_read_pose({}))),
    )
    return srv


if __name__ == "__main__":
    build_unknown_gantry_server().serve_stdio()
