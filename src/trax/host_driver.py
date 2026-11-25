"""Host-independent driver protocol and pluggable drivers (fixture & generic HTTP edge).

TRAX's benchmark engine, interactive Studio, development loop, and acceptance gates
depend only on ``SceneHostDriver`` — never on any specific host repository or
hardcoded robot service. External robotics stacks plug in via ``HttpEdgeHostDriver``
(configured purely through ``SceneConfig``), while ``FixtureHostDriver`` lets anyone
run TRAX's benchmark and interactive Studio standalone with zero external services.
"""
from __future__ import annotations

import ast
from dataclasses import dataclass
from importlib import metadata as importlib_metadata
import math
import time
from typing import Any, Callable, Mapping, Protocol, Sequence
import urllib.error
import urllib.request

from .config import SceneConfig
from .contracts import digest
from .edge_adapter import (
    CapabilityCheckResult,
    DiscoveredCapabilities,
    EdgeExecutionResult,
    EdgeHostAdapter,
    ReconciliationResult,
    SceneObservationResult,
)
from .journal import Journal


class SceneHostDriver(Protocol):
    """Generic host contract consumed by TRAX orchestration, benchmark, and Studio."""

    driver_name: str
    evidence_source: str

    def discover_capabilities(self, *, allow_busy: bool = False) -> CapabilityCheckResult: ...
    def check_missing_capabilities(self, required_methods: Sequence[str]) -> dict[str, Any]: ...
    def observe_scene(self) -> SceneObservationResult: ...
    def get_live_runtime(self) -> dict[str, Any]: ...
    def list_cameras(self) -> tuple[str, ...]: ...
    def fetch_camera_image(self, camera_name: str) -> tuple[bytes, str] | None: ...
    def confirm_object(self, object_id: str) -> dict[str, Any]: ...
    def reset_scenario(self, scenario: Mapping[str, Any]) -> dict[str, Any]: ...
    def run_policy_scenario(
        self,
        policy_source: str,
        *,
        command_id: str,
        scenario: Mapping[str, Any],
        journal: Journal | None = None,
    ) -> dict[str, Any]: ...
    def reconcile_command(self, command_id: str, *, journal: Journal | None = None) -> ReconciliationResult: ...


class _FixturePolicySDK:
    """In-memory deterministic coffee SDK used strictly by FixtureHostDriver.

    Explicitly tagged ``evidence_source='fixture'``; models protocol state transitions
    only (never physical dynamics, grasping, or food quality).
    """

    ALLOWED_METHODS = (
        "get_order",
        "observe_scene",
        "request_operator_confirmation",
        "retract_safe_pose",
        "escalate_to_human_operator",
        "complete_order_delivery",
        "query_command_status",
        "move_to_target",
        "place_cup_under_machine",
        "recenter_cup_under_spout",
        "start_coffee_machine",
        "wait_for_coffee_ready",
        "retrieve_and_deliver_cup",
    )

    def __init__(self, driver: FixtureHostDriver, scenario: Mapping[str, Any]) -> None:
        self._driver = driver
        self._scenario = scenario
        self.steps: list[dict[str, Any]] = []
        self.brew_count = 0
        self.unknown_events = 0
        self.queried_after_unknown = True
        self._pending_unknown = False
        self._command_receipts: dict[str, dict[str, Any]] = {}
        self.human_interventions = 0
        self._slip = scenario.get("fault_inject_slip_xy_m")
        self._sim_timeout = bool(scenario.get("simulate_timeout", False))
        self._safe_to_pour = True

    def _rec(self, method: str) -> None:
        self.steps.append({"method": method})

    def get_order(self, order_id: str = "A042") -> dict[str, Any]:
        self._rec("get_order")
        special = self._scenario.get("special_request")
        if special is None and order_id == "A044":
            special = "steam_wand_manual_purge_and_latte_art"
        return {
            "order_id": order_id,
            "drink": "steam_wand_purge_and_latte_art" if order_id == "A044" else "hot_americano",
            "cup_preference": "takeout_green" if order_id == "A043" else "dine_in_red",
            "destination_slot": str(self._scenario.get("target_slot", "$SLOT_PICKUP_2" if order_id == "A043" else "$SLOT_PICKUP_1")),
            "special_request": special,
            "status": "pending",
        }

    def observe_scene(self) -> dict[str, Any]:
        self._rec("observe_scene")
        objs = list(self._driver._state["objects"])
        return {
            "control_ready": True,
            "pipeline_health": {"source": "fixture_bench", "ok": True},
            "objects": [
                {
                    "id": o["object_id"],
                    "cup_type": o["semantic_class"],
                    "label": o["label"],
                    "confidence": o["confidence"],
                    "world": list(o["center_world_m"]),
                    "tilt_deg": 0.0,
                }
                for o in objs
            ],
            "coffee_machine": {
                "status": self._driver._state["machine_status"],
                "cup_filled": self._driver._state["cup_filled"],
            },
            "l2_pouring_safety": {
                "safe_to_pour": self._safe_to_pour,
                "current_radial_offset_m": 0.0 if self._safe_to_pour else 0.04,
                "blocker_reasons": [] if self._safe_to_pour else ["radial_offset_exceeded"],
            },
            "confidence_threshold": 0.75,
            "operator_confirmed_cup_id": self._driver._state["confirmed_cup_id"],
        }

    def request_operator_confirmation(self, reason: str, candidates: list[Any]) -> dict[str, Any]:
        self._rec("request_operator_confirmation")
        self.human_interventions += 1
        return {"receipt": "paused_for_operator", "reason": reason, "candidates": candidates}

    def retract_safe_pose(self, *, command_id: str) -> dict[str, Any]:
        self._rec("retract_safe_pose")
        r = {"receipt": "done", "command_id": command_id}
        self._command_receipts[command_id] = r
        return r

    def escalate_to_human_operator(self, order_id: str, reason: str, *, command_id: str) -> dict[str, Any]:
        self._rec("escalate_to_human_operator")
        self.human_interventions += 1
        r = {"receipt": "done", "command_id": command_id, "order_id": order_id, "reason": reason}
        self._command_receipts[command_id] = r
        return r

    def move_to_target(self, target_world: Any, *, command_id: str) -> dict[str, Any]:
        self._rec("move_to_target")
        r = {"receipt": "done", "command_id": command_id}
        self._command_receipts[command_id] = r
        return r

    def place_cup_under_machine(self, cup_id: str, *, command_id: str) -> dict[str, Any]:
        self._rec("place_cup_under_machine")
        dx, dy = (float(self._slip[0]), float(self._slip[1])) if self._slip else (0.0, 0.0)
        self._driver._move_cup(cup_id, [0.45 + dx, 0.0 + dy, 1.075])
        self._safe_to_pour = math.hypot(dx, dy) <= 0.015
        r = {"receipt": "done", "command_id": command_id, "cup_id": cup_id}
        self._command_receipts[command_id] = r
        return r

    def recenter_cup_under_spout(self, cup_id: str, *, command_id: str) -> dict[str, Any]:
        self._rec("recenter_cup_under_spout")
        self._driver._move_cup(cup_id, [0.45, 0.0, 1.075])
        self._safe_to_pour = True
        r = {"receipt": "done", "command_id": command_id, "cup_id": cup_id}
        self._command_receipts[command_id] = r
        return r

    def start_coffee_machine(self, *, command_id: str) -> dict[str, Any]:
        self._rec("start_coffee_machine")
        if not self._safe_to_pour:
            r = {"receipt": "rejected", "command_id": command_id, "reason": "l2_pouring_unsafe"}
            self._command_receipts[command_id] = r
            return r
        self.brew_count += 1
        self._driver._state["machine_status"] = "running"
        final_r = {"receipt": "running", "command_id": command_id}
        self._command_receipts[command_id] = final_r
        if self._sim_timeout and self.brew_count == 1:
            self._rec("command_ack_timeout_unknown")
            self.unknown_events += 1
            self._pending_unknown = True
            self.queried_after_unknown = False
            return {"receipt": "unknown", "command_id": command_id}
        return final_r

    def query_command_status(self, command_id: str) -> dict[str, Any]:
        self._rec("query_command_status")
        if self._pending_unknown:
            self._pending_unknown = False
            self.queried_after_unknown = True
        return self._command_receipts.get(command_id, {"receipt": "unknown", "command_id": command_id})

    def wait_for_coffee_ready(self, *, command_id: str) -> dict[str, Any]:
        self._rec("wait_for_coffee_ready")
        if self.brew_count >= 1:
            self._driver._state["machine_status"] = "completed"
            self._driver._state["cup_filled"] = True
            r = {"receipt": "done", "command_id": command_id}
        else:
            r = {"receipt": "rejected", "command_id": command_id, "reason": "brew_not_started"}
        self._command_receipts[command_id] = r
        return r

    def retrieve_and_deliver_cup(self, cup_id: str, target_slot: str, *, command_id: str) -> dict[str, Any]:
        self._rec("retrieve_and_deliver_cup")
        target_xy = [0.38, -0.17, 1.075] if target_slot == "$SLOT_PICKUP_1" else [0.38, 0.17, 1.075]
        self._driver._move_cup(cup_id, target_xy)
        r = {"receipt": "done", "command_id": command_id}
        self._command_receipts[command_id] = r
        return r

    def complete_order_delivery(self, order_id: str, cup_id: str, destination_slot: str, *, command_id: str) -> dict[str, Any]:
        self._rec("complete_order_delivery")
        r = {"receipt": "done", "command_id": command_id, "order_id": order_id}
        self._command_receipts[command_id] = r
        return r


class FixtureHostDriver:
    """Zero-dependency standalone fixture host driver with live SVG bench visualization."""

    driver_name = "fixture"
    evidence_source = "fixture"

    def __init__(self, config: SceneConfig) -> None:
        self.config = config
        self._contract_sha = digest(list(_FixturePolicySDK.ALLOWED_METHODS))
        self._last_command: dict[str, Any] = {"command_id": None, "status": "idle", "brew_count": 0}
        self._state: dict[str, Any] = {}
        self.reset_scenario({"preset": "default"})

    def _move_cup(self, cup_id: str, xyz: list[float]) -> None:
        for obj in self._state["objects"]:
            if obj["object_id"] == cup_id:
                obj["center_world_m"] = list(xyz)
                return

    def discover_capabilities(self, *, allow_busy: bool = False) -> CapabilityCheckResult:
        allowed = _FixturePolicySDK.ALLOWED_METHODS
        discovered = DiscoveredCapabilities(
            execution_authority="craft-standalone-fixture",
            perception_owner="FixtureScene-Deterministic",
            sdk_version="fixture-1.0",
            allowed_methods=allowed,
            sdk_contract_sha256=self._contract_sha,
            device_bundle_id="craft-fixture-bench",
            device_bundle_sha256=self._contract_sha,
            runtime_status=str(self._last_command.get("status", "idle")),
        )
        missing = tuple(m for m in self.config.capabilities.required_methods if m not in allowed)
        if missing:
            return CapabilityCheckResult(
                status="missing_capabilities",
                discovered=discovered,
                missing_methods=missing,
                detail=f"Fixture host lacks required SDK methods: {', '.join(missing)}",
            )
        return CapabilityCheckResult(status="ready", discovered=discovered, detail="Standalone fixture host ready")

    def check_missing_capabilities(self, required_methods: Sequence[str]) -> dict[str, Any]:
        allowed = set(_FixturePolicySDK.ALLOWED_METHODS)
        missing = [m for m in required_methods if m not in allowed]
        return {
            "status": "missing_capabilities" if missing else "ready",
            "missing_methods": missing,
            "detail": f"Host lacks required SDK methods: {', '.join(missing)}" if missing else "All capabilities present",
        }

    def observe_scene(self) -> SceneObservationResult:
        now_ns = time.time_ns()
        snapshot = {
            "source": "FixtureScene",
            "timestamp_ns": now_ns,
            "step_seq": self._state["step_seq"],
            "objects": list(self._state["objects"]),
            "coffee_machine": {
                "status": self._state["machine_status"],
                "cup_filled": self._state["cup_filled"],
            },
            "l2_pouring_safety": {"safe_to_pour": True},
        }
        bound: dict[str, tuple[Mapping[str, Any], ...]] = {}
        status = "ok"
        detail = "Standalone fixture scene snapshot (evidence_source=fixture)"
        for binding in self.config.objects:
            matches = tuple(
                o
                for o in self._state["objects"]
                if o["semantic_class"] in binding.semantic_classes and o["confidence"] >= binding.min_confidence
            )
            bound[binding.name] = matches
            if len(matches) < binding.required_count:
                status = "missing_object"
                detail = f"Object binding {binding.name!r} missing"
            elif not binding.allow_ambiguous and len(matches) > binding.required_count:
                status = "ambiguous_object"
                detail = f"Object binding {binding.name!r} ambiguous ({len(matches)} matches)"
        return SceneObservationResult(
            status=status,
            evidence_source="fixture",
            snapshot_sha256=digest(snapshot),
            timestamp_ns=now_ns,
            age_ms=0.0,
            bound_objects=bound,
            raw_snapshot=snapshot,
            detail=detail,
        )

    def list_cameras(self) -> tuple[str, ...]:
        return ("overhead",)

    def get_live_runtime(self) -> dict[str, Any]:
        return {
            "status": str(self._last_command.get("status", "idle")),
            "command_id": self._last_command.get("command_id"),
            "steps": list(self._last_command.get("steps") or []),
        }

    def fetch_camera_image(self, camera_name: str) -> tuple[bytes, str] | None:
        if camera_name != "overhead":
            return None
        cups_svg = []
        for obj in self._state["objects"]:
            x, y = obj["center_world_m"][0], obj["center_world_m"][1]
            px = int(120 + (x - 0.25) * 700)
            py = int(180 - y * 420)
            fill = "#e11d48" if obj["semantic_class"] == "dine_in_red" else "#10b981"
            ring = "#facc15" if self._state["cup_filled"] and obj["object_id"] == "obj-cup-a" else "#f8fafc"
            cups_svg.append(
                f'<circle cx="{px}" cy="{py}" r="18" fill="{fill}" stroke="{ring}" stroke-width="3"/>'
                f'<text x="{px}" y="{py + 34}" fill="#e2e8f0" font-size="11" text-anchor="middle">{obj["object_id"]}</text>'
            )
        svg = f"""<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 640 360">
  <rect width="640" height="360" fill="#0f172a"/>
  <rect x="40" y="30" width="560" height="300" rx="14" fill="#1e293b" stroke="#334155" stroke-width="2"/>
  <rect x="235" y="145" width="70" height="70" rx="8" fill="#334155" stroke="#38bdf8" stroke-dasharray="4 3"/>
  <text x="270" y="138" fill="#38bdf8" font-size="11" text-anchor="middle">SPOUT [0.45, 0.00]</text>
  <rect x="180" y="220" width="66" height="58" rx="8" fill="#1e3a5f" stroke="#22c55e" stroke-dasharray="4 3"/>
  <text x="213" y="293" fill="#4ade80" font-size="11" text-anchor="middle">PICKUP_1 [0.38,-0.17]</text>
  <rect x="430" y="55" width="130" height="54" rx="8" fill="#450a0a" stroke="#ef4444" stroke-dasharray="4 3"/>
  <text x="495" y="86" fill="#fca5a5" font-size="11" text-anchor="middle">STEAM NO-GO ZONE</text>
  {''.join(cups_svg)}
  <text x="52" y="54" fill="#94a3b8" font-size="12">TRAX Standalone Fixture Bench (evidence_source=fixture, NOT simulator/robot)</text>
  <text x="52" y="316" fill="#cbd5e1" font-size="11">preset={self._state['preset']} | machine={self._state['machine_status']} | filled={self._state['cup_filled']}</text>
</svg>"""
        return svg.encode("utf-8"), "image/svg+xml"

    def confirm_object(self, object_id: str) -> dict[str, Any]:
        self._state["confirmed_cup_id"] = object_id
        return {"ok": True, "confirmed_cup_id": object_id}

    def reset_scenario(self, scenario: Mapping[str, Any]) -> dict[str, Any]:
        preset = str(scenario.get("preset", "default"))
        cup_a_xy = scenario.get("cup_a_xy")
        if preset == "sim_dual_cup":
            a_xy = list(cup_a_xy) if cup_a_xy else [0.34, -0.09]
            b_xy = list(scenario.get("cup_b_xy") or [0.35, 0.12])
            objs = [
                {"object_id": "obj-cup-a", "semantic_class": "dine_in_red", "label": "Dine-in Red", "confidence": 0.64, "center_world_m": [a_xy[0], a_xy[1], 1.075]},
                {"object_id": "obj-cup-b", "semantic_class": "takeout_green", "label": "Takeout Green", "confidence": 0.61, "center_world_m": [b_xy[0], b_xy[1], 1.082]},
            ]
        elif preset == "sim_pose_offset":
            a_xy = list(cup_a_xy) if cup_a_xy else [0.35, 0.064]
            objs = [{"object_id": "obj-cup-a", "semantic_class": "dine_in_red", "label": "Dine-in Red", "confidence": 0.94, "center_world_m": [a_xy[0], a_xy[1], 1.075]}]
        else:
            a_xy = list(cup_a_xy) if cup_a_xy else [0.33, 0.0]
            objs = [{"object_id": "obj-cup-a", "semantic_class": "dine_in_red", "label": "Dine-in Red", "confidence": 0.95, "center_world_m": [a_xy[0], a_xy[1], 1.075]}]
        self._state = {
            "preset": preset,
            "step_seq": self._state.get("step_seq", 0) + 5,
            "objects": objs,
            "machine_status": "idle",
            "cup_filled": False,
            "confirmed_cup_id": None,
        }
        return {"ok": True, "preset": preset}

    def run_policy_scenario(
        self,
        policy_source: str,
        *,
        command_id: str,
        scenario: Mapping[str, Any],
        journal: Journal | None = None,
    ) -> dict[str, Any]:
        started = time.monotonic()
        sdk = _FixturePolicySDK(self, scenario)
        safe_builtins = {
            name: __builtins__[name] if isinstance(__builtins__, dict) else getattr(__builtins__, name)
            for name in ("dict", "list", "len", "str", "float", "int", "bool", "min", "max", "round", "range", "abs", "all", "any", "sum", "sorted", "tuple")
        }
        ns: dict[str, Any] = {"__builtins__": safe_builtins}
        try:
            ast.parse(policy_source)
            exec(compile(policy_source, "<fixture-policy>", "exec"), ns, ns)  # noqa: S102
            fn = ns["run_policy"]
            params = {
                "order_id": str(scenario.get("order_id", "A042")),
                "target_slot": str(scenario.get("target_slot", "$SLOT_PICKUP_1")),
                "command_prefix": command_id,
                "instruction": str(scenario.get("instruction", "")),
            }
            out = fn(sdk, params)
            host_status = str((out or {}).get("status", "failed"))
        except Exception as exc:  # noqa: BLE001
            out = {"status": "failed", "error": str(exc)}
            host_status = "failed"

        wall_s = time.monotonic() - started
        cup_pos = self._state["objects"][0]["center_world_m"] if self._state["objects"] else [0.0, 0.0, 0.0]
        target_slot = str(scenario.get("target_slot", "$SLOT_PICKUP_1"))
        goal_xy = (0.38, 0.17) if target_slot == "$SLOT_PICKUP_2" else (0.38, -0.17)
        err_mm = round(math.hypot(float(cup_pos[0]) - goal_xy[0], float(cup_pos[1]) - goal_xy[1]) * 1000.0, 3)
        expect_status = str(scenario.get("expect_status", "completed"))
        max_err = float(scenario.get("max_delivery_error_mm", self.config.acceptance.max_delivery_error_mm))
        fault = bool(scenario.get("simulate_timeout", False) or scenario.get("fault_inject_slip_xy_m"))

        if expect_status == "completed":
            passed = (
                host_status in {"completed", "done"}
                and bool(self._state["cup_filled"])
                and err_mm <= max_err
                and sdk.brew_count == 1
                and (sdk.unknown_events == 0 or sdk.queried_after_unknown)
            )
            if scenario.get("fault_inject_slip_xy_m"):
                passed = passed and any(s["method"] == "recenter_cup_under_spout" for s in sdk.steps)
        else:
            acceptable = {expect_status}
            if expect_status == "paused_for_operator":
                acceptable.add("rejected_observation_not_ready")
            passed = (host_status in acceptable) and sdk.brew_count == 0

        self._last_command = {
            "command_id": command_id,
            "status": host_status,
            "brew_count": sdk.brew_count,
            "steps": list(sdk.steps),
        }
        return {
            "command_id": command_id,
            "evidence_source": "fixture",
            "host_runtime_status": host_status,
            "status": "completed" if host_status in {"completed", "done"} else host_status,
            "wall_s": round(wall_s, 4),
            "methods": [s["method"] for s in sdk.steps],
            "non_idempotent_counts": {"start_coffee_machine": sdk.brew_count},
            "duplicate_non_idempotent": sdk.brew_count > 1,
            "unknown_events_observed": sdk.unknown_events,
            "queried_after_unknown": sdk.queried_after_unknown,
            "human_interventions": sdk.human_interventions,
            "oracle_post_run": {
                "cup_filled": bool(self._state["cup_filled"]),
                "delivery_error_mm": err_mm,
            },
            "fault_injection": fault,
            "expect_status": expect_status,
            "task_passed": passed,
            "detail": f"Fixture run finished with status={host_status}",
        }

    def reconcile_command(self, command_id: str, *, journal: Journal | None = None) -> ReconciliationResult:
        if self._last_command.get("command_id") == command_id:
            st = self._last_command.get("status")
            return ReconciliationResult(
                status="reconciled_done" if st in {"completed", "done"} else "reconciled_failed",
                command_id=command_id,
                host_runtime_status=str(st),
                non_idempotent_counts={"start_coffee_machine": int(self._last_command.get("brew_count", 0))},
                detail="Reconciled from fixture state",
            )
        return ReconciliationResult(status="still_unknown", command_id=command_id, detail="Unknown fixture command")


class HttpHostDriver:
    """Generic config-driven HTTP host driver (zero dependency on any specific host package)."""

    driver_name = "http"

    def __init__(self, config: SceneConfig, *, adapter: EdgeHostAdapter | None = None) -> None:
        self.config = config
        self.evidence_source = config.budget.deployment_scope
        self.adapter = adapter or EdgeHostAdapter(config)
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def discover_capabilities(self, *, allow_busy: bool = False) -> CapabilityCheckResult:
        return self.adapter.discover_capabilities(allow_busy=allow_busy)

    def check_missing_capabilities(self, required_methods: Sequence[str]) -> dict[str, Any]:
        cap = self.adapter.discover_capabilities(allow_busy=True)
        if cap.status == "disconnected" or cap.discovered is None:
            return {"status": "disconnected", "missing_methods": list(required_methods), "detail": cap.detail}
        allowed = set(cap.discovered.allowed_methods)
        missing = [m for m in required_methods if m not in allowed]
        return {
            "status": "missing_capabilities" if missing else "ready",
            "missing_methods": missing,
            "detail": f"Host lacks required SDK methods: {', '.join(missing)}" if missing else "All capabilities present",
        }

    def observe_scene(self) -> SceneObservationResult:
        return self.adapter.observe_scene()

    def get_live_runtime(self) -> dict[str, Any]:
        url = self.config.host.normalized_base_url + self.config.host.cap_state_path
        try:
            state = self.adapter.transport.get_json(url, self.config.host.timeout_s)
        except Exception:  # noqa: BLE001
            return {"status": "disconnected", "command_id": None, "steps": []}
        rt = state.get("runtime") or {}
        return {
            "status": str(rt.get("status") or "idle"),
            "command_id": rt.get("commandId"),
            "steps": list(rt.get("steps") or []),
            "updated_at_ms": rt.get("updatedAtMs"),
        }

    def list_cameras(self) -> tuple[str, ...]:
        obs = self.adapter.observe_scene()
        cams = (obs.raw_snapshot or {}).get("cameras_available") or []
        return tuple(str(c) for c in cams)

    def fetch_camera_image(self, camera_name: str) -> tuple[bytes, str] | None:
        safe = "".join(ch for ch in camera_name if ch.isalnum() or ch in ("_", "-"))
        if not safe:
            return None
        rel = self.config.host.camera_path_template.format(camera=safe)
        url = f"{self.config.host.normalized_base_url}{rel}"
        req = urllib.request.Request(url, method="GET")
        try:
            with self._opener.open(req, timeout=self.config.host.timeout_s) as resp:
                return resp.read(), resp.headers.get("Content-Type", "image/jpeg")
        except (urllib.error.URLError, TimeoutError):
            return None

    def confirm_object(self, object_id: str) -> dict[str, Any]:
        url = f"{self.config.host.normalized_base_url}{self.config.host.confirm_path}"
        return self.adapter.transport.post_json(url, {"cupId": object_id}, self.config.host.timeout_s)

    def reset_scenario(self, scenario: Mapping[str, Any]) -> dict[str, Any]:
        if not self.config.budget.allow_sim_reset:
            raise PermissionError("budget.allow_sim_reset is False")
        eval_base = self.config.host.normalized_evaluator_base
        if not eval_base:
            raise ValueError("host evaluator/sim base URL is required for scenario reset")
        cap_reset = self.adapter.transport.post_json(
            self.config.host.normalized_base_url + self.config.host.cap_reset_path,
            {},
            self.config.host.timeout_s,
        )
        setup_body: dict[str, Any] = {"preset": str(scenario.get("preset", "default"))}
        if scenario.get("cup_a_xy") is not None:
            setup_body["cupAXy"] = list(scenario["cup_a_xy"])
        if scenario.get("cup_b_xy") is not None:
            setup_body["cupBXy"] = list(scenario["cup_b_xy"])
        sim_setup = self.adapter.transport.post_json(
            eval_base + self.config.host.testbed_setup_path,
            setup_body,
            self.config.host.timeout_s,
        )
        initial = self.adapter.observe_scene()
        init_seq = int((initial.raw_snapshot or {}).get("step_seq") or 0)
        deadline = time.monotonic() + 15.0
        while True:
            obs = self.adapter.observe_scene()
            seq = int((obs.raw_snapshot or {}).get("step_seq") or 0)
            if seq >= init_seq + 3 and obs.status not in {"stale_evidence", "disconnected"}:
                break
            if time.monotonic() > deadline:
                break
            time.sleep(0.25)
        return {"cap_reset": cap_reset, "sim_setup": sim_setup}

    def run_policy_scenario(
        self,
        policy_source: str,
        *,
        command_id: str,
        scenario: Mapping[str, Any],
        journal: Journal | None = None,
    ) -> dict[str, Any]:
        slip = scenario.get("fault_inject_slip_xy_m")
        sim_timeout = bool(scenario.get("simulate_timeout", False))
        exec_res = self.adapter.execute_policy(
            policy_source,
            command_id=command_id,
            order_id=str(scenario.get("order_id", "A042")),
            target_slot=str(scenario.get("target_slot", "$SLOT_PICKUP_1")),
            simulate_timeout=sim_timeout,
            inject_place_slip_xy_m=list(slip) if slip is not None else None,
            journal=journal,
        )
        oracle: dict[str, Any] = {}
        err_mm: float | None = None
        cup_filled = False
        target_slot = str(scenario.get("target_slot", "$SLOT_PICKUP_1"))
        goal_xy = (0.38, 0.17) if target_slot == "$SLOT_PICKUP_2" else (0.38, -0.17)
        if self.config.host.normalized_evaluator_base:
            oracle = self.adapter.transport.get_json(
                self.config.host.normalized_evaluator_base + self.config.host.testbed_oracle_path,
                self.config.host.timeout_s,
            )
            cup_pos = oracle.get("cup") or [0.0, 0.0, 0.0]
            err_mm = round(math.hypot(float(cup_pos[0]) - goal_xy[0], float(cup_pos[1]) - goal_xy[1]) * 1000.0, 3)
            cup_filled = bool((oracle.get("coffee_machine") or {}).get("cup_filled"))

        brew_count = int(exec_res.non_idempotent_counts.get("start_coffee_machine", 0))
        expect_status = str(scenario.get("expect_status", "completed"))
        max_err = float(scenario.get("max_delivery_error_mm", self.config.acceptance.max_delivery_error_mm))
        fault = bool(sim_timeout or slip is not None)

        if expect_status == "completed":
            passed = (
                exec_res.status == "completed"
                and cup_filled
                and (err_mm is None or err_mm <= max_err)
                and not exec_res.duplicate_non_idempotent
            )
            if slip is not None:
                passed = passed and ("recenter_cup_under_spout" in exec_res.methods)
            if sim_timeout:
                passed = (
                    passed
                    and brew_count == 1
                    and exec_res.unknown_events_observed >= 1
                    and exec_res.queried_after_unknown
                )
        else:
            host_st = str(exec_res.host_runtime_status or "")
            acceptable = {expect_status}
            if expect_status == "paused_for_operator":
                acceptable.add("rejected_observation_not_ready")
            passed = (host_st in acceptable) and brew_count == 0

        return {
            "command_id": command_id,
            "evidence_source": self.evidence_source,
            "host_runtime_status": exec_res.host_runtime_status,
            "status": exec_res.status,
            "wall_s": round(exec_res.wall_s, 3),
            "methods": list(exec_res.methods),
            "step_records": list((exec_res.raw_runtime or {}).get("steps") or []),
            "non_idempotent_counts": dict(exec_res.non_idempotent_counts),
            "duplicate_non_idempotent": exec_res.duplicate_non_idempotent,
            "unknown_events_observed": exec_res.unknown_events_observed,
            "queried_after_unknown": exec_res.queried_after_unknown,
            "human_interventions": exec_res.human_interventions,
            "sandbox_mode": exec_res.sandbox_mode,
            "oracle_post_run": {
                "cup_filled": cup_filled,
                "delivery_error_mm": err_mm,
            },
            "fault_injection": fault,
            "expect_status": expect_status,
            "task_passed": passed,
            "detail": exec_res.detail,
        }

    def reconcile_command(self, command_id: str, *, journal: Journal | None = None) -> ReconciliationResult:
        return self.adapter.reconcile_command(command_id, journal=journal)


# Compatibility alias
HttpEdgeHostDriver = HttpHostDriver

DriverFactory = Callable[[SceneConfig], SceneHostDriver]

def _create_recorded_driver(config: SceneConfig) -> SceneHostDriver:
    from .recorded_driver import RecordedDataHostDriver
    return RecordedDataHostDriver(config)


def _create_mujoco_cafe_driver(config: SceneConfig) -> SceneHostDriver:
    from .mujoco_cafe_driver import MuJoCoCafeHostDriver
    return MuJoCoCafeHostDriver(config)


def _create_mujoco_so101_driver(config: SceneConfig) -> SceneHostDriver:
    from .mujoco_so101_driver import MuJoCoSO101HostDriver
    return MuJoCoSO101HostDriver(config)


def _create_gazebo_cafe_driver(config: SceneConfig) -> SceneHostDriver:
    from .gazebo_cafe_driver import GazeboCafeHostDriver
    return GazeboCafeHostDriver(config)


_DRIVER_REGISTRY: dict[str, DriverFactory] = {
    "fixture": FixtureHostDriver,
    "http": HttpHostDriver,
    "http_edge": HttpHostDriver,
    "recorded": _create_recorded_driver,
    "mujoco_cafe": _create_mujoco_cafe_driver,
    "mujoco_so101": _create_mujoco_so101_driver,
    "gazebo_cafe": _create_gazebo_cafe_driver,
}


def register_host_driver(name: str, factory: DriverFactory) -> None:
    """Register an external SceneHostDriver plugin (dependency inversion: hosts depend on TRAX)."""
    if not name or not name.strip():
        raise ValueError("Driver name must be non-empty")
    _DRIVER_REGISTRY[name.strip()] = factory


def _discover_entrypoint_drivers() -> dict[str, DriverFactory]:
    discovered: dict[str, DriverFactory] = {}
    try:
        eps = importlib_metadata.entry_points(group="craft.drivers")
    except TypeError:
        eps = importlib_metadata.entry_points().get("craft.drivers", ())  # type: ignore[arg-type]
    for ep in eps:
        try:
            discovered[ep.name] = ep.load()
        except Exception:  # noqa: BLE001
            continue
    return discovered


def list_host_drivers() -> tuple[str, ...]:
    """List built-in, runtime-registered, and entry-point ('craft.drivers') host drivers."""
    names = set(_DRIVER_REGISTRY.keys()) | set(_discover_entrypoint_drivers().keys())
    return tuple(sorted(names))


def create_host_driver(driver_type: str, config: SceneConfig) -> SceneHostDriver:
    """Instantiate a SceneHostDriver by registered name or Python entry point ('craft.drivers')."""
    if driver_type in _DRIVER_REGISTRY:
        return _DRIVER_REGISTRY[driver_type](config)
    ep_drivers = _discover_entrypoint_drivers()
    if driver_type in ep_drivers:
        return ep_drivers[driver_type](config)
    raise ValueError(
        f"Unknown TRAX host driver: {driver_type!r}. "
        f"Available drivers: {', '.join(list_host_drivers())}"
    )
