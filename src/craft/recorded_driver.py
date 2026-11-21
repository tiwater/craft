"""Host-independent offline recorded-data replay driver (evidence_source='recorded_data').

Replays real-world recorded perception frames (JSON or JSONL) in open-loop shadow
mode with ZERO physical actuation. Used to verify fail-closed policy behavior under
authentic sensor noise, staleness, occlusion, ambiguity, and L2 safety violations
before touching a physical robot.
"""
from __future__ import annotations

import ast
import json
from pathlib import Path
import time
from typing import Any, Mapping, Sequence

from .config import SceneConfig
from .contracts import digest
from .edge_adapter import (
    CapabilityCheckResult,
    DiscoveredCapabilities,
    ReconciliationResult,
    SceneObservationResult,
)
from .journal import Journal


class _ShadowRecordedSDK:
    """Open-loop shadow SDK backed by recorded real-world frames (never actuates)."""

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

    def __init__(self, frames: Sequence[Mapping[str, Any]], scenario: Mapping[str, Any]) -> None:
        self._frames = list(frames)
        self._cursor = 0
        self._scenario = scenario
        self.steps: list[dict[str, Any]] = []
        self.shadow_actuations: list[dict[str, Any]] = []
        self.brew_intents = 0
        self.human_interventions = 0

    def _current_frame(self) -> Mapping[str, Any]:
        if not self._frames:
            return {"control_ready": False, "objects": [], "l2_pouring_safety": {"safe_to_pour": False}}
        idx = min(self._cursor, len(self._frames) - 1)
        return self._frames[idx]

    def _rec(self, method: str, detail: str = "") -> None:
        self.steps.append({"method": method, "receipt": "shadow", "detail": detail})

    def get_order(self, order_id: str = "A042") -> dict[str, Any]:
        self._rec("get_order")
        return {
            "order_id": order_id,
            "drink": "hot_americano",
            "cup_preference": "dine_in_red",
            "destination_slot": str(self._scenario.get("target_slot", "$SLOT_PICKUP_1")),
            "special_request": self._scenario.get("special_request"),
            "status": "pending",
        }

    def observe_scene(self) -> dict[str, Any]:
        frame = self._current_frame()
        self._cursor = min(self._cursor + 1, max(0, len(self._frames) - 1))
        self._rec("observe_scene")
        raw_objs = frame.get("objects") or []
        norm_objs = [
            {
                "id": str(o.get("id") or o.get("object_id") or f"obj-{i}"),
                "cup_type": str(o.get("cup_type") or o.get("semantic_class") or "unknown"),
                "label": str(o.get("label") or o.get("semantic_class") or "object"),
                "confidence": float(o.get("confidence", 0.0)),
                "world": list(o.get("world") or o.get("center_world_m") or [0.0, 0.0, 0.0]),
                "tilt_deg": float(o.get("tilt_deg", 0.0)),
            }
            for i, o in enumerate(raw_objs)
        ]
        l2 = frame.get("l2_pouring_safety") or {"safe_to_pour": False}
        return {
            "control_ready": bool(frame.get("control_ready", False)),
            "pipeline_health": dict(frame.get("pipeline_health") or {"source": "recorded_data"}),
            "objects": norm_objs,
            "coffee_machine": dict(
                frame.get("coffee_machine")
                or {"status": frame.get("brew_status", "idle"), "cup_filled": bool(frame.get("cup_filled", False))}
            ),
            "l2_pouring_safety": dict(l2),
            "confidence_threshold": float(frame.get("confidence_threshold", 0.75)),
            "operator_confirmed_cup_id": frame.get("operator_confirmed_cup_id"),
        }

    def request_operator_confirmation(self, reason: str, candidates: list[Any]) -> dict[str, Any]:
        self._rec("request_operator_confirmation", reason)
        self.human_interventions += 1
        return {"receipt": "paused_for_operator", "reason": reason, "candidates": candidates}

    def retract_safe_pose(self, *, command_id: str) -> dict[str, Any]:
        self._rec("retract_safe_pose")
        self.shadow_actuations.append({"method": "retract_safe_pose", "command_id": command_id})
        return {"receipt": "done", "shadow": True, "command_id": command_id}

    def escalate_to_human_operator(self, order_id: str, reason: str, *, command_id: str) -> dict[str, Any]:
        self._rec("escalate_to_human_operator", reason)
        self.human_interventions += 1
        return {"receipt": "done", "shadow": True, "command_id": command_id}

    def _shadow_actuate(self, method: str, command_id: str, **kw: Any) -> dict[str, Any]:
        self._rec(method)
        self.shadow_actuations.append({"method": method, "command_id": command_id, **kw})
        return {"receipt": "done", "shadow": True, "command_id": command_id}

    def move_to_target(self, target_world: Any, *, command_id: str) -> dict[str, Any]:
        return self._shadow_actuate("move_to_target", command_id)

    def place_cup_under_machine(self, cup_id: str, *, command_id: str) -> dict[str, Any]:
        return self._shadow_actuate("place_cup_under_machine", command_id, cup_id=cup_id)

    def recenter_cup_under_spout(self, cup_id: str, *, command_id: str) -> dict[str, Any]:
        return self._shadow_actuate("recenter_cup_under_spout", command_id, cup_id=cup_id)

    def start_coffee_machine(self, *, command_id: str) -> dict[str, Any]:
        self.brew_intents += 1
        self._rec("start_coffee_machine")
        self.shadow_actuations.append({"method": "start_coffee_machine", "command_id": command_id})
        return {"receipt": "running", "shadow": True, "command_id": command_id}

    def query_command_status(self, command_id: str) -> dict[str, Any]:
        self._rec("query_command_status")
        return {"receipt": "running", "shadow": True, "command_id": command_id}

    def wait_for_coffee_ready(self, *, command_id: str) -> dict[str, Any]:
        return self._shadow_actuate("wait_for_coffee_ready", command_id)

    def retrieve_and_deliver_cup(self, cup_id: str, target_slot: str, *, command_id: str) -> dict[str, Any]:
        return self._shadow_actuate("retrieve_and_deliver_cup", command_id, cup_id=cup_id, target_slot=target_slot)

    def complete_order_delivery(self, order_id: str, cup_id: str, destination_slot: str, *, command_id: str) -> dict[str, Any]:
        return self._shadow_actuate("complete_order_delivery", command_id, order_id=order_id)


def load_recorded_frames(path: str | Path) -> list[dict[str, Any]]:
    """Load recorded perception frames from a JSON array or JSONL stream."""
    target = Path(path)
    if target.suffix.lower() == ".jsonl":
        frames: list[dict[str, Any]] = []
        with target.open("r", encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                frames.append(json.loads(line))
        return frames
    raw = json.loads(target.read_text(encoding="utf-8"))
    if isinstance(raw, list):
        return [dict(x) for x in raw]
    if isinstance(raw, dict) and isinstance(raw.get("frames"), list):
        return [dict(x) for x in raw["frames"]]
    raise ValueError(f"Unsupported recorded-data structure in {target}")


class RecordedDataHostDriver:
    """Offline recorded-data replay driver implementing SceneHostDriver (shadow-only)."""

    driver_name = "recorded"
    evidence_source = "recorded_data"

    def __init__(
        self,
        config: SceneConfig,
        *,
        frames: Sequence[Mapping[str, Any]] | None = None,
        recording_path: str | Path | None = None,
    ) -> None:
        self.config = config
        if frames is not None:
            self._frames = [dict(f) for f in frames]
        else:
            rec_path = recording_path or config.metadata.get("recording_path")
            self._frames = load_recorded_frames(rec_path) if rec_path else []
        self._index = 0
        self._contract_sha = digest(list(_ShadowRecordedSDK.ALLOWED_METHODS))
        self._last_runtime: dict[str, Any] = {"status": "idle", "command_id": None, "steps": []}

    def discover_capabilities(self, *, allow_busy: bool = False) -> CapabilityCheckResult:
        allowed = _ShadowRecordedSDK.ALLOWED_METHODS
        discovered = DiscoveredCapabilities(
            execution_authority="craft-recorded-shadow-replay",
            perception_owner="RecordedDataReplay",
            sdk_version="recorded-shadow-1.0",
            allowed_methods=allowed,
            sdk_contract_sha256=self._contract_sha,
            device_bundle_id="recorded-replay",
            device_bundle_sha256=self._contract_sha,
            runtime_status=str(self._last_runtime.get("status", "idle")),
        )
        missing = tuple(m for m in self.config.capabilities.required_methods if m not in allowed)
        if missing:
            return CapabilityCheckResult(
                status="missing_capabilities",
                discovered=discovered,
                missing_methods=missing,
                detail=f"Recorded replay lacks methods: {', '.join(missing)}",
            )
        return CapabilityCheckResult(status="ready", discovered=discovered, detail="Recorded shadow replay ready")

    def check_missing_capabilities(self, required_methods: Sequence[str]) -> dict[str, Any]:
        allowed = set(_ShadowRecordedSDK.ALLOWED_METHODS)
        missing = [m for m in required_methods if m not in allowed]
        return {
            "status": "missing_capabilities" if missing else "ready",
            "missing_methods": missing,
            "detail": f"Missing: {', '.join(missing)}" if missing else "All capabilities present",
        }

    def observe_scene(self) -> SceneObservationResult:
        if not self._frames:
            return SceneObservationResult(
                status="stale_evidence",
                evidence_source="recorded_data",
                detail="No recorded frames loaded",
            )
        frame = self._frames[min(self._index, len(self._frames) - 1)]
        raw_objs = frame.get("objects") or []
        bound: dict[str, tuple[Mapping[str, Any], ...]] = {}
        status = "ok" if frame.get("control_ready", False) else "stale_evidence"
        for binding in self.config.objects:
            matches = tuple(
                o
                for o in raw_objs
                if str(o.get("semantic_class") or o.get("cup_type") or "") in binding.semantic_classes
                and float(o.get("confidence", 0.0)) >= binding.min_confidence
            )
            bound[binding.name] = matches
            if status == "ok":
                if len(matches) < binding.required_count:
                    status = "missing_object"
                elif not binding.allow_ambiguous and len(matches) > binding.required_count:
                    status = "ambiguous_object"
        return SceneObservationResult(
            status=status,
            evidence_source="recorded_data",
            snapshot_sha256=digest(frame),
            timestamp_ns=int(frame.get("timestamp_ns") or 0) or None,
            age_ms=float(frame["age_ms"]) if frame.get("age_ms") is not None else 0.0,
            bound_objects=bound,
            raw_snapshot=frame,
            detail=f"Recorded frame {self._index + 1}/{len(self._frames)}",
        )

    def get_live_runtime(self) -> dict[str, Any]:
        return dict(self._last_runtime)

    def list_cameras(self) -> tuple[str, ...]:
        return ()

    def fetch_camera_image(self, camera_name: str) -> tuple[bytes, str] | None:
        return None

    def confirm_object(self, object_id: str) -> dict[str, Any]:
        for f in self._frames:
            f["operator_confirmed_cup_id"] = object_id
        return {"ok": True, "confirmed_cup_id": object_id}

    def reset_scenario(self, scenario: Mapping[str, Any]) -> dict[str, Any]:
        self._index = int(scenario.get("start_frame", 0))
        return {"ok": True, "start_frame": self._index, "total_frames": len(self._frames)}

    def run_policy_scenario(
        self,
        policy_source: str,
        *,
        command_id: str,
        scenario: Mapping[str, Any],
        journal: Journal | None = None,
    ) -> dict[str, Any]:
        started = time.monotonic()
        start_idx = int(scenario.get("start_frame", self._index))
        sdk = _ShadowRecordedSDK(self._frames[start_idx:], scenario)
        safe_builtins = {
            name: __builtins__[name] if isinstance(__builtins__, dict) else getattr(__builtins__, name)
            for name in ("dict", "list", "len", "str", "float", "int", "bool", "min", "max", "round", "range", "abs", "all", "any", "sum", "sorted", "tuple")
        }
        ns: dict[str, Any] = {"__builtins__": safe_builtins}
        try:
            ast.parse(policy_source)
            exec(compile(policy_source, "<recorded-shadow-policy>", "exec"), ns, ns)  # noqa: S102
            params = {
                "order_id": str(scenario.get("order_id", "A042")),
                "target_slot": str(scenario.get("target_slot", "$SLOT_PICKUP_1")),
                "command_prefix": command_id,
            }
            out = ns["run_policy"](sdk, params)
            host_status = str((out or {}).get("status", "failed"))
        except Exception as exc:  # noqa: BLE001
            host_status = "failed"
            out = {"status": "failed", "error": str(exc)}

        wall_s = time.monotonic() - started
        expect_status = str(scenario.get("expect_status", "rejected_observation_not_ready"))
        expect_zero_brew = bool(scenario.get("expect_zero_brew", expect_status != "completed"))
        passed = (host_status == expect_status) and (not expect_zero_brew or sdk.brew_intents == 0)

        self._last_runtime = {"status": host_status, "command_id": command_id, "steps": list(sdk.steps)}
        return {
            "command_id": command_id,
            "evidence_source": "recorded_data",
            "shadow_only": True,
            "physical_commands": 0,
            "host_runtime_status": host_status,
            "status": host_status,
            "wall_s": round(wall_s, 4),
            "methods": [s["method"] for s in sdk.steps],
            "step_records": list(sdk.steps),
            "shadow_actuations": list(sdk.shadow_actuations),
            "non_idempotent_counts": {"start_coffee_machine": sdk.brew_intents},
            "duplicate_non_idempotent": sdk.brew_intents > 1,
            "unknown_events_observed": 0,
            "queried_after_unknown": True,
            "human_interventions": sdk.human_interventions,
            "expect_status": expect_status,
            "task_passed": passed,
            "detail": f"Recorded shadow replay finished with status={host_status} (0 physical commands)",
        }

    def reconcile_command(self, command_id: str, *, journal: Journal | None = None) -> ReconciliationResult:
        return ReconciliationResult(
            status="reconciled_done",
            command_id=command_id,
            host_runtime_status=str(self._last_runtime.get("status", "idle")),
            detail="Recorded replay is shadow-only (0 physical commands)",
        )
