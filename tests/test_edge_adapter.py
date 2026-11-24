"""Tests for EdgeHostAdapter capability discovery, EvidenceScene pass-through, and reconciliation."""
from __future__ import annotations

from typing import Any, Mapping
import pytest

from trax.config import (
    CapabilityRequirementConfig,
    ExperimentBudgetConfig,
    HostConnectionConfig,
    ObjectBindingConfig,
    SceneConfig,
)
from trax.contracts import digest
from trax.edge_adapter import EdgeHostAdapter, TransportError, TransportTimeoutError
from trax.journal import Journal


COFFEE_METHODS = [
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
]


class FakeTransport:
    def __init__(self) -> None:
        self.get_responses: dict[str, Any] = {}
        self.post_responses: dict[str, Any] = {}
        self.calls: list[tuple[str, str]] = []

    def get_json(self, url: str, timeout_s: float) -> dict[str, Any]:
        self.calls.append(("GET", url))
        resp = self.get_responses.get(url)
        if isinstance(resp, Exception):
            raise resp
        if callable(resp):
            return resp()
        return resp

    def post_json(self, url: str, body: Mapping[str, Any], timeout_s: float) -> dict[str, Any]:
        self.calls.append(("POST", url))
        resp = self.post_responses.get(url)
        if isinstance(resp, Exception):
            raise resp
        if callable(resp):
            return resp(body)
        return resp


def _make_config(
    *,
    required_methods: tuple[str, ...] = ("observe_scene", "start_coffee_machine", "query_command_status"),
    expected_hash: str | None = None,
    allow_live_actuation: bool = True,
    max_evidence_age_ms: float = 2000.0,
) -> SceneConfig:
    return SceneConfig(
        scene_id="cafe-sim",
        task_name="single_cup_coffee",
        host=HostConnectionConfig(
            edge_base_url="http://127.0.0.1:8080",
            max_evidence_age_ms=max_evidence_age_ms,
        ),
        objects=(
            ObjectBindingConfig(
                name="coffee_cup",
                semantic_classes=("dine_in_red", "takeout_green"),
                required_count=1,
                allow_ambiguous=False,
                min_confidence=0.5,
            ),
        ),
        capabilities=CapabilityRequirementConfig(
            required_methods=required_methods,
            non_idempotent_methods=("start_coffee_machine",),
            expected_sdk_contract_sha256=expected_hash,
        ),
        budget=ExperimentBudgetConfig(
            allow_live_actuation=allow_live_actuation,
            deployment_scope="simulator",
        ),
    )


def _cap_state_payload(*, status: str = "idle", methods=None, contract_sha="abc123", command_id=None, plan=None):
    return {
        "ok": True,
        "executionAuthority": "ticos-edge-zero-gt",
        "perceptionOwner": "EvidenceScene-v2.0.0-4cam-RGBD",
        "sdkVersion": "1.0.0",
        "sdkAllowedMethods": COFFEE_METHODS if methods is None else methods,
        "sdkContractSha256": contract_sha,
        "deviceBundleId": "cafe-default",
        "deviceBundleSha256": "bundle-sha",
        "runtime": {
            "status": status,
            "commandId": command_id,
            "auditRecord": {"command_id": command_id, "plan": plan or []},
        },
    }


def test_discover_capabilities_ready_missing_drift_busy_and_disconnected():
    t = FakeTransport()
    state_url = "http://127.0.0.1:8080/cap/state"

    # Ready
    t.get_responses[state_url] = _cap_state_payload(contract_sha="hash-1")
    res = EdgeHostAdapter(_make_config(expected_hash="hash-1"), transport=t).discover_capabilities()
    assert res.status == "ready"
    assert res.discovered is not None
    assert res.discovered.perception_owner == "EvidenceScene-v2.0.0-4cam-RGBD"

    # Missing oven capabilities
    oven_cfg = _make_config(required_methods=("observe_scene", "open_oven_door", "start_oven_heat"))
    res = EdgeHostAdapter(oven_cfg, transport=t).discover_capabilities()
    assert res.status == "missing_capabilities"
    assert res.missing_methods == ("open_oven_door", "start_oven_heat")

    # Contract drift
    res = EdgeHostAdapter(_make_config(expected_hash="different-hash"), transport=t).discover_capabilities()
    assert res.status == "contract_drift"

    # Host busy
    t.get_responses[state_url] = _cap_state_payload(status="running")
    res = EdgeHostAdapter(_make_config(), transport=t).discover_capabilities()
    assert res.status == "host_busy"

    # Disconnected
    t.get_responses[state_url] = TransportError("Connection refused")
    res = EdgeHostAdapter(_make_config(), transport=t).discover_capabilities()
    assert res.status == "disconnected"


def test_observe_scene_passthrough_fresh_stale_missing_ambiguous_disconnected():
    t = FakeTransport()
    scene_url = "http://127.0.0.1:8080/edge-scene"
    adapter = EdgeHostAdapter(_make_config(max_evidence_age_ms=1000.0), transport=t)
    now_ns = 10_000_000_000

    # Fresh single cup: exact snapshot preserved without fabrication
    snap = {
        "source": "EvidenceScene",
        "timestamp_ns": now_ns - 150_000_000,  # 150 ms old
        "objects": [
            {"object_id": "cup_1", "semantic_class": "dine_in_red", "confidence": 0.92, "center_world_m": [0.1, 0.2, 0.3]},
            {"object_id": "machine_1", "semantic_class": "equipment", "confidence": 0.99},
        ],
    }
    t.get_responses[scene_url] = snap
    obs = adapter.observe_scene(now_ns=now_ns)
    assert obs.status == "ok"
    assert obs.age_ms == 150.0
    assert obs.snapshot_sha256 == digest(snap)
    assert obs.raw_snapshot == snap
    assert len(obs.bound_objects["coffee_cup"]) == 1

    # Stale evidence
    stale_snap = dict(snap, timestamp_ns=now_ns - 5_000_000_000)  # 5000 ms old
    t.get_responses[scene_url] = stale_snap
    obs = adapter.observe_scene(now_ns=now_ns)
    assert obs.status == "stale_evidence"
    assert obs.age_ms == 5000.0

    # Missing object
    t.get_responses[scene_url] = dict(snap, objects=[])
    obs = adapter.observe_scene(now_ns=now_ns)
    assert obs.status == "missing_object"

    # Ambiguous object (2 cups when 1 required)
    t.get_responses[scene_url] = dict(
        snap,
        objects=[
            {"object_id": "cup_1", "semantic_class": "dine_in_red", "confidence": 0.9},
            {"object_id": "cup_2", "semantic_class": "takeout_green", "confidence": 0.85},
        ],
    )
    obs = adapter.observe_scene(now_ns=now_ns)
    assert obs.status == "ambiguous_object"

    # Disconnected
    t.get_responses[scene_url] = TransportError("timeout")
    obs = adapter.observe_scene(now_ns=now_ns)
    assert obs.status == "disconnected"


def test_execute_permission_denied_and_unknown_outcome_never_blind_retries(tmp_path):
    t = FakeTransport()
    exec_url = "http://127.0.0.1:8080/cap/execute"
    state_url = "http://127.0.0.1:8080/cap/state"

    # Permission denied when allow_live_actuation is False
    denied_adapter = EdgeHostAdapter(_make_config(allow_live_actuation=False), transport=t)
    denied = denied_adapter.execute_policy("def run_policy(sdk, p): return {}", command_id="cmd-0")
    assert denied.status == "permission_denied"
    assert t.calls == []

    # Timeout during execute -> unknown_outcome; second call refuses blind retry; reconcile queries /cap/state
    adapter = EdgeHostAdapter(_make_config(allow_live_actuation=True), transport=t)
    t.post_responses[exec_url] = TransportTimeoutError("read timed out")
    with Journal(tmp_path / "j.sqlite") as journal:
        first = adapter.execute_policy("def run_policy(sdk, p): return {}", command_id="cmd-1", journal=journal)
        assert first.status == "unknown_outcome"
        assert sum(1 for m, _ in t.calls if m == "POST") == 1

        # Second attempt with same command_id must NOT dispatch POST again
        second = adapter.execute_policy("def run_policy(sdk, p): return {}", command_id="cmd-1", journal=journal)
        assert second.status == "reconciliation_required"
        assert sum(1 for m, _ in t.calls if m == "POST") == 1

        # Reconcile queries GET /cap/state and resolves completed
        t.get_responses[state_url] = _cap_state_payload(
            status="completed",
            command_id="cmd-1",
            plan=[{"skill_kind": "start_coffee_machine", "execution_count": 1}],
        )
        rec = adapter.reconcile_command("cmd-1", journal=journal)
        assert rec.status == "reconciled_done"
        assert rec.non_idempotent_counts == {"start_coffee_machine": 1}
        assert sum(1 for m, _ in t.calls if m == "POST") == 1


def test_execute_detects_fault_recovery_and_duplicate_non_idempotent():
    t = FakeTransport()
    exec_url = "http://127.0.0.1:8080/cap/execute"
    adapter = EdgeHostAdapter(_make_config(allow_live_actuation=True), transport=t)

    # Proper recovery: unknown ack -> query_command_status, execution_count == 1
    t.post_responses[exec_url] = {
        "runtime": {
            "status": "completed",
            "steps": [
                {"method": "observe_scene"},
                {"method": "place_cup_under_machine"},
                {"method": "recenter_cup_under_spout"},
                {"method": "start_coffee_machine"},
                {"method": "command_ack_timeout_unknown"},
                {"method": "query_command_status"},
                {"method": "retrieve_and_deliver_cup"},
            ],
            "auditRecord": {"plan": [{"skill_kind": "start_coffee_machine", "execution_count": 1}]},
            "result": {"sandbox_isolation": {"mode": "subprocess"}},
        }
    }
    res = adapter.execute_policy("def run_policy(sdk, p): return {}", command_id="cmd-ok")
    assert res.status == "completed"
    assert res.unknown_events_observed == 1
    assert res.queried_after_unknown is True
    assert res.duplicate_non_idempotent is False
    assert res.sandbox_mode == "subprocess"

    # Duplicate non-idempotent brew detected
    t.post_responses[exec_url] = {
        "runtime": {
            "status": "completed",
            "steps": [{"method": "start_coffee_machine"}, {"method": "start_coffee_machine"}],
            "auditRecord": {"plan": [{"skill_kind": "start_coffee_machine", "execution_count": 2}]},
            "result": {},
        }
    }
    dup = adapter.execute_policy("def run_policy(sdk, p): return {}", command_id="cmd-dup")
    assert dup.duplicate_non_idempotent is True
    assert dup.non_idempotent_counts["start_coffee_machine"] == 2

    # Host runtime returning status='done' (policy alias for completed) is recognized as completed
    t.post_responses[exec_url] = {
        "runtime": {
            "status": "done",
            "steps": [{"method": "start_coffee_machine"}],
            "auditRecord": {"plan": [{"skill_kind": "start_coffee_machine", "execution_count": 1}]},
            "result": {"sandbox_isolation": {"mode": "os_subprocess_isolated"}},
        }
    }
    done_res = adapter.execute_policy("def run_policy(sdk, p): return {'status': 'done'}", command_id="cmd-done")
    assert done_res.status == "completed"
    assert done_res.host_runtime_status == "done"
