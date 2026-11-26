"""Tests for Spatial State Contract v1, clock-domain freshness, cursor continuity, and perception attribution."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

import pytest

from trax.attribution import classify_mcp_execution
from trax.config import (
    CapabilityRequirementConfig,
    ExperimentBudgetConfig,
    HostConnectionConfig,
    ObjectBindingConfig,
    SceneConfig,
)
from trax.edge_adapter import EdgeHostAdapter
from trax.journal import Journal
from trax.mcp_protocol import MCPClient, MCPServer, MCPToolSpec
from trax.sandbox import GovernedMCPProxy, check_mcp_policy_ast, execute_sandboxed_python
from trax.spatial_contract import (
    ClockTimestamp,
    SceneCursor,
    SpatialChange,
    check_cursor_continuity,
    evaluate_freshness,
)


class _FakeTransport:
    def __init__(self, payload: dict[str, Any]) -> None:
        self.payload = payload

    def get_json(self, url: str, timeout_s: float) -> dict[str, Any]:
        return self.payload

    def post_json(self, url: str, body: Mapping[str, Any], timeout_s: float) -> dict[str, Any]:
        return {}


def _make_cfg(max_age_ms: float = 1000.0) -> SceneConfig:
    return SceneConfig(
        scene_id="v3-test",
        task_name="pick_object",
        host=HostConnectionConfig(edge_base_url="http://127.0.0.1:8080", max_evidence_age_ms=max_age_ms),
        objects=(
            ObjectBindingConfig(
                name="target_obj",
                semantic_classes=("mug_red",),
                required_count=1,
                allow_ambiguous=False,
                min_confidence=0.5,
            ),
        ),
        capabilities=CapabilityRequirementConfig(
            required_methods=("observe_scene",),
            non_idempotent_methods=("trigger_station",),
        ),
        budget=ExperimentBudgetConfig(allow_live_actuation=True, deployment_scope="simulator"),
    )


def test_clock_domain_freshness_rejects_cross_clock_and_inverted_timestamps() -> None:
    obs = ClockTimestamp(clock_id="sensor_mono_0", ns=10_000_000_000)

    # Matching clock within bound -> fresh
    ok = evaluate_freshness(obs, ClockTimestamp(clock_id="sensor_mono_0", ns=10_250_000_000), max_age_ms=500.0)
    assert ok.status == "fresh"
    assert ok.age_ms == 250.0

    # Matching clock past bound -> stale
    stale = evaluate_freshness(obs, ClockTimestamp(clock_id="sensor_mono_0", ns=11_500_000_000), max_age_ms=500.0)
    assert stale.status == "stale"
    assert stale.age_ms == 1500.0

    # Cross-clock domain -> MUST return unknown, never subtract
    cross = evaluate_freshness(obs, ClockTimestamp(clock_id="host_wall_ns", ns=10_010_000_000), max_age_ms=500.0)
    assert cross.status == "unknown"
    assert cross.age_ms is None
    assert "clock_domain_mismatch" in cross.reason

    # Reference precedes observation -> MUST return unknown, never clamp to 0ms fresh
    inv = evaluate_freshness(obs, ClockTimestamp(clock_id="sensor_mono_0", ns=9_999_000_000), max_age_ms=500.0)
    assert inv.status == "unknown"
    assert inv.age_ms is None
    assert "reference_precedes_observation" in inv.reason


def test_cursor_continuity_detects_reset_scene_change_regression_and_eviction() -> None:
    c0 = SceneCursor(scene_id="scn-a", epoch=1, revision=10)

    assert check_cursor_continuity(c0, SceneCursor("scn-a", 1, 10))[0] == "head"
    assert check_cursor_continuity(c0, SceneCursor("scn-a", 1, 14), earliest_retained_revision=8)[0] == "continuous"
    assert check_cursor_continuity(c0, SceneCursor("scn-b", 1, 11))[0] == "resnapshot_required"
    assert check_cursor_continuity(c0, SceneCursor("scn-a", 2, 0))[0] == "resnapshot_required"
    assert check_cursor_continuity(c0, SceneCursor("scn-a", 1, 9))[0] == "resnapshot_required"
    assert check_cursor_continuity(c0, SceneCursor("scn-a", 1, 25), earliest_retained_revision=15)[0] == "resnapshot_required"


def test_edge_adapter_v3_semantics_clock_revision_three_state_and_observability() -> None:
    now_ns = 50_000_000_000
    base_v3 = {
        "schema_version": 1,
        "cursor": {"scene_id": "scn-1", "epoch": 2, "revision": 5},
        "clock_id": "sensor_mono_2",
        "observation_time": {"clock_id": "sensor_mono_2", "ns": now_ns - 120_000_000},
        "revision_time": {"clock_id": "sensor_mono_2", "ns": now_ns - 50_000_000},
        "observability": {"status": "sufficient", "rank": 6, "required_rank": 6},
        "entities": {
            "ent-1": {
                "entity_id": "ent-1",
                "semantic_label": "mug_red",
                "lifecycle": "active",
                "confidence": 0.94,
                "observability": {"status": "sufficient", "rank": 6},
            },
            "ent-old": {
                "entity_id": "ent-old",
                "semantic_label": "mug_red",
                "lifecycle": "retired",
                "confidence": 0.99,
            },
        },
    }

    # 1. Matching clock & continuous cursor -> ok (retired tombstone ignored)
    adapter = EdgeHostAdapter(_make_cfg(max_age_ms=500.0), transport=_FakeTransport(base_v3))
    res = adapter.observe_scene(
        now_ns=now_ns,
        reference_clock_id="sensor_mono_2",
        expected_cursor=SceneCursor("scn-1", 2, 3),
    )
    assert res.status == "ok"
    assert res.freshness_status == "fresh"
    assert res.age_ms == 120.0
    assert res.cursor == SceneCursor("scn-1", 2, 5)
    assert res.clock_id == "sensor_mono_2"
    assert len(res.bound_objects["target_obj"]) == 1
    assert res.bound_objects["target_obj"][0]["entity_id"] == "ent-1"

    # 2. Cross-clock omission (default wall clock vs sensor_mono_2) -> unknown_freshness
    cross = adapter.observe_scene(now_ns=now_ns)
    assert cross.status == "unknown_freshness"
    assert cross.freshness_status == "unknown"

    # 3. Historical correction with FRESH revision_time but STALE observation_time -> STILL stale_evidence
    late_rev = dict(
        base_v3,
        observation_time={"clock_id": "sensor_mono_2", "ns": now_ns - 3_000_000_000},
        revision_time={"clock_id": "sensor_mono_2", "ns": now_ns - 10_000_000},
    )
    stale = EdgeHostAdapter(_make_cfg(max_age_ms=500.0), transport=_FakeTransport(late_rev)).observe_scene(
        now_ns=now_ns,
        reference_clock_id="sensor_mono_2",
    )
    assert stale.status == "stale_evidence"
    assert stale.freshness_status == "stale"
    assert stale.age_ms == 3000.0
    assert stale.revision_time_ns == now_ns - 10_000_000

    # 4. Epoch reset against expected cursor -> resnapshot_required
    reset_res = adapter.observe_scene(
        now_ns=now_ns,
        reference_clock_id="sensor_mono_2",
        expected_cursor=SceneCursor("scn-1", 1, 99),
    )
    assert reset_res.status == "resnapshot_required"
    assert reset_res.resnapshot_required is True

    # 5. Distinct three-state upstream availability: unknown vs unavailable
    unk = EdgeHostAdapter(_make_cfg(), transport=_FakeTransport(dict(base_v3, availability="unknown"))).observe_scene(
        now_ns=now_ns, reference_clock_id="sensor_mono_2"
    )
    unavail = EdgeHostAdapter(
        _make_cfg(), transport=_FakeTransport(dict(base_v3, availability="unavailable"))
    ).observe_scene(now_ns=now_ns, reference_clock_id="sensor_mono_2")
    assert unk.status == "unknown_evidence"
    assert unavail.status == "unavailable_evidence"

    # 6. Entity-level insufficient observability -> insufficient_observability
    degen = dict(
        base_v3,
        entities={
            "ent-1": {
                "entity_id": "ent-1",
                "semantic_label": "mug_red",
                "lifecycle": "active",
                "observability": {"status": "insufficient", "reason": "handle_occluded"},
            }
        },
    )
    insuff = EdgeHostAdapter(_make_cfg(), transport=_FakeTransport(degen)).observe_scene(
        now_ns=now_ns, reference_clock_id="sensor_mono_2"
    )
    assert insuff.status == "insufficient_observability"
    assert insuff.observability_status == "insufficient"


def test_mcp_proxy_cursor_pinning_and_perception_vs_policy_attribution(tmp_path: Path) -> None:
    state = {"obs_calls": 0, "act_calls": 0}
    server = MCPServer(name="spatial-test-mcp")
    server.register_tool(
        MCPToolSpec(
            name="assess_spatial",
            description="Check entity spatial contract",
            input_schema={"type": "object", "properties": {"mode": {"type": "string"}}},
            read_only=True,
            idempotent=True,
        ),
        lambda args: _assess_handler(args, state),
    )
    server.register_tool(
        MCPToolSpec(
            name="trigger_station",
            description="Non-idempotent actuation",
            input_schema={"type": "object", "properties": {"command_id": {"type": "string"}}},
            read_only=False,
            idempotent=False,
            destructive=True,
        ),
        lambda args: _act_handler(args, state),
    )

    client = MCPClient(transport="local", local_server=server)
    _, tools_resp, _ = client.rpc("tools/list", {})
    tools = tools_resp["result"]["tools"]
    tool_names = [t["name"] for t in tools]
    jpath = tmp_path / "journal.sqlite"

    # Case A: Policy observes insufficient observability and halts cleanly -> perception layer
    src_halt = """
def run_policy(mcp, params):
    r = mcp.assess_spatial(mode="insufficient")
    if r.get("status") != "satisfied":
        return {"status": "blocked_perception"}
    mcp.trigger_station(command_id="never-called")
    return {"status": "done"}
"""
    ast_a = check_mcp_policy_ast(src_halt, allowed_tools=tool_names)
    proxy_a = GovernedMCPProxy(client, tools, journal_path=jpath, run_tag="caseA")
    _, err_a = execute_sandboxed_python(src_halt, proxy_a, entry_fn="run_policy", params={})
    assert err_a is None
    assert state["act_calls"] == 0
    attr_a = classify_mcp_execution(
        ast_check=ast_a,
        proxy=proxy_a,
        verify_result={"passed": False, "detail": "Not executed"},
        runtime_error=err_a,
    )
    assert attr_a["layer"] == "perception"
    assert attr_a["code"] == "insufficient_observability"
    assert attr_a["recommended_owner"] == "perception_owner"
    assert attr_a["auto_repair_eligible"] is False

    # Case B: Reckless policy ignores unresolved perception block and fires non-idempotent tool -> policy layer!
    src_reckless = """
def run_policy(mcp, params):
    mcp.assess_spatial(mode="stale")
    mcp.trigger_station(command_id="reckless-1")
    return {"status": "done"}
"""
    ast_b = check_mcp_policy_ast(src_reckless, allowed_tools=tool_names)
    proxy_b = GovernedMCPProxy(client, tools, journal_path=jpath, run_tag="caseB")
    _, err_b = execute_sandboxed_python(src_reckless, proxy_b, entry_fn="run_policy", params={})
    assert err_b is None
    assert proxy_b.acted_after_perception_block is True
    attr_b = classify_mcp_execution(
        ast_check=ast_b,
        proxy=proxy_b,
        verify_result={"passed": True, "detail": "Lucky pass"},
        runtime_error=err_b,
    )
    assert attr_b["layer"] == "policy"
    assert attr_b["code"] == "acted_on_unverified_perception"

    # Verify SQLite Journal pinned the spatial cursor on non-idempotent claim
    with Journal(jpath) as journal:
        rows = [json.loads(r[0]) for r in journal.db.execute("SELECT request FROM commands").fetchall()]
    assert len(rows) == 1
    assert rows[0]["spatial_cursor"] == {"scene_id": "scn-lab", "epoch": 1, "revision": 7}

    # Case C: Active re-observation clears block -> passes cleanly; post-run historical correction attributes to perception
    src_reobserve = """
def run_policy(mcp, params):
    r1 = mcp.assess_spatial(mode="insufficient")
    if r1.get("status") != "satisfied":
        r2 = mcp.assess_spatial(mode="satisfied")
        if r2.get("status") != "satisfied":
            return {"status": "blocked"}
    mcp.trigger_station(command_id="safe-after-reobs")
    return {"status": "done"}
"""
    ast_c = check_mcp_policy_ast(src_reobserve, allowed_tools=tool_names)
    proxy_c = GovernedMCPProxy(client, tools, journal_path=jpath, run_tag="caseC")
    _, err_c = execute_sandboxed_python(src_reobserve, proxy_c, entry_fn="run_policy", params={})
    assert err_c is None
    assert proxy_c.acted_after_perception_block is False

    attr_pass = classify_mcp_execution(
        ast_check=ast_c, proxy=proxy_c, verify_result={"passed": True, "detail": "ok"}, runtime_error=None
    )
    assert attr_pass["layer"] == "none"

    hist_rev = SpatialChange(
        cursor=SceneCursor("scn-lab", 1, 9),
        affected_entities=("ent-1",),
        cause="historical_correction",
        detail="Late extrinsic update shifted mug_red by 14mm",
    )
    attr_rev = classify_mcp_execution(
        ast_check=ast_c,
        proxy=proxy_c,
        verify_result={"passed": False, "detail": "Missed grasp by 14mm"},
        runtime_error=None,
        revision_change=hist_rev,
    )
    assert attr_rev["layer"] == "perception"
    assert attr_rev["code"] == "historical_correction_invalidated"


def _assess_handler(args: Mapping[str, Any], state: dict[str, int]) -> dict[str, Any]:
    state["obs_calls"] += 1
    mode = str(args.get("mode") or "satisfied")
    if mode == "insufficient":
        return {
            "status": "unsatisfied",
            "cursor": {"scene_id": "scn-lab", "epoch": 1, "revision": 6},
            "observability": {"status": "insufficient", "rank": 4, "reason": "handle_occluded"},
            "detail": "Handle orientation unobservable from static view",
        }
    if mode == "stale":
        return {
            "status": "stale_evidence",
            "cursor": {"scene_id": "scn-lab", "epoch": 1, "revision": 7},
            "detail": "Observation age 2800ms > 1000ms",
        }
    return {
        "status": "satisfied",
        "cursor": {"scene_id": "scn-lab", "epoch": 1, "revision": 8},
        "observability": {"status": "sufficient", "rank": 6},
    }


def _act_handler(args: Mapping[str, Any], state: dict[str, int]) -> dict[str, Any]:
    state["act_calls"] += 1
    return {"receipt": "done", "command_id": str(args.get("command_id") or "")}
