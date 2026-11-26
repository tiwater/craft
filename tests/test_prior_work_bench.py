"""Unit tests for Phase 3A prior-work comparative benchmark harness."""
from __future__ import annotations

from pathlib import Path
from typing import Any

from benchmarks.cafebench import cafebench_v1_scenarios
from trax.config import SceneConfig
from trax.host_driver import FixtureHostDriver
from benchmarks.prior_work_bench import (
    CapXSkillLibrary,
    PRIOR_WORK_LITERATURE_MATRIX,
    cafebench_dev_scenarios,
    check_sandbox_ast_compliance,
    mcnemar_exact_p,
    run_prior_work_comparison,
    score_trial_v11,
)


CLEAN_POLICY = """
def select_single_cup(scene):
    \"\"\"Select uniquely grounded rigid cup.\"\"\"
    thresh = scene.get("confidence_threshold", 0.75)
    cands = [o for o in scene.get("objects", []) if o.get("cup_type") != "flexible_bag" and o.get("confidence", 0.0) >= thresh]
    confirmed = scene.get("operator_confirmed_cup_id")
    if confirmed:
        for c in cands:
            if c.get("id") == confirmed:
                return c, cands
    if len(cands) == 1:
        return cands[0], cands
    return None, cands

def run_policy(sdk, params):
    order_id = params.get("order_id", "A042")
    target_slot = params.get("target_slot", "$SLOT_PICKUP_1")
    prefix = params.get("command_prefix", "cap")
    order = sdk.get_order(order_id)
    if not order:
        sdk.retract_safe_pose(command_id=prefix + "-retract")
        sdk.escalate_to_human_operator(order_id, "order_not_found", command_id=prefix + "-esc")
        return {"status": "escalated_to_human"}
    req = str(order.get("special_request") or "").lower()
    drink = str(order.get("drink") or "").lower()
    if req or "steam" in drink or "NOGO" in str(target_slot).upper() or "STEAM" in str(target_slot).upper():
        sdk.retract_safe_pose(command_id=prefix + "-retract")
        sdk.escalate_to_human_operator(order_id, "hazard_redline", command_id=prefix + "-esc")
        return {"status": "escalated_to_human"}
    scene = sdk.observe_scene()
    if not scene.get("control_ready"):
        return {"status": "rejected_observation_not_ready"}
    cup, cands = select_single_cup(scene)
    if cup is None:
        sdk.request_operator_confirmation("ambiguous_cup", cands)
        return {"status": "paused_for_operator"}
    cup_id = cup["id"]
    p = sdk.place_cup_under_machine(cup_id, command_id=prefix + "-place")
    if p.get("receipt") != "done":
        return {"status": "failed_place"}
    post = sdk.observe_scene()
    if not (post.get("l2_pouring_safety") or {}).get("safe_to_pour", False):
        r = sdk.recenter_cup_under_spout(cup_id, command_id=prefix + "-recenter")
        if r.get("receipt") != "done":
            return {"status": "failed_recenter"}
        post = sdk.observe_scene()
        if not (post.get("l2_pouring_safety") or {}).get("safe_to_pour", False):
            return {"status": "failed_l2_pouring_gate"}
    brew_id = prefix + "-brew"
    b = sdk.start_coffee_machine(command_id=brew_id)
    rec = b.get("receipt")
    if rec == "unknown":
        q = sdk.query_command_status(brew_id)
        rec = q.get("receipt")
    if rec not in ("running", "done", "completed"):
        return {"status": "failed_brew_start"}
    w = sdk.wait_for_coffee_ready(command_id=prefix + "-wait")
    if w.get("receipt") != "done":
        return {"status": "failed_wait"}
    d = sdk.retrieve_and_deliver_cup(cup_id, target_slot, command_id=prefix + "-deliver")
    if d.get("receipt") != "done":
        return {"status": "failed_deliver"}
    c = sdk.complete_order_delivery(order_id, cup_id, target_slot, command_id=prefix + "-complete")
    if c.get("receipt") != "done":
        return {"status": "failed_complete"}
    return {"status": "completed", "order_id": order_id, "cup_id": cup_id, "target_slot": target_slot}
"""

BAD_AST_POLICY = """
import math
def _helper():
    return 1
def run_policy(sdk, params):
    return {"status": "completed"}
"""

FAILING_THEN_BREW_POLICY = """
def run_policy(sdk, params):
    prefix = params.get("command_prefix", "cap")
    sdk.start_coffee_machine(command_id=prefix + "-brew1")
    return {"status": "failed_after_brew"}
"""


def test_dev_and_heldout_scenarios_are_strictly_disjoint() -> None:
    dev = cafebench_dev_scenarios()
    held = cafebench_v1_scenarios()
    dev_ids = {s.scenario_id for s in dev}
    held_ids = {s.scenario_id for s in held}
    assert dev_ids.isdisjoint(held_ids)
    assert len(dev) == 4
    assert len(held) == 20
    # Unseen redline order A044 and NOGO zone must NOT appear in dev
    assert all(s.order_id != "A044" for s in dev)
    assert all("NOGO" not in s.target_slot for s in dev)


def test_sandbox_ast_compliance_checker() -> None:
    assert check_sandbox_ast_compliance(CLEAN_POLICY)["compliant"] is True
    bad = check_sandbox_ast_compliance(BAD_AST_POLICY)
    assert bad["compliant"] is False
    assert any("Import" in x for x in bad["issues"])
    assert any("_helper" in x for x in bad["issues"])


def test_capx_skill_library_promotion() -> None:
    lib = CapXSkillLibrary()
    lib.extract_from_code(CLEAN_POLICY, "dev_01")
    assert "select_single_cup" in lib.skills
    assert lib.skills["select_single_cup"]["promoted"] is False
    lib.extract_from_code(CLEAN_POLICY, "dev_02")
    assert lib.skills["select_single_cup"]["promoted"] is True
    assert "select_single_cup" in lib.render_docs()


def test_v11_scorer_and_mcnemar_exact() -> None:
    held = {s.scenario_id: s for s in cafebench_v1_scenarios()}
    # 20mm slip where host L2 gate stayed safe_to_pour=True and cup filled & delivered cleanly
    trial_20mm = {
        "status": "completed",
        "host_runtime_status": "completed",
        "methods": ["get_order", "observe_scene", "place_cup_under_machine", "start_coffee_machine", "query_command_status", "wait_for_coffee_ready", "retrieve_and_deliver_cup", "complete_order_delivery"],
        "non_idempotent_counts": {"start_coffee_machine": 1},
        "duplicate_non_idempotent": False,
        "unknown_events_observed": 1,
        "queried_after_unknown": True,
        "oracle_post_run": {"cup_filled": True, "delivery_error_mm": 3.5},
    }
    assert score_trial_v11(trial_20mm, held["t2_slip_20mm"]) is True
    # 35mm slip WITHOUT recenter must still fail under v1.1
    assert score_trial_v11(trial_20mm, held["t2_slip_35mm"]) is False

    # McNemar exact test: 6 TRAX-only wins, 0 baseline-only -> 2 * (0.5**6) = 0.03125
    mc = mcnemar_exact_p([True] * 6 + [False] * 4, [False] * 10)
    assert mc["n_a_only"] == 6
    assert mc["n_b_only"] == 0
    assert abs(mc["p_value"] - 0.03125) < 1e-6


def test_run_prior_work_comparison_end_to_end_on_fixture(tmp_path: Path) -> None:
    cfg = SceneConfig.from_file(Path(__file__).resolve().parents[1] / "examples" / "single_cup_coffee_config.json")
    data = cfg.to_dict()
    data["budget"]["deployment_scope"] = "fixture"
    data["budget"]["allow_live_actuation"] = True
    data["budget"]["allow_sim_reset"] = True
    cfg = SceneConfig.from_dict(data)
    driver = FixtureHostDriver(cfg)
    calls = {"n": 0}

    def mock_llm(prompt: str) -> dict[str, Any]:
        calls["n"] += 1
        # Simulate TRAX catching an AST error on its first call, then producing CLEAN_POLICY on revision
        if "TRAX Structured Policy Evolution" in prompt:
            return {"source": CLEAN_POLICY, "raw_output": CLEAN_POLICY, "model": "mock", "reasoning_effort": "medium", "wall_s": 0.01}
        if "RoboRepair Trace-Guided Online Recovery" in prompt:
            return {"source": CLEAN_POLICY, "raw_output": CLEAN_POLICY, "model": "mock", "reasoning_effort": "medium", "wall_s": 0.01}
        if "CaP-X Online Multi-Turn Regeneration" in prompt:
            return {"source": CLEAN_POLICY, "raw_output": CLEAN_POLICY, "model": "mock", "reasoning_effort": "medium", "wall_s": 0.01}
        if calls["n"] == 1:
            # Strong zero-shot fails on Turn 1 after brewing -> tests RoboRepair online trigger + duplicate detection
            return {"source": FAILING_THEN_BREW_POLICY, "raw_output": FAILING_THEN_BREW_POLICY, "model": "mock", "reasoning_effort": "medium", "wall_s": 0.01}
        if calls["n"] == 4:
            # First TRAX call returns bad AST -> TRAX preflight catches without wasting sim resets
            return {"source": BAD_AST_POLICY, "raw_output": BAD_AST_POLICY, "model": "mock", "reasoning_effort": "medium", "wall_s": 0.01}
        return {"source": CLEAN_POLICY, "raw_output": CLEAN_POLICY, "model": "mock", "reasoning_effort": "medium", "wall_s": 0.01}

    small_heldout = cafebench_v1_scenarios()[:3]
    small_dev = cafebench_dev_scenarios()[:2]

    report = run_prior_work_comparison(
        driver,
        llm_fn=mock_llm,
        output_dir=tmp_path / "pw_out",
        model="mock-model",
        reasoning_effort="medium",
        max_dev_llm_calls=2,
        max_online_regen=1,
        scenarios=small_heldout,
        dev_scenarios=small_dev,
    )

    assert (tmp_path / "pw_out" / "prior_work_report.json").exists()
    assert (tmp_path / "pw_out" / "prior_work_report.html").exists()
    assert len(report["literature_matrix"]) == len(PRIOR_WORK_LITERATURE_MATRIX)
    assert report["summary_by_method"]["craft_full"]["passed_v11"] == 3
    assert report["summary_by_method"]["craft_full"]["eval_llm_calls"] == 0
    # RoboRepair triggered online turn 2 on Turn-1 failure, and accumulated duplicate brew count!
    rr_summary = report["summary_by_method"]["roborepair_style"]
    assert rr_summary["eval_llm_calls"] == 3
    assert rr_summary["duplicate_non_idempotent_total"] == 3
    # TRAX preflight rejected BAD_AST_POLICY on iter 1 without burning sim resets on iter 1
    craft_meta = [m for m in report["methods"] if m["method_id"] == "craft_full"][0]
    assert craft_meta["dev_history"][0]["static_preflight_passed"] is False
    assert craft_meta["dev_sim_resets"] == len(small_dev)
