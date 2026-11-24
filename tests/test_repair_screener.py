"""Behavioral tests for replay-aligned multi-hypothesis repair screening,
capability-bank replay in DevelopmentLoop, and structured SkillExperience boundaries.
"""
from __future__ import annotations

import json
import threading
import urllib.request

import pytest

from craft.benchmark import builtin_reference_policy_source
from craft.config import SceneConfig
from craft.dev_loop import DevelopmentLoop, TrialRecord
from craft.engine_api import CraftEngineService, create_engine_server
from craft.host_driver import create_host_driver
from craft.regression import RegressionSuite
from craft.repair_screener import RepairHypothesis, RepairScreener
from craft.skills import SkillExperience, SkillExperienceStore
from craft.teaching import TaskStep, build_task_specification


# Fails on slip+timeout (no recenter, no query_command_status)
BASELINE_NO_RECENTER = """def run_policy(sdk, params):
    obs = sdk.observe_scene()
    cup_id = obs["objects"][0]["id"]
    sdk.place_cup_under_machine(cup_id, command_id="p")
    r = sdk.start_coffee_machine(command_id="b")
    if r.get("receipt") not in ("running", "done"):
        return {"status": "failed_brew_start"}
    sdk.wait_for_coffee_ready(command_id="w")
    sdk.retrieve_and_deliver_cup(cup_id, params["target_slot"], command_id="d")
    sdk.complete_order_delivery(params["order_id"], cup_id, params["target_slot"], command_id="c")
    return {"status": "completed"}
"""

# Unsafe candidate: blindly retries start_coffee_machine on unknown receipt
UNSAFE_DUPLICATE_BREW = """def run_policy(sdk, params):
    obs = sdk.observe_scene()
    cup_id = obs["objects"][0]["id"]
    sdk.place_cup_under_machine(cup_id, command_id="p")
    sdk.recenter_cup_under_spout(cup_id, command_id="rc")
    r = sdk.start_coffee_machine(command_id="b1")
    if r.get("receipt") == "unknown":
        sdk.start_coffee_machine(command_id="b2")
    sdk.wait_for_coffee_ready(command_id="w")
    sdk.retrieve_and_deliver_cup(cup_id, params["target_slot"], command_id="d")
    sdk.complete_order_delivery(params["order_id"], cup_id, params["target_slot"], command_id="c")
    return {"status": "completed"}
"""

# Regressing candidate: fixes slip+timeout on single cup, but ignores ambiguous-cup pause
REGRESSING_NO_AMBIGUITY_GUARD = """def run_policy(sdk, params):
    obs = sdk.observe_scene()
    cup_id = obs["objects"][0]["id"]
    sdk.place_cup_under_machine(cup_id, command_id="p")
    obs2 = sdk.observe_scene()
    if not (obs2.get("l2_pouring_safety") or {}).get("safe_to_pour", True):
        sdk.recenter_cup_under_spout(cup_id, command_id="rc")
    r = sdk.start_coffee_machine(command_id="b")
    if r.get("receipt") == "unknown":
        sdk.query_command_status("b")
    sdk.wait_for_coffee_ready(command_id="w")
    sdk.retrieve_and_deliver_cup(cup_id, params["target_slot"], command_id="d")
    sdk.complete_order_delivery(params["order_id"], cup_id, params["target_slot"], command_id="c")
    return {"status": "completed"}
"""


def _fixture_config(*, max_model_calls: int = 6, max_trials: int = 30) -> SceneConfig:
    cfg = SceneConfig.from_file("examples/single_cup_coffee_config.json")
    data = cfg.to_dict()
    data["budget"]["deployment_scope"] = "fixture"
    data["budget"]["allow_live_actuation"] = True
    data["budget"]["allow_sim_reset"] = True
    data["budget"]["max_model_calls"] = max_model_calls
    data["budget"]["max_trials"] = max_trials
    return SceneConfig.from_dict(data)


def _capture_slip_and_ambiguity_cases(suite: RegressionSuite):
    slip_case = suite.capture_failure(
        {
            "trial_id": "t-slip",
            "task_passed": False,
            "host_runtime_status": "failed_brew_start",
            "fault_injection": True,
            "methods": ["observe_scene", "place_cup_under_machine", "start_coffee_machine"],
            "evidence_source": "physical_robot",
        },
        scenario_context={
            "label": "Slip + timeout on default",
            "preset": "default",
            "fault_inject_slip_xy_m": [0.038, -0.012],
            "simulate_timeout": True,
            "expect_status": "completed",
        },
    )
    amb_case = suite.capture_failure(
        {
            "trial_id": "t-amb",
            "task_passed": False,
            "host_runtime_status": "completed",
            "detail": "Brewed despite two ambiguous red cups",
            "evidence_source": "simulator",
        },
        scenario_context={
            "label": "Dual low-confidence cups must pause",
            "preset": "sim_dual_cup",
            "expect_status": "paused_for_operator",
        },
    )
    assert slip_case is not None and amb_case is not None
    return slip_case, amb_case


def test_repair_screener_replays_filters_unsafe_and_regressing_candidates(tmp_path):
    cfg = _fixture_config()
    driver = create_host_driver("fixture", cfg)
    suite = RegressionSuite(tmp_path / "reg.json")
    slip_case, amb_case = _capture_slip_and_ambiguity_cases(suite)

    hyps = [
        RepairHypothesis("h_syntax", "Broken syntax", "diag", "strat"),
        RepairHypothesis("h_dup", "Blind retry brew", "diag", "strat"),
        RepairHypothesis("h_reg", "Fixes slip only", "diag", "strat"),
        RepairHypothesis("h_good", "Full safe fix", "diag", "strat"),
    ]
    sources = {
        "h_syntax": "def run_policy(sdk, params:\n    return {}",
        "h_dup": UNSAFE_DUPLICATE_BREW,
        "h_reg": REGRESSING_NO_AMBIGUITY_GUARD,
        "h_good": builtin_reference_policy_source(),
    }

    screener = RepairScreener(tmp_path / "ws", cfg, driver, regression_suite=suite)
    report = screener.screen_repair(
        baseline_source=BASELINE_NO_RECENTER,
        case=slip_case,
        hypotheses=hyps,
        candidate_sources=sources,
        max_candidates=4,
    )

    assert report["status"] == "completed_with_recommendation"
    assert report["approved"] is False
    assert report["requires_independent_acceptance"] is True
    assert report["evidence_source"] == "fixture"
    assert report["alignment"]["reproduced"] is True
    assert report["alignment"]["reproduction_status"] == "reproduced_exact"

    by_hyp = {c["hypothesis_id"]: c for c in report["candidates"]}
    assert by_hyp["h_syntax"]["syntax_valid"] is False
    assert "syntax_error" in by_hyp["h_syntax"]["rejection_reasons"]

    assert by_hyp["h_dup"]["duplicate_non_idempotent"] is True
    assert by_hyp["h_dup"]["eligible_for_acceptance"] is False
    assert "duplicate_non_idempotent" in by_hyp["h_dup"]["rejection_reasons"]

    assert by_hyp["h_reg"]["target_fixed"] is True
    assert amb_case.case_id in by_hyp["h_reg"]["regression_failed_ids"]
    assert by_hyp["h_reg"]["eligible_for_acceptance"] is False
    assert "regression_failed" in by_hyp["h_reg"]["rejection_reasons"]

    assert by_hyp["h_good"]["eligible_for_acceptance"] is True
    assert by_hyp["h_good"]["approved"] is False
    assert report["recommended_candidate_id"] == by_hyp["h_good"]["candidate_id"]

    # Saved source can be retrieved cleanly
    loaded = screener.get_candidate_source(report["screening_id"], report["recommended_candidate_id"])
    assert "recenter_cup_under_spout" in loaded


def test_repair_screener_blocks_non_policy_and_unreproduced_failures(tmp_path):
    cfg = _fixture_config()
    driver = create_host_driver("fixture", cfg)
    suite = RegressionSuite(tmp_path / "reg.json")

    # 1. Control-layer failure (empty_grasp) must not be auto-patched
    ctrl_case = suite.capture_failure(
        {
            "trial_id": "t-ctrl",
            "task_passed": False,
            "host_runtime_status": "empty_grasp",
            "evidence_source": "physical_robot",
        },
        scenario_context={"preset": "default"},
    )
    assert ctrl_case is not None and ctrl_case.attribution_layer == "control"

    calls = {"n": 0}

    def fail_if_called(c, prompt):
        calls["n"] += 1
        return {"source": builtin_reference_policy_source()}

    screener = RepairScreener(tmp_path / "ws", cfg, driver, regression_suite=suite)
    rep_ctrl = screener.screen_repair(
        baseline_source=BASELINE_NO_RECENTER,
        case=ctrl_case,
        generator_fn=fail_if_called,
    )
    assert rep_ctrl["status"] == "blocked_non_policy_owner"
    assert calls["n"] == 0

    # 2. Policy failure that passes in simulation (not reproduced) stops honestly
    unrep_case = suite.capture_failure(
        {
            "trial_id": "t-unrep",
            "task_passed": False,
            "host_runtime_status": "failed_brew_start",
            "fault_injection": True,
            "methods": ["place_cup_under_machine", "start_coffee_machine"],
            "evidence_source": "physical_robot",
        },
        scenario_context={"preset": "default"},  # no slip/timeout in sim!
    )
    assert unrep_case is not None and unrep_case.attribution_layer == "policy"

    rep_unrep = screener.screen_repair(
        baseline_source=builtin_reference_policy_source(),
        case=unrep_case,
        generator_fn=fail_if_called,
    )
    assert rep_unrep["status"] == "not_reproduced_in_sim"
    assert rep_unrep["alignment"]["reproduced"] is False
    assert calls["n"] == 0


def test_repair_screener_budget_resume_and_identity_drift(tmp_path):
    cfg = _fixture_config(max_model_calls=1, max_trials=20)
    driver = create_host_driver("fixture", cfg)
    suite = RegressionSuite(tmp_path / "reg.json")
    slip_case, _ = _capture_slip_and_ambiguity_cases(suite)

    generated: list[str] = []

    def fake_gen(c, prompt):
        generated.append(prompt)
        src = REGRESSING_NO_AMBIGUITY_GUARD if len(generated) == 1 else builtin_reference_policy_source()
        return {"source": src}

    screener = RepairScreener(tmp_path / "ws", cfg, driver, regression_suite=suite)
    r1 = screener.screen_repair(
        baseline_source=BASELINE_NO_RECENTER,
        case=slip_case,
        screening_id="scr-resume-test",
        generator_fn=fake_gen,
        max_candidates=2,
        max_model_calls=1,
    )
    assert r1["status"] == "budget_exhausted"
    assert len(r1["candidates"]) == 1
    assert len(generated) == 1

    # Resume same screening_id with higher model budget -> skips first hypothesis, completes second
    r2 = screener.screen_repair(
        baseline_source=BASELINE_NO_RECENTER,
        case=slip_case,
        screening_id="scr-resume-test",
        generator_fn=fake_gen,
        max_candidates=2,
        max_model_calls=3,
    )
    assert r2["status"] == "completed_with_recommendation"
    assert len(r2["candidates"]) == 2
    assert len(generated) == 2  # only 1 additional call on resume

    # Identity drift: changed baseline_source must fail to reuse screening_id
    with pytest.raises(ValueError, match="identity drift"):
        screener.screen_repair(
            baseline_source=builtin_reference_policy_source(),
            case=slip_case,
            screening_id="scr-resume-test",
            generator_fn=fake_gen,
        )


def test_dev_loop_capability_bank_regression_replay(tmp_path):
    cfg = _fixture_config(max_model_calls=3, max_trials=10)
    driver = create_host_driver("fixture", cfg)
    suite = RegressionSuite(tmp_path / "reg.json")
    slip_case, _ = _capture_slip_and_ambiguity_cases(suite)

    steps = (
        TaskStep("s1", "Observe scene", "observe_scene", completion_evidence=("cup_visible",)),
        TaskStep("s2", "Brew once", "start_coffee_machine", completion_evidence=("brew_started",), non_idempotent=True),
    )
    spec = build_task_specification(cfg, "Make one coffee", steps=steps)

    prompts: list[str] = []
    drafts = [BASELINE_NO_RECENTER, builtin_reference_policy_source()]

    def gen(c, prompt):
        prompts.append(prompt)
        return {
            "source": drafts[len(prompts) - 1],
            "provider": "codex_cli",
            "model": "fixture-model",
            "reasoning_effort": "medium",
            "evidence_source": "fixture",
        }

    def eval_normal(cand_id, source, condition):
        return TrialRecord(
            trial_id=f"normal-{cand_id}",
            candidate_id=cand_id,
            condition=condition,
            evidence_source="fixture",
            status="completed",
            passed=True,
            wall_s=0.5,
        )

    loop = DevelopmentLoop.create_or_resume(tmp_path / "dev_ws", cfg, spec)
    # Iteration 1 passes normal condition, but fails accumulated regression cases -> must NOT complete!
    r1 = loop.run_iteration(
        generator_fn=gen,
        evaluator_fn=eval_normal,
        regression_suite=suite,
        regression_driver=driver,
    )
    assert r1["status"] == "running"
    assert any(t["condition"] == f"regression:{slip_case.case_id}" and not t["passed"] for t in r1["trials"])

    # Iteration 2 sees regression failure in prompt and fixes all regression cases -> completes!
    r2 = loop.run_iteration(
        generator_fn=gen,
        evaluator_fn=eval_normal,
        regression_suite=suite,
        regression_driver=driver,
    )
    assert slip_case.case_id in prompts[1]
    assert r2["status"] == "completed"
    assert loop.state["active_candidate_id"] == "cand_002"


def test_skill_experience_measured_limits_and_backward_compat(tmp_path):
    store_path = tmp_path / "skills.json"
    # Simulate legacy JSON entry without new fields
    legacy = {
        "sk-legacy": {
            "skill_id": "sk-legacy",
            "title": "Legacy skill",
            "guidance": "Check safety",
            "task_name": "single_cup_coffee",
            "required_methods": ["observe_scene"],
            "sdk_contract_sha256": "abc",
            "evidence_source": "simulator",
            "verification_report_id": "rep-1",
            "verified": True,
            "verified_at_utc": "2026-11-20T00:00:00+00:00",
        }
    }
    store_path.write_text(json.dumps(legacy), encoding="utf-8")

    store = SkillExperienceStore(store_path)
    store.add(
        SkillExperience(
            skill_id="sk-recenter",
            title="Recenter under slip",
            guidance="Call recenter_cup_under_spout when safe_to_pour is False.",
            task_name="single_cup_coffee",
            required_methods=("observe_scene", "recenter_cup_under_spout"),
            sdk_contract_sha256="abc",
            evidence_source="simulator",
            verification_report_id="rep-2",
            verified=True,
            verified_at_utc="2026-11-22T00:00:00+00:00",
            measured_limits={"max_radial_offset_mm": 38.0, "max_delivery_error_mm": 15.0},
            known_failure_modes=("empty_grasp_beyond_45mm_offset",),
            regression_case_ids=("reg_001_deadbe",),
        )
    )

    reloaded = SkillExperienceStore(store_path)
    matched = reloaded.query_applicable(
        task_name="single_cup_coffee",
        available_methods=("observe_scene", "recenter_cup_under_spout"),
        sdk_contract_sha256="abc",
    )
    by_id = {s.skill_id: s for s in matched}
    assert by_id["sk-legacy"].measured_limits == {}
    assert by_id["sk-legacy"].known_failure_modes == ()
    assert by_id["sk-recenter"].measured_limits["max_radial_offset_mm"] == 38.0
    assert by_id["sk-recenter"].known_failure_modes == ("empty_grasp_beyond_45mm_offset",)
    assert by_id["sk-recenter"].regression_case_ids == ("reg_001_deadbe",)


def test_engine_api_repair_screen_endpoint(tmp_path):
    cfg = _fixture_config()
    drv = create_host_driver("fixture", cfg)
    svc = CraftEngineService(cfg, drv, tmp_path / "eng_ws")
    slip_case, _ = _capture_slip_and_ambiguity_cases(svc.regression)

    server = create_engine_server(svc, host="127.0.0.1", port=0)
    port = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        req = urllib.request.Request(
            f"http://127.0.0.1:{port}/v1/repair/screen",
            data=json.dumps(
                {
                    "case_id": slip_case.case_id,
                    "baseline_source": BASELINE_NO_RECENTER,
                    "candidate_sources": {
                        "bad_dup": UNSAFE_DUPLICATE_BREW,
                        "good_fix": builtin_reference_policy_source(),
                    },
                }
            ).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with opener.open(req, timeout=5) as resp:
            assert resp.status == 200
            data = json.loads(resp.read().decode("utf-8"))
            assert data["status"] == "completed_with_recommendation"
            assert data["approved"] is False
            assert data["requires_independent_acceptance"] is True
            assert data["recommended_candidate_id"].endswith("good_fix")
    finally:
        server.shutdown()
        server.server_close()
