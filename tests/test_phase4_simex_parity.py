"""Phase 4 (CRAFT v3) verification tests:
- Zero task-hint leakage audit in Robosuite attribution & hypotheses
- Public-SDK Active Physical Probing (grasp-offset scan & articulated sweep)
- Closed-loop simulator parameter calibration in RepairScreener
- True Sim-to-Sim 5-trial adaptation benchmark
- Verified executable Skill Toolbox (AST validation, prelude injection, contract-drift invalidation)
- Multi-appliance Coffee + Bagel/Oven concurrent operations (interlocks, dual crash recovery, throughput)
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import pytest

from craft.benchmark import builtin_reference_policy_source
from craft.combo_bench import (
    ComboWorkstationHost,
    builtin_pipelined_combo_policy_source,
    builtin_sequential_combo_policy_source,
    run_combo_operations_benchmark,
    run_combo_trial,
)
from craft.config import SceneConfig
from craft.host_driver import create_host_driver
from craft.journal import Journal
from craft.probing import (
    analyze_policy_subroutines,
    build_articulated_sweep_probe_source,
    build_grasp_offset_scan_probe_source,
    parse_llm_hypotheses_json,
    parse_probe_telemetry,
)
from craft.regression import RegressionSuite
from craft.repair_screener import RepairScreener, calibrate_simulator_alignment
from craft.sim2sim_bench import run_sim2sim_adaptation_benchmark
from craft.skills import (
    SkillExperience,
    SkillExperienceStore,
    inject_toolbox_into_policy,
    validate_skill_code_snippet,
)


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


def _fixture_cfg() -> SceneConfig:
    cfg = SceneConfig.from_file("examples/single_cup_coffee_config.json")
    data = cfg.to_dict()
    data["budget"]["deployment_scope"] = "fixture"
    data["budget"]["allow_live_actuation"] = True
    data["budget"]["allow_sim_reset"] = True
    return SceneConfig.from_dict(data)


def test_zero_task_hint_leakage_in_robosuite_attribution_and_hypotheses() -> None:
    from craft.robosuite_bench import _robo_craft_hypotheses, attribute_robosuite_failure

    # Verify Door & NutAssemblySquare hypotheses contain ZERO hardcoded oracle physics numbers
    door_fail = {
        "success": False,
        "min_eef_to_obj_m": 0.02,
        "max_handle_qpos": 0.4,
        "max_hinge_qpos": 0.0,
        "final_state": {"objects": {"handle": {"pos": [0, 0, 1]}}, "handle_qpos": 0.4, "hinge_qpos": 0.0},
    }
    door_fail["attribution"] = attribute_robosuite_failure(door_fail, "Door")
    assert door_fail["attribution"]["code"] == "latch_not_unlatched"
    assert "1.45" not in door_fail["attribution"]["prescription"]
    assert "1.55" not in door_fail["attribution"]["prescription"]

    nut_fail = {
        "success": False,
        "min_eef_to_obj_m": 0.02,
        "ever_closed_gripper": True,
        "closed_gripper_width_max": 0.002,
        "obj_lift_delta_m": 0.0,
        "final_state": {"objects": {"nut": {"pos": [0, 0, 0.85]}}, "goal_pos": [0.2, 0.1, 0.85]},
    }
    nut_fail["attribution"] = attribute_robosuite_failure(nut_fail, "NutAssemblySquare")
    assert nut_fail["attribution"]["code"] == "grasp_missed_or_slip"
    assert "0.055" not in nut_fail["attribution"]["prescription"]

    for task_name, run in (("Door", door_fail), ("NutAssemblySquare", nut_fail)):
        hyps = _robo_craft_hypotheses(task_name, [run])
        joined = " ".join(f"{t} {s}" for t, s in hyps)
        for forbidden in ("1.45", "1.55", "0.055", "0.045", "goal_pos[2] + 0.015"):
            assert forbidden not in joined, f"Hardcoded physics leak {forbidden!r} found in {task_name} hypotheses!"


def test_probing_parsers_and_llm_hypothesis_extraction() -> None:
    rep = parse_probe_telemetry(
        "NutAssemblySquare",
        [
            {
                "probe": "grasp_offset_scan",
                "target": "nut",
                "samples": [
                    {"local_offset_m": 0.0, "closed_width_m": 0.001, "lift_delta_m": 0.0},
                    {"local_offset_m": 0.055, "closed_width_m": 0.022, "lift_delta_m": 0.095},
                ],
            }
        ],
        evidence_source="simulator",
        sim_resets_used=1,
        wall_s=1.2,
    )
    assert rep.sim_resets_used == 1
    assert rep.findings[0].measurements["best_local_offset_m"] == 0.055
    assert "Empirical Active-Probe Findings" in rep.render_prompt_block()

    raw_json = """```json
    [
      {"hypothesis_id": "h1", "title": "Offset grasp", "diagnosis": "missed center", "proposed_strategy": "grasp at +0.055m"}
    ]
    ```"""
    parsed = parse_llm_hypotheses_json(raw_json, max_hypotheses=3)
    assert len(parsed) == 1
    assert parsed[0]["hypothesis_id"] == "h1"


def test_robosuite_real_active_probing_public_sdk() -> None:
    os.environ.setdefault("MUJOCO_GL", "egl")
    pytest.importorskip("robosuite")
    from craft.robosuite_bench import check_robosuite_ast_compliance, run_robosuite_active_probe

    assert check_robosuite_ast_compliance(build_grasp_offset_scan_probe_source())["compliant"] is True
    assert check_robosuite_ast_compliance(build_articulated_sweep_probe_source())["compliant"] is True

    # Run 1 real public-SDK articulated sweep probe on Door (0 hardcoded answers)
    door_probe = run_robosuite_active_probe(
        "Door",
        seed=101,
        dev_runs=[
            {
                "success": False,
                "attribution": {"code": "latch_not_unlatched"},
                "final_state": {"objects": {"handle": {"pos": [0.1, -0.3, 1.0]}}, "handle_qpos": 0.0},
            }
        ],
        horizon=320,
    )
    assert door_probe is not None
    assert door_probe.sim_resets_used == 1
    assert len(door_probe.findings) == 1
    assert door_probe.findings[0].probe_kind == "articulated_sweep"


def test_repair_screener_closed_loop_simulator_calibration(tmp_path: Path) -> None:
    cfg = _fixture_cfg()
    driver = create_host_driver("fixture", cfg)
    suite = RegressionSuite(tmp_path / "reg.json")

    # Capture a deployment failure WITHOUT recording the hidden slip/timeout in scenario_context
    case = suite.capture_failure(
        {
            "trial_id": "deploy-slip-unaligned",
            "task_passed": False,
            "host_runtime_status": "failed_brew_start",
            "fault_injection": True,
            "methods": ["observe_scene", "place_cup_under_machine", "start_coffee_machine"],
            "evidence_source": "physical_robot",
        },
        scenario_context={"preset": "default", "expect_status": "completed"},
    )
    assert case is not None and case.attribution_layer == "policy"

    # Without calibration, nominal fixture passes -> not_reproduced_in_sim
    screener = RepairScreener(tmp_path / "ws_no_cal", cfg, driver, regression_suite=suite)
    rep_plain = screener.screen_repair(
        baseline_source=BASELINE_NO_RECENTER,
        case=case,
        candidate_sources={"fix": builtin_reference_policy_source()},
        enable_sim_calibration=False,
    )
    assert rep_plain["status"] == "not_reproduced_in_sim"

    # With closed-loop calibration enabled, sandbox searches disturbances until failure reproduces!
    cal = calibrate_simulator_alignment(driver, BASELINE_NO_RECENTER, case, max_calibration_trials=4)
    assert cal.reproduced is True
    assert cal.alignment_method == "calibrated_sim_search"
    assert cal.calibration_trials_used >= 1
    assert cal.calibrated_scenario

    screener2 = RepairScreener(tmp_path / "ws_cal", cfg, driver, regression_suite=suite)
    rep_cal = screener2.screen_repair(
        baseline_source=BASELINE_NO_RECENTER,
        case=case,
        candidate_sources={"fix": builtin_reference_policy_source()},
        enable_sim_calibration=True,
        max_calibration_trials=4,
    )
    assert rep_cal["status"] == "completed_with_recommendation"
    assert rep_cal["alignment"]["alignment_method"] == "calibrated_sim_search"
    assert rep_cal["approved"] is False
    assert rep_cal["requires_independent_acceptance"] is True


def test_sim2sim_5_trial_adaptation_benchmark_on_fixture(tmp_path: Path) -> None:
    cfg = _fixture_cfg()
    sandbox = create_host_driver("fixture", cfg)
    target = create_host_driver("fixture", cfg)

    def mock_llm(prompt: str) -> dict[str, Any]:
        return {"source": builtin_reference_policy_source(), "raw_output": builtin_reference_policy_source()}

    rep = run_sim2sim_adaptation_benchmark(
        sandbox,
        target,
        cfg,
        llm_fn=mock_llm,
        output_dir=tmp_path / "s2s",
        initial_policy_source=BASELINE_NO_RECENTER,
        adaptation_scenarios=[
            {
                "scenario_id": "a1",
                "label": "Slip + Timeout",
                "preset": "default",
                "fault_inject_slip_xy_m": [0.036, -0.012],
                "simulate_timeout": True,
                "order_id": "A042",
                "target_slot": "$SLOT_PICKUP_1",
                "expect_status": "completed",
            }
        ],
        heldout_scenarios=[
            {
                "scenario_id": "h1",
                "label": "Heldout Slip + Timeout",
                "preset": "default",
                "fault_inject_slip_xy_m": [0.035, -0.010],
                "simulate_timeout": True,
                "order_id": "A042",
                "target_slot": "$SLOT_PICKUP_1",
                "expect_status": "completed",
            },
            {
                "scenario_id": "h2",
                "label": "Heldout Ambiguity",
                "preset": "sim_dual_cup",
                "order_id": "A042",
                "target_slot": "$SLOT_PICKUP_1",
                "expect_status": "paused_for_operator",
            },
        ],
        model="mock",
    )
    assert (tmp_path / "s2s" / "sim2sim_report.json").is_file()
    assert (tmp_path / "s2s" / "sim2sim_report.html").is_file()
    assert rep["summary_by_method"]["craft_full"]["passed"] == 6  # 2 target + 4 recorded shadow
    assert rep["summary_by_method"]["craft_full"]["adapt_target_trials"] <= 5
    assert rep["summary_by_method"]["craft_full"]["adapt_calibration_trials"] >= 1
    assert rep["summary_by_method"]["zeroshot_sim2sim"]["passed"] < rep["summary_by_method"]["craft_full"]["passed"]


def test_verified_executable_skill_toolbox_safety_injection_and_drift(tmp_path: Path) -> None:
    good_code = """def reconcile_receipt(sdk, cmd_id, resp):
    if resp.get("receipt") == "unknown":
        return sdk.query_command_status(cmd_id)
    return resp
"""
    chk_ok = validate_skill_code_snippet(good_code)
    assert chk_ok["valid"] is True
    assert chk_ok["entry_functions"] == ["reconcile_receipt"]

    # Unsafe snippets (imports, exec, top-level run_policy) must be rejected
    assert validate_skill_code_snippet("import os\ndef f(): return 1")["valid"] is False
    assert validate_skill_code_snippet("def run_policy(sdk, p): return {}")["valid"] is False
    assert validate_skill_code_snippet("def f(): return eval('1')")["valid"] is False

    store = SkillExperienceStore(tmp_path / "toolbox.json")
    sk = SkillExperience(
        skill_id="sk-reconcile",
        title="Query-first receipt reconciler",
        guidance="Call reconcile_receipt(sdk, cmd_id, resp) on any unknown receipt.",
        task_name="single_cup_coffee",
        required_methods=("query_command_status",),
        sdk_contract_sha256="contract-v1",
        evidence_source="simulator",
        verification_report_id="acc-1",
        verified=True,
        verified_at_utc="2026-11-24T00:00:00+00:00",
        code_snippet=good_code,
        preconditions=("command_id_committed",),
        postconditions=("no_duplicate_dispatch",),
    )
    store.add(sk)

    # Cross-task query & safe prelude injection
    applicable = store.query_applicable(
        task_name="combo_coffee_bagel",
        available_methods=("query_command_status", "start_oven_heat"),
        sdk_contract_sha256="contract-v1",
        allow_cross_task=True,
    )
    assert len(applicable) == 1
    assert applicable[0].entry_functions == ("reconcile_receipt",)

    client_policy = """def run_policy(sdk, params):
    r = sdk.start_coffee_machine(command_id="b")
    return reconcile_receipt(sdk, "b", r)
"""
    injected = inject_toolbox_into_policy(client_policy, applicable)
    assert "def reconcile_receipt" in injected

    # Host SDK contract drift must automatically invalidate bound skills
    inv = store.invalidate_on_contract_drift("contract-v2-changed")
    assert inv == ["sk-reconcile"]
    assert store.list_verified() == []


def test_combo_multi_appliance_interlocks_crash_recovery_and_throughput(tmp_path: Path) -> None:
    # 1. Verify thermal interlock rejects opening oven door while heating
    host = ComboWorkstationHost()
    unsafe_open_while_heating = """def run_policy(sdk, params):
    sdk.open_oven_door(command_id="o1")
    sdk.load_bagel("obj-bagel-1", command_id="l1")
    sdk.close_oven_door(command_id="c1")
    sdk.start_oven_heat(command_id="h1")
    r = sdk.open_oven_door(command_id="o_bad")
    return {"status": "completed" if r.get("receipt") == "done" else "interlock_blocked"}
"""
    res_bad = run_combo_trial(host, unsafe_open_while_heating, {"scenario_id": "interlock"}, run_id="t-bad")
    assert res_bad["interlock_violations"] >= 1
    assert res_bad["task_passed"] is False

    # 2. Verify full CafeBench v3 combo benchmark:
    #    - CRAFT pipelined+journaled achieves 12/12, 0 duplicates, 0 interlock violations
    #    - Stateless baselines fail mid-brew & mid-heat crash scenarios with duplicate non-idempotent actuations
    #    - Pipelined scheduling cuts cycle time significantly vs sequential (+40% throughput!)
    rep = run_combo_operations_benchmark(tmp_path / "combo_out")
    seq = rep["summary_by_method"]["sequential_stateless"]
    pipe_no_j = rep["summary_by_method"]["pipelined_stateless"]
    craft = rep["summary_by_method"]["craft_pipelined_journaled"]

    assert craft["passed"] == 12
    assert craft["total_trials"] == 12
    assert craft["duplicate_non_idempotent_orders"] == 0
    assert craft["interlock_violations"] == 0

    assert seq["passed"] < craft["passed"]
    assert seq["duplicate_non_idempotent_orders"] >= 2
    assert pipe_no_j["duplicate_non_idempotent_orders"] >= 2
    assert craft["mean_completed_cycle_s"] < seq["mean_completed_cycle_s"] * 0.75
    assert craft["throughput_orders_per_hour"] > seq["throughput_orders_per_hour"] * 1.35


def test_selective_subroutine_analysis_and_adaptive_polish_escape() -> None:
    from craft.robosuite_bench import _robo_craft_hypotheses, _robo_craft_prompt

    sample_policy = """def grasp_obj(sdk, pos):
    sdk.move_to(pos, gripper="open")
    sdk.set_gripper("close")

def seat_obj(sdk, goal):
    sdk.move_to(goal, gripper="close")
    sdk.set_gripper("open")

def run_policy(sdk):
    st = sdk.get_state()
    grasp_obj(sdk, st["objects"]["nut"]["pos"])
    seat_obj(sdk, st["goal_pos"])
    return {"status": "done"}
"""
    sub = analyze_policy_subroutines(sample_policy, ["insertion_too_high_or_step_exhausted"])
    assert sub["repair_scope"] == "downstream_alignment_and_seating"
    names = [s["name"] for s in sub["subroutines"]]
    assert names == ["grasp_obj", "seat_obj", "run_policy"]

    partial_runs = [
        {"seed": 101, "success": True, "steps_used": 250, "max_handle_qpos": 0.0, "max_hinge_qpos": 0.0},
        {
            "seed": 102,
            "success": False,
            "steps_used": 380,
            "min_eef_to_obj_m": 0.01,
            "ever_closed_gripper": True,
            "closed_gripper_width_max": 0.02,
            "obj_lift_delta_m": 0.12,
            "min_obj_to_goal_m": 0.005,
            "max_handle_qpos": 0.0,
            "max_hinge_qpos": 0.0,
            "final_state": {"eef_pos": [0.2, 0.1, 0.95], "objects": {"nut": {"pos": [0.2, 0.1, 0.93]}}, "goal_pos": [0.2, 0.1, 0.85]},
            "attribution": {"layer": "control", "code": "insertion_too_high_or_step_exhausted", "prescription": "Descend cleanly"},
        },
    ]
    h_fresh = _robo_craft_hypotheses("NutAssemblySquare", partial_runs, stagnant_iters=0)
    assert h_fresh[0][0].startswith("H-Subroutine1")

    # After 1 stagnant iteration, must escape narrow polish and unlock H-Place* mechanistic hypotheses (with 0 hardcoded leaks)
    h_escaped = _robo_craft_hypotheses("NutAssemblySquare", partial_runs, stagnant_iters=1)
    assert h_escaped[0][0].startswith("H-Place1")
    joined = " ".join(f"{t} {s}" for t, s in h_escaped)
    for forbidden in ("0.055", "1.45", "goal_pos[2] + 0.015"):
        assert forbidden not in joined

    prompt = _robo_craft_prompt(
        "NutAssemblySquare",
        sample_policy,
        partial_runs,
        [],
        [102],
        4,
        hypothesis_title=h_escaped[0][0],
        hypothesis_strategy=h_escaped[0][1],
        failed_attempts=[
            {"iter": 3, "hypothesis": "H-Subroutine1", "dev_passed": 1, "dev_total": 2, "progress_score": 0.05, "attributions": ["insertion_too_high_or_step_exhausted"]}
        ],
    )
    assert "Selective Subroutine Scope (downstream_alignment_and_seating)" in prompt
    assert "Recent Non-Improving Attempts" in prompt

