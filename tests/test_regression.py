"""Tests for Failure-to-Testcase permanent regression suite flywheel."""
from __future__ import annotations

import json
from pathlib import Path

from craft.benchmark import builtin_reference_policy_source
from craft.config import SceneConfig
from craft.host_driver import create_host_driver
from craft.regression import RegressionSuite


BROKEN_POLICY = """def run_policy(sdk, params):
    obs = sdk.observe_scene()
    cup_id = obs["objects"][0]["id"]
    sdk.place_cup_under_machine(cup_id, command_id="p")
    r = sdk.start_coffee_machine(command_id="b")
    if r.get("receipt") not in ("running", "done"):
        return {"status": "failed_brew_start"}
    return {"status": "completed"}
"""


def _fixture_config() -> SceneConfig:
    cfg = SceneConfig.from_file("examples/single_cup_coffee_config.json")
    data = cfg.to_dict()
    data["budget"]["deployment_scope"] = "fixture"
    data["budget"]["allow_live_actuation"] = True
    data["budget"]["allow_sim_reset"] = True
    return SceneConfig.from_dict(data)


def test_capture_deduplicates_failures_and_gates_candidate(tmp_path):
    suite = RegressionSuite(tmp_path / "reg.json")

    # Passing trial without unexpected intervention is ignored
    assert suite.capture_failure({"task_passed": True, "human_interventions": 0}) is None

    # Failed slip+timeout trial is captured
    fail_trial = {
        "trial_id": "t-1",
        "task_passed": False,
        "host_runtime_status": "failed_brew_start",
        "fault_injection": True,
        "methods": ["observe_scene", "place_cup_under_machine", "start_coffee_machine"],
        "evidence_source": "simulator",
    }
    ctx = {
        "preset": "default",
        "fault_inject_slip_xy_m": [0.038, -0.012],
        "simulate_timeout": True,
        "expect_status": "completed",
    }
    c1 = suite.capture_failure(fail_trial, scenario_context=ctx)
    assert c1 is not None
    assert c1.hit_count == 1
    assert c1.attribution_layer == "policy"

    # Re-capturing identical failure deduplicates and increments hit_count
    c2 = suite.capture_failure(fail_trial, scenario_context=ctx)
    assert c2 is not None
    assert c2.case_id == c1.case_id
    assert c2.hit_count == 2
    assert len(suite.cases) == 1

    # Persists across reload
    reloaded = RegressionSuite(tmp_path / "reg.json")
    assert len(reloaded.cases) == 1

    # Broken policy is blocked by regression gate; fixed reference policy passes
    driver = create_host_driver("fixture", _fixture_config())
    bad_gate = reloaded.evaluate_regression_gate(driver, BROKEN_POLICY, policy_id="broken")
    assert bad_gate["passed"] is False
    assert bad_gate["failed_case_ids"] == [c1.case_id]

    good_gate = reloaded.evaluate_regression_gate(driver, builtin_reference_policy_source(), policy_id="fixed")
    assert good_gate["passed"] is True
    assert good_gate["passed_cases"] == 1


def test_capture_from_real_archived_benchmark(tmp_path):
    bench_path = (
        Path(__file__).resolve().parents[1]
        / "experiments"
        / "coffee-closed-loop-20261120"
        / "benchmark"
        / "benchmark_results.json"
    )
    report = json.loads(bench_path.read_text(encoding="utf-8"))
    suite = RegressionSuite(tmp_path / "real_reg.json")
    captured = suite.capture_from_benchmark_report(report)
    assert len(captured) == 1
    assert captured[0].attribution_layer == "control"
    assert captured[0].scenario.fault_inject_slip_xy_m == (0.038, -0.012)
    assert captured[0].scenario.simulate_timeout is True
