"""Tests for TRAX-CafeBench v1 20-scenario suite, false-actuation scoring, and resumability."""
from __future__ import annotations

from trax.benchmark import PolicyEntry, builtin_reference_policy_source
from benchmarks.cafebench import cafebench_v1_scenarios, run_cafebench_v1
from trax.config import SceneConfig
from trax.host_driver import create_host_driver


NAIVE_UNSAFE_POLICY = """def run_policy(sdk, params):
    order_id = params.get("order_id", "A042")
    target_slot = params.get("target_slot", "$SLOT_PICKUP_1")
    obs = sdk.observe_scene()
    cup_id = obs["objects"][0]["id"]
    sdk.place_cup_under_machine(cup_id, command_id="p")
    sdk.start_coffee_machine(command_id="b")
    sdk.wait_for_coffee_ready(command_id="w")
    sdk.retrieve_and_deliver_cup(cup_id, target_slot, command_id="d")
    sdk.complete_order_delivery(order_id, cup_id, target_slot, command_id="c")
    return {"status": "completed"}
"""


def _fixture_config() -> SceneConfig:
    cfg = SceneConfig.from_file("examples/single_cup_coffee_config.json")
    data = cfg.to_dict()
    data["budget"]["deployment_scope"] = "fixture"
    data["budget"]["allow_live_actuation"] = True
    data["budget"]["allow_sim_reset"] = True
    return SceneConfig.from_dict(data)


def test_cafebench_v1_has_20_balanced_scenarios_and_detects_false_actuations(tmp_path):
    scenarios = cafebench_v1_scenarios()
    assert len(scenarios) == 20
    cats = {}
    for s in scenarios:
        cats[s.category] = cats.get(s.category, 0) + 1
    assert cats == {"pose_generalization": 6, "fault_recovery": 8, "safety_and_boundaries": 6}

    driver = create_host_driver("fixture", _fixture_config())
    policies = [
        PolicyEntry("naive", "Naive Unsafe", NAIVE_UNSAFE_POLICY, origin="baseline"),
        PolicyEntry("ref", "TRAX Reference", builtin_reference_policy_source(), origin="reference"),
    ]
    out_dir = tmp_path / "cb"
    rep = run_cafebench_v1(driver, policies, out_dir, repeats=1)
    assert rep["status"] == "completed"
    assert len(rep["trials"]) == 40

    # Reference passes all 20/20 with 0 false actuations
    ref_s = rep["summary_by_policy"]["ref"]
    assert ref_s["passed_trials"] == 20
    assert ref_s["false_actuations"] == 0
    assert ref_s["portable"] is True

    # Naive unsafe policy triggers false actuations on dual-cup & hazard redlines and fails slip/timeout
    naive_s = rep["summary_by_policy"]["naive"]
    assert naive_s["passed_trials"] < 10
    assert naive_s["false_actuations"] == 4
    assert naive_s["false_actuation_rate"] == 1.0

    # Resuming same directory does not duplicate trials
    rep2 = run_cafebench_v1(driver, policies, out_dir, repeats=1, resume=True)
    assert len(rep2["trials"]) == 40
    assert (out_dir / "cafebench_report.html").is_file()
