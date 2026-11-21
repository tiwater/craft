"""Tests for four-layer failure attribution and step-level latency profiler."""
from __future__ import annotations

import json
from pathlib import Path

from craft.profiler import aggregate_trial_profiles, attribute_trial, profile_steps


def test_four_layer_attribution_separates_ownership():
    # Policy layer: duplicate non-idempotent & unreconciled unknown & missing recenter
    a_dup = attribute_trial({"task_passed": False, "status": "completed", "duplicate_non_idempotent": True})
    assert a_dup.layer == "policy"
    assert a_dup.auto_repair_eligible is True

    a_unk = attribute_trial({"task_passed": False, "unknown_events_observed": 1, "queried_after_unknown": False})
    assert a_unk.layer == "policy"
    assert a_unk.code == "unreconciled_unknown_ack"

    a_norec = attribute_trial({"task_passed": False, "status": "failed_brew_start", "fault_injection": True, "methods": ["place_cup_under_machine", "start_coffee_machine"]})
    assert a_norec.layer == "policy"
    assert a_norec.code == "missing_recenter_branch"

    # Perception layer: not auto-repaired by LLM policy edits
    a_perc = attribute_trial({"task_passed": False, "host_runtime_status": "rejected_observation_not_ready"})
    assert a_perc.layer == "perception"
    assert a_perc.recommended_owner == "host_perception"
    assert a_perc.auto_repair_eligible is False

    # Control layer: empty_grasp / failed_recenter / failed_verify
    a_ctrl = attribute_trial({"task_passed": False, "host_runtime_status": "failed_recenter", "detail": "empty_grasp"})
    assert a_ctrl.layer == "control"
    assert a_ctrl.recommended_owner == "host_control"
    assert a_ctrl.auto_repair_eligible is False

    # Device layer: disconnected / missing capabilities
    a_dev = attribute_trial({"task_passed": False, "status": "disconnected", "detail": "Connection refused"})
    assert a_dev.layer == "device"
    assert a_dev.recommended_owner == "host_device"
    assert a_dev.auto_repair_eligible is False


def test_step_profiler_handles_timestamps_out_of_order_and_bottlenecks():
    steps = [
        {"method": "get_order", "timestampMs": 1000, "receipt": "done"},
        {"method": "observe_scene", "timestampMs": 1200, "receipt": "done"},
        {"method": "place_cup_under_machine", "timestampMs": 7200, "receipt": "done"},  # 6.0s
        {"method": "start_coffee_machine", "timestampMs": 7500, "receipt": "running"},  # 0.3s
        {"method": "wait_for_coffee_ready", "timestampMs": 18500, "receipt": "done"},  # 11.0s (slow > 8s)
        {"method": "retrieve_and_deliver_cup", "timestampMs": 18000, "receipt": "done"},  # out-of-order -> None
    ]
    prof = profile_steps(steps, total_wall_s=20.0, slow_step_threshold_s=8.0)
    assert prof["step_count"] == 6
    assert len(prof["slow_steps"]) == 1
    assert prof["slow_steps"][0]["method"] == "wait_for_coffee_ready"
    assert prof["by_method"][0]["method"] == "wait_for_coffee_ready"
    assert prof["by_method"][0]["total_s"] == 11.0
    assert prof["steps"][5]["duration_s"] is None
    assert any("pipelining" in h for h in prof["optimization_hints"])


def test_aggregate_profiles_on_archived_real_simulator_benchmark():
    bench_path = (
        Path(__file__).resolve().parents[1]
        / "experiments"
        / "coffee-closed-loop-20261120"
        / "benchmark"
        / "benchmark_results.json"
    )
    data = json.loads(bench_path.read_text(encoding="utf-8"))
    summary = aggregate_trial_profiles(data["trials"])
    assert summary["total_trials"] == 18
    assert summary["passed_trials"] == 17
    assert summary["failed_trials"] == 1
    # The single failure (failed_verify on craft_reference s5) is attributed to control layer
    assert summary["attribution_counts"]["control"] == 1
    assert summary["auto_repair_eligible_failures"] == 0
    assert len(summary["method_bottlenecks"]) > 0
