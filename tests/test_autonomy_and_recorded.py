"""Tests for progressive autonomy promotion gates and RecordedDataHostDriver."""
from __future__ import annotations

import json
import pytest

from craft.autonomy import (
    AutonomyPromotionLedger,
    StageEvidence,
    actuation_policy_for_stage,
)
from craft.benchmark import builtin_reference_policy_source
from craft.config import SceneConfig
from craft.recorded_driver import RecordedDataHostDriver


def _sample_config() -> SceneConfig:
    cfg = SceneConfig.from_file("examples/single_cup_coffee_config.json")
    data = cfg.to_dict()
    data["budget"]["deployment_scope"] = "recorded_data"
    return SceneConfig.from_dict(data)


def test_autonomy_gate_blocks_sim_to_physical_live_jump_and_enforces_intervention_rate(tmp_path):
    ledger = AutonomyPromotionLedger(tmp_path / "autonomy.json")
    pol_sha = "policy-sha-1"
    cfg_sha = "cfg-sha-1"
    sdk_sha = "sdk-sha-1"

    # Dry-run never permits physical actuation
    assert actuation_policy_for_stage("physical_dry_run")["physical_actuation_allowed"] is False
    assert actuation_policy_for_stage("physical_step_gated")["step_confirmation_required"] is True

    # Only simulator passed -> physical_live and physical_step_gated are BLOCKED!
    ledger.record_evidence(
        StageEvidence(
            stage="simulator",
            policy_sha256=pol_sha,
            config_sha256=cfg_sha,
            sdk_contract_sha256=sdk_sha,
            report_id="sim-1",
            passed=True,
            total_trials=10,
            human_interventions=0,
        )
    )
    jump = ledger.check_promotion(policy_sha256=pol_sha, target_stage="physical_live", sdk_contract_sha256=sdk_sha)
    assert jump["allowed"] is False
    assert "physical_dry_run" in jump["missing_prerequisites"]
    assert "physical_step_gated" in jump["missing_prerequisites"]

    # Passing physical_dry_run unlocks physical_step_gated
    assert ledger.check_promotion(policy_sha256=pol_sha, target_stage="physical_dry_run", sdk_contract_sha256=sdk_sha)["allowed"] is True
    ledger.record_evidence(
        StageEvidence(
            stage="physical_dry_run",
            policy_sha256=pol_sha,
            config_sha256=cfg_sha,
            sdk_contract_sha256=sdk_sha,
            report_id="dry-1",
            passed=True,
            total_trials=5,
            human_interventions=0,
        )
    )
    assert ledger.check_promotion(policy_sha256=pol_sha, target_stage="physical_step_gated", sdk_contract_sha256=sdk_sha)["allowed"] is True

    # High intervention rate (30% > 10%) in step_gated blocks physical_live
    ledger.record_evidence(
        StageEvidence(
            stage="physical_step_gated",
            policy_sha256=pol_sha,
            config_sha256=cfg_sha,
            sdk_contract_sha256=sdk_sha,
            report_id="gate-1",
            passed=True,
            total_trials=10,
            human_interventions=3,
        )
    )
    too_many = ledger.check_promotion(
        policy_sha256=pol_sha,
        target_stage="physical_live",
        sdk_contract_sha256=sdk_sha,
        max_live_intervention_rate=0.1,
    )
    assert too_many["allowed"] is False
    assert any("intervention_rate" in r for r in too_many["reasons"])

    # Lower intervention rate (0%) unlocks physical_live
    ledger.record_evidence(
        StageEvidence(
            stage="physical_step_gated",
            policy_sha256=pol_sha,
            config_sha256=cfg_sha,
            sdk_contract_sha256=sdk_sha,
            report_id="gate-2",
            passed=True,
            total_trials=10,
            human_interventions=0,
        )
    )
    live_ok = ledger.check_promotion(policy_sha256=pol_sha, target_stage="physical_live", sdk_contract_sha256=sdk_sha)
    assert live_ok["allowed"] is True
    assert len(live_ok["intervention_trend"]) == 3


def test_recorded_data_driver_verifies_fail_closed_under_noisy_real_frames(tmp_path):
    jsonl = tmp_path / "recorded_pouring.jsonl"
    frames = [
        # Frame 0: real occlusion -> control_ready=False (must fail closed, 0 brew)
        {"control_ready": False, "timestamp_ns": 1000, "objects": [], "l2_pouring_safety": {"safe_to_pour": False}},
        # Frame 1: real glare -> low confidence cup (must pause_for_operator, 0 brew)
        {
            "control_ready": True,
            "timestamp_ns": 2000,
            "confidence_threshold": 0.75,
            "objects": [{"object_id": "cup-1", "semantic_class": "dine_in_red", "confidence": 0.42, "center_world_m": [0.35, 0.0, 1.07]}],
            "l2_pouring_safety": {"safe_to_pour": False},
        },
    ]
    jsonl.write_text("\n".join(json.dumps(f) for f in frames) + "\n", encoding="utf-8")

    drv = RecordedDataHostDriver(_sample_config(), recording_path=jsonl)
    assert drv.evidence_source == "recorded_data"
    assert drv.discover_capabilities().status == "ready"

    policy = builtin_reference_policy_source()

    # Frame 0: occluded / unready -> fails closed cleanly
    r0 = drv.run_policy_scenario(
        policy,
        command_id="rec-0",
        scenario={"start_frame": 0, "expect_status": "rejected_observation_not_ready"},
    )
    assert r0["task_passed"] is True
    assert r0["physical_commands"] == 0
    assert r0["non_idempotent_counts"]["start_coffee_machine"] == 0

    # Frame 1: low confidence glare -> pauses for operator cleanly
    r1 = drv.run_policy_scenario(
        policy,
        command_id="rec-1",
        scenario={"start_frame": 1, "expect_status": "paused_for_operator"},
    )
    assert r1["task_passed"] is True
    assert r1["physical_commands"] == 0
    assert r1["non_idempotent_counts"]["start_coffee_machine"] == 0
