"""Tests for independent acceptance, fixed-version release, rollback, and verified skills."""
from __future__ import annotations

import pytest

from trax.acceptance import (
    FrozenArtifact,
    HeldoutTrialOutcome,
    compare_improvement_candidate,
    evaluate_acceptance,
)
from trax.config import AcceptanceCriteriaConfig
from trax.release import ReleaseRegistry
from trax.skills import SkillExperience, SkillExperienceStore


POLICY_V1 = "def run_policy(sdk, params):\n    return {'status': 'completed', 'v': 1}\n"
POLICY_V2_FAST = "def run_policy(sdk, params):\n    return {'status': 'completed', 'v': 2}\n"
POLICY_V2_RISKY = "def run_policy(sdk, params):\n    return {'status': 'completed', 'v': 'risky'}\n"


def _trial(tid: str, *, wall_s: float = 30.0, completed: bool = True, **kw) -> HeldoutTrialOutcome:
    return HeldoutTrialOutcome(
        trial_id=tid,
        condition="heldout_normal",
        evidence_source="simulator",
        completed=completed,
        wall_s=wall_s,
        delivery_error_mm=kw.pop("delivery_error_mm", 4.0),
        **kw,
    )


def test_acceptance_rejects_mutated_criteria_debug_leak_and_failures(tmp_path):
    locked = AcceptanceCriteriaConfig(min_completion_rate=1.0, max_unknown_outcomes=0, min_heldout_trials=2)
    relaxed = AcceptanceCriteriaConfig(min_completion_rate=0.5, max_unknown_outcomes=1, min_heldout_trials=1)

    # Policy frozen against relaxed criteria cannot pass locked independent criteria
    bad_art = FrozenArtifact.freeze(
        policy_source=POLICY_V1,
        config_sha256="cfg-1",
        sdk_contract_sha256="sdk-1",
        criteria=relaxed,
        deployment_scope="simulator",
    )
    rep_mismatch = evaluate_acceptance(bad_art, locked, [_trial("t1"), _trial("t2")])
    assert rep_mismatch.passed is False
    assert any("Criteria hash mismatch" in r for r in rep_mismatch.failure_reasons)

    # Debug trial leak rejected
    art = FrozenArtifact.freeze(
        policy_source=POLICY_V1,
        config_sha256="cfg-1",
        sdk_contract_sha256="sdk-1",
        criteria=locked,
        deployment_scope="simulator",
    )
    rep_debug = evaluate_acceptance(art, locked, [_trial("t1", used_in_debugging=True), _trial("t2")])
    assert rep_debug.passed is False
    assert any("used_in_debugging" in r for r in rep_debug.failure_reasons)

    # Duplicate non-idempotent or unknown outcome fails acceptance and blocks publish
    rep_dup = evaluate_acceptance(art, locked, [_trial("t1", duplicate_non_idempotent=True), _trial("t2")])
    assert rep_dup.passed is False
    reg = ReleaseRegistry(tmp_path / "reg")
    with pytest.raises(ValueError, match="Cannot publish failed acceptance report"):
        reg.publish(art, rep_dup)

    # Disclaimer explicitly states simulator != physical/food certification
    assert "NOT physical-robot approval" in rep_dup.certification_disclaimer
    assert "NOT food-quality certification" in rep_dup.certification_disclaimer
    assert "host robot controller" in rep_dup.certification_disclaimer


def test_slow_reliable_policy_publishes_queues_optimization_and_enforces_reliability_first(tmp_path):
    criteria = AcceptanceCriteriaConfig(
        min_completion_rate=1.0,
        target_wall_s=45.0,
        slow_step_threshold_s=20.0,
        min_heldout_trials=2,
    )
    art_v1 = FrozenArtifact.freeze(
        policy_source=POLICY_V1,
        config_sha256="cfg-1",
        sdk_contract_sha256="sdk-1",
        criteria=criteria,
        deployment_scope="simulator",
    )
    # Reliable but slow (wall_s=60 > target 45, slow step 25 > 20)
    rep_v1 = evaluate_acceptance(
        art_v1,
        criteria,
        [
            _trial("t1", wall_s=60.0, step_durations_s={"place_cup": 25.0}),
            _trial("t2", wall_s=58.0, step_durations_s={"place_cup": 24.0}),
        ],
    )
    assert rep_v1.passed is True
    assert rep_v1.queue_for_optimization is True

    reg = ReleaseRegistry(tmp_path / "reg")
    v1 = reg.publish(art_v1, rep_v1, version="v1")
    assert reg.active_version_id == "v1"
    assert len(reg.optimization_queue) == 1
    queue_id = reg.optimization_queue[0]["queue_id"]

    # Fixed-version execution rejects silent edits
    reg.verify_fixed_execution(POLICY_V1, config_sha256="cfg-1", sdk_contract_sha256="sdk-1")
    with pytest.raises(ValueError, match="differs from active frozen release"):
        reg.verify_fixed_execution(POLICY_V2_FAST, config_sha256="cfg-1", sdk_contract_sha256="sdk-1")

    # Faster candidate with reliability regression (unknown_outcome=True) is REJECTED
    art_risky = FrozenArtifact.freeze(
        policy_source=POLICY_V2_RISKY,
        config_sha256="cfg-1",
        sdk_contract_sha256="sdk-1",
        criteria=criteria,
        deployment_scope="simulator",
    )
    rep_risky = evaluate_acceptance(
        art_risky,
        criteria,
        [_trial("t3", wall_s=20.0, unknown_outcome=True), _trial("t4", wall_s=20.0)],
    )
    verdict_risky = compare_improvement_candidate(rep_v1, rep_risky)
    assert verdict_risky["decision"] == "rejected"

    # Faster candidate maintaining reliability is ACCEPTED and published as v2
    art_v2 = FrozenArtifact.freeze(
        policy_source=POLICY_V2_FAST,
        config_sha256="cfg-1",
        sdk_contract_sha256="sdk-1",
        criteria=criteria,
        deployment_scope="simulator",
    )
    rep_v2 = evaluate_acceptance(
        art_v2,
        criteria,
        [_trial("t5", wall_s=32.0, step_durations_s={"place_cup": 12.0}), _trial("t6", wall_s=30.0)],
    )
    assert rep_v2.passed is True
    assert rep_v2.queue_for_optimization is False

    res = reg.evaluate_improvement(
        queue_id=queue_id,
        baseline_report=rep_v1,
        candidate_artifact=art_v2,
        candidate_report=rep_v2,
        new_version="v2",
    )
    assert res["decision"] == "accepted"
    assert res["published_version"] == "v2"
    assert reg.active_version_id == "v2"

    # Rollback to v1 restores v1 and logs audit event
    rolled = reg.rollback("v1", reason="operator requested rollback")
    assert rolled.version == "v1"
    assert reg.active_version_id == "v1"
    assert reg.get_policy_source() == POLICY_V1
    assert any(h["event"] == "rollback" and h["version"] == "v1" for h in reg.history)


def test_skill_experience_store_enforces_verification_and_applicability(tmp_path):
    store = SkillExperienceStore(tmp_path / "skills.json")
    store.add(
        SkillExperience(
            skill_id="sk-recenter",
            title="Recenter cup on slip",
            guidance="If place_cup_under_machine reports slip, call recenter_cup_under_spout before brewing.",
            task_name="single_cup_coffee",
            required_methods=("place_cup_under_machine", "recenter_cup_under_spout"),
            sdk_contract_sha256="sdk-hash-1",
            evidence_source="simulator",
            verification_report_id="acc-123",
            verified=True,
            verified_at_utc="2026-11-20T00:00:00+00:00",
        )
    )
    store.add(
        SkillExperience(
            skill_id="sk-unverified",
            title="Unverified hunch",
            guidance="Skip observe_scene",
            task_name="single_cup_coffee",
            required_methods=("observe_scene",),
            sdk_contract_sha256="sdk-hash-1",
            evidence_source="fixture",
            verification_report_id="draft-0",
            verified=False,
            verified_at_utc="2026-11-20T00:00:00+00:00",
        )
    )

    # Matching capabilities & contract hash -> returns only verified skill
    matched = store.query_applicable(
        task_name="single_cup_coffee",
        available_methods=("observe_scene", "place_cup_under_machine", "recenter_cup_under_spout"),
        sdk_contract_sha256="sdk-hash-1",
    )
    assert [s.skill_id for s in matched] == ["sk-recenter"]

    # Drifted contract hash -> excluded
    drifted = store.query_applicable(
        task_name="single_cup_coffee",
        available_methods=("observe_scene", "place_cup_under_machine", "recenter_cup_under_spout"),
        sdk_contract_sha256="sdk-hash-DIFFERENT",
    )
    assert drifted == []
