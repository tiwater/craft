"""Behavioral tests for unified teaching interaction and persistent development loop."""
from __future__ import annotations

import pytest

from trax.config import (
    CapabilityRequirementConfig,
    ExperimentBudgetConfig,
    HostConnectionConfig,
    ObjectBindingConfig,
    SceneConfig,
)
from trax.dev_loop import DevelopmentLoop, TrialRecord
from trax.teaching import (
    ClarificationRequest,
    TaskStep,
    TeachingMaterial,
    build_task_specification,
)


def _config(*, max_model_calls: int = 5, max_trials: int = 10, scope: str = "fixture") -> SceneConfig:
    return SceneConfig(
        scene_id="cafe-test",
        task_name="single_cup_coffee",
        host=HostConnectionConfig(edge_base_url="http://127.0.0.1:8080"),
        objects=(ObjectBindingConfig(name="cup", semantic_classes=("dine_in_red",)),),
        capabilities=CapabilityRequirementConfig(
            required_methods=("observe_scene", "place_cup_under_machine", "start_coffee_machine", "query_command_status"),
            non_idempotent_methods=("start_coffee_machine",),
        ),
        budget=ExperimentBudgetConfig(
            max_model_calls=max_model_calls,
            max_trials=max_trials,
            deployment_scope=scope,
        ),
    )


def _valid_steps() -> tuple[TaskStep, ...]:
    return (
        TaskStep(
            step_id="s1",
            description="Observe scene and locate single coffee cup",
            action_binding="observe_scene",
            completion_evidence=("single_cup_visible",),
        ),
        TaskStep(
            step_id="s2",
            description="Start brew once and query if receipt unknown",
            action_binding="start_coffee_machine",
            preconditions=("cup_under_spout",),
            completion_evidence=("brew_started",),
            failure_handling="query_command_status",
            non_idempotent=True,
        ),
    )


def test_teaching_waits_honestly_when_materials_or_contracts_missing():
    cfg = _config()

    # Missing steps -> awaiting_teaching, never fabricates
    spec_missing = build_task_specification(cfg, "Make one cup of coffee", steps=())
    assert spec_missing.status == "awaiting_teaching"
    assert any(c.reason == "missing_task_steps" for c in spec_missing.clarifications)

    # Video material is non-authoritative; trajectory without contract_id triggers clarification
    video = TeachingMaterial("vid-1", "video", "Barista places cup under spout")
    assert video.authoritative_scene_state is False
    traj_bad = TeachingMaterial("traj-1", "robot_trajectory", "Teleop place trajectory", contract_id=None)
    spec_traj = build_task_specification(cfg, "Make coffee", materials=(video, traj_bad), steps=_valid_steps())
    assert spec_traj.status == "awaiting_teaching"
    assert any("non-authoritative" in n or "EvidenceScene" in n for n in spec_traj.evidence_notes)
    assert any(c.reason == "missing_trajectory_contract" for c in spec_traj.clarifications)

    # Missing host capability -> blocked_missing_capability
    oven_step = TaskStep("s-oven", "Open oven", "open_oven_door", completion_evidence=("door_open",))
    spec_oven = build_task_specification(cfg, "Heat bagel", steps=(oven_step,), discovered_methods=("observe_scene",))
    assert spec_oven.status == "blocked_missing_capability"


def test_dev_loop_revises_from_feedback_retains_failures_and_resumes(tmp_path):
    cfg = _config(max_model_calls=4, max_trials=6, scope="fixture")
    spec = build_task_specification(
        cfg,
        "Brew single cup of coffee",
        materials=(TeachingMaterial("vid-1", "video", "Cup placement reference"),),
        steps=_valid_steps(),
    )
    assert spec.status == "ready"

    prompts_seen: list[str] = []
    drafts = [
        # Iteration 1: syntax error (must be retained!)
        "def run_policy(sdk, params:\n    return {}",
        # Iteration 2: valid syntax, fails runtime with empty_grasp
        "def run_policy(sdk, params):\n    return {'status': 'failed'}",
        # Iteration 3: revised using failure feedback, succeeds
        "def run_policy(sdk, params):\n    return {'status': 'completed'}",
    ]

    def fake_generator(c, prompt):
        prompts_seen.append(prompt)
        return {
            "source": drafts[len(prompts_seen) - 1],
            "provider": "codex_cli",
            "model": "fixture-model",
            "reasoning_effort": "medium",
            "evidence_source": "fixture",
        }

    trial_counter = {"n": 0}

    def fake_evaluator(cand_id, source, condition):
        trial_counter["n"] += 1
        passed = "'completed'" in source
        return TrialRecord(
            trial_id=f"trial_{trial_counter['n']:03d}",
            candidate_id=cand_id,
            condition=condition,
            evidence_source="fixture",
            status="completed" if passed else "failed",
            passed=passed,
            wall_s=1.2,
            failure_detail="" if passed else "empty_grasp during recenter",
        )

    loop = DevelopmentLoop.create_or_resume(tmp_path / "ws", cfg, spec)
    r1 = loop.run_iteration(generator_fn=fake_generator, evaluator_fn=fake_evaluator)
    assert r1["status"] == "running"
    assert loop.candidates[0].syntax_valid is False
    assert loop.candidates[0].approved is False

    # Pause and reload from disk to verify persistence and resume
    loop.pause()
    reloaded = DevelopmentLoop.create_or_resume(tmp_path / "ws", cfg, spec)
    assert reloaded.status == "paused"
    reloaded.resume()

    r2 = reloaded.run_iteration(generator_fn=fake_generator, evaluator_fn=fake_evaluator)
    assert r2["status"] == "running"
    assert "SyntaxError" in prompts_seen[1]

    r3 = reloaded.run_iteration(generator_fn=fake_generator, evaluator_fn=fake_evaluator)
    assert r3["status"] == "completed"
    assert "empty_grasp during recenter" in prompts_seen[2]
    assert reloaded.state["active_candidate_id"] == "cand_003"
    # All 3 candidates (including 2 failed) and all 3 trials retained
    assert len(reloaded.candidates) == 3
    assert len(reloaded.trials) == 3

    # Identity drift check: different config must fail to resume same workspace
    other_cfg = _config(max_model_calls=99)
    with pytest.raises(ValueError, match="identity drift"):
        DevelopmentLoop.create_or_resume(tmp_path / "ws", other_cfg, spec)


def test_dev_loop_enforces_budget_and_clarification_wait(tmp_path):
    cfg = _config(max_model_calls=1, max_trials=1, scope="simulator")
    spec = build_task_specification(cfg, "Brew coffee", steps=_valid_steps())

    def gen(c, prompt):
        return {
            "source": "def run_policy(sdk, params):\n    return {'status': 'failed'}",
            "provider": "openai_api",
            "model": "real-model-name",
            "reasoning_effort": "medium",
            "evidence_source": "simulator",
        }

    def eval_needs_clarification(cand_id, source, condition):
        return TrialRecord(
            trial_id="trial_001",
            candidate_id=cand_id,
            condition=condition,
            evidence_source="simulator",
            status="blocked",
            passed=False,
            wall_s=2.0,
            failure_detail="Ambiguous cup selection",
            needs_clarification=ClarificationRequest(
                request_id="clarify-cup",
                reason="ambiguous_cup",
                question="Which cup class should be prioritized when two are visible?",
                required_kind="text",
            ),
        )

    loop = DevelopmentLoop.create_or_resume(tmp_path / "ws2", cfg, spec)
    r1 = loop.run_iteration(generator_fn=gen, evaluator_fn=eval_needs_clarification)
    assert r1["status"] == "awaiting_teaching"
    assert loop.trials[0].evidence_source == "simulator"

    # Supply updated teaching -> now budget (1 model call) is exhausted on next attempt
    loop.supply_teaching(spec)
    r2 = loop.run_iteration(generator_fn=gen, evaluator_fn=eval_needs_clarification)
    assert r2["status"] == "budget_exhausted"
