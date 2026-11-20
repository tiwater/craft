"""End-to-end Phase 1 single-cup coffee simulation closed-loop orchestrator.

Connects:
1. SceneConfig + EdgeHostAdapter capability discovery (plus explicit oven/bagel gap report)
2. Read-only EvidenceScene observation pass-through
3. Unified teaching interaction (TaskSpecification)
4. Persistent DevelopmentLoop with real LLM generation + live edge simulation trials
5. Independent Acceptance evaluation (held-out conditions, locked criteria)
6. Fixed-Version ReleaseRegistry publishing + Verified SkillExperience storage
7. Improvement candidate accept/reject verification (reliability-first, then speed)

All raw trial records, candidates, failures, acceptance reports, and release history
are persisted in the workspace. No endpoints or private paths are hardcoded.
"""
from __future__ import annotations

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from typing import Any, Callable, Mapping, Sequence

from .acceptance import (
    AcceptanceReport,
    FrozenArtifact,
    HeldoutTrialOutcome,
    evaluate_acceptance,
)
from .config import CapabilityRequirementConfig, SceneConfig
from .contracts import digest
from .dev_loop import (
    DevelopmentLoop,
    GeneratorFn,
    TrialRecord,
    default_live_generator,
)
from .edge_adapter import EdgeHostAdapter
from .journal import Journal
from .release import ReleaseRegistry
from .skills import SkillExperience, SkillExperienceStore
from .teaching import TaskStep, TeachingMaterial, build_task_specification


OVEN_REQUIRED_METHODS = (
    "open_oven_door",
    "load_bagel",
    "start_oven_heat",
    "query_oven_status",
    "unload_bagel",
)

EDGE_SANDBOX_CONTRACT_NOTES = """Host EdgeZeroGtCoffeeSDK Sandbox Contract (enforced by host AST gate & OS subprocess):
- NO import, try/except, while, with, class, global, nonlocal, del, or async statements.
- NO identifiers or attributes starting with '_' (no _private or __dunder__).
- Use ONLY safe builtins (dict, list, len, str, float, int, bool, min, max, round, range, abs, all, any, sum, sorted, tuple) and the sdk proxy passed into run_policy(sdk, params).
- Allowed sdk methods:
  - sdk.get_order(order_id) -> {"order_id", "drink", "cup_preference", "destination_slot", "special_request", "status"}
  - sdk.observe_scene() -> {"control_ready": bool, "pipeline_health": dict, "objects": [{"id", "cup_type", "label", "confidence", "world": [x,y,z], "tilt_deg"}], "coffee_machine": {"status", "cup_filled"}, "l2_pouring_safety": {"safe_to_pour": bool, ...}, "confidence_threshold": float, "operator_confirmed_cup_id": str | None}
  - sdk.request_operator_confirmation(reason, candidates)
  - sdk.retract_safe_pose(command_id=...)
  - sdk.escalate_to_human_operator(order_id, reason, command_id=...)
  - sdk.place_cup_under_machine(cup_id, command_id=...) -> {"receipt": "done"|"rejected"}
  - sdk.recenter_cup_under_spout(cup_id, command_id=...) -> {"receipt": "done"|"rejected"}
  - sdk.start_coffee_machine(command_id=...) -> {"receipt": "running"|"done"|"rejected"|"unknown"}
  - sdk.query_command_status(command_id) -> {"receipt": ...}
  - sdk.wait_for_coffee_ready(command_id=...) -> {"receipt": "done"|"rejected"}
  - sdk.retrieve_and_deliver_cup(cup_id, target_slot, command_id=...) -> {"receipt": "done"|"rejected"}
  - sdk.complete_order_delivery(order_id, cup_id, destination_slot, command_id=...) -> {"receipt": "done"|"rejected"}
- Use params.get("order_id", "A042"), params.get("target_slot", "$SLOT_PICKUP_1"), params.get("command_prefix", "cap").
- Filter out cup_type == "flexible_bag". Check control_ready, select single cup with confidence >= threshold (or operator_confirmed_cup_id), else request_operator_confirmation.
- After place_cup_under_machine, re-observe via sdk.observe_scene(); if not l2_pouring_safety.get("safe_to_pour"), call recenter_cup_under_spout and re-observe.
- For start_coffee_machine, if receipt == "unknown", call sdk.query_command_status(brew_cmd_id) ONCE and NEVER call start_coffee_machine a second time.
- Return {"status": "completed", ...} only after complete_order_delivery returns receipt == "done"."""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    tmp.replace(path)


def default_coffee_teaching_steps() -> tuple[TaskStep, ...]:
    """Structured Phase 1 single-cup coffee steps grounded in EdgeZeroGtCoffeeSDK."""
    return (
        TaskStep(
            step_id="observe_and_select_cup",
            description="Read order and live EvidenceScene; verify control_ready and uniquely bind a single rigid cup.",
            action_binding="observe_scene",
            preconditions=("host_capabilities_ready",),
            completion_evidence=("control_ready", "single_cup_bound"),
            failure_handling="request_operator_confirmation or return rejected_observation_not_ready",
        ),
        TaskStep(
            step_id="place_cup",
            description="Place selected cup under coffee machine spout and re-observe L2 pouring safety.",
            action_binding="place_cup_under_machine",
            preconditions=("single_cup_bound",),
            completion_evidence=("place_receipt_done",),
            failure_handling="return failed_place without brewing",
        ),
        TaskStep(
            step_id="recenter_if_offset",
            description="If l2_pouring_safety.safe_to_pour is False after placement, recenter cup under spout and re-observe.",
            action_binding="recenter_cup_under_spout",
            preconditions=("place_receipt_done", "l2_pouring_not_safe"),
            completion_evidence=("l2_safe_to_pour",),
            failure_handling="return failed_recenter without brewing",
        ),
        TaskStep(
            step_id="start_brew_reconcile_unknown",
            description="Start coffee machine once; if receipt is unknown, query_command_status and never reissue brew.",
            action_binding="start_coffee_machine",
            preconditions=("l2_safe_to_pour",),
            completion_evidence=("brew_running_or_done", "brew_execution_count_eq_1"),
            failure_handling="query_command_status; if unresolved return failed_brew_start",
            non_idempotent=True,
        ),
        TaskStep(
            step_id="wait_deliver_verify",
            description="Wait for coffee ready, retrieve and deliver cup to target_slot, and complete_order_delivery.",
            action_binding="retrieve_and_deliver_cup",
            preconditions=("brew_running_or_done",),
            completion_evidence=("cup_filled", "delivery_verified"),
            failure_handling="return step failure receipt",
        ),
    )


def check_oven_capability_gap(config: SceneConfig, adapter: EdgeHostAdapter) -> dict[str, Any]:
    """Explicitly report missing oven/bagel capabilities on the current host."""
    oven_data = config.to_dict()
    oven_data["capabilities"] = {
        "required_methods": list(OVEN_REQUIRED_METHODS),
        "non_idempotent_methods": ["start_oven_heat"],
        "expected_sdk_contract_sha256": None,
    }
    oven_cfg = SceneConfig.from_dict(oven_data)
    oven_adapter = EdgeHostAdapter(oven_cfg, transport=adapter.transport)
    res = oven_adapter.discover_capabilities(allow_busy=True)
    return {
        "scope": "bagel_and_combo_extension_readiness",
        "status": res.status,
        "missing_methods": list(res.missing_methods),
        "detail": res.detail,
    }


def wait_for_fresh_single_cup_scene(adapter: EdgeHostAdapter, *, timeout_s: float = 15.0) -> dict[str, Any]:
    """Wait for a fresh EvidenceScene snapshot with a single cup after simulator reset."""
    initial = adapter.observe_scene()
    init_seq = int((initial.raw_snapshot or {}).get("step_seq") or 0)
    deadline = time.monotonic() + timeout_s
    while True:
        obs = adapter.observe_scene()
        seq = int((obs.raw_snapshot or {}).get("step_seq") or 0)
        if obs.status == "ok" and seq >= init_seq + 3:
            return obs.to_dict()
        if time.monotonic() > deadline:
            raise RuntimeError(f"Fresh single-cup EvidenceScene snapshot unavailable (last status={obs.status}: {obs.detail})")
        time.sleep(0.25)


def score_coffee_sim_trial(
    adapter: EdgeHostAdapter,
    *,
    policy_source: str,
    command_id: str,
    fault_injection: bool,
    journal: Journal,
    max_delivery_error_mm: float = 15.0,
) -> dict[str, Any]:
    """Reset sim, wait for fresh EvidenceScene, execute policy via edge, and score via post-run oracle."""
    adapter.reset_host_and_sim(preset="default")
    scene_info = wait_for_fresh_single_cup_scene(adapter)

    exec_res = adapter.execute_policy(
        policy_source,
        command_id=command_id,
        order_id="A042",
        target_slot="$SLOT_PICKUP_1",
        simulate_timeout=fault_injection,
        inject_place_slip_xy_m=[0.038, -0.012] if fault_injection else None,
        journal=journal,
    )

    oracle = adapter.read_sim_oracle()
    cup_pos = oracle.get("cup") or [0.0, 0.0, 0.0]
    err_mm = round(math.hypot(float(cup_pos[0]) - 0.38, float(cup_pos[1]) + 0.17) * 1000.0, 3)
    cup_filled = bool((oracle.get("coffee_machine") or {}).get("cup_filled"))
    brew_count = int(exec_res.non_idempotent_counts.get("start_coffee_machine", 0))

    task_passed = (
        exec_res.status == "completed"
        and cup_filled
        and err_mm <= max_delivery_error_mm
        and not exec_res.duplicate_non_idempotent
    )
    fault_checks_passed: bool | None = None
    if fault_injection:
        fault_checks_passed = (
            "recenter_cup_under_spout" in exec_res.methods
            and brew_count == 1
            and exec_res.unknown_events_observed >= 1
            and exec_res.queried_after_unknown
        )
        task_passed = task_passed and bool(fault_checks_passed)

    return {
        "command_id": command_id,
        "fault_injection": fault_injection,
        "scene_timestamp_ns": scene_info.get("timestamp_ns"),
        "scene_age_ms": scene_info.get("age_ms"),
        "execution": exec_res.to_dict(),
        "oracle_post_run": {
            "cup_filled": cup_filled,
            "delivery_error_mm": err_mm,
        },
        "fault_checks_passed": fault_checks_passed,
        "task_passed": task_passed,
    }


def run_coffee_closed_loop(
    config: SceneConfig,
    workspace_dir: str | Path,
    *,
    adapter: EdgeHostAdapter | None = None,
    generator_fn: GeneratorFn | None = None,
    instruction: str = "制作一杯堂食热美式，送到1号取餐口；若放杯偏移先微调居中，若冲煮回执未知先查询状态切勿重复启动。",
    run_improvement_step: bool = True,
) -> dict[str, Any]:
    """Execute the full Phase 1 single-cup coffee simulation closed loop."""
    ws = Path(workspace_dir)
    ws.mkdir(parents=True, exist_ok=True)
    host = adapter or EdgeHostAdapter(config)
    gen = generator_fn or default_live_generator

    # 1. Pre-declare protocol, conditions, budgets, and stop rules
    protocol = {
        "task_name": config.task_name,
        "config_sha256": config.config_sha256,
        "criteria_sha256": config.acceptance.criteria_sha256,
        "evidence_source": config.budget.deployment_scope,
        "generation": {
            "provider": config.generation.provider,
            "model": config.generation.model,
            "reasoning_effort": config.generation.reasoning_effort,
        },
        "budget": {
            "max_model_calls": config.budget.max_model_calls,
            "max_trials": config.budget.max_trials,
            "max_wall_s": config.budget.max_wall_s,
            "allow_live_actuation": config.budget.allow_live_actuation,
            "allow_sim_reset": config.budget.allow_sim_reset,
        },
        "dev_conditions": ["dev_normal", "dev_fault_offset_and_timeout"],
        "heldout_conditions": ["heldout_normal", "heldout_fault_offset_and_timeout"],
        "stop_rules": [
            "Stop immediately if host capabilities are missing, drifted, busy, or disconnected.",
            "Stop development if budget (model_calls, trials, or wall_s) is exhausted.",
            "Do not publish if independent held-out acceptance fails.",
            "Do not accept an improvement candidate if reliability regresses or acceptance fails.",
            "Preserve all failed candidates, failed trials, and diagnostics.",
        ],
        "started_at_utc": _utc_now(),
    }
    _write_json(ws / "protocol.json", protocol)

    # 2. Discover capabilities + report oven gap + observe live scene
    cap_check = host.discover_capabilities()
    oven_gap = check_oven_capability_gap(config, host)
    _write_json(ws / "discovery.json", {"coffee_host": cap_check.to_dict(), "oven_gap": oven_gap})
    if cap_check.status != "ready" or cap_check.discovered is None:
        summary = {
            "status": "blocked_host_not_ready",
            "capability_check": cap_check.to_dict(),
            "oven_gap": oven_gap,
        }
        _write_json(ws / "summary.json", summary)
        return summary

    sdk_contract_sha256 = cap_check.discovered.sdk_contract_sha256
    obs_check = host.observe_scene()
    _write_json(ws / "initial_observation.json", obs_check.to_dict())

    # 3. Unified teaching specification
    materials = (
        TeachingMaterial(
            material_id="host-sdk-contract-v1.1.0",
            kind="text",
            summary="EdgeZeroGtCoffeeSDK sandbox and method contract",
            notes=EDGE_SANDBOX_CONTRACT_NOTES,
        ),
        TeachingMaterial(
            material_id="barista-single-cup-reference",
            kind="video",
            summary="Reference single-cup dine-in americano workflow (non-authoritative teaching evidence only)",
            source_ref="teaching://single-cup-coffee-ref",
        ),
    )
    spec = build_task_specification(
        config,
        instruction,
        materials=materials,
        steps=default_coffee_teaching_steps(),
        discovered_methods=cap_check.discovered.allowed_methods,
    )
    _write_json(ws / "task_spec.json", spec.to_dict())

    # 4. Development loop (real model generation + real simulator execution)
    journal_path = ws / "execution_journal.sqlite"
    skill_store = SkillExperienceStore(ws / "skills.json")
    dev_loop = DevelopmentLoop.create_or_resume(ws / "dev", config, spec)
    raw_dev_runs: list[dict[str, Any]] = []
    trial_seq = {"n": 0}

    with Journal(journal_path) as journal:
        def dev_evaluator(candidate_id: str, source: str, condition: str) -> TrialRecord:
            trial_seq["n"] += 1
            fault = "fault" in condition
            cmd_id = f"craft-dev-{candidate_id}-{condition}-{time.time_ns()}"
            scored = score_coffee_sim_trial(
                host,
                policy_source=source,
                command_id=cmd_id,
                fault_injection=fault,
                journal=journal,
                max_delivery_error_mm=config.acceptance.max_delivery_error_mm,
            )
            raw_dev_runs.append(scored)
            _write_json(ws / "raw_dev_runs.json", {"runs": raw_dev_runs})
            ex = scored["execution"]
            return TrialRecord(
                trial_id=f"dev_trial_{trial_seq['n']:03d}",
                candidate_id=candidate_id,
                condition=condition,
                evidence_source=config.budget.deployment_scope,
                status=ex["status"],
                passed=bool(scored["task_passed"]),
                wall_s=float(ex["wall_s"]),
                non_idempotent_counts=ex["non_idempotent_counts"],
                duplicate_non_idempotent=bool(ex["duplicate_non_idempotent"]),
                unknown_events_observed=int(ex["unknown_events_observed"]),
                queried_after_unknown=bool(ex["queried_after_unknown"]),
                human_interventions=int(ex["human_interventions"]),
                delivery_error_mm=scored["oracle_post_run"]["delivery_error_mm"],
                failure_detail="" if scored["task_passed"] else f"status={ex['status']} detail={ex['detail']} err_mm={scored['oracle_post_run']['delivery_error_mm']} cup_filled={scored['oracle_post_run']['cup_filled']}",
            )

        while dev_loop.status in {"initialized", "running"}:
            dev_loop.run_iteration(
                generator_fn=gen,
                evaluator_fn=dev_evaluator,
                conditions=("dev_normal", "dev_fault_offset_and_timeout"),
            )

        active_cand_id = dev_loop.state.get("active_candidate_id")
        if not active_cand_id:
            summary = {
                "status": f"development_{dev_loop.status}",
                "oven_gap": oven_gap,
                "dev_budget_used": dev_loop.state["budget_used"],
                "candidates_generated": len(dev_loop.candidates),
                "dev_trials": len(dev_loop.trials),
                "published_version": None,
                "improvement_result": None,
            }
            _write_json(ws / "summary.json", summary)
            return summary

        # 5. Independent held-out acceptance & fixed-version release (v1)
        v1_source = dev_loop.get_candidate_source(active_cand_id)
        v1_artifact = FrozenArtifact.freeze(
            policy_source=v1_source,
            config_sha256=config.config_sha256,
            sdk_contract_sha256=sdk_contract_sha256,
            criteria=config.acceptance,
            deployment_scope=config.budget.deployment_scope,
        )

        def run_heldout(tag: str, source: str) -> tuple[list[HeldoutTrialOutcome], list[dict[str, Any]]]:
            outcomes: list[HeldoutTrialOutcome] = []
            raw_runs: list[dict[str, Any]] = []
            for idx, (cond, fault) in enumerate(
                (("heldout_normal", False), ("heldout_fault_offset_and_timeout", True)), start=1
            ):
                cmd_id = f"craft-acc-{tag}-{cond}-{time.time_ns()}"
                scored = score_coffee_sim_trial(
                    host,
                    policy_source=source,
                    command_id=cmd_id,
                    fault_injection=fault,
                    journal=journal,
                    max_delivery_error_mm=config.acceptance.max_delivery_error_mm,
                )
                raw_runs.append(scored)
                ex = scored["execution"]
                outcomes.append(
                    HeldoutTrialOutcome(
                        trial_id=f"{tag}_heldout_{idx:02d}",
                        condition=cond,
                        evidence_source=config.budget.deployment_scope,
                        completed=bool(scored["task_passed"]),
                        wall_s=float(ex["wall_s"]),
                        used_in_debugging=False,
                        unknown_outcome=(ex["status"] == "unknown_outcome") or (
                            int(ex["unknown_events_observed"]) > 0 and not bool(ex["queried_after_unknown"])
                        ),
                        duplicate_non_idempotent=bool(ex["duplicate_non_idempotent"]),
                        human_interventions=int(ex["human_interventions"]),
                        delivery_error_mm=scored["oracle_post_run"]["delivery_error_mm"],
                        detail=ex["detail"],
                    )
                )
            return outcomes, raw_runs

        v1_outcomes, v1_raw_heldout = run_heldout("v1", v1_source)
        _write_json(ws / "v1_heldout_runs.json", {"runs": v1_raw_heldout})
        v1_report = evaluate_acceptance(v1_artifact, config.acceptance, v1_outcomes, report_id="acc-v1")
        _write_json(ws / "v1_acceptance_report.json", v1_report.to_dict())

        registry = ReleaseRegistry(ws / "registry")
        if not v1_report.passed:
            summary = {
                "status": "acceptance_failed_not_published",
                "oven_gap": oven_gap,
                "v1_acceptance": v1_report.to_dict(),
                "published_version": None,
                "improvement_result": None,
            }
            _write_json(ws / "summary.json", summary)
            return summary

        v1_release = registry.publish(v1_artifact, v1_report, version="v1", notes="Initial Phase 1 single-cup coffee release")
        registry.verify_fixed_execution(
            v1_source,
            config_sha256=config.config_sha256,
            sdk_contract_sha256=sdk_contract_sha256,
        )

        # Record verified skill experience with applicability conditions & provenance
        skill_store.add(
            SkillExperience(
                skill_id="skill-coffee-recenter-and-brew-reconcile",
                title="Closed-loop recentering and unknown brew receipt reconciliation",
                guidance=(
                    "After place_cup_under_machine, re-observe via sdk.observe_scene() and call "
                    "recenter_cup_under_spout if l2_pouring_safety.safe_to_pour is False. On unknown "
                    "start_coffee_machine receipt, call query_command_status(brew_cmd_id) once; never reissue brew."
                ),
                task_name=config.task_name,
                required_methods=("observe_scene", "place_cup_under_machine", "recenter_cup_under_spout", "start_coffee_machine", "query_command_status"),
                sdk_contract_sha256=sdk_contract_sha256,
                evidence_source=config.budget.deployment_scope,
                verification_report_id=v1_report.report_id,
                verified=True,
                verified_at_utc=_utc_now(),
                applicability_notes="Verified on single-cup coffee simulator with default and offset+timeout fault conditions.",
            )
        )

        # 6. Improvement candidate evaluation (accept or reject honestly!)
        improvement_result: dict[str, Any] | None = None
        if run_improvement_step:
            # Ensure an optimization queue entry exists for evaluating a candidate improvement
            if not registry.optimization_queue:
                registry._state["optimization_queue"].append(
                    {
                        "queue_id": "opt-001",
                        "baseline_version": v1_release.version,
                        "baseline_report_id": v1_report.report_id,
                        "reason": "Scheduled feedback-driven improvement candidate check (reduce redundant observe_scene calls without sacrificing reliability)",
                        "status": "pending",
                        "created_at_utc": _utc_now(),
                    }
                )
                registry._save()
            queue_id = registry.optimization_queue[0]["queue_id"]

            skills = [s.to_dict() for s in skill_store.query_applicable(
                task_name=config.task_name,
                available_methods=cap_check.discovered.allowed_methods,
                sdk_contract_sha256=sdk_contract_sha256,
            )]
            opt_prompt = (
                "Improve the following verified single-cup coffee policy to minimize unnecessary "
                "redundant sdk.observe_scene() calls while strictly preserving ALL safety gates, "
                "recentering on L2 offset, and query_command_status on unknown brew receipt.\n"
                + EDGE_SANDBOX_CONTRACT_NOTES
                + "\nVerified skill hints:\n"
                + json.dumps(skills, sort_keys=True)
                + "\nBaseline policy:\n```python\n"
                + v1_source
                + "\n```\nOutput ONLY a single ```python ... ``` block defining def run_policy(sdk, params)."
            )
            opt_gen = gen(config, opt_prompt)
            cand2_source = str(opt_gen["source"]).strip()
            if cand2_source.startswith("```python\n") and cand2_source.endswith("```"):
                cand2_source = cand2_source[len("```python\n") : -3].strip()
            (ws / "improvement_candidate.py").write_text(cand2_source + "\n", encoding="utf-8")

            cand2_artifact = FrozenArtifact.freeze(
                policy_source=cand2_source,
                config_sha256=config.config_sha256,
                sdk_contract_sha256=sdk_contract_sha256,
                criteria=config.acceptance,
                deployment_scope=config.budget.deployment_scope,
            )
            cand2_outcomes, cand2_raw = run_heldout("v2_cand", cand2_source)
            _write_json(ws / "v2_heldout_runs.json", {"runs": cand2_raw})
            cand2_report = evaluate_acceptance(cand2_artifact, config.acceptance, cand2_outcomes, report_id="acc-v2-cand")
            _write_json(ws / "v2_acceptance_report.json", cand2_report.to_dict())

            improvement_result = registry.evaluate_improvement(
                queue_id=queue_id,
                baseline_report=v1_report,
                candidate_artifact=cand2_artifact,
                candidate_report=cand2_report,
                new_version="v2",
            )

        summary = {
            "status": "completed",
            "evidence_source": config.budget.deployment_scope,
            "sdk_contract_sha256": sdk_contract_sha256,
            "oven_gap": oven_gap,
            "dev_budget_used": dev_loop.state["budget_used"],
            "candidates_generated": len(dev_loop.candidates),
            "dev_trials": len(dev_loop.trials),
            "v1_acceptance": {
                "report_id": v1_report.report_id,
                "passed": v1_report.passed,
                "completion_rate": v1_report.completion_rate,
                "mean_wall_s": round(v1_report.mean_wall_s, 3),
                "max_delivery_error_mm": v1_report.max_delivery_error_mm,
                "unknown_outcomes": v1_report.unknown_outcomes,
                "duplicate_non_idempotent_count": v1_report.duplicate_non_idempotent_count,
                "certification_disclaimer": v1_report.certification_disclaimer,
            },
            "published_version": registry.active_version_id,
            "improvement_result": improvement_result,
            "finished_at_utc": _utc_now(),
        }
        _write_json(ws / "summary.json", summary)
        return summary
