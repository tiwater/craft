"""End-to-end commercial operations loop: Day-Shift -> Nightly Repair -> Acceptance -> Release -> Skills.

Orchestrates TRAX's production lifecycle strictly over ``SceneHostDriver`` and
existing TRAX governance modules (zero perception/control duplication):

1. **Day Shift (``run_continuous_shift``)**:
   Runs a multi-order commercial queue under a fixed frozen policy (verified against
   ``ReleaseRegistry`` when present), backed by SQLite ``Journal`` intent-before-dispatch
   across simulated mid-order crashes/restarts. Automatically captures every failure or
   unexpected human intervention into ``RegressionSuite`` and computes shift throughput.
2. **Recorded-Data Shadow Check (``run_recorded_shadow_check``)**:
   Replays recorded perception frames via ``RecordedReplayDriver`` (``evidence_source='recorded_data'``,
   zero physical actuation) to verify fail-closed safety under noisy/stale/ambiguous/L2-unsafe frames.
3. **Nightly Evolution (``run_nightly_evolution``)**:
   Takes captured ``RegressionCase`` defects -> runs ``RepairScreener`` (replay alignment +
   multi-hypothesis generation + full regression screening) -> runs recorded-data shadow check ->
   runs independent ``evaluate_acceptance`` on held-out scenarios -> publishes a frozen new
   release to ``ReleaseRegistry`` (with rollback history) -> stores a verified ``SkillExperience``
   carrying ``measured_limits``, ``known_failure_modes``, and ``regression_case_ids``.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
from pathlib import Path
import time
from typing import Any, Mapping, Sequence

from .acceptance import (
    FrozenArtifact,
    HeldoutTrialOutcome,
    evaluate_acceptance,
)
from .config import SceneConfig
from .contracts import digest
from .dev_loop import GeneratorFn
from .host_driver import SceneHostDriver
from .journal import Journal
from .portability import check_policy_portability
from .profiler import aggregate_trial_profiles, attribute_trial
from .recorded_driver import RecordedDataHostDriver
from .regression import RegressionCase, RegressionSuite
from .release import ReleaseRegistry
from .repair_screener import RepairHypothesis, RepairScreener
from .skills import SkillExperience, SkillExperienceStore


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _write_json_atomic(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    tmp.replace(path)


def default_recorded_shadow_scenarios() -> list[dict[str, Any]]:
    """Curated recorded-frame shadow scenarios (evidence_source='recorded_data', 0 physical commands)."""
    return [
        {
            "scenario_id": "rec_01_stale_not_ready",
            "label": "Recorded stale perception (control_ready=False)",
            "expect_status": "rejected_observation_not_ready",
            "frames": [
                {
                    "control_ready": False,
                    "pipeline_health": {"source": "recorded_camera_stream", "ok": False, "stale": True},
                    "objects": [],
                    "l2_pouring_safety": {"safe_to_pour": False},
                }
            ],
        },
        {
            "scenario_id": "rec_02_low_conf_noisy_cup",
            "label": "Recorded occluded/low-confidence cup (0.41 < 0.75)",
            "expect_status": "paused_for_operator",
            "frames": [
                {
                    "control_ready": True,
                    "objects": [
                        {
                            "id": "rec-cup-1",
                            "cup_type": "dine_in_red",
                            "confidence": 0.41,
                            "world": [0.33, -0.01, 1.075],
                            "tilt_deg": 2.0,
                        }
                    ],
                    "l2_pouring_safety": {"safe_to_pour": True},
                    "confidence_threshold": 0.75,
                }
            ],
        },
        {
            "scenario_id": "rec_03_ambiguous_dual_cups",
            "label": "Recorded dual ambiguous rigid cups",
            "expect_status": "paused_for_operator",
            "frames": [
                {
                    "control_ready": True,
                    "objects": [
                        {"id": "rec-cup-a", "cup_type": "dine_in_red", "confidence": 0.91, "world": [0.32, -0.05, 1.075]},
                        {"id": "rec-cup-b", "cup_type": "dine_in_red", "confidence": 0.89, "world": [0.34, 0.06, 1.075]},
                    ],
                    "l2_pouring_safety": {"safe_to_pour": True},
                    "confidence_threshold": 0.75,
                }
            ],
        },
        {
            "scenario_id": "rec_04_distractor_bag_only",
            "label": "Recorded coffee-bean bag distractor only (no rigid cup)",
            "expect_status": "paused_for_operator",
            "acceptable_statuses": ["paused_for_operator", "rejected_observation_not_ready", "escalated_to_human", "failed"],
            "frames": [
                {
                    "control_ready": True,
                    "objects": [
                        {"id": "rec-bag-1", "cup_type": "flexible_bag", "confidence": 0.95, "world": [0.30, 0.04, 1.03]},
                    ],
                    "l2_pouring_safety": {"safe_to_pour": True},
                    "confidence_threshold": 0.75,
                }
            ],
        },
    ]


def run_recorded_shadow_check(
    config: SceneConfig,
    policy_source: str,
    *,
    shadow_scenarios: Sequence[Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    """Run zero-actuation recorded-data shadow replay across noisy/fail-closed frame streams."""
    cases = list(shadow_scenarios or default_recorded_shadow_scenarios())
    results: list[dict[str, Any]] = []
    passed_count = 0
    false_actuations = 0

    for item in cases:
        drv = RecordedDataHostDriver(config, frames=list(item.get("frames") or []))
        scen = {
            "order_id": str(item.get("order_id", "A042")),
            "target_slot": str(item.get("target_slot", "$SLOT_PICKUP_1")),
            "expect_status": str(item.get("expect_status", "paused_for_operator")),
        }
        res = drv.run_policy_scenario(
            policy_source,
            command_id=f"shadow-{item['scenario_id']}-{time.time_ns()}",
            scenario=scen,
        )
        host_st = str(res.get("host_runtime_status") or "")
        brews = int((res.get("non_idempotent_counts") or {}).get("start_coffee_machine", 0))
        acceptable = set(item.get("acceptable_statuses") or [scen["expect_status"]])
        if scen["expect_status"] == "paused_for_operator":
            acceptable.add("rejected_observation_not_ready")
        ok = (host_st in acceptable) and (brews == 0 if scen["expect_status"] != "completed" else True)
        if scen["expect_status"] != "completed" and brews > 0:
            false_actuations += 1
        if ok:
            passed_count += 1
        results.append(
            {
                "scenario_id": item["scenario_id"],
                "label": item.get("label", item["scenario_id"]),
                "expect_status": scen["expect_status"],
                "host_runtime_status": res.get("host_runtime_status"),
                "task_passed": ok,
                "shadow_brew_intents": brews,
                "evidence_source": "recorded_data",
            }
        )

    return {
        "evidence_source": "recorded_data",
        "total": len(cases),
        "passed": passed_count,
        "false_actuations": false_actuations,
        "all_passed": passed_count == len(cases) and false_actuations == 0,
        "trials": results,
    }


def default_day_shift_orders() -> list[dict[str, Any]]:
    """Representative 6-order commercial shift queue including crash/restart, busy machine, layout shift & redlines."""
    return [
        {
            "shift_order_id": "shift_01_nominal",
            "label": "Order 1: Standard Hot Americano",
            "preset": "default",
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "completed",
        },
        {
            "shift_order_id": "shift_02_store_b_shift",
            "label": "Order 2: Cross-Store Layout Shift (+25,-25mm) Takeout",
            "preset": "default",
            "order_id": "A043",
            "target_slot": "$SLOT_PICKUP_2",
            "cup_class": "takeout_green",
            "store_layout_shift_xy_m": [0.025, -0.025],
            "expect_status": "completed",
        },
        {
            "shift_order_id": "shift_03_slip_timeout",
            "label": "Order 3: Placement Slip (32,-12mm) + Brew Ack Timeout",
            "preset": "default",
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "fault_inject_slip_xy_m": [0.032, -0.012],
            "simulate_timeout": True,
            "expect_status": "completed",
        },
        {
            "shift_order_id": "shift_04_mid_brew_crash_restart",
            "label": "Order 4: Mid-Brew Host Crash & Journaled Restart",
            "preset": "default",
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "simulate_crash_mid_brew": True,
            "expect_status": "completed",
        },
        {
            "shift_order_id": "shift_05_ambiguity_pause",
            "label": "Order 5: Dual-Cup Ambiguity (Must Pause, 0 Brew)",
            "preset": "sim_dual_cup",
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "paused_for_operator",
        },
        {
            "shift_order_id": "shift_06_hazard_nogo",
            "label": "Order 6: Forbidden Steam Zone (Must Escalate, 0 Brew)",
            "preset": "default",
            "order_id": "A042",
            "target_slot": "$ZONE_STEAM_NOGO",
            "expect_status": "escalated_to_human",
        },
    ]


def _run_driver_order(
    driver: SceneHostDriver,
    policy_source: str,
    *,
    command_id: str,
    scenario: Mapping[str, Any],
    journal: Journal | None,
    enable_journal_proxy: bool,
    journal_run_id: str | None = None,
) -> dict[str, Any]:
    """Execute one order on ``driver``, passing journal proxy kwargs when supported."""
    try:
        return driver.run_policy_scenario(
            policy_source,
            command_id=command_id,
            scenario=scenario,
            journal=journal,
            enable_journal_proxy=enable_journal_proxy,  # type: ignore[call-arg]
            journal_run_id=journal_run_id,  # type: ignore[call-arg]
        )
    except TypeError:
        return driver.run_policy_scenario(
            policy_source,
            command_id=command_id,
            scenario=scenario,
            journal=journal,
        )


def run_continuous_shift(
    driver: SceneHostDriver,
    config: SceneConfig,
    workspace_dir: str | Path,
    *,
    policy_source: str,
    orders: Sequence[Mapping[str, Any]] | None = None,
    shift_id: str | None = None,
    release_registry: ReleaseRegistry | None = None,
    regression_suite: RegressionSuite | None = None,
    enable_journal_proxy: bool = True,
    verify_frozen_release: bool = False,
) -> dict[str, Any]:
    """Execute a multi-order continuous shift under a fixed policy with crash/restart support."""
    ws = Path(workspace_dir)
    ws.mkdir(parents=True, exist_ok=True)
    sid = shift_id or f"shift-{int(time.time())}"
    queue = list(orders or default_day_shift_orders())

    if verify_frozen_release and release_registry is not None and release_registry.active_version_id:
        cap = driver.discover_capabilities(allow_busy=True)
        sdk_sha = cap.discovered.sdk_contract_sha256 if cap.discovered else ""
        release_registry.verify_fixed_execution(
            policy_source,
            config_sha256=config.config_sha256,
            sdk_contract_sha256=sdk_sha,
        )

    journal_path = ws / "shifts" / f"{sid}_journal.sqlite"
    journal_path.parent.mkdir(parents=True, exist_ok=True)

    t0 = time.monotonic()
    trials: list[dict[str, Any]] = []
    captured_ids: list[str] = []
    completed_orders = 0
    duplicate_non_idempotent_total = 0
    unreconciled_unknown_total = 0
    false_actuations = 0

    with Journal(journal_path) as journal:
        active_journal = journal if enable_journal_proxy else None
        for idx, raw_order in enumerate(queue, start=1):
            scen = dict(raw_order)
            order_tag = str(scen.get("shift_order_id") or f"order_{idx:02d}")
            cmd_id_att1 = f"{sid}-{order_tag}-att1"
            logical_run_id = f"{sid}-{order_tag}"
            driver.reset_scenario(scen)

            res = _run_driver_order(
                driver,
                policy_source,
                command_id=cmd_id_att1,
                scenario=scen,
                journal=active_journal,
                enable_journal_proxy=enable_journal_proxy,
                journal_run_id=logical_run_id,
            )

            # If a simulated mid-brew crash occurred, restart process WITHOUT resetting physical hardware.
            # The restarted process receives a fresh attempt prefix (-att2); only a durable SQLite Journal
            # keyed on logical_run_id retains the crashed process's committed command_id.
            restarted = False
            if bool(res.get("crashed_mid_brew")) or (
                bool(scen.get("simulate_crash_mid_brew")) and res.get("host_runtime_status") in {"crashed_mid_brew", "exception"}
            ):
                restarted = True
                resume_scen = dict(scen)
                resume_scen["resume_after_crash"] = True
                resume_scen["simulate_crash_mid_brew"] = False
                res = _run_driver_order(
                    driver,
                    policy_source,
                    command_id=f"{sid}-{order_tag}-att2",
                    scenario=resume_scen,
                    journal=active_journal,
                    enable_journal_proxy=enable_journal_proxy,
                    journal_run_id=logical_run_id,
                )

            attr = attribute_trial(res).to_dict()
            passed = bool(res.get("task_passed", False))
            dup = bool(res.get("duplicate_non_idempotent", False))
            unrec = int(res.get("unknown_events_observed", 0)) > 0 and not bool(res.get("queried_after_unknown", True))
            expect_status = str(scen.get("expect_status", "completed"))
            brews = int((res.get("non_idempotent_counts") or {}).get("start_coffee_machine", 0))
            if expect_status != "completed" and not scen.get("contradictory_receipt") and brews > 0:
                false_actuations += 1
            if passed:
                completed_orders += 1
            if dup:
                duplicate_non_idempotent_total += 1
            if unrec:
                unreconciled_unknown_total += 1

            captured_case_id = None
            if regression_suite is not None:
                case = regression_suite.capture_failure(res, scenario_context=scen)
                if case is not None:
                    captured_case_id = case.case_id
                    if case.case_id not in captured_ids:
                        captured_ids.append(case.case_id)

            trials.append(
                {
                    "shift_order_id": order_tag,
                    "label": scen.get("label", order_tag),
                    "restarted_after_crash": restarted,
                    "attribution": attr,
                    "regression_case_id": captured_case_id,
                    **res,
                }
            )

    total_wall_s = round(time.monotonic() - t0, 3)
    orders_per_hour = round((completed_orders / total_wall_s) * 3600.0, 1) if total_wall_s > 0 else 0.0
    report = {
        "schema_version": 1,
        "shift_id": sid,
        "driver": driver.driver_name,
        "evidence_source": driver.evidence_source,
        "policy_sha256": digest(policy_source),
        "enable_journal_proxy": enable_journal_proxy,
        "total_orders": len(queue),
        "passed_orders": completed_orders,
        "pass_rate": round(completed_orders / len(queue), 4) if queue else 0.0,
        "duplicate_non_idempotent_orders": duplicate_non_idempotent_total,
        "unreconciled_unknown_orders": unreconciled_unknown_total,
        "false_actuation_orders": false_actuations,
        "captured_regression_case_ids": captured_ids,
        "total_wall_s": total_wall_s,
        "throughput_orders_per_hour": orders_per_hour,
        "portability": check_policy_portability(policy_source),
        "aggregate_profile": aggregate_trial_profiles(trials),
        "trials": trials,
        "finished_at_utc": _utc_now(),
    }
    _write_json_atomic(ws / "shifts" / f"{sid}.json", report)
    _write_json_atomic(ws / "latest_shift_report.json", report)
    return report


def run_nightly_evolution(
    driver: SceneHostDriver,
    config: SceneConfig,
    workspace_dir: str | Path,
    *,
    baseline_source: str,
    target_case: RegressionCase,
    regression_suite: RegressionSuite,
    release_registry: ReleaseRegistry,
    skill_store: SkillExperienceStore,
    heldout_scenarios: Sequence[Mapping[str, Any]],
    new_version: str,
    hypotheses: Sequence[RepairHypothesis] | None = None,
    candidate_sources: Mapping[str, str] | None = None,
    generator_fn: GeneratorFn | None = None,
    run_shadow_check: bool = True,
    max_candidates: int = 3,
    skill_measured_limits: Mapping[str, Any] | None = None,
    skill_known_failure_modes: Sequence[str] = (),
) -> dict[str, Any]:
    """Full nightly closed-loop evolution:
    RepairScreener -> Recorded-Data Shadow Check -> Independent Acceptance -> Release -> SkillExperience.
    """
    ws = Path(workspace_dir)
    screener = RepairScreener(ws, config, driver, regression_suite=regression_suite)
    screening = screener.screen_repair(
        baseline_source=baseline_source,
        case=target_case,
        hypotheses=hypotheses,
        candidate_sources=candidate_sources,
        generator_fn=generator_fn,
        max_candidates=max_candidates,
    )

    if screening.get("status") != "completed_with_recommendation" or not screening.get("recommended_candidate_id"):
        result = {
            "status": "no_promotable_candidate",
            "reason": screening.get("status"),
            "screening": screening,
            "shadow_check": None,
            "acceptance_report": None,
            "published_release": None,
            "skill_recorded": None,
        }
        _write_json_atomic(ws / "latest_nightly_evolution.json", result)
        return result

    cand_id = str(screening["recommended_candidate_id"])
    cand_source = screener.get_candidate_source(str(screening["screening_id"]), cand_id)

    # 1) Recorded-data shadow safety check (evidence_source='recorded_data', 0 actuation)
    shadow = run_recorded_shadow_check(config, cand_source) if run_shadow_check else None
    if shadow is not None and not shadow["all_passed"]:
        result = {
            "status": "blocked_by_recorded_shadow_check",
            "screening": screening,
            "shadow_check": shadow,
            "acceptance_report": None,
            "published_release": None,
            "skill_recorded": None,
        }
        _write_json_atomic(ws / "latest_nightly_evolution.json", result)
        return result

    # 2) Independent acceptance gate on held-out scenarios
    cap = driver.discover_capabilities(allow_busy=True)
    sdk_sha = cap.discovered.sdk_contract_sha256 if cap.discovered else "unknown"
    artifact = FrozenArtifact.freeze(
        policy_source=cand_source,
        config_sha256=config.config_sha256,
        sdk_contract_sha256=sdk_sha,
        criteria=config.acceptance,
        deployment_scope=driver.evidence_source,
    )

    acc_trials: list[HeldoutTrialOutcome] = []
    journal_path = ws / "nightly_acceptance_journal.sqlite"
    with Journal(journal_path) as journal:
        for idx, raw_scen in enumerate(heldout_scenarios, start=1):
            scen = dict(raw_scen)
            driver.reset_scenario(scen)
            cmd_id = f"acc-{new_version}-{idx:02d}-{time.time_ns()}"
            res = _run_driver_order(
                driver,
                cand_source,
                command_id=f"{cmd_id}-att1",
                scenario=scen,
                journal=journal,
                enable_journal_proxy=True,
                journal_run_id=cmd_id,
            )
            if bool(res.get("crashed_mid_brew")) or (
                bool(scen.get("simulate_crash_mid_brew")) and res.get("host_runtime_status") in {"crashed_mid_brew", "exception"}
            ):
                resume_scen = dict(scen)
                resume_scen["resume_after_crash"] = True
                resume_scen["simulate_crash_mid_brew"] = False
                res = _run_driver_order(
                    driver,
                    cand_source,
                    command_id=f"{cmd_id}-att2",
                    scenario=resume_scen,
                    journal=journal,
                    enable_journal_proxy=True,
                    journal_run_id=cmd_id,
                )
            unrec = int(res.get("unknown_events_observed", 0)) > 0 and not bool(res.get("queried_after_unknown", True))
            acc_trials.append(
                HeldoutTrialOutcome(
                    trial_id=f"acc_{idx:02d}",
                    condition=str(scen.get("scenario_id") or scen.get("label") or f"heldout_{idx:02d}"),
                    evidence_source=str(res.get("evidence_source", driver.evidence_source)),
                    used_in_debugging=False,
                    completed=bool(res.get("task_passed", False)),
                    wall_s=float(res.get("wall_s", 0.0)),
                    unknown_outcome=unrec,
                    duplicate_non_idempotent=bool(res.get("duplicate_non_idempotent", False)),
                    human_interventions=int(res.get("human_interventions", 0)),
                    delivery_error_mm=(res.get("oracle_post_run") or {}).get("delivery_error_mm"),
                )
            )

    acc_report = evaluate_acceptance(artifact, config.acceptance, acc_trials)
    if not acc_report.passed:
        result = {
            "status": "blocked_by_independent_acceptance",
            "screening": screening,
            "shadow_check": shadow,
            "acceptance_report": acc_report.to_dict(),
            "published_release": None,
            "skill_recorded": None,
        }
        _write_json_atomic(ws / "latest_nightly_evolution.json", result)
        return result

    # 3) Publish frozen release to ReleaseRegistry
    published = release_registry.publish(artifact, acc_report, version=new_version)

    # 4) Record verified SkillExperience with measured limits & regression anchors
    chosen_cand = next(c for c in screening["candidates"] if c["candidate_id"] == cand_id)
    measured = dict(skill_measured_limits or {})
    if target_case.scenario.fault_inject_slip_xy_m is not None:
        sx, sy = target_case.scenario.fault_inject_slip_xy_m
        measured.setdefault("verified_slip_mm", round(((sx * sx + sy * sy) ** 0.5) * 1000.0, 1))
    measured.setdefault("screening_mean_wall_s", round(float(chosen_cand.get("mean_wall_s", 0.0)), 4))
    measured.setdefault("acceptance_completion_rate", acc_report.completion_rate)

    skill = SkillExperience(
        skill_id=f"skill-{new_version}-{target_case.case_id}",
        title=f"Verified repair for {target_case.attribution_code} ({new_version})",
        guidance=f"Hypothesis '{chosen_cand['hypothesis_title']}' verified on {target_case.scenario.label} with 0 regressions.",
        task_name=config.task_name,
        required_methods=config.capabilities.required_methods,
        sdk_contract_sha256=sdk_sha,
        evidence_source=driver.evidence_source,
        verification_report_id=acc_report.report_id,
        verified=True,
        verified_at_utc=_utc_now(),
        applicability_notes=f"Promoted via RepairScreener ({screening['screening_id']}) + independent acceptance ({acc_report.report_id}).",
        measured_limits=measured,
        known_failure_modes=tuple(skill_known_failure_modes) or (target_case.attribution_code,),
        regression_case_ids=tuple(c.case_id for c in regression_suite.cases),
    )
    skill_store.add(skill)

    result = {
        "status": "promoted",
        "screening": screening,
        "shadow_check": shadow,
        "acceptance_report": acc_report.to_dict(),
        "published_release": published.to_dict(),
        "skill_recorded": skill.to_dict(),
        "finished_at_utc": _utc_now(),
    }
    _write_json_atomic(ws / "latest_nightly_evolution.json", result)
    return result
