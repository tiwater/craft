"""Simulation-assisted replay-alignment and multi-hypothesis repair screening.

Adapts the Stage-2 replay-and-screen methodology from SimEX (``hu2026simex``) strictly
within CRAFT's ownership, budget, and independent-acceptance boundaries:

1. **Ownership guard**: Only ``policy``-layer defects are eligible for automated code
   repair by default; ``perception``, ``control``, and ``device`` failures are surfaced
   to their respective host owners rather than papered over with policy hacks.
2. **Replay alignment check**: Before generating any repair candidate, replay the
   failing baseline policy on the supplied simulation ``SceneHostDriver``. If the
   simulator does not reproduce the failure, stop honestly (``not_reproduced_in_sim``)
   instead of screening repairs against an unaligned simulation.
3. **Multi-hypothesis candidate generation & screening**: Generate up to ``K`` distinct
   diagnostic hypotheses and candidate repairs within explicit model/trial budgets,
   then screen every candidate against both the reproduced failure scenario AND all
   accumulated ``RegressionSuite`` cases (capability-bank replay).
4. **Evidence separation & independent acceptance**: Every screening report carries
   ``evidence_source=driver.evidence_source``, ``approved=False``, and
   ``requires_independent_acceptance=True``. Simulator screening eliminates bad patches
   offline; it never substitutes for independent acceptance or physical approval.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import json
from pathlib import Path
import time
from typing import Any, Mapping, Sequence

from .config import SceneConfig
from .contracts import canonical, digest
from .dev_loop import (
    GeneratorFn,
    _strip_code_fence,
    default_live_generator,
    validate_host_policy_syntax,
)
from .host_driver import SceneHostDriver
from .journal import Journal
from .prior_work_bench import check_sandbox_ast_compliance
from .probing import build_llm_hypothesis_synthesis_prompt, parse_llm_hypotheses_json
from .profiler import attribute_trial
from .regression import RegressionCase, RegressionSuite


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _write_json_atomic(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    tmp.replace(path)


@dataclass(frozen=True)
class RepairHypothesis:
    """One concrete, testable diagnostic hypothesis for a policy-layer defect."""

    hypothesis_id: str
    title: str
    diagnosis: str
    proposed_strategy: str
    target_layer: str = "policy"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> RepairHypothesis:
        return cls(
            hypothesis_id=str(data["hypothesis_id"]),
            title=str(data["title"]),
            diagnosis=str(data["diagnosis"]),
            proposed_strategy=str(data["proposed_strategy"]),
            target_layer=str(data.get("target_layer", "policy")),
        )


@dataclass(frozen=True)
class ReplayAlignmentResult:
    """Outcome of replaying the failing baseline policy on the simulation driver."""

    case_id: str
    baseline_sha256: str
    driver_name: str
    evidence_source: str
    reproduced: bool
    attribution_matched: bool
    reproduction_status: str  # reproduced_exact | reproduced_different_mode | not_reproduced_in_sim
    original_attribution_layer: str
    original_attribution_code: str
    baseline_attribution: Mapping[str, Any]
    baseline_trial: Mapping[str, Any]
    sim_trust_notes: str
    checked_at_utc: str = field(default_factory=_utc_now)
    # P4-2: Closed-loop Sim-to-Sim / Real-to-Sim parameter alignment provenance
    alignment_method: str = "direct_replay"  # direct_replay | calibrated_sim_search
    calibration_trials_used: int = 0
    calibrated_scenario: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "case_id": self.case_id,
            "baseline_sha256": self.baseline_sha256,
            "driver_name": self.driver_name,
            "evidence_source": self.evidence_source,
            "reproduced": self.reproduced,
            "attribution_matched": self.attribution_matched,
            "reproduction_status": self.reproduction_status,
            "original_attribution_layer": self.original_attribution_layer,
            "original_attribution_code": self.original_attribution_code,
            "baseline_attribution": dict(self.baseline_attribution),
            "baseline_trial": dict(self.baseline_trial),
            "sim_trust_notes": self.sim_trust_notes,
            "checked_at_utc": self.checked_at_utc,
            "alignment_method": self.alignment_method,
            "calibration_trials_used": self.calibration_trials_used,
            "calibrated_scenario": dict(self.calibrated_scenario),
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> ReplayAlignmentResult:
        return cls(
            case_id=str(data["case_id"]),
            baseline_sha256=str(data["baseline_sha256"]),
            driver_name=str(data["driver_name"]),
            evidence_source=str(data["evidence_source"]),
            reproduced=bool(data["reproduced"]),
            attribution_matched=bool(data["attribution_matched"]),
            reproduction_status=str(data["reproduction_status"]),
            original_attribution_layer=str(data["original_attribution_layer"]),
            original_attribution_code=str(data["original_attribution_code"]),
            baseline_attribution=dict(data.get("baseline_attribution") or {}),
            baseline_trial=dict(data.get("baseline_trial") or {}),
            sim_trust_notes=str(data.get("sim_trust_notes", "")),
            checked_at_utc=str(data.get("checked_at_utc") or _utc_now()),
            alignment_method=str(data.get("alignment_method", "direct_replay")),
            calibration_trials_used=int(data.get("calibration_trials_used", 0)),
            calibrated_scenario=dict(data.get("calibrated_scenario") or {}),
        )


@dataclass(frozen=True)
class CandidateScreeningResult:
    """Screening evidence for a single repair candidate across target + regression suite."""

    candidate_id: str
    hypothesis_id: str
    hypothesis_title: str
    source_sha256: str
    source_path: str
    evidence_source: str
    syntax_valid: bool
    validation_error: str
    target_fixed: bool
    target_trial: Mapping[str, Any]
    regression_total: int
    regression_passed: int
    regression_failed_ids: tuple[str, ...]
    duplicate_non_idempotent: bool
    unreconciled_unknown_ack: bool
    mean_wall_s: float
    eligible_for_acceptance: bool
    rejection_reasons: tuple[str, ...]
    approved: bool = False
    screened_at_utc: str = field(default_factory=_utc_now)

    def to_dict(self) -> dict[str, Any]:
        return {
            "candidate_id": self.candidate_id,
            "hypothesis_id": self.hypothesis_id,
            "hypothesis_title": self.hypothesis_title,
            "source_sha256": self.source_sha256,
            "source_path": self.source_path,
            "evidence_source": self.evidence_source,
            "syntax_valid": self.syntax_valid,
            "validation_error": self.validation_error,
            "target_fixed": self.target_fixed,
            "target_trial": dict(self.target_trial),
            "regression_total": self.regression_total,
            "regression_passed": self.regression_passed,
            "regression_failed_ids": list(self.regression_failed_ids),
            "duplicate_non_idempotent": self.duplicate_non_idempotent,
            "unreconciled_unknown_ack": self.unreconciled_unknown_ack,
            "mean_wall_s": round(self.mean_wall_s, 4),
            "eligible_for_acceptance": self.eligible_for_acceptance,
            "rejection_reasons": list(self.rejection_reasons),
            "approved": False,
            "screened_at_utc": self.screened_at_utc,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> CandidateScreeningResult:
        return cls(
            candidate_id=str(data["candidate_id"]),
            hypothesis_id=str(data["hypothesis_id"]),
            hypothesis_title=str(data["hypothesis_title"]),
            source_sha256=str(data["source_sha256"]),
            source_path=str(data["source_path"]),
            evidence_source=str(data["evidence_source"]),
            syntax_valid=bool(data["syntax_valid"]),
            validation_error=str(data.get("validation_error", "")),
            target_fixed=bool(data["target_fixed"]),
            target_trial=dict(data.get("target_trial") or {}),
            regression_total=int(data.get("regression_total", 0)),
            regression_passed=int(data.get("regression_passed", 0)),
            regression_failed_ids=tuple(str(x) for x in (data.get("regression_failed_ids") or ())),
            duplicate_non_idempotent=bool(data.get("duplicate_non_idempotent", False)),
            unreconciled_unknown_ack=bool(data.get("unreconciled_unknown_ack", False)),
            mean_wall_s=float(data.get("mean_wall_s", 0.0)),
            eligible_for_acceptance=bool(data.get("eligible_for_acceptance", False)),
            rejection_reasons=tuple(str(x) for x in (data.get("rejection_reasons") or ())),
            approved=False,
            screened_at_utc=str(data.get("screened_at_utc") or _utc_now()),
        )


def default_hypotheses_for_case(
    case: RegressionCase,
    *,
    max_hypotheses: int = 3,
) -> list[RepairHypothesis]:
    """Generate distinct, layer-aware repair hypotheses for a captured failure case."""
    code = case.attribution_code
    layer = case.attribution_layer
    if layer != "policy":
        return []

    if code == "missing_recenter_branch":
        pool = [
            RepairHypothesis(
                hypothesis_id="hyp_recenter_gate",
                title="Pre-brew L2 safety check + recenter",
                diagnosis="Policy invoked start_coffee_machine without checking l2_pouring_safety after cup placement.",
                proposed_strategy=(
                    "Re-observe scene immediately after place_cup_under_machine; if l2_pouring_safety.safe_to_pour "
                    "is False, call recenter_cup_under_spout and re-verify before starting brew."
                ),
            ),
            RepairHypothesis(
                hypothesis_id="hyp_recenter_and_reconcile",
                title="Defensive recenter + unknown-receipt reconciliation",
                diagnosis="Placement slip may combine with delayed brew acknowledgements during fault injection.",
                proposed_strategy=(
                    "Verify l2_pouring_safety after placement, invoke recenter_cup_under_spout when unsafe, "
                    "and reconcile any unknown start_coffee_machine receipt via query_command_status."
                ),
            ),
            RepairHypothesis(
                hypothesis_id="hyp_recenter_escalate_fallback",
                title="Recenter with fail-closed operator escalation",
                diagnosis="If recentering fails to restore safe_to_pour, policy must escalate instead of brewing.",
                proposed_strategy=(
                    "Attempt recenter_cup_under_spout once when safe_to_pour is False, re-observe, and escalate "
                    "to human operator without starting brew if L2 safety remains false."
                ),
            ),
        ]
    elif code in {"unreconciled_unknown_ack", "duplicate_non_idempotent"}:
        pool = [
            RepairHypothesis(
                hypothesis_id="hyp_query_first",
                title="Query-first reconciliation on unknown receipt",
                diagnosis="Policy either failed to query an unknown acknowledgement or redispatched a non-idempotent command.",
                proposed_strategy=(
                    "Capture command_id for start_coffee_machine; whenever receipt is 'unknown', call "
                    "query_command_status(command_id) instead of reissuing start_coffee_machine."
                ),
            ),
            RepairHypothesis(
                hypothesis_id="hyp_single_dispatch_guard",
                title="Single-dispatch guard with fail-closed escalation",
                diagnosis="Repeated brew dispatch or missing reconciliation violates non-idempotent safety invariants.",
                proposed_strategy=(
                    "Enforce at most one start_coffee_machine call per order; on unknown receipt, poll "
                    "query_command_status and escalate to human operator if status remains unresolved."
                ),
            ),
            RepairHypothesis(
                hypothesis_id="hyp_state_verified_resume",
                title="Scene + command status cross-check before wait",
                diagnosis="Ambiguous brew receipts require reconciling both command status and machine state.",
                proposed_strategy=(
                    "Query command_status on unknown receipt and confirm machine status is running/completed "
                    "before calling wait_for_coffee_ready; never issue a second start_coffee_machine."
                ),
            ),
        ]
    else:
        pool = [
            RepairHypothesis(
                hypothesis_id="hyp_targeted_patch",
                title="Targeted policy branch fix",
                diagnosis=f"Policy failed with attribution code '{code}' on scenario '{case.scenario.label}'.",
                proposed_strategy=(
                    "Fix the failing control-flow branch while preserving all existing safety checks, "
                    "L2 pouring guards, and non-idempotent command reconciliation."
                ),
            ),
            RepairHypothesis(
                hypothesis_id="hyp_defensive_state_checks",
                title="Defensive pre-action observation & fail-closed guards",
                diagnosis=f"Policy lacked defensive observation or receipt validation for '{code}'.",
                proposed_strategy=(
                    "Check observe_scene() readiness, object ambiguity, L2 safety, and every action receipt "
                    "explicitly, escalating cleanly whenever preconditions are not satisfied."
                ),
            ),
            RepairHypothesis(
                hypothesis_id="hyp_minimal_recovery_cleanup",
                title="Minimal structured recovery preserving throughput",
                diagnosis=f"Failure '{code}' requires a minimal localized fix without adding unnecessary steps.",
                proposed_strategy=(
                    "Apply the smallest possible fix to resolve the reproduced failure while keeping all "
                    "regression cases passing and avoiding redundant host calls."
                ),
            ),
        ]

    return pool[: max(1, int(max_hypotheses))]


def _run_case_scenario(
    driver: SceneHostDriver,
    source: str,
    scenario: Mapping[str, Any],
    *,
    command_prefix: str,
    journal: Journal | None = None,
    enable_journal_proxy: bool = False,
) -> dict[str, Any]:
    """Execute a scenario on ``driver``, faithfully replaying mid-brew crash/restart if configured."""
    scen = dict(scenario)
    driver.reset_scenario(scen)
    att1 = f"{command_prefix}-att1"
    active_journal = journal if enable_journal_proxy else None
    try:
        res = driver.run_policy_scenario(
            source,
            command_id=att1,
            scenario=scen,
            journal=active_journal,
            enable_journal_proxy=enable_journal_proxy,  # type: ignore[call-arg]
            journal_run_id=command_prefix,  # type: ignore[call-arg]
        )
    except TypeError:
        res = driver.run_policy_scenario(source, command_id=att1, scenario=scen, journal=active_journal)

    if bool(res.get("crashed_mid_brew")) or (
        bool(scen.get("simulate_crash_mid_brew"))
        and res.get("host_runtime_status") in {"crashed_mid_brew", "exception"}
    ):
        resume_scen = dict(scen)
        resume_scen["resume_after_crash"] = True
        resume_scen["simulate_crash_mid_brew"] = False
        att2 = f"{command_prefix}-att2"
        try:
            res = driver.run_policy_scenario(
                source,
                command_id=att2,
                scenario=resume_scen,
                journal=active_journal,
                enable_journal_proxy=enable_journal_proxy,  # type: ignore[call-arg]
                journal_run_id=command_prefix,  # type: ignore[call-arg]
            )
        except TypeError:
            res = driver.run_policy_scenario(source, command_id=att2, scenario=resume_scen, journal=active_journal)
    return res


def verify_replay_alignment(
    driver: SceneHostDriver,
    baseline_source: str,
    case: RegressionCase,
) -> ReplayAlignmentResult:
    """Replay the failing baseline policy on ``driver`` to verify whether sim reproduces the defect."""
    scen_dict = case.to_scenario_dict()
    cmd_id = f"align-{case.case_id}-{time.time_ns()}"
    trial = _run_case_scenario(
        driver,
        baseline_source,
        scen_dict,
        command_prefix=cmd_id,
        journal=None,
        enable_journal_proxy=False,
    )
    attr = attribute_trial(trial)
    reproduced = not bool(trial.get("task_passed", False))
    attr_matched = reproduced and (attr.code == case.attribution_code)

    if not reproduced:
        rep_status = "not_reproduced_in_sim"
        trust = (
            f"Baseline policy PASSED on {driver.driver_name} ({driver.evidence_source}) instead of "
            f"reproducing failure '{case.attribution_code}'. Simulator lacks fidelity for this defect; "
            "do not rely on simulator screening alone."
        )
    elif attr_matched:
        rep_status = "reproduced_exact"
        trust = (
            f"Reproduced exact attribution '{attr.code}' ({attr.layer}) on {driver.driver_name} "
            f"({driver.evidence_source}). Screening evidence is informative for pre-filtering, "
            "but still requires independent acceptance."
        )
    else:
        rep_status = "reproduced_different_mode"
        trust = (
            f"Baseline failed on {driver.driver_name} ({driver.evidence_source}), but with "
            f"'{attr.code}' ({attr.layer}) instead of original '{case.attribution_code}' "
            f"({case.attribution_layer}). Treat screening scores with caution."
        )

    return ReplayAlignmentResult(
        case_id=case.case_id,
        baseline_sha256=digest(baseline_source),
        driver_name=driver.driver_name,
        evidence_source=driver.evidence_source,
        reproduced=reproduced,
        attribution_matched=attr_matched,
        reproduction_status=rep_status,
        original_attribution_layer=case.attribution_layer,
        original_attribution_code=case.attribution_code,
        baseline_attribution=attr.to_dict(),
        baseline_trial=trial,
        sim_trust_notes=trust,
    )


def calibrate_simulator_alignment(
    driver: SceneHostDriver,
    baseline_source: str,
    case: RegressionCase,
    *,
    max_calibration_trials: int = 6,
    candidate_overrides: Sequence[Mapping[str, Any]] | None = None,
) -> ReplayAlignmentResult:
    """Closed-loop Sim-to-Sim / Real-to-Sim parameter alignment (SimEX Stage-2 style).

    First runs direct replay. If the nominal sandbox simulator fails to reproduce
    the observed deployment failure (``not_reproduced_in_sim``), searches a bounded,
    attribution-conditioned set of sandbox disturbance overrides (e.g., slip offset,
    ack timeout, sensor noise + domain randomization, layout shift, machine busy)
    until the sandbox reproduces the exact attribution mode or the trial budget is met.
    """
    direct = verify_replay_alignment(driver, baseline_source, case)
    if direct.reproduced:
        return direct

    base_scen = case.to_scenario_dict()
    if candidate_overrides is not None:
        overrides = [dict(o) for o in candidate_overrides]
    else:
        code = case.attribution_code
        overrides = []
        if code in {"missing_recenter_branch", "l2_safety_violation", "delivery_offset_exceeded"}:
            overrides.extend(
                [
                    {"fault_inject_slip_xy_m": [0.032, -0.012]},
                    {"fault_inject_slip_xy_m": [0.038, -0.014], "sensor_noise": True, "domain_randomization": True},
                    {"fault_inject_slip_xy_m": [0.036, 0.015], "simulate_timeout": True},
                    {"store_layout_shift_xy_m": [0.03, -0.025], "fault_inject_slip_xy_m": [0.032, -0.012]},
                ]
            )
        elif code in {"unreconciled_unknown_ack", "duplicate_non_idempotent", "timeout_unhandled"}:
            overrides.extend(
                [
                    {"simulate_timeout": True},
                    {"simulate_crash_mid_brew": True},
                    {"fault_inject_slip_xy_m": [0.032, -0.012], "simulate_timeout": True},
                    {"contradictory_receipt": True},
                ]
            )
        else:
            overrides.extend(
                [
                    {"fault_inject_slip_xy_m": [0.034, -0.012]},
                    {"simulate_timeout": True},
                    {"sensor_noise": True, "domain_randomization": True, "fault_inject_slip_xy_m": [0.034, -0.012]},
                    {"simulate_machine_busy_polls": 2},
                    {"simulate_crash_mid_brew": True},
                    {"store_layout_shift_xy_m": [0.03, -0.025], "sensor_noise": True},
                ]
            )

    best_partial: ReplayAlignmentResult | None = None
    trials_used = 0
    cap = max(0, int(max_calibration_trials))

    for ov in overrides[:cap]:
        merged = dict(base_scen)
        merged.update(ov)
        trials_used += 1
        cmd_id = f"cal-{case.case_id}-{trials_used}-{time.time_ns()}"
        trial = _run_case_scenario(
            driver,
            baseline_source,
            merged,
            command_prefix=cmd_id,
            journal=None,
            enable_journal_proxy=False,
        )
        attr = attribute_trial(trial)
        reproduced = not bool(trial.get("task_passed", False))
        attr_matched = reproduced and (attr.code == case.attribution_code)
        if not reproduced:
            continue
        rep_status = "reproduced_exact" if attr_matched else "reproduced_different_mode"
        trust = (
            f"Calibrated sandbox parameters ({json.dumps(ov, sort_keys=True)}) reproduced "
            f"{'exact' if attr_matched else 'different'} attribution '{attr.code}' ({attr.layer}) on "
            f"{driver.driver_name} ({driver.evidence_source}) after {trials_used} calibration trial(s). "
            "Screening evidence is informative for pre-filtering only; independent acceptance remains mandatory."
        )
        cand_res = ReplayAlignmentResult(
            case_id=case.case_id,
            baseline_sha256=digest(baseline_source),
            driver_name=driver.driver_name,
            evidence_source=driver.evidence_source,
            reproduced=True,
            attribution_matched=attr_matched,
            reproduction_status=rep_status,
            original_attribution_layer=case.attribution_layer,
            original_attribution_code=case.attribution_code,
            baseline_attribution=attr.to_dict(),
            baseline_trial=trial,
            sim_trust_notes=trust,
            alignment_method="calibrated_sim_search",
            calibration_trials_used=trials_used,
            calibrated_scenario=merged,
        )
        if attr_matched:
            return cand_res
        if best_partial is None:
            best_partial = cand_res

    if best_partial is not None:
        return best_partial

    return ReplayAlignmentResult(
        case_id=direct.case_id,
        baseline_sha256=direct.baseline_sha256,
        driver_name=direct.driver_name,
        evidence_source=direct.evidence_source,
        reproduced=False,
        attribution_matched=False,
        reproduction_status="not_reproduced_in_sim",
        original_attribution_layer=direct.original_attribution_layer,
        original_attribution_code=direct.original_attribution_code,
        baseline_attribution=direct.baseline_attribution,
        baseline_trial=direct.baseline_trial,
        sim_trust_notes=(
            f"{direct.sim_trust_notes} Bounded calibration search ({trials_used} trials) also failed to "
            "reproduce the defect in simulation."
        ),
        alignment_method="calibrated_sim_search",
        calibration_trials_used=trials_used,
        calibrated_scenario={},
    )


def synthesize_repair_hypotheses(
    config: SceneConfig,
    baseline_source: str,
    case: RegressionCase,
    alignment: ReplayAlignmentResult,
    generator_fn: GeneratorFn,
    *,
    max_hypotheses: int = 3,
) -> list[RepairHypothesis]:
    """Ask the configured LLM generator to propose K distinct mechanistic repair hypotheses."""
    prompt = build_llm_hypothesis_synthesis_prompt(
        task_description=f"Task: {config.task_name} (scene={config.scene_id}). Non-idempotent methods: {list(config.capabilities.non_idempotent_methods)}.",
        sdk_contract=f"Allowed SDK methods: {list(config.capabilities.required_methods)}.",
        baseline_source=baseline_source,
        failure_telemetry=[
            {
                "case_id": case.case_id,
                "label": case.scenario.label,
                "original_attribution": {"layer": case.attribution_layer, "code": case.attribution_code},
                "alignment_status": alignment.reproduction_status,
                "alignment_method": alignment.alignment_method,
                "baseline_attribution": dict(alignment.baseline_attribution),
                "baseline_status": alignment.baseline_trial.get("host_runtime_status"),
                "baseline_detail": alignment.baseline_trial.get("detail"),
                "methods_called": alignment.baseline_trial.get("methods"),
            }
        ],
        max_hypotheses=max_hypotheses,
    )
    gen_res = generator_fn(config, prompt)
    raw = str(gen_res.get("raw_output") or gen_res.get("source") or "")
    parsed = parse_llm_hypotheses_json(raw, max_hypotheses=max_hypotheses)
    return [
        RepairHypothesis(
            hypothesis_id=item["hypothesis_id"],
            title=item["title"],
            diagnosis=item["diagnosis"],
            proposed_strategy=item["proposed_strategy"],
        )
        for item in parsed
    ]


def build_hypothesis_repair_prompt(
    config: SceneConfig,
    baseline_source: str,
    case: RegressionCase,
    alignment: ReplayAlignmentResult,
    hypothesis: RepairHypothesis,
    *,
    skill_hints: Sequence[Mapping[str, Any]] = (),
    regression_summaries: Sequence[Mapping[str, Any]] = (),
    sdk_contract: str = "",
) -> str:
    """Build a traceable prompt requesting one hypothesis-specific repair candidate."""
    payload = {
        "task_name": config.task_name,
        "allowed_host_methods": list(config.capabilities.required_methods),
        "non_idempotent_methods": list(config.capabilities.non_idempotent_methods),
        "target_case": {
            "case_id": case.case_id,
            "label": case.scenario.label,
            "attribution_layer": case.attribution_layer,
            "attribution_code": case.attribution_code,
            "scenario": dict(alignment.calibrated_scenario) if alignment.calibrated_scenario else case.scenario.to_dict(),
        },
        "replay_alignment": {
            "reproduction_status": alignment.reproduction_status,
            "alignment_method": alignment.alignment_method,
            "baseline_attribution": alignment.baseline_attribution,
            "baseline_status": alignment.baseline_trial.get("host_runtime_status"),
            "baseline_detail": alignment.baseline_trial.get("detail"),
            "baseline_methods": list(alignment.baseline_trial.get("methods") or []),
            "baseline_steps": list(alignment.baseline_trial.get("step_records") or [])[-12:],
        },
        "assigned_hypothesis": hypothesis.to_dict(),
        "regression_cases_to_preserve": list(regression_summaries),
        "verified_skill_hints": list(skill_hints),
    }
    contract_block = f"\nHost SDK Contract:\n{sdk_contract.strip()}\n" if sdk_contract.strip() else ""
    header = (
        "Write ONLY valid Python defining `def run_policy(sdk, params):`.\n"
        "Implement the specific repair described in assigned_hypothesis to fix target_case "
        "without regressing any case in regression_cases_to_preserve.\n"
        "Strict AST Sandbox Rules: NO `import`, `try`/`except`, `while`, `with`, `class`, `lambda`, "
        "or ANY identifier/attribute starting with `_` (do NOT write `for _ in range(...)` — use `for i in range(...)`).\n"
        "Use only allowed_host_methods on sdk. Never invent capabilities or hardcoded 3D coordinates.\n"
        "For non-idempotent methods (e.g. start_coffee_machine), never blindly reissue after an "
        "unknown receipt—always call sdk.query_command_status(command_id).\n"
        "Generated code is an unapproved draft subject to simulation screening and independent acceptance.\n"
        f"{contract_block}\n"
        "Baseline policy source:\n```python\n"
        f"{baseline_source.strip()}\n"
        "```\n"
        "Repair context (JSON):\n"
    )
    return header + canonical(payload)


def _trial_has_unreconciled_unknown(trial: Mapping[str, Any]) -> bool:
    return int(trial.get("unknown_events_observed", 0)) > 0 and not bool(
        trial.get("queried_after_unknown", True)
    )


class RepairScreener:
    """Durable, budget-capped simulation replay-and-screen coordinator."""

    def __init__(
        self,
        workspace_dir: str | Path,
        config: SceneConfig,
        driver: SceneHostDriver,
        *,
        regression_suite: RegressionSuite | None = None,
    ) -> None:
        self.workspace_dir = Path(workspace_dir)
        self.config = config
        self.driver = driver
        self.regression_suite = regression_suite

    def _session_dir(self, screening_id: str) -> Path:
        return self.workspace_dir / "screening" / screening_id

    def screen_repair(
        self,
        *,
        baseline_source: str,
        case: RegressionCase,
        screening_id: str | None = None,
        hypotheses: Sequence[RepairHypothesis] | None = None,
        candidate_sources: Mapping[str, str] | None = None,
        generator_fn: GeneratorFn | None = None,
        skill_hints: Sequence[Mapping[str, Any]] = (),
        max_candidates: int = 3,
        max_model_calls: int | None = None,
        max_trials: int | None = None,
        allow_non_policy_repair: bool = False,
        allow_unreproduced: bool = False,
        enable_sim_calibration: bool = False,
        max_calibration_trials: int = 6,
        calibration_overrides: Sequence[Mapping[str, Any]] | None = None,
        synthesize_hypotheses_with_llm: bool = False,
        sdk_contract: str = "",
    ) -> dict[str, Any]:
        """Run or resume a replay-aligned multi-hypothesis repair screening session.

        Returns a persistent JSON-serializable report. ``approved`` is always ``False``
        and ``requires_independent_acceptance`` is always ``True``.
        """
        baseline_sha = digest(baseline_source)
        sid = screening_id or f"screen-{case.case_id}-{baseline_sha[:8]}"
        sdir = self._session_dir(sid)
        session_file = sdir / "session.json"

        budget_model_cap = (
            int(max_model_calls) if max_model_calls is not None else int(self.config.budget.max_model_calls)
        )
        budget_trial_cap = (
            int(max_trials) if max_trials is not None else int(self.config.budget.max_trials)
        )

        if session_file.is_file():
            state = json.loads(session_file.read_text(encoding="utf-8"))
            if state.get("config_sha256") != self.config.config_sha256:
                raise ValueError("Screening identity drift: config_sha256 mismatch")
            if state.get("baseline_sha256") != baseline_sha:
                raise ValueError("Screening identity drift: baseline_sha256 mismatch")
            if state.get("case_id") != case.case_id:
                raise ValueError("Screening identity drift: case_id mismatch")
        else:
            state = {
                "schema_version": 1,
                "screening_id": sid,
                "case_id": case.case_id,
                "config_sha256": self.config.config_sha256,
                "baseline_sha256": baseline_sha,
                "driver_name": self.driver.driver_name,
                "evidence_source": self.driver.evidence_source,
                "status": "initialized",
                "budget_used": {"model_calls": 0, "trials": 0},
                "alignment": None,
                "hypotheses": [],
                "candidates": [],
                "recommended_candidate_id": None,
                "recommended_source_sha256": None,
                "approved": False,
                "requires_independent_acceptance": True,
                "created_at_utc": _utc_now(),
                "updated_at_utc": _utc_now(),
            }
            _write_json_atomic(session_file, state)

        def _persist() -> None:
            state["updated_at_utc"] = _utc_now()
            _write_json_atomic(session_file, state)

        # Gate 1: ownership boundary (do not paper over perception/control/device failures)
        if case.attribution_layer != "policy" and not allow_non_policy_repair:
            state["status"] = "blocked_non_policy_owner"
            state["detail"] = (
                f"Case {case.case_id} is attributed to layer '{case.attribution_layer}' "
                f"({case.attribution_code}); escalate to host owner instead of auto-patching policy."
            )
            _persist()
            return state

        # Gate 2: replay-based simulator alignment (with optional closed-loop calibration)
        if state.get("alignment") is None:
            if state["budget_used"]["trials"] >= budget_trial_cap:
                state["status"] = "budget_exhausted"
                _persist()
                return state
            if enable_sim_calibration:
                rem_cal = max(0, min(int(max_calibration_trials), budget_trial_cap - state["budget_used"]["trials"] - 1))
                alignment = calibrate_simulator_alignment(
                    self.driver,
                    baseline_source,
                    case,
                    max_calibration_trials=rem_cal,
                    candidate_overrides=calibration_overrides,
                )
                state["budget_used"]["trials"] += 1 + alignment.calibration_trials_used
            else:
                alignment = verify_replay_alignment(self.driver, baseline_source, case)
                state["budget_used"]["trials"] += 1
            state["alignment"] = alignment.to_dict()
            _persist()
        else:
            alignment = ReplayAlignmentResult.from_dict(state["alignment"])

        if not alignment.reproduced and not allow_unreproduced:
            state["status"] = "not_reproduced_in_sim"
            state["detail"] = alignment.sim_trust_notes
            _persist()
            return state

        # Resolve hypothesis list (persisted on first run for deterministic resume)
        if state.get("hypotheses"):
            hyp_list = [RepairHypothesis.from_dict(h) for h in state["hypotheses"]]
        else:
            if hypotheses is not None:
                hyp_list = list(hypotheses)[: max(1, int(max_candidates))]
            elif candidate_sources is not None:
                hyp_list = [
                    RepairHypothesis(
                        hypothesis_id=hid,
                        title=f"Provided candidate ({hid})",
                        diagnosis=f"Caller-supplied repair hypothesis for {case.attribution_code}",
                        proposed_strategy="Evaluate supplied candidate patch in simulation screening.",
                    )
                    for hid in list(candidate_sources.keys())[: max(1, int(max_candidates))]
                ]
            elif (
                synthesize_hypotheses_with_llm
                and state["budget_used"]["model_calls"] < budget_model_cap
            ):
                gen_h = generator_fn or default_live_generator
                synth = synthesize_repair_hypotheses(
                    self.config,
                    baseline_source,
                    case,
                    alignment,
                    gen_h,
                    max_hypotheses=max_candidates,
                )
                state["budget_used"]["model_calls"] += 1
                hyp_list = synth if synth else default_hypotheses_for_case(case, max_hypotheses=max_candidates)
            else:
                hyp_list = default_hypotheses_for_case(case, max_hypotheses=max_candidates)
            state["hypotheses"] = [h.to_dict() for h in hyp_list]
            _persist()

        # Gather regression cases excluding the target case itself to avoid double-counting
        other_reg_cases: list[RegressionCase] = []
        if self.regression_suite is not None:
            other_reg_cases = [c for c in self.regression_suite.cases if c.case_id != case.case_id]

        reg_summaries = [
            {
                "case_id": c.case_id,
                "label": c.scenario.label,
                "attribution_code": c.attribution_code,
                "preset": c.scenario.preset,
            }
            for c in other_reg_cases
        ]

        already_screened = {c["hypothesis_id"] for c in state["candidates"]}
        gen = generator_fn or default_live_generator
        state["status"] = "running"

        for idx, hyp in enumerate(hyp_list, start=1):
            if hyp.hypothesis_id in already_screened:
                continue

            # Acquire candidate source (either supplied directly or via generator within budget)
            prompt = ""
            if candidate_sources is not None and hyp.hypothesis_id in candidate_sources:
                source = _strip_code_fence(str(candidate_sources[hyp.hypothesis_id]))
            else:
                if state["budget_used"]["model_calls"] >= budget_model_cap:
                    state["status"] = "budget_exhausted"
                    _persist()
                    return state
                prompt = build_hypothesis_repair_prompt(
                    self.config,
                    baseline_source,
                    case,
                    alignment,
                    hyp,
                    skill_hints=skill_hints,
                    regression_summaries=reg_summaries,
                    sdk_contract=sdk_contract,
                )
                gen_res = gen(self.config, prompt)
                state["budget_used"]["model_calls"] += 1
                source = _strip_code_fence(str(gen_res["source"]))

            cand_id = f"scr_{idx:02d}_{hyp.hypothesis_id}"
            rel_path = f"candidates/{cand_id}.py"
            abs_path = sdir / rel_path
            abs_path.parent.mkdir(parents=True, exist_ok=True)
            abs_path.write_text(source + "\n", encoding="utf-8")
            if prompt:
                (sdir / f"candidates/{cand_id}.prompt.txt").write_text(prompt, encoding="utf-8")

            syntax_ok, val_err = validate_host_policy_syntax(source)
            if syntax_ok:
                ast_chk = check_sandbox_ast_compliance(source)
                if not ast_chk["compliant"]:
                    syntax_ok = False
                    val_err = "AST sandbox violation: " + "; ".join(ast_chk["issues"][:3])
            if not syntax_ok:
                rec = CandidateScreeningResult(
                    candidate_id=cand_id,
                    hypothesis_id=hyp.hypothesis_id,
                    hypothesis_title=hyp.title,
                    source_sha256=digest(source),
                    source_path=rel_path,
                    evidence_source=self.driver.evidence_source,
                    syntax_valid=False,
                    validation_error=val_err,
                    target_fixed=False,
                    target_trial={},
                    regression_total=len(other_reg_cases),
                    regression_passed=0,
                    regression_failed_ids=tuple(c.case_id for c in other_reg_cases),
                    duplicate_non_idempotent=False,
                    unreconciled_unknown_ack=False,
                    mean_wall_s=0.0,
                    eligible_for_acceptance=False,
                    rejection_reasons=("syntax_error",),
                )
                state["candidates"].append(rec.to_dict())
                _persist()
                continue

            # Check trial budget before running target + regression suite
            needed_trials = 1 + len(other_reg_cases)
            if state["budget_used"]["trials"] + needed_trials > budget_trial_cap:
                state["status"] = "budget_exhausted"
                _persist()
                return state

            # 1) Screen on target reproduced failure scenario (using calibrated scenario if aligned via search)
            target_scen = dict(alignment.calibrated_scenario) if alignment.calibrated_scenario else case.to_scenario_dict()
            cand_journal_path = sdir / "candidates" / f"{cand_id}_journal.sqlite"
            with Journal(cand_journal_path) as cand_journal:
                target_trial = _run_case_scenario(
                    self.driver,
                    source,
                    target_scen,
                    command_prefix=f"scr-target-{cand_id}-{time.time_ns()}",
                    journal=cand_journal,
                    enable_journal_proxy=True,
                )
                state["budget_used"]["trials"] += 1
                target_fixed = bool(target_trial.get("task_passed", False))

                dup_non_idem = bool(target_trial.get("duplicate_non_idempotent", False))
                unrec_unknown = _trial_has_unreconciled_unknown(target_trial)
                wall_samples = [float(target_trial.get("wall_s", 0.0))]

                # 2) Screen on all other accumulated RegressionSuite cases (capability-bank replay)
                reg_failed: list[str] = []
                for reg_case in other_reg_cases:
                    reg_scen = reg_case.to_scenario_dict()
                    reg_res = _run_case_scenario(
                        self.driver,
                        source,
                        reg_scen,
                        command_prefix=f"scr-reg-{cand_id}-{reg_case.case_id}-{time.time_ns()}",
                        journal=cand_journal,
                        enable_journal_proxy=True,
                    )
                    state["budget_used"]["trials"] += 1
                    wall_samples.append(float(reg_res.get("wall_s", 0.0)))
                    if not bool(reg_res.get("task_passed", False)):
                        reg_failed.append(reg_case.case_id)
                    if bool(reg_res.get("duplicate_non_idempotent", False)):
                        dup_non_idem = True
                    if _trial_has_unreconciled_unknown(reg_res):
                        unrec_unknown = True

            reasons: list[str] = []
            if not target_fixed:
                reasons.append("target_failure_not_fixed")
            if reg_failed:
                reasons.append("regression_failed")
            if dup_non_idem:
                reasons.append("duplicate_non_idempotent")
            if unrec_unknown:
                reasons.append("unreconciled_unknown_ack")

            eligible = len(reasons) == 0
            rec = CandidateScreeningResult(
                candidate_id=cand_id,
                hypothesis_id=hyp.hypothesis_id,
                hypothesis_title=hyp.title,
                source_sha256=digest(source),
                source_path=rel_path,
                evidence_source=self.driver.evidence_source,
                syntax_valid=True,
                validation_error="",
                target_fixed=target_fixed,
                target_trial=target_trial,
                regression_total=len(other_reg_cases),
                regression_passed=len(other_reg_cases) - len(reg_failed),
                regression_failed_ids=tuple(reg_failed),
                duplicate_non_idempotent=dup_non_idem,
                unreconciled_unknown_ack=unrec_unknown,
                mean_wall_s=sum(wall_samples) / len(wall_samples) if wall_samples else 0.0,
                eligible_for_acceptance=eligible,
                rejection_reasons=tuple(reasons),
            )
            state["candidates"].append(rec.to_dict())
            _persist()

        # Select best eligible candidate (reliability-first already enforced; rank by mean_wall_s)
        eligible_cands = [c for c in state["candidates"] if c.get("eligible_for_acceptance")]
        eligible_cands.sort(key=lambda c: (float(c.get("mean_wall_s", 0.0)), str(c.get("candidate_id"))))
        if eligible_cands:
            best = eligible_cands[0]
            state["recommended_candidate_id"] = best["candidate_id"]
            state["recommended_source_sha256"] = best["source_sha256"]
            state["status"] = "completed_with_recommendation"
        else:
            state["recommended_candidate_id"] = None
            state["recommended_source_sha256"] = None
            state["status"] = "completed_no_eligible_candidate"

        state["approved"] = False
        state["requires_independent_acceptance"] = True
        _persist()
        return state

    def get_candidate_source(self, screening_id: str, candidate_id: str) -> str:
        """Load persisted source code for a screened candidate."""
        sdir = self._session_dir(screening_id)
        session = json.loads((sdir / "session.json").read_text(encoding="utf-8"))
        for cand in session.get("candidates") or []:
            if cand.get("candidate_id") == candidate_id:
                return (sdir / cand["source_path"]).read_text(encoding="utf-8")
        raise KeyError(f"Unknown candidate_id {candidate_id!r} in screening {screening_id!r}")
