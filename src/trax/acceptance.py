"""Independent acceptance verification and reliability-first improvement comparison.

Acceptance criteria are frozen outside policy generation; a generated policy cannot
modify its own acceptance gate. Simulation acceptance is explicitly labeled as
non-equivalent to physical-robot or food-quality certification, and motion
smoothness remains dependent on the host robot controller.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Any, Mapping, Sequence

from .config import AcceptanceCriteriaConfig, VALID_EVIDENCE_SOURCES
from .contracts import digest


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def certification_disclaimer_for(scope: str) -> str:
    if scope == "physical_robot":
        return (
            "Physical robot acceptance verifies recorded task gates only; food-quality "
            "and safety compliance require separate operational sign-off. Motion smoothness "
            "depends on the host robot controller, not upper-layer CaP."
        )
    return (
        f"Scope={scope}: NOT physical-robot approval and NOT food-quality certification. "
        "Motion smoothness depends on the host robot controller, not upper-layer CaP."
    )


@dataclass(frozen=True)
class FrozenArtifact:
    """Immutable snapshot of policy code, scene config, host capability contract, and criteria."""

    policy_source: str
    policy_sha256: str
    config_sha256: str
    sdk_contract_sha256: str
    criteria_sha256: str
    deployment_scope: str

    def __post_init__(self) -> None:
        if self.deployment_scope not in VALID_EVIDENCE_SOURCES:
            raise ValueError(f"deployment_scope must be one of {sorted(VALID_EVIDENCE_SOURCES)}")
        if digest(self.policy_source) != self.policy_sha256:
            raise ValueError("FrozenArtifact.policy_sha256 does not match policy_source")

    @classmethod
    def freeze(
        cls,
        *,
        policy_source: str,
        config_sha256: str,
        sdk_contract_sha256: str,
        criteria: AcceptanceCriteriaConfig,
        deployment_scope: str,
    ) -> FrozenArtifact:
        return cls(
            policy_source=policy_source,
            policy_sha256=digest(policy_source),
            config_sha256=config_sha256,
            sdk_contract_sha256=sdk_contract_sha256,
            criteria_sha256=criteria.criteria_sha256,
            deployment_scope=deployment_scope,
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> FrozenArtifact:
        return cls(
            policy_source=str(data["policy_source"]),
            policy_sha256=str(data["policy_sha256"]),
            config_sha256=str(data["config_sha256"]),
            sdk_contract_sha256=str(data["sdk_contract_sha256"]),
            criteria_sha256=str(data["criteria_sha256"]),
            deployment_scope=str(data["deployment_scope"]),
        )


@dataclass(frozen=True)
class HeldoutTrialOutcome:
    """One independent (non-debug) acceptance trial outcome."""

    trial_id: str
    condition: str
    evidence_source: str
    completed: bool
    wall_s: float
    used_in_debugging: bool = False
    unknown_outcome: bool = False
    duplicate_non_idempotent: bool = False
    human_interventions: int = 0
    delivery_error_mm: float | None = None
    step_durations_s: Mapping[str, float] = field(default_factory=dict)
    detail: str = ""

    def __post_init__(self) -> None:
        if self.evidence_source not in VALID_EVIDENCE_SOURCES:
            raise ValueError(f"evidence_source must be one of {sorted(VALID_EVIDENCE_SOURCES)}")

    def to_dict(self) -> dict[str, Any]:
        return {
            "trial_id": self.trial_id,
            "condition": self.condition,
            "evidence_source": self.evidence_source,
            "completed": self.completed,
            "wall_s": round(self.wall_s, 3),
            "used_in_debugging": self.used_in_debugging,
            "unknown_outcome": self.unknown_outcome,
            "duplicate_non_idempotent": self.duplicate_non_idempotent,
            "human_interventions": self.human_interventions,
            "delivery_error_mm": self.delivery_error_mm,
            "step_durations_s": dict(self.step_durations_s),
            "detail": self.detail,
        }


@dataclass(frozen=True)
class AcceptanceReport:
    """Immutable acceptance verdict and metrics for a FrozenArtifact."""

    report_id: str
    artifact: FrozenArtifact
    criteria_sha256: str
    evidence_source: str
    passed: bool
    total_trials: int
    completed_trials: int
    completion_rate: float
    unknown_outcomes: int
    duplicate_non_idempotent_count: int
    human_interventions: int
    max_delivery_error_mm: float | None
    mean_wall_s: float
    slow_steps: tuple[dict[str, Any], ...]
    failure_reasons: tuple[str, ...]
    queue_for_optimization: bool
    optimization_reason: str
    certification_disclaimer: str
    evaluated_at_utc: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "report_id": self.report_id,
            "artifact": self.artifact.to_dict(),
            "criteria_sha256": self.criteria_sha256,
            "evidence_source": self.evidence_source,
            "passed": self.passed,
            "total_trials": self.total_trials,
            "completed_trials": self.completed_trials,
            "completion_rate": round(self.completion_rate, 4),
            "unknown_outcomes": self.unknown_outcomes,
            "duplicate_non_idempotent_count": self.duplicate_non_idempotent_count,
            "human_interventions": self.human_interventions,
            "max_delivery_error_mm": self.max_delivery_error_mm,
            "mean_wall_s": round(self.mean_wall_s, 3),
            "slow_steps": list(self.slow_steps),
            "failure_reasons": list(self.failure_reasons),
            "queue_for_optimization": self.queue_for_optimization,
            "optimization_reason": self.optimization_reason,
            "certification_disclaimer": self.certification_disclaimer,
            "evaluated_at_utc": self.evaluated_at_utc,
        }


def evaluate_acceptance(
    artifact: FrozenArtifact,
    criteria: AcceptanceCriteriaConfig,
    trials: Sequence[HeldoutTrialOutcome],
    *,
    report_id: str | None = None,
) -> AcceptanceReport:
    """Evaluate a frozen artifact against locked independent acceptance criteria."""
    failures: list[str] = []

    if artifact.criteria_sha256 != criteria.criteria_sha256:
        failures.append("Criteria hash mismatch: policy artifact was not bound to the locked acceptance criteria")

    if len(trials) < criteria.min_heldout_trials:
        failures.append(f"Insufficient held-out trials: {len(trials)} < {criteria.min_heldout_trials}")

    if any(t.used_in_debugging for t in trials):
        failures.append("Acceptance trials must not include conditions marked used_in_debugging=True")

    if any(t.evidence_source != artifact.deployment_scope for t in trials):
        failures.append("Evidence source mismatch: all acceptance trials must match artifact deployment_scope")

    total = len(trials)
    completed = sum(1 for t in trials if t.completed)
    completion_rate = (completed / total) if total > 0 else 0.0
    unknown_count = sum(1 for t in trials if t.unknown_outcome)
    dup_count = sum(1 for t in trials if t.duplicate_non_idempotent)
    interventions = sum(t.human_interventions for t in trials)
    errors = [t.delivery_error_mm for t in trials if t.delivery_error_mm is not None]
    max_err = max(errors) if errors else None
    mean_wall = (sum(t.wall_s for t in trials) / total) if total > 0 else 0.0

    slow_steps: list[dict[str, Any]] = []
    for t in trials:
        for step_name, dur in t.step_durations_s.items():
            if dur > criteria.slow_step_threshold_s:
                slow_steps.append({"trial_id": t.trial_id, "step": step_name, "duration_s": round(dur, 3)})

    if completion_rate < criteria.min_completion_rate:
        failures.append(f"Completion rate {completion_rate:.3f} < {criteria.min_completion_rate:.3f}")
    if unknown_count > criteria.max_unknown_outcomes:
        failures.append(f"Unknown outcomes {unknown_count} > {criteria.max_unknown_outcomes}")
    if dup_count > criteria.max_duplicate_non_idempotent:
        failures.append(f"Duplicate non-idempotent actions {dup_count} > {criteria.max_duplicate_non_idempotent}")
    if interventions > criteria.max_human_interventions:
        failures.append(f"Human interventions {interventions} > {criteria.max_human_interventions}")
    if max_err is not None and max_err > criteria.max_delivery_error_mm:
        failures.append(f"Max delivery error {max_err:.2f} mm > {criteria.max_delivery_error_mm:.2f} mm")

    passed = len(failures) == 0

    # Successful but slow runs pass reliability acceptance AND enter the optimization queue.
    queue_opt = False
    opt_reasons: list[str] = []
    if passed:
        if mean_wall > criteria.target_wall_s:
            queue_opt = True
            opt_reasons.append(f"mean_wall_s={mean_wall:.2f}s exceeds target {criteria.target_wall_s:.2f}s")
        if slow_steps:
            queue_opt = True
            opt_reasons.append(f"{len(slow_steps)} slow step(s) exceeded {criteria.slow_step_threshold_s:.2f}s")

    rid = report_id or f"acc-{artifact.policy_sha256[:10]}-{digest([t.to_dict() for t in trials])[:8]}"
    return AcceptanceReport(
        report_id=rid,
        artifact=artifact,
        criteria_sha256=criteria.criteria_sha256,
        evidence_source=artifact.deployment_scope,
        passed=passed,
        total_trials=total,
        completed_trials=completed,
        completion_rate=completion_rate,
        unknown_outcomes=unknown_count,
        duplicate_non_idempotent_count=dup_count,
        human_interventions=interventions,
        max_delivery_error_mm=max_err,
        mean_wall_s=mean_wall,
        slow_steps=tuple(slow_steps),
        failure_reasons=tuple(failures),
        queue_for_optimization=queue_opt,
        optimization_reason="; ".join(opt_reasons),
        certification_disclaimer=certification_disclaimer_for(artifact.deployment_scope),
        evaluated_at_utc=_utc_now(),
    )


def compare_improvement_candidate(
    baseline: AcceptanceReport,
    candidate: AcceptanceReport,
) -> dict[str, Any]:
    """Compare an improvement candidate against a baseline: reliability first, then speed."""
    if not candidate.passed:
        return {
            "decision": "rejected",
            "reason": f"Candidate failed independent acceptance: {'; '.join(candidate.failure_reasons)}",
        }
    if candidate.evidence_source != baseline.evidence_source:
        return {
            "decision": "rejected",
            "reason": "Evidence source mismatch between baseline and candidate",
        }
    if candidate.criteria_sha256 != baseline.criteria_sha256:
        return {
            "decision": "rejected",
            "reason": "Criteria hash mismatch between baseline and candidate",
        }

    # Reliability-first gate: never trade reliability for speed.
    if (
        candidate.completion_rate < baseline.completion_rate
        or candidate.unknown_outcomes > baseline.unknown_outcomes
        or candidate.duplicate_non_idempotent_count > baseline.duplicate_non_idempotent_count
        or candidate.human_interventions > baseline.human_interventions
    ):
        return {
            "decision": "rejected",
            "reason": "Reliability regression relative to baseline (speed gains cannot override reliability)",
        }

    reliability_improved = (
        candidate.completion_rate > baseline.completion_rate
        or candidate.unknown_outcomes < baseline.unknown_outcomes
        or candidate.duplicate_non_idempotent_count < baseline.duplicate_non_idempotent_count
        or candidate.human_interventions < baseline.human_interventions
    )
    if reliability_improved:
        return {
            "decision": "accepted",
            "reason": "Candidate improved reliability while passing independent acceptance",
        }

    speed_improved = (
        candidate.mean_wall_s < baseline.mean_wall_s
        or len(candidate.slow_steps) < len(baseline.slow_steps)
    )
    if speed_improved:
        return {
            "decision": "accepted",
            "reason": (
                f"Candidate matched baseline reliability and improved speed "
                f"(mean_wall_s {baseline.mean_wall_s:.2f}s -> {candidate.mean_wall_s:.2f}s, "
                f"slow_steps {len(baseline.slow_steps)} -> {len(candidate.slow_steps)})"
            ),
        }

    return {
        "decision": "rejected",
        "reason": "Candidate did not improve reliability or speed over baseline",
    }
