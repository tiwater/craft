"""Progressive autonomy ladder and Sim -> Recorded -> Real promotion gate.

Prevents simulator-verified policies from jumping directly into live physical
execution. Tracks human intervention rate across stages so operators can deploy
safely from day one (zero-accident gated operation) and progressively expand
autonomy as verified evidence accumulates.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any, Mapping


AUTONOMY_STAGES: tuple[str, ...] = (
    "fixture",
    "simulator",
    "recorded_data",
    "physical_dry_run",
    "physical_step_gated",
    "physical_live",
)

# Minimum required prerequisite stage before entering a target stage.
STAGE_PREREQUISITES: Mapping[str, tuple[str, ...]] = {
    "fixture": (),
    "simulator": (),
    "recorded_data": (),
    "physical_dry_run": ("simulator",),
    "physical_step_gated": ("simulator", "physical_dry_run"),
    "physical_live": ("simulator", "physical_dry_run", "physical_step_gated"),
}


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _write_json_atomic(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    tmp.replace(path)


def actuation_policy_for_stage(stage: str) -> dict[str, Any]:
    """Return mandatory safety controls for a given autonomy stage."""
    if stage not in AUTONOMY_STAGES:
        raise ValueError(f"Unknown autonomy stage: {stage!r}")
    if stage in {"fixture", "recorded_data", "physical_dry_run"}:
        return {
            "stage": stage,
            "physical_actuation_allowed": False,
            "step_confirmation_required": False,
            "shadow_only": stage in {"recorded_data", "physical_dry_run"},
        }
    if stage == "simulator":
        return {
            "stage": stage,
            "physical_actuation_allowed": False,
            "simulated_actuation_allowed": True,
            "step_confirmation_required": False,
            "shadow_only": False,
        }
    if stage == "physical_step_gated":
        return {
            "stage": stage,
            "physical_actuation_allowed": True,
            "step_confirmation_required": True,
            "shadow_only": False,
        }
    return {
        "stage": stage,
        "physical_actuation_allowed": True,
        "step_confirmation_required": False,
        "shadow_only": False,
    }


@dataclass(frozen=True)
class StageEvidence:
    stage: str
    policy_sha256: str
    config_sha256: str
    sdk_contract_sha256: str
    report_id: str
    passed: bool
    total_trials: int
    human_interventions: int
    recorded_at_utc: str = ""
    notes: str = ""

    def __post_init__(self) -> None:
        if self.stage not in AUTONOMY_STAGES:
            raise ValueError(f"Invalid autonomy stage: {self.stage!r}")
        if self.total_trials < 1:
            raise ValueError("total_trials must be >= 1")
        if self.human_interventions < 0:
            raise ValueError("human_interventions must be >= 0")

    @property
    def intervention_rate(self) -> float:
        return round(self.human_interventions / self.total_trials, 4)

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        d["intervention_rate"] = self.intervention_rate
        if not d["recorded_at_utc"]:
            d["recorded_at_utc"] = _utc_now()
        return d

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> StageEvidence:
        return cls(
            stage=str(data["stage"]),
            policy_sha256=str(data["policy_sha256"]),
            config_sha256=str(data["config_sha256"]),
            sdk_contract_sha256=str(data["sdk_contract_sha256"]),
            report_id=str(data["report_id"]),
            passed=bool(data["passed"]),
            total_trials=int(data["total_trials"]),
            human_interventions=int(data["human_interventions"]),
            recorded_at_utc=str(data.get("recorded_at_utc") or ""),
            notes=str(data.get("notes", "")),
        )


class AutonomyPromotionLedger:
    """Persistent ledger enforcing staged Sim -> Recorded -> Real promotion."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        if self.path.is_file():
            self._state = json.loads(self.path.read_text(encoding="utf-8"))
        else:
            self._state = {"schema_version": 1, "evidence": []}
            self._save()

    def _save(self) -> None:
        _write_json_atomic(self.path, self._state)

    def record_evidence(self, ev: StageEvidence) -> dict[str, Any]:
        item = ev.to_dict()
        self._state["evidence"].append(item)
        self._save()
        return item

    def evidence_for_policy(self, policy_sha256: str) -> list[StageEvidence]:
        return [
            StageEvidence.from_dict(item)
            for item in self._state["evidence"]
            if item.get("policy_sha256") == policy_sha256
        ]

    def check_promotion(
        self,
        *,
        policy_sha256: str,
        target_stage: str,
        sdk_contract_sha256: str | None = None,
        require_recorded_data: bool = False,
        max_live_intervention_rate: float = 0.1,
    ) -> dict[str, Any]:
        """Evaluate whether a frozen policy is authorized to enter ``target_stage``."""
        if target_stage not in AUTONOMY_STAGES:
            raise ValueError(f"Unknown target_stage: {target_stage!r}")

        prereqs = list(STAGE_PREREQUISITES[target_stage])
        if require_recorded_data and target_stage.startswith("physical_") and "recorded_data" not in prereqs:
            prereqs.append("recorded_data")

        records = self.evidence_for_policy(policy_sha256)
        if sdk_contract_sha256 is not None:
            records = [r for r in records if r.sdk_contract_sha256 == sdk_contract_sha256]

        passed_by_stage: dict[str, StageEvidence] = {}
        for r in records:
            if r.passed:
                passed_by_stage[r.stage] = r

        missing = [st for st in prereqs if st not in passed_by_stage]
        reasons: list[str] = []
        if missing:
            reasons.append(f"Missing passing prerequisite stage(s): {', '.join(missing)}")

        if target_stage == "physical_live" and "physical_step_gated" in passed_by_stage:
            gated_ev = passed_by_stage["physical_step_gated"]
            if gated_ev.intervention_rate > max_live_intervention_rate:
                reasons.append(
                    f"physical_step_gated intervention_rate {gated_ev.intervention_rate:.2%} "
                    f"exceeds live threshold {max_live_intervention_rate:.2%}"
                )

        intervention_trend = [
            {
                "stage": st,
                "intervention_rate": passed_by_stage[st].intervention_rate,
                "total_trials": passed_by_stage[st].total_trials,
                "report_id": passed_by_stage[st].report_id,
            }
            for st in AUTONOMY_STAGES
            if st in passed_by_stage
        ]

        return {
            "allowed": len(reasons) == 0,
            "policy_sha256": policy_sha256,
            "target_stage": target_stage,
            "missing_prerequisites": missing,
            "reasons": reasons,
            "actuation_policy": actuation_policy_for_stage(target_stage),
            "intervention_trend": intervention_trend,
        }
