"""Failure-to-Testcase permanent regression suite flywheel.

Turns every observed failure or unexpected human intervention into a deduplicated,
persistent ``BenchmarkScenario``. Future policy edits and optimization candidates
must pass all accumulated regression cases before promotion so fixed defects never
silently return.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
from pathlib import Path
import time
from typing import Any, Mapping, Sequence

from .benchmark import BenchmarkScenario
from .contracts import digest
from .host_driver import SceneHostDriver
from .profiler import attribute_trial


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _write_json_atomic(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    tmp.replace(path)


def _scenario_from_dict(data: Mapping[str, Any]) -> BenchmarkScenario:
    cup_a = data.get("cup_a_xy")
    cup_b = data.get("cup_b_xy")
    slip = data.get("fault_inject_slip_xy_m")
    return BenchmarkScenario(
        scenario_id=str(data["scenario_id"]),
        label=str(data["label"]),
        category=str(data.get("category", "regression")),
        preset=str(data.get("preset", "default")),
        cup_a_xy=(float(cup_a[0]), float(cup_a[1])) if cup_a is not None else None,
        cup_b_xy=(float(cup_b[0]), float(cup_b[1])) if cup_b is not None else None,
        order_id=str(data.get("order_id", "A042")),
        target_slot=str(data.get("target_slot", "$SLOT_PICKUP_1")),
        fault_inject_slip_xy_m=(float(slip[0]), float(slip[1])) if slip is not None else None,
        simulate_timeout=bool(data.get("simulate_timeout", False)),
        expect_status=str(data.get("expect_status", "completed")),
        max_delivery_error_mm=float(data.get("max_delivery_error_mm", 15.0)),
    )


_EXTRA_SCENARIO_KEYS = (
    "simulate_crash_mid_brew",
    "simulate_foreign_brew_running",
    "simulate_machine_fault",
    "simulate_machine_busy_polls",
    "store_layout_shift_xy_m",
    "cup_class",
    "cup_confidence",
    "stale_observation",
    "contradictory_receipt",
)


@dataclass(frozen=True)
class RegressionCase:
    case_id: str
    fingerprint: str
    scenario: BenchmarkScenario
    source_trial_id: str
    attribution_layer: str
    attribution_code: str
    evidence_source: str
    hit_count: int
    captured_at_utc: str
    last_seen_utc: str
    notes: str = ""
    extra_scenario: Mapping[str, Any] = field(default_factory=dict)

    def to_scenario_dict(self) -> dict[str, Any]:
        merged = self.scenario.to_dict()
        merged.update(dict(self.extra_scenario))
        return merged

    def to_dict(self) -> dict[str, Any]:
        d = {
            "case_id": self.case_id,
            "fingerprint": self.fingerprint,
            "scenario": self.scenario.to_dict(),
            "source_trial_id": self.source_trial_id,
            "attribution_layer": self.attribution_layer,
            "attribution_code": self.attribution_code,
            "evidence_source": self.evidence_source,
            "hit_count": self.hit_count,
            "captured_at_utc": self.captured_at_utc,
            "last_seen_utc": self.last_seen_utc,
            "notes": self.notes,
        }
        if self.extra_scenario:
            d["extra_scenario"] = dict(self.extra_scenario)
        return d

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> RegressionCase:
        return cls(
            case_id=str(data["case_id"]),
            fingerprint=str(data["fingerprint"]),
            scenario=_scenario_from_dict(data["scenario"]),
            source_trial_id=str(data["source_trial_id"]),
            attribution_layer=str(data["attribution_layer"]),
            attribution_code=str(data["attribution_code"]),
            evidence_source=str(data["evidence_source"]),
            hit_count=int(data.get("hit_count", 1)),
            captured_at_utc=str(data["captured_at_utc"]),
            last_seen_utc=str(data["last_seen_utc"]),
            notes=str(data.get("notes", "")),
            extra_scenario=dict(data.get("extra_scenario") or {}),
        )


class RegressionSuite:
    """Durable, deduplicated collection of failure-derived regression scenarios."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        if self.path.is_file():
            self._state = json.loads(self.path.read_text(encoding="utf-8"))
        else:
            self._state = {"schema_version": 1, "cases": {}}
            self._save()

    def _save(self) -> None:
        _write_json_atomic(self.path, self._state)

    @property
    def cases(self) -> list[RegressionCase]:
        return [RegressionCase.from_dict(v) for v in self._state["cases"].values()]

    def scenarios(self) -> tuple[BenchmarkScenario, ...]:
        return tuple(c.scenario for c in self.cases)

    def capture_failure(
        self,
        trial: Mapping[str, Any],
        *,
        scenario_context: Mapping[str, Any] | None = None,
        notes: str = "",
    ) -> RegressionCase | None:
        """Convert a failed trial or unexpected intervention into a deduplicated regression case."""
        attr = attribute_trial(trial)
        interventions = int(trial.get("human_interventions", 0))
        expect_status = str((scenario_context or {}).get("expect_status", trial.get("expect_status", "completed")))
        unexpected_intervention = interventions > 0 and expect_status == "completed"

        if attr.layer == "none" and not unexpected_intervention:
            return None

        ctx = dict(scenario_context or {})
        preset = str(ctx.get("preset", trial.get("preset", "default")))
        cup_a = ctx.get("cup_a_xy", trial.get("cup_a_xy"))
        cup_b = ctx.get("cup_b_xy", trial.get("cup_b_xy"))
        slip = ctx.get("fault_inject_slip_xy_m", trial.get("fault_inject_slip_xy_m"))
        sim_timeout = bool(ctx.get("simulate_timeout", trial.get("simulate_timeout", False)))
        order_id = str(ctx.get("order_id", trial.get("order_id", "A042")))
        target_slot = str(ctx.get("target_slot", trial.get("target_slot", "$SLOT_PICKUP_1")))
        max_err = float(ctx.get("max_delivery_error_mm", trial.get("max_delivery_error_mm", 15.0)))

        extra_scen = {k: ctx[k] for k in _EXTRA_SCENARIO_KEYS if k in ctx and ctx[k] not in (None, False, 0)}

        fp_payload = {
            "preset": preset,
            "cup_a_xy": list(cup_a) if cup_a is not None else None,
            "cup_b_xy": list(cup_b) if cup_b is not None else None,
            "fault_inject_slip_xy_m": list(slip) if slip is not None else None,
            "simulate_timeout": sim_timeout,
            "order_id": order_id,
            "target_slot": target_slot,
            "expect_status": expect_status,
            "extra_scenario": extra_scen,
            "attribution_code": attr.code if attr.layer != "none" else "unexpected_human_intervention",
        }
        fp = digest(fp_payload)[:16]
        now = _utc_now()

        if fp in self._state["cases"]:
            existing = dict(self._state["cases"][fp])
            existing["hit_count"] = int(existing.get("hit_count", 1)) + 1
            existing["last_seen_utc"] = now
            self._state["cases"][fp] = existing
            self._save()
            return RegressionCase.from_dict(existing)

        case_num = len(self._state["cases"]) + 1
        case_id = f"reg_{case_num:03d}_{fp[:6]}"
        code = attr.code if attr.layer != "none" else "unexpected_human_intervention"
        layer = attr.layer if attr.layer != "none" else "policy"
        scen = BenchmarkScenario(
            scenario_id=case_id,
            label=str(ctx.get("label") or f"Regression ({code}) on {preset}"),
            category="regression",
            preset=preset,
            cup_a_xy=(float(cup_a[0]), float(cup_a[1])) if cup_a is not None else None,
            cup_b_xy=(float(cup_b[0]), float(cup_b[1])) if cup_b is not None else None,
            order_id=order_id,
            target_slot=target_slot,
            fault_inject_slip_xy_m=(float(slip[0]), float(slip[1])) if slip is not None else None,
            simulate_timeout=sim_timeout,
            expect_status=expect_status,
            max_delivery_error_mm=max_err,
        )
        case = RegressionCase(
            case_id=case_id,
            fingerprint=fp,
            scenario=scen,
            source_trial_id=str(trial.get("trial_id") or trial.get("command_id") or "unknown"),
            attribution_layer=layer,
            attribution_code=code,
            evidence_source=str(trial.get("evidence_source", "unknown")),
            hit_count=1,
            captured_at_utc=now,
            last_seen_utc=now,
            notes=notes or attr.summary,
            extra_scenario=extra_scen,
        )
        self._state["cases"][fp] = case.to_dict()
        self._save()
        return case

    def capture_from_benchmark_report(self, report: Mapping[str, Any]) -> list[RegressionCase]:
        """Extract regression cases for all failures recorded in a paired benchmark report."""
        scen_by_id = {s["scenario_id"]: s for s in (report.get("scenarios") or [])}
        captured: list[RegressionCase] = []
        for trial in report.get("trials") or []:
            ctx = scen_by_id.get(trial.get("scenario_id"), {})
            case = self.capture_failure(trial, scenario_context=ctx)
            if case is not None:
                captured.append(case)
        return captured

    def evaluate_regression_gate(
        self,
        driver: SceneHostDriver,
        policy_source: str,
        *,
        policy_id: str = "candidate",
    ) -> dict[str, Any]:
        """Run all accumulated regression cases against a candidate policy; any failure blocks promotion."""
        cases = self.cases
        results: list[dict[str, Any]] = []
        failed_ids: list[str] = []
        for case in cases:
            scen_dict = case.to_scenario_dict()
            driver.reset_scenario(scen_dict)
            cmd_id = f"reg-gate-{policy_id}-{case.case_id}-{time.time_ns()}"
            res = driver.run_policy_scenario(policy_source, command_id=cmd_id, scenario=scen_dict)
            results.append({"case_id": case.case_id, "scenario_label": case.scenario.label, **res})
            if not res.get("task_passed"):
                failed_ids.append(case.case_id)
        return {
            "passed": len(failed_ids) == 0,
            "total_cases": len(cases),
            "passed_cases": len(cases) - len(failed_ids),
            "failed_case_ids": failed_ids,
            "trials": results,
            "evidence_source": driver.evidence_source,
        }
