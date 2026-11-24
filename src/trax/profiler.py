"""Four-layer failure attribution and step-level latency profiler for TRAX engine.

Separates failures cleanly across four ownership layers:
- ``perception``: stale/unready evidence, missing/ambiguous objects, low confidence
- ``control``: grasp failure (empty_grasp), placement/recenter/delivery execution error
- ``device``: appliance/hardware faults, disconnects, unresolved ack timeouts, missing host capabilities
- ``policy``: syntax errors, duplicate non-idempotent dispatches, missing reconciliation, logic errors

Only policy-layer defects are marked ``auto_repair_eligible=True`` so TRAX never
asks a model to paper over perception, calibration, or hardware defects.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Mapping, Sequence


VALID_ATTRIBUTION_LAYERS = frozenset({"none", "perception", "control", "device", "policy"})


@dataclass(frozen=True)
class FailureAttribution:
    layer: str  # none | perception | control | device | policy
    code: str
    recommended_owner: str  # none | host_perception | host_control | host_device | craft_policy
    auto_repair_eligible: bool
    summary: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class StepTiming:
    index: int
    method: str
    duration_s: float | None
    receipt: str = ""
    slow: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "index": self.index,
            "method": self.method,
            "duration_s": round(self.duration_s, 3) if self.duration_s is not None else None,
            "receipt": self.receipt,
            "slow": self.slow,
        }


def attribute_trial(trial: Mapping[str, Any]) -> FailureAttribution:
    """Classify a trial outcome into one of the four ownership layers."""
    passed = bool(trial.get("task_passed", trial.get("passed", False)))
    if passed:
        return FailureAttribution(
            layer="none",
            code="passed",
            recommended_owner="none",
            auto_repair_eligible=False,
            summary="Trial passed independent verification",
        )

    status = str(trial.get("host_runtime_status") or trial.get("status") or "").lower()
    detail = str(trial.get("failure_detail") or trial.get("detail") or "").lower()
    combined = f"{status} {detail}"

    # 1. Policy-layer safety/protocol violations take highest priority
    if "syntax" in combined or status == "syntax_error":
        return FailureAttribution(
            layer="policy",
            code="syntax_error",
            recommended_owner="craft_policy",
            auto_repair_eligible=True,
            summary="Generated policy failed syntax or entry-point validation",
        )
    if bool(trial.get("duplicate_non_idempotent", False)):
        return FailureAttribution(
            layer="policy",
            code="duplicate_non_idempotent",
            recommended_owner="craft_policy",
            auto_repair_eligible=True,
            summary="Policy dispatched a non-idempotent action more than once",
        )
    if int(trial.get("unknown_events_observed", 0)) > 0 and not bool(trial.get("queried_after_unknown", True)):
        return FailureAttribution(
            layer="policy",
            code="unreconciled_unknown_ack",
            recommended_owner="craft_policy",
            auto_repair_eligible=True,
            summary="Policy failed to call query_command_status after an unknown acknowledgement",
        )
    if "failed_l2_pouring_gate" in combined or (
        bool(trial.get("fault_injection"))
        and "recenter_cup_under_spout" not in (trial.get("methods") or ())
        and status in {"failed_brew_start", "failed"}
    ):
        return FailureAttribution(
            layer="policy",
            code="missing_recenter_branch",
            recommended_owner="craft_policy",
            auto_repair_eligible=True,
            summary="Policy attempted to proceed without recentering after placement offset",
        )

    # 2. Perception-layer issues
    if any(
        tok in combined
        for tok in (
            "stale_evidence",
            "observation_not_ready",
            "missing_object",
            "no_cup_detected",
            "ambiguous_object",
            "paused_for_operator",
            "low_confidence",
        )
    ):
        return FailureAttribution(
            layer="perception",
            code=status or "perception_not_ready",
            recommended_owner="host_perception",
            auto_repair_eligible=False,
            summary="Perception evidence was stale, unready, missing, or ambiguous",
        )

    # 3. Control / physical execution issues
    if any(
        tok in combined
        for tok in (
            "empty_grasp",
            "failed_recenter",
            "failed_place",
            "failed_deliver",
            "failed_delivery",
            "failed_verify",
            "failed_complete_order_delivery",
            "collision",
            "ik_",
        )
    ):
        return FailureAttribution(
            layer="control",
            code=status or "control_execution_failed",
            recommended_owner="host_control",
            auto_repair_eligible=False,
            summary=f"Host physical/simulated manipulation step failed ({status or 'control'})",
        )

    # 4. Device / infrastructure / capability gaps
    if any(
        tok in combined
        for tok in (
            "disconnected",
            "missing_capabilities",
            "contract_drift",
            "host_busy",
            "unknown_outcome",
            "reconciliation_required",
            "failed_brew_start",
            "failed_wait",
            "timeout",
            "permission_denied",
        )
    ):
        return FailureAttribution(
            layer="device",
            code=status or "device_or_transport_fault",
            recommended_owner="host_device",
            auto_repair_eligible=False,
            summary=f"Host device, capability contract, or transport fault ({status or 'device'})",
        )

    # Fallback: unattributed policy/logic failure
    return FailureAttribution(
        layer="policy",
        code=status or "policy_logic_failure",
        recommended_owner="craft_policy",
        auto_repair_eligible=True,
        summary=f"Trial did not meet acceptance criteria ({status or 'unknown'})",
    )


def profile_steps(
    steps: Sequence[Mapping[str, Any]],
    *,
    total_wall_s: float | None = None,
    slow_step_threshold_s: float = 8.0,
) -> dict[str, Any]:
    """Compute per-step and per-method timing breakdown from host step records."""
    timings: list[StepTiming] = []
    by_method: dict[str, dict[str, Any]] = {}

    # Extract timestamps (ms) or explicit durations gracefully
    parsed_ts: list[float | None] = []
    for s in steps:
        ts_ms = s.get("timestampMs")
        if ts_ms is None:
            ts_ms = s.get("timestamp_ms")
        try:
            parsed_ts.append(float(ts_ms) / 1000.0 if ts_ms is not None else None)
        except (TypeError, ValueError):
            parsed_ts.append(None)

    for i, step in enumerate(steps):
        method = str(step.get("method") or "unknown")
        receipt = str(step.get("receipt") or "")
        dur: float | None = None
        if step.get("duration_s") is not None:
            try:
                dur = max(0.0, float(step["duration_s"]))
            except (TypeError, ValueError):
                dur = None
        elif step.get("duration_ms") is not None:
            try:
                dur = max(0.0, float(step["duration_ms"]) / 1000.0)
            except (TypeError, ValueError):
                dur = None
        elif parsed_ts[i] is not None and i > 0 and parsed_ts[i - 1] is not None:
            delta = parsed_ts[i] - parsed_ts[i - 1]  # type: ignore[operator]
            # Guard against out-of-order clock jumps
            dur = round(delta, 4) if delta >= 0.0 else None

        is_slow = dur is not None and dur > slow_step_threshold_s
        timings.append(StepTiming(index=i, method=method, duration_s=dur, receipt=receipt, slow=is_slow))

        agg = by_method.setdefault(method, {"method": method, "calls": 0, "total_s": 0.0, "timed_calls": 0, "max_s": 0.0})
        agg["calls"] += 1
        if dur is not None:
            agg["total_s"] += dur
            agg["timed_calls"] += 1
            agg["max_s"] = max(agg["max_s"], dur)

    measured_total = sum(v["total_s"] for v in by_method.values())
    denom = total_wall_s if (total_wall_s and total_wall_s > 0) else (measured_total if measured_total > 0 else None)

    method_rows: list[dict[str, Any]] = []
    for row in by_method.values():
        tot = round(row["total_s"], 3)
        mean_s = round(row["total_s"] / row["timed_calls"], 3) if row["timed_calls"] else None
        pct = round((row["total_s"] / denom) * 100.0, 1) if (denom and row["timed_calls"]) else None
        method_rows.append(
            {
                "method": row["method"],
                "calls": row["calls"],
                "total_s": tot if row["timed_calls"] else None,
                "mean_s": mean_s,
                "max_s": round(row["max_s"], 3) if row["timed_calls"] else None,
                "pct_of_wall": pct,
            }
        )
    method_rows.sort(key=lambda r: (r["total_s"] if r["total_s"] is not None else -1.0, r["calls"]), reverse=True)

    slow_steps = [t.to_dict() for t in timings if t.slow]
    hints: list[str] = []
    obs_calls = by_method.get("observe_scene", {}).get("calls", 0)
    if obs_calls >= 6:
        hints.append(f"High observe_scene frequency ({obs_calls} calls); audit redundant re-observations.")
    wait_entry = by_method.get("wait_for_coffee_ready")
    if wait_entry and wait_entry["total_s"] >= 5.0:
        hints.append(
            f"Appliance wait (wait_for_coffee_ready={wait_entry['total_s']:.1f}s) is a prime "
            "candidate for asynchronous multi-order / cross-appliance pipelining."
        )

    return {
        "step_count": len(steps),
        "measured_total_s": round(measured_total, 3),
        "total_wall_s": round(total_wall_s, 3) if total_wall_s is not None else None,
        "slow_step_threshold_s": slow_step_threshold_s,
        "slow_steps": slow_steps,
        "by_method": method_rows,
        "steps": [t.to_dict() for t in timings],
        "optimization_hints": hints,
    }


def aggregate_trial_profiles(
    trials: Sequence[Mapping[str, Any]],
    *,
    slow_step_threshold_s: float = 8.0,
) -> dict[str, Any]:
    """Produce fleet/benchmark-level four-layer failure attribution and latency bottleneck report."""
    layer_counts: dict[str, int] = {"perception": 0, "control": 0, "device": 0, "policy": 0}
    code_counts: dict[str, int] = {}
    auto_repair_count = 0
    passed_count = 0
    failed_count = 0
    attributed_failures: list[dict[str, Any]] = []

    method_totals: dict[str, dict[str, Any]] = {}
    walls: list[float] = []

    for idx, t in enumerate(trials):
        attr = attribute_trial(t)
        if attr.layer == "none":
            passed_count += 1
        else:
            failed_count += 1
            layer_counts[attr.layer] = layer_counts.get(attr.layer, 0) + 1
            code_counts[attr.code] = code_counts.get(attr.code, 0) + 1
            if attr.auto_repair_eligible:
                auto_repair_count += 1
            attributed_failures.append(
                {
                    "trial_index": idx,
                    "trial_id": t.get("trial_id") or t.get("command_id") or f"trial_{idx}",
                    "policy_id": t.get("policy_id"),
                    "scenario_id": t.get("scenario_id") or t.get("condition"),
                    **attr.to_dict(),
                }
            )

        if t.get("wall_s") is not None:
            walls.append(float(t["wall_s"]))

        raw_steps = t.get("step_records") or t.get("steps")
        if not raw_steps and t.get("methods"):
            raw_steps = [{"method": m} for m in t["methods"]]
        if raw_steps:
            prof = profile_steps(
                raw_steps,
                total_wall_s=float(t["wall_s"]) if t.get("wall_s") is not None else None,
                slow_step_threshold_s=slow_step_threshold_s,
            )
            for row in prof["by_method"]:
                m = row["method"]
                agg = method_totals.setdefault(m, {"method": m, "total_calls": 0, "total_s": 0.0, "timed_trials": 0, "max_s": 0.0})
                agg["total_calls"] += row["calls"]
                if row["total_s"] is not None:
                    agg["total_s"] += row["total_s"]
                    agg["timed_trials"] += 1
                    agg["max_s"] = max(agg["max_s"], row["max_s"] or 0.0)

    bottlenecks = [
        {
            "method": v["method"],
            "total_calls": v["total_calls"],
            "mean_calls_per_trial": round(v["total_calls"] / len(trials), 2) if trials else 0.0,
            "total_s": round(v["total_s"], 3) if v["timed_trials"] else None,
            "mean_s_per_timed_trial": round(v["total_s"] / v["timed_trials"], 3) if v["timed_trials"] else None,
            "max_step_s": round(v["max_s"], 3) if v["timed_trials"] else None,
        }
        for v in method_totals.values()
    ]
    bottlenecks.sort(key=lambda r: (r["total_s"] if r["total_s"] is not None else -1.0, r["total_calls"]), reverse=True)

    return {
        "total_trials": len(trials),
        "passed_trials": passed_count,
        "failed_trials": failed_count,
        "pass_rate": round(passed_count / len(trials), 4) if trials else 0.0,
        "mean_wall_s": round(sum(walls) / len(walls), 3) if walls else 0.0,
        "attribution_counts": layer_counts,
        "failure_code_counts": code_counts,
        "auto_repair_eligible_failures": auto_repair_count,
        "attributed_failures": attributed_failures,
        "method_bottlenecks": bottlenecks,
    }
