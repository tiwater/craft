"""Deterministic execution profiler, Pareto frontier gate, and immutable self-audit ledger.

Pure Python stdlib (zero third-party dependencies, zero hardcoded robot/domain tokens).
Provides the measurement & governance backbone for TRAX's continuous self-audit loop:
  1. `profile_mcp_trace()`      — Per-tool latency breakdown, p95, and redundancy diagnostics
  2. `evaluate_pareto_gate()`   — Dual-gate ACCEPT/REJECT: Hard Safety/Arch Veto + Accuracy x Speed Pareto
  3. `AuditLedger`              — Append-only JSONL experiment ledger keyed by Git SHA & evidence tier
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


def _utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def percentile(values: Sequence[float], pct: float) -> float:
    """Compute linear-interpolated percentile over a numeric sequence (empty -> 0.0)."""
    if not values:
        return 0.0
    ordered = sorted(float(v) for v in values)
    if len(ordered) == 1:
        return round(ordered[0], 3)
    k = (max(0.0, min(100.0, float(pct))) / 100.0) * (len(ordered) - 1)
    lo = int(math.floor(k))
    hi = int(math.ceil(k))
    if lo == hi:
        return round(ordered[lo], 3)
    frac = k - lo
    return round(ordered[lo] + (ordered[hi] - ordered[lo]) * frac, 3)


def profile_mcp_trace(trace: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """Summarize per-tool latency, call counts, and redundant-step hints from an MCP trace."""
    by_tool: dict[str, list[float]] = {}
    total_ms = 0.0
    read_only_calls = 0
    non_idempotent_calls = 0
    redundant_consecutive: list[dict[str, Any]] = []

    prev_sig: tuple[str, str] | None = None
    for idx, step in enumerate(trace, start=1):
        action = str(step.get("action") or "unknown")
        ms = max(0.0, float(step.get("latency_ms") or 0.0))
        by_tool.setdefault(action, []).append(ms)
        total_ms += ms
        if step.get("read_only"):
            read_only_calls += 1
        if step.get("non_idempotent"):
            non_idempotent_calls += 1

        args_sig = json.dumps(step.get("arguments") or {}, sort_keys=True, ensure_ascii=False)
        sig = (action, args_sig)
        if prev_sig == sig:
            redundant_consecutive.append({"step": idx, "action": action, "arguments": step.get("arguments") or {}})
        prev_sig = sig

    tool_stats: list[dict[str, Any]] = []
    for name, latencies in by_tool.items():
        tool_stats.append(
            {
                "tool": name,
                "calls": len(latencies),
                "total_ms": round(sum(latencies), 2),
                "avg_ms": round(sum(latencies) / len(latencies), 2),
                "p95_ms": round(percentile(latencies, 95.0), 2),
            }
        )
    tool_stats.sort(key=lambda r: (-r["total_ms"], r["tool"]))

    bottleneck = tool_stats[0] if tool_stats else None
    hints: list[str] = []
    if redundant_consecutive:
        hints.append(
            f"Detected {len(redundant_consecutive)} consecutive identical MCP call(s) "
            f"(first: `{redundant_consecutive[0]['action']}` at step {redundant_consecutive[0]['step']})."
        )
    if bottleneck and total_ms > 0 and (bottleneck["total_ms"] / total_ms) >= 0.45 and bottleneck["calls"] >= 3:
        share = round((bottleneck["total_ms"] / total_ms) * 100.0, 1)
        hints.append(
            f"Primary latency bottleneck: `{bottleneck['tool']}` accounts for {share}% of total MCP time "
            f"({bottleneck['total_ms']}ms across {bottleneck['calls']} calls)."
        )
    if len(trace) > 0 and read_only_calls > max(4, len(trace) // 2):
        hints.append(
            f"Read-only polling accounts for {read_only_calls}/{len(trace)} calls; consider caching static observations."
        )

    return {
        "total_calls": len(trace),
        "read_only_calls": read_only_calls,
        "non_idempotent_calls": non_idempotent_calls,
        "total_mcp_ms": round(total_ms, 2),
        "p50_call_ms": round(percentile([float(s.get("latency_ms") or 0.0) for s in trace], 50.0), 2),
        "p95_call_ms": round(percentile([float(s.get("latency_ms") or 0.0) for s in trace], 95.0), 2),
        "bottleneck_tool": bottleneck["tool"] if bottleneck else None,
        "by_tool": tool_stats,
        "redundant_consecutive_calls": redundant_consecutive,
        "optimization_hints": hints,
    }


@dataclass(frozen=True)
class ParetoDecision:
    """Structured dual-gate verdict comparing a candidate run against an accepted baseline."""

    accepted: bool
    reason_code: str
    detail: str
    accuracy_delta: float
    p95_speedup_pct: float
    hard_veto_failures: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return {
            "accepted": self.accepted,
            "reason_code": self.reason_code,
            "detail": self.detail,
            "accuracy_delta": round(self.accuracy_delta, 4),
            "p95_speedup_pct": round(self.p95_speedup_pct, 2),
            "hard_veto_failures": list(self.hard_veto_failures),
        }


def evaluate_pareto_gate(
    baseline: Mapping[str, Any] | None,
    candidate: Mapping[str, Any],
    *,
    accuracy_tolerance: float = 0.0,
    max_p95_regression_pct: float = 15.0,
    min_speedup_pct: float = 5.0,
) -> ParetoDecision:
    """Dual-gate evaluator: Gate A (Hard Safety/Arch Veto) -> Gate B (Accuracy x Speed Pareto).

    Expected keys in `baseline` and `candidate`:
      - `hard_checks`: mapping of check_name -> bool (all must be True)
      - `accuracy`: float in [0.0, 1.0]
      - `p95_ms`: float >= 0.0
      - `duplicate_non_idempotent`: int (must be 0)
      - `unreconciled_unknown`: int (must be 0)
    """
    hard_checks = dict(candidate.get("hard_checks") or {})
    vetoes: list[str] = [k for k, ok in sorted(hard_checks.items()) if not bool(ok)]
    if int(candidate.get("duplicate_non_idempotent") or 0) > 0:
        vetoes.append("duplicate_non_idempotent>0")
    if int(candidate.get("unreconciled_unknown") or 0) > 0:
        vetoes.append("unreconciled_unknown>0")

    if vetoes:
        return ParetoDecision(
            accepted=False,
            reason_code="hard_veto",
            detail=f"Rejected by hard safety/architecture veto: {', '.join(vetoes)}",
            accuracy_delta=0.0,
            p95_speedup_pct=0.0,
            hard_veto_failures=tuple(vetoes),
        )

    cand_acc = float(candidate.get("accuracy") or 0.0)
    cand_p95 = float(candidate.get("p95_ms") or 0.0)

    if baseline is None:
        return ParetoDecision(
            accepted=True,
            reason_code="initial_baseline",
            detail=f"Established initial accepted baseline (accuracy={cand_acc:.3f}, p95={cand_p95:.1f}ms).",
            accuracy_delta=0.0,
            p95_speedup_pct=0.0,
        )

    base_acc = float(baseline.get("accuracy") or 0.0)
    base_p95 = float(baseline.get("p95_ms") or 0.0)
    acc_delta = cand_acc - base_acc
    if base_p95 > 1e-6:
        speedup_pct = ((base_p95 - cand_p95) / base_p95) * 100.0
    else:
        speedup_pct = 0.0

    # Reject any accuracy regression beyond tolerance
    if acc_delta < -abs(float(accuracy_tolerance)) - 1e-9:
        return ParetoDecision(
            accepted=False,
            reason_code="accuracy_regression",
            detail=(
                f"Accuracy regressed from {base_acc:.3f} to {cand_acc:.3f} "
                f"(delta={acc_delta:+.3f}, tolerance={accuracy_tolerance:.3f})."
            ),
            accuracy_delta=acc_delta,
            p95_speedup_pct=speedup_pct,
        )

    # Reject unacceptable latency regression even when accuracy improves or stays flat
    if speedup_pct < -abs(float(max_p95_regression_pct)):
        return ParetoDecision(
            accepted=False,
            reason_code="latency_regression",
            detail=(
                f"p95 latency regressed from {base_p95:.1f}ms to {cand_p95:.1f}ms "
                f"({-speedup_pct:.1f}% slower, max allowed {max_p95_regression_pct:.1f}%)."
            ),
            accuracy_delta=acc_delta,
            p95_speedup_pct=speedup_pct,
        )

    # Accept strict accuracy gain (with bounded latency)
    if acc_delta > 1e-9:
        return ParetoDecision(
            accepted=True,
            reason_code="accuracy_gain",
            detail=(
                f"Accepted accuracy improvement {base_acc:.3f} -> {cand_acc:.3f} ({acc_delta:+.3f}) "
                f"with p95 {base_p95:.1f}ms -> {cand_p95:.1f}ms ({speedup_pct:+.1f}%)."
            ),
            accuracy_delta=acc_delta,
            p95_speedup_pct=speedup_pct,
        )

    # Accept meaningful Pareto speedup at equal/non-regressing accuracy
    if speedup_pct >= float(min_speedup_pct):
        return ParetoDecision(
            accepted=True,
            reason_code="speedup_pareto",
            detail=(
                f"Accepted Pareto speedup {base_p95:.1f}ms -> {cand_p95:.1f}ms ({speedup_pct:+.1f}%) "
                f"at maintained accuracy {cand_acc:.3f}."
            ),
            accuracy_delta=acc_delta,
            p95_speedup_pct=speedup_pct,
        )

    # Equal accuracy and within noise band -> hold (no regression, no improvement)
    return ParetoDecision(
        accepted=True,
        reason_code="parity_no_regression",
        detail=(
            f"Maintained baseline parity (accuracy={cand_acc:.3f}, p95={cand_p95:.1f}ms, "
            f"delta={speedup_pct:+.1f}% within +/-{min_speedup_pct:.1f}% threshold)."
        ),
        accuracy_delta=acc_delta,
        p95_speedup_pct=speedup_pct,
    )


class AuditLedger:
    """Append-only JSONL ledger recording every self-audit & optimization trial."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def read_all(self) -> list[dict[str, Any]]:
        if not self.path.is_file():
            return []
        rows: list[dict[str, Any]] = []
        for line in self.path.read_text(encoding="utf-8", errors="replace").splitlines():
            s = line.strip()
            if not s:
                continue
            try:
                rows.append(json.loads(s))
            except ValueError:
                continue
        return rows

    def latest_accepted_baseline(self, *, suite: str | None = None) -> dict[str, Any] | None:
        rows = self.read_all()
        for row in reversed(rows):
            if suite and row.get("suite") != suite:
                continue
            dec = row.get("decision") or {}
            if dec.get("accepted") and dec.get("reason_code") in (
                "initial_baseline",
                "accuracy_gain",
                "speedup_pareto",
            ):
                return row
        return None

    def append(self, record: Mapping[str, Any]) -> dict[str, Any]:
        entry = {"recorded_utc": _utc_iso(), **dict(record)}
        with self.path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(entry, ensure_ascii=False) + "\n")
        return entry
