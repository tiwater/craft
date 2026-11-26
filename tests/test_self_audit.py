"""Tests for TRAX execution profiler, dual-gate Pareto evaluator, and self-audit ledger."""
from __future__ import annotations

from pathlib import Path

from trax.self_audit import AuditLedger, evaluate_pareto_gate, percentile, profile_mcp_trace


def test_profile_mcp_trace_detects_bottleneck_and_redundant_calls() -> None:
    trace = [
        {"action": "get_robot_state", "arguments": {}, "latency_ms": 2.0, "read_only": True, "non_idempotent": False},
        {"action": "get_robot_state", "arguments": {}, "latency_ms": 2.5, "read_only": True, "non_idempotent": False},
        {"action": "move_ee", "arguments": {"x": 0.3, "y": 0.0, "z": 1.15}, "latency_ms": 40.0, "read_only": False, "non_idempotent": False},
        {"action": "move_ee", "arguments": {"x": 0.3, "y": 0.0, "z": 1.06}, "latency_ms": 45.0, "read_only": False, "non_idempotent": False},
        {"action": "move_ee", "arguments": {"x": 0.38, "y": -0.17, "z": 1.12}, "latency_ms": 50.0, "read_only": False, "non_idempotent": False},
        {"action": "activate_workstation", "arguments": {"command_id": "c1"}, "latency_ms": 10.0, "read_only": False, "non_idempotent": True},
    ]
    prof = profile_mcp_trace(trace)
    assert prof["total_calls"] == 6
    assert prof["non_idempotent_calls"] == 1
    assert prof["bottleneck_tool"] == "move_ee"
    assert len(prof["redundant_consecutive_calls"]) == 1
    assert any("consecutive identical" in h for h in prof["optimization_hints"])
    assert any("Primary latency bottleneck" in h for h in prof["optimization_hints"])
    assert percentile([10.0, 20.0, 30.0, 40.0], 50.0) == 25.0


def test_pareto_gate_enforces_hard_veto_and_accuracy_speed_frontier() -> None:
    base = {
        "hard_checks": {"check_arch": True, "pytest": True},
        "accuracy": 0.90,
        "p95_ms": 100.0,
        "duplicate_non_idempotent": 0,
        "unreconciled_unknown": 0,
    }

    # 1. Hard veto rejects even if accuracy & speed improved
    d_veto = evaluate_pareto_gate(
        base,
        {**base, "accuracy": 1.0, "p95_ms": 50.0, "hard_checks": {"check_arch": False, "pytest": True}},
    )
    assert d_veto.accepted is False
    assert d_veto.reason_code == "hard_veto"
    assert "check_arch" in d_veto.hard_veto_failures

    d_dup = evaluate_pareto_gate(base, {**base, "duplicate_non_idempotent": 1})
    assert d_dup.accepted is False
    assert d_dup.reason_code == "hard_veto"

    # 2. Accuracy regression rejected even if much faster
    d_acc_reg = evaluate_pareto_gate(base, {**base, "accuracy": 0.85, "p95_ms": 40.0})
    assert d_acc_reg.accepted is False
    assert d_acc_reg.reason_code == "accuracy_regression"

    # 3. Severe p95 latency regression rejected even if accuracy improved
    d_lat_reg = evaluate_pareto_gate(base, {**base, "accuracy": 0.95, "p95_ms": 150.0}, max_p95_regression_pct=15.0)
    assert d_lat_reg.accepted is False
    assert d_lat_reg.reason_code == "latency_regression"

    # 4. Accuracy gain within latency budget -> ACCEPTED
    d_acc_gain = evaluate_pareto_gate(base, {**base, "accuracy": 0.95, "p95_ms": 105.0})
    assert d_acc_gain.accepted is True
    assert d_acc_gain.reason_code == "accuracy_gain"

    # 5. Equal accuracy with >= 5% speedup -> ACCEPTED (speedup_pareto)
    d_speed = evaluate_pareto_gate(base, {**base, "accuracy": 0.90, "p95_ms": 88.0}, min_speedup_pct=5.0)
    assert d_speed.accepted is True
    assert d_speed.reason_code == "speedup_pareto"
    assert d_speed.p95_speedup_pct == 12.0

    # 6. Initial baseline -> ACCEPTED
    d_init = evaluate_pareto_gate(None, base)
    assert d_init.accepted is True
    assert d_init.reason_code == "initial_baseline"


def test_audit_ledger_tracks_latest_accepted_baseline(tmp_path: Path) -> None:
    ledger = AuditLedger(tmp_path / "ledger.jsonl")
    assert ledger.latest_accepted_baseline(suite="s1") is None

    ledger.append({"suite": "s1", "accuracy": 0.90, "p95_ms": 100.0, "decision": {"accepted": True, "reason_code": "initial_baseline"}})
    ledger.append({"suite": "s1", "accuracy": 0.80, "p95_ms": 90.0, "decision": {"accepted": False, "reason_code": "accuracy_regression"}})
    b1 = ledger.latest_accepted_baseline(suite="s1")
    assert b1 is not None and b1["accuracy"] == 0.90

    ledger.append({"suite": "s1", "accuracy": 0.90, "p95_ms": 80.0, "decision": {"accepted": True, "reason_code": "speedup_pareto"}})
    b2 = ledger.latest_accepted_baseline(suite="s1")
    assert b2 is not None and b2["p95_ms"] == 80.0
