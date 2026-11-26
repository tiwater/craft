#!/usr/bin/env python3
"""TRAX Continuous Self-Audit & Accuracy x Speed Pareto Gate (`make self-audit`).

Runs deterministic hard checks + multi-seed/cross-embodiment benchmark trials,
computes accuracy and p50/p95 wall-clock latency + per-tool bottlenecks, evaluates
against the latest accepted baseline in an append-only JSONL ledger, and exits
non-zero on any hard safety/architecture veto or accuracy/latency regression.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "src"))

from trax.config import SceneConfig
from trax.host_driver import FixtureHostDriver
from trax.mcp_protocol import MCPClient, build_driver_mcp_server
from trax.sandbox import GovernedMCPProxy, check_mcp_policy_ast, execute_sandboxed_python
from trax.self_audit import AuditLedger, evaluate_pareto_gate, percentile, profile_mcp_trace


def _git_sha() -> str:
    res = subprocess.run(
        ["git", "rev-parse", "--short", "HEAD"],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        check=False,
    )
    return res.stdout.strip() or "unknown"


def _run_cmd_check(cmd: list[str], *, timeout_s: float = 120.0) -> tuple[bool, float, str]:
    t0 = time.monotonic()
    res = subprocess.run(
        cmd,
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=timeout_s,
        check=False,
    )
    ms = round((time.monotonic() - t0) * 1000.0, 2)
    tail = (res.stdout + "\n" + res.stderr).strip().splitlines()[-3:]
    return res.returncode == 0, ms, " | ".join(tail)


def _run_deterministic_protocol_trials(seeds: list[int], tmp_dir: Path) -> dict[str, Any]:
    """Run deterministic multi-seed MCP governance & unknown-robot persistent-stdio trials."""
    latencies_ms: list[float] = []
    passed = 0
    total = 0
    dup_non_idemp = 0
    unreconciled = 0
    all_trace: list[dict[str, Any]] = []
    failures: list[dict[str, Any]] = []

    # 1. Unknown external robot over persistent stdio MCP (zero core coupling)
    unknown_script = REPO_ROOT / "examples" / "custom_mcp_robot" / "unknown_arm_mcp.py"
    policy_unknown = (
        "def run_policy(mcp, params):\n"
        "    mcp.read_gantry_pose()\n"
        "    mcp.move_gantry_xy(x_mm=120.0, y_mm=80.0)\n"
        "    mcp.fire_dispense_valve(command_id=f\"audit-{params['seed']}\")\n"
        "    return {'status': 'completed'}\n"
    )
    for seed in seeds:
        total += 1
        t0 = time.monotonic()
        client = MCPClient(transport="stdio", command=[sys.executable, str(unknown_script)], timeout_s=10.0)
        try:
            _, tools_resp, _ = client.rpc("tools/list", {})
            tools = (tools_resp.get("result") or {}).get("tools") or []
            ast_res = check_mcp_policy_ast(policy_unknown, allowed_tools=[t["name"] for t in tools])
            proxy = GovernedMCPProxy(
                client,
                tools,
                journal_path=tmp_dir / f"stdio_s{seed}.sqlite",
                run_tag=f"audit-stdio-s{seed}",
            )
            ret, err = execute_sandboxed_python(
                policy_unknown,
                proxy,
                entry_fn="run_policy",
                params={"seed": seed},
            )
            _, v_resp, _ = client.rpc("tools/call", {"name": "verify_task", "arguments": {}})
            v_ok = bool(((v_resp.get("result") or {}).get("structuredContent") or {}).get("passed"))
            ms = round((time.monotonic() - t0) * 1000.0, 2)
            latencies_ms.append(ms)
            all_trace.extend(proxy.trace)
            if any(c > 1 for c in proxy.non_idempotent_counts.values()):
                dup_non_idemp += 1
            if proxy.unknown_events_observed > 0 and not proxy.queried_after_unknown:
                unreconciled += 1
            if ast_res["compliant"] and err is None and v_ok:
                passed += 1
            else:
                failures.append({"suite": "unknown_stdio_mcp", "seed": seed, "error": err or "verify_failed"})
        finally:
            client.close()

    # 2. Local fixture MCP governance check
    cfg = SceneConfig.from_file(REPO_ROOT / "examples" / "single_cup_coffee_config.json")
    drv = FixtureHostDriver(cfg)
    srv = build_driver_mcp_server(drv)
    local_client = MCPClient(transport="local", local_server=srv)
    _, tools_resp, _ = local_client.rpc("tools/list", {})
    tools = (tools_resp.get("result") or {}).get("tools") or []
    fixture_policy = (
        "def run_policy(mcp, params):\n"
        "    mcp.observe_scene()\n"
        "    mcp.start_coffee_machine(command_id=f\"fix-{params['seed']}\")\n"
        "    return {'status': 'completed'}\n"
    )
    for seed in seeds:
        total += 1
        t0 = time.monotonic()
        proxy = GovernedMCPProxy(
            local_client,
            tools,
            journal_path=tmp_dir / f"fixture_s{seed}.sqlite",
            run_tag=f"audit-fix-s{seed}",
        )
        ret, err = execute_sandboxed_python(fixture_policy, proxy, entry_fn="run_policy", params={"seed": seed})
        ms = round((time.monotonic() - t0) * 1000.0, 2)
        latencies_ms.append(ms)
        all_trace.extend(proxy.trace)
        if any(c > 1 for c in proxy.non_idempotent_counts.values()):
            dup_non_idemp += 1
        if err is None and isinstance(ret, dict) and ret.get("status") == "completed":
            passed += 1
        else:
            failures.append({"suite": "fixture_mcp", "seed": seed, "error": err or "failed"})

    return {
        "passed": passed,
        "total": total,
        "accuracy": round(passed / max(1, total), 4),
        "p50_ms": percentile(latencies_ms, 50.0),
        "p95_ms": percentile(latencies_ms, 95.0),
        "duplicate_non_idempotent": dup_non_idemp,
        "unreconciled_unknown": unreconciled,
        "profile": profile_mcp_trace(all_trace),
        "failures": failures,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="TRAX Self-Audit & Accuracy x Speed Pareto Gate")
    parser.add_argument(
        "--ledger",
        default=str(REPO_ROOT / "workspace" / "traces" / "self_audit_ledger.jsonl"),
        help="Path to append-only JSONL self-audit ledger",
    )
    parser.add_argument("--suite", default="trax_core_contract_v1", help="Benchmark suite name in ledger")
    parser.add_argument("--seeds", default="0,1,2", help="Comma-separated seeds for deterministic audit trials")
    parser.add_argument("--full-pytest", action="store_true", help="Run full pytest suite instead of fast contract subset")
    parser.add_argument("--max-p95-regression-pct", type=float, default=20.0, help="Max allowed p95 regression vs baseline")
    parser.add_argument("--min-speedup-pct", type=float, default=5.0, help="Minimum p95 speedup to promote as speedup_pareto")
    parser.add_argument("--dry-run", action="store_true", help="Do not append result to ledger")
    args = parser.parse_args()

    py = sys.executable
    arch_ok, arch_ms, arch_msg = _run_cmd_check([py, str(REPO_ROOT / "tools" / "check_arch.py")])
    pytest_cmd = [py, "-m", "pytest", "-q"]
    if not args.full_pytest:
        pytest_cmd.extend(
            [
                "tests/test_architecture_contracts.py",
                "tests/test_mcp_protocol.py",
                "tests/test_knowledge_base.py",
                "tests/test_self_audit.py",
            ]
        )
    test_ok, test_ms, test_msg = _run_cmd_check(pytest_cmd, timeout_s=180.0)

    seeds = [int(x.strip()) for x in args.seeds.split(",") if x.strip()]
    with tempfile.TemporaryDirectory(prefix="trax-self-audit-") as tmp:
        bench = _run_deterministic_protocol_trials(seeds, Path(tmp))

    candidate = {
        "suite": args.suite,
        "git_sha": _git_sha(),
        "evidence_source": "fixture",
        "hard_checks": {
            "check_arch": arch_ok,
            "pytest": test_ok,
        },
        "hard_check_timings_ms": {
            "check_arch": arch_ms,
            "pytest": test_ms,
        },
        "hard_check_summaries": {
            "check_arch": arch_msg,
            "pytest": test_msg,
        },
        "accuracy": bench["accuracy"],
        "passed": bench["passed"],
        "total": bench["total"],
        "p50_ms": bench["p50_ms"],
        "p95_ms": bench["p95_ms"],
        "duplicate_non_idempotent": bench["duplicate_non_idempotent"],
        "unreconciled_unknown": bench["unreconciled_unknown"],
        "bottleneck_tool": bench["profile"].get("bottleneck_tool"),
        "optimization_hints": bench["profile"].get("optimization_hints") or [],
        "failures": bench["failures"],
    }

    ledger = AuditLedger(args.ledger)
    baseline = ledger.latest_accepted_baseline(suite=args.suite)
    decision = evaluate_pareto_gate(
        baseline,
        candidate,
        max_p95_regression_pct=args.max_p95_regression_pct,
        min_speedup_pct=args.min_speedup_pct,
    )
    record = {**candidate, "decision": decision.to_dict()}
    if not args.dry_run:
        ledger.append(record)

    print(json.dumps(record, indent=2, ensure_ascii=False))
    return 0 if decision.accepted else 1


if __name__ == "__main__":
    raise SystemExit(main())
