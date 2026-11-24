"""Cross-store portability static checker and multi-order throughput evaluation.

Ensures policies remain decoupled from site-specific physical coordinates so a
verified role policy transfers cleanly across stores with different layouts, and
quantifies multi-order throughput (orders/hour) plus serial-vs-pipelined schedule
analysis grounded in measured step timings.
"""
from __future__ import annotations

import ast
from dataclasses import asdict, dataclass
import re
import time
from typing import Any, Mapping, Sequence

from .host_driver import SceneHostDriver


_URL_OR_PATH_RE = re.compile(r"(?:https?://|/home/|/Users/|127\.0\.0\.1|\b\d{1,3}(?:\.\d{1,3}){3}\b)")


@dataclass(frozen=True)
class PortabilityFinding:
    line: int
    rule: str  # hardcoded_coordinate_literal | hardcoded_world_comparison | hardcoded_endpoint_or_path | syntax_error
    severity: str  # error | warning
    detail: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _is_numeric_literal(node: ast.AST) -> bool:
    if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)) and not isinstance(node.value, bool):
        return True
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
        return _is_numeric_literal(node.operand)
    return False


def _numeric_value(node: ast.AST) -> float | None:
    if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)) and not isinstance(node.value, bool):
        return float(node.value)
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub):
        v = _numeric_value(node.operand)
        return -v if v is not None else None
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.UAdd):
        return _numeric_value(node.operand)
    return None


def check_policy_portability(source: str) -> dict[str, Any]:
    """Statically check a policy for hardcoded site coordinates, paths, or endpoints."""
    findings: list[PortabilityFinding] = []
    try:
        tree = ast.parse(source)
    except SyntaxError as exc:
        findings.append(
            PortabilityFinding(
                line=exc.lineno or 1,
                rule="syntax_error",
                severity="error",
                detail=f"Cannot analyze portability due to SyntaxError: {exc}",
            )
        )
        return {"portable": False, "findings": [f.to_dict() for f in findings]}

    for node in ast.walk(tree):
        # 1. Detect 2D/3D coordinate literals like [0.45, 0.0, 1.075] or (0.38, -0.17)
        if isinstance(node, (ast.List, ast.Tuple)) and len(node.elts) in (2, 3):
            if all(_is_numeric_literal(e) for e in node.elts):
                vals = [_numeric_value(e) for e in node.elts]
                if any(isinstance(v, float) and (v != int(v)) for v in vals if v is not None):
                    findings.append(
                        PortabilityFinding(
                            line=getattr(node, "lineno", 1),
                            rule="hardcoded_coordinate_literal",
                            severity="error",
                            detail=f"Hardcoded spatial coordinate literal {vals}; bind via semantic slot or host state instead.",
                        )
                    )

        # 2. Detect direct subtraction against magic float coordinates on indexed world vectors, e.g. cup_xyz[0] - 0.45
        if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Sub):
            left_sub = isinstance(node.left, ast.Subscript)
            right_num = _numeric_value(node.right)
            if left_sub and right_num is not None and abs(right_num) >= 0.05 and (right_num != int(right_num)):
                findings.append(
                    PortabilityFinding(
                        line=getattr(node, "lineno", 1),
                        rule="hardcoded_world_comparison",
                        severity="error",
                        detail=f"Direct coordinate subtraction against magic float {right_num}; consume host semantic state (e.g. l2_pouring_safety) instead.",
                    )
                )

        # 3. Detect hardcoded URLs, IPs, or private home paths in string literals
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            if _URL_OR_PATH_RE.search(node.value):
                findings.append(
                    PortabilityFinding(
                        line=getattr(node, "lineno", 1),
                        rule="hardcoded_endpoint_or_path",
                        severity="error",
                        detail=f"Policy embeds site-specific URL/IP/path: {node.value!r}",
                    )
                )

    has_errors = any(f.severity == "error" for f in findings)
    return {
        "portable": not has_errors,
        "error_count": sum(1 for f in findings if f.severity == "error"),
        "warning_count": sum(1 for f in findings if f.severity == "warning"),
        "findings": [f.to_dict() for f in findings],
    }


def evaluate_multi_order_throughput(
    driver: SceneHostDriver,
    policy_source: str,
    scenarios: Sequence[Mapping[str, Any]],
    *,
    policy_id: str = "candidate",
) -> dict[str, Any]:
    """Run a continuous batch of orders through ``driver`` and measure orders/hour throughput."""
    if not scenarios:
        raise ValueError("scenarios must be non-empty")

    batch_started = time.monotonic()
    trials: list[dict[str, Any]] = []
    completed = 0
    sum_order_wall = 0.0

    for idx, scen in enumerate(scenarios, start=1):
        scen_dict = dict(scen)
        driver.reset_scenario(scen_dict)
        cmd_id = f"tput-{policy_id}-o{idx}-{time.time_ns()}"
        res = driver.run_policy_scenario(policy_source, command_id=cmd_id, scenario=scen_dict)
        trials.append({"order_index": idx, **res})
        sum_order_wall += float(res.get("wall_s") or 0.0)
        if res.get("task_passed"):
            completed += 1

    batch_wall = max(1e-6, time.monotonic() - batch_started)
    eff_wall = sum_order_wall if sum_order_wall > 0 else batch_wall
    mean_order_s = eff_wall / len(scenarios)
    orders_per_hour = round((completed / eff_wall) * 3600.0, 2) if completed > 0 else 0.0

    return {
        "policy_id": policy_id,
        "evidence_source": driver.evidence_source,
        "total_orders": len(scenarios),
        "completed_orders": completed,
        "completion_rate": round(completed / len(scenarios), 4),
        "sum_execution_wall_s": round(sum_order_wall, 3),
        "batch_wall_s": round(batch_wall, 3),
        "mean_order_s": round(mean_order_s, 3),
        "throughput_orders_per_hour": orders_per_hour,
        "trials": trials,
    }


def simulate_pipeline_schedule(
    *,
    num_orders: int,
    robot_prep_s: float,
    appliance_busy_s: float,
    robot_deliver_s: float,
    appliance_slots: int = 1,
) -> dict[str, Any]:
    """Analytical serial vs. asynchronous pipelined throughput model from measured step timings.

    Explicitly tagged ``evidence_source='schedule_model_from_measured_steps'`` so it is
    never confused with live multi-order hardware execution.
    """
    if num_orders < 1 or appliance_slots < 1:
        raise ValueError("num_orders and appliance_slots must be >= 1")
    if robot_prep_s < 0 or appliance_busy_s < 0 or robot_deliver_s < 0:
        raise ValueError("Step durations must be non-negative")

    serial_order_s = robot_prep_s + appliance_busy_s + robot_deliver_s
    serial_total_s = serial_order_s * num_orders
    serial_oph = round((num_orders / serial_total_s) * 3600.0, 2) if serial_total_s > 0 else 0.0

    # Single robot arm + `appliance_slots` concurrent appliance(s):
    # Robot can start next order's prep while previous order(s) are brewing/heating.
    robot_free_at = 0.0
    slot_free_at = [0.0 for _ in range(appliance_slots)]
    order_ready_at: list[tuple[float, int]] = []  # (ready_time, slot_idx)

    finished_at: list[float] = []
    started = 0
    while len(finished_at) < num_orders:
        # Prefer finishing any ready appliance order first to free the slot
        ready_now = [item for item in order_ready_at if item[0] <= robot_free_at + 1e-9]
        if ready_now:
            ready_now.sort(key=lambda x: x[0])
            ready_t, s_idx = ready_now[0]
            order_ready_at.remove((ready_t, s_idx))
            start_del = max(robot_free_at, ready_t)
            end_del = start_del + robot_deliver_s
            robot_free_at = end_del
            slot_free_at[s_idx] = end_del
            finished_at.append(end_del)
            continue

        # Otherwise start a new order if an appliance slot is free
        free_slots = [i for i, t in enumerate(slot_free_at) if t <= robot_free_at + 1e-9]
        if started < num_orders and free_slots:
            s_idx = free_slots[0]
            start_prep = max(robot_free_at, slot_free_at[s_idx])
            end_prep = start_prep + robot_prep_s
            robot_free_at = end_prep
            slot_free_at[s_idx] = float("inf")  # occupied until delivered
            order_ready_at.append((end_prep + appliance_busy_s, s_idx))
            started += 1
            continue

        # Otherwise advance robot clock to the earliest ready appliance order
        order_ready_at.sort(key=lambda x: x[0])
        ready_t, s_idx = order_ready_at.pop(0)
        start_del = max(robot_free_at, ready_t)
        end_del = start_del + robot_deliver_s
        robot_free_at = end_del
        slot_free_at[s_idx] = end_del
        finished_at.append(end_del)

    pipelined_total_s = max(finished_at) if finished_at else 0.0
    pipelined_oph = round((num_orders / pipelined_total_s) * 3600.0, 2) if pipelined_total_s > 0 else 0.0
    speedup_pct = round(((pipelined_oph / serial_oph) - 1.0) * 100.0, 1) if serial_oph > 0 else 0.0

    return {
        "evidence_source": "schedule_model_from_measured_steps",
        "num_orders": num_orders,
        "appliance_slots": appliance_slots,
        "serial_total_s": round(serial_total_s, 3),
        "serial_orders_per_hour": serial_oph,
        "pipelined_total_s": round(pipelined_total_s, 3),
        "pipelined_orders_per_hour": pipelined_oph,
        "projected_throughput_gain_pct": speedup_pct,
    }
