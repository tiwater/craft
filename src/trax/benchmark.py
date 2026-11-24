"""Host-agnostic paired benchmark runner and self-contained HTML report generator.

Operates strictly through ``SceneHostDriver`` so the exact same benchmark suite can
run offline against ``FixtureHostDriver`` or live against any compatible HTTP
simulator/robot host without coupling TRAX core to a specific host repository.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import html
import json
import math
from pathlib import Path
import time
from typing import Any, Callable, Mapping, Sequence

from .contracts import digest
from .host_driver import SceneHostDriver
from .journal import Journal


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _write_json_atomic(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    tmp.replace(path)


def wilson_ci_95(successes: int, total: int) -> tuple[float, float]:
    """95% Wilson score confidence interval for a binomial proportion."""
    if total <= 0:
        return (0.0, 0.0)
    z = 1.959963984540054
    p = successes / total
    denom = 1.0 + (z * z) / total
    center = (p + (z * z) / (2.0 * total)) / denom
    half = (z * math.sqrt((p * (1.0 - p) + (z * z) / (4.0 * total)) / total)) / denom
    return (round(max(0.0, center - half), 4), round(min(1.0, center + half), 4))


def percentile(values: Sequence[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, int(math.ceil((pct / 100.0) * len(ordered))) - 1))
    return round(float(ordered[idx]), 3)


@dataclass(frozen=True)
class BenchmarkScenario:
    scenario_id: str
    label: str
    category: str  # normal | pose_perturbation | fault_recovery | safety_or_ambiguity
    preset: str = "default"
    cup_a_xy: tuple[float, float] | None = None
    cup_b_xy: tuple[float, float] | None = None
    order_id: str = "A042"
    target_slot: str = "$SLOT_PICKUP_1"
    fault_inject_slip_xy_m: tuple[float, float] | None = None
    simulate_timeout: bool = False
    expect_status: str = "completed"
    max_delivery_error_mm: float = 15.0

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        if self.cup_a_xy is not None:
            d["cup_a_xy"] = list(self.cup_a_xy)
        if self.cup_b_xy is not None:
            d["cup_b_xy"] = list(self.cup_b_xy)
        if self.fault_inject_slip_xy_m is not None:
            d["fault_inject_slip_xy_m"] = list(self.fault_inject_slip_xy_m)
        return d


@dataclass(frozen=True)
class PolicyEntry:
    policy_id: str
    label: str
    source: str
    origin: str = "craft_generated"

    @property
    def source_sha256(self) -> str:
        return digest(self.source)

    def to_meta(self) -> dict[str, Any]:
        return {
            "policy_id": self.policy_id,
            "label": self.label,
            "origin": self.origin,
            "source_sha256": self.source_sha256,
        }


def default_coffee_benchmark_suite() -> tuple[BenchmarkScenario, ...]:
    """Balanced multi-condition coffee suite: normal, pose offset, slip+timeout faults, ambiguity/hazard."""
    return (
        BenchmarkScenario(
            scenario_id="s1_default",
            label="Default single-cup",
            category="normal",
            preset="default",
        ),
        BenchmarkScenario(
            scenario_id="s2_pose_offset_preset",
            label="Pose offset (+20,+64 mm)",
            category="pose_perturbation",
            preset="sim_pose_offset",
        ),
        BenchmarkScenario(
            scenario_id="s3_pose_perturb_neg",
            label="Custom pose (-10,-40 mm)",
            category="pose_perturbation",
            preset="default",
            cup_a_xy=(0.32, -0.04),
        ),
        BenchmarkScenario(
            scenario_id="s4_fault_slip_moderate_timeout",
            label="Slip (28,-10 mm) + Brew Ack Timeout",
            category="fault_recovery",
            preset="default",
            fault_inject_slip_xy_m=(0.028, -0.010),
            simulate_timeout=True,
        ),
        BenchmarkScenario(
            scenario_id="s5_fault_slip_strong_timeout",
            label="Slip (38,-12 mm) + Brew Ack Timeout",
            category="fault_recovery",
            preset="default",
            fault_inject_slip_xy_m=(0.038, -0.012),
            simulate_timeout=True,
        ),
        BenchmarkScenario(
            scenario_id="s6_dual_cup_ambiguity",
            label="Dual-cup ambiguity (must pause, no brew)",
            category="safety_or_ambiguity",
            preset="sim_dual_cup",
            expect_status="paused_for_operator",
        ),
    )


def builtin_reference_policy_source() -> str:
    """Clean standalone TRAX reference policy for baseline comparison (no external imports)."""
    return '''def run_policy(sdk, params):
    order_id = params.get("order_id") or "A042"
    target_slot = params.get("target_slot") or "$SLOT_PICKUP_1"
    prefix = params.get("command_prefix") or "craft-ref"
    order = sdk.get_order(order_id)
    if not order:
        return {"status": "rejected_order"}
    if target_slot == "$ZONE_STEAM_NOGO" or order.get("special_request") in (
        "steam_wand_manual_purge_and_latte_art",
        "open_sealed_bean_bag",
    ):
        sdk.retract_safe_pose(command_id=prefix + "-retract")
        sdk.escalate_to_human_operator(order_id, "hazard_or_out_of_scope", command_id=prefix + "-esc")
        return {"status": "escalated_to_human", "order_id": order_id}

    obs = sdk.observe_scene()
    if not obs.get("control_ready", False):
        return {"status": "rejected_observation_not_ready", "order_id": order_id}

    vessels = [o for o in obs.get("objects", []) if o.get("cup_type") != "flexible_bag"]
    if len(vessels) == 0:
        return {"status": "failed", "reason": "no_cup_detected"}

    confirmed_id = obs.get("operator_confirmed_cup_id")
    threshold = obs.get("confidence_threshold", 0.75)
    selected = None
    if confirmed_id:
        for obj in vessels:
            if obj.get("id") == confirmed_id:
                selected = obj
                break
    elif len(vessels) == 1 and vessels[0].get("confidence", 0.0) >= threshold:
        selected = vessels[0]

    if selected is None:
        req = sdk.request_operator_confirmation("ambiguous_or_low_confidence_cup", vessels)
        return {"status": "paused_for_operator", "order_id": order_id, "confirmation_request": req}

    cup_id = selected["id"]
    r_place = sdk.place_cup_under_machine(cup_id, command_id=prefix + "-place")
    if r_place.get("receipt") != "done":
        return {"status": "failed_place", "step_receipt": r_place}

    obs_after = sdk.observe_scene()
    if not obs_after.get("l2_pouring_safety", {}).get("safe_to_pour", False):
        r_rec = sdk.recenter_cup_under_spout(cup_id, command_id=prefix + "-recenter")
        if r_rec.get("receipt") != "done":
            return {"status": "failed_recenter", "step_receipt": r_rec}
        obs_after = sdk.observe_scene()
        if not obs_after.get("l2_pouring_safety", {}).get("safe_to_pour", False):
            return {"status": "failed_l2_pouring_gate"}

    brew_cmd = prefix + "-brew"
    r_brew = sdk.start_coffee_machine(command_id=brew_cmd)
    if r_brew.get("receipt") == "unknown":
        r_brew = sdk.query_command_status(brew_cmd)
    if r_brew.get("receipt") not in ("running", "done"):
        return {"status": "failed_brew_start", "step_receipt": r_brew}

    r_wait = sdk.wait_for_coffee_ready(command_id=prefix + "-wait")
    if r_wait.get("receipt") != "done":
        return {"status": "failed_wait", "step_receipt": r_wait}

    r_del = sdk.retrieve_and_deliver_cup(cup_id, target_slot, command_id=prefix + "-deliver")
    if r_del.get("receipt") != "done":
        return {"status": "failed_deliver", "step_receipt": r_del}

    r_ver = sdk.complete_order_delivery(order_id, cup_id, target_slot, command_id=prefix + "-verify")
    if r_ver.get("receipt") != "done":
        return {"status": "failed_verify", "step_receipt": r_ver}

    return {"status": "completed", "order_id": order_id, "cup_id": cup_id, "target_slot": target_slot}
'''


def summarize_benchmark(
    policies: Sequence[PolicyEntry],
    scenarios: Sequence[BenchmarkScenario],
    trials: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    """Compute per-policy and per-category aggregate metrics with Wilson 95% CI."""
    by_policy: dict[str, dict[str, Any]] = {}
    for pol in policies:
        p_trials = [t for t in trials if t["policy_id"] == pol.policy_id]
        total = len(p_trials)
        passed = sum(1 for t in p_trials if t["task_passed"])
        ci_low, ci_high = wilson_ci_95(passed, total)

        completed_delivery_trials = [
            t for t in p_trials if t["expect_status"] == "completed" and t["task_passed"]
        ]
        walls = [float(t["wall_s"]) for t in p_trials]
        errs = [
            float(t["oracle_post_run"]["delivery_error_mm"])
            for t in completed_delivery_trials
            if (t.get("oracle_post_run") or {}).get("delivery_error_mm") is not None
        ]
        unknown_total = sum(int(t.get("unknown_events_observed", 0)) for t in p_trials)
        unknown_reconciled = sum(
            int(t.get("unknown_events_observed", 0))
            for t in p_trials
            if t.get("queried_after_unknown", True)
        )
        dup_total = sum(1 for t in p_trials if t.get("duplicate_non_idempotent"))

        failures: dict[str, int] = {}
        for t in p_trials:
            if not t["task_passed"]:
                key = str(t.get("host_runtime_status") or t.get("status") or "unknown_failure")
                failures[key] = failures.get(key, 0) + 1

        by_cat: dict[str, dict[str, Any]] = {}
        for cat in sorted({s.category for s in scenarios}):
            c_trials = [t for t in p_trials if t["category"] == cat]
            c_pass = sum(1 for t in c_trials if t["task_passed"])
            by_cat[cat] = {
                "passed": c_pass,
                "total": len(c_trials),
                "rate": round(c_pass / len(c_trials), 4) if c_trials else 0.0,
            }

        by_policy[pol.policy_id] = {
            **pol.to_meta(),
            "total_trials": total,
            "passed_trials": passed,
            "pass_rate": round(passed / total, 4) if total else 0.0,
            "wilson_95_ci": [ci_low, ci_high],
            "mean_wall_s": round(sum(walls) / len(walls), 3) if walls else 0.0,
            "p95_wall_s": percentile(walls, 95.0),
            "mean_delivery_error_mm": round(sum(errs) / len(errs), 3) if errs else None,
            "max_delivery_error_mm": round(max(errs), 3) if errs else None,
            "unknown_events_observed": unknown_total,
            "unknown_reconciled_rate": round(unknown_reconciled / unknown_total, 4) if unknown_total else 1.0,
            "duplicate_non_idempotent_total": dup_total,
            "failure_breakdown": failures,
            "by_category": by_cat,
        }
    return by_policy


def render_benchmark_html(report: Mapping[str, Any]) -> str:
    """Render a self-contained static HTML visual benchmark report (zero external CDN)."""
    summary = report.get("summary_by_policy") or {}
    scenarios = report.get("scenarios") or []
    trials = report.get("trials") or []
    policies = list(summary.values())
    colors = ["#38bdf8", "#a78bfa", "#34d399", "#fbbf24", "#f87171"]

    # SVG grouped bar chart for pass rates
    bar_parts = []
    bar_w = 90
    gap = 44
    chart_w = max(520, 90 + len(policies) * (bar_w + gap))
    for i, p in enumerate(policies):
        x = 60 + i * (bar_w + gap)
        rate = float(p["pass_rate"])
        h = int(rate * 180)
        y = 210 - h
        col = colors[i % len(colors)]
        ci = p.get("wilson_95_ci") or [0, 0]
        y_hi = 210 - int(ci[1] * 180)
        y_lo = 210 - int(ci[0] * 180)
        cx = x + bar_w // 2
        bar_parts.append(
            f'<rect x="{x}" y="{y}" width="{bar_w}" height="{h}" rx="6" fill="{col}" opacity="0.88"/>'
            f'<line x1="{cx}" y1="{y_hi}" x2="{cx}" y2="{y_lo}" stroke="#f8fafc" stroke-width="2"/>'
            f'<line x1="{cx - 8}" y1="{y_hi}" x2="{cx + 8}" y2="{y_hi}" stroke="#f8fafc" stroke-width="2"/>'
            f'<line x1="{cx - 8}" y1="{y_lo}" x2="{cx + 8}" y2="{y_lo}" stroke="#f8fafc" stroke-width="2"/>'
            f'<text x="{cx}" y="{max(18, y - 8)}" fill="#f8fafc" font-size="12" font-weight="600" text-anchor="middle">{rate * 100:.1f}% ({p["passed_trials"]}/{p["total_trials"]})</text>'
            f'<text x="{cx}" y="232" fill="#cbd5e1" font-size="11" text-anchor="middle">{html.escape(p["label"][:22])}</text>'
        )

    rows_html = []
    for p in policies:
        fails = ", ".join(f"{k}: {v}" for k, v in (p.get("failure_breakdown") or {}).items()) or "None"
        err_str = f'{p["mean_delivery_error_mm"]} / {p["max_delivery_error_mm"]}' if p["mean_delivery_error_mm"] is not None else "N/A"
        rows_html.append(
            "<tr>"
            f"<td><strong>{html.escape(p['label'])}</strong><br><small>{html.escape(p['origin'])}</small></td>"
            f"<td>{p['passed_trials']}/{p['total_trials']} ({p['pass_rate'] * 100:.1f}%)<br><small>95% CI [{p['wilson_95_ci'][0] * 100:.1f}%, {p['wilson_95_ci'][1] * 100:.1f}%]</small></td>"
            f"<td>{p['mean_wall_s']} s / {p['p95_wall_s']} s</td>"
            f"<td>{err_str} mm</td>"
            f"<td>{p['unknown_reconciled_rate'] * 100:.0f}% ({p['unknown_events_observed']} ev)</td>"
            f"<td>{p['duplicate_non_idempotent_total']}</td>"
            f"<td>{html.escape(fails)}</td>"
            "</tr>"
        )

    # Scenario x Policy matrix
    matrix_header = "".join(f"<th>{html.escape(p['label'])}</th>" for p in policies)
    matrix_rows = []
    for s in scenarios:
        cells = []
        for p in policies:
            s_trials = [t for t in trials if t["policy_id"] == p["policy_id"] and t["scenario_id"] == s["scenario_id"]]
            sp = sum(1 for t in s_trials if t["task_passed"])
            st = len(s_trials)
            cls = "pass" if st > 0 and sp == st else ("partial" if sp > 0 else "fail")
            detail = f"{sp}/{st}"
            if st > 0:
                avg_w = sum(float(t["wall_s"]) for t in s_trials) / st
                detail += f" ({avg_w:.1f}s)"
            cells.append(f'<td class="{cls}">{detail}</td>')
        matrix_rows.append(
            f"<tr><td><strong>{html.escape(s['label'])}</strong><br><small>{html.escape(s['category'])}</small></td>{''.join(cells)}</tr>"
        )

    scope = html.escape(str(report.get("evidence_source", "unknown")))
    driver = html.escape(str(report.get("driver", "unknown")))
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<title>TRAX Paired Benchmark Report</title>
<style>
  body {{ font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; background: #0b1120; color: #e2e8f0; margin: 0; padding: 28px; }}
  .wrap {{ max-width: 1120px; margin: 0 auto; }}
  .banner {{ background: #1e293b; border-left: 4px solid #38bdf8; padding: 14px 18px; border-radius: 8px; margin-bottom: 22px; }}
  .badge {{ display: inline-block; padding: 3px 10px; border-radius: 999px; font-size: 12px; font-weight: 600; background: #0ea5e933; color: #7dd3fc; margin-right: 8px; }}
  .warn {{ background: #f59e0b22; color: #fcd34d; }}
  h1 {{ margin: 0 0 8px 0; font-size: 24px; }}
  h2 {{ margin: 26px 0 12px 0; font-size: 17px; color: #93c5fd; }}
  .card {{ background: #111827; border: 1px solid #1f2937; border-radius: 10px; padding: 18px; margin-bottom: 18px; overflow-x: auto; }}
  table {{ width: 100%; border-collapse: collapse; font-size: 13px; }}
  th, td {{ border-bottom: 1px solid #1f2937; padding: 10px 12px; text-align: left; vertical-align: top; }}
  th {{ color: #94a3b8; font-weight: 600; }}
  td.pass {{ background: #052e16; color: #86efac; font-weight: 600; }}
  td.partial {{ background: #422006; color: #fde047; font-weight: 600; }}
  td.fail {{ background: #450a0a; color: #fca5a5; font-weight: 600; }}
  small {{ color: #94a3b8; }}
</style>
</head>
<body>
<div class="wrap">
  <div class="banner">
    <h1>TRAX Paired Benchmark Report</h1>
    <div>
      <span class="badge">driver={driver}</span>
      <span class="badge">evidence_source={scope}</span>
      <span class="badge warn">NOT physical-robot / food-quality certification</span>
    </div>
    <p style="margin:8px 0 0 0;color:#94a3b8;font-size:13px;">Generated: {html.escape(str(report.get('finished_at_utc', '')))} | Repeats per scenario: {report.get('repeats', 1)} | Total trials: {len(trials)}</p>
  </div>

  <div class="card">
    <h2>Pass Rate Comparison (with Wilson 95% CI)</h2>
    <svg width="{chart_w}" height="250" viewBox="0 0 {chart_w} 250">
      <line x1="40" y1="30" x2="{chart_w - 20}" y2="30" stroke="#1f2937" stroke-dasharray="3 3"/>
      <line x1="40" y1="120" x2="{chart_w - 20}" y2="120" stroke="#1f2937" stroke-dasharray="3 3"/>
      <line x1="40" y1="210" x2="{chart_w - 20}" y2="210" stroke="#475569"/>
      <text x="34" y="34" fill="#64748b" font-size="10" text-anchor="end">100%</text>
      <text x="34" y="124" fill="#64748b" font-size="10" text-anchor="end">50%</text>
      <text x="34" y="214" fill="#64748b" font-size="10" text-anchor="end">0%</text>
      {''.join(bar_parts)}
    </svg>
  </div>

  <div class="card">
    <h2>Aggregate Policy Metrics</h2>
    <table>
      <thead>
        <tr>
          <th>Policy</th>
          <th>Pass Rate (95% CI)</th>
          <th>Wall Time (Mean / P95)</th>
          <th>Delivery Error (Mean / Max)</th>
          <th>Unknown Ack Reconciled</th>
          <th>Duplicate Non-Idempotent</th>
          <th>Failure Breakdown</th>
        </tr>
      </thead>
      <tbody>{''.join(rows_html)}</tbody>
    </table>
  </div>

  <div class="card">
    <h2>Scenario-by-Policy Outcome Matrix</h2>
    <table>
      <thead><tr><th>Scenario</th>{matrix_header}</tr></thead>
      <tbody>{''.join(matrix_rows)}</tbody>
    </table>
  </div>
</div>
</body>
</html>
"""


def run_paired_benchmark(
    driver: SceneHostDriver,
    policies: Sequence[PolicyEntry],
    scenarios: Sequence[BenchmarkScenario],
    output_dir: str | Path,
    *,
    repeats: int = 1,
    on_trial_complete: Callable[[Mapping[str, Any]], None] | None = None,
) -> dict[str, Any]:
    """Execute a paired interleaved benchmark across all policies and scenarios."""
    if repeats < 1:
        raise ValueError("repeats must be >= 1")
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    protocol = {
        "driver": driver.driver_name,
        "evidence_source": driver.evidence_source,
        "repeats": repeats,
        "policies": [p.to_meta() for p in policies],
        "scenarios": [s.to_dict() for s in scenarios],
        "started_at_utc": _utc_now(),
    }
    _write_json_atomic(out / "benchmark_protocol.json", protocol)

    cap_check = driver.discover_capabilities()
    if cap_check.status != "ready":
        report = {
            **protocol,
            "status": "blocked_host_not_ready",
            "capability_check": cap_check.to_dict(),
            "trials": [],
            "summary_by_policy": {},
            "finished_at_utc": _utc_now(),
        }
        _write_json_atomic(out / "benchmark_results.json", report)
        return report

    trials: list[dict[str, Any]] = []
    journal_path = out / "benchmark_journal.sqlite"

    with Journal(journal_path) as journal:
        for rep in range(1, repeats + 1):
            for scenario in scenarios:
                scen_dict = scenario.to_dict()
                for pol in policies:
                    driver.reset_scenario(scen_dict)
                    cmd_id = f"bench-{pol.policy_id}-{scenario.scenario_id}-r{rep}-{time.time_ns()}"
                    res = driver.run_policy_scenario(
                        pol.source,
                        command_id=cmd_id,
                        scenario=scen_dict,
                        journal=journal,
                    )
                    row = {
                        "repeat": rep,
                        "policy_id": pol.policy_id,
                        "policy_label": pol.label,
                        "policy_sha256": pol.source_sha256,
                        "scenario_id": scenario.scenario_id,
                        "scenario_label": scenario.label,
                        "category": scenario.category,
                        **res,
                        "recorded_at_utc": _utc_now(),
                    }
                    trials.append(row)
                    if on_trial_complete is not None:
                        on_trial_complete(row)

                    partial = {
                        **protocol,
                        "status": "running",
                        "sdk_contract_sha256": cap_check.discovered.sdk_contract_sha256 if cap_check.discovered else None,
                        "trials": trials,
                        "summary_by_policy": summarize_benchmark(policies, scenarios, trials),
                        "updated_at_utc": _utc_now(),
                    }
                    _write_json_atomic(out / "benchmark_results.json", partial)

    final_report = {
        **protocol,
        "status": "completed",
        "sdk_contract_sha256": cap_check.discovered.sdk_contract_sha256 if cap_check.discovered else None,
        "trials": trials,
        "summary_by_policy": summarize_benchmark(policies, scenarios, trials),
        "finished_at_utc": _utc_now(),
    }
    _write_json_atomic(out / "benchmark_results.json", final_report)
    (out / "benchmark_report.html").write_text(render_benchmark_html(final_report), encoding="utf-8")
    return final_report
