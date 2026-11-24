"""Task-agnostic, budget-capped Active Physical Probing over public host SDKs.

Instead of encoding task-specific physics constants into prompts (which leaks oracle
knowledge) or burning thousands of open-ended offline calls (as in full Stage-1
SimEX population discovery), TRAX runs short, structured diagnostic probes strictly
through the public host SDK whenever development stalls (``dev_passed == 0``).

Strict architectural invariants (per ``AGENTS.md``):
- **Public SDK only**: Probes call only the same host SDK methods available to every
  evaluated policy; they never inspect privileged simulator state or internal physics.
- **Honest budget accounting**: Every probe rollout is counted in ``sim_resets`` and
  persisted as a JSON artifact with explicit ``evidence_source``.
- **Pure Python stdlib**: Zero external dependencies in core TRAX.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import ast
import json
import math
from pathlib import Path
import time
from typing import Any, Callable, Mapping, Sequence

from .contracts import canonical, digest


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass(frozen=True)
class ProbeFinding:
    """One empirically measured physical/interface relationship from a public-SDK probe."""

    finding_id: str
    probe_kind: str  # grasp_offset_scan | articulated_sweep | stall_clearance | host_safety_boundary
    summary: str
    measurements: Mapping[str, Any]
    evidence_source: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "finding_id": self.finding_id,
            "probe_kind": self.probe_kind,
            "summary": self.summary,
            "measurements": dict(self.measurements),
            "evidence_source": self.evidence_source,
        }


@dataclass(frozen=True)
class ProbeReport:
    """Persistent, auditable summary of an Active Physical Probing session."""

    probe_id: str
    task_name: str
    evidence_source: str
    sim_resets_used: int
    wall_s: float
    findings: tuple[ProbeFinding, ...]
    raw_samples: tuple[Mapping[str, Any], ...] = ()
    created_at_utc: str = field(default_factory=_utc_now)

    def to_dict(self) -> dict[str, Any]:
        return {
            "probe_id": self.probe_id,
            "task_name": self.task_name,
            "evidence_source": self.evidence_source,
            "sim_resets_used": self.sim_resets_used,
            "wall_s": round(self.wall_s, 3),
            "findings": [f.to_dict() for f in self.findings],
            "raw_samples": [dict(s) for s in self.raw_samples],
            "created_at_utc": self.created_at_utc,
        }

    def render_prompt_block(self) -> str:
        """Render concise, purely factual empirical measurements for LLM conditioning."""
        if not self.findings:
            return ""
        lines = [
            f"Empirical Active-Probe Findings (measured via public SDK, evidence_source={self.evidence_source}, "
            f"resets={self.sim_resets_used}):"
        ]
        for f in self.findings:
            lines.append(f"- [{f.probe_kind}] {f.summary} | data={json.dumps(f.measurements, sort_keys=True)}")
        return "\n".join(lines)

    def save(self, path: str | Path) -> None:
        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        tmp = out.with_suffix(out.suffix + ".tmp")
        tmp.write_text(json.dumps(self.to_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
        tmp.replace(out)


# ---------------------------------------------------------------------------
# Generic, Task-Agnostic Public-SDK Probe Programs (Zero Task Hardcoding)
# ---------------------------------------------------------------------------

def build_grasp_offset_scan_probe_source(
    *,
    radial_offsets_m: Sequence[float] = (0.0, 0.03, 0.055, -0.03, -0.055),
) -> str:
    """Return a task-agnostic public-SDK probe that empirically tests grasp offsets along an object's local X axis.

    Note: The probe discovers which target object exists dynamically from ``sdk.get_state()['objects']``
    and tests a symmetric geometric scan centered at 0.0 without knowing the task name.
    """
    offs_repr = repr([round(float(r), 4) for r in radial_offsets_m])
    return f"""def run_policy(sdk):
    st0 = sdk.get_state()
    objs = st0.get("objects") or {{}}
    obj_names = sorted(k for k in objs.keys() if k != "door")
    if not obj_names:
        return {{"status": "done", "probe": "grasp_offset_scan", "samples": []}}
    target = obj_names[0]
    samples = []
    for r in {offs_repr}:
        st = sdk.get_state()
        if st["steps_used"] > st["max_steps"] - 70:
            break
        o = st["objects"][target]
        pos = o["pos"]
        yaw = o["yaw"]
        z0 = pos[2]
        gx = pos[0] + r * cos(yaw)
        gy = pos[1] + r * sin(yaw)
        sdk.set_gripper("open", steps=6)
        sdk.move_to([gx, gy, z0 + 0.10], yaw=yaw, gripper="open", max_steps=25, tol=0.015)
        sdk.move_to([gx, gy, z0], yaw=yaw, gripper="open", max_steps=25, tol=0.015)
        g = sdk.set_gripper("close", steps=14)
        sdk.move_to([gx, gy, z0 + 0.10], yaw=yaw, gripper="close", max_steps=25, tol=0.015)
        st_after = sdk.get_state()
        z1 = st_after["objects"][target]["pos"][2]
        samples.append({{
            "target": target,
            "local_offset_m": r,
            "closed_width_m": round(g["gripper_width"], 4),
            "lift_delta_m": round(z1 - z0, 4),
            "eef_pos": [round(v, 4) for v in st_after["eef_pos"]],
        }})
        sdk.set_gripper("open", steps=6)
    return {{"status": "done", "probe": "grasp_offset_scan", "target": target, "samples": samples}}
"""


def build_articulated_sweep_probe_source() -> str:
    """Return a task-agnostic public-SDK probe for articulated mechanisms exposing handle/hinge telemetry.

    Approaches the first articulated handle object along the robot-facing direction,
    performs a monotonic downward press sweep recording ``handle_qpos`` and ``hinge_qpos``,
    then performs a pull sweep while recording when hinge motion starts.
    """
    return """def run_policy(sdk):
    st0 = sdk.get_state()
    objs = st0.get("objects") or {}
    if "handle" not in objs:
        return {"status": "done", "probe": "articulated_sweep", "samples": []}
    h0 = objs["handle"]["pos"]
    eef0 = st0["eef_pos"]
    dy_sign = 1.0 if (eef0[1] - h0[1]) >= 0.0 else -1.0
    dx_sign = 1.0 if (eef0[0] - h0[0]) >= 0.0 else -1.0
    sdk.set_gripper("open", steps=8)
    sdk.move_to([h0[0], h0[1] + dy_sign * 0.07, h0[2] + 0.04], gripper="open", max_steps=45, tol=0.015)
    sdk.move_to([h0[0], h0[1] + dy_sign * 0.01, h0[2] + 0.005], gripper="open", max_steps=45, tol=0.012)
    sdk.set_gripper("close", steps=14)
    press_curve = []
    for i in range(14):
        st = sdk.get_state()
        if st["steps_used"] > st["max_steps"] - 110:
            break
        prev_z = st["eef_pos"][2]
        prev_h = st["handle_qpos"] or 0.0
        sdk.move_delta([0.0, 0.0, -0.022], gripper="close", steps=10)
        st2 = sdk.get_state()
        cur_z = st2["eef_pos"][2]
        cur_h = st2["handle_qpos"] or 0.0
        press_curve.append({
            "step": i + 1,
            "eef_z": round(cur_z, 4),
            "handle_qpos": round(cur_h, 4),
            "hinge_qpos": round(st2["hinge_qpos"] or 0.0, 4),
        })
        if i >= 3 and abs(cur_z - prev_z) < 0.002 and abs(cur_h - prev_h) < 0.01:
            break
    pull_curve = []
    for i in range(10):
        st = sdk.get_state()
        if st["steps_used"] > st["max_steps"] - 20:
            break
        sdk.move_delta([dx_sign * 0.015, dy_sign * 0.038, -0.004], gripper="close", steps=10)
        st2 = sdk.get_state()
        pull_curve.append({
            "step": i + 1,
            "eef_pos": [round(v, 4) for v in st2["eef_pos"]],
            "handle_qpos": round(st2["handle_qpos"] or 0.0, 4),
            "hinge_qpos": round(st2["hinge_qpos"] or 0.0, 4),
        })
    return {
        "status": "done",
        "probe": "articulated_sweep",
        "approach_signs": {"dx_sign": dx_sign, "dy_sign": dy_sign},
        "press_curve": press_curve,
        "pull_curve": pull_curve,
    }
"""


def parse_probe_telemetry(
    task_name: str,
    probe_outputs: Sequence[Mapping[str, Any]],
    *,
    evidence_source: str = "simulator",
    sim_resets_used: int = 0,
    wall_s: float = 0.0,
) -> ProbeReport:
    """Summarize raw public-SDK probe outputs into factual, quantitative ``ProbeFinding`` entries."""
    findings: list[ProbeFinding] = []
    raw_samples: list[Mapping[str, Any]] = []

    for idx, out in enumerate(probe_outputs, start=1):
        kind = str(out.get("probe") or "unknown")
        raw_samples.append(dict(out))
        if kind == "grasp_offset_scan":
            samples = list(out.get("samples") or [])
            if samples:
                def _grasp_quality(s: Mapping[str, Any]) -> float:
                    lift = float(s.get("lift_delta_m", 0.0))
                    width = float(s.get("closed_width_m", 0.0))
                    retained = s.get("retained")
                    # Reject empty-gripper collision bounces (width < 0.007m or object detached from EEF)
                    if retained is False or (lift >= 0.03 and width < 0.007):
                        return -1.0
                    # Penalize fragile near-open outer-chord grasps (>0.055m) when secure inner pinches exist
                    chord_pen = 0.045 if (lift >= 0.03 and width > 0.055) else 0.0
                    return lift - chord_pen

                best = max(samples, key=_grasp_quality)
                lifted = [
                    s for s in samples
                    if float(s.get("lift_delta_m", 0.0)) >= 0.03
                    and float(s.get("closed_width_m", 0.0)) >= 0.007
                    and s.get("retained") is not False
                ]
                frame = str(out.get("frame") or ("world_free_space_radial" if "approach_angle_rad" in best else "object_local_x"))
                is_world_radial = frame == "world_free_space_radial" or ("approach_angle_rad" in best)
                if is_world_radial:
                    summary = (
                        f"Scanned {len(samples)} free-space radial/center grasps on `{out.get('target', 'object')}` "
                        f"(frame={frame}, NOT random object yaw); best secure lift={best.get('lift_delta_m')}m "
                        f"at radius_m={best.get('local_offset_m')}, z_offset_m={best.get('z_offset_m')}, "
                        f"world_angle_rad={best.get('approach_angle_rad')}, gripper_yaw_rad={best.get('gripper_yaw_rad')} "
                        f"(jaw_rel_rad={best.get('jaw_rel_angle_rad')}, clearance_m={best.get('clearance_m')}, "
                        f"closed_width={best.get('closed_width_m')}m; {len(lifted)}/{len(samples)} samples lifted >=0.03m). "
                        f"Prefer secure narrow pinch (closed_width<0.05m) over fragile wide outer-chord grasp."
                    )
                else:
                    summary = (
                        f"Scanned {len(samples)} local X offsets on `{out.get('target', 'object')}`; "
                        f"best empirical lift={best.get('lift_delta_m')}m at local_offset_m={best.get('local_offset_m')} "
                        f"(closed_width={best.get('closed_width_m')}m; {len(lifted)}/{len(samples)} offsets lifted >=0.03m)."
                    )
                meas: dict[str, Any] = {
                    "target": out.get("target"),
                    "frame": frame,
                    "best_local_offset_m": best.get("local_offset_m"),
                    "best_lift_delta_m": best.get("lift_delta_m"),
                    "best_closed_width_m": best.get("closed_width_m"),
                }
                for opt_k in ("z_offset_m", "approach_angle_rad", "gripper_yaw_rad", "jaw_rel_angle_rad", "clearance_m"):
                    if best.get(opt_k) is not None:
                        meas[f"best_{opt_k}"] = best.get(opt_k)
                meas["all_samples"] = [
                    {
                        k_out: s.get(k_in)
                        for k_out, k_in in (
                            ("offset_m", "local_offset_m"),
                            ("z_offset_m", "z_offset_m"),
                            ("angle_rad", "approach_angle_rad"),
                            ("gripper_yaw_rad", "gripper_yaw_rad"),
                            ("jaw_rel_rad", "jaw_rel_angle_rad"),
                            ("clearance_m", "clearance_m"),
                            ("lift_m", "lift_delta_m"),
                            ("width_m", "closed_width_m"),
                        )
                        if s.get(k_in) is not None
                    }
                    for s in samples
                ]
                findings.append(
                    ProbeFinding(
                        finding_id=f"probe_{idx:02d}_grasp_offset",
                        probe_kind=kind,
                        summary=summary,
                        measurements=meas,
                        evidence_source=evidence_source,
                    )
                )
        elif kind == "articulated_sweep":
            press = list(out.get("press_curve") or [])
            pull = list(out.get("pull_curve") or [])
            max_press_h = max((float(p.get("handle_qpos", 0.0)) for p in press), default=0.0)
            max_pull_hinge = max((float(p.get("hinge_qpos", 0.0)) for p in pull), default=0.0)
            first_hinge_move = next(
                (p for p in (press + pull) if float(p.get("hinge_qpos", 0.0)) >= 0.03),
                None,
            )
            unlatch_obs = (
                float(first_hinge_move.get("handle_qpos", max_press_h))
                if first_hinge_move is not None
                else None
            )
            summary = (
                f"Downward press sweep reached max handle_qpos={max_press_h:.3f} rad; "
                f"subsequent pull sweep reached max hinge_qpos={max_pull_hinge:.3f} rad "
                f"(approach_signs={out.get('approach_signs')}, "
                f"first_hinge_motion_at_handle_qpos={unlatch_obs})."
            )
            findings.append(
                ProbeFinding(
                    finding_id=f"probe_{idx:02d}_articulated",
                    probe_kind=kind,
                    summary=summary,
                    measurements={
                        "approach_signs": out.get("approach_signs"),
                        "max_press_handle_qpos": round(max_press_h, 4),
                        "max_pull_hinge_qpos": round(max_pull_hinge, 4),
                        "first_hinge_motion_handle_qpos": round(unlatch_obs, 4) if unlatch_obs is not None else None,
                        "press_steps": len(press),
                        "pull_steps": len(pull),
                    },
                    evidence_source=evidence_source,
                )
            )
        elif kind == "host_safety_boundary":
            findings.append(
                ProbeFinding(
                    finding_id=f"probe_{idx:02d}_host_boundary",
                    probe_kind=kind,
                    summary=str(out.get("summary") or "Measured host safety boundary via public SDK."),
                    measurements=dict(out.get("measurements") or {}),
                    evidence_source=evidence_source,
                )
            )

    pid = f"probe-{task_name.lower()}-{digest({'t': task_name, 'n': len(findings), 's': raw_samples})[:8]}"
    return ProbeReport(
        probe_id=pid,
        task_name=task_name,
        evidence_source=evidence_source,
        sim_resets_used=sim_resets_used,
        wall_s=wall_s,
        findings=tuple(findings),
        raw_samples=tuple(raw_samples),
    )


def build_llm_hypothesis_synthesis_prompt(
    *,
    task_description: str,
    sdk_contract: str,
    baseline_source: str,
    failure_telemetry: Sequence[Mapping[str, Any]],
    probe_report: ProbeReport | None = None,
    max_hypotheses: int = 3,
) -> str:
    """Build a task-agnostic prompt asking an LLM to synthesize K distinct diagnostic hypotheses.

    Zero hardcoded task knowledge: the LLM must derive hypotheses solely from the
    shared contract, failure telemetry, and optional empirical public-SDK probe findings.
    """
    payload = {
        "max_hypotheses": max(1, int(max_hypotheses)),
        "failure_telemetry": list(failure_telemetry),
        "empirical_probe_findings": probe_report.to_dict()["findings"] if probe_report else [],
    }
    return "\n".join(
        [
            "You are a robotics systems diagnostician.",
            f"Analyze the factual failure telemetry and empirical probe measurements below and propose "
            f"up to {max(1, int(max_hypotheses))} DISTINCT, testable, mechanistic repair hypotheses.",
            "Do NOT guess hidden simulator internals; ground every hypothesis in observed telemetry/probe numbers.",
            "Output ONLY a JSON list of objects with keys: `hypothesis_id`, `title`, `diagnosis`, `proposed_strategy`.",
            "",
            task_description,
            "",
            sdk_contract,
            "",
            "Baseline Policy Source:",
            "```python",
            baseline_source.strip(),
            "```",
            "",
            "Diagnostic Telemetry (JSON):",
            canonical(payload),
        ]
    )


def parse_llm_hypotheses_json(raw_text: str, *, max_hypotheses: int = 3) -> list[dict[str, str]]:
    """Robustly extract structured hypotheses JSON from an LLM response."""
    text = raw_text.strip()
    if "```" in text:
        parts = text.split("```")
        for chunk in parts:
            c = chunk.strip()
            if c.startswith("json"):
                c = c[4:].strip()
            if c.startswith("["):
                text = c
                break
    start = text.find("[")
    end = text.rfind("]")
    if start == -1 or end == -1 or end <= start:
        return []
    try:
        parsed = json.loads(text[start : end + 1])
    except json.JSONDecodeError:
        return []
    if not isinstance(parsed, list):
        return []
    out: list[dict[str, str]] = []
    for idx, item in enumerate(parsed[: max(1, int(max_hypotheses))], start=1):
        if not isinstance(item, Mapping):
            continue
        title = str(item.get("title") or f"Hypothesis {idx}").strip()
        strategy = str(item.get("proposed_strategy") or item.get("strategy") or "").strip()
        diagnosis = str(item.get("diagnosis") or "").strip()
        if not strategy:
            continue
        out.append(
            {
                "hypothesis_id": str(item.get("hypothesis_id") or f"llm_hyp_{idx:02d}").strip(),
                "title": title,
                "diagnosis": diagnosis,
                "proposed_strategy": strategy,
            }
        )
    return out


# ---------------------------------------------------------------------------
# AST-Based Selective Subroutine Repair Analysis (RAX Section 5.5.2 Pattern)
# ---------------------------------------------------------------------------

def analyze_policy_subroutines(
    policy_source: str,
    failure_codes: Sequence[str],
) -> dict[str, Any]:
    """Inspect a policy's AST to identify helper subroutines and scope selective repairs.

    Task-agnostic: inspects only AST function names, line spans, and called SDK methods,
    plus generic failure-stage attribution codes, to advise which subroutines appear healthy
    (freeze) vs. which stage needs targeted rewriting (replace/extend).
    """
    try:
        tree = ast.parse(policy_source)
    except SyntaxError:
        return {"subroutines": [], "freeze_hint": "", "repair_scope": "syntax_fix"}

    subroutines: list[dict[str, Any]] = []
    for node in tree.body:
        if not isinstance(node, ast.FunctionDef):
            continue
        sdk_calls: set[str] = set()
        for child in ast.walk(node):
            if isinstance(child, ast.Call) and isinstance(child.func, ast.Attribute):
                if isinstance(child.func.value, ast.Name) and child.func.value.id == "sdk":
                    sdk_calls.add(child.func.attr)
        subroutines.append(
            {
                "name": node.name,
                "lines": (int(getattr(node, "end_lineno", node.lineno) or node.lineno) - int(node.lineno) + 1),
                "sdk_methods": sorted(sdk_calls),
            }
        )

    codes = set(failure_codes)
    if codes & {"insertion_too_high_or_step_exhausted", "insertion_or_release_precision_error"}:
        repair_scope = "downstream_alignment_and_seating"
        freeze_hint = (
            "Upstream approach/grasp succeeded (object was lifted). Keep working grasp/helper subroutines frozen; "
            "focus edits on goal-alignment, grasp-offset compensation, step budget margin, and final seating/release."
        )
    elif codes & {"transport_or_goal_alignment_error"}:
        repair_scope = "transport_and_goal_alignment"
        freeze_hint = (
            "Upstream grasp succeeded. Keep working grasp subroutines frozen; "
            "rewrite the transport/clearance and goal-centering stage."
        )
    elif codes & {"door_pull_arc_incomplete"}:
        repair_scope = "post_unlatch_pull_trajectory"
        freeze_hint = (
            "Initial contact/unlatch started moving the hinge. Preserve the working approach/unlatch subroutine "
            "and rewrite the sustained pull-arc tracking stage."
        )
    elif codes & {"never_closed_gripper_early_abort", "grasp_missed_or_slip", "approach_missed_object", "handle_approach_missed", "latch_not_unlatched"}:
        repair_scope = "upstream_approach_and_contact"
        freeze_hint = (
            "Failure occurs before stable grasp/release. Preserve low-level servo helpers if clean, "
            "and rewrite the approach/contact/actuation subroutine grounded in telemetry and probe data."
        )
    else:
        repair_scope = "general_policy_stage"
        freeze_hint = "Preserve verified helper functions and rewrite only the failing control stage."

    return {
        "subroutines": subroutines,
        "repair_scope": repair_scope,
        "freeze_hint": freeze_hint,
    }

