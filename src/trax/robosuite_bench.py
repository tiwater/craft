"""Phase 3B: Public continuous-manipulation benchmark parity on Robosuite (MuJoCo).

Evaluates TRAX against prior-work protocols under matched model, shared 6-DoF
operational-space primitive SDK, matched development budget, and ASPIRE-aligned
disjoint seed splits (dev seeds 101..105, held-out eval seeds 1..20) across four
canonical Robosuite Panda tasks:
- Lift (cube grasping & lifting)
- PickPlaceCan (cylindrical grasp + bin transport)
- NutAssemblySquare (handle-offset grasp, yaw alignment, tight-clearance peg insertion)
- Door (articulated handle approach, latch unlatching, hinge arc pulling)

Methods compared identically:
1. Strong Zero-Shot (Full Contract) — 1 dev LLM call, 0 dev rollouts, 0 eval LLM calls
2. ASPIRE-Protocol (Adapted) — iterative raw-trace debugging on dev seeds -> frozen
3. CaP-X-Protocol (Adapted) — feedback regen + SkillLibrary -> Turn-1 frozen & Online Turn-2
4. TRAX (Full Engine) — AST preflight + 4-layer manipulation attribution +
   Failure-to-Testcase regression gating -> frozen (0 eval LLM calls)
"""
from __future__ import annotations

import ast
from dataclasses import dataclass, field
from datetime import datetime, timezone
import html
import json
import math
import os
from pathlib import Path
import time
from typing import Any, Callable, Mapping, Sequence

# Headless EGL before mujoco/robosuite import
if not os.environ.get("MUJOCO_GL") and not os.environ.get("DISPLAY"):
    os.environ["MUJOCO_GL"] = "egl"

from .benchmark import percentile, wilson_ci_95
from .cafebench import _git_provenance, _write_json_atomic
from .contracts import digest
from .prior_work_bench import (
    CapXSkillLibrary,
    LLMGenerateFn,
    make_live_llm_fn,
    mcnemar_exact_p,
)
from .probing import (
    ProbeReport,
    analyze_policy_subroutines,
    build_articulated_sweep_probe_source,
    build_grasp_offset_scan_probe_source,
    parse_probe_telemetry,
)
from .skills import SkillExperience, SkillExperienceStore, validate_skill_code_snippet


ROBOSUITE_BENCH_VERSION = "TRAX-Robosuite-Parity-v3.0"
DEFAULT_ROBOSUITE_TASKS: tuple[str, ...] = ("Lift", "PickPlaceCan", "NutAssemblySquare", "Door")
DEFAULT_DEV_SEEDS: tuple[int, ...] = (101, 102, 103, 104, 105)
DEFAULT_EVAL_SEEDS: tuple[int, ...] = tuple(range(1, 21))


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _quat_to_yaw(quat_xyzw: Sequence[float]) -> float:
    """Extract yaw angle (radians) around world Z from robosuite xyzw quaternion."""
    x, y, z, w = float(quat_xyzw[0]), float(quat_xyzw[1]), float(quat_xyzw[2]), float(quat_xyzw[3])
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


# ---------------------------------------------------------------------------
# Task Descriptions & Identical Shared 6-DoF Primitive SDK Contract
# ---------------------------------------------------------------------------

ROBOSUITE_TASK_DESCRIPTIONS: dict[str, str] = {
    "Lift": (
        "Task: Robosuite `Lift` (Panda 7-DoF arm). Grasp the cube (`state['objects']['cube']`) "
        "and lift it clearly above the table using live observations from `sdk.get_state()`."
    ),
    "PickPlaceCan": (
        "Task: Robosuite `PickPlaceCan` (Panda 7-DoF arm). Grasp the soda can (`state['objects']['can']`), "
        "transport it cleanly into the target bin at `state['goal_pos']`, and release it."
    ),
    "NutAssemblySquare": (
        "Task: Robosuite `NutAssemblySquare` (Panda 7-DoF arm). Grasp the square nut (`state['objects']['nut']`) "
        "and insert its center hole cleanly onto the square peg at `state['goal_pos']` using live poses and yaw from `sdk.get_state()`."
    ),
    "Door": (
        "Task: Robosuite `Door` (Panda 7-DoF arm). Unlatch and open the hinged door (`state['objects']['handle']`, "
        "`state['objects']['door']`, `state['handle_qpos']`, `state['hinge_qpos']`) until `hinge_qpos > 0.30`."
    ),
}

SHARED_ROBOSUITE_SDK_CONTRACT = """Robosuite 6-DoF Continuous Control Policy SDK Contract (identical for all methods):
1. Entry point: define `def run_policy(sdk):` returning a dict `{"status": "done" | "failed", ...}`.
2. Safe Sandbox Rules:
   - NO `import`, `try`/`except`, `with`, `class`, `global`, `nonlocal`, `del`, or `_`-prefixed names/attributes.
   - Math helpers are pre-injected directly as globals: `sin`, `cos`, `tan`, `atan2`, `sqrt`, `hypot`, `fabs`, `pi`, `min`, `max`, `abs`, `round`, `range`, `len`, `sum`, `list`, `dict`, `tuple`, `float`, `int`, `bool`, `enumerate`, `sorted`, `all`, `any`.
   - Top-level helper functions (names NOT starting with `_`) are allowed.
3. Available `sdk` methods (real MuJoCo `OSC_POSE` Panda control, 20 Hz, max horizon 400 steps):
   - `sdk.get_state()` -> dict:
     `{
       "task": str,
       "eef_pos": [x, y, z],
       "eef_quat": [x, y, z, w],
       "eef_yaw": float (radians),
       "gripper_width": float (meters; ~0.08 open, <0.01 empty closed, 0.015..0.065 holding object),
       "objects": {name: {"pos": [x, y, z], "quat": [x, y, z, w], "yaw": float}},
       "goal_pos": [x, y, z] | None,
       "handle_qpos": float | None,   # Door only
       "hinge_qpos": float | None,    # Door only
       "steps_used": int,
       "max_steps": int,
     }`
   - `sdk.move_to(pos, yaw=None, gripper=None, max_steps=60, tol=0.012)` -> dict:
     Closed-loop operational-space move to world `[x, y, z]` (and optional target `yaw` in radians; `gripper` is `"open"`, `"close"`, or `None` to keep current).
     Returns `{"reached": bool, "pos_err_m": float, "eef_pos": [x, y, z], "steps": int}`.
   - `sdk.move_delta(dpos, dyaw=0.0, gripper=None, steps=20)` -> dict:
     Smooth relative operational-space motion `[dx, dy, dz]` + optional yaw delta `dyaw` over `steps` control steps.
     Returns `{"eef_pos": [x, y, z], "steps": int}`.
   - `sdk.set_gripper(action, steps=15)` -> dict:
     `action` is `"open"` or `"close"`. Steps physics while actuating gripper fingers.
     Returns `{"gripper_width": float, "grasped_hint": bool}`.
   - `sdk.step_raw(action_7d, steps=1)` -> dict:
     Applies raw 7-DoF OSC action `[dx, dy, dz, drx, dry, drz, grip]` (`grip=-1` open, `+1` close) for `steps` control ticks.
4. Generalization rule: NEVER hardcode object poses from a single seed — always read live coordinates from `sdk.get_state()`.
5. Output ONLY valid Python source code defining `run_policy(sdk)` (no prose outside an optional Python code fence).
"""


# ---------------------------------------------------------------------------
# Static AST Sandbox Checker for Robosuite Policies
# ---------------------------------------------------------------------------

_ROBO_FORBIDDEN_NODES = (
    ast.Import,
    ast.ImportFrom,
    ast.Try,
    ast.With,
    ast.ClassDef,
    ast.Global,
    ast.Nonlocal,
    ast.Delete,
    ast.AsyncFunctionDef,
    ast.AsyncFor,
    ast.AsyncWith,
    ast.Await,
    ast.Lambda,
)

_ROBO_SAFE_GLOBALS: dict[str, Any] = {
    "__builtins__": {
        "dict": dict,
        "list": list,
        "len": len,
        "str": str,
        "float": float,
        "int": int,
        "bool": bool,
        "min": min,
        "max": max,
        "round": round,
        "range": range,
        "abs": abs,
        "all": all,
        "any": any,
        "sum": sum,
        "sorted": sorted,
        "tuple": tuple,
        "enumerate": enumerate,
        "isinstance": isinstance,
        "None": None,
        "True": True,
        "False": False,
    },
    "sin": math.sin,
    "cos": math.cos,
    "tan": math.tan,
    "atan2": math.atan2,
    "sqrt": math.sqrt,
    "hypot": math.hypot,
    "fabs": math.fabs,
    "pi": math.pi,
}


def check_robosuite_ast_compliance(source: str) -> dict[str, Any]:
    """Static AST compliance check for Robosuite policies."""
    try:
        tree = ast.parse(source)
    except SyntaxError as exc:
        return {"compliant": False, "issues": [f"SyntaxError: {exc}"]}
    has_entry = any(isinstance(n, ast.FunctionDef) and n.name == "run_policy" for n in tree.body)
    if not has_entry:
        return {"compliant": False, "issues": ["Missing top-level function `def run_policy(sdk):`"]}
    issues: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, _ROBO_FORBIDDEN_NODES):
            issues.append(f"Line {getattr(node, 'lineno', '?')}: forbidden construct `{type(node).__name__}`")
        elif isinstance(node, ast.FunctionDef) and node.name.startswith("_"):
            issues.append(f"Line {node.lineno}: function name `{node.name}` starts with `_`")
        elif isinstance(node, ast.Name) and node.id.startswith("_"):
            issues.append(f"Line {node.lineno}: identifier `{node.id}` starts with `_`")
        elif isinstance(node, ast.Attribute) and node.attr.startswith("_"):
            issues.append(f"Line {node.lineno}: attribute `.{node.attr}` starts with `_`")
    return {"compliant": len(issues) == 0, "issues": issues}


# ---------------------------------------------------------------------------
# Live Robosuite 6-DoF Primitive SDK & Rollout Harness
# ---------------------------------------------------------------------------

class RobosuitePolicySDK:
    """Physics-faithful 6-DoF operational-space primitive SDK over Robosuite Panda."""

    def __init__(self, env: Any, task_name: str, *, max_steps: int = 400) -> None:
        self._env = env
        self._task = task_name
        self._max_steps = max_steps
        self._steps_used = 0
        self._grip_cmd = -1.0  # -1 open, +1 close
        self._obs: dict[str, Any] = {}
        self.trace: list[dict[str, Any]] = []
        self.max_obj_z_seen: float = 0.0
        self.init_obj_pos: list[float] | None = None
        self.min_eef_to_obj_m: float = 9.99
        self.min_obj_to_goal_m: float = 9.99
        self.max_handle_qpos: float = 0.0
        self.max_hinge_qpos: float = 0.0
        self.closed_gripper_width_min: float = 9.99
        self.closed_gripper_width_max: float = 0.0
        self.ever_closed_gripper: bool = False

    def bind_initial_obs(self, obs: dict[str, Any]) -> None:
        self._obs = obs
        st = self._build_state()
        pobj = self._primary_obj_pos(st)
        if pobj is not None:
            self.init_obj_pos = list(pobj)
            self.max_obj_z_seen = float(pobj[2])

    def _primary_obj_pos(self, state: Mapping[str, Any]) -> list[float] | None:
        objs = state.get("objects") or {}
        for key in ("cube", "can", "nut", "handle"):
            if key in objs:
                return list(objs[key]["pos"])
        return None

    def _update_telemetry(self) -> None:
        st = self._build_state()
        eef = st["eef_pos"]
        pobj = self._primary_obj_pos(st)
        if pobj is not None:
            self.max_obj_z_seen = max(self.max_obj_z_seen, float(pobj[2]))
            d_eef = math.sqrt(sum((eef[i] - pobj[i]) ** 2 for i in range(3)))
            self.min_eef_to_obj_m = min(self.min_eef_to_obj_m, d_eef)
            goal = st.get("goal_pos")
            if goal is not None:
                d_goal = math.hypot(pobj[0] - goal[0], pobj[1] - goal[1])
                self.min_obj_to_goal_m = min(self.min_obj_to_goal_m, d_goal)
        if st.get("handle_qpos") is not None:
            self.max_handle_qpos = max(self.max_handle_qpos, abs(float(st["handle_qpos"])))
        if st.get("hinge_qpos") is not None:
            self.max_hinge_qpos = max(self.max_hinge_qpos, abs(float(st["hinge_qpos"])))

    def _build_state(self) -> dict[str, Any]:
        obs = self._obs
        eef_pos = [round(float(x), 5) for x in obs["robot0_eef_pos"]]
        eef_quat = [round(float(x), 5) for x in obs["robot0_eef_quat"]]
        gq = obs.get("robot0_gripper_qpos")
        g_width = round(float(abs(gq[0] - gq[1])), 5) if gq is not None and len(gq) >= 2 else 0.08
        objects: dict[str, dict[str, Any]] = {}
        goal_pos: list[float] | None = None
        handle_qpos: float | None = None
        hinge_qpos: float | None = None

        if self._task == "Lift":
            cpos = [round(float(x), 5) for x in obs["cube_pos"]]
            cquat = [round(float(x), 5) for x in obs["cube_quat"]]
            objects["cube"] = {"pos": cpos, "quat": cquat, "yaw": round(_quat_to_yaw(cquat), 5)}
            goal_pos = [cpos[0], cpos[1], round(cpos[2] + 0.12, 5)]
        elif self._task == "PickPlaceCan":
            cpos = [round(float(x), 5) for x in obs["Can_pos"]]
            cquat = [round(float(x), 5) for x in obs["Can_quat"]]
            objects["can"] = {"pos": cpos, "quat": cquat, "yaw": round(_quat_to_yaw(cquat), 5)}
            idx = int(getattr(self._env, "object_id", 3))
            bp = self._env.target_bin_placements[idx]
            goal_pos = [round(float(bp[0]), 5), round(float(bp[1]), 5), round(float(bp[2]) + 0.08, 5)]
        elif self._task == "NutAssemblySquare":
            npos = [round(float(x), 5) for x in obs["SquareNut_pos"]]
            nquat = [round(float(x), 5) for x in obs["SquareNut_quat"]]
            objects["nut"] = {"pos": npos, "quat": nquat, "yaw": round(_quat_to_yaw(nquat), 5)}
            p1 = self._env.sim.data.body_xpos[self._env.peg1_body_id]
            goal_pos = [round(float(p1[0]), 5), round(float(p1[1]), 5), round(float(p1[2]), 5)]
        elif self._task == "Door":
            dpos = [round(float(x), 5) for x in obs["door_pos"]]
            hpos = [round(float(x), 5) for x in obs["handle_pos"]]
            objects["door"] = {"pos": dpos, "quat": [0.0, 0.0, 0.0, 1.0], "yaw": 0.0}
            objects["handle"] = {"pos": hpos, "quat": [0.0, 0.0, 0.0, 1.0], "yaw": 0.0}
            handle_qpos = round(float(obs.get("handle_qpos", 0.0)), 5)
            hinge_qpos = round(float(obs.get("hinge_qpos", 0.0)), 5)
            goal_pos = [round(dpos[0] - 0.15, 5), round(dpos[1] - 0.25, 5), round(hpos[2], 5)]

        return {
            "task": self._task,
            "eef_pos": eef_pos,
            "eef_quat": eef_quat,
            "eef_yaw": round(_quat_to_yaw(eef_quat), 5),
            "gripper_width": g_width,
            "objects": objects,
            "goal_pos": goal_pos,
            "handle_qpos": handle_qpos,
            "hinge_qpos": hinge_qpos,
            "steps_used": self._steps_used,
            "max_steps": self._max_steps,
        }

    def _step_env(self, action_7d: list[float]) -> None:
        import numpy as np

        if self._steps_used >= self._max_steps:
            return
        act = np.clip(np.asarray(action_7d, dtype=np.float64), -1.0, 1.0)
        obs, _rew, _done, _info = self._env.step(act)
        self._obs = obs
        self._steps_used += 1
        self._update_telemetry()

    def get_state(self) -> dict[str, Any]:
        return self._build_state()

    def set_gripper(self, action: str, steps: int = 15) -> dict[str, Any]:
        self._grip_cmd = 1.0 if str(action).lower() == "close" else -1.0
        n = max(1, min(40, int(steps)))
        for _ in range(n):
            if self._steps_used >= self._max_steps:
                break
            self._step_env([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self._grip_cmd])
        st = self._build_state()
        w = st["gripper_width"]
        if self._grip_cmd > 0:
            self.ever_closed_gripper = True
            self.closed_gripper_width_min = min(self.closed_gripper_width_min, w)
            self.closed_gripper_width_max = max(self.closed_gripper_width_max, w)
        grasped = (self._grip_cmd > 0) and (w > 0.008)
        self.trace.append({"op": "set_gripper", "action": action, "width": w, "grasped_hint": grasped, "step": self._steps_used})
        return {"gripper_width": w, "grasped_hint": grasped}

    def move_to(
        self,
        pos: Sequence[float],
        yaw: float | None = None,
        gripper: str | None = None,
        max_steps: int = 60,
        tol: float = 0.012,
    ) -> dict[str, Any]:
        if gripper is not None:
            self._grip_cmd = 1.0 if str(gripper).lower() == "close" else -1.0
        tx, ty, tz = float(pos[0]), float(pos[1]), float(pos[2])
        n = max(1, min(120, int(max_steps)))
        steps_done = 0
        reached = False
        err = 9.99
        for _ in range(n):
            if self._steps_used >= self._max_steps:
                break
            st = self._build_state()
            ex, ey, ez = st["eef_pos"]
            dx, dy, dz = tx - ex, ty - ey, tz - ez
            err = math.sqrt(dx * dx + dy * dy + dz * dz)
            yaw_err = _wrap_angle(float(yaw) - st["eef_yaw"]) if yaw is not None else 0.0
            if err <= float(tol) and (yaw is None or abs(yaw_err) <= 0.08):
                reached = True
                break
            # Proportional operational-space velocity command
            kp = 5.5
            act = [
                max(-1.0, min(1.0, dx * kp)),
                max(-1.0, min(1.0, dy * kp)),
                max(-1.0, min(1.0, dz * kp)),
                0.0,
                0.0,
                max(-1.0, min(1.0, yaw_err * 2.5)) if yaw is not None else 0.0,
                self._grip_cmd,
            ]
            self._step_env(act)
            steps_done += 1
        final_eef = self._build_state()["eef_pos"]
        self.trace.append(
            {
                "op": "move_to",
                "target": [round(tx, 3), round(ty, 3), round(tz, 3)],
                "reached": reached,
                "err_m": round(err, 4),
                "steps": steps_done,
            }
        )
        return {"reached": reached, "pos_err_m": round(err, 4), "eef_pos": final_eef, "steps": steps_done}

    def move_delta(
        self,
        dpos: Sequence[float],
        dyaw: float = 0.0,
        gripper: str | None = None,
        steps: int = 20,
    ) -> dict[str, Any]:
        st = self._build_state()
        target = [st["eef_pos"][0] + float(dpos[0]), st["eef_pos"][1] + float(dpos[1]), st["eef_pos"][2] + float(dpos[2])]
        target_yaw = (st["eef_yaw"] + float(dyaw)) if abs(float(dyaw)) > 1e-6 else None
        return self.move_to(target, yaw=target_yaw, gripper=gripper, max_steps=max(5, int(steps)), tol=0.008)

    def step_raw(self, action_7d: Sequence[float], steps: int = 1) -> dict[str, Any]:
        vals = [float(x) for x in action_7d]
        if len(vals) != 7:
            raise ValueError("step_raw requires a 7-element action vector")
        n = max(1, min(60, int(steps)))
        for _ in range(n):
            if self._steps_used >= self._max_steps:
                break
            self._step_env(vals)
        st = self._build_state()
        self.trace.append({"op": "step_raw", "steps": n, "eef": [round(x, 3) for x in st["eef_pos"]]})
        return {"eef_pos": st["eef_pos"], "steps_used": self._steps_used}


# ---------------------------------------------------------------------------
# Four-Layer Continuous-Manipulation Failure Attribution (TRAX)
# ---------------------------------------------------------------------------

def attribute_robosuite_failure(rollout: Mapping[str, Any], task_name: str) -> dict[str, str]:
    """Classify a failed Robosuite rollout into a structured, zero-leakage 4-layer causal diagnosis.

    Strictly factual and telemetry-driven: never injects hardcoded task-specific physics
    constants (e.g. latch angles, handle offsets, or exact seat heights) into prescriptions.
    """
    if rollout.get("success"):
        return {"layer": "none", "code": "passed", "summary": "Rollout succeeded", "prescription": "None"}
    err = str(rollout.get("error") or "")
    if err:
        return {
            "layer": "policy",
            "code": "sandbox_or_runtime_exception",
            "summary": f"Policy raised exception or failed AST sandbox: {err}",
            "prescription": "Fix AST/syntax/runtime error using only allowed sandbox constructs.",
        }

    min_eef_obj = float(rollout.get("min_eef_to_obj_m", 9.99))
    lift_delta = float(rollout.get("obj_lift_delta_m", 0.0))
    min_obj_goal = float(rollout.get("min_obj_to_goal_m", 9.99))
    ever_closed = bool(rollout.get("ever_closed_gripper", False))
    closed_w_max = float(rollout.get("closed_gripper_width_max", 0.0))
    fs = rollout.get("final_state") or {}
    has_articulated = (
        task_name == "Door"
        or fs.get("handle_qpos") is not None
        or fs.get("hinge_qpos") is not None
        or ("handle" in (fs.get("objects") or {}))
    )

    if has_articulated:
        max_handle = float(rollout.get("max_handle_qpos", 0.0))
        max_hinge = float(rollout.get("max_hinge_qpos", 0.0))
        if min_eef_obj > 0.08:
            return {
                "layer": "control",
                "code": "handle_approach_missed",
                "summary": f"EEF did not closely approach articulated handle (min_dist={min_eef_obj:.3f}m).",
                "prescription": "Verify approach direction and trajectory from live EEF and handle coordinates before contact.",
            }
        if max_hinge < 0.05:
            return {
                "layer": "control",
                "code": "latch_not_unlatched",
                "summary": (
                    f"Reached handle vicinity (min_dist={min_eef_obj:.3f}m) and actuated handle to "
                    f"max_handle_qpos={max_handle:.3f} rad, but hinge barely moved (max_hinge_qpos={max_hinge:.3f} rad)."
                ),
                "prescription": (
                    "Hinge remained near zero despite contact; use observed telemetry or active-probe measurements "
                    "to check whether deeper handle actuation or a different pull vector is required to release the mechanism."
                ),
            }
        return {
            "layer": "control",
            "code": "door_pull_arc_incomplete",
            "summary": f"Hinge began moving (max_handle_qpos={max_handle:.3f} rad, max_hinge_qpos={max_hinge:.3f} rad) but did not reach target threshold.",
            "prescription": "Maintain contact and continue tracking the live handle trajectory through the full motion arc within the step budget.",
        }

    if min_eef_obj > 0.08:
        return {
            "layer": "control",
            "code": "approach_missed_object",
            "summary": f"EEF never reached target object (min_eef_to_obj={min_eef_obj:.3f}m).",
            "prescription": "Read live object coordinates from sdk.get_state(), approach safely, and verify reachability.",
        }
    if not ever_closed:
        return {
            "layer": "policy",
            "code": "never_closed_gripper_early_abort",
            "summary": (
                f"Reached object vicinity (min_dist={min_eef_obj:.3f}m) but NEVER called set_gripper('close') "
                f"(policy_return={rollout.get('policy_return', '')!r}, steps_used={rollout.get('steps_used', 0)}/400)."
            ),
            "prescription": (
                "An overly strict pre-grasp convergence guard aborted before closing the gripper. "
                "Relax intermediate waypoint checks so gripper closing executes once positioned near the target."
            ),
        }
    if lift_delta < 0.04:
        return {
            "layer": "control",
            "code": "grasp_missed_or_slip",
            "summary": (
                f"Reached object vicinity (min_dist={min_eef_obj:.3f}m) and closed gripper "
                f"(closed_width={closed_w_max:.4f}m), but object did not lift (lift_delta={lift_delta:.3f}m)."
            ),
            "prescription": (
                "Inspect closed gripper width, object geometry/yaw, and any empirical grasp-offset probe findings "
                "to ensure the fingers engage a graspable feature before lifting."
            ),
        }
    if min_obj_goal > 0.06:
        return {
            "layer": "control",
            "code": "transport_or_goal_alignment_error",
            "summary": f"Object lifted (lift_delta={lift_delta:.3f}m) but did not align above goal (min_obj_to_goal={min_obj_goal:.3f}m).",
            "prescription": (
                "Lift clear of workspace obstacles, compensate for any grasp-point-to-object-center offset when "
                "centering the object over live goal_pos, and verify alignment before lowering."
            ),
        }
    goal = fs.get("goal_pos")
    objs = fs.get("objects") or {}
    p_obj = next((v for k, v in sorted(objs.items()) if k != "door"), {})
    p_pos = p_obj.get("pos") if isinstance(p_obj, Mapping) else None
    steps_used = int(rollout.get("steps_used", 0))
    if goal is not None and p_pos is not None:
        final_xy = math.hypot(float(p_pos[0]) - float(goal[0]), float(p_pos[1]) - float(goal[1]))
        final_dz = float(p_pos[2]) - float(goal[2])
        if final_xy < 0.030 and final_dz > 0.045:
            return {
                "layer": "control",
                "code": "insertion_too_high_or_step_exhausted",
                "summary": (
                    f"Object reached close XY alignment over goal (final_xy={final_xy:.4f}m, min_xy={min_obj_goal:.4f}m) "
                    f"but remained dz=+{final_dz:.3f}m above goal_pos[2] (steps_used={steps_used}/400)."
                ),
                "prescription": (
                    "Preserve the working grasp and XY alignment; tighten step efficiency in earlier phases and "
                    "complete smooth vertical descent with orientation alignment before releasing."
                ),
            }
    return {
        "layer": "control",
        "code": "insertion_or_release_precision_error",
        "summary": (
            f"Object lifted ({lift_delta:.3f}m) and reached goal vicinity (min_obj_to_goal={min_obj_goal:.3f}m), "
            f"but final placement/release did not satisfy success (steps_used={steps_used}/400)."
        ),
        "prescription": "Preserve working grasp, refine final XY/yaw alignment and vertical seating onto goal_pos before opening the gripper.",
    }


# ---------------------------------------------------------------------------
# Single-Episode Executor (Supports Un-Reset Online Turn-2 for CaP-X)
# ---------------------------------------------------------------------------

def _make_robosuite_env(task_name: str, seed: int, *, horizon: int = 400) -> Any:
    import numpy as np
    import robosuite as suite

    np.random.seed(seed)
    env = suite.make(
        env_name=task_name,
        robots="Panda",
        has_renderer=False,
        has_offscreen_renderer=False,
        use_camera_obs=False,
        control_freq=20,
        horizon=horizon,
        ignore_done=True,
    )
    obs = env.reset()
    return env, obs


def _execute_source_with_raw_return(source: str, sdk: RobosuitePolicySDK) -> tuple[bool, str, str, Any]:
    chk = check_robosuite_ast_compliance(source)
    if not chk["compliant"]:
        return False, "AST sandbox rejected: " + "; ".join(chk["issues"][:3]), "", None
    ns = dict(_ROBO_SAFE_GLOBALS)
    try:
        exec(compile(source, "<trax-robosuite-policy>", "exec"), ns, ns)  # noqa: S102
        fn = ns.get("run_policy")
        if not callable(fn):
            return False, "Missing callable run_policy(sdk)", "", None
        ret = fn(sdk)
        return True, "", (repr(ret)[:180] if ret is not None else ""), ret
    except Exception as exc:  # noqa: BLE001
        return False, f"{type(exc).__name__}: {exc}", "", None


def _execute_source_on_sdk(source: str, sdk: RobosuitePolicySDK) -> tuple[bool, str, str]:
    ok, err, rep, _raw = _execute_source_with_raw_return(source, sdk)
    return ok, err, rep


def run_robosuite_active_probe(
    task_name: str,
    seed: int,
    dev_runs: Sequence[Mapping[str, Any]],
    *,
    horizon: int = 400,
) -> ProbeReport | None:
    """Execute 1 task-agnostic public-SDK diagnostic probe when development is stuck (0 dev passes).

    Decides probe family strictly from observed public state schema (``handle`` presence vs
    object-grasp failure), never from hardcoded task names or physics constants.
    """
    if any(r.get("success") for r in dev_runs):
        return None
    sample_fs = (dev_runs[0].get("final_state") or {}) if dev_runs else {}
    objs = sample_fs.get("objects") or {}
    codes = {str((r.get("attribution") or {}).get("code", "")) for r in dev_runs}

    if "handle" in objs or sample_fs.get("handle_qpos") is not None:
        probe_src = build_articulated_sweep_probe_source()
    elif "grasp_missed_or_slip" in codes or "approach_missed_object" in codes:
        probe_src = build_grasp_offset_scan_probe_source()
    else:
        return None

    t0 = time.monotonic()
    env, obs = _make_robosuite_env(task_name, seed, horizon=horizon)
    try:
        sdk = RobosuitePolicySDK(env, task_name, max_steps=horizon)
        sdk.bind_initial_obs(obs)
        ok, _err, _rep, raw = _execute_source_with_raw_return(probe_src, sdk)
        wall_s = time.monotonic() - t0
        if not ok or not isinstance(raw, Mapping):
            return None
        return parse_probe_telemetry(
            task_name,
            [raw],
            evidence_source="simulator",
            sim_resets_used=1,
            wall_s=wall_s,
        )
    finally:
        env.close()


def _extract_verified_helper_snippet(policy_source: str) -> str:
    """Extract AST-safe non-`run_policy` helper functions from a verified policy for cross-task Skill Toolbox reuse."""
    try:
        tree = ast.parse(policy_source)
    except SyntaxError:
        return ""
    lines = policy_source.splitlines()
    blocks: list[str] = []
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name != "run_policy" and not node.name.startswith("_"):
            if node.lineno is not None and node.end_lineno is not None:
                chunk = "\n".join(lines[node.lineno - 1 : node.end_lineno])
                if validate_skill_code_snippet(chunk)["valid"]:
                    blocks.append(chunk)
    return "\n\n".join(blocks[:3])


def run_robosuite_rollout(
    task_name: str,
    seed: int,
    source: str,
    *,
    horizon: int = 400,
    online_regen_fn: Callable[[dict[str, Any]], Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    """Execute one seeded Robosuite rollout (with optional un-reset online Turn-2 for CaP-X)."""
    t0 = time.monotonic()
    env, obs = _make_robosuite_env(task_name, seed, horizon=horizon)
    try:
        sdk = RobosuitePolicySDK(env, task_name, max_steps=horizon)
        sdk.bind_initial_obs(obs)
        ok_exec, err_msg, policy_ret = _execute_source_on_sdk(source, sdk)
        turn1_success = bool(env._check_success())
        eval_llm_calls = 0
        eval_llm_wall_s = 0.0
        online_triggered = False

        # Optional CaP-X online multi-turn regeneration on the SAME un-reset episode
        if (not turn1_success) and (online_regen_fn is not None) and (sdk._steps_used < horizon - 20):
            online_triggered = True
            eval_llm_calls = 1
            st_mid = sdk.get_state()
            regen = online_regen_fn(
                {
                    "seed": seed,
                    "error": err_msg,
                    "state": st_mid,
                    "trace": sdk.trace[-10:],
                }
            )
            eval_llm_wall_s = float(regen.get("wall_s", 0.0))
            repaired = str(regen.get("source", ""))
            if repaired:
                ok_exec2, err_msg2, policy_ret2 = _execute_source_on_sdk(repaired, sdk)
                if policy_ret2:
                    policy_ret = policy_ret2
                if not ok_exec2:
                    err_msg = err_msg2

        final_success = bool(env._check_success())
        final_st = sdk.get_state()
        init_z = float(sdk.init_obj_pos[2]) if sdk.init_obj_pos else 0.0
        lift_delta = round(max(0.0, sdk.max_obj_z_seen - init_z), 4)
        wall_s = round(time.monotonic() - t0, 3)

        rec = {
            "task": task_name,
            "seed": seed,
            "success": final_success,
            "turn1_success": turn1_success,
            "program_ok": ok_exec,
            "error": err_msg,
            "policy_return": policy_ret,
            "steps_used": sdk._steps_used,
            "wall_s": wall_s,
            "eval_llm_calls": eval_llm_calls,
            "eval_llm_wall_s": round(eval_llm_wall_s, 3),
            "online_regen_triggered": online_triggered,
            "min_eef_to_obj_m": round(sdk.min_eef_to_obj_m, 4),
            "obj_lift_delta_m": lift_delta,
            "min_obj_to_goal_m": round(sdk.min_obj_to_goal_m, 4),
            "ever_closed_gripper": sdk.ever_closed_gripper,
            "closed_gripper_width_max": round(sdk.closed_gripper_width_max, 4),
            "max_handle_qpos": round(sdk.max_handle_qpos, 4),
            "max_hinge_qpos": round(sdk.max_hinge_qpos, 4),
            "final_state": final_st,
            "trace_tail": sdk.trace[-8:],
        }
        rec["attribution"] = attribute_robosuite_failure(rec, task_name)
        return rec
    finally:
        env.close()


# ---------------------------------------------------------------------------
# Protocol Prompts (100% Identical Shared Contract Across All 4 Methods)
# ---------------------------------------------------------------------------

def _robo_initial_prompt(task_name: str, extra: str = "") -> str:
    parts = [
        f"Write a generalizable Python robot manipulation policy `run_policy(sdk)` for Robosuite task `{task_name}`.",
        "",
        ROBOSUITE_TASK_DESCRIPTIONS[task_name],
        "",
        SHARED_ROBOSUITE_SDK_CONTRACT,
    ]
    if extra.strip():
        parts.extend(["", extra.strip()])
    return "\n".join(parts)


def _robo_aspire_prompt(task_name: str, prev_src: str, dev_runs: Sequence[Mapping[str, Any]], it: int) -> str:
    logs = [
        f"- seed={r['seed']} success={r['success']} steps={r['steps_used']} err={r['error']!r} "
        f"final_eef={r['final_state']['eef_pos']} objs={r['final_state']['objects']} "
        f"goal={r['final_state']['goal_pos']} handle_qpos={r['final_state']['handle_qpos']} hinge_qpos={r['final_state']['hinge_qpos']}"
        for r in dev_runs
    ]
    return "\n".join(
        [
            f"ASPIRE Iterative Debugging Revision for `{task_name}` (iteration {it}):",
            "Diagnose the failed dev rollouts from the raw logs below and output a revised, generalizable `run_policy(sdk)`.",
            "",
            ROBOSUITE_TASK_DESCRIPTIONS[task_name],
            "",
            SHARED_ROBOSUITE_SDK_CONTRACT,
            "",
            "Previous Policy:",
            "```python",
            prev_src,
            "```",
            "",
            "Dev Rollout Logs:",
            *logs,
            "",
            "Return ONLY the complete revised Python code.",
        ]
    )


def _robo_capx_prompt(
    task_name: str,
    prev_src: str,
    failed_run: Mapping[str, Any],
    skill_docs: str,
    *,
    online_turn: bool = False,
) -> str:
    hdr = (
        f"CaP-X Online Multi-Turn Regeneration for `{task_name}` (NO reset performed — resume from current physical state):"
        if online_turn
        else f"CaP-X Development Execution-Feedback Regeneration for `{task_name}`:"
    )
    st = failed_run.get("state") or failed_run.get("final_state") or {}
    return "\n".join(
        [
            hdr,
            "",
            ROBOSUITE_TASK_DESCRIPTIONS[task_name],
            "",
            SHARED_ROBOSUITE_SDK_CONTRACT,
            "",
            skill_docs,
            "",
            "Previous Program:",
            "```python",
            prev_src,
            "```",
            "",
            "Environment Feedback:",
            f"  Seed: {failed_run.get('seed')}",
            f"  Error: {failed_run.get('error', '')}",
            f"  Current State: {json.dumps(st)}",
            f"  Recent Ops: {json.dumps(failed_run.get('trace') or failed_run.get('trace_tail') or [])}",
            "",
            "Output ONLY corrected Python code defining `run_policy(sdk)`.",
        ]
    )


def _robo_simex_direct_prompt(task_name: str, prev_src: str, failed_run: Mapping[str, Any], it: int) -> str:
    return "\n".join(
        [
            f"Direct Failure Repair for `{task_name}` (iteration {it}, single-rollout feedback without hypothesis screening):",
            "Edit the policy below to resolve the observed rollout failure.",
            "",
            ROBOSUITE_TASK_DESCRIPTIONS[task_name],
            "",
            SHARED_ROBOSUITE_SDK_CONTRACT,
            "",
            "Previous Policy:",
            "```python",
            prev_src,
            "```",
            "",
            f"Failed Rollout: seed={failed_run.get('seed')} steps={failed_run.get('steps_used')} err={failed_run.get('error')!r} "
            f"final_eef={failed_run.get('final_state', {}).get('eef_pos')} "
            f"handle_qpos={failed_run.get('max_handle_qpos')} hinge_qpos={failed_run.get('max_hinge_qpos')}",
            "",
            "Return ONLY the revised Python code defining `run_policy(sdk)`.",
        ]
    )


def _robo_simex_protocol_prompt(
    task_name: str,
    prev_src: str,
    dev_runs: Sequence[Mapping[str, Any]],
    passed_ever: set[int],
    hypothesis_slot: int,
    toolbox_notes: Sequence[str],
    it: int,
) -> str:
    """Matched-budget SimEX-Protocol (Adapted) prompt: multi-hypothesis repair + capability bank + paired toolbox."""
    hyp_angles = (
        "Hypothesis A (Primary Physical Mechanics): Re-examine object/mechanism contact geometry, grasp point, or actuation depth from the failed rollout traces and propose a distinct physical strategy.",
        "Hypothesis B (Closed-Loop Servoing & Timing): Use tighter closed-loop live state feedback (`sdk.get_state()` + `move_delta`/`step_raw`) and trim wasted steps so every stage completes reliably within horizon.",
        "Hypothesis C (Regression-Safe Recovery Branch): Preserve the core trajectory that works on passing seeds and add a verified local recovery/retry branch when intermediate progress checks fail.",
    )
    angle = hyp_angles[hypothesis_slot % len(hyp_angles)]
    logs = [
        f"- seed={r['seed']} success={r['success']} steps={r['steps_used']} err={r['error']!r} "
        f"final_eef={r['final_state']['eef_pos']} objs={r['final_state']['objects']} "
        f"goal={r['final_state']['goal_pos']} handle_qpos={r['final_state']['handle_qpos']} hinge_qpos={r['final_state']['hinge_qpos']}"
        for r in dev_runs
    ]
    tb_block = (
        ["Paired Toolbox & Skill Notes (SimEX persistent memory):", *[f"- {t}" for t in toolbox_notes], ""]
        if toolbox_notes
        else []
    )
    return "\n".join(
        [
            f"SimEX-Protocol (Adapted) Multi-Hypothesis Candidate {hypothesis_slot + 1} for `{task_name}` (iteration {it}):",
            angle,
            "Screen requirement: preserve all previously passing seeds in the capability bank while fixing failing seeds.",
            "",
            *tb_block,
            ROBOSUITE_TASK_DESCRIPTIONS[task_name],
            "",
            SHARED_ROBOSUITE_SDK_CONTRACT,
            "",
            "Current Best Policy:",
            "```python",
            prev_src,
            "```",
            "",
            f"Capability-Bank Passing Seeds to Preserve: {sorted(passed_ever)}",
            "Dev Rollout Traces:",
            *logs,
            "",
            "Return ONLY the complete revised Python code defining `run_policy(sdk)`.",
        ]
    )


def _robo_craft_hypotheses(
    task_name: str,
    dev_runs: Sequence[Mapping[str, Any]],
    *,
    stagnant_iters: int = 0,
) -> list[tuple[str, str]]:
    """Formulate 100% task-agnostic, zero-leakage repair hypotheses from factual telemetry.

    Adaptive mode-switching RepairScreener (strictly zero task-specific hardcoded physics constants):
    - When ``passed > 0`` and ``stagnant_iters == 0``: selective subroutine repair (RAX Section 5.5.2 style)
      preserving verified upstream subroutines while rewriting the failing stage.
    - When ``passed > 0`` and ``stagnant_iters >= 1`` (polish stalled): automatically escape narrow
      micro-edits and unlock full attribution-grounded mechanistic hypotheses so the LLM can redesign
      the failing stage cleanly without being over-constrained.
    - When ``passed == 0``: generic mechanistic hypotheses grounded strictly in telemetry codes
      and any empirically measured Active Physical Probe findings.
    """
    _ = task_name  # Explicitly unused: hypotheses must NEVER branch on task name!
    passed_cnt = sum(1 for r in dev_runs if r.get("success"))
    codes = [str((r.get("attribution") or {}).get("code", "")) for r in dev_runs if not r.get("success")]
    code_set = set(codes)

    if passed_cnt > 0 and stagnant_iters == 0:
        return [
            (
                f"H-Subroutine1: Selective Subroutine Repair for {len(codes)} Failing Seed(s) (Freeze Working Stages)",
                "Your baseline already passes multiple dev seeds. Keep working upstream subroutines frozen, inspect "
                "the exact telemetry and policy_return of the failing seeds, and cleanly rewrite ONLY the failing "
                "stage (e.g., alignment/offset compensation, step-budget margin, or seating/release logic) with 0 regressions.",
            ),
            (
                "H-Subroutine2: Preserve Working Stage + Add Closed-Loop Recovery Branch on Verification Failure",
                "Keep the primary execution path intact for passing seeds, and add a compact closed-loop recovery "
                "subroutine triggered when live state verification detects grasp, alignment, or seating failure.",
            ),
        ]

    if "never_closed_gripper_early_abort" in code_set:
        return [
            (
                "H-Unblock1: Remove Strict Early-Abort Guards Before Gripper Actuation",
                "Telemetry shows the policy never closed the gripper because a strict intermediate convergence check "
                "aborted early. Relax intermediate waypoint gating so gripper actuation executes once near the object.",
            ),
            (
                "H-Unblock2: Simplified Direct Closed-Loop Sequence",
                "Replace multi-stage brittle gating with a clean direct open -> approach -> contact -> actuate -> "
                "verify sequence using live coordinates from sdk.get_state().",
            ),
        ]

    if "latch_not_unlatched" in code_set or "door_pull_arc_incomplete" in code_set or "handle_approach_missed" in code_set:
        return [
            (
                "H-Artic1: Probe-Guided Articulated Engagement & Full-Stroke Actuation",
                "Ground approach direction, actuation stroke, and pull vector strictly in the Empirical Active-Probe "
                "Findings and live handle/hinge telemetry. Ensure the mechanism releases before and during the pull arc.",
            ),
            (
                "H-Artic2: Continuous Closed-Loop Servoing Along Live Mechanism Trajectory",
                "Use compact closed-loop incremental moves (`move_delta` / `step_raw`) continuously re-reading live "
                "handle/hinge state so the end-effector tracks the articulated arc without slipping off.",
            ),
            (
                "H-Artic3: Alternative Contact Geometry + Step-Budget Reallocation",
                "Test an alternative contact/grasp posture on the handle (e.g., firm grip vs top-contact press) and "
                "reallocate steps so sufficient budget remains for the full opening stroke.",
            ),
        ]

    if "insertion_too_high_or_step_exhausted" in code_set or "insertion_or_release_precision_error" in code_set:
        return [
            (
                "H-Place1: Preserve Working Grasp + Tighten Alignment & Full Vertical Seating",
                "Keep the working grasp intact, remove premature release/abort checks, compensate for any grasp-to-center "
                "offset, align orientation cleanly over live goal_pos, and descend smoothly until seated before releasing.",
            ),
            (
                "H-Place2: Closed-Loop 6D Servoing via step_raw + Step-Budget Optimization",
                "Trim unnecessary hover/approach steps so ample budget remains for final seating, and use closed-loop "
                "6D orientation-stabilized control (`step_raw`) to maintain level posture during final descent.",
            ),
            (
                "H-Place3: Guided Micro-Correction Descent on Stall Detection",
                "During final descent over goal_pos, monitor live object Z progress; if vertical descent stalls above "
                "goal height, apply small closed-loop XY/yaw micro-adjustments before opening the gripper.",
            ),
        ]

    return [
        (
            f"H-Gen1: Probe-Grounded Contact & Grasp Strategy ({','.join(sorted(code_set)) or 'general'})",
            "Use the Empirical Active-Probe Findings (if present) and live object pose/yaw to select a verified grasp "
            "point, orient the gripper cleanly, verify lift via live Z delta, and compensate for any grasp offset at goal.",
        ),
        (
            "H-Gen2: High-Clearance Transport & Closed-Loop Goal Verification",
            "Verify grasp before transport, lift cleanly above workspace obstacles, servo tightly over live goal_pos "
            "using closed-loop feedback, and verify final object pose before releasing.",
        ),
        (
            "H-Gen3: Compact Budget + Re-Grasp Recovery Within Horizon",
            "Keep nominal approach compact and add one clean re-grasp fallback if live vertical lift verification fails.",
        ),
    ]


def _robo_craft_prompt(
    task_name: str,
    prev_src: str,
    dev_runs: Sequence[Mapping[str, Any]],
    static_issues: Sequence[str],
    reg_seeds: Sequence[int],
    it: int,
    *,
    hypothesis_title: str = "",
    hypothesis_strategy: str = "",
    skill_hints: Sequence[str] = (),
    probe_report: ProbeReport | None = None,
    failed_attempts: Sequence[Mapping[str, Any]] = (),
) -> str:
    diag_lines: list[str] = []
    passed_seeds = [int(r["seed"]) for r in dev_runs if r.get("success")]
    failed_runs = [r for r in dev_runs if not r.get("success")]
    seen_prescriptions: dict[str, str] = {}

    for r in dev_runs:
        fs = r.get("final_state") or {}
        if r["success"]:
            diag_lines.append(
                f"- seed={r['seed']}: PASS (steps={r['steps_used']}, max_handle={r['max_handle_qpos']}, max_hinge={r['max_hinge_qpos']})"
            )
        else:
            a = r["attribution"]
            seen_prescriptions.setdefault(f"[{a['layer']}:{a['code']}]", a["prescription"])
            goal = fs.get("goal_pos")
            objs = fs.get("objects") or {}
            p_obj = objs.get("nut") or objs.get("can") or objs.get("cube") or {}
            p_pos = p_obj.get("pos")
            dz_str = f" final_dz={float(p_pos[2]) - float(goal[2]):+.3f}m" if (goal and p_pos) else ""
            ret_str = f" ret={r.get('policy_return')}" if r.get("policy_return") else ""
            err_str = f" err={r.get('error')!r}" if r.get("error") else ""
            diag_lines.append(
                f"- seed={r['seed']}: FAIL [{a['code']}] steps={r['steps_used']} "
                f"min_eef_obj={r['min_eef_to_obj_m']} closed={r.get('ever_closed_gripper')} "
                f"closed_w={r['closed_gripper_width_max']} lift={r['obj_lift_delta_m']} "
                f"min_obj_goal={r['min_obj_to_goal_m']}{dz_str} "
                f"max_handle={r['max_handle_qpos']} max_hinge={r['max_hinge_qpos']}{err_str}{ret_str}\n"
                f"    final_eef={fs.get('eef_pos')} objs={fs.get('objects')} goal={fs.get('goal_pos')}"
            )

    rx_lines = [f"- {code}: {rx}" for code, rx in seen_prescriptions.items()]
    first_trace = (
        [f"Sample Trace Tail (seed={failed_runs[0]['seed']}): {json.dumps(failed_runs[0].get('trace_tail') or [])}"]
        if failed_runs
        else []
    )
    skill_block = (
        ["Verified Panda SDK SkillExperience (from earlier verified tasks):", *[f"- {s}" for s in skill_hints], ""]
        if skill_hints
        else []
    )
    hyp_block = (
        [f"RepairScreener Focus ({hypothesis_title}): {hypothesis_strategy}", ""]
        if hypothesis_title
        else []
    )
    probe_block = (
        [probe_report.render_prompt_block(), ""]
        if (probe_report is not None and probe_report.findings)
        else []
    )
    fail_codes = [str((r.get("attribution") or {}).get("code", "")) for r in failed_runs]
    sub_analysis = analyze_policy_subroutines(prev_src, fail_codes)
    sub_block: list[str] = []
    if sub_analysis.get("subroutines"):
        fn_summaries = ", ".join(
            f"{s['name']}({s['lines']}L, sdk={s['sdk_methods']})" for s in sub_analysis["subroutines"]
        )
        sub_block = [
            f"Selective Subroutine Scope ({sub_analysis['repair_scope']}): {sub_analysis['freeze_hint']}",
            f"Current Policy Subroutines: {fn_summaries}",
            "",
        ]
    attempts_block: list[str] = []
    if failed_attempts:
        attempts_block = ["Recent Non-Improving Attempts (do NOT repeat these dead-end edits):"]
        for fa in failed_attempts[-3:]:
            attempts_block.append(
                f"- it={fa.get('iter')} [{fa.get('hypothesis')}] -> dev={fa.get('dev_passed')}/{fa.get('dev_total')} "
                f"prog={fa.get('progress_score')} attrs={fa.get('attributions')}"
            )
        attempts_block.append("")
    return "\n".join(
        [
            f"TRAX RepairScreener Evolution for `{task_name}` (iteration {it}):",
            "Fix the attributed failure below while preserving ALL passing dev seeds.",
            "",
            *hyp_block,
            *sub_block,
            *probe_block,
            *attempts_block,
            *skill_block,
            ROBOSUITE_TASK_DESCRIPTIONS[task_name],
            "",
            SHARED_ROBOSUITE_SDK_CONTRACT,
            "",
            "Best Baseline Candidate Code:",
            "```python",
            prev_src,
            "```",
            "",
            *(["Static Preflight Issues:", *static_issues, ""] if static_issues else []),
            "Dev Rollout Telemetry:",
            *diag_lines,
            "",
            "Deduplicated Four-Layer Causal Diagnosis:",
            *(rx_lines if rx_lines else ["- None"]),
            *first_trace,
            "",
            f"Passing Seeds to Preserve (0 regressions): {passed_seeds} | Active Regression Seeds: {list(reg_seeds)}",
            "Return ONLY the complete revised Python code defining `run_policy(sdk)`.",
        ]
    )


def _robo_candidate_score(task_name: str, runs: Sequence[Mapping[str, Any]], passed_ever: set[int]) -> tuple[int, int, float]:
    """Reliability-first screening score: (dev_passed, -regressions, physical_progress_margin)."""
    passed = sum(1 for r in runs if r["success"])
    cur_ok = {int(r["seed"]) for r in runs if r["success"]}
    regs = len(passed_ever - cur_ok)
    if task_name == "Door":
        progress = sum(float(r.get("max_hinge_qpos", 0.0)) + 0.1 * float(r.get("max_handle_qpos", 0.0)) for r in runs) / max(1, len(runs))
    elif task_name in {"PickPlaceCan", "NutAssemblySquare"}:
        vals = []
        for r in runs:
            lift = float(r.get("obj_lift_delta_m", 0.0))
            xy_d = float(r.get("min_obj_to_goal_m", 1.0))
            dz_pen = 0.0
            if lift >= 0.04 and xy_d < 0.06:
                fs = r.get("final_state") or {}
                goal = fs.get("goal_pos")
                objs = fs.get("objects") or {}
                p_obj = objs.get("nut") or objs.get("can") or {}
                p_pos = p_obj.get("pos")
                if goal and p_pos:
                    dz_pen = max(0.0, float(p_pos[2]) - float(goal[2]))
            vals.append(lift - xy_d - 0.5 * dz_pen)
        progress = sum(vals) / max(1, len(vals))
    else:
        progress = sum(float(r.get("obj_lift_delta_m", 0.0)) - float(r.get("min_eef_to_obj_m", 1.0)) for r in runs) / max(1, len(runs))
    return (passed, -regs, round(progress, 5))


# ---------------------------------------------------------------------------
# Per-Task Development & Held-Out Evaluation Orchestrator
# ---------------------------------------------------------------------------

def _develop_task_methods(
    task_name: str,
    llm_fn: LLMGenerateFn,
    dev_seeds: Sequence[int],
    *,
    max_dev_llm_calls: int = 4,
    horizon: int = 400,
    skill_hints: Sequence[str] = (),
    simex_toolbox_notes: Sequence[str] = (),
    enable_active_probing: bool = True,
    log_cb: Callable[[str], None] | None = None,
) -> dict[str, dict[str, Any]]:
    def _log(msg: str) -> None:
        if log_cb:
            log_cb(msg)

    artifacts: dict[str, dict[str, Any]] = {}

    # 1. Shared Paired Initial Draft = Strong Zero-Shot (1 call, 0 dev rollouts)
    # All iterative methods start from this EXACT same it=1 draft so protocol differences
    # are measured cleanly without stochastic initialization lottery variance.
    _log(f"[{task_name}] Dev 1/5: Strong Zero-Shot (shared paired it=1 draft)...")
    t0 = time.monotonic()
    g_zs = llm_fn(_robo_initial_prompt(task_name))
    src_zs = str(g_zs["source"])
    zs_wall = round(time.monotonic() - t0, 3)
    artifacts["strong_zeroshot"] = {
        "method_id": "strong_zeroshot",
        "label": "Strong Zero-Shot",
        "provenance": "baseline_control",
        "online_mode": "frozen",
        "source": src_zs,
        "sha256": digest(src_zs),
        "dev_llm_calls": 1,
        "dev_sim_resets": 0,
        "dev_wall_s": zs_wall,
        "dev_passed": 0,
        "dev_total": 0,
        "history": [],
    }
    # Evaluate shared it=1 draft once on dev_seeds for all iterative methods
    init_dev_runs = [run_robosuite_rollout(task_name, s, src_zs, horizon=horizon) for s in dev_seeds]
    init_dev_passed = sum(1 for r in init_dev_runs if r["success"])
    _log(f"  [{task_name}][Shared it=1] dev={init_dev_passed}/{len(dev_seeds)}")

    # 2. ASPIRE-Protocol (iterative raw-trace debugging on dev_seeds)
    _log(f"[{task_name}] Dev 2/5: ASPIRE-Protocol (dev seeds {list(dev_seeds)})...")
    t0 = time.monotonic()
    best_src, best_pass, resets, calls = src_zs, init_dev_passed, len(dev_seeds), 1
    hist: list[dict[str, Any]] = [
        {"iter": 1, "dev_passed": init_dev_passed, "dev_total": len(dev_seeds), "sha": digest(src_zs)[:12]}
    ]
    cur_src, last_runs = src_zs, list(init_dev_runs)
    for it in range(2, max_dev_llm_calls + 1):
        if best_pass == len(dev_seeds):
            break
        prompt = _robo_aspire_prompt(task_name, cur_src, last_runs, it)
        gen = llm_fn(prompt)
        calls += 1
        cur_src = str(gen["source"])
        runs = [run_robosuite_rollout(task_name, s, cur_src, horizon=horizon) for s in dev_seeds]
        resets += len(dev_seeds)
        last_runs = runs
        passed = sum(1 for r in runs if r["success"])
        hist.append({"iter": it, "dev_passed": passed, "dev_total": len(dev_seeds), "sha": digest(cur_src)[:12]})
        _log(f"  [{task_name}][ASPIRE it={it}] dev={passed}/{len(dev_seeds)}")
        if passed >= best_pass:
            best_pass, best_src = passed, cur_src
        if passed == len(dev_seeds):
            break
    artifacts["aspire_protocol"] = {
        "method_id": "aspire_protocol",
        "label": "ASPIRE-Protocol (Adapted)",
        "provenance": "protocol_adapted_from_official_repo",
        "online_mode": "frozen",
        "source": best_src,
        "sha256": digest(best_src),
        "dev_llm_calls": calls,
        "dev_sim_resets": resets,
        "dev_wall_s": round(zs_wall + (time.monotonic() - t0), 3),
        "dev_passed": max(0, best_pass),
        "dev_total": len(dev_seeds),
        "history": hist,
    }

    # 3. CaP-X-Protocol (execution feedback + SkillLibrary + online regen)
    _log(f"[{task_name}] Dev 3/5: CaP-X-Protocol (dev seeds {list(dev_seeds)})...")
    t0 = time.monotonic()
    skill_lib = CapXSkillLibrary()
    for r in init_dev_runs:
        if r["success"]:
            skill_lib.extract_from_code(src_zs, task_name=f"{task_name}_s{r['seed']}")
    init_failed = [r for r in init_dev_runs if not r["success"]]
    best_src, best_pass, resets, calls = src_zs, init_dev_passed, len(dev_seeds), 1
    hist = [
        {
            "iter": 1,
            "dev_passed": init_dev_passed,
            "dev_total": len(dev_seeds),
            "promoted_skills": [k for k, v in skill_lib.skills.items() if v.get("promoted")],
            "sha": digest(src_zs)[:12],
        }
    ]
    cur_src, last_fail = src_zs, (init_failed[0] if init_failed else init_dev_runs[0])
    for it in range(2, max_dev_llm_calls + 1):
        if best_pass == len(dev_seeds):
            break
        prompt = _robo_capx_prompt(task_name, cur_src, last_fail, skill_lib.render_docs(), online_turn=False)
        gen = llm_fn(prompt)
        calls += 1
        cur_src = str(gen["source"])
        runs = [run_robosuite_rollout(task_name, s, cur_src, horizon=horizon) for s in dev_seeds]
        resets += len(dev_seeds)
        passed = sum(1 for r in runs if r["success"])
        for r in runs:
            if r["success"]:
                skill_lib.extract_from_code(cur_src, task_name=f"{task_name}_s{r['seed']}")
        failed = [r for r in runs if not r["success"]]
        last_fail = failed[0] if failed else runs[0]
        hist.append(
            {
                "iter": it,
                "dev_passed": passed,
                "dev_total": len(dev_seeds),
                "promoted_skills": [k for k, v in skill_lib.skills.items() if v.get("promoted")],
                "sha": digest(cur_src)[:12],
            }
        )
        _log(f"  [{task_name}][CaP-X it={it}] dev={passed}/{len(dev_seeds)}")
        if passed >= best_pass:
            best_pass, best_src = passed, cur_src
        if passed == len(dev_seeds):
            break
    artifacts["capx_protocol"] = {
        "method_id": "capx_protocol",
        "label": "CaP-X-Protocol (Adapted)",
        "provenance": "protocol_adapted_from_official_repo",
        "online_mode": "capx_online_regen",
        "source": best_src,
        "sha256": digest(best_src),
        "skill_library": skill_lib.to_dict(),
        "dev_llm_calls": calls,
        "dev_sim_resets": resets,
        "dev_wall_s": round(zs_wall + (time.monotonic() - t0), 3),
        "dev_passed": max(0, best_pass),
        "dev_total": len(dev_seeds),
        "history": hist,
    }

    # 4. SimEX-Direct-Repair (single-rollout direct repair without hypothesis screening / regression bank)
    _log(f"[{task_name}] Dev 4/5: SimEX-Direct-Repair (dev seeds {list(dev_seeds)})...")
    t0 = time.monotonic()
    best_src, best_pass, resets, calls = src_zs, init_dev_passed, len(dev_seeds), 1
    hist = [
        {"iter": 1, "dev_passed": init_dev_passed, "dev_total": len(dev_seeds), "sha": digest(src_zs)[:12]}
    ]
    cur_src, last_fail = src_zs, (init_failed[0] if init_failed else init_dev_runs[0])
    for it in range(2, max_dev_llm_calls + 1):
        if best_pass == len(dev_seeds):
            break
        prompt = _robo_simex_direct_prompt(task_name, cur_src, last_fail, it)
        gen = llm_fn(prompt)
        calls += 1
        cur_src = str(gen["source"])
        runs = [run_robosuite_rollout(task_name, s, cur_src, horizon=horizon) for s in dev_seeds]
        resets += len(dev_seeds)
        passed = sum(1 for r in runs if r["success"])
        failed = [r for r in runs if not r["success"]]
        last_fail = failed[0] if failed else runs[0]
        hist.append({"iter": it, "dev_passed": passed, "dev_total": len(dev_seeds), "sha": digest(cur_src)[:12]})
        _log(f"  [{task_name}][SimEX-Direct it={it}] dev={passed}/{len(dev_seeds)}")
        # Direct repair commits latest revision without global regression bank screening
        best_src, best_pass = cur_src, passed
        if passed == len(dev_seeds):
            break
    artifacts["simex_direct_repair"] = {
        "method_id": "simex_direct_repair",
        "label": "SimEX-Direct-Repair (Ablation)",
        "provenance": "ablation_protocol_from_simex_2609_38982",
        "online_mode": "frozen",
        "source": best_src,
        "sha256": digest(best_src),
        "dev_llm_calls": calls,
        "dev_sim_resets": resets,
        "dev_wall_s": round(zs_wall + (time.monotonic() - t0), 3),
        "dev_passed": max(0, best_pass),
        "dev_total": len(dev_seeds),
        "history": hist,
    }

    # 5. SimEX-Protocol (Adapted): multi-hypothesis repair + capability-bank regression replay + paired toolbox
    _log(f"[{task_name}] Dev 5/6: SimEX-Protocol (Adapted, multi-hypothesis + capability bank)...")
    t0 = time.monotonic()
    sx_passed_ever: set[int] = {int(r["seed"]) for r in init_dev_runs if r["success"]}
    sx_best_src, sx_best_pass, sx_best_regs = src_zs, init_dev_passed, 0
    sx_best_runs: list[dict[str, Any]] = list(init_dev_runs)
    resets, calls = len(dev_seeds), 1
    hist = [
        {
            "iter": 1,
            "hypothesis_slot": 0,
            "dev_passed": init_dev_passed,
            "dev_total": len(dev_seeds),
            "regressions": 0,
            "sha": digest(src_zs)[:12],
        }
    ]
    for it in range(2, max_dev_llm_calls + 1):
        if sx_best_pass == len(dev_seeds):
            break
        h_slot = it - 2
        prompt = _robo_simex_protocol_prompt(
            task_name,
            sx_best_src,
            sx_best_runs,
            sx_passed_ever,
            h_slot,
            simex_toolbox_notes,
            it,
        )
        gen = llm_fn(prompt)
        calls += 1
        cur_src = str(gen["source"])
        runs = [run_robosuite_rollout(task_name, s, cur_src, horizon=horizon) for s in dev_seeds]
        resets += len(dev_seeds)
        passed = sum(1 for r in runs if r["success"])
        cur_ok = {int(r["seed"]) for r in runs if r["success"]}
        regs = len(sx_passed_ever - cur_ok)
        sx_passed_ever.update(cur_ok)
        hist.append(
            {
                "iter": it,
                "hypothesis_slot": h_slot,
                "dev_passed": passed,
                "dev_total": len(dev_seeds),
                "regressions": regs,
                "sha": digest(cur_src)[:12],
            }
        )
        _log(f"  [{task_name}][SimEX-Protocol it={it}] dev={passed}/{len(dev_seeds)} regs={regs}")
        if (passed, -regs) >= (sx_best_pass, -sx_best_regs):
            sx_best_pass, sx_best_regs, sx_best_src, sx_best_runs = passed, regs, cur_src, runs
        if passed == len(dev_seeds):
            break
    artifacts["simex_protocol"] = {
        "method_id": "simex_protocol",
        "label": "SimEX-Protocol (Adapted)",
        "provenance": "protocol_adapted_from_simex_2609_38982",
        "online_mode": "frozen",
        "source": sx_best_src,
        "sha256": digest(sx_best_src),
        "dev_llm_calls": calls,
        "dev_sim_resets": resets,
        "dev_wall_s": round(zs_wall + (time.monotonic() - t0), 3),
        "dev_passed": max(0, sx_best_pass),
        "dev_total": len(dev_seeds),
        "history": hist,
    }

    # 6. TRAX Full Engine (Zero-Leakage Active Probing + AST preflight + 4-layer attribution + RepairScreener + Regression Gate)
    _log(f"[{task_name}] Dev 6/6: TRAX Full Engine (Active Probing + RepairScreener + Zero-Leakage)...")
    t0 = time.monotonic()
    reg_seeds: set[int] = {int(r["seed"]) for r in init_dev_runs if not r["success"]}
    passed_ever: set[int] = {int(r["seed"]) for r in init_dev_runs if r["success"]}
    init_score = _robo_candidate_score(task_name, init_dev_runs, passed_ever)
    best_src, best_score, best_pass, resets, calls = src_zs, init_score, init_dev_passed, len(dev_seeds), 1
    best_runs: list[dict[str, Any]] = list(init_dev_runs)

    probe_report: ProbeReport | None = None
    if enable_active_probing and init_dev_passed == 0 and dev_seeds:
        probe_report = run_robosuite_active_probe(task_name, int(dev_seeds[0]), init_dev_runs, horizon=horizon)
        if probe_report is not None:
            resets += probe_report.sim_resets_used
            _log(
                f"  [{task_name}][TRAX ActiveProbe] +{probe_report.sim_resets_used} sim reset -> "
                f"{[f.summary for f in probe_report.findings]}"
            )

    hist = [
        {
            "iter": 1,
            "hypothesis": "Shared Initial Policy",
            "static_preflight_passed": check_robosuite_ast_compliance(src_zs)["compliant"],
            "dev_passed": init_dev_passed,
            "dev_total": len(dev_seeds),
            "progress_score": init_score[2],
            "attributions": [r["attribution"]["code"] for r in init_dev_runs if not r["success"]],
            "regression_seeds": sorted(reg_seeds),
            "probe_report": probe_report.to_dict() if probe_report else None,
            "sha": digest(src_zs)[:12],
        }
    ]
    cur_src, last_static = src_zs, []
    stagnant_iters = 0
    failed_attempts: list[dict[str, Any]] = []
    for it in range(2, max_dev_llm_calls + 1):
        if best_pass == len(dev_seeds):
            break
        hyps = _robo_craft_hypotheses(task_name, best_runs, stagnant_iters=stagnant_iters)
        h_idx = (it - 2) % len(hyps)
        ht, hs = hyps[h_idx]
        prompt = _robo_craft_prompt(
            task_name,
            best_src or cur_src,
            best_runs,
            last_static,
            sorted(reg_seeds),
            it,
            hypothesis_title=ht,
            hypothesis_strategy=hs,
            skill_hints=skill_hints,
            probe_report=probe_report,
            failed_attempts=failed_attempts,
        )
        gen = llm_fn(prompt)
        calls += 1
        cur_src = str(gen["source"])
        ast_chk = check_robosuite_ast_compliance(cur_src)
        if not ast_chk["compliant"]:
            last_static = list(ast_chk["issues"])
            stagnant_iters += 1
            rec_fail = {"iter": it, "hypothesis": ht, "static_preflight_passed": False, "issues": last_static, "dev_passed": 0, "dev_total": len(dev_seeds), "progress_score": 0.0, "attributions": ["ast_preflight_blocked"]}
            hist.append(rec_fail)
            failed_attempts.append(rec_fail)
            _log(f"  [{task_name}][TRAX it={it}] AST preflight blocked (0 sim resets wasted)")
            continue
        last_static = []
        runs = [run_robosuite_rollout(task_name, s, cur_src, horizon=horizon) for s in dev_seeds]
        resets += len(dev_seeds)
        passed = sum(1 for r in runs if r["success"])
        for r in runs:
            if not r["success"]:
                reg_seeds.add(int(r["seed"]))
            else:
                passed_ever.add(int(r["seed"]))
        score = _robo_candidate_score(task_name, runs, passed_ever)
        rec_it = {
            "iter": it,
            "hypothesis": ht,
            "stagnant_iters_before": stagnant_iters,
            "static_preflight_passed": True,
            "dev_passed": passed,
            "dev_total": len(dev_seeds),
            "progress_score": score[2],
            "attributions": [r["attribution"]["code"] for r in runs if not r["success"]],
            "regression_seeds": sorted(reg_seeds),
            "sha": digest(cur_src)[:12],
        }
        hist.append(rec_it)
        _log(
            f"  [{task_name}][TRAX it={it}] ({ht}) dev={passed}/{len(dev_seeds)} "
            f"prog={score[2]:.3f} attr={[r['attribution']['code'] for r in runs if not r['success']]}"
        )
        if passed > best_pass:
            stagnant_iters = 0
        else:
            stagnant_iters += 1
            failed_attempts.append(rec_it)
        if score >= best_score:
            best_score, best_pass, best_src, best_runs = score, passed, cur_src, runs
        if passed == len(dev_seeds):
            break
    artifacts["craft_full"] = {
        "method_id": "craft_full",
        "label": "TRAX (ActiveProbe + RepairScreener + Skills)",
        "provenance": "craft_proposed_system",
        "online_mode": "frozen",
        "source": best_src,
        "sha256": digest(best_src),
        "dev_llm_calls": calls,
        "dev_sim_resets": resets,
        "dev_wall_s": round(zs_wall + (time.monotonic() - t0), 3),
        "dev_passed": max(0, best_pass),
        "dev_total": len(dev_seeds),
        "probe_report": probe_report.to_dict() if probe_report else None,
        "known_failure_modes": sorted({r["attribution"]["code"] for r in best_runs if not r.get("success")}),
        "history": hist,
    }
    return artifacts


def summarize_robosuite_benchmark(
    tasks: Sequence[str],
    eval_seeds: Sequence[int],
    method_meta: Mapping[str, Mapping[str, Any]],
    trials: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    method_ids = (
        "strong_zeroshot",
        "aspire_protocol",
        "capx_protocol",
        "simex_direct_repair",
        "simex_protocol",
        "craft_full",
    )
    summary: dict[str, dict[str, Any]] = {}

    for mid in method_ids:
        mt = [t for t in trials if t["method_id"] == mid]
        tot = len(mt)
        ok = sum(1 for t in mt if t["success"])
        t1_ok = sum(1 for t in mt if t["turn1_success"])
        by_task: dict[str, dict[str, Any]] = {}
        dev_llm = 0
        dev_rst = 0
        for task in tasks:
            tt = [t for t in mt if t["task"] == task]
            tp = sum(1 for t in tt if t["success"])
            t1p = sum(1 for t in tt if t["turn1_success"])
            by_task[task] = {
                "passed": tp,
                "turn1_passed": t1p,
                "total": len(tt),
                "rate": round(tp / len(tt), 4) if tt else 0.0,
                "wilson_95_ci": list(wilson_ci_95(tp, len(tt))),
            }
            tm = (method_meta.get(task) or {}).get(mid) or {}
            dev_llm += int(tm.get("dev_llm_calls", 0))
            dev_rst += int(tm.get("dev_sim_resets", 0))

        walls = [float(t["wall_s"]) for t in mt]
        eval_llm = sum(int(t.get("eval_llm_calls", 0)) for t in mt)
        label = next(((method_meta.get(tk) or {}).get(mid, {}).get("label") for tk in tasks if (method_meta.get(tk) or {}).get(mid)), mid)
        prov = next(((method_meta.get(tk) or {}).get(mid, {}).get("provenance") for tk in tasks if (method_meta.get(tk) or {}).get(mid)), "")
        summary[mid] = {
            "method_id": mid,
            "label": label,
            "provenance": prov,
            "total_trials": tot,
            "passed": ok,
            "pass_rate": round(ok / tot, 4) if tot else 0.0,
            "wilson_95_ci": list(wilson_ci_95(ok, tot)),
            "turn1_passed": t1_ok,
            "turn1_pass_rate": round(t1_ok / tot, 4) if tot else 0.0,
            "by_task": by_task,
            "dev_llm_calls": dev_llm,
            "dev_sim_resets": dev_rst,
            "eval_llm_calls": eval_llm,
            "mean_wall_s": round(sum(walls) / len(walls), 3) if walls else 0.0,
            "p95_wall_s": round(percentile(walls, 95.0) or 0.0, 3) if walls else 0.0,
        }

    # Paired exact McNemar across all (task, seed) pairs: TRAX vs each baseline
    keys = [(tk, sd) for tk in tasks for sd in eval_seeds]
    craft_map = {(t["task"], int(t["seed"])): bool(t["success"]) for t in trials if t["method_id"] == "craft_full"}
    paired: dict[str, dict[str, Any]] = {}
    if len(craft_map) == len(keys):
        c_vec = [craft_map[k] for k in keys]
        for mid in ("strong_zeroshot", "aspire_protocol", "capx_protocol", "simex_direct_repair", "simex_protocol"):
            b_map = {(t["task"], int(t["seed"])): bool(t["success"]) for t in trials if t["method_id"] == mid}
            if len(b_map) == len(keys):
                paired[mid] = mcnemar_exact_p(c_vec, [b_map[k] for k in keys])

    return {"by_method": summary, "paired_mcnemar_craft_vs": paired}


def render_robosuite_html(report: Mapping[str, Any]) -> str:
    by_method = list((report.get("summary_by_method") or {}).values())
    tasks = list(report.get("tasks") or [])
    mcnemar = report.get("paired_mcnemar_craft_vs") or {}
    colors = ["#94a3b8", "#f59e0b", "#a78bfa", "#38bdf8", "#06b6d4", "#22c55e"]

    bar_w, gap = 120, 48
    chart_w = max(680, 90 + len(by_method) * (bar_w + gap))
    bars = []
    for i, m in enumerate(by_method):
        x = 60 + i * (bar_w + gap)
        rate = float(m["pass_rate"])
        h = int(rate * 190)
        y = 220 - h
        ci = m.get("wilson_95_ci") or [0, 0]
        y_hi = 220 - int(ci[1] * 190)
        y_lo = 220 - int(ci[0] * 190)
        cx = x + bar_w // 2
        bars.append(
            f'<rect x="{x}" y="{y}" width="{bar_w}" height="{h}" rx="6" fill="{colors[i % len(colors)]}" opacity="0.88"/>'
            f'<line x1="{cx}" y1="{y_hi}" x2="{cx}" y2="{y_lo}" stroke="#f8fafc" stroke-width="2"/>'
            f'<line x1="{cx-8}" y1="{y_hi}" x2="{cx+8}" y2="{y_hi}" stroke="#f8fafc" stroke-width="2"/>'
            f'<line x1="{cx-8}" y1="{y_lo}" x2="{cx+8}" y2="{y_lo}" stroke="#f8fafc" stroke-width="2"/>'
            f'<text x="{cx}" y="{max(16, y-9)}" fill="#f8fafc" font-size="12" font-weight="700" text-anchor="middle">{rate*100:.1f}% ({m["passed"]}/{m["total_trials"]})</text>'
            f'<text x="{cx}" y="242" fill="#cbd5e1" font-size="11" text-anchor="middle">{html.escape(m["label"])}</text>'
            f'<text x="{cx}" y="257" fill="#64748b" font-size="10" text-anchor="middle">devLLM={m["dev_llm_calls"]} · evalLLM={m["eval_llm_calls"]}</text>'
        )

    t_headers = "".join(f"<th>{html.escape(tk)}</th>" for tk in tasks)
    rows = []
    for m in by_method:
        t_cells = []
        for tk in tasks:
            bt = (m.get("by_task") or {}).get(tk) or {}
            p, tot = bt.get("passed", 0), bt.get("total", 0)
            pct = (p / tot * 100.0) if tot else 0.0
            cls = "pass" if pct >= 80 else ("partial" if pct >= 40 else "fail")
            t_cells.append(f'<td class="{cls}">{p}/{tot} ({pct:.0f}%)</td>')
        mc = mcnemar.get(m["method_id"])
        mc_str = f"p={mc['p_value']} (+{mc['n_a_only']}/-{mc['n_b_only']})" if mc else "ref"
        t1_note = (
            f"<br><small>Turn-1 frozen: {m['turn1_passed']}/{m['total_trials']} ({m['turn1_pass_rate']*100:.1f}%)</small>"
            if m["method_id"] == "capx_protocol"
            else ""
        )
        rows.append(
            "<tr>"
            f"<td><strong>{html.escape(m['label'])}</strong><br><small>{html.escape(m['provenance'])}</small></td>"
            f"<td><strong>{m['passed']}/{m['total_trials']} ({m['pass_rate']*100:.1f}%)</strong>"
            f"<br><small>CI [{m['wilson_95_ci'][0]*100:.1f}%, {m['wilson_95_ci'][1]*100:.1f}%]</small>{t1_note}</td>"
            f"{''.join(t_cells)}"
            f"<td>dev={m['dev_llm_calls']} / rst={m['dev_sim_resets']}<br><small>evalLLM={m['eval_llm_calls']}</small></td>"
            f"<td>{m['mean_wall_s']}s / {m['p95_wall_s']}s</td>"
            f"<td><small>{html.escape(mc_str)}</small></td>"
            "</tr>"
        )

    prov = report.get("provenance") or {}
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<title>{html.escape(str(report.get('benchmark_version', ROBOSUITE_BENCH_VERSION)))}</title>
<style>
  body {{ font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; background: #070b14; color: #f1f5f9; margin: 0; padding: 26px; }}
  .wrap {{ max-width: 1280px; margin: 0 auto; }}
  .hero {{ background: #0f172a; border: 1px solid #1e2f4f; border-left: 4px solid #38bdf8; border-radius: 10px; padding: 16px 20px; margin-bottom: 18px; }}
  .badge {{ display: inline-block; padding: 3px 10px; border-radius: 999px; font-size: 12px; font-weight: 600; background: #1e293b; color: #7dd3fc; margin-right: 8px; }}
  .card {{ background: #0f172a; border: 1px solid #1e2f4f; border-radius: 10px; padding: 16px 18px; margin-bottom: 18px; overflow-x: auto; }}
  table {{ width: 100%; border-collapse: collapse; font-size: 12.5px; }}
  th, td {{ border-bottom: 1px solid #1e2f4f; padding: 8px 10px; text-align: left; }}
  th {{ color: #94a3b8; background: #0a101f; }}
  td.pass {{ background: rgba(34,197,94,0.14); color: #86efac; font-weight: 700; }}
  td.partial {{ background: rgba(245,158,11,0.16); color: #fde047; font-weight: 700; }}
  td.fail {{ background: rgba(239,68,68,0.16); color: #fca5a5; font-weight: 700; }}
  small {{ color: #94a3b8; }}
</style>
</head>
<body>
<div class="wrap">
  <div class="hero">
    <h1>{html.escape(str(report.get('benchmark_version', ROBOSUITE_BENCH_VERSION)))} — Public MuJoCo Robosuite Benchmark</h1>
    <div>
      <span class="badge">simulator=MuJoCo 3.3 / Robosuite 1.5 (Panda)</span>
      <span class="badge">model={html.escape(str(report.get('model')))} ({html.escape(str(report.get('reasoning_effort')))})</span>
      <span class="badge">dev_seeds={html.escape(str(report.get('dev_seeds')))}</span>
      <span class="badge">heldout_seeds={len(report.get('eval_seeds') or [])}/task</span>
    </div>
    <p style="margin:8px 0 0;font-size:12px;color:#94a3b8;">
      git={html.escape(str(prov.get('git_commit') or 'n/a')[:12])} · finished={html.escape(str(report.get('finished_at_utc') or ''))}
    </p>
  </div>
  <div class="card">
    <h2>1. Overall Held-Out Success Rate Across All Public Tasks (Wilson 95% CI)</h2>
    <svg viewBox="0 0 {chart_w} 275" style="width:100%;max-width:{chart_w}px;background:#0a101f;border-radius:8px;">
      <line x1="40" y1="220" x2="{chart_w-20}" y2="220" stroke="#334155" stroke-width="1"/>
      <line x1="40" y1="125" x2="{chart_w-20}" y2="125" stroke="#1e293b" stroke-dasharray="4 4"/>
      <line x1="40" y1="30" x2="{chart_w-20}" y2="30" stroke="#1e293b" stroke-dasharray="4 4"/>
      <text x="34" y="224" fill="#64748b" font-size="10" text-anchor="end">0%</text>
      <text x="34" y="129" fill="#64748b" font-size="10" text-anchor="end">50%</text>
      <text x="34" y="34" fill="#64748b" font-size="10" text-anchor="end">100%</text>
      {''.join(bars)}
    </svg>
  </div>
  <div class="card">
    <h2>2. Per-Task Quantitative Results, Cost Accounting &amp; Paired McNemar Significance</h2>
    <table>
      <thead>
        <tr>
          <th>Method</th>
          <th>Overall Held-Out</th>
          {t_headers}
          <th>Dev &amp; Eval Cost</th>
          <th>Mean / P95 Wall</th>
          <th>McNemar vs TRAX</th>
        </tr>
      </thead>
      <tbody>{''.join(rows)}</tbody>
    </table>
  </div>
</div>
</body>
</html>"""


def run_robosuite_parity_benchmark(
    *,
    llm_fn: LLMGenerateFn,
    output_dir: Path,
    tasks: Sequence[str] = DEFAULT_ROBOSUITE_TASKS,
    dev_seeds: Sequence[int] = DEFAULT_DEV_SEEDS,
    eval_seeds: Sequence[int] = DEFAULT_EVAL_SEEDS,
    model: str = "gpt-6-astra",
    reasoning_effort: str = "medium",
    max_dev_llm_calls: int = 4,
    enable_capx_online_regen: bool = True,
    horizon: int = 400,
    resume: bool = True,
    progress_cb: Callable[[str], None] | None = None,
) -> dict[str, Any]:
    """Run full Phase 3B Robosuite parity comparison across tasks, seeds, and protocols."""
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    pol_dir = out / "policies"
    pol_dir.mkdir(parents=True, exist_ok=True)
    report_path = out / "robosuite_prior_work_report.json"
    html_path = out / "robosuite_prior_work_report.html"
    started_at = _utc_now()
    method_ids = (
        "strong_zeroshot",
        "aspire_protocol",
        "capx_protocol",
        "simex_direct_repair",
        "simex_protocol",
        "craft_full",
    )

    def _log(msg: str) -> None:
        if progress_cb:
            progress_cb(msg)

    method_meta: dict[str, dict[str, Any]] = {}
    trials: list[dict[str, Any]] = []
    completed_tasks: set[str] = set()

    if resume and report_path.is_file():
        try:
            prev = json.loads(report_path.read_text(encoding="utf-8"))
            if (
                prev.get("benchmark_version") == ROBOSUITE_BENCH_VERSION
                and prev.get("model") == model
                and prev.get("reasoning_effort") == reasoning_effort
                and list(prev.get("dev_seeds") or []) == list(dev_seeds)
                and list(prev.get("eval_seeds") or []) == list(eval_seeds)
                and int(prev.get("max_dev_llm_calls") or 0) == int(max_dev_llm_calls)
            ):
                prev_mbt = prev.get("methods_by_task") or {}
                prev_trials = list(prev.get("trials") or [])
                for tk in tasks:
                    tk_meta = prev_mbt.get(tk) or {}
                    tk_trials = [t for t in prev_trials if t.get("task") == tk]
                    all_ok = all(
                        mid in tk_meta
                        and (pol_dir / f"{tk}_{mid}.py").is_file()
                        and sum(1 for t in tk_trials if t.get("method_id") == mid) == len(eval_seeds)
                        for mid in method_ids
                    )
                    if all_ok:
                        restored: dict[str, Any] = {}
                        for mid in method_ids:
                            art = dict(tk_meta[mid])
                            art["source"] = (pol_dir / f"{tk}_{mid}.py").read_text(encoding="utf-8")
                            restored[mid] = art
                        method_meta[tk] = restored
                        trials.extend(tk_trials)
                        completed_tasks.add(tk)
                        _log(f"[{tk}] Resumed completed task from checkpoint ({len(tk_trials)} trials)")
        except Exception:  # noqa: BLE001
            method_meta.clear()
            trials.clear()
            completed_tasks.clear()

    def _build_payload(*, finished: bool) -> dict[str, Any]:
        stats = summarize_robosuite_benchmark(tasks, eval_seeds, method_meta, trials)
        return {
            "benchmark_version": ROBOSUITE_BENCH_VERSION,
            "simulator_engine": "mujoco_robosuite",
            "evidence_source": "simulator",
            "model": model,
            "reasoning_effort": reasoning_effort,
            "tasks": list(tasks),
            "dev_seeds": list(dev_seeds),
            "eval_seeds": list(eval_seeds),
            "max_dev_llm_calls": max_dev_llm_calls,
            "enable_capx_online_regen": enable_capx_online_regen,
            "provenance": _git_provenance(),
            "started_at_utc": started_at,
            "updated_at_utc": _utc_now(),
            "finished_at_utc": _utc_now() if finished else None,
            "methods_by_task": {
                tk: {
                    mid: {k: v for k, v in art.items() if k != "source"}
                    for mid, art in arts.items()
                }
                for tk, arts in method_meta.items()
            },
            "trials": trials,
            "summary_by_method": stats["by_method"],
            "paired_mcnemar_craft_vs": stats["paired_mcnemar_craft_vs"],
        }

    skill_store = SkillExperienceStore(out / "robosuite_skills.json")
    simex_skill_store = SkillExperienceStore(out / "simex_toolbox_skills.json")
    contract_sha = digest(SHARED_ROBOSUITE_SDK_CONTRACT)
    for task in tasks:
        if task in completed_tasks:
            continue
        verified_hints = [
            f"{sk.title}: {sk.guidance} (measured_limits={json.dumps(sk.measured_limits)}"
            + (f", helpers={list(sk.entry_functions)}" if sk.entry_functions else "")
            + ")"
            for sk in skill_store.list_verified(sdk_contract_sha256=contract_sha)
            if sk.task_name != task
        ]
        simex_notes = [
            f"{sk.title}: {sk.guidance}"
            for sk in simex_skill_store.list_verified(sdk_contract_sha256=contract_sha)
            if sk.task_name != task
        ]
        arts = _develop_task_methods(
            task,
            llm_fn,
            dev_seeds,
            max_dev_llm_calls=max_dev_llm_calls,
            horizon=horizon,
            skill_hints=verified_hints,
            simex_toolbox_notes=simex_notes,
            log_cb=_log,
        )
        method_meta[task] = arts
        for mid, art in arts.items():
            (pol_dir / f"{task}_{mid}.py").write_text(str(art["source"]) + "\n", encoding="utf-8")

        # Evaluate all 6 frozen/online methods on disjoint held-out eval_seeds
        for mid in method_ids:
            art = arts[mid]
            online_cb = None
            if mid == "capx_protocol" and enable_capx_online_regen:
                sk = CapXSkillLibrary()
                sk.skills = dict((art.get("skill_library") or {}).get("skills") or {})

                def _make_regen(src_ref: str, sk_docs: str, t_name: str) -> Callable[[dict[str, Any]], Mapping[str, Any]]:
                    def _regen(fb: dict[str, Any]) -> Mapping[str, Any]:
                        p = _robo_capx_prompt(t_name, src_ref, fb, sk_docs, online_turn=True)
                        return llm_fn(p)
                    return _regen

                online_cb = _make_regen(str(art["source"]), sk.render_docs(), task)

            passed_cnt = 0
            for sd in eval_seeds:
                rec = run_robosuite_rollout(
                    task,
                    sd,
                    str(art["source"]),
                    horizon=horizon,
                    online_regen_fn=online_cb,
                )
                rec["method_id"] = mid
                trials.append(rec)
                if rec["success"]:
                    passed_cnt += 1
            if mid == "simex_protocol":
                simex_skill_store.add(
                    SkillExperience(
                        skill_id=f"simex-{task.lower()}-v3",
                        title=f"SimEX {task} Toolbox Entry",
                        guidance=f"Capability-bank screened policy for {task} ({passed_cnt}/{len(eval_seeds)} held-out).",
                        task_name=task,
                        required_methods=("get_state", "move_to", "move_delta", "set_gripper", "step_raw"),
                        sdk_contract_sha256=contract_sha,
                        evidence_source="simulator",
                        verification_report_id=f"simex-v3-{task}",
                        verified=passed_cnt >= max(1, int(0.75 * len(eval_seeds))),
                        verified_at_utc=_utc_now(),
                        code_snippet=_extract_verified_helper_snippet(str(art["source"])),
                    )
                )
            if mid == "craft_full":
                skill_store.add(
                    SkillExperience(
                        skill_id=f"robosuite-{task.lower()}-v3",
                        title=f"Robosuite {task} Verified Policy Skill",
                        guidance=f"ActiveProbe + RepairScreener frozen policy for {task} ({passed_cnt}/{len(eval_seeds)} held-out).",
                        task_name=task,
                        required_methods=("get_state", "move_to", "move_delta", "set_gripper", "step_raw"),
                        sdk_contract_sha256=contract_sha,
                        evidence_source="simulator",
                        verification_report_id=f"robosuite-v3-{task}",
                        verified=passed_cnt >= max(1, int(0.75 * len(eval_seeds))),
                        verified_at_utc=_utc_now(),
                        measured_limits={
                            "dev_pass_rate": round(art["dev_passed"] / max(1, art["dev_total"]), 4),
                            "heldout_pass_rate": round(passed_cnt / max(1, len(eval_seeds)), 4),
                            "dev_llm_calls": art["dev_llm_calls"],
                        },
                        known_failure_modes=tuple(art.get("known_failure_modes") or ()),
                        code_snippet=_extract_verified_helper_snippet(str(art["source"])),
                    )
                )
            _log(
                f"[{task}][EVAL] {mid:16s} -> {passed_cnt}/{len(eval_seeds)} "
                f"({passed_cnt / len(eval_seeds) * 100:.1f}%)"
            )
            _write_json_atomic(report_path, _build_payload(finished=False))

    final_payload = _build_payload(finished=True)
    _write_json_atomic(report_path, final_payload)
    html_path.write_text(render_robosuite_html(final_payload), encoding="utf-8")
    return final_payload
