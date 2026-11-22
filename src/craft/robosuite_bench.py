"""Phase 3B: Public continuous-manipulation benchmark parity on Robosuite (MuJoCo).

Evaluates CRAFT against prior-work protocols under matched model, shared 6-DoF
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
4. CRAFT (Full Engine) — AST preflight + 4-layer manipulation attribution +
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


ROBOSUITE_BENCH_VERSION = "CRAFT-Robosuite-Parity-v1.0"
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
        "Task: Robosuite `Lift` (Panda 7-DoF arm). Grasp the red cube (`state['objects']['cube']['pos']`, "
        "approx 4cm cube) and lift it vertically at least 6cm above the table (`z >= cube_z + 0.08`)."
    ),
    "PickPlaceCan": (
        "Task: Robosuite `PickPlaceCan` (Panda 7-DoF arm). Grasp the cylindrical red soda can "
        "(`state['objects']['can']['pos']`, approx 10cm tall) from the source bin, lift cleanly over "
        "the bin divider (`z >= 1.04`), move above `state['goal_pos']`, lower inside the target bin, and release."
    ),
    "NutAssemblySquare": (
        "Task: Robosuite `NutAssemblySquare` (Panda 7-DoF arm). Grasp the square nut (`state['objects']['nut']`) "
        "by its protruding handle (NOTE: the square nut center `pos` is the HOLE center; the handle extends "
        "~0.055m along the nut's local axis determined by `yaw`, so grasping the center hole misses the solid rim! "
        "Use `cos(yaw), sin(yaw)` to compute the handle grasp offset from `pos`, verify grasp width/lift, "
        "align the nut hole directly above `state['goal_pos']` — the square peg — adjusting for the handle-to-hole "
        "offset and matching peg yaw, lower straight down onto the peg, and release)."
    ),
    "Door": (
        "Task: Robosuite `Door` (Panda 7-DoF arm). Open the latched door (`state['objects']['handle']['pos']`, "
        "`state['objects']['door']['pos']`, `state['handle_qpos']`, `state['hinge_qpos']`). "
        "To open the door, the latch handle must be rotated/pushed downward to unlatch (`handle_qpos` increases) "
        "while pulling the door open (`hinge_qpos > 0.30`). Approach the handle from the front (`-y` side), "
        "hook or grasp the handle, press downward (`-z`) while pulling backward (`-y` / `-x` along the door swing arc)."
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
# Four-Layer Continuous-Manipulation Failure Attribution (CRAFT)
# ---------------------------------------------------------------------------

def attribute_robosuite_failure(rollout: Mapping[str, Any], task_name: str) -> dict[str, str]:
    """Classify a failed Robosuite rollout into a structured 4-layer causal diagnosis."""
    if rollout.get("success"):
        return {"layer": "none", "code": "passed", "summary": "Rollout succeeded", "prescription": "None"}
    err = str(rollout.get("error") or "")
    if err:
        return {
            "layer": "policy",
            "code": "sandbox_or_runtime_exception",
            "summary": f"Policy raised exception or failed AST sandbox: {err}",
            "prescription": "Fix AST/syntax/runtime error without using forbidden constructs.",
        }

    min_eef_obj = float(rollout.get("min_eef_to_obj_m", 9.99))
    lift_delta = float(rollout.get("obj_lift_delta_m", 0.0))
    min_obj_goal = float(rollout.get("min_obj_to_goal_m", 9.99))
    ever_closed = bool(rollout.get("ever_closed_gripper", False))
    closed_w_max = float(rollout.get("closed_gripper_width_max", 0.0))

    if task_name == "Door":
        max_handle = float(rollout.get("max_handle_qpos", 0.0))
        max_hinge = float(rollout.get("max_hinge_qpos", 0.0))
        if min_eef_obj > 0.08:
            return {
                "layer": "control",
                "code": "handle_approach_missed",
                "summary": f"EEF never approached door handle closely (min_dist={min_eef_obj:.3f}m).",
                "prescription": "Approach handle_pos cleanly from front (-y offset ~0.06m) before engaging handle.",
            }
        if max_handle < 0.15:
            return {
                "layer": "control",
                "code": "latch_not_unlatched",
                "summary": f"Reached handle (min_dist={min_eef_obj:.3f}m) but handle_qpos stayed {max_handle:.3f} (latch stayed locked).",
                "prescription": "Engage the door handle and push firmly downward (-z) to rotate the latch (increase handle_qpos) BEFORE pulling backward.",
            }
        return {
            "layer": "control",
            "code": "door_pull_arc_incomplete",
            "summary": f"Unlatched handle (max_handle_qpos={max_handle:.3f}) but hinge opened only to {max_hinge:.3f} rad.",
            "prescription": "After pressing handle down to unlatch, pull smoothly backward (-y and slightly -x) to swing hinge past 0.35 rad.",
        }

    if min_eef_obj > 0.08:
        return {
            "layer": "control",
            "code": "approach_missed_object",
            "summary": f"EEF never reached target object (min_eef_to_obj={min_eef_obj:.3f}m).",
            "prescription": "Read live object pos from sdk.get_state(), approach directly above, and descend accurately to grasp height.",
        }
    if (not ever_closed) or lift_delta < 0.04:
        return {
            "layer": "control",
            "code": "grasp_missed_or_slip",
            "summary": f"Reached object vicinity (min_dist={min_eef_obj:.3f}m) but failed to lift (lift_delta={lift_delta:.3f}m, closed_width={closed_w_max:.4f}m).",
            "prescription": (
                "Compute exact grasp point (for NutAssemblySquare, grasp the HANDLE offset ~0.055m from nut hole center using cos(yaw)/sin(yaw), "
                "orient gripper perpendicular to handle, descend to exact object z, close gripper for >=18 steps, then verify lift)."
                if task_name == "NutAssemblySquare"
                else "Ensure gripper is fully open before descending to exact object center z, close gripper for >=18 steps, and lift vertically."
            ),
        }
    if min_obj_goal > 0.06:
        return {
            "layer": "control",
            "code": "transport_or_goal_alignment_error",
            "summary": f"Object lifted (lift_delta={lift_delta:.3f}m) but did not align above goal (min_obj_to_goal={min_obj_goal:.3f}m).",
            "prescription": (
                "Lift high enough to clear obstacles/peg (z >= 1.05), transport the NUT HOLE (compensating for the handle grasp offset!) "
                "directly above goal_pos, lower vertically onto peg, then open gripper."
                if task_name == "NutAssemblySquare"
                else "Lift over bin divider (z >= 1.06), move directly over goal_pos, descend gently, and open gripper."
            ),
        }
    return {
        "layer": "control",
        "code": "insertion_or_release_precision_error",
        "summary": f"Object reached goal vicinity (min_obj_to_goal={min_obj_goal:.3f}m) and lifted ({lift_delta:.3f}m), but final placement/insertion did not satisfy task success.",
        "prescription": "Align yaw and xy tightly (tol<=0.008m) above goal_pos, descend slowly straight down before releasing gripper, and verify final state.",
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


def _execute_source_on_sdk(source: str, sdk: RobosuitePolicySDK) -> tuple[bool, str]:
    chk = check_robosuite_ast_compliance(source)
    if not chk["compliant"]:
        return False, "AST sandbox rejected: " + "; ".join(chk["issues"][:3])
    ns = dict(_ROBO_SAFE_GLOBALS)
    try:
        exec(compile(source, "<craft-robosuite-policy>", "exec"), ns, ns)  # noqa: S102
        fn = ns.get("run_policy")
        if not callable(fn):
            return False, "Missing callable run_policy(sdk)"
        fn(sdk)
        return True, ""
    except Exception as exc:  # noqa: BLE001
        return False, f"{type(exc).__name__}: {exc}"


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
        ok_exec, err_msg = _execute_source_on_sdk(source, sdk)
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
                ok_exec2, err_msg2 = _execute_source_on_sdk(repaired, sdk)
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


def _robo_craft_prompt(
    task_name: str,
    prev_src: str,
    dev_runs: Sequence[Mapping[str, Any]],
    static_issues: Sequence[str],
    reg_seeds: Sequence[int],
    it: int,
) -> str:
    diag_lines = []
    for r in dev_runs:
        if r["success"]:
            diag_lines.append(f"- seed={r['seed']}: PASS (steps={r['steps_used']})")
        else:
            a = r["attribution"]
            diag_lines.append(
                f"- seed={r['seed']}: FAIL [{a['layer']}:{a['code']}] {a['summary']} "
                f"| Prescription: {a['prescription']} "
                f"| metrics(min_eef_obj={r['min_eef_to_obj_m']}, lift_delta={r['obj_lift_delta_m']}, "
                f"min_obj_goal={r['min_obj_to_goal_m']}, max_handle={r['max_handle_qpos']}, max_hinge={r['max_hinge_qpos']})"
            )
    return "\n".join(
        [
            f"CRAFT Structured Evolution for `{task_name}` (iteration {it}):",
            "Resolve the four-layer attributed control/policy defects below while keeping ALL passing regression seeds green.",
            "",
            ROBOSUITE_TASK_DESCRIPTIONS[task_name],
            "",
            SHARED_ROBOSUITE_SDK_CONTRACT,
            "",
            "Previous Candidate Code:",
            "```python",
            prev_src,
            "```",
            "",
            "Static Preflight Gate (AST Sandbox):",
            *(static_issues if static_issues else ["  PASS"]),
            "",
            "Four-Layer Manipulation Failure Attribution:",
            *diag_lines,
            "",
            f"Active Permanent Regression Seeds (must all pass): {list(reg_seeds)}",
            "",
            "Return ONLY the complete revised Python code defining `run_policy(sdk)`.",
        ]
    )


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
    log_cb: Callable[[str], None] | None = None,
) -> dict[str, dict[str, Any]]:
    def _log(msg: str) -> None:
        if log_cb:
            log_cb(msg)

    artifacts: dict[str, dict[str, Any]] = {}

    # 1. Strong Zero-Shot (1 call, 0 dev rollouts)
    _log(f"[{task_name}] Dev 1/4: Strong Zero-Shot...")
    t0 = time.monotonic()
    g_zs = llm_fn(_robo_initial_prompt(task_name))
    src_zs = str(g_zs["source"])
    artifacts["strong_zeroshot"] = {
        "method_id": "strong_zeroshot",
        "label": "Strong Zero-Shot",
        "provenance": "baseline_control",
        "online_mode": "frozen",
        "source": src_zs,
        "sha256": digest(src_zs),
        "dev_llm_calls": 1,
        "dev_sim_resets": 0,
        "dev_wall_s": round(time.monotonic() - t0, 3),
        "dev_passed": 0,
        "dev_total": 0,
        "history": [],
    }

    # 2. ASPIRE-Protocol (iterative raw-trace debugging on dev_seeds)
    _log(f"[{task_name}] Dev 2/4: ASPIRE-Protocol (dev seeds {list(dev_seeds)})...")
    t0 = time.monotonic()
    best_src, best_pass, resets, calls = "", -1, 0, 0
    hist: list[dict[str, Any]] = []
    cur_src, last_runs = "", []
    for it in range(1, max_dev_llm_calls + 1):
        prompt = _robo_initial_prompt(task_name) if it == 1 else _robo_aspire_prompt(task_name, cur_src, last_runs, it)
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
        "dev_wall_s": round(time.monotonic() - t0, 3),
        "dev_passed": max(0, best_pass),
        "dev_total": len(dev_seeds),
        "history": hist,
    }

    # 3. CaP-X-Protocol (execution feedback + SkillLibrary + online regen)
    _log(f"[{task_name}] Dev 3/4: CaP-X-Protocol (dev seeds {list(dev_seeds)})...")
    t0 = time.monotonic()
    skill_lib = CapXSkillLibrary()
    best_src, best_pass, resets, calls = "", -1, 0, 0
    hist = []
    cur_src, last_fail = "", {}
    for it in range(1, max_dev_llm_calls + 1):
        prompt = (
            _robo_initial_prompt(
                task_name,
                "Structure reusable geometric/control helper functions (names NOT starting with `_`) for skill library extraction.",
            )
            if it == 1
            else _robo_capx_prompt(task_name, cur_src, last_fail, skill_lib.render_docs(), online_turn=False)
        )
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
        "dev_wall_s": round(time.monotonic() - t0, 3),
        "dev_passed": max(0, best_pass),
        "dev_total": len(dev_seeds),
        "history": hist,
    }

    # 4. CRAFT Full Engine (Static AST preflight + 4-layer attribution + Regression Gate)
    _log(f"[{task_name}] Dev 4/4: CRAFT Full Engine (dev seeds {list(dev_seeds)})...")
    t0 = time.monotonic()
    best_src, best_pass, resets, calls = "", -1, 0, 0
    hist = []
    reg_seeds: set[int] = set()
    cur_src, last_runs, last_static = "", [], []
    for it in range(1, max_dev_llm_calls + 1):
        prompt = (
            _robo_initial_prompt(task_name)
            if it == 1
            else _robo_craft_prompt(task_name, cur_src, last_runs, last_static, sorted(reg_seeds), it)
        )
        gen = llm_fn(prompt)
        calls += 1
        cur_src = str(gen["source"])
        ast_chk = check_robosuite_ast_compliance(cur_src)
        if not ast_chk["compliant"]:
            last_static = list(ast_chk["issues"])
            last_runs = []
            hist.append({"iter": it, "static_preflight_passed": False, "issues": last_static, "dev_passed": 0})
            _log(f"  [{task_name}][CRAFT it={it}] AST preflight blocked (0 sim resets wasted)")
            if best_pass < 0:
                best_src = cur_src
            continue
        last_static = []
        runs = [run_robosuite_rollout(task_name, s, cur_src, horizon=horizon) for s in dev_seeds]
        resets += len(dev_seeds)
        last_runs = runs
        passed = sum(1 for r in runs if r["success"])
        for r in runs:
            if not r["success"]:
                reg_seeds.add(int(r["seed"]))
        hist.append(
            {
                "iter": it,
                "static_preflight_passed": True,
                "dev_passed": passed,
                "dev_total": len(dev_seeds),
                "attributions": [r["attribution"]["code"] for r in runs if not r["success"]],
                "regression_seeds": sorted(reg_seeds),
                "sha": digest(cur_src)[:12],
            }
        )
        _log(f"  [{task_name}][CRAFT it={it}] dev={passed}/{len(dev_seeds)} attr={[r['attribution']['code'] for r in runs if not r['success']]}")
        if passed >= best_pass:
            best_pass, best_src = passed, cur_src
        if passed == len(dev_seeds):
            break
    artifacts["craft_full"] = {
        "method_id": "craft_full",
        "label": "CRAFT (Full Engine)",
        "provenance": "craft_proposed_system",
        "online_mode": "frozen",
        "source": best_src,
        "sha256": digest(best_src),
        "dev_llm_calls": calls,
        "dev_sim_resets": resets,
        "dev_wall_s": round(time.monotonic() - t0, 3),
        "dev_passed": max(0, best_pass),
        "dev_total": len(dev_seeds),
        "history": hist,
    }
    return artifacts


def summarize_robosuite_benchmark(
    tasks: Sequence[str],
    eval_seeds: Sequence[int],
    method_meta: Mapping[str, Mapping[str, Any]],
    trials: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    method_ids = ("strong_zeroshot", "aspire_protocol", "capx_protocol", "craft_full")
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

    # Paired exact McNemar across all (task, seed) pairs: CRAFT vs each baseline
    keys = [(tk, sd) for tk in tasks for sd in eval_seeds]
    craft_map = {(t["task"], int(t["seed"])): bool(t["success"]) for t in trials if t["method_id"] == "craft_full"}
    paired: dict[str, dict[str, Any]] = {}
    if len(craft_map) == len(keys):
        c_vec = [craft_map[k] for k in keys]
        for mid in ("strong_zeroshot", "aspire_protocol", "capx_protocol"):
            b_map = {(t["task"], int(t["seed"])): bool(t["success"]) for t in trials if t["method_id"] == mid}
            if len(b_map) == len(keys):
                paired[mid] = mcnemar_exact_p(c_vec, [b_map[k] for k in keys])

    return {"by_method": summary, "paired_mcnemar_craft_vs": paired}


def render_robosuite_html(report: Mapping[str, Any]) -> str:
    by_method = list((report.get("summary_by_method") or {}).values())
    tasks = list(report.get("tasks") or [])
    mcnemar = report.get("paired_mcnemar_craft_vs") or {}
    colors = ["#94a3b8", "#f59e0b", "#a78bfa", "#22c55e"]

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
          <th>McNemar vs CRAFT</th>
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

    def _log(msg: str) -> None:
        if progress_cb:
            progress_cb(msg)

    method_meta: dict[str, dict[str, Any]] = {}
    trials: list[dict[str, Any]] = []

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

    for task in tasks:
        arts = _develop_task_methods(
            task,
            llm_fn,
            dev_seeds,
            max_dev_llm_calls=max_dev_llm_calls,
            horizon=horizon,
            log_cb=_log,
        )
        method_meta[task] = arts
        for mid, art in arts.items():
            (pol_dir / f"{task}_{mid}.py").write_text(str(art["source"]) + "\n", encoding="utf-8")

        # Evaluate all 4 frozen/online methods on disjoint held-out eval_seeds
        for mid in ("strong_zeroshot", "aspire_protocol", "capx_protocol", "craft_full"):
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
            _log(
                f"[{task}][EVAL] {mid:16s} -> {passed_cnt}/{len(eval_seeds)} "
                f"({passed_cnt / len(eval_seeds) * 100:.1f}%)"
            )
            _write_json_atomic(report_path, _build_payload(finished=False))

    final_payload = _build_payload(finished=True)
    _write_json_atomic(report_path, final_payload)
    html_path.write_text(render_robosuite_html(final_payload), encoding="utf-8")
    return final_payload
