"""Public LIBERO Benchmark Integration for TRAX (optional simulator module).

Evaluates TRAX head-to-head against matched-budget coding-agent protocols on
official LIBERO suites (``libero_spatial``, ``libero_object``, ``libero_goal``,
``libero_10``) using strictly disjoint official ``init_states`` splits:
- Dev split: ``init_states[0:dev_count]`` (default 5)
- Held-out Eval split: ``init_states[dev_count : dev_count + eval_count]`` (default 10 or 20)

Honest observation-contract provenance:
- Uses a shared 6-DoF operational-space host SDK (``LiberoPolicySDK``) exposing
  host-owned 3D object/fixture/site poses (analogous to ``EvidenceScene`` /
  state-conditioned CaP-X & ASPIRE setups).
- Explicitly tagged ``observation_contract="host_state_api"`` and
  ``evidence_source="simulator"`` — NOT directly rank-comparable to RGB-D-only
  agents (such as RAX) without matched perception pipelines.
"""
from __future__ import annotations

import argparse
import ast
from datetime import datetime, timezone
import html
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any, Callable, Mapping, Sequence

if not os.environ.get("MUJOCO_GL") and not os.environ.get("DISPLAY"):
    os.environ["MUJOCO_GL"] = "egl"

from trax.benchmark import percentile, wilson_ci_95
from .cafebench import _git_provenance, _write_json_atomic
from trax.contracts import digest
from .prior_work_bench import LLMGenerateFn, mcnemar_exact_p
from trax.probing import ProbeReport, analyze_policy_subroutines, parse_probe_telemetry
from .robosuite_bench import (
    _ROBO_SAFE_GLOBALS,
    _extract_verified_helper_snippet,
    _quat_to_yaw,
    _wrap_angle,
    check_robosuite_ast_compliance,
)
from trax.skills import SkillExperience, SkillExperienceStore, validate_skill_code_snippet


LIBERO_BENCH_VERSION = "TRAX-LIBERO-v1.1"


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


SHARED_LIBERO_SDK_CONTRACT = """LIBERO 6-DoF Manipulation Host SDK Contract (identical for all evaluated methods):
1. Entry point: define `def run_policy(sdk):` returning `{"status": "done" | "failed", ...}`.
2. Safe AST Sandbox Rules:
   - NO `import`, `try`/`except`, `with`, `class`, `lambda`, `global`, `nonlocal`, `del`, or `_`-prefixed names/attributes (use `def` helpers instead of `lambda`).
   - Pre-injected globals: `sin`, `cos`, `tan`, `atan2`, `sqrt`, `hypot`, `fabs`, `pi`, `min`, `max`, `abs`, `round`, `range`, `len`, `sum`, `list`, `dict`, `tuple`, `float`, `int`, `bool`, `enumerate`, `sorted`, `all`, `any`.
   - Top-level helper functions (names NOT starting with `_`) are allowed.
3. Available `sdk` methods (official LIBERO Panda OSC_POSE, 20 Hz, max horizon 450 steps):
   - `sdk.get_state()` -> dict:
     `{
       "task_language": str,
       "eef_pos": [x, y, z],
       "eef_yaw": float (radians),
       "gripper_width": float (meters; ~0.078 open, <0.01 empty closed),
       "objects": {name: {"pos": [x, y, z], "yaw": float}},
       "fixtures": {name: {"pos": [x, y, z], "yaw": float}},
       "sites": {site_name: [x, y, z]},   # key named regions/handles/surfaces exposed by host scene
       "steps_used": int,
       "max_steps": int,
     }`
   - `sdk.move_to(pos, yaw=None, gripper=None, max_steps=70, tol=0.015)` -> `{"reached": bool, "pos_err_m": float, "eef_pos": [x,y,z], "steps": int}`
   - `sdk.move_delta(dpos, dyaw=0.0, gripper=None, steps=20)` -> `{"eef_pos": [x,y,z], "steps": int}`
   - `sdk.set_gripper(action, steps=15)` -> `{"gripper_width": float, "grasped_hint": bool}` (`"open"` or `"close"`)
   - `sdk.step_raw(action_7d, steps=1)` -> `{"eef_pos": [x,y,z], "steps_used": int}` (`[dx,dy,dz,drx,dry,drz,grip]`, `grip=-1` open, `+1` close)
4. Generalization rule: NEVER hardcode object coordinates from one init state — always read live poses from `sdk.get_state()`.
5. Output ONLY valid Python code defining `run_policy(sdk)` (no prose outside optional code fence).
"""


class LiberoPolicySDK:
    """Shared host SDK over official LIBERO OffScreenRenderEnv."""

    def __init__(self, env: Any, task_language: str, *, max_steps: int = 450) -> None:
        self._env = env
        self._inner = env.env
        self._task_language = task_language
        self._max_steps = int(max_steps)
        self._steps_used = 0
        self._grip_cmd = -1.0  # -1 open, +1 close
        self._last_obs: dict[str, Any] = {}
        self.trace: list[dict[str, Any]] = []
        self.ever_closed_gripper = False
        self.closed_gripper_width_max = 0.0
        self.init_obj_positions: dict[str, list[float]] = {}
        self.max_obj_z_delta: dict[str, float] = {}
        self.min_eef_to_any_obj_m = 9.99

    def bind_initial_obs(self, obs: Mapping[str, Any]) -> None:
        self._last_obs = dict(obs)
        st = self._build_state()
        for k, v in (st.get("objects") or {}).items():
            self.init_obj_positions[k] = list(v["pos"])
            self.max_obj_z_delta[k] = 0.0

    def _sim_body_pose(self, body_name: str) -> dict[str, Any] | None:
        sim = self._inner.sim
        for cand in (body_name, f"{body_name}_main"):
            try:
                bid = sim.model.body_name2id(cand)
                pos = [round(float(v), 5) for v in sim.data.body_xpos[bid]]
                xquat = sim.data.body_xquat[bid]  # wxyz in mujoco
                xyzw = [float(xquat[1]), float(xquat[2]), float(xquat[3]), float(xquat[0])]
                return {"pos": pos, "yaw": round(_quat_to_yaw(xyzw), 4)}
            except Exception:  # noqa: BLE001
                continue
        return None

    def _build_state(self) -> dict[str, Any]:
        obs = self._last_obs
        eef_pos = [round(float(x), 5) for x in obs["robot0_eef_pos"]]
        eef_quat = [float(x) for x in obs["robot0_eef_quat"]]
        eef_yaw = round(_quat_to_yaw(eef_quat), 4)
        gq = obs.get("robot0_gripper_qpos")
        grip_w = round(float(abs(gq[0] - gq[1])), 5) if gq is not None and len(gq) >= 2 else 0.04

        objects: dict[str, dict[str, Any]] = {}
        for name in sorted((getattr(self._inner, "objects_dict", None) or {}).keys()):
            pose = self._sim_body_pose(name)
            if pose is not None:
                objects[name] = pose

        fixtures: dict[str, dict[str, Any]] = {}
        for name in sorted((getattr(self._inner, "fixtures_dict", None) or {}).keys()):
            pose = self._sim_body_pose(name)
            if pose is not None:
                fixtures[name] = pose

        sites: dict[str, list[float]] = {}
        sim = self._inner.sim
        for sid in range(sim.model.nsite):
            sname = sim.model.site_id2name(sid)
            if not sname or sname.startswith(("robot", "gripper")):
                continue
            if any(k in sname for k in ("region", "handle", "top", "button", "knob", "stove", "cabinet", "drawer", "rack", "plate", "basket", "tray", "bowl", "mug", "cook")):
                sites[sname] = [round(float(v), 5) for v in sim.data.site_xpos[sid]]

        return {
            "task_language": self._task_language,
            "eef_pos": eef_pos,
            "eef_yaw": eef_yaw,
            "gripper_width": grip_w,
            "objects": objects,
            "fixtures": fixtures,
            "sites": sites,
            "steps_used": self._steps_used,
            "max_steps": self._max_steps,
        }

    def _update_metrics(self) -> None:
        st = self._build_state()
        eef = st["eef_pos"]
        for k, v in st["objects"].items():
            p = v["pos"]
            d = math.sqrt((eef[0] - p[0]) ** 2 + (eef[1] - p[1]) ** 2 + (eef[2] - p[2]) ** 2)
            if d < self.min_eef_to_any_obj_m:
                self.min_eef_to_any_obj_m = d
            if k in self.init_obj_positions:
                dz = p[2] - self.init_obj_positions[k][2]
                if dz > self.max_obj_z_delta.get(k, 0.0):
                    self.max_obj_z_delta[k] = dz
        if self._grip_cmd > 0.0:
            self.ever_closed_gripper = True
            if st["gripper_width"] > self.closed_gripper_width_max:
                self.closed_gripper_width_max = st["gripper_width"]

    def _step_env(self, action_7d: Sequence[float]) -> None:
        if self._steps_used >= self._max_steps:
            return
        act = [max(-1.0, min(1.0, float(v))) for v in action_7d]
        obs, _rew, _done, _info = self._env.step(act)
        self._last_obs = dict(obs)
        self._steps_used += 1
        self._update_metrics()

    def get_state(self) -> dict[str, Any]:
        return self._build_state()

    def _parse_gripper_cmd(self, action: Any) -> float:
        if isinstance(action, (int, float)) and not isinstance(action, bool):
            return 1.0 if float(action) > 0.0 else -1.0
        if isinstance(action, bool):
            return 1.0 if action else -1.0
        return 1.0 if str(action).strip().lower() in {"close", "closed", "grasp", "1", "+1", "true"} else -1.0

    def set_gripper(self, action: str, steps: int = 15) -> dict[str, Any]:
        self._grip_cmd = self._parse_gripper_cmd(action)
        n = max(1, min(40, int(steps)))
        for _ in range(n):
            if self._steps_used >= self._max_steps:
                break
            self._step_env([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self._grip_cmd])
        st = self._build_state()
        self.trace.append({"op": "set_gripper", "action": action, "w": st["gripper_width"]})
        return {"gripper_width": st["gripper_width"], "grasped_hint": self._grip_cmd > 0 and st["gripper_width"] > 0.012}

    def move_to(
        self,
        pos: Sequence[float],
        yaw: float | None = None,
        gripper: str | None = None,
        max_steps: int = 70,
        tol: float = 0.015,
    ) -> dict[str, Any]:
        if gripper is not None:
            self._grip_cmd = self._parse_gripper_cmd(gripper)
        tx, ty, tz = float(pos[0]), float(pos[1]), float(pos[2])
        lim = max(1, min(160, int(max_steps)))
        tol_m = max(0.003, float(tol))
        used = 0
        reached = False
        err = 9.99
        for _ in range(lim):
            if self._steps_used >= self._max_steps:
                break
            st = self._build_state()
            ex, ey, ez = st["eef_pos"]
            dx, dy, dz = tx - ex, ty - ey, tz - ez
            err = math.sqrt(dx * dx + dy * dy + dz * dz)
            yaw_err = _wrap_angle(float(yaw) - st["eef_yaw"]) if yaw is not None else 0.0
            if err <= tol_m and abs(yaw_err) <= 0.14:
                reached = True
                break
            vx = max(-1.0, min(1.0, dx * 7.5))
            vy = max(-1.0, min(1.0, dy * 7.5))
            vz = max(-1.0, min(1.0, dz * 7.5))
            vrz = max(-1.0, min(1.0, yaw_err * 2.5)) if yaw is not None else 0.0
            self._step_env([vx, vy, vz, 0.0, 0.0, vrz, self._grip_cmd])
            used += 1
        st = self._build_state()
        self.trace.append({"op": "move_to", "target": [round(tx, 3), round(ty, 3), round(tz, 3)], "reached": reached, "err": round(err, 4), "steps": used})
        return {"reached": reached, "pos_err_m": round(err, 5), "eef_pos": st["eef_pos"], "steps": used}

    def move_delta(
        self,
        dpos: Sequence[float],
        dyaw: float = 0.0,
        gripper: str | None = None,
        steps: int = 20,
    ) -> dict[str, Any]:
        st0 = self._build_state()
        tgt = [st0["eef_pos"][i] + float(dpos[i]) for i in range(3)]
        tgt_yaw = (st0["eef_yaw"] + float(dyaw)) if abs(float(dyaw)) > 1e-4 else None
        return self.move_to(tgt, yaw=tgt_yaw, gripper=gripper, max_steps=max(5, int(steps)), tol=0.012)

    def step_raw(self, action_7d: Sequence[float], steps: int = 1) -> dict[str, Any]:
        vals = [float(x) for x in action_7d]
        if len(vals) != 7:
            raise ValueError("step_raw requires 7 floats")
        self._grip_cmd = 1.0 if vals[6] > 0 else -1.0
        n = max(1, min(60, int(steps)))
        for _ in range(n):
            if self._steps_used >= self._max_steps:
                break
            self._step_env(vals)
        st = self._build_state()
        self.trace.append({"op": "step_raw", "steps": n, "eef": [round(x, 3) for x in st["eef_pos"]]})
        return {"eef_pos": st["eef_pos"], "steps_used": self._steps_used}


def _load_libero_task(suite_name: str, task_id: int) -> tuple[str, str, Any]:
    import torch
    from libero.libero import benchmark, get_libero_path

    bdict = benchmark.get_benchmark_dict()
    suite = bdict[suite_name]()
    task = suite.get_task(task_id)
    bddl = os.path.join(get_libero_path("bddl_files"), task.problem_folder, task.bddl_file)
    init_path = os.path.join(get_libero_path("init_states"), task.problem_folder, task.init_states_file)
    init_states = torch.load(init_path, weights_only=False)
    return str(task.language), bddl, init_states


def _execute_libero_source(source: str, sdk: LiberoPolicySDK, timeout_s: int = 25) -> tuple[bool, str, str, Any]:
    import signal

    chk = check_robosuite_ast_compliance(source)
    if not chk["compliant"]:
        return False, "AST sandbox rejected: " + "; ".join(chk["issues"][:3]), "", None
    ns = dict(_ROBO_SAFE_GLOBALS)

    def _on_timeout(_signum: int, _frame: Any) -> None:
        raise TimeoutError(f"LIBERO policy exceeded {timeout_s}s rollout timeout (possible infinite loop)")

    use_alarm = hasattr(signal, "SIGALRM") and timeout_s > 0
    prev_handler = None
    try:
        exec(compile(source, "<trax-libero-policy>", "exec"), ns, ns)  # noqa: S102
        fn = ns.get("run_policy")
        if not callable(fn):
            return False, "Missing callable run_policy(sdk)", "", None
        if use_alarm:
            prev_handler = signal.signal(signal.SIGALRM, _on_timeout)
            signal.alarm(int(timeout_s))
        ret = fn(sdk)
        return True, "", (repr(ret)[:180] if ret is not None else ""), ret
    except Exception as exc:  # noqa: BLE001
        return False, f"{type(exc).__name__}: {exc}", "", None
    finally:
        if use_alarm:
            signal.alarm(0)
            if prev_handler is not None:
                signal.signal(signal.SIGALRM, prev_handler)


def _reset_and_settle_libero_env(env: Any, init_states: Any, init_idx: int, settle_steps: int = 10) -> Any:
    """Reset LIBERO env to official init_state and run standard 10-step object settling."""
    env.seed(int(init_idx))
    env.reset()
    env.set_init_state(init_states[int(init_idx) % len(init_states)])
    obs = None
    for _ in range(max(1, int(settle_steps))):
        obs, _, _, _ = env.step([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0])
    return obs


def _build_libero_grasp_probe_source(target_obj: str) -> str:
    """Task-agnostic public-SDK free-space radial + jaw-alignment + lateral-stability grasp probe."""
    return f"""def run_policy(sdk):
    target = {target_obj!r}
    samples = []
    st0 = sdk.get_state()
    if target not in st0["objects"]:
        return {{"status": "done", "probe": "grasp_offset_scan", "frame": "world_free_space_radial", "target": target, "samples": []}}
    tpos = st0["objects"][target]["pos"]
    eef0_yaw = st0["eef_yaw"]
    obstacles = []
    for oname, oinfo in st0["objects"].items():
        if oname != target:
            obstacles.append(oinfo["pos"])
    for fname, finfo in (st0.get("fixtures") or {{}}).items():
        if abs(finfo["pos"][2] - tpos[2]) < 0.12:
            obstacles.append(finfo["pos"])

    best_ang = 0.0
    best_clr = -1.0
    for k in range(8):
        ang = k * (pi / 4.0)
        px = tpos[0] + 0.040 * cos(ang)
        py = tpos[1] + 0.040 * sin(ang)
        clr = 9.99
        for op in obstacles:
            d = hypot(px - op[0], py - op[1])
            if d < clr:
                clr = d
        if clr > best_clr:
            best_clr = clr
            best_ang = ang

    def score_sample(rec):
        if rec is None or not rec.get("retained"):
            return -9.0
        lift = rec["lift_delta_m"]
        width = rec["closed_width_m"]
        if lift >= 0.03 and width < 0.007:
            return -9.0
        if lift < 0.03:
            return lift
        return min(lift, 0.06) + 0.25 * max(0.0, 0.06 - width)

    def nearest_jaw_yaw(desired, cur_yaw):
        return cur_yaw + 0.5 * atan2(sin(2.0 * (desired - cur_yaw)), cos(2.0 * (desired - cur_yaw)))

    def run_sample(r, dz, ang, desired_yaw, jaw_rel, transit_steps=26):
        st = sdk.get_state()
        if st["steps_used"] > st["max_steps"] - 70 or target not in st["objects"]:
            return None
        o = st["objects"][target]
        pos = o["pos"]
        z0 = pos[2]
        gx = pos[0] + r * cos(ang)
        gy = pos[1] + r * sin(ang)
        yaw = nearest_jaw_yaw(desired_yaw, st["eef_yaw"])
        clr = 9.99
        for oname, oinfo in st["objects"].items():
            if oname != target:
                d = hypot(gx - oinfo["pos"][0], gy - oinfo["pos"][1])
                if d < clr:
                    clr = d
        sdk.set_gripper("open", steps=4)
        sdk.move_to([gx, gy, z0 + 0.11], yaw=yaw, gripper="open", max_steps=transit_steps, tol=0.014)
        sdk.move_to([gx, gy, z0 + dz], yaw=yaw, gripper="open", max_steps=22, tol=0.010)
        g = sdk.set_gripper("close", steps=12)
        sdk.move_to([gx, gy, z0 + 0.11], yaw=yaw, gripper="close", max_steps=18, tol=0.015)
        st2 = sdk.get_state()
        p2 = st2["objects"][target]["pos"]
        eef2 = st2["eef_pos"]
        z1 = p2[2]
        eef_obj_xy = hypot(p2[0] - eef2[0], p2[1] - eef2[1])
        eef_obj_z = abs(p2[2] - eef2[2])
        retained = (z1 - z0 >= 0.03) and (g["gripper_width"] >= 0.007) and (eef_obj_xy <= 0.085) and (eef_obj_z <= 0.085)
        rec = {{
            "target": target,
            "local_offset_m": r,
            "z_offset_m": dz,
            "approach_angle_rad": round(ang, 3),
            "gripper_yaw_rad": round(yaw, 3),
            "jaw_rel_angle_rad": round(jaw_rel, 3),
            "clearance_m": round(clr, 4),
            "closed_width_m": round(g["gripper_width"], 4),
            "lift_delta_m": round(z1 - z0, 4),
            "eef_obj_xy_m": round(eef_obj_xy, 4),
            "retained": retained,
        }}
        samples.append(rec)
        # Gently set object back down before releasing so next sample starts cleanly
        if z1 - z0 > 0.02:
            sdk.move_to([gx, gy, z0 + dz + 0.005], yaw=yaw, gripper="close", max_steps=14, tol=0.015)
        sdk.set_gripper("open", steps=4)
        return rec

    # Balanced object-agnostic scan: radial rim (both jaw axes) + center (both orthogonal jaw axes)
    s_perp = run_sample(0.040, 0.020, best_ang, best_ang + pi / 2.0, pi / 2.0, transit_steps=42)
    s_par = run_sample(0.040, 0.020, best_ang, best_ang, 0.0, transit_steps=20)
    best_jaw = pi / 2.0 if score_sample(s_perp) >= score_sample(s_par) else 0.0
    run_sample(0.0, 0.0, 0.0, eef0_yaw, 0.0, transit_steps=20)
    run_sample(0.0, 0.020, 0.0, eef0_yaw + pi / 2.0, pi / 2.0, transit_steps=20)
    opp_ang = best_ang + pi
    run_sample(0.040, 0.020, opp_ang, opp_ang + best_jaw, best_jaw, transit_steps=22)
    return {{
        "status": "done",
        "probe": "grasp_offset_scan",
        "frame": "world_free_space_radial",
        "target": target,
        "samples": samples,
    }}
"""


def _select_probe_target(task_lang: str, sample_state: Mapping[str, Any]) -> str | None:
    objs = sorted((sample_state.get("objects") or {}).keys())
    if not objs:
        return None
    low = task_lang.lower()
    # Pick first object whose stem tokens appear earliest in the instruction
    best_pos = 10**9
    best_obj = objs[0]
    for o in objs:
        tokens = [t for t in o.lower().split("_") if len(t) >= 3 and not t.isdigit()]
        for tok in tokens:
            idx = low.find(tok)
            if 0 <= idx < best_pos:
                best_pos = idx
                best_obj = o
    return best_obj


def run_libero_active_probe(
    env: Any,
    task_lang: str,
    init_states: Any,
    init_idx: int,
    sample_state: Mapping[str, Any],
    *,
    horizon: int = 450,
) -> ProbeReport | None:
    target = _select_probe_target(task_lang, sample_state)
    if not target:
        return None
    t0 = time.monotonic()
    obs = _reset_and_settle_libero_env(env, init_states, int(init_idx))
    sdk = LiberoPolicySDK(env, task_lang, max_steps=horizon)
    sdk.bind_initial_obs(obs)
    ok, _err, _rep, raw = _execute_libero_source(_build_libero_grasp_probe_source(target), sdk)
    if not ok or not isinstance(raw, Mapping):
        return None
    return parse_probe_telemetry(
        target,
        [raw],
        evidence_source="simulator",
        sim_resets_used=1,
        wall_s=time.monotonic() - t0,
    )


def attribute_libero_failure(rollout: Mapping[str, Any]) -> dict[str, str]:
    """Zero-leakage factual 4-layer attribution for LIBERO manipulation rollouts."""
    if rollout.get("success"):
        return {"layer": "none", "code": "passed", "summary": "Rollout succeeded", "prescription": "None"}
    err = str(rollout.get("error") or "")
    if err:
        return {
            "layer": "policy",
            "code": "sandbox_or_runtime_exception",
            "summary": f"Policy failed AST/runtime: {err}",
            "prescription": "Fix AST/runtime error using only allowed sandbox constructs and live state keys.",
        }
    min_d = float(rollout.get("min_eef_to_any_obj_m", 9.99))
    max_lift = float(rollout.get("max_any_obj_lift_m", 0.0))
    ever_closed = bool(rollout.get("ever_closed_gripper", False))
    steps = int(rollout.get("steps_used", 0))
    if min_d > 0.09:
        return {
            "layer": "control",
            "code": "approach_missed_object",
            "summary": f"EEF did not closely approach any task object (min_dist={min_d:.3f}m).",
            "prescription": "Inspect exact object/fixture/site keys in sdk.get_state() and approach the target accurately.",
        }
    if not ever_closed:
        return {
            "layer": "policy",
            "code": "never_closed_gripper_early_abort",
            "summary": f"Approached object (min_dist={min_d:.3f}m) but never closed gripper (steps={steps}).",
            "prescription": "Relax strict pre-grasp convergence guards so gripper closing/actuation runs once positioned.",
        }
    if max_lift < 0.05:
        return {
            "layer": "control",
            "code": "grasp_missed_or_slip",
            "summary": f"Closed gripper near object (min_dist={min_d:.3f}m) but max object lift was only {max_lift:.3f}m.",
            "prescription": "Verify target object grasp point/height, gripper opening prior to descent, and close duration.",
        }
    return {
        "layer": "control",
        "code": "insertion_or_release_precision_error",
        "summary": f"Object moved/lifted (max_lift={max_lift:.3f}m, steps={steps}) but goal predicate not satisfied.",
        "prescription": "Check target region/site/fixture pose in sdk.get_state(), ensure clearance during transport, align accurately over destination, and verify release height.",
    }


def run_libero_rollout(
    suite_name: str,
    task_id: int,
    init_idx: int,
    source: str,
    *,
    horizon: int = 450,
    cached_task: tuple[str, str, Any] | None = None,
    reuse_env: Any | None = None,
) -> dict[str, Any]:
    from libero.libero.envs import OffScreenRenderEnv

    t0 = time.monotonic()
    task_lang, bddl, init_states = cached_task or _load_libero_task(suite_name, task_id)
    owns_env = reuse_env is None
    env = OffScreenRenderEnv(bddl_file_name=bddl, camera_heights=64, camera_widths=64) if owns_env else reuse_env
    try:
        obs = _reset_and_settle_libero_env(env, init_states, int(init_idx))
        sdk = LiberoPolicySDK(env, task_lang, max_steps=horizon)
        sdk.bind_initial_obs(obs)
        ok_exec, err_msg, ret_str, _raw = _execute_libero_source(source, sdk)
        success = bool(env.check_success())
        final_st = sdk.get_state()
        max_lift = max(sdk.max_obj_z_delta.values(), default=0.0)
        rec = {
            "suite": suite_name,
            "task_id": task_id,
            "task_language": task_lang,
            "init_idx": int(init_idx),
            "success": success,
            "program_ok": ok_exec,
            "error": err_msg,
            "policy_return": ret_str,
            "steps_used": sdk._steps_used,
            "wall_s": round(time.monotonic() - t0, 3),
            "min_eef_to_any_obj_m": round(sdk.min_eef_to_any_obj_m, 4),
            "ever_closed_gripper": sdk.ever_closed_gripper,
            "closed_gripper_width_max": round(sdk.closed_gripper_width_max, 4),
            "max_any_obj_lift_m": round(max_lift, 4),
            "obj_lifts": {k: round(v, 4) for k, v in sdk.max_obj_z_delta.items()},
            "final_state": final_st,
            "trace_tail": sdk.trace[-8:],
        }
        rec["attribution"] = attribute_libero_failure(rec)
        return rec
    finally:
        if owns_env:
            env.close()


def _probe_initial_scene_schema(
    suite_name: str,
    task_id: int,
    init_idx: int,
    cached_task: tuple[str, str, Any],
    *,
    reuse_env: Any | None = None,
) -> dict[str, Any]:
    """Read-only initial observation schema from public sdk.get_state() (identical for ALL methods)."""
    from libero.libero.envs import OffScreenRenderEnv

    task_lang, bddl, init_states = cached_task
    owns_env = reuse_env is None
    env = OffScreenRenderEnv(bddl_file_name=bddl, camera_heights=64, camera_widths=64) if owns_env else reuse_env
    try:
        obs = _reset_and_settle_libero_env(env, init_states, int(init_idx))
        sdk = LiberoPolicySDK(env, task_lang)
        sdk.bind_initial_obs(obs)
        return sdk.get_state()
    finally:
        if owns_env:
            env.close()


def _libero_initial_prompt(task_lang: str, sample_state: Mapping[str, Any]) -> str:
    return "\n".join(
        [
            f"Write a generalizable Python robot manipulation policy `run_policy(sdk)` for the LIBERO task:",
            f'  Task instruction: "{task_lang}"',
            "",
            SHARED_LIBERO_SDK_CONTRACT,
            "",
            "Example initial `sdk.get_state()` keys/schema on this scene (do NOT hardcode coordinates; read live values):",
            json.dumps(
                {
                    "objects": list((sample_state.get("objects") or {}).keys()),
                    "fixtures": list((sample_state.get("fixtures") or {}).keys()),
                    "sites": list((sample_state.get("sites") or {}).keys()),
                    "sample_objects": sample_state.get("objects"),
                    "sample_fixtures": sample_state.get("fixtures"),
                    "sample_sites": sample_state.get("sites"),
                },
                indent=2,
            ),
        ]
    )


def _libero_aspire_prompt(task_lang: str, prev_src: str, dev_runs: Sequence[Mapping[str, Any]], it: int) -> str:
    logs = [
        f"- init={r['init_idx']} success={r['success']} steps={r['steps_used']} err={r['error']!r} "
        f"ret={r.get('policy_return')} lifts={r['obj_lifts']} eef={r['final_state']['eef_pos']}"
        for r in dev_runs
    ]
    return "\n".join(
        [
            f'ASPIRE Iterative Debugging for LIBERO task "{task_lang}" (iteration {it}):',
            "Diagnose the failed dev rollouts below and return a revised, generalizable `run_policy(sdk)`.",
            "",
            SHARED_LIBERO_SDK_CONTRACT,
            "",
            "Previous Policy:",
            "```python",
            prev_src,
            "```",
            "",
            "Dev Rollout Logs:",
            *logs,
            "",
            "Return ONLY valid Python code defining `run_policy(sdk)`.",
        ]
    )


def _libero_simex_prompt(
    task_lang: str,
    prev_src: str,
    dev_runs: Sequence[Mapping[str, Any]],
    passed_ever: set[int],
    slot: int,
    it: int,
    toolbox_notes: Sequence[str] = (),
) -> str:
    angles = (
        "Hypothesis A (Object/Region Grounding): Verify the exact object/site selection and target coordinates from `sdk.get_state()`.",
        "Hypothesis B (Grasp & Contact Precision): Refine pre-grasp approach, descent height, gripper orientation/timing, and lift clearance.",
        "Hypothesis C (Transport & Placement): Preserve working grasp and improve alignment, release height, or articulated pull/push trajectory.",
    )
    logs = [
        f"- init={r['init_idx']} success={r['success']} steps={r['steps_used']} err={r['error']!r} "
        f"ret={r.get('policy_return')} lifts={r['obj_lifts']} eef={r['final_state']['eef_pos']} "
        f"objs={r['final_state']['objects']}"
        for r in dev_runs
    ]
    tb_block = (
        ["Paired Toolbox Notes (SimEX persistent memory):", *[f"- {t}" for t in toolbox_notes], ""]
        if toolbox_notes
        else []
    )
    return "\n".join(
        [
            f'SimEX-Protocol (Adapted) Candidate {slot + 1} for LIBERO task "{task_lang}" (iteration {it}):',
            angles[slot % len(angles)],
            f"Capability-bank passing init_indices to preserve: {sorted(passed_ever)}",
            "",
            *tb_block,
            SHARED_LIBERO_SDK_CONTRACT,
            "",
            "Current Best Policy:",
            "```python",
            prev_src,
            "```",
            "",
            "Dev Rollout Traces:",
            *logs,
            "",
            "Return ONLY valid Python code defining `run_policy(sdk)`.",
        ]
    )


def _libero_rax_prompt(
    task_lang: str,
    prev_src: str,
    dev_runs: Sequence[Mapping[str, Any]],
    progress_notes: Sequence[str],
    skill_repo_entries: Sequence[str],
    it: int,
) -> str:
    """Matched-budget RAX-Protocol (Adapted from arXiv:2610.22496 Sections 3.3, 3.4, 5.5).

    Combines RAX's multi-timescale working memory (PROGRESS/TIPS), modular subroutine
    preservation, and cross-task versioned skill repository over the identical host_state_api
    SDK and dev LLM budget (without TRAX's 1-reset ActiveProbe or 4-layer failure attribution).
    """
    sub = analyze_policy_subroutines(prev_src, [])
    fn_names = [s["name"] for s in sub.get("subroutines") or []]
    logs = [
        f"- init={r['init_idx']} success={r['success']} steps={r['steps_used']} err={r['error']!r} "
        f"ret={r.get('policy_return')} lifts={r['obj_lifts']} eef={r['final_state']['eef_pos']}\n"
        f"    objs={r['final_state']['objects']}"
        for r in dev_runs
    ]
    prog_block = (
        ["Session Working Memory (PROGRESS / TIPS from prior exploration iterations):"]
        + [f"- {n}" for n in progress_notes[-4:]]
        + [""]
        if progress_notes
        else []
    )
    repo_block = (
        ["Versioned Cross-Task Skill Repository (verified modular skills from prior tasks):"]
        + [f"- {s}" for s in skill_repo_entries[-2:]]
        + [""]
        if skill_repo_entries
        else []
    )
    mod_block = (
        [f"Modular Program Subroutines ({', '.join(fn_names)}): preserve working helper routines and selectively revise only failing stages.", ""]
        if fn_names
        else []
    )
    return "\n".join(
        [
            f'RAX-Protocol (Adapted) Exploration & Compilation for LIBERO task "{task_lang}" (iteration {it}):',
            "Diagnose execution feedback from the dev rollouts, compile a clean modular `run_policy(sdk)` with reusable helpers, "
            "and generalize across all unseen layouts without hardcoding scene coordinates.",
            "",
            *mod_block,
            *prog_block,
            *repo_block,
            SHARED_LIBERO_SDK_CONTRACT,
            "",
            "Current Best Modular Program:",
            "```python",
            prev_src,
            "```",
            "",
            "Dev Rollout Execution Feedback:",
            *logs,
            "",
            "Return ONLY valid Python code defining `run_policy(sdk)`.",
        ]
    )


def _libero_craft_hypotheses(
    dev_runs: Sequence[Mapping[str, Any]],
    *,
    stagnant_iters: int = 0,
) -> list[tuple[str, str]]:
    """Task-agnostic multi-hypothesis RepairScreener strategies for LIBERO manipulation."""
    passed_cnt = sum(1 for r in dev_runs if r.get("success"))
    fail_codes = [str((r.get("attribution") or {}).get("code", "")) for r in dev_runs if not r.get("success")]
    if passed_cnt > 0 and stagnant_iters == 0:
        return [
            (
                f"H-Subroutine1: Selective Subroutine Repair ({len(fail_codes)} failing init state(s))",
                "Keep working target-selection and grasp subroutines frozen; inspect the exact failing init states "
                "and fix only the failing stage (clearance, alignment, or fallback retry) with 0 regressions.",
            ),
            (
                "H-Subroutine2: Preserve Working Path + Add Closed-Loop Multi-Side Grasp Fallback",
                "Keep the primary execution path intact for passing init states, and add a compact fallback attempt "
                "along the next highest-clearance free-space radial direction when lift verification fails.",
            ),
        ]
    return [
        (
            "H-FreeSpace1: Max-Clearance Free-Space Contact & Probe-Grounded Jaw Alignment",
            "Ground grasp radius, z_offset_m, and jaw-to-radial alignment strictly in the Empirical Active-Probe Findings. "
            "Compute the radial grasp angle dynamically in WORLD coordinates maximizing clearance from ALL neighbor objects/fixtures "
            "(NEVER rely on random object `yaw`), verify grasp purely via live object Z lift (>=0.03m) and EEF proximity "
            "(never reject a successful lift on gripper_width alone), and retry alternate free-space angles if needed.",
        ),
        (
            "H-Ground2: Exact Spatial-Relation Grounding + Unblocked Approach",
            "Re-verify which object instance satisfies the spatial relation in `task_language` using live `objects`, `fixtures`, "
            "and `sites` poses; remove overly strict pre-grasp convergence aborts and keep step budget compact so transport completes.",
        ),
        (
            "H-Place3: High-Clearance Transport, Obstacle Avoidance & Precision Release",
            "Lift well above all surrounding objects/fixtures (e.g. max(obj_z, dest_z) + 0.16m), servo accurately over destination center, "
            "descend smoothly to seating height before opening the gripper, and verify object rests stably on target.",
        ),
    ]


def _libero_craft_prompt(
    task_lang: str,
    prev_src: str,
    dev_runs: Sequence[Mapping[str, Any]],
    passed_ever: set[int],
    stagnant_iters: int,
    failed_attempts: Sequence[Mapping[str, Any]],
    skill_hints: Sequence[str],
    it: int,
    *,
    probe_report: ProbeReport | None = None,
    hypothesis_title: str = "",
    hypothesis_strategy: str = "",
) -> str:
    failed = [r for r in dev_runs if not r["success"]]
    passed_cnt = sum(1 for r in dev_runs if r.get("success"))
    fail_codes = [str(r["attribution"]["code"]) for r in failed]
    sub = analyze_policy_subroutines(prev_src, fail_codes)

    hyp_block = (
        [f"RepairScreener Focus ({hypothesis_title}): {hypothesis_strategy}", ""]
        if hypothesis_title
        else []
    )
    if passed_cnt > 0:
        sub_block = [
            f"Selective Subroutine Scope ({sub['repair_scope']}): {sub['freeze_hint']}",
            "",
        ]
    else:
        sub_block = [
            f"Full Stage Redesign Scope (0 dev passes so far; do NOT freeze failing grasp/approach logic): "
            "Rewrite approach, grasp, and transport cleanly from empirical ActiveProbe measurements and verified skills.",
            "",
        ]

    diag = []
    seen_rx: dict[str, str] = {}
    for r in dev_runs:
        if r["success"]:
            diag.append(f"- init={r['init_idx']}: PASS (steps={r['steps_used']})")
        else:
            a = r["attribution"]
            seen_rx.setdefault(f"[{a['layer']}:{a['code']}]", f"{a['summary']} -> {a['prescription']}")
            diag.append(
                f"- init={r['init_idx']}: FAIL [{a['layer']}:{a['code']}] steps={r['steps_used']} "
                f"min_d={r['min_eef_to_any_obj_m']} closed={r['ever_closed_gripper']} max_lift={r['max_any_obj_lift_m']} "
                f"err={r['error']!r} ret={r.get('policy_return')}\n"
                f"    lifts={r['obj_lifts']} eef={r['final_state']['eef_pos']}\n"
                f"    objs={r['final_state']['objects']}\n"
                f"    fixtures={r['final_state'].get('fixtures', {})}\n"
                f"    sites={r['final_state']['sites']}"
            )
    rx_block = (
        ["Deduplicated Four-Layer Causal Diagnosis:"] + [f"- {k}: {v}" for k, v in seen_rx.items()] + [""]
        if seen_rx
        else []
    )
    trace_block = (
        [f"Sample Trace Tail (init={failed[0]['init_idx']}): {json.dumps(failed[0].get('trace_tail') or [])}", ""]
        if failed
        else []
    )
    att_block = (
        ["Recent Non-Improving Attempts (avoid repeating; obey AST rules strictly — NO leading-underscore names like `for _ in range`, NO imports):"]
        + [
            f"- it={fa['iter']}: dev={fa['dev_passed']}/{fa['dev_total']} attrs={fa['attributions']}"
            + (f" ast_issues={fa['ast_issues']}" if fa.get("ast_issues") else "")
            for fa in failed_attempts[-3:]
        ]
        + [""]
        if failed_attempts
        else []
    )
    sk_block = (
        ["Verified Cross-Task LIBERO SkillExperience (contract-verified reusable helpers & findings):"]
        + [f"- {s}" for s in skill_hints]
        + [""]
        if skill_hints
        else []
    )
    pr_block = (
        [probe_report.render_prompt_block(), ""]
        if (probe_report is not None and probe_report.findings)
        else []
    )
    return "\n".join(
        [
            f'TRAX RepairScreener Evolution for LIBERO task "{task_lang}" (iteration {it}):',
            *hyp_block,
            *sub_block,
            *pr_block,
            *att_block,
            *sk_block,
            SHARED_LIBERO_SDK_CONTRACT,
            "",
            "Best Baseline Policy:",
            "```python",
            prev_src,
            "```",
            "",
            f"Passing Init States to Preserve: {sorted(passed_ever)}",
            "Dev Rollout Telemetry:",
            *diag,
            "",
            *rx_block,
            *trace_block,
            "Return ONLY valid Python code defining `run_policy(sdk)`.",
        ]
    )


def render_libero_html(report: Mapping[str, Any]) -> str:
    by_m = list((report.get("summary_by_method") or {}).values())
    mcnemar = report.get("paired_mcnemar_craft_vs") or {}
    rows = []
    for m in by_m:
        mc = mcnemar.get(m["method_id"])
        mc_str = f"p={mc['p_value']} (+{mc['n_a_only']}/-{mc['n_b_only']})" if mc else "ref"
        rows.append(
            "<tr>"
            f"<td><strong>{html.escape(m['label'])}</strong></td>"
            f"<td><strong>{m['passed']}/{m['total_trials']} ({m['pass_rate']*100:.1f}%)</strong>"
            f"<br><small>CI [{m['wilson_95_ci'][0]*100:.1f}%, {m['wilson_95_ci'][1]*100:.1f}%]</small></td>"
            f"<td>devLLM={m['dev_llm_calls']} / rst={m['dev_sim_resets']} / evalLLM=0</td>"
            f"<td>{m['mean_wall_s']}s / {m['p95_wall_s']}s</td>"
            f"<td><small>{html.escape(mc_str)}</small></td>"
            "</tr>"
        )
    return f"""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"/>
<title>{html.escape(str(report.get('benchmark_version', LIBERO_BENCH_VERSION)))}</title>
<style>
body {{ font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; background: #0b1120; color: #e2e8f0; padding: 24px; }}
.wrap {{ max-width: 1100px; margin: 0 auto; }}
.card {{ background: #111827; border: 1px solid #1f2937; border-radius: 8px; padding: 16px 20px; margin-bottom: 16px; }}
table {{ width: 100%; border-collapse: collapse; font-size: 13px; }}
th, td {{ border-bottom: 1px solid #1f2937; padding: 8px 10px; text-align: left; }}
small {{ color: #94a3b8; }}
</style></head><body><div class="wrap">
<div class="card">
  <h1>{html.escape(str(report.get('benchmark_version', LIBERO_BENCH_VERSION)))} — Official LIBERO Suite Benchmark</h1>
  <small>suite={html.escape(str(report.get('suite_name')))} · tasks={html.escape(str(report.get('task_ids')))} · observation_contract=host_state_api (NOT RGB-D) · model={html.escape(str(report.get('model')))}</small>
</div>
<div class="card">
  <table><thead><tr><th>Method</th><th>Held-Out Success</th><th>Budget</th><th>Mean / P95 Wall</th><th>McNemar vs TRAX</th></tr></thead>
  <tbody>{''.join(rows)}</tbody></table>
</div></div></body></html>"""


def run_libero_benchmark(
    *,
    llm_fn: LLMGenerateFn,
    output_dir: str | Path,
    suite_name: str = "libero_spatial",
    task_ids: Sequence[int] = (0, 1, 2),
    dev_count: int = 5,
    eval_count: int = 10,
    provider: str = "codex_cli",
    model: str = "gpt-6-astra",
    reasoning_effort: str = "medium",
    max_dev_llm_calls: int = 5,
    horizon: int = 450,
    isolate_tasks: bool = True,
    progress_cb: Callable[[str], None] | None = None,
) -> dict[str, Any]:
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    pol_dir = out / "policies"
    pol_dir.mkdir(parents=True, exist_ok=True)
    task_ckpt_dir = out / "tasks"
    task_ckpt_dir.mkdir(parents=True, exist_ok=True)
    dev_indices = list(range(int(dev_count)))
    eval_indices = list(range(int(dev_count), int(dev_count) + int(eval_count)))
    method_ids = ("strong_zeroshot", "aspire_protocol", "simex_protocol", "rax_protocol", "craft_full")

    def _log(msg: str) -> None:
        if progress_cb:
            progress_cb(msg)

    skill_store = SkillExperienceStore(out / "libero_skills.json")
    simex_store = SkillExperienceStore(out / "simex_libero_skills.json")
    rax_store = SkillExperienceStore(out / "rax_libero_skills.json")
    contract_sha = digest(SHARED_LIBERO_SDK_CONTRACT)
    trials: list[dict[str, Any]] = []
    meta_by_task: dict[str, dict[str, Any]] = {}

    # Per-task subprocess isolation prevents upstream robosuite 1.4.0 XML FD exhaustion across tasks
    if isolate_tasks and len(task_ids) > 1:
        for tid in task_ids:
            tkey = f"{suite_name}_t{int(tid)}"
            ckpt = task_ckpt_dir / f"{tkey}.json"
            if ckpt.is_file():
                _log(f"[{tkey}] Resuming cached task checkpoint: {ckpt.name}")
                cached_res = json.loads(ckpt.read_text(encoding="utf-8"))
                meta_by_task.update(cached_res["meta_by_task"])
                trials.extend(cached_res["trials"])
                continue
            cmd = [
                sys.executable,
                "-m",
                "benchmarks.libero_bench",
                "--suite",
                suite_name,
                "--task-id",
                str(int(tid)),
                "--dev-count",
                str(int(dev_count)),
                "--eval-count",
                str(int(eval_count)),
                "--provider",
                provider,
                "--model",
                model,
                "--effort",
                reasoning_effort,
                "--max-dev-calls",
                str(int(max_dev_llm_calls)),
                "--horizon",
                str(int(horizon)),
                "--output",
                str(out),
                "--checkpoint-out",
                str(ckpt),
            ]
            _log(f"[{tkey}] Launching isolated worker subprocess (FD-leak safe)...")
            proc = subprocess.run(cmd, check=False)
            if proc.returncode != 0 or not ckpt.is_file():
                raise RuntimeError(f"LIBERO isolated worker failed on {tkey} (exit={proc.returncode})")
            cached_res = json.loads(ckpt.read_text(encoding="utf-8"))
            meta_by_task.update(cached_res["meta_by_task"])
            trials.extend(cached_res["trials"])
    else:
        for tid in task_ids:
            from libero.libero.envs import OffScreenRenderEnv

            cached = _load_libero_task(suite_name, int(tid))
            task_lang, bddl, init_states = cached
            tkey = f"{suite_name}_t{tid}"
            _log(f"[{tkey}] Task: {task_lang!r} (dev={dev_indices}, eval={eval_indices[0]}..{eval_indices[-1]})")
            task_env = OffScreenRenderEnv(bddl_file_name=bddl, camera_heights=64, camera_widths=64)
            try:
                sample_state = _probe_initial_scene_schema(suite_name, int(tid), dev_indices[0], cached, reuse_env=task_env)

                # Shared paired it=1 draft (Strong Zero-Shot)
                g0 = llm_fn(_libero_initial_prompt(task_lang, sample_state))
                src0 = str(g0["source"])
                init_runs = [run_libero_rollout(suite_name, int(tid), i, src0, horizon=horizon, cached_task=cached, reuse_env=task_env) for i in dev_indices]
                init_pass = sum(1 for r in init_runs if r["success"])
                _log(f"  [{tkey}][Shared it=1] dev={init_pass}/{len(dev_indices)}")

                arts: dict[str, dict[str, Any]] = {
                    "strong_zeroshot": {
                        "method_id": "strong_zeroshot",
                        "label": "Strong Zero-Shot",
                        "source": src0,
                        "dev_llm_calls": 1,
                        "dev_sim_resets": 0,
                        "dev_passed": init_pass,
                    }
                }

                # ASPIRE-Protocol (Adapted)
                best_src, best_pass, resets, calls = src0, init_pass, len(dev_indices), 1
                cur_src, last_runs = src0, list(init_runs)
                for it in range(2, max_dev_llm_calls + 1):
                    if best_pass == len(dev_indices):
                        break
                    cur_src = str(llm_fn(_libero_aspire_prompt(task_lang, cur_src, last_runs, it))["source"])
                    calls += 1
                    runs = [run_libero_rollout(suite_name, int(tid), i, cur_src, horizon=horizon, cached_task=cached, reuse_env=task_env) for i in dev_indices]
                    resets += len(dev_indices)
                    last_runs = runs
                    p = sum(1 for r in runs if r["success"])
                    _log(f"  [{tkey}][ASPIRE it={it}] dev={p}/{len(dev_indices)}")
                    if p >= best_pass:
                        best_pass, best_src = p, cur_src
                arts["aspire_protocol"] = {
                    "method_id": "aspire_protocol",
                    "label": "ASPIRE-Protocol (Adapted)",
                    "source": best_src,
                    "dev_llm_calls": calls,
                    "dev_sim_resets": resets,
                    "dev_passed": best_pass,
                }

                # SimEX-Protocol (Adapted, multi-hypothesis + capability bank + paired toolbox)
                sx_ever = {r["init_idx"] for r in init_runs if r["success"]}
                sx_src, sx_pass, sx_regs, sx_runs = src0, init_pass, 0, list(init_runs)
                sx_notes = [f"{sk.title}: {sk.guidance}" for sk in simex_store.list_verified(sdk_contract_sha256=contract_sha)[-2:]]
                resets, calls = len(dev_indices), 1
                for it in range(2, max_dev_llm_calls + 1):
                    if sx_pass == len(dev_indices):
                        break
                    cand = str(llm_fn(_libero_simex_prompt(task_lang, sx_src, sx_runs, sx_ever, it - 2, it, toolbox_notes=sx_notes))["source"])
                    calls += 1
                    runs = [run_libero_rollout(suite_name, int(tid), i, cand, horizon=horizon, cached_task=cached, reuse_env=task_env) for i in dev_indices]
                    resets += len(dev_indices)
                    p = sum(1 for r in runs if r["success"])
                    ok_set = {r["init_idx"] for r in runs if r["success"]}
                    regs = len(sx_ever - ok_set)
                    sx_ever.update(ok_set)
                    _log(f"  [{tkey}][SimEX-Protocol it={it}] dev={p}/{len(dev_indices)} regs={regs}")
                    if (p, -regs) >= (sx_pass, -sx_regs):
                        sx_pass, sx_regs, sx_src, sx_runs = p, regs, cand, runs
                arts["simex_protocol"] = {
                    "method_id": "simex_protocol",
                    "label": "SimEX-Protocol (Adapted)",
                    "provenance": "protocol_adapted_from_simex_2609_38982",
                    "source": sx_src,
                    "dev_llm_calls": calls,
                    "dev_sim_resets": resets,
                    "dev_passed": sx_pass,
                }

                # RAX-Protocol (Adapted: PROGRESS/TIPS working memory + modular subroutine preservation + versioned skill repo; NO TRAX ActiveProbe/4-layer attribution)
                rx_src, rx_pass, rx_runs = src0, init_pass, list(init_runs)
                rx_notes: list[str] = [
                    f"it=1 baseline dev={init_pass}/{len(dev_indices)}; "
                    f"sample_ret={init_runs[0].get('policy_return') if init_runs else None} err={init_runs[0].get('error') if init_runs else ''}"
                ]
                rx_repo_entries: list[str] = []
                for sk in rax_store.list_verified(sdk_contract_sha256=contract_sha)[-2:]:
                    line = f"{sk.title}: {sk.guidance}"
                    if sk.code_snippet:
                        snip = sk.code_snippet.strip()
                        if len(snip) > 1400:
                            snip = snip[:1400] + "\n# ..."
                        line += f"\n  Verified module:\n```python\n{snip}\n```"
                    rx_repo_entries.append(line)
                resets, calls = len(dev_indices), 1
                for it in range(2, max_dev_llm_calls + 1):
                    if rx_pass == len(dev_indices):
                        break
                    cand = str(llm_fn(_libero_rax_prompt(task_lang, rx_src, rx_runs, rx_notes, rx_repo_entries, it))["source"])
                    calls += 1
                    runs = [run_libero_rollout(suite_name, int(tid), i, cand, horizon=horizon, cached_task=cached, reuse_env=task_env) for i in dev_indices]
                    resets += len(dev_indices)
                    p = sum(1 for r in runs if r["success"])
                    fails = [r for r in runs if not r["success"]]
                    fail_sample = fails[0] if fails else {}
                    rx_notes.append(
                        f"it={it} dev={p}/{len(dev_indices)}; "
                        + (
                            f"fail_err={fail_sample.get('error')!r} ret={fail_sample.get('policy_return')} lifts={fail_sample.get('obj_lifts')}"
                            if fails
                            else "all dev init states passed"
                        )
                    )
                    _log(f"  [{tkey}][RAX-Protocol it={it}] dev={p}/{len(dev_indices)}")
                    if p >= rx_pass:
                        rx_pass, rx_src, rx_runs = p, cand, runs
                arts["rax_protocol"] = {
                    "method_id": "rax_protocol",
                    "label": "RAX-Protocol (Adapted)",
                    "provenance": "protocol_adapted_from_rax_2610_22496",
                    "source": rx_src,
                    "dev_llm_calls": calls,
                    "dev_sim_resets": resets,
                    "dev_passed": rx_pass,
                }

                # TRAX Full Engine (ActiveProbe + AST preflight + 4-layer attribution + Selective Subroutine Repair + Adaptive Escape + Skills)
                def _libero_prog(run_list: Sequence[Mapping[str, Any]]) -> float:
                    if not run_list:
                        return 0.0
                    vals = []
                    for r in run_list:
                        lift_c = min(0.15, max(0.0, float(r.get("max_any_obj_lift_m", 0.0))))
                        dist_p = 0.35 * min(0.5, float(r.get("min_eef_to_any_obj_m", 0.5)))
                        err_p = 1.0 if r.get("error") else 0.0
                        vals.append(lift_c - dist_p - err_p)
                    return round(sum(vals) / len(vals), 5)

                cr_ever = {r["init_idx"] for r in init_runs if r["success"]}
                cr_src, cr_pass, cr_regs, cr_runs = src0, init_pass, 0, list(init_runs)
                cr_prog = _libero_prog(init_runs)
                resets, calls = len(dev_indices), 1
                probe_rep: ProbeReport | None = None
                if init_pass == 0:
                    probe_rep = run_libero_active_probe(task_env, task_lang, init_states, dev_indices[0], sample_state, horizon=horizon)
                    if probe_rep is not None:
                        resets += probe_rep.sim_resets_used
                        _log(f"  [{tkey}][TRAX ActiveProbe] +1 sim reset -> {[f.summary for f in probe_rep.findings]}")
                stagnant = 0
                failed_atts: list[dict[str, Any]] = []
                verified_skills = skill_store.list_verified(sdk_contract_sha256=contract_sha)
                hints: list[str] = []
                for sk in verified_skills[-2:]:
                    line = f"{sk.title}: {sk.guidance}"
                    if sk.code_snippet:
                        snip = sk.code_snippet.strip()
                        if len(snip) > 1400:
                            snip = snip[:1400] + "\n# ..."
                        line += f"\n  Verified helper snippet:\n```python\n{snip}\n```"
                    hints.append(line)
                for it in range(2, max_dev_llm_calls + 1):
                    if cr_pass == len(dev_indices):
                        break
                    hyps = _libero_craft_hypotheses(cr_runs, stagnant_iters=stagnant)
                    ht, hs = hyps[(it - 2) % len(hyps)]
                    prompt = _libero_craft_prompt(
                        task_lang,
                        cr_src,
                        cr_runs,
                        cr_ever,
                        stagnant,
                        failed_atts,
                        hints,
                        it,
                        probe_report=probe_rep,
                        hypothesis_title=ht,
                        hypothesis_strategy=hs,
                    )
                    cand = str(llm_fn(prompt)["source"])
                    calls += 1
                    ast_chk = check_robosuite_ast_compliance(cand)
                    if not ast_chk["compliant"]:
                        stagnant += 1
                        issues = list(ast_chk.get("issues") or [])[:3]
                        failed_atts.append({
                            "iter": it,
                            "dev_passed": 0,
                            "dev_total": len(dev_indices),
                            "attributions": ["ast_blocked"],
                            "ast_issues": issues,
                        })
                        _log(f"  [{tkey}][TRAX it={it}] AST blocked (0 resets wasted): {issues}")
                        continue
                    runs = [run_libero_rollout(suite_name, int(tid), i, cand, horizon=horizon, cached_task=cached, reuse_env=task_env) for i in dev_indices]
                    resets += len(dev_indices)
                    p = sum(1 for r in runs if r["success"])
                    ok_set = {r["init_idx"] for r in runs if r["success"]}
                    regs = len(cr_ever - ok_set)
                    cr_ever.update(ok_set)
                    prog = _libero_prog(runs)
                    _log(f"  [{tkey}][TRAX it={it}] ({ht}) dev={p}/{len(dev_indices)} regs={regs} prog={prog:.3f} attr={[r['attribution']['code'] for r in runs if not r['success']]}")
                    if p > cr_pass:
                        stagnant = 0
                    else:
                        stagnant += 1
                        failed_atts.append({"iter": it, "dev_passed": p, "dev_total": len(dev_indices), "attributions": [r["attribution"]["code"] for r in runs if not r["success"]]})
                    if (p, -regs, prog) >= (cr_pass, -cr_regs, cr_prog):
                        cr_pass, cr_regs, cr_prog, cr_src, cr_runs = p, regs, prog, cand, runs
                arts["craft_full"] = {
                    "method_id": "craft_full",
                    "label": "TRAX (ActiveProbe + SubroutineRepair + Skills)",
                    "source": cr_src,
                    "dev_llm_calls": calls,
                    "dev_sim_resets": resets,
                    "dev_passed": cr_pass,
                }

                meta_by_task[tkey] = {"task_language": task_lang, "methods": {k: {mk: mv for mk, mv in v.items() if mk != "source"} for k, v in arts.items()}}
                for mid in method_ids:
                    src = str(arts[mid]["source"])
                    (pol_dir / f"{tkey}_{mid}.py").write_text(src + "\n", encoding="utf-8")
                    ok_cnt = 0
                    for ei in eval_indices:
                        rec = run_libero_rollout(suite_name, int(tid), ei, src, horizon=horizon, cached_task=cached, reuse_env=task_env)
                        rec["method_id"] = mid
                        rec["task_key"] = tkey
                        trials.append(rec)
                        if rec["success"]:
                            ok_cnt += 1
                    _log(f"[{tkey}][EVAL] {mid:16s} -> {ok_cnt}/{len(eval_indices)} ({ok_cnt/len(eval_indices)*100:.1f}%)")
                    if mid == "simex_protocol" and ok_cnt >= max(1, int(0.7 * len(eval_indices))):
                        simex_store.add(
                            SkillExperience(
                                skill_id=f"simex-libero-{tkey}",
                                title=f"SimEX {tkey} Verified Skill Note",
                                guidance=f"Verified frozen policy for '{task_lang}' ({ok_cnt}/{len(eval_indices)} held-out).",
                                task_name=tkey,
                                required_methods=("get_state", "move_to", "move_delta", "set_gripper"),
                                sdk_contract_sha256=contract_sha,
                                evidence_source="simulator",
                                verification_report_id=f"simex-libero-{tkey}",
                                verified=True,
                                verified_at_utc=_utc_now(),
                                measured_limits={"heldout_pass_rate": round(ok_cnt / len(eval_indices), 4)},
                            )
                        )
                    if mid == "rax_protocol" and ok_cnt >= max(1, int(0.7 * len(eval_indices))):
                        rx_snip = _extract_verified_helper_snippet(src)
                        if not rx_snip and validate_skill_code_snippet(src)["valid"]:
                            rx_snip = src.strip()
                        rax_store.add(
                            SkillExperience(
                                skill_id=f"rax-libero-{tkey}",
                                title=f"RAX {tkey} Verified Modular Skill",
                                guidance=f"Verified modular policy for '{task_lang}' ({ok_cnt}/{len(eval_indices)} held-out).",
                                task_name=tkey,
                                required_methods=("get_state", "move_to", "move_delta", "set_gripper"),
                                sdk_contract_sha256=contract_sha,
                                evidence_source="simulator",
                                verification_report_id=f"rax-libero-{tkey}",
                                verified=True,
                                verified_at_utc=_utc_now(),
                                measured_limits={"heldout_pass_rate": round(ok_cnt / len(eval_indices), 4)},
                                code_snippet=rx_snip,
                            )
                        )
                    if mid == "craft_full" and ok_cnt >= max(1, int(0.7 * len(eval_indices))):
                        helper_snip = _extract_verified_helper_snippet(src)
                        if not helper_snip and validate_skill_code_snippet(src)["valid"]:
                            helper_snip = src.strip()
                        probe_note = (
                            f" ActiveProbe: {probe_rep.findings[0].summary}"
                            if (probe_rep is not None and probe_rep.findings)
                            else ""
                        )
                        skill_store.add(
                            SkillExperience(
                                skill_id=f"libero-{tkey}",
                                title=f"LIBERO {tkey} Verified Manipulation Skill",
                                guidance=f"Verified frozen policy for '{task_lang}' ({ok_cnt}/{len(eval_indices)} held-out).{probe_note}",
                                task_name=tkey,
                                required_methods=("get_state", "move_to", "move_delta", "set_gripper"),
                                sdk_contract_sha256=contract_sha,
                                evidence_source="simulator",
                                verification_report_id=f"libero-{tkey}",
                                verified=True,
                                verified_at_utc=_utc_now(),
                                measured_limits={"heldout_pass_rate": round(ok_cnt / len(eval_indices), 4)},
                                code_snippet=helper_snip,
                            )
                        )
            finally:
                task_env.close()

    summary: dict[str, dict[str, Any]] = {}
    for mid in method_ids:
        mt = [t for t in trials if t["method_id"] == mid]
        tot = len(mt)
        ok = sum(1 for t in mt if t["success"])
        walls = [float(t["wall_s"]) for t in mt]
        dev_llm = sum(int(meta_by_task[tk]["methods"][mid]["dev_llm_calls"]) for tk in meta_by_task)
        dev_rst = sum(int(meta_by_task[tk]["methods"][mid]["dev_sim_resets"]) for tk in meta_by_task)
        by_task: dict[str, dict[str, Any]] = {}
        for tid in task_ids:
            tk = f"{suite_name}_t{int(tid)}"
            tt = [t for t in mt if t["task_key"] == tk]
            tok = sum(1 for t in tt if t["success"])
            by_task[tk] = {
                "passed": tok,
                "total": len(tt),
                "rate": round(tok / len(tt), 4) if tt else 0.0,
            }
        summary[mid] = {
            "method_id": mid,
            "label": next((meta_by_task[tk]["methods"][mid]["label"] for tk in meta_by_task), mid),
            "passed": ok,
            "total_trials": tot,
            "pass_rate": round(ok / tot, 4) if tot else 0.0,
            "wilson_95_ci": list(wilson_ci_95(ok, tot)),
            "by_task": by_task,
            "dev_llm_calls": dev_llm,
            "dev_sim_resets": dev_rst,
            "eval_llm_calls": 0,
            "mean_wall_s": round(sum(walls) / len(walls), 3) if walls else 0.0,
            "p95_wall_s": round(percentile(walls, 95.0) or 0.0, 3) if walls else 0.0,
        }

    keys = [(f"{suite_name}_t{tid}", ei) for tid in task_ids for ei in eval_indices]
    c_map = {(t["task_key"], int(t["init_idx"])): bool(t["success"]) for t in trials if t["method_id"] == "craft_full"}
    paired: dict[str, dict[str, Any]] = {}
    if len(c_map) == len(keys):
        c_vec = [c_map[k] for k in keys]
        for mid in ("strong_zeroshot", "aspire_protocol", "simex_protocol", "rax_protocol"):
            b_map = {(t["task_key"], int(t["init_idx"])): bool(t["success"]) for t in trials if t["method_id"] == mid}
            if len(b_map) == len(keys):
                paired[mid] = mcnemar_exact_p(c_vec, [b_map[k] for k in keys])

    report = {
        "benchmark_version": LIBERO_BENCH_VERSION,
        "suite_name": suite_name,
        "task_ids": list(task_ids),
        "dev_indices": dev_indices,
        "eval_indices": eval_indices,
        "observation_contract": "host_state_api (3D object/fixture/site poses; NOT raw RGB-D)",
        "evidence_source": "simulator",
        "model": model,
        "reasoning_effort": reasoning_effort,
        "provenance": _git_provenance(),
        "finished_at_utc": _utc_now(),
        "meta_by_task": meta_by_task,
        "summary_by_method": summary,
        "paired_mcnemar_craft_vs": paired,
        "trials": trials,
    }
    _write_json_atomic(out / "libero_report.json", report)
    (out / "libero_report.html").write_text(render_libero_html(report), encoding="utf-8")
    return report


def _worker_main() -> None:
    from .prior_work_bench import make_live_llm_fn

    parser = argparse.ArgumentParser(description="Isolated single-task LIBERO benchmark worker")
    parser.add_argument("--suite", default="libero_spatial")
    parser.add_argument("--task-id", type=int, required=True)
    parser.add_argument("--dev-count", type=int, default=5)
    parser.add_argument("--eval-count", type=int, default=10)
    parser.add_argument("--provider", default="codex_cli")
    parser.add_argument("--model", default="gpt-6-astra")
    parser.add_argument("--effort", default="medium")
    parser.add_argument("--max-dev-calls", type=int, default=5)
    parser.add_argument("--horizon", type=int, default=450)
    parser.add_argument("--output", required=True)
    parser.add_argument("--checkpoint-out", required=True)
    args = parser.parse_args()

    llm_fn = make_live_llm_fn(provider=args.provider, model=args.model, reasoning_effort=args.effort)
    rep = run_libero_benchmark(
        llm_fn=llm_fn,
        output_dir=args.output,
        suite_name=args.suite,
        task_ids=(args.task_id,),
        dev_count=args.dev_count,
        eval_count=args.eval_count,
        provider=args.provider,
        model=args.model,
        reasoning_effort=args.effort,
        max_dev_llm_calls=args.max_dev_calls,
        horizon=args.horizon,
        isolate_tasks=False,
        progress_cb=lambda msg: print(msg, flush=True),
    )
    _write_json_atomic(
        Path(args.checkpoint_out),
        {"meta_by_task": rep["meta_by_task"], "trials": rep["trials"]},
    )


if __name__ == "__main__":
    _worker_main()
