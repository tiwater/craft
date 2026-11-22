"""Native MuJoCo café workstation simulator & SceneHostDriver (`driver="mujoco_cafe"`).

Provides a 100% open-source, self-contained, Isaac-Sim-free MuJoCo physics
simulation of the single-cup café workstation for CRAFT:
- Articulated 3-axis + yaw + parallel-jaw robotic manipulator with MuJoCo position actuators
- Real MuJoCo rigid-body physics for cups (`dine_in_red`, `takeout_green`) and `flexible_bag` distractor
- Espresso machine housing, brew spout, drip tray, animated brew stream, Pickup Slots 1/2,
  and Forbidden Steam Zone (`$ZONE_STEAM_NOGO`)
- Hardware-accelerated headless EGL (or OSMesa fallback) multi-camera rendering
  (`overview`, `overhead`, `spout`, `wrist`) for CRAFT Studio
- Full 13-method EdgeZeroGtCoffeeSDK contract + strict AST sandbox execution +
  host-owned L2 geometric pouring gate + intent/receipt ledger + oracle scoring

Strictly preserves separation of ownership:
- Perception owner: MuJoCo-derived read-only EvidenceScene snapshot (oracle is scoring-only)
- Policy execution: AST-sandboxed `run_policy(sdk, params)`
- Host actuation: MuJoCo physics stepping & command ledger
"""
from __future__ import annotations

import ast
import atexit
from concurrent.futures import ThreadPoolExecutor
import io
import math
import os
import threading
import time
from typing import Any, Mapping, Sequence

# Configure headless EGL before any downstream `import mujoco` executes.
if not os.environ.get("MUJOCO_GL") and not os.environ.get("DISPLAY"):
    os.environ["MUJOCO_GL"] = "egl"

from .config import SceneConfig
from .contracts import digest
from .edge_adapter import (
    CapabilityCheckResult,
    DiscoveredCapabilities,
    ReconciliationResult,
    SceneObservationResult,
)
from .journal import Journal
from .prior_work_bench import check_sandbox_ast_compliance


# ---------------------------------------------------------------------------
# Open-Source MuJoCo MJCF Café Workstation Model
# ---------------------------------------------------------------------------

SPOUT_XY = (0.45, 0.00)
PICKUP_1_XY = (0.38, -0.17)
PICKUP_2_XY = (0.38, 0.17)
TABLE_TOP_Z = 1.00
CUP_HALF_H = 0.055
CUP_REST_Z = TABLE_TOP_Z + CUP_HALF_H
L2_SAFE_RADIUS_M = 0.025
L2_MAX_TILT_DEG = 12.0

CAFE_WORKSTATION_MJCF = """<mujoco model="craft_cafe_workstation">
  <compiler angle="degree" coordinate="local" inertiafromgeom="true"/>
  <option timestep="0.005" gravity="0 0 -9.81" integrator="implicitfast"/>

  <visual>
    <global offwidth="640" offheight="480"/>
    <headlight diffuse="0.65 0.65 0.68" ambient="0.32 0.32 0.35" specular="0.2 0.2 0.2"/>
    <rgba haze="0.06 0.09 0.14 1"/>
    <quality shadowsize="2048"/>
  </visual>

  <asset>
    <texture name="sky_tex" type="skybox" builtin="gradient" rgb1="0.16 0.23 0.34" rgb2="0.05 0.08 0.13" width="256" height="256"/>
    <texture name="grid_tex" type="2d" builtin="checker" rgb1="0.14 0.17 0.22" rgb2="0.10 0.13 0.18" width="512" height="512"/>
    <material name="floor_mat" texture="grid_tex" texrepeat="8 8" reflectance="0.1"/>
    <material name="table_mat" rgba="0.24 0.19 0.15 1" specular="0.2" shininess="0.3"/>
    <material name="steel_mat" rgba="0.62 0.66 0.72 1" specular="0.7" shininess="0.8"/>
    <material name="dark_steel_mat" rgba="0.16 0.19 0.24 1" specular="0.5" shininess="0.6"/>
    <material name="arm_mat" rgba="0.88 0.90 0.94 1" specular="0.5" shininess="0.6"/>
    <material name="joint_mat" rgba="0.18 0.52 0.86 1" specular="0.6" shininess="0.7"/>
    <material name="cup_red_mat" rgba="0.88 0.18 0.26 1" specular="0.5" shininess="0.6"/>
    <material name="cup_green_mat" rgba="0.10 0.72 0.45 1" specular="0.4" shininess="0.5"/>
    <material name="bag_mat" rgba="0.62 0.50 0.36 1" specular="0.1" shininess="0.1"/>
    <material name="slot1_mat" rgba="0.15 0.68 0.92 0.85"/>
    <material name="slot2_mat" rgba="0.95 0.68 0.15 0.85"/>
    <material name="nogo_mat" rgba="0.92 0.20 0.18 0.80"/>
    <material name="coffee_mat" rgba="0.24 0.12 0.05 1"/>
  </asset>

  <worldbody>
    <light pos="0.20 -0.55 2.35" dir="0.1 0.25 -1" diffuse="0.88 0.85 0.80" specular="0.3 0.3 0.3" castshadow="true"/>
    <light pos="0.55 0.55 2.15" dir="-0.1 -0.2 -1" diffuse="0.48 0.52 0.60" castshadow="false"/>

    <!-- Unobstructed Cameras -->
    <camera name="overview" pos="-0.12 -0.58 1.56" xyaxes="0.86 -0.51 0 0.27 0.45 0.85" fovy="45"/>
    <camera name="overhead" pos="0.39 0.00 1.54" xyaxes="0 1 0 -1 0 0" fovy="62"/>
    <camera name="spout" pos="0.16 -0.26 1.24" xyaxes="0.72 0.69 0 -0.22 0.23 0.95" fovy="42"/>

    <!-- Floor & Café Counter -->
    <geom name="floor" type="plane" size="3 3 0.1" material="floor_mat"/>
    <body name="counter" pos="0.40 0.0 0.50">
      <geom name="table_top" type="box" size="0.42 0.44 0.50" material="table_mat" friction="0.9 0.01 0.001"/>
    </body>

    <!-- Pickup Slots & Forbidden Steam Zone Markers -->
    <geom name="slot_pickup_1" type="cylinder" pos="0.38 -0.17 1.001" size="0.055 0.0015" material="slot1_mat" contype="0" conaffinity="0"/>
    <geom name="slot_pickup_2" type="cylinder" pos="0.38 0.17 1.001" size="0.055 0.0015" material="slot2_mat" contype="0" conaffinity="0"/>
    <geom name="zone_steam_nogo" type="box" pos="0.56 0.22 1.001" size="0.075 0.075 0.0015" material="nogo_mat" contype="0" conaffinity="0"/>

    <!-- Espresso Machine Housing & Spout (zero z-fighting) -->
    <body name="espresso_machine" pos="0.58 0.0 1.00">
      <geom name="machine_base" type="box" pos="0.04 0 0.15" size="0.105 0.135 0.15" material="steel_mat"/>
      <geom name="machine_top" type="box" pos="-0.02 0 0.34" size="0.165 0.14 0.04" material="dark_steel_mat"/>
      <geom name="machine_badge" type="box" pos="-0.186 0 0.34" size="0.003 0.045 0.014" material="joint_mat" contype="0" conaffinity="0"/>
      <geom name="drip_tray" type="box" pos="-0.13 0 0.003" size="0.075 0.095 0.003" material="dark_steel_mat"/>
      <geom name="l2_safe_ring" type="cylinder" pos="-0.13 0 0.0065" size="0.032 0.0008" rgba="0.18 0.82 0.45 0.55" contype="0" conaffinity="0"/>
      <geom name="brew_spout" type="cylinder" pos="-0.13 0 0.27" size="0.014 0.032" material="steel_mat" contype="0" conaffinity="0"/>
      <geom name="steam_wand" type="capsule" fromto="-0.05 0.11 0.26 -0.02 0.21 0.14" size="0.008" material="steel_mat" contype="0" conaffinity="0"/>
    </body>

    <!-- Animated Brew Stream (parked invisibly inside machine_top at z=1.34, extends while brewing) -->
    <body name="brew_stream" pos="0.45 0.0 1.32">
      <joint name="stream_slide" type="slide" axis="0 0 1" range="-0.16 0.02" damping="10"/>
      <geom name="stream_geom" type="cylinder" size="0.0055 0.038" material="coffee_mat" contype="0" conaffinity="0"/>
    </body>

    <!-- Articulated 3-Axis + Yaw + Parallel-Jaw Café Robot (rear cantilever keeps overview & overhead 100% clear) -->
    <body name="robot_rail" pos="0.12 0.0 1.66">
      <geom name="rail_pillar_back" type="cylinder" pos="-0.12 0.36 -0.33" size="0.028 0.33" material="dark_steel_mat" contype="0" conaffinity="0"/>
      <geom name="rail_cantilever" type="box" pos="-0.12 0.16 0.01" size="0.022 0.22 0.022" material="dark_steel_mat" contype="0" conaffinity="0"/>
      <body name="ee_x" pos="0.0 0.0 -0.10">
        <joint name="arm_x" type="slide" axis="1 0 0" range="-0.10 0.55" damping="18"/>
        <geom name="carriage_x" type="box" size="0.032 0.042 0.026" material="joint_mat" contype="0" conaffinity="0"/>
        <body name="ee_y" pos="0.0 0.0 0.0">
          <joint name="arm_y" type="slide" axis="0 1 0" range="-0.34 0.34" damping="18"/>
          <geom name="carriage_y" type="cylinder" size="0.026 0.032" material="arm_mat" contype="0" conaffinity="0"/>
          <body name="ee_z" pos="0.0 0.0 -0.10">
            <joint name="arm_z" type="slide" axis="0 0 1" range="-0.45 0.08" damping="22"/>
            <geom name="arm_shaft" type="cylinder" pos="0 0 0.14" size="0.015 0.14" material="arm_mat" contype="0" conaffinity="0"/>
            <body name="ee_wrist" pos="0 0 -0.02">
              <joint name="arm_yaw" type="hinge" axis="0 0 1" range="-180 180" damping="6"/>
              <camera name="wrist" pos="-0.06 0.0 0.07" xyaxes="0 -1 0 0.45 0 0.89" fovy="58"/>
              <geom name="wrist_hub" type="cylinder" size="0.030 0.016" material="joint_mat" contype="0" conaffinity="0"/>
              <geom name="gripper_bar" type="box" pos="0 0 -0.024" size="0.015 0.054 0.009" material="dark_steel_mat" contype="0" conaffinity="0"/>
              <body name="finger_left" pos="0 0.044 -0.062">
                <joint name="finger_l" type="slide" axis="0 1 0" range="-0.028 0.016" damping="8"/>
                <geom name="finger_l_geom" type="box" size="0.010 0.006 0.032" material="steel_mat" contype="0" conaffinity="0"/>
              </body>
              <body name="finger_right" pos="0 -0.044 -0.062">
                <joint name="finger_r" type="slide" axis="0 -1 0" range="-0.028 0.016" damping="8"/>
                <geom name="finger_r_geom" type="box" size="0.010 0.006 0.032" material="steel_mat" contype="0" conaffinity="0"/>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>

    <!-- Rigid Cup A (dine_in_red) -->
    <body name="cup_a" pos="0.33 -0.02 1.055">
      <freejoint name="cup_a_free"/>
      <geom name="cup_a_body" type="cylinder" size="0.034 0.055" mass="0.22" material="cup_red_mat" friction="0.9 0.01 0.001"/>
      <geom name="cup_a_rim" type="cylinder" pos="0 0 0.052" size="0.036 0.004" mass="0.01" rgba="0.96 0.96 0.98 1" contype="0" conaffinity="0"/>
      <geom name="cup_a_coffee" type="cylinder" pos="0 0 0.046" size="0.030 0.002" mass="0.005" material="coffee_mat" contype="0" conaffinity="0"/>
    </body>

    <!-- Rigid Cup B (takeout_green, parked off-table unless dual-cup preset active) -->
    <body name="cup_b" pos="1.80 1.80 1.055">
      <freejoint name="cup_b_free"/>
      <geom name="cup_b_body" type="cylinder" size="0.033 0.055" mass="0.18" material="cup_green_mat" friction="0.9 0.01 0.001"/>
      <geom name="cup_b_lid" type="cylinder" pos="0 0 0.055" size="0.036 0.005" mass="0.01" rgba="0.95 0.95 0.95 1" contype="0" conaffinity="0"/>
    </body>

    <!-- Distractor Flexible Bag (non-cup object that policies must ignore) -->
    <body name="bag_distractor" pos="0.24 0.24 1.030">
      <freejoint name="bag_free"/>
      <geom name="bag_geom" type="box" size="0.052 0.035 0.030" mass="0.12" material="bag_mat" friction="0.85 0.01 0.001"/>
    </body>
  </worldbody>

  <actuator>
    <position name="act_x" joint="arm_x" kp="280" ctrlrange="-0.10 0.55"/>
    <position name="act_y" joint="arm_y" kp="280" ctrlrange="-0.34 0.34"/>
    <position name="act_z" joint="arm_z" kp="320" ctrlrange="-0.45 0.08"/>
    <position name="act_yaw" joint="arm_yaw" kp="60" ctrlrange="-180 180"/>
    <position name="act_fl" joint="finger_l" kp="120" ctrlrange="-0.028 0.016"/>
    <position name="act_fr" joint="finger_r" kp="120" ctrlrange="-0.028 0.016"/>
    <position name="act_stream" joint="stream_slide" kp="120" ctrlrange="-0.16 0.02"/>
  </actuator>
</mujoco>
"""


# ---------------------------------------------------------------------------
# Restricted AST-Sandboxed Policy SDK for MuJoCo Café Host
# ---------------------------------------------------------------------------

_SAFE_BUILTINS: dict[str, Any] = {
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
}


class _MuJoCoCafePolicySDK:
    """Host-owned 13-method Coffee SDK backed directly by MuJoCo physics."""

    ALLOWED_METHODS = (
        "get_order",
        "observe_scene",
        "request_operator_confirmation",
        "retract_safe_pose",
        "escalate_to_human_operator",
        "complete_order_delivery",
        "query_command_status",
        "move_to_target",
        "place_cup_under_machine",
        "recenter_cup_under_spout",
        "start_coffee_machine",
        "wait_for_coffee_ready",
        "retrieve_and_deliver_cup",
    )

    def __init__(self, driver: MuJoCoCafeHostDriver, scenario: Mapping[str, Any]) -> None:
        self._driver = driver
        self._scenario = scenario
        self.steps: list[dict[str, Any]] = []
        self.brew_count = 0
        self.unknown_events = 0
        self.queried_after_unknown = True
        self._pending_unknown = False
        self._command_receipts: dict[str, dict[str, Any]] = {}
        self.human_interventions = 0
        self._slip = scenario.get("fault_inject_slip_xy_m")
        self._sim_timeout = bool(scenario.get("simulate_timeout", False))
        self._recenter_calls = 0

    def _rec(self, method: str, receipt: str = "done", details: Mapping[str, Any] | None = None) -> None:
        entry = {
            "stepIndex": len(self.steps),
            "method": method,
            "receipt": receipt,
            "timestampMs": int(time.time() * 1000),
            "details": dict(details or {}),
        }
        self.steps.append(entry)
        self._driver._update_live_steps(self.steps)

    def get_order(self, order_id: str = "A042") -> dict[str, Any] | None:
        orders = self._driver._orders_catalog(self._scenario)
        order = orders.get(order_id)
        self._rec("get_order", "done" if order else "not_found", {"orderId": order_id, "order": order})
        return dict(order) if order else None

    def observe_scene(self) -> dict[str, Any]:
        obs = self._driver._build_sdk_observation()
        self._rec(
            "observe_scene",
            "done",
            {
                "controlReady": obs["control_ready"],
                "objectCount": len(obs["objects"]),
                "safeToPour": obs["l2_pouring_safety"]["safe_to_pour"],
                "radialOffsetM": obs["l2_pouring_safety"]["current_radial_offset_m"],
            },
        )
        return obs

    def request_operator_confirmation(self, reason: str, candidates: list[Any]) -> dict[str, Any]:
        self.human_interventions += 1
        r = {"receipt": "paused_for_operator", "reason": reason, "candidates": candidates}
        self._rec("request_operator_confirmation", "paused_for_operator", {"reason": reason, "candidates": len(candidates)})
        return r

    def retract_safe_pose(self, *, command_id: str) -> dict[str, Any]:
        self._driver._move_arm_world(0.18, 0.00, 1.38, gripper_open=True)
        r = {"receipt": "done", "command_id": command_id}
        self._command_receipts[command_id] = r
        self._rec("retract_safe_pose", "done", {"commandId": command_id})
        return r

    def escalate_to_human_operator(self, order_id: str, reason: str, *, command_id: str) -> dict[str, Any]:
        self.human_interventions += 1
        r = {"receipt": "done", "command_id": command_id, "order_id": order_id, "reason": reason}
        self._command_receipts[command_id] = r
        self._rec("escalate_to_human_operator", "escalated_to_human", {"commandId": command_id, "orderId": order_id, "reason": reason})
        return r

    def move_to_target(self, target_world: Any, *, command_id: str) -> dict[str, Any]:
        tx, ty, tz = float(target_world[0]), float(target_world[1]), float(target_world[2])
        self._driver._move_arm_world(tx, ty, max(1.08, tz), gripper_open=True)
        r = {"receipt": "done", "command_id": command_id}
        self._command_receipts[command_id] = r
        self._rec("move_to_target", "done", {"commandId": command_id})
        return r

    def place_cup_under_machine(self, cup_id: str, *, command_id: str) -> dict[str, Any]:
        dx, dy = (float(self._slip[0]), float(self._slip[1])) if self._slip else (0.0, 0.0)
        ok = self._driver._transport_cup_physics(cup_id, SPOUT_XY[0] + dx, SPOUT_XY[1] + dy)
        receipt = "done" if ok else "rejected"
        r = {"receipt": receipt, "command_id": command_id, "cup_id": cup_id}
        self._command_receipts[command_id] = r
        self._rec("place_cup_under_machine", receipt, {"commandId": command_id, "cupId": cup_id})
        return r

    def recenter_cup_under_spout(self, cup_id: str, *, command_id: str) -> dict[str, Any]:
        self._recenter_calls += 1
        # Physically realistic bounded recentering: reduces remaining radial offset smoothly
        cur_xy = self._driver._get_cup_xy(cup_id)
        if cur_xy is None:
            r = {"receipt": "rejected", "command_id": command_id, "reason": "cup_not_found"}
            self._command_receipts[command_id] = r
            self._rec("recenter_cup_under_spout", "rejected", {"commandId": command_id})
            return r
        err_x = cur_xy[0] - SPOUT_XY[0]
        err_y = cur_xy[1] - SPOUT_XY[1]
        # Each recenter step removes ~85% of residual offset (so <=45mm clears in 1 recenter cleanly)
        residual_scale = 0.12 if self._recenter_calls == 1 else 0.02
        ok = self._driver._transport_cup_physics(
            cup_id,
            SPOUT_XY[0] + err_x * residual_scale,
            SPOUT_XY[1] + err_y * residual_scale,
        )
        receipt = "done" if ok else "rejected"
        r = {"receipt": receipt, "command_id": command_id, "cup_id": cup_id}
        self._command_receipts[command_id] = r
        self._rec("recenter_cup_under_spout", receipt, {"commandId": command_id, "cupId": cup_id})
        return r

    def start_coffee_machine(self, *, command_id: str) -> dict[str, Any]:
        l2 = self._driver._compute_l2_pouring_safety()
        if not l2["safe_to_pour"]:
            r = {"receipt": "rejected", "command_id": command_id, "reason": "l2_pouring_unsafe"}
            self._command_receipts[command_id] = r
            self._rec("start_coffee_machine", "rejected", {"commandId": command_id, "reason": "l2_pouring_unsafe"})
            return r
        self.brew_count += 1
        self._driver._machine_status = "running"
        self._driver._brew_progress = 0.15
        self._driver._set_brew_stream(active=True)
        final_r = {"receipt": "running", "command_id": command_id}
        self._command_receipts[command_id] = final_r
        if self._sim_timeout and self.brew_count == 1:
            self.unknown_events += 1
            self._pending_unknown = True
            self.queried_after_unknown = False
            self._rec("start_coffee_machine", "unknown", {"commandId": command_id})
            self._rec("command_ack_timeout_unknown", "unknown", {"commandId": command_id})
            return {"receipt": "unknown", "command_id": command_id}
        self._rec("start_coffee_machine", "running", {"commandId": command_id})
        return final_r

    def query_command_status(self, command_id: str) -> dict[str, Any]:
        if self._pending_unknown:
            self._pending_unknown = False
            self.queried_after_unknown = True
        r = self._command_receipts.get(command_id, {"receipt": "unknown", "command_id": command_id})
        self._rec("query_command_status", str(r.get("receipt", "unknown")), {"commandId": command_id})
        return dict(r)

    def wait_for_coffee_ready(self, *, command_id: str) -> dict[str, Any]:
        if self.brew_count < 1:
            r = {"receipt": "rejected", "command_id": command_id, "reason": "brew_not_started"}
            self._command_receipts[command_id] = r
            self._rec("wait_for_coffee_ready", "rejected", {"commandId": command_id})
            return r
        self._driver._simulate_brew_cycle()
        r = {"receipt": "done", "command_id": command_id}
        self._command_receipts[command_id] = r
        self._rec("wait_for_coffee_ready", "done", {"commandId": command_id, "cupFilled": True})
        return r

    def retrieve_and_deliver_cup(self, cup_id: str, target_slot: str, *, command_id: str) -> dict[str, Any]:
        goal_xy = PICKUP_2_XY if target_slot == "$SLOT_PICKUP_2" else PICKUP_1_XY
        ok = self._driver._transport_cup_physics(cup_id, goal_xy[0], goal_xy[1])
        self._driver._move_arm_world(0.20, 0.00, 1.38, gripper_open=True)
        receipt = "done" if ok else "rejected"
        r = {"receipt": receipt, "command_id": command_id, "cup_id": cup_id, "target_slot": target_slot}
        self._command_receipts[command_id] = r
        self._rec("retrieve_and_deliver_cup", receipt, {"commandId": command_id, "targetSlot": target_slot})
        return r

    def complete_order_delivery(self, order_id: str, cup_id: str, destination_slot: str, *, command_id: str) -> dict[str, Any]:
        r = {"receipt": "done", "command_id": command_id, "order_id": order_id}
        self._command_receipts[command_id] = r
        self._rec("complete_order_delivery", "done", {"commandId": command_id, "orderId": order_id})
        return r


# ---------------------------------------------------------------------------
# MuJoCoCafeHostDriver (SceneHostDriver implementation)
# ---------------------------------------------------------------------------

class MuJoCoCafeHostDriver:
    """Native MuJoCo physics host driver for the CRAFT single-cup café workstation."""

    driver_name = "mujoco_cafe"
    evidence_source = "simulator"

    def __init__(self, config: SceneConfig, *, realtime_pacing: bool = False) -> None:
        # Configure headless EGL before importing mujoco if not already set
        if not os.environ.get("MUJOCO_GL"):
            os.environ["MUJOCO_GL"] = "egl"
        try:
            import mujoco
            import numpy as np
        except ImportError as exc:
            raise RuntimeError(
                "MuJoCo driver requires optional dependencies. Install with: pip install 'tiwater-craft[mujoco]'"
            ) from exc

        self._mj = mujoco
        self._np = np
        self.config = config
        self._realtime_pacing = realtime_pacing
        self._lock = threading.RLock()
        self._contract_sha = digest(list(_MuJoCoCafePolicySDK.ALLOWED_METHODS))

        self._model = mujoco.MjModel.from_xml_string(CAFE_WORKSTATION_MJCF)
        self._data = mujoco.MjData(self._model)
        self._renderer: Any = None
        # Dedicated single worker thread keeps MuJoCo's EGL GLContext thread-local and safe under HTTP concurrency
        self._render_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="mujoco-egl")
        atexit.register(self.close)

        # Look up body / joint / actuator IDs cleanly
        self._body_ids = {
            "obj-cup-a": mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, "cup_a"),
            "obj-cup-b": mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, "cup_b"),
            "obj-bag-1": mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, "bag_distractor"),
        }
        self._jnt_qpos = {
            "obj-cup-a": int(self._model.jnt_qposadr[mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, "cup_a_free")]),
            "obj-cup-b": int(self._model.jnt_qposadr[mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, "cup_b_free")]),
            "obj-bag-1": int(self._model.jnt_qposadr[mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, "bag_free")]),
        }
        self._jnt_qvel = {
            "obj-cup-a": int(self._model.jnt_dofadr[mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, "cup_a_free")]),
            "obj-cup-b": int(self._model.jnt_dofadr[mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, "cup_b_free")]),
            "obj-bag-1": int(self._model.jnt_dofadr[mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, "bag_free")]),
        }
        self._act_ids = {
            name: mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in ("act_x", "act_y", "act_z", "act_yaw", "act_fl", "act_fr", "act_stream")
        }
        self._cup_coffee_geom_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_GEOM, "cup_a_coffee")
        self._step_seq = 0
        self._preset = "default"
        self._dual_cup_active = False
        self._confirmed_cup_id: str | None = None
        self._machine_status = "idle"
        self._cup_filled = False
        self._brew_progress = 0.0
        self._last_command: dict[str, Any] = {"command_id": None, "status": "idle", "steps": []}
        self.reset_scenario({"preset": "default"})

    def _update_live_steps(self, steps: list[dict[str, Any]]) -> None:
        self._last_command["steps"] = list(steps)

    def _set_free_body_pose(self, obj_id: str, x: float, y: float, z: float) -> None:
        qa = self._jnt_qpos[obj_id]
        va = self._jnt_qvel[obj_id]
        self._data.qpos[qa : qa + 7] = [x, y, z, 1.0, 0.0, 0.0, 0.0]
        self._data.qvel[va : va + 6] = 0.0

    def _set_arm_ctrl_world(self, wx: float, wy: float, wz: float, gripper_open: bool = True) -> None:
        # robot_rail origin is (0.12, 0.0, 1.56); ee_z base is -0.10, wrist is -0.02, finger tip -0.08
        self._data.ctrl[self._act_ids["act_x"]] = max(-0.10, min(0.55, wx - 0.12))
        self._data.ctrl[self._act_ids["act_y"]] = max(-0.34, min(0.34, wy))
        self._data.ctrl[self._act_ids["act_z"]] = max(-0.45, min(0.08, wz - 1.44))
        self._data.ctrl[self._act_ids["act_yaw"]] = 0.0
        finger_val = 0.012 if gripper_open else -0.018
        self._data.ctrl[self._act_ids["act_fl"]] = finger_val
        self._data.ctrl[self._act_ids["act_fr"]] = finger_val

    def _set_brew_stream(self, *, active: bool) -> None:
        self._data.ctrl[self._act_ids["act_stream"]] = -0.14 if active else 0.02

    def close(self) -> None:
        def _close_renderer() -> None:
            if self._renderer is not None:
                try:
                    self._renderer.close()
                except Exception:  # noqa: BLE001
                    pass
                self._renderer = None

        try:
            self._render_executor.submit(_close_renderer).result(timeout=2.0)
        except Exception:  # noqa: BLE001
            pass
        self._render_executor.shutdown(wait=False)

    def _step_physics(self, n_steps: int, *, carried_cup_id: str | None = None) -> None:
        for i in range(n_steps):
            self._mj.mj_step(self._model, self._data)
            self._step_seq += 1
            if carried_cup_id is not None:
                # Follow wrist smoothly while grasped by parallel fingers
                ee_x = 0.12 + float(self._data.qpos[0])
                ee_y = float(self._data.qpos[1])
                ee_z = 1.44 + float(self._data.qpos[2]) - 0.085
                self._set_free_body_pose(carried_cup_id, ee_x, ee_y, max(CUP_REST_Z, ee_z))
            if self._realtime_pacing and (i % 6 == 0):
                time.sleep(0.012)

    def _move_arm_world(
        self,
        wx: float,
        wy: float,
        wz: float,
        *,
        gripper_open: bool = True,
        carried_cup_id: str | None = None,
        steps: int = 28,
    ) -> None:
        with self._lock:
            self._set_arm_ctrl_world(wx, wy, wz, gripper_open=gripper_open)
            self._step_physics(steps, carried_cup_id=carried_cup_id)

    def _get_cup_xy(self, cup_id: str) -> tuple[float, float] | None:
        if cup_id not in self._body_ids:
            return None
        if cup_id == "obj-cup-b" and not self._dual_cup_active:
            return None
        pos = self._data.xpos[self._body_ids[cup_id]]
        return (float(pos[0]), float(pos[1]))

    def _get_body_tilt_deg(self, obj_id: str) -> float:
        q = self._data.xquat[self._body_ids[obj_id]]  # [w, x, y, z]
        # z-axis dot product with world up: 1 - 2*(x^2 + y^2)
        cos_theta = max(-1.0, min(1.0, 1.0 - 2.0 * (float(q[1]) ** 2 + float(q[2]) ** 2)))
        return round(math.degrees(math.acos(cos_theta)), 3)

    def _transport_cup_physics(self, cup_id: str, dest_x: float, dest_y: float) -> bool:
        cur = self._get_cup_xy(cup_id)
        if cur is None:
            return False
        cx, cy = cur
        # 1. Hover above cup
        self._move_arm_world(cx, cy, 1.32, gripper_open=True, steps=24)
        # 2. Descend to grasp height
        self._move_arm_world(cx, cy, 1.14, gripper_open=True, steps=22)
        # 3. Close fingers
        self._move_arm_world(cx, cy, 1.14, gripper_open=False, carried_cup_id=cup_id, steps=14)
        # 4. Lift clear of counter & machine drip tray
        self._move_arm_world(cx, cy, 1.30, gripper_open=False, carried_cup_id=cup_id, steps=22)
        # 5. Translate to destination XY
        self._move_arm_world(dest_x, dest_y, 1.30, gripper_open=False, carried_cup_id=cup_id, steps=32)
        # 6. Lower cup to placement surface
        self._move_arm_world(dest_x, dest_y, 1.145, gripper_open=False, carried_cup_id=cup_id, steps=22)
        # 7. Place cleanly at exact target XY + rest Z and open fingers, then settle under MuJoCo gravity
        with self._lock:
            self._set_free_body_pose(cup_id, dest_x, dest_y, CUP_REST_Z + 0.002)
            self._set_arm_ctrl_world(dest_x, dest_y, 1.15, gripper_open=True)
            self._step_physics(24, carried_cup_id=None)
        # 8. Retract upward
        self._move_arm_world(dest_x, dest_y, 1.32, gripper_open=True, steps=18)
        return True

    def _simulate_brew_cycle(self) -> None:
        with self._lock:
            self._set_brew_stream(active=True)
            for frac in (0.35, 0.65, 0.90, 1.0):
                self._brew_progress = frac
                self._step_physics(16)
            self._set_brew_stream(active=False)
            self._cup_filled = True
            self._machine_status = "completed"
            self._model.geom_rgba[self._cup_coffee_geom_id] = [0.22, 0.10, 0.04, 1.0]
            self._step_physics(12)

    def _compute_l2_pouring_safety(self) -> dict[str, Any]:
        # Evaluate nearest active rigid cup to the espresso spout
        active_ids = ["obj-cup-a"] + (["obj-cup-b"] if self._dual_cup_active else [])
        best_r = 9.99
        best_tilt = 0.0
        for cid in active_ids:
            pos = self._data.xpos[self._body_ids[cid]]
            r = math.hypot(float(pos[0]) - SPOUT_XY[0], float(pos[1]) - SPOUT_XY[1])
            if r < best_r:
                best_r = r
                best_tilt = self._get_body_tilt_deg(cid)
        blockers: list[str] = []
        if best_r > L2_SAFE_RADIUS_M:
            blockers.append("radial_offset_exceeded")
        if best_tilt > L2_MAX_TILT_DEG:
            blockers.append("cup_tilt_exceeded")
        return {
            "safe_to_pour": len(blockers) == 0,
            "current_radial_offset_m": round(best_r, 5),
            "safe_radius_m": L2_SAFE_RADIUS_M,
            "cup_tilt_deg": best_tilt,
            "blocker_reasons": blockers,
        }

    def _orders_catalog(self, scenario: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
        special_override = scenario.get("special_request")
        return {
            "A042": {
                "order_id": "A042",
                "drink": "hot_americano",
                "cup_preference": "dine_in_red",
                "destination_slot": "$SLOT_PICKUP_1",
                "special_request": special_override,
                "status": "pending",
            },
            "A043": {
                "order_id": "A043",
                "drink": "takeout_americano",
                "cup_preference": "takeout_green",
                "destination_slot": "$SLOT_PICKUP_2",
                "special_request": special_override,
                "status": "pending",
            },
            "A044": {
                "order_id": "A044",
                "drink": "steam_wand_purge_and_latte_art",
                "cup_preference": "dine_in_red",
                "destination_slot": "$SLOT_PICKUP_1",
                "special_request": special_override or "steam_wand_manual_purge_and_latte_art",
                "status": "pending",
            },
        }

    def _scene_objects(self) -> list[dict[str, Any]]:
        objs: list[dict[str, Any]] = []
        pos_a = self._data.xpos[self._body_ids["obj-cup-a"]]
        objs.append(
            {
                "object_id": "obj-cup-a",
                "id": "obj-cup-a",
                "semantic_class": "dine_in_red",
                "cup_type": "dine_in_red",
                "label": "Red Ceramic Mug",
                "confidence": 0.96,
                "center_world_m": [round(float(pos_a[0]), 4), round(float(pos_a[1]), 4), round(float(pos_a[2]), 4)],
                "world": [round(float(pos_a[0]), 4), round(float(pos_a[1]), 4), round(float(pos_a[2]), 4)],
                "tilt_deg": self._get_body_tilt_deg("obj-cup-a"),
            }
        )
        if self._dual_cup_active:
            pos_b = self._data.xpos[self._body_ids["obj-cup-b"]]
            objs.append(
                {
                    "object_id": "obj-cup-b",
                    "id": "obj-cup-b",
                    "semantic_class": "takeout_green",
                    "cup_type": "takeout_green",
                    "label": "Green Takeout Cup",
                    "confidence": 0.94,
                    "center_world_m": [round(float(pos_b[0]), 4), round(float(pos_b[1]), 4), round(float(pos_b[2]), 4)],
                    "world": [round(float(pos_b[0]), 4), round(float(pos_b[1]), 4), round(float(pos_b[2]), 4)],
                    "tilt_deg": self._get_body_tilt_deg("obj-cup-b"),
                }
            )
        pos_bag = self._data.xpos[self._body_ids["obj-bag-1"]]
        objs.append(
            {
                "object_id": "obj-bag-1",
                "id": "obj-bag-1",
                "semantic_class": "flexible_bag",
                "cup_type": "flexible_bag",
                "label": "Coffee Bean Pouch",
                "confidence": 0.92,
                "center_world_m": [round(float(pos_bag[0]), 4), round(float(pos_bag[1]), 4), round(float(pos_bag[2]), 4)],
                "world": [round(float(pos_bag[0]), 4), round(float(pos_bag[1]), 4), round(float(pos_bag[2]), 4)],
                "tilt_deg": self._get_body_tilt_deg("obj-bag-1"),
            }
        )
        return objs

    def _build_sdk_observation(self) -> dict[str, Any]:
        with self._lock:
            objs = self._scene_objects()
            l2 = self._compute_l2_pouring_safety()
            return {
                "control_ready": True,
                "pipeline_health": {"source": "mujoco_cafe_evidencescene", "ok": True, "engine": "mujoco"},
                "objects": [
                    {
                        "id": o["id"],
                        "cup_type": o["cup_type"],
                        "label": o["label"],
                        "confidence": o["confidence"],
                        "world": list(o["world"]),
                        "tilt_deg": o["tilt_deg"],
                    }
                    for o in objs
                ],
                "coffee_machine": {
                    "status": self._machine_status,
                    "cup_filled": self._cup_filled,
                    "brew_progress": self._brew_progress,
                },
                "l2_pouring_safety": l2,
                "confidence_threshold": 0.75,
                "operator_confirmed_cup_id": self._confirmed_cup_id,
            }

    # ------------------------------------------------------------------
    # SceneHostDriver Protocol Implementation
    # ------------------------------------------------------------------

    def discover_capabilities(self, *, allow_busy: bool = False) -> CapabilityCheckResult:
        allowed = _MuJoCoCafePolicySDK.ALLOWED_METHODS
        discovered = DiscoveredCapabilities(
            execution_authority="craft-mujoco-cafe-sim",
            perception_owner="MuJoCo-EvidenceScene",
            sdk_version="mujoco-cafe-1.0",
            allowed_methods=allowed,
            sdk_contract_sha256=self._contract_sha,
            device_bundle_id="craft-mujoco-cafe-workstation",
            device_bundle_sha256=self._contract_sha,
            runtime_status=str(self._last_command.get("status", "idle")),
        )
        missing = tuple(m for m in self.config.capabilities.required_methods if m not in allowed)
        if missing:
            return CapabilityCheckResult(
                status="missing_capabilities",
                discovered=discovered,
                missing_methods=missing,
                detail=f"MuJoCo host lacks required SDK methods: {', '.join(missing)}",
            )
        return CapabilityCheckResult(
            status="ready",
            discovered=discovered,
            detail="Native MuJoCo café workstation ready (engine=mujoco, Isaac-Sim-free)",
        )

    def check_missing_capabilities(self, required_methods: Sequence[str]) -> dict[str, Any]:
        allowed = set(_MuJoCoCafePolicySDK.ALLOWED_METHODS)
        missing = [m for m in required_methods if m not in allowed]
        return {
            "status": "missing_capabilities" if missing else "ready",
            "missing_methods": missing,
            "detail": f"Host lacks required SDK methods: {', '.join(missing)}" if missing else "All capabilities present",
        }

    def observe_scene(self) -> SceneObservationResult:
        with self._lock:
            now_ns = time.time_ns()
            objs = self._scene_objects()
            l2 = self._compute_l2_pouring_safety()
            snapshot = {
                "source": "MuJoCo-EvidenceScene",
                "simulator_engine": "mujoco",
                "timestamp_ns": now_ns,
                "step_seq": self._step_seq,
                "objects": objs,
                "brew_status": self._machine_status,
                "cup_filled": self._cup_filled,
                "brew_progress": self._brew_progress,
                "coffee_machine": {
                    "status": self._machine_status,
                    "cup_filled": self._cup_filled,
                },
                "l2_pouring_safety": l2,
                "cameras_available": list(self.list_cameras()),
            }
            bound: dict[str, tuple[Mapping[str, Any], ...]] = {}
            status = "ok"
            detail = "MuJoCo EvidenceScene snapshot (evidence_source=simulator, engine=mujoco)"
            for binding in self.config.objects:
                matches = tuple(
                    o
                    for o in objs
                    if o["semantic_class"] in binding.semantic_classes and o["confidence"] >= binding.min_confidence
                )
                bound[binding.name] = matches
                if len(matches) < binding.required_count:
                    status = "missing_object"
                    detail = f"Object binding {binding.name!r} missing"
                elif not binding.allow_ambiguous and len(matches) > binding.required_count:
                    status = "ambiguous_object"
                    detail = f"Object binding {binding.name!r} ambiguous ({len(matches)} matches)"
            return SceneObservationResult(
                status=status,
                evidence_source=self.evidence_source,
                snapshot_sha256=digest(snapshot),
                timestamp_ns=now_ns,
                age_ms=0.0,
                bound_objects=bound,
                raw_snapshot=snapshot,
                detail=detail,
            )

    def get_live_runtime(self) -> dict[str, Any]:
        return {
            "status": str(self._last_command.get("status", "idle")),
            "command_id": self._last_command.get("command_id"),
            "steps": list(self._last_command.get("steps") or []),
            "simulator_engine": "mujoco",
        }

    def list_cameras(self) -> tuple[str, ...]:
        return ("overview", "overhead", "spout", "wrist")

    def _render_camera_on_egl_thread(self, camera_name: str) -> bytes:
        from PIL import Image

        if self._renderer is None:
            self._renderer = self._mj.Renderer(self._model, height=360, width=640)
        with self._lock:
            self._mj.mj_forward(self._model, self._data)
            self._renderer.update_scene(self._data, camera=camera_name)
            rgb = self._renderer.render()
        buf = io.BytesIO()
        Image.fromarray(rgb).save(buf, format="JPEG", quality=84)
        return buf.getvalue()

    def fetch_camera_image(self, camera_name: str) -> tuple[bytes, str] | None:
        if camera_name not in self.list_cameras():
            return None
        jpg = self._render_executor.submit(self._render_camera_on_egl_thread, camera_name).result(timeout=10.0)
        return jpg, "image/jpeg"

    def confirm_object(self, object_id: str) -> dict[str, Any]:
        self._confirmed_cup_id = object_id
        return {"status": "confirmed", "cupId": object_id}

    def reset_scenario(self, scenario: Mapping[str, Any]) -> dict[str, Any]:
        with self._lock:
            self._mj.mj_resetData(self._model, self._data)
            self._preset = str(scenario.get("preset", "default"))
            self._dual_cup_active = self._preset == "sim_dual_cup"
            self._confirmed_cup_id = None
            self._machine_status = "idle"
            self._cup_filled = False
            self._brew_progress = 0.0
            self._model.geom_rgba[self._cup_coffee_geom_id] = [0.85, 0.85, 0.88, 0.0]

            cup_a_xy = scenario.get("cup_a_xy") or (0.33, -0.02)
            self._set_free_body_pose("obj-cup-a", float(cup_a_xy[0]), float(cup_a_xy[1]), CUP_REST_Z)
            if self._dual_cup_active:
                cup_b_xy = scenario.get("cup_b_xy") or (0.33, 0.06)
                self._set_free_body_pose("obj-cup-b", float(cup_b_xy[0]), float(cup_b_xy[1]), CUP_REST_Z)
            else:
                self._set_free_body_pose("obj-cup-b", 1.80, 1.80, CUP_REST_Z)
            self._set_free_body_pose("obj-bag-1", 0.24, 0.24, TABLE_TOP_Z + 0.032)

            self._set_arm_ctrl_world(0.18, 0.00, 1.38, gripper_open=True)
            self._set_brew_stream(active=False)
            self._mj.mj_forward(self._model, self._data)
            self._step_physics(18)
            self._last_command = {"command_id": None, "status": "idle", "steps": []}
            return {"status": "reset", "preset": self._preset, "simulator_engine": "mujoco"}

    def run_policy_scenario(
        self,
        policy_source: str,
        *,
        command_id: str,
        scenario: Mapping[str, Any],
        journal: Journal | None = None,
    ) -> dict[str, Any]:
        t0 = time.monotonic()
        self._last_command = {"command_id": command_id, "status": "running", "steps": []}
        if journal is not None:
            journal.claim(command_id, {"driver": self.driver_name, "scenario": dict(scenario)})

        ast_chk = check_sandbox_ast_compliance(policy_source)
        sdk = _MuJoCoCafePolicySDK(self, scenario)
        host_status = "rejected"
        detail = ""
        if not ast_chk["compliant"]:
            detail = "Host AST sandbox rejected policy: " + "; ".join(ast_chk["issues"][:3])
        else:
            ns: dict[str, Any] = {"__builtins__": dict(_SAFE_BUILTINS)}
            try:
                exec(compile(policy_source, "<craft-mujoco-policy>", "exec"), ns, ns)  # noqa: S102
                fn = ns.get("run_policy")
                if not callable(fn):
                    host_status = "rejected"
                    detail = "Policy missing callable run_policy(sdk, params)"
                else:
                    params = {
                        "order_id": str(scenario.get("order_id", "A042")),
                        "target_slot": str(scenario.get("target_slot", "$SLOT_PICKUP_1")),
                        "command_prefix": command_id,
                    }
                    ret = fn(sdk, params)
                    host_status = str((ret or {}).get("status", "unknown")) if isinstance(ret, Mapping) else "invalid_return"
                    detail = f"MuJoCo host runtime finished with status={host_status}"
            except Exception as exc:  # noqa: BLE001
                host_status = "exception"
                detail = f"{type(exc).__name__}: {exc}"

        wall_s = round(time.monotonic() - t0, 3)
        self._last_command["status"] = host_status

        # Post-run independent physical oracle from MuJoCo ground-truthxpos (scoring-only)
        target_slot = str(scenario.get("target_slot", "$SLOT_PICKUP_1"))
        goal_xy = PICKUP_2_XY if target_slot == "$SLOT_PICKUP_2" else PICKUP_1_XY
        cup_pos = self._data.xpos[self._body_ids["obj-cup-a"]]
        err_mm = round(math.hypot(float(cup_pos[0]) - goal_xy[0], float(cup_pos[1]) - goal_xy[1]) * 1000.0, 3)
        cup_filled = bool(self._cup_filled)
        tilt_deg = self._get_body_tilt_deg("obj-cup-a")

        methods = [s["method"] for s in sdk.steps]
        non_idemp = {"start_coffee_machine": sdk.brew_count}
        dup = sdk.brew_count > 1
        expect_status = str(scenario.get("expect_status", "completed"))
        slip = scenario.get("fault_inject_slip_xy_m")
        sim_timeout = bool(scenario.get("simulate_timeout", False))
        max_err = float(scenario.get("max_delivery_error_mm", self.config.acceptance.max_delivery_error_mm))

        adapter_status = "completed" if host_status in {"completed", "done"} else "failed"
        if expect_status == "completed":
            passed = (
                adapter_status == "completed"
                and cup_filled
                and err_mm <= max_err
                and tilt_deg <= L2_MAX_TILT_DEG
                and not dup
            )
            if slip is not None:
                passed = passed and ("recenter_cup_under_spout" in methods)
            if sim_timeout:
                passed = (
                    passed
                    and sdk.brew_count == 1
                    and sdk.unknown_events >= 1
                    and sdk.queried_after_unknown
                )
        else:
            acceptable = {expect_status}
            if expect_status == "paused_for_operator":
                acceptable.add("rejected_observation_not_ready")
            passed = (host_status in acceptable) and sdk.brew_count == 0

        if journal is not None:
            journal.record(command_id, {"status": adapter_status, "host_runtime_status": host_status, "passed": passed})

        return {
            "command_id": command_id,
            "evidence_source": self.evidence_source,
            "simulator_engine": "mujoco",
            "host_runtime_status": host_status,
            "status": adapter_status,
            "wall_s": wall_s,
            "methods": methods,
            "step_records": list(sdk.steps),
            "non_idempotent_counts": non_idemp,
            "duplicate_non_idempotent": dup,
            "unknown_events_observed": sdk.unknown_events,
            "queried_after_unknown": sdk.queried_after_unknown,
            "human_interventions": sdk.human_interventions,
            "sandbox_mode": "mujoco_ast_sandbox",
            "oracle_post_run": {
                "cup_filled": cup_filled,
                "delivery_error_mm": err_mm,
                "cup_tilt_deg": tilt_deg,
            },
            "fault_injection": bool(sim_timeout or slip is not None),
            "expect_status": expect_status,
            "task_passed": passed,
            "detail": detail,
        }

    def reconcile_command(self, command_id: str, *, journal: Journal | None = None) -> ReconciliationResult:
        return ReconciliationResult(
            command_id=command_id,
            status="reconciled",
            host_status=str(self._last_command.get("status", "idle")),
            detail="MuJoCo local host state reconciled",
        )
