"""Native MuJoCo café workstation simulator & SceneHostDriver (`driver="mujoco_cafe"`).

Provides a 100% open-source, self-contained, Isaac-Sim-free MuJoCo physics
simulation of the single-cup café workstation for TRAX:
- Articulated 3-axis + yaw + parallel-jaw robotic manipulator with MuJoCo position actuators
- Real MuJoCo rigid-body physics for cups (`dine_in_red`, `takeout_green`) and `flexible_bag` distractor
- Espresso machine housing, brew spout, drip tray, animated brew stream, Pickup Slots 1/2,
  and Forbidden Steam Zone (`$ZONE_STEAM_NOGO`)
- Hardware-accelerated headless EGL (or OSMesa fallback) multi-camera rendering
  (`overview`, `overhead`, `spout`, `wrist`) for TRAX Studio
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
import importlib.util
import io
import math
import os
from pathlib import Path
import random
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

PANDA_HOME_QPOS = (0.0, -0.45, 0.0, -2.15, 0.0, 1.70, 0.785)


def _discover_robosuite_assets_dir() -> Path | None:
    """Locate optional robosuite visual assets dynamically without hardcoding private paths."""
    spec = importlib.util.find_spec("robosuite")
    if spec is None or not spec.submodule_search_locations:
        return None
    base = Path(list(spec.submodule_search_locations)[0]) / "models" / "assets"
    required = [
        base / "robots" / "panda" / "obj_meshes" / "link1_vis" / "link1_vis.obj",
        base / "grippers" / "meshes" / "panda_gripper" / "hand_vis.stl",
        base / "textures" / "dark-wood.png",
        base / "textures" / "steel-brushed.png",
    ]
    return base if all(p.is_file() for p in required) else None


def _build_high_fidelity_cafe_mjcf(assets_dir: Path) -> str:
    """Build high-fidelity 7-DOF Franka Panda + PBR textured commercial café workstation MJCF."""
    panda_obj = assets_dir / "robots" / "panda" / "obj_meshes"
    grip_mesh = assets_dir / "grippers" / "meshes" / "panda_gripper"
    tex_dir = assets_dir / "textures"

    def _p(path: Path) -> str:
        return path.as_posix()

    link0_meshes = "\n".join(
        f'    <mesh name="link0_vis_{i}" file="{_p(panda_obj / "link0_vis" / f"link0_vis_{i}.obj")}"/>'
        for i in range(12)
    )
    link3_meshes = "\n".join(
        f'    <mesh name="link3_vis_{i}" file="{_p(panda_obj / "link3_vis" / f"link3_vis_{i}.obj")}"/>'
        for i in range(4)
    )
    link4_meshes = "\n".join(
        f'    <mesh name="link4_vis_{i}" file="{_p(panda_obj / "link4_vis" / f"link4_vis_{i}.obj")}"/>'
        for i in range(4)
    )
    link5_meshes = "\n".join(
        f'    <mesh name="link5_vis_{i}" file="{_p(panda_obj / "link5_vis" / f"link5_vis_{i}.obj")}"/>'
        for i in range(3)
    )
    link6_meshes = "\n".join(
        f'    <mesh name="link6_vis_{i}" file="{_p(panda_obj / "link6_vis" / f"link6_vis_{i}.obj")}"/>'
        for i in range(17)
    )
    link7_meshes = "\n".join(
        f'    <mesh name="link7_vis_{i}" file="{_p(panda_obj / "link7_vis" / f"link7_vis_{i}.obj")}"/>'
        for i in range(8)
    )
    link0_geoms = "\n".join(
        f'          <geom mesh="link0_vis_{i}" material="{"panda_dark" if i in (1, 3, 5, 9) else "panda_white"}" type="mesh" contype="0" conaffinity="0"/>'
        for i in range(12)
    )
    link3_geoms = "\n".join(
        f'                <geom mesh="link3_vis_{i}" material="{"panda_dark" if i == 3 else "panda_white"}" type="mesh" contype="0" conaffinity="0"/>'
        for i in range(4)
    )
    link4_geoms = "\n".join(
        f'                  <geom mesh="link4_vis_{i}" material="{"panda_dark" if i == 2 else "panda_white"}" type="mesh" contype="0" conaffinity="0"/>'
        for i in range(4)
    )
    link5_geoms = "\n".join(
        f'                    <geom mesh="link5_vis_{i}" material="{"panda_dark" if i == 0 else "panda_white"}" type="mesh" contype="0" conaffinity="0"/>'
        for i in range(3)
    )
    link6_geoms = "\n".join(
        f'                      <geom mesh="link6_vis_{i}" material="{"panda_blue" if i in (7, 8) else ("panda_dark" if i in (2, 9, 10, 14, 15) else "panda_white")}" type="mesh" contype="0" conaffinity="0"/>'
        for i in range(17)
    )
    link7_geoms = "\n".join(
        f'                        <geom mesh="link7_vis_{i}" material="{"panda_white" if i in (0, 7) else "panda_dark"}" type="mesh" contype="0" conaffinity="0"/>'
        for i in range(8)
    )

    return f"""<mujoco model="craft_cafe_panda_pbr">
  <compiler angle="radian" coordinate="local" inertiafromgeom="true"/>
  <option timestep="0.004" gravity="0 0 -9.81" integrator="implicitfast"/>

  <visual>
    <global offwidth="800" offheight="450" azimuth="140" elevation="-22"/>
    <headlight diffuse="0.52 0.50 0.48" ambient="0.28 0.27 0.26" specular="0.25 0.25 0.25"/>
    <rgba haze="0.10 0.11 0.13 1"/>
    <quality shadowsize="4096" offsamples="4"/>
  </visual>

  <asset>
    <texture name="sky_tex" type="skybox" builtin="gradient" rgb1="0.22 0.25 0.30" rgb2="0.09 0.10 0.12" width="256" height="256"/>
    <texture name="floor_tex" type="2d" file="{_p(tex_dir / "light-gray-floor-tile.png")}"/>
    <texture name="wood_tex" type="2d" file="{_p(tex_dir / "dark-wood.png")}"/>
    <texture name="wall_tex" type="2d" file="{_p(tex_dir / "white-bricks.png")}"/>
    <texture name="steel_tex" type="2d" file="{_p(tex_dir / "steel-brushed.png")}"/>
    <texture name="scratched_tex" type="2d" file="{_p(tex_dir / "steel-scratched.png")}"/>
    <texture name="brass_tex" type="2d" file="{_p(tex_dir / "brass-ambra.png")}"/>
    <texture name="ceramic_tex" type="2d" file="{_p(tex_dir / "ceramic.png")}"/>
    <texture name="felt_tex" type="2d" file="{_p(tex_dir / "gray-felt.png")}"/>

    <material name="floor_mat" texture="floor_tex" texuniform="true" texrepeat="2 2" reflectance="0.06" specular="0.18" shininess="0.25"/>
    <material name="counter_wood_mat" texture="wood_tex" texuniform="true" texrepeat="1.5 1.5" rgba="0.80 0.65 0.52 1" specular="0.28" shininess="0.38" reflectance="0.05"/>
    <material name="wall_mat" texture="wall_tex" texuniform="true" texrepeat="2 2" rgba="0.90 0.88 0.85 1" specular="0.06" shininess="0.08"/>
    <material name="steel_brushed_mat" texture="steel_tex" texuniform="true" texrepeat="2 2" rgba="0.72 0.75 0.78 1" specular="0.55" shininess="0.65" reflectance="0.14"/>
    <material name="steel_dark_mat" texture="scratched_tex" texuniform="true" texrepeat="2 2" rgba="0.20 0.22 0.25 1" specular="0.45" shininess="0.50" reflectance="0.08"/>
    <material name="brass_mat" texture="brass_tex" texuniform="true" texrepeat="2 2" rgba="0.88 0.72 0.38 1" specular="0.75" shininess="0.80" reflectance="0.18"/>
    <material name="barmat_mat" texture="felt_tex" texuniform="true" texrepeat="4 4" rgba="0.16 0.17 0.19 1" specular="0.04" shininess="0.06"/>
    <material name="panda_white" rgba="0.95 0.96 0.97 1" specular="0.55" shininess="0.55"/>
    <material name="panda_dark" rgba="0.20 0.22 0.25 1" specular="0.45" shininess="0.50"/>
    <material name="panda_blue" rgba="0.05 0.52 0.82 1" specular="0.65" shininess="0.70"/>
    <material name="cup_red_mat" texture="ceramic_tex" rgba="0.86 0.18 0.22 1" specular="0.75" shininess="0.85" reflectance="0.12"/>
    <material name="cup_white_ceramic" texture="ceramic_tex" rgba="0.96 0.95 0.93 1" specular="0.70" shininess="0.80"/>
    <material name="cup_green_mat" texture="ceramic_tex" rgba="0.12 0.62 0.40 1" specular="0.35" shininess="0.40"/>
    <material name="bag_mat" texture="wood_tex" rgba="0.72 0.58 0.42 1" specular="0.08" shininess="0.10"/>
    <material name="slot1_mat" rgba="0.12 0.72 0.95 0.75" specular="0.3"/>
    <material name="slot2_mat" rgba="0.96 0.68 0.14 0.75" specular="0.3"/>
    <material name="nogo_mat" rgba="0.92 0.18 0.16 0.72" specular="0.2"/>
    <material name="coffee_mat" rgba="0.20 0.10 0.04 1" specular="0.7" shininess="0.9" reflectance="0.15"/>
    <material name="crema_mat" rgba="0.52 0.31 0.14 1" specular="0.5" shininess="0.6"/>
    <material name="gauge_face" rgba="0.96 0.95 0.91 1" specular="0.3"/>

{link0_meshes}
    <mesh name="link1_vis" file="{_p(panda_obj / "link1_vis" / "link1_vis.obj")}"/>
    <mesh name="link2_vis" file="{_p(panda_obj / "link2_vis" / "link2_vis.obj")}"/>
{link3_meshes}
{link4_meshes}
{link5_meshes}
{link6_meshes}
{link7_meshes}
    <mesh name="hand_vis" file="{_p(grip_mesh / "hand_vis.stl")}"/>
    <mesh name="finger_vis" file="{_p(grip_mesh / "finger_vis.stl")}"/>
  </asset>

  <worldbody>
    <!-- 3-Point Warm Café Studio Lighting -->
    <light pos="0.10 -0.85 2.45" dir="0.20 0.45 -1.0" diffuse="0.92 0.86 0.78" specular="0.45 0.43 0.40" castshadow="true" cutoff="65"/>
    <light pos="0.60 0.80 2.25" dir="-0.15 -0.35 -1.0" diffuse="0.42 0.48 0.56" specular="0.20 0.22 0.25" castshadow="false"/>
    <light pos="-0.45 0.05 1.95" dir="0.55 0.0 -0.65" diffuse="0.36 0.34 0.32" specular="0.25 0.25 0.25" castshadow="false"/>

    <!-- Unobstructed HD Studio Cameras -->
    <camera name="overview" pos="-0.42 -0.94 1.72" xyaxes="0.82 -0.57 0 0.24 0.34 0.91" fovy="46"/>
    <camera name="overhead" pos="0.33 0.00 2.02" xyaxes="0 1 0 -1 0 0" fovy="48"/>
    <camera name="spout" pos="0.12 -0.38 1.24" xyaxes="0.745 -0.667 0 0.141 0.157 0.977" fovy="38"/>

    <!-- Café Floor & Back Brick Wall -->
    <geom name="floor" type="plane" size="4 4 0.1" material="floor_mat"/>
    <geom name="back_wall" type="box" pos="1.02 0.0 1.65" size="0.04 2.2 1.65" material="wall_mat" contype="0" conaffinity="0"/>
    <geom name="side_wall" type="box" pos="0.15 1.15 1.65" size="1.4 0.04 1.65" material="wall_mat" contype="0" conaffinity="0"/>

    <!-- Walnut Café Bar Counter + Stainless Trim + Bar Mat -->
    <body name="counter" pos="0.40 0.0 0.50">
      <geom name="table_top" type="box" size="0.44 0.48 0.50" material="counter_wood_mat" friction="0.9 0.01 0.001"/>
      <geom name="counter_trim" type="box" pos="-0.442 0 0.47" size="0.004 0.482 0.025" material="steel_brushed_mat" contype="0" conaffinity="0"/>
      <geom name="bar_mat" type="box" pos="-0.10 0.0 0.5006" size="0.15 0.24 0.0006" material="barmat_mat" contype="0" conaffinity="0"/>
    </body>

    <!-- Decorative Café Shelf Props (outside active manipulation envelope) -->
    <body name="cafe_props" pos="0.92 -0.36 1.48">
      <geom name="wall_shelf" type="box" pos="0 0.36 0" size="0.09 0.58 0.012" material="counter_wood_mat" contype="0" conaffinity="0"/>
      <geom name="prop_mug1" type="cylinder" pos="-0.02 0.12 0.055" size="0.032 0.045" material="cup_white_ceramic" contype="0" conaffinity="0"/>
      <geom name="prop_mug2" type="cylinder" pos="-0.02 0.22 0.055" size="0.032 0.045" material="cup_white_ceramic" contype="0" conaffinity="0"/>
      <geom name="prop_pitcher" type="cylinder" pos="-0.02 0.56 0.068" size="0.038 0.058" material="steel_brushed_mat" contype="0" conaffinity="0"/>
      <geom name="prop_tamper_base" type="cylinder" pos="-0.38 0.04 -0.468" size="0.026 0.012" material="steel_brushed_mat" contype="0" conaffinity="0"/>
      <geom name="prop_tamper_handle" type="capsule" pos="-0.38 0.04 -0.435" size="0.014 0.026" material="counter_wood_mat" contype="0" conaffinity="0"/>
    </body>

    <!-- Pickup Slots & Forbidden Steam Zone Markers -->
    <geom name="slot_pickup_1" type="cylinder" pos="0.38 -0.17 1.001" size="0.054 0.0012" material="slot1_mat" contype="0" conaffinity="0"/>
    <geom name="slot_pickup_1_inner" type="cylinder" pos="0.38 -0.17 1.0016" size="0.044 0.0008" material="counter_wood_mat" contype="0" conaffinity="0"/>
    <geom name="slot_pickup_2" type="cylinder" pos="0.38 0.17 1.001" size="0.054 0.0012" material="slot2_mat" contype="0" conaffinity="0"/>
    <geom name="slot_pickup_2_inner" type="cylinder" pos="0.38 0.17 1.0016" size="0.044 0.0008" material="counter_wood_mat" contype="0" conaffinity="0"/>
    <geom name="zone_steam_nogo" type="box" pos="0.56 0.22 1.001" size="0.075 0.075 0.0012" material="nogo_mat" contype="0" conaffinity="0"/>

    <!-- Detailed Commercial Espresso Machine -->
    <body name="espresso_machine" pos="0.58 0.0 1.00">
      <geom name="machine_base" type="box" pos="0.045 0 0.155" size="0.11 0.145 0.155" material="steel_brushed_mat"/>
      <geom name="machine_side_l" type="box" pos="0.045 -0.148 0.165" size="0.112 0.006 0.165" material="steel_dark_mat" contype="0" conaffinity="0"/>
      <geom name="machine_side_r" type="box" pos="0.045 0.148 0.165" size="0.112 0.006 0.165" material="steel_dark_mat" contype="0" conaffinity="0"/>
      <geom name="machine_top" type="box" pos="-0.015 0 0.345" size="0.172 0.148 0.042" material="steel_dark_mat"/>
      <geom name="warmer_rail_l" type="capsule" fromto="-0.16 -0.13 0.40 0.13 -0.13 0.40" size="0.005" material="steel_brushed_mat" contype="0" conaffinity="0"/>
      <geom name="warmer_rail_r" type="capsule" fromto="-0.16 0.13 0.40 0.13 0.13 0.40" size="0.005" material="steel_brushed_mat" contype="0" conaffinity="0"/>
      <geom name="warmer_rail_b" type="capsule" fromto="0.13 -0.13 0.40 0.13 0.13 0.40" size="0.005" material="steel_brushed_mat" contype="0" conaffinity="0"/>
      <geom name="machine_badge" type="box" pos="-0.188 0 0.348" size="0.003 0.048 0.014" material="panda_blue" contype="0" conaffinity="0"/>
      <geom name="pressure_bezel" type="cylinder" pos="-0.068 -0.075 0.245" quat="0.7071 0 0.7071 0" size="0.022 0.006" material="brass_mat" contype="0" conaffinity="0"/>
      <geom name="pressure_dial" type="cylinder" pos="-0.072 -0.075 0.245" quat="0.7071 0 0.7071 0" size="0.018 0.003" material="gauge_face" contype="0" conaffinity="0"/>
      <geom name="group_head" type="cylinder" pos="-0.13 0 0.285" size="0.036 0.024" material="brass_mat" contype="0" conaffinity="0"/>
      <geom name="portafilter_cup" type="cylinder" pos="-0.13 0 0.256" size="0.028 0.016" material="steel_brushed_mat" contype="0" conaffinity="0"/>
      <geom name="portafilter_handle" type="capsule" fromto="-0.155 0 0.256 -0.225 -0.045 0.245" size="0.009" material="steel_dark_mat" contype="0" conaffinity="0"/>
      <geom name="brew_spout" type="cylinder" pos="-0.13 0 0.232" size="0.011 0.016" material="steel_brushed_mat" contype="0" conaffinity="0"/>
      <geom name="drip_tray" type="box" pos="-0.13 0 0.004" size="0.078 0.105 0.004" material="steel_dark_mat"/>
      <geom name="drip_grate" type="box" pos="-0.13 0 0.0085" size="0.072 0.098 0.001" material="steel_dark_mat" contype="0" conaffinity="0"/>
      <geom name="l2_safe_ring" type="cylinder" pos="-0.13 0 0.0098" size="0.030 0.0006" rgba="0.16 0.86 0.48 0.60" contype="0" conaffinity="0"/>
      <geom name="steam_wand" type="capsule" fromto="-0.05 0.11 0.265 -0.015 0.21 0.135" size="0.0065" material="steel_brushed_mat" contype="0" conaffinity="0"/>
      <geom name="steam_tip" type="cylinder" pos="-0.015 0.21 0.130" size="0.009 0.008" material="brass_mat" contype="0" conaffinity="0"/>
    </body>

    <!-- Animated Espresso Extraction Stream -->
    <body name="brew_stream" pos="0.45 0.0 1.29">
      <joint name="stream_slide" type="slide" axis="0 0 1" range="-0.13 0.02" damping="10"/>
      <geom name="stream_geom" type="cylinder" size="0.0045 0.048" material="crema_mat" contype="0" conaffinity="0"/>
    </body>

    <!-- Real 7-DOF Franka Emika Panda + Parallel Gripper on Steel Pedestal -->
    <body name="panda_pedestal" pos="-0.18 0.0 0.48">
      <geom name="pedestal_col" type="cylinder" size="0.105 0.48" material="steel_dark_mat" contype="0" conaffinity="0"/>
      <geom name="pedestal_plate" type="cylinder" pos="0 0 0.48" size="0.125 0.012" material="steel_brushed_mat" contype="0" conaffinity="0"/>
      <body name="link0" pos="0 0 0.49">
        <inertial pos="0 0 0.05" mass="4" diaginertia="0.4 0.4 0.4"/>
{link0_geoms}
        <body name="link1" pos="0 0 0.333">
          <inertial pos="0 0 -0.07" mass="3" diaginertia="0.3 0.3 0.3"/>
          <joint name="panda_j1" pos="0 0 0" axis="0 0 1" limited="true" range="-2.8973 2.8973" damping="12"/>
          <geom mesh="link1_vis" material="panda_white" type="mesh" contype="0" conaffinity="0"/>
          <body name="link2" pos="0 0 0" quat="0.707107 -0.707107 0 0">
            <inertial pos="0 -0.1 0" mass="3" diaginertia="0.3 0.3 0.3"/>
            <joint name="panda_j2" pos="0 0 0" axis="0 0 1" limited="true" range="-1.7628 1.7628" damping="12"/>
            <geom mesh="link2_vis" material="panda_white" type="mesh" contype="0" conaffinity="0"/>
            <body name="link3" pos="0 -0.316 0" quat="0.707107 0.707107 0 0">
              <inertial pos="0.04 0 -0.05" mass="2" diaginertia="0.2 0.2 0.2"/>
              <joint name="panda_j3" pos="0 0 0" axis="0 0 1" limited="true" range="-2.8973 2.8973" damping="10"/>
{link3_geoms}
              <body name="link4" pos="0.0825 0 0" quat="0.707107 0.707107 0 0">
                <inertial pos="-0.04 0.05 0" mass="2" diaginertia="0.2 0.2 0.2"/>
                <joint name="panda_j4" pos="0 0 0" axis="0 0 1" limited="true" range="-3.0718 -0.0698" damping="10"/>
{link4_geoms}
                <body name="link5" pos="-0.0825 0.384 0" quat="0.707107 -0.707107 0 0">
                  <inertial pos="0 0 -0.15" mass="2" diaginertia="0.2 0.2 0.2"/>
                  <joint name="panda_j5" pos="0 0 0" axis="0 0 1" limited="true" range="-2.8973 2.8973" damping="8"/>
{link5_geoms}
                  <body name="link6" pos="0 0 0" quat="0.707107 0.707107 0 0">
                    <inertial pos="0.06 0 0" mass="1.5" diaginertia="0.1 0.1 0.1"/>
                    <joint name="panda_j6" pos="0 0 0" axis="0 0 1" limited="true" range="-0.0175 3.7525" damping="6"/>
{link6_geoms}
                    <body name="link7" pos="0.088 0 0" quat="0.707107 0.707107 0 0">
                      <inertial pos="0 0 0.08" mass="0.5" diaginertia="0.05 0.05 0.05"/>
                      <joint name="panda_j7" pos="0 0 0" axis="0 0 1" limited="true" range="-2.8973 2.8973" damping="4"/>
{link7_geoms}
                      <body name="right_hand" pos="0 0 0.1065" quat="0.92388 0 0 -0.38268">
                        <inertial pos="0 0 0.05" mass="0.6" diaginertia="0.05 0.05 0.05"/>
                        <camera name="wrist" pos="0.06 0.0 -0.01" quat="0 0.707107 0.707107 0" fovy="62"/>
                        <geom name="hand_visual" type="mesh" mesh="hand_vis" material="panda_white" contype="0" conaffinity="0"/>
                        <site name="ee_tcp" pos="0 0 0.102" size="0.005" rgba="0.1 0.8 0.3 0.0"/>
                        <body name="leftfinger" pos="0 0 0.0584">
                          <inertial pos="0 0 0.025" mass="0.05" diaginertia="0.005 0.005 0.005"/>
                          <joint name="finger_l" type="slide" axis="0 1 0" limited="true" range="0.0 0.042" damping="12"/>
                          <geom name="finger_l_geom" type="mesh" mesh="finger_vis" material="panda_dark" contype="0" conaffinity="0"/>
                        </body>
                        <body name="rightfinger" pos="0 0 0.0584" quat="0 0 0 1">
                          <inertial pos="0 0 0.025" mass="0.05" diaginertia="0.005 0.005 0.005"/>
                          <joint name="finger_r" type="slide" axis="0 1 0" limited="true" range="0.0 0.042" damping="12"/>
                          <geom name="finger_r_geom" type="mesh" mesh="finger_vis" material="panda_dark" contype="0" conaffinity="0"/>
                        </body>
                      </body>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>

    <!-- Rigid Ceramic Mug A (dine_in_red) with Handle & Crema -->
    <body name="cup_a" pos="0.33 -0.02 1.055">
      <freejoint name="cup_a_free"/>
      <geom name="cup_a_body" type="cylinder" size="0.034 0.055" mass="0.22" material="cup_red_mat" friction="0.9 0.01 0.001"/>
      <geom name="cup_a_rim" type="cylinder" pos="0 0 0.053" size="0.0355 0.003" mass="0.01" material="cup_white_ceramic" contype="0" conaffinity="0"/>
      <geom name="cup_a_handle_top" type="capsule" fromto="-0.033 0 0.024 -0.052 0 0.024" size="0.006" mass="0.005" material="cup_red_mat" contype="0" conaffinity="0"/>
      <geom name="cup_a_handle_mid" type="capsule" fromto="-0.052 0 0.024 -0.052 0 -0.020" size="0.006" mass="0.005" material="cup_red_mat" contype="0" conaffinity="0"/>
      <geom name="cup_a_handle_bot" type="capsule" fromto="-0.052 0 -0.020 -0.033 0 -0.020" size="0.006" mass="0.005" material="cup_red_mat" contype="0" conaffinity="0"/>
      <geom name="cup_a_coffee" type="cylinder" pos="0 0 0.047" size="0.030 0.002" mass="0.005" material="coffee_mat" contype="0" conaffinity="0"/>
    </body>

    <!-- Rigid Takeout Cup B (takeout_green) with Sleeve & Lid -->
    <body name="cup_b" pos="1.80 1.80 1.055">
      <freejoint name="cup_b_free"/>
      <geom name="cup_b_body" type="cylinder" size="0.033 0.055" mass="0.18" material="cup_green_mat" friction="0.9 0.01 0.001"/>
      <geom name="cup_b_sleeve" type="cylinder" pos="0 0 -0.005" size="0.0345 0.024" mass="0.005" material="bag_mat" contype="0" conaffinity="0"/>
      <geom name="cup_b_lid" type="cylinder" pos="0 0 0.055" size="0.0365 0.006" mass="0.01" material="cup_white_ceramic" contype="0" conaffinity="0"/>
    </body>

    <!-- Distractor Kraft Coffee-Bean Bag -->
    <body name="bag_distractor" pos="0.24 0.24 1.032">
      <freejoint name="bag_free"/>
      <geom name="bag_geom" type="box" size="0.050 0.034 0.032" mass="0.12" material="bag_mat" friction="0.85 0.01 0.001"/>
      <geom name="bag_label" type="box" pos="-0.051 0 0" size="0.001 0.022 0.016" material="cup_white_ceramic" contype="0" conaffinity="0"/>
    </body>
  </worldbody>

  <actuator>
    <position name="act_j1" joint="panda_j1" kp="360" ctrlrange="-2.8973 2.8973"/>
    <position name="act_j2" joint="panda_j2" kp="360" ctrlrange="-1.7628 1.7628"/>
    <position name="act_j3" joint="panda_j3" kp="320" ctrlrange="-2.8973 2.8973"/>
    <position name="act_j4" joint="panda_j4" kp="320" ctrlrange="-3.0718 -0.0698"/>
    <position name="act_j5" joint="panda_j5" kp="220" ctrlrange="-2.8973 2.8973"/>
    <position name="act_j6" joint="panda_j6" kp="220" ctrlrange="-0.0175 3.7525"/>
    <position name="act_j7" joint="panda_j7" kp="160" ctrlrange="-2.8973 2.8973"/>
    <position name="act_fl" joint="finger_l" kp="220" ctrlrange="0.0 0.042"/>
    <position name="act_fr" joint="finger_r" kp="220" ctrlrange="0.0 0.042"/>
    <position name="act_stream" joint="stream_slide" kp="120" ctrlrange="-0.13 0.02"/>
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
        self.brew_count = int(driver._persistent_brew_count) if scenario.get("resume_after_crash") else 0
        self.unknown_events = int(driver._persistent_unknown_events) if scenario.get("resume_after_crash") else 0
        self.queried_after_unknown = (
            bool(driver._persistent_queried_after_unknown) if scenario.get("resume_after_crash") else True
        )
        self.reconciled_after_unknown = (
            bool(driver._persistent_reconciled_after_unknown) if scenario.get("resume_after_crash") else True
        )
        self._pending_unknown = bool(driver._persistent_pending_unknown) if scenario.get("resume_after_crash") else False
        self._command_receipts: dict[str, dict[str, Any]] = (
            dict(driver._host_command_ledger) if scenario.get("resume_after_crash") else {}
        )
        self.human_interventions = 0
        self._slip = scenario.get("fault_inject_slip_xy_m")
        self._sim_timeout = bool(scenario.get("simulate_timeout", False))
        self._sim_crash_mid_brew = bool(scenario.get("simulate_crash_mid_brew", False)) and not bool(
            scenario.get("resume_after_crash", False)
        )
        self._contradictory_receipt = bool(scenario.get("contradictory_receipt", False))
        self._recenter_calls = 0
        self.crashed_mid_brew = False

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
        # Host-owned busy countdown: machine transitions busy -> idle after configured polls
        if self._driver._machine_busy_polls_left > 0:
            self._driver._machine_busy_polls_left -= 1
            if self._driver._machine_busy_polls_left == 0 and self._driver._machine_status == "busy":
                self._driver._machine_status = "idle"
        self._rec(
            "observe_scene",
            "done",
            {
                "controlReady": obs["control_ready"],
                "objectCount": len(obs["objects"]),
                "safeToPour": obs["l2_pouring_safety"]["safe_to_pour"],
                "radialOffsetM": obs["l2_pouring_safety"]["current_radial_offset_m"],
                "machineStatus": obs["coffee_machine"]["status"],
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
        if self._driver._machine_status in {"busy", "cleaning", "fault"}:
            reason = f"machine_{self._driver._machine_status}"
            r = {"receipt": "rejected", "command_id": command_id, "reason": reason}
            self._command_receipts[command_id] = r
            self._driver._host_command_ledger[command_id] = r
            self._rec("start_coffee_machine", "rejected", {"commandId": command_id, "reason": reason})
            return r
        l2 = self._driver._compute_l2_pouring_safety()
        if not l2["safe_to_pour"]:
            r = {"receipt": "rejected", "command_id": command_id, "reason": "l2_pouring_unsafe"}
            self._command_receipts[command_id] = r
            self._driver._host_command_ledger[command_id] = r
            self._rec("start_coffee_machine", "rejected", {"commandId": command_id, "reason": "l2_pouring_unsafe"})
            return r
        self.brew_count += 1
        self._driver._persistent_brew_count = self.brew_count
        self._driver._machine_status = "running"
        self._driver._brew_progress = 0.15
        self._driver._set_brew_stream(active=True)
        final_r = {"receipt": "running", "command_id": command_id}
        self._command_receipts[command_id] = final_r
        self._driver._host_command_ledger[command_id] = final_r

        if self._sim_crash_mid_brew and self.brew_count == 1:
            self.unknown_events += 1
            self._pending_unknown = True
            self.queried_after_unknown = False
            self.reconciled_after_unknown = False
            self._driver._persistent_unknown_events = self.unknown_events
            self._driver._persistent_pending_unknown = True
            self._driver._persistent_queried_after_unknown = False
            self._driver._persistent_reconciled_after_unknown = False
            self.crashed_mid_brew = True
            self._rec("start_coffee_machine", "unknown", {"commandId": command_id, "crash": True})
            raise RuntimeError("host_process_crash_after_brew_dispatch")

        if self._contradictory_receipt and self.brew_count == 1:
            self.unknown_events += 1
            self._pending_unknown = True
            self.queried_after_unknown = False
            self.reconciled_after_unknown = False
            self._command_receipts[command_id] = {
                "receipt": "unknown",
                "command_id": command_id,
                "reason": "contradictory_device_telemetry",
            }
            self._driver._host_command_ledger[command_id] = dict(self._command_receipts[command_id])
            self._rec("start_coffee_machine", "unknown", {"commandId": command_id, "contradictory": True})
            return {"receipt": "unknown", "command_id": command_id}

        if self._sim_timeout and self.brew_count == 1:
            self.unknown_events += 1
            self._pending_unknown = True
            self.queried_after_unknown = False
            self.reconciled_after_unknown = False
            self._driver._persistent_unknown_events = self.unknown_events
            self._driver._persistent_pending_unknown = True
            self._driver._persistent_queried_after_unknown = False
            self._driver._persistent_reconciled_after_unknown = False
            self._rec("start_coffee_machine", "unknown", {"commandId": command_id})
            self._rec("command_ack_timeout_unknown", "unknown", {"commandId": command_id})
            return {"receipt": "unknown", "command_id": command_id}
        self._rec("start_coffee_machine", "running", {"commandId": command_id})
        return final_r

    def query_command_status(self, command_id: str) -> dict[str, Any]:
        r = self._command_receipts.get(
            command_id,
            self._driver._host_command_ledger.get(command_id, {"receipt": "unknown", "command_id": command_id}),
        )
        rec = str(r.get("receipt", "unknown"))
        if self._pending_unknown:
            self.queried_after_unknown = True
            self._driver._persistent_queried_after_unknown = True
            if rec in {"running", "done", "completed"}:
                self._pending_unknown = False
                self.reconciled_after_unknown = True
                self._driver._persistent_pending_unknown = False
                self._driver._persistent_reconciled_after_unknown = True
        self._rec("query_command_status", rec, {"commandId": command_id})
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
        goal_xy = self._driver._slot_xy(target_slot)
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


class _JournaledMuJoCoSDKProxy:
    """TRAX intent-before-dispatch journal proxy over the host SDK.

    Commits durable command intent to SQLite before physical dispatch. On crash/restart
    with the same ``run_id`` and journal:
    - Already-completed preparation steps (place/recenter) return their recorded receipt
      instead of moving hardware redundantly.
    - Already-claimed non-idempotent actions (``start_coffee_machine``) are NEVER
      blindly redispatched; instead, the proxy calls ``query_command_status`` on the
      previously committed command_id to reconcile live physical state.
    """

    def __init__(self, inner: _MuJoCoCafePolicySDK, journal: Journal, *, run_id: str) -> None:
        self._inner = inner
        self._journal = journal
        self._run_id = run_id

    def get_order(self, order_id: str = "A042") -> dict[str, Any] | None:
        return self._inner.get_order(order_id)

    def observe_scene(self) -> dict[str, Any]:
        return self._inner.observe_scene()

    def request_operator_confirmation(self, reason: str, candidates: list[Any]) -> dict[str, Any]:
        return self._inner.request_operator_confirmation(reason, candidates)

    def retract_safe_pose(self, *, command_id: str) -> dict[str, Any]:
        return self._inner.retract_safe_pose(command_id=command_id)

    def escalate_to_human_operator(self, order_id: str, reason: str, *, command_id: str) -> dict[str, Any]:
        return self._inner.escalate_to_human_operator(order_id, reason, command_id=command_id)

    def move_to_target(self, target_world: Any, *, command_id: str) -> dict[str, Any]:
        return self._inner.move_to_target(target_world, command_id=command_id)

    def query_command_status(self, command_id: str) -> dict[str, Any]:
        resp = self._inner.query_command_status(command_id)
        if resp.get("receipt") == "unknown":
            # Check durable SQLite journal for a prior committed brew command_id under this logical run_id
            jkey = f"{self._run_id}:start_coffee_machine"
            row = self._journal.db.execute("SELECT receipt FROM commands WHERE id=?", (jkey,)).fetchone()
            if row and row[0]:
                import json as _json
                saved = _json.loads(row[0])
                prior_cmd_id = str(saved.get("command_id") or "")
                if prior_cmd_id and prior_cmd_id != command_id:
                    reconciled = self._inner.query_command_status(prior_cmd_id)
                    self._journal.record(
                        jkey,
                        {"phase": "reconciled", "command_id": prior_cmd_id, "response": dict(reconciled)},
                    )
                    return dict(reconciled)
        return resp

    def _run_idempotent_step(self, step_name: str, fn: Any, *args: Any, command_id: str, **kwargs: Any) -> dict[str, Any]:
        jkey = f"{self._run_id}:{step_name}"
        inserted, saved = self._journal.claim(jkey, {"run_id": self._run_id, "step": step_name})
        if not inserted and isinstance(saved, Mapping) and saved.get("phase") == "recorded":
            rec = saved.get("response")
            if isinstance(rec, Mapping) and rec.get("receipt") == "done":
                return dict(rec)
        self._journal.record(jkey, {"phase": "dispatched", "command_id": command_id})
        resp = fn(*args, command_id=command_id, **kwargs)
        self._journal.record(jkey, {"phase": "recorded", "command_id": command_id, "response": dict(resp)})
        return resp

    def place_cup_under_machine(self, cup_id: str, *, command_id: str) -> dict[str, Any]:
        return self._run_idempotent_step("place_cup_under_machine", self._inner.place_cup_under_machine, cup_id, command_id=command_id)

    def recenter_cup_under_spout(self, cup_id: str, *, command_id: str) -> dict[str, Any]:
        return self._run_idempotent_step("recenter_cup_under_spout", self._inner.recenter_cup_under_spout, cup_id, command_id=command_id)

    def start_coffee_machine(self, *, command_id: str) -> dict[str, Any]:
        jkey = f"{self._run_id}:start_coffee_machine"
        inserted, saved = self._journal.claim(jkey, {"run_id": self._run_id, "step": "start_coffee_machine"})
        if not inserted:
            prior_cmd_id = str((saved or {}).get("command_id") or command_id)
            reconciled = self._inner.query_command_status(prior_cmd_id)
            self._journal.record(
                jkey,
                {"phase": "reconciled", "command_id": prior_cmd_id, "response": dict(reconciled)},
            )
            return dict(reconciled)
        # Durable intent + command_id committed BEFORE physical dispatch
        self._journal.record(jkey, {"phase": "dispatched", "command_id": command_id})
        resp = self._inner.start_coffee_machine(command_id=command_id)
        self._journal.record(jkey, {"phase": "recorded", "command_id": command_id, "response": dict(resp)})
        return resp

    def wait_for_coffee_ready(self, *, command_id: str) -> dict[str, Any]:
        return self._run_idempotent_step("wait_for_coffee_ready", self._inner.wait_for_coffee_ready, command_id=command_id)

    def retrieve_and_deliver_cup(self, cup_id: str, target_slot: str, *, command_id: str) -> dict[str, Any]:
        return self._run_idempotent_step(
            "retrieve_and_deliver_cup",
            self._inner.retrieve_and_deliver_cup,
            cup_id,
            target_slot,
            command_id=command_id,
        )

    def complete_order_delivery(self, order_id: str, cup_id: str, destination_slot: str, *, command_id: str) -> dict[str, Any]:
        return self._run_idempotent_step(
            "complete_order_delivery",
            self._inner.complete_order_delivery,
            order_id,
            cup_id,
            destination_slot,
            command_id=command_id,
        )


# ---------------------------------------------------------------------------
# MuJoCoCafeHostDriver (SceneHostDriver implementation)
# ---------------------------------------------------------------------------

class MuJoCoCafeHostDriver:
    """Native MuJoCo physics host driver for the TRAX single-cup café workstation."""

    driver_name = "mujoco_cafe"
    evidence_source = "simulator"

    def __init__(
        self,
        config: SceneConfig,
        *,
        realtime_pacing: bool = False,
        high_fidelity: bool = True,
        sensor_noise: bool = False,
        domain_randomization: bool = False,
    ) -> None:
        # Configure headless EGL before importing mujoco if not already set
        if not os.environ.get("MUJOCO_GL"):
            os.environ["MUJOCO_GL"] = "egl"
        try:
            import mujoco
            import numpy as np
        except ImportError as exc:
            raise RuntimeError(
                "MuJoCo driver requires optional dependencies. Install with: pip install 'tiwater-trax[mujoco]'"
            ) from exc

        self._mj = mujoco
        self._np = np
        self.config = config
        self._realtime_pacing = realtime_pacing
        self._default_sensor_noise = bool(sensor_noise)
        self._default_domain_randomization = bool(domain_randomization)
        self._sensor_noise_active = bool(sensor_noise)
        self._domain_randomization_active = bool(domain_randomization)
        self._noise_rng = random.Random(0)
        self._lock = threading.RLock()
        self._contract_sha = digest(list(_MuJoCoCafePolicySDK.ALLOWED_METHODS))

        assets_dir = _discover_robosuite_assets_dir() if high_fidelity else None
        self._panda_mode = assets_dir is not None
        xml_str = _build_high_fidelity_cafe_mjcf(assets_dir) if assets_dir is not None else CAFE_WORKSTATION_MJCF
        self._model = mujoco.MjModel.from_xml_string(xml_str)
        self._data = mujoco.MjData(self._model)
        self._ik_data = mujoco.MjData(self._model) if self._panda_mode else None
        self._renderer: Any = None
        self._render_width = 800 if self._panda_mode else 640
        self._render_height = 450 if self._panda_mode else 360
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
        if self._panda_mode:
            self._panda_jnt_ids = [
                mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, f"panda_j{i}") for i in range(1, 8)
            ]
            self._panda_qpos_idx = [int(self._model.jnt_qposadr[jid]) for jid in self._panda_jnt_ids]
            self._panda_dof_idx = [int(self._model.jnt_dofadr[jid]) for jid in self._panda_jnt_ids]
            self._ee_site_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SITE, "ee_tcp")
            self._act_ids = {
                name: mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                for name in ("act_j1", "act_j2", "act_j3", "act_j4", "act_j5", "act_j6", "act_j7", "act_fl", "act_fr", "act_stream")
            }
        else:
            self._panda_jnt_ids = []
            self._panda_qpos_idx = []
            self._panda_dof_idx = []
            self._ee_site_id = -1
            self._act_ids = {
                name: mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                for name in ("act_x", "act_y", "act_z", "act_yaw", "act_fl", "act_fr", "act_stream")
            }
        self._cup_a_body_geom_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_GEOM, "cup_a_body")
        self._base_cup_a_mass = float(self._model.body_mass[self._body_ids["obj-cup-a"]])
        self._base_cup_a_friction = self._model.geom_friction[self._cup_a_body_geom_id].copy()
        self._cup_coffee_geom_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_GEOM, "cup_a_coffee")
        self._fidelity_meta: dict[str, Any] = {
            "robot_model": "franka_emika_panda_7dof" if self._panda_mode else "gantry_3axis",
            "pbr_textures": self._panda_mode,
            "render_resolution": f"{self._render_width}x{self._render_height}",
            "sensor_noise": self._sensor_noise_active,
            "domain_randomization": self._domain_randomization_active,
        }
        self._step_seq = 0
        self._preset = "default"
        self._dual_cup_active = False
        self._confirmed_cup_id: str | None = None
        self._machine_status = "idle"
        self._machine_busy_polls_left = 0
        self._stale_observation = False
        self._cup_class_override: str | None = None
        self._cup_confidence_override: float | None = None
        self._cup_b_confidence_override: float | None = None
        self._layout_shift_xy: tuple[float, float] = (0.0, 0.0)
        self._cup_filled = False
        self._brew_progress = 0.0
        self._persistent_brew_count = 0
        self._persistent_unknown_events = 0
        self._persistent_pending_unknown = False
        self._persistent_queried_after_unknown = True
        self._persistent_reconciled_after_unknown = True
        self._host_command_ledger: dict[str, dict[str, Any]] = {}
        self._last_command: dict[str, Any] = {"command_id": None, "status": "idle", "steps": []}
        self.reset_scenario({"preset": "default"})

    def _slot_xy(self, target_slot: str) -> tuple[float, float]:
        base = PICKUP_2_XY if target_slot == "$SLOT_PICKUP_2" else PICKUP_1_XY
        dx, dy = self._layout_shift_xy
        # Clamp within reachable MuJoCo counter bounds
        gx = max(0.22, min(0.50, base[0] + dx))
        gy = max(-0.28, min(0.28, base[1] + dy))
        return (gx, gy)

    def _update_live_steps(self, steps: list[dict[str, Any]]) -> None:
        self._last_command["steps"] = list(steps)

    def _set_free_body_pose(self, obj_id: str, x: float, y: float, z: float) -> None:
        qa = self._jnt_qpos[obj_id]
        va = self._jnt_qvel[obj_id]
        self._data.qpos[qa : qa + 7] = [x, y, z, 1.0, 0.0, 0.0, 0.0]
        self._data.qvel[va : va + 6] = 0.0

    def _solve_panda_ik(self, wx: float, wy: float, wz: float) -> list[float]:
        """Damped least-squares 7-DOF IK keeping Panda gripper pointing straight downward."""
        np = self._np
        mj = self._mj
        ik_d = self._ik_data
        assert ik_d is not None
        ik_d.qpos[:] = self._data.qpos[:]
        target_pos = np.array([wx, wy, wz], dtype=float)
        # Desired site z-axis pointing straight down (world [0, 0, -1]) and x-axis forward (+x)
        desired_z = np.array([0.0, 0.0, -1.0], dtype=float)
        home_q = np.array(PANDA_HOME_QPOS, dtype=float)
        jpos = np.zeros((3, self._model.nv), dtype=float)
        jrot = np.zeros((3, self._model.nv), dtype=float)
        dofs = self._panda_dof_idx
        qidx = self._panda_qpos_idx

        for _ in range(18):
            mj.mj_kinematics(self._model, ik_d)
            mj.mj_comPos(self._model, ik_d)
            cur_pos = ik_d.site_xpos[self._ee_site_id].copy()
            cur_mat = ik_d.site_xmat[self._ee_site_id].reshape(3, 3)
            pos_err = target_pos - cur_pos
            cur_z = cur_mat[:, 2]
            rot_err = np.cross(cur_z, desired_z)
            if float(np.linalg.norm(pos_err)) < 0.0025 and float(np.linalg.norm(rot_err)) < 0.03:
                break
            mj.mj_jacSite(self._model, ik_d, jpos, jrot, self._ee_site_id)
            J = np.vstack([jpos[:, dofs], 0.45 * jrot[:, dofs]])  # 6x7
            err = np.concatenate([pos_err, 0.45 * rot_err])
            jjt = J @ J.T + 0.025 * np.eye(6)
            dq = J.T @ np.linalg.solve(jjt, err)
            # Null-space posture regularization toward comfortable elbow-up home pose
            q_cur = np.array([ik_d.qpos[i] for i in qidx], dtype=float)
            dq += 0.05 * (home_q - q_cur)
            dq = np.clip(dq, -0.35, 0.35)
            for k, (qi, jid) in enumerate(zip(qidx, self._panda_jnt_ids)):
                lo, hi = float(self._model.jnt_range[jid, 0]), float(self._model.jnt_range[jid, 1])
                ik_d.qpos[qi] = max(lo + 0.03, min(hi - 0.03, float(ik_d.qpos[qi]) + float(dq[k])))
        return [float(ik_d.qpos[i]) for i in qidx]

    def _set_arm_ctrl_world(self, wx: float, wy: float, wz: float, gripper_open: bool = True) -> None:
        if self._panda_mode:
            q_target = self._solve_panda_ik(wx, wy, wz)
            for idx, qv in enumerate(q_target, start=1):
                self._data.ctrl[self._act_ids[f"act_j{idx}"]] = qv
            finger_val = 0.039 if gripper_open else 0.028
            self._data.ctrl[self._act_ids["act_fl"]] = finger_val
            self._data.ctrl[self._act_ids["act_fr"]] = finger_val
            return
        # Legacy 3-axis gantry fallback
        self._data.ctrl[self._act_ids["act_x"]] = max(-0.10, min(0.55, wx - 0.12))
        self._data.ctrl[self._act_ids["act_y"]] = max(-0.34, min(0.34, wy))
        self._data.ctrl[self._act_ids["act_z"]] = max(-0.45, min(0.08, wz - 1.44))
        self._data.ctrl[self._act_ids["act_yaw"]] = 0.0
        finger_val = 0.012 if gripper_open else -0.018
        self._data.ctrl[self._act_ids["act_fl"]] = finger_val
        self._data.ctrl[self._act_ids["act_fr"]] = finger_val

    def _set_brew_stream(self, *, active: bool) -> None:
        self._data.ctrl[self._act_ids["act_stream"]] = (-0.11 if self._panda_mode else -0.14) if active else 0.02

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
                if self._panda_mode:
                    tcp = self._data.site_xpos[self._ee_site_id]
                    ee_x, ee_y, ee_z = float(tcp[0]), float(tcp[1]), float(tcp[2]) - 0.018
                else:
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
        # 7. Place cleanly at target XY + rest Z (with optional domain-randomization micro perturbation) and settle
        with self._lock:
            px, py = dest_x, dest_y
            if self._domain_randomization_active:
                px += self._noise_rng.gauss(0.0, 0.0022)
                py += self._noise_rng.gauss(0.0, 0.0022)
            self._set_free_body_pose(cup_id, px, py, CUP_REST_Z + 0.002)
            self._set_arm_ctrl_world(px, py, 1.15, gripper_open=True)
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

    def _noisy_xyz(self, pos: Any) -> list[float]:
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        if self._sensor_noise_active:
            x += self._noise_rng.gauss(0.0, 0.0022)
            y += self._noise_rng.gauss(0.0, 0.0022)
            z += self._noise_rng.gauss(0.0, 0.0018)
        return [round(x, 4), round(y, 4), round(z, 4)]

    def _noisy_tilt(self, tilt_deg: float) -> float:
        if self._sensor_noise_active:
            return round(max(0.0, tilt_deg + self._noise_rng.gauss(0.0, 0.7)), 3)
        return tilt_deg

    def _scene_objects(self) -> list[dict[str, Any]]:
        objs: list[dict[str, Any]] = []
        pos_a = self._noisy_xyz(self._data.xpos[self._body_ids["obj-cup-a"]])
        cup_cls = self._cup_class_override or "dine_in_red"
        base_conf = 0.96 if self._cup_confidence_override is None else float(self._cup_confidence_override)
        cup_conf = round(max(0.05, min(0.99, base_conf + (self._noise_rng.gauss(0.0, 0.015) if self._sensor_noise_active else 0.0))), 3)
        objs.append(
            {
                "object_id": "obj-cup-a",
                "id": "obj-cup-a",
                "semantic_class": cup_cls,
                "cup_type": cup_cls,
                "label": f"Cup ({cup_cls})",
                "confidence": cup_conf,
                "center_world_m": list(pos_a),
                "world": list(pos_a),
                "tilt_deg": self._noisy_tilt(self._get_body_tilt_deg("obj-cup-a")),
            }
        )
        if self._dual_cup_active:
            pos_b = self._noisy_xyz(self._data.xpos[self._body_ids["obj-cup-b"]])
            base_b_conf = 0.94 if self._cup_b_confidence_override is None else float(self._cup_b_confidence_override)
            cup_b_conf = round(max(0.05, min(0.99, base_b_conf + (self._noise_rng.gauss(0.0, 0.015) if self._sensor_noise_active else 0.0))), 3)
            objs.append(
                {
                    "object_id": "obj-cup-b",
                    "id": "obj-cup-b",
                    "semantic_class": "takeout_green",
                    "cup_type": "takeout_green",
                    "label": "Green Takeout Cup",
                    "confidence": cup_b_conf,
                    "center_world_m": list(pos_b),
                    "world": list(pos_b),
                    "tilt_deg": self._noisy_tilt(self._get_body_tilt_deg("obj-cup-b")),
                }
            )
        pos_bag = self._noisy_xyz(self._data.xpos[self._body_ids["obj-bag-1"]])
        objs.append(
            {
                "object_id": "obj-bag-1",
                "id": "obj-bag-1",
                "semantic_class": "flexible_bag",
                "cup_type": "flexible_bag",
                "label": "Coffee Bean Pouch",
                "confidence": 0.92,
                "center_world_m": list(pos_bag),
                "world": list(pos_bag),
                "tilt_deg": self._noisy_tilt(self._get_body_tilt_deg("obj-bag-1")),
            }
        )
        return objs

    def _build_sdk_observation(self) -> dict[str, Any]:
        with self._lock:
            objs = self._scene_objects()
            l2 = self._compute_l2_pouring_safety()
            control_ready = not self._stale_observation
            return {
                "control_ready": control_ready,
                "pipeline_health": {
                    "source": "mujoco_cafe_evidencescene",
                    "ok": control_ready,
                    "stale": self._stale_observation,
                    "engine": "mujoco",
                },
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
                "fidelity": dict(self._fidelity_meta),
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
            self._renderer = self._mj.Renderer(self._model, height=self._render_height, width=self._render_width)
        with self._lock:
            self._mj.mj_forward(self._model, self._data)
            self._renderer.update_scene(self._data, camera=camera_name)
            rgb = self._renderer.render()
            apply_noise = self._sensor_noise_active
        if apply_noise:
            np = self._np
            rng = np.random.default_rng(self._step_seq + hash(camera_name) % 10000)
            arr = rgb.astype(np.float32)
            # Realistic CMOS Gaussian readout noise + mild radial lens vignette + exposure jitter
            h, w, _ = arr.shape
            yy, xx = np.mgrid[-1.0:1.0:complex(0, h), -1.0:1.0:complex(0, w)]
            vignette = 1.0 - 0.14 * np.clip(xx * xx + yy * yy, 0.0, 1.5)
            exposure = float(rng.normal(1.0, 0.015))
            noise = rng.normal(0.0, 4.5, size=arr.shape)
            arr = np.clip(arr * vignette[..., None] * exposure + noise, 0.0, 255.0)
            rgb = arr.astype(np.uint8)
        buf = io.BytesIO()
        Image.fromarray(rgb).save(buf, format="JPEG", quality=88)
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
            self._sensor_noise_active = bool(scenario.get("sensor_noise", self._default_sensor_noise))
            self._domain_randomization_active = bool(scenario.get("domain_randomization", self._default_domain_randomization))
            self._noise_rng = random.Random(int(scenario.get("noise_seed", 42)))
            self._fidelity_meta["sensor_noise"] = self._sensor_noise_active
            self._fidelity_meta["domain_randomization"] = self._domain_randomization_active
            if self._domain_randomization_active:
                mass_scale = self._noise_rng.uniform(0.80, 1.20)
                fric_scale = self._noise_rng.uniform(0.60, 1.15)
                self._model.body_mass[self._body_ids["obj-cup-a"]] = self._base_cup_a_mass * mass_scale
                self._model.geom_friction[self._cup_a_body_geom_id] = self._base_cup_a_friction * fric_scale
            else:
                self._model.body_mass[self._body_ids["obj-cup-a"]] = self._base_cup_a_mass
                self._model.geom_friction[self._cup_a_body_geom_id] = self._base_cup_a_friction
            if self._panda_mode:
                for qi, qv in zip(self._panda_qpos_idx, PANDA_HOME_QPOS):
                    self._data.qpos[qi] = qv
            self._preset = str(scenario.get("preset", "default"))
            self._dual_cup_active = self._preset == "sim_dual_cup"
            self._confirmed_cup_id = scenario.get("operator_confirmed_cup_id")
            self._stale_observation = bool(scenario.get("stale_observation", False))
            self._cup_class_override = scenario.get("cup_class")
            self._cup_confidence_override = scenario.get("cup_confidence")
            self._cup_b_confidence_override = scenario.get("cup_b_confidence")
            shift = scenario.get("store_layout_shift_xy_m")
            self._layout_shift_xy = (float(shift[0]), float(shift[1])) if shift is not None else (0.0, 0.0)

            busy_polls = int(scenario.get("simulate_machine_busy_polls", 0))
            if bool(scenario.get("simulate_machine_fault", False)):
                self._machine_status = "fault"
                self._machine_busy_polls_left = 0
            elif bool(scenario.get("simulate_foreign_brew_running", False)):
                # Another customer's / operator's brew is already running on the machine (not this order!)
                self._machine_status = "running"
                self._machine_busy_polls_left = 0
                self._brew_progress = 0.45
            elif busy_polls > 0:
                self._machine_status = "busy"
                self._machine_busy_polls_left = busy_polls
            else:
                self._machine_status = "idle"
                self._machine_busy_polls_left = 0

            self._cup_filled = False
            if not bool(scenario.get("simulate_foreign_brew_running", False)):
                self._brew_progress = 0.0
            self._persistent_brew_count = 0
            self._persistent_unknown_events = 0
            self._persistent_pending_unknown = False
            self._persistent_queried_after_unknown = True
            self._persistent_reconciled_after_unknown = True
            self._host_command_ledger = (
                {"foreign-order-999-brew": {"receipt": "running", "command_id": "foreign-order-999-brew"}}
                if bool(scenario.get("simulate_foreign_brew_running", False))
                else {}
            )
            self._model.geom_rgba[self._cup_coffee_geom_id] = [0.85, 0.85, 0.88, 0.0]

            raw_a = scenario.get("cup_a_xy") or (0.33, -0.02)
            ax = max(0.24, min(0.42, float(raw_a[0]) + self._layout_shift_xy[0]))
            ay = max(-0.22, min(0.22, float(raw_a[1]) + self._layout_shift_xy[1]))
            self._set_free_body_pose("obj-cup-a", ax, ay, CUP_REST_Z)
            if self._dual_cup_active:
                raw_b = scenario.get("cup_b_xy") or (0.33, 0.06)
                bx = max(0.24, min(0.42, float(raw_b[0]) + self._layout_shift_xy[0]))
                by = max(-0.22, min(0.22, float(raw_b[1]) + self._layout_shift_xy[1]))
                self._set_free_body_pose("obj-cup-b", bx, by, CUP_REST_Z)
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
        enable_journal_proxy: bool = False,
        journal_run_id: str | None = None,
    ) -> dict[str, Any]:
        t0 = time.monotonic()
        self._last_command = {"command_id": command_id, "status": "running", "steps": []}
        if journal is not None and not enable_journal_proxy:
            journal.claim(command_id, {"driver": self.driver_name, "scenario": dict(scenario)})

        ast_chk = check_sandbox_ast_compliance(policy_source)
        sdk = _MuJoCoCafePolicySDK(self, scenario)
        active_sdk: Any = sdk
        if enable_journal_proxy and journal is not None:
            active_sdk = _JournaledMuJoCoSDKProxy(sdk, journal, run_id=journal_run_id or command_id)
        host_status = "rejected"
        detail = ""
        if not ast_chk["compliant"]:
            detail = "Host AST sandbox rejected policy: " + "; ".join(ast_chk["issues"][:3])
        else:
            ns: dict[str, Any] = {"__builtins__": dict(_SAFE_BUILTINS)}
            try:
                exec(compile(policy_source, "<trax-mujoco-policy>", "exec"), ns, ns)  # noqa: S102
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
                    ret = fn(active_sdk, params)
                    host_status = str((ret or {}).get("status", "unknown")) if isinstance(ret, Mapping) else "invalid_return"
                    detail = f"MuJoCo host runtime finished with status={host_status}"
            except Exception as exc:  # noqa: BLE001
                host_status = "crashed_mid_brew" if sdk.crashed_mid_brew else "exception"
                detail = f"{type(exc).__name__}: {exc}"

        wall_s = round(time.monotonic() - t0, 3)
        self._last_command["status"] = host_status

        # Post-run independent physical oracle from MuJoCo ground-truth xpos (scoring-only)
        target_slot = str(scenario.get("target_slot", "$SLOT_PICKUP_1"))
        goal_xy = self._slot_xy(target_slot)
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
            if slip is not None and not scenario.get("resume_after_crash"):
                passed = passed and ("recenter_cup_under_spout" in methods)
            if sim_timeout or scenario.get("resume_after_crash"):
                passed = (
                    passed
                    and sdk.brew_count == 1
                    and sdk.unknown_events >= 1
                    and sdk.queried_after_unknown
                    and sdk.reconciled_after_unknown
                )
        else:
            acceptable = {expect_status}
            if expect_status == "paused_for_operator":
                acceptable.add("rejected_observation_not_ready")
            if expect_status == "escalated_to_human" and scenario.get("contradictory_receipt"):
                # Contradictory receipt occurs after single brew dispatch; must NOT duplicate brew
                passed = (host_status in acceptable) and sdk.brew_count <= 1 and sdk.queried_after_unknown
            elif expect_status == "escalated_to_human" and scenario.get("simulate_foreign_brew_running"):
                # Foreign order already running: must NOT start a brew and must NOT steal/deliver the foreign cup!
                passed = (
                    (host_status in acceptable)
                    and sdk.brew_count == 0
                    and ("retrieve_and_deliver_cup" not in methods)
                    and ("complete_order_delivery" not in methods)
                )
            else:
                passed = (host_status in acceptable) and sdk.brew_count == 0

        if journal is not None and not enable_journal_proxy:
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
            "reconciled_after_unknown": sdk.reconciled_after_unknown,
            "human_interventions": sdk.human_interventions,
            "crashed_mid_brew": sdk.crashed_mid_brew,
            "sandbox_mode": "mujoco_ast_sandbox",
            "oracle_post_run": {
                "cup_filled": cup_filled,
                "delivery_error_mm": err_mm,
                "cup_tilt_deg": tilt_deg,
            },
            "fault_injection": bool(
                sim_timeout
                or slip is not None
                or scenario.get("simulate_crash_mid_brew")
                or scenario.get("contradictory_receipt")
            ),
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
