"""Native MuJoCo LeRobot SO-101 (SO-ARM100) host driver (`driver="mujoco_so101"`).

Integrates the official Apache-2.0 ``TheRobotStudio/SO-ARM100/Simulation/SO101``
6-DOF articulated manipulator (5 arm joints + moving jaw gripper + wrist camera mount)
as an isolated optional ``SceneHostDriver`` without modifying any pure-stdlib module
in ``src/trax/``.

Strictly preserves TRAX ownership separation:
- Perception owner: MuJoCo-derived read-only EvidenceScene snapshot (no fabricated state)
- Policy governance: pure-stdlib AST sandbox + SQLite intent-before-dispatch journal
- Host actuation: SO-101 5-DOF damped-least-squares IK, STS3215 joint limits, workspace
  clamps, L2 pouring safety interlock, and non-idempotent command ledger
- Evidence provenance: explicitly tagged ``evidence_source="simulator"`` (NOT physical_robot)
"""
from __future__ import annotations

import atexit
from concurrent.futures import ThreadPoolExecutor
import math
import os
from pathlib import Path
import random
import threading
from typing import Any, Mapping
import xml.etree.ElementTree as ET

if not os.environ.get("MUJOCO_GL") and not os.environ.get("DISPLAY"):
    os.environ["MUJOCO_GL"] = "egl"

from .config import SceneConfig
from .contracts import digest
from .edge_adapter import CapabilityCheckResult, DiscoveredCapabilities
from .mujoco_cafe_driver import (
    MuJoCoCafeHostDriver,
    PICKUP_1_XY,
    PICKUP_2_XY,
    SPOUT_XY,
    TABLE_TOP_Z,
    _MuJoCoCafePolicySDK,
    _discover_robosuite_assets_dir,
)

SO101_BASE_XYZ = (0.13, 0.0, TABLE_TOP_Z)
SO101_ARM_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
SO101_GRIPPER_JOINT = "gripper"
SO101_HOME_QPOS = (0.0, -0.35, 0.55, 0.95, 0.0)
SO101_GRIPPER_OPEN = 0.88
SO101_GRIPPER_CLOSED = 0.10
SO101_CUP_HALF_H = 0.036
SO101_CUP_REST_Z = TABLE_TOP_Z + SO101_CUP_HALF_H
SO101_WORKSPACE_MIN_R = 0.10
SO101_WORKSPACE_MAX_R = 0.34
SO101_WORKSPACE_MIN_Z = TABLE_TOP_Z + 0.025
SO101_WORKSPACE_MAX_Z = TABLE_TOP_Z + 0.28


def discover_so101_dir() -> Path:
    """Locate bundled Apache-2.0 SO-101 Simulation/SO101 assets dynamically."""
    repo_root = Path(__file__).resolve().parents[2]
    candidates = (
        repo_root / "sim" / "so101",
        Path.cwd() / "sim" / "so101",
    )
    for cand in candidates:
        if (cand / "so101_new_calib_camera.xml").is_file() and (cand / "assets").is_dir():
            return cand
    raise FileNotFoundError(
        "SO-101 MuJoCo assets not found under sim/so101/. "
        "Expected sim/so101/so101_new_calib_camera.xml and sim/so101/assets/*.stl."
    )


def build_so101_workstation_mjcf(so101_dir: Path | None = None) -> str:
    """Compose official SO-101 MJCF with tabletop espresso workstation and 4 cameras."""
    base_dir = so101_dir or discover_so101_dir()
    assets_dir = base_dir / "assets"
    so101_xml_path = base_dir / "so101_new_calib_camera.xml"
    src_root = ET.parse(so101_xml_path).getroot()

    pbr_dir = _discover_robosuite_assets_dir()
    tex_dir = (pbr_dir / "textures") if pbr_dir is not None else None

    if tex_dir is not None:
        tex_block = f"""
    <texture name="sky_tex" type="skybox" builtin="gradient" rgb1="0.22 0.25 0.30" rgb2="0.09 0.10 0.12" width="256" height="256"/>
    <texture name="floor_tex" type="2d" file="{(tex_dir / 'light-gray-floor-tile.png').as_posix()}"/>
    <texture name="wood_tex" type="2d" file="{(tex_dir / 'dark-wood.png').as_posix()}"/>
    <texture name="wall_tex" type="2d" file="{(tex_dir / 'white-bricks.png').as_posix()}"/>
    <texture name="steel_tex" type="2d" file="{(tex_dir / 'steel-brushed.png').as_posix()}"/>
    <texture name="scratched_tex" type="2d" file="{(tex_dir / 'steel-scratched.png').as_posix()}"/>
    <texture name="ceramic_tex" type="2d" file="{(tex_dir / 'ceramic.png').as_posix()}"/>
    <texture name="felt_tex" type="2d" file="{(tex_dir / 'gray-felt.png').as_posix()}"/>
    <material name="floor_mat" texture="floor_tex" texuniform="true" texrepeat="2 2" reflectance="0.06" specular="0.18" shininess="0.25"/>
    <material name="counter_wood_mat" texture="wood_tex" texuniform="true" texrepeat="1.5 1.5" rgba="0.80 0.66 0.52 1" specular="0.26" shininess="0.35" reflectance="0.05"/>
    <material name="wall_mat" texture="wall_tex" texuniform="true" texrepeat="2 2" rgba="0.90 0.88 0.85 1" specular="0.05" shininess="0.08"/>
    <material name="steel_brushed_mat" texture="steel_tex" texuniform="true" texrepeat="2 2" rgba="0.72 0.75 0.78 1" specular="0.55" shininess="0.65" reflectance="0.14"/>
    <material name="steel_dark_mat" texture="scratched_tex" texuniform="true" texrepeat="2 2" rgba="0.20 0.22 0.25 1" specular="0.45" shininess="0.50" reflectance="0.08"/>
    <material name="barmat_mat" texture="felt_tex" texuniform="true" texrepeat="4 4" rgba="0.16 0.17 0.19 1" specular="0.04" shininess="0.06"/>
    <material name="cup_red_mat" texture="ceramic_tex" rgba="0.88 0.18 0.22 1" specular="0.75" shininess="0.85" reflectance="0.12"/>
    <material name="cup_green_mat" texture="ceramic_tex" rgba="0.12 0.62 0.40 1" specular="0.35" shininess="0.40"/>
    <material name="bag_mat" texture="wood_tex" rgba="0.72 0.58 0.42 1" specular="0.08" shininess="0.10"/>
"""
    else:
        tex_block = """
    <texture name="sky_tex" type="skybox" builtin="gradient" rgb1="0.20 0.24 0.30" rgb2="0.07 0.09 0.13" width="256" height="256"/>
    <texture name="grid_tex" type="2d" builtin="checker" rgb1="0.18 0.21 0.26" rgb2="0.13 0.16 0.21" width="512" height="512"/>
    <material name="floor_mat" texture="grid_tex" texrepeat="8 8" reflectance="0.08"/>
    <material name="counter_wood_mat" rgba="0.42 0.31 0.22 1" specular="0.22" shininess="0.32"/>
    <material name="wall_mat" rgba="0.84 0.83 0.80 1" specular="0.05"/>
    <material name="steel_brushed_mat" rgba="0.68 0.72 0.76 1" specular="0.55" shininess="0.65"/>
    <material name="steel_dark_mat" rgba="0.18 0.20 0.24 1" specular="0.45" shininess="0.50"/>
    <material name="barmat_mat" rgba="0.15 0.16 0.19 1" specular="0.04"/>
    <material name="cup_red_mat" rgba="0.88 0.18 0.22 1" specular="0.7" shininess="0.8"/>
    <material name="cup_green_mat" rgba="0.12 0.62 0.40 1" specular="0.35" shininess="0.4"/>
    <material name="bag_mat" rgba="0.62 0.50 0.36 1" specular="0.08"/>
"""

    scene_xml = f"""<mujoco model="trax_so101_workstation">
  <compiler angle="radian" coordinate="local" inertiafromgeom="true" autolimits="true"/>
  <option timestep="0.004" gravity="0 0 -9.81" integrator="implicitfast"/>

  <visual>
    <global offwidth="800" offheight="450" azimuth="140" elevation="-22"/>
    <headlight diffuse="0.56 0.54 0.52" ambient="0.30 0.29 0.28" specular="0.25 0.25 0.25"/>
    <rgba haze="0.08 0.10 0.13 1"/>
    <quality shadowsize="2048" offsamples="4"/>
  </visual>

  <asset>
{tex_block}
    <material name="slot1_mat" rgba="0.12 0.72 0.95 0.78" specular="0.3"/>
    <material name="slot2_mat" rgba="0.96 0.68 0.14 0.78" specular="0.3"/>
    <material name="nogo_mat" rgba="0.92 0.18 0.16 0.72" specular="0.2"/>
    <material name="coffee_mat" rgba="0.20 0.10 0.04 1" specular="0.7" shininess="0.9"/>
    <material name="badge_blue" rgba="0.05 0.52 0.82 1" specular="0.6"/>
  </asset>

  <worldbody>
    <light pos="0.15 -0.45 2.05" dir="0.12 0.22 -1" diffuse="0.88 0.85 0.80" specular="0.35 0.35 0.35" castshadow="true"/>
    <light pos="0.55 0.50 1.95" dir="-0.15 -0.20 -1" diffuse="0.45 0.48 0.56" castshadow="false"/>

    <!-- Unobstructed Cameras framed specifically for LeRobot SO-101 desktop scale -->
    <camera name="overview" pos="-0.10 -0.48 1.42" xyaxes="0.86 -0.51 0 0.27 0.45 0.85" fovy="46"/>
    <camera name="overhead" pos="0.34 0.00 1.48" xyaxes="0 1 0 -1 0 0" fovy="58"/>
    <camera name="spout" pos="0.22 -0.28 1.20" xyaxes="0.786 -0.618 0 0.212 0.270 0.939" fovy="44"/>

    <!-- Floor & Backdrop Wall -->
    <geom name="floor" type="plane" size="3 3 0.1" material="floor_mat"/>
    <geom name="back_wall" type="box" pos="0.85 0.0 1.50" size="0.02 1.60 1.50" material="wall_mat" contype="0" conaffinity="0"/>

    <!-- Tabletop Bench -->
    <body name="counter" pos="0.36 0.0 0.50">
      <geom name="table_top" type="box" size="0.46 0.44 0.50" material="counter_wood_mat" friction="0.9 0.01 0.001"/>
      <geom name="bar_mat" type="box" pos="0.02 0.0 0.5008" size="0.30 0.26 0.0008" material="barmat_mat" contype="0" conaffinity="0"/>
    </body>

    <!-- Pickup Slots & Forbidden Steam Zone Markers -->
    <geom name="slot_pickup_1" type="cylinder" pos="0.38 -0.17 1.0016" size="0.045 0.0012" material="slot1_mat" contype="0" conaffinity="0"/>
    <geom name="slot_pickup_2" type="cylinder" pos="0.38 0.17 1.0016" size="0.045 0.0012" material="slot2_mat" contype="0" conaffinity="0"/>
    <geom name="zone_steam_nogo" type="box" pos="0.56 0.22 1.0016" size="0.060 0.060 0.0012" material="nogo_mat" contype="0" conaffinity="0"/>

    <!-- Compact Desktop Espresso Machine (clear overhang height for SO-101 arm) -->
    <body name="espresso_machine" pos="0.58 0.0 1.00">
      <geom name="machine_base" type="box" pos="0.04 0 0.13" size="0.095 0.115 0.13" material="steel_brushed_mat"/>
      <geom name="machine_top" type="box" pos="-0.02 0 0.29" size="0.145 0.120 0.032" material="steel_dark_mat"/>
      <geom name="machine_badge" type="box" pos="-0.166 0 0.29" size="0.003 0.040 0.012" material="badge_blue" contype="0" conaffinity="0"/>
      <geom name="drip_tray" type="box" pos="-0.13 0 0.003" size="0.065 0.080 0.003" material="steel_dark_mat"/>
      <geom name="l2_safe_ring" type="cylinder" pos="-0.13 0 0.0065" size="0.028 0.0008" rgba="0.18 0.82 0.45 0.65" contype="0" conaffinity="0"/>
      <geom name="brew_spout" type="cylinder" pos="-0.13 0 0.225" size="0.012 0.028" material="steel_brushed_mat" contype="0" conaffinity="0"/>
      <geom name="steam_wand" type="capsule" fromto="-0.04 0.095 0.22 -0.01 0.185 0.12" size="0.006" material="steel_brushed_mat" contype="0" conaffinity="0"/>
    </body>

    <!-- Animated Brew Stream -->
    <body name="brew_stream" pos="0.45 0.0 1.27">
      <joint name="stream_slide" type="slide" axis="0 0 1" range="-0.15 0.02" damping="10"/>
      <geom name="stream_geom" type="cylinder" size="0.0045 0.034" material="coffee_mat" contype="0" conaffinity="0"/>
    </body>

    <!-- Compact Rigid Espresso Cup A (dine_in_red, sized for SO-101 jaw) -->
    <body name="cup_a" pos="0.33 -0.02 {SO101_CUP_REST_Z:.4f}">
      <freejoint name="cup_a_free"/>
      <geom name="cup_a_body" type="cylinder" size="0.024 {SO101_CUP_HALF_H:.4f}" mass="0.085" material="cup_red_mat" friction="0.9 0.01 0.001"/>
      <geom name="cup_a_rim" type="cylinder" pos="0 0 0.034" size="0.026 0.003" mass="0.005" rgba="0.96 0.96 0.98 1" contype="0" conaffinity="0"/>
      <geom name="cup_a_coffee" type="cylinder" pos="0 0 0.030" size="0.021 0.002" mass="0.003" material="coffee_mat" contype="0" conaffinity="0"/>
    </body>

    <!-- Rigid Cup B (takeout_green, parked off-table unless dual-cup preset active) -->
    <body name="cup_b" pos="1.80 1.80 {SO101_CUP_REST_Z:.4f}">
      <freejoint name="cup_b_free"/>
      <geom name="cup_b_body" type="cylinder" size="0.023 {SO101_CUP_HALF_H:.4f}" mass="0.075" material="cup_green_mat" friction="0.9 0.01 0.001"/>
      <geom name="cup_b_lid" type="cylinder" pos="0 0 0.036" size="0.025 0.004" mass="0.005" rgba="0.95 0.95 0.95 1" contype="0" conaffinity="0"/>
    </body>

    <!-- Distractor Bag -->
    <body name="bag_distractor" pos="0.24 0.24 1.024">
      <freejoint name="bag_free"/>
      <geom name="bag_geom" type="box" size="0.042 0.028 0.024" mass="0.09" material="bag_mat" friction="0.85 0.01 0.001"/>
    </body>
  </worldbody>

  <actuator>
    <position name="act_stream" joint="stream_slide" kp="120" ctrlrange="-0.15 0.02"/>
  </actuator>
</mujoco>
"""
    dst_root = ET.fromstring(scene_xml)

    # 1. Copy <default> blocks from official SO-101 MJCF
    insert_idx = 2
    for default_el in src_root.findall("default"):
        # Keep SO-101 collision geoms non-colliding with tabletop/cups during smooth IK transport
        for sub in default_el.iter("default"):
            if sub.attrib.get("class") == "collision":
                for g in sub.findall("geom"):
                    g.set("contype", "0")
                    g.set("conaffinity", "0")
        dst_root.insert(insert_idx, default_el)
        insert_idx += 1

    # 2. Merge SO-101 <asset> meshes (resolved to absolute POSIX paths) and materials
    dst_asset = dst_root.find("asset")
    assert dst_asset is not None
    src_asset = src_root.find("asset")
    if src_asset is not None:
        for child in src_asset:
            if child.tag == "mesh" and "file" in child.attrib:
                child.set("file", (assets_dir / child.attrib["file"]).as_posix())
            dst_asset.append(child)

    # 3. Merge SO-101 <body name="base"> mounted onto tabletop counter + attach wrist camera
    dst_wb = dst_root.find("worldbody")
    src_wb = src_root.find("worldbody")
    assert dst_wb is not None and src_wb is not None
    base_body = src_wb.find("body[@name='base']")
    if base_body is None:
        raise ValueError("Official SO-101 XML missing <body name='base'>")
    base_body.set("pos", f"{SO101_BASE_XYZ[0]:.4f} {SO101_BASE_XYZ[1]:.4f} {SO101_BASE_XYZ[2]:.4f}")

    # Inject a real MuJoCo wrist camera on the SO-101 gripper body looking down along -Z past the jaws
    gripper_body = base_body.find(".//body[@name='gripper']")
    if gripper_body is not None:
        cam_el = ET.Element(
            "camera",
            {
                "name": "wrist",
                "pos": "-0.018 0.052 -0.028",
                "xyaxes": "1 0 0 0 0.94 -0.34",
                "fovy": "65",
            },
        )
        gripper_body.append(cam_el)
    dst_wb.append(base_body)

    # 4. Merge SO-101 STS3215 position actuators
    dst_act = dst_root.find("actuator")
    src_act = src_root.find("actuator")
    assert dst_act is not None and src_act is not None
    for act in src_act:
        dst_act.append(act)

    return ET.tostring(dst_root, encoding="unicode")


class MuJoCoSO101HostDriver(MuJoCoCafeHostDriver):
    """Native MuJoCo SceneHostDriver driven by the official LeRobot SO-101 6-DOF arm."""

    driver_name = "mujoco_so101"
    evidence_source = "simulator"

    def __init__(
        self,
        config: SceneConfig,
        *,
        realtime_pacing: bool = False,
        sensor_noise: bool = False,
        domain_randomization: bool = False,
        so101_dir: Path | None = None,
    ) -> None:
        if not os.environ.get("MUJOCO_GL"):
            os.environ["MUJOCO_GL"] = "egl"
        try:
            import mujoco
            import numpy as np
        except ImportError as exc:
            raise RuntimeError(
                "MuJoCo SO-101 driver requires optional dependencies. "
                "Install with: pip install 'tiwater-trax[mujoco]'"
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

        self._panda_mode = False
        self._so101_dir = so101_dir or discover_so101_dir()
        xml_str = build_so101_workstation_mjcf(self._so101_dir)
        self._model = mujoco.MjModel.from_xml_string(xml_str)
        self._data = mujoco.MjData(self._model)
        self._ik_data = mujoco.MjData(self._model)
        self._renderer: Any = None
        self._render_width = 800
        self._render_height = 450
        self._render_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="mujoco-so101-egl")
        atexit.register(self.close)

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

        self._so101_jnt_ids = [
            mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in SO101_ARM_JOINTS
        ]
        self._so101_qpos_idx = [int(self._model.jnt_qposadr[jid]) for jid in self._so101_jnt_ids]
        self._so101_dof_idx = [int(self._model.jnt_dofadr[jid]) for jid in self._so101_jnt_ids]
        self._gripper_jnt_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, SO101_GRIPPER_JOINT)
        self._gripper_qpos_idx = int(self._model.jnt_qposadr[self._gripper_jnt_id])
        self._ee_site_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")

        self._act_ids = {
            name: mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in (*SO101_ARM_JOINTS, SO101_GRIPPER_JOINT, "act_stream")
        }
        self._cup_a_body_geom_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_GEOM, "cup_a_body")
        self._base_cup_a_mass = float(self._model.body_mass[self._body_ids["obj-cup-a"]])
        self._base_cup_a_friction = self._model.geom_friction[self._cup_a_body_geom_id].copy()
        self._cup_coffee_geom_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_GEOM, "cup_a_coffee")
        self._fidelity_meta: dict[str, Any] = {
            "robot_model": "lerobot_so101_6dof",
            "upstream_model": "TheRobotStudio/SO-ARM100/Simulation/SO101",
            "upstream_license": "Apache-2.0",
            "pbr_textures": _discover_robosuite_assets_dir() is not None,
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

    def clamp_world_target(self, wx: float, wy: float, wz: float) -> tuple[float, float, float]:
        """Enforce host-owned SO-101 tabletop workspace safety bounds."""
        dx = float(wx) - SO101_BASE_XYZ[0]
        dy = float(wy) - SO101_BASE_XYZ[1]
        r = math.hypot(dx, dy)
        if r < 1e-6:
            dx, dy = SO101_WORKSPACE_MIN_R, 0.0
        elif r < SO101_WORKSPACE_MIN_R:
            scale = SO101_WORKSPACE_MIN_R / r
            dx, dy = dx * scale, dy * scale
        elif r > SO101_WORKSPACE_MAX_R:
            scale = SO101_WORKSPACE_MAX_R / r
            dx, dy = dx * scale, dy * scale
        cx = SO101_BASE_XYZ[0] + dx
        cy = SO101_BASE_XYZ[1] + dy
        cz = max(SO101_WORKSPACE_MIN_Z, min(SO101_WORKSPACE_MAX_Z, float(wz)))
        return (cx, cy, cz)

    def _solve_so101_ik(self, wx: float, wy: float, wz: float) -> list[float]:
        """Null-space 5-DOF damped-least-squares IK for SO-101 gripperframe TCP."""
        np = self._np
        mj = self._mj
        cx, cy, cz = self.clamp_world_target(wx, wy, wz)
        ik_d = self._ik_data
        assert ik_d is not None
        ik_d.qpos[:] = self._data.qpos[:]
        qidx = self._so101_qpos_idx
        dofs = self._so101_dof_idx

        # Warm-start shoulder_pan toward target azimuth (+pan rotates toward -Y in SO-101)
        pan_init = -math.atan2(cy - SO101_BASE_XYZ[1], max(0.05, cx - SO101_BASE_XYZ[0]))
        pan_lo = float(self._model.jnt_range[self._so101_jnt_ids[0], 0]) + 0.03
        pan_hi = float(self._model.jnt_range[self._so101_jnt_ids[0], 1]) - 0.03
        ik_d.qpos[qidx[0]] = max(pan_lo, min(pan_hi, pan_init))
        for k in range(1, 5):
            ik_d.qpos[qidx[k]] = SO101_HOME_QPOS[k]

        target_pos = np.array([cx, cy, cz], dtype=float)
        jpos = np.zeros((3, self._model.nv), dtype=float)
        jrot = np.zeros((3, self._model.nv), dtype=float)

        for _ in range(42):
            mj.mj_kinematics(self._model, ik_d)
            mj.mj_comPos(self._model, ik_d)
            cur_pos = ik_d.site_xpos[self._ee_site_id].copy()
            perr = target_pos - cur_pos
            if float(np.linalg.norm(perr)) < 0.0012:
                break
            mj.mj_jacSite(self._model, ik_d, jpos, jrot, self._ee_site_id)
            J = jpos[:, dofs]  # 3x5
            jjt = J @ J.T + 0.004 * np.eye(3)
            J_pinv = J.T @ np.linalg.inv(jjt)
            null = np.eye(5) - J_pinv @ J
            q_cur = np.array([float(ik_d.qpos[i]) for i in qidx], dtype=float)
            q_pref = np.array([q_cur[0], -0.25, 0.45, 0.85, 0.0], dtype=float)
            dq = J_pinv @ perr + 0.14 * (null @ (q_pref - q_cur))
            dq = np.clip(dq, -0.35, 0.35)
            for k, (qi, jid) in enumerate(zip(qidx, self._so101_jnt_ids)):
                lo, hi = float(self._model.jnt_range[jid, 0]), float(self._model.jnt_range[jid, 1])
                ik_d.qpos[qi] = max(lo + 0.02, min(hi - 0.02, float(ik_d.qpos[qi]) + float(dq[k])))

        q_out = [float(ik_d.qpos[i]) for i in qidx]
        if self._domain_randomization_active:
            # Optional STS3215 servo backlash simulation (+/- 0.5 deg ~ 0.0087 rad)
            for k, jid in enumerate(self._so101_jnt_ids):
                lo, hi = float(self._model.jnt_range[jid, 0]), float(self._model.jnt_range[jid, 1])
                jitter = self._noise_rng.uniform(-0.0087, 0.0087)
                q_out[k] = max(lo + 0.01, min(hi - 0.01, q_out[k] + jitter))
        return q_out

    def _set_arm_ctrl_world(self, wx: float, wy: float, wz: float, gripper_open: bool = True) -> None:
        q_target = self._solve_so101_ik(wx, wy, wz)
        for name, qv in zip(SO101_ARM_JOINTS, q_target):
            self._data.ctrl[self._act_ids[name]] = qv
        self._data.ctrl[self._act_ids[SO101_GRIPPER_JOINT]] = (
            SO101_GRIPPER_OPEN if gripper_open else SO101_GRIPPER_CLOSED
        )

    def _set_brew_stream(self, *, active: bool) -> None:
        self._data.ctrl[self._act_ids["act_stream"]] = -0.11 if active else 0.02

    def _step_physics(self, n_steps: int, *, carried_cup_id: str | None = None) -> None:
        import time

        for i in range(n_steps):
            self._mj.mj_step(self._model, self._data)
            self._step_seq += 1
            if carried_cup_id is not None:
                tcp = self._data.site_xpos[self._ee_site_id]
                ee_x, ee_y, ee_z = float(tcp[0]), float(tcp[1]), float(tcp[2]) - 0.010
                self._set_free_body_pose(carried_cup_id, ee_x, ee_y, max(SO101_CUP_REST_Z, ee_z))
            if self._realtime_pacing and (i % 6 == 0):
                time.sleep(0.012)

    def _transport_cup_physics(self, cup_id: str, dest_x: float, dest_y: float) -> bool:
        cur = self._get_cup_xy(cup_id)
        if cur is None:
            return False
        cx, cy = cur
        self._move_arm_world(cx, cy, 1.155, gripper_open=True, steps=24)
        self._move_arm_world(cx, cy, 1.045, gripper_open=True, steps=22)
        self._move_arm_world(cx, cy, 1.045, gripper_open=False, carried_cup_id=cup_id, steps=14)
        self._move_arm_world(cx, cy, 1.155, gripper_open=False, carried_cup_id=cup_id, steps=22)
        self._move_arm_world(dest_x, dest_y, 1.155, gripper_open=False, carried_cup_id=cup_id, steps=32)
        self._move_arm_world(dest_x, dest_y, 1.050, gripper_open=False, carried_cup_id=cup_id, steps=22)
        with self._lock:
            px, py = dest_x, dest_y
            if self._domain_randomization_active:
                px += self._noise_rng.gauss(0.0, 0.0022)
                py += self._noise_rng.gauss(0.0, 0.0022)
            self._set_free_body_pose(cup_id, px, py, SO101_CUP_REST_Z + 0.002)
            self._set_arm_ctrl_world(px, py, 1.055, gripper_open=True)
            self._step_physics(24, carried_cup_id=None)
        self._move_arm_world(dest_x, dest_y, 1.165, gripper_open=True, steps=18)
        return True

    def discover_capabilities(self, *, allow_busy: bool = False) -> CapabilityCheckResult:
        with self._lock:
            if self._machine_status == "busy" and not allow_busy:
                return CapabilityCheckResult(
                    status="busy",
                    discovered=None,
                    missing_methods=(),
                    detail="SO-101 simulator coffee machine busy",
                )
            allowed = _MuJoCoCafePolicySDK.ALLOWED_METHODS
            discovered = DiscoveredCapabilities(
                execution_authority="trax-mujoco-so101-sim",
                perception_owner="MuJoCo-SO101-EvidenceScene",
                sdk_version="so101-tabletop-1.0",
                allowed_methods=allowed,
                sdk_contract_sha256=self._contract_sha,
                device_bundle_id="trax-lerobot-so101-workstation",
                device_bundle_sha256=self._contract_sha,
                runtime_status=str(self._last_command.get("status", "idle")),
            )
            missing = tuple(
                m for m in self.config.capabilities.required_methods if m not in discovered.allowed_methods
            )
            return CapabilityCheckResult(
                status="ready" if not missing else "missing_capabilities",
                discovered=discovered,
                missing_methods=missing,
                detail="Native MuJoCo LeRobot SO-101 workstation ready (engine=mujoco, Apache-2.0 SO-ARM100)"
                if not missing
                else f"missing={missing}",
            )

    def reset_scenario(self, scenario: Mapping[str, Any]) -> dict[str, Any]:
        with self._lock:
            self._mj.mj_resetData(self._model, self._data)
            self._sensor_noise_active = bool(scenario.get("sensor_noise", self._default_sensor_noise))
            self._domain_randomization_active = bool(
                scenario.get("domain_randomization", self._default_domain_randomization)
            )
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

            for qi, qv in zip(self._so101_qpos_idx, SO101_HOME_QPOS):
                self._data.qpos[qi] = qv
            self._data.qpos[self._gripper_qpos_idx] = SO101_GRIPPER_OPEN

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
            ay = max(-0.20, min(0.20, float(raw_a[1]) + self._layout_shift_xy[1]))
            self._set_free_body_pose("obj-cup-a", ax, ay, SO101_CUP_REST_Z)
            if self._dual_cup_active:
                raw_b = scenario.get("cup_b_xy") or (0.33, 0.06)
                bx = max(0.24, min(0.42, float(raw_b[0]) + self._layout_shift_xy[0]))
                by = max(-0.20, min(0.20, float(raw_b[1]) + self._layout_shift_xy[1]))
                self._set_free_body_pose("obj-cup-b", bx, by, SO101_CUP_REST_Z)
            else:
                self._set_free_body_pose("obj-cup-b", 1.80, 1.80, SO101_CUP_REST_Z)
            self._set_free_body_pose("obj-bag-1", 0.24, 0.24, TABLE_TOP_Z + 0.026)

            self._set_arm_ctrl_world(0.32, 0.00, 1.18, gripper_open=True)
            self._set_brew_stream(active=False)
            self._mj.mj_forward(self._model, self._data)
            self._step_physics(18)
            self._last_command = {"command_id": None, "status": "idle", "steps": []}
            return {
                "status": "reset",
                "preset": self._preset,
                "simulator_engine": "mujoco",
                "robot_model": "lerobot_so101_6dof",
            }
