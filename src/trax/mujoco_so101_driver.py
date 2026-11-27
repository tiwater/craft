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
import time
from typing import Any, Mapping, Sequence
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
    """Locate bundled Apache-2.0 SO-101 robot assets under `assets/robots/so101/` (or env override)."""
    repo_root = Path(__file__).resolve().parents[2]
    env_dir = os.environ.get("TRAX_ROBOT_ASSETS_DIR", "").strip()
    candidates: list[Path] = []
    if env_dir:
        candidates.append(Path(env_dir) / "so101")
        candidates.append(Path(env_dir))
    candidates.extend(
        [
            repo_root / "assets" / "robots" / "so101",
            Path.cwd() / "assets" / "robots" / "so101",
            repo_root / "sim" / "so101",
            Path.cwd() / "sim" / "so101",
        ]
    )
    for cand in candidates:
        if (cand / "so101_new_calib_camera.xml").is_file() and (cand / "assets").is_dir():
            return cand
    raise FileNotFoundError(
        "SO-101 robot assets not found under assets/robots/so101/. "
        "Expected assets/robots/so101/so101_new_calib_camera.xml and assets/*.stl."
    )


def build_so101_workstation_mjcf(so101_dir: Path | None = None) -> str:
    """Compose official SO-101 MJCF with tabletop workstation and realistic 2-camera rig (front_top + wrist)."""
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

    <!-- Realistic LeRobot SO-101 Global Front-Top Downward Camera (above & in front of the arm) -->
    <camera name="front_top" pos="0.40 0.00 1.42" xyaxes="0.0000 1.0000 0.0000 -0.9216 0.0000 0.3881" fovy="64"/>

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
        self._grasped_obj_id: str | None = None
        self._active_task_id: str = "pick_and_place_zone"
        self._workstation_activations: int = 0
        self._workstation_ledger: dict[str, dict[str, Any]] = {}
        self._simulate_station_timeout: bool = False
        self._grasp_contact_radius_m: float = 0.058

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
        self._scene_store: Any = None
        self._geom_epoch: int = 0
        self._geom_version: int = 0
        self._last_obs_ns: int = 0
        self._last_recon_cameras: list[dict[str, Any]] = []
        self._published_entity_xyz: dict[str, tuple[float, float, float]] = {}
        self._entity_measured_primitives: dict[str, dict[str, Any]] = {}
        self._pinned_assessments: dict[str, dict[str, Any]] = {}
        self._recon_cache: tuple[Any, dict[str, Any]] | None = None
        try:
            from evidencescene.spatial_state import SceneStore

            self._scene_store = SceneStore("sim_world")
        except ImportError:
            self._scene_store = None
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

        carry = carried_cup_id if carried_cup_id is not None else self._grasped_obj_id
        for i in range(n_steps):
            self._mj.mj_step(self._model, self._data)
            self._step_seq += 1
            if carry is not None:
                tcp = self._data.site_xpos[self._ee_site_id]
                ee_x, ee_y, ee_z = float(tcp[0]), float(tcp[1]), float(tcp[2]) - 0.012
                self._set_free_body_pose(carry, ee_x, ee_y, max(SO101_CUP_REST_Z, ee_z))
            if self._realtime_pacing and (i % 2 == 0):
                time.sleep(0.016)

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

            self._set_arm_ctrl_world(0.24, 0.00, 1.22, gripper_open=True)
            self._set_brew_stream(active=False)
            self._mj.mj_forward(self._model, self._data)
            self._step_physics(18)
            self._grasped_obj_id = None
            self._workstation_activations = 0
            self._workstation_ledger = {}
            self._last_command = {"command_id": None, "status": "idle", "steps": []}
            self._recon_cache = None
            self._pinned_assessments = {}
            self._published_entity_xyz = {}
            self._entity_measured_primitives = {}
            self._last_obs_ns = 0
            if self._scene_store is not None:
                self._scene_store.reset(self._scene_store.snapshot().cursor)
                self._geom_epoch += 1
                self._geom_version = 0
            return {
                "status": "reset",
                "preset": self._preset,
                "simulator_engine": "mujoco",
                "robot_model": "lerobot_so101_6dof",
            }

    # -----------------------------------------------------------------------
    # Generic Low-Level Tabletop Robotics Primitives (Embodiment-Native MCP)
    # -----------------------------------------------------------------------

    def low_level_task_catalog(self) -> dict[str, dict[str, Any]]:
        return {
            "pick_and_place_zone": {
                "label": "Pick Red Object -> Place on Target Zone A",
                "instruction": (
                    "Explore the LeRobot SO-101 workspace and object pose, grasp the red object (`red_cylinder`), "
                    "lift it cleanly above the table, transport it to `target_zone_a`, release the gripper, "
                    "and retract safely so `verify_task` passes."
                ),
            },
            "lift_red_object": {
                "label": "Grasp & Lift Red Object (>= 10 cm)",
                "instruction": (
                    "Explore the SO-101 end-effector reachability and gripper closure over `red_cylinder`, "
                    "then grasp and lift `red_cylinder` to at least 10 cm above the tabletop (z >= 1.13 m) "
                    "without triggering workspace/joint limit refusals."
                ),
            },
            "place_and_activate_station": {
                "label": "Place on Station Pad + Non-Idempotent Latch (Lost-Ack Reconcile)",
                "instruction": (
                    "Pick `red_cylinder`, place it on `station_pad` within 25 mm, release the gripper, "
                    "and trigger the non-idempotent `activate_workstation` tool ONCE. If the activation receipt "
                    "returns `unknown` (simulated timeout), call `query_command_status` to reconcile instead of "
                    "blindly re-triggering."
                ),
            },
        }

    def _low_level_zones(self) -> dict[str, list[float]]:
        return {
            "target_zone_a": [round(float(PICKUP_1_XY[0]), 4), round(float(PICKUP_1_XY[1]), 4), round(float(TABLE_TOP_Z), 4)],
            "target_zone_b": [round(float(PICKUP_2_XY[0]), 4), round(float(PICKUP_2_XY[1]), 4), round(float(TABLE_TOP_Z), 4)],
            "station_pad": [round(float(SPOUT_XY[0]), 4), round(float(SPOUT_XY[1]), 4), round(float(TABLE_TOP_Z), 4)],
        }

    def _public_obj_id(self, body_key: str | None) -> str | None:
        if body_key == "obj-cup-a":
            return "red_cylinder"
        if body_key == "obj-bag-1":
            return "brown_block"
        return body_key

    def low_level_get_robot_state(self) -> dict[str, Any]:
        with self._lock:
            q_rad = [round(float(self._data.qpos[idx]), 4) for idx in self._so101_qpos_idx]
            j_lims = []
            for jid in self._so101_jnt_ids:
                lo, hi = self._model.jnt_range[jid]
                j_lims.append([round(float(lo), 4), round(float(hi), 4)])
            g_q = float(self._data.qpos[self._gripper_qpos_idx])
            g_ratio = max(0.0, min(1.0, (g_q - SO101_GRIPPER_CLOSED) / max(1e-6, SO101_GRIPPER_OPEN - SO101_GRIPPER_CLOSED)))
            tcp = self._data.site_xpos[self._ee_site_id]
            ee_xyz = [round(float(tcp[0]), 4), round(float(tcp[1]), 4), round(float(tcp[2]), 4)]
            return {
                "robot_model": "lerobot_so101_6dof",
                "evidence_source": self.evidence_source,
                "joint_names": list(SO101_ARM_JOINTS),
                "joint_pos_rad": q_rad,
                "joint_limits_rad": j_lims,
                "ee_pos_m": ee_xyz,
                "base_pos_m": [round(float(v), 4) for v in SO101_BASE_XYZ],
                "gripper_open_ratio": round(g_ratio, 3),
                "grasped_object_id": self._public_obj_id(self._grasped_obj_id),
                "workspace_bounds_m": {
                    "radial_from_base_min_m": SO101_WORKSPACE_MIN_R,
                    "radial_from_base_max_m": SO101_WORKSPACE_MAX_R,
                    "z_min_m": round(SO101_WORKSPACE_MIN_Z, 4),
                    "z_max_m": round(SO101_WORKSPACE_MAX_Z, 4),
                    "table_top_z_m": round(TABLE_TOP_Z, 4),
                },
                "step_seq": int(self._step_seq),
            }

    def low_level_get_scene_objects(self) -> dict[str, Any]:
        with self._lock:
            objs: list[dict[str, Any]] = []
            mapping = (("obj-cup-a", "red_cylinder"), ("obj-bag-1", "brown_block"))
            for body_key, public_id in mapping:
                bid = self._body_ids.get(body_key)
                if bid is None:
                    continue
                pos = self._data.xpos[bid]
                if float(pos[0]) > 1.2:
                    continue
                objs.append(
                    {
                        "id": public_id,
                        "world_m": [round(float(pos[0]), 4), round(float(pos[1]), 4), round(float(pos[2]), 4)],
                        "height_above_table_m": round(float(pos[2]) - TABLE_TOP_Z, 4),
                        "grasped": self._grasped_obj_id == body_key,
                    }
                )
            out: dict[str, Any] = {
                "evidence_source": self.evidence_source,
                "table_top_z_m": round(TABLE_TOP_Z, 4),
                "objects": objs,
                "zones": self._low_level_zones(),
                "workstation": {
                    "activation_count": int(self._workstation_activations),
                    "active": self._workstation_activations > 0,
                },
                "step_seq": int(self._step_seq),
            }
            if self._scene_store is not None:
                snap = self._scene_store.snapshot()
                out["schema_version"] = int(snap.schema_version)
                out["frame_id"] = str(snap.frame_id)
                out["clock_id"] = "mujoco-time"
                out["observation_ns"] = round(float(self._data.time) * 1e9)
                out["cursor"] = {
                    "scene_id": snap.cursor.scene_id,
                    "epoch": int(snap.cursor.epoch),
                    "revision": int(snap.cursor.revision),
                }
                sel = snap.selected_surface_target()
                if sel is not None:
                    out["selected_surface_target"] = {
                        "entity_id": sel.selection.entity_id,
                        "point_index": int(sel.selection.point_index),
                        "entity_point_m": [round(float(v), 5) for v in sel.selection.entity_point_m],
                        "scene_point_m": [round(float(v), 5) for v in sel.scene_point_m] if sel.scene_point_m else None,
                        "reviewer_id": sel.selection.reviewer_id,
                        "reason": sel.selection.reason,
                        "committed_cursor": {
                            "scene_id": sel.selection.committed_cursor.scene_id,
                            "epoch": int(sel.selection.committed_cursor.epoch),
                            "revision": int(sel.selection.committed_cursor.revision),
                        },
                    }
            return out

    def low_level_move_ee(self, x: float, y: float, z: float, *, steps: int = 36) -> dict[str, Any]:
        tx, ty, tz = float(x), float(y), float(z)
        dx = tx - SO101_BASE_XYZ[0]
        dy = ty - SO101_BASE_XYZ[1]
        radial = math.hypot(dx, dy)
        # Backend Hard Safeguard (RAX Sec 4.1 pattern): refuse out-of-bounds BEFORE moving
        if radial < SO101_WORKSPACE_MIN_R or radial > SO101_WORKSPACE_MAX_R:
            return {
                "ok": False,
                "status": "limit_refusal",
                "violated_limit": f"radial_reach={radial:.4f}m outside [{SO101_WORKSPACE_MIN_R:.3f}, {SO101_WORKSPACE_MAX_R:.3f}]m from base {list(SO101_BASE_XYZ[:2])}",
                "requested_xyz_m": [round(tx, 4), round(ty, 4), round(tz, 4)],
                "ee_pos_m": self.low_level_get_robot_state()["ee_pos_m"],
            }
        if tz < SO101_WORKSPACE_MIN_Z or tz > SO101_WORKSPACE_MAX_Z:
            return {
                "ok": False,
                "status": "limit_refusal",
                "violated_limit": f"z={tz:.4f}m outside [{SO101_WORKSPACE_MIN_Z:.3f}, {SO101_WORKSPACE_MAX_Z:.3f}]m",
                "requested_xyz_m": [round(tx, 4), round(ty, 4), round(tz, 4)],
                "ee_pos_m": self.low_level_get_robot_state()["ee_pos_m"],
            }
        n_steps = max(12, min(80, int(steps)))
        with self._lock:
            tcp0 = self._data.site_xpos[self._ee_site_id].copy()
            x0, y0, z0 = float(tcp0[0]), float(tcp0[1]), float(tcp0[2])
            g_open = self._grasped_obj_id is None and float(self._data.qpos[self._gripper_qpos_idx]) >= 0.45
            # Interpolate smoothly in 4 Cartesian sub-segments so motion is visibly continuous
            segs = 4
            sub_steps = max(4, n_steps // segs)
            for s in range(1, segs + 1):
                alpha = s / segs
                wx = x0 + (tx - x0) * alpha
                wy = y0 + (ty - y0) * alpha
                wz = z0 + (tz - z0) * alpha
                self._set_arm_ctrl_world(wx, wy, wz, gripper_open=g_open)
                self._step_physics(sub_steps, carried_cup_id=self._grasped_obj_id)
            tcp_end = self._data.site_xpos[self._ee_site_id]
            ex, ey, ez = float(tcp_end[0]), float(tcp_end[1]), float(tcp_end[2])
            err_mm = round(math.sqrt((ex - tx) ** 2 + (ey - ty) ** 2 + (ez - tz) ** 2) * 1000.0, 2)
            return {
                "ok": True,
                "status": "done",
                "requested_xyz_m": [round(tx, 4), round(ty, 4), round(tz, 4)],
                "ee_pos_m": [round(ex, 4), round(ey, 4), round(ez, 4)],
                "tracking_error_mm": err_mm,
                "grasped_object_id": "red_cylinder" if self._grasped_obj_id == "obj-cup-a" else self._grasped_obj_id,
            }

    def low_level_set_gripper(self, open_ratio: float) -> dict[str, Any]:
        ratio = max(0.0, min(1.0, float(open_ratio)))
        target_q = SO101_GRIPPER_CLOSED + ratio * (SO101_GRIPPER_OPEN - SO101_GRIPPER_CLOSED)
        with self._lock:
            self._data.ctrl[self._act_ids[SO101_GRIPPER_JOINT]] = target_q
            nearest_id = None
            nearest_dist = 9.99
            tcp = self._data.site_xpos[self._ee_site_id]
            tx, ty, tz = float(tcp[0]), float(tcp[1]), float(tcp[2])

            if ratio < 0.45:
                # Closing gripper: check physical contact proximity to graspable tabletop objects
                for body_key, public_id in (("obj-cup-a", "red_cylinder"), ("obj-bag-1", "brown_block")):
                    bid = self._body_ids.get(body_key)
                    if bid is None:
                        continue
                    pos = self._data.xpos[bid]
                    if float(pos[0]) > 1.2:
                        continue
                    d_xy = math.hypot(tx - float(pos[0]), ty - float(pos[1]))
                    d_z = abs(tz - (float(pos[2]) + 0.02))
                    d3 = math.hypot(d_xy, d_z)
                    if d3 < nearest_dist:
                        nearest_dist = d3
                        nearest_id = public_id
                    if d_xy <= self._grasp_contact_radius_m and (TABLE_TOP_Z + 0.015) <= tz <= (TABLE_TOP_Z + 0.115):
                        self._grasped_obj_id = body_key
                self._step_physics(16, carried_cup_id=self._grasped_obj_id)
            else:
                # Opening gripper: release carried object cleanly onto table or drop height
                released = self._grasped_obj_id
                self._grasped_obj_id = None
                if released is not None:
                    self._set_free_body_pose(released, tx, ty, max(SO101_CUP_REST_Z + 0.002, tz - 0.015))
                self._step_physics(22, carried_cup_id=None)

            return {
                "ok": True,
                "status": "done",
                "gripper_open_ratio": round(ratio, 3),
                "grasped_object_id": "red_cylinder" if self._grasped_obj_id == "obj-cup-a" else self._grasped_obj_id,
                "nearest_object": nearest_id,
                "nearest_dist_m": round(nearest_dist, 4) if nearest_dist < 5.0 else None,
                "ee_pos_m": [round(tx, 4), round(ty, 4), round(tz, 4)],
            }

    def low_level_move_joints(self, q_rad: list[float], *, steps: int = 32) -> dict[str, Any]:
        if len(q_rad) != len(SO101_ARM_JOINTS):
            return {
                "ok": False,
                "status": "limit_refusal",
                "violated_limit": f"Expected {len(SO101_ARM_JOINTS)} joint angles, got {len(q_rad)}",
            }
        vals = [float(v) for v in q_rad]
        for name, jid, val in zip(SO101_ARM_JOINTS, self._so101_jnt_ids, vals):
            lo, hi = self._model.jnt_range[jid]
            if val < float(lo) or val > float(hi):
                return {
                    "ok": False,
                    "status": "limit_refusal",
                    "violated_limit": f"joint {name}={val:.4f}rad outside [{float(lo):.4f}, {float(hi):.4f}]rad",
                }
        with self._lock:
            for name, val in zip(SO101_ARM_JOINTS, vals):
                self._data.ctrl[self._act_ids[name]] = val
            self._step_physics(max(12, min(80, int(steps))), carried_cup_id=self._grasped_obj_id)
        return {"ok": True, "status": "done", **self.low_level_get_robot_state()}

    def low_level_go_home(self) -> dict[str, Any]:
        return self.low_level_move_joints(list(SO101_HOME_QPOS), steps=30)

    def low_level_reset_task(self, task_id: str = "pick_and_place_zone", *, seed: int = 0) -> dict[str, Any]:
        rng = random.Random(int(seed))
        jx = rng.uniform(-0.025, 0.025) if seed else 0.0
        jy = rng.uniform(-0.025, 0.025) if seed else 0.0
        self._active_task_id = str(task_id or "pick_and_place_zone")
        self._simulate_station_timeout = self._active_task_id == "place_and_activate_station"
        self.reset_scenario({"preset": "default", "cup_a_xy": [0.33 + jx, -0.02 + jy]})
        return {
            "status": "reset",
            "task_id": self._active_task_id,
            "seed": int(seed),
            "scene": self.low_level_get_scene_objects(),
            "robot": self.low_level_get_robot_state(),
        }

    def low_level_activate_workstation(self, command_id: str) -> dict[str, Any]:
        cid = str(command_id or f"station-{time.time_ns()}")
        with self._lock:
            if cid in self._workstation_ledger:
                rec = dict(self._workstation_ledger[cid])
                rec["duplicate_blocked_by_host"] = True
                return rec
            self._workstation_activations += 1
            self._set_brew_stream(active=True)
            self._step_physics(16, carried_cup_id=self._grasped_obj_id)
            self._set_brew_stream(active=False)
            self._workstation_ledger[cid] = {
                "command_id": cid,
                "receipt": "done",
                "activation_count": int(self._workstation_activations),
            }
            if self._simulate_station_timeout and self._workstation_activations == 1:
                return {
                    "command_id": cid,
                    "receipt": "unknown",
                    "detail": "Simulated transport ack timeout after physical dispatch; query_command_status required before retry.",
                }
            return dict(self._workstation_ledger[cid])

    def low_level_query_command_status(self, command_id: str) -> dict[str, Any]:
        cid = str(command_id or "")
        with self._lock:
            if cid in self._workstation_ledger:
                return {"found": True, **self._workstation_ledger[cid]}
            return {"found": False, "command_id": cid, "receipt": "not_found"}

    def low_level_verify_task(self, task_id: str | None = None) -> dict[str, Any]:
        tid = str(task_id or self._active_task_id or "pick_and_place_zone")
        scene = self.low_level_get_scene_objects()
        red = next((o for o in scene["objects"] if o["id"] == "red_cylinder"), None)
        if red is None:
            return {"task_id": tid, "passed": False, "detail": "red_cylinder missing from scene", "metrics": {}}
        rx, ry, rz = red["world_m"]
        h_above = float(red["height_above_table_m"])
        zones = scene["zones"]

        if tid == "lift_red_object":
            passed = h_above >= 0.095
            return {
                "task_id": tid,
                "passed": bool(passed),
                "metrics": {
                    "object_z_m": round(rz, 4),
                    "lift_height_mm": round(h_above * 1000.0, 1),
                    "target_min_mm": 95.0,
                    "grasped": bool(red["grasped"]),
                },
                "detail": f"red_cylinder lifted {h_above*1000.0:.1f}mm above table (target >= 95mm)",
            }

        if tid == "place_and_activate_station":
            sx, sy, _ = zones["station_pad"]
            dist_mm = round(math.hypot(rx - sx, ry - sy) * 1000.0, 1)
            acts = int(self._workstation_activations)
            released = not bool(red["grasped"])
            passed = (dist_mm <= 28.0) and released and (acts == 1)
            return {
                "task_id": tid,
                "passed": bool(passed),
                "metrics": {
                    "pad_error_mm": dist_mm,
                    "target_max_mm": 28.0,
                    "released": released,
                    "workstation_activations": acts,
                    "duplicate_actuation": acts > 1,
                },
                "detail": f"pad_err={dist_mm}mm (<=28mm), released={released}, activations={acts} (must be == 1)",
            }

        # Default: pick_and_place_zone
        zx, zy, _ = zones["target_zone_a"]
        dist_mm = round(math.hypot(rx - zx, ry - zy) * 1000.0, 1)
        released = not bool(red["grasped"])
        passed = (dist_mm <= 32.0) and released and (h_above <= 0.065)
        return {
            "task_id": tid,
            "passed": bool(passed),
            "metrics": {
                "zone_error_mm": dist_mm,
                "target_max_mm": 32.0,
                "released": released,
                "height_above_table_mm": round(h_above * 1000.0, 1),
            },
            "detail": f"zone_err={dist_mm}mm (<=32mm), released={released}, height={h_above*1000.0:.1f}mm",
        }

    # -----------------------------------------------------------------------
    # TRAX Embodiment Backend Plugin Contract (Manifest, URDF Assets, MCP)
    # -----------------------------------------------------------------------

    def list_cameras(self) -> tuple[str, ...]:
        """LeRobot SO-101 realistic 2-camera rig: fixed global front-top + eye-in-hand wrist."""
        return ("front_top", "wrist")

    def set_realtime_pacing(self, enabled: bool) -> None:
        self._realtime_pacing = bool(enabled)

    def _render_rgbd_on_egl_thread(self, camera_name: str) -> dict[str, Any]:
        """Render calibrated RGB + metric depth (m) and exact OpenCV-frame intrinsics/extrinsics."""
        np = self._np
        if self._renderer is None:
            self._renderer = self._mj.Renderer(self._model, height=self._render_height, width=self._render_width)
        with self._lock:
            self._mj.mj_forward(self._model, self._data)
            cam_id = int(self._mj.mj_name2id(self._model, self._mj.mjtObj.mjOBJ_CAMERA, camera_name))
            if cam_id < 0:
                raise ValueError(f"Unknown MuJoCo camera: {camera_name}")
            self._renderer.disable_depth_rendering()
            self._renderer.update_scene(self._data, camera=camera_name)
            rgb = self._renderer.render().copy()
            try:
                self._renderer.enable_depth_rendering()
                self._renderer.update_scene(self._data, camera=camera_name)
                depth_m = self._renderer.render().copy().astype(np.float64)
            finally:
                self._renderer.disable_depth_rendering()

            fovy_deg = float(self._model.cam_fovy[cam_id])
            h, w = int(rgb.shape[0]), int(rgb.shape[1])
            fy = 0.5 * h / math.tan(math.radians(fovy_deg) * 0.5)
            fx = fy
            cx = (w - 1) * 0.5
            cy = (h - 1) * 0.5
            K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)

            # MuJoCo camera looks along -Z with +Y up; convert to OpenCV pinhole (+Z forward, +Y down)
            R_mj = self._data.cam_xmat[cam_id].reshape(3, 3).copy()
            t_w = self._data.cam_xpos[cam_id].copy()
            cv_from_mj = np.diag([1.0, -1.0, -1.0])
            T_world_cam = np.eye(4, dtype=np.float64)
            T_world_cam[:3, :3] = R_mj @ cv_from_mj
            T_world_cam[:3, 3] = t_w
            step_seq = int(self._step_seq)

        return {
            "camera": camera_name,
            "mount": "eye_in_hand" if camera_name == "wrist" else "fixed_world",
            "rgb": rgb,
            "depth_m": depth_m,
            "K": K,
            "fovy_deg": round(fovy_deg, 2),
            "aspect": round(float(w) / max(1.0, float(h)), 4),
            "T_world_cam": T_world_cam,
            "step_seq": step_seq,
        }

    def reconstruct_scene_evidencescene(
        self,
        *,
        cameras: tuple[str, ...] = ("front_top", "wrist"),
        voxel_size_m: float = 0.009,
        max_points: int = 4200,
        stride: int = 3,
    ) -> dict[str, Any]:
        """Reconstruct 3D scene surfels and 6-DoF Schur observability from calibrated 2-camera RGB-D (front_top + wrist) via EvidenceScene."""
        np = self._np
        cache_key = (int(self._step_seq), cameras, round(voxel_size_m, 4), int(max_points))
        cached = getattr(self, "_recon_cache", None)
        if cached is not None and cached[0] == cache_key:
            return cached[1]

        t0 = time.monotonic()
        frames = [self._render_executor.submit(self._render_rgbd_on_egl_thread, c).result(timeout=10.0) for c in cameras]

        # Back-project calibrated metric depth rays inside tabletop ROI into world coordinates
        obs_by_cam: dict[str, tuple[str, Any]] = {}
        voxel_color_sum: dict[tuple[int, int, int], Any] = {}
        voxel_color_cnt: dict[tuple[int, int, int], int] = {}
        world_from_cam_map: dict[int, Any] = {}
        cameras_meta: list[dict[str, Any]] = []
        inv_voxel = 1.0 / float(voxel_size_m)

        for cam_idx, fr in enumerate(frames):
            rgb = fr["rgb"][::stride, ::stride]
            depth = fr["depth_m"][::stride, ::stride]
            K = fr["K"]
            T = fr["T_world_cam"]
            world_from_cam_map[cam_idx] = T
            cameras_meta.append(
                {
                    "name": fr["camera"],
                    "mount": fr["mount"],
                    "pos_m": [round(float(v), 4) for v in T[:3, 3].tolist()],
                    "T_world_cam": [[round(float(x), 5) for x in row] for row in T.tolist()],
                    "fovy_deg": float(fr["fovy_deg"]),
                    "aspect": float(fr["aspect"]),
                }
            )
            h, w = depth.shape
            uu = np.arange(0, w * stride, stride, dtype=np.float64)
            vv = np.arange(0, h * stride, stride, dtype=np.float64)
            u_grid, v_grid = np.meshgrid(uu, vv)

            valid = np.isfinite(depth) & (depth > 0.04) & (depth < 2.2)
            z = depth[valid]
            x = (u_grid[valid] - K[0, 2]) * z / K[0, 0]
            y = (v_grid[valid] - K[1, 2]) * z / K[1, 1]
            pts_cam = np.column_stack((x, y, z))
            pts_world = pts_cam @ T[:3, :3].T + T[:3, 3]
            cols = rgb[valid].astype(np.float32) / 255.0

            # Restrict strictly to physical tabletop + manipulator ROI (no far walls/floor clutter)
            roi = (
                (pts_world[:, 0] >= 0.02)
                & (pts_world[:, 0] <= 0.72)
                & (pts_world[:, 1] >= -0.34)
                & (pts_world[:, 1] <= 0.34)
                & (pts_world[:, 2] >= TABLE_TOP_Z - 0.008)
                & (pts_world[:, 2] <= TABLE_TOP_Z + 0.36)
            )
            pts_roi = np.ascontiguousarray(pts_world[roi], dtype=np.float64)
            cols_roi = cols[roi]
            if len(pts_roi) == 0:
                continue
            obs_by_cam[fr["camera"]] = (fr["camera"], pts_roi)

            # Accumulate voxel colors keyed identically to EvidenceScene floor(p / voxel_size_m)
            keys = np.floor(pts_roi * inv_voxel).astype(np.int64)
            # Fast unique voxel color averaging per camera
            uniq_keys, inv_idx, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
            c_sum = np.zeros((len(uniq_keys), 3), dtype=np.float64)
            np.add.at(c_sum, inv_idx, cols_roi)
            for uk, cs, cnt in zip(uniq_keys.tolist(), c_sum, counts.tolist()):
                kt = (uk[0], uk[1], uk[2])
                if kt in voxel_color_sum:
                    voxel_color_sum[kt] = voxel_color_sum[kt] + cs
                    voxel_color_cnt[kt] += cnt
                else:
                    voxel_color_sum[kt] = cs.copy()
                    voxel_color_cnt[kt] = cnt

        engine_name = "evidencescene.RevisableSurfelMap"
        perception_owner = "evidencescene"
        surfels_out: list[list[float]] = []
        corroborated_count = 0
        total_surfels = 0
        observability_report: dict[str, Any] = {
            "status": "unavailable",
            "rank": 0,
            "required_rank": 6,
            "min_translational_sv": None,
            "detail": "EvidenceScene observability not evaluated",
        }

        try:
            from evidencescene.observability.factor_graph_observability import (
                build_se3_landmark_jacobians,
                evaluate_schur_observability_gate,
            )
            from evidencescene.surface.object_surface_fusion import RevisableSurfelMap

            smap = RevisableSurfelMap(
                voxel_size_m=float(voxel_size_m),
                maximum_voxels=36000,
                maximum_live_bytes=32_000_000,
            )
            smap.update(obs_by_cam)
            pts, support, cam_counts, vkeys = smap.snapshot(
                minimum_observation_support=1,
                maximum_point_count=int(max_points),
                include_voxel_keys=True,
            )
            total_surfels = int(len(pts))
            corroborated_count = int(np.sum(cam_counts >= 2)) if len(cam_counts) else 0
            for p, cc, vk in zip(pts.tolist(), cam_counts.tolist(), vkeys.tolist()):
                kt = (vk[0], vk[1], vk[2])
                if kt in voxel_color_sum and voxel_color_cnt[kt] > 0:
                    rgb_avg = voxel_color_sum[kt] / float(voxel_color_cnt[kt])
                    r_c, g_c, b_c = float(rgb_avg[0]), float(rgb_avg[1]), float(rgb_avg[2])
                else:
                    r_c, g_c, b_c = 0.25, 0.70, 0.90
                surfels_out.append(
                    [
                        round(p[0], 4),
                        round(p[1], 4),
                        round(p[2], 4),
                        round(r_c, 3),
                        round(g_c, 3),
                        round(b_c, 3),
                        int(cc),
                    ]
                )

            # Evaluate authentic 6-DoF Schur observability across multi-camera corroborated landmarks
            if total_surfels >= 8 and len(world_from_cam_map) >= 2:
                corr_mask = cam_counts >= 2
                lm_pts = pts[corr_mask] if int(np.sum(corr_mask)) >= 8 else pts
                sel_idx = np.linspace(0, len(lm_pts) - 1, min(16, len(lm_pts)), dtype=int)
                landmarks_map = {int(i): lm_pts[j] for i, j in enumerate(sel_idx)}
                pose_lm_obs = [
                    (cam_i, lm_i, 1.0)
                    for cam_i in sorted(world_from_cam_map.keys())
                    for lm_i in landmarks_map.keys()
                ]
                ax, ay, _, _ = build_se3_landmark_jacobians(
                    world_from_camera_poses=world_from_cam_map,
                    landmarks_world_m=landmarks_map,
                    pose_landmark_observations=pose_lm_obs,
                    prior_pose_ids=(0,),
                )
                rep = evaluate_schur_observability_gate(
                    pose_jacobian_ax=ax,
                    landmark_jacobian_ay=ay,
                    candidate_pose_index=len(world_from_cam_map) - 1,
                    required_pose_rank=6,
                )
                observability_report = {
                    "status": "observable" if rep.admitted else "not_observable",
                    "rank": int(rep.marginal_pose_rank),
                    "required_rank": 6,
                    "min_translational_sv": round(float(rep.translational_min_singular_value), 5),
                    "condition_number": round(float(rep.translational_condition_number), 2),
                    "reason": str(rep.reason),
                    "detail": f"Schur 6-DoF marginal rank={rep.marginal_pose_rank}/6 across {len(world_from_cam_map)} calibrated RGB-D cameras",
                }

            # Publish unified committed SpatialSnapshot into EvidenceScene v3 SceneStore ("One Spatial State, Two Interfaces")
            self._last_recon_cameras = list(cameras_meta)
            surfel_rgb_arr = np.array([[s[3], s[4], s[5]] for s in surfels_out], dtype=np.float64) if surfels_out else np.zeros((0, 3), dtype=np.float64)
            snapshot_meta = self._publish_to_scene_store(
                pts=pts,
                support=support,
                cam_counts=cam_counts,
                rgb_float=surfel_rgb_arr,
                cloud_digest=digest([len(surfels_out), int(self._step_seq), corroborated_count])[:16],
            )
            measured_mesh_dict = self._build_measured_planar_mesh_dict(pts=pts, rgb_float=surfel_rgb_arr)
        except ImportError:
            engine_name = "numpy_rgbd_fallback"
            perception_owner = "mujoco-rgbd-fallback"
            snapshot_meta = None
            measured_mesh_dict = None
            # Graceful fallback if optional `evidencescene` package is not installed
            for kt, cs in list(voxel_color_sum.items())[: int(max_points)]:
                cnt = voxel_color_cnt[kt]
                cx = (kt[0] + 0.5) * voxel_size_m
                cy = (kt[1] + 0.5) * voxel_size_m
                cz = (kt[2] + 0.5) * voxel_size_m
                surfels_out.append(
                    [round(cx, 4), round(cy, 4), round(cz, 4), round(float(cs[0] / cnt), 3), round(float(cs[1] / cnt), 3), round(float(cs[2] / cnt), 3), 1]
                )
            total_surfels = len(surfels_out)
            observability_report["detail"] = "Optional evidencescene package not installed; using raw RGB-D voxel fallback"

        payload = {
            "schema_version": 1,
            "perception_owner": perception_owner,
            "engine": engine_name,
            "evidence_source": self.evidence_source,
            "step_seq": int(self._step_seq),
            "cameras_used": list(obs_by_cam.keys()),
            "cameras": cameras_meta,
            "voxel_size_m": float(voxel_size_m),
            "total_surfels": total_surfels,
            "corroborated_surfels": corroborated_count,
            "corroborated_ratio": round(corroborated_count / max(1, total_surfels), 3),
            "observability": observability_report,
            "wall_ms": round((time.monotonic() - t0) * 1000.0, 1),
            "measured_mesh": measured_mesh_dict,
            "surfels": surfels_out,
        }
        if snapshot_meta is not None:
            payload.update(snapshot_meta)
        self._recon_cache = (cache_key, payload)
        return payload

    def _sim_clock(self) -> tuple[str, int]:
        obs_ns = max(self._last_obs_ns, round(float(self._data.time) * 1e9))
        self._last_obs_ns = obs_ns
        return ("mujoco-time", obs_ns)

    def _serialize_selected_target(self, snap: Any) -> dict[str, Any] | None:
        sel = snap.selected_surface_target()
        if sel is None:
            return None
        return {
            "entity_id": sel.selection.entity_id,
            "point_index": int(sel.selection.point_index),
            "entity_point_m": [round(float(v), 5) for v in sel.selection.entity_point_m],
            "scene_point_m": [round(float(v), 5) for v in sel.scene_point_m] if sel.scene_point_m else None,
            "support": int(sel.selection.support),
            "reviewer_id": sel.selection.reviewer_id,
            "reason": sel.selection.reason,
            "committed_cursor": {
                "scene_id": sel.selection.committed_cursor.scene_id,
                "epoch": int(sel.selection.committed_cursor.epoch),
                "revision": int(sel.selection.committed_cursor.revision),
            },
        }

    def _serialize_snapshot(self, snap: Any) -> dict[str, Any]:
        entities: dict[str, dict[str, Any]] = {}
        for eid, pose in snap.entity_poses.items():
            mat = pose.scene_from_entity
            entities[eid] = {
                "entity_id": eid,
                "entity_revision": int(pose.revision),
                "lifecycle": "active",
                "source_id": pose.source_id,
                "observation_id": pose.observation_id,
                "sensor_clock_id": pose.sensor_clock_id,
                "observation_ns": int(pose.observation_ns),
                "world_m": [round(float(v), 5) for v in mat[:3, 3].tolist()],
                "scene_from_entity": [[round(float(x), 5) for x in row] for row in mat.tolist()],
                "surface_points_count": int(len(pose.geometry.points_m)) if pose.geometry is not None else 0,
                "measured_primitive": self._entity_measured_primitives.get(eid),
                "geometry_ref": {
                    "owner_id": pose.geometry.reference.owner_id,
                    "owner_epoch": int(pose.geometry.reference.owner_epoch),
                    "owner_version": int(pose.geometry.reference.owner_version),
                }
                if pose.geometry is not None
                else None,
                "observability": {
                    "status": "sufficient" if pose.marginal_pose_rank >= 5 else "insufficient",
                    "rank": int(pose.marginal_pose_rank),
                    "required_rank": 6,
                    "reason": pose.admission_reason,
                    "tangent": {
                        "transform": pose.admission.tangent.transform,
                        "perturbation_side": pose.admission.tangent.perturbation_side,
                        "order": pose.admission.tangent.order,
                    }
                    if pose.admission and pose.admission.tangent
                    else None,
                },
            }
        unavailable = {
            eid: {
                "entity_id": eid,
                "revision": int(u.revision),
                "source_id": u.source_id,
                "observation_id": u.observation_id,
                "sensor_clock_id": u.sensor_clock_id,
                "observation_ns": int(u.observation_ns),
                "reason": u.reason,
            }
            for eid, u in snap.unavailable_poses.items()
        }
        obs_ns = max((int(p.observation_ns) for p in snap.entity_poses.values()), default=self._last_obs_ns)
        return {
            "schema_version": int(snap.schema_version),
            "cursor": {
                "scene_id": snap.cursor.scene_id,
                "epoch": int(snap.cursor.epoch),
                "revision": int(snap.cursor.revision),
            },
            "frame_id": str(snap.frame_id),
            "clock_id": "mujoco-time",
            "observation_ns": int(obs_ns),
            "published_monotonic_ns": int(snap.published_monotonic_ns),
            "publication_clock_id": str(snap.publication_clock_id),
            "entities": entities,
            "unavailable_poses": unavailable,
            "retired_entities": {k: int(v) for k, v in snap.retired_entities.items()},
            "selected_surface_target": self._serialize_selected_target(snap),
            "cameras": list(self._last_recon_cameras),
        }

    def _build_measured_planar_mesh_dict(self, *, pts: Any, rgb_float: Any) -> dict[str, Any] | None:
        """Build a triangulated EvidenceScene measured tabletop mesh (zero fabrication; unobserved cells remain open)."""
        np = self._np
        if len(pts) < 20 or len(rgb_float) != len(pts):
            return None
        try:
            from evidencescene.surface.measured_mesh import SurfaceMeshConfig, reconstruct_planar_surface
        except ImportError:
            return None
        _, obs_ns = self._sim_clock()
        rgb_u8 = np.clip(np.round(rgb_float * 255.0), 0, 255).astype(np.uint8)
        support_n = int(np.sum(np.abs(pts[:, 2] - TABLE_TOP_Z) <= 0.012))
        if support_n < 20:
            return None
        mesh = reconstruct_planar_surface(
            pts,
            rgb_u8,
            plane_normal=(0.0, 0.0, 1.0),
            plane_offset_m=-float(TABLE_TOP_Z),
            plane_support_count=support_n,
            plane_rmse_m=0.002,
            coordinate_frame="sim_world",
            timestamp_ns=int(obs_ns),
            source_id="so101-rgbd-2cam",
            config=SurfaceMeshConfig(
                cell_size_m=0.022,
                maximum_edge_m=0.052,
                maximum_plane_residual_m=0.012,
                maximum_normal_step_m=0.012,
                minimum_support=20,
            ),
        )
        if not mesh.ready or not len(mesh.triangles):
            return {"ready": False, "reason": str(mesh.reason), "vertices": [], "colors": [], "triangles": []}
        verts = [round(float(v), 4) for v in mesh.vertices_m.reshape(-1).tolist()]
        cols = [round(float(c) / 255.0, 3) for c in mesh.colors_rgb_u8.reshape(-1).tolist()]
        tris = [int(i) for i in mesh.triangles.reshape(-1).tolist()]
        return {
            "ready": True,
            "reason": str(mesh.reason),
            "vertex_count": int(len(mesh.vertices_m)),
            "triangle_count": int(len(mesh.triangles)),
            "surface_area_m2": round(float(mesh.surface_area_m2), 4),
            "vertices": verts,
            "colors": cols,
            "triangles": tris,
        }

    def _publish_to_scene_store(
        self,
        *,
        pts: Any,
        support: Any,
        cam_counts: Any,
        rgb_float: Any = None,
        cloud_digest: str,
    ) -> dict[str, Any] | None:
        """Publish calibrated tabletop and object surfels/poses into EvidenceScene v3 SceneStore."""
        if self._scene_store is None:
            return None
        from evidencescene.observability.factor_graph_observability import evaluate_schur_observability_gate
        from evidencescene.observability.task_observability import inspection_point_jacobian
        from evidencescene.spatial_state import GeometryRef, ObjectSurface, PoseTangent

        np = self._np
        store = self._scene_store
        clock_id, obs_ns = self._sim_clock()
        tangent = PoseTangent("entity_from_scene", "right")
        obs_id = f"rgbd-step-{int(self._step_seq)}-{cloud_digest}"

        # 1. Fixed tabletop surface (known simulator world frame prior + measured tabletop surfels)
        table_mask = np.abs(pts[:, 2] - TABLE_TOP_Z) <= 0.012 if len(pts) else np.zeros((0,), dtype=bool)
        table_pts = pts[table_mask] if len(pts) else np.zeros((0, 3), dtype=np.float64)
        table_sup = np.maximum(1, cam_counts[table_mask].astype(np.int64)) if len(table_pts) else np.zeros((0,), dtype=np.int64)
        self._geom_version += 1
        table_surf = (
            ObjectSurface.from_points(
                "fixed_tabletop",
                GeometryRef("so101-rgbd", self._geom_epoch, self._geom_version),
                (obs_id,),
                table_pts,
                table_sup,
            )
            if len(table_pts)
            else None
        )
        prior_gate = evaluate_schur_observability_gate(pose_jacobian_ax=np.eye(6), candidate_pose_index=0)
        snap = store.publish_pose(
            store.snapshot().cursor,
            "fixed_tabletop",
            np.eye(4, dtype=np.float64),
            frame_id="sim_world",
            gate=prior_gate,
            source_id="known_sim_world_frame",
            observation_id=obs_id,
            sensor_clock_id=clock_id,
            observation_ns=obs_ns,
            geometry=table_surf,
            tangent=tangent,
        )

        # 2. Tabletop dynamic entities with point-to-plane surface normal SE(3) information matrices
        with self._lock:
            entity_poses_sim: list[tuple[str, Any, Any, str]] = []
            for body_key, eid, shape in (("obj-cup-a", "red_cylinder", "cylinder"), ("obj-bag-1", "brown_block", "box")):
                bid = self._body_ids.get(body_key)
                if bid is None:
                    continue
                pos = self._data.xpos[bid].copy()
                if float(pos[0]) > 1.2:
                    continue
                rot = self._data.xmat[bid].reshape(3, 3).copy()
                entity_poses_sim.append((eid, pos, rot, shape))

        for eid, pos, rot, shape in entity_poses_sim:
            T_se = np.eye(4, dtype=np.float64)
            T_se[:3, :3] = rot
            T_se[:3, 3] = pos
            # Crop measured RGB-D surfels in the entity's physical bounding volume
            if len(pts):
                rel_w = pts - pos[None, :]
                r_xy = np.hypot(rel_w[:, 0], rel_w[:, 1])
                rad_lim = 0.048 if shape == "cylinder" else 0.068
                z_lim = 0.065 if shape == "cylinder" else 0.045
                ent_mask = (r_xy <= rad_lim) & (np.abs(rel_w[:, 2]) <= z_lim) & (pts[:, 2] >= TABLE_TOP_Z + 0.005)
                ent_pts_w = pts[ent_mask]
                ent_sup = np.maximum(1, cam_counts[ent_mask].astype(np.int64))
                ent_rgb = rgb_float[ent_mask] if (rgb_float is not None and len(rgb_float) == len(pts)) else None
            else:
                ent_pts_w = np.zeros((0, 3), dtype=np.float64)
                ent_sup = np.zeros((0,), dtype=np.int64)
                ent_rgb = None

            if len(ent_pts_w) < 4:
                if snap.is_active(eid):
                    snap = store.report_pose_unavailable(
                        snap.cursor,
                        eid,
                        source_id="so101-rgbd-2cam",
                        observation_id=obs_id,
                        sensor_clock_id=clock_id,
                        observation_ns=obs_ns,
                        reason="insufficient_visible_surfels",
                    )
                continue

            # Transform measured world surfels into entity-local frame.
            # Under PoseTangent("scene_from_entity", "right"), T_se exp(xi) maps p_loc -> R(p_loc + w x p_loc + v) + t,
            # so point-to-plane residual Jacobian is n_loc^T [-[p_loc]_x, I_3] = [(p_loc x n_loc)^T, n_loc^T].
            # For a vertical cylinder, radial normals yield exact rank=5 with ker(H) = span{e_wz} (pure axial yaw)
            # while translational Schur block stays full-rank 3x3!
            ent_pts_local = (ent_pts_w - pos[None, :]) @ rot
            rows_jac: list[Any] = []
            inv_sigma = 1.0 / 0.004
            for p_loc, sup_val in zip(ent_pts_local, ent_sup.tolist()):
                w_obs = math.sqrt(float(sup_val)) * inv_sigma
                if shape == "cylinder":
                    if p_loc[2] >= 0.022:
                        n_loc = np.array([0.0, 0.0, 1.0], dtype=np.float64)
                    else:
                        nr = math.hypot(float(p_loc[0]), float(p_loc[1]))
                        if nr < 1e-4:
                            continue
                        n_loc = np.array([float(p_loc[0]) / nr, float(p_loc[1]) / nr, 0.0], dtype=np.float64)
                else:
                    ax_idx = int(np.argmax(np.abs(p_loc)))
                    n_loc = np.zeros(3, dtype=np.float64)
                    n_loc[ax_idx] = 1.0 if p_loc[ax_idx] >= 0.0 else -1.0
                rot_row = np.cross(p_loc, n_loc)
                rows_jac.append((w_obs * np.concatenate([rot_row, n_loc])).astype(np.float64))

            if len(rows_jac) < 5:
                continue
            A_ent = np.vstack(rows_jac)
            req_rank = 5 if shape == "cylinder" else 6
            gate = evaluate_schur_observability_gate(
                pose_jacobian_ax=A_ent,
                candidate_pose_index=0,
                required_pose_rank=req_rank,
            )
            if not gate.admitted:
                if snap.is_active(eid):
                    snap = store.report_pose_unavailable(
                        snap.cursor,
                        eid,
                        source_id="so101-rgbd-2cam",
                        observation_id=obs_id,
                        sensor_clock_id=clock_id,
                        observation_ns=obs_ns,
                        reason=str(gate.reason),
                    )
                continue

            prev_xyz = self._published_entity_xyz.get(eid)
            cur_xyz = (float(pos[0]), float(pos[1]), float(pos[2]))
            cause = "observation"
            if prev_xyz is not None and math.dist(prev_xyz, cur_xyz) > 0.005:
                cause = "physical_motion"
            self._published_entity_xyz[eid] = cur_xyz
            # Record measured bounding primitive strictly derived from this entity's real RGB-D surfels
            mean_rgb = (
                [round(float(c), 3) for c in np.median(ent_rgb, axis=0).tolist()]
                if (ent_rgb is not None and len(ent_rgb))
                else [0.75, 0.78, 0.82]
            )
            if shape == "cylinder":
                r_loc = np.hypot(ent_pts_local[:, 0], ent_pts_local[:, 1])
                self._entity_measured_primitives[eid] = {
                    "shape": "cylinder",
                    "radius_m": round(max(0.012, float(np.percentile(r_loc, 85))), 4),
                    "z_min_m": round(float(np.percentile(ent_pts_local[:, 2], 5)), 4),
                    "z_max_m": round(float(np.percentile(ent_pts_local[:, 2], 95)), 4),
                    "mean_rgb": mean_rgb,
                    "measured_points": int(len(ent_pts_local)),
                }
            else:
                p_lo = np.percentile(ent_pts_local, 5, axis=0)
                p_hi = np.percentile(ent_pts_local, 95, axis=0)
                self._entity_measured_primitives[eid] = {
                    "shape": "box",
                    "min_m": [round(float(v), 4) for v in p_lo.tolist()],
                    "max_m": [round(float(v), 4) for v in p_hi.tolist()],
                    "mean_rgb": mean_rgb,
                    "measured_points": int(len(ent_pts_local)),
                }

            self._geom_version += 1
            ent_surf = ObjectSurface.from_points(
                eid,
                GeometryRef("so101-rgbd", self._geom_epoch, self._geom_version),
                (obs_id,),
                ent_pts_local,
                ent_sup,
            )
            snap = store.publish_pose(
                snap.cursor,
                eid,
                T_se,
                frame_id="sim_world",
                gate=gate,
                source_id="so101-rgbd-2cam",
                observation_id=obs_id,
                sensor_clock_id=clock_id,
                observation_ns=obs_ns,
                geometry=ent_surf,
                cause=cause,
                tangent=PoseTangent("scene_from_entity", "right"),
            )

        return self._serialize_snapshot(snap)

    def get_spatial_snapshot_dict(self, *, refresh: bool = False) -> dict[str, Any]:
        """Return canonical EvidenceScene v3 SpatialSnapshot JSON (unified human/machine state)."""
        if self._scene_store is None:
            return {
                "schema_version": 0,
                "availability": "unavailable",
                "detail": "Optional evidencescene v3 spatial_state runtime not installed",
            }
        snap = self._scene_store.snapshot()
        if refresh or not snap.entity_poses:
            self.reconstruct_scene_evidencescene()
            snap = self._scene_store.snapshot()
        return self._serialize_snapshot(snap)

    def _point_jacobian_for_tangent(self, transform: Any, pt_local: Any, tangent: Any) -> Any:
        """Exact 3x6 point Jacobian d(p_scene)/d(xi) matching the pose's declared PoseTangent."""
        from evidencescene.observability.task_observability import inspection_point_jacobian

        np = self._np
        if tangent is not None and tangent.transform == "scene_from_entity" and tangent.perturbation_side == "right":
            # T exp(xi) maps p_loc -> R (p_loc + w x p_loc + v) + t = R (-[p_loc]_x w + v) + p_scene
            px, py, pz = float(pt_local[0]), float(pt_local[1]), float(pt_local[2])
            neg_skew = np.array([[0.0, pz, -py], [-pz, 0.0, px], [py, -px, 0.0]], dtype=float)
            return transform[:3, :3] @ np.hstack([neg_skew, np.eye(3, dtype=float)])
        return inspection_point_jacobian(transform, pt_local)

    def _build_task_jacobian(
        self,
        transform: Any,
        task_mode: str,
        entity_point_m: Sequence[float] | None = None,
        tangent: Any = None,
    ) -> tuple[Any, list[float]]:
        np = self._np
        mode = str(task_mode or "body_grasp").strip().lower()
        if mode == "handle_grasp":
            pt = np.array(entity_point_m if entity_point_m is not None else [0.028, 0.0, 0.0], dtype=float)
        else:
            pt = np.array(entity_point_m if entity_point_m is not None else [0.0, 0.0, 0.0], dtype=float)

        pt_jac = self._point_jacobian_for_tangent(transform, pt, tangent)
        if mode in ("body_grasp", "handle_grasp"):
            if tangent is not None and tangent.transform == "scene_from_entity" and tangent.perturbation_side == "right":
                # Local Z axis under right perturbation: d(R e_z)/d(w) = R (w x e_z) = R (-[e_z]_x w)
                # Two tangent coordinates on S^2 at e_z are local X and local Y: rows [0, 1, 0, 0,0,0] and [-1, 0, 0, 0,0,0]
                angular_rows = np.array(
                    [[0.0, 1.0, 0.0, 0.0, 0.0, 0.0], [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
                    dtype=float,
                )
            else:
                direction = transform[:3, :3] @ np.array([0.0, 0.0, 1.0], dtype=float)
                x, y, z = direction
                axis_rows = np.array([[0.0, -z, y, 0.0, 0.0, 0.0], [z, 0.0, -x, 0.0, 0.0, 0.0], [-y, x, 0.0, 0.0, 0.0, 0.0]])
                basis = np.eye(3)[int(np.argmin(np.abs(direction)))]
                first = np.cross(direction, basis)
                first /= np.linalg.norm(first)
                angular_rows = np.stack((first, np.cross(direction, first))) @ axis_rows
            rows = np.vstack([pt_jac, angular_rows])
        else:
            rows = pt_jac
        scene_pt = (transform[:3, :3] @ pt + transform[:3, 3]).tolist()
        return rows, [round(float(v), 5) for v in scene_pt]

    def low_level_assess_spatial_requirements(
        self,
        entity_id: str = "red_cylinder",
        task_mode: str = "body_grasp",
        *,
        max_age_ms: float = 2000.0,
        max_stddev_m: float | None = None,
        entity_point_m: Sequence[float] | None = None,
    ) -> dict[str, Any]:
        """Assess task-conditioned observability (ker(H) <= ker(J_task)) via EvidenceScene v3."""
        if self._scene_store is None:
            return {"status": "unavailable_evidence", "detail": "EvidenceScene v3 SceneStore unavailable"}
        from evidencescene.spatial_state import PoseTangent

        self.reconstruct_scene_evidencescene()
        snap = self._scene_store.snapshot()
        clock_id, ref_ns = self._sim_clock()
        max_age_ns = max(0, round(float(max_age_ms) * 1e6))
        mode = str(task_mode or "body_grasp").strip().lower()

        if mode == "selected_point":
            if snap.selected_point is None:
                return {
                    "status": "unknown",
                    "detail": "No operator surface target selected yet; call select_surface_target first.",
                    "cursor": {
                        "scene_id": snap.cursor.scene_id,
                        "epoch": int(snap.cursor.epoch),
                        "revision": int(snap.cursor.revision),
                    },
                }
            limits = [float(max_stddev_m)] * 3 if max_stddev_m is not None else None
            assessment = snap.assess_selected_point(
                reference_ns=ref_ns,
                reference_clock_id=clock_id,
                maximum_age_ns=max_age_ns,
                maximum_model_stddev=limits,
            )
            sel_tgt = snap.selected_surface_target()
            scene_pt = [round(float(v), 5) for v in sel_tgt.scene_point_m] if sel_tgt and sel_tgt.scene_point_m else None
            resolved_entity = snap.selected_point.entity_id
            resolved_pt = list(snap.selected_point.entity_point_m)
        else:
            if entity_id not in snap.entity_poses:
                unavail = snap.unavailable_poses.get(entity_id)
                return {
                    "status": "unknown" if unavail is None else "unavailable_evidence",
                    "entity_id": entity_id,
                    "detail": unavail.reason if unavail is not None else f"Entity {entity_id!r} not in committed snapshot",
                    "cursor": {
                        "scene_id": snap.cursor.scene_id,
                        "epoch": int(snap.cursor.epoch),
                        "revision": int(snap.cursor.revision),
                    },
                }
            pose = snap.entity_poses[entity_id]
            pose_tangent = pose.admission.tangent if pose.admission and pose.admission.tangent else PoseTangent("entity_from_scene", "right")
            rows, scene_pt = self._build_task_jacobian(pose.scene_from_entity, mode, entity_point_m, tangent=pose_tangent)
            limits = [float(max_stddev_m)] * len(rows) if max_stddev_m is not None else None
            assessment = snap.assess_spatial_requirements(
                entity_id,
                rows,
                reference_ns=ref_ns,
                reference_clock_id=clock_id,
                maximum_age_ns=max_age_ns,
                maximum_model_stddev=limits,
                required_tangent=pose_tangent,
            )
            resolved_entity = entity_id
            resolved_pt = list(entity_point_m) if entity_point_m is not None else ([0.028, 0.0, 0.0] if mode == "handle_grasp" else [0.0, 0.0, 0.0])

        aid = f"assess-{snap.cursor.epoch}-{snap.cursor.revision}-{len(self._pinned_assessments) + 1}"
        self._pinned_assessments[aid] = {
            "assessment": assessment,
            "snapshot": snap,
            "entity_id": resolved_entity,
            "task_mode": mode,
            "entity_point_m": resolved_pt,
        }
        obs_eval = assessment.evidence.assessment
        return {
            "assessment_id": aid,
            "entity_id": resolved_entity,
            "task_mode": mode,
            "status": str(assessment.status),
            "cursor": {
                "scene_id": snap.cursor.scene_id,
                "epoch": int(snap.cursor.epoch),
                "revision": int(snap.cursor.revision),
            },
            "pose_revision": int(assessment.evidence.pose_revision),
            "freshness": {
                "status": assessment.freshness.status,
                "reason": assessment.freshness.reason,
                "age_ms": round(assessment.freshness.age_ns / 1e6, 2) if assessment.freshness.age_ns is not None else None,
            },
            "observability": {
                "status": "sufficient" if assessment.status == "satisfied" else ("insufficient" if assessment.status == "unsatisfied" else "unknown"),
                "rank": int(obs_eval.pose_rank) if obs_eval is not None else None,
                "status_by_row": list(obs_eval.status_by_row) if obs_eval is not None else [],
                "model_stddev_by_row": [
                    round(float(v), 6) if v is not None else None
                    for v in (obs_eval.model_stddev_by_row if obs_eval is not None else ())
                ],
            },
            "missing_evidence": list(assessment.missing_evidence),
            "scene_point_m": scene_pt,
            "detail": f"task_mode={mode}, status={assessment.status}, missing={list(assessment.missing_evidence)}",
        }

    def low_level_revalidate_spatial_requirements(self, assessment_id: str) -> dict[str, Any]:
        """Revalidate a pinned assessment against the current committed SceneStore snapshot."""
        if self._scene_store is None:
            return {"decision": "hold", "status": "unavailable_evidence", "reason": "scene_store_unavailable"}
        rec = self._pinned_assessments.get(str(assessment_id or ""))
        if rec is None:
            return {
                "decision": "invalidate_plan",
                "status": "unknown",
                "reason": "unknown_assessment_id",
                "resnapshot_required": True,
                "detail": f"Assessment {assessment_id!r} was cleared (e.g. after epoch reset) or does not exist.",
            }
        prev = rec["assessment"]
        snap = self._scene_store.snapshot()
        clock_id, ref_ns = self._sim_clock()
        check = snap.revalidate_spatial_requirements(prev, reference_clock_id=clock_id, reference_ns=ref_ns)
        scene_pt = None
        if check.status == "satisfied":
            decision = "target_available"
            pose = snap.entity_poses[prev.evidence.entity_id]
            pt = self._np.asarray(rec["entity_point_m"], dtype=float)
            scene_pt = [round(float(v), 5) for v in (pose.scene_from_entity[:3, :3] @ pt + pose.scene_from_entity[:3, 3]).tolist()]
        elif check.reason in ("task_requires_new_linearization", "scene_scope_changed", "entity_retired", "surface_selection_changed"):
            decision = "invalidate_plan"
        else:
            decision = "hold"
        resnap = check.reason in ("scene_scope_changed",)
        return {
            "assessment_id": assessment_id,
            "decision": decision,
            "status": str(check.status),
            "reason": str(check.reason),
            "resnapshot_required": resnap,
            "planned_cursor": {
                "scene_id": prev.evidence.cursor.scene_id,
                "epoch": int(prev.evidence.cursor.epoch),
                "revision": int(prev.evidence.cursor.revision),
            },
            "cursor": {
                "scene_id": check.cursor.scene_id,
                "epoch": int(check.cursor.epoch),
                "revision": int(check.cursor.revision),
            },
            "scene_point_m": scene_pt,
            "missing_evidence": list(check.assessment.missing_evidence) if check.assessment is not None else [check.reason],
            "detail": f"decision={decision}, reason={check.reason}, status={check.status}",
        }

    def low_level_evaluate_wrist_viewpoints(
        self,
        assessment_id: str,
        candidates_xyz_m: Sequence[Sequence[float]] | None = None,
    ) -> dict[str, Any]:
        """Active perception forecast via EvidenceScene v3 `evaluate_observation` over reachable SO-101 wrist poses."""
        if self._scene_store is None:
            return {"status": "unavailable_evidence", "detail": "EvidenceScene v3 SceneStore unavailable"}
        rec = self._pinned_assessments.get(str(assessment_id or ""))
        if rec is None:
            return {"status": "unknown", "reason": "unknown_assessment_id", "selected_candidate": None}
        from evidencescene.observability.task_observability import inspection_point_jacobian
        from evidencescene.spatial_state import PoseTangent

        np = self._np
        req = rec["assessment"]
        snap = rec["snapshot"]
        eid = rec["entity_id"]
        if eid not in snap.entity_poses:
            return {"status": "unknown", "reason": "entity_absent", "selected_candidate": None}
        pose = snap.entity_poses[eid]
        ent_xyz = pose.scene_from_entity[:3, 3]
        ee_now = np.asarray(self.low_level_get_robot_state()["ee_pos_m"], dtype=float)

        if candidates_xyz_m:
            raw_cands = [(f"wrist_cand_{i}", tuple(float(v) for v in xyz[:3])) for i, xyz in enumerate(candidates_xyz_m)]
        else:
            raw_cands = [
                ("wrist_left_closeup", (float(ent_xyz[0] - 0.04), float(ent_xyz[1] + 0.09), TABLE_TOP_Z + 0.16)),
                ("wrist_right_closeup", (float(ent_xyz[0] - 0.04), float(ent_xyz[1] - 0.09), TABLE_TOP_Z + 0.16)),
                ("wrist_top_closeup", (float(ent_xyz[0] - 0.02), float(ent_xyz[1]), TABLE_TOP_Z + 0.19)),
            ]

        tangent = pose.admission.tangent if pose.admission and pose.admission.tangent else PoseTangent("entity_from_scene", "right")
        target_local = np.asarray(rec["entity_point_m"], dtype=float)
        J_feat = self._point_jacobian_for_tangent(pose.scene_from_entity, target_local, tangent)
        evaluated: list[dict[str, Any]] = []
        eligible: list[tuple[float, str, list[float]]] = []

        for cid, raw_xyz in raw_cands:
            cx, cy, cz = self.clamp_world_target(*raw_xyz)
            cand_xyz = np.array([cx, cy, cz], dtype=float)
            dist_to_obj = max(0.08, float(np.linalg.norm(cand_xyz - ent_xyz)))
            motion_m = float(np.linalg.norm(cand_xyz - ee_now))
            # Expected depth/feature noise scales linearly with standoff distance (3mm @ 0.15m)
            sigma_m = 0.003 * (dist_to_obj / 0.15)
            # Only side viewpoints with horizontal offset break cylindrical yaw ambiguity on a side feature
            horiz_offset = float(np.hypot(cand_xyz[0] - ent_xyz[0], cand_xyz[1] - ent_xyz[1]))
            if horiz_offset < 0.045:
                # Top-down view constrains Z and XY center well, weaker tangential yaw discrimination
                B = np.vstack([J_feat[2:3] / sigma_m, J_feat[0:2] / (sigma_m * 1.6)])
            else:
                B = J_feat / sigma_m

            fc = snap.evaluate_observation(req, candidate_id=cid, tangent=tangent, whitened_pose_jacobian=B)
            pred = fc.predicted
            unmet: list[int] = []
            if pred is not None:
                limits = req.maximum_model_stddev
                for i, st in enumerate(pred.status_by_row):
                    dev = pred.model_stddev_by_row[i]
                    if st != "observable" or (limits is not None and (dev is None or dev > limits[i])):
                        unmet.append(i)
            meets = pred is not None and not unmet and req.freshness.status == "fresh"
            if meets:
                eligible.append((motion_m, cid, [round(cx, 4), round(cy, 4), round(cz, 4)]))
            evaluated.append(
                {
                    "candidate_id": cid,
                    "ee_xyz_m": [round(cx, 4), round(cy, 4), round(cz, 4)],
                    "estimated_motion_m": round(motion_m, 4),
                    "standoff_m": round(dist_to_obj, 4),
                    "forecast_reason": fc.reason,
                    "predicted_rank": int(pred.pose_rank) if pred is not None else None,
                    "predicted_status_by_row": list(pred.status_by_row) if pred is not None else [],
                    "unmet_task_rows": unmet if pred is not None else None,
                    "predicted_to_meet_requirements": meets,
                }
            )

        selected_id = None
        selected_xyz = None
        if req.status == "satisfied":
            reason = "already_satisfied"
        elif eligible:
            _, selected_id, selected_xyz = min(eligible, key=lambda x: x[0])
            reason = "least_motion_wrist_viewpoint_meeting_requirements"
        else:
            reason = "no_single_candidate_meets_requirements"

        return {
            "assessment_id": assessment_id,
            "cursor": {
                "scene_id": snap.cursor.scene_id,
                "epoch": int(snap.cursor.epoch),
                "revision": int(snap.cursor.revision),
            },
            "current_status": str(req.status),
            "selected_candidate": selected_id,
            "recommended_ee_xyz_m": selected_xyz,
            "reason": reason,
            "candidates": evaluated,
        }

    def low_level_select_surface_target(
        self,
        *,
        entity_id: str = "fixed_tabletop",
        point_index: int | None = None,
        target_xyz_m: Sequence[float] | None = None,
        reviewer_id: str = "operator",
        reason: str = "3d_twin_surface_selection",
    ) -> dict[str, Any]:
        """Commit an operator-selected measured surface vertex into EvidenceScene v3 SceneStore."""
        if self._scene_store is None:
            return {"ok": False, "status": "unavailable_evidence", "detail": "EvidenceScene v3 SceneStore unavailable"}
        snap = self._scene_store.snapshot()
        if not snap.entity_poses:
            self.reconstruct_scene_evidencescene()
            snap = self._scene_store.snapshot()

        eid = str(entity_id or "fixed_tabletop")
        if eid == "auto" and target_xyz_m is not None and len(target_xyz_m) >= 3:
            tgt = self._np.asarray([float(target_xyz_m[0]), float(target_xyz_m[1]), float(target_xyz_m[2])], dtype=float)
            best_eid, best_dist = "fixed_tabletop", 1e9
            for cand_eid in ("red_cylinder", "brown_block", "fixed_tabletop"):
                cand_pose = snap.entity_poses.get(cand_eid)
                if cand_pose is None or cand_pose.geometry is None or not len(cand_pose.geometry.points_m):
                    continue
                dmin = float(self._np.linalg.norm(cand_pose.surface_points_in_scene() - tgt[None, :], axis=1).min())
                # Prefer object surface if click is within 2.8cm of a tracked object
                if cand_eid != "fixed_tabletop" and dmin <= 0.028:
                    best_eid, best_dist = cand_eid, dmin
                    break
                if dmin < best_dist:
                    best_eid, best_dist = cand_eid, dmin
            eid = best_eid
        pose = snap.entity_poses.get(eid)
        if pose is None or pose.geometry is None or not len(pose.geometry.points_m):
            return {"ok": False, "status": "missing_object", "detail": f"Entity {eid!r} has no measured surface points"}

        pts_scene = pose.surface_points_in_scene()
        if point_index is not None:
            idx = int(point_index)
        elif target_xyz_m is not None and len(target_xyz_m) >= 3:
            tgt = self._np.asarray([float(target_xyz_m[0]), float(target_xyz_m[1]), float(target_xyz_m[2])], dtype=float)
            idx = int(self._np.linalg.norm(pts_scene - tgt[None, :], axis=1).argmin())
        else:
            idx = len(pts_scene) // 2

        new_snap = self._scene_store.select_surface_point(
            snap.cursor,
            eid,
            idx,
            reviewer_id=str(reviewer_id or "operator"),
            reason=str(reason or "3d_twin_surface_selection"),
        )
        # Invalidate cached reconstruction so next poll reflects updated cursor & selection
        self._recon_cache = None
        sel_dict = self._serialize_selected_target(new_snap)
        return {
            "ok": True,
            "status": "selected",
            "cursor": {
                "scene_id": new_snap.cursor.scene_id,
                "epoch": int(new_snap.cursor.epoch),
                "revision": int(new_snap.cursor.revision),
            },
            "selected_surface_target": sel_dict,
        }

    def embodiment_manifest(self) -> Any:
        from .plugin import EmbodimentManifest

        urdf_path = self._so101_dir / "robot.urdf"
        urdf_sha = digest(urdf_path.read_text(encoding="utf-8"))[:16] if urdf_path.is_file() else ""
        return EmbodimentManifest(
            embodiment_id="trax-mujoco-so101-sim",
            display_name="LeRobot SO-101 (MuJoCo 6-DoF Simulator)",
            evidence_source=self.evidence_source,
            execution_authority="trax-mujoco-so101-sim",
            perception_owner="mujoco-so101-sim",
            urdf_uri="trax://robot/urdf" if urdf_path.is_file() else None,
            urdf_sha256=urdf_sha,
            mesh_base_url="/assets/robot/",
            kinematics_notes="5-DoF arm (STS3215 servos) + 1-DoF moving jaw gripper + wrist camera mount.",
            physics_approximations=(
                "kinematic_proximity_grasp_attach_58mm: gripper closure attaches objects within 58mm XY radius rather than pure contact-friction grasping.",
            ),
        )

    def resolve_robot_asset(self, rel_path: str) -> tuple[bytes, str, str] | None:
        from .plugin import read_safe_robot_asset

        return read_safe_robot_asset(self._so101_dir, rel_path)

    def register_mcp(self, server: Any, lock: threading.RLock) -> None:
        import json as _json
        from .mcp_protocol import MCPResourceSpec, MCPToolSpec
        from .plugin import rewrite_urdf_mesh_uris

        server.set_embodiment_meta({"native_low_level": True})
        server.register_tool(
            MCPToolSpec(
                name="get_robot_state",
                description="Read current 5-DoF joint angles (rad), joint limits, 3D end-effector pose (m), gripper open ratio, grasped object, and workspace bounds.",
                input_schema={"type": "object", "properties": {}, "additionalProperties": False},
                read_only=True,
                idempotent=True,
            ),
            lambda args: self.low_level_get_robot_state(),
        )
        server.register_tool(
            MCPToolSpec(
                name="get_scene_objects",
                description="Read authoritative tabletop objects (`id`, `world_m`, `height_above_table_m`, `grasped`) and named target zones (`target_zone_a`, `target_zone_b`, `station_pad`).",
                input_schema={"type": "object", "properties": {}, "additionalProperties": False},
                read_only=True,
                idempotent=True,
            ),
            lambda args: self.low_level_get_scene_objects(),
        )
        server.register_tool(
            MCPToolSpec(
                name="move_ee",
                description="Move robot end-effector smoothly to Cartesian world pose [x, y, z] in meters via IK. Enforces workspace bounds and returns structured `limit_refusal` BEFORE moving if out of bounds.",
                input_schema={
                    "type": "object",
                    "required": ["x", "y", "z"],
                    "properties": {
                        "x": {"type": "number", "default": 0.33, "description": "World X in meters"},
                        "y": {"type": "number", "default": -0.02, "description": "World Y in meters"},
                        "z": {"type": "number", "default": 1.15, "description": "World Z in meters (tabletop is ~1.00 m)"},
                        "steps": {"type": "integer", "default": 36, "description": "Simulation integration steps (12..80)"},
                    },
                },
                read_only=False,
                idempotent=True,
            ),
            lambda args: self.low_level_move_ee(
                float(args.get("x", 0.33)),
                float(args.get("y", -0.02)),
                float(args.get("z", 1.15)),
                steps=int(args.get("steps", 36)),
            ),
        )
        server.register_tool(
            MCPToolSpec(
                name="set_gripper",
                description="Control SO-101 moving jaw gripper: open_ratio=1.0 (fully open / release) or 0.0 (closed / grasp nearby object within contact radius).",
                input_schema={
                    "type": "object",
                    "required": ["open_ratio"],
                    "properties": {
                        "open_ratio": {"type": "number", "minimum": 0.0, "maximum": 1.0, "default": 0.0},
                    },
                },
                read_only=False,
                idempotent=True,
            ),
            lambda args: self.low_level_set_gripper(float(args.get("open_ratio", 0.0))),
        )
        server.register_tool(
            MCPToolSpec(
                name="move_joints",
                description="Command 5-DoF arm joint angles [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll] in radians. Refuses out-of-range joints with `limit_refusal`.",
                input_schema={
                    "type": "object",
                    "required": ["q_rad"],
                    "properties": {
                        "q_rad": {"type": "array", "items": {"type": "number"}, "minItems": 5, "maxItems": 5, "default": [0.0, -0.35, 0.55, 0.95, 0.0]},
                        "steps": {"type": "integer", "default": 32},
                    },
                },
                read_only=False,
                idempotent=True,
            ),
            lambda args: self.low_level_move_joints(list(args.get("q_rad") or [0.0, -0.35, 0.55, 0.95, 0.0]), steps=int(args.get("steps", 32))),
        )
        server.register_tool(
            MCPToolSpec(
                name="go_home",
                description="Retract the SO-101 arm safely to its neutral home joint configuration.",
                input_schema={"type": "object", "properties": {}, "additionalProperties": False},
                read_only=False,
                idempotent=True,
            ),
            lambda args: self.low_level_go_home(),
        )
        server.register_tool(
            MCPToolSpec(
                name="reset_task",
                description="Reset the tabletop environment for a specified task_id (`pick_and_place_zone`, `lift_red_object`, `place_and_activate_station`) and seed.",
                input_schema={
                    "type": "object",
                    "properties": {
                        "task_id": {
                            "type": "string",
                            "default": "pick_and_place_zone",
                            "enum": ["pick_and_place_zone", "lift_red_object", "place_and_activate_station"],
                        },
                        "seed": {"type": "integer", "default": 0},
                    },
                },
                read_only=False,
                idempotent=True,
            ),
            lambda args: self.low_level_reset_task(str(args.get("task_id") or "pick_and_place_zone"), seed=int(args.get("seed", 0))),
        )
        server.register_tool(
            MCPToolSpec(
                name="activate_workstation",
                description="NON-IDEMPOTENT: Trigger external workstation actuator once. Must pass a stable `command_id` and reconcile via `query_command_status` if receipt is `unknown`.",
                input_schema={
                    "type": "object",
                    "required": ["command_id"],
                    "properties": {
                        "command_id": {"type": "string", "default": "station-cmd-1"},
                    },
                },
                read_only=False,
                idempotent=False,
                destructive=True,
                requires_reconciliation=True,
            ),
            lambda args: self.low_level_activate_workstation(str(args.get("command_id") or "station-cmd-1")),
        )
        server.register_tool(
            MCPToolSpec(
                name="query_command_status",
                description="Query host ledger for a prior non-idempotent `command_id` to reconcile unknown receipts before any retry.",
                input_schema={
                    "type": "object",
                    "required": ["command_id"],
                    "properties": {"command_id": {"type": "string", "default": "station-cmd-1"}},
                },
                read_only=True,
                idempotent=True,
            ),
            lambda args: self.low_level_query_command_status(str(args.get("command_id") or "station-cmd-1")),
        )
        server.register_tool(
            MCPToolSpec(
                name="verify_task",
                description="Run independent physics-based verification for the active tabletop task.",
                input_schema={
                    "type": "object",
                    "properties": {"task_id": {"type": "string", "default": "pick_and_place_zone"}},
                },
                read_only=True,
                idempotent=True,
            ),
            lambda args: self.low_level_verify_task(args.get("task_id")),
        )
        server.register_tool(
            MCPToolSpec(
                name="assess_spatial_requirements",
                description="Evaluate task-conditioned observability (ker(H) <= ker(J_task)) & freshness via EvidenceScene v3 SceneStore.",
                input_schema={
                    "type": "object",
                    "properties": {
                        "entity_id": {"type": "string", "default": "red_cylinder"},
                        "task_mode": {
                            "type": "string",
                            "default": "body_grasp",
                            "enum": ["body_grasp", "handle_grasp", "inspect_point", "selected_point"],
                        },
                        "max_age_ms": {"type": "number", "default": 2000.0},
                        "max_stddev_m": {"type": "number"},
                    },
                },
                read_only=True,
                idempotent=True,
            ),
            lambda args: self.low_level_assess_spatial_requirements(
                str(args.get("entity_id") or "red_cylinder"),
                str(args.get("task_mode") or "body_grasp"),
                max_age_ms=float(args.get("max_age_ms", 2000.0)),
                max_stddev_m=float(args["max_stddev_m"]) if args.get("max_stddev_m") is not None else None,
            ),
        )
        server.register_tool(
            MCPToolSpec(
                name="revalidate_spatial_requirements",
                description="Revalidate a pinned assessment_id against the current committed SceneStore cursor before actuation.",
                input_schema={
                    "type": "object",
                    "required": ["assessment_id"],
                    "properties": {"assessment_id": {"type": "string"}},
                },
                read_only=True,
                idempotent=True,
            ),
            lambda args: self.low_level_revalidate_spatial_requirements(str(args.get("assessment_id") or "")),
        )
        server.register_tool(
            MCPToolSpec(
                name="evaluate_wrist_viewpoints",
                description="Active perception forecast (H + B^T B) over reachable SO-101 wrist camera poses via EvidenceScene v3.",
                input_schema={
                    "type": "object",
                    "required": ["assessment_id"],
                    "properties": {
                        "assessment_id": {"type": "string"},
                        "candidates_xyz_m": {"type": "array", "items": {"type": "array", "items": {"type": "number"}}},
                    },
                },
                read_only=True,
                idempotent=True,
            ),
            lambda args: self.low_level_evaluate_wrist_viewpoints(
                str(args.get("assessment_id") or ""),
                candidates_xyz_m=args.get("candidates_xyz_m"),
            ),
        )
        server.register_tool(
            MCPToolSpec(
                name="select_surface_target",
                description="Commit a provenance-tagged surface vertex selection into EvidenceScene v3 SceneStore.",
                input_schema={
                    "type": "object",
                    "properties": {
                        "entity_id": {"type": "string", "default": "fixed_tabletop"},
                        "point_index": {"type": "integer"},
                        "target_xyz_m": {"type": "array", "items": {"type": "number"}},
                        "reviewer_id": {"type": "string", "default": "operator"},
                        "reason": {"type": "string", "default": "3d_twin_surface_selection"},
                    },
                },
                read_only=False,
                idempotent=True,
            ),
            lambda args: self.low_level_select_surface_target(
                entity_id=str(args.get("entity_id") or "fixed_tabletop"),
                point_index=int(args["point_index"]) if args.get("point_index") is not None else None,
                target_xyz_m=args.get("target_xyz_m"),
                reviewer_id=str(args.get("reviewer_id") or "operator"),
                reason=str(args.get("reason") or "3d_twin_surface_selection"),
            ),
        )
        server.register_resource(
            MCPResourceSpec(
                uri="trax://scene/state",
                name="Live Robot & Tabletop Scene State",
                description="Authoritative SO-101 joint/EE state and tabletop object poses.",
                mime_type="application/json",
            ),
            lambda: (
                "application/json",
                _json.dumps(
                    {
                        "robot": self.low_level_get_robot_state(),
                        "scene": self.low_level_get_scene_objects(),
                        "tasks": self.low_level_task_catalog(),
                    },
                    indent=2,
                ),
            ),
        )
        urdf_file = self._so101_dir / "robot.urdf"
        if urdf_file.is_file():
            server.register_resource(
                MCPResourceSpec(
                    uri="trax://robot/urdf",
                    name="LeRobot SO-101 URDF Description",
                    description="Authoritative SO-101 URDF XML with rewritten /assets/robot/ mesh URIs.",
                    mime_type="application/xml",
                ),
                lambda p=urdf_file: (
                    "application/xml",
                    rewrite_urdf_mesh_uris(p.read_text(encoding="utf-8"), asset_url_prefix="/assets/robot"),
                ),
            )
        server.register_resource(
            MCPResourceSpec(
                uri="trax://scene/reconstruction",
                name="EvidenceScene 3D Surfel Reconstruction",
                description="Multi-camera calibrated RGB-D surfel fusion & 6-DoF Schur observability via EvidenceScene.",
                mime_type="application/json",
            ),
            lambda: ("application/json", _json.dumps(self.reconstruct_scene_evidencescene())),
        )
        server.register_resource(
            MCPResourceSpec(
                uri="trax://scene/snapshot",
                name="EvidenceScene v3 Canonical SpatialSnapshot",
                description="Committed Schema v1 SpatialSnapshot (cursor, clock_id, entities, observability, selected target).",
                mime_type="application/json",
            ),
            lambda: ("application/json", _json.dumps(self.get_spatial_snapshot_dict())),
        )


def create_so101_backend(config: SceneConfig) -> MuJoCoSO101HostDriver:
    """Entry-point factory for `trax.backends` (`mujoco_so101`)."""
    return MuJoCoSO101HostDriver(config)
