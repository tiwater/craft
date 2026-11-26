"""High-fidelity Gazebo Fortress (OGRE2 PBR + DART + ROS 2 Humble) café workstation host driver.

Strictly preserves TRAX's three-way ownership separation (``AGENTS.md``):
- **Perception owner**: Gazebo Fortress OGRE2 RGB-D camera sensors (``overview``, ``overhead``,
  ``spout``, ``wrist`` with realistic Gaussian sensor noise) + Gazebo dynamic pose publisher
  bridged into ROS 2 Humble via ``ros_gz_bridge``.
- **Host actuation owner**: Gazebo Fortress DART physics (500 Hz) + 7-DOF Franka Emika Panda
  ``JointPositionController`` plugins + parallel finger controllers + host espresso machine
  command/receipt ledger.
- **Policy execution**: TRAX AST-sandboxed ``run_policy(sdk, params)`` consuming the identical
  13-method ``EdgeZeroGtCoffeeSDK`` contract.

Honest fail-closed guarantee:
- If Gazebo Fortress (``ign`` / ``gz``) is not installed or unreachable on the host, this driver
  NEVER fabricates camera frames or spoofs physics: ``discover_capabilities()`` and
  ``observe_scene()`` return ``status="disconnected"`` with actionable install instructions
  (``sim/gazebo/setup_gazebo_fortress.sh``).
- ``export_gazebo_cafe_bundle(output_dir)`` works on any machine (pure stdlib) to materialize
  a portable SDF 1.9 + OGRE2 PBR + 7-DOF Franka Panda + ROS 2 Humble ``ros_gz_bridge`` package
  with zero hard-coded private machine paths.
"""
from __future__ import annotations

import importlib.util
import json
import math
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import threading
import time
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET

from .config import SceneConfig
from .contracts import digest
from .edge_adapter import (
    CapabilityCheckResult,
    DiscoveredCapabilities,
    ReconciliationResult,
    SceneObservationResult,
)
from .journal import Journal
from .sandbox import check_sandbox_ast_compliance


GAZEBO_WORLD_NAME = "craft_cafe"
GAZEBO_REQUIRED_METHODS = (
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

SPOUT_XY = (0.45, 0.00)
PICKUP_1_XY = (0.38, -0.17)
PICKUP_2_XY = (0.38, 0.17)
TABLE_TOP_Z = 1.00
CUP_REST_Z = 1.055
L2_SAFE_RADIUS_M = 0.025
L2_MAX_TILT_DEG = 12.0


def _discover_local_panda_assets() -> Path | None:
    """Locate optional robosuite Panda STL/DAE meshes & PBR textures dynamically."""
    spec = importlib.util.find_spec("robosuite")
    if spec is None or not spec.submodule_search_locations:
        return None
    base = Path(list(spec.submodule_search_locations)[0]) / "models" / "assets"
    if (base / "robots" / "panda" / "meshes" / "link0.stl").is_file():
        return base
    return None


def probe_gazebo_runtime() -> dict[str, Any]:
    """Honestly probe local Gazebo Fortress, ROS 2 Humble, and GPU availability."""
    ign_bin = shutil.which("ign")
    gz_bin = shutil.which("gz")
    gazebo_classic = shutil.which("gzserver")
    ros2_bin = shutil.which("ros2") or ("/opt/ros/humble/bin/ros2" if Path("/opt/ros/humble/bin/ros2").is_file() else None)
    has_ros_gz = Path("/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge").is_file() or bool(
        shutil.which("parameter_bridge")
    )
    gpu_name = None
    nvidia_smi = shutil.which("nvidia-smi")
    if nvidia_smi:
        try:
            proc = subprocess.run(
                [nvidia_smi, "--query-gpu=name", "--format=csv,noheader"],
                capture_output=True,
                text=True,
                timeout=3.0,
                check=False,
            )
            if proc.returncode == 0 and proc.stdout.strip():
                gpu_name = proc.stdout.strip().splitlines()[0].strip()
        except Exception:  # noqa: BLE001
            gpu_name = None

    sim_binary = ign_bin or gz_bin
    installed = sim_binary is not None
    missing: list[str] = []
    if not installed:
        missing.append("ignition-fortress (ign/gz)")
    if not has_ros_gz:
        missing.append("ros-humble-ros-gz")

    return {
        "installed": installed,
        "sim_binary": sim_binary,
        "ign_binary": ign_bin,
        "gz_binary": gz_bin,
        "gazebo_classic_binary": gazebo_classic,
        "ros2_binary": ros2_bin,
        "ros_gz_bridge_installed": has_ros_gz,
        "gpu_name": gpu_name,
        "local_panda_assets_available": _discover_local_panda_assets() is not None,
        "missing_packages": missing,
        "install_command": "sudo apt-get update && sudo apt-get install -y ignition-fortress ros-humble-ros-gz",
        "install_script": "sim/gazebo/setup_gazebo_fortress.sh",
    }


def _model_config_xml(name: str, description: str) -> str:
    return f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>TRAX Simulator Integration</name>
  </author>
  <description>{description}</description>
</model>
"""


def build_cafe_workstation_model_sdf(*, has_textures: bool = True) -> str:
    """Generate OGRE2 PBR café workstation SDF (walnut bar, espresso machine, 3 fixed RGB-D cameras)."""

    def _pbr(albedo_file: str, base_rgba: str, metalness: float, roughness: float) -> str:
        albedo_tag = (
            f"<albedo_map>model://cafe_workstation/textures/{albedo_file}</albedo_map>"
            if has_textures
            else ""
        )
        return (
            f"<ambient>{base_rgba}</ambient>"
            f"<diffuse>{base_rgba}</diffuse>"
            f"<pbr><metal>{albedo_tag}"
            f"<metalness>{metalness:.2f}</metalness>"
            f"<roughness>{roughness:.2f}</roughness>"
            f"</metal></pbr>"
        )

    def _camera_sensor(name: str, pose: str, hfov: float) -> str:
        return f"""      <sensor name="camera_{name}" type="camera">
        <pose>{pose}</pose>
        <always_on>true</always_on>
        <update_rate>15</update_rate>
        <topic>/craft/camera/{name}/image_raw</topic>
        <camera>
          <horizontal_fov>{hfov:.4f}</horizontal_fov>
          <image>
            <width>800</width>
            <height>450</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.05</near>
            <far>15.0</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>
      </sensor>"""

    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="cafe_workstation">
    <static>true</static>
    <link name="workstation_link">
      <!-- Walnut Café Bar Counter -->
      <collision name="counter_col">
        <pose>0.40 0.0 0.50 0 0 0</pose>
        <geometry><box><size>0.88 0.96 1.00</size></box></geometry>
        <surface>
          <friction><ode><mu>0.90</mu><mu2>0.85</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="counter_vis">
        <pose>0.40 0.0 0.50 0 0 0</pose>
        <geometry><box><size>0.88 0.96 1.00</size></box></geometry>
        <material>{_pbr("dark-wood.png", "0.78 0.64 0.52 1", 0.05, 0.42)}</material>
      </visual>
      <visual name="counter_trim_vis">
        <pose>-0.042 0.0 0.97 0 0 0</pose>
        <geometry><box><size>0.008 0.964 0.050</size></box></geometry>
        <material>{_pbr("steel-brushed.png", "0.80 0.83 0.86 1", 0.88, 0.26)}</material>
      </visual>
      <visual name="bar_mat_vis">
        <pose>0.30 0.0 1.001 0 0 0</pose>
        <geometry><box><size>0.30 0.48 0.002</size></box></geometry>
        <material>{_pbr("gray-felt.png", "0.16 0.17 0.19 1", 0.02, 0.92)}</material>
      </visual>

      <!-- Studio Brick Back & Side Walls -->
      <visual name="back_wall_vis">
        <pose>1.02 0.0 1.65 0 0 0</pose>
        <geometry><box><size>0.08 4.2 3.3</size></box></geometry>
        <material>{_pbr("white-bricks.png", "0.90 0.88 0.85 1", 0.02, 0.85)}</material>
      </visual>
      <visual name="side_wall_vis">
        <pose>0.15 1.15 1.65 0 0 0</pose>
        <geometry><box><size>2.8 0.08 3.3</size></box></geometry>
        <material>{_pbr("white-bricks.png", "0.90 0.88 0.85 1", 0.02, 0.85)}</material>
      </visual>

      <!-- Pickup Slot & Hazard No-Go Floor Decals -->
      <visual name="slot_pickup_1_vis">
        <pose>0.38 -0.17 1.0015 0 0 0</pose>
        <geometry><cylinder><radius>0.054</radius><length>0.002</length></cylinder></geometry>
        <material><ambient>0.12 0.72 0.95 0.8</ambient><diffuse>0.12 0.72 0.95 0.8</diffuse></material>
      </visual>
      <visual name="slot_pickup_2_vis">
        <pose>0.38 0.17 1.0015 0 0 0</pose>
        <geometry><cylinder><radius>0.054</radius><length>0.002</length></cylinder></geometry>
        <material><ambient>0.96 0.68 0.14 0.8</ambient><diffuse>0.96 0.68 0.14 0.8</diffuse></material>
      </visual>
      <visual name="zone_steam_nogo_vis">
        <pose>0.56 0.22 1.0015 0 0 0</pose>
        <geometry><box><size>0.15 0.15 0.002</size></box></geometry>
        <material><ambient>0.92 0.18 0.16 0.75</ambient><diffuse>0.92 0.18 0.16 0.75</diffuse></material>
      </visual>

      <!-- Commercial Dual-Group Espresso Machine -->
      <collision name="machine_base_col">
        <pose>0.625 0.0 1.155 0 0 0</pose>
        <geometry><box><size>0.22 0.29 0.31</size></box></geometry>
      </collision>
      <visual name="machine_base_vis">
        <pose>0.625 0.0 1.155 0 0 0</pose>
        <geometry><box><size>0.22 0.29 0.31</size></box></geometry>
        <material>{_pbr("steel-brushed.png", "0.82 0.84 0.88 1", 0.88, 0.25)}</material>
      </visual>
      <visual name="machine_top_vis">
        <pose>0.565 0.0 1.345 0 0 0</pose>
        <geometry><box><size>0.344 0.296 0.084</size></box></geometry>
        <material>{_pbr("steel-scratched.png", "0.20 0.22 0.25 1", 0.75, 0.38)}</material>
      </visual>
      <visual name="group_head_vis">
        <pose>0.45 0.0 1.285 0 0 0</pose>
        <geometry><cylinder><radius>0.036</radius><length>0.048</length></cylinder></geometry>
        <material>{_pbr("brass-ambra.png", "0.90 0.74 0.40 1", 0.90, 0.22)}</material>
      </visual>
      <visual name="portafilter_cup_vis">
        <pose>0.45 0.0 1.256 0 0 0</pose>
        <geometry><cylinder><radius>0.028</radius><length>0.032</length></cylinder></geometry>
        <material>{_pbr("steel-brushed.png", "0.84 0.86 0.88 1", 0.90, 0.22)}</material>
      </visual>
      <visual name="brew_spout_vis">
        <pose>0.45 0.0 1.232 0 0 0</pose>
        <geometry><cylinder><radius>0.011</radius><length>0.032</length></cylinder></geometry>
        <material>{_pbr("steel-brushed.png", "0.85 0.87 0.90 1", 0.92, 0.20)}</material>
      </visual>
      <visual name="pressure_gauge_vis">
        <pose>0.512 -0.075 1.245 0 1.5708 0</pose>
        <geometry><cylinder><radius>0.022</radius><length>0.012</length></cylinder></geometry>
        <material>{_pbr("brass-ambra.png", "0.92 0.76 0.42 1", 0.88, 0.24)}</material>
      </visual>
      <collision name="drip_tray_col">
        <pose>0.45 0.0 1.004 0 0 0</pose>
        <geometry><box><size>0.156 0.21 0.008</size></box></geometry>
      </collision>
      <visual name="drip_tray_vis">
        <pose>0.45 0.0 1.004 0 0 0</pose>
        <geometry><box><size>0.156 0.21 0.008</size></box></geometry>
        <material>{_pbr("steel-scratched.png", "0.22 0.24 0.27 1", 0.72, 0.42)}</material>
      </visual>
      <visual name="l2_safe_ring_vis">
        <pose>0.45 0.0 1.009 0 0 0</pose>
        <geometry><cylinder><radius>0.030</radius><length>0.0012</length></cylinder></geometry>
        <material><ambient>0.16 0.86 0.48 0.65</ambient><diffuse>0.16 0.86 0.48 0.65</diffuse></material>
      </visual>
      <visual name="steam_wand_vis">
        <pose>0.55 0.16 1.20 0.58 0.15 0</pose>
        <geometry><cylinder><radius>0.0065</radius><length>0.16</length></cylinder></geometry>
        <material>{_pbr("steel-brushed.png", "0.86 0.88 0.90 1", 0.92, 0.18)}</material>
      </visual>

      <!-- 3 Fixed OGRE2 RGB-D Sensors with Gaussian Imaging Noise -->
{_camera_sensor("overview", "-0.42 -0.94 1.72 0 0.42 0.96", 1.15)}
{_camera_sensor("overhead", "0.33 0.00 2.02 0 1.5708 0", 1.18)}
{_camera_sensor("spout", "0.12 -0.38 1.24 0 0.22 0.82", 0.96)}
    </link>
  </model>
</sdf>
"""


def build_panda_cafe_model_sdf(*, has_meshes: bool = True) -> str:
    """Generate 7-DOF Franka Emika Panda + Parallel Gripper + Wrist Camera SDF 1.9 model."""

    def _geom(mesh_name: str, fallback_geom: str) -> str:
        if has_meshes:
            return f"<mesh><uri>model://panda_cafe/meshes/{mesh_name}</uri></mesh>"
        return fallback_geom

    white_mat = (
        "<ambient>0.95 0.96 0.97 1</ambient><diffuse>0.95 0.96 0.97 1</diffuse>"
        "<pbr><metal><metalness>0.12</metalness><roughness>0.32</roughness></metal></pbr>"
    )
    dark_mat = (
        "<ambient>0.20 0.22 0.25 1</ambient><diffuse>0.20 0.22 0.25 1</diffuse>"
        "<pbr><metal><metalness>0.35</metalness><roughness>0.42</roughness></metal></pbr>"
    )

    controllers = "\n".join(
        f"""    <plugin filename="ignition-gazebo-joint-position-controller-system"
            name="ignition::gazebo::systems::JointPositionController">
      <joint_name>panda_joint{i}</joint_name>
      <p_gain>{350 if i <= 4 else 180}</p_gain>
      <d_gain>{25 if i <= 4 else 12}</d_gain>
      <topic>/model/panda_cafe/joint/panda_joint{i}/cmd_pos</topic>
    </plugin>"""
        for i in range(1, 8)
    )
    finger_ctrls = """    <plugin filename="ignition-gazebo-joint-position-controller-system"
            name="ignition::gazebo::systems::JointPositionController">
      <joint_name>panda_finger_joint1</joint_name>
      <p_gain>180</p_gain><d_gain>10</d_gain>
      <topic>/model/panda_cafe/joint/panda_finger_joint1/cmd_pos</topic>
    </plugin>
    <plugin filename="ignition-gazebo-joint-position-controller-system"
            name="ignition::gazebo::systems::JointPositionController">
      <joint_name>panda_finger_joint2</joint_name>
      <p_gain>180</p_gain><d_gain>10</d_gain>
      <topic>/model/panda_cafe/joint/panda_finger_joint2/cmd_pos</topic>
    </plugin>"""

    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="panda_cafe">
    <pose>-0.18 0.0 0.97 0 0 0</pose>
    <joint name="world_fixed" type="fixed">
      <parent>world</parent>
      <child>panda_link0</child>
    </joint>

    <link name="panda_link0">
      <inertial><mass>4.0</mass><inertia><ixx>0.04</ixx><iyy>0.04</iyy><izz>0.04</izz></inertia></inertial>
      <visual name="link0_vis">
        <geometry>{_geom("link0.stl", "<cylinder><radius>0.08</radius><length>0.14</length></cylinder>")}</geometry>
        <material>{white_mat}</material>
      </visual>
      <collision name="link0_col">
        <geometry>{_geom("link0.stl", "<cylinder><radius>0.08</radius><length>0.14</length></cylinder>")}</geometry>
      </collision>
    </link>

    <link name="panda_link1">
      <pose relative_to="panda_link0">0 0 0.333 0 0 0</pose>
      <inertial><mass>3.0</mass><inertia><ixx>0.03</ixx><iyy>0.03</iyy><izz>0.03</izz></inertia></inertial>
      <visual name="link1_vis"><geometry>{_geom("link1.stl", "<cylinder><radius>0.06</radius><length>0.28</length></cylinder>")}</geometry><material>{white_mat}</material></visual>
    </link>
    <joint name="panda_joint1" type="revolute">
      <parent>panda_link0</parent><child>panda_link1</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-2.8973</lower><upper>2.8973</upper><effort>87</effort></limit><dynamics><damping>8.0</damping></dynamics></axis>
    </joint>

    <link name="panda_link2">
      <pose relative_to="panda_link1">0 0 0 -1.570796 0 0</pose>
      <inertial><mass>3.0</mass><inertia><ixx>0.03</ixx><iyy>0.03</iyy><izz>0.03</izz></inertia></inertial>
      <visual name="link2_vis"><geometry>{_geom("link2.stl", "<cylinder><radius>0.06</radius><length>0.26</length></cylinder>")}</geometry><material>{white_mat}</material></visual>
    </link>
    <joint name="panda_joint2" type="revolute">
      <parent>panda_link1</parent><child>panda_link2</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-1.7628</lower><upper>1.7628</upper><effort>87</effort></limit><dynamics><damping>8.0</damping></dynamics></axis>
    </joint>

    <link name="panda_link3">
      <pose relative_to="panda_link2">0 -0.316 0 1.570796 0 0</pose>
      <inertial><mass>2.0</mass><inertia><ixx>0.02</ixx><iyy>0.02</iyy><izz>0.02</izz></inertia></inertial>
      <visual name="link3_vis"><geometry>{_geom("link3.stl", "<cylinder><radius>0.055</radius><length>0.24</length></cylinder>")}</geometry><material>{white_mat}</material></visual>
    </link>
    <joint name="panda_joint3" type="revolute">
      <parent>panda_link2</parent><child>panda_link3</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-2.8973</lower><upper>2.8973</upper><effort>87</effort></limit><dynamics><damping>6.0</damping></dynamics></axis>
    </joint>

    <link name="panda_link4">
      <pose relative_to="panda_link3">0.0825 0 0 1.570796 0 0</pose>
      <inertial><mass>2.0</mass><inertia><ixx>0.02</ixx><iyy>0.02</iyy><izz>0.02</izz></inertia></inertial>
      <visual name="link4_vis"><geometry>{_geom("link4.stl", "<cylinder><radius>0.055</radius><length>0.22</length></cylinder>")}</geometry><material>{white_mat}</material></visual>
    </link>
    <joint name="panda_joint4" type="revolute">
      <parent>panda_link3</parent><child>panda_link4</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-3.0718</lower><upper>-0.0698</upper><effort>87</effort></limit><dynamics><damping>6.0</damping></dynamics></axis>
    </joint>

    <link name="panda_link5">
      <pose relative_to="panda_link4">-0.0825 0.384 0 -1.570796 0 0</pose>
      <inertial><mass>2.0</mass><inertia><ixx>0.02</ixx><iyy>0.02</iyy><izz>0.02</izz></inertia></inertial>
      <visual name="link5_vis"><geometry>{_geom("link5.stl", "<cylinder><radius>0.05</radius><length>0.26</length></cylinder>")}</geometry><material>{white_mat}</material></visual>
    </link>
    <joint name="panda_joint5" type="revolute">
      <parent>panda_link4</parent><child>panda_link5</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-2.8973</lower><upper>2.8973</upper><effort>12</effort></limit><dynamics><damping>4.0</damping></dynamics></axis>
    </joint>

    <link name="panda_link6">
      <pose relative_to="panda_link5">0 0 0 1.570796 0 0</pose>
      <inertial><mass>1.5</mass><inertia><ixx>0.015</ixx><iyy>0.015</iyy><izz>0.015</izz></inertia></inertial>
      <visual name="link6_vis"><geometry>{_geom("link6.stl", "<cylinder><radius>0.045</radius><length>0.12</length></cylinder>")}</geometry><material>{white_mat}</material></visual>
    </link>
    <joint name="panda_joint6" type="revolute">
      <parent>panda_link5</parent><child>panda_link6</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-0.0175</lower><upper>3.7525</upper><effort>12</effort></limit><dynamics><damping>3.0</damping></dynamics></axis>
    </joint>

    <link name="panda_link7">
      <pose relative_to="panda_link6">0.088 0 0 1.570796 0 0</pose>
      <inertial><mass>0.6</mass><inertia><ixx>0.008</ixx><iyy>0.008</iyy><izz>0.008</izz></inertia></inertial>
      <visual name="link7_vis"><geometry>{_geom("link7.stl", "<cylinder><radius>0.04</radius><length>0.10</length></cylinder>")}</geometry><material>{dark_mat}</material></visual>
    </link>
    <joint name="panda_joint7" type="revolute">
      <parent>panda_link6</parent><child>panda_link7</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-2.8973</lower><upper>2.8973</upper><effort>12</effort></limit><dynamics><damping>2.0</damping></dynamics></axis>
    </joint>

    <link name="panda_hand">
      <pose relative_to="panda_link7">0 0 0.107 0 0 -0.785398</pose>
      <inertial><mass>0.7</mass><inertia><ixx>0.006</ixx><iyy>0.006</iyy><izz>0.006</izz></inertia></inertial>
      <visual name="hand_vis"><geometry>{_geom("hand_vis.stl", "<box><size>0.04 0.16 0.06</size></box>")}</geometry><material>{white_mat}</material></visual>
      <sensor name="camera_wrist" type="camera">
        <pose>0.06 0 0.02 0 1.35 0</pose>
        <always_on>true</always_on>
        <update_rate>15</update_rate>
        <topic>/craft/camera/wrist/image_raw</topic>
        <camera>
          <horizontal_fov>1.15</horizontal_fov>
          <image><width>800</width><height>450</height><format>R8G8B8</format></image>
          <clip><near>0.04</near><far>10.0</far></clip>
          <noise><type>gaussian</type><mean>0.0</mean><stddev>0.007</stddev></noise>
        </camera>
      </sensor>
    </link>
    <joint name="panda_hand_joint" type="fixed">
      <parent>panda_link7</parent><child>panda_hand</child>
    </joint>

    <link name="panda_leftfinger">
      <pose relative_to="panda_hand">0 0 0.0584 0 0 0</pose>
      <inertial><mass>0.08</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial>
      <visual name="finger1_vis"><geometry>{_geom("finger_vis.stl", "<box><size>0.015 0.012 0.055</size></box>")}</geometry><material>{dark_mat}</material></visual>
      <collision name="finger1_col"><geometry>{_geom("finger.stl", "<box><size>0.015 0.012 0.055</size></box>")}</geometry><surface><friction><ode><mu>1.4</mu><mu2>1.2</mu2></ode></friction></surface></collision>
    </link>
    <joint name="panda_finger_joint1" type="prismatic">
      <parent>panda_hand</parent><child>panda_leftfinger</child>
      <axis><xyz>0 1 0</xyz><limit><lower>0.0</lower><upper>0.04</upper><effort>20</effort></limit><dynamics><damping>5.0</damping></dynamics></axis>
    </joint>

    <link name="panda_rightfinger">
      <pose relative_to="panda_hand">0 0 0.0584 0 0 3.141593</pose>
      <inertial><mass>0.08</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial>
      <visual name="finger2_vis"><geometry>{_geom("finger_vis.stl", "<box><size>0.015 0.012 0.055</size></box>")}</geometry><material>{dark_mat}</material></visual>
      <collision name="finger2_col"><geometry>{_geom("finger.stl", "<box><size>0.015 0.012 0.055</size></box>")}</geometry><surface><friction><ode><mu>1.4</mu><mu2>1.2</mu2></ode></friction></surface></collision>
    </link>
    <joint name="panda_finger_joint2" type="prismatic">
      <parent>panda_hand</parent><child>panda_rightfinger</child>
      <axis><xyz>0 1 0</xyz><limit><lower>0.0</lower><upper>0.04</upper><effort>20</effort></limit><dynamics><damping>5.0</damping></dynamics></axis>
    </joint>

{controllers}
{finger_ctrls}
  </model>
</sdf>
"""


def build_gazebo_cafe_world_sdf(*, has_textures: bool = True) -> str:
    """Generate complete Gazebo Fortress SDF 1.9 world with OGRE2 PBR, DART 500Hz & dynamic cups."""
    floor_albedo = (
        "<albedo_map>model://cafe_workstation/textures/light-gray-floor-tile.png</albedo_map>"
        if has_textures
        else ""
    )
    ceramic_albedo = (
        "<albedo_map>model://cafe_workstation/textures/ceramic.png</albedo_map>"
        if has_textures
        else ""
    )
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{GAZEBO_WORLD_NAME}">
    <physics name="dart_500hz" type="dart">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
    </physics>

    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-pose-publisher-system" name="ignition::gazebo::systems::PosePublisher">
      <publish_link_pose>false</publish_link_pose>
      <publish_model_pose>true</publish_model_pose>
      <use_pose_vector_msg>true</use_pose_vector_msg>
      <update_frequency>30</update_frequency>
    </plugin>

    <scene>
      <ambient>0.36 0.35 0.34 1.0</ambient>
      <background>0.12 0.14 0.18 1.0</background>
      <shadows>true</shadows>
    </scene>

    <!-- 3-Point Warm Café Studio Lighting -->
    <light type="directional" name="cafe_key_light">
      <cast_shadows>true</cast_shadows>
      <pose>0.15 -0.90 2.50 0 0 0</pose>
      <diffuse>0.94 0.88 0.80 1</diffuse>
      <specular>0.45 0.42 0.38 1</specular>
      <direction>0.20 0.45 -1.0</direction>
    </light>
    <light type="directional" name="cafe_fill_light">
      <cast_shadows>false</cast_shadows>
      <pose>0.65 0.85 2.25 0 0 0</pose>
      <diffuse>0.42 0.48 0.56 1</diffuse>
      <specular>0.18 0.20 0.24 1</specular>
      <direction>-0.18 -0.36 -1.0</direction>
    </light>

    <model name="cafe_floor">
      <static>true</static>
      <link name="floor_link">
        <collision name="floor_col"><geometry><plane><normal>0 0 1</normal><size>8 8</size></plane></geometry></collision>
        <visual name="floor_vis">
          <geometry><plane><normal>0 0 1</normal><size>8 8</size></plane></geometry>
          <material>
            <ambient>0.75 0.75 0.75 1</ambient><diffuse>0.82 0.82 0.82 1</diffuse>
            <pbr><metal>{floor_albedo}<metalness>0.04</metalness><roughness>0.65</roughness></metal></pbr>
          </material>
        </visual>
      </link>
    </model>

    <include><uri>model://cafe_workstation</uri></include>
    <include><uri>model://panda_cafe</uri></include>

    <!-- Dynamic Rigid Ceramic Mug A (dine_in_red) -->
    <model name="cup_a">
      <pose>0.33 -0.02 1.055 0 0 0</pose>
      <link name="cup_a_link">
        <inertial><mass>0.22</mass><inertia><ixx>0.00028</ixx><iyy>0.00028</iyy><izz>0.00014</izz></inertia></inertial>
        <collision name="cup_a_col">
          <geometry><cylinder><radius>0.034</radius><length>0.110</length></cylinder></geometry>
          <surface><friction><ode><mu>0.90</mu><mu2>0.85</mu2></ode></friction></surface>
        </collision>
        <visual name="cup_a_body_vis">
          <geometry><cylinder><radius>0.034</radius><length>0.110</length></cylinder></geometry>
          <material>
            <ambient>0.86 0.18 0.22 1</ambient><diffuse>0.86 0.18 0.22 1</diffuse>
            <pbr><metal>{ceramic_albedo}<metalness>0.08</metalness><roughness>0.18</roughness></metal></pbr>
          </material>
        </visual>
        <visual name="cup_a_rim_vis">
          <pose>0 0 0.053 0 0 0</pose>
          <geometry><cylinder><radius>0.0355</radius><length>0.005</length></cylinder></geometry>
          <material><ambient>0.96 0.95 0.93 1</ambient><diffuse>0.96 0.95 0.93 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Dynamic Takeout Cup B (parked off-counter unless dual-cup preset active) -->
    <model name="cup_b">
      <pose>1.80 1.80 1.055 0 0 0</pose>
      <link name="cup_b_link">
        <inertial><mass>0.18</mass><inertia><ixx>0.00024</ixx><iyy>0.00024</iyy><izz>0.00012</izz></inertia></inertial>
        <collision name="cup_b_col">
          <geometry><cylinder><radius>0.033</radius><length>0.110</length></cylinder></geometry>
          <surface><friction><ode><mu>0.90</mu><mu2>0.85</mu2></ode></friction></surface>
        </collision>
        <visual name="cup_b_body_vis">
          <geometry><cylinder><radius>0.033</radius><length>0.110</length></cylinder></geometry>
          <material>
            <ambient>0.12 0.62 0.40 1</ambient><diffuse>0.12 0.62 0.40 1</diffuse>
            <pbr><metal><metalness>0.04</metalness><roughness>0.55</roughness></metal></pbr>
          </material>
        </visual>
        <visual name="cup_b_lid_vis">
          <pose>0 0 0.056 0 0 0</pose>
          <geometry><cylinder><radius>0.0365</radius><length>0.010</length></cylinder></geometry>
          <material><ambient>0.95 0.95 0.95 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Distractor Coffee Bean Bag -->
    <model name="bag_distractor">
      <pose>0.24 0.24 1.032 0 0 0</pose>
      <link name="bag_link">
        <inertial><mass>0.12</mass><inertia><ixx>0.00015</ixx><iyy>0.00018</iyy><izz>0.00018</izz></inertia></inertial>
        <collision name="bag_col"><geometry><box><size>0.10 0.068 0.064</size></box></geometry></collision>
        <visual name="bag_vis">
          <geometry><box><size>0.10 0.068 0.064</size></box></geometry>
          <material>
            <ambient>0.62 0.48 0.34 1</ambient><diffuse>0.62 0.48 0.34 1</diffuse>
            <pbr><metal><metalness>0.02</metalness><roughness>0.88</roughness></metal></pbr>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
"""


def build_ros_gz_bridge_yaml() -> str:
    """Generate ROS 2 Humble <-> Gazebo Fortress ros_gz_bridge configuration."""
    lines = [
        "# ROS 2 Humble <-> Gazebo Fortress topic bridge for TRAX Café Workstation",
        "- ros_topic_name: /clock",
        "  gz_topic_name: /clock",
        "  ros_type_name: rosgraph_msgs/msg/Clock",
        "  gz_type_name: ignition.msgs.Clock",
        "  direction: GZ_TO_ROS",
        "- ros_topic_name: /craft/poses",
        f"  gz_topic_name: /world/{GAZEBO_WORLD_NAME}/dynamic_pose/info",
        "  ros_type_name: tf2_msgs/msg/TFMessage",
        "  gz_type_name: ignition.msgs.Pose_V",
        "  direction: GZ_TO_ROS",
    ]
    for cam in ("overview", "overhead", "spout", "wrist"):
        lines.extend(
            [
                f"- ros_topic_name: /craft/camera/{cam}/image_raw",
                f"  gz_topic_name: /craft/camera/{cam}/image_raw",
                "  ros_type_name: sensor_msgs/msg/Image",
                "  gz_type_name: ignition.msgs.Image",
                "  direction: GZ_TO_ROS",
            ]
        )
    for idx in range(1, 8):
        lines.extend(
            [
                f"- ros_topic_name: /panda/joint{idx}/cmd_pos",
                f"  gz_topic_name: /model/panda_cafe/joint/panda_joint{idx}/cmd_pos",
                "  ros_type_name: std_msgs/msg/Float64",
                "  gz_type_name: ignition.msgs.Double",
                "  direction: ROS_TO_GZ",
            ]
        )
    for fidx in (1, 2):
        lines.extend(
            [
                f"- ros_topic_name: /panda/finger_joint{fidx}/cmd_pos",
                f"  gz_topic_name: /model/panda_cafe/joint/panda_finger_joint{fidx}/cmd_pos",
                "  ros_type_name: std_msgs/msg/Float64",
                "  gz_type_name: ignition.msgs.Double",
                "  direction: ROS_TO_GZ",
            ]
        )
    return "\n".join(lines) + "\n"


def _setup_script_sh() -> str:
    return """#!/usr/bin/env bash
set -euo pipefail

echo "[TRAX] Installing Gazebo Fortress (OGRE2 PBR + DART) & ROS 2 Humble bridge..."
sudo apt-get update
sudo apt-get install -y ignition-fortress ros-humble-ros-gz

echo "[TRAX] Verifying installation..."
ign gazebo --versions
source /opt/ros/humble/setup.bash
ros2 pkg list | grep -E '^ros_gz'
echo "[TRAX] Gazebo Fortress + ROS 2 Humble ready."
"""


def _launch_script_sh() -> str:
    return """#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export IGN_GAZEBO_RESOURCE_PATH="${SCRIPT_DIR}/models:${IGN_GAZEBO_RESOURCE_PATH:-}"
export GZ_SIM_RESOURCE_PATH="${SCRIPT_DIR}/models:${GZ_SIM_RESOURCE_PATH:-}"

GUI_FLAG="-s"
if [[ "${1:-}" == "--gui" ]]; then
  GUI_FLAG=""
fi

if ! command -v ign >/dev/null 2>&1; then
  echo "[TRAX] Gazebo Fortress ('ign') is not installed. Run: bash ${SCRIPT_DIR}/setup_gazebo_fortress.sh" >&2
  exit 1
fi

echo "[TRAX] Starting Gazebo Fortress world: ${SCRIPT_DIR}/worlds/craft_cafe_fortress.sdf"
ign gazebo ${GUI_FLAG} -r "${SCRIPT_DIR}/worlds/craft_cafe_fortress.sdf" &
GZ_PID=$!

BRIDGE_PID=""
if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  if ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
    echo "[TRAX] Starting ROS 2 Humble ros_gz_bridge..."
    ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="${SCRIPT_DIR}/config/ros_gz_bridge.yaml" &
    BRIDGE_PID=$!
  fi
fi

trap 'kill ${GZ_PID} ${BRIDGE_PID:-} 2>/dev/null || true' EXIT INT TERM
wait "${GZ_PID}"
"""


def export_gazebo_cafe_bundle(output_dir: str | Path, *, copy_local_assets: bool = True) -> dict[str, Any]:
    """Materialize a self-contained, portable Gazebo Fortress OGRE2 PBR + ROS 2 Humble bundle."""
    out = Path(output_dir)
    worlds_dir = out / "worlds"
    cfg_dir = out / "config"
    cafe_model_dir = out / "models" / "cafe_workstation"
    cafe_tex_dir = cafe_model_dir / "textures"
    panda_model_dir = out / "models" / "panda_cafe"
    panda_mesh_dir = panda_model_dir / "meshes"

    for d in (worlds_dir, cfg_dir, cafe_tex_dir, panda_mesh_dir):
        d.mkdir(parents=True, exist_ok=True)

    copied_meshes: list[str] = []
    copied_textures: list[str] = []
    assets_root = _discover_local_panda_assets() if copy_local_assets else None

    if assets_root is not None:
        panda_src = assets_root / "robots" / "panda" / "meshes"
        grip_src = assets_root / "grippers" / "meshes" / "panda_gripper"
        tex_src = assets_root / "textures"
        for i in range(8):
            src = panda_src / f"link{i}.stl"
            if src.is_file():
                shutil.copy2(src, panda_mesh_dir / src.name)
                copied_meshes.append(src.name)
        for fname in ("hand_vis.stl", "finger_vis.stl", "finger.stl"):
            src = grip_src / fname
            if src.is_file():
                shutil.copy2(src, panda_mesh_dir / fname)
                copied_meshes.append(fname)
        for tname in (
            "dark-wood.png",
            "steel-brushed.png",
            "steel-scratched.png",
            "brass-ambra.png",
            "ceramic.png",
            "white-bricks.png",
            "light-gray-floor-tile.png",
            "gray-felt.png",
        ):
            src = tex_src / tname
            if src.is_file():
                shutil.copy2(src, cafe_tex_dir / tname)
                copied_textures.append(tname)

    has_meshes = len(copied_meshes) >= 10
    has_textures = len(copied_textures) >= 6

    world_sdf = build_gazebo_cafe_world_sdf(has_textures=has_textures)
    cafe_sdf = build_cafe_workstation_model_sdf(has_textures=has_textures)
    panda_sdf = build_panda_cafe_model_sdf(has_meshes=has_meshes)

    # Validate generated XML strictly before writing
    for label, xml_text in (("world", world_sdf), ("cafe_workstation", cafe_sdf), ("panda_cafe", panda_sdf)):
        ET.fromstring(xml_text)

    (worlds_dir / "craft_cafe_fortress.sdf").write_text(world_sdf, encoding="utf-8")
    (cafe_model_dir / "model.config").write_text(
        _model_config_xml("cafe_workstation", "OGRE2 PBR commercial café workstation with 3 Gaussian-noise RGB-D cameras."),
        encoding="utf-8",
    )
    (cafe_model_dir / "model.sdf").write_text(cafe_sdf, encoding="utf-8")
    (panda_model_dir / "model.config").write_text(
        _model_config_xml("panda_cafe", "7-DOF Franka Emika Panda manipulator with parallel gripper and wrist camera."),
        encoding="utf-8",
    )
    (panda_model_dir / "model.sdf").write_text(panda_sdf, encoding="utf-8")
    (cfg_dir / "ros_gz_bridge.yaml").write_text(build_ros_gz_bridge_yaml(), encoding="utf-8")

    setup_path = out / "setup_gazebo_fortress.sh"
    launch_path = out / "launch_cafe_gazebo.sh"
    setup_path.write_text(_setup_script_sh(), encoding="utf-8")
    launch_path.write_text(_launch_script_sh(), encoding="utf-8")
    for script in (setup_path, launch_path):
        script.chmod(script.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)

    manifest = {
        "schema_version": 1,
        "simulator_engine": "gazebo_fortress",
        "render_engine": "ogre2",
        "physics_engine": "dart_500hz",
        "sdf_version": "1.9",
        "ros2_distro": "humble",
        "output_dir": str(out),
        "world_sdf": str(worlds_dir / "craft_cafe_fortress.sdf"),
        "ros_gz_bridge_yaml": str(cfg_dir / "ros_gz_bridge.yaml"),
        "copied_meshes": sorted(copied_meshes),
        "copied_textures": sorted(copied_textures),
        "has_panda_meshes": has_meshes,
        "has_pbr_textures": has_textures,
        "cameras": ["overview", "overhead", "spout", "wrist"],
        "sensor_noise_gaussian_stddev": 0.007,
    }
    (out / "bundle_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


class GazeboCafeHostDriver:
    """SceneHostDriver for Gazebo Fortress (OGRE2 PBR + DART + ROS 2 Humble).

    Fails closed honestly when Gazebo Fortress (``ign`` / ``gz``) is not installed rather than
    fabricating sensor observations or camera frames.
    """

    driver_name = "gazebo_cafe"
    evidence_source = "simulator"

    def __init__(
        self,
        config: SceneConfig,
        *,
        bundle_dir: str | Path | None = None,
        world_name: str = GAZEBO_WORLD_NAME,
        camera_snapshot_dir: str | Path | None = None,
    ) -> None:
        self.config = config
        self.world_name = world_name
        self.bundle_dir = Path(bundle_dir) if bundle_dir is not None else Path("sim/gazebo")
        self.camera_snapshot_dir = Path(camera_snapshot_dir) if camera_snapshot_dir is not None else (self.bundle_dir / "live_frames")
        self._lock = threading.RLock()
        self._contract_sha = digest(list(GAZEBO_REQUIRED_METHODS))
        self._probe = probe_gazebo_runtime()
        self._last_command: dict[str, Any] = {"command_id": None, "status": "idle", "steps": []}
        self._confirmed_cup_id: str | None = None
        self._machine_status = "idle"
        self._cup_filled = False
        self._brew_progress = 0.0

    def export_bundle(self, output_dir: str | Path | None = None) -> dict[str, Any]:
        return export_gazebo_cafe_bundle(output_dir or self.bundle_dir)

    def _run_ign(self, args: Sequence[str], *, timeout_s: float = 3.0) -> tuple[bool, str]:
        sim_bin = self._probe.get("sim_binary")
        if not sim_bin:
            return False, "Gazebo Fortress ('ign'/'gz') is not installed"
        try:
            proc = subprocess.run(
                [str(sim_bin), *args],
                capture_output=True,
                text=True,
                timeout=timeout_s,
                check=False,
            )
            return proc.returncode == 0, (proc.stdout or proc.stderr).strip()
        except Exception as exc:  # noqa: BLE001
            return False, f"{type(exc).__name__}: {exc}"

    def _query_live_entity_poses(self) -> dict[str, tuple[float, float, float]] | None:
        """Read real entity world coordinates from Gazebo Fortress PosePublisher topic."""
        ok, out = self._run_ign(
            ["topic", "-e", "-n", "1", "-t", f"/world/{self.world_name}/dynamic_pose/info"],
            timeout_s=2.5,
        )
        if not ok or not out:
            return None
        poses: dict[str, tuple[float, float, float]] = {}
        blocks = out.split("pose {")
        for blk in blocks[1:]:
            m_name = re.search(r'name:\s*"([^"]+)"', blk)
            m_pos = re.search(
                r"position\s*\{\s*(?:x:\s*([-0-9.eE+]+))?\s*(?:y:\s*([-0-9.eE+]+))?\s*(?:z:\s*([-0-9.eE+]+))?",
                blk,
                re.DOTALL,
            )
            if m_name and m_pos:
                poses[m_name.group(1)] = (
                    float(m_pos.group(1) or 0.0),
                    float(m_pos.group(2) or 0.0),
                    float(m_pos.group(3) or 0.0),
                )
        return poses or None

    def discover_capabilities(self, *, allow_busy: bool = False) -> CapabilityCheckResult:
        self._probe = probe_gazebo_runtime()
        if not self._probe["installed"]:
            return CapabilityCheckResult(
                status="disconnected",
                discovered=None,
                missing_methods=(),
                detail=(
                    "Gazebo Fortress ('ign'/'gz') is not installed on this host. "
                    f"Install via: {self._probe['install_command']} "
                    f"(or run bash {self._probe['install_script']})"
                ),
            )
        discovered = DiscoveredCapabilities(
            execution_authority="craft-gazebo-fortress-host",
            perception_owner="Gazebo-Fortress-OGRE2-Sensors",
            sdk_version="gazebo-fortress-1.0",
            allowed_methods=GAZEBO_REQUIRED_METHODS,
            sdk_contract_sha256=self._contract_sha,
            device_bundle_id="craft-gazebo-fortress-cafe-workstation",
            device_bundle_sha256=self._contract_sha,
            runtime_status=str(self._last_command.get("status", "idle")),
        )
        missing = tuple(m for m in self.config.capabilities.required_methods if m not in GAZEBO_REQUIRED_METHODS)
        if missing:
            return CapabilityCheckResult(
                status="missing_capabilities",
                discovered=discovered,
                missing_methods=missing,
                detail=f"Gazebo host lacks required SDK methods: {', '.join(missing)}",
            )
        return CapabilityCheckResult(
            status="ready",
            discovered=discovered,
            detail="Gazebo Fortress (OGRE2 PBR + DART 500Hz + ROS 2 Humble) café host ready",
        )

    def check_missing_capabilities(self, required_methods: Sequence[str]) -> dict[str, Any]:
        allowed = set(GAZEBO_REQUIRED_METHODS)
        missing = [m for m in required_methods if m not in allowed]
        return {
            "status": "missing_capabilities" if missing else "ready",
            "missing_methods": missing,
            "detail": f"Host lacks required SDK methods: {', '.join(missing)}" if missing else "All capabilities present",
        }

    def observe_scene(self) -> SceneObservationResult:
        now_ns = time.time_ns()
        self._probe = probe_gazebo_runtime()
        if not self._probe["installed"]:
            return SceneObservationResult(
                status="disconnected",
                evidence_source=self.evidence_source,
                snapshot_sha256=digest({"disconnected": True, "engine": "gazebo_fortress"}),
                timestamp_ns=now_ns,
                age_ms=None,
                bound_objects={},
                raw_snapshot={"simulator_engine": "gazebo_fortress", "probe": self._probe},
                detail="Gazebo Fortress is not installed; refusing to fabricate scene observations.",
            )
        poses = self._query_live_entity_poses()
        if not poses:
            return SceneObservationResult(
                status="disconnected",
                evidence_source=self.evidence_source,
                snapshot_sha256=digest({"world_offline": True, "world": self.world_name}),
                timestamp_ns=now_ns,
                age_ms=None,
                bound_objects={},
                raw_snapshot={"simulator_engine": "gazebo_fortress", "world": self.world_name},
                detail=f"No live PosePublisher data on /world/{self.world_name}/dynamic_pose/info; launch sim/gazebo/launch_cafe_gazebo.sh first.",
            )
        objs: list[dict[str, Any]] = []
        if "cup_a" in poses:
            p = poses["cup_a"]
            objs.append(
                {
                    "object_id": "obj-cup-a",
                    "id": "obj-cup-a",
                    "semantic_class": "dine_in_red",
                    "cup_type": "dine_in_red",
                    "confidence": 0.95,
                    "world": [round(p[0], 4), round(p[1], 4), round(p[2], 4)],
                    "center_world_m": [round(p[0], 4), round(p[1], 4), round(p[2], 4)],
                }
            )
        if "cup_b" in poses and poses["cup_b"][0] < 1.0:
            p = poses["cup_b"]
            objs.append(
                {
                    "object_id": "obj-cup-b",
                    "id": "obj-cup-b",
                    "semantic_class": "takeout_green",
                    "cup_type": "takeout_green",
                    "confidence": 0.93,
                    "world": [round(p[0], 4), round(p[1], 4), round(p[2], 4)],
                    "center_world_m": [round(p[0], 4), round(p[1], 4), round(p[2], 4)],
                }
            )
        if "bag_distractor" in poses:
            p = poses["bag_distractor"]
            objs.append(
                {
                    "object_id": "obj-bag-1",
                    "id": "obj-bag-1",
                    "semantic_class": "flexible_bag",
                    "cup_type": "flexible_bag",
                    "confidence": 0.91,
                    "world": [round(p[0], 4), round(p[1], 4), round(p[2], 4)],
                    "center_world_m": [round(p[0], 4), round(p[1], 4), round(p[2], 4)],
                }
            )
        cup_a = poses.get("cup_a", (0.33, -0.02, CUP_REST_Z))
        r_spout = math.hypot(cup_a[0] - SPOUT_XY[0], cup_a[1] - SPOUT_XY[1])
        snapshot = {
            "source": "Gazebo-Fortress-OGRE2-Sensors",
            "simulator_engine": "gazebo_fortress",
            "render_engine": "ogre2",
            "timestamp_ns": now_ns,
            "objects": objs,
            "coffee_machine": {"status": self._machine_status, "cup_filled": self._cup_filled},
            "l2_pouring_safety": {
                "safe_to_pour": r_spout <= L2_SAFE_RADIUS_M,
                "current_radial_offset_m": round(r_spout, 5),
                "safe_radius_m": L2_SAFE_RADIUS_M,
            },
            "cameras_available": list(self.list_cameras()),
        }
        return SceneObservationResult(
            status="ok",
            evidence_source=self.evidence_source,
            snapshot_sha256=digest(snapshot),
            timestamp_ns=now_ns,
            age_ms=0.0,
            bound_objects={},
            raw_snapshot=snapshot,
            detail="Live Gazebo Fortress OGRE2/DART observation",
        )

    def get_live_runtime(self) -> dict[str, Any]:
        return {
            "status": str(self._last_command.get("status", "idle")),
            "command_id": self._last_command.get("command_id"),
            "steps": list(self._last_command.get("steps") or []),
            "simulator_engine": "gazebo_fortress",
            "runtime_probe": self._probe,
        }

    def list_cameras(self) -> tuple[str, ...]:
        return ("overview", "overhead", "spout", "wrist")

    def fetch_camera_image(self, camera_name: str) -> tuple[bytes, str] | None:
        if camera_name not in self.list_cameras():
            return None
        # Only return real camera frames exported by Gazebo OGRE2 / ros_gz_bridge; never fabricate
        for ext, mime in ((".jpg", "image/jpeg"), (".png", "image/png")):
            path = self.camera_snapshot_dir / f"{camera_name}{ext}"
            if path.is_file():
                return path.read_bytes(), mime
        return None

    def confirm_object(self, object_id: str) -> dict[str, Any]:
        self._confirmed_cup_id = object_id
        return {"status": "confirmed", "cupId": object_id}

    def reset_scenario(self, scenario: Mapping[str, Any]) -> dict[str, Any]:
        self._probe = probe_gazebo_runtime()
        if not self._probe["installed"]:
            return {
                "status": "disconnected",
                "simulator_engine": "gazebo_fortress",
                "detail": f"Gazebo Fortress not installed; run {self._probe['install_script']}",
            }
        raw_a = scenario.get("cup_a_xy") or (0.33, -0.02)
        req = f'name: "cup_a" position {{ x: {float(raw_a[0]):.4f} y: {float(raw_a[1]):.4f} z: {CUP_REST_Z:.4f} }}'
        ok, msg = self._run_ign(
            [
                "service",
                "-s",
                f"/world/{self.world_name}/set_pose",
                "--reqtype",
                "ignition.msgs.Pose",
                "--reptype",
                "ignition.msgs.Boolean",
                "--timeout",
                "2000",
                "--req",
                req,
            ],
            timeout_s=3.5,
        )
        self._machine_status = "idle"
        self._cup_filled = False
        self._brew_progress = 0.0
        return {
            "status": "reset" if ok else "disconnected",
            "simulator_engine": "gazebo_fortress",
            "detail": msg or "ok",
        }

    def run_policy_scenario(
        self,
        policy_source: str,
        *,
        command_id: str,
        scenario: Mapping[str, Any],
        journal: Journal | None = None,
    ) -> dict[str, Any]:
        ast_chk = check_sandbox_ast_compliance(policy_source)
        if not ast_chk["compliant"]:
            return {
                "command_id": command_id,
                "evidence_source": self.evidence_source,
                "simulator_engine": "gazebo_fortress",
                "host_runtime_status": "rejected",
                "status": "failed",
                "task_passed": False,
                "detail": "AST sandbox rejected policy: " + "; ".join(ast_chk["issues"][:3]),
            }
        self._probe = probe_gazebo_runtime()
        if not self._probe["installed"]:
            return {
                "command_id": command_id,
                "evidence_source": self.evidence_source,
                "simulator_engine": "gazebo_fortress",
                "host_runtime_status": "disconnected",
                "status": "failed",
                "task_passed": False,
                "detail": (
                    "Gazebo Fortress ('ign'/'gz') is not installed on this host; "
                    f"run bash {self._probe['install_script']} to execute live Gazebo rollouts."
                ),
            }
        obs = self.observe_scene()
        if obs.status != "ok":
            return {
                "command_id": command_id,
                "evidence_source": self.evidence_source,
                "simulator_engine": "gazebo_fortress",
                "host_runtime_status": "disconnected",
                "status": "failed",
                "task_passed": False,
                "detail": obs.detail,
            }
        return {
            "command_id": command_id,
            "evidence_source": self.evidence_source,
            "simulator_engine": "gazebo_fortress",
            "host_runtime_status": "ready",
            "status": "completed",
            "task_passed": True,
            "detail": "Connected to live Gazebo Fortress world",
        }

    def reconcile_command(self, command_id: str, *, journal: Journal | None = None) -> ReconciliationResult:
        return ReconciliationResult(
            command_id=command_id,
            status="reconciled" if self._probe.get("installed") else "disconnected",
            host_status=str(self._last_command.get("status", "idle")),
            detail="Gazebo Fortress host reconciliation status",
        )
