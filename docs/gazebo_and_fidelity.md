# High-Fidelity Simulation Architecture: Gazebo Fortress (OGRE2 PBR + ROS 2 Humble) & 7-DOF Franka Panda MuJoCo

This document records producer/state ownership, fidelity upgrades, and evidence boundaries for TRAX's two high-fidelity simulator host drivers: **`gazebo_cafe`** and **`mujoco_cafe`**.

---

## 1. Producer & State Ownership (Preserved Strictly per `AGENTS.md`)

| Concern | `gazebo_cafe` (`src/trax/gazebo_cafe_driver.py`) | `mujoco_cafe` (`src/trax/mujoco_cafe_driver.py`) | TRAX Core Policy Layer |
|---|---|---|---|
| **Perception owner** | Gazebo Fortress `ignition-gazebo-sensors-system` (OGRE2 PBR 800×450 RGB cameras with Gaussian imaging noise `stddev=0.007`) + `PosePublisher` bridged via ROS 2 Humble `ros_gz_bridge` | MuJoCo 3.3 EGL renderer (800×450 HD PBR) + optional host-owned CMOS camera noise & RGB-D 3D pose jitter (`sensor_noise=True`) | Read-only consumer of `EvidenceScene` snapshots (never fabricates scene state) |
| **Actuation & Physics owner** | Gazebo Fortress DART 500 Hz physics + 7-DOF Franka Emika Panda `JointPositionController` plugins + parallel fingers | MuJoCo 3.3 implicitfast physics + 7-DOF Franka Emika Panda damped least-squares Jacobian IK + parallel fingers + optional mass/friction/placement DR (`domain_randomization=True`) | AST-sandboxed `run_policy(sdk, params)` calling only the 13-method `EdgeZeroGtCoffeeSDK` contract |
| **Scoring oracle** | Post-run physical poses from Gazebo `PosePublisher` (never exposed to policy) | Post-run ground-truth MuJoCo `data.xpos` (strictly isolated from noisy `observe_scene`) | Independent acceptance & benchmark scoring only |
| **Evidence source tag** | `evidence_source = "simulator"` | `evidence_source = "simulator"` | Never promoted as `physical_robot` approval |

---

## 2. Gazebo Fortress (`driver="gazebo_cafe"`) — OGRE2 PBR + DART 500 Hz + ROS 2 Humble

Export or refresh the portable Gazebo Fortress bundle (zero hard-coded machine paths; uses `model://` URIs):

```bash
.venv/bin/craft gazebo-export --output sim/gazebo
```

Generated artifacts under `sim/gazebo/`:
- `worlds/craft_cafe_fortress.sdf` — SDF 1.9 world with DART 500 Hz physics, OGRE2 PBR rendering, 3-point café studio lighting with soft shadows, and dynamic rigid bodies (`cup_a`, `cup_b`, `bag_distractor`).
- `models/cafe_workstation/model.sdf` — PBR walnut bar counter (`dark-wood.png`), brushed stainless steel espresso machine (`steel-brushed.png`, `metalness=0.88`, `roughness=0.25`), brass portafilter/pressure gauge (`brass-ambra.png`), white brick backdrop (`white-bricks.png`), and 3 fixed OGRE2 cameras (`overview`, `overhead`, `spout`) with Gaussian imaging noise.
- `models/panda_cafe/model.sdf` — 7-DOF Franka Emika Panda with STL meshes, `JointPositionController` plugins on all 7 arm joints + 2 parallel fingers, and a wrist-mounted OGRE2 camera (`wrist`).
- `config/ros_gz_bridge.yaml` — ROS 2 Humble ↔ Gazebo Fortress topic bridge (`/clock`, `/craft/poses`, 4× `/craft/camera/*/image_raw`, 7× arm joint commands, 2× finger commands).
- `setup_gazebo_fortress.sh` — Ubuntu 22.04 + ROS 2 Humble one-command installer (`ignition-fortress` + `ros-humble-ros-gz`).
- `launch_cafe_gazebo.sh` — Starts `ign gazebo` (headless `-s -r` by default, or `--gui` for interactive RTX visualization) alongside `ros_gz_bridge`.

### Launching Live Gazebo Fortress on Ubuntu 22.04 + ROS 2 Humble

```bash
# 1. One-time install (requires sudo)
bash sim/gazebo/setup_gazebo_fortress.sh

# 2. Launch Gazebo Fortress (add --gui to open the OGRE2 PBR viewport on GPU)
bash sim/gazebo/launch_cafe_gazebo.sh --gui

# 3. Connect TRAX Engine / Studio / Sim Host
.venv/bin/craft serve-sim-host --config examples/single_cup_coffee_config.json --driver gazebo_cafe --port 8880
```

> **Fail-Closed Guarantee**: If `ign` / `gz` is not installed or the Gazebo world is offline, `GazeboCafeHostDriver` explicitly returns `status="disconnected"` and `fetch_camera_image() -> None`. It never fabricates observations or camera images.

---

## 3. Upgraded Built-In `mujoco_cafe` (7-DOF Franka Panda + PBR + Sensor Noise & Domain Randomization)

For immediate zero-install execution on headless or GPU servers (including RTX 5090 EGL):
- Automatically loads real **7-DOF Franka Emika Panda** multi-part visual meshes (`link0..link7`, `hand_vis`, `finger_vis`) with damped least-squares Jacobian IK (`mj_jacSite`) and **real PBR textures** (`dark-wood.png`, `steel-brushed.png`, `brass-ambra.png`, `ceramic.png`, `white-bricks.png`, `light-gray-floor-tile.png`, `gray-felt.png`) rendered at `800×450` (16:9 HD).
- Supports realistic host-owned degradation via CLI flags or per-scenario options:
  - `--sensor-noise` / `scenario["sensor_noise"] = True`: CMOS Gaussian readout noise + radial vignetting + exposure jitter on rendered camera frames, plus RGB-D 3D pose estimation jitter (`xy_sigma=2.2 mm`, `z_sigma=1.8 mm`, `tilt_sigma=0.7°`) strictly on `observe_scene` (never corrupting the physical oracle).
  - `--domain-randomization` / `scenario["domain_randomization"] = True`: Seeded per-episode randomization of cup mass (`±20%`), sliding friction (`0.60×–1.15×`), and placement micro-perturbation (`σ=2.2 mm`).

```bash
# Serve upgraded 7-DOF Panda PBR café workstation with realistic sensor noise & DR
.venv/bin/craft serve-sim-host \
  --config examples/single_cup_coffee_config.json \
  --driver mujoco_cafe --realtime-pacing --sensor-noise --domain-randomization --port 8880
```

---

## 4. Honest Evidence Boundaries

Upgrading visual PBR materials, 7-DOF kinematics, Gaussian camera noise, and mass/friction domain randomization significantly narrows the sim-to-real pre-screening gap, **but does not replace physical-robot verification**:
- Simulation (`gazebo_cafe`, `mujoco_cafe`) remains tagged **`evidence_source="simulator"`**.
- Real camera frame replay remains tagged **`evidence_source="recorded_data"`**.
- Physical hardware qualification requires live execution on the target robot host (**`evidence_source="physical_robot"`**) with independent acceptance and operator safety review.
