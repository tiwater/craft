# SO-101 (LeRobot / SO-ARM100) Host Capability Contract

- **Date:** 2026-11-25
- **Scope:** First cross-embodiment low-cost manipulator integration in MuJoCo simulation (`driver="mujoco_so101"`, `evidence_source="simulator"`), using the official Apache-2.0 `TheRobotStudio/SO-ARM100/Simulation/SO101` model (`so101_new_calib_camera.xml` + STL meshes under `sim/so101/`).
- **Evidence Boundary:** All runs under `driver="mujoco_so101"` carry explicit `evidence_source="simulator"`. Passing MuJoCo SO-101 tests is **not** physical-robot safety approval or hardware qualification for a physical LeRobot SO-101 arm.

---

## 1. Three-Owner Separation

| Authority | Component | Owns | Explicitly Forbidden |
|---|---|---|---|
| **Perception Owner** | `MuJoCo-SO101-EvidenceScene` (sim) / future RGB-D + wrist camera EvidenceScene (real) | Timestamped object poses, cup/item class, tilt angle, confidence, freshness, ambiguity flags | Policy control flow, appliance dispatch, fabricating missing objects |
| **Policy Governance Owner** | TRAX pure-stdlib core (`src/trax/policy.py`, `journal.py`, `runtime.py`, `repair_screener.py`, `probing.py`) | Frozen SHA-256 AST execution, intent-before-dispatch SQLite journal, query-first crash reconciliation, 4-layer attribution, offline screening | Direct servo writes, hardcoding SO-101 joint angles, fabricating scene state |
| **Host Actuation Owner** | `MuJoCoSO101HostDriver` (`src/trax/mujoco_so101_driver.py`, isolated optional driver) | 5-DOF damped-least-squares IK, STS3215 joint limits, workspace radius/height safety clamps, gripper jaw actuation, host L2 pouring interlock, non-idempotent appliance command ledger (`query_command_status`) | Bypassing TRAX acceptance gates, silent retries on `unknown` receipts |

---

## 2. Kinematic & Servo Limits (Official `so101_new_calib_camera.xml`)

| Joint | Type | Range (rad) | Force Limit (N·m) | Role |
|---|---|---|---|---|
| `shoulder_pan` | Hinge (`Z`) | `[-1.920, +1.920]` | `[-3.35, +3.35]` | Base yaw |
| `shoulder_lift` | Hinge (`Z`) | `[-1.745, +1.745]` | `[-3.35, +3.35]` | Shoulder elevation |
| `elbow_flex` | Hinge (`Z`) | `[-1.690, +1.690]` | `[-3.35, +3.35]` | Elbow pitch |
| `wrist_flex` | Hinge (`Z`) | `[-1.658, +1.658]` | `[-3.35, +3.35]` | End-effector pitch |
| `wrist_roll` | Hinge (`Z`) | `[-2.744, +2.841]` | `[-3.35, +3.35]` | Gripper roll |
| `gripper` | Hinge (`Z`) | `[-0.175, +1.745]` | `[-3.35, +3.35]` | Moving jaw (`~0.85` open / `~0.08` grasp) |

- **TCP Site:** `gripperframe` (defined in upstream `so101_new_calib_camera.xml`, `pos="-0.0079 -0.0002 -0.0981"` in `gripper` frame).
- **Safe Tabletop Workspace Envelope (relative to SO-101 base at `(0.0, 0.0, 0.78)`):**
  - Radial reach: `r = hypot(x, y) in [0.10, 0.34] m`
  - Height above tabletop: `z - TABLE_TOP_Z in [0.025, 0.30] m`
  - Optional Sim-to-Real disturbance knobs (for ActiveProbe & calibration testing): `--sensor-noise`, `--domain-randomization` (servo backlash / placement drift).

---

## 3. Exposed Host SDK Contract (`EdgeZeroGtCoffeeSDK`-Compatible Tabletop Cell)

To keep TRAX core, TRAX Engine (`/v1`), TRAX UI (`trax ui`), `CafeBench v2`, and `Sim2Sim` 100% embodiment-agnostic, `MuJoCoSO101HostDriver` exposes the exact same 13-method public SDK contract and 4-camera observer surface (`overview`, `overhead`, `spout`, `wrist`) as `MuJoCoCafeHostDriver`, scaled faithfully to the SO-101 desktop coffee/appliance cell:

- **Idempotent / Observation Methods:** `get_order`, `observe_scene`, `query_command_status`, `request_operator_confirmation`, `retract_safe_pose`, `escalate_to_human_operator`, `move_to_target`, `place_cup_under_machine`, `recenter_cup_under_spout`, `retrieve_and_deliver_cup`, `complete_order_delivery`.
- **Non-Idempotent Methods (Journal-Protected):** `start_coffee_machine` (enforces host-owned L2 radial offset `<= 0.020 m` and tilt `<= 12 deg` interlock; returns `unknown` on injected ack timeout and requires `query_command_status` reconciliation), `wait_for_coffee_ready`.
- **Authority Tag:** `execution_authority="trax-mujoco-so101-sim"`, `sdk_version="so101-tabletop-1.0"`, `evidence_source="simulator"`.

---

## 4. Verification Requirements

Before any physical SO-101 (`LeRobotHostDriver`) promotion:
1. Automated MuJoCo tests (`tests/test_mujoco_so101_driver.py`) verifying MJCF load, 5-DOF IK tabletop accuracy, joint-limit clamping, 4-camera rendering, L2 interlock enforcement, and lost-receipt crash reconciliation (`0` duplicate brews).
2. `make check-stdlib` confirms `src/trax/` core remains 100% pure Python stdlib (with `mujoco_so101_driver.py` isolated as an optional simulator module).
3. Physical hardware qualification will separately require a real `LeRobotHostDriver` with physical E-stop, current/torque limits, and live perception before any `evidence_source="physical_robot"` claim.
