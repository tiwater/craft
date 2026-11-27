"""Tests for the LeRobot SO-101 (SO-ARM100) MuJoCo SceneHostDriver."""
from __future__ import annotations

import importlib.util
from pathlib import Path
import tempfile

import pytest

from trax.config import SceneConfig
from trax.host_driver import create_host_driver, list_host_drivers
from trax.journal import Journal

HAS_MUJOCO = importlib.util.find_spec("mujoco") is not None


def _load_cfg() -> SceneConfig:
    return SceneConfig.from_file(Path("examples/single_cup_coffee_config.json"))


def test_mujoco_so101_driver_registered() -> None:
    assert "mujoco_so101" in list_host_drivers()


@pytest.mark.skipif(not HAS_MUJOCO, reason="mujoco optional dependency not installed")
def test_mujoco_so101_capabilities_ik_and_cameras() -> None:
    from trax.mujoco_so101_driver import MuJoCoSO101HostDriver

    driver = create_host_driver("mujoco_so101", _load_cfg())
    assert isinstance(driver, MuJoCoSO101HostDriver)
    try:
        cap = driver.discover_capabilities()
        assert cap.status == "ready"
        assert cap.discovered is not None
        assert driver.evidence_source == "simulator"
        assert cap.discovered.execution_authority == "trax-mujoco-so101-sim"
        assert cap.discovered.device_bundle_id == "trax-lerobot-so101-workstation"
        assert driver._fidelity_meta["robot_model"] == "lerobot_so101_6dof"

        # Workspace clamping enforces SO-101 reach safety envelope
        clamped = driver.clamp_world_target(2.5, 2.5, 0.2)
        assert clamped[2] >= 1.025
        assert clamped[0] < 0.60

        # 5-DOF IK reaches tabletop spout and pickup slots accurately
        for wx, wy, wz in (
            (0.33, -0.02, 1.045),
            (0.45, 0.00, 1.050),
            (0.38, -0.17, 1.050),
            (0.38, 0.17, 1.050),
        ):
            q = driver._solve_so101_ik(wx, wy, wz)
            assert len(q) == 5
            for qi, qv in zip(driver._so101_qpos_idx, q):
                driver._ik_data.qpos[qi] = qv
            driver._mj.mj_kinematics(driver._model, driver._ik_data)
            tcp = driver._ik_data.site_xpos[driver._ee_site_id]
            err_m = float(((tcp[0] - wx) ** 2 + (tcp[1] - wy) ** 2 + (tcp[2] - wz) ** 2) ** 0.5)
            assert err_m < 0.006, f"SO-101 IK error too large ({err_m:.4f}m) for {(wx, wy, wz)}"

        # Realistic LeRobot 2-camera rig: global front_top + eye-in-hand wrist
        cams = driver.list_cameras()
        assert cams == ("front_top", "wrist")
        for cam in cams:
            rendered = driver.fetch_camera_image(cam)
            assert rendered is not None
            raw_bytes, mime = rendered
            assert len(raw_bytes) > 1000
            assert mime in ("image/jpeg", "image/png")

        # EvidenceScene reconstruction fuses the exact 2-camera rig and exposes calibrated poses
        recon = driver.reconstruct_scene_evidencescene()
        assert recon["cameras_used"] == ["front_top", "wrist"]
        assert len(recon["cameras"]) == 2
        assert {c["mount"] for c in recon["cameras"]} == {"fixed_world", "eye_in_hand"}
        assert recon["total_surfels"] > 100

        # URDF mesh URI rewriting handles /assets/robot/ -> api/robot/assets/ (subpath-safe browser STL loading)
        from trax.plugin import rewrite_urdf_mesh_uris
        raw_urdf = (driver._so101_dir / "robot.urdf").read_text(encoding="utf-8")
        mcp_urdf = rewrite_urdf_mesh_uris(raw_urdf, asset_url_prefix="/assets/robot")
        browser_urdf = rewrite_urdf_mesh_uris(mcp_urdf, asset_url_prefix="api/robot/assets")
        assert 'filename="api/robot/assets/assets/base_so101_v2.stl"' in browser_urdf
        assert 'filename="/assets/robot/' not in browser_urdf

        # URDF forward kinematics matches live MuJoCo link positions (< 2mm, no gimbal-lock drift)
        import math, xml.etree.ElementTree as ET
        import numpy as np
        from trax.mujoco_so101_driver import SO101_BASE_XYZ

        def _rpy_to_R(r: float, p: float, y: float) -> np.ndarray:
            cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
            return np.array([
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ], dtype=float)

        st = driver.low_level_get_robot_state()
        qmap = dict(zip(st["joint_names"], st["joint_pos_rad"]))
        qmap["gripper"] = 0.10 + st["gripper_open_ratio"] * (0.88 - 0.10)
        root = ET.fromstring(raw_urdf)
        tf: dict[str, np.ndarray] = {"base": np.eye(4)}
        for j in root.findall("joint"):
            par, ch = j.find("parent").attrib["link"], j.find("child").attrib["link"]
            orig = j.find("origin")
            xyz = [float(v) for v in orig.attrib.get("xyz", "0 0 0").split()]
            rpy = [float(v) for v in orig.attrib.get("rpy", "0 0 0").split()]
            m = np.eye(4)
            m[:3, :3] = _rpy_to_R(*rpy)
            m[:3, 3] = xyz
            cur = tf[par] @ m
            if j.attrib.get("type") == "revolute":
                ax = np.array([float(v) for v in j.find("axis").attrib["xyz"].split()], dtype=float)
                ax /= np.linalg.norm(ax)
                ang = float(qmap.get(j.attrib["name"], 0.0))
                c, s = math.cos(ang), math.sin(ang)
                t = 1.0 - c
                x, y, z = ax
                rot = np.array([
                    [t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0],
                    [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0],
                    [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0],
                    [0, 0, 0, 1],
                ], dtype=float)
                cur = cur @ rot
            tf[ch] = cur
            bid = driver._mj.mj_name2id(driver._model, driver._mj.mjtObj.mjOBJ_BODY, ch)
            mj_local = driver._data.xpos[bid] - np.array(SO101_BASE_XYZ)
            err_m = float(np.linalg.norm(cur[:3, 3] - mj_local))
            assert err_m < 0.0025, f"URDF FK mismatch on link {ch}: {err_m*1000:.2f}mm"
    finally:
        driver.close()


@pytest.mark.skipif(not HAS_MUJOCO, reason="mujoco optional dependency not installed")
def test_mujoco_so101_policy_execution_and_lost_receipt_reconciliation() -> None:
    policy_path = Path(
        "experiments/coffee-closed-loop-20261120/cafebench-v1-prior-work/policies/craft_full.py"
    )
    policy_src = policy_path.read_text(encoding="utf-8")
    driver = create_host_driver("mujoco_so101", _load_cfg())
    try:
        with tempfile.TemporaryDirectory(prefix="so101-journal-") as tmp:
            with Journal(Path(tmp) / "journal.sqlite") as journal:
                res = driver.run_policy_scenario(
                    policy_src,
                    command_id="so101-fault-001",
                    scenario={
                        "preset": "default",
                        "fault_inject_slip_xy_m": [0.038, -0.012],
                        "simulate_timeout": True,
                    },
                    journal=journal,
                )
        assert res["evidence_source"] == "simulator"
        assert res["task_passed"] is True
        assert res["duplicate_non_idempotent"] == 0
        assert res["unknown_events_observed"] >= 1
        assert res["queried_after_unknown"] is True
        assert res["oracle_post_run"]["cup_filled"] is True
        assert res["oracle_post_run"]["delivery_error_mm"] <= 15.0
    finally:
        driver.close()
