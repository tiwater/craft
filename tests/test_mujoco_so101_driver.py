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
