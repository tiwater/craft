"""Tests for high-fidelity 7-DOF Panda PBR MuJoCo upgrades & Gazebo Fortress OGRE2/ROS2 driver."""
from __future__ import annotations

import os
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest

from trax.benchmark import builtin_reference_policy_source
from trax.config import SceneConfig
from trax.gazebo_cafe_driver import (
    GazeboCafeHostDriver,
    export_gazebo_cafe_bundle,
    probe_gazebo_runtime,
)
from trax.host_driver import create_host_driver, list_host_drivers

os.environ.setdefault("MUJOCO_GL", "egl")


def _cfg() -> SceneConfig:
    return SceneConfig.from_file(Path(__file__).resolve().parents[1] / "examples" / "single_cup_coffee_config.json")


def test_gazebo_fortress_bundle_export_validates_sdf_pbr_noise_and_portability(tmp_path: Path) -> None:
    out_dir = tmp_path / "gz_bundle"
    manifest = export_gazebo_cafe_bundle(out_dir, copy_local_assets=True)

    assert manifest["simulator_engine"] == "gazebo_fortress"
    assert manifest["render_engine"] == "ogre2"
    assert manifest["physics_engine"] == "dart_500hz"
    assert manifest["cameras"] == ["overview", "overhead", "spout", "wrist"]

    world_path = out_dir / "worlds" / "craft_cafe_fortress.sdf"
    cafe_path = out_dir / "models" / "cafe_workstation" / "model.sdf"
    panda_path = out_dir / "models" / "panda_cafe" / "model.sdf"
    bridge_path = out_dir / "config" / "ros_gz_bridge.yaml"

    for p in (world_path, cafe_path, panda_path, bridge_path, out_dir / "setup_gazebo_fortress.sh", out_dir / "launch_cafe_gazebo.sh"):
        assert p.is_file(), f"Missing exported artifact: {p}"

    # Validate strict XML well-formedness & key OGRE2 PBR / DART / Gaussian noise / 7-DOF Panda elements
    world_txt = world_path.read_text(encoding="utf-8")
    cafe_txt = cafe_path.read_text(encoding="utf-8")
    panda_txt = panda_path.read_text(encoding="utf-8")
    bridge_txt = bridge_path.read_text(encoding="utf-8")

    ET.fromstring(world_txt)
    ET.fromstring(cafe_txt)
    ET.fromstring(panda_txt)

    assert "<render_engine>ogre2</render_engine>" in world_txt
    assert 'type="dart"' in world_txt
    assert "<pbr><metal>" in cafe_txt
    assert "<type>gaussian</type>" in cafe_txt
    assert "<type>gaussian</type>" in panda_txt
    for j in range(1, 8):
        assert f"panda_joint{j}" in panda_txt
        assert f"/panda/joint{j}/cmd_pos" in bridge_txt
    for cam in ("overview", "overhead", "spout", "wrist"):
        assert f"/craft/camera/{cam}/image_raw" in bridge_txt

    # Zero hard-coded private machine paths in exported SDF / YAML / shell scripts
    for txt in (world_txt, cafe_txt, panda_txt, bridge_txt):
        assert "/home/" not in txt
        assert "/Users/" not in txt


def test_gazebo_cafe_driver_registered_and_honest_fail_closed(tmp_path: Path) -> None:
    assert "gazebo_cafe" in list_host_drivers()
    drv = create_host_driver("gazebo_cafe", _cfg())
    assert isinstance(drv, GazeboCafeHostDriver)
    assert drv.driver_name == "gazebo_cafe"
    assert drv.evidence_source == "simulator"

    probe = probe_gazebo_runtime()
    cap = drv.discover_capabilities()
    obs = drv.observe_scene()
    if not probe["installed"]:
        assert cap.status == "disconnected"
        assert obs.status == "disconnected"
        assert drv.fetch_camera_image("overview") is None
        res = drv.run_policy_scenario(
            builtin_reference_policy_source(),
            command_id="gz-offline-test",
            scenario={"order_id": "A042", "expect_status": "completed"},
        )
        assert res["task_passed"] is False
        assert res["host_runtime_status"] == "disconnected"


def test_mujoco_high_fidelity_panda_sensor_noise_and_domain_randomization() -> None:
    pytest.importorskip("mujoco")
    from trax.mujoco_cafe_driver import MuJoCoCafeHostDriver

    drv = MuJoCoCafeHostDriver(_cfg(), high_fidelity=True)
    try:
        # Clean nominal observation
        drv.reset_scenario({"preset": "default", "sensor_noise": False, "domain_randomization": False})
        obs_clean = drv.observe_scene()
        assert obs_clean.status == "ok"
        fid = obs_clean.raw_snapshot["fidelity"]
        assert fid["sensor_noise"] is False
        assert fid["domain_randomization"] is False
        img_clean, mime = drv.fetch_camera_image("overview")
        assert mime == "image/jpeg" and len(img_clean) > 5000

        # Noisy + Domain-Randomized observation & execution
        drv.reset_scenario(
            {
                "preset": "default",
                "sensor_noise": True,
                "domain_randomization": True,
                "noise_seed": 123,
            }
        )
        obs_noisy = drv.observe_scene()
        assert obs_noisy.status == "ok"
        assert obs_noisy.raw_snapshot["fidelity"]["sensor_noise"] is True
        assert obs_noisy.raw_snapshot["fidelity"]["domain_randomization"] is True
        img_noisy, _ = drv.fetch_camera_image("overview")
        assert img_noisy != img_clean

        # Reference TRAX policy still completes cleanly under realistic sensor noise + DR
        res = drv.run_policy_scenario(
            builtin_reference_policy_source(),
            command_id="hf-panda-noisy-dr-1",
            scenario={
                "order_id": "A042",
                "target_slot": "$SLOT_PICKUP_1",
                "sensor_noise": True,
                "domain_randomization": True,
                "noise_seed": 123,
                "expect_status": "completed",
            },
        )
        assert res["task_passed"] is True
        assert res["duplicate_non_idempotent"] is False
        assert res["oracle_post_run"]["delivery_error_mm"] <= 15.0
    finally:
        drv.close()
