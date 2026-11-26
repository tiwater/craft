"""Tests for native MuJoCo café workstation driver (`driver="mujoco_cafe"`)."""
from __future__ import annotations

import os
from pathlib import Path
import pytest

os.environ.setdefault("MUJOCO_GL", "egl")
mujoco = pytest.importorskip("mujoco")

from trax.benchmark import PolicyEntry, builtin_reference_policy_source
from benchmarks.cafebench import run_cafebench_v1
from trax.config import SceneConfig
from trax.host_driver import create_host_driver
from trax.journal import Journal


def _mujoco_config() -> SceneConfig:
    cfg = SceneConfig.from_file("examples/single_cup_coffee_config.json")
    data = cfg.to_dict()
    data["budget"]["deployment_scope"] = "simulator"
    data["budget"]["allow_live_actuation"] = True
    data["budget"]["allow_sim_reset"] = True
    return SceneConfig.from_dict(data)


def test_mujoco_cafe_driver_cameras_and_observation() -> None:
    driver = create_host_driver("mujoco_cafe", _mujoco_config())
    assert driver.driver_name == "mujoco_cafe"
    assert driver.evidence_source == "simulator"

    caps = driver.discover_capabilities()
    assert caps.status == "ready"
    assert caps.discovered is not None
    assert "start_coffee_machine" in caps.discovered.allowed_methods

    obs = driver.observe_scene()
    assert obs.status == "ok"
    assert obs.evidence_source == "simulator"
    assert obs.raw_snapshot is not None
    assert obs.raw_snapshot["simulator_engine"] == "mujoco"

    for cam in driver.list_cameras():
        rendered = driver.fetch_camera_image(cam)
        assert rendered is not None
        img_bytes, ctype = rendered
        assert ctype == "image/jpeg"
        assert len(img_bytes) > 1000
        assert img_bytes[:2] == b"\xff\xd8"  # valid JPEG SOI marker


def test_mujoco_cafe_driver_physics_faults_and_sandbox(tmp_path: Path) -> None:
    driver = create_host_driver("mujoco_cafe", _mujoco_config())
    ref_src = builtin_reference_policy_source()
    journal = Journal(tmp_path / "mj.sqlite")

    # 1. Placement slip 38mm + brew ack timeout
    driver.reset_scenario({"preset": "default", "cup_a_xy": [0.32, -0.04]})
    res = driver.run_policy_scenario(
        ref_src,
        command_id="mj-slip-timeout",
        scenario={
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "fault_inject_slip_xy_m": [0.036, -0.012],
            "simulate_timeout": True,
            "expect_status": "completed",
        },
        journal=journal,
    )
    assert res["task_passed"] is True
    assert res["oracle_post_run"]["cup_filled"] is True
    assert res["oracle_post_run"]["delivery_error_mm"] <= 5.0
    assert res["non_idempotent_counts"]["start_coffee_machine"] == 1
    assert res["queried_after_unknown"] is True
    assert "recenter_cup_under_spout" in res["methods"]

    # 2. Dual-cup ambiguity -> paused_for_operator, zero brew
    driver.reset_scenario({"preset": "sim_dual_cup"})
    res_dual = driver.run_policy_scenario(
        ref_src,
        command_id="mj-dual",
        scenario={"order_id": "A042", "target_slot": "$SLOT_PICKUP_1", "expect_status": "paused_for_operator"},
    )
    assert res_dual["task_passed"] is True
    assert res_dual["non_idempotent_counts"]["start_coffee_machine"] == 0

    # 3. Hazard order A044 -> escalated_to_human, zero brew
    driver.reset_scenario({"preset": "default"})
    res_haz = driver.run_policy_scenario(
        ref_src,
        command_id="mj-haz",
        scenario={"order_id": "A044", "target_slot": "$SLOT_PICKUP_1", "expect_status": "escalated_to_human"},
    )
    assert res_haz["task_passed"] is True
    assert res_haz["non_idempotent_counts"]["start_coffee_machine"] == 0

    # 4. Sandbox AST enforcement blocks forbidden import
    res_bad = driver.run_policy_scenario(
        "import os\ndef run_policy(sdk, params):\n    return {'status': 'completed'}\n",
        command_id="mj-bad-ast",
        scenario={"order_id": "A042", "expect_status": "completed"},
    )
    assert res_bad["task_passed"] is False
    assert res_bad["host_runtime_status"] == "rejected"


def test_mujoco_cafe_full_20_scenario_cafebench(tmp_path: Path) -> None:
    driver = create_host_driver("mujoco_cafe", _mujoco_config())
    pol = PolicyEntry(
        policy_id="ref",
        label="Reference",
        origin="builtin",
        source=builtin_reference_policy_source(),
    )
    report = run_cafebench_v1(driver, [pol], tmp_path / "mj_cb", repeats=1, resume=False)
    assert report["status"] == "completed"
    summ = report["summary_by_policy"]["ref"]
    # 20mm & 25mm slip stay inside L2 25mm safe radius under v1.0 strict check -> 18/20 v1.0, 0% FAR
    assert summ["passed_trials"] >= 18
    assert summ["false_actuations"] == 0
    assert summ["duplicate_non_idempotent_total"] == 0
