"""Unit & real-MuJoCo integration tests for Phase 3B Robosuite parity benchmark."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any
import pytest

os.environ.setdefault("MUJOCO_GL", "egl")
pytest.importorskip("robosuite")

from craft.robosuite_bench import (
    attribute_robosuite_failure,
    check_robosuite_ast_compliance,
    run_robosuite_parity_benchmark,
    run_robosuite_rollout,
)


WORKING_LIFT_POLICY = """
def run_policy(sdk):
    st = sdk.get_state()
    c = st["objects"]["cube"]["pos"]
    sdk.set_gripper("open", steps=10)
    sdk.move_to([c[0], c[1], c[2] + 0.12], gripper="open", max_steps=60)
    sdk.move_to([c[0], c[1], c[2]], gripper="open", max_steps=60)
    sdk.set_gripper("close", steps=20)
    sdk.move_to([c[0], c[1], c[2] + 0.18], gripper="close", max_steps=60)
    return {"status": "done"}
"""

BROKEN_AST_POLICY = """
import os
def run_policy(sdk):
    return {"status": "done"}
"""


def test_robosuite_ast_and_attribution() -> None:
    assert check_robosuite_ast_compliance(WORKING_LIFT_POLICY)["compliant"] is True
    assert check_robosuite_ast_compliance(BROKEN_AST_POLICY)["compliant"] is False

    a1 = attribute_robosuite_failure(
        {"success": False, "min_eef_to_obj_m": 0.02, "obj_lift_delta_m": 0.005, "ever_closed_gripper": True, "closed_gripper_width_max": 0.002},
        "NutAssemblySquare",
    )
    assert a1["layer"] == "control"
    assert a1["code"] == "grasp_missed_or_slip"

    a2 = attribute_robosuite_failure(
        {"success": False, "min_eef_to_obj_m": 0.03, "max_handle_qpos": 0.02, "max_hinge_qpos": 0.0},
        "Door",
    )
    assert a2["code"] == "latch_not_unlatched"


def test_robosuite_real_mujoco_lift_rollout_succeeds() -> None:
    rec = run_robosuite_rollout("Lift", seed=101, source=WORKING_LIFT_POLICY, horizon=260)
    assert rec["program_ok"] is True
    assert rec["success"] is True
    assert rec["obj_lift_delta_m"] >= 0.06


def test_robosuite_parity_benchmark_end_to_end(tmp_path: Path) -> None:
    def mock_llm(prompt: str) -> dict[str, Any]:
        return {
            "source": WORKING_LIFT_POLICY,
            "raw_output": WORKING_LIFT_POLICY,
            "model": "mock",
            "reasoning_effort": "medium",
            "wall_s": 0.01,
        }

    rep = run_robosuite_parity_benchmark(
        llm_fn=mock_llm,
        output_dir=tmp_path / "rb_out",
        tasks=("Lift",),
        dev_seeds=(101,),
        eval_seeds=(1, 2),
        model="mock",
        max_dev_llm_calls=2,
        enable_capx_online_regen=False,
        horizon=260,
    )
    assert (tmp_path / "rb_out" / "robosuite_prior_work_report.json").is_file()
    assert (tmp_path / "rb_out" / "robosuite_prior_work_report.html").is_file()
    assert rep["summary_by_method"]["craft_full"]["passed"] == 2
    assert rep["summary_by_method"]["craft_full"]["eval_llm_calls"] == 0
