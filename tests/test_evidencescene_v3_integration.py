"""End-to-end tests for TRAX <-> EvidenceScene v3.0 SceneStore integration on LeRobot SO-101."""
from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path

import pytest

os.environ.setdefault("MUJOCO_GL", "egl")

from trax.config import SceneConfig
from trax.knowledge_base import ExecutableKnowledgeBase
from trax.mcp_protocol import MCPClient, build_driver_mcp_server
from trax.spatial_contract import SpatialRequirementSpec, parse_spatial_snapshot_meta

HAS_MUJOCO = importlib.util.find_spec("mujoco") is not None
HAS_ES_V3 = importlib.util.find_spec("evidencescene.spatial_state") is not None


@pytest.mark.skipif(not (HAS_MUJOCO and HAS_ES_V3), reason="Requires optional mujoco and evidencescene v3 spatial_state")
def test_so101_unified_scene_store_task_observability_revalidation_and_active_perception() -> None:
    from trax.mujoco_so101_driver import MuJoCoSO101HostDriver

    cfg = SceneConfig.from_file(Path("examples/single_cup_coffee_config.json"))
    drv = MuJoCoSO101HostDriver(cfg)
    try:
        # 1. One Spatial State, Two Interfaces: reconstruction, snapshot, and MCP scene objects share exact cursor
        recon = drv.reconstruct_scene_evidencescene()
        snap = drv.get_spatial_snapshot_dict()
        objs = drv.low_level_get_scene_objects()

        assert recon["cameras_used"] == ["front_top", "wrist"]
        assert recon["cursor"] == snap["cursor"] == objs["cursor"]
        assert snap["clock_id"] == "mujoco-time"
        assert "fixed_tabletop" in snap["entities"]
        assert "red_cylinder" in snap["entities"]

        # Verify pure-stdlib TRAX consumer parser parses EvidenceScene v3 snapshot natively
        meta = parse_spatial_snapshot_meta(snap)
        assert meta.schema_version == 1
        assert meta.cursor is not None
        assert meta.cursor.epoch == snap["cursor"]["epoch"]
        assert meta.cursor.revision == snap["cursor"]["revision"]
        assert meta.observation_time is not None
        assert meta.observation_time.clock_id == "mujoco-time"

        # 2. Task-conditioned observability: symmetric cylinder body_grasp satisfied (rank=5), handle_grasp unsatisfied!
        body = drv.low_level_assess_spatial_requirements("red_cylinder", "body_grasp")
        assert body["status"] == "satisfied"
        assert body["observability"]["rank"] == 5
        assert all(st == "observable" for st in body["observability"]["status_by_row"])

        handle = drv.low_level_assess_spatial_requirements("red_cylinder", "handle_grasp")
        assert handle["status"] == "unsatisfied"
        assert "unobservable" in handle["observability"]["status_by_row"]

        # 3. Active perception forecast (H + B^T B) recommends a reachable wrist viewpoint to satisfy handle_grasp
        ap = drv.low_level_evaluate_wrist_viewpoints(handle["assessment_id"])
        assert ap["selected_candidate"] is not None
        assert ap["recommended_ee_xyz_m"] is not None
        assert ap["reason"] == "least_motion_wrist_viewpoint_meeting_requirements"
        assert any(c["predicted_to_meet_requirements"] for c in ap["candidates"])

        # 4. Human-in-the-loop surface target selection + revision-aware revalidation
        sel1 = drv.low_level_select_surface_target(
            entity_id="fixed_tabletop",
            target_xyz_m=[0.34, -0.09, 1.00],
            reviewer_id="operator_test",
            reason="pick_left_zone",
        )
        assert sel1["ok"] is True
        assert sel1["cursor"]["revision"] > snap["cursor"]["revision"]

        plan1 = drv.low_level_assess_spatial_requirements(task_mode="selected_point")
        assert plan1["status"] == "satisfied"
        reval_ok = drv.low_level_revalidate_spatial_requirements(plan1["assessment_id"])
        assert reval_ok["decision"] == "target_available"

        # Operator moves target -> stale plan MUST invalidate with surface_selection_changed
        sel2 = drv.low_level_select_surface_target(
            entity_id="fixed_tabletop",
            target_xyz_m=[0.34, 0.09, 1.00],
            reviewer_id="operator_test",
            reason="pick_right_zone",
        )
        assert sel2["ok"] is True
        reval_stale = drv.low_level_revalidate_spatial_requirements(plan1["assessment_id"])
        assert reval_stale["decision"] == "invalidate_plan"
        assert reval_stale["reason"] == "surface_selection_changed"

        # 5. Reset increments epoch and invalidates old assessments (resnapshot_required)
        prev_epoch = sel2["cursor"]["epoch"]
        drv.reset_scenario({"preset": "default"})
        snap_after = drv.get_spatial_snapshot_dict()
        assert snap_after["cursor"]["epoch"] == prev_epoch + 1
        reval_reset = drv.low_level_revalidate_spatial_requirements(plan1["assessment_id"])
        assert reval_reset["decision"] == "invalidate_plan"
        assert reval_reset["resnapshot_required"] is True

        # 6. MCP server exposes trax://scene/snapshot and all 4 EvidenceScene v3 tools over JSON-RPC
        server = build_driver_mcp_server(drv)
        client = MCPClient(transport="local", local_server=server)
        _, tools_resp, _ = client.rpc("tools/list", {})
        tool_names = {t["name"] for t in tools_resp["result"]["tools"]}
        assert {
            "assess_spatial_requirements",
            "revalidate_spatial_requirements",
            "evaluate_wrist_viewpoints",
            "select_surface_target",
        }.issubset(tool_names)

        _, res_resp, _ = client.rpc("resources/read", {"uri": "trax://scene/snapshot"})
        snap_via_mcp = json.loads(res_resp["result"]["contents"][0]["text"])
        assert snap_via_mcp["schema_version"] == 1
        assert snap_via_mcp["cursor"] == snap_after["cursor"]
    finally:
        drv.close()


def test_knowledge_base_persists_declarative_spatial_requirements(tmp_path: Path) -> None:
    kb = ExecutableKnowledgeBase(tmp_path)
    req = SpatialRequirementSpec(entity_id="red_cylinder", task_mode="body_grasp", max_age_ms=1500.0, max_stddev_m=0.008)
    res = kb.promote_verified_skill_to_main(
        embodiment_id="so101-test",
        evidence_source="simulator",
        mcp_tools_sha256="abcdef1234567890",
        task_id="pick_red",
        task_instruction="Pick the red cylinder",
        policy_source="def run_policy(mcp, params):\n    return {'status': 'done'}\n",
        verify_result={"passed": True, "detail": "ok", "metrics": {}},
        verified_seeds=(0, 1, 2),
        min_verified_seeds=3,
        spatial_requirements=(req.to_dict(),),
    )
    assert res["promoted"] is True
    meta_path = tmp_path / "knowledge" / "skills" / "so101-test" / "pick_red" / "meta.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    assert meta["spatial_requirements"] == [req.to_dict()]
    skill_md = (meta_path.parent / "SKILL.md").read_text(encoding="utf-8")
    assert "spatial_requirements:" in skill_md
