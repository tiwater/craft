"""Tests for TRAX Git-versioned Executable Knowledge Base, contract-drift invalidation, and rollback."""
from __future__ import annotations

from pathlib import Path
import tempfile

from trax.knowledge_base import AuditTraceStore, ExecutableKnowledgeBase


def test_executable_knowledge_base_git_branches_promotion_drift_and_rollback() -> None:
    with tempfile.TemporaryDirectory(prefix="trax-kb-") as tmp:
        ws = Path(tmp)
        kb = ExecutableKnowledgeBase(ws)
        traces = AuditTraceStore(ws)

        assert kb.current_branch() == "main"
        assert (ws / "knowledge" / ".git").is_dir()
        assert (ws / "knowledge" / "FACTS.md").is_file()
        assert (ws / "knowledge" / "FORBIDDEN.md").is_file()

        # Start isolated session branch off main
        br = kb.begin_session_branch("trax-mujoco-so101-sim", "pick_and_place_zone")
        assert br.startswith("session/trax-mujoco-so101-sim/pick_and_place_zone-")
        assert kb.current_branch() == br

        # Runtime-write objective physical facts + backend limit_refusal into FORBIDDEN.md
        sha_probe = kb.record_discovery_and_probe(
            embodiment_id="trax-mujoco-so101-sim",
            evidence_source="simulator",
            mcp_tools_sha256="abcdef1234567890fedcba",
            tools=[
                {"name": "move_ee", "annotations": {"idempotentHint": True}},
                {"name": "activate_workstation", "annotations": {"idempotentHint": False, "destructiveHint": True}},
            ],
            task_id="pick_and_place_zone",
            round_idx=1,
            probe_source="def run_probe(mcp):\n    return {'ok': True}\n",
            finding={
                "wall_s": 0.8,
                "methods_called": ["get_robot_state", "move_ee"],
                "limit_refusals": [{"tool": "move_ee", "violated_limit": "radial_reach=0.82m > 0.34m"}],
                "return_value": {"ok": True},
            },
            obs_snapshot={"get_robot_state": {"workspace_bounds_m": {"z_min_m": 1.025, "z_max_m": 1.28}}},
        )
        assert sha_probe is not None
        assert "activate_workstation" in kb.read_text_file("FACTS.md")
        assert "backend_limit_refusal" in kb.read_text_file("FORBIDDEN.md")

        # Record a failed iteration (v0) then a passing iteration (v1)
        bad_src = "def run_policy(mcp, params):\n    return {'status': 'failed'}\n"
        good_src = "def run_policy(mcp, params):\n    mcp.go_home()\n    return {'status': 'completed'}\n"

        sha_bad = kb.record_policy_iteration(
            embodiment_id="trax-mujoco-so101-sim",
            evidence_source="simulator",
            mcp_tools_sha256="abcdef1234567890fedcba",
            task_id="pick_and_place_zone",
            iter_index=0,
            origin="llm_compile",
            source=bad_src,
            attribution={"layer": "control", "code": "task_verification_failed", "summary": "zone_err=120mm"},
            passed=False,
        )
        assert sha_bad is not None
        assert "task_verification_failed" in kb.read_text_file("FORBIDDEN.md")

        kb.record_policy_iteration(
            embodiment_id="trax-mujoco-so101-sim",
            evidence_source="simulator",
            mcp_tools_sha256="abcdef1234567890fedcba",
            task_id="pick_and_place_zone",
            iter_index=1,
            origin="llm_repair",
            source=good_src,
            attribution={"layer": "none", "code": "passed", "summary": "zone_err=1.3mm"},
            passed=True,
        )

        # Verified promotion merges --no-ff into main and creates immutable tag + SKILL.md + skill.py
        promo = kb.promote_verified_skill_to_main(
            embodiment_id="trax-mujoco-so101-sim",
            evidence_source="simulator",
            mcp_tools_sha256="abcdef1234567890fedcba",
            task_id="pick_and_place_zone",
            task_instruction="Pick red cylinder and place on target_zone_a.",
            policy_source=good_src,
            verify_result={"passed": True, "detail": "zone_err=1.3mm", "metrics": {"zone_error_mm": 1.3}},
        )
        assert promo["promoted"] is True
        assert kb.current_branch() == "main"
        assert promo["tag"].startswith("verified/trax-mujoco-so101-sim/pick_and_place_zone-")

        # Active skill under matching contract + simulator evidence
        sk_active = kb.list_skills(
            current_embodiment="trax-mujoco-so101-sim",
            current_tools_sha="abcdef1234567890fedcba",
            current_evidence="simulator",
        )
        assert len(sk_active) == 1 and sk_active[0]["status"] == "active"

        # Automatic sim-to-real evidence downgrade on physical_robot
        sk_phys = kb.list_skills(
            current_embodiment="trax-mujoco-so101-sim",
            current_tools_sha="abcdef1234567890fedcba",
            current_evidence="physical_robot",
        )
        assert sk_phys[0]["status"] == "advisory_sim_prior"

        # Automatic contract-drift quarantine when MCP tools/list SHA changes
        sk_drift = kb.list_skills(
            current_embodiment="trax-mujoco-so101-sim",
            current_tools_sha="9999999999999999",
            current_evidence="simulator",
        )
        assert sk_drift[0]["status"] == "stale_contract"

        # Git diff & Rollback to prior commit work cleanly
        diff_txt = kb.git_diff(promo["commit"])
        assert "SKILL.md" in diff_txt or "merge(" in diff_txt
        rb = kb.rollback_policy_from_commit(sha_bad, "pick_and_place_zone")
        assert rb["source"].strip() == bad_src.strip()

        # Audit trace store logs cleanly
        traces.append_llm_call({"call_id": 1, "model": "mock"})
        traces.append_mcp_wire({"method": "tools/list", "ok": True})
        traces.save_recording("rec-1", {"passed": True})
        st = traces.stats()
        assert st["llm_calls_logged"] == 1
        assert st["mcp_wire_logged"] == 1
        assert st["recordings_count"] == 1
