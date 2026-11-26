"""End-to-end tests for the unified MCP-native TRAX Agent Engine on generic low-level SO-101."""
from __future__ import annotations

from pathlib import Path
import tempfile
import threading
import time

from trax.agent_engine import TraxAgentEngine
from trax.config import SceneConfig
from trax.mcp_protocol import MCPClient
from trax.mujoco_so101_driver import MuJoCoSO101HostDriver
from trax.sim_host_server import StandaloneSimHostApp, create_sim_host_server


def _wait_idle(eng: TraxAgentEngine, timeout_s: float = 30.0) -> dict:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        st = eng.get_state()
        if not st["busy"]:
            return st
        time.sleep(0.05)
    raise TimeoutError("TraxAgentEngine did not finish within timeout")


def test_unified_mcp_agent_explore_compile_governed_exec_and_repair_on_so101() -> None:
    cfg = SceneConfig.from_file(Path("examples/single_cup_coffee_config.json"))
    driver = MuJoCoSO101HostDriver(cfg, realtime_pacing=False)
    host_app = StandaloneSimHostApp(driver)
    server = create_sim_host_server(host_app, host="127.0.0.1", port=0)
    port = server.server_address[1]
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()

    probe_r1 = '''def run_probe(mcp):
    r0 = mcp.get_robot_state()
    sc = mcp.get_scene_objects()
    red = sc["objects"][0]
    pos = red["world_m"]
    refusal = mcp.move_ee(x=0.95, y=0.0, z=1.15)
    mv1 = mcp.move_ee(x=pos[0], y=pos[1], z=pos[2] + 0.10)
    mv2 = mcp.move_ee(x=pos[0], y=pos[1], z=pos[2] + 0.015)
    gr = mcp.set_gripper(open_ratio=0.0)
    lift = mcp.move_ee(x=pos[0], y=pos[1], z=pos[2] + 0.14)
    return {"refusal": refusal.get("status"), "grasped": gr.get("grasped_object_id"), "lift_ee": lift.get("ee_pos_m")}
'''

    probe_r2 = '''def run_probe(mcp):
    sc = mcp.get_scene_objects()
    pad = sc["zones"]["station_pad"]
    mv = mcp.move_ee(x=pad[0], y=pad[1], z=pad[2] + 0.14)
    hm = mcp.go_home()
    return {"pad_hover_ok": mv.get("ok"), "home_ok": hm.get("ok")}
'''

    # Iter 0 policy: places on station_pad and calls activate_workstation, but FORGETS to reconcile unknown ack!
    bad_policy = '''def run_policy(mcp, params):
    sc = mcp.get_scene_objects()
    red = [o for o in sc["objects"] if o["id"] == "red_cylinder"][0]
    pad = sc["zones"]["station_pad"]
    px, py, pz = red["world_m"]
    mcp.set_gripper(open_ratio=1.0)
    mcp.move_ee(x=px, y=py, z=pz + 0.12)
    mcp.move_ee(x=px, y=py, z=pz + 0.015)
    mcp.set_gripper(open_ratio=0.0)
    mcp.move_ee(x=px, y=py, z=pz + 0.14)
    mcp.move_ee(x=pad[0], y=pad[1], z=pad[2] + 0.14)
    mcp.move_ee(x=pad[0], y=pad[1], z=pad[2] + 0.05)
    mcp.set_gripper(open_ratio=1.0)
    mcp.go_home()
    mcp.activate_workstation(command_id="st-bad-1")
    return {"status": "completed"}
'''

    # Iter 1 repaired policy: reconciles unknown receipt via query_command_status!
    good_policy = '''def run_policy(mcp, params):
    sc = mcp.get_scene_objects()
    red = [o for o in sc["objects"] if o["id"] == "red_cylinder"][0]
    pad = sc["zones"]["station_pad"]
    px, py, pz = red["world_m"]
    mcp.set_gripper(open_ratio=1.0)
    mcp.move_ee(x=px, y=py, z=pz + 0.12)
    mcp.move_ee(x=px, y=py, z=pz + 0.015)
    mcp.set_gripper(open_ratio=0.0)
    mcp.move_ee(x=px, y=py, z=pz + 0.14)
    mcp.move_ee(x=pad[0], y=pad[1], z=pad[2] + 0.14)
    mcp.move_ee(x=pad[0], y=pad[1], z=pad[2] + 0.05)
    mcp.set_gripper(open_ratio=1.0)
    mcp.go_home()
    cid = "st-good-1"
    rec = mcp.activate_workstation(command_id=cid)
    if rec.get("receipt") == "unknown":
        rec = mcp.query_command_status(command_id=cid)
    return {"status": "completed", "receipt": rec}
'''

    calls = {"n": 0}

    def mock_llm(prompt: str) -> dict:
        calls["n"] += 1
        assert "DYNAMICALLY DISCOVERED MCP TOOLS" in prompt
        if calls["n"] == 1:
            assert "Exploration Round 1/2" in prompt
            return {"raw_output": f"```python\n{probe_r1}```", "source": probe_r1, "model": "mock-llm", "wall_s": 0.04}
        if calls["n"] == 2:
            assert "Exploration Round 2/2" in prompt
            return {"raw_output": f"```python\n{probe_r2}```", "source": probe_r2, "model": "mock-llm", "wall_s": 0.04}
        if calls["n"] == 3:
            assert "STAGE-1 EMPIRICAL EXPLORATION FINDINGS" in prompt
            return {"raw_output": f"```python\n{bad_policy}```", "source": bad_policy, "model": "mock-llm", "wall_s": 0.05}
        assert "unreconciled_unknown_ack" in prompt
        return {"raw_output": f"```python\n{good_policy}```", "source": good_policy, "model": "mock-llm", "wall_s": 0.05}

    try:
        mcp_url = f"http://127.0.0.1:{port}/mcp"
        with tempfile.TemporaryDirectory(prefix="trax-agent-mcp-") as tmp:
            eng = TraxAgentEngine(
                Path(tmp),
                lambda: (MCPClient(transport="http", url=mcp_url, timeout_s=15.0), mcp_url),
                llm_fn=mock_llm,
            )
            res = eng.start_closed_loop_async(
                {"task_id": "place_and_activate_station", "explore_rounds": 2, "max_iters": 3, "fresh": True}
            )
            assert res["status"] == "started"
            st = _wait_idle(eng)

            # Verify full real system invariants
            assert st["discovered_tools_count"] >= 10
            assert "move_ee" in st["discovered_tools"]
            assert "activate_workstation" in st["discovered_tools"]
            assert len(st["exploration_findings"]) == 2
            # Round 1 probe explicitly triggered and recorded backend hard safeguard limit_refusal!
            assert len(st["exploration_findings"][0]["limit_refusals"]) == 1
            assert st["exploration_findings"][0]["return_value"]["grasped"] == "red_cylinder"

            assert len(st["llm_calls"]) == 4  # 2 explore probes + 1 compile + 1 repair
            assert len(st["executed_runs"]) == 4  # 2 explore probes + 2 policy executions
            assert len(st["iterations"]) == 2

            it0, it1 = st["iterations"]
            assert it0["execution"]["task_passed"] is False
            assert it0["attribution"]["code"] == "unreconciled_unknown_ack"
            assert it1["execution"]["task_passed"] is True
            assert it1["execution"]["duplicate_non_idempotent"] is False
            assert it1["execution"]["queried_after_unknown"] is True
            assert it1["unified_diff"].startswith("---")
            assert (Path(tmp) / "trax_agent_journal.sqlite").is_file()
            assert st["phase"] == "verified"
    finally:
        server.shutdown()
        server.server_close()
        driver.close()
