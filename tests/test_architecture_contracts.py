"""Production architecture & contract tests for TRAX plugin ABI, persistent stdio MCP, URDF, abort, and KB gates."""
from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import threading
import urllib.error
import urllib.request

import pytest

from trax.agent_engine import TraxAgentEngine
from trax.config import SceneConfig
from trax.knowledge_base import ExecutableKnowledgeBase
from trax.mcp_protocol import MCPClient
from trax.plugin import (
    list_backend_plugins,
    read_safe_robot_asset,
    register_backend_plugin,
    resolve_backend_factory,
    rewrite_urdf_mesh_uris,
)
from trax.sandbox import CancellationToken, ExecutionAbortedError, GovernedMCPProxy
from trax.sim_host_server import StandaloneSimHostApp, create_sim_host_server


def test_unknown_robot_persistent_stdio_mcp_end_to_end() -> None:
    """Any unknown external robot plugs into TRAX over persistent stdio MCP with zero core changes."""
    script = Path("examples/custom_mcp_robot/unknown_arm_mcp.py").resolve()
    assert script.is_file()

    with tempfile.TemporaryDirectory(prefix="trax-unknown-robot-") as tmp:
        ws = Path(tmp)
        client = MCPClient(transport="stdio", command=[sys.executable, str(script)], timeout_s=10.0)
        try:
            # 1. Verify persistent state across multiple RPCs inside the SAME subprocess
            _, init_resp, _ = client.rpc("initialize", {"protocolVersion": "2025-03-26"})
            info = init_resp["result"]["serverInfo"]
            assert info["embodiment_id"] == "custom-xy-gantry-v1"

            _, m_resp, _ = client.rpc("tools/call", {"name": "move_gantry_xy", "arguments": {"x_mm": 50.0, "y_mm": 25.0}})
            assert m_resp["result"]["structuredContent"]["ok"] is True

            _, r_resp, _ = client.rpc("tools/call", {"name": "read_gantry_pose", "arguments": {}})
            # If stdio spawned a fresh process per RPC, pose would reset to (0, 0)!
            assert r_resp["result"]["structuredContent"]["x_mm"] == 50.0
            assert r_resp["result"]["structuredContent"]["y_mm"] == 25.0

            # 2. Run full TraxAgentEngine closed loop against the unknown robot
            responses = iter(
                [
                    {
                        "source": (
                            "def run_probe(mcp):\n"
                            "    st = mcp.read_gantry_pose()\n"
                            "    ref = mcp.move_gantry_xy(x_mm=500.0, y_mm=0.0)\n"
                            "    return {'pose': st, 'refusal': ref}\n"
                        ),
                        "model": "mock-llm",
                        "wall_s": 0.01,
                    },
                    {
                        "source": (
                            "def run_policy(mcp, params):\n"
                            "    mcp.move_gantry_xy(x_mm=120.0, y_mm=80.0)\n"
                            "    mcp.fire_dispense_valve(command_id='gantry-cmd-1')\n"
                            "    return {'status': 'completed'}\n"
                        ),
                        "model": "mock-llm",
                        "wall_s": 0.01,
                    },
                ]
            )
            engine = TraxAgentEngine(
                ws,
                lambda: (client, "stdio:custom-xy-gantry-v1"),
                llm_fn=lambda prompt: next(responses),
            )
            findings = engine._run_explore_sync(rounds=1, task_id="custom_dispense", seed=0)
            assert len(findings) == 1
            assert len(findings[0]["limit_refusals"]) == 1

            engine._synthesize_sync(force_fresh=True)
            it = engine._execute_sync(task_id="custom_dispense")
            assert it["execution"]["task_passed"] is True
            assert it["execution"]["non_idempotent_counts"]["fire_dispense_valve"] == 1
            assert any("sqlite.journal[fire_dispense_valve]" in e["action"] for e in it["journal_entries"])
            assert engine.kb.current_branch() == "main"
        finally:
            client.close()


def test_urdf_uri_rewriting_and_asset_path_traversal_protection(tmp_path: Path) -> None:
    """URDF mesh URI rewriting handles ROS package:// and blocks path traversal / non-asset files."""
    raw_urdf = (
        '<robot name="demo"><link name="base">'
        '<visual><geometry><mesh filename="package://my_arm_desc/meshes/base.stl"/></geometry></visual>'
        '<visual><geometry><mesh filename="assets/link1.stl"/></geometry></visual>'
        "</link></robot>"
    )
    rewritten = rewrite_urdf_mesh_uris(raw_urdf, asset_url_prefix="/api/robot/assets")
    assert 'filename="/api/robot/assets/meshes/base.stl"' in rewritten
    assert 'filename="/api/robot/assets/assets/link1.stl"' in rewritten

    root = tmp_path / "robot_assets"
    (root / "assets").mkdir(parents=True)
    (root / "assets" / "base.stl").write_bytes(b"solid test")
    (root / "secret.py").write_text("SECRET = 1", encoding="utf-8")
    (tmp_path / "outside.stl").write_bytes(b"outside")

    ok = read_safe_robot_asset(root, "assets/base.stl")
    assert ok is not None and ok[0] == b"solid test" and ok[1] == "model/stl"
    assert read_safe_robot_asset(root, "../outside.stl") is None
    assert read_safe_robot_asset(root, "secret.py") is None
    assert read_safe_robot_asset(root, "/etc/passwd") is None


def test_so101_plugin_serves_urdf_manifest_and_assets_over_http() -> None:
    """Built-in SO-101 plugin owns and serves its URDF, manifest, physics approximations, and STL meshes."""
    import os

    os.environ.setdefault("MUJOCO_GL", "egl")
    pytest.importorskip("mujoco")
    from trax.mujoco_so101_driver import MuJoCoSO101HostDriver

    cfg = SceneConfig.from_file(Path("examples/single_cup_coffee_config.json"))
    driver = MuJoCoSO101HostDriver(cfg)
    app = StandaloneSimHostApp(driver)
    server = create_sim_host_server(app, host="127.0.0.1", port=0)
    port = server.server_address[1]
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        client = MCPClient(transport="http", url=f"http://127.0.0.1:{port}/mcp")
        _, init_res, _ = client.rpc("initialize", {"protocolVersion": "2025-03-26"})
        info = init_res["result"]["serverInfo"]
        assert info["urdf_uri"] == "trax://robot/urdf"
        assert len(info["urdf_sha256"]) == 16

        _, urdf_res, _ = client.rpc("resources/read", {"uri": "trax://robot/urdf"})
        urdf_xml = urdf_res["result"]["contents"][0]["text"]
        assert "<robot name=\"lerobot_so101\">" in urdf_xml
        assert "/assets/robot/assets/base_so101_v2.stl" in urdf_xml

        _, man_res, _ = client.rpc("resources/read", {"uri": "trax://embodiment/manifest"})
        assert "kinematic_proximity_grasp_attach_58mm" in man_res["result"]["contents"][0]["text"]

        with opener.open(f"http://127.0.0.1:{port}/assets/robot/assets/base_so101_v2.stl", timeout=5) as r:
            assert r.status == 200
            assert r.headers.get("ETag")
            assert len(r.read()) > 1000

        with pytest.raises(urllib.error.HTTPError) as exc_info:
            opener.open(f"http://127.0.0.1:{port}/assets/robot/../../pyproject.toml", timeout=5)
        assert exc_info.value.code == 404

        with opener.open(f"http://127.0.0.1:{port}/scene/reconstruction", timeout=10) as r:
            assert r.status == 200
            recon = json.loads(r.read().decode("utf-8"))
            assert recon["total_surfels"] > 100
            assert recon["evidence_source"] == "simulator"
            if recon["perception_owner"] == "evidencescene":
                assert recon["observability"]["status"] == "observable"
                assert recon["observability"]["rank"] == 6
    finally:
        server.shutdown()
        server.server_close()
        driver.close()


def test_operator_abort_cancellation_token_stops_governed_proxy(tmp_path: Path) -> None:
    """CancellationToken immediately halts further MCP tool calls when operator aborts."""
    script = Path("examples/custom_mcp_robot/unknown_arm_mcp.py").resolve()
    client = MCPClient(transport="stdio", command=[sys.executable, str(script)], timeout_s=10.0)
    try:
        _, tools_resp, _ = client.rpc("tools/list", {})
        tools = tools_resp["result"]["tools"]
        tok = CancellationToken()
        proxy = GovernedMCPProxy(
            client,
            tools,
            journal_path=tmp_path / "j.sqlite",
            run_tag="abort-test",
            cancel_token=tok,
        )
        assert proxy.read_gantry_pose()["x_mm"] == 0.0
        tok.cancel("Operator E-Stop pressed")
        with pytest.raises(ExecutionAbortedError, match="Operator E-Stop"):
            proxy.move_gantry_xy(x_mm=10.0, y_mm=10.0)
    finally:
        client.close()


def test_knowledge_base_multi_seed_gate_and_urdf_kinematics_drift(tmp_path: Path) -> None:
    """Multi-seed gate holds single-seed passes in candidate state and URDF changes quarantine skills."""
    kb = ExecutableKnowledgeBase(tmp_path)
    br = kb.begin_session_branch("test-arm", "pick_task")
    src = "def run_policy(mcp, params):\n    return {'status': 'completed'}\n"

    # Single seed fails min_verified_seeds=2 gate -> stays on session branch as candidate
    cand = kb.promote_verified_skill_to_main(
        embodiment_id="test-arm",
        evidence_source="simulator",
        mcp_tools_sha256="1111222233334444",
        urdf_sha256="aaaa1111bbbb2222",
        task_id="pick_task",
        task_instruction="Pick object.",
        policy_source=src,
        verify_result={"passed": True, "detail": "seed0 ok"},
        verified_seeds=(0,),
        min_verified_seeds=2,
    )
    assert cand["promoted"] is False
    assert cand["stage"] == "candidate"
    assert kb.current_branch() == br

    # Multi-seed verification passes gate -> merges --no-ff to main
    promo = kb.promote_verified_skill_to_main(
        embodiment_id="test-arm",
        evidence_source="simulator",
        mcp_tools_sha256="1111222233334444",
        urdf_sha256="aaaa1111bbbb2222",
        task_id="pick_task",
        task_instruction="Pick object.",
        policy_source=src,
        verify_result={"passed": True, "detail": "seeds 0,1 ok"},
        verified_seeds=(0, 1),
        min_verified_seeds=2,
    )
    assert promo["promoted"] is True
    assert kb.current_branch() == "main"

    # Matching URDF -> active; changed URDF -> stale_kinematics
    ok_list = kb.list_skills(
        current_embodiment="test-arm",
        current_tools_sha="1111222233334444",
        current_urdf_sha="aaaa1111bbbb2222",
    )
    assert ok_list[0]["status"] == "active"

    drift_list = kb.list_skills(
        current_embodiment="test-arm",
        current_tools_sha="1111222233334444",
        current_urdf_sha="deadbeef99990000",
    )
    assert drift_list[0]["status"] == "stale_kinematics"


def test_backend_plugin_registry() -> None:
    sentinel = object()
    register_backend_plugin("test_custom_plugin", lambda cfg: sentinel)
    assert "test_custom_plugin" in list_backend_plugins()
    factory = resolve_backend_factory("test_custom_plugin")
    assert factory is not None
    assert factory(SceneConfig.from_file(Path("examples/single_cup_coffee_config.json"))) is sentinel
