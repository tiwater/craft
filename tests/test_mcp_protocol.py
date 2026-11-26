"""Tests for TRAX pure-stdlib MCP JSON-RPC 2.0 server, HTTP client, and inspector workbench."""
from __future__ import annotations

from pathlib import Path
import tempfile
import threading

from trax.config import SceneConfig
from trax.host_driver import FixtureHostDriver
from trax.mcp_protocol import MCPClient, MCPInspectorController, build_driver_mcp_server
from trax.sim_host_server import StandaloneSimHostApp, create_sim_host_server


def test_mcp_server_over_http_discovery_call_and_resources() -> None:
    cfg = SceneConfig.from_file(Path("examples/single_cup_coffee_config.json"))
    driver = FixtureHostDriver(cfg)
    app = StandaloneSimHostApp(driver)
    server = create_sim_host_server(app, host="127.0.0.1", port=0)
    port = server.server_address[1]
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    try:
        client = MCPClient(transport="http", url=f"http://127.0.0.1:{port}/mcp")
        _, init_res, _ = client.rpc("initialize", {"protocolVersion": "2025-03-26"})
        assert "error" not in init_res
        assert init_res["result"]["serverInfo"]["name"].startswith("trax-mcp-")

        _, tools_res, _ = client.rpc("tools/list", {})
        tools = tools_res["result"]["tools"]
        names = {tool["name"] for tool in tools}
        assert "observe_scene" in names
        assert "start_coffee_machine" in names
        brew_tool = next(tool for tool in tools if tool["name"] == "start_coffee_machine")
        assert brew_tool["annotations"]["idempotentHint"] is False
        assert brew_tool["annotations"]["destructiveHint"] is True

        _, call_res, _ = client.rpc("tools/call", {"name": "observe_scene", "arguments": {}})
        assert call_res["result"]["isError"] is False
        assert "objects" in call_res["result"]["structuredContent"]

        _, res_list, _ = client.rpc("resources/list", {})
        uris = {r["uri"] for r in res_list["result"]["resources"]}
        assert "trax://scene/state" in uris

        _, res_read, _ = client.rpc("resources/read", {"uri": "trax://scene/state"})
        assert res_read["result"]["contents"][0]["mimeType"] == "application/json"
    finally:
        server.shutdown()
        server.server_close()


def test_mcp_inspector_controller_journals_non_idempotent_tool_calls() -> None:
    cfg = SceneConfig.from_file(Path("examples/single_cup_coffee_config.json"))
    driver = FixtureHostDriver(cfg)
    with tempfile.TemporaryDirectory(prefix="trax-mcp-") as tmp:
        insp = MCPInspectorController(Path(tmp), lambda: driver, threading.Lock())
        st = insp.inspect({"transport": "local"})
        assert len(st["tools"]) >= 8
        assert "DYNAMICALLY DISCOVERED MCP TOOLS" in st["prompt_contract_preview"]

        # Read-only call -> no journal required
        r1 = insp.call_tool({"tool_name": "observe_scene", "arguments": {}})
        assert r1["journal_id"] is None
        assert "error" not in r1["response"]

        # Non-idempotent call -> automatically journaled in SQLite
        r2 = insp.call_tool({"tool_name": "start_coffee_machine", "arguments": {"command_id": "mcp-brew-test"}})
        assert r2["non_idempotent"] is True
        assert r2["journal_id"] is not None
        assert (Path(tmp) / "ui_mcp_journal.sqlite").is_file()
