"""Integration tests for StandaloneSimHostApp (`craft serve-sim-host`) + HttpHostDriver."""
from __future__ import annotations

import os
from pathlib import Path
import threading
import pytest

from craft.benchmark import builtin_reference_policy_source
from craft.config import SceneConfig
from craft.host_driver import create_host_driver
from craft.journal import Journal
from craft.sim_host_server import StandaloneSimHostApp, create_sim_host_server


def _run_http_roundtrip(driver_name: str, tmp_path: Path) -> None:
    cfg = SceneConfig.from_file("examples/single_cup_coffee_config.json")
    data = cfg.to_dict()
    data["budget"]["deployment_scope"] = "fixture" if driver_name == "fixture" else "simulator"
    data["budget"]["allow_live_actuation"] = True
    data["budget"]["allow_sim_reset"] = True
    backend_cfg = SceneConfig.from_dict(data)

    sim_driver = create_host_driver(driver_name, backend_cfg)
    app = StandaloneSimHostApp(sim_driver)
    server = create_sim_host_server(app, host="127.0.0.1", port=0)
    port = server.server_address[1]
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    try:
        http_data = backend_cfg.to_dict()
        http_data["host"]["base_url"] = f"http://127.0.0.1:{port}"
        http_data["host"]["evaluator_base_url"] = f"http://127.0.0.1:{port}"
        http_cfg = SceneConfig.from_dict(http_data)
        client = create_host_driver("http", http_cfg)

        caps = client.discover_capabilities()
        assert caps.status == "ready"
        assert caps.discovered is not None
        assert "start_coffee_machine" in caps.discovered.allowed_methods

        obs = client.observe_scene()
        assert obs.status == "ok"

        # Camera fetch over HTTP
        cam = sim_driver.list_cameras()[0]
        img = client.fetch_camera_image(cam)
        assert img is not None
        assert len(img[0]) > 100

        # Reset + Slip + Timeout execution via HttpHostDriver (same path as ticos-edge!)
        client.reset_scenario({"preset": "default", "cup_a_xy": [0.32, -0.04]})
        res = client.run_policy_scenario(
            builtin_reference_policy_source(),
            command_id="http-sim-e2e-1",
            scenario={
                "order_id": "A042",
                "target_slot": "$SLOT_PICKUP_1",
                "fault_inject_slip_xy_m": [0.036, -0.012],
                "simulate_timeout": True,
                "expect_status": "completed",
            },
            journal=Journal(tmp_path / f"{driver_name}_http.sqlite"),
        )
        assert res["task_passed"] is True
        assert res["oracle_post_run"]["cup_filled"] is True
        assert res["non_idempotent_counts"]["start_coffee_machine"] == 1
        assert res["queried_after_unknown"] is True
        assert res["duplicate_non_idempotent"] is False
    finally:
        server.shutdown()
        server.server_close()


def test_standalone_sim_host_http_roundtrip_fixture(tmp_path: Path) -> None:
    _run_http_roundtrip("fixture", tmp_path)


def test_standalone_sim_host_http_roundtrip_mujoco(tmp_path: Path) -> None:
    os.environ.setdefault("MUJOCO_GL", "egl")
    pytest.importorskip("mujoco")
    _run_http_roundtrip("mujoco_cafe", tmp_path)
