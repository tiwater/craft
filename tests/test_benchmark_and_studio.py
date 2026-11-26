"""Standalone tests for decoupled HostDriver, paired Benchmark, and TRAX Studio."""
from __future__ import annotations

import json
import threading
import urllib.request

from trax.benchmark import (
    PolicyEntry,
    builtin_reference_policy_source,
    default_coffee_benchmark_suite,
    run_paired_benchmark,
)
from trax.config import SceneConfig
from trax.host_driver import FixtureHostDriver, create_host_driver, list_host_drivers, register_host_driver
from trax.ui import TraxUIApp, create_ui_server


BROKEN_NO_RECENTER_POLICY = """def run_policy(sdk, params):
    order_id = params.get("order_id", "A042")
    obs = sdk.observe_scene()
    objs = obs.get("objects", [])
    if len(objs) != 1:
        return {"status": "paused_for_operator"}
    cup_id = objs[0]["id"]
    sdk.place_cup_under_machine(cup_id, command_id="p")
    # Deliberately skips recenter -> fails when slip is injected
    r = sdk.start_coffee_machine(command_id="b")
    if r.get("receipt") == "unknown":
        r = sdk.query_command_status("b")
    if r.get("receipt") not in ("running", "done"):
        return {"status": "failed_brew_start"}
    sdk.wait_for_coffee_ready(command_id="w")
    sdk.retrieve_and_deliver_cup(cup_id, params.get("target_slot", "$SLOT_PICKUP_1"), command_id="d")
    sdk.complete_order_delivery(order_id, cup_id, params.get("target_slot", "$SLOT_PICKUP_1"), command_id="c")
    return {"status": "completed"}
"""


def _fixture_config() -> SceneConfig:
    cfg = SceneConfig.from_file("examples/single_cup_coffee_config.json")
    data = cfg.to_dict()
    data["budget"]["deployment_scope"] = "fixture"
    data["budget"]["allow_live_actuation"] = True
    data["budget"]["allow_sim_reset"] = True
    return SceneConfig.from_dict(data)


def test_paired_benchmark_standalone_fixture_and_html_report(tmp_path):
    cfg = _fixture_config()
    driver = create_host_driver("fixture", cfg)
    policies = [
        PolicyEntry("ref", "TRAX Reference", builtin_reference_policy_source(), origin="reference"),
        PolicyEntry("broken", "No-Recenter Ablation", BROKEN_NO_RECENTER_POLICY, origin="ablation"),
    ]
    report = run_paired_benchmark(
        driver,
        policies,
        default_coffee_benchmark_suite(),
        tmp_path / "bench",
        repeats=1,
    )
    assert report["status"] == "completed"
    assert report["evidence_source"] == "fixture"
    assert report["summary_by_policy"]["ref"]["pass_rate"] == 1.0
    # Broken policy fails the 2 slip-fault scenarios
    assert report["summary_by_policy"]["broken"]["pass_rate"] < 1.0
    assert report["summary_by_policy"]["broken"]["failure_breakdown"].get("failed_brew_start", 0) == 2

    html_path = tmp_path / "bench" / "benchmark_report.html"
    assert html_path.is_file()
    html_text = html_path.read_text(encoding="utf-8")
    assert "TRAX Paired Benchmark Report" in html_text
    assert "No-Recenter Ablation" in html_text
    assert "NOT physical-robot / food-quality certification" in html_text


def test_trax_ui_http_endpoints_standalone(tmp_path):
    cfg = _fixture_config()
    driver = create_host_driver("fixture", cfg)
    app = TraxUIApp(cfg, driver, tmp_path / "ui_ws")
    server = create_ui_server(app, host="127.0.0.1", port=0)
    port = server.server_address[1]
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    base = f"http://127.0.0.1:{port}"
    try:
        # Index page
        with opener.open(base + "/", timeout=5) as r:
            body = r.read().decode("utf-8")
            assert r.status == 200
            assert "TRAX UI" in body

        # State endpoint
        with opener.open(base + "/api/state", timeout=5) as r:
            st = json.loads(r.read().decode("utf-8"))
            assert st["driver"] == "fixture"
            assert st["evidence_source"] == "fixture"
            assert st["capability_check"]["status"] == "ready"
            assert st["cameras"] == ["overhead"]

        # Static CSS & JS assets served cleanly
        with opener.open(base + "/static/app.css", timeout=5) as r:
            assert r.status == 200
            assert "text/css" in r.headers.get("Content-Type", "")
        with opener.open(base + "/static/app.js", timeout=5) as r:
            assert r.status == 200
            assert "javascript" in r.headers.get("Content-Type", "")

        # Standalone SVG camera stream
        with opener.open(base + "/api/camera/overhead", timeout=5) as r:
            svg = r.read().decode("utf-8")
            assert "<svg" in svg

        # Lightweight live telemetry endpoint
        with opener.open(base + "/api/live", timeout=5) as r:
            live = json.loads(r.read().decode("utf-8"))
            assert "scene_observation" in live
            assert "live_runtime" in live
            assert "run_state" in live

        # Interactive scenario run with slip + timeout fault injection
        req = urllib.request.Request(
            base + "/api/run",
            data=json.dumps(
                {
                    "policy_id": "craft_reference",
                    "preset": "default",
                    "fault_slip": True,
                    "simulate_timeout": True,
                }
            ).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with opener.open(req, timeout=5) as r:
            run_res = json.loads(r.read().decode("utf-8"))
            assert run_res["task_passed"] is True
            assert "recenter_cup_under_spout" in run_res["methods"]
            assert "query_command_status" in run_res["methods"]
            assert run_res["non_idempotent_counts"]["start_coffee_machine"] == 1
    finally:
        server.shutdown()
        server.server_close()


def test_host_driver_plugin_registry_inversion():
    """External hosts register themselves into TRAX; TRAX does not assume any external host."""
    class CustomExternalHostDriver(FixtureHostDriver):
        driver_name = "custom_external_sim"

    register_host_driver("custom_external_sim", CustomExternalHostDriver)
    assert "custom_external_sim" in list_host_drivers()
    assert "fixture" in list_host_drivers()
    assert "http" in list_host_drivers()
    drv = create_host_driver("custom_external_sim", _fixture_config())
    assert drv.driver_name == "custom_external_sim"
