"""Contract tests for headless CRAFT Engine /v1/... REST API."""
from __future__ import annotations

import json
import threading
import urllib.request

from craft.benchmark import builtin_reference_policy_source
from craft.config import SceneConfig
from craft.engine_api import CraftEngineService, create_engine_server
from craft.host_driver import create_host_driver


def _fixture_config() -> SceneConfig:
    cfg = SceneConfig.from_file("examples/single_cup_coffee_config.json")
    data = cfg.to_dict()
    data["budget"]["deployment_scope"] = "fixture"
    data["budget"]["allow_live_actuation"] = True
    data["budget"]["allow_sim_reset"] = True
    return SceneConfig.from_dict(data)


def _req(opener, url: str, body=None):
    if body is None:
        with opener.open(url, timeout=5) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    req = urllib.request.Request(
        url,
        data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with opener.open(req, timeout=5) as r:
        return r.status, json.loads(r.read().decode("utf-8"))


def test_craft_engine_v1_api_end_to_end(tmp_path):
    cfg = _fixture_config()
    drv = create_host_driver("fixture", cfg)
    svc = CraftEngineService(cfg, drv, tmp_path / "engine_ws")
    server = create_engine_server(svc, host="127.0.0.1", port=0)
    port = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    base = f"http://127.0.0.1:{port}"
    try:
        code, health = _req(opener, base + "/v1/health")
        assert code == 200 and health["api_version"] == "v1" and health["driver"] == "fixture"

        code, caps = _req(opener, base + "/v1/capabilities")
        assert code == 200 and caps["capability_check"]["status"] == "ready"
        assert "open_oven_door" in caps["gap_check"]["missing_methods"]

        # Teaching spec validation
        code, spec = _req(
            opener,
            base + "/v1/teaching/spec",
            {"instruction": "Make one hot americano", "materials": [{"material_id": "v1", "kind": "video", "summary": "ref"}]},
        )
        assert code == 200 and spec["status"] == "ready"

        # Execute policy with fault injection + automatic attribution & profiling
        code, ex = _req(
            opener,
            base + "/v1/execute",
            {
                "policy_source": builtin_reference_policy_source(),
                "scenario": {"preset": "default", "fault_inject_slip_xy_m": [0.038, -0.012], "simulate_timeout": True},
            },
        )
        assert code == 200
        assert ex["execution"]["task_passed"] is True
        assert ex["attribution"]["layer"] == "none"
        assert ex["step_profile"]["step_count"] > 0

        # Capture a failure into regression suite and run regression gate
        code, cap_res = _req(
            opener,
            base + "/v1/regression/capture",
            {
                "trial": {"trial_id": "fail-1", "task_passed": False, "host_runtime_status": "failed_brew_start", "fault_injection": True, "methods": ["place_cup_under_machine", "start_coffee_machine"]},
                "scenario_context": {"preset": "default", "fault_inject_slip_xy_m": [0.038, -0.012], "simulate_timeout": True},
            },
        )
        assert code == 200 and cap_res["captured"] is True

        code, gate = _req(opener, base + "/v1/regression/gate", {"policy_source": builtin_reference_policy_source()})
        assert code == 200 and gate["passed"] is True and gate["total_cases"] == 1

        # Autonomy gate check blocks sim -> physical_live jump
        _req(
            opener,
            base + "/v1/autonomy/evidence",
            {
                "stage": "simulator",
                "policy_sha256": "p1",
                "config_sha256": "c1",
                "sdk_contract_sha256": "s1",
                "report_id": "r1",
                "passed": True,
                "total_trials": 6,
                "human_interventions": 0,
            },
        )
        code, promo = _req(
            opener,
            base + "/v1/autonomy/check",
            {"policy_sha256": "p1", "target_stage": "physical_live", "sdk_contract_sha256": "s1"},
        )
        assert code == 200 and promo["allowed"] is False
    finally:
        server.shutdown()
        server.server_close()
