"""Software orchestration test for run_coffee_closed_loop (explicitly tagged fixture)."""
from __future__ import annotations

from typing import Any, Mapping
import time

from trax.coffee_loop import run_coffee_closed_loop
from trax.config import SceneConfig
from trax.edge_adapter import EdgeHostAdapter


COFFEE_METHODS = [
    "get_order",
    "observe_scene",
    "request_operator_confirmation",
    "retract_safe_pose",
    "escalate_to_human_operator",
    "complete_order_delivery",
    "query_command_status",
    "move_to_target",
    "place_cup_under_machine",
    "recenter_cup_under_spout",
    "start_coffee_machine",
    "wait_for_coffee_ready",
    "retrieve_and_deliver_cup",
]


class SimStubTransport:
    def __init__(self) -> None:
        self.seq = 10
        self.call_idx = 0

    def get_json(self, url: str, timeout_s: float) -> dict[str, Any]:
        if url.endswith("/cap/state"):
            return {
                "ok": True,
                "executionAuthority": "ticos-edge-zero-gt",
                "perceptionOwner": "EvidenceScene-v2.0.0-4cam-RGBD",
                "sdkVersion": "1.1.0",
                "sdkAllowedMethods": COFFEE_METHODS,
                "sdkContractSha256": "contract-sha-test",
                "deviceBundleId": "sim-test",
                "deviceBundleSha256": "bundle-sha-test",
                "runtime": {"status": "idle"},
            }
        if url.endswith("/edge-scene"):
            self.seq += 1
            return {
                "source": "EvidenceScene",
                "timestamp_ns": time.time_ns(),
                "step_seq": self.seq,
                "objects": [{"object_id": "cup_1", "semantic_class": "dine_in_red", "confidence": 0.95}],
            }
        if url.endswith("/v1/sim/testbed/oracle"):
            return {
                "cup": [0.382, -0.171, 1.075],
                "coffee_machine": {"cup_filled": True},
            }
        raise AssertionError(f"Unexpected GET {url}")

    def post_json(self, url: str, body: Mapping[str, Any], timeout_s: float) -> dict[str, Any]:
        if url.endswith("/cap/reset") or url.endswith("/v1/sim/testbed/setup"):
            return {"ok": True}
        if url.endswith("/cap/execute"):
            self.call_idx += 1
            fault = bool(body.get("simulateTimeout"))
            methods = ["get_order", "observe_scene", "place_cup_under_machine"]
            if fault:
                methods.extend(["recenter_cup_under_spout", "start_coffee_machine", "command_ack_timeout_unknown", "query_command_status"])
            else:
                methods.append("start_coffee_machine")
            methods.extend(["wait_for_coffee_ready", "retrieve_and_deliver_cup", "complete_order_delivery"])
            return {
                "runtime": {
                    "status": "completed",
                    "steps": [{"method": m} for m in methods],
                    "auditRecord": {"plan": [{"skill_kind": "start_coffee_machine", "execution_count": 1}]},
                    "result": {"sandbox_isolation": {"mode": "os_subprocess_isolated"}},
                }
            }
        raise AssertionError(f"Unexpected POST {url}")


def test_coffee_closed_loop_orchestration_fixture(tmp_path):
    cfg = SceneConfig.from_file("examples/single_cup_coffee_config.json")
    data = cfg.to_dict()
    data["budget"]["allow_live_actuation"] = True
    data["budget"]["allow_sim_reset"] = True
    data["budget"]["deployment_scope"] = "fixture"
    fixture_cfg = SceneConfig.from_dict(data)

    gen_calls = {"n": 0}

    def fake_gen(c, prompt):
        gen_calls["n"] += 1
        return {
            "source": f"def run_policy(sdk, params):\n    return {{'status': 'completed', 'iter': {gen_calls['n']}}}\n",
            "provider": "codex_cli",
            "model": "fixture-stub",
            "reasoning_effort": "medium",
            "evidence_source": "fixture",
        }

    adapter = EdgeHostAdapter(fixture_cfg, transport=SimStubTransport())
    summary = run_coffee_closed_loop(fixture_cfg, tmp_path / "loop", adapter=adapter, generator_fn=fake_gen)

    assert summary["status"] == "completed"
    assert summary["oven_gap"]["status"] == "missing_capabilities"
    assert "open_oven_door" in summary["oven_gap"]["missing_methods"]
    assert summary["v1_acceptance"]["passed"] is True
    assert summary["improvement_result"]["decision"] in {"accepted", "rejected"}
    assert (tmp_path / "loop" / "protocol.json").is_file()
    assert (tmp_path / "loop" / "summary.json").is_file()
    assert (tmp_path / "loop" / "v1_acceptance_report.json").is_file()
    assert (tmp_path / "loop" / "v2_acceptance_report.json").is_file()
