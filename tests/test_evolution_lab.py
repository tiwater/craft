"""Tests for the TRAX Closed-Loop Evolution Lab backend (Explore -> Synthesize -> Execute -> Iterate)."""
from __future__ import annotations

from pathlib import Path
import tempfile
import threading
import time

from trax.benchmark import builtin_reference_policy_source
from trax.config import SceneConfig
from trax.evolution_lab import EvolutionLabController
from trax.host_driver import FixtureHostDriver


def _wait_idle(lab: EvolutionLabController, timeout_s: float = 10.0) -> dict:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        st = lab.get_state()
        if not st["busy"]:
            return st
        time.sleep(0.05)
    raise TimeoutError("EvolutionLabController did not finish in time")


def test_closed_loop_explore_synthesize_execute_iterate_with_diff_and_journal() -> None:
    cfg = SceneConfig.from_file(Path("examples/single_cup_coffee_config.json"))
    driver = FixtureHostDriver(cfg)

    # First LLM call returns a naive policy that ignores slip recentering and unknown-receipt reconciliation.
    naive_src = '''def run_policy(sdk, params):
    order = sdk.get_order(params.get("order_id", "A042"))
    scene = sdk.observe_scene()
    cup = scene["objects"][0]
    sdk.place_cup_under_machine(cup["id"], command_id="p1")
    sdk.start_coffee_machine(command_id="b1")
    sdk.wait_for_coffee_ready(command_id="w1")
    sdk.retrieve_and_deliver_cup(cup["id"], order["destination_slot"], command_id="d1")
    sdk.complete_order_delivery(order["order_id"], cup["id"], order["destination_slot"], command_id="c1")
    return {"status": "completed"}
'''
    repaired_src = builtin_reference_policy_source()
    calls = {"n": 0}

    def fake_llm(prompt: str) -> dict:
        calls["n"] += 1
        assert "PUBLIC-SDK EXPLORATION TELEMETRY" in prompt
        if calls["n"] == 1:
            return {"source": naive_src, "model": "mock-llm", "reasoning_effort": "medium", "wall_s": 0.05}
        assert "PREVIOUS ITERATION FAILURE DIAGNOSTICS" in prompt
        return {"source": repaired_src, "model": "mock-llm", "reasoning_effort": "medium", "wall_s": 0.06}

    with tempfile.TemporaryDirectory(prefix="trax-evo-") as tmp:
        lab = EvolutionLabController(
            cfg,
            driver,
            Path(tmp),
            threading.Lock(),
            llm_fn=fake_llm,
        )
        res = lab.start_closed_loop_async({"scenario_key": "fault", "max_iters": 3, "fresh": True})
        assert res["status"] == "started"
        st = _wait_idle(lab)

    assert st["probe_report"] is not None
    assert st["probe_report"]["allowed_methods_count"] >= 10
    assert len(st["iterations"]) == 2
    it0, it1 = st["iterations"]
    assert it0["origin"] == "llm_draft"
    assert it0["execution"]["task_passed"] is False
    assert it0["attribution"]["layer"] in ("policy", "control", "device")
    assert it1["origin"] == "llm_repair"
    assert it1["execution"]["task_passed"] is True
    assert it1["execution"]["duplicate_non_idempotent"] is False
    assert it1["unified_diff"].startswith("---")
    assert len(it1["journal_entries"]) >= 4
    assert st["phase"] == "verified"
