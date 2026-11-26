"""Tests for the TRAX Closed-Loop Evolution Lab backend (Explore -> Synthesize -> Execute -> Iterate)."""
from __future__ import annotations

from pathlib import Path
import tempfile
import threading
import time

from trax.benchmark import builtin_reference_policy_source
from trax.config import SceneConfig
from trax.embodiments import EmbodimentManager, build_rax_coverage_matrix, load_archived_policy_comparison
from benchmarks.evolution_lab import EvolutionLabController
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
            return {
                "raw_output": f"```python\n{naive_src}```",
                "source": naive_src,
                "model": "mock-llm",
                "reasoning_effort": "medium",
                "wall_s": 0.05,
            }
        assert "PREVIOUS ITERATION FAILURE DIAGNOSTICS" in prompt
        return {
            "raw_output": f"Here is the repaired policy:\n```python\n{repaired_src}```",
            "source": repaired_src,
            "model": "mock-llm",
            "reasoning_effort": "medium",
            "wall_s": 0.06,
        }

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

    # Full transparency assertions: exact LLM prompts/responses & exact executed Python code
    assert len(st["llm_calls"]) == 2
    assert "PUBLIC-SDK EXPLORATION TELEMETRY" in st["llm_calls"][0]["prompt"]
    assert "```python" in st["llm_calls"][0]["raw_response"]
    assert "PREVIOUS ITERATION FAILURE DIAGNOSTICS" in st["llm_calls"][1]["prompt"]
    assert "Here is the repaired policy:" in st["llm_calls"][1]["raw_response"]

    assert len(st["executed_runs"]) == 3
    assert st["executed_runs"][0]["stage"] == "01_explore_probe"
    assert "sdk.move_to_target" in st["executed_runs"][0]["source"]
    assert st["executed_runs"][1]["stage"] == "03_policy_exec"
    assert st["executed_runs"][1]["source"].strip() == naive_src.strip()
    assert st["executed_runs"][2]["stage"] == "03_policy_exec"
    assert st["executed_runs"][2]["source"].strip() == repaired_src.strip()
    assert st["executed_runs"][2]["task_passed"] is True

    # Regression test: select_iteration and reset_session must NEVER deadlock self._state_lock
    with tempfile.TemporaryDirectory(prefix="trax-evo-reset-") as tmp2:
        lab2 = EvolutionLabController(cfg, driver, Path(tmp2), threading.RLock(), llm_fn=fake_llm)
        st_reset = lab2.reset_session(scenario_key="fault")
        assert st_reset["phase"] == "idle"
        assert st_reset["iterations"] == []
        st_sel = lab2.select_iteration(0)
        assert st_sel["active_iter_index"] == -1


def test_embodiment_manager_hot_switch_and_rax_matrix_and_comparator() -> None:
    cfg = SceneConfig.from_file(Path("examples/single_cup_coffee_config.json"))
    primary = FixtureHostDriver(cfg)
    ws = Path("experiments/coffee-closed-loop-20261120")
    mgr = EmbodimentManager(cfg, primary, ws)
    snap0 = mgr.snapshot()
    assert snap0["active_id"] == "primary_live"
    assert len(snap0["rax_matrix"]) >= 10
    families = {row["family"] for row in snap0["rax_matrix"]}
    assert "Bimanual Arms" in families
    assert "Dexterous Hand" in families
    assert "Mobile Bimanual" in families
    assert "Humanoid Whole-Body" in families

    # Hot-switch to deterministic fixture preset cleanly
    drv, meta = mgr.switch(target_id="fixture")
    assert drv.driver_name == "fixture"
    assert meta["evidence_source"] == "fixture"

    # Real archived cross-method policy comparison (if LIBERO-Object blind archive exists in repo)
    if "libero_object_blind" in snap0["experiment_catalog"]:
        cmp_res = load_archived_policy_comparison(
            ws,
            suite_key="libero_object_blind",
            task_key="libero_object_t1",
            left_method="craft_full",
            right_method="rax_protocol",
        )
        assert cmp_res["left"]["available"] is True
        assert cmp_res["right"]["available"] is True
        assert "def run_policy" in cmp_res["left"]["source"]
        assert "def run_policy" in cmp_res["right"]["source"]
        assert cmp_res["unified_diff"].startswith("---")
