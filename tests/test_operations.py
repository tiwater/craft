"""End-to-end behavioral tests for Milestone 1:
Continuous Day-Shift (with mid-brew crash/restart & journal durability),
Nightly Evolution (RepairScreener -> Recorded Shadow -> Independent Acceptance -> Release -> Rollback -> SkillExperience),
and Engine API / Studio endpoints.
"""
from __future__ import annotations

import json
import threading
import urllib.request

from craft.benchmark import builtin_reference_policy_source
from craft.config import SceneConfig
from craft.engine_api import CraftEngineService, create_engine_server
from craft.host_driver import create_host_driver
from craft.operations import (
    run_continuous_shift,
    run_nightly_evolution,
    run_recorded_shadow_check,
)
from craft.regression import RegressionSuite
from craft.release import ReleaseRegistry
from craft.skills import SkillExperienceStore
from craft.studio import CraftStudioApp, create_studio_server


BROKEN_BASELINE = """def run_policy(sdk, params):
    obs = sdk.observe_scene()
    if not obs.get("control_ready"):
        return {"status": "rejected_observation_not_ready"}
    cups = [o for o in (obs.get("objects") or []) if o.get("cup_type") != "flexible_bag" and o.get("confidence", 0.0) >= obs.get("confidence_threshold", 0.75)]
    if len(cups) != 1:
        sdk.request_operator_confirmation("ambiguous", cups)
        return {"status": "paused_for_operator"}
    cup_id = cups[0]["id"]
    sdk.place_cup_under_machine(cup_id, command_id=params["command_prefix"] + "-place")
    # Missing L2 recenter check and missing unknown-ack reconciliation!
    r = sdk.start_coffee_machine(command_id=params["command_prefix"] + "-brew")
    if r.get("receipt") not in ("running", "done"):
        return {"status": "failed_brew_start"}
    sdk.wait_for_coffee_ready(command_id=params["command_prefix"] + "-wait")
    sdk.retrieve_and_deliver_cup(cup_id, params["target_slot"], command_id=params["command_prefix"] + "-deliver")
    sdk.complete_order_delivery(params["order_id"], cup_id, params["target_slot"], command_id=params["command_prefix"] + "-complete")
    return {"status": "completed"}
"""


def _mujoco_config() -> SceneConfig:
    cfg = SceneConfig.from_file("examples/single_cup_coffee_config.json")
    data = cfg.to_dict()
    data["budget"]["deployment_scope"] = "simulator"
    data["budget"]["allow_live_actuation"] = True
    data["budget"]["allow_sim_reset"] = True
    data["acceptance"]["min_trials"] = 2
    data["acceptance"]["min_completion_rate"] = 1.0
    return SceneConfig.from_dict(data)


def test_mujoco_mid_brew_crash_stateless_duplicates_vs_craft_journal_recovers(tmp_path):
    cfg = _mujoco_config()
    driver = create_host_driver("mujoco_cafe", cfg)
    try:
        crash_order = [
            {
                "shift_order_id": "crash_01",
                "label": "Mid-brew host crash",
                "preset": "default",
                "order_id": "A042",
                "target_slot": "$SLOT_PICKUP_1",
                "simulate_crash_mid_brew": True,
                "expect_status": "completed",
            }
        ]
        # 1. Stateless restart (no journal proxy): re-runs from start and DUPLICATES start_coffee_machine!
        stateless = run_continuous_shift(
            driver,
            cfg,
            tmp_path / "stateless_ws",
            policy_source=builtin_reference_policy_source(),
            orders=crash_order,
            enable_journal_proxy=False,
        )
        assert stateless["passed_orders"] == 0
        assert stateless["duplicate_non_idempotent_orders"] == 1
        assert stateless["trials"][0]["restarted_after_crash"] is True
        assert stateless["trials"][0]["non_idempotent_counts"]["start_coffee_machine"] == 2

        # 2. CRAFT Journaled restart: reconciles committed brew intent via query_command_status -> 0 duplicates!
        journaled = run_continuous_shift(
            driver,
            cfg,
            tmp_path / "journaled_ws",
            policy_source=builtin_reference_policy_source(),
            orders=crash_order,
            enable_journal_proxy=True,
        )
        assert journaled["passed_orders"] == 1
        assert journaled["duplicate_non_idempotent_orders"] == 0
        assert journaled["unreconciled_unknown_orders"] == 0
        assert journaled["trials"][0]["restarted_after_crash"] is True
        assert journaled["trials"][0]["non_idempotent_counts"]["start_coffee_machine"] == 1
    finally:
        driver.close()


def test_full_day_shift_to_nightly_evolution_release_rollback_and_skills(tmp_path):
    cfg = _mujoco_config()
    driver = create_host_driver("mujoco_cafe", cfg)
    try:
        ws = tmp_path / "ops_ws"
        reg_suite = RegressionSuite(ws / "regression_suite.json")
        registry = ReleaseRegistry(ws / "registry")
        skills = SkillExperienceStore(ws / "skills.json")

        # Day shift with broken baseline captures slip+timeout failure into RegressionSuite
        shift = run_continuous_shift(
            driver,
            cfg,
            ws,
            policy_source=BROKEN_BASELINE,
            orders=[
                {
                    "shift_order_id": "o1_normal",
                    "preset": "default",
                    "order_id": "A042",
                    "target_slot": "$SLOT_PICKUP_1",
                    "expect_status": "completed",
                },
                {
                    "shift_order_id": "o2_slip_timeout",
                    "preset": "default",
                    "order_id": "A042",
                    "target_slot": "$SLOT_PICKUP_1",
                    "fault_inject_slip_xy_m": [0.036, -0.012],
                    "simulate_timeout": True,
                    "expect_status": "completed",
                },
            ],
            regression_suite=reg_suite,
            enable_journal_proxy=True,
        )
        assert shift["passed_orders"] == 1
        assert len(shift["captured_regression_case_ids"]) == 1
        target_case = reg_suite.cases[0]
        assert target_case.attribution_layer == "policy"

        # Nightly evolution: RepairScreener -> Recorded Shadow -> Independent Acceptance -> Publish v2 -> SkillExperience
        heldout = [
            {"scenario_id": "h1_store_b", "preset": "default", "store_layout_shift_xy_m": [0.02, -0.02], "expect_status": "completed"},
            {"scenario_id": "h2_crash", "preset": "default", "simulate_crash_mid_brew": True, "expect_status": "completed"},
        ]
        evo = run_nightly_evolution(
            driver,
            cfg,
            ws,
            baseline_source=BROKEN_BASELINE,
            target_case=target_case,
            regression_suite=reg_suite,
            release_registry=registry,
            skill_store=skills,
            heldout_scenarios=heldout,
            new_version="v2.0",
            candidate_sources={
                "bad_broken": BROKEN_BASELINE,
                "good_ref": builtin_reference_policy_source(),
            },
            run_shadow_check=True,
        )
        assert evo["status"] == "promoted"
        assert evo["shadow_check"]["evidence_source"] == "recorded_data"
        assert evo["shadow_check"]["all_passed"] is True
        assert evo["acceptance_report"]["passed"] is True
        assert registry.active_version_id == "v2.0"

        # Verify SkillExperience recorded with measured limits and regression case IDs
        cap = driver.discover_capabilities(allow_busy=True)
        matched_skills = skills.query_applicable(
            task_name=cfg.task_name,
            available_methods=cap.discovered.allowed_methods,
            sdk_contract_sha256=cap.discovered.sdk_contract_sha256,
        )
        assert len(matched_skills) == 1
        assert matched_skills[0].measured_limits.get("verified_slip_mm", 0) > 30.0
        assert target_case.case_id in matched_skills[0].regression_case_ids

        # Next day shift with frozen v2.0 release passes all 6 default shift orders (0 duplicates, 0 false actuations)
        day2 = run_continuous_shift(
            driver,
            cfg,
            ws,
            policy_source=registry.get_policy_source(),
            release_registry=registry,
            regression_suite=reg_suite,
            enable_journal_proxy=True,
            verify_frozen_release=True,
        )
        assert day2["passed_orders"] == day2["total_orders"] == 6
        assert day2["duplicate_non_idempotent_orders"] == 0
        assert day2["false_actuation_orders"] == 0
    finally:
        driver.close()


def test_recorded_shadow_check_and_engine_studio_operations_endpoints(tmp_path):
    cfg = SceneConfig.from_file("examples/single_cup_coffee_config.json")
    data = cfg.to_dict()
    data["budget"]["deployment_scope"] = "fixture"
    data["budget"]["allow_live_actuation"] = True
    data["budget"]["allow_sim_reset"] = True
    data["acceptance"]["min_trials"] = 2
    cfg = SceneConfig.from_dict(data)

    shadow = run_recorded_shadow_check(cfg, builtin_reference_policy_source())
    assert shadow["evidence_source"] == "recorded_data"
    assert shadow["all_passed"] is True
    assert shadow["false_actuations"] == 0

    driver = create_host_driver("fixture", cfg)
    svc = CraftEngineService(cfg, driver, tmp_path / "eng_ws")
    server = create_engine_server(svc, host="127.0.0.1", port=0)
    port = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        # POST /v1/shift/run
        req = urllib.request.Request(
            f"http://127.0.0.1:{port}/v1/shift/run",
            data=json.dumps(
                {
                    "policy_source": builtin_reference_policy_source(),
                    "orders": [{"shift_order_id": "s1", "preset": "default", "expect_status": "completed"}],
                }
            ).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with opener.open(req, timeout=5) as r:
            sh = json.loads(r.read().decode("utf-8"))
            assert sh["passed_orders"] == 1

        # GET /v1/screening
        with opener.open(f"http://127.0.0.1:{port}/v1/screening", timeout=5) as r:
            sc = json.loads(r.read().decode("utf-8"))
            assert "sessions" in sc
    finally:
        server.shutdown()
        server.server_close()

    # Studio exposes latest_shift_report and screening_sessions cleanly
    studio_app = CraftStudioApp(cfg, driver, tmp_path / "eng_ws")
    st = studio_app.get_state()
    assert st["latest_shift_report"]["passed_orders"] == 1
    assert "screening_sessions" in st
