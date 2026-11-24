"""Behavioral contract tests for CafeBench v2 commercial operations resilience benchmark."""
from __future__ import annotations

from craft.cafebench_v2 import (
    cafebench_v2_dev_scenarios,
    cafebench_v2_eval_scenarios,
    run_cafebench_v2_comparison,
)
from craft.config import SceneConfig
from craft.host_driver import create_host_driver


ROBUST_CAFE_V2_POLICY = """def run_policy(sdk, params):
    order_id = params.get("order_id", "A042")
    prefix = params.get("command_prefix", "cap")
    order = sdk.get_order(order_id)
    if not order:
        sdk.retract_safe_pose(command_id=prefix + "-retract")
        sdk.escalate_to_human_operator(order_id, "missing_order", command_id=prefix + "-esc")
        return {"status": "escalated_to_human"}
    target_slot = params.get("target_slot") or order.get("destination_slot") or "$SLOT_PICKUP_1"
    special = str(order.get("special_request") or "").strip()
    if special or ("NOGO" in target_slot) or ("STEAM" in target_slot):
        sdk.retract_safe_pose(command_id=prefix + "-retract")
        sdk.escalate_to_human_operator(order_id, "hazard_or_forbidden", command_id=prefix + "-esc")
        return {"status": "escalated_to_human"}

    obs = sdk.observe_scene()
    if not obs.get("control_ready"):
        return {"status": "rejected_observation_not_ready"}

    cm = obs.get("coffee_machine") or {}
    m_status = str(cm.get("status", "idle"))
    if m_status == "fault":
        sdk.retract_safe_pose(command_id=prefix + "-retract")
        sdk.escalate_to_human_operator(order_id, "machine_fault", command_id=prefix + "-esc")
        return {"status": "escalated_to_human"}
    if m_status in ("busy", "cleaning"):
        for poll_idx in range(5):
            obs = sdk.observe_scene()
            if not obs.get("control_ready"):
                return {"status": "rejected_observation_not_ready"}
            m_status = str((obs.get("coffee_machine") or {}).get("status", "idle"))
            if m_status in ("idle", "running", "completed"):
                break
        if m_status not in ("idle", "running", "completed"):
            sdk.retract_safe_pose(command_id=prefix + "-retract")
            sdk.escalate_to_human_operator(order_id, "machine_busy_timeout", command_id=prefix + "-esc")
            return {"status": "escalated_to_human"}

    thresh = float(obs.get("confidence_threshold", 0.75))
    objs = obs.get("objects") or []
    rigid = [o for o in objs if o.get("cup_type") != "flexible_bag" and float(o.get("confidence", 0.0)) >= thresh]
    confirmed = obs.get("operator_confirmed_cup_id")
    cup_id = None
    if confirmed:
        for o in rigid:
            if o.get("id") == confirmed:
                cup_id = confirmed
                break
    if cup_id is None:
        if len(rigid) != 1:
            sdk.request_operator_confirmation("ambiguous_or_missing_cup", rigid)
            return {"status": "paused_for_operator"}
        cup_id = rigid[0]["id"]

    brew_cmd = prefix + "-brew"
    if m_status in ("running", "completed"):
        q_res = sdk.query_command_status(brew_cmd)
        if q_res.get("receipt") not in ("running", "done", "completed"):
            sdk.retract_safe_pose(command_id=prefix + "-retract")
            sdk.escalate_to_human_operator(order_id, "foreign_order_running", command_id=prefix + "-esc")
            return {"status": "escalated_to_human"}
    else:
        p_res = sdk.place_cup_under_machine(cup_id, command_id=prefix + "-place")
        if p_res.get("receipt") != "done":
            return {"status": "failed_place"}

        obs2 = sdk.observe_scene()
        if not (obs2.get("l2_pouring_safety") or {}).get("safe_to_pour", False):
            rc = sdk.recenter_cup_under_spout(cup_id, command_id=prefix + "-recenter")
            if rc.get("receipt") != "done":
                return {"status": "failed_recenter"}
            obs3 = sdk.observe_scene()
            if not (obs3.get("l2_pouring_safety") or {}).get("safe_to_pour", False):
                sdk.retract_safe_pose(command_id=prefix + "-retract")
                sdk.escalate_to_human_operator(order_id, "l2_unsafe_after_recenter", command_id=prefix + "-esc")
                return {"status": "escalated_to_human"}

        b_res = sdk.start_coffee_machine(command_id=brew_cmd)
        rec = b_res.get("receipt")
        if rec == "unknown":
            q_res = sdk.query_command_status(brew_cmd)
            rec = q_res.get("receipt")
        if rec not in ("running", "done", "completed"):
            sdk.retract_safe_pose(command_id=prefix + "-retract")
            sdk.escalate_to_human_operator(order_id, "unreconciled_brew_receipt", command_id=prefix + "-esc")
            return {"status": "escalated_to_human"}

    w_res = sdk.wait_for_coffee_ready(command_id=prefix + "-wait")
    if w_res.get("receipt") != "done":
        return {"status": "failed_wait"}
    d_res = sdk.retrieve_and_deliver_cup(cup_id, target_slot, command_id=prefix + "-deliver")
    if d_res.get("receipt") != "done":
        return {"status": "failed_deliver"}
    c_res = sdk.complete_order_delivery(order_id, cup_id, target_slot, command_id=prefix + "-complete")
    if c_res.get("receipt") != "done":
        return {"status": "failed_complete"}
    return {"status": "completed"}
"""


def test_cafebench_v2_disjoint_split_and_journal_crash_advantage_on_mujoco(tmp_path):
    dev = cafebench_v2_dev_scenarios()
    ev = cafebench_v2_eval_scenarios()
    assert len(dev) == 6
    assert len(ev) == 30
    assert set(s["scenario_id"] for s in dev).isdisjoint(set(s["scenario_id"] for s in ev))
    assert sum(1 for s in ev if s["evidence_source"] == "simulator") == 26
    assert sum(1 for s in ev if s["evidence_source"] == "recorded_data") == 4

    cfg = SceneConfig.from_file("examples/single_cup_coffee_config.json")
    data = cfg.to_dict()
    data["budget"]["deployment_scope"] = "simulator"
    data["budget"]["allow_live_actuation"] = True
    data["budget"]["allow_sim_reset"] = True
    cfg = SceneConfig.from_dict(data)

    driver = create_host_driver("mujoco_cafe", cfg)
    try:
        def fake_llm(prompt: str):
            return {"source": ROBUST_CAFE_V2_POLICY, "wall_s": 0.01}

        report = run_cafebench_v2_comparison(
            driver,
            cfg,
            llm_fn=fake_llm,
            output_dir=tmp_path / "cb2",
            model="deterministic-contract-test",
            max_dev_llm_calls=2,
        )
        summ = report["summary_by_method"]
        # Even given the exact same robust policy, CRAFT's SQLite Journal survives all 7 mid-brew crashes
        # with 0 duplicate brews (30/30 = 100%), while stateless baselines duplicate brew on crashes (23/30 = 76.7%, p=0.0156)
        assert summ["craft_full"]["passed"] == 30
        assert summ["craft_full"]["duplicate_non_idempotent_count"] == 0
        assert summ["craft_full"]["false_actuation_count"] == 0
        assert summ["craft_full"]["recorded_shadow_passed"] == 4
        assert summ["aspire_protocol"]["passed"] == 23
        assert report["paired_mcnemar_craft_vs"]["aspire_protocol"]["p_value"] < 0.05
        assert report["paired_mcnemar_craft_vs"]["aspire_protocol"]["delta_pp"] >= 20.0
        assert (tmp_path / "cb2" / "cafebench_v2_report.json").is_file()
        assert (tmp_path / "cb2" / "cafebench_v2_report.html").is_file()
    finally:
        driver.close()
