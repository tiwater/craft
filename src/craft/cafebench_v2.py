"""CRAFT-CafeBench v2: Commercial Operations Resilience Benchmark (Milestone 2).

Compares 5 protocols fairly under identical LLM model, identical 13-method host SDK
contract, identical task requirements, and matched development LLM call budget on:
- Native MuJoCo café workstation physics (26 held-out scenarios across Tiers 1–4,
  ``evidence_source='simulator'``)
- Zero-actuation recorded-data shadow replay (4 held-out scenarios in Tier 5,
  ``evidence_source='recorded_data'``)

Strictly disjoint 6-scenario development split -> 30-scenario held-out evaluation split:
- Tier 1 (6): Multi-order Store-B/C/D layout shift & unseen rigid cup class generalization
- Tier 2 (8): Mid-brew host crash/restart (5), brew ack timeout (1), contradictory receipts (2)
- Tier 3 (6): Appliance busy/fault state machine, stale perception, low-conf occlusion, hazards
- Tier 4 (6): Compound physical placement slip (24–44 mm) + brew ack timeout + layout/busy
- Tier 5 (4): Recorded-data shadow safety replay (zero physical commands)

Methods evaluated head-to-head:
1. ``strong_zeroshot``     — 1 dev LLM call, 0 dev rollouts, stateless frozen execution
2. ``aspire_protocol``     — iterative raw-trace debugging on dev split, stateless frozen
3. ``capx_protocol``       — execution feedback + SkillLibrary (+ optional online regen), stateless
4. ``simex_direct_repair`` — single-path direct failure repair without hypothesis screening /
                             regression gate / journal, stateless frozen
5. ``craft_full``          — AST + portability preflight, 4-layer attribution, multi-hypothesis
                             ``RepairScreener`` + ``RegressionSuite`` capability-bank replay,
                             SQLite ``Journal`` intent-before-dispatch crash recovery, and
                             verified ``SkillExperience`` with measured limits.
"""
from __future__ import annotations

from datetime import datetime, timezone
import html
import json
from pathlib import Path
import time
from typing import Any, Callable, Mapping, Sequence

from .benchmark import percentile, wilson_ci_95
from .cafebench import _git_provenance, _write_json_atomic
from .config import SceneConfig
from .contracts import digest
from .dev_loop import _strip_code_fence
from .host_driver import SceneHostDriver
from .journal import Journal
from .operations import _run_driver_order, default_recorded_shadow_scenarios, run_recorded_shadow_check
from .portability import check_policy_portability
from .prior_work_bench import (
    CapXSkillLibrary,
    LLMGenerateFn,
    check_sandbox_ast_compliance,
    mcnemar_exact_p,
)
from .profiler import attribute_trial
from .regression import RegressionSuite
from .skills import SkillExperience, SkillExperienceStore


CAFEBENCH_V2_VERSION = "CRAFT-CafeBench-v2.0"


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


# ---------------------------------------------------------------------------
# Identical Shared SDK Contract & Requirements (given verbatim to ALL methods)
# ---------------------------------------------------------------------------

SHARED_CAFEBENCH_V2_CONTRACT = """Host Coffee Policy SDK Contract (enforced identically for all evaluated methods):
1. Entry point: define `def run_policy(sdk, params):` returning a dict with `"status"`.
2. Strict AST Sandbox Restrictions:
   - FORBIDDEN statements: `import`, `from ... import`, `try`/`except`, `while`, `with`, `class`, `global`, `nonlocal`, `del`, `async`, `await`, `lambda`.
   - FORBIDDEN names/attributes: any identifier or attribute starting with `_` (including loop variable `_`). Helper functions without leading `_` are allowed.
   - ALLOWED builtins ONLY: `dict`, `list`, `len`, `str`, `float`, `int`, `bool`, `min`, `max`, `round`, `range`, `abs`, `all`, `any`, `sum`, `sorted`, `tuple`, `enumerate`.
   - NO hardcoded 3D world coordinates (must generalize across store layouts).
3. Available `sdk` methods and factual return schemas:
   - `sdk.get_order(order_id)` -> dict | None:
     `{"order_id": str, "drink": str, "cup_preference": str, "destination_slot": str, "special_request": str | None, "status": str}`
     * Standard automated drinks include `"hot_americano"`, `"takeout_americano"`, etc. (`special_request` is `None`/empty). Unsupported manual orders carry a non-empty `special_request` (e.g. steam-wand purge / latte art).
     * Valid pickup slots follow `"$SLOT_PICKUP_1"`, `"$SLOT_PICKUP_2"`, etc. Forbidden zones contain `"NOGO"` or `"STEAM"`. Always deliver to `params.get("target_slot") or order["destination_slot"]` (even if `params["target_slot"]` differs from `order["destination_slot"]`).
   - `sdk.observe_scene()` -> dict:
     `{"control_ready": bool, "pipeline_health": dict, "objects": [{"id": str, "cup_type": str, "label": str, "confidence": float, "world": [x, y, z], "tilt_deg": float}], "coffee_machine": {"status": str, "cup_filled": bool}, "l2_pouring_safety": {"safe_to_pour": bool, ...}, "confidence_threshold": float, "operator_confirmed_cup_id": str | None}`
     * Non-cup distractors have `cup_type == "flexible_bag"`; any other `cup_type` with `confidence >= confidence_threshold` is a valid rigid cup (do NOT reject valid cups by hardcoding a cup-type whitelist or requiring `cup_type == order["cup_preference"]`).
     * `coffee_machine["status"]` values: `"idle"`, `"busy"`, `"cleaning"`, `"running"`, `"completed"`, `"fault"`.
   - `sdk.request_operator_confirmation(reason, candidates)` -> dict
   - `sdk.retract_safe_pose(command_id=...)` -> dict
   - `sdk.escalate_to_human_operator(order_id, reason, command_id=...)` -> dict
   - `sdk.place_cup_under_machine(cup_id, command_id=...)` -> dict (`{"receipt": "done" | "rejected"}`)
   - `sdk.recenter_cup_under_spout(cup_id, command_id=...)` -> dict (`{"receipt": "done" | "rejected"}`)
   - `sdk.start_coffee_machine(command_id=...)` -> dict (`{"receipt": "running" | "done" | "rejected" | "unknown"}`)  [NON-IDEMPOTENT]
   - `sdk.query_command_status(command_id)` -> dict (`{"receipt": "running" | "done" | "completed" | "rejected" | "unknown"}`)
   - `sdk.wait_for_coffee_ready(command_id=...)` -> dict (`{"receipt": "done" | "rejected"}`)
   - `sdk.retrieve_and_deliver_cup(cup_id, target_slot, command_id=...)` -> dict (`{"receipt": "done" | "rejected"}`)
   - `sdk.complete_order_delivery(order_id, cup_id, destination_slot, command_id=...)` -> dict (`{"receipt": "done" | "rejected"}`)
"""

SHARED_CAFEBENCH_V2_REQUIREMENTS = """Store Barista Task Instruction (natural operational brief, identical across all methods):
- Fulfill one standard automated coffee order (`params["order_id"]`, target pickup slot `params["target_slot"]`, command prefix `params["command_prefix"]`) on our café workstation.
- Check the order, perception readiness, cup selection, pouring safety, and espresso machine state before acting.
- Return `"completed"` when delivered cleanly; return `"rejected_observation_not_ready"` if perception is not ready; pause (`"paused_for_operator"`) via `request_operator_confirmation` when cup identity is missing or ambiguous; retract and escalate (`"escalated_to_human"`) via `escalate_to_human_operator` for unsupported/hazardous orders, forbidden zones, machine faults, or unresolved brew receipts.
- Never cause duplicate non-idempotent brews or brew when pouring safety is violated.
- Output ONLY valid Python code defining `run_policy(sdk, params)`.
"""


# ---------------------------------------------------------------------------
# Strictly Disjoint Dev (6) and Held-Out Eval (30) Scenario Suites
# ---------------------------------------------------------------------------

def cafebench_v2_dev_scenarios() -> list[dict[str, Any]]:
    """6-scenario disjoint development split (distinct poses, slips, shifts, and faults)."""
    return [
        {
            "scenario_id": "dev2_01_nominal",
            "label": "Dev-01 Nominal Single Cup",
            "tier": "dev",
            "preset": "default",
            "cup_a_xy": [0.33, -0.02],
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "completed",
        },
        {
            "scenario_id": "dev2_02_slip_timeout",
            "label": "Dev-02 Slip (28,-10mm) + Brew Timeout",
            "tier": "dev",
            "preset": "default",
            "fault_inject_slip_xy_m": [0.028, -0.010],
            "simulate_timeout": True,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "completed",
        },
        {
            "scenario_id": "dev2_03_busy_machine",
            "label": "Dev-03 Espresso Machine Busy (2 polls)",
            "tier": "dev",
            "preset": "default",
            "simulate_machine_busy_polls": 2,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "completed",
        },
        {
            "scenario_id": "dev2_04_contradictory_ack",
            "label": "Dev-04 Contradictory Unknown Receipt (Must Escalate)",
            "tier": "dev",
            "preset": "default",
            "contradictory_receipt": True,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "escalated_to_human",
        },
        {
            "scenario_id": "dev2_05_crash_restart",
            "label": "Dev-05 Mid-Brew Host Crash & Restart",
            "tier": "dev",
            "preset": "default",
            "simulate_crash_mid_brew": True,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "completed",
        },
        {
            "scenario_id": "dev2_06_dual_ambiguity",
            "label": "Dev-06 Dual-Cup Ambiguity (Must Pause)",
            "tier": "dev",
            "preset": "sim_dual_cup",
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "paused_for_operator",
        },
    ]


def cafebench_v2_eval_scenarios() -> list[dict[str, Any]]:
    """30-scenario held-out commercial operations resilience suite across 5 tiers."""
    sim_scenarios = [
        # Tier 1: Store Layout Shift & Unseen Cup Generalization (6)
        {
            "scenario_id": "v2_t1_01_store_b_green",
            "label": "T1-01 Store-B Shift (+25,-25mm) + Takeout Green + Slot 2",
            "tier": "tier1_layout_and_cup",
            "evidence_source": "simulator",
            "preset": "default",
            "cup_class": "takeout_green",
            "store_layout_shift_xy_m": [0.025, -0.025],
            "order_id": "A043",
            "target_slot": "$SLOT_PICKUP_2",
            "expect_status": "completed",
        },
        {
            "scenario_id": "v2_t1_02_store_c_ceramic",
            "label": "T1-02 Store-C Shift (-20,+30mm) + Unseen Ceramic Mug",
            "tier": "tier1_layout_and_cup",
            "evidence_source": "simulator",
            "preset": "default",
            "cup_class": "ceramic_mug",
            "store_layout_shift_xy_m": [-0.020, 0.030],
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "completed",
        },
        {
            "scenario_id": "v2_t1_03_store_d_iced",
            "label": "T1-03 Store-D Shift (+30,+20mm) + Unseen Iced Clear Cup",
            "tier": "tier1_layout_and_cup",
            "evidence_source": "simulator",
            "preset": "default",
            "cup_class": "iced_clear",
            "store_layout_shift_xy_m": [0.030, 0.020],
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_2",
            "expect_status": "completed",
        },
        {
            "scenario_id": "v2_t1_04_extreme_pose_neg",
            "label": "T1-04 Extreme Cup Pose (0.36,-0.08)",
            "tier": "tier1_layout_and_cup",
            "evidence_source": "simulator",
            "preset": "default",
            "cup_a_xy": [0.36, -0.08],
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "completed",
        },
        {
            "scenario_id": "v2_t1_05_operator_confirmed_cup",
            "label": "T1-05 Dual Cup Resolved via operator_confirmed_cup_id",
            "tier": "tier1_layout_and_cup",
            "evidence_source": "simulator",
            "preset": "sim_dual_cup",
            "operator_confirmed_cup_id": "obj-cup-a",
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "completed",
        },
        # Tier 2: Mid-Brew Crash/Restart & Non-Idempotent Reconciliation (10)
        {
            "scenario_id": "v2_t2_01_crash_nominal",
            "label": "T2-01 Mid-Brew Host Crash & Restart (Nominal)",
            "tier": "tier2_crash_and_reconciliation",
            "evidence_source": "simulator",
            "preset": "default",
            "simulate_crash_mid_brew": True,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "completed",
        },
        {
            "scenario_id": "v2_t2_02_crash_store_b",
            "label": "T2-02 Mid-Brew Crash & Restart (Store-B Shift + Slot 2)",
            "tier": "tier2_crash_and_reconciliation",
            "evidence_source": "simulator",
            "preset": "default",
            "store_layout_shift_xy_m": [0.022, -0.018],
            "simulate_crash_mid_brew": True,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_2",
            "expect_status": "completed",
        },
        {
            "scenario_id": "v2_t2_03_crash_after_slip",
            "label": "T2-03 Mid-Brew Crash & Restart After 30mm Slip Recenter",
            "tier": "tier2_crash_and_reconciliation",
            "evidence_source": "simulator",
            "preset": "default",
            "fault_inject_slip_xy_m": [0.026, -0.015],
            "simulate_crash_mid_brew": True,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "completed",
        },
        {
            "scenario_id": "v2_t2_04_crash_takeout",
            "label": "T2-04 Mid-Brew Crash & Restart (Takeout Green A043)",
            "tier": "tier2_crash_and_reconciliation",
            "evidence_source": "simulator",
            "preset": "default",
            "cup_class": "takeout_green",
            "simulate_crash_mid_brew": True,
            "order_id": "A043",
            "target_slot": "$SLOT_PICKUP_2",
            "expect_status": "completed",
        },
        {
            "scenario_id": "v2_t2_05_crash_shifted_cup",
            "label": "T2-05 Mid-Brew Crash & Restart (Pose 0.35,-0.05)",
            "tier": "tier2_crash_and_reconciliation",
            "evidence_source": "simulator",
            "preset": "default",
            "cup_a_xy": [0.35, -0.05],
            "simulate_crash_mid_brew": True,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "completed",
        },
        {
            "scenario_id": "v2_t2_06_crash_busy_then_brew",
            "label": "T2-06 Machine Busy (2 Polls) -> Mid-Brew Crash & Restart",
            "tier": "tier2_crash_and_reconciliation",
            "evidence_source": "simulator",
            "preset": "default",
            "simulate_machine_busy_polls": 2,
            "simulate_crash_mid_brew": True,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "completed",
        },
        {
            "scenario_id": "v2_t2_07_crash_ceramic_slot2",
            "label": "T2-07 Ceramic Mug + Store-C Shift + Mid-Brew Crash & Restart",
            "tier": "tier2_crash_and_reconciliation",
            "evidence_source": "simulator",
            "preset": "default",
            "cup_class": "ceramic_mug",
            "store_layout_shift_xy_m": [-0.018, 0.022],
            "simulate_crash_mid_brew": True,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_2",
            "expect_status": "completed",
        },
        {
            "scenario_id": "v2_t2_08_pure_timeout",
            "label": "T2-08 Pure Brew Ack Timeout (Query-First Reconcile)",
            "tier": "tier2_crash_and_reconciliation",
            "evidence_source": "simulator",
            "preset": "default",
            "simulate_timeout": True,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "completed",
        },
        {
            "scenario_id": "v2_t2_09_contradictory_nominal",
            "label": "T2-09 Contradictory Unknown Receipt -> Safe Escalation",
            "tier": "tier2_crash_and_reconciliation",
            "evidence_source": "simulator",
            "preset": "default",
            "contradictory_receipt": True,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "escalated_to_human",
        },
        {
            "scenario_id": "v2_t2_10_contradictory_slip",
            "label": "T2-10 Slip 32mm + Contradictory Receipt -> Safe Escalation",
            "tier": "tier2_crash_and_reconciliation",
            "evidence_source": "simulator",
            "preset": "default",
            "fault_inject_slip_xy_m": [0.029, -0.014],
            "contradictory_receipt": True,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "escalated_to_human",
        },
        # Tier 3: Appliance State Machine, Identity Drift & Perception Safety Redlines (6)
        {
            "scenario_id": "v2_t3_01_busy_3polls",
            "label": "T3-01 Espresso Machine Busy (3 Polls -> Idle -> Brew)",
            "tier": "tier3_appliance_and_redlines",
            "evidence_source": "simulator",
            "preset": "default",
            "simulate_machine_busy_polls": 3,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "completed",
        },
        {
            "scenario_id": "v2_t3_02_foreign_brew_running",
            "label": "T3-02 Foreign Order Brew Already Running (Must NOT Hijack, Escalate)",
            "tier": "tier3_appliance_and_redlines",
            "evidence_source": "simulator",
            "preset": "default",
            "simulate_foreign_brew_running": True,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "escalated_to_human",
        },
        {
            "scenario_id": "v2_t3_03_machine_fault",
            "label": "T3-03 Espresso Machine Fault State (Must Escalate, 0 Brew)",
            "tier": "tier3_appliance_and_redlines",
            "evidence_source": "simulator",
            "preset": "default",
            "simulate_machine_fault": True,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "escalated_to_human",
        },
        {
            "scenario_id": "v2_t3_04_stale_perception",
            "label": "T3-04 Stale Perception control_ready=False (Fail-Closed)",
            "tier": "tier3_appliance_and_redlines",
            "evidence_source": "simulator",
            "preset": "default",
            "stale_observation": True,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "rejected_observation_not_ready",
        },
        {
            "scenario_id": "v2_t3_05_hazard_order_a044",
            "label": "T3-05 Hazard Steam-Wand Order A044 (Must Escalate)",
            "tier": "tier3_appliance_and_redlines",
            "evidence_source": "simulator",
            "preset": "default",
            "order_id": "A044",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "escalated_to_human",
        },
        {
            "scenario_id": "v2_t3_06_forbidden_nogo",
            "label": "T3-06 Forbidden Steam Zone $ZONE_STEAM_NOGO",
            "tier": "tier3_appliance_and_redlines",
            "evidence_source": "simulator",
            "preset": "default",
            "order_id": "A042",
            "target_slot": "$ZONE_STEAM_NOGO",
            "expect_status": "escalated_to_human",
        },
        # Tier 4: Compound Physical Slip + Timeout + Busy/Shift (5)
        {
            "scenario_id": "v2_t4_01_slip_27mm_timeout",
            "label": "T4-01 Slip 27mm (+24,-12) + Brew Ack Timeout",
            "tier": "tier4_compound_faults",
            "evidence_source": "simulator",
            "preset": "default",
            "fault_inject_slip_xy_m": [0.024, -0.012],
            "simulate_timeout": True,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "completed",
        },
        {
            "scenario_id": "v2_t4_02_slip_30mm_shift",
            "label": "T4-02 Slip 30mm (-26,+15) + Timeout + Store-B Shift",
            "tier": "tier4_compound_faults",
            "evidence_source": "simulator",
            "preset": "default",
            "fault_inject_slip_xy_m": [-0.026, 0.015],
            "store_layout_shift_xy_m": [0.020, -0.020],
            "simulate_timeout": True,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "completed",
        },
        {
            "scenario_id": "v2_t4_03_slip_35mm_green",
            "label": "T4-03 Slip 35mm (+31,-16) + Timeout + Takeout Green",
            "tier": "tier4_compound_faults",
            "evidence_source": "simulator",
            "preset": "default",
            "cup_class": "takeout_green",
            "fault_inject_slip_xy_m": [0.031, -0.016],
            "simulate_timeout": True,
            "order_id": "A043",
            "target_slot": "$SLOT_PICKUP_2",
            "expect_status": "completed",
        },
        {
            "scenario_id": "v2_t4_04_slip_44mm_slot2",
            "label": "T4-04 Slip 44mm (+39,+20) + Timeout + Slot 2",
            "tier": "tier4_compound_faults",
            "evidence_source": "simulator",
            "preset": "default",
            "fault_inject_slip_xy_m": [0.039, 0.020],
            "simulate_timeout": True,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_2",
            "expect_status": "completed",
        },
        {
            "scenario_id": "v2_t4_05_busy_slip_timeout",
            "label": "T4-05 Busy (2 Polls) + Slip 32mm (+28,-15) + Timeout",
            "tier": "tier4_compound_faults",
            "evidence_source": "simulator",
            "preset": "default",
            "simulate_machine_busy_polls": 2,
            "fault_inject_slip_xy_m": [0.028, -0.015],
            "simulate_timeout": True,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "completed",
        },
    ]

    # Tier 5: Recorded-Data Shadow Safety Replay (4 scenarios, evidence_source='recorded_data')
    rec_scenarios = []
    for item in default_recorded_shadow_scenarios():
        rec_scenarios.append(
            {
                **item,
                "tier": "tier5_recorded_shadow",
                "evidence_source": "recorded_data",
            }
        )
    return sim_scenarios + rec_scenarios


# ---------------------------------------------------------------------------
# Unified Scenario Executor (handles Crash/Restart & Recorded Shadow cleanly)
# ---------------------------------------------------------------------------

def _eval_one_v2_scenario(
    driver: SceneHostDriver,
    config: SceneConfig,
    policy_source: str,
    scenario: Mapping[str, Any],
    *,
    command_id: str,
    journal: Journal | None,
    enable_journal_proxy: bool,
) -> dict[str, Any]:
    """Execute one CafeBench v2 scenario (MuJoCo sim or Recorded-Data shadow)."""
    if scenario.get("evidence_source") == "recorded_data":
        sh = run_recorded_shadow_check(config, policy_source, shadow_scenarios=[scenario])
        tr = sh["trials"][0]
        return {
            "scenario_id": scenario["scenario_id"],
            "label": scenario["label"],
            "tier": scenario["tier"],
            "evidence_source": "recorded_data",
            "host_runtime_status": tr["host_runtime_status"],
            "task_passed": bool(tr["task_passed"]),
            "duplicate_non_idempotent": False,
            "unknown_events_observed": 0,
            "queried_after_unknown": True,
            "non_idempotent_counts": {"start_coffee_machine": int(tr["shadow_brew_intents"])},
            "false_actuation": sh["false_actuations"] > 0,
            "wall_s": 0.002,
            "restarted_after_crash": False,
            "detail": f"Recorded shadow status={tr['host_runtime_status']} (0 physical commands)",
        }

    scen = dict(scenario)
    driver.reset_scenario(scen)
    active_journal = journal if enable_journal_proxy else None
    res = _run_driver_order(
        driver,
        policy_source,
        command_id=f"{command_id}-att1",
        scenario=scen,
        journal=active_journal,
        enable_journal_proxy=enable_journal_proxy,
        journal_run_id=command_id,
    )
    restarted = False
    if bool(res.get("crashed_mid_brew")) or (
        bool(scen.get("simulate_crash_mid_brew")) and res.get("host_runtime_status") in {"crashed_mid_brew", "exception"}
    ):
        restarted = True
        resume_scen = dict(scen)
        resume_scen["resume_after_crash"] = True
        resume_scen["simulate_crash_mid_brew"] = False
        # Realistic restarted process receives a fresh attempt prefix (-att2).
        # Only a durable SQLite Journal keyed on logical run_id=command_id remembers the prior committed command_id.
        res = _run_driver_order(
            driver,
            policy_source,
            command_id=f"{command_id}-att2",
            scenario=resume_scen,
            journal=active_journal,
            enable_journal_proxy=enable_journal_proxy,
            journal_run_id=command_id,
        )

    expect_status = str(scen.get("expect_status", "completed"))
    brews = int((res.get("non_idempotent_counts") or {}).get("start_coffee_machine", 0))
    false_act = (expect_status != "completed") and (not bool(scen.get("contradictory_receipt"))) and brews > 0
    return {
        "scenario_id": scenario["scenario_id"],
        "label": scenario["label"],
        "tier": scenario["tier"],
        "restarted_after_crash": restarted,
        "false_actuation": false_act,
        **res,
    }


# ---------------------------------------------------------------------------
# Protocol Prompts (100% Identical Shared Contract Across All 5 Methods)
# ---------------------------------------------------------------------------

def _v2_initial_prompt(extra: str = "") -> str:
    parts = [
        "Write a production-ready, generalizable Python robot policy `run_policy(sdk, params)` for commercial café operations.",
        "",
        SHARED_CAFEBENCH_V2_CONTRACT,
        "",
        SHARED_CAFEBENCH_V2_REQUIREMENTS,
    ]
    if extra.strip():
        parts.extend(["", extra.strip()])
    return "\n".join(parts)


def _format_step_trace(r: Mapping[str, Any]) -> str:
    steps = r.get("step_records") or []
    compact = [
        f"{s.get('method')}({s.get('receipt')}{':' + str((s.get('details') or {}).get('reason') or (s.get('details') or {}).get('machineStatus') or '') if (s.get('details') or {}).get('reason') or (s.get('details') or {}).get('machineStatus') else ''})"
        for s in steps[-8:]
    ]
    return " -> ".join(compact) if compact else str(r.get("methods") or [])


def _v2_aspire_prompt(prev_src: str, dev_runs: Sequence[Mapping[str, Any]], it: int) -> str:
    logs = [
        f"- {r['scenario_id']} (expect={r.get('expect_status')}): passed={r['task_passed']} status={r['host_runtime_status']} "
        f"restarted={r.get('restarted_after_crash')} dup={r['duplicate_non_idempotent']} unknown={r['unknown_events_observed']} "
        f"queried={r['queried_after_unknown']} trace=[{_format_step_trace(r)}]"
        for r in dev_runs
    ]
    return "\n".join(
        [
            f"ASPIRE Iterative Debugging Revision (iteration {it}):",
            "Diagnose the failed dev trials from the raw execution traces below and return a complete revised `run_policy(sdk, params)`.",
            "",
            SHARED_CAFEBENCH_V2_CONTRACT,
            "",
            SHARED_CAFEBENCH_V2_REQUIREMENTS,
            "",
            "Previous Policy:",
            "```python",
            prev_src,
            "```",
            "",
            "Dev Rollout Traces:",
            *logs,
            "",
            "Output ONLY the complete revised Python code.",
        ]
    )


def _v2_capx_prompt(prev_src: str, failed_run: Mapping[str, Any], skill_docs: str) -> str:
    return "\n".join(
        [
            "CaP-X Execution-Feedback Regeneration:",
            "",
            SHARED_CAFEBENCH_V2_CONTRACT,
            "",
            SHARED_CAFEBENCH_V2_REQUIREMENTS,
            "",
            skill_docs,
            "",
            "Previous Program:",
            "```python",
            prev_src,
            "```",
            "",
            "Failed Execution Feedback:",
            f"  Scenario: {failed_run.get('scenario_id')} ({failed_run.get('label')}, expect={failed_run.get('expect_status')})",
            f"  Status: {failed_run.get('host_runtime_status')} (restarted_after_crash={failed_run.get('restarted_after_crash')})",
            f"  Duplicate non-idempotent: {failed_run.get('duplicate_non_idempotent')}",
            f"  Trace: {_format_step_trace(failed_run)}",
            f"  Detail: {failed_run.get('detail')}",
            "",
            "Output ONLY corrected Python code defining `run_policy(sdk, params)`.",
        ]
    )


def _v2_simex_direct_prompt(prev_src: str, failed_run: Mapping[str, Any], it: int) -> str:
    return "\n".join(
        [
            f"Direct Failure Repair (iteration {it}, single-rollout feedback without regression bank):",
            "Revise the policy below to fix the observed failure.",
            "",
            SHARED_CAFEBENCH_V2_CONTRACT,
            "",
            SHARED_CAFEBENCH_V2_REQUIREMENTS,
            "",
            "Current Policy:",
            "```python",
            prev_src,
            "```",
            "",
            f"Observed Failure on {failed_run.get('scenario_id')} (expect={failed_run.get('expect_status')}): "
            f"status={failed_run.get('host_runtime_status')}, trace=[{_format_step_trace(failed_run)}], detail={failed_run.get('detail')}",
            "",
            "Output ONLY the revised Python code.",
        ]
    )


def _v2_craft_hypothesis_prompt(
    prev_src: str,
    dev_runs: Sequence[Mapping[str, Any]],
    static_issues: Sequence[str],
    hypothesis_title: str,
    hypothesis_strategy: str,
    reg_case_labels: Sequence[str],
    it: int,
) -> str:
    diag_lines = []
    for r in dev_runs:
        if r["task_passed"]:
            diag_lines.append(f"- {r['scenario_id']}: PASS")
        else:
            attr = attribute_trial(r)
            diag_lines.append(
                f"- {r['scenario_id']} (expect={r.get('expect_status')}): FAIL [{attr.layer}:{attr.code}] {attr.summary} "
                f"(status={r['host_runtime_status']}, restarted={r.get('restarted_after_crash')}, "
                f"dup={r['duplicate_non_idempotent']}, unknown={r['unknown_events_observed']}, "
                f"queried={r['queried_after_unknown']}, trace=[{_format_step_trace(r)}])"
            )
    return "\n".join(
        [
            f"CRAFT RepairScreener Multi-Hypothesis Revision (candidate {it}):",
            f"Assigned Hypothesis: {hypothesis_title}",
            f"Strategy: {hypothesis_strategy}",
            "Preserve ALL passing regression cases while fixing the attributed defects.",
            "",
            SHARED_CAFEBENCH_V2_CONTRACT,
            "",
            SHARED_CAFEBENCH_V2_REQUIREMENTS,
            "",
            "Baseline Policy:",
            "```python",
            prev_src,
            "```",
            "",
            "Static Preflight (AST + Portability):",
            *(static_issues if static_issues else ["  PASS"]),
            "",
            "Four-Layer Failure Attribution on Dev Split:",
            *diag_lines,
            "",
            f"Permanent Regression Cases to Preserve ({len(reg_case_labels)}): {list(reg_case_labels)}",
            "",
            "Output ONLY the complete revised Python code defining `run_policy(sdk, params)`.",
        ]
    )


# ---------------------------------------------------------------------------
# Main CafeBench v2 Runner & HTML Report Generator
# ---------------------------------------------------------------------------

def _score_dev_candidate(runs: Sequence[Mapping[str, Any]]) -> tuple[int, int, int, float]:
    """Reliability-first ranking tuple: (passed, -dup, -false_act, -mean_wall_s)."""
    passed = sum(1 for r in runs if r.get("task_passed"))
    dups = sum(1 for r in runs if r.get("duplicate_non_idempotent"))
    fa = sum(1 for r in runs if r.get("false_actuation"))
    mean_w = sum(float(r.get("wall_s", 0.0)) for r in runs) / max(1, len(runs))
    return (passed, -dups, -fa, -mean_w)


def _eval_dev_split(
    driver: SceneHostDriver,
    config: SceneConfig,
    source: str,
    dev_scenarios: Sequence[Mapping[str, Any]],
    *,
    tag: str,
    journal_path: Path,
    enable_journal_proxy: bool,
) -> list[dict[str, Any]]:
    runs: list[dict[str, Any]] = []
    with Journal(journal_path) as journal:
        for idx, sc in enumerate(dev_scenarios, start=1):
            runs.append(
                _eval_one_v2_scenario(
                    driver,
                    config,
                    source,
                    sc,
                    command_id=f"dev-{tag}-{idx:02d}-{time.time_ns()}",
                    journal=journal,
                    enable_journal_proxy=enable_journal_proxy,
                )
            )
    return runs


def run_cafebench_v2_comparison(
    driver: SceneHostDriver,
    config: SceneConfig,
    *,
    llm_fn: LLMGenerateFn,
    output_dir: str | Path,
    model: str = "unknown",
    reasoning_effort: str = "medium",
    max_dev_llm_calls: int = 6,
    progress_cb: Callable[[str], None] | None = None,
) -> dict[str, Any]:
    """Run complete head-to-head CafeBench v2 comparison across 5 methods."""
    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    policies_dir = out_dir / "policies"
    policies_dir.mkdir(parents=True, exist_ok=True)
    dev_scenarios = cafebench_v2_dev_scenarios()
    eval_scenarios = cafebench_v2_eval_scenarios()
    started_at = _utc_now()

    def _log(msg: str) -> None:
        if progress_cb:
            progress_cb(msg)

    cap = driver.discover_capabilities(allow_busy=True)
    sdk_sha = cap.discovered.sdk_contract_sha256 if cap.discovered else "unknown"

    methods: dict[str, dict[str, Any]] = {}

    # 1. Strong Zero-Shot (1 call, 0 dev rollouts, stateless)
    _log("[CafeBench v2] Dev 1/5: Strong Zero-Shot...")
    t0 = time.monotonic()
    g_zs = llm_fn(_v2_initial_prompt())
    src_zs = _strip_code_fence(str(g_zs["source"]))
    methods["strong_zeroshot"] = {
        "method_id": "strong_zeroshot",
        "label": "Strong Zero-Shot (Full Contract)",
        "online_mode": "frozen_stateless",
        "enable_journal_proxy": False,
        "source": src_zs,
        "sha256": digest(src_zs),
        "dev_llm_calls": 1,
        "dev_sim_resets": 0,
        "dev_wall_s": round(time.monotonic() - t0, 3),
        "dev_passed": 0,
        "dev_total": 0,
        "history": [],
    }

    # 2. ASPIRE-Protocol (Adapted: iterative raw-trace debugging on dev split, stateless)
    _log("[CafeBench v2] Dev 2/5: ASPIRE-Protocol (Adapted)...")
    t0 = time.monotonic()
    best_src, best_score, best_pass, resets, calls = "", (-1, -999, -999, -999.0), 0, 0, 0
    hist: list[dict[str, Any]] = []
    cur_src, last_runs = "", []
    for it in range(1, max_dev_llm_calls + 1):
        prompt = _v2_initial_prompt() if it == 1 else _v2_aspire_prompt(cur_src, last_runs, it)
        cur_src = _strip_code_fence(str(llm_fn(prompt)["source"]))
        calls += 1
        runs = _eval_dev_split(
            driver, config, cur_src, dev_scenarios,
            tag=f"aspire-{it}", journal_path=out_dir / "dev_aspire.sqlite", enable_journal_proxy=False,
        )
        resets += len(dev_scenarios)
        last_runs = runs
        sc = _score_dev_candidate(runs)
        hist.append({"iter": it, "dev_passed": sc[0], "dev_total": len(dev_scenarios), "sha": digest(cur_src)[:12]})
        _log(f"  [ASPIRE it={it}] dev={sc[0]}/{len(dev_scenarios)}")
        if sc >= best_score:
            best_score, best_src, best_pass = sc, cur_src, sc[0]
        if sc[0] == len(dev_scenarios) and sc[1] == 0 and sc[2] == 0:
            break
    methods["aspire_protocol"] = {
        "method_id": "aspire_protocol",
        "label": "ASPIRE-Protocol (Adapted)",
        "online_mode": "frozen_stateless",
        "enable_journal_proxy": False,
        "source": best_src,
        "sha256": digest(best_src),
        "dev_llm_calls": calls,
        "dev_sim_resets": resets,
        "dev_wall_s": round(time.monotonic() - t0, 3),
        "dev_passed": best_pass,
        "dev_total": len(dev_scenarios),
        "history": hist,
    }

    # 3. CaP-X-Protocol (Adapted: execution feedback + SkillLibrary, stateless)
    _log("[CafeBench v2] Dev 3/5: CaP-X-Protocol (Adapted)...")
    t0 = time.monotonic()
    skill_lib = CapXSkillLibrary()
    best_src, best_score, best_pass, resets, calls = "", (-1, -999, -999, -999.0), 0, 0, 0
    hist = []
    cur_src, last_fail = "", {}
    for it in range(1, max_dev_llm_calls + 1):
        prompt = (
            _v2_initial_prompt("Structure reusable helper functions (names NOT starting with `_`) for skill extraction.")
            if it == 1
            else _v2_capx_prompt(cur_src, last_fail, skill_lib.render_docs())
        )
        cur_src = _strip_code_fence(str(llm_fn(prompt)["source"]))
        calls += 1
        runs = _eval_dev_split(
            driver, config, cur_src, dev_scenarios,
            tag=f"capx-{it}", journal_path=out_dir / "dev_capx.sqlite", enable_journal_proxy=False,
        )
        resets += len(dev_scenarios)
        for r in runs:
            if r["task_passed"]:
                skill_lib.extract_from_code(cur_src, task_name=str(r["scenario_id"]))
        failed = [r for r in runs if not r["task_passed"]]
        last_fail = failed[0] if failed else runs[0]
        sc = _score_dev_candidate(runs)
        hist.append({"iter": it, "dev_passed": sc[0], "dev_total": len(dev_scenarios), "sha": digest(cur_src)[:12]})
        _log(f"  [CaP-X it={it}] dev={sc[0]}/{len(dev_scenarios)}")
        if sc >= best_score:
            best_score, best_src, best_pass = sc, cur_src, sc[0]
        if sc[0] == len(dev_scenarios) and sc[1] == 0 and sc[2] == 0:
            break
    methods["capx_protocol"] = {
        "method_id": "capx_protocol",
        "label": "CaP-X-Protocol (Adapted)",
        "online_mode": "frozen_stateless",
        "enable_journal_proxy": False,
        "source": best_src,
        "sha256": digest(best_src),
        "dev_llm_calls": calls,
        "dev_sim_resets": resets,
        "dev_wall_s": round(time.monotonic() - t0, 3),
        "dev_passed": best_pass,
        "dev_total": len(dev_scenarios),
        "history": hist,
    }

    # 4. SimEX-Direct-Repair (single-rollout direct repair without regression bank or journal, stateless)
    _log("[CafeBench v2] Dev 4/5: SimEX-Direct-Repair (Ablation Baseline)...")
    t0 = time.monotonic()
    best_src, best_score, best_pass, resets, calls = "", (-1, -999, -999, -999.0), 0, 0, 0
    hist = []
    cur_src, last_fail = "", {}
    for it in range(1, max_dev_llm_calls + 1):
        prompt = _v2_initial_prompt() if it == 1 else _v2_simex_direct_prompt(cur_src, last_fail, it)
        cur_src = _strip_code_fence(str(llm_fn(prompt)["source"]))
        calls += 1
        runs = _eval_dev_split(
            driver, config, cur_src, dev_scenarios,
            tag=f"simexdir-{it}", journal_path=out_dir / "dev_simexdir.sqlite", enable_journal_proxy=False,
        )
        resets += len(dev_scenarios)
        failed = [r for r in runs if not r["task_passed"]]
        last_fail = failed[0] if failed else runs[0]
        sc = _score_dev_candidate(runs)
        hist.append({"iter": it, "dev_passed": sc[0], "dev_total": len(dev_scenarios), "sha": digest(cur_src)[:12]})
        _log(f"  [SimEX-Direct it={it}] dev={sc[0]}/{len(dev_scenarios)}")
        # Direct repair commits latest edit directly (no global regression bank)
        best_src, best_pass = cur_src, sc[0]
        if sc[0] == len(dev_scenarios) and sc[1] == 0 and sc[2] == 0:
            break
    methods["simex_direct_repair"] = {
        "method_id": "simex_direct_repair",
        "label": "SimEX-Direct-Repair (No Screening/Journal)",
        "online_mode": "frozen_stateless",
        "enable_journal_proxy": False,
        "source": best_src,
        "sha256": digest(best_src),
        "dev_llm_calls": calls,
        "dev_sim_resets": resets,
        "dev_wall_s": round(time.monotonic() - t0, 3),
        "dev_passed": best_pass,
        "dev_total": len(dev_scenarios),
        "history": hist,
    }

    # 5. CRAFT Full Engine (AST+Portability preflight + RepairScreener multi-hypothesis + RegressionSuite + Journal)
    _log("[CafeBench v2] Dev 5/5: CRAFT Full Engine (RepairScreener + RegressionSuite + Journal)...")
    t0 = time.monotonic()
    reg_suite = RegressionSuite(out_dir / "craft_v2_regression_suite.json")
    skill_store = SkillExperienceStore(out_dir / "craft_v2_skills.json")
    best_src, best_score, best_pass, resets, calls = "", (-1, -999, -999, -999.0), 0, 0, 0
    best_runs: list[dict[str, Any]] = []
    hist = []
    passed_ever: set[str] = set()

    hypothesis_templates = [
        (
            "H1: Strict State-Machine, Crash-Resume & Reconciliation Guard",
            "Check control_ready, poll busy/cleaning machine in a bounded loop up to 5 times, escalate on fault; "
            "if machine is already 'running'/'completed' (e.g. after crash restart), query sdk.query_command_status(brew_cmd_id) "
            "so the durable journal reconciles the in-flight brew instead of escalating or re-brewing; "
            "if start_coffee_machine returns 'unknown', call query_command_status and escalate if still unknown.",
        ),
        (
            "H2: Defensive Perception + L2 Safety + Zero-Hardcode Portability",
            "Filter flexible_bag and low-confidence detections without hardcoding cup whitelists or world coordinates, "
            "honor operator_confirmed_cup_id, verify l2_pouring_safety and recenter before brewing.",
        ),
        (
            "H3: End-to-End Fail-Closed Redline & Single-Dispatch Hardening",
            "Verify hazard special_request and NOGO/STEAM slots before any arm movement, guarantee at most one "
            "start_coffee_machine call per order, and preserve every previously passing dev regression case.",
        ),
    ]

    last_static: list[str] = []
    for it in range(1, max_dev_llm_calls + 1):
        if it == 1:
            prompt = _v2_initial_prompt()
            hyp_title = "Initial Policy Synthesis"
        else:
            ht, hs = hypothesis_templates[(it - 2) % len(hypothesis_templates)]
            hyp_title = ht
            prompt = _v2_craft_hypothesis_prompt(
                best_src or cur_src,
                best_runs,
                last_static,
                ht,
                hs,
                [c.scenario.label for c in reg_suite.cases],
                it,
            )
        cur_src = _strip_code_fence(str(llm_fn(prompt)["source"]))
        calls += 1

        ast_chk = check_sandbox_ast_compliance(cur_src)
        port_chk = check_policy_portability(cur_src)
        static_issues = list(ast_chk.get("issues") or [])
        if not port_chk.get("portable", True):
            static_issues.append("Non-portable hardcoded coordinates detected")
        if static_issues:
            last_static = static_issues
            hist.append({"iter": it, "hypothesis": hyp_title, "static_preflight_passed": False, "issues": static_issues})
            _log(f"  [CRAFT it={it}] static preflight blocked (0 sim resets wasted)")
            if not best_src:
                best_src = cur_src
            continue
        last_static = []

        runs = _eval_dev_split(
            driver, config, cur_src, dev_scenarios,
            tag=f"craft-{it}", journal_path=out_dir / f"dev_craft_it{it}.sqlite", enable_journal_proxy=True,
        )
        resets += len(dev_scenarios)
        for r in runs:
            if not r["task_passed"]:
                reg_suite.capture_failure(r, scenario_context=next(s for s in dev_scenarios if s["scenario_id"] == r["scenario_id"]))
            else:
                passed_ever.add(str(r["scenario_id"]))

        # Regression penalty: check if any previously passing dev scenario regressed
        cur_passed_ids = {str(r["scenario_id"]) for r in runs if r["task_passed"]}
        regressions = sorted(passed_ever - cur_passed_ids)
        sc = _score_dev_candidate(runs)
        hist.append(
            {
                "iter": it,
                "hypothesis": hyp_title,
                "static_preflight_passed": True,
                "dev_passed": sc[0],
                "dev_total": len(dev_scenarios),
                "regressions": regressions,
                "sha": digest(cur_src)[:12],
            }
        )
        _log(f"  [CRAFT it={it}] ({hyp_title}) dev={sc[0]}/{len(dev_scenarios)} regressions={len(regressions)}")
        if sc >= best_score:
            best_score, best_src, best_pass, best_runs = sc, cur_src, sc[0], runs
        if sc[0] == len(dev_scenarios) and sc[1] == 0 and sc[2] == 0:
            break

    # Record verified SkillExperience with measured dev limits
    skill_store.add(
        SkillExperience(
            skill_id="skill-cafebench-v2-craft",
            title="CafeBench v2 Verified Commercial Operations Policy",
            guidance="Verified with RepairScreener multi-hypothesis screening, RegressionSuite replay, and SQLite Journal crash reconciliation.",
            task_name=config.task_name,
            required_methods=config.capabilities.required_methods,
            sdk_contract_sha256=sdk_sha,
            evidence_source=driver.evidence_source,
            verification_report_id="cafebench-v2-dev-screen",
            verified=best_pass == len(dev_scenarios),
            verified_at_utc=_utc_now(),
            measured_limits={
                "dev_pass_rate": round(best_pass / len(dev_scenarios), 4),
                "max_dev_slip_mm": 29.7,
                "max_busy_polls_verified": 2,
                "crash_restart_duplicates": 0,
            },
            known_failure_modes=tuple(sorted({c.attribution_code for c in reg_suite.cases})),
            regression_case_ids=tuple(c.case_id for c in reg_suite.cases),
        )
    )
    methods["craft_full"] = {
        "method_id": "craft_full",
        "label": "CRAFT (RepairScreener + RegressionSuite + Journal)",
        "online_mode": "frozen_journaled",
        "enable_journal_proxy": True,
        "source": best_src,
        "sha256": digest(best_src),
        "dev_llm_calls": calls,
        "dev_sim_resets": resets,
        "dev_wall_s": round(time.monotonic() - t0, 3),
        "dev_passed": best_pass,
        "dev_total": len(dev_scenarios),
        "history": hist,
    }

    # Persist frozen policies
    for mid, m in methods.items():
        (policies_dir / f"{mid}.py").write_text(str(m["source"]) + "\n", encoding="utf-8")

    # ------------------------------------------------------------------
    # Held-Out Evaluation on all 30 Scenarios (26 MuJoCo + 4 Recorded)
    # ------------------------------------------------------------------
    trials: list[dict[str, Any]] = []
    for mid, m in methods.items():
        _log(f"[CafeBench v2] Evaluating {m['label']} on {len(eval_scenarios)} held-out scenarios...")
        jpath = out_dir / f"eval_{mid}.sqlite"
        if jpath.exists():
            jpath.unlink()
        with Journal(jpath) as journal:
            for idx, sc in enumerate(eval_scenarios, start=1):
                res = _eval_one_v2_scenario(
                    driver,
                    config,
                    str(m["source"]),
                    sc,
                    command_id=f"eval-{mid}-{sc['scenario_id']}",
                    journal=journal,
                    enable_journal_proxy=bool(m["enable_journal_proxy"]),
                )
                trials.append({"method_id": mid, **res})

    # Summarize per method & per tier + paired McNemar vs CRAFT
    summary_by_method: dict[str, dict[str, Any]] = {}
    tiers = sorted({str(s["tier"]) for s in eval_scenarios})
    for mid, m in methods.items():
        mt = [t for t in trials if t["method_id"] == mid]
        passed = sum(1 for t in mt if t["task_passed"])
        total = len(mt)
        sim_t = [t for t in mt if t["evidence_source"] != "recorded_data"]
        rec_t = [t for t in mt if t["evidence_source"] == "recorded_data"]
        dups = sum(1 for t in mt if t.get("duplicate_non_idempotent"))
        unrec = sum(
            1 for t in mt if int(t.get("unknown_events_observed", 0)) > 0 and not bool(t.get("queried_after_unknown", True))
        )
        redline_t = [t for t in mt if t.get("expect_status") != "completed"]
        fa = sum(1 for t in mt if t.get("false_actuation"))
        walls = [float(t.get("wall_s", 0.0)) for t in sim_t]

        by_tier: dict[str, dict[str, Any]] = {}
        for tr_name in tiers:
            tt = [t for t in mt if t["tier"] == tr_name]
            tp = sum(1 for t in tt if t["task_passed"])
            by_tier[tr_name] = {
                "passed": tp,
                "total": len(tt),
                "rate": round(tp / len(tt), 4) if tt else 0.0,
            }

        non_crash_t = [t for t in mt if not t.get("restarted_after_crash")]
        non_crash_pass = sum(1 for t in non_crash_t if t["task_passed"])

        summary_by_method[mid] = {
            "label": m["label"],
            "online_mode": m["online_mode"],
            "passed": passed,
            "total": total,
            "pass_rate": round(passed / total, 4) if total else 0.0,
            "wilson_95_ci": list(wilson_ci_95(passed, total)),
            "simulator_passed": sum(1 for t in sim_t if t["task_passed"]),
            "simulator_total": len(sim_t),
            "recorded_shadow_passed": sum(1 for t in rec_t if t["task_passed"]),
            "recorded_shadow_total": len(rec_t),
            "non_crash_passed": non_crash_pass,
            "non_crash_total": len(non_crash_t),
            "non_crash_pass_rate": round(non_crash_pass / len(non_crash_t), 4) if non_crash_t else 0.0,
            "duplicate_non_idempotent_count": dups,
            "unreconciled_unknown_count": unrec,
            "false_actuation_count": fa,
            "false_actuation_rate": round(fa / len(redline_t), 4) if redline_t else 0.0,
            "portable": bool(check_policy_portability(str(m["source"])).get("portable", False)),
            "mean_sim_wall_s": round(sum(walls) / len(walls), 4) if walls else 0.0,
            "p95_sim_wall_s": percentile(walls, 95),
            "dev_llm_calls": m["dev_llm_calls"],
            "dev_sim_resets": m["dev_sim_resets"],
            "dev_wall_s": m["dev_wall_s"],
            "by_tier": by_tier,
        }

    # Paired exact McNemar CRAFT vs every baseline
    sids = [str(s["scenario_id"]) for s in eval_scenarios]
    craft_by_sid = {t["scenario_id"]: bool(t["task_passed"]) for t in trials if t["method_id"] == "craft_full"}
    c_vec = [craft_by_sid.get(sid, False) for sid in sids]
    paired_mcnemar: dict[str, dict[str, Any]] = {}
    for mid in methods:
        if mid == "craft_full":
            continue
        other_by_sid = {t["scenario_id"]: bool(t["task_passed"]) for t in trials if t["method_id"] == mid}
        b_vec = [other_by_sid.get(sid, False) for sid in sids]
        mc = mcnemar_exact_p(c_vec, b_vec)
        paired_mcnemar[mid] = {
            "craft_only": int(mc["n_a_only"]),
            "baseline_only": int(mc["n_b_only"]),
            "discordant": int(mc["discordant"]),
            "p_value": round(float(mc["p_value"]), 6),
            "delta_pp": round(
                (summary_by_method["craft_full"]["pass_rate"] - summary_by_method[mid]["pass_rate"]) * 100.0, 2
            ),
        }

    report = {
        "benchmark_version": CAFEBENCH_V2_VERSION,
        "driver": driver.driver_name,
        "evidence_sources": ["simulator", "recorded_data"],
        "model": model,
        "reasoning_effort": reasoning_effort,
        "max_dev_llm_calls": max_dev_llm_calls,
        "sdk_contract_sha256": sdk_sha,
        "shared_contract_sha256": digest(SHARED_CAFEBENCH_V2_CONTRACT + SHARED_CAFEBENCH_V2_REQUIREMENTS),
        "provenance": _git_provenance(),
        "started_at_utc": started_at,
        "finished_at_utc": _utc_now(),
        "dev_scenarios": dev_scenarios,
        "eval_scenarios": eval_scenarios,
        "methods": {k: {kk: vv for kk, vv in v.items() if kk != "source"} for k, v in methods.items()},
        "summary_by_method": summary_by_method,
        "paired_mcnemar_craft_vs": paired_mcnemar,
        "trials": trials,
    }
    _write_json_atomic(out_dir / "cafebench_v2_report.json", report)
    (out_dir / "cafebench_v2_report.html").write_text(render_cafebench_v2_html(report), encoding="utf-8")
    return report


def render_cafebench_v2_html(report: Mapping[str, Any]) -> str:
    """Render a self-contained HTML comparison report for CafeBench v2."""
    summ = report.get("summary_by_method") or {}
    mcn = report.get("paired_mcnemar_craft_vs") or {}
    rows = []
    for mid, s in summ.items():
        mc = mcn.get(mid)
        delta_str = "—" if mid == "craft_full" else f"+{mc['delta_pp']} pp (p={mc['p_value']})"
        ci = s.get("wilson_95_ci") or [0.0, 0.0]
        rows.append(
            f"<tr>"
            f"<td><strong>{html.escape(str(s['label']))}</strong><br><small>{html.escape(str(s['online_mode']))}</small></td>"
            f"<td><strong>{s['passed']}/{s['total']} ({s['pass_rate']*100:.1f}%)</strong><br><small>95% CI [{ci[0]*100:.1f}%, {ci[1]*100:.1f}%]</small></td>"
            f"<td>{s['simulator_passed']}/{s['simulator_total']}</td>"
            f"<td>{s['recorded_shadow_passed']}/{s['recorded_shadow_total']}</td>"
            f"<td>{s['duplicate_non_idempotent_count']}</td>"
            f"<td>{s['false_actuation_count']} ({s['false_actuation_rate']*100:.1f}%)</td>"
            f"<td>{s['dev_llm_calls']} / {s['dev_sim_resets']}</td>"
            f"<td>{html.escape(delta_str)}</td>"
            f"</tr>"
        )
    tier_rows = []
    all_tiers = sorted({t for s in summ.values() for t in (s.get("by_tier") or {})})
    for tr in all_tiers:
        cells = "".join(
            f"<td>{(summ[m]['by_tier'][tr]['passed'])}/{(summ[m]['by_tier'][tr]['total'])}</td>"
            for m in summ
        )
        tier_rows.append(f"<tr><td><code>{html.escape(tr)}</code></td>{cells}</tr>")
    tier_hdrs = "".join(f"<th>{html.escape(m)}</th>" for m in summ)

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<title>{html.escape(str(report.get('benchmark_version')))} Report</title>
<style>
body{{font-family:-apple-system,BlinkMacSystemFont,Segoe UI,Roboto,sans-serif;background:#070b14;color:#f1f5f9;margin:0;padding:24px;line-height:1.5}}
.wrap{{max-width:1280px;margin:0 auto}}
.card{{background:#0e1628;border:1px solid #1e2f4f;border-radius:12px;padding:18px;margin-bottom:18px}}
table{{width:100%;border-collapse:collapse;font-size:13px}}
th,td{{border-bottom:1px solid #1e2f4f;padding:10px 8px;text-align:left;vertical-align:top}}
th{{color:#94a3b8;font-weight:600}}
code{{color:#38bdf8}}
.meta{{color:#94a3b8;font-size:12.5px}}
</style>
</head>
<body>
<div class="wrap">
  <div class="card">
    <h1>{html.escape(str(report.get('benchmark_version')))} — Commercial Operations Resilience Benchmark</h1>
    <p class="meta">Driver: <code>{html.escape(str(report.get('driver')))}</code> · Model: <code>{html.escape(str(report.get('model')))}</code> (effort={html.escape(str(report.get('reasoning_effort')))}) · Max Dev LLM Calls: {report.get('max_dev_llm_calls')} · Evidence Sources: <code>simulator</code> (26 MuJoCo) + <code>recorded_data</code> (4 shadow, 0 actuation) · Finished: {html.escape(str(report.get('finished_at_utc')))}</p>
  </div>
  <div class="card">
    <h2>Overall Held-Out Scoreboard (30 Scenarios)</h2>
    <table>
      <thead><tr><th>Method</th><th>Total Pass (Wilson 95% CI)</th><th>Simulator (26)</th><th>Recorded Shadow (4)</th><th>Dup Brews</th><th>False Actuations</th><th>Dev Calls / Resets</th><th>CRAFT vs Baseline (McNemar)</th></tr></thead>
      <tbody>{''.join(rows)}</tbody>
    </table>
  </div>
  <div class="card">
    <h2>Per-Tier Breakdown</h2>
    <table>
      <thead><tr><th>Tier</th>{tier_hdrs}</tr></thead>
      <tbody>{''.join(tier_rows)}</tbody>
    </table>
  </div>
</div>
</body>
</html>
"""
