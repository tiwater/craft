"""True Sim-to-Sim 5-Trial Adaptation Benchmark (SimEX Stage-2 Parity & Beyond).

Compares four adaptation protocols under an identical starting policy, identical
public 13-method host SDK, identical LLM backend, and a strict **5-trial target-
environment adaptation budget** (mirroring SimEX Section 2.4):

- **Sandbox Simulator (``sandbox_driver``)**: Nominal, noise-free simulation where
  unseen target disturbances (slip, sensor noise, domain randomization, timeouts,
  mid-brew crashes) are NOT pre-encoded.
- **Target Deployment Environment (``target_driver``)**: Physics-mismatched
  deployment host with CMOS/RGB-D sensor noise, cup mass/friction domain
  randomization, unseen placement slip, layout shifts, unseen cup classes,
  brew ack timeouts, and mid-brew process crashes.

Protocols evaluated head-to-head:
1. ``zeroshot_sim2sim`` — Zero target adaptation trials; deploys nominal sandbox policy.
2. ``direct_repair`` — Direct LLM code edits from target failures (no sandbox calibration,
   no multi-hypothesis screening, no regression bank, no durable Journal).
3. ``simex_protocol`` — SimEX Stage-2 (Adapted): target failure -> closed-loop sandbox
   parameter calibration -> multi-hypothesis screening + capability-bank replay ->
   deploy screened candidate (stateless execution; no SQLite Journal or recorded shadow gate).
4. ``craft_full`` — Full TRAX: 4-layer attribution guard -> closed-loop sandbox parameter
   calibration -> multi-hypothesis ``RepairScreener`` + ``RegressionSuite`` + zero-actuation
   ``recorded_data`` shadow check + durable SQLite ``Journal`` crash/restart reconciliation.
"""
from __future__ import annotations

from datetime import datetime, timezone
import html
import json
from pathlib import Path
import time
from typing import Any, Callable, Mapping, Sequence

from trax.benchmark import wilson_ci_95
from .cafebench import _git_provenance, _write_json_atomic
from .cafebench_v2 import SHARED_CAFEBENCH_V2_CONTRACT, SHARED_CAFEBENCH_V2_REQUIREMENTS
from trax.config import SceneConfig
from trax.contracts import digest
from trax.dev_loop import _strip_code_fence
from trax.host_driver import SceneHostDriver
from trax.journal import Journal
from trax.operations import _run_driver_order, run_recorded_shadow_check
from .prior_work_bench import LLMGenerateFn, check_sandbox_ast_compliance, mcnemar_exact_p
from trax.profiler import attribute_trial
from trax.regression import RegressionSuite
from trax.repair_screener import RepairScreener


SIM2SIM_BENCH_VERSION = "TRAX-Sim2Sim-Stage2-v1.0"


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sim2sim_adaptation_scenarios() -> list[dict[str, Any]]:
    """5 target-environment adaptation trials (SimEX Stage-2 budget = 5 rollouts)."""
    return [
        {
            "scenario_id": "adapt_01_noisy_slip",
            "label": "Target Adapt-1: Sensor Noise + DR + Unseen Slip (36,-12mm)",
            "preset": "default",
            "sensor_noise": True,
            "domain_randomization": True,
            "noise_seed": 201,
            "fault_inject_slip_xy_m": [0.036, -0.012],
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "completed",
        },
        {
            "scenario_id": "adapt_02_noisy_timeout",
            "label": "Target Adapt-2: Sensor Noise + DR + Brew Ack Timeout",
            "preset": "default",
            "sensor_noise": True,
            "domain_randomization": True,
            "noise_seed": 202,
            "simulate_timeout": True,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "completed",
        },
        {
            "scenario_id": "adapt_03_crash_restart",
            "label": "Target Adapt-3: Sensor Noise + DR + Mid-Brew Host Crash/Restart",
            "preset": "default",
            "sensor_noise": True,
            "domain_randomization": True,
            "noise_seed": 203,
            "simulate_crash_mid_brew": True,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "completed",
        },
        {
            "scenario_id": "adapt_04_shift_slip_unseen_cup",
            "label": "Target Adapt-4: Store Shift (+28,-22mm) + Ceramic Mug + Slip",
            "preset": "default",
            "sensor_noise": True,
            "domain_randomization": True,
            "noise_seed": 204,
            "cup_class": "ceramic_mug",
            "store_layout_shift_xy_m": [0.028, -0.022],
            "fault_inject_slip_xy_m": [0.034, 0.014],
            "order_id": "A043",
            "target_slot": "$SLOT_PICKUP_2",
            "expect_status": "completed",
        },
        {
            "scenario_id": "adapt_05_noisy_ambiguity",
            "label": "Target Adapt-5: Sensor Noise + Dual-Cup Ambiguity (Must Pause)",
            "preset": "sim_dual_cup",
            "sensor_noise": True,
            "domain_randomization": True,
            "noise_seed": 205,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "expect_status": "paused_for_operator",
        },
    ]


def sim2sim_heldout_target_scenarios() -> list[dict[str, Any]]:
    """16 disjoint physics-mismatched target evaluation scenarios (sensor noise + DR across all tiers)."""
    base = [
        ("s2s_01_nominal_dr", "H01 Noisy+DR Nominal", {"preset": "default", "noise_seed": 301, "expect_status": "completed"}),
        ("s2s_02_shift_green", "H02 Store-B Shift + Takeout Green", {"preset": "default", "noise_seed": 302, "cup_class": "takeout_green", "store_layout_shift_xy_m": [0.025, -0.028], "order_id": "A043", "target_slot": "$SLOT_PICKUP_2", "expect_status": "completed"}),
        ("s2s_03_shift_iced", "H03 Store-C Shift + Unseen Iced Cup", {"preset": "default", "noise_seed": 303, "cup_class": "iced_clear", "store_layout_shift_xy_m": [-0.022, 0.030], "target_slot": "$SLOT_PICKUP_2", "expect_status": "completed"}),
        ("s2s_04_slip_30", "H04 Noisy+DR Slip (30,-10mm)", {"preset": "default", "noise_seed": 304, "fault_inject_slip_xy_m": [0.030, -0.010], "expect_status": "completed"}),
        ("s2s_05_slip_40", "H05 Noisy+DR Severe Slip (38,+16mm)", {"preset": "default", "noise_seed": 305, "fault_inject_slip_xy_m": [0.038, 0.016], "expect_status": "completed"}),
        ("s2s_06_timeout", "H06 Noisy+DR Brew Ack Timeout", {"preset": "default", "noise_seed": 306, "simulate_timeout": True, "expect_status": "completed"}),
        ("s2s_07_slip_timeout", "H07 Noisy+DR Slip + Brew Timeout", {"preset": "default", "noise_seed": 307, "fault_inject_slip_xy_m": [0.035, -0.014], "simulate_timeout": True, "expect_status": "completed"}),
        ("s2s_08_crash_nom", "H08 Noisy+DR Mid-Brew Crash/Restart", {"preset": "default", "noise_seed": 308, "simulate_crash_mid_brew": True, "expect_status": "completed"}),
        ("s2s_09_crash_shift", "H09 Store Shift + Mid-Brew Crash/Restart", {"preset": "default", "noise_seed": 309, "store_layout_shift_xy_m": [0.024, -0.024], "cup_class": "ceramic_mug", "simulate_crash_mid_brew": True, "expect_status": "completed"}),
        ("s2s_10_crash_slot2", "H10 Slot-2 + Takeout + Crash/Restart", {"preset": "default", "noise_seed": 310, "cup_class": "takeout_green", "order_id": "A043", "target_slot": "$SLOT_PICKUP_2", "simulate_crash_mid_brew": True, "expect_status": "completed"}),
        ("s2s_11_busy_machine", "H11 Noisy+DR Machine Busy (2 polls)", {"preset": "default", "noise_seed": 311, "simulate_machine_busy_polls": 2, "expect_status": "completed"}),
        ("s2s_12_dual_confirm", "H12 Dual Cup Resolved via Operator Confirm", {"preset": "sim_dual_cup", "noise_seed": 312, "operator_confirmed_cup_id": "obj-cup-a", "expect_status": "completed"}),
        ("s2s_13_dual_pause", "H13 Noisy Dual-Cup Ambiguity (Must Pause)", {"preset": "sim_dual_cup", "noise_seed": 313, "expect_status": "paused_for_operator"}),
        ("s2s_14_stale_obs", "H14 Stale Perception (Must Reject)", {"preset": "default", "noise_seed": 314, "stale_observation": True, "expect_status": "rejected_observation_not_ready"}),
        ("s2s_15_hazard_nogo", "H15 Forbidden Steam Zone (Must Escalate)", {"preset": "default", "noise_seed": 315, "target_slot": "$ZONE_STEAM_NOGO", "expect_status": "escalated_to_human"}),
        ("s2s_16_contradictory", "H16 Contradictory Brew Ack (Must Escalate)", {"preset": "default", "noise_seed": 316, "contradictory_receipt": True, "expect_status": "escalated_to_human"}),
    ]
    out: list[dict[str, Any]] = []
    for sid, label, extra in base:
        item: dict[str, Any] = {
            "scenario_id": sid,
            "label": label,
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "sensor_noise": True,
            "domain_randomization": True,
        }
        item.update(extra)
        out.append(item)
    return out


def _execute_target_trial(
    driver: SceneHostDriver,
    source: str,
    scenario: Mapping[str, Any],
    *,
    run_id: str,
    journal: Journal | None,
    enable_journal_proxy: bool,
) -> dict[str, Any]:
    scen = dict(scenario)
    driver.reset_scenario(scen)
    res = _run_driver_order(
        driver,
        source,
        command_id=f"{run_id}-att1",
        scenario=scen,
        journal=journal if enable_journal_proxy else None,
        enable_journal_proxy=enable_journal_proxy,
        journal_run_id=run_id,
    )
    if bool(res.get("crashed_mid_brew")) or (
        bool(scen.get("simulate_crash_mid_brew"))
        and res.get("host_runtime_status") in {"crashed_mid_brew", "exception"}
    ):
        resume_scen = dict(scen)
        resume_scen["resume_after_crash"] = True
        resume_scen["simulate_crash_mid_brew"] = False
        res = _run_driver_order(
            driver,
            source,
            command_id=f"{run_id}-att2",
            scenario=resume_scen,
            journal=journal if enable_journal_proxy else None,
            enable_journal_proxy=enable_journal_proxy,
            journal_run_id=run_id,
        )
    return res


def _strip_hidden_target_params(scenario: Mapping[str, Any]) -> dict[str, Any]:
    """Strip hidden physical disturbances so the sandbox starts unaligned (realistic Sim-to-Sim gap)."""
    return {
        "preset": str(scenario.get("preset", "default")),
        "order_id": str(scenario.get("order_id", "A042")),
        "target_slot": str(scenario.get("target_slot", "$SLOT_PICKUP_1")),
        "expect_status": str(scenario.get("expect_status", "completed")),
        "label": str(scenario.get("label", scenario.get("scenario_id", "target_failure"))),
    }


def _direct_repair_prompt(prev_src: str, target_scen: Mapping[str, Any], trial: Mapping[str, Any]) -> str:
    return "\n".join(
        [
            "Direct Target-Failure Repair (no sandbox calibration or regression bank):",
            "Fix the failed target rollout below and output ONLY valid Python defining `run_policy(sdk, params)`.",
            "",
            SHARED_CAFEBENCH_V2_CONTRACT,
            "",
            SHARED_CAFEBENCH_V2_REQUIREMENTS,
            "",
            "Previous Policy:",
            "```python",
            prev_src.strip(),
            "```",
            "",
            f"Target Scenario: {target_scen.get('label')}",
            f"Observed Failure: status={trial.get('host_runtime_status')} detail={trial.get('detail')} methods={trial.get('methods')}",
        ]
    )


def render_sim2sim_html(report: Mapping[str, Any]) -> str:
    summary = list((report.get("summary_by_method") or {}).values())
    mcnemar = report.get("paired_mcnemar_craft_vs") or {}
    rows = []
    for m in summary:
        mc = mcnemar.get(m["method_id"])
        mc_str = f"p={mc['p_value']} (+{mc['n_a_only']}/-{mc['n_b_only']})" if mc else "ref"
        rows.append(
            "<tr>"
            f"<td><strong>{html.escape(str(m['label']))}</strong><br><small>{html.escape(str(m['provenance']))}</small></td>"
            f"<td><strong>{m['passed']}/{m['total_trials']} ({m['pass_rate']*100:.1f}%)</strong>"
            f"<br><small>CI [{m['wilson_95_ci'][0]*100:.1f}%, {m['wilson_95_ci'][1]*100:.1f}%]</small></td>"
            f"<td>{m['target_passed']}/{m['target_total']}</td>"
            f"<td>{m['shadow_passed']}/{m['shadow_total']}</td>"
            f"<td>{m['duplicate_non_idempotent']} / {m['false_actuations']}</td>"
            f"<td>target={m['adapt_target_trials']}/5 · cal={m['adapt_calibration_trials']} · llm={m['adapt_llm_calls']}</td>"
            f"<td><small>{html.escape(mc_str)}</small></td>"
            "</tr>"
        )
    prov = report.get("provenance") or {}
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<title>{html.escape(str(report.get('benchmark_version', SIM2SIM_BENCH_VERSION)))}</title>
<style>
  body {{ font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; background: #0b1120; color: #e2e8f0; margin: 0; padding: 24px; }}
  .wrap {{ max-width: 1180px; margin: 0 auto; }}
  .card {{ background: #111827; border: 1px solid #1f2937; border-radius: 8px; padding: 16px 20px; margin-bottom: 16px; }}
  h1, h2 {{ margin: 0 0 10px; font-weight: 600; }}
  table {{ width: 100%; border-collapse: collapse; font-size: 13px; }}
  th, td {{ border-bottom: 1px solid #1f2937; padding: 8px 10px; text-align: left; }}
  th {{ color: #94a3b8; }}
  small {{ color: #94a3b8; }}
</style>
</head>
<body>
<div class="wrap">
  <div class="card">
    <h1>{html.escape(str(report.get('benchmark_version', SIM2SIM_BENCH_VERSION)))} — Sim-to-Sim 5-Trial Adaptation Benchmark</h1>
    <small>sandbox={html.escape(str(report.get('sandbox_driver')))} -> target={html.escape(str(report.get('target_driver')))} (sensor_noise=True, domain_randomization=True) · model={html.escape(str(report.get('model')))} · git={html.escape(str(prov.get('git_commit') or 'n/a')[:12])}</small>
  </div>
  <div class="card">
    <h2>Held-Out Physics-Mismatched Target &amp; Recorded-Shadow Results (Strict &le;5 Target Adaptation Trials)</h2>
    <table>
      <thead>
        <tr>
          <th>Method</th>
          <th>Overall Held-Out (Target + Shadow)</th>
          <th>Target Sim (Noisy+DR)</th>
          <th>Recorded Shadow</th>
          <th>Dup Non-Idem / False Act</th>
          <th>Adaptation Budget</th>
          <th>McNemar vs TRAX</th>
        </tr>
      </thead>
      <tbody>{''.join(rows)}</tbody>
    </table>
  </div>
</div>
</body>
</html>"""


def run_sim2sim_adaptation_benchmark(
    sandbox_driver: SceneHostDriver,
    target_driver: SceneHostDriver,
    config: SceneConfig,
    *,
    llm_fn: LLMGenerateFn,
    output_dir: str | Path,
    initial_policy_source: str | None = None,
    adaptation_scenarios: Sequence[Mapping[str, Any]] | None = None,
    heldout_scenarios: Sequence[Mapping[str, Any]] | None = None,
    model: str = "gpt-6-astra",
    reasoning_effort: str = "medium",
    max_candidates_per_failure: int = 2,
    progress_cb: Callable[[str], None] | None = None,
) -> dict[str, Any]:
    """Run true Sim-to-Sim 5-trial adaptation benchmark across all 4 protocols."""
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    pol_dir = out / "policies"
    pol_dir.mkdir(parents=True, exist_ok=True)
    adapt_queue = list(adaptation_scenarios or sim2sim_adaptation_scenarios())[:5]
    eval_suite = list(heldout_scenarios or sim2sim_heldout_target_scenarios())

    def _log(msg: str) -> None:
        if progress_cb:
            progress_cb(msg)

    # Shared starting policy developed strictly in nominal sandbox (or supplied directly)
    # Must pass nominal sandbox default scenario (matching SimEX Zero-Shot Sim2Real starting point)
    if initial_policy_source is not None:
        init_src = _strip_code_fence(initial_policy_source)
    else:
        _log("[Sim2Sim] Generating shared nominal sandbox baseline policy (verified on nominal default)...")
        prompt0 = "\n".join(
            [
                "Write a clean nominal single-cup coffee policy `run_policy(sdk, params)` for our café workstation.",
                "Remember: `l2_pouring_safety.safe_to_pour` is evaluated AFTER `place_cup_under_machine` places the cup under the spout.",
                "",
                SHARED_CAFEBENCH_V2_CONTRACT,
                "",
                SHARED_CAFEBENCH_V2_REQUIREMENTS,
            ]
        )
        init_src = _strip_code_fence(str(llm_fn(prompt0)["source"]))
        nom_scen = {"preset": "default", "order_id": "A042", "target_slot": "$SLOT_PICKUP_1", "expect_status": "completed"}
        nom_res = _execute_target_trial(
            sandbox_driver, init_src, nom_scen, run_id="init-nom-1", journal=None, enable_journal_proxy=False
        )
        if not bool(nom_res.get("task_passed", False)):
            _log(f"  [Sim2Sim] Nominal draft failed ({nom_res.get('host_runtime_status')}), refining once on nominal sandbox...")
            fix_p = _direct_repair_prompt(init_src, {"label": "Nominal sandbox default"}, nom_res)
            cand0 = _strip_code_fence(str(llm_fn(fix_p)["source"]))
            if check_sandbox_ast_compliance(cand0)["compliant"]:
                init_src = cand0

    def _gen_adapter(cfg: SceneConfig, prompt: str) -> dict[str, Any]:
        _ = cfg
        res = llm_fn(prompt)
        return {"source": _strip_code_fence(str(res["source"])), "raw_output": res.get("raw_output", "")}

    method_specs = (
        ("zeroshot_sim2sim", "Zero-Shot Sim2Sim", "baseline_control", False),
        ("direct_repair", "Direct Target Repair", "ablation_direct_repair", False),
        ("simex_protocol", "SimEX-Protocol (Adapted, Calibrated Sim + Screening)", "protocol_adapted_from_simex_2609_38982", False),
        ("craft_full", "TRAX (Calibrated Sim + RepairScreener + Shadow + Journal)", "craft_proposed_system", True),
    )

    adapted: dict[str, dict[str, Any]] = {}
    for mid, label, prov, use_journal in method_specs:
        _log(f"[Sim2Sim] Adapting method: {mid} (max {len(adapt_queue)} target trials)...")
        cur_src = init_src
        target_trials_used = 0
        cal_trials_used = 0
        llm_calls_used = 0
        adapt_log: list[dict[str, Any]] = []

        if mid != "zeroshot_sim2sim":
            m_ws = out / f"ws_{mid}"
            suite = RegressionSuite(m_ws / "regression_suite.json")
            screener = RepairScreener(m_ws, config, sandbox_driver, regression_suite=suite)
            adapt_journal_path = m_ws / "adapt_journal.sqlite"

            with Journal(adapt_journal_path) as adapt_journal:
                for idx, t_scen in enumerate(adapt_queue, start=1):
                    target_trials_used += 1
                    res = _execute_target_trial(
                        target_driver,
                        cur_src,
                        t_scen,
                        run_id=f"{mid}-adapt-{idx}",
                        journal=adapt_journal if use_journal else None,
                        enable_journal_proxy=use_journal,
                    )
                    passed = bool(res.get("task_passed", False))
                    attr = attribute_trial(res)
                    step_rec: dict[str, Any] = {
                        "trial": idx,
                        "scenario_id": t_scen["scenario_id"],
                        "passed": passed,
                        "attribution": attr.to_dict(),
                    }
                    if not passed:
                        if mid == "direct_repair":
                            gen = llm_fn(_direct_repair_prompt(cur_src, t_scen, res))
                            llm_calls_used += 1
                            cand = _strip_code_fence(str(gen["source"]))
                            if check_sandbox_ast_compliance(cand)["compliant"]:
                                cur_src = cand
                                step_rec["action"] = "direct_overwrite"
                            else:
                                step_rec["action"] = "direct_ast_rejected"
                        else:
                            # Strip hidden target disturbances so sandbox MUST calibrate via search!
                            unaligned_ctx = _strip_hidden_target_params(t_scen)
                            case = suite.capture_failure(res, scenario_context=unaligned_ctx)
                            if case is not None:
                                screen_rep = screener.screen_repair(
                                    baseline_source=cur_src,
                                    case=case,
                                    screening_id=f"{mid}-t{idx}-{case.case_id}",
                                    generator_fn=_gen_adapter,
                                    max_candidates=max_candidates_per_failure,
                                    max_model_calls=max_candidates_per_failure + 1,
                                    max_trials=30,
                                    enable_sim_calibration=True,
                                    max_calibration_trials=6,
                                    sdk_contract=SHARED_CAFEBENCH_V2_CONTRACT,
                                )
                                cal_trials_used += int((screen_rep.get("alignment") or {}).get("calibration_trials_used", 0))
                                llm_calls_used += int((screen_rep.get("budget_used") or {}).get("model_calls", 0))
                                step_rec["alignment_status"] = (screen_rep.get("alignment") or {}).get("reproduction_status")
                                step_rec["alignment_method"] = (screen_rep.get("alignment") or {}).get("alignment_method")
                                step_rec["screening_status"] = screen_rep.get("status")
                                if screen_rep.get("recommended_candidate_id"):
                                    cand_src = screener.get_candidate_source(
                                        str(screen_rep["screening_id"]),
                                        str(screen_rep["recommended_candidate_id"]),
                                    )
                                    # TRAX adds recorded-data shadow safety check before adopting candidate
                                    if mid == "craft_full":
                                        shadow_chk = run_recorded_shadow_check(config, cand_src)
                                        step_rec["recorded_shadow_passed"] = shadow_chk["all_passed"]
                                        if shadow_chk["all_passed"]:
                                            cur_src = cand_src
                                            step_rec["action"] = "promoted_after_shadow_gate"
                                        else:
                                            step_rec["action"] = "blocked_by_recorded_shadow"
                                    else:
                                        cur_src = cand_src
                                        step_rec["action"] = "promoted_simex_screened"
                    adapt_log.append(step_rec)

        (pol_dir / f"{mid}.py").write_text(cur_src + "\n", encoding="utf-8")
        adapted[mid] = {
            "method_id": mid,
            "label": label,
            "provenance": prov,
            "enable_journal_proxy": use_journal,
            "source": cur_src,
            "source_sha256": digest(cur_src),
            "adapt_target_trials": target_trials_used,
            "adapt_calibration_trials": cal_trials_used,
            "adapt_llm_calls": llm_calls_used,
            "adaptation_log": adapt_log,
        }

    # Evaluate all 4 frozen policies on disjoint physics-mismatched held-out target + recorded shadow
    trials: list[dict[str, Any]] = []
    summary: dict[str, dict[str, Any]] = {}
    eval_vectors: dict[str, list[bool]] = {}

    for mid, label, prov, use_journal in method_specs:
        art = adapted[mid]
        src = str(art["source"])
        eval_journal_path = out / f"eval_{mid}_journal.sqlite"
        target_ok = 0
        dup_cnt = 0
        false_act = 0
        vec: list[bool] = []

        with Journal(eval_journal_path) as ej:
            for idx, scen in enumerate(eval_suite, start=1):
                res = _execute_target_trial(
                    target_driver,
                    src,
                    scen,
                    run_id=f"eval-{mid}-{scen['scenario_id']}",
                    journal=ej if use_journal else None,
                    enable_journal_proxy=use_journal,
                )
                ok = bool(res.get("task_passed", False))
                dup = bool(res.get("duplicate_non_idempotent", False))
                brews = int((res.get("non_idempotent_counts") or {}).get("start_coffee_machine", 0))
                fa = (scen.get("expect_status") != "completed" and not scen.get("contradictory_receipt") and brews > 0)
                if ok:
                    target_ok += 1
                if dup:
                    dup_cnt += 1
                if fa:
                    false_act += 1
                vec.append(ok)
                trials.append(
                    {
                        "method_id": mid,
                        "scenario_id": scen["scenario_id"],
                        "evidence_source": target_driver.evidence_source,
                        "task_passed": ok,
                        "duplicate_non_idempotent": dup,
                        "false_actuation": fa,
                        "host_runtime_status": res.get("host_runtime_status"),
                    }
                )

        shadow = run_recorded_shadow_check(config, src)
        for st in shadow["trials"]:
            vec.append(bool(st["task_passed"]))
            trials.append(
                {
                    "method_id": mid,
                    "scenario_id": st["scenario_id"],
                    "evidence_source": "recorded_data",
                    "task_passed": bool(st["task_passed"]),
                    "duplicate_non_idempotent": False,
                    "false_actuation": bool(st["shadow_brew_intents"] > 0),
                    "host_runtime_status": st.get("host_runtime_status"),
                }
            )
        false_act += int(shadow["false_actuations"])
        tot = len(eval_suite) + int(shadow["total"])
        passed = target_ok + int(shadow["passed"])
        eval_vectors[mid] = vec
        summary[mid] = {
            "method_id": mid,
            "label": label,
            "provenance": prov,
            "passed": passed,
            "total_trials": tot,
            "pass_rate": round(passed / tot, 4) if tot else 0.0,
            "wilson_95_ci": list(wilson_ci_95(passed, tot)),
            "target_passed": target_ok,
            "target_total": len(eval_suite),
            "shadow_passed": int(shadow["passed"]),
            "shadow_total": int(shadow["total"]),
            "duplicate_non_idempotent": dup_cnt,
            "false_actuations": false_act,
            "adapt_target_trials": art["adapt_target_trials"],
            "adapt_calibration_trials": art["adapt_calibration_trials"],
            "adapt_llm_calls": art["adapt_llm_calls"],
        }
        _log(
            f"[Sim2Sim][EVAL] {mid:16s} -> {passed}/{tot} ({passed/tot*100:.1f}%) "
            f"[target={target_ok}/{len(eval_suite)}, shadow={shadow['passed']}/{shadow['total']}, dup={dup_cnt}]"
        )

    paired: dict[str, dict[str, Any]] = {}
    c_vec = eval_vectors["craft_full"]
    for mid in ("zeroshot_sim2sim", "direct_repair", "simex_protocol"):
        paired[mid] = mcnemar_exact_p(c_vec, eval_vectors[mid])

    report = {
        "benchmark_version": SIM2SIM_BENCH_VERSION,
        "sandbox_driver": sandbox_driver.driver_name,
        "target_driver": target_driver.driver_name,
        "evidence_sources": sorted({sandbox_driver.evidence_source, target_driver.evidence_source, "recorded_data"}),
        "model": model,
        "reasoning_effort": reasoning_effort,
        "max_target_adaptation_trials": len(adapt_queue),
        "provenance": _git_provenance(),
        "finished_at_utc": _utc_now(),
        "methods": {k: {mk: mv for mk, mv in v.items() if mk != "source"} for k, v in adapted.items()},
        "summary_by_method": summary,
        "paired_mcnemar_craft_vs": paired,
        "trials": trials,
    }
    _write_json_atomic(out / "sim2sim_report.json", report)
    (out / "sim2sim_report.html").write_text(render_sim2sim_html(report), encoding="utf-8")
    return report
