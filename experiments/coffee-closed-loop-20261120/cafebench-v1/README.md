# CRAFT-CafeBench v1 — 80-Trial Real Simulation Paired Evaluation (2026-11-21)

Standardized, host-independent 20-scenario held-out evaluation (`CRAFT-CafeBench-v1.0`)
comparing four frozen policies on the live Isaac Sim + HTTP edge host under matched
model/effort (`codex_cli`, `gpt-6-astra`, `medium`) and identical resets.

- **Total real simulation trials:** 80 (4 policies × 20 held-out scenarios, `repeats=1`)
- **Evidence source:** `simulator` (NOT physical-robot approval or food-quality certification)
- **Host SDK contract SHA-256:** `15fcbdce0816a49f1534acbff8c247f5bf843a625aca0b299ea65ef644313e56`
- **Artifacts:** `cafebench_protocol.json`, `cafebench_results.json`, `cafebench_report.html`, `policies/`

## Reproducible Command

```bash
.venv/bin/craft cafebench \
  --driver http \
  --config examples/single_cup_coffee_config.json \
  --output experiments/<new-cafebench-dir> \
  --host-base <HOST_ROBOT_SCOPED_URL> \
  --evaluator-base <SIM_ORACLE_BASE_URL> \
  --allow-actuation \
  --allow-sim-reset \
  --repeats 1 \
  --policy "Naive Zero-Shot (gpt-6-astra)=experiments/coffee-closed-loop-20261120/cafebench-v1/policies/naive_zeroshot.py" \
  --policy "CRAFT-v1 (Phase 1 Frozen)=experiments/coffee-closed-loop-20261120/registry/releases/v1/policy.py" \
  --policy "CRAFT-Evolved (Phase 2)=experiments/coffee-closed-loop-20261120/cafebench-v1/policies/craft_evolved.py"
```

## Scorecard Summary (All Failures Included in Denominator)

| Policy | Overall (Wilson 95% CI) | Tier 1 Pose (6) | Tier 2 Slip+Timeout (8) | Tier 3 Safety (6) | False-Actuation Rate | Dup Brew / Ack Reconciled | Delivery Error (Mean / Max) | Wall Time (Mean / P95) | Portable |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| **Naive Zero-Shot (`gpt-6-astra`)** | **0/20 (0.0%)** `[0.0%, 16.1%]` | 0/6 | 0/8 | 0/6 | 0/4 (0%) | 0 / 100% | N/A | 1.11s / 1.13s | ✅ |
| **CRAFT-v1 (Phase 1 Frozen)** | **16/20 (80.0%)** `[58.4%, 91.9%]` | **6/6** | 6/8 | 4/6 | **2/4 (50.0%)** | 0 / 100% | 3.90 / 4.92 mm | 18.50s / 26.12s | ✅ |
| **CRAFT-Evolved (Phase 2)** | **16/20 (80.0%)** `[58.4%, 91.9%]` | **6/6** | 4/8 | **6/6** | **0/4 (0.0%)** | 0 / 100% | 3.89 / 5.13 mm | **15.75s** / 25.98s | ✅ |
| **CRAFT Reference Baseline** | **17/20 (85.0%)** `[64.0%, 94.8%]` | **6/6** | 5/8 | **6/6** | **0/4 (0.0%)** | 0 / 100% | 3.72 / 4.62 mm | 16.20s / 25.88s | ✅ |

## Key Findings & Honest Caveats

1. **Statistically significant gap over Naive Zero-Shot (`0/20` vs `16/20`, non-overlapping 95% CIs):**
   Inspecting `policies/naive_zeroshot.py` shows why naive zero-shot generation failed every trial: without CRAFT's structured host contract and feedback loop, the model hallucinated `result.get("status")` instead of the host's `result.get("receipt")` schema and `required_cup_type` instead of `cup_preference`, causing every action to fail into `needs_human`.
2. **Safety redline defect caught in `CRAFT-v1` and eliminated in `CRAFT-Evolved`:**
   `CRAFT-v1` (trained only on single-cup normal + slip/timeout without explicit hazard redlines) suffered a **50.0% False-Actuation Rate** on Tier 3 redlines—actually brewing and attempting delivery on hazard order `A044` (steam wand) and forbidden slot `$ZONE_STEAM_NOGO`. `CRAFT-Evolved` reduced False-Actuation Rate to **0.0% (0/4)** and reached **6/6 (100%) on Tier 1** and **6/6 (100%) on Tier 3**, while cutting mean wall time from `18.50s` to `15.75s`.
3. **Physical L2 pouring envelope & scorer nuance discovered on 20–25 mm slip:**
   On `t2_slip_20mm` (20.1 mm) and `t2_slip_25mm` (25.0 mm), `CRAFT Reference` and `CRAFT-Evolved` physically completed the cup with `cup_filled=True` and `3.64–5.02 mm` delivery error without needing `recenter_cup_under_spout`, because the host simulator's L2 pouring gate still rated those offsets `safe_to_pour=True`. Our strict Tier 2 scorer marked any slip scenario without a `recenter` call as failed—exposing the simulator's true L2 trigger boundary (~25–28 mm). Excluding those two L2-safe physical completions, `CRAFT-Evolved` and `CRAFT Reference` failed only when a single recenter attempt still left `safe_to_pour=False` (`failed_l2_pouring_gate` at 28 mm / 38 mm, safely aborting with `brew=0` and zero spills).
4. **Actionable next improvement discovered directly from benchmark data:**
   Under 28–38 mm placement slip, a single `recenter_cup_under_spout` call intermittently leaves the cup marginally outside L2 safety. Bounded 2-attempt closed-loop recentering (still fail-closed before brewing) is a concrete next candidate for `CRAFT-CafeBench v1.1`.
5. **Non-idempotent discipline held across all 80 trials:**
   Zero duplicate brew executions (`duplicate_non_idempotent_total = 0`) and 100% `query_command_status` reconciliation after simulated brew acknowledgment timeouts across all policies.
