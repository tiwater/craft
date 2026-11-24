# CRAFT Goal v2 Experimental Archive (`2026-11-22`)

This archive collects reproducible evidence across all four `/goal` milestones, keeping `fixture`, `simulator`, `recorded_data`, and `physical_robot` evidence strictly separate per `AGENTS.md`.

> **Evidence boundary notice**: All benchmark and operations-loop evidence in this archive is **`simulator`** (MuJoCo 3.3.0 / Robosuite 1.5.2) and **`recorded_data`** (zero-actuation recorded-frame shadow replay). It is **not** physical-robot approval or hardware safety certification.

---

## Executive Milestone Verification

| Milestone | Status | Key Quantitative Result | Archive Path |
|---|---|---|---|
| **M1 — Real-World Operational Closed Loop** | **PASS** | Day-Shift v1.0 (`5/6`, auto-captured `reg_001_047faa`) → Nightly `RepairScreener` (`reproduced_exact`, H1 rejected, H2 promoted) → `recorded_data` shadow (`4/4`, 0 false actuations) → Independent Acceptance (`100%`, max delivery error `0.067 mm`) → Publish `v2.0` + Rollback drill (`v2.0 → v1.0 → v2.0`) → `SkillExperience` (`measured_limits`) → Day-Shift v2.0 (`6/6`, 0 duplicate brews, 0 false actuations). Verified via both `/v1` Engine API and CRAFT Studio HTTP servers (`HTTP 200`). | [`operations-loop/`](operations-loop/operations_closed_loop_summary.json) |
| **M2 — CafeBench v2 (30 Held-Out, Live `gpt-6-astra`)** | **PASS** | **CRAFT `30/30 (100.0%)`** vs ASPIRE `23/30 (76.7%)`, CaP-X `23/30 (76.7%)`, SimEX-Direct `21/30 (70.0%)`, Zero-Shot `7/30 (23.3%)`. Lead **+23.33 pp** over strongest baseline, exact McNemar **`p = 0.015625`**, **0 duplicates**, **0% false actuation**, **0% regression**, **2 LLM calls / 12 resets / 102.3 s** (3× cheaper than iterative baselines). | [`cafebench-v2/`](cafebench-v2/cafebench_v2_report.html) |
| **M3 — Robosuite Parity v2 (4 Tasks × 20 Held-Out Seeds)** | **PASS** | **CRAFT `78/80 (97.5%)`** (`Lift 20/20`, `PickPlaceCan 20/20`, `NutAssemblySquare 18/20`, **`Door 20/20`**) vs CaP-X `70/80 (87.5%, +9 eval LLM)`, ASPIRE `49/80 (61.3%)`, SimEX-Direct `40/80 (50.0%)`, Zero-Shot `23/80 (28.7%)`. Beats ASPIRE by **+36.25 pp** (`p < 0.0001`), beats CaP-X by **+10.00 pp** (`p = 0.0386`) with **0 online LLM calls** & **21× lower p95 latency** (`2.27 s` vs `47.78 s`), using **fewer dev LLM calls (`11` vs `15`)** than every iterative baseline. | [`robosuite-parity-v2/`](robosuite-parity-v2/robosuite_prior_work_report.html) |
| **M4 — Engineering Quality & Honest Evidence** | **PASS** | Pure Python stdlib core (non-stdlib imports strictly isolated to optional `mujoco_cafe_driver.py` and `robosuite_bench.py`), **99/99 pytest passing (`100%`)**, strict evidence separation, full logs, JSON, HTML reports, policies, skills, and earlier engineering runs preserved honestly. | Repo tests & logs below |

---

## 1. Milestone 1 — Real-World Operational Closed Loop

Artifact: [`operations-loop/operations_closed_loop_summary.json`](operations-loop/operations_closed_loop_summary.json)

- **Day-Shift `v1.0`** (`enable_journal_proxy=False`, frozen live-generated `aspire_protocol.py`): `5/6` passed; mid-brew crash/restart failed and was automatically captured into `regression_suite.json` as `reg_001_047faa`.
- **Nightly Evolution (`run_nightly_evolution`)**:
  - **Replay Alignment (`RepairScreener`)**: `reproduced=True` (`reproduction_status="reproduced_exact"`).
  - **Multi-Hypothesis Screening**:
    - `scr_01_h1_zeroshot_rewrite`: `target_fixed=False`, rejected (`target_failure_not_fixed`).
    - `scr_02_h2_craft_journal_reconcile`: `target_fixed=True`, `eligible_for_acceptance=True`, `rejection_reasons=[]`, recommended.
  - **Recorded-Data Shadow Check (`evidence_source="recorded_data"`)**: `4/4` passed, `0` false actuations (stale perception, low-confidence cup, dual ambiguous cups, bean-bag distractor).
  - **Independent Held-Out Acceptance (`acc-d32c867c18-986d7dd9`)**: `passed=True`, `completion_rate=1.0`, `max_delivery_error_mm=0.067 mm` (gate `<= 15.0 mm`).
  - **Release & Rollback Drill**: Published frozen `v2.0`, rolled back `v2.0 -> v1.0`, re-activated `v2.0` (`active_version="v2.0"`).
  - **SkillExperience (`skill-v2.0-reg_001_047faa`)**: Recorded `measured_limits` (`verified_slip_mm=34.2`, `verified_layout_shift_mm=35.4`, `crash_restart_reconciliation=True`, `recorded_shadow_pass_rate=1.0`).
- **Day-Shift `v2.0`** (`enable_journal_proxy=True`, `verify_frozen_release=True`): **`6/6 (100%)`**, `duplicate_non_idempotent_orders=0`, `false_actuation_orders=0`, `unreconciled_unknown_orders=0`.
- **Control Plane Verified**: Both `CraftEngineService` (`/v1/health`, `/v1/screening`, `/v1/skills`, `/v1/releases`) and `CraftStudioApp` (`/api/state`, `/cafebench_v2_report.html`, `/robosuite_v2_report.html`) verified live with `HTTP 200`.

---

## 2. Milestone 2 — CafeBench v2 (`CRAFT-CafeBench-v2.0`)

Matched model: `gpt-6-astra` (`provider=codex_cli`, `effort=medium`, `max_dev_calls=6`, 6 disjoint dev scenarios / 30 held-out scenarios = 26 MuJoCo simulator + 4 zero-actuation recorded-data shadow).

Artifacts:
- HTML report: [`cafebench-v2/cafebench_v2_report.html`](cafebench-v2/cafebench_v2_report.html)
- JSON report: [`cafebench-v2/cafebench_v2_report.json`](cafebench-v2/cafebench_v2_report.json)
- Raw execution log: [`cafebench_v2_run.log`](cafebench_v2_run.log)
- Frozen policies & skills: [`cafebench-v2/policies/`](cafebench-v2/policies/), [`cafebench-v2/craft_v2_skills.json`](cafebench-v2/craft_v2_skills.json)

| Method | Held-Out Overall | Wilson 95% CI | Simulator (`n=26`) | Recorded Shadow (`n=4`) | Dup Non-Idem | False Actuation | Dev LLM Calls | Dev Resets | Dev Wall (s) | Paired McNemar vs CRAFT |
|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---|
| **CRAFT (RepairScreener + RegressionSuite + Journal)** | **30/30 (100.0%)** | `[88.7%, 100.0%]` | **26/26** | **4/4** | **0** | **0 (0.0%)** | **2** | **12** | **102.3** | reference |
| ASPIRE-Protocol (Adapted) | 23/30 (76.7%) | `[59.1%, 88.2%]` | 19/26 | 4/4 | 0 | 0 (0.0%) | 6 | 36 | 326.2 | **+23.33 pp**, `p = 0.015625` (`+7/-0`) |
| CaP-X-Protocol (Adapted) | 23/30 (76.7%) | `[59.1%, 88.2%]` | 19/26 | 4/4 | 0 | 0 (0.0%) | 6 | 36 | 257.0 | **+23.33 pp**, `p = 0.015625` (`+7/-0`) |
| SimEX-Direct-Repair (Ablation) | 21/30 (70.0%) | `[52.1%, 83.3%]` | 19/26 | 2/4 | 0 | 0 (0.0%) | 6 | 36 | 310.8 | **+30.00 pp**, `p = 0.003906` (`+9/-0`) |
| Strong Zero-Shot | 7/30 (23.3%) | `[11.8%, 40.9%]` | 3/26 | 4/4 | 0 | 0 (0.0%) | 1 | 0 | 55.4 | **+76.67 pp**, `p < 0.0001` (`+23/-0`) |

### Reproduction Command

```bash
.venv/bin/craft cafebench-v2 \
  --output experiments/goal-v2-20261122/cafebench-v2 \
  --provider codex_cli --model gpt-6-astra --effort medium --max-dev-calls 6
```

---

## 3. Milestone 3 — Robosuite Parity v2 (`CRAFT-Robosuite-Parity-v2.0`)

Matched model: `gpt-6-astra` (`provider=codex_cli`, `effort=medium`, `max_dev_calls=6`, shared paired `it=1` initial draft across all 5 methods to eliminate initialization lottery variance, dev seeds `[101..105]`, disjoint held-out eval seeds `[1..20]`, 80 trials per method across `Lift`, `PickPlaceCan`, `NutAssemblySquare`, `Door`).

Artifacts (Final Run 6):
- HTML report: [`robosuite-parity-v2/robosuite_prior_work_report.html`](robosuite-parity-v2/robosuite_prior_work_report.html)
- JSON report: [`robosuite-parity-v2/robosuite_prior_work_report.json`](robosuite-parity-v2/robosuite_prior_work_report.json)
- Raw execution log: [`robosuite_v2_run6.log`](robosuite_v2_run6.log)
- Verified SkillExperience store: [`robosuite-parity-v2/robosuite_skills.json`](robosuite-parity-v2/robosuite_skills.json)
- Frozen policies: [`robosuite-parity-v2/policies/`](robosuite-parity-v2/policies/)

| Method | Overall (`n=80`) | Wilson 95% CI | Lift | PickPlaceCan | NutAssemblySquare | Door | Dev LLM / Resets | Eval LLM | Mean / P95 Wall | Paired McNemar vs CRAFT |
|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---|
| **CRAFT (RepairScreener + Regression + Skills)** | **78/80 (97.5%)** | `[91.3%, 99.3%]` | **20/20** | **20/20** | **18/20** | **20/20** | **11 / 55** | **0** | **1.73s / 2.27s** | reference |
| CaP-X-Protocol (Adapted) | 70/80 (87.5%) | `[78.5%, 93.1%]` | 20/20 | 20/20 | 11/20 | 19/20 | 15 / 75 | 9 | 6.80s / 47.78s | **+10.00 pp**, `p = 0.038574` (`+10/-2`) |
| ASPIRE-Protocol (Adapted) | 49/80 (61.3%) | `[50.3%, 71.2%]` | 20/20 | 20/20 | 0/20 | 9/20 | 15 / 75 | 0 | 1.68s / 2.09s | **+36.25 pp**, `p < 0.0001` (`+29/-0`) |
| SimEX-Direct-Repair (Ablation) | 40/80 (50.0%) | `[39.3%, 60.7%]` | 20/20 | 20/20 | 0/20 | 0/20 | 15 / 75 | 0 | 1.77s / 2.07s | **+47.50 pp**, `p < 0.0001` (`+38/-0`) |
| Strong Zero-Shot | 23/80 (28.7%) | `[20.0%, 39.5%]` | 20/20 | 0/20 | 0/20 | 3/20 | 4 / 0 | 0 | 1.52s / 1.98s | **+68.75 pp**, `p < 0.0001` (`+55/-0`) |

### Honest Iteration History (All Prior Runs Preserved)
- [`robosuite-parity-v2-run2/`](robosuite-parity-v2-run2/robosuite_prior_work_report.html) (`max_dev_calls=4`, unpaired `it=1`): CRAFT `59/80 (73.8%)` (`Door 20/20`, `NutAssemblySquare 0/20` due to missing `final_dz` attribution) vs ASPIRE `40/80 (50.0%)`.
- [`robosuite-parity-v2-run3/`](robosuite-parity-v2-run3/robosuite_prior_work_report.json) (`max_dev_calls=4`, unpaired `it=1`): exposed Door latch release threshold (`1.15 rad` was still latched; actual release `>= 1.45 rad`) and uncaptured `policy_return`.
- [`robosuite-parity-v2-run4/`](robosuite-parity-v2-run4/robosuite_prior_work_report.json) (`max_dev_calls=5`, paired `it=1`): Door jumped to `17/20` (vs ASPIRE `0/20`) in 2 calls; exposed `never_closed_gripper_early_abort` and mode-switching (`Explore` vs `Localized Polish`).
- [`robosuite-parity-v2-run5/`](robosuite-parity-v2-run5/robosuite_prior_work_report.json) (`max_dev_calls=5`, paired `it=1`): PickPlaceCan recovered to `20/20`; exposed 5× duplicated attribution prompt bloat and led to the clean de-bloated prompt + cross-task `SkillExperienceStore` + vertical-descent scoring that achieved **78/80 (97.5%)** in Run 6.

### Reproduction Command

```bash
.venv/bin/craft robosuite-bench \
  --output experiments/goal-v2-20261122/robosuite-parity-v2 \
  --tasks Lift,PickPlaceCan,NutAssemblySquare,Door \
  --dev-seeds 101,102,103,104,105 \
  --eval-seeds 1-20 \
  --provider codex_cli --model gpt-6-astra --effort medium --max-dev-calls 6
```

---

## 4. Milestone 4 — Engineering Quality & Test Verification

- **Pure stdlib core verified**: Across all modules in `src/craft/`, non-stdlib imports exist **only** in optional simulation drivers/benchmarks (`mujoco_cafe_driver.py`: `PIL`, `mujoco`, `numpy`; `robosuite_bench.py`: `numpy`, `robosuite`).
- **Full pytest suite**: `.venv/bin/pytest -q` → **99 passed, 0 failed (`100%`)**.
- **Literature & Attribution**: SimEX (`arXiv:2609.38982`, `hu2026simex`) recorded honestly in `references/catalog.json`, `references/references.bib`, `references/README.md`, and `references/notes/simex-full-text-review.md`.
