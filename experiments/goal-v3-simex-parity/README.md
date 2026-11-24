# CRAFT v3 / Phase 4 Experimental Archive (`goal-v3-simex-parity`)

> **Evidence boundary notice**: All benchmark results archived here carry explicit `evidence_source` tags (`simulator` or `recorded_data`). None of these simulation or shadow-replay passes constitute physical-robot approval or hardware safety certification.

---

## Summary of Verified Phase 4 Results

| Benchmark | Evidence Source | Key Result | Artifact |
|---|---|---|---|
| **1. Sim-to-Sim Stage-2 (5-Trial Adaptation, Run 2)** | `simulator` (`mujoco_cafe` nominal → noisy+DR target) + `recorded_data` shadow | **CRAFT 19/20 (95.0%, `target=15/16`, `shadow=4/4`, 0 dup, 0 false act)** vs `SimEX-Protocol (Adapted)` 16/20 (80.0%), `Direct Target Repair` 16/20 (80.0%), `Zero-Shot Sim2Sim` 13/20 (65.0%, exact McNemar `p = 0.03125`). Used 5 target trials, 3 closed-loop sandbox calibration trials, 4 LLM calls. | [`sim2sim-stage2/sim2sim_report.html`](sim2sim-stage2/sim2sim_report.html) |
| **2. CafeBench v3 Multi-Appliance Combo (Coffee + Bagel/Oven)** | `simulator` (`ComboWorkstationHost`) | **CRAFT Pipelined + Dual-Appliance Journal: 12/12 (100.0%, 0 dup, 0 interlock violations, 47.45 s/order = 75.9 orders/hr)** vs Sequential Stateless 9/12 (75.0%, 3 dup, 4 interlocks, 77.1 s/order = 46.7 orders/hr, **+62.5% throughput**) and Pipelined Stateless 9/12 (75.0%, 3 dup, 6 interlocks). | [`combo-v3/combo_report.json`](combo-v3/combo_report.json) |
| **3. Zero-Leakage Robosuite v3 (Pilot: `Door` + `Nut`, 3 dev seeds)** | `simulator` (`mujoco_robosuite` Panda, 0 hardcoded task hints) | Pilot run (`dev_seeds=[101..103]`, `eval_seeds=1..10`): **CRAFT 8/20 (40%, 0 eval LLMs, p95 2.18s)** vs `SimEX-Protocol` 0/20 (`p=0.0078`), `SimEX-Direct` 0/20, `Zero-Shot` 0/20, `ASPIRE` 7/20 (35%), `CaP-X` 11/20 (55%, 3 eval LLMs, p95 55.64s). | [`robosuite-v3-zero-leakage/robosuite_prior_work_report.html`](robosuite-v3-zero-leakage/robosuite_prior_work_report.html) |
| **4. Full Zero-Leakage Robosuite v3 (4 Tasks × 20 Seeds = 480 Rollouts, 5 dev seeds)** | `simulator` (`mujoco_robosuite` Panda, 0 hardcoded task hints, commit `e467bc3`) | **Lift 20/20, PickPlaceCan 20/20, Door 20/20 (60/60 = 100% on 3/4 tasks)**; overall: `CaP-X` **78/80 (97.5%, 3 eval LLMs)** > `ASPIRE` **70/80 (87.5%)** > **CRAFT 64/80 (80.0%, 0 eval LLMs, p95 2.32s)** > `SimEX-Protocol (Adapted)` **62/80 (77.5%)** > `SimEX-Direct` **59/80 (73.8%)** > `Zero-Shot` **40/80 (50.0%, `p < 0.0001`)**. Honest gap: on `NutAssemblySquare` (4/20 vs ASPIRE 11/20, CaP-X 18/20), CRAFT's `H-Polish` constraint over-constrained edits after reaching `3/5` dev at `it=3`. | [`robosuite-parity-v3/robosuite_prior_work_report.html`](robosuite-parity-v3/robosuite_prior_work_report.html) |

---

## Honest Engineering Findings & Preserved Runs

1. **Sim2Sim Run 1 (`sim2sim-stage2-run1/`)**: Exposed two real bugs in `RepairScreener` — `build_hypothesis_repair_prompt()` omitted AST sandbox rules (causing `for _ in range(...)` rejections) and failed step traces, and preflight did not invoke `check_sandbox_ast_compliance()`. Both were fixed in `src/craft/repair_screener.py`, raising CRAFT from `8/20` (Run 1) to **`19/20 (95.0%)`** (Run 2). Run 1 artifacts are preserved honestly.
2. **Zero-Leakage Robosuite v3 (`robosuite-v3-zero-leakage/`)**:
   - Removing all hardcoded task-specific physics hints revealed the true low-budget difficulty of `Door` and `NutAssemblySquare`.
   - CRAFT's 1-reset public-SDK Active Physical Probe autonomously discovered `NutAssemblySquare`'s `local_offset_m = 0.055m` (`lift = 0.0693m`) and `Door`'s approach/press telemetry without any task hardcoding, allowing CRAFT to hit `3/3` on `Door` dev seeds in 2 LLM calls (vs `0/3` for ASPIRE and `1/3` for SimEX-Protocol).
   - Remaining gap observed honestly: stopping early on only 3 dev seeds (`dev_seeds=[101,102,103]`) caused `Door` held-out generalization to land at `4/10` (and `NutAssemblySquare` at `4/10` vs `CaP-X` `7/10`), indicating that a 5-seed dev validation gate + broader probe sweep is the clear next step to push zero-leakage Robosuite generalization above `CaP-X` without online LLM latency.
