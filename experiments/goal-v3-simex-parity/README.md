# CRAFT v3 / Phase 4 Experimental Archive (`goal-v3-simex-parity`)

> **Evidence boundary notice**: All benchmark results archived here carry explicit `evidence_source` tags (`simulator` or `recorded_data`). None of these simulation or shadow-replay passes constitute physical-robot approval or hardware safety certification.

---

## Summary of Verified Phase 4 Results

| Benchmark | Evidence Source | Key Result | Artifact |
|---|---|---|---|
| **1. Sim-to-Sim Stage-2 (5-Trial Adaptation, Run 2)** | `simulator` (`mujoco_cafe` nominal → noisy+DR target) + `recorded_data` shadow | **CRAFT 19/20 (95.0%, `target=15/16`, `shadow=4/4`, 0 dup, 0 false act)** vs `SimEX-Protocol (Adapted)` 16/20 (80.0%), `Direct Target Repair` 16/20 (80.0%), `Zero-Shot Sim2Sim` 13/20 (65.0%, exact McNemar `p = 0.03125`). Used 5 target trials, 3 closed-loop sandbox calibration trials, 4 LLM calls. | [`sim2sim-stage2/sim2sim_report.html`](sim2sim-stage2/sim2sim_report.html) |
| **2. CafeBench v3 Multi-Appliance Combo (Coffee + Bagel/Oven)** | `simulator` (`ComboWorkstationHost`) | **CRAFT Pipelined + Dual-Appliance Journal: 12/12 (100.0%, 0 dup, 0 interlock violations, 47.45 s/order = 75.9 orders/hr)** vs Sequential Stateless 9/12 (75.0%, 3 dup, 4 interlocks, 77.1 s/order = 46.7 orders/hr, **+62.5% throughput**) and Pipelined Stateless 9/12 (75.0%, 3 dup, 6 interlocks). | [`combo-v3/combo_report.json`](combo-v3/combo_report.json) |
| **3. Zero-Leakage Robosuite v3 (`Door` + `NutAssemblySquare`)** | `simulator` (`mujoco_robosuite` Panda, 0 hardcoded task hints) | With 100% of hardcoded physics constants removed (`1.45 rad`, `0.055m`, etc.) and replaced by 1-reset public-SDK **Active Physical Probing**: **CRAFT 8/20 (40%, 0 eval LLMs, p95 2.18s)** significantly outperforms matched-budget **`SimEX-Protocol (Adapted)` 0/20 (`p = 0.0078`)**, `SimEX-Direct` 0/20 (`p = 0.0078`), and `Zero-Shot` 0/20 (`p = 0.0078`); beats `ASPIRE` 7/20 (35%, Door 0/10); trails `CaP-X` 11/20 (55%, which uses 3 online eval LLM calls and 25× higher p95 latency 55.64s). | [`robosuite-v3-zero-leakage/robosuite_prior_work_report.html`](robosuite-v3-zero-leakage/robosuite_prior_work_report.html) |

---

## Honest Engineering Findings & Preserved Runs

1. **Sim2Sim Run 1 (`sim2sim-stage2-run1/`)**: Exposed two real bugs in `RepairScreener` — `build_hypothesis_repair_prompt()` omitted AST sandbox rules (causing `for _ in range(...)` rejections) and failed step traces, and preflight did not invoke `check_sandbox_ast_compliance()`. Both were fixed in `src/craft/repair_screener.py`, raising CRAFT from `8/20` (Run 1) to **`19/20 (95.0%)`** (Run 2). Run 1 artifacts are preserved honestly.
2. **Zero-Leakage Robosuite v3 (`robosuite-v3-zero-leakage/`)**:
   - Removing all hardcoded task-specific physics hints revealed the true low-budget difficulty of `Door` and `NutAssemblySquare`.
   - CRAFT's 1-reset public-SDK Active Physical Probe autonomously discovered `NutAssemblySquare`'s `local_offset_m = 0.055m` (`lift = 0.0693m`) and `Door`'s approach/press telemetry without any task hardcoding, allowing CRAFT to hit `3/3` on `Door` dev seeds in 2 LLM calls (vs `0/3` for ASPIRE and `1/3` for SimEX-Protocol).
   - Remaining gap observed honestly: stopping early on only 3 dev seeds (`dev_seeds=[101,102,103]`) caused `Door` held-out generalization to land at `4/10` (and `NutAssemblySquare` at `4/10` vs `CaP-X` `7/10`), indicating that a 5-seed dev validation gate + broader probe sweep is the clear next step to push zero-leakage Robosuite generalization above `CaP-X` without online LLM latency.
