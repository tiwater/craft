# TRAX 2.0 Research Plan — Publication Roadmap & Exit Criteria

**TRAX** (*Transactional Robot Action Execution*): Transactional, Crash-Recoverable, and Ownership-Separated Code Policies for Real-World Robotics.

> **Evidence boundary notice:** This document states research hypotheses, experimental milestones, and quantitative exit criteria. Planned tracks are **not** completed claims. Every reported result must carry an explicit `evidence_source` tag (`fixture`, `simulator`, `recorded_data`, or `physical_robot`).

---

## Core Research Question

Can a strictly ownership-separated, pure-stdlib transactional execution and bounded offline evolution layer—combining intent-before-dispatch SQLite journaling, four-layer failure attribution, task-agnostic public-SDK active physical probing, closed-loop simulator calibration, AST-guided selective subroutine repair, decoupled acceptance, and contract-drift-aware executable skill toolboxes—close the reliability gap between exploratory coding agents (ASPIRE, CaP-X, SimEX, RAX) and continuous real-world robotic operations without online LLM latency?

---

## Phase 1: Exhaustive Simulation Verification (Current Priority)

Goal: Eliminate all "home-court advantage" objections, establish scientific scaling/ablation laws, and validate visual-degradation attribution in simulation before physical-robot trials.

### M1 — External Public Benchmarks (Zero Task-Hint Leakage)
1. **Zero-Leakage Robosuite v4 (4 Tasks × 20 Held-Out Seeds = 480 Rollouts):**
   - Full rerun of `Lift`, `PickPlaceCan`, `Door`, `NutAssemblySquare` with AST Selective Subroutine Repair, adaptive stagnation escape, negative-attempt memory, and 5 dev seeds (`101..105`).
   - **Exit criterion:** TRAX (`0` online eval LLM calls) ranks #1 among all frozen Turn-1 policies (beating adapted ASPIRE and SimEX-Protocol overall; `60/60` on `Lift`/`PickPlaceCan`/`Door`, `>= 14/20` on `NutAssemblySquare`).
2. **Official LIBERO-Spatial Full Suite (10/10 Tasks × 10 Held-Out Init States = 100 Trials/Method):**
   - Per-task subprocess isolation with checkpoint resume (eliminating upstream `robosuite 1.4.0` XML FD leaks), 4-direction free-space rim probing, exact AST-violation feedback, and `dev_count=5`.
   - Explicitly tagged `observation_contract="host_state_api"` (matched-contract comparison only; never ranked numerically against raw RGB-D RAX).
   - **Exit criterion:** Complete 10/10 suite archived with TRAX outperforming adapted ASPIRE, SimEX-Protocol, and Strong Zero-Shot.

### M2 — Component Ablations & Scaling/Degradation Curves
1. **Full 6-Component Ablation Matrix:** Systematically disable (a) SQLite Journal, (b) 4-Layer Attribution, (c) ActiveProbe, (d) Closed-Loop Sim Calibration, (e) Selective Subroutine Repair & Stagnation Escape, and (f) Verified Skill Toolbox.
2. **Three Publication Curves (auto-generated from archived JSON receipts):**
   - **Disturbance / Mismatch Degradation Curve:** Pass rate and duplicate non-idempotent actions across slip magnitude (`10–50 mm`), sensor noise `σ`, and crash probability (`0–40%`).
   - **Budget–Performance Pareto Curve:** Held-out success vs. dev LLM budget (`1, 2, 4, 6, 10` calls) and wall time.
   - **100-Order Long-Horizon Stress & Skill-Reuse Curve:** Continuous mixed-order execution under injected crashes, perception drift, appliance faults, and host contract drift, measuring cumulative duplicates (`0`) and skill-toolbox adaptation speedup across task variants.

### M3 — Simulated RGB-D Visual Degradation ↔ Four-Layer Attribution Loop
- Render multi-camera RGB-D frames from the MuJoCo 7-DOF Panda PBR cell with controlled occlusion, depth dropout, dual-object ambiguity, and pose noise.
- Couple an EvidenceScene-style observability gate with TRAX's four-layer attribution (`perception` / `control` / `device` / `policy`).
- **Exit criterion:** `0` false actuations under degraded perception; failures attributed accurately to `perception` rather than triggering policy-code overfitting.

### M4 — Formal Transactional Guarantees & Engineering Consolidation
- Formalize and prove two safety propositions in Section III of `papers/trax/main.tex`:
  1. *At-Most-Once Physical Actuation under Cooperative Host Idempotency Keys* (and explicit impossibility without host persistence).
  2. *Fail-Closed Reconciliation and Perception-Drift Isolation.*
- Consolidate the lightweight web observer under **`TRAX UI`** (`trax ui`, `src/trax/ui.py`) and ensure every new JSON receipt flows deterministically through `make artifacts && make paper`.

---

## Phase 2: Dual-Hardware Physical Validation (After Phase 1 Exit)

All physical tracks require a frozen, pre-verified Host Capability Contract (workspace/joint limits, hardware E-stop, timeouts, `query_action_status`, and dry-run preview) per `AGENTS.md`.

1. **Cross-Embodiment Low-Cost Arm (LeRobot):**
   - Thin `LeRobotHostDriver` with zero changes to `src/trax/` stdlib core.
   - 2–3 tabletop manipulation / physical switch-actuation tasks (`n >= 20` controlled trials per task), quantifying real-world ActiveProbe compensation for servo backlash and drift.
2. **Commercial-Grade Arm + Live RGB-D Perception (RealMan + EvidenceScene):**
   - Real non-idempotent appliance crash/network-drop reconciliation (`n = 20`, `0` duplicate physical actuations).
   - Real Sim-to-Real `<= 5`-trial adaptation under physical displacement and friction mismatch.
   - Live RGB-D hand occlusion / dual-cup ambiguity / distractor trials (`n = 15`, `0` false actuations via EvidenceScene observability gate + TRAX attribution).
   - Continuous 50–100 order physical endurance run with full video and JSON receipt archive.
