# SimEX text review — 2026-11-22

- **Stable cite key and exact source/version:** `hu2026simex` — *SimEX: Simulation-Integrated Robotics AutoResearch*, `https://arxiv.org/html/2609.38982` (arXiv:2609.38982v1, submitted 2026-09-30; project page `https://robo-simex.github.io/`).
- **Reading date / coverage:** 2026-11-22. Sections 1–5 and Appendices A–E text and Tables 1–9 reviewed. Figure images (Figures 1–11) were not independently audited. Not reproduced.

## Problem and scope

SimEX studies how LLM coding agents can acquire physical robot manipulation capabilities from a low-level robot API without demonstrations and under a tight physical trial budget (5 trials / ~10 minutes per task on a bimanual YAM robot). Instead of direct one-shot Code-as-Policies or extensive real-world trial-and-error, SimEX treats a roughly correct simulator as an experimentation laboratory.

## Method and state owners (Sections 2.1–2.4)

- **Persistent artifact (Section 2.1):** A paired portable toolbox consisting of `toolbox.py` (perception and control primitives built on top of a fixed low-level robot API) and `skill.md` (usage guidance, measured limits, and known failure modes). Task-specific CaP scripts are generated fresh per trial and discarded after execution; cross-iteration learning persists only in the toolbox.
- **Preparation (Section 2.2):** A coding agent reconstructs an approximate simulation sandbox (e.g., `mjlab`) from a workspace image, natural-language object description, an empty YAM station template, and a predefined 3D mesh library, initializing `toolbox.py` and `skill.md`.
- **Stage 1 — Open-ended capability discovery in simulation (Section 2.3):** Runs 4 independent population members for 20 iterations each (`Table 2`). Each cycle runs: (1) broad capability probing (24–32 self-generated task/workspace variations with agent-written simulator-state success predicates), (2) sub-goal selection isolating one systematic weakness (~10 focused tasks), and (3) sub-goal-driven hill-climbing jointly evaluated with capability-bank replay (4–6 previously solved tasks to prevent regressions). Population members exchange short textual summaries at probe boundaries without directly copying code.
- **Stage 2 — Simulation-assisted few-trial adaptation (Section 2.4):** Budget of 5 real-robot rollouts (1 rollout/iteration, human scene resets). After each rollout, the harness replays the fixed executed policy in simulation; the coding agent updates the simulator until replay reproduces the observed deployment behavior, formulates distinct diagnostic hypotheses, implements 4 candidate toolbox repairs, and screens all candidates plus the current toolbox on fixed-script replay and 5 agent-written simulated test scenes. The coding agent inspects videos, traces, and scores to choose which candidate (or none) to run next—without an automatic rank-selection rule.

## Action/observation interface and privileged inputs (Sections 2.1, 3.2)

- **Deployment/toolbox boundary:** Raw RGB-D images (1 overhead + 2 wrist cameras), joint/end-effector proprioception, IK/reachability queries, Cartesian EE or absolute joint targets, and gripper commands. No predefined task-level manipulation skills and no privileged target-environment state.
- **Privileged simulation access during Stage 1:** In the sandbox simulator, the agent writes executable success predicates over simulator state and can edit simulator parameters during Stage 2 alignment; it cannot inspect the evaluation simulator's state or source code in sim-to-sim tests.

## Development vs held-out protocol & setting-matched baselines (Section 3.1, Appendix C)

- **Tasks:** Three families (7 held-out tasks each in sim-to-sim; 1 representative task each on real hardware): *plate to tote*, *towel folding*, and *barcode scanning*.
- **Sim-to-sim transfer pairs (Section 3.2):** MuJoCo → Isaac Sim (plate, barcode); FLASH → MJWarp Flex (towel).
- **Evaluation protocol:** 5 adaptation trials per adaptive method, then freeze the toolbox and run 5 i.i.d. rollouts across 5 random seeds (25 trials/task in sim-to-sim; 10 physical trials/task on real hardware).
- **Asterisk (`*`) baseline caveat (Appendix C):** `ENPIRE*` and `ASPIRE*` are reimplementations under SimEX's shared low-level API, 5-trial physical budget, and matched simulation pretraining budget. Specifically, `ENPIRE*` is restricted to 5 code-policy deployment rollouts (without ENPIRE's automated reset/verification harness or multi-hour fleet/RL training), and `ASPIRE*` builds a textual skill library in simulation for deployment-time repair without SimEX's paired executable toolbox evolution or Stage 2 simulator replay/screening.

## Runtime vs development LLM calls (Appendix A.2, A.3, D.4)

- **Development cost (`Table 3`, barcode scanning average, Fable 5.1):**
  - Stage 1 (per member; default uses 4 members): 840 orchestrator calls, 5.4 h agent thinking time, 1.98M output tokens (1.16M reasoning), 155M cache-read input tokens.
  - Stage 2 (per run, 5 robot trials): 124 orchestrator calls, 31 min thinking time, 161k output tokens (64k reasoning), 19.5M cache-read input tokens.
- **Runtime inference (`Appendix A.2`, `Appendix D.4`):** One text-only one-shot CaP call (~30 s) generates the task script before execution (no in-loop model calls; reusable when the task interface is unchanged). Comparing against online step-by-step agent control on 3 barcode tasks (`Appendix D.4`): SimEX completes 3/3 validation rollouts; online control averages ~30 calls/rollout and 40 min waiting (>50× motion time) with 0/15 completions within budget.

## Failure / unknown-outcome handling

- Focuses on geometric, contact, deformable, and multi-object physical manipulation failures. Human operators manually reset scenes between Stage 2 trials.
- Does **not** address non-idempotent external appliances (e.g., beverage brewers, ovens), delayed/lost command receipts, crash/restart reconciliation, or multi-owner perception/interlock authority boundaries.

## Reported results with exact tables and denominators (not cross-paper ranking)

- **Physical robot (`Table 1`, successful evaluation trials out of 10 per task; 30 total across 3 tasks):**
  - `Direct CaP`: Plate 0/10, Towel 0/10, Barcode 0/10 (0/30)
  - `ENPIRE*`: Plate 2/10, Towel 1/10, Barcode 0/10 (3/30)
  - `Zero-Shot Sim2Real`: Plate 0/10, Towel 2/10, Barcode 0/10 (2/30)
  - `ASPIRE*`: Plate 2/10, Towel 0/10, Barcode 0/10 (2/30)
  - `SimEX (ours)`: Plate 10/10, Towel 8/10, Barcode 8/10 (26/30)
- **Sim-to-sim barcode scanning (`Table 4` / `Table 6`, Fable 5.1, 7 tasks × 25 trials = 175):**
  - `SimEX`: 149/175 (85%); `SimEX (direct repair)`: 102/175 (58%); `SimEX (no discovery)`: 42/175 (24%); `ASPIRE*`: 32/175 (18%); `Zero-Shot Sim2Real`: 2/175; `Direct CaP`: 1/175; `ENPIRE*`: 0/175.
- **Fine-grained ablations on barcode scanning (`Table 5`, out of 25/task):** Removing Stage 1 population diversity (`No population`) or capability-bank replay (`No capability bank`) causes the largest drops (e.g., *Bottle + block* drops from 14/25 to 1/25 without population and 7/25 without capability bank; *Wall-side bottle* drops from 25/25 to 15/25 without capability bank).
- **Cross-agent robustness (`Tables 6–9`, barcode scanning):** SimEX remains the top method across Fable 5.1, Opus 5, GPT-5.5 (67% avg), and GPT-6 Astra (82% avg), though absolute success varies substantially with the underlying coding agent.

## Authors' limitations vs our inference

- **Authors' stated limitations (Section 5):** Evaluated on only one robot platform (dual-arm YAM); Stage 2 still requires human scene resets between physical trials.
- **Our inference:**
  1. Stage 1 offline compute/token cost (~21.6 agent-hours and ~3,360 calls across 4 members) is heavy for fast per-store onboarding unless scoped and budget-capped.
  2. Preparation still relies on a predefined station template and 3D mesh library.
  3. Letting the coding agent write its own Stage 1 success predicates and make Stage 2 selection decisions couples proposer and evaluator; production deployment requires an independent acceptance gate.
  4. `Zero-Shot Sim2Real` scoring 2/30 on physical hardware confirms that simulation screening cannot substitute for physical-robot evidence.

## Code/license and reproducibility

- **Official code availability:** Checked `https://arxiv.org/abs/2609.38982` and `https://robo-simex.github.io/` on 2026-11-22. No official public source repository or software license is linked or confirmed. Treat as methodology reference only; do not assume reusable code.

## Overlap with CRAFT and remaining questions

- **What CRAFT can selectively adopt (methodology only):**
  1. **Replay-and-screen Stage 2 pattern:** Turn scarce real-robot/site failures into simulation parameter reproductions, multiple distinct diagnostic hypotheses, candidate repairs, and simulation regression screening before proposing a release candidate to CRAFT's independent acceptance gate.
  2. **Capability-bank replay:** Enforce regression checks against previously solved tasks and historical fault cases on every repair or optimization edit.
  3. **Paired boundary documentation:** Store explicit measured applicability limits and known failure modes alongside verified skill records.
  4. **Empirical support for offline synthesis over online agent control (`Appendix D.4`):** Reinforces CRAFT's fixed-version frozen policy execution model.
- **What CRAFT must NOT adopt:**
  1. Do **not** let CRAFT synthesize raw RGB-D perception or low-level motion primitives (`ticos-edge` / `EvidenceScene` remain sole perception and actuation authorities).
  2. Do **not** let the proposing coding agent define or mutate acceptance criteria; independent host oracles and CRAFT release gates remain mandatory.
  3. Do **not** conflate simulation screening evidence with physical-robot approval.

## Reproduction status and evidence path

- Not reproduced (no public author code as of 2026-11-22). Literature record maintained in `references/catalog.json` (`hu2026simex`) and `references/references.bib`.
