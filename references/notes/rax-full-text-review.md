# RAX text review — 2026-11-24

- **Stable cite key and exact source/version:** `huang2026rax` — *Scaling Online Exploration for Robot Agents*, `https://arxiv.org/abs/2610.22496` (arXiv:2610.22496v1, submitted 2026-10-15; PDF `https://arxiv.org/pdf/2610.22496`).
- **Authors:** Runhan Huang, Xiangcheng Zhang, Haoyu Chen, Yitong Li, Yen-Jen Wang, Bohan Lyu, Haotian Ye, Chuye Hong, Yixuan Wang, Chloe Su, Haoyu Xiong, Yangzhen Wu, Enze Xie, Travis Manderson, Stefano Ermon, Pieter Abbeel, Antonio Torralba, Dawn Song, Weirui Ye, Yilun Du.
- **Reading date / coverage:** 2026-11-24. Abstract, Sections 1–6 (Introduction, Related Work, Cognitive Architecture, System, Experiments, Ablations, Continual Learning, Discussion), and Tables 1, 2, 4, 7, 8 text reviewed from arXiv PDF text extraction. Figure images and full appendices not independently audited. Not reproduced.

## Problem and scope

RAX (*Robot Agent eXploration*) studies scaling online exploration for general-purpose coding agents (Claude Code, Codex) directly across diverse robot embodiments—without teleoperation demonstration corpora, hand-engineered action primitives, or value-function guidance.

## Method and system architecture (Sections 3–4)

- **Frontend–Backend separation (Section 4.1, Figure 4):**
  - **Backend** owns hardware drivers/simulators, low-level controllers (joint PD/impedance, OSC, whole-body tracker), calibrated RGB-D + intrinsics/poses, continuous episodic recording, and **hard safeguards** (operator arming, workspace bounds, joint limits, self-collision, joint load; refusing invalid commands with the exact violated limit before moving, plus robot-only dry-run preview).
  - **Frontend** exposes backend capabilities via MCP tool interfaces + 4-axis documentation skills (task strategy, coordination, perception, robot-specific control). The coding agent writes 3D perception pipelines, object geometric models, planning, and control programs inside an isolated container.
- **Exploration-to-Program compilation (Section 3.3):** Starts with interactive LLM tool-call deliberation and progressively compiles successful multi-stage decision sequences into standalone modular Python programs that execute at robot rate, dropping inference overhead dramatically (e.g., `make toast random`: 17 min initial deliberation → 6 s compiled program; `loading the car`: 123 min / 254k output tokens → 4 min / 4k tokens, Section 5.3.3).
- **Continual learning with multi-timescale memory (Section 3.4, 5.5):** Session working notes (`PROGRESS.md`, `TIPS.md`) + a per-robot **versioned skill repository** (session branches merged only after structure, entry point, and unattended execution evidence are validated). Supports modular subroutine reuse and selective adaptation (e.g., 20/41 functions preserved on `find-the-cube` variant; wiping module imported unchanged for paper-towel task, Section 5.5.2).
- **Parallel multi-agent / multi-environment exploration (Section 3.5, 5.3.1):** 4 agents × 4 parallel environments on `make toast` reaches 87.5% success / 92.2 score in 4.9h vs. 1 agent × 1 env reaching 50.0% / 63.3 in 13.7h (`Table 7`).

## Key reported results (not cross-paper ranking; exact paper scope)

- **Simulation (6 benchmarks, Section 5.2):** Outperforms top baselines by **+37.4% on average** across RoboDojo (bimanual ARX X5), BEHAVIOR-1K (Galaxea R1Pro), HumanoidArena (Unitree G1 + Dex3: **89.8%** avg across 60 official test episodes, +7.9 pts over WB-WAM, `Table 4`), DexJoCo (Franka + 16-DoF Allegro), and LIBERO / LIBERO-Pro (exceeds VPP2 by +33.7 pts and **ASPIRE by +12.8 pts**).
- **Depth ablation (`Table 8`):** RGB-D outperforms RGB-only by **+17.3 points** on average across domain-randomized RoboDojo tasks (`31.1%` vs `13.8%`).
- **Real robots (Sections 5.1, 5.5):** Franka FR3 (5 tasks: memory cup shuffle, air-fryer articulated drawer + banana, wet/paper towel wiping, 0.1 mm peg insertion) and wheeled dual-arm Rainbow RB-Y1 (6-stage long-horizon lab tidying composing prior skills).

## Comparison with SimEX (`hu2026simex`)

- **Where RAX goes beyond SimEX:**
  1. **Multi-embodiment breadth:** Covers 5 embodiment families across 6 public benchmarks + 2 real robots (including dexterous hands, mobile manipulators, and legged humanoids), whereas SimEX evaluates one bimanual YAM platform.
  2. **Explicit Backend Safeguards & Dry-Run Preview:** Clean frontend–backend split with pre-motion limit validation and continuous recording.
  3. **Direct online scaling & open-source positioning:** No simulator reconstruction step required; announces an open-source framework (repo URL/license to be verified once released).
- **Where SimEX remains distinct / stronger:**
  1. **Tight physical trial budget (≤5 trials):** SimEX offloads exploration and multi-hypothesis screening to an aligned simulator sandbox before touching real hardware. RAX scales **16–64–128 online exploration episodes** (4.9–14.2 agent-hours in `Table 7`), which is expensive or unsafe when physical resets/consumables are costly.
  2. **Explicit real-to-sim parameter alignment & counterfactual screening:** SimEX aligns simulator parameters to reproduce real failures and screens multiple diagnostic hypotheses offline before deploying.

## Overlap with CRAFT and remaining gaps for commercial restaurant deployment

- **What RAX still does NOT solve for restaurant/commercial operations:**
  1. **Non-idempotent external appliances & crash/restart reconciliation:** RAX assumes interactive online exploration or exception-triggered LLM takeover (`Section 3.3`). It has no intent-before-dispatch journal, query-first command reconciliation, or protection against duplicate irreversible appliance actuations (brewing, oven heating) after timeouts or process crashes.
  2. **Perception ownership boundary:** RAX delegates 3D perception pipeline synthesis (`RGB-D → point clouds / geometric models`) to the coding agent (`Section 3.1`). In `ticos-edge` / `EvidenceScene` deployments, perception must remain host-owned to prevent unverified parallel estimators and preserve 4-layer fault attribution.
  3. **Independent acceptance & evidence separation:** RAX merges skills based on unattended agent runs rather than an independent frozen acceptance gate with zero-actuation `recorded_data` shadow replay and explicit evidence-tier separation.
  4. **Multi-appliance concurrent throughput scheduling:** RAX targets single-task / sequential subtask composition, not concurrent pipelining across asynchronous commercial appliances on a shared arm.
- **Methodology ideas compatible with CRAFT:**
  1. Pre-motion backend dry-run / limit-refusal diagnostics returning exact violated constraints.
  2. Versioned skill branching + selective subroutine patching validated against historical recorded logs before promotion.
