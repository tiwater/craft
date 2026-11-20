# Product architecture and selective reuse boundary

2026-11-20. Supersedes the early "thin-adapter-only workspace" positioning in
[`0002-current-reuse-recommendation.md`](0002-current-reuse-recommendation.md)
for product delivery, while preserving `0002` as the historical record of the
initial research-necessity investigation.

## Context and product direction

The primary goal is a usable scene-automation product rather than paper
novelty. CRAFT is the unified entry point through which an operator provides:

1. Scene connection and host endpoint configuration.
2. Target objects of interest and binding rules (consuming host perception).
3. Available host capabilities (discovered automatically where supported).
4. Independent acceptance criteria and experiment budgets/permissions.
5. Task teaching via text, video evidence, or host-contract robot
   demonstrations.

From these inputs, CRAFT orchestrates policy development, independent
acceptance verification, fixed-version release execution, rollback, and
evidence-backed iterative improvement.

**Core engineering principle:** Do not make the entire upstream ASPIRE stack a
mandatory runtime dependency, and do not re-implement perception or robot
control already owned by `ticos-edge` / EvidenceScene. Implement CRAFT's core
business orchestration directly in `src/craft/`, selectively reusing ASPIRE's
proven designs/patterns and ACE's on-demand teaching/experiment methodology.
Writing code locally does **not** automatically make it more reliable than
upstream; every CRAFT module must carry behavioral tests covering failure,
restart, identity drift, budget limits, and partial execution.

## Upstream audit summary

- **ASPIRE** (`https://github.com/NVlabs/ASPIRE`, pinned commit
  `fa00f0ef2bc1d91d594b9aa502c22133c9f538f7`, Apache-2.0 for NVIDIA-owned
  source; third-party submodules carry separate licenses):
  - Audited codebase size (excluding `third_party/`): `src/` 106 Python files
    (30,133 physical lines), `tests/` 58 files (14,947 lines), iterative
    debugging / evosearch scripts 20 files (4,512 lines), Markdown 28 files
    (4,619 lines).
  - Most of `src/` consists of benchmark/embodiment adapters (LIBERO-Pro,
    Robosuite, BEHAVIOR, YAM real) and perception/planning providers (SAM3,
    Contact-GraspNet, cuRobo, Pyroki), which are already owned by `ticos-edge`
    and must not be duplicated.
  - 204 selected upstream software tests (96 core + 108 pipeline) passed in an
    isolated Python 3.10 environment; this verifies software contracts, not
    cafe manipulation or learning superiority.
- **ACE-CaP** (`672f9e515887e90aa14cc3b0899a402cb74e350c`):
  - Public repository contains only static website files (HTML 398, JS 257,
    CSS 297 lines plus media). No public controller implementation or reusable
    code license was confirmed.
  - Used strictly as a **methodology reference** for on-demand teaching,
    clarification requests, and targeted experiment acquisition. CRAFT does not
    claim to integrate an official ACE controller.
- **`ticos-edge` & `ticos`** (existing sibling repositories):
  - `ticos-edge` remains the sole perception, world-state, SDK, sandbox, and
    actuation authority (`EdgeCapRuntime._run_job`, `EdgeZeroGtCoffeeSDK`,
    `SDKPublicProxy`, `execute_policy_in_subprocess_sandbox`).
  - `ticos` remains a read-only presentation and transport consumer.

## Module-by-module reuse decision matrix

| Module / Capability | Upstream Source Files & License | Required Dependencies (Upstream) | Decision | Rationale & Behavioral Test Requirement |
| --- | --- | --- | --- | --- |
| **Host perception, SDK, sandbox & actuation** | `ticos-edge/tools/edge_cap_runtime.py`, `coffee_cap_sdk.py`, `serve_isaac_live_cap_daemon.py` (Tiwater internal host) | Live/sim edge HTTP service (`/cap/state`, `/cap/execute`) | **Direct host reuse** | Preserves `ticos-edge/AGENTS.md` authority. CRAFT consumes `EvidenceScene` snapshots and delegates policy execution to edge's subprocess sandbox without adding a second detector or controller. Test: missing capability, stale evidence, ambiguous object, disconnect, unknown receipt reconciliation. |
| **LLM generation transport** | `ticos-edge/tools/cap_llm_backends.py` (already adapted into `src/craft/backends.py`, `src/craft/generation.py`; Apache-2.0) | Python stdlib (`subprocess`, `urllib`) | **Direct reuse (already adapted)** | Supports explicit switchable `codex_cli` and `openai_api` backends with `medium` default reasoning effort, no silent fallback, and explicit draft status. Test: `tests/test_backends.py`, `tests/test_generation.py`. |
| **Iterative debugging & EvoSearch campaign scripts** | ASPIRE `.claude/skills/iterative-debugging/scripts/{campaign.py,init_run.py,run_validation.py,dispatch.py,gen_progress.py,trace_stats.py,watchdog.py,audit_seeding.py}`, `.claude/skills/evosearch/scripts/*` (Apache-2.0) | Python stdlib + `cap-harness` CLI + benchmark env vars (EGL/MuJoCo/OmniGibson) | **Implement in CRAFT (adapt patterns)** | Upstream scripts are coupled to `cap-harness run`, fixed simulator seed bands, and agent CLI layouts. CRAFT needs unified product orchestration with user teaching, clarification requests, pause/resume persistence, budget caps, and non-idempotent reconciliation. Adapt atomic JSON writes, content-hashed run IDs, and disjoint dev/acceptance splits. Test: budget stop, pause/resume recovery, failure retention, missing-material wait state. |
| **Sealed validation & provenance** | ASPIRE `src/cap_harness/validation/{model.py,provenance.py,plan.py,runner.py}` (Apache-2.0) | `model.py`/`provenance.py`: stdlib; `runner.py`: `cap-harness` + benchmark selectors | **Adapt design; implement CRAFT acceptance & release** | `provenance.py` provides clean stdlib patterns (SHA-256 hashing, frozen manifests, dirty-worktree checks), whereas `runner.py` hardcodes `behavior`/`libero-pro`/`robosuite` and `run_validation.py` counts `task_success` even when `program_ok=False`. CRAFT requires independent acceptance gates (completion rate, zero unreconciled unknown outcomes, zero duplicate non-idempotent actions, latency/slow-step tracking) decoupled from policy generation, plus fixed-version release and rollback. Test: failed gate blocks release, model cannot mutate criteria, rollback restores prior version, regression required on edit. |
| **Episode artifact & trial recorder** | ASPIRE `src/cap_harness/artifacts.py` (Apache-2.0) | `imageio[ffmpeg]`, `numpy`, `cap_harness.contracts` | **Implement lightweight store in CRAFT** | Pulling `imageio`/`numpy` into CRAFT core just to record structured JSON/SQLite trials adds heavy media dependencies while edge already owns RGB-D and section-9 audit records. Implement stdlib JSON/SQLite trial persistence with explicit `evidence_source` tagging (`fixture`, `simulator`, `recorded_data`, `physical_robot`). Test: corrupt/partial write recovery, evidence-source separation. |
| **Skill library, linter & independent verifier** | ASPIRE `.claude/skills/iterative-debugging/skills/*.md`, `scripts/lint_skills.py`, `verifier-prompt.md` (Apache-2.0) | `lint_skills.py`: stdlib + `cap_harness.runtime._ProgramValidator` | **Adapt methodology; implement CRAFT skill store** | Upstream `.md` skills are LIBERO/Robosuite/BEHAVIOR specific and include disallowed perception shortcuts (e.g., mask dilation). Adopt the *independent verification* principle (verifier checks applicability/evidence without blindly trusting proposer code) and AST/contract linting against discovered host capabilities. Store applicability conditions and verification provenance on every skill entry. Test: unverified skill cannot be promoted; stale capability hash invalidates skill applicability. |
| **ASPIRE in-process runtime & ToolRegistry** | ASPIRE `src/cap_harness/{runtime.py,registry.py,contracts.py,api.py,cli.py}` (Apache-2.0) | `numpy`, `scipy`, `requests`, `PyYAML` | **Do not adopt as CRAFT core dependency** | Proven usable as an optional experiment seam (`experiments/reuse-audit/`), but in-process execution does not replace edge's subprocess sandbox, SDK interlocks, or command ledger. Keeping CRAFT core stdlib-only avoids dependency conflicts with `ticos-edge`. |
| **ASPIRE perception/planning providers & benchmark envs** | ASPIRE `src/cap_harness/providers/*`, `{libero,robosuite,behavior,yam_real}/*`, `third_party/*` (Apache-2.0 + third-party licenses) | PyTorch, CUDA, MuJoCo, Open3D, cuRobo, Pyroki, SAM3, etc. | **Explicitly exclude** | Directly duplicates `ticos-edge` / EvidenceScene perception and robot control, violating repository source-of-truth rules and introducing large third-party license/runtime overhead. |
| **ACE-CaP teaching & experiment loop** | ACE-CaP static site (`index.html`, `static/*`; no public controller code) | N/A | **Methodology reference only** | Adopt on-demand teaching requests, explicit clarification when materials are missing, and targeted failure-driven experiments. Never treat video-inferred coordinates as authoritative live scene state. Test: missing teaching material enters explicit waiting state rather than fabricating demonstrations. |

## Phase 1 scope vs. future extensions

### Phase 1: Engineering must-haves (Single-cup coffee E2E simulation loop)

The first milestone is complete when a user can go from configuration and
teaching to verified execution and feedback-driven iteration on **single-cup
coffee** in the existing `ticos-edge` simulation environment:

1. **Configuration & Host Capability Adapter (`craft-h9k.2`)**
   - Declarative scene/host/object/capability/acceptance/budget config contract
     (no private paths, credentials, or hardcoded personal endpoints).
   - Automatic discovery of existing host capabilities from `/cap/state` and
     SDK contract hash verification.
   - Read-only EvidenceScene observation pass-through (staleness, ambiguity,
     and disconnect explicitly surfaced).
   - Query-first reconciliation for unknown non-idempotent outcomes.
   - Explicit switchable CLI/API generation backend configuration (`medium`
     default effort, tracked in run metadata).
2. **Unified Teaching Interaction & Policy Development Loop (`craft-h9k.3`)**
   - Structured task specification from text + teaching materials (video
     evidence metadata, host-contract trajectory demonstrations).
   - Explicit clarification/wait requests when required teaching evidence or
     bindings are missing (never fabricate demonstrations or coordinates).
   - Iterative generate -> validate -> execute -> diagnose -> revise loop with
     persistent state (candidates, prompts, sources, trial records, budgets,
     pause/resume checkpoints) and strict tagging of fixture vs. real
     model/simulator trials.
3. **Independent Acceptance, Fixed-Version Release & Improvement Queue (`craft-h9k.4`)**
   - Independent acceptance criteria locked outside policy generation.
   - Frozen release bundles (code hash + config hash + capability contract
     hash) evaluated on held-out / non-debug conditions.
   - Reliability-first gating (completion rate, unknown outcomes, duplicate
     non-idempotent actions, human interventions) before speed comparison;
     slow-but-successful runs queued as optimization candidates.
   - Fixed-version execution (no silent runtime mutation), explicit deployment
     scope, rollback to prior verified version, and regression gating on edits.
   - Verified skill experience records carrying applicability conditions and
     verification provenance.
4. **Single-Cup Coffee Simulation Closed Loop (`craft-h9k.5`)**
   - End-to-end execution on the live `ticos-edge` simulator covering normal
     coffee service plus supported faults (unknown brew receipt reconciliation,
     cup placement offset recentering).
   - Full accept/reject verification of at least one improvement candidate with
     raw archived logs, declared budgets/stop rules, and explicit reporting of
     failures and oven/bagel capability gaps.

### Phase 2+: Future extensions (explicitly out of Phase 1)

- **Bagel heating and combined coffee+bagel orders**: Blocked until the host
  exposes executable oven door, load, heat, query, unload contracts and an
  independent completion oracle (see [`host-capability-audit.md`](host-capability-audit.md)).
- **Physical robot deployment and food-quality certification**: Simulation
  acceptance is never promoted to physical-robot or food-safety approval.
- **Motion smoothness and low-level trajectory optimization**: Depends on the
  host robot controller; upper-layer CaP policies do not claim to solve
  controller smoothness.
- **Multi-order concurrent scheduling and multi-robot fleet orchestration**.
- **Large-scale multi-GPU parallel search (EvoSearch-style) and automated
  video-frame rendering**.

## Attribution and licensing

CRAFT remains Apache-2.0 licensed (`LICENSE`). Upstream research citations are
maintained in `references/catalog.json` and `references/references.bib`.
Attribution for adapted `ticos-edge` transport code and referenced ASPIRE/ACE
concepts is recorded in `NOTICE` and `docs/provenance.md`.
