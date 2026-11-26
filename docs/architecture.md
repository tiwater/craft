# TRAX System Architecture: MCP-Native Transactional Governance & Git-Versioned Executable Knowledge Base

**TRAX** (*Transactional Robot Action Execution*) is a pure-Python-stdlib architecture for **embodiment-agnostic, crash-recoverable, and auditable robot code-policy agents** across simulators and physical robots.

Unlike demonstration-based imitation policies or unguarded coding-agent loops that mix exploration prose, actuation, and unverified skills, TRAX enforces **strict separation of ownership**, **MCP-based heterogeneous embodiment discovery**, **transactional SQLite governance for non-idempotent actions**, and a **Git-versioned Executable Knowledge Base**.

---

## 1. End-to-End Four-Layer System Architecture

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│  Layer 4 · Research Console & Inspector UI (src/trax/studio.py, ui_template)│
│  Full-viewport pure-English Light+ IDE (zero external web dependencies)     │
│  [Closed-Loop Lab] [MCP Inspector] [Knowledge & Git] [Embodiments/Matrix]   │
└────────────────────────────────────┬────────────────────────────────────────┘
                                     │
┌────────────────────────────────────▼────────────────────────────────────────┐
│  Layer 3 · Unified MCP-Native Agent Engine (src/trax/agent_engine.py)       │
│                                                                             │
│  01. MCP Discovery       initialize + tools/list + resources/list (SHA-256) │
│  02. Active Exploration  Multi-turn LLM writes & executes `run_probe(mcp)`  │
│  03. Policy Compilation  Compiles findings + KB into `run_policy(mcp, p)`   │
│  04. Governed Execution  AST Gate + GovernedMCPProxy + SQLite Intent Journal│
│  05. 4-Layer Attribution policy / control / perception / device -> Diff Fix │
└───────────────┬────────────────────────────────────────────┬────────────────┘
                │                                            │
┌───────────────▼──────────────────────────┐  ┌──────────────▼────────────────┐
│  Layer 2 · Four-Zone Governed Workspace  │  │ Layer 1 · Heterogeneous Robot │
│  (src/trax/knowledge_base.py, journal.py)│  │ Backends via MCP JSON-RPC 2.0 │
│                                          │  │ (src/trax/mcp_protocol.py)    │
│  1. workspace/knowledge/ (Git Repo)      │  │                               │
│     • FACTS.md      (Runtime-owned)      │  │ • LeRobot SO-101 6-DoF MuJoCo │
│     • FORBIDDEN.md  (4-Layer dedup)      │  │   (`move_ee`, `set_gripper`,  │
│     • probes/*.py   (Executed probes)    │  │    `move_joints`, `go_home`,  │
│     • policies/*.py (Task policies)      │  │    `activate_workstation`,    │
│     • skills/<emb>/ (SKILL.md + skill.py)│  │    `query_command_status`,    │
│       - session/* branches -> verified   │  │    `verify_task`, 4 cameras,  │
│         `--no-ff` merge to `main` + tag  │  │    pre-motion limit_refusal)  │
│                                          │  │ • Tabletop 6-DoF / Gazebo Sim │
│  2. workspace/runtime/   (SQLite WAL)    │  │ • LIBERO / Robosuite Suites   │
│     • agent_journal.sqlite (ACID)        │  │ • Physical Robot MCP Bridge   │
│     • mcp_journal.sqlite   (ACID)        │  │   (HTTP `/mcp` or Stdio)      │
│                                          │  │                               │
│  3. workspace/traces/    (JSONL Audit)   │  │ Backend owns: drivers, IK,    │
│     • llm_calls.jsonl (full prompt/resp) │  │ perception, hard safeguards,  │
│     • mcp_wire.jsonl  (raw JSON-RPC 2.0) │  │ and independent verification. │
│                                          │  │                               │
│  4. workspace/recordings/ (Episodes)     │  └───────────────────────────────┘
│     • <run_id>.json   (shadow replay)    │
└──────────────────────────────────────────┘
```

---

## 2. Core Architectural Principles

### 2.1 Dynamic MCP Embodiment Discovery (`src/trax/mcp_protocol.py`)
Different robot morphologies expose fundamentally different control and sensing APIs (single-arm IK/gripper, bimanual arms, dexterous 16-DoF hands, mobile bases, humanoids, external appliances). Instead of hardcoding a fixed domain verb list, TRAX communicates with robot backends via standard **MCP JSON-RPC 2.0** (`initialize`, `tools/list`, `tools/call`, `resources/list`, `resources/read`, over Streamable HTTP `/mcp` or Stdio):
- The backend declares its native tool JSON Schemas and MCP safety annotations (`readOnlyHint`, `idempotentHint`, `destructiveHint`, `traxRequiresReconciliation`).
- TRAX dynamically computes `mcp_tools_sha256`, binds the AST sandbox whitelist strictly to the discovered tools, and grounds every LLM prompt directly in the live schema.
- Backends enforce **pre-motion hard safeguards** (workspace reach and joint-limit checks) and return structured `limit_refusal` diagnostics *before* moving actuators.

### 2.2 Transactional Safety Governance (`src/trax/journal.py`, `GovernedMCPProxy`)
Online execution of compiled Python policies runs deterministically inside a pure-stdlib AST sandbox with **zero online LLM calls**. For any MCP tool annotated `idempotentHint: false` or `destructiveHint: true`:
1. **Intent-Before-Dispatch**: TRAX commits an ACID SQLite WAL intent row (`Journal.claim`) *before* dispatching `tools/call`.
2. **Query-First Reconciliation**: If a transport timeout or crash leaves a receipt `unknown`, blind retry is forbidden; the policy must reconcile state via `query_command_status` (or equivalent host status tool).
3. **Four-Layer Fault Attribution**: Failures are classified deterministically into `perception`, `policy` (`ast_sandbox_violation`, `duplicate_non_idempotent`, `unreconciled_unknown_ack`), `control` (`backend_limit_refusal`, `task_verification_failed`), or `device`.

### 2.3 Four-Zone Workspace & Git-Versioned Executable Knowledge Base (`src/trax/knowledge_base.py`)
Data in `workspace/` is partitioned strictly by mutability, durability, and audit role:
1. **`workspace/knowledge/` (Independent Git Repository — Executable Knowledge Base)**:
   - **`FACTS.md` (Runtime-Owned, LLM Read-Only)**: Objective physical measurements and MCP contract facts extracted deterministically from live telemetry and backend refusals—preventing LLM reflection hallucinations from poisoning memory.
   - **`FORBIDDEN.md` (4-Layer Attribution Deduplicated)**: Compact negative-constraint ledger keyed by `(embodiment, layer, code, summary)` so bounded context never forgets prior safety violations.
   - **`probes/` & `policies/`**: Every executed exploration probe and policy iteration is committed on an isolated `session/<embodiment>/<task>-<ts>` Git branch.
   - **`skills/<embodiment>/<skill_name>/SKILL.md + skill.py`**: Only policies passing AST preflight + independent `verify_task=PASS` + zero non-idempotent violations are promoted via `--no-ff` merge onto protected `main` with an immutable `verified/...` tag.
   - **Automatic Contract-Drift & Evidence-Tier Governance**: Skills whose `mcp_tools_sha256` diverges from the live backend are quarantined as `stale_contract`; `simulator` skills loaded onto a `physical_robot` backend are automatically downgraded to `advisory_sim_prior` until re-verified on real hardware.
2. **`workspace/runtime/` (SQLite WAL)**: ACID transactional command ledgers (`agent_journal.sqlite`, `mcp_journal.sqlite`), strictly excluded from Git.
3. **`workspace/traces/` (Append-Only JSONL)**: Complete untruncated LLM prompts/responses (`llm_calls.jsonl`) and raw MCP JSON-RPC 2.0 wire exchanges (`mcp_wire.jsonl`).
4. **`workspace/recordings/`**: Per-run verification snapshots and telemetry for zero-actuation `recorded_data` shadow regression.

---

## 3. Systematic Comparison vs. RAX (`arXiv:2610.22496`)

| Architectural Dimension | RAX (`arXiv:2610.22496`) | **TRAX (Ours)** |
| :--- | :--- | :--- |
| **Embodiment Interface** | Frontend–Backend split via MCP + 4-axis docs | **MCP JSON-RPC 2.0 + SHA-256 Contract Binding + 3-Owner Separation** (Perception / Policy Governance / Host Actuation) |
| **Non-Idempotent Actions & Crash Recovery** | Interactive online takeover on exception; no transactional intent ledger | **ACID SQLite WAL Intent-Before-Dispatch + Query-First Reconciliation** (auto-enforced from MCP `idempotentHint: false`) |
| **Perception Ownership** | Coding agent synthesizes 3D RGB-D perception pipelines | **Host-owned authoritative perception** (zero fabricated parallel estimators; deterministic 4-layer attribution) |
| **Knowledge & Memory Governance** | LLM-authored `PROGRESS.md` / `TIPS.md` + versioned skill repo | **Runtime-owned `FACTS.md` & `FORBIDDEN.md` (anti-hallucination) + Git-gated `session/* -> main` Executable Knowledge Base** |
| **Contract Drift & Sim-to-Real Safety** | Unattended execution evidence on session branch | **Automatic `stale_contract` invalidation on MCP schema drift + `simulator -> physical_robot` evidence-tier downgrade** |
| **Core Runtime Dependencies** | Containerized coding agent + backend stack | **100% pure Python stdlib core** (`make check-stdlib` enforced); optional simulator backends isolated |
