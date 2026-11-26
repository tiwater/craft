# ADR 0004: Spatial State Contract v1 (TRAX ↔ EvidenceScene v3.0 Alignment)

- **Status:** Accepted (Phase 0/1 alignment; backward-compatible with EvidenceScene v2.x)
- **Date:** 2026-11-26
- **Related:** `AGENTS.md`, `0003-product-architecture-and-reuse-boundary.md`, `EvidenceScene v3.0 architecture-baseline-1.md`

## 1. Context & Problem

EvidenceScene is evolving from a perception/reconstruction engine (v2.8) into an **Evidence-Aware Spatial State Runtime** (v3.0) built around **One Spatial State, Two Interfaces**:

- **EvidenceScene** owns *what is known about the physical world* (entity identity, SE(3) poses, measured/inferred geometry, directional observability, evidence provenance, immutable versioned snapshots, and lineage-aware revisions).
- **Skill Runtime / Host** owns *what the robot can do* (actuation, kinematics, physical safety interlocks, non-idempotent host ledger).
- **TRAX** owns *what the robot should do next* (reasoning, policy synthesis, AST sandbox, SQLite intent journaling, skill preconditions, active-perception scheduling, and 4-layer failure attribution).

Auditing TRAX against EvidenceScene v3.0 Schema v1 exposed three concrete gaps in TRAX today:

1. **Dual-source human/machine drift in simulation:** `3D Twin` renders `RevisableSurfelMap` surfels while `get_scene_objects()` reads raw simulator truth rather than sharing one immutable `SpatialSnapshot` cursor `(scene_id, epoch, revision)`.
2. **Unsafe clock/freshness semantics in `edge_adapter.py`:** Computing `time.time_ns() - snapshot["timestamp_ns"]` implicitly assumes a single shared wall clock, conflates observation time with revision time, and lacks explicit separation between `unknown`, `unavailable`, `stale`, and `resnapshot_required`.
3. **Runtime dependency boundary & missing `perception` attribution:** EvidenceScene v3 targets Python 3.12 + NumPy 2.5+, whereas TRAX core is strictly pure Python stdlib (Python 3.10+). Meanwhile `src/trax/attribution.py` dropped the `perception` attribution layer for MCP executions.

## 2. Decision

### 2.1 Process & Transport Boundary

TRAX core (`src/trax/`) **must never** import EvidenceScene internal Python classes directly. Integration across repos uses a **transport-independent JSON / MCP contract** (HTTP `/edge-scene` or MCP resources/tools such as `trax://scene/snapshot` and `assess_spatial_requirements`), preserving:
- Pure Python stdlib in TRAX core (`make check-stdlib` & `make check-arch`).
- Independent Python/NumPy/C11/CUDA runtime environments between EvidenceScene and TRAX.
- Single commit boundary owned exclusively by EvidenceScene's Spatial State Manager.

### 2.2 Canonical Consumer Schema (v1 Subset)

Implemented in pure stdlib at `src/trax/spatial_contract.py` with full backward compatibility for v2 snapshots:

| Concept | Logical Fields | Consumer Invariants |
| --- | --- | --- |
| **SceneCursor** | `(scene_id, epoch, revision)` | `epoch >= 0`, `revision >= 0`. Reset increments `epoch` and restarts `revision = 0`. Different `scene_id`, stale `epoch`, future `revision`, or evicted history gap **must** trigger `resnapshot_required`. |
| **ClockTimestamp** | `(clock_id, ns)` | Explicit clock domain. Producer clock restart mints a new `clock_id`. |
| **Freshness** | `fresh \| stale \| unknown` | Evaluated strictly from `observation_time` (never `revision_time` or `published_at`). Cross-clock subtraction or `ref_ns < obs_ns` **must** return `unknown`, never `0 ms` fresh. |
| **Epistemic Status** | `unknown \| unavailable \| stale \| insufficient_observability` | `unknown` = no supporting observation; `unavailable` = known source failed to supply; `stale` = previously accepted value past max age. **Unknown space is never treated as free space.** |
| **SpatialChange** | `(cursor, affected_entities, cause)` | `cause ∈ {observation, physical_motion, historical_correction, removal, reset}`. A `historical_correction` preserves original `observation_time`. |
| **SpatialRequirementAssessment** | `(status, entity_id, cursor, freshness, unobservable_directions, missing_information)` | `status ∈ {satisfied, unsatisfied, unknown}`. EvidenceScene assesses geometric requirements; TRAX decides whether to act, replan, or schedule active observation (e.g., moving a wrist camera). |

### 2.3 Perception–Action Consistency & 4-Layer Attribution

1. **Journal Cursor Pinning:** When `GovernedMCPProxy` claims a non-idempotent action in SQLite `Journal`, it records the latest observed `SceneCursor` (if present), linking actuation intent to the spatial revision it depended upon.
2. **Restored `perception` Layer in `src/trax/attribution.py`:**
   - Pre-actuation epistemic blocks (`stale_evidence`, `unknown_freshness`, `unknown_evidence`, `unavailable_evidence`, `insufficient_observability`, `resnapshot_required`, `missing_object`, `ambiguous_object`) attribute to `layer="perception"`, `recommended_owner="perception_owner"`.
   - If a policy dispatches a **non-idempotent** actuation *after* receiving an unresolved perception block, attribution escalates to `layer="policy"`, `code="acted_on_unverified_perception"`.
   - If post-run revision inspection reveals `cause == "historical_correction"` invalidated the prior spatial assumption, attribution classifies `layer="perception"`, `code="historical_correction_invalidated"` instead of blaming robot control.

## 3. Phased Cross-Repo Rollout

- **Phase 0–1 (Implemented now in TRAX):** `spatial_contract.py`, clock-domain/three-state hardening in `edge_adapter.py`, cursor pinning in `sandbox.py`, and restored `perception` attribution in `attribution.py`.
- **Phase 2 (When EvidenceScene P0 `SceneStore` lands):** Bind both TRAX `3D Twin` and Agent MCP queries to the same immutable `SpatialSnapshot` cursor over HTTP/MCP.
- **Phase 3–5 (P1/P2 Joint Closed Loop):** Attach declarative spatial preconditions to promoted TRAX Skills and run active-perception wrist-camera viewpoints + revision-aware replanning on the LeRobot SO-101 2-camera (`front_top + wrist`) rig.
