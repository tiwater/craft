# Phase 1 Single-Cup Coffee Simulation Closed Loop — 2026-11-20

Reproducible operator record for the end-to-end CRAFT Phase 1 single-cup coffee
simulation closed loop (`craft-h9k.5`), connecting:

1. Declarative `SceneConfig` (`examples/single_cup_coffee_config.json`)
2. Automatic host capability discovery via `EdgeHostAdapter` (`GET /cap/state`)
   and explicit oven/bagel capability gap check
3. Read-only `EvidenceScene` observation pass-through (`GET /edge-scene`)
4. Unified teaching specification (`TaskSpecification`, text + non-authoritative
   video reference + structured steps)
5. Persistent `DevelopmentLoop` with real model generation (`codex_cli`,
   `gpt-6-astra`, `medium` reasoning effort) and live `ticos-edge` subprocess
   sandbox execution (`POST /cap/execute`)
6. Independent held-out acceptance (`evaluate_acceptance`, locked criteria hash,
   post-run simulator oracle scoring only)
7. Fixed-version release (`ReleaseRegistry`, `v1`), verified `SkillExperience`
   storage, and feedback-driven improvement candidate (`v2`) accept/reject gating

## Reproducible Operator Command

With an existing `ticos-edge` CaP runtime and Isaac cafe simulator already
running (do not restart or overwrite live services), run from the repository
root using the project virtual environment:

```bash
# 1. Read-only capability discovery, EvidenceScene check, and oven gap report
.venv/bin/craft discover \
  --config examples/single_cup_coffee_config.json \
  --edge-base <EDGE_ROBOT_SCOPED_URL> \
  --check-oven-gap

# 2. Full Phase 1 single-cup coffee simulation closed loop
.venv/bin/craft run-coffee-loop \
  --config examples/single_cup_coffee_config.json \
  --workspace experiments/<new-run-dir> \
  --edge-base <EDGE_ROBOT_SCOPED_URL> \
  --sim-base <SIM_BASE_URL> \
  --provider codex_cli \
  --model gpt-6-astra \
  --effort medium \
  --allow-actuation \
  --allow-sim-reset
```

Use a fresh `--workspace` directory to preserve archived runs. All endpoints are
passed at runtime and are not archived into repository files.

## Pre-Declared Conditions, Budget & Stop Rules

Recorded in `protocol.json` before any model call or simulation episode:

- **Evidence source:** `simulator` (live `ticos-edge` + Isaac simulator; NOT
  physical robot and NOT food-quality certification).
- **Host SDK contract hash:** `15fcbdce0816a49f1534acbff8c247f5bf843a625aca0b299ea65ef644313e56`
  (`executionAuthority=ticos-edge-zero-gt`, `perceptionOwner=EvidenceScene-v2.0.0-4cam-RGBD`).
- **Development conditions:** `dev_normal`, `dev_fault_offset_and_timeout`
  (injected 38/-12 mm placement slip + simulated first-brew acknowledgment timeout).
- **Held-out acceptance conditions:** `heldout_normal`,
  `heldout_fault_offset_and_timeout` (fresh simulator resets, `used_in_debugging=False`).
- **Budget:** `max_model_calls=6`, `max_trials=10`, `max_wall_s=600.0`.
- **Stop rules:** Stop on missing/drifted/busy host capabilities, stop on budget
  exhaustion, block publication if held-out acceptance fails, reject improvement
  candidates that regress reliability or do not improve reliability/speed, and
  preserve every failure.

## Observed Runs (Including Failures)

All three chronological runs are preserved without cherry-picking:

| Run Directory | Outcome | What Was Demonstrated |
| --- | --- | --- |
| `../coffee-closed-loop-20261120-run1-budget-exhausted/` | `development_budget_exhausted` (6 model calls, 10 trials, `published_version=null`) | **Real contract bug caught & budget enforcement verified:** Generated `cand_001` completed both physical/sim conditions (`cup_filled=true`, 3.19 mm / 5.00 mm error, `fault_checks_passed=true`), but returned `{"status": "done"}` while `EdgeHostAdapter` initially only accepted `"completed"`. The loop respected the budget cap and retained all 6 candidates and 10 raw trials. Fixed in `src/craft/edge_adapter.py` + `src/craft/coffee_loop.py` with a regression test. |
| `../coffee-closed-loop-20261120-run2-acceptance-blocked/` | `acceptance_failed_not_published` (`completion_rate=0.5`, `published_version=null`) | **Independent acceptance gate blocked unsafe publish:** `cand_001` passed both dev conditions, then hit an intermittent `failed_recenter` on `heldout_fault_offset_and_timeout` (`delivery_error_mm=187.82 mm`, `cup_filled=false`, `brew_execution_count=0`), matching the intermittent recentering failure previously noted in `experiments/reuse-audit/generated-coffee-baseline/`. CRAFT honestly refused to publish `v1`. |
| `./` (`coffee-closed-loop-20261120/`) | `completed` (`published_version="v1"`, `improvement_result.decision="rejected"`) | **Full closed loop + improvement rejection verified:** `cand_001` passed both dev conditions (1 model call, 2 trials) and both held-out conditions (`heldout_normal`: 4.052 mm, 17.335 s; `heldout_fault_offset_and_timeout`: 4.101 mm, 25.997 s, recentered, unknown brew ack reconciled via `query_command_status`, `brew_execution_count=1`). Published frozen `v1` (`mean_wall_s=21.666 s`) and stored verified skill `skill-coffee-recenter-and-brew-reconcile`. Improvement candidate `v2` (generated with verified skill hints) also passed both held-out conditions (`3.895 mm / 17.774 s` and `4.118 mm / 25.807 s`, `mean_wall_s=21.791 s`), and was **honestly rejected** (`decision="rejected"`) because it did not improve reliability or speed over `v1`. Active release remained frozen at `v1`. |

## Oven / Bagel Capability Gap Report

Verified live against `/cap/state` on all three runs (`discovery.json`):

- `status`: `missing_capabilities`
- `missing_methods`: `open_oven_door`, `load_bagel`, `start_oven_heat`,
  `query_oven_status`, `unload_bagel`
- Bagel heating and combined coffee+bagel orders remain blocked until the host
  exposes executable oven capability contracts and an independent completion
  oracle.

## Limitations

- This is a **simulator** closed-loop verification (`n=2` held-out trials per
  candidate), not a statistical reliability benchmark, physical-robot approval,
  or food-quality certification.
- The intermittent `failed_recenter` observed in Run 2 (and earlier in
  `experiments/reuse-audit/`) remains an open host/control/perception variance
  factor; motion smoothness and grasp reliability depend on the host robot
  controller and perception stack, not upper-layer CaP alone.
- Presentation (`ticos`) consumes status and archived evidence read-only and was
  not modified to perform inference or policy execution.
