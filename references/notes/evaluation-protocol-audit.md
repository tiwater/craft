# Evaluation protocol audit — 2026-11-19

Reading depth: selected methods, evaluation and limitation sections, not full
appendix review. These are reported protocols, not reproduced results. Versioned
reading URLs below may differ from current abstract-page citation metadata.

## ASPIRE

Source: https://arxiv.org/html/2607.00272v1, sections 2–3 and 5.

LIBERO-Pro uses debug seeds 51–65 and evaluation seeds 1–50; Robosuite uses
101–125 and 1–100 respectively. One developed program per task is evaluated on
held-out seeds. BEHAVIOR instead generates incremental blocks. The real-robot
study adapts simulation-derived guidance through execution feedback; it is not
direct deployment of unchanged simulation policies. Its limitations explicitly
include real-world success detection, resets and monitoring. Therefore our
registry probe validates neither skill discovery nor the published method.

## CaP-X

Source: https://arxiv.org/html/2603.22435v1, sections 3–4.

The main protocol uses 100 trials per task/tier, with no reset within a trial.
Pass@1 can include multiple code turns. S1 has privileged state; S2 uses
perception and high-level actions; lower tiers alter action abstraction and
examples. Agent0 also uses learned utility guidance and candidate ensembles.
An equal-model comparison must account for all candidate and synthesis calls.

## Consequences for our local experiment (our proposed protocol)

Keep two separate questions:

1. Can an existing upstream executor run the existing host contract with
   identical evidence and action semantics? Use the same frozen program and
   compare call traces, outcomes and intervention needs. This is integration
   evidence, not a comparison of learning algorithms.
2. Does upstream development produce reusable policies under a fixed budget?
   Separate development scenes from held-out evaluation; freeze policy and skill
   hashes before evaluation. Record model identifier, effort, prompt, all model
   calls, tokens, wall time, resets and human interventions. During evaluation,
   distinguish frozen-policy feedback control from LLM-driven code repair.

Existing edge benchmark owner: `tools/benchmark_cap_backends.py` uses
`EdgeCapRuntime._build_zero_gt_observation_summary`, compiler generation and
the existing `/cap/execute` SDK route. Its fixture mode is software-only.
Its five cases mix task success, refusal and injected repair, so a single
aggregate success number would obscure what was tested. Preserve per-case
outcomes. Simulator oracle is scoring-only, never model input.

Before any new actuation comparison, document which host calls map to upstream
tools, preserve existing interlocks and unknown-outcome handling, and measure
the unchanged baseline. Coffee interfaces exist; oven/heat-cycle capability
must be established separately. Missing device APIs do not establish a new
algorithmic contribution.
