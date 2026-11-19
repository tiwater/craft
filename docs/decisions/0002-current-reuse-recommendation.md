# Current recommendation: upstream first, no second general runtime

2026-11-19. Decision for the initial necessity investigation. This is not a
claim of complete paper reproduction or deployment readiness. Reopen the
research decision only when new comparative evidence warrants it.

## Decision

Do not expand or promote CRAFT's custom general execution runtime. Keep the
repository as a literature, experiment and potential cafe-adapter workspace.
Prioritize ASPIRE as the policy-development integration candidate while retaining
edge as execution and perception authority. A production adoption remains gated
on the actual development campaign and task contracts, not our wrapper tests.

| Candidate | Appropriate use now | Evidence and remaining constraint |
| --- | --- | --- |
| ASPIRE | First integration candidate for policy development and retained repair knowledge | Pinned Apache-2.0-owned source; 96 core and 108 pipeline tests pass; cafe campaign adapter and learning evaluation still missing |
| CaP-X | Benchmark protocol/reference implementation; possible environment contribution | Pinned MIT-owned source; useful abstraction/feedback distinctions; its default executor and perception stack cannot silently replace edge owners |
| ACE-CaP-0 | Demonstration and experiment-acquisition methodology | Main text and appendix reviewed; public site repository does not establish deployable controller code or reuse license |
| RoboRepair | Recovery baseline design and trace-based metrics | Recovery without repeated completed steps is already covered; confirmed author code not located |
| NeSyRo | Symbolic applicability/probing baseline design | Requires domain definitions and a separate probability-scoring model with calibration; confirmed author code not located; generation can still use Astra |

Repository license findings cover the inspected project-owned source, not all
third-party dependencies, model weights or datasets. Paper availability is not
source-code licensing evidence.

## What the experiments establish

Unmodified ASPIRE tools preserve selected live edge observation values. The
existing coffee policy also preserves ten scripted branch outcomes after an
explicit signature adaptation. Whole-policy host delegation completes actual
simulator tasks while keeping edge's SDK and subprocess boundary.

A newly generated Astra-medium policy required one model call under a prompt
that already specifies recovery behavior. Its normal trials passed; one fault
trial failed at SDK recentering with `empty_grasp`. Two subsequent fixed-policy
fault repetitions passed, as did two interleaved existing-policy repetitions.
The failure remains unexplained. This is neither skill discovery nor a robust
failure-rate comparison. We have no measured framework superiority result.

Warm-food scene geometry exists but the inspected runtime lacks executable oven
and heated-item contracts. No candidate can be ranked on bagel/combined-order
tasks using this setup yet. This is an infrastructure gap shared by candidates.

## What would justify more independent research

A new research claim needs a reproducible task-level failure of strong existing
methods under matched evidence, action abstraction, model and development budget,
followed by a mechanism whose ablation explains the improvement. Domain naming,
another journal, generic preconditions, tracing, frozen programs and retry loops
are insufficient on their own.

A possible domain evaluation contribution is still open: measuring development
cost, continuous execution, uncertain appliance outcomes and combined-order
completion. It requires executable host capabilities and independent scoring
before it becomes a benchmark contribution. If an upstream adapter solves the
problem, adopt it or propose it upstream rather than manufacturing a new method.

## Traceability and next gate

- `0001-reuse-before-research.md`: investigation scope and changing evidence.
- `host-capability-audit.md`: device/interface availability.
- `references/notes/`: versioned paper and source audits.
- `experiments/reuse-audit/`: scripts, prompt, policy, failed and successful trials.

The five closest methods now have main-text and appendix-text review records;
image-only figures are not independently audited. Source inspection and runnable
coverage differ by candidate and are recorded separately.

The initial necessity investigation ends with **thin adaptation, not independent
framework development**. Minimal compatibility and simulator validation support
this direction without establishing production adoption. A complete upstream
learning campaign is the next deployment experiment, not evidence already held.

## Bounded adoption experiment

1. Expose only existing edge observation and execution contracts to ASPIRE's
   current development workflow. Producer/owner: `EdgeCapRuntime._run_job` and
   `EdgeZeroGtCoffeeSDK`; execution remains in the existing subprocess sandbox.
   No new perception, geometry, robot controller or direct device access.
2. Predeclare development conditions, held-out conditions and equal model/effort,
   action abstraction and total development budget. Record human interventions,
   resets, all model calls/tokens when available, wall time and admitted skills.
   Missing usage data remains missing; do not infer subscription dollar costs.
3. Compare direct generation, upstream development without retained skills and
   upstream development with retained skills. Freeze programs before evaluation.
   Score completion independently and preserve crashes, unknown outcomes,
   duplicate non-idempotent actions and unsuccessful trials.
4. Add oven tasks only after the host supplies executable door, load, heat,
   query and unload contracts plus an independent completion oracle. Existing
   visual geometry is insufficient. This prerequisite is shared by all methods.
5. Adopt upstream if it meets task requirements. Keep local code limited to the
   host/environment adapter; propose a generic adapter upstream only after it
   works and external communication is authorized. If it fails, first attribute
   failure to capability, perception/control, integration or policy development.
   Consider independent research only for reproducible method-level gaps that
   survive matched-budget baselines and an explanatory ablation.

No new runtime or upstream message is part of this decision. The alpha code is
retained for reproducibility; its presence is not an instruction to extend it.
