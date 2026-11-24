# Research plan — not reported findings

TRAX: Transactional Robot Action Execution for Verifiable, Crash-Recoverable Code Policies.

Question: can applicability-aware reusable programs and explicit execution
evidence improve recovery under changed task conditions and uncertain physical
outcomes, without regenerating a policy on every order?

## Tasks and perturbations

1. Coffee: select/place cup, brew, verify, deliver.
2. Bagel: load oven, heat, verify completion, unload, deliver.
3. Combined order: coordinate shared arm and device waiting.

Held-out disturbances: object/layout changes, failed placement, delayed or lost
receipts, interruption after irreversible actions, changed equipment contract.
Oven hardware interlocks are adapter requirements, not assumed library features.

## Comparisons

Direct CaP; feedback-driven repair; skill retrieval; frozen policies with no
explicit reconciliation; proposed applicability/evidence/recovery mechanism.
Prioritize CaP-X, ASPIRE, ACE-CaP-0, RoboRepair and Reliable CaP. Compare only with
matched model, input evidence, action abstraction, debug budget and task splits.
Do not relabel a locally reimplemented baseline as an official reproduction.

Freeze policies, memories and thresholds before held-out evaluation. Record
all seeds, failures, budgets and retries. Separate development cost, runtime
cost and amortized cost per completed order. Report first-attempt success,
recovered success, false completion, duplicate side effects, interventions,
wall time and model/physical trial cost with uncertainty intervals.

Ablate applicability checks, receipt reconciliation and local recovery. Domain
focus alone does not demonstrate novelty. RoboRepair already studies avoiding
repeated actions; Reliable CaP studies symbolic/interactive validation; ASPIRE
and ACE-CaP-0 already study reusable programs and experience-driven improvement.
The first milestone is baseline reproduction and operational task definitions,
not a claimed new algorithm. See `references/novelty-matrix.md`.
