# Related-work overlap and candidate contribution

These are research hypotheses and reading questions, not claims of novelty.

| Candidate claim | Existing coverage | Required distinction/evidence |
| --- | --- | --- |
| LLM writes robot code | Code as Policies, ProgPrompt, RoboCodeX | Not novel; use as baseline |
| Generate once, reuse with observations | VoxPoser, ACE-CaP-0 | Not novel; measure applicability under held-out condition changes |
| Trace failures and repair code | RoboRepair, ASPIRE | Not novel; investigate physical partial completion and ambiguous receipts |
| Store reusable skills | Voyager, DROC, ASPIRE, RoboSkill | Not novel; define applicability and invalidate stale skills |
| Symbolic/observational checks | RoboEval, Reliable CaP | Not novel; compare evidence-conditioned task progress under uncertainty |
| Automatic physical trial loop | ENPIRE, RoboClaw | Not novel; quantify reset/verification assumptions and human work |
| Low-cost model execution | CaPtain, CaP-RL | Compare training cost, runtime cost and frozen-program alternatives fairly |
| Commercial workstation domain | A useful scoped application | Domain branding alone is not a methodological contribution |

## Candidate focal question

Can explicit skill applicability and a receipt/evidence-aware recovery protocol
improve frozen-program reuse when physical effects have occurred but their
acknowledgements are delayed, missing or contradictory?

Closest prior work requiring full reading before any novelty assertion:

- `schlesinger2024roborepair`: current-state recovery and avoiding repeated actions.
  Check handling of irreversible effects, unknown outcomes and durable identities.
- `ace2026`: experience acquisition and frozen-program deployment. Check skill
  validity conditions and runtime recovery boundaries.
- `lu2026aspire`: multimodal traces, repair and skill memory. Check persistence,
  success verification, invalidation and repair evaluation splits.
- `ahn2025reliablecap`: symbolic and active validation. Check uncertainty models,
  partial execution and cost of evidence acquisition.
- `hu2023codebotler`: temporal trace correctness. Check whether appliance outcomes
  and delayed receipts can be represented and evaluated with its properties.

Do not assume these papers lack a mechanism because their abstracts omit it.
The v0.1.0 journal is an engineering baseline, not proof that the focal question
has been solved. No comparisons or physical trials are reported in this release.
