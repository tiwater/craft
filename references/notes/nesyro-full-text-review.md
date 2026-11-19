# NeSyRo text review — 2026-11-19

Source: https://arxiv.org/html/2510.21302v1
Coverage: sections 1–5, appendices A–D, textual tables, algorithms and prompts.
No independent figure-image audit; no reproduction or confirmed author code.

Section 4 separates generation/feedback (GPT-4o-mini) from confidence scoring
(Llama-3.2-3B). Thus our generator's missing probabilities are not a fundamental
obstacle: a separate scorer is required. Z3 and Fast Downward provide different
verification/validation roles. Appendix C restricts generation to supplied skills.

Verification checks code against a generated specification; our inference is
that this cannot independently certify the specification or physical evidence.
The authors acknowledge binary logic confidence and predefined domain knowledge
as limitations, proposing probabilistic/temporal reasoning as future work.

Real-world auxiliary-manipulation results remain limited by switch pressing
even with complete observations. This supports separating execution failures
from reasoning failures in our own evaluation, without proving a common cause.

Reuse prerequisites: explicit cafe domain/skill definitions, observed predicates,
scoring model and calibration, probe accounting, and verification implementation.
No guessed repository or self-reported confidence should substitute for these.
