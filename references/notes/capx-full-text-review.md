# CaP-X text review — 2026-11-19

Source: https://arxiv.org/html/2603.22435v1
Coverage: main sections and appendices A–I text, tables, API listings, generated
code and prompts. Image-only plots were not independently digitized. Not reproduced.
Source implementation audit: `capx-initial-code-audit.md`.

Compare observation/action tiers separately. Multi-turn pass@1 permits continued
execution within a trial, not a fresh environment reset. Agent0 mines successful
S3 utilities in a single synthesis pass; possible future iteration is not an
implemented continual-learning result. Nine candidate queries plus synthesis,
visual differencing and subsequent turns all belong in the inference budget.
Fewer turns alone does not establish lower cost. RL results additionally require
training-budget accounting.

Appendix G's stronger verification prompt did not improve reported success.
Appendix I identifies perception, grasp and collision-related failures. These
support separate operational diagnostics, not confidence in generated assertions.
Real-world case studies include optional human feedback and are not a uniform
autonomous cafe benchmark.

Its primitives include perception and geometry; importing these into edge would
violate our owner boundary. Reuse evaluation structure or add an explicit host
adapter. Successful source examples are not safety-certified policies, and
paper metrics do not establish performance on coffee or oven tasks.
