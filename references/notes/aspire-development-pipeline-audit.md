# ASPIRE development pipeline audit — 2026-11-19

Pinned source: NVlabs/ASPIRE `fa00f0ef2bc1d91d594b9aa502c22133c9f538f7`.
This is inspection of upstream workflow files, not execution of the agent skill
or a reproduction of its learning campaign.

The Codex skill files point to canonical `.claude/skills/iterative-debugging/`
and `evosearch/` workflows. The debugging workflow defines development,
unseen-check and held-out partitions, failure-guided iteration, separate skill
verification and promotion. The verifier receives proposed knowledge without
the proposer's code. BEHAVIOR uses a distinct incremental-block protocol.
These are already substantial research infrastructure; recreating them locally
would add maintenance without establishing a contribution.

Read source: canonical SKILL.md, Codex pointer files,
`scripts/run_validation.py`, `scripts/campaign.py`, CLI choices, and selected
pipeline tests. Three upstream test files ran unmodified in the isolated venv:

```
python -m pytest tests/test_iterative_debugging_pipeline.py tests/test_evosearch_pipeline.py tests/test_pipeline_commands.py -q
108 passed in 0.30s
```

Tests cover software behavior and workflow contracts, not robot performance or
LLM skill acquisition. The earlier 96 core tests are a separate selected set.

## Integration findings

`src/cap_harness/cli.py` constrains the run benchmark argument to existing
families, including LIBERO-Pro, Robosuite, BEHAVIOR and YAM real. Our cafe host
is not registered. A successful `ToolRegistry` probe alone does not enable the
campaign runner. The smallest meaningful extension needs a cafe task/reset
adapter, existing observation and action bindings, trace artifact export and
an outcome evaluator. Keep device and perception state in existing owners.

`run_validation.py` explicitly counts `task_success` even if `program_ok` is
false. Preserve that metric when reproducing upstream; additionally report
operational completion, errors, unresolved effects and interventions for our
deployment evaluation. Do not silently change the upstream denominator.

Decision implication: prefer an upstream environment extension or a separately
versioned thin experiment adapter. Do not call a host-delegation smoke test an
ASPIRE development campaign. Do not create a parallel general runtime while
the missing cafe integration is the only demonstrated obstacle.

Paper/code version caveat: arXiv v1 Appendix E describes baseline-run repair,
while this pinned workflow explicitly starts without external baseline code
and adds independent skill proposal verification. See `aspire-full-text-review.md`.
An experiment using the current workflow must not be called an exact replication
of the published campaign without resolving this difference.
