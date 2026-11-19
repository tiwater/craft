# Generated coffee baseline

One model generation via the existing edge compiler and Codex CLI, requested
model `gpt-6-astra`, requested effort `medium`. `generation.json` records the
transport metadata, hashes and generation time. The transport does not archive
resolved serving-version attestation or token usage; do not infer either.

`prompt.txt` is the exact compiler prompt, built from the existing observation
summary and host contract after resetting the simulator. It contains explicit
instructions for recentering and uncertain-receipt reconciliation. Consequently
this is a strongly scaffolded code-generation baseline, not discovery of those
behaviors. `policy.py` is the untouched generated output after code-fence
extraction. It passed the existing AST gate before any execution.

`evaluation.json` is written incrementally by `probe_aspire_host_simulation.py`
using `--policy-file`. It executes the same fixed bytes directly through edge
and through ASPIRE's host-delegation tool, with normal and offset/timeout
conditions. There is no runtime model call, repair or learning. These are smoke
tests with fresh observations, not independent policy-development repetitions
or a new held-out benchmark. One successful generation cannot establish
generation reliability or framework superiority.

Reproduction: run `../generate_coffee_baseline.py` with the existing edge service
Python interpreter and explicit `--edge-root`, `--edge-base`, `--sim-base`,
`--output-dir` arguments. Then use the isolated upstream interpreter for
`../probe_aspire_host_simulation.py`, supplying `--host-source`, endpoints,
`--policy-file` and `--output`. Both commands reset the simulator; use a new
output directory to retain this historical result. Original generation used an
equivalent temporary script with local paths; the checked-in script parameterizes
those paths without changing the generation call.

## Observed outcome

The two normal-condition runs completed. The first offset/timeout-condition run
failed during recentering: the existing SDK returned `empty_grasp`, with a
rejected receipt. Policy execution stopped before brewing (brew execution count
zero). Per the prewritten stop rule, the paired ASPIRE fault run was not executed.
Thus results are **2 passed, 1 failed, 1 unrun**, not a complete paired comparison.
`failure-diagnostics.json` preserves the existing runtime result and relevant
steps. It does not establish whether the root cause was perception, actuation,
or simulator variation. No retry, policy edit or model repair was applied.

This failure contradicts any broad reliability inference from the earlier
four successful hand-authored-policy runs. Next compare reproducibility with
the unchanged host policy and frozen generated policy before choosing a repair
method. Keep failed attempts in the denominator and preserve source diagnostics.
