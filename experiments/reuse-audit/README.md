# ASPIRE minimal reuse audit

Upstream: https://github.com/NVlabs/ASPIRE
Commit: fa00f0ef2bc1d91d594b9aa502c22133c9f538f7
Python: 3.10. Date: 2026-11-19.

Install the pinned checkout in a separate venv with `pip install -e '.[dev]'`
and `pip install 'imageio[ffmpeg]' pillow`. No third-party submodules or model
weights are needed for this selected test set. From the upstream checkout:

```sh
python -m pytest tests/test_contracts.py tests/test_runtime.py tests/test_api.py tests/test_artifacts.py -q
```

Then run this directory's `probe_aspire_registry.py` using that interpreter.
The probe registers read-only fixture tools in the unmodified upstream registry.
It imports no CRAFT runtime and does not execute a robot or generate a policy.
Results prove only core executability and the host tool extension seam.

Failure history: missing imageio during collection, missing ffmpeg for four
artifact tests, and a probe equality check expecting mutable list/dict output
instead of upstream's frozen tuple/mapping output. Dependency installation and
JSON-value normalization resolved these without upstream source changes.

## Existing edge observation probe

`probe_aspire_edge_readonly.py --endpoint <existing-edge-scene-URL> --output <file>`
uses the same upstream interpreter. Producer/state owner: the existing edge
scene route consumed by `EdgeZeroGtCoffeeSDK.observe_scene` in
`ticos-edge/tools/edge_cap_runtime.py`; its snapshot remains authoritative.
The adapter selects existing fields and compares their JSON values after
upstream execution. It adds no estimator, actuation, or CRAFT runtime.

`aspire-edge-readonly-result.json` records a successful live read (six objects,
183.4 ms snapshot age at measurement, identical input/output hashes). This is
transport/executor evidence, not validation of object accuracy, policy quality,
robot execution, or ASPIRE's learning method. The endpoint and scene payload
are deliberately not archived.

## Existing policy compatibility probe

Run `probe_aspire_policy_compatibility.py --host-source <edge_cap_runtime.py>
--output <result.json>` with the pinned upstream interpreter. The script reads
the existing `EDGE_CLOSED_LOOP_COFFEE_POLICY_CODE` constant without importing
edge. It compares trusted direct Python execution with the unmodified ASPIRE
executor on identical scripted host responses. The explicit AST adapter maps
`sdk.method(...)` to registered tools and removes the unused SDK parameter;
branch logic is unchanged. Both policy hashes are recorded.

Ten cases have equal returned JSON values and host call traces: normal,
recentering, resolved/unresolved unknown brew receipts, observation-not-ready,
missing/ambiguous cup, hazard escalation, rejected placement and L2 rejection.
Unknown brew outcomes query once and never issue a second brew command in
these cases. The observation-not-ready case does not test timestamp detection.

These are scripted contract fixtures, not the real SDK, existing edge sandbox,
physics, persistent recovery, or ASPIRE's learning pipeline. No model is used.
This supports signature adaptation feasibility only. Production integration
must retain edge's subprocess boundary and SDK guards. ASPIRE's in-process
executor alone does not replace those controls; custom action names also need
explicit trace instrumentation if before/after motion keyframes are required.

## Actual simulator host delegation

`probe_aspire_host_simulation.py` requires explicit `--edge-base`, `--sim-base`,
`--host-source` and `--output`. It resets the simulator between runs. It checks
the existing runtime is not running, then requires a fresh single-cup scene.
No model is used: the same existing policy source is passed to `/cap/execute`
directly or through one registered ASPIRE `execute_host_policy` tool. The latter
delegates the entire policy to edge; it does NOT move primitive execution into
ASPIRE or reproduce its learning algorithm. Edge retains `_run_job`, the SDK,
process isolation and the command ledger. No production source was modified.

The paired conditions are default placement and injected 38/-12 mm placement
offset plus a simulated first-brew acknowledgment timeout. Scoring uses the
existing simulator oracle only after execution: completed status, filled cup
and delivery error within 15 mm. Additional fault checks use recorded method
order and brew execution count; they do not prove persistence across a crash.

One pair per condition is a smoke test, not a statistical benchmark. Both runs
use the same reset preset and policy, but fresh RGB-D inputs are not identical.
Direct mode always precedes delegation, so order effects are not controlled.
The result cannot establish a success-rate advantage, ASPIRE skill discovery,
bagel/oven capability, or real-robot reliability.
