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
