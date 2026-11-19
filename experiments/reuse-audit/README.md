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
