# ASPIRE initial code audit

Source: https://github.com/NVlabs/ASPIRE
Commit: fa00f0ef2bc1d91d594b9aa502c22133c9f538f7. Date: 2026-11-19.
Reading depth: README, pyproject.toml, docs/environments.md,
docs/api-reference.md, EnvironmentAdapter protocol and selected test setup.
Not a completed implementation audit, full-paper reading or reproduction.

Observed:
- Package cap-harness supports Python >=3.10,<3.13; base and provider/simulator
  extras are separate. Core API tests use fake adapters and need no live robot.
- EnvironmentAdapter defines task/observation/state, one-period step, trajectory
  execution, gripper operations and close. This is a concrete reuse boundary.
- Public API includes atomic and composed actions: comparisons must control
  abstraction level. Existing providers include segmentation, grasp, IK/planning.
- Main README describes Apache-2.0 for project-owned code; third-party components
  retain separate terms. Complete dependency/license audit is not done.
- Codex and Claude workflows are already supported. Backend switching is not
  a reason for an independent CRAFT research framework.

Inference: ASPIRE is a strong reuse candidate. Whether existing EvidenceScene
state can be consumed without an alternative estimator needs an actual interface
probe. No absence claim about durable recovery follows from keyword searches.
Next: upstream core tests, runtime/provider trace, full-paper method/evaluation,
and measured integration cost. No comparative performance claim yet.

## Executed evidence

Upstream base/dev installed in an isolated Python 3.10 virtual environment.
Unmodified upstream tests/test_contracts.py, test_runtime.py, test_api.py and
test_artifacts.py: **96 passed**. Initial collection required imageio/pillow;
video artifact tests additionally required imageio[ffmpeg]. These are environment
setup observations, not upstream algorithm failures. No GPU/model/robot was used.

A custom shared-tool allowlist successfully registered observation and appliance
query functions in upstream ToolRegistry and executed them with ProgramExecutor,
without editing ASPIRE or importing CRAFT's runtime. The returned fixture state
retained its values; ASPIRE freezes mappings/lists to mappingproxy/tuple. The
first probe compared Python containers directly and failed; comparing normalized
JSON values corrected the probe, not the upstream implementation.

Reproduction scripts and outputs: experiments/reuse-audit/. This establishes a
usable public-tool extension seam. It does not establish full robot integration,
provider compatibility, durable recovery or superiority on non-idempotent multi-appliance tasks.
