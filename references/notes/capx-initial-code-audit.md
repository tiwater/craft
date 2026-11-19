# CaP-X initial source audit — 2026-11-19

Source: https://github.com/capgym/cap-x
Pinned commit: `53e9966d7a8e2fa7494676772bccc35280f5c0ed`.
Depth: README, root license/dependencies and selected executor source only;
not a full paper review or reproduction. Checkout excludes recursive submodules.

The root LICENSE is MIT; third-party components still need separate review.
The public implementation exists despite ambiguous links on the project page.
`capx/envs/tasks/base.py` exposes `CodeExecEnvConfig` and `SimpleExecutor`:
the executor injects the low-level environment, APIs and inputs into generated
Python and returns a result or error. It explicitly permits full imports.
This interface is useful for benchmark integration, but adopting it directly
would not enforce edge's existing evidence/actuation boundaries. That is an
integration constraint, not evidence of a novel CRAFT research contribution.

The dependency manifest includes simulator/perception and training packages;
do not install it into the running edge environment. Any reproduction must use
an isolated environment and preserve existing perception owners.

Next: read the evaluation protocol, identify permitted observation/action tiers,
and determine the smallest upstream execution path before installing optional
simulators. No benchmark score or robot execution was measured in this audit.
