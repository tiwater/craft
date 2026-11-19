# Source provenance

The generation transport in `src/craft/backends.py` was extracted from the
Tiwater ticos-edge `tools/cap_llm_backends.py` implementation on 2026-11-19.
Its mocked regression tests were adapted from `tools/test_cap_llm_backends.py`.
Changes: CRAFT-specific environment variables, explicit model selection, removal
of edge/compiler-only tests and no dependency on private directories or robot
services. Other runtime modules in this baseline were written for CRAFT.

The architecture builds on prior CaP research. See `references/` for source
citations and overlap analysis. No implementation from the referenced research
repositories has been copied into this release. EvidenceScene is a sibling
project, not bundled source or a second estimator implemented here.
