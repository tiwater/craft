# Source provenance

The generation transport in `src/trax/backends.py` was extracted from the
Tiwater ticos-edge `tools/cap_llm_backends.py` implementation on 2026-11-19.
Its mocked regression tests were adapted from `tools/test_cap_llm_backends.py`.
Changes: TRAX-specific environment variables, explicit model selection, removal
of edge/compiler-only tests and no dependency on private directories or robot
services. Other runtime modules in this baseline were written for TRAX.

The architecture builds on prior CaP research. See `references/` for source
citations and overlap analysis. No implementation from the referenced research
repositories has been copied into this release; design patterns from
NVlabs/ASPIRE (Apache-2.0, commit `fa00f0ef2bc1d91d594b9aa502c22133c9f538f7`)
and teaching methodology concepts from ACE-CaP are documented in
`docs/decisions/0003-product-architecture-and-reuse-boundary.md`. EvidenceScene
is a sibling project, not bundled source or a second estimator implemented here.
