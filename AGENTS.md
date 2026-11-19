# CRAFT contribution rules

- Preserve the separation between perception owner, CRAFT policy execution and host actuation.
- Never add inferred or fabricated scene state to improve a demonstration.
- Document existing producer/state ownership before changing an integration.
- Unknown physical outcomes require reconciliation; do not automatically retry non-idempotent actions.
- Do not claim exactly-once physical execution from a local journal alone.
- Model output is a draft. Syntax validation is not hardware approval or safety verification.
- Keep fixture, simulator, recorded-data and physical-robot evidence explicitly separate.
- No hard-coded private machine paths, credentials, live endpoints or customer data.
- Literature records belong in references/catalog.json and references/references.bib.
  Record reading depth honestly; do not invent authors, results or publication venues.
- Test failure, restart, identity drift and partial execution, not just happy paths.
- New robot actions require documented host capability contracts and verification.
