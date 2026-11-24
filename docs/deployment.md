# Deployment contract

The package distribution is `tiwater-trax`; the Python namespace is `craft`.
Only GitHub wheel/sdist releases are planned for this bootstrap; no PyPI upload
is implied. Install into an isolated virtual environment.

Codex generation inherits the CLI-owned login, uses an empty temporary working
directory, ignores user/project configuration/rules, disables shell/web tools,
and removes API-key variables. It requires a CLI supporting `exec`,
`--ignore-user-config`, `--ignore-rules`, `--ephemeral`, `--sandbox read-only`
and `--output-last-message`. Unsupported CLIs fail; there is no silent fallback.
The API backend uses Responses, explicitly selected model/effort and no tools.
These transports are tested with mocks; release testing does not spend tokens.

A ticos-edge adapter should provide the existing EvidenceScene observation
snapshot and existing blind actuator commands through Adapter. Keep machine
calibration and credentials outside the package. Do not change the production
pose owner. Compare held-out recordings and physical task outcomes before a
cutover. v0.1.0 has no installed live edge adapter and no oven driver.

Required real oven contract: door state, occupancy identity, heat-cycle identity,
timer/completion evidence, permitted tools, temperature handling conditions,
interlock rejection, cancellation behavior and persistent command queries.
`heat` being acknowledged is not evidence that food is ready to remove.
