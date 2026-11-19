# Recentring failure repetition

`protocol.json` was written before execution. Four fresh simulator trials use
the same offset and simulated acknowledgment timeout, in fixed order:
existing policy, generated policy, generated policy, existing policy.
No source or policy repair is permitted between trials. This is an attribution
probe, not a framework success-rate comparison. Two trials per policy cannot
establish equivalence or reliably estimate failure probability.

Each numbered JSON is produced by `../probe_aspire_host_simulation.py` with
`--fault-only --direct-only` and explicit host/simulator endpoints. Generated
trials additionally pass `--policy-file ../generated-coffee-baseline/policy.py`.
Existing-policy trials read the named constant from `--host-source`. Hashes
identify the evaluated policy bytes. All failures remain recorded.

The upstream wrapper is deliberately absent here: both variants execute via
the same existing host, SDK and isolated process. Generation budget is zero.
Scene resets are matched by preset, not identical recorded RGB-D observations.

Source inspection localizes the previous `empty_grasp` report to
`serve_isaac_live_cap_daemon.py` after `submit_stage_op("attach_nearest")` returns
without attachment. `EdgeZeroGtCoffeeSDK.recenter_cup_under_spout` passes its
existing measured cup position to the same driver and reports the rejection.
This identifies the reporting path, not the root cause. It does not demonstrate
that a different reasoning model, retry rule or perception owner would fix it.

## Results

All four predefined repetitions completed and passed fault checks. Delivery
errors in sequence were 12.947, 3.744, 2.663 and 5.148 mm. Both policies called
recentring and reconciled the injected unknown acknowledgment; each trial
recorded one brew execution. Both generated-policy runs used the same hash as
the earlier failed trial, with no edits. The failure was not reproduced in this
small follow-up. Its root cause remains unresolved; do not erase it or infer
that either policy is uniformly reliable. These results do not justify a new
CaP framework as the solution to the observed failure.
