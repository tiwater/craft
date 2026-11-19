# Execution and ownership

Existing source used during extraction: `ticos-edge/tools/cap_llm_backends.py`
is the generation transport producer, called by the existing coffee compiler.
The edge runtime owns the original command records, device-specific policy
execution and perception adapter. CRAFT reuses the transport design, but does
not copy simulator-oracle helpers, hard-coded workspace anchors or private paths.

CRAFT's v0.1.0 owners are `PolicyBundle` (frozen source/configuration), `Journal`
(command intent and receipt records) and `Execution` (ordered policy effects).
`Adapter.observe` returns external state without inference. `Adapter.execute`
is the only actuation boundary. `Adapter.query` must never redispatch.
`Adapter.verify` inspects the host-bound goal rather than trusting policy text.

## Receipt lifecycle

1. Bind a stable run ID to the policy hash, parameters, deployment and goal.
2. Derive each command ID from run ID and a unique logical step key.
3. Commit intent durably before dispatch.
4. Record the adapter receipt. `running` or `unknown` suspends; `rejected` blocks.
5. Resume the same frozen policy with the same journal. Reuse completed receipts;
   query unresolved ones. Never infer absence of side effects from a timeout.
6. Verify the current goal before accepting a policy's `completed` return.

A crash between intent commit and dispatch deliberately leaves an unresolved
command. It may need operator reconciliation. We prefer an explicit block to
an unproven retry. The adapter must persist its own command identities/receipts
across restarts if recovery after device or process loss is required.

A fresh run ID can repeat effects. The operator must not create one to bypass an
uncertain command. Journal deletion loses protection. Cross-policy recovery,
automatic skill applicability and multi-worker scheduling are not implemented.
Use one execution worker per run/device. SQLite is a local journal, not a
distributed exactly-once protocol.

The interpreter supports a deliberately small Python subset; unsupported code
is rejected before execution. Adapters are trusted code and may block: the host
must enforce deadlines, cancellation and robot safety independently. Evidence
payloads are passed through, not semantically authenticated by this library.
