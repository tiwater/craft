# Evidence and limitations

- This is an alpha software research baseline, not a completed scientific result.
- Coffee/bagel examples are hand-authored deterministic fixtures. They validate
  protocol behavior only, not manipulation, heating, food quality or timing.
- No physical robot, live simulator, paid LLM or EvidenceScene accuracy benchmark
  is exercised by the standalone test suite.
- No learned skill library, applicability classifier, automatic repair loop,
  cross-policy recovery or concurrent multi-order scheduler is implemented yet.
- SHA-256 detects content drift relative to the expected hash; an attacker who
  replaces both source and hash is not prevented. It does not establish trust.
- The restricted interpreter is not a general sandbox for arbitrary Python.
  Host adapters retain physical authority and must enforce action conditions.
- Journal reconciliation cannot promise physical exactly-once execution, and
  cannot recover commands unknown to an external device after journal loss.
- Existing ticos-edge experiment results are historical integration evidence;
  they are not CRAFT benchmark results and have not been republished as such.
