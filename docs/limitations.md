# Evidence and limitations

- TRAX separates four evidence sources strictly across every report, skill entry, and release artifact:
  - `fixture`: deterministic in-process software contract fixtures (no robot physics).
  - `simulator`: MuJoCo (`mujoco_cafe` / Robosuite Panda), Gazebo Fortress (`gazebo_cafe`), or external HTTP simulation hosts.
  - `recorded_data`: zero-actuation offline shadow replay on recorded perception frames.
  - `physical_robot`: live physical-robot execution (not yet claimed by standalone repo benchmarks).
- Simulation passes (including noisy/domain-randomized MuJoCo, Sim-to-Sim adaptation, Robosuite parity, and multi-appliance combo benchmarks) are **not** physical-robot approval, hardware safety certification, or food-quality certification.
- SHA-256 content hashes detect drift relative to frozen policy/contract/criteria digests; they are not cryptographic signatures and do not establish physical trust.
- The restricted AST interpreter and skill-snippet validator are not general-purpose OS sandboxes for arbitrary untrusted code. Host adapters retain physical authority and must enforce all hardware, thermal, door, tool, occupancy, and motion interlocks.
- Local SQLite intent-before-dispatch journals cannot guarantee distributed physical exactly-once execution, and cannot recover commands unknown to an external device after journal loss; operator reconciliation remains mandatory whenever outcomes are ambiguous.
- Multi-appliance oven/bagel execution (`combo_bench.py`, `examples/combo_coffee_bagel_config.json`) models explicit host capability contracts and interlocks in simulation only; physical oven deployment remains blocked until host hardware interlocks are verified on the target appliance.
- Prior-work baselines (`ASPIRE-Protocol`, `CaP-X-Protocol`, `SimEX-Protocol`) implemented inside TRAX's benchmarks are matched-contract, matched-budget **adapted protocols**, not official reproductions of unreleased author codebases.
- Existing `ticos-edge` experiment results are historical integration evidence; they are not republished as standalone TRAX benchmark results.
