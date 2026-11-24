# Host capability audit — 2026-11-19

Scope: inspected current local edge and simulator sources and the running
`/cap/state` response. No new estimator, simulated appliance or action added.
Runtime SDK contract hash:
`15fcbdce0816a49f1534acbff8c247f5bf843a625aca0b299ea65ef644313e56`.

| Required task capability | Current evidence | Readiness |
| --- | --- | --- |
| Cup placement, brewing and delivery | Existing `EdgeZeroGtCoffeeSDK`; four host-delegation simulator runs | Smoke-tested, not statistically validated |
| Offset correction and uncertain brew acknowledgment | Existing policy + SDK, two injected simulator runs | Observed recovery within one process/session |
| Bagel pickup and placement | Scene meshes exist; inspected attachment/reset code enumerates cup primitives | Not validated for bagels |
| Oven door operation and state | No such method in running SDK or inspected actuator dispatch | Missing integration contract |
| Heating start, completion and reconciliation | Actuator dispatch provides coffee-machine operations only | Missing integration contract |
| Heated-item unloading and task completion | No inspected host primitive or evaluator | Missing capability and scoring evidence |
| Combined coffee/bagel order | Existing runtime completes a drink order; no complete warm-food path | Not evaluable end to end |

## Authoritative sources and ownership

- `ticos-edge/tools/edge_cap_runtime.py`: `EdgeCapRuntime._run_job` constructs
  `EdgeZeroGtCoffeeSDK`; existing scene provider supplies observations.
- `ticos-edge/tools/coffee_cap_sdk.py`: `SDKPublicProxy._ALLOWED_METHODS` lists
  the same 13 methods exposed by the running service. The SDK owns command
  handling and interlocks; the isolated worker executes policy code.
- `ticos-edge/tools/serve_isaac_live_cap_daemon.py`: `blind_actuate` dispatches
  halt/resume, retract, blind pick/place and coffee start/wait operations;
  unsupported operation names are rejected. Its testbed reset and attachment
  paths explicitly handle cup bodies.
- `ticos-sim/cafe_scene.py`: bagel/oven geometry uses `box` and `cyl` scene
  construction helpers. This is visual scene evidence, not evidence of an
  operational heating appliance or articulated door.
- `ticos-sim/sim_state.py`: inspected physics and coffee-machine state owners.
  No bagel/oven cycle appeared in this inspected execution path. This audit
  does not claim no other implementation exists anywhere.

## Consequences for the reuse decision

All candidate frameworks face the same missing host capabilities. Failing to
heat a bagel on this setup cannot distinguish their algorithms. Do not emulate
success with CRAFT fixtures, inject geometry into perception, or claim that
adding an oven API is a novel policy-learning method.

Before a bagel comparison, implement device behavior in its existing simulator
or device owner and expose a documented host contract. Establish controllable
item identity, door/occupancy evidence, cycle command identity, completion
evidence and recovery after uncertain acknowledgments. Define the task score
from independent simulator/device evidence. A timer alone must not be described
as measured heating or food quality. All methods must then receive identical
observation privileges, actions and task definitions.

For now, prioritize the coffee policy-development experiment, which is feasible
with the current host. Treat broader multi-appliance workstation research as a hypothesis pending
this common infrastructure, not justification for a new general CaP runtime.
