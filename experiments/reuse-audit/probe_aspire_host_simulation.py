"""Paired simulator host-delegation probe, not ASPIRE policy learning.

Owners: existing edge /cap/execute -> EdgeCapRuntime._run_job -> isolated
coffee_cap_sdk worker -> EdgeZeroGtCoffeeSDK. Scene state stays on that path.
This explicitly resets the simulator. The oracle is used for scoring only.
"""
import argparse
import ast
from collections.abc import Mapping
import hashlib
import json
import math
from pathlib import Path
import time
import requests
from cap_harness.registry import ToolRegistry
from cap_harness.runtime import ProgramExecutor


def plain(v):
    if isinstance(v, Mapping): return {k: plain(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)): return [plain(x) for x in v]
    return v


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--edge-base', required=True, help='Existing robot-scoped edge URL')
    p.add_argument('--sim-base', required=True)
    p.add_argument('--host-source', required=True)
    p.add_argument('--policy-file', help='Optional frozen generated policy; never repaired during this run')
    p.add_argument('--fault-only', action='store_true')
    p.add_argument('--direct-only', action='store_true')
    p.add_argument('--output', required=True)
    a = p.parse_args()
    session = requests.Session()
    session.trust_env = False
    def get(url):
        r = session.get(url, timeout=5); r.raise_for_status(); return r.json()
    def post(url, body):
        r = session.post(url, json=body, timeout=180); r.raise_for_status(); return r.json()
    tree = ast.parse(Path(a.host_source).read_text())
    code = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == 'EDGE_CLOSED_LOOP_COFFEE_POLICY_CODE' for t in n.targets))
    if a.policy_file:
        code = Path(a.policy_file).read_text()
    # Do not interrupt another execution; simulator testbed presence is required.
    state = get(a.edge_base + '/cap/state')
    assert state['runtime']['status'] != 'running', 'Host already executing'
    get(a.sim_base + '/v1/sim/testbed/oracle')
    report = {'scope': 'simulator_host_delegation_smoke_test', 'model_calls': 0,
              'policy_source': 'provided_frozen_file' if a.policy_file else 'existing_host_constant',
              'policy_sha256': hashlib.sha256(code.encode()).hexdigest(),
              'upstream_commit': 'fa00f0ef2bc1d91d594b9aa502c22133c9f538f7',
              'actual_host_sdk_tested': True, 'aspire_learning_tested': False,
              'oracle_in_policy_input': False, 'runs': []}
    def save():
        Path(a.output).write_text(json.dumps(report, indent=2) + '\n')
    save()
    for fault in ((True,) if a.fault_only else (False, True)):
        for mode in (('direct_host',) if a.direct_only else ('direct_host', 'aspire_delegates_to_host')):
            post(a.edge_base + '/cap/reset', {})
            post(a.sim_base + '/v1/sim/testbed/setup', {'preset': 'default'})
            initial = get(a.edge_base + '/edge-scene')
            deadline = time.monotonic() + 15
            while True:
                scene = get(a.edge_base + '/edge-scene')
                cups = [o for o in scene.get('objects', []) if o.get('semantic_class') in ('dine_in_red', 'takeout_green')]
                if scene.get('step_seq', 0) >= initial.get('step_seq', 0) + 3 and len(cups) == 1:
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError('Fresh single-cup producer snapshot unavailable')
                time.sleep(.25)
            prefix = 'reuse-audit-' + str(time.time_ns())
            body = {'commandId': prefix, 'policyCode': code, 'wait': True,
                    'orderId': 'A042', 'targetSlot': '$SLOT_PICKUP_1',
                    'simulateTimeout': fault,
                    'injectPlaceSlipXyM': [0.038, -0.012] if fault else None}
            def execute_host_policy():
                # Existing host retains isolation, observation, interlocks and ledger.
                return post(a.edge_base + '/cap/execute', body)
            started = time.monotonic()
            if mode == 'direct_host':
                response = execute_host_policy()
            else:
                registry = ToolRegistry(shared_public_names={'execute_host_policy'})
                registry.register(execute_host_policy, name='execute_host_policy',
                                  layer='shared', capability='host_policy_dispatch', public=True)
                result = ProgramExecutor(registry).execute_program('result = execute_host_policy()')
                if not result.ok:
                    report['upstream_error'] = str(result.error); save()
                    raise RuntimeError(result.error)
                response = plain(result.result)
            rt = response['runtime']
            oracle = get(a.sim_base + '/v1/sim/testbed/oracle')
            err = math.hypot(oracle['cup'][0] - .38, oracle['cup'][1] + .17) * 1000
            plan = (rt.get('auditRecord') or {}).get('plan', [])
            methods = [s.get('method') for s in rt.get('steps', [])]
            res = rt.get('result') or {}
            row = {'mode': mode, 'fault_injection': fault, 'status': rt.get('status'),
                   'wall_s': round(time.monotonic() - started, 3),
                   'delivery_error_mm': round(err, 3),
                   'cup_filled': bool(oracle['coffee_machine'].get('cup_filled')),
                   'methods': methods,
                   'brew_execution_count': sum(int(x.get('execution_count', 0)) for x in plan if x.get('skill_kind') == 'start_coffee_machine'),
                   'sandbox_mode': (res.get('sandbox_isolation') or {}).get('mode'),
                   'producer_timestamp_ns': str(scene.get('timestamp_ns'))}
            row['task_passed'] = row['status'] == 'completed' and row['cup_filled'] and err <= 15
            if not row['task_passed']:
                row['failure_receipt'] = res.get('step_receipt')
            row['fault_checks_passed'] = None
            if fault:
                unknown_at = [i for i, name in enumerate(methods) if name == 'command_ack_timeout_unknown']
                row['fault_checks_passed'] = (
                    'recenter_cup_under_spout' in methods
                    and row['brew_execution_count'] == 1
                    and bool(unknown_at)
                    and all('query_command_status' in methods[i+1:] for i in unknown_at))
            report['runs'].append(row); save(); print(json.dumps(row), flush=True)
            if not row['task_passed']:
                # Record the actual failure before attempting further comparisons.
                report['stopped_after_host_failure'] = True; save(); return


if __name__ == '__main__':
    main()
