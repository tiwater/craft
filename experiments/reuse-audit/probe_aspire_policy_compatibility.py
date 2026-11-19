"""Differential execution of an existing host policy on scripted contract inputs.

Source owner: EdgeZeroGtCoffeeSDK / EDGE_CLOSED_LOOP_COFFEE_POLICY_CODE in
edge_cap_runtime.py. No edge imports, live endpoints, CRAFT runtime or physics.
The original trusted source is evaluated only against this local scripted host.
This is NOT a test of the existing edge subprocess sandbox or actual SDK.
"""
import argparse
import ast
import hashlib
import json
from collections.abc import Mapping
from pathlib import Path
from cap_harness.registry import ToolRegistry
from cap_harness.runtime import ProgramExecutor


def plain(v):
    if isinstance(v, Mapping): return {k: plain(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)): return [plain(x) for x in v]
    return v


class ScriptedHost:
    def __init__(self, case):
        self.case = case
        self.calls = []
        self.observations = 0

    def call(self, name, *args, **kwargs):
        self.calls.append({'method': name, 'args': plain(args), 'kwargs': plain(kwargs)})
        if name == 'get_order':
            return {'destination_slot': '$SLOT_PICKUP_1',
                    'special_request': 'steam_wand_manual_purge_and_latte_art' if self.case == 'hazard' else None}
        if name == 'observe_scene':
            self.observations += 1
            cups = [{'id': 'fixture-cup', 'cup_type': 'cup', 'confidence': .9}]
            if self.case == 'no_cup': cups = []
            if self.case == 'ambiguity': cups += [{'id': 'fixture-cup-2', 'confidence': .9}]
            safe = self.case not in ('recenter', 'l2_block') or self.observations >= 3
            if self.case == 'l2_block': safe = False
            return {'control_ready': self.case != 'stale', 'objects': cups,
                    'confidence_threshold': .7, 'l2_pouring_safety': {'safe_to_pour': safe}}
        if name == 'start_coffee_machine':
            return {'receipt': 'unknown' if self.case.startswith('unknown') else 'running'}
        if name == 'query_command_status':
            return {'receipt': 'unknown' if self.case == 'unknown_unresolved' else 'running'}
        if name == 'place_cup_under_machine' and self.case == 'place_rejected':
            return {'receipt': 'rejected'}
        return {'receipt': 'done'}

    def __getattr__(self, name):
        return lambda *args, **kwargs: self.call(name, *args, **kwargs)


class ToolCallAdapter(ast.NodeTransformer):
    def visit_Attribute(self, node):
        if isinstance(node.value, ast.Name) and node.value.id == 'sdk':
            return ast.copy_location(ast.Name(id=node.attr, ctx=node.ctx), node)
        return self.generic_visit(node)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host-source', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    source = Path(args.host_source).read_text()
    module = ast.parse(source)
    policy = next(ast.literal_eval(n.value) for n in module.body
                  if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and
                  t.id == 'EDGE_CLOSED_LOOP_COFFEE_POLICY_CODE' for t in n.targets))
    tree = ast.parse(policy)
    names = sorted({n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)
                    and isinstance(n.value, ast.Name) and n.value.id == 'sdk'})
    # Explicit signature adaptation only: sdk.method -> registered method;
    # sdk is no longer needed as a function argument. Keep all branch logic.
    adapted = ToolCallAdapter().visit(tree)
    adapted.body[0].args.args = [a for a in adapted.body[0].args.args if a.arg != 'sdk']
    adapted_code = ast.unparse(ast.fix_missing_locations(adapted)) + '\nresult = run_policy({})\n'
    expected = {'normal': 'completed', 'recenter': 'completed',
                'unknown_resolved': 'completed', 'unknown_unresolved': 'failed_brew_start',
                'stale': 'rejected_observation_not_ready', 'no_cup': 'failed',
                'ambiguity': 'paused_for_operator', 'hazard': 'escalated_to_human',
                'place_rejected': 'failed_place', 'l2_block': 'failed_l2_pouring_gate'}
    rows = []
    for case, status in expected.items():
        baseline = ScriptedHost(case)
        namespace = {}
        exec(compile(policy, '<trusted-existing-policy>', 'exec'), namespace)
        baseline_result = namespace['run_policy'](baseline, {})
        upstream_host = ScriptedHost(case)
        registry = ToolRegistry(shared_public_names=names)
        for name in names:
            registry.register(getattr(upstream_host, name), name=name,
                              layer='shared', capability='contract_fixture', public=True)
        result = ProgramExecutor(registry).execute_program(adapted_code)
        assert result.ok, (case, result.error)
        assert plain(result.result) == baseline_result, case
        assert upstream_host.calls == baseline.calls, case
        assert baseline_result['status'] == status, case
        if case.startswith('unknown'):
            assert sum(c['method'] == 'start_coffee_machine' for c in baseline.calls) == 1
            assert sum(c['method'] == 'query_command_status' for c in baseline.calls) == 1
        rows.append({'case': case, 'status': status, 'same_result': True,
                     'same_calls': True, 'call_count': len(baseline.calls)})
    report = {'scope': 'scripted_host_differential_policy_execution',
              'policy_sha256': hashlib.sha256(policy.encode()).hexdigest(),
              'adapted_sha256': hashlib.sha256(adapted_code.encode()).hexdigest(),
              'upstream_commit': 'fa00f0ef2bc1d91d594b9aa502c22133c9f538f7',
              'physical_commands': 0, 'model_calls': 0, 'craft_runtime_used': False,
              'actual_host_sdk_tested': False, 'cases': rows}
    Path(args.output).write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
