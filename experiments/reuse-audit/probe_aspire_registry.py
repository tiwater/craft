"""Probe an unmodified ASPIRE executor with host tools; no physical dispatch.

Run with an environment containing pinned cap-harness. The observation is a
contract fixture, not a perception result or robot benchmark. No CRAFT runtime.
"""
import json
from collections.abc import Mapping
from cap_harness.registry import ToolRegistry
from cap_harness.runtime import ProgramExecutor

observation = {'source':'contract_fixture', 'control_ready':False,
               'objects':[{'id':'fixture-cup','world':[0.4,0.1,1.0]}]}
calls=[]
def observe_scene():
    calls.append('observe_scene')
    return observation

def query_appliance(command_id):
    calls.append('query_appliance')
    return {'command_id':command_id,'receipt':'unknown'}

registry=ToolRegistry(shared_public_names={'observe_scene','query_appliance'})
for function in (observe_scene,query_appliance):
    registry.register(function,name=function.__name__,layer='shared',capability='read_only',public=True)
executor=ProgramExecutor(registry)
result=executor.execute_program('scene = observe_scene()\nreceipt = query_appliance("heat-1")\nresult = {"scene": scene, "receipt": receipt}\n')
assert result.ok, result.error
def json_value(value):
    if isinstance(value, Mapping): return {k:json_value(v) for k,v in value.items()}
    if isinstance(value, (list,tuple)): return [json_value(v) for v in value]
    return value
assert json_value(result.result['scene'])==observation
assert result.result['receipt']['receipt']=='unknown'
assert calls==['observe_scene','query_appliance']
print(json.dumps({'scope':'unmodified_upstream_executor_contract_probe','ok':result.ok,
                 'observation_preserved':True,'tools':calls,'physical_commands':0,
                 'craft_runtime_used':False,'upstream_patch_required':False},indent=2))
