"""Read-only integration probe using the upstream executor and an existing edge snapshot.

No CRAFT runtime, perception inference, simulator oracle or actuation. Endpoint is
explicit configuration and is not stored in the public result.
"""
import argparse
from collections.abc import Mapping
import hashlib
import json
import time
import requests
from cap_harness.registry import ToolRegistry
from cap_harness.runtime import ProgramExecutor

p=argparse.ArgumentParser()
p.add_argument('--endpoint',required=True)
p.add_argument('--output',required=True)
a=p.parse_args()
s=requests.Session();s.trust_env=False
r=s.get(a.endpoint,timeout=5);r.raise_for_status();snapshot=r.json()
# Only existing producer fields are adapted; no derived scene state is published.
fields=['source','timestamp_ns','objects','cameras_available','l2_pouring_safety','brew_status','cup_filled']
observation={k:snapshot[k] for k in fields}
calls=[]
def observe_scene():
    calls.append('observe_scene')
    return observation
registry=ToolRegistry(shared_public_names={'observe_scene'})
registry.register(observe_scene,name='observe_scene',layer='shared',capability='read_only',public=True)
result=ProgramExecutor(registry).execute_program('result = observe_scene()\n')
def plain(v):
    if isinstance(v,Mapping):return {k:plain(x) for k,x in v.items()}
    if isinstance(v,(list,tuple)):return [plain(x) for x in v]
    return v
assert result.ok,str(result.error)
returned=plain(result.result)
assert returned==observation,'Upstream public-value conversion changed the selected evidence'
def sha(v):return hashlib.sha256(json.dumps(v,sort_keys=True,allow_nan=False).encode()).hexdigest()
report={'scope':'read_only_existing_edge_snapshot_through_upstream_executor',
        'upstream_commit':'fa00f0ef2bc1d91d594b9aa502c22133c9f538f7',
        'source_timestamp_ns':str(snapshot['timestamp_ns']),
        'snapshot_age_ms':round((time.time_ns()-int(snapshot['timestamp_ns']))/1e6,1),
        'object_count':len(observation['objects']),'fields':fields,
        'input_sha256':sha(observation),'output_sha256':sha(returned),
        'values_preserved':True,'calls':calls,'physical_commands':0,
        'new_perception_estimator':False,'craft_runtime_used':False}
with open(a.output,'w') as f:json.dump(report,f,indent=2);f.write('\n')
print(json.dumps(report,indent=2))
