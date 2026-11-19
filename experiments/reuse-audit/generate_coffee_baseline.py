"""Generate once through existing edge compiler; resets simulator, never actuates.
Uses existing producer summary and compiler rules. Not ASPIRE skill learning.
Run with the existing edge service interpreter; all endpoints are explicit.
"""
import sys,os,json,time,hashlib,argparse
from pathlib import Path
parser=argparse.ArgumentParser()
parser.add_argument('--edge-root',required=True)
parser.add_argument('--edge-base',required=True)
parser.add_argument('--sim-base',required=True)
parser.add_argument('--output-dir',required=True)
a=parser.parse_args()
sys.path.insert(0,str(Path(a.edge_root)/'tools'))
import httpx
import edge_cap_runtime as rt
import coffee_cap_compiler as compiler
out=Path(a.output_dir)
out.mkdir(parents=True, exist_ok=False)
os.environ['TICOS_CAP_LLM_PROVIDER']='codex_cli'
os.environ['TICOS_CAP_MODEL']='gpt-6-astra'
os.environ['TICOS_CAP_REASONING_EFFORT']='medium'
base=a.edge_base
with httpx.Client(timeout=20,trust_env=False) as c:
 state=c.get(base+'/cap/state').json()
 assert state['runtime']['status']!='running'
 c.post(base+'/cap/reset',json={}).raise_for_status()
 c.post(a.sim_base+'/v1/sim/testbed/setup',json={'preset':'default'}).raise_for_status()
 time.sleep(2)
 scene=c.get(base+'/edge-scene').json()
 runtime=rt.EdgeCapRuntime(robot_id='lsrx-sim-01',plant_base_url=a.sim_base,scene_provider=lambda:scene,wait_fresh_scene_fn=lambda n,t:False)
 summary=runtime._build_zero_gt_observation_summary()
 assert summary['control_ready'],summary.get('pipeline_health')
 def capture(prompt):
  (out/'prompt.txt').write_text(prompt)
  return {'code':'def run_policy(sdk, params):\n    return {}'}
 compiler.synthesize_or_repair_policy(order=runtime.orders['A042'],available_orders=runtime.orders,
 observation_summary=summary,instruction='制作一杯堂食热美式，送到1号取餐口。',llm_callable=capture)
 prompt=(out/'prompt.txt').read_text()
 started=time.monotonic()
 result=compiler.invoke_live_llm(prompt,timeout_s=180,validate_ast=False)
 (out/'policy.py').write_text(result['code'])
 violations=compiler.collect_policy_ast_violations(result['code'])
 report={k:v for k,v in result.items() if k in ('generator','model','reasoning_effort','latency_ms')}
 report.update(scope='single_generation_existing_engineered_prompt',prompt_sha256=hashlib.sha256(prompt.encode()).hexdigest(),policy_sha256=hashlib.sha256(result['code'].encode()).hexdigest(),wall_s=round(time.monotonic()-started,3),ast_violations=violations,model_calls=1,skill_learning_tested=False,oracle_in_prompt=False)
 (out/'generation.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(report),flush=True)
