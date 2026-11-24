import dataclasses
import pytest
from trax.contracts import Receipt
from trax.fixtures import DEPLOYMENT, FixtureAdapter, example_policy
from trax.journal import Journal
from trax.runtime import PolicyBundle, execute


def invoke(bundle, adapter, path, **changes):
    args = dict(run_id="order-1", parameters={"order_id": "order-1"}, deployment=DEPLOYMENT, goal={"task":"combo"})
    args.update(changes)
    with Journal(path) as journal:
        return execute(bundle, adapter, journal, **args)


@pytest.mark.parametrize("task", ["coffee", "bagel", "combo"])
@pytest.mark.parametrize("loss", [False, True])
def test_resume_queries_without_repeating_side_effects(tmp_path, task, loss):
    adapter = FixtureAdapter(lose_receipt=("brew" if task == "coffee" else "heat") if loss else None)
    bundle = example_policy(task)
    path = tmp_path / "journal.sqlite"
    first = invoke(bundle, adapter, path, goal={"task":task})
    assert first['result']['status'] == ('pending' if loss else 'completed')
    # New SQLite connection models journal reopening, not a physical robot restart.
    second = invoke(bundle, adapter, path, goal={"task":task})
    assert second['result']['status'] == 'completed'
    assert len(adapter.calls) == len(set(adapter.calls))


def test_unknown_remains_pending_without_redispatch(tmp_path):
    class Disconnected(FixtureAdapter):
        def execute(self, action, arguments, command_id):
            self.calls.append(action)
            raise TimeoutError()
    adapter = Disconnected()
    for _ in range(3):
        assert invoke(example_policy('combo'),adapter,tmp_path/'j')['result']['status'] == 'pending'
    assert len(adapter.calls) == 1


def test_crash_after_claim_before_dispatch_does_not_assume_no_effect(tmp_path):
    from trax.contracts import digest
    bundle = example_policy('combo')
    with Journal(tmp_path/'j') as j:
        j.claim(digest(['order-1','place_cup']), {'action':'place_cup','arguments':{'order_id':'order-1'}})
    adapter = FixtureAdapter()
    assert invoke(bundle,adapter,tmp_path/'j')['result']['status'] == 'pending'
    assert adapter.calls == []


@pytest.mark.parametrize('change', ['parameters','deployment','goal','source'])
def test_resume_rejects_identity_drift_before_dispatch(tmp_path, change):
    adapter = FixtureAdapter()
    bundle = example_policy('combo')
    invoke(bundle,adapter,tmp_path/'j')
    count = len(adapter.calls)
    changes = {}
    if change == 'source': bundle = PolicyBundle.freeze(bundle.source+'\n', DEPLOYMENT, bundle.required_capabilities)
    elif change == 'parameters': changes[change] = {'order_id':'different'}
    elif change == 'deployment': changes[change] = {'adapter':'changed'}
    else: changes[change] = {'task':'coffee'}
    with pytest.raises(ValueError): invoke(bundle,adapter,tmp_path/'j',**changes)
    assert len(adapter.calls) == count


def test_tampered_bundle_rejected(tmp_path):
    b = example_policy('combo')
    with pytest.raises(ValueError,match='hash'):
        invoke(dataclasses.replace(b,source=b.source+'\n'),FixtureAdapter(),tmp_path/'j')


def test_command_key_collision_is_not_dispatched(tmp_path):
    with Journal(tmp_path/'j') as j:
        assert j.claim('x',{'action':'heat'})[0]
        with pytest.raises(ValueError): j.claim('x',{'action':'brew'})


@pytest.mark.parametrize('source',[
    'def run_policy(sdk, params):\n    return {"status":"completed"}',
    'def run_policy(sdk, params):\n    sdk.verify({"task":"combo"})\n    return {"status":"completed"}',
])
def test_claiming_success_without_verified_goal_is_blocked(tmp_path,source):
    b=PolicyBundle.freeze(source,DEPLOYMENT,[])
    assert invoke(b,FixtureAdapter(),tmp_path/'j')['result']['status']=='blocked'


def test_policy_cannot_redefine_goal(tmp_path):
    b=PolicyBundle.freeze('def run_policy(sdk, params):\n    return sdk.verify({"task":"coffee"})',DEPLOYMENT,[])
    with pytest.raises(ValueError,match='redefine'): invoke(b,FixtureAdapter(),tmp_path/'j')


@pytest.mark.parametrize('invalid', ['identity','empty_evidence','status'])
def test_bad_receipt_blocks_progress(tmp_path,invalid):
    class Bad(FixtureAdapter):
        def execute(self,action,arguments,command_id):
            if invalid=='identity': return Receipt('wrong','done',{'fake':True})
            if invalid=='empty_evidence': return Receipt(command_id,'done',{})
            return Receipt(command_id,'invalid',{})
    assert invoke(example_policy('combo'),Bad(),tmp_path/'j')['result']['status']=='pending'


def test_missing_capability_blocks_before_execution(tmp_path):
    adapter=FixtureAdapter(); adapter.capabilities=frozenset()
    with pytest.raises(ValueError,match='capabilities'):
        invoke(example_policy('combo'),adapter,tmp_path/'j')
    assert adapter.calls==[]


def test_rejected_action_does_not_continue(tmp_path):
    b=PolicyBundle.freeze('def run_policy(sdk, params):\n    sdk.perform("heat", {}, "heat")\n    return {"status":"completed"}',DEPLOYMENT,['heat'])
    adapter=FixtureAdapter()
    for _ in range(2): assert invoke(b,adapter,tmp_path/'j')['result']['status']=='blocked'
    assert adapter.calls==['heat']
