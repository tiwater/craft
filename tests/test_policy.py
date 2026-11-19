import pytest
from craft.policy import validate, run

@pytest.mark.parametrize('body',[
 'import os', 'while True:\n        pass', 'sdk.__class__',
 'open("/tmp/file")', 'params.clear()', 'sdk = params',
 'return [x for x in params]', 'def nested():\n        return 1',
 'return sdk.perform(**params)', 'return {**params}',
 'return (lambda: 1)()', 'return params.__class__',
])
def test_rejects_unsupported_code_before_execution(body):
    with pytest.raises((ValueError,SyntaxError)):
        validate('def run_policy(sdk, params):\n    '+body)


def test_top_level_side_effect_is_rejected():
    with pytest.raises(ValueError): validate('sdk.perform("heat",{},"a")\ndef run_policy(sdk, params):\n    return {}')


def test_chained_comparison_and_short_circuit():
    source='def run_policy(sdk, params):\n    if 1 < params["x"] < 3 and True:\n        return {"ok": True}\n    return {"ok": False}'
    assert run(source,None,{'x':2})=={'ok':True}
    assert run(source,None,{'x':4})=={'ok':False}
