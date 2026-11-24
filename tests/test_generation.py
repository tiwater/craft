import pytest
from trax import generation


def test_generation_is_only_a_draft(monkeypatch):
    def backend(provider,prompt,timeout):
        assert 'host' in prompt
        return {'raw_output':'```python\ndef run_policy(sdk, params):\n    return {"status":"blocked"}\n```'}
    monkeypatch.setattr(generation,'invoke_backend',backend)
    result=generation.generate('make coffee',{'host':'test'})
    assert result['approved'] is False
    assert result['validation']=='syntax_only'


def test_unsupported_model_output_is_rejected(monkeypatch):
    monkeypatch.setattr(generation,'invoke_backend',lambda *a: {'raw_output':'import os'})
    with pytest.raises(ValueError): generation.generate('make coffee',{})
