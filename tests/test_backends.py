"""Transport regression tests: no cloud requests or robot commands."""
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import io

import pytest

from craft import backends


def test_codex_preserves_prompt_and_uses_subscription_isolation(monkeypatch):
    monkeypatch.setenv("CRAFT_MODEL", "test-model")
    monkeypatch.setenv("OPENAI_API_KEY", "must-not-forward")
    def run(command, **kwargs):
        assert kwargs["input"] == "existing scene and repair feedback"
        assert "OPENAI_API_KEY" not in kwargs["env"]
        assert "--ignore-user-config" in command
        assert 'forced_login_method="chatgpt"' in command
        assert "features.shell_tool=false" in command
        assert command[command.index("--sandbox") + 1] == "read-only"
        assert kwargs["timeout"] == 2
        Path(command[command.index("--output-last-message") + 1]).write_text("policy")
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(backends.subprocess, "run", run)
    result = backends.invoke_backend("codex_cli", "existing scene and repair feedback", 2)
    assert result["raw_output"] == "policy"
    assert result["generator"] == "codex-cli-live"


@pytest.mark.parametrize("failure", ["timeout", "exit", "empty"])
def test_codex_fails_closed(monkeypatch, failure):
    monkeypatch.setenv("CRAFT_MODEL", "test-model")
    def run(command, **kwargs):
        if failure == "timeout":
            raise subprocess.TimeoutExpired(command, 1)
        return SimpleNamespace(returncode=1 if failure == "exit" else 0)
    monkeypatch.setattr(backends.subprocess, "run", run)
    with pytest.raises((TimeoutError, RuntimeError)):
        backends.invoke_backend("codex_cli", "prompt", 1)


@pytest.mark.parametrize("status", ["completed", "incomplete"])
def test_api_preserves_prompt_and_rejects_incomplete(monkeypatch, status):
    monkeypatch.delenv("CRAFT_REASONING_EFFORT", raising=False)
    monkeypatch.setenv("CRAFT_MODEL", "api-test-model")
    monkeypatch.setenv("OPENAI_API_KEY", "test-key")
    def urlopen(request, **kwargs):
        body = json.loads(request.data)
        assert body["input"] == "existing observation"
        assert body["model"] == "api-test-model"
        assert body["tools"] == []
        assert body["store"] is False
        assert body["reasoning"] == {"effort": "medium"}
        return io.StringIO(json.dumps({"status": status, "output": [
            {"type": "message", "content": [{"type": "output_text", "text": "policy"}]}]}))
    monkeypatch.setattr(backends.urllib.request, "urlopen", urlopen)
    if status == "completed":
        assert backends.invoke_backend("openai_api", "existing observation", 1)["raw_output"] == "policy"
    else:
        with pytest.raises(RuntimeError):
            backends.invoke_backend("openai_api", "existing observation", 1)


def test_configuration_required(monkeypatch):
    monkeypatch.delenv("CRAFT_MODEL", raising=False)
    with pytest.raises(ValueError, match="CRAFT_MODEL"):
        backends.invoke_backend("openai_api", "prompt", 1)
    monkeypatch.setenv("CRAFT_MODEL", "test")
    with pytest.raises(ValueError, match="Unknown"):
        backends.invoke_backend("typo", "prompt", 1)


def test_invalid_effort_rejected_before_transport(monkeypatch):
    monkeypatch.setenv("CRAFT_REASONING_EFFORT", "typo")
    with pytest.raises(ValueError, match="CRAFT_REASONING_EFFORT"):
        backends.invoke_backend("codex_cli", "prompt", 1)
