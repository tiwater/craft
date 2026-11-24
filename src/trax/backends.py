"""Model transports extracted from ticos-edge; no execution authority or paid fallback.

TRAX_MODEL is required. TRAX_REASONING_EFFORT defaults to medium.
Codex CLI authentication remains CLI-owned; API uses OPENAI_API_KEY.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import tempfile
from typing import Any
import urllib.error
import urllib.request


def invoke_backend(
    provider: str,
    prompt: str,
    timeout_s: float,
    *,
    model: str | None = None,
    reasoning_effort: str | None = None,
) -> dict[str, Any]:
    effort = (reasoning_effort if reasoning_effort is not None else os.environ.get("TRAX_REASONING_EFFORT", "medium")).strip()
    if effort not in {"low", "medium", "high", "xhigh"}:
        raise ValueError("TRAX_REASONING_EFFORT must be low, medium, high, or xhigh")
    resolved_model = (model if model is not None else os.environ.get("TRAX_MODEL", "")).strip()
    if not resolved_model:
        raise ValueError("TRAX_MODEL must name an available model for the selected backend")
    model = resolved_model
    if provider == "codex_cli":
        binary = os.environ.get("TRAX_CODEX_BIN", "codex")
        # Keep project instructions, plugins and MCP out of the compiler context.
        # Authentication remains owned by Codex; never extract or forward OAuth tokens.
        with tempfile.TemporaryDirectory(prefix="craft-generation-") as directory:
            output = Path(directory) / "policy.txt"
            command = [
                binary, "exec", "--ignore-user-config", "--ignore-rules",
                "--ephemeral", "--skip-git-repo-check", "--sandbox", "read-only",
                "-c", "features.shell_tool=false", "-c", 'web_search="disabled"',
                "-c", 'forced_login_method="chatgpt"',
                "-c", f'model_reasoning_effort="{effort}"',
                "--cd", directory, "--model", model,
                "--output-last-message", str(output), "-",
            ]
            env = dict(os.environ)
            for key in ("OPENAI_API_KEY", "CODEX_API_KEY"):
                env.pop(key, None)
            try:
                result = subprocess.run(command, input=prompt, text=True,
                                        capture_output=True, timeout=timeout_s, env=env)
            except subprocess.TimeoutExpired:
                raise TimeoutError(f"codex exec timed out after {timeout_s}s") from None
            if result.returncode:
                # CLI logs can contain prompt data; don't echo them to consumers.
                raise RuntimeError(f"codex exec failed (code={result.returncode}); check CLI login and quota")
            raw = output.read_text() if output.exists() else ""
        generator = "codex-cli-live"
    elif provider == "openai_api":
        key = os.environ.get("OPENAI_API_KEY", "").strip()
        if not key:
            raise ValueError("OPENAI_API_KEY is required for openai_api")
        body = json.dumps({"model": model, "input": prompt, "store": False,
                           "reasoning": {"effort": effort},
                           "tools": [], "max_output_tokens": 8192}).encode()
        request = urllib.request.Request(
            "https://api.openai.com/v1/responses", data=body,
            headers={"Authorization": f"Bearer {key}", "Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(request, timeout=timeout_s) as response:
                payload = json.load(response)
        except urllib.error.HTTPError as exc:
            raise RuntimeError(f"OpenAI Responses API failed (HTTP {exc.code})") from None
        if payload.get("status") != "completed":
            raise RuntimeError("OpenAI Responses API did not complete the policy")
        raw = "\n".join(
            part["text"] for item in payload.get("output", [])
            if item.get("type") == "message"
            for part in item.get("content", []) if part.get("type") == "output_text"
        )
        generator, binary = "openai-api-live", "openai-responses-api"
    else:
        raise ValueError(f"Unknown TRAX_LLM_PROVIDER: {provider}")
    if not raw.strip():
        raise RuntimeError(f"{provider} returned an empty policy")
    return {"raw_output": raw, "generator": generator, "binary": binary,
            "model": model, "reasoning_effort": effort}
