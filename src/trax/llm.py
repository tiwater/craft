"""Pure-stdlib LLM provider helper for TRAX policy exploration, compilation, and repair."""
from __future__ import annotations

import re
import time
from typing import Any, Callable, Mapping

from .backends import invoke_backend

LLMGenerateFn = Callable[[str], Mapping[str, Any]]


def strip_code_fence(text: str) -> str:
    """Extract Python source code from a markdown-fenced or raw LLM response."""
    s = (text or "").strip()
    m = re.search(r"```(?:python)?\s*\n(.*?)```", s, flags=re.DOTALL | re.IGNORECASE)
    if m:
        return m.group(1).strip()
    return s


def make_live_llm_fn(
    *,
    provider: str = "codex_cli",
    model: str = "gpt-6-astra",
    reasoning_effort: str = "medium",
    timeout_s: float = 180.0,
    max_retries: int = 3,
    retry_backoff_s: float = 8.0,
) -> LLMGenerateFn:
    """Create a retry-capable LLM invocation callable backed by TRAX's stdlib provider adapter."""

    def _call(prompt: str) -> dict[str, Any]:
        last_exc: Exception | None = None
        for attempt in range(max(1, int(max_retries) + 1)):
            t0 = time.monotonic()
            try:
                res = invoke_backend(
                    provider,
                    prompt,
                    timeout_s,
                    model=model,
                    reasoning_effort=reasoning_effort,
                )
                return {
                    "raw_output": res["raw_output"],
                    "source": strip_code_fence(res["raw_output"]),
                    "model": res["model"],
                    "reasoning_effort": res["reasoning_effort"],
                    "wall_s": round(time.monotonic() - t0, 3),
                }
            except (RuntimeError, TimeoutError) as exc:
                last_exc = exc
                if attempt >= max_retries:
                    raise
                time.sleep(retry_backoff_s * (attempt + 1))
        assert last_exc is not None
        raise last_exc

    return _call
