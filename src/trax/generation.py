"""Generate a draft; generation never approves or executes robot commands."""
from .backends import invoke_backend
from .contracts import canonical
from .policy import validate


def generate(instruction: str, context: dict, *, provider="codex_cli", timeout_s=180):
    prompt = """Write only Python defining def run_policy(sdk, params).
Supported: assignments to local names, if/else, return, JSON literals,
params['key'], comparisons and boolean logic. No loops, imports, arithmetic,
other functions or arbitrary method calls.
Only SDK calls: sdk.observe(), sdk.perform(action, arguments, key), sdk.verify(goal).
perform requires a stable unique logical step key and returns only when done;
otherwise the runtime suspends. Never invent capability names or scene evidence.
Return status completed only after sdk.verify(goal) returns True.
Generated code is a draft requiring external review and validation before use.
Input follows as JSON:
""" + canonical({"instruction": instruction, "context": context})
    result = invoke_backend(provider, prompt, timeout_s)
    source = result["raw_output"].strip()
    if source.startswith("```python\n") and source.endswith("```"):
        source = source[len("```python\n"):-3].strip()
    validate(source)
    return {**result, "source": source, "validation": "syntax_only", "approved": False}
