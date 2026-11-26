"""AST sandbox preflight, cancellation token, and SQLite-journaled MCP execution proxy.

Pure Python stdlib. Enforces static AST safety, runtime call budgets, cooperative
operator abort/E-Stop cancellation, and intent-before-dispatch SQLite journaling for
any MCP tool marked `idempotentHint: false` or `destructiveHint: true`.
"""
from __future__ import annotations

import ast
import json
import math
from pathlib import Path
import threading
import time
from typing import Any, Callable, Mapping, Sequence

from .journal import Journal
from .mcp_protocol import MCPClient
from .spatial_contract import PERCEPTION_BLOCK_STATUSES, SceneCursor


class ExecutionAbortedError(RuntimeError):
    """Raised when an operator or supervisor cancels an active closed-loop/policy execution."""


class CancellationToken:
    """Thread-safe cooperative cancellation token for live exploration, LLM, and policy runs."""

    def __init__(self) -> None:
        self._event = threading.Event()
        self._lock = threading.Lock()
        self._reason: str = ""

    def cancel(self, reason: str = "Aborted by operator") -> None:
        with self._lock:
            if not self._reason:
                self._reason = reason or "Aborted by operator"
        self._event.set()

    @property
    def is_cancelled(self) -> bool:
        return self._event.is_set()

    @property
    def reason(self) -> str:
        with self._lock:
            return self._reason

    def raise_if_cancelled(self) -> None:
        if self._event.is_set():
            raise ExecutionAbortedError(self.reason or "Execution aborted")


SAFE_BUILTINS: dict[str, Any] = {
    "abs": abs,
    "all": all,
    "any": any,
    "bool": bool,
    "chr": chr,
    "dict": dict,
    "divmod": divmod,
    "enumerate": enumerate,
    "filter": filter,
    "float": float,
    "int": int,
    "isinstance": isinstance,
    "iter": iter,
    "len": len,
    "list": list,
    "map": map,
    "max": max,
    "min": min,
    "next": next,
    "ord": ord,
    "pow": pow,
    "range": range,
    "reversed": reversed,
    "round": round,
    "set": set,
    "sorted": sorted,
    "str": str,
    "sum": sum,
    "tuple": tuple,
    "zip": zip,
    "math": math,
    "Exception": Exception,
    "ValueError": ValueError,
    "RuntimeError": RuntimeError,
    "KeyError": KeyError,
    "TypeError": TypeError,
}

FORBIDDEN_CALLS = frozenset(
    {
        "eval",
        "exec",
        "compile",
        "open",
        "__import__",
        "getattr",
        "setattr",
        "delattr",
        "globals",
        "locals",
        "vars",
        "input",
        "breakpoint",
    }
)

_FORBIDDEN_HOST_AST_NODES = (
    ast.Import,
    ast.ImportFrom,
    ast.ClassDef,
    ast.With,
    ast.AsyncFunctionDef,
    ast.AsyncFor,
    ast.AsyncWith,
    ast.Await,
    ast.Lambda,
)


def _utc_hms() -> str:
    return time.strftime("%H:%M:%S", time.gmtime())


def check_sandbox_ast_compliance(source: str) -> dict[str, Any]:
    """Static AST compliance check for host-sandboxed Python policies."""
    try:
        tree = ast.parse(source)
    except SyntaxError as exc:
        return {"compliant": False, "issues": [f"SyntaxError line {exc.lineno}: {exc.msg}"]}
    has_run_policy = any(isinstance(n, ast.FunctionDef) and n.name == "run_policy" for n in tree.body)
    if not has_run_policy:
        return {"compliant": False, "issues": ["Missing required `def run_policy(sdk, params):` entry point"]}
    issues: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, _FORBIDDEN_HOST_AST_NODES):
            issues.append(f"Line {getattr(node, 'lineno', '?')}: forbidden construct `{type(node).__name__}`")
        elif isinstance(node, ast.FunctionDef) and node.name.startswith("_"):
            issues.append(f"Line {node.lineno}: function name `{node.name}` starts with `_`")
        elif isinstance(node, ast.Name) and node.id.startswith("_"):
            issues.append(f"Line {node.lineno}: identifier `{node.id}` starts with `_`")
        elif isinstance(node, ast.Attribute) and node.attr.startswith("_"):
            issues.append(f"Line {node.lineno}: attribute `.{node.attr}` starts with `_`")
    return {"compliant": len(issues) == 0, "issues": issues}


def check_mcp_policy_ast(
    source: str,
    *,
    allowed_tools: Sequence[str],
    entry_fn: str = "run_policy",
) -> dict[str, Any]:
    """Static AST preflight gate grounded dynamically in the live MCP `tools/list` catalog."""
    violations: list[str] = []
    allowed_set = set(allowed_tools)
    try:
        tree = ast.parse(source)
    except SyntaxError as exc:
        return {"compliant": False, "violations": [f"SyntaxError line {exc.lineno}: {exc.msg}"], "called_tools": []}

    found_entry = False
    called_tools: set[str] = set()

    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == entry_fn:
            found_entry = True

    for node in ast.walk(tree):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            violations.append(f"Line {getattr(node, 'lineno', '?')}: imports are forbidden inside TRAX AST sandbox")
        elif isinstance(node, ast.Lambda):
            violations.append(f"Line {getattr(node, 'lineno', '?')}: lambda expressions are forbidden (use named def)")
        elif isinstance(node, ast.Name) and node.id.startswith("_"):
            violations.append(f"Line {getattr(node, 'lineno', '?')}: private/dunder name `{node.id}` is forbidden")
        elif isinstance(node, ast.Attribute) and node.attr.startswith("_"):
            violations.append(f"Line {getattr(node, 'lineno', '?')}: private/dunder attribute `.{node.attr}` is forbidden")
        elif isinstance(node, ast.Call):
            if isinstance(node.func, ast.Name) and node.func.id in FORBIDDEN_CALLS:
                violations.append(f"Line {getattr(node, 'lineno', '?')}: builtin `{node.func.id}` is forbidden")
            if isinstance(node.func, ast.Attribute) and isinstance(node.func.value, ast.Name):
                if node.func.value.id in ("mcp", "sdk"):
                    tool_name = node.func.attr
                    called_tools.add(tool_name)
                    if allowed_set and tool_name not in allowed_set:
                        violations.append(
                            f"Line {getattr(node, 'lineno', '?')}: tool `{tool_name}` is not in live MCP tools/list"
                        )

    if not found_entry:
        violations.insert(0, f"Missing required entry point `def {entry_fn}(mcp, ...):`")

    return {
        "compliant": len(violations) == 0,
        "violations": violations,
        "called_tools": sorted(called_tools),
    }


def extract_mcp_payload(resp_obj: Mapping[str, Any]) -> Any:
    if "error" in resp_obj:
        raise RuntimeError(f"MCP error: {resp_obj['error'].get('message', resp_obj['error'])}")
    result = resp_obj.get("result") or {}
    if "structuredContent" in result:
        return result["structuredContent"]
    content = result.get("content") or []
    if content and isinstance(content[0], Mapping) and "text" in content[0]:
        txt = str(content[0]["text"])
        try:
            return json.loads(txt)
        except ValueError:
            return {"text": txt}
    return result


class GovernedMCPProxy:
    """Dynamic Python proxy exposing every discovered MCP tool with SQLite Intent Journal governance."""

    def __init__(
        self,
        client: MCPClient,
        tools: Sequence[Mapping[str, Any]],
        *,
        journal_path: Path,
        run_tag: str,
        max_calls: int = 60,
        cancel_token: CancellationToken | None = None,
        on_wire: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        self._client = client
        self._tools_by_name: dict[str, dict[str, Any]] = {str(t["name"]): dict(t) for t in tools if t.get("name")}
        self._journal_path = journal_path
        self._run_tag = run_tag
        self._max_calls = max_calls
        self._cancel_token = cancel_token
        self._on_wire = on_wire

        self.call_count: int = 0
        self.trace: list[dict[str, Any]] = []
        self.methods: list[str] = []
        self.non_idempotent_counts: dict[str, int] = {}
        self.limit_refusals: list[dict[str, Any]] = []
        self.unknown_events_observed: int = 0
        self.pending_unknown: bool = False
        self.queried_after_unknown: bool = True
        self.journal_entries: list[dict[str, Any]] = []
        self.spatial_cursors: list[dict[str, Any]] = []
        self.last_spatial_cursor: dict[str, Any] | None = None
        self.perception_blocks: list[dict[str, Any]] = []
        self._unresolved_perception_block: bool = False
        self.acted_after_perception_block: bool = False

    def __getattr__(self, name: str) -> Callable[..., Any]:
        if name.startswith("_") or name not in self._tools_by_name:
            raise AttributeError(f"MCP tool `{name}` is not exposed by the active robot backend")
        spec = self._tools_by_name[name]
        prop_names = list(((spec.get("inputSchema") or {}).get("properties") or {}).keys())
        ann = spec.get("annotations") or {}
        is_non_idempotent = (ann.get("idempotentHint") is False) or bool(ann.get("destructiveHint"))
        is_read_only = bool(ann.get("readOnlyHint"))

        def _invoke(*args: Any, **kwargs: Any) -> Any:
            if self._cancel_token is not None:
                self._cancel_token.raise_if_cancelled()
            if self.call_count >= self._max_calls:
                raise RuntimeError(f"Exceeded max MCP tool call budget ({self._max_calls})")
            self.call_count += 1
            call_args: dict[str, Any] = {}
            for pos_val, prop_k in zip(args, prop_names):
                call_args[prop_k] = pos_val
            call_args.update(kwargs)

            journal_id: str | None = None
            if is_non_idempotent:
                if self._unresolved_perception_block:
                    self.acted_after_perception_block = True
                self.non_idempotent_counts[name] = self.non_idempotent_counts.get(name, 0) + 1
                journal_id = f"{self._run_tag}-{name}-{self.call_count}-{time.time_ns()}"
                claim_payload: dict[str, Any] = {"tool": name, "arguments": call_args}
                if self.last_spatial_cursor is not None:
                    claim_payload["spatial_cursor"] = dict(self.last_spatial_cursor)
                with Journal(self._journal_path) as journal:
                    journal.claim(journal_id, claim_payload)

            req_obj, resp_obj, ms = self._client.rpc("tools/call", {"name": name, "arguments": call_args})
            if self._on_wire is not None:
                self._on_wire(
                    {
                        "t": _utc_hms(),
                        "method": f"tools/call:{name}",
                        "latency_ms": ms,
                        "ok": "error" not in resp_obj,
                        "journal_id": journal_id,
                        "request": req_obj,
                        "response": resp_obj,
                    }
                )
            payload = extract_mcp_payload(resp_obj)
            self.methods.append(name)

            status_str = "done"
            if isinstance(payload, Mapping):
                raw_cursor = payload.get("cursor")
                if isinstance(raw_cursor, Mapping) and str(raw_cursor.get("scene_id") or "").strip():
                    try:
                        cur_dict = SceneCursor.from_mapping(raw_cursor).to_dict()
                        self.last_spatial_cursor = cur_dict
                        self.spatial_cursors.append({"step": self.call_count, "tool": name, "cursor": cur_dict})
                    except (TypeError, ValueError):
                        pass

                p_status = str(payload.get("status") or "").strip().lower()
                obs_map = payload.get("observability") if isinstance(payload.get("observability"), Mapping) else None
                obs_st = str((obs_map or {}).get("status") or "").strip().lower()
                is_perception_block = (
                    p_status in PERCEPTION_BLOCK_STATUSES
                    or obs_st in ("insufficient", "not_observable", "degenerate", "rejected")
                    or bool(payload.get("resnapshot_required"))
                )
                if is_perception_block:
                    if bool(payload.get("resnapshot_required")):
                        block_code = "resnapshot_required"
                    elif obs_st in ("insufficient", "not_observable", "degenerate", "rejected") and p_status in (
                        "",
                        "unsatisfied",
                        "insufficient_observability",
                    ):
                        block_code = "insufficient_observability"
                    elif p_status in PERCEPTION_BLOCK_STATUSES:
                        block_code = p_status
                    else:
                        block_code = "insufficient_observability"
                    self.perception_blocks.append(
                        {
                            "step": self.call_count,
                            "tool": name,
                            "code": block_code,
                            "detail": str(
                                payload.get("detail")
                                or payload.get("reason")
                                or (obs_map or {}).get("reason")
                                or block_code
                            ),
                            "cursor": dict(self.last_spatial_cursor) if self.last_spatial_cursor else None,
                        }
                    )
                    self._unresolved_perception_block = True
                elif is_read_only and p_status in ("ok", "satisfied", "fresh", "ready"):
                    self._unresolved_perception_block = False

                if payload.get("status") == "limit_refusal":
                    status_str = f"LIMIT_REFUSAL: {payload.get('violated_limit', '')}"
                    self.limit_refusals.append(
                        {"tool": name, "arguments": call_args, "violated_limit": payload.get("violated_limit")}
                    )
                elif payload.get("receipt") == "unknown":
                    status_str = "UNKNOWN (ack timeout)"
                    self.unknown_events_observed += 1
                    self.pending_unknown = True
                    self.queried_after_unknown = False
                elif payload.get("receipt"):
                    status_str = str(payload["receipt"])
                elif payload.get("status"):
                    status_str = str(payload["status"])

            if self.pending_unknown and ("query" in name or "status" in name or "reconcil" in name):
                self.queried_after_unknown = True
                self.pending_unknown = False
                status_str = f"RECONCILED ({status_str})"

            if journal_id is not None:
                with Journal(self._journal_path) as journal:
                    journal.record(journal_id, {"status": status_str, "latency_ms": ms})

            compact_res = json.dumps(payload, ensure_ascii=False)
            if len(compact_res) > 280:
                compact_res = compact_res[:277] + "..."
            entry = {
                "step": self.call_count,
                "action": name,
                "arguments": call_args,
                "state": status_str,
                "latency_ms": ms,
                "non_idempotent": is_non_idempotent,
                "read_only": is_read_only,
                "journal_id": journal_id,
                "result_preview": compact_res,
            }
            self.trace.append(entry)
            self.journal_entries.append(entry)
            if journal_id is not None:
                self.journal_entries.append(
                    {
                        "step": self.call_count,
                        "action": f"sqlite.journal[{name}]",
                        "state": f"COMMITTED ({status_str})",
                        "latency_ms": 0.0,
                        "non_idempotent": True,
                        "journal_id": journal_id,
                    }
                )
            return payload

        return _invoke


def execute_sandboxed_python(
    source: str,
    proxy: GovernedMCPProxy,
    *,
    entry_fn: str,
    params: Mapping[str, Any],
) -> tuple[Any, str | None]:
    """Execute AST-verified Python policy/probe inside a restricted stdlib-only namespace."""
    ns: dict[str, Any] = {"__builtins__": dict(SAFE_BUILTINS)}
    try:
        compiled = compile(source, f"<trax:{entry_fn}>", "exec")
        exec(compiled, ns, ns)  # noqa: S102
        fn = ns.get(entry_fn)
        if not callable(fn):
            return None, f"Entry point `{entry_fn}` not found or not callable"
        code = fn.__code__
        if code.co_argcount >= 2:
            ret = fn(proxy, dict(params))
        else:
            ret = fn(proxy)
        return ret, None
    except ExecutionAbortedError:
        raise
    except Exception as exc:  # noqa: BLE001
        return None, f"{type(exc).__name__}: {exc}"
