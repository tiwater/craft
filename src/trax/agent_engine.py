"""Unified, embodiment-agnostic TRAX Closed-Loop Agent & Governance Engine over MCP.

Pure Python stdlib (zero external dependencies). Connects to ANY robot embodiment exposing
a standard MCP JSON-RPC 2.0 backend (`tools/list`, `tools/call`, `resources/list`, `resources/read`):
  1. Dynamic MCP Discovery (zero hardcoded robot verbs or domain assumptions)
  2. Multi-Turn LLM Active Exploration (LLM writes & runs diagnostic Python probes live via MCP)
  3. Modular Policy Compilation (LLM synthesizes deterministic `run_policy(mcp, params)`)
  4. Transactional Governed Execution (AST Sandbox + SQLite Intent Journal for non-idempotent MCP tools)
  5. 4-Layer Attribution & Selective Subroutine Diff Repair
"""
from __future__ import annotations

import ast
import difflib
import json
import math
from pathlib import Path
import threading
import time
from typing import Any, Callable, Mapping, Sequence

from .contracts import digest
from .journal import Journal
from .mcp_protocol import MCPClient, MCP_PROTOCOL_VERSION, format_mcp_tools_for_prompt
from .prior_work_bench import make_live_llm_fn
from .probing import analyze_policy_subroutines

LLMCallable = Callable[[str], dict[str, Any]]

_SAFE_BUILTINS: dict[str, Any] = {
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

_FORBIDDEN_CALLS = frozenset({"eval", "exec", "compile", "open", "__import__", "getattr", "setattr", "delattr", "globals", "locals", "vars", "input", "breakpoint"})


def _utc_hms() -> str:
    return time.strftime("%H:%M:%S", time.gmtime())


def _compute_unified_diff(prev_source: str, new_source: str, prev_label: str, new_label: str) -> str:
    if not prev_source:
        return ""
    return "\n".join(
        difflib.unified_diff(
            prev_source.splitlines(),
            new_source.splitlines(),
            fromfile=prev_label,
            tofile=new_label,
            lineterm="",
            n=3,
        )
    )


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
            if isinstance(node.func, ast.Name) and node.func.id in _FORBIDDEN_CALLS:
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


def _extract_mcp_payload(resp_obj: Mapping[str, Any]) -> Any:
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
        on_wire: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        self._client = client
        self._tools_by_name: dict[str, dict[str, Any]] = {str(t["name"]): dict(t) for t in tools if t.get("name")}
        self._journal_path = journal_path
        self._run_tag = run_tag
        self._max_calls = max_calls
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

    def __getattr__(self, name: str) -> Callable[..., Any]:
        if name.startswith("_") or name not in self._tools_by_name:
            raise AttributeError(f"MCP tool `{name}` is not exposed by the active robot backend")
        spec = self._tools_by_name[name]
        prop_names = list(((spec.get("inputSchema") or {}).get("properties") or {}).keys())
        ann = spec.get("annotations") or {}
        is_non_idempotent = (ann.get("idempotentHint") is False) or bool(ann.get("destructiveHint"))
        is_read_only = bool(ann.get("readOnlyHint"))

        def _invoke(*args: Any, **kwargs: Any) -> Any:
            if self.call_count >= self._max_calls:
                raise RuntimeError(f"Exceeded max MCP tool call budget ({self._max_calls})")
            self.call_count += 1
            call_args: dict[str, Any] = {}
            for pos_val, prop_k in zip(args, prop_names):
                call_args[prop_k] = pos_val
            call_args.update(kwargs)

            journal_id: str | None = None
            if is_non_idempotent:
                self.non_idempotent_counts[name] = self.non_idempotent_counts.get(name, 0) + 1
                journal_id = f"{self._run_tag}-{name}-{self.call_count}-{time.time_ns()}"
                with Journal(self._journal_path) as journal:
                    journal.claim(journal_id, {"tool": name, "arguments": call_args})

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
            payload = _extract_mcp_payload(resp_obj)
            self.methods.append(name)

            # Inspect receipt / safety status generically
            status_str = "done"
            if isinstance(payload, Mapping):
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


def _execute_sandboxed_python(
    source: str,
    proxy: GovernedMCPProxy,
    *,
    entry_fn: str,
    params: Mapping[str, Any],
) -> tuple[Any, str | None]:
    """Execute AST-verified Python policy/probe inside a restricted stdlib-only namespace."""
    ns: dict[str, Any] = {"__builtins__": dict(_SAFE_BUILTINS)}
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
    except Exception as exc:  # noqa: BLE001
        return None, f"{type(exc).__name__}: {exc}"


def classify_mcp_execution(
    *,
    ast_check: Mapping[str, Any],
    proxy: GovernedMCPProxy | None,
    verify_result: Mapping[str, Any] | None,
    runtime_error: str | None,
) -> dict[str, Any]:
    """Generic 4-layer attribution over any MCP embodiment execution."""
    if not ast_check.get("compliant"):
        return {
            "layer": "policy",
            "code": "ast_sandbox_violation",
            "recommended_owner": "trax_policy",
            "auto_repair_eligible": True,
            "summary": "; ".join(ast_check.get("violations") or ["AST violation"]),
        }
    if proxy is not None:
        dups = {k: v for k, v in proxy.non_idempotent_counts.items() if v > 1}
        if dups:
            return {
                "layer": "policy",
                "code": "duplicate_non_idempotent",
                "recommended_owner": "trax_policy",
                "auto_repair_eligible": True,
                "summary": f"Non-idempotent MCP tool invoked more than once: {dups}",
            }
        if proxy.unknown_events_observed > 0 and not proxy.queried_after_unknown:
            return {
                "layer": "policy",
                "code": "unreconciled_unknown_ack",
                "recommended_owner": "trax_policy",
                "auto_repair_eligible": True,
                "summary": "Observed `unknown` receipt on non-idempotent MCP tool without calling reconciliation/query tool.",
            }
        if proxy.limit_refusals:
            first = proxy.limit_refusals[0]
            return {
                "layer": "control",
                "code": "backend_limit_refusal",
                "recommended_owner": "trax_policy",
                "auto_repair_eligible": True,
                "summary": f"Backend safeguard refused `{first['tool']}`: {first['violated_limit']}",
            }
    if runtime_error:
        return {
            "layer": "policy",
            "code": "runtime_exception",
            "recommended_owner": "trax_policy",
            "auto_repair_eligible": True,
            "summary": runtime_error,
        }
    if verify_result and bool(verify_result.get("passed")):
        return {
            "layer": "none",
            "code": "passed",
            "recommended_owner": "none",
            "auto_repair_eligible": False,
            "summary": str(verify_result.get("detail") or "Passed independent task verification."),
        }
    detail = str((verify_result or {}).get("detail") or "Task verification criteria not yet satisfied.")
    return {
        "layer": "control",
        "code": "task_verification_failed",
        "recommended_owner": "trax_policy",
        "auto_repair_eligible": True,
        "summary": detail,
    }


DEFAULT_TASK_PROFILES: dict[str, dict[str, str]] = {
    "pick_and_place_zone": {
        "label": "Pick Red Object -> Place on Target Zone A",
        "instruction": (
            "Explore the SO-101 workspace and object pose, grasp `red_cylinder`, lift it cleanly "
            "above the tabletop, transport it to `target_zone_a`, release the gripper, and retract "
            "safely so `verify_task` passes."
        ),
    },
    "lift_red_object": {
        "label": "Grasp & Lift Red Object (>= 10 cm)",
        "instruction": (
            "Explore SO-101 reachability and gripper closure around `red_cylinder`, then grasp and "
            "lift `red_cylinder` at least 10 cm above the tabletop (z >= 1.13 m) without hitting "
            "workspace or joint limit refusals."
        ),
    },
    "place_and_activate_station": {
        "label": "Place on Station Pad + Non-Idempotent Activation (Reconcile)",
        "instruction": (
            "Pick `red_cylinder`, place it onto `station_pad` within 25 mm, release the gripper, "
            "and invoke the non-idempotent `activate_workstation` tool ONCE with a stable `command_id`. "
            "If `activate_workstation` returns receipt `unknown`, reconcile via `query_command_status` "
            "instead of re-triggering."
        ),
    },
}


class TraxAgentEngine:
    """Unified MCP-native closed-loop Explore -> Compile -> Governed Exec -> Iterate engine."""

    def __init__(
        self,
        workspace_dir: Path,
        resolve_mcp_client: Callable[[], tuple[MCPClient, str]],
        *,
        llm_fn: LLMCallable | None = None,
        on_wire: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        self.workspace_dir = Path(workspace_dir)
        self.workspace_dir.mkdir(parents=True, exist_ok=True)
        self._resolve_mcp_client = resolve_mcp_client
        self._llm_fn = llm_fn
        self._on_wire = on_wire
        self._state_lock = threading.RLock()
        self._exec_lock = threading.RLock()
        self._journal_path = self.workspace_dir / "trax_agent_journal.sqlite"

        self._phase: str = "idle"
        self._busy: bool = False
        self._busy_detail: str = ""
        self._task_id: str = "pick_and_place_zone"
        self._task_instruction: str = DEFAULT_TASK_PROFILES["pick_and_place_zone"]["instruction"]
        self._explore_rounds: int = 2
        self._seed: int = 0

        self._mcp_endpoint: str = ""
        self._server_info: dict[str, Any] | None = None
        self._discovered_tools: list[dict[str, Any]] = []
        self._discovered_resources: list[dict[str, Any]] = []
        self._exploration_findings: list[dict[str, Any]] = []
        self._iterations: list[dict[str, Any]] = []
        self._active_iter_idx: int = -1
        self._llm_calls: list[dict[str, Any]] = []
        self._pending_llm_call: dict[str, Any] | None = None
        self._executed_runs: list[dict[str, Any]] = []
        self._event_log: list[dict[str, str]] = []
        self._last_verify: dict[str, Any] | None = None

        self._log("INIT", "TRAX Unified MCP Agent Engine ready. Click 1. Agent Explore (MCP+LLM) or Run Closed Loop (1->4).")

    def _log(self, tag: str, message: str) -> None:
        self._event_log.append({"t": _utc_hms(), "tag": tag, "msg": message})
        if len(self._event_log) > 150:
            self._event_log = self._event_log[-150:]

    def _get_llm(self) -> LLMCallable:
        if self._llm_fn is not None:
            return self._llm_fn
        return make_live_llm_fn(
            provider="codex_cli",
            model="gpt-6-astra",
            reasoning_effort="medium",
            timeout_s=150.0,
            max_retries=1,
        )

    def _discover_mcp_sync(self) -> tuple[MCPClient, str, dict[str, Any], list[dict[str, Any]], list[dict[str, Any]]]:
        client, endpoint_label = self._resolve_mcp_client()
        init_req, init_resp, init_ms = client.rpc(
            "initialize",
            {
                "protocolVersion": MCP_PROTOCOL_VERSION,
                "capabilities": {},
                "clientInfo": {"name": "trax-agent-engine", "version": "0.2.0"},
            },
        )
        tools_req, tools_resp, tools_ms = client.rpc("tools/list", {})
        res_req, res_resp, res_ms = client.rpc("resources/list", {})
        if self._on_wire is not None:
            for m, rq, rs, ms in (
                ("initialize", init_req, init_resp, init_ms),
                ("tools/list", tools_req, tools_resp, tools_ms),
                ("resources/list", res_req, res_resp, res_ms),
            ):
                self._on_wire({"t": _utc_hms(), "method": m, "latency_ms": ms, "ok": "error" not in rs, "request": rq, "response": rs})

        if "error" in init_resp:
            raise RuntimeError(f"MCP initialize failed: {init_resp['error']}")
        if "error" in tools_resp:
            raise RuntimeError(f"MCP tools/list failed: {tools_resp['error']}")

        info = (init_resp.get("result") or {}).get("serverInfo") or {}
        tools = list((tools_resp.get("result") or {}).get("tools") or [])
        resources = list((res_resp.get("result") or {}).get("resources") or [])

        with self._state_lock:
            self._mcp_endpoint = endpoint_label
            self._server_info = info
            self._discovered_tools = tools
            self._discovered_resources = resources
        return client, endpoint_label, info, tools, resources

    def _read_initial_observation(self, client: MCPClient, tools: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
        names = {str(t.get("name")) for t in tools}
        obs: dict[str, Any] = {}
        for fn in ("get_robot_state", "get_scene_objects", "observe_scene"):
            if fn in names:
                try:
                    _, resp, _ = client.rpc("tools/call", {"name": fn, "arguments": {}})
                    obs[fn] = _extract_mcp_payload(resp)
                except Exception:  # noqa: BLE001
                    pass
        return obs

    def _reset_backend_if_supported(self, client: MCPClient, tools: Sequence[Mapping[str, Any]], task_id: str, seed: int) -> None:
        names = {str(t.get("name")) for t in tools}
        if "reset_task" in names:
            client.rpc("tools/call", {"name": "reset_task", "arguments": {"task_id": task_id, "seed": seed}})
        elif "reset_scenario" in names:
            client.rpc("tools/call", {"name": "reset_scenario", "arguments": {"preset": "default"}})

    def _invoke_llm_tracked(self, *, origin: str, prompt: str, iter_index: int | None) -> tuple[dict[str, Any], str]:
        with self._state_lock:
            started = _utc_hms()
            cid = len(self._llm_calls) + 1
            self._pending_llm_call = {
                "call_id": cid,
                "iter_index": iter_index,
                "origin": origin,
                "provider": "codex_cli" if self._llm_fn is None else "custom_llm",
                "model": "gpt-6-astra",
                "reasoning_effort": "medium",
                "status": "running",
                "started_utc": started,
                "prompt": prompt,
                "prompt_chars": len(prompt),
                "raw_response": None,
                "response_chars": 0,
            }
            self._log("LLM", f"Call #{cid} ({origin}, prompt={len(prompt)} chars) dispatched to LLM...")

        try:
            res = self._get_llm()(prompt)
        finally:
            with self._state_lock:
                self._pending_llm_call = None

        source = str(res.get("source") or "").strip()
        raw_resp = str(res.get("raw_output") if res.get("raw_output") is not None else source)
        with self._state_lock:
            rec = {
                "call_id": len(self._llm_calls) + 1,
                "iter_index": iter_index,
                "origin": origin,
                "provider": "codex_cli" if self._llm_fn is None else "custom_llm",
                "model": str(res.get("model", "gpt-6-astra")),
                "reasoning_effort": str(res.get("reasoning_effort", "medium")),
                "status": "completed",
                "started_utc": started,
                "completed_utc": _utc_hms(),
                "wall_s": res.get("wall_s", 0.0),
                "prompt": prompt,
                "prompt_chars": len(prompt),
                "raw_response": raw_resp,
                "response_chars": len(raw_resp),
                "extracted_source": source,
                "extracted_sha256": digest(source),
            }
            self._llm_calls.append(rec)
            self._log("LLM", f"Call #{rec['call_id']} ({origin}) returned {rec['response_chars']} chars in {rec['wall_s']}s.")
        return rec, source

    def _build_explore_prompt(
        self,
        *,
        round_idx: int,
        total_rounds: int,
        tools: Sequence[Mapping[str, Any]],
        obs: Mapping[str, Any],
        prior_findings: Sequence[Mapping[str, Any]],
    ) -> str:
        tool_doc = format_mcp_tools_for_prompt(tools)
        prior_txt = json.dumps(list(prior_findings), indent=2, ensure_ascii=False) if prior_findings else "(none yet)"
        focus = (
            "Round 1 focus: probe workspace reachability, hover above the target object, descend to grasp height, "
            "close the gripper to test physical grasp capture, and lift briefly to measure actual lift height and tracking error."
            if round_idx == 1
            else (
                "Round 2+ focus: probe the downstream stage (transport to target zone/pad, release settling, and if the task "
                "involves a non-idempotent tool like `activate_workstation`, test calling it ONCE with a stable `command_id` "
                "and reconciling via `query_command_status` on `unknown` receipt) while respecting workspace bounds."
            )
        )
        return "\n".join(
            [
                f"You are TRAX's Active Physical Exploration Agent (Exploration Round {round_idx}/{total_rounds}).",
                "Your goal is to write a short, concrete diagnostic Python function `def run_probe(mcp):` that actively",
                "interacts with the robot through its dynamically discovered MCP tools to gather real physical measurements",
                "(poses, grasp capture z-height, lift delta, zone alignment, limit bounds, or non-idempotent receipts).",
                "",
                tool_doc,
                "",
                f"TARGET TASK ({self._task_id}):\n{self._task_instruction}",
                "",
                f"LIVE OBSERVATION BEFORE ROUND {round_idx}:\n{json.dumps(obs, indent=2, ensure_ascii=False)}",
                "",
                f"PRIOR EXPLORATION FINDINGS:\n{prior_txt}",
                "",
                focus,
                "",
                "STRICT AST SANDBOX RULES:",
                "- Define `def run_probe(mcp):` returning a dict summarizing what you measured.",
                "- Call ONLY tools listed in `tools/list` above via `mcp.<tool_name>(...)`.",
                "- NO imports, NO leading-underscore names/attributes, NO lambda, NO exec/eval/open.",
                "- Return ONLY the Python code inside a ```python block.",
            ]
        )

    def _run_explore_sync(self, *, rounds: int, task_id: str, seed: int) -> list[dict[str, Any]]:
        with self._exec_lock:
            client, ep_label, info, tools, resources = self._discover_mcp_sync()
            tool_names = [str(t["name"]) for t in tools]
            self._log(
                "MCP",
                f"Discovered {len(tools)} MCP tools & {len(resources)} resources on {info.get('name', ep_label)} ({ep_label}).",
            )
            self._reset_backend_if_supported(client, tools, task_id, seed)
            findings: list[dict[str, Any]] = []

            for r in range(1, max(1, min(3, rounds)) + 1):
                with self._state_lock:
                    self._phase = "exploring"
                    self._busy_detail = f"Stage 1 Explore Round {r}/{rounds}: LLM writing diagnostic MCP probe..."
                obs = self._read_initial_observation(client, tools)
                prompt = self._build_explore_prompt(
                    round_idx=r,
                    total_rounds=rounds,
                    tools=tools,
                    obs=obs,
                    prior_findings=findings,
                )
                llm_rec, probe_src = self._invoke_llm_tracked(
                    origin=f"explore_probe_r{r}",
                    prompt=prompt,
                    iter_index=None,
                )
                ast_res = check_mcp_policy_ast(probe_src, allowed_tools=tool_names, entry_fn="run_probe")
                with self._state_lock:
                    self._busy_detail = f"Stage 1 Explore Round {r}/{rounds}: Executing probe live on robot via MCP..."

                t0 = time.monotonic()
                proxy = GovernedMCPProxy(
                    client,
                    tools,
                    journal_path=self._journal_path,
                    run_tag=f"probe-r{r}",
                    max_calls=35,
                    on_wire=self._on_wire,
                )
                ret_val: Any = None
                err: str | None = None
                if ast_res["compliant"]:
                    ret_val, err = _execute_sandboxed_python(
                        probe_src,
                        proxy,
                        entry_fn="run_probe",
                        params={"task_id": task_id, "round": r},
                    )
                else:
                    err = "; ".join(ast_res["violations"])
                wall_s = round(time.monotonic() - t0, 3)

                obs_after = self._read_initial_observation(client, tools)
                finding = {
                    "round": r,
                    "llm_call_id": llm_rec["call_id"],
                    "probe_sha256": digest(probe_src)[:12],
                    "ast_compliant": ast_res["compliant"],
                    "wall_s": wall_s,
                    "methods_called": list(proxy.methods),
                    "limit_refusals": list(proxy.limit_refusals),
                    "unknown_events": proxy.unknown_events_observed,
                    "queried_after_unknown": proxy.queried_after_unknown,
                    "return_value": ret_val,
                    "error": err,
                    "post_probe_robot": obs_after.get("get_robot_state"),
                    "post_probe_objects": (obs_after.get("get_scene_objects") or {}).get("objects"),
                }
                findings.append(finding)

                with self._state_lock:
                    self._exploration_findings = list(findings)
                    self._executed_runs.append(
                        {
                            "run_index": len(self._executed_runs) + 1,
                            "stage": f"01_explore_r{r}",
                            "label": f"Explore Probe R{r} (LLM+MCP)",
                            "iter_index": None,
                            "command_id": f"probe-r{r}-{llm_rec['call_id']}",
                            "scenario_key": task_id,
                            "source": probe_src,
                            "source_sha256": digest(probe_src),
                            "ast_check": ast_res,
                            "methods": list(proxy.methods),
                            "status": "error" if err else "probed",
                            "task_passed": err is None,
                            "wall_s": wall_s,
                            "executed_utc": _utc_hms(),
                        }
                    )
                    self._log(
                        "PROBE",
                        f"Explore R{r}/{rounds} finished in {wall_s}s: {len(proxy.methods)} MCP calls "
                        f"({', '.join(proxy.methods[:6])}{'...' if len(proxy.methods) > 6 else ''})"
                        + (f" | err={err}" if err else f" | refusals={len(proxy.limit_refusals)}"),
                    )
                # Reset cleanly between exploration rounds so next probe starts from known state
                self._reset_backend_if_supported(client, tools, task_id, seed)

            with self._state_lock:
                self._phase = "explored"
            return findings

    def _build_compile_prompt(
        self,
        *,
        tools: Sequence[Mapping[str, Any]],
        obs: Mapping[str, Any],
        prev_iter: Mapping[str, Any] | None,
        negative_history: Sequence[str],
    ) -> tuple[str, str]:
        tool_doc = format_mcp_tools_for_prompt(tools)
        findings_txt = (
            json.dumps(self._exploration_findings, indent=2, ensure_ascii=False)
            if self._exploration_findings
            else "(no exploration rounds run yet)"
        )
        if prev_iter is None or not prev_iter.get("execution"):
            origin = "llm_compile"
            prompt = "\n".join(
                [
                    "You are TRAX's Policy Compilation Engine.",
                    "Compile the empirical exploration findings below into a clean, modular, deterministic Python",
                    "policy `def run_policy(mcp, params):` that solves the task reliably via the discovered MCP tools.",
                    "",
                    tool_doc,
                    "",
                    f"TASK ({self._task_id}):\n{self._task_instruction}",
                    "",
                    f"CURRENT OBSERVATION:\n{json.dumps(obs, indent=2, ensure_ascii=False)}",
                    "",
                    f"STAGE-1 EMPIRICAL EXPLORATION FINDINGS:\n{findings_txt}",
                    "",
                    "REQUIREMENTS:",
                    "- Define `def run_policy(mcp, params):` returning a dict `{'status': 'completed', ...}`.",
                    "- Read live object/zone poses dynamically via MCP (`get_scene_objects`, `get_robot_state`) — never hardcode object positions.",
                    "- Respect workspace bounds returned by `get_robot_state()` to avoid `limit_refusal`.",
                    "- For ANY non-idempotent tool (`idempotentHint: false` / `destructiveHint: true`), pass a stable `command_id` and call it AT MOST ONCE; if receipt is `unknown`, call `query_command_status(command_id)` to reconcile instead of re-calling.",
                    "- Strict AST sandbox: NO imports, NO leading-underscore identifiers, NO lambda, NO exec/eval.",
                    "- Return ONLY the complete Python code inside a ```python block.",
                ]
            )
            return origin, prompt

        origin = "llm_repair"
        ex = prev_iter["execution"]
        attr = prev_iter.get("attribution") or {}
        sub = prev_iter.get("subroutines") or {}
        neg_txt = "\n".join(f"  - {x}" for x in negative_history[-4:]) if negative_history else "  (none)"
        prompt = "\n".join(
            [
                "You are TRAX's Offline Policy Repair Engine. Fix ONLY the failing stage in the policy below.",
                "",
                tool_doc,
                "",
                f"TASK ({self._task_id}):\n{self._task_instruction}",
                "",
                f"STAGE-1 EXPLORATION FINDINGS:\n{findings_txt}",
                "",
                "PREVIOUS ITERATION FAILURE DIAGNOSTICS:",
                f"- 4-Layer Attribution: layer={attr.get('layer')} code={attr.get('code')} ({attr.get('summary')})",
                f"- Independent Verifier: {json.dumps(ex.get('verify_result') or {}, ensure_ascii=False)}",
                f"- Backend Limit Refusals: {json.dumps(ex.get('limit_refusals') or [], ensure_ascii=False)}",
                f"- Non-idempotent calls: {json.dumps(ex.get('non_idempotent_counts') or {})} | unknown={ex.get('unknown_events_observed')} | reconciled={ex.get('queried_after_unknown')}",
                f"- MCP Trace (last 12 calls): {json.dumps((ex.get('mcp_trace') or [])[-12:], ensure_ascii=False)}",
                f"- AST Subroutine Analysis: {json.dumps(sub, ensure_ascii=False)}",
                f"- Prior Failed Iterations:\n{neg_txt}",
                "",
                "PREVIOUS PYTHON POLICY SOURCE:",
                "```python",
                str(prev_iter.get("source") or ""),
                "```",
                "",
                "Return ONLY the complete repaired Python policy defining `def run_policy(mcp, params):` inside a ```python block.",
            ]
        )
        return origin, prompt

    def _synthesize_sync(self, *, force_fresh: bool = False) -> dict[str, Any]:
        with self._exec_lock:
            client, _, _, tools, _ = self._discover_mcp_sync()
            tool_names = [str(t["name"]) for t in tools]
            obs = self._read_initial_observation(client, tools)

        with self._state_lock:
            prev_iter = None
            if not force_fresh and 0 <= self._active_iter_idx < len(self._iterations):
                cand = self._iterations[self._active_iter_idx]
                if cand.get("execution") and not cand["execution"].get("task_passed"):
                    prev_iter = cand
            neg_hist = [
                f"iter_{it['iter_index']}: {(it.get('attribution') or {}).get('layer')}/{(it.get('attribution') or {}).get('code')}"
                for it in self._iterations
                if it.get("execution") and not it["execution"].get("task_passed")
            ]
            origin, prompt = self._build_compile_prompt(
                tools=tools,
                obs=obs,
                prev_iter=prev_iter,
                negative_history=neg_hist,
            )
            next_idx = len(self._iterations)

        llm_rec, source = self._invoke_llm_tracked(origin=origin, prompt=prompt, iter_index=next_idx)
        ast_res = check_mcp_policy_ast(source, allowed_tools=tool_names, entry_fn="run_policy")
        sub = analyze_policy_subroutines(source, failure_codes=())

        with self._state_lock:
            idx = len(self._iterations)
            prev_src = str(self._iterations[self._active_iter_idx]["source"]) if 0 <= self._active_iter_idx < len(self._iterations) else ""
            prev_lbl = f"iter_{self._active_iter_idx}" if prev_src else ""
            diff_txt = _compute_unified_diff(prev_src, source, prev_lbl, f"iter_{idx} ({origin})")
            rec = {
                "iter_index": idx,
                "label": f"iter_{idx} ({origin})",
                "origin": origin,
                "policy_id": f"trax_iter_{idx}",
                "source": source,
                "source_sha256": digest(source),
                "unified_diff": diff_txt,
                "ast_check": ast_res,
                "subroutines": sub,
                "llm_meta": llm_rec,
                "execution": None,
                "attribution": None,
                "journal_entries": [],
                "created_utc": _utc_hms(),
            }
            self._iterations.append(rec)
            self._active_iter_idx = idx
            self._phase = "synthesized"
            ast_tag = "COMPLIANT" if ast_res["compliant"] else f"BLOCKED ({len(ast_res['violations'])} violations)"
            self._log("AST", f"iter_{idx} ({origin}) -> AST {ast_tag} | tools={ast_res['called_tools']}.")
            return rec

    def _execute_sync(self, *, edited_source: str | None = None, task_id: str | None = None) -> dict[str, Any]:
        with self._exec_lock:
            client, _, info, tools, _ = self._discover_mcp_sync()
            tool_names = [str(t["name"]) for t in tools]

            with self._state_lock:
                if task_id and task_id in DEFAULT_TASK_PROFILES:
                    self._task_id = task_id
                cur_idx = self._active_iter_idx
                if edited_source is not None and edited_source.strip():
                    cleaned = edited_source.strip()
                    existing_src = self._iterations[cur_idx]["source"].strip() if 0 <= cur_idx < len(self._iterations) else ""
                    if cleaned != existing_src:
                        idx = len(self._iterations)
                        ast_res = check_mcp_policy_ast(cleaned, allowed_tools=tool_names, entry_fn="run_policy")
                        sub = analyze_policy_subroutines(cleaned, failure_codes=())
                        diff_txt = _compute_unified_diff(existing_src, cleaned, f"iter_{cur_idx}", f"iter_{idx} (manual_edit)")
                        self._iterations.append(
                            {
                                "iter_index": idx,
                                "label": f"iter_{idx} (manual_edit)",
                                "origin": "manual_edit",
                                "policy_id": f"trax_iter_{idx}",
                                "source": cleaned,
                                "source_sha256": digest(cleaned),
                                "unified_diff": diff_txt,
                                "ast_check": ast_res,
                                "subroutines": sub,
                                "llm_meta": None,
                                "execution": None,
                                "attribution": None,
                                "journal_entries": [],
                                "created_utc": _utc_hms(),
                            }
                        )
                        self._active_iter_idx = idx
                        cur_idx = idx
                        self._log("EDIT", f"Saved manual Python edit as iter_{idx}.")

                if not (0 <= cur_idx < len(self._iterations)):
                    raise ValueError("No Python policy iteration exists yet. Click 2. Compile Policy (LLM) or Run Closed Loop.")
                target_iter = self._iterations[cur_idx]
                source = str(target_iter["source"])

            ast_res = check_mcp_policy_ast(source, allowed_tools=tool_names, entry_fn="run_policy")
            cmd_id = f"trax-exec-it{cur_idx}-{time.time_ns()}"
            t0 = time.monotonic()

            if not ast_res["compliant"]:
                attr = classify_mcp_execution(ast_check=ast_res, proxy=None, verify_result=None, runtime_error=None)
                exec_res = {
                    "task_passed": False,
                    "host_runtime_status": "ast_preflight_rejected",
                    "wall_s": 0.0,
                    "methods": [],
                    "mcp_trace": [],
                    "limit_refusals": [],
                    "non_idempotent_counts": {},
                    "duplicate_non_idempotent": False,
                    "unknown_events_observed": 0,
                    "queried_after_unknown": True,
                    "verify_result": {"passed": False, "detail": attr["summary"], "metrics": {}},
                    "evidence_source": info.get("evidence_source", "simulator"),
                }
                journal_entries: list[dict[str, Any]] = []
            else:
                self._reset_backend_if_supported(client, tools, self._task_id, self._seed)
                proxy = GovernedMCPProxy(
                    client,
                    tools,
                    journal_path=self._journal_path,
                    run_tag=f"it{cur_idx}",
                    max_calls=60,
                    on_wire=self._on_wire,
                )
                with self._state_lock:
                    self._log("EXEC", f"Running iter_{cur_idx} inside AST sandbox + SQLite MCP governance (task={self._task_id})...")
                ret_val, runtime_err = _execute_sandboxed_python(
                    source,
                    proxy,
                    entry_fn="run_policy",
                    params={"task_id": self._task_id, "command_prefix": f"it{cur_idx}"},
                )
                verify_res: dict[str, Any] = {"passed": False, "detail": "No verifier exposed", "metrics": {}}
                if "verify_task" in tool_names:
                    _, v_resp, _ = client.rpc("tools/call", {"name": "verify_task", "arguments": {"task_id": self._task_id}})
                    verify_res = dict(_extract_mcp_payload(v_resp) or verify_res)
                elif runtime_err is None and isinstance(ret_val, Mapping) and ret_val.get("status") in ("completed", "done"):
                    verify_res = {"passed": True, "detail": "Policy completed cleanly", "metrics": dict(ret_val)}

                wall_s = round(time.monotonic() - t0, 3)
                attr = classify_mcp_execution(
                    ast_check=ast_res,
                    proxy=proxy,
                    verify_result=verify_res,
                    runtime_error=runtime_err,
                )
                dups = any(v > 1 for v in proxy.non_idempotent_counts.values())
                passed = bool(verify_res.get("passed")) and (attr["layer"] == "none")
                exec_res = {
                    "task_passed": passed,
                    "host_runtime_status": "completed" if passed else attr["code"],
                    "wall_s": wall_s,
                    "methods": list(proxy.methods),
                    "mcp_trace": list(proxy.trace),
                    "limit_refusals": list(proxy.limit_refusals),
                    "non_idempotent_counts": dict(proxy.non_idempotent_counts),
                    "duplicate_non_idempotent": dups,
                    "unknown_events_observed": proxy.unknown_events_observed,
                    "queried_after_unknown": proxy.queried_after_unknown,
                    "return_value": ret_val,
                    "runtime_error": runtime_err,
                    "verify_result": verify_res,
                    "evidence_source": info.get("evidence_source", "simulator"),
                    "executed_command_id": cmd_id,
                    "executed_sha256": digest(source),
                }
                journal_entries = list(proxy.journal_entries)

            failure_codes = () if exec_res["task_passed"] else (str(attr.get("code") or ""),)
            sub = analyze_policy_subroutines(source, failure_codes=failure_codes)

            with self._state_lock:
                target_iter = self._iterations[cur_idx]
                target_iter["ast_check"] = ast_res
                target_iter["execution"] = exec_res
                target_iter["attribution"] = attr
                target_iter["journal_entries"] = journal_entries
                target_iter["subroutines"] = sub
                self._last_verify = exec_res.get("verify_result")
                self._executed_runs.append(
                    {
                        "run_index": len(self._executed_runs) + 1,
                        "stage": "03_policy_exec",
                        "label": f"iter_{cur_idx} ({target_iter.get('origin')})",
                        "iter_index": cur_idx,
                        "command_id": cmd_id,
                        "scenario_key": self._task_id,
                        "source": source,
                        "source_sha256": digest(source),
                        "ast_check": ast_res,
                        "methods": list(exec_res.get("methods") or []),
                        "status": str(exec_res.get("host_runtime_status") or ""),
                        "task_passed": bool(exec_res.get("task_passed")),
                        "wall_s": exec_res.get("wall_s", 0.0),
                        "executed_utc": _utc_hms(),
                    }
                )
                self._phase = "verified" if exec_res["task_passed"] else "attributed"
                verdict = "PASS" if exec_res["task_passed"] else f"FAIL ({attr['layer']}/{attr['code']})"
                self._log(
                    "EXEC",
                    f"iter_{cur_idx} -> {verdict} | {exec_res['wall_s']}s | {len(exec_res['methods'])} MCP calls | "
                    f"{(exec_res.get('verify_result') or {}).get('detail', '')}",
                )
                return target_iter

    # --- Public Async & State APIs ---

    def set_backend_changed(self, label: str) -> None:
        with self._state_lock:
            if self._busy:
                raise RuntimeError("Cannot switch embodiment while agent loop is running.")
            self._phase = "idle"
            self._server_info = None
            self._discovered_tools = []
            self._discovered_resources = []
            self._exploration_findings = []
            self._log("HOST", f"Switched active MCP backend -> {label}. Ready for dynamic MCP discovery & exploration.")

    def select_iteration(self, index: int) -> dict[str, Any]:
        with self._state_lock:
            if 0 <= index < len(self._iterations):
                self._active_iter_idx = index
        return self.get_state()

    def reset_session(
        self,
        *,
        task_instruction: str | None = None,
        task_id: str | None = None,
        explore_rounds: int | None = None,
    ) -> dict[str, Any]:
        with self._state_lock:
            if self._busy:
                return {"status": "busy", "detail": self._busy_detail}
            if task_id and task_id in DEFAULT_TASK_PROFILES:
                self._task_id = task_id
                if not task_instruction:
                    self._task_instruction = DEFAULT_TASK_PROFILES[task_id]["instruction"]
            if task_instruction is not None and task_instruction.strip():
                self._task_instruction = task_instruction.strip()
            if explore_rounds is not None:
                self._explore_rounds = max(1, min(3, int(explore_rounds)))
            self._phase = "idle"
            self._exploration_findings = []
            self._iterations = []
            self._active_iter_idx = -1
            self._llm_calls = []
            self._pending_llm_call = None
            self._executed_runs = []
            self._last_verify = None
            self._event_log = []
            self._log("RESET", f"Fresh MCP Agent session ready (task={self._task_id}, explore_rounds={self._explore_rounds}).")
        return self.get_state()

    def start_explore_async(self, body: Mapping[str, Any]) -> dict[str, Any]:
        with self._state_lock:
            if self._busy:
                return {"status": "busy", "detail": self._busy_detail}
            tid = str(body.get("task_id") or body.get("scenario_key") or self._task_id)
            if tid in DEFAULT_TASK_PROFILES:
                self._task_id = tid
            if body.get("task_instruction"):
                self._task_instruction = str(body["task_instruction"]).strip()
            rounds = max(1, min(3, int(body.get("explore_rounds") or self._explore_rounds)))
            self._explore_rounds = rounds
            self._busy = True
            self._phase = "exploring"
            self._busy_detail = f"Stage 1: Discovering MCP tools & running {rounds} LLM exploration round(s)..."

        def _worker() -> None:
            try:
                self._run_explore_sync(rounds=rounds, task_id=self._task_id, seed=self._seed)
            except Exception as exc:  # noqa: BLE001
                with self._state_lock:
                    self._log("ERROR", f"Exploration failed: {exc}")
                    self._phase = "idle"
            finally:
                with self._state_lock:
                    self._busy = False
                    self._busy_detail = ""

        threading.Thread(target=_worker, daemon=True).start()
        return {"status": "started", "phase": "exploring", "explore_rounds": rounds}

    def start_synthesize_async(self, body: Mapping[str, Any]) -> dict[str, Any]:
        with self._state_lock:
            if self._busy:
                return {"status": "busy", "detail": self._busy_detail}
            tid = str(body.get("task_id") or body.get("scenario_key") or self._task_id)
            if tid in DEFAULT_TASK_PROFILES:
                self._task_id = tid
            if body.get("task_instruction"):
                self._task_instruction = str(body["task_instruction"]).strip()
            force_fresh = bool(body.get("force_fresh", False))
            self._busy = True
            self._phase = "synthesizing"
            self._busy_detail = "Stage 2: Compiling exploration findings into modular Python policy..."

        def _worker() -> None:
            try:
                self._synthesize_sync(force_fresh=force_fresh)
            except Exception as exc:  # noqa: BLE001
                with self._state_lock:
                    self._log("ERROR", f"Policy compilation failed: {exc}")
                    self._phase = "idle"
            finally:
                with self._state_lock:
                    self._busy = False
                    self._busy_detail = ""

        threading.Thread(target=_worker, daemon=True).start()
        return {"status": "started", "phase": "synthesizing"}

    def start_execute_async(self, body: Mapping[str, Any]) -> dict[str, Any]:
        with self._state_lock:
            if self._busy:
                return {"status": "busy", "detail": self._busy_detail}
            tid = str(body.get("task_id") or body.get("scenario_key") or self._task_id)
            edited = body.get("source")
            self._busy = True
            self._phase = "executing"
            self._busy_detail = "Stage 3: Executing Python policy via governed MCP proxy + SQLite Journal..."

        def _worker() -> None:
            try:
                self._execute_sync(edited_source=str(edited) if edited is not None else None, task_id=tid)
            except Exception as exc:  # noqa: BLE001
                with self._state_lock:
                    self._log("ERROR", f"Governed execution failed: {exc}")
                    self._phase = "idle"
            finally:
                with self._state_lock:
                    self._busy = False
                    self._busy_detail = ""

        threading.Thread(target=_worker, daemon=True).start()
        return {"status": "started", "phase": "executing"}

    def start_closed_loop_async(self, body: Mapping[str, Any]) -> dict[str, Any]:
        with self._state_lock:
            if self._busy:
                return {"status": "busy", "detail": self._busy_detail}
            tid = str(body.get("task_id") or body.get("scenario_key") or self._task_id)
            if tid in DEFAULT_TASK_PROFILES:
                self._task_id = tid
            if body.get("task_instruction"):
                self._task_instruction = str(body["task_instruction"]).strip()
            rounds = max(1, min(3, int(body.get("explore_rounds") or self._explore_rounds)))
            max_iters = max(1, min(4, int(body.get("max_iters", 3))))
            self._explore_rounds = rounds
            if bool(body.get("fresh", True)):
                self._exploration_findings = []
                self._iterations = []
                self._active_iter_idx = -1
                self._llm_calls = []
                self._pending_llm_call = None
                self._executed_runs = []
                self._last_verify = None
            self._busy = True
            self._phase = "exploring"
            self._busy_detail = f"Closed Loop: Stage 1 Exploring ({rounds} rounds)..."
            self._log("LOOP", f"=== Starting MCP Closed Loop (task={self._task_id}, explore_rounds={rounds}, max_iters={max_iters}) ===")

        def _worker() -> None:
            try:
                self._run_explore_sync(rounds=rounds, task_id=self._task_id, seed=self._seed)
                for step in range(1, max_iters + 1):
                    with self._state_lock:
                        self._phase = "synthesizing" if step == 1 else "iterating"
                        self._busy_detail = f"Iteration {step}/{max_iters}: LLM compiling/repairing Python policy..."
                    self._synthesize_sync(force_fresh=(step == 1))

                    with self._state_lock:
                        self._phase = "executing"
                        self._busy_detail = f"Iteration {step}/{max_iters}: Executing Python via MCP + SQLite Journal..."
                    it_rec = self._execute_sync(task_id=self._task_id)
                    if (it_rec.get("execution") or {}).get("task_passed"):
                        with self._state_lock:
                            self._phase = "verified"
                            self._log("LOOP", f"=== Converged at iter_{it_rec['iter_index']} (PASS) ===")
                        return
                    attr = it_rec.get("attribution") or {}
                    with self._state_lock:
                        self._log(
                            "ITER",
                            f"iter_{it_rec['iter_index']} failed ({attr.get('layer')}/{attr.get('code')}); feeding diagnostics into next repair iteration...",
                        )
                with self._state_lock:
                    self._log("LOOP", f"=== Reached max_iters={max_iters} budget ===")
            except Exception as exc:  # noqa: BLE001
                with self._state_lock:
                    self._log("ERROR", f"Closed loop aborted: {exc}")
                    self._phase = "idle"
            finally:
                with self._state_lock:
                    self._busy = False
                    self._busy_detail = ""

        threading.Thread(target=_worker, daemon=True).start()
        return {"status": "started", "phase": "exploring", "explore_rounds": rounds, "max_iters": max_iters}

    def get_state(self) -> dict[str, Any]:
        with self._state_lock:
            return {
                "engine": "trax_mcp_agent_v1",
                "phase": self._phase,
                "busy": self._busy,
                "busy_detail": self._busy_detail,
                "task_id": self._task_id,
                "scenario_key": self._task_id,
                "task_instruction": self._task_instruction,
                "task_profiles": {k: v["label"] for k, v in DEFAULT_TASK_PROFILES.items()},
                "task_instructions": {k: v["instruction"] for k, v in DEFAULT_TASK_PROFILES.items()},
                "explore_rounds": self._explore_rounds,
                "mcp_endpoint": self._mcp_endpoint,
                "server_info": self._server_info,
                "discovered_tools_count": len(self._discovered_tools),
                "discovered_tools": [t.get("name") for t in self._discovered_tools],
                "discovered_resources_count": len(self._discovered_resources),
                "exploration_findings": list(self._exploration_findings),
                "iterations": list(self._iterations),
                "active_iter_index": self._active_iter_idx,
                "llm_calls": list(self._llm_calls),
                "pending_llm_call": dict(self._pending_llm_call) if self._pending_llm_call else None,
                "executed_runs": list(self._executed_runs),
                "last_verify": self._last_verify,
                "event_log": list(self._event_log),
            }
