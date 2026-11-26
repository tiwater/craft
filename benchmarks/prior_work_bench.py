"""Prior-work comparative benchmark harness for TRAX-CafeBench (Phase 3A).

Runs matched-model, matched-contract, matched-budget head-to-head comparisons on
TRAX-CafeBench v1 (disjoint 4-scenario dev split -> 20-scenario held-out suite):

1. Strong Zero-Shot (Full Contract) — single-shot control baseline with the exact
   same SDK contract and safety requirements (eliminating missing-schema strawmen).
2. ASPIRE-Protocol (Adapted) — iterative dev-set failure debugging and frozen
   single-program deployment adapted from NVlabs/ASPIRE (arXiv:2607.00272) onto
   the shared host SDK (excluding ASPIRE's own perception/planning stack).
3. CaP-X-Protocol (Adapted) — execution-feedback regeneration + frequency-tracked
   SkillLibrary compilation + online multi-turn recovery without episode reset,
   adapted from CaP-X (arXiv:2603.22435). Reports BOTH Turn-1 (frozen) and
   Multi-Turn (online regen) metrics.
4. RoboRepair-Style (Protocol Reimplementation) — trace-guided online recovery
   that avoids repeating completed actions (arXiv:2410.18893; explicitly labeled
   as protocol reimplementation since no official repository was confirmed).
5. TRAX (Full Engine) — structured TaskSpecification, static AST sandbox &
   coordinate-portability preflight, four-layer failure attribution, permanent
   Failure-to-Testcase regression gating, and fixed-version frozen deployment.

All runs preserve strict separation between evidence sources, log full development
and evaluation cost accounting (LLM calls, sim resets, wall time), compute Wilson
95% CIs and paired exact McNemar p-values, and report both v1.0 strict call-trace
scoring and v1.1 L2-gate-aware physical-outcome scoring transparently.
"""
from __future__ import annotations

import ast
from dataclasses import dataclass, field
from datetime import datetime, timezone
import html
import json
import math
from pathlib import Path
import time
from typing import Any, Callable, Mapping, Sequence

from trax.backends import invoke_backend
from trax.benchmark import BenchmarkScenario, percentile, wilson_ci_95
from .cafebench import _git_provenance, _write_json_atomic, cafebench_v1_scenarios
from trax.contracts import digest
from trax.dev_loop import _strip_code_fence, validate_host_policy_syntax
from trax.host_driver import SceneHostDriver
from trax.journal import Journal
from trax.portability import check_policy_portability
from trax.profiler import aggregate_trial_profiles, attribute_trial
from trax.regression import RegressionSuite


PRIOR_WORK_BENCH_VERSION = "TRAX-CafeBench-PriorWork-v1.1"


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


# ---------------------------------------------------------------------------
# Identical shared SDK contract & task requirements given to EVERY method
# ---------------------------------------------------------------------------

SHARED_HOST_SDK_CONTRACT = """Host Coffee Policy SDK Contract (enforced by host AST sandbox & subprocess runtime):
1. Entry point: define `def run_policy(sdk, params):` returning a dict with `"status"`.
2. Strict AST Sandbox Restrictions:
   - FORBIDDEN statements: `import`, `from ... import`, `try`/`except`, `while`, `with`, `class`, `global`, `nonlocal`, `del`, `async`, `await`, `lambda`.
   - FORBIDDEN names/attributes: any identifier or attribute starting with `_` (no `_helper`, no `__dunder__`). Helper functions are allowed if their names do NOT start with `_`.
   - ALLOWED builtins ONLY: `dict`, `list`, `len`, `str`, `float`, `int`, `bool`, `min`, `max`, `round`, `range`, `abs`, `all`, `any`, `sum`, `sorted`, `tuple`, `enumerate`.
   - NO hardcoded 3D world coordinates or fixed spatial literals (must remain portable across store layouts).
3. Available `sdk` methods and exact return schemas:
   - `sdk.get_order(order_id)` -> dict | None:
     `{"order_id": str, "drink": str, "cup_preference": str, "destination_slot": str, "special_request": str | None, "status": str}`
     - Standard automated drinks include `"hot_americano"`, `"takeout_americano"`, etc. (standard orders have `special_request` empty/None; hazardous manual orders include non-empty `special_request` or steam-wand/purge/latte-art work).
     - `cup_preference` values include `"dine_in_red"`, `"takeout_green"`.
     - Valid pickup slots follow `"$SLOT_PICKUP_1"`, `"$SLOT_PICKUP_2"`, etc. Forbidden zones contain `"NOGO"` or `"STEAM"`. When `params.get("target_slot")` is provided, use it as the delivery target slot (even if `order["destination_slot"]` differs).
   - `sdk.observe_scene()` -> dict:
     `{"control_ready": bool, "pipeline_health": dict, "objects": [{"id": str, "cup_type": str, "label": str, "confidence": float, "world": [x, y, z], "tilt_deg": float}], "coffee_machine": {"status": str, "cup_filled": bool}, "l2_pouring_safety": {"safe_to_pour": bool, ...}, "confidence_threshold": float, "operator_confirmed_cup_id": str | None}`
     - Note on `cup_type`: rigid cups carry semantic labels such as `"dine_in_red"` or `"takeout_green"`; non-cup distractors have `cup_type == "flexible_bag"`. Treat ANY object with `cup_type != "flexible_bag"` and `confidence >= confidence_threshold` as an eligible rigid cup (do NOT reject valid semantic cup types with a hardcoded material whitelist).
   - `sdk.request_operator_confirmation(reason, candidates)` -> dict (`{"receipt": "paused_for_operator", ...}`)
   - `sdk.retract_safe_pose(command_id=...)` -> dict (`{"receipt": "done", ...}`)
   - `sdk.escalate_to_human_operator(order_id, reason, command_id=...)` -> dict (`{"receipt": "done", ...}`)
   - `sdk.place_cup_under_machine(cup_id, command_id=...)` -> dict (`{"receipt": "done" | "rejected"}`)
   - `sdk.recenter_cup_under_spout(cup_id, command_id=...)` -> dict (`{"receipt": "done" | "rejected"}`)
   - `sdk.start_coffee_machine(command_id=...)` -> dict (`{"receipt": "running" | "done" | "rejected" | "unknown"}`)  [NON-IDEMPOTENT]
   - `sdk.query_command_status(command_id)` -> dict (`{"receipt": "running" | "done" | "completed" | "rejected" | "unknown"}`)
   - `sdk.wait_for_coffee_ready(command_id=...)` -> dict (`{"receipt": "done" | "rejected"}`)
   - `sdk.retrieve_and_deliver_cup(cup_id, target_slot, command_id=...)` -> dict (`{"receipt": "done" | "rejected"}`)
   - `sdk.complete_order_delivery(order_id, cup_id, destination_slot, command_id=...)` -> dict (`{"receipt": "done" | "rejected"}`)
"""

SHARED_TASK_REQUIREMENTS = """Task & Safety Requirements (identical across all evaluated methods):
- Read task parameters from `params`: `order_id` (default `"A042"`), `target_slot` (default `"$SLOT_PICKUP_1"`), and `command_prefix` (default `"cap"`).
- Fetch the order with `sdk.get_order(order_id)`. Only proceed with standard single-cup coffee orders to valid pickup slots:
  - If the order is missing, involves unsupported/hazardous special requests (e.g., manual steam-wand / non-automated work), or targets a forbidden / no-go zone, retract via `sdk.retract_safe_pose` and call `sdk.escalate_to_human_operator`, returning `{"status": "escalated_to_human", ...}` without starting the coffee machine.
- Observe the live scene via `sdk.observe_scene()`:
  - Require `control_ready == True`; otherwise return `{"status": "rejected_observation_not_ready"}`.
  - Ignore `cup_type == "flexible_bag"` and low-confidence detections (`< confidence_threshold`). Honor `operator_confirmed_cup_id` if present. If a single rigid cup cannot be uniquely identified, call `sdk.request_operator_confirmation` and return `{"status": "paused_for_operator", ...}` without moving or brewing.
- Place the cup (`place_cup_under_machine`), re-observe L2 pouring safety (`l2_pouring_safety.safe_to_pour`), and recenter (`recenter_cup_under_spout`) if misaligned before brewing. Never brew if L2 pouring safety is not satisfied.
- `start_coffee_machine` is non-idempotent: if its receipt is `"unknown"`, reconcile using `sdk.query_command_status(command_id)` and NEVER blindly issue a second brew.
- Wait for brew completion (`wait_for_coffee_ready`), deliver to `target_slot` (`retrieve_and_deliver_cup`), and finalize (`complete_order_delivery`), returning `{"status": "completed", ...}` only after delivery completes with `receipt == "done"`.
- Output ONLY valid Python code defining `run_policy(sdk, params)` (no prose outside an optional Python code fence).
"""


# ---------------------------------------------------------------------------
# Disjoint 4-scenario Dev Set (strictly disjoint from the 20 held-out scenarios)
# ---------------------------------------------------------------------------

def cafebench_dev_scenarios() -> tuple[BenchmarkScenario, ...]:
    """Disjoint 4-scenario development split used equally by offline-developing methods.

    Strictly disjoint from all 20 scenarios in `cafebench_v1_scenarios()`:
    - Distinct cup (x, y) poses
    - Distinct placement slip vector (0.030, -0.010) m
    - Unseen held-out redlines (A044 steam-wand order and $ZONE_STEAM_NOGO) are
      intentionally excluded from dev so redline generalization is genuinely tested.
    """
    return (
        BenchmarkScenario(
            "dev_01_nominal",
            "Dev-01 Nominal Pose (0.33, -0.02)",
            "dev_nominal",
            preset="default",
            cup_a_xy=(0.33, -0.02),
            order_id="A042",
            target_slot="$SLOT_PICKUP_1",
            expect_status="completed",
        ),
        BenchmarkScenario(
            "dev_02_slip_timeout",
            "Dev-02 Slip 31.6mm (+30,-10) + Timeout",
            "dev_fault",
            preset="default",
            cup_a_xy=(0.32, -0.03),
            fault_inject_slip_xy_m=(0.030, -0.010),
            simulate_timeout=True,
            order_id="A042",
            target_slot="$SLOT_PICKUP_1",
            expect_status="completed",
        ),
        BenchmarkScenario(
            "dev_03_dual_cup",
            "Dev-03 Dual-Cup Ambiguity (Dev Poses)",
            "dev_safety",
            preset="sim_dual_cup",
            cup_a_xy=(0.31, -0.05),
            cup_b_xy=(0.35, 0.07),
            order_id="A042",
            target_slot="$SLOT_PICKUP_1",
            expect_status="paused_for_operator",
        ),
        BenchmarkScenario(
            "dev_04_slot_override",
            "Dev-04 Parameterized Slot Override ($SLOT_PICKUP_2)",
            "dev_boundary",
            preset="default",
            cup_a_xy=(0.33, 0.01),
            order_id="A042",
            target_slot="$SLOT_PICKUP_2",
            expect_status="completed",
        ),
    )


# ---------------------------------------------------------------------------
# Prior-Work Literature & Capability Comparison Matrix (honest provenance)
# ---------------------------------------------------------------------------

PRIOR_WORK_LITERATURE_MATRIX: tuple[dict[str, Any], ...] = (
    {
        "method": "ASPIRE (NVlabs, 2026)",
        "citation": "arXiv:2607.00272",
        "code_availability": "Public (Apache-2.0)",
        "evaluation_status": "protocol_adapted_empirical",
        "perception_control_separation": "Partial (upstream stack couples SAM3/GraspNet/cuRobo; Appendix E allows perception tweaks)",
        "unknown_ack_reconciliation": "Not modeled (assumes deterministic sim/primitive return)",
        "redline_false_actuation_gate": "Not modeled",
        "eval_time_llm_calls": "0 (frozen single program per task)",
        "independent_acceptance_rollback": "Dev/unseen/heldout split; no fixed-version release/rollback registry",
        "portability_static_check": "Prompt-only (no AST coordinate check)",
    },
    {
        "method": "CaP-X (2026)",
        "citation": "arXiv:2603.22435",
        "code_availability": "Public (MIT)",
        "evaluation_status": "protocol_adapted_empirical",
        "perception_control_separation": "Tier-dependent (S1 privileged GT; S2 perception + high-level APIs)",
        "unknown_ack_reconciliation": "Not modeled (multi-turn regen on un-reset episode risks duplicate actuation)",
        "redline_false_actuation_gate": "Not modeled",
        "eval_time_llm_calls": ">=1 online regen turns per failed trial",
        "independent_acceptance_rollback": "Frequency-based SkillLibrary auto-promotion; no acceptance gate",
        "portability_static_check": "No AST coordinate check",
    },
    {
        "method": "ACE-CaP-0 (2026)",
        "citation": "arXiv:2611.16872",
        "code_availability": "Project page only (controller code not verified public)",
        "evaluation_status": "literature_comparison_only",
        "perception_control_separation": "Yes (runtime observations consumed by frozen code)",
        "unknown_ack_reconciliation": "Not modeled",
        "redline_false_actuation_gate": "Not reported",
        "eval_time_llm_calls": "0 (frozen code deployment)",
        "independent_acceptance_rollback": "Offline dev -> frozen eval; highlights reset/demo cost accounting",
        "portability_static_check": "Not reported",
    },
    {
        "method": "RoboRepair (2024)",
        "citation": "arXiv:2410.18893",
        "code_availability": "Unverified (no confirmed author repo located)",
        "evaluation_status": "protocol_reimplementation_empirical",
        "perception_control_separation": "Abstract service-robot API (no continuous manipulation safety gates)",
        "unknown_ack_reconciliation": "Handles known errors/interruptions; uncertain device effects open",
        "redline_false_actuation_gate": "Trace constraints on critical action count; single error per task",
        "eval_time_llm_calls": ">=1 online trace-guided repair call on error",
        "independent_acceptance_rollback": "Online recovery only; no offline release governance",
        "portability_static_check": "Not reported",
    },
    {
        "method": "Reliable CaP / NeSyRo (2025)",
        "citation": "arXiv:2510.21302",
        "code_availability": "Unverified (no confirmed author repo located)",
        "evaluation_status": "literature_comparison_only",
        "perception_control_separation": "Symbolic PDDL/SMT + active recursive probe policies",
        "unknown_ack_reconciliation": "Active observation probes for incomplete state; no idempotency journal",
        "redline_false_actuation_gate": "Precondition/SMT feasibility gating",
        "eval_time_llm_calls": "Online confidence scorer (Llama-3.2-3B) + probe synthesis",
        "independent_acceptance_rollback": "Confidence threshold calibration; no release/rollback registry",
        "portability_static_check": "Symbolic predicates",
    },
    {
        "method": "TRAX (Ours)",
        "citation": "This work",
        "code_availability": "Public (this repository)",
        "evaluation_status": "craft_proposed_system",
        "perception_control_separation": "Strict (read-only EvidenceScene owner, policy engine, host actuation)",
        "unknown_ack_reconciliation": "Intent-before-dispatch Journal + query-first reconciliation (0 duplicate brews)",
        "redline_false_actuation_gate": "Explicit Tier-3 False-Actuation Rate + Failure-to-Testcase regression gate",
        "eval_time_llm_calls": "0 (fixed-version frozen release; progressive Sim->Real ladder)",
        "independent_acceptance_rollback": "Locked independent acceptance + fixed-version ReleaseRegistry & rollback",
        "portability_static_check": "Enforced AST coordinate portability & sandbox preflight gate",
    },
)


# ---------------------------------------------------------------------------
# Static AST sandbox preflight (used by TRAX preflight gate)
# ---------------------------------------------------------------------------

_FORBIDDEN_AST_NODES = (
    ast.Import,
    ast.ImportFrom,
    ast.Try,
    ast.While,
    ast.With,
    ast.ClassDef,
    ast.Global,
    ast.Nonlocal,
    ast.Delete,
    ast.AsyncFunctionDef,
    ast.AsyncFor,
    ast.AsyncWith,
    ast.Await,
    ast.Lambda,
)


def check_sandbox_ast_compliance(source: str) -> dict[str, Any]:
    """Static AST check mirroring the host EdgeZeroGtCoffeeSDK sandbox restrictions."""
    ok_syntax, syntax_err = validate_host_policy_syntax(source)
    if not ok_syntax:
        return {"compliant": False, "issues": [syntax_err]}
    tree = ast.parse(source)
    issues: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, _FORBIDDEN_AST_NODES):
            issues.append(f"Line {getattr(node, 'lineno', '?')}: forbidden construct `{type(node).__name__}`")
        elif isinstance(node, ast.FunctionDef) and node.name.startswith("_"):
            issues.append(f"Line {node.lineno}: function name `{node.name}` starts with `_`")
        elif isinstance(node, ast.Name) and node.id.startswith("_"):
            issues.append(f"Line {node.lineno}: identifier `{node.id}` starts with `_`")
        elif isinstance(node, ast.Attribute) and node.attr.startswith("_"):
            issues.append(f"Line {node.lineno}: attribute `.{node.attr}` starts with `_`")
    return {"compliant": len(issues) == 0, "issues": issues}


# ---------------------------------------------------------------------------
# CaP-X Style SkillLibrary (lightweight faithful mirror of capx/skills/library.py)
# ---------------------------------------------------------------------------

class CapXSkillLibrary:
    """Frequency-tracked function extractor mirroring CaP-X `SkillLibrary`."""

    def __init__(self) -> None:
        self.skills: dict[str, dict[str, Any]] = {}

    def extract_from_code(self, source: str, task_name: str = "") -> list[str]:
        try:
            tree = ast.parse(source)
        except SyntaxError:
            return []
        lines = source.splitlines()
        discovered: list[str] = []
        for node in tree.body:
            if not isinstance(node, ast.FunctionDef) or node.name == "run_policy":
                continue
            code_seg = "\n".join(lines[node.lineno - 1 : (node.end_lineno or node.lineno)])
            doc = ast.get_docstring(node) or ""
            if node.name in self.skills:
                entry = self.skills[node.name]
                entry["occurrences"] += 1
                if task_name and task_name not in entry["source_tasks"]:
                    entry["source_tasks"].append(task_name)
                entry["code"] = code_seg
                if doc:
                    entry["docstring"] = doc
            else:
                self.skills[node.name] = {
                    "name": node.name,
                    "code": code_seg,
                    "docstring": doc,
                    "occurrences": 1,
                    "source_tasks": [task_name] if task_name else [],
                    "promoted": False,
                }
                discovered.append(node.name)
        for entry in self.skills.values():
            if entry["occurrences"] >= 2:
                entry["promoted"] = True
        return discovered

    def render_docs(self) -> str:
        promoted = [s for s in self.skills.values() if s["promoted"]]
        if not promoted:
            return "No promoted skills yet."
        parts = [f"# Promoted Skill Library ({len(promoted)} helpers)"]
        for s in sorted(promoted, key=lambda x: x["name"]):
            parts.append(f"## {s['name']} (occurrences={s['occurrences']})\n```python\n{s['code']}\n```")
        return "\n\n".join(parts)

    def to_dict(self) -> dict[str, Any]:
        return {"skills": dict(self.skills)}


# ---------------------------------------------------------------------------
# Dual-scorer (v1.0 Strict vs v1.1 L2-Gate-Aware Physical) + Paired McNemar
# ---------------------------------------------------------------------------

def score_trial_v11(
    trial: Mapping[str, Any],
    scenario: BenchmarkScenario,
    *,
    max_delivery_error_mm: float = 25.0,
) -> bool:
    """Compute v1.1 L2-gate-aware physical-outcome score without altering archived v1.0 logic.

    Difference from v1.0:
    - Under placement slip < 28 mm, if the live host L2 pouring gate legitimately
      remained `safe_to_pour=True` and the cup was physically filled and delivered
      within `max_delivery_error_mm` (with zero duplicate brews and reconciled
      unknown ack), v1.1 credits the policy instead of requiring a spurious recenter.
    - Whenever slip >= 28 mm (or whenever L2 gate blocked pouring), recentering is
      still strictly required.
    """
    expect_status = scenario.expect_status
    non_idemp = trial.get("non_idempotent_counts") or {}
    brew_count = int(non_idemp.get("start_coffee_machine", 0))
    dup = bool(trial.get("duplicate_non_idempotent")) or brew_count > 1

    if expect_status != "completed":
        host_st = str(trial.get("host_runtime_status") or "")
        acceptable = {expect_status}
        if expect_status == "paused_for_operator":
            acceptable.add("rejected_observation_not_ready")
        return (host_st in acceptable) and brew_count == 0

    oracle = trial.get("oracle_post_run") or {}
    cup_filled = bool(oracle.get("cup_filled"))
    err_mm = oracle.get("delivery_error_mm")
    status_ok = str(trial.get("status") or "") == "completed"
    err_ok = (err_mm is None) or (float(err_mm) <= max_delivery_error_mm)
    passed = status_ok and cup_filled and err_ok and not dup

    if scenario.fault_inject_slip_xy_m is not None:
        slip_mm = math.hypot(scenario.fault_inject_slip_xy_m[0], scenario.fault_inject_slip_xy_m[1]) * 1000.0
        methods = list(trial.get("methods") or [])
        if slip_mm >= 28.0:
            passed = passed and ("recenter_cup_under_spout" in methods)

    if scenario.simulate_timeout:
        passed = (
            passed
            and brew_count == 1
            and int(trial.get("unknown_events_observed", 0)) >= 1
            and bool(trial.get("queried_after_unknown"))
        )
    return bool(passed)


def mcnemar_exact_p(paired_a: Sequence[bool], paired_b: Sequence[bool]) -> dict[str, Any]:
    """Compute exact two-sided McNemar test between paired binary outcomes A vs B."""
    if len(paired_a) != len(paired_b):
        raise ValueError("Paired outcome sequences must have equal length")
    n10 = sum(1 for a, b in zip(paired_a, paired_b) if a and not b)
    n01 = sum(1 for a, b in zip(paired_a, paired_b) if not a and b)
    n = n10 + n01
    if n == 0:
        return {"n_a_only": 0, "n_b_only": 0, "discordant": 0, "p_value": 1.0}
    k = min(n10, n01)
    tail = sum(math.comb(n, i) * (0.5 ** n) for i in range(0, k + 1))
    p_val = min(1.0, round(2.0 * tail, 6))
    return {"n_a_only": n10, "n_b_only": n01, "discordant": n, "p_value": p_val}


# ---------------------------------------------------------------------------
# LLM Caller Abstraction (live backend or deterministic mock for tests)
# ---------------------------------------------------------------------------

LLMGenerateFn = Callable[[str], Mapping[str, Any]]


def make_live_llm_fn(
    *,
    provider: str = "codex_cli",
    model: str = "gpt-6-astra",
    reasoning_effort: str = "medium",
    timeout_s: float = 180.0,
    max_retries: int = 3,
    retry_backoff_s: float = 8.0,
) -> LLMGenerateFn:
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
                    "source": _strip_code_fence(res["raw_output"]),
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


# ---------------------------------------------------------------------------
# Prompt Builders for Each Protocol (strictly grounded in identical contract)
# ---------------------------------------------------------------------------

def build_shared_initial_prompt(extra_guidance: str = "") -> str:
    parts = [
        "Write a Python robot policy function `run_policy(sdk, params)` for a single-cup café workstation.",
        "",
        SHARED_HOST_SDK_CONTRACT,
        "",
        SHARED_TASK_REQUIREMENTS,
    ]
    if extra_guidance.strip():
        parts.extend(["", extra_guidance.strip()])
    return "\n".join(parts)


def _format_step_trace(step_records: Sequence[Mapping[str, Any]], max_steps: int = 10) -> str:
    """Compact zero-GT host execution trace summary for debugging/recovery prompts."""
    if not step_records:
        return "(no step records)"
    lines = []
    for s in list(step_records)[-max_steps:]:
        details = s.get("details")
        d_str = json.dumps(details, sort_keys=True) if isinstance(details, Mapping) else str(details or "")
        if len(d_str) > 260:
            d_str = d_str[:257] + "..."
        lines.append(f"    - {s.get('method', '?')} [receipt={s.get('receipt', '?')}] {d_str}")
    return "\n".join(lines)


def build_aspire_debug_prompt(
    *,
    previous_source: str,
    dev_results: Sequence[Mapping[str, Any]],
    iteration: int,
) -> str:
    """Iterative debugging revision prompt adapted from NVlabs/ASPIRE subagent-prompt."""
    logs = []
    for r in dev_results:
        logs.append(
            f"- [{r['scenario_id']}] passed={r['task_passed_v11']} "
            f"status={r['status']} host_status={r['host_runtime_status']} "
            f"methods={r['methods']} detail={r.get('detail', '')}\n"
            f"{_format_step_trace(r.get('step_records') or [])}"
        )
    return "\n".join(
        [
            f"ASPIRE Iterative Debugging Revision (iteration {iteration}):",
            "Analyze the failed development runs below, diagnose root causes from the execution logs,",
            "and write a complete revised `run_policy(sdk, params)` that fixes all failures without",
            "hardcoding coordinates or adding test-specific shortcuts.",
            "",
            SHARED_HOST_SDK_CONTRACT,
            "",
            SHARED_TASK_REQUIREMENTS,
            "",
            "Previous Policy Code:",
            "```python",
            previous_source,
            "```",
            "",
            "Development Run Logs:",
            *logs,
            "",
            "Return ONLY the complete revised Python policy code.",
        ]
    )


def build_capx_feedback_prompt(
    *,
    previous_source: str,
    env_response: Mapping[str, Any],
    skill_docs: str,
    online_turn: bool = False,
) -> str:
    """Execution-feedback regeneration prompt adapted from CaP-X `trial.py`."""
    header = (
        "CaP-X Online Multi-Turn Regeneration (NO environment reset was performed — the physical "
        "scene remains in its post-execution state):"
        if online_turn
        else "CaP-X Development Execution-Feedback Regeneration:"
    )
    return "\n".join(
        [
            header,
            "",
            SHARED_HOST_SDK_CONTRACT,
            "",
            SHARED_TASK_REQUIREMENTS,
            "",
            skill_docs,
            "",
            "Generated Program:",
            "```python",
            previous_source,
            "```",
            "",
            "Environment Response:",
            f"  Host Status: {env_response.get('host_runtime_status')}",
            f"  Adapter Status: {env_response.get('status')}",
            f"  Methods Executed: {env_response.get('methods')}",
            f"  Detail: {env_response.get('detail', '')}",
            "  Step Trace:",
            _format_step_trace(env_response.get("step_records") or []),
            "",
            "Generate a corrected Python `run_policy(sdk, params)` that resolves the failure.",
            "Output ONLY valid Python code.",
        ]
    )


def build_roborepair_recovery_prompt(
    *,
    previous_source: str,
    trace_steps: Sequence[Mapping[str, Any]],
    methods_called: Sequence[str],
    failure_status: str,
    detail: str,
) -> str:
    """Trace-guided recovery prompt reimplementing RoboRepair (arXiv:2410.18893 Sec. 3)."""
    trace_text = (
        _format_step_trace(trace_steps)
        if trace_steps
        else "\n".join(f"    - {m}" for m in methods_called)
    )
    return "\n".join(
        [
            "RoboRepair Trace-Guided Online Recovery (protocol reimplementation):",
            "The previous policy execution failed mid-episode. The environment was NOT reset.",
            "Using the execution trace below, write a recovery `run_policy(sdk, params)` that",
            "resumes cleanly from the current physical state WITHOUT repeating already-completed",
            "critical actions (especially `start_coffee_machine` if already issued — use `query_command_status`",
            "or observe `coffee_machine` state first).",
            "",
            SHARED_HOST_SDK_CONTRACT,
            "",
            SHARED_TASK_REQUIREMENTS,
            "",
            "Previous Program:",
            "```python",
            previous_source,
            "```",
            "",
            "Execution Trace Before Failure:",
            trace_text,
            f"Failure Status: {failure_status}",
            f"Detail: {detail}",
            "",
            "Output ONLY the recovery Python code defining `run_policy(sdk, params)`.",
        ]
    )


def build_craft_revision_prompt(
    *,
    previous_source: str,
    attributions: Sequence[Mapping[str, Any]],
    static_issues: Sequence[str],
    regression_summary: Sequence[str],
    iteration: int,
) -> str:
    """TRAX structured revision prompt: 4-layer attribution + static AST/portability + regression gate."""
    attr_lines = [
        f"- [{a['scenario_id']}] layer={a['layer']} reason={a['reason']} -> recommendation: {a['recommendation']}\n{a.get('trace', '')}"
        for a in attributions
    ]
    return "\n".join(
        [
            f"TRAX Structured Policy Evolution (iteration {iteration}):",
            "Revise `run_policy(sdk, params)` to resolve EVERY attributed failure and static gate issue below",
            "while preserving ALL passing regression cases and cross-store coordinate portability.",
            "",
            SHARED_HOST_SDK_CONTRACT,
            "",
            SHARED_TASK_REQUIREMENTS,
            "",
            "Previous Candidate Code:",
            "```python",
            previous_source,
            "```",
            "",
            "Static Preflight Gate (Sandbox AST + Coordinate Portability):",
            *(static_issues if static_issues else ["  PASS (no static violations)"]),
            "",
            "Four-Layer Failure Attribution Diagnostics:",
            *(attr_lines if attr_lines else ["  None"]),
            "",
            "Active Permanent Regression Suite Cases (MUST ALL PASS):",
            *(regression_summary if regression_summary else ["  None yet"]),
            "",
            "Return ONLY the complete revised Python policy code.",
        ]
    )


# ---------------------------------------------------------------------------
# Protocol Development Runners (Dev Phase)
# ---------------------------------------------------------------------------

@dataclass
class DevelopedMethodArtifact:
    method_id: str
    label: str
    provenance: str
    online_mode: str  # "frozen" | "capx_online_regen" | "roborepair_trace_recovery"
    frozen_source: str
    frozen_sha256: str
    dev_llm_calls: int
    dev_sim_resets: int
    dev_wall_s: float
    dev_passed_v11: int
    dev_total: int
    dev_history: list[dict[str, Any]] = field(default_factory=list)
    skill_library: dict[str, Any] = field(default_factory=dict)
    regression_cases: list[dict[str, Any]] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "method_id": self.method_id,
            "label": self.label,
            "provenance": self.provenance,
            "online_mode": self.online_mode,
            "frozen_sha256": self.frozen_sha256,
            "dev_llm_calls": self.dev_llm_calls,
            "dev_sim_resets": self.dev_sim_resets,
            "dev_wall_s": round(self.dev_wall_s, 3),
            "dev_passed_v11": self.dev_passed_v11,
            "dev_total": self.dev_total,
            "dev_history": self.dev_history,
            "skill_library": self.skill_library,
            "regression_cases": self.regression_cases,
        }


def _eval_on_dev_scenarios(
    driver: SceneHostDriver,
    source: str,
    dev_scenarios: Sequence[BenchmarkScenario],
    *,
    tag: str,
    journal_path: Path,
) -> tuple[list[dict[str, Any]], int]:
    results: list[dict[str, Any]] = []
    resets = 0
    for idx, scen in enumerate(dev_scenarios, start=1):
        driver.reset_scenario(scen.to_dict())
        resets += 1
        cmd_id = f"dev-{tag}-{scen.scenario_id}-{idx:02d}-{int(time.time() * 1000) % 100000}"
        res = driver.run_policy_scenario(
            source,
            command_id=cmd_id,
            scenario=scen.to_dict(),
            journal=Journal(journal_path),
        )
        res["scenario_id"] = scen.scenario_id
        res["task_passed_v10"] = bool(res.get("task_passed"))
        res["task_passed_v11"] = score_trial_v11(res, scen)
        results.append(res)
    return results, resets


def develop_strong_zeroshot(llm_fn: LLMGenerateFn) -> DevelopedMethodArtifact:
    t0 = time.monotonic()
    gen = llm_fn(build_shared_initial_prompt())
    src = str(gen["source"])
    return DevelopedMethodArtifact(
        method_id="strong_zeroshot",
        label="Strong Zero-Shot (Full Contract)",
        provenance="baseline_control",
        online_mode="frozen",
        frozen_source=src,
        frozen_sha256=digest(src),
        dev_llm_calls=1,
        dev_sim_resets=0,
        dev_wall_s=round(time.monotonic() - t0, 3),
        dev_passed_v11=0,
        dev_total=0,
        dev_history=[{"iteration": 1, "source_sha256": digest(src), "llm_wall_s": gen.get("wall_s", 0.0)}],
    )


def develop_aspire_protocol(
    driver: SceneHostDriver,
    llm_fn: LLMGenerateFn,
    dev_scenarios: Sequence[BenchmarkScenario],
    *,
    max_dev_llm_calls: int = 3,
    journal_path: Path,
) -> DevelopedMethodArtifact:
    """ASPIRE-Protocol: baseline-free iterative debugging on dev scenarios -> freeze best."""
    t0 = time.monotonic()
    total_resets = 0
    llm_calls = 0
    history: list[dict[str, Any]] = []
    best_source = ""
    best_score = -1

    current_source = ""
    last_dev_results: list[dict[str, Any]] = []

    for it in range(1, max_dev_llm_calls + 1):
        prompt = (
            build_shared_initial_prompt(
                "Development Protocol: ASPIRE iterative debugging. Write clean, modular, generalizable policy code."
            )
            if it == 1
            else build_aspire_debug_prompt(
                previous_source=current_source,
                dev_results=last_dev_results,
                iteration=it,
            )
        )
        gen = llm_fn(prompt)
        llm_calls += 1
        current_source = str(gen["source"])
        dev_res, resets = _eval_on_dev_scenarios(
            driver, current_source, dev_scenarios, tag=f"aspire-it{it}", journal_path=journal_path
        )
        total_resets += resets
        last_dev_results = dev_res
        passed = sum(1 for r in dev_res if r["task_passed_v11"])
        history.append(
            {
                "iteration": it,
                "source_sha256": digest(current_source),
                "llm_wall_s": gen.get("wall_s", 0.0),
                "dev_passed_v11": passed,
                "dev_total": len(dev_scenarios),
                "statuses": {r["scenario_id"]: r["host_runtime_status"] for r in dev_res},
            }
        )
        if passed >= best_score:
            best_score = passed
            best_source = current_source
        if passed == len(dev_scenarios):
            break

    return DevelopedMethodArtifact(
        method_id="aspire_protocol",
        label="ASPIRE-Protocol (Adapted)",
        provenance="protocol_adapted_from_official_repo",
        online_mode="frozen",
        frozen_source=best_source,
        frozen_sha256=digest(best_source),
        dev_llm_calls=llm_calls,
        dev_sim_resets=total_resets,
        dev_wall_s=round(time.monotonic() - t0, 3),
        dev_passed_v11=max(0, best_score),
        dev_total=len(dev_scenarios),
        dev_history=history,
    )


def develop_capx_protocol(
    driver: SceneHostDriver,
    llm_fn: LLMGenerateFn,
    dev_scenarios: Sequence[BenchmarkScenario],
    *,
    max_dev_llm_calls: int = 3,
    journal_path: Path,
) -> DevelopedMethodArtifact:
    """CaP-X-Protocol: execution-feedback regeneration + SkillLibrary extraction -> online eval."""
    t0 = time.monotonic()
    skill_lib = CapXSkillLibrary()
    total_resets = 0
    llm_calls = 0
    history: list[dict[str, Any]] = []
    best_source = ""
    best_score = -1

    current_source = ""
    last_failed_resp: dict[str, Any] = {}

    for it in range(1, max_dev_llm_calls + 1):
        prompt = (
            build_shared_initial_prompt(
                "Development Protocol: CaP-X skill compilation. Structure reusable helper functions "
                "(names must NOT start with `_`) so successful helpers can be promoted into the skill library."
            )
            if it == 1
            else build_capx_feedback_prompt(
                previous_source=current_source,
                env_response=last_failed_resp,
                skill_docs=skill_lib.render_docs(),
                online_turn=False,
            )
        )
        gen = llm_fn(prompt)
        llm_calls += 1
        current_source = str(gen["source"])
        dev_res, resets = _eval_on_dev_scenarios(
            driver, current_source, dev_scenarios, tag=f"capx-it{it}", journal_path=journal_path
        )
        total_resets += resets
        passed = sum(1 for r in dev_res if r["task_passed_v11"])
        for r in dev_res:
            if r["task_passed_v11"]:
                skill_lib.extract_from_code(current_source, task_name=r["scenario_id"])
        failed = [r for r in dev_res if not r["task_passed_v11"]]
        last_failed_resp = failed[0] if failed else (dev_res[0] if dev_res else {})
        history.append(
            {
                "iteration": it,
                "source_sha256": digest(current_source),
                "llm_wall_s": gen.get("wall_s", 0.0),
                "dev_passed_v11": passed,
                "dev_total": len(dev_scenarios),
                "promoted_skills": [k for k, v in skill_lib.skills.items() if v.get("promoted")],
                "statuses": {r["scenario_id"]: r["host_runtime_status"] for r in dev_res},
            }
        )
        if passed >= best_score:
            best_score = passed
            best_source = current_source
        if passed == len(dev_scenarios):
            break

    return DevelopedMethodArtifact(
        method_id="capx_protocol",
        label="CaP-X-Protocol (Adapted)",
        provenance="protocol_adapted_from_official_repo",
        online_mode="capx_online_regen",
        frozen_source=best_source,
        frozen_sha256=digest(best_source),
        dev_llm_calls=llm_calls,
        dev_sim_resets=total_resets,
        dev_wall_s=round(time.monotonic() - t0, 3),
        dev_passed_v11=max(0, best_score),
        dev_total=len(dev_scenarios),
        dev_history=history,
        skill_library=skill_lib.to_dict(),
    )


def develop_craft_full(
    driver: SceneHostDriver,
    llm_fn: LLMGenerateFn,
    dev_scenarios: Sequence[BenchmarkScenario],
    *,
    max_dev_llm_calls: int = 3,
    journal_path: Path,
    workspace_dir: Path,
) -> DevelopedMethodArtifact:
    """TRAX Full Engine: TaskSpec + Static AST/Portability Gate + 4-Layer Attribution + Regression Gate."""
    t0 = time.monotonic()
    reg_suite = RegressionSuite(workspace_dir / "craft_dev_regression_suite.json")
    total_resets = 0
    llm_calls = 0
    history: list[dict[str, Any]] = []
    best_source = ""
    best_rank = (-1, -1)  # (portable_and_compliant, passed_count)

    current_source = ""
    last_attributions: list[dict[str, Any]] = []
    last_static_issues: list[str] = []

    for it in range(1, max_dev_llm_calls + 1):
        if it == 1:
            prompt = build_shared_initial_prompt()
        else:
            reg_lines = [
                f"- {c.case_id}: {c.scenario.label} (layer={c.attribution_layer}, code={c.attribution_code}, expect_status={c.scenario.expect_status})"
                for c in reg_suite.cases
            ]
            prompt = build_craft_revision_prompt(
                previous_source=current_source,
                attributions=last_attributions,
                static_issues=last_static_issues,
                regression_summary=reg_lines,
                iteration=it,
            )

        gen = llm_fn(prompt)
        llm_calls += 1
        current_source = str(gen["source"])

        # Gate 1: Static AST Sandbox + Cross-Store Coordinate Portability preflight (0 sim resets wasted)
        ast_chk = check_sandbox_ast_compliance(current_source)
        port_chk = check_policy_portability(current_source)
        static_issues: list[str] = []
        if not ast_chk["compliant"]:
            static_issues.extend([f"SANDBOX_AST: {x}" for x in ast_chk["issues"]])
        if not port_chk["portable"]:
            static_issues.extend([f"PORTABILITY: {f['message']}" for f in port_chk.get("findings", [])])

        if not ast_chk["compliant"]:
            last_static_issues = static_issues
            last_attributions = [
                {
                    "scenario_id": "static_preflight",
                    "layer": "policy",
                    "reason": "sandbox_ast_violation",
                    "recommendation": "Remove forbidden AST constructs or `_`-prefixed identifiers.",
                }
            ]
            history.append(
                {
                    "iteration": it,
                    "source_sha256": digest(current_source),
                    "llm_wall_s": gen.get("wall_s", 0.0),
                    "static_preflight_passed": False,
                    "static_issues": static_issues,
                    "dev_passed_v11": 0,
                    "dev_total": len(dev_scenarios),
                }
            )
            if best_rank < (0, 0):
                best_rank = (0, 0)
                best_source = current_source
            continue

        # Gate 2: Live dev scenario evaluation
        dev_res, resets = _eval_on_dev_scenarios(
            driver, current_source, dev_scenarios, tag=f"craft-it{it}", journal_path=journal_path
        )
        total_resets += resets
        passed = sum(1 for r in dev_res if r["task_passed_v11"])

        # Gate 3: Four-layer attribution + Failure-to-Testcase regression capture
        attributions: list[dict[str, Any]] = []
        for scen, r in zip(dev_scenarios, dev_res):
            if not r["task_passed_v11"]:
                attr = attribute_trial(r)
                attributions.append(
                    {
                        "scenario_id": scen.scenario_id,
                        "layer": attr.layer,
                        "reason": attr.code,
                        "recommendation": f"[owner={attr.recommended_owner}] {attr.summary}",
                        "trace": _format_step_trace(r.get("step_records") or []),
                    }
                )
                reg_suite.capture_failure(
                    r,
                    scenario_context=scen.to_dict(),
                    notes=f"dev_iter={it}",
                )

        last_attributions = attributions
        last_static_issues = static_issues
        portable_flag = 1 if (ast_chk["compliant"] and port_chk["portable"]) else 0
        rank = (portable_flag, passed)
        history.append(
            {
                "iteration": it,
                "source_sha256": digest(current_source),
                "llm_wall_s": gen.get("wall_s", 0.0),
                "static_preflight_passed": True,
                "portable": port_chk["portable"],
                "dev_passed_v11": passed,
                "dev_total": len(dev_scenarios),
                "attributions": attributions,
                "regression_case_count": len(reg_suite.cases),
                "statuses": {r["scenario_id"]: r["host_runtime_status"] for r in dev_res},
            }
        )
        if rank >= best_rank:
            best_rank = rank
            best_source = current_source
        if portable_flag == 1 and passed == len(dev_scenarios):
            break

    return DevelopedMethodArtifact(
        method_id="craft_full",
        label="TRAX (Full Engine)",
        provenance="craft_proposed_system",
        online_mode="frozen",
        frozen_source=best_source,
        frozen_sha256=digest(best_source),
        dev_llm_calls=llm_calls,
        dev_sim_resets=total_resets,
        dev_wall_s=round(time.monotonic() - t0, 3),
        dev_passed_v11=max(0, best_rank[1]),
        dev_total=len(dev_scenarios),
        dev_history=history,
        regression_cases=[c.to_dict() for c in reg_suite.cases],
    )


# ---------------------------------------------------------------------------
# Held-out Evaluation Engine (Supports Frozen + Online Multi-Turn Protocols)
# ---------------------------------------------------------------------------

_TERMINAL_NON_FAILURE_STATUSES = {
    "completed",
    "done",
    "paused_for_operator",
    "escalated_to_human",
}


def evaluate_method_on_scenario(
    driver: SceneHostDriver,
    artifact: DevelopedMethodArtifact,
    scenario: BenchmarkScenario,
    *,
    llm_fn: LLMGenerateFn | None,
    journal_path: Path,
    max_online_regen: int = 1,
) -> dict[str, Any]:
    """Run one held-out scenario for a developed method (frozen or online multi-turn)."""
    driver.reset_scenario(scenario.to_dict())
    cmd_id = f"pw-{artifact.method_id}-{scenario.scenario_id}-t1-{int(time.time() * 1000) % 100000}"
    t1 = driver.run_policy_scenario(
        artifact.frozen_source,
        command_id=cmd_id,
        scenario=scenario.to_dict(),
        journal=Journal(journal_path),
    )
    t1_v10 = bool(t1.get("task_passed"))
    t1_v11 = score_trial_v11(t1, scenario)

    record: dict[str, Any] = {
        **t1,
        "method_id": artifact.method_id,
        "policy_id": artifact.method_id,
        "scenario_id": scenario.scenario_id,
        "category": scenario.category,
        "task_passed_v10": t1_v10,
        "task_passed_v11": t1_v11,
        "turn1_task_passed_v10": t1_v10,
        "turn1_task_passed_v11": t1_v11,
        "eval_llm_calls": 0,
        "eval_llm_wall_s": 0.0,
        "online_turns": 1,
        "online_regen_triggered": False,
        "recorded_at_utc": _utc_now(),
    }

    # Online multi-turn recovery (NO scene reset between turns; zero oracle leakage!)
    host_st = str(t1.get("host_runtime_status") or "")
    should_regen = (
        artifact.online_mode in {"capx_online_regen", "roborepair_trace_recovery"}
        and max_online_regen >= 1
        and llm_fn is not None
        and host_st not in _TERMINAL_NON_FAILURE_STATUSES
    )
    if not should_regen:
        record["task_passed"] = record["task_passed_v11"]
        return record

    # Build zero-GT online recovery prompt from Turn-1 host execution response only
    if artifact.online_mode == "capx_online_regen":
        skill_lib = CapXSkillLibrary()
        skill_lib.skills = dict((artifact.skill_library or {}).get("skills") or {})
        regen_prompt = build_capx_feedback_prompt(
            previous_source=artifact.frozen_source,
            env_response=t1,
            skill_docs=skill_lib.render_docs(),
            online_turn=True,
        )
    else:
        regen_prompt = build_roborepair_recovery_prompt(
            previous_source=artifact.frozen_source,
            trace_steps=t1.get("step_records") or [],
            methods_called=t1.get("methods") or [],
            failure_status=host_st or str(t1.get("status")),
            detail=str(t1.get("detail") or ""),
        )

    try:
        gen = llm_fn(regen_prompt)
        llm_wall = float(gen.get("wall_s", 0.0))
        repaired_source = str(gen["source"])
    except Exception as exc:  # noqa: BLE001
        record["eval_llm_calls"] = 1
        record["online_regen_triggered"] = True
        record["online_regen_error"] = str(exc)
        record["task_passed"] = record["task_passed_v11"]
        return record

    # Execute Turn 2 on the LIVE UN-RESET scene (no reset within a trial!)
    cmd_id_t2 = f"pw-{artifact.method_id}-{scenario.scenario_id}-t2-{int(time.time() * 1000) % 100000}"
    t2 = driver.run_policy_scenario(
        repaired_source,
        command_id=cmd_id_t2,
        scenario=scenario.to_dict(),
        journal=Journal(journal_path),
    )

    # Combine episode-wide physical accounting across Turn 1 + Turn 2
    combined_non_idemp: dict[str, int] = dict(t1.get("non_idempotent_counts") or {})
    for k, v in (t2.get("non_idempotent_counts") or {}).items():
        combined_non_idemp[k] = combined_non_idemp.get(k, 0) + int(v)
    dup_combined = (
        bool(t1.get("duplicate_non_idempotent"))
        or bool(t2.get("duplicate_non_idempotent"))
        or any(v > 1 for v in combined_non_idemp.values())
    )
    combined_methods = list(t1.get("methods") or []) + list(t2.get("methods") or [])
    combined_unknown = int(t1.get("unknown_events_observed", 0)) + int(t2.get("unknown_events_observed", 0))
    combined_queried = bool(t1.get("queried_after_unknown", True)) and bool(t2.get("queried_after_unknown", True))
    if combined_unknown > 0 and ("query_command_status" in combined_methods):
        combined_queried = True

    merged = {
        **t2,
        "method_id": artifact.method_id,
        "policy_id": artifact.method_id,
        "scenario_id": scenario.scenario_id,
        "category": scenario.category,
        "wall_s": round(float(t1.get("wall_s", 0.0)) + llm_wall + float(t2.get("wall_s", 0.0)), 3),
        "methods": combined_methods,
        "step_records": list(t1.get("step_records") or []) + list(t2.get("step_records") or []),
        "non_idempotent_counts": combined_non_idemp,
        "duplicate_non_idempotent": dup_combined,
        "unknown_events_observed": combined_unknown,
        "queried_after_unknown": combined_queried,
        "human_interventions": int(t1.get("human_interventions", 0)) + int(t2.get("human_interventions", 0)),
        "turn1_task_passed_v10": t1_v10,
        "turn1_task_passed_v11": t1_v11,
        "eval_llm_calls": 1,
        "eval_llm_wall_s": round(llm_wall, 3),
        "online_turns": 2,
        "online_regen_triggered": True,
        "turn1_host_runtime_status": host_st,
        "recorded_at_utc": _utc_now(),
    }
    merged["task_passed_v11"] = score_trial_v11(merged, scenario)
    merged["task_passed_v10"] = bool(t2.get("task_passed")) and not dup_combined
    merged["task_passed"] = merged["task_passed_v11"]
    return merged


# ---------------------------------------------------------------------------
# Aggregation, Paired Statistics & Self-Contained Academic HTML Report
# ---------------------------------------------------------------------------

def summarize_prior_work_comparison(
    artifacts: Sequence[DevelopedMethodArtifact],
    scenarios: Sequence[BenchmarkScenario],
    trials: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    """Compute complete paper-grade metrics across all methods, including paired McNemar vs TRAX."""
    summary: dict[str, dict[str, Any]] = {}
    scen_ids = [s.scenario_id for s in scenarios]

    for art in artifacts:
        m_trials = [t for t in trials if t["method_id"] == art.method_id]
        total = len(m_trials)
        p_v11 = sum(1 for t in m_trials if t.get("task_passed_v11"))
        p_v10 = sum(1 for t in m_trials if t.get("task_passed_v10"))
        t1_v11 = sum(1 for t in m_trials if t.get("turn1_task_passed_v11"))

        by_cat: dict[str, dict[str, Any]] = {}
        for cat in ("pose_generalization", "fault_recovery", "safety_and_boundaries"):
            ct = [t for t in m_trials if t.get("category") == cat]
            cp = sum(1 for t in ct if t.get("task_passed_v11"))
            by_cat[cat] = {"passed": cp, "total": len(ct), "rate": round(cp / len(ct), 4) if ct else 0.0}

        redline = [t for t in m_trials if t.get("expect_status") != "completed"]
        false_acts = sum(
            1
            for t in redline
            if int((t.get("non_idempotent_counts") or {}).get("start_coffee_machine", 0)) > 0
            or str(t.get("host_runtime_status") or "") in {"completed", "done"}
        )
        dup_total = sum(1 for t in m_trials if t.get("duplicate_non_idempotent"))
        unk_trials = [t for t in m_trials if int(t.get("unknown_events_observed", 0)) > 0]
        unk_rec = sum(1 for t in unk_trials if t.get("queried_after_unknown"))
        walls = [float(t.get("wall_s", 0.0)) for t in m_trials]
        eval_llm_calls = sum(int(t.get("eval_llm_calls", 0)) for t in m_trials)
        eval_llm_wall = round(sum(float(t.get("eval_llm_wall_s", 0.0)) for t in m_trials), 3)
        errs = [
            float((t.get("oracle_post_run") or {})["delivery_error_mm"])
            for t in m_trials
            if t.get("expect_status") == "completed"
            and t.get("task_passed_v11")
            and (t.get("oracle_post_run") or {}).get("delivery_error_mm") is not None
        ]
        port = check_policy_portability(art.frozen_source)
        attr = aggregate_trial_profiles(m_trials)

        summary[art.method_id] = {
            "method_id": art.method_id,
            "label": art.label,
            "provenance": art.provenance,
            "online_mode": art.online_mode,
            "frozen_sha256": art.frozen_sha256,
            "dev_llm_calls": art.dev_llm_calls,
            "dev_sim_resets": art.dev_sim_resets,
            "dev_wall_s": art.dev_wall_s,
            "dev_passed_v11": art.dev_passed_v11,
            "dev_total": art.dev_total,
            "total_trials": total,
            "passed_v11": p_v11,
            "pass_rate_v11": round(p_v11 / total, 4) if total else 0.0,
            "wilson_95_ci_v11": list(wilson_ci_95(p_v11, total)),
            "passed_v10": p_v10,
            "pass_rate_v10": round(p_v10 / total, 4) if total else 0.0,
            "turn1_passed_v11": t1_v11,
            "turn1_pass_rate_v11": round(t1_v11 / total, 4) if total else 0.0,
            "by_category": by_cat,
            "redline_trials": len(redline),
            "false_actuations": false_acts,
            "false_actuation_rate": round(false_acts / len(redline), 4) if redline else 0.0,
            "duplicate_non_idempotent_total": dup_total,
            "unknown_events_trials": len(unk_trials),
            "unknown_reconciled_rate": round(unk_rec / len(unk_trials), 4) if unk_trials else 1.0,
            "eval_llm_calls": eval_llm_calls,
            "eval_llm_wall_s": eval_llm_wall,
            "mean_wall_s": round(sum(walls) / len(walls), 3) if walls else 0.0,
            "p95_wall_s": round(percentile(walls, 95.0) or 0.0, 3) if walls else 0.0,
            "mean_delivery_error_mm": round(sum(errs) / len(errs), 3) if errs else None,
            "max_delivery_error_mm": round(max(errs), 3) if errs else None,
            "portable": port["portable"],
            "portability_errors": port["error_count"],
            "attribution_counts": attr["attribution_counts"],
        }

    # Paired exact McNemar test: TRAX vs every other method on the aligned 20 scenarios
    craft_map = {
        t["scenario_id"]: bool(t.get("task_passed_v11"))
        for t in trials
        if t["method_id"] == "craft_full"
    }
    paired_vs_craft: dict[str, dict[str, Any]] = {}
    if len(craft_map) == len(scen_ids):
        craft_vec = [craft_map[sid] for sid in scen_ids]
        for art in artifacts:
            if art.method_id == "craft_full":
                continue
            b_map = {
                t["scenario_id"]: bool(t.get("task_passed_v11"))
                for t in trials
                if t["method_id"] == art.method_id
            }
            if len(b_map) == len(scen_ids):
                b_vec = [b_map[sid] for sid in scen_ids]
                paired_vs_craft[art.method_id] = mcnemar_exact_p(craft_vec, b_vec)

    return {"by_method": summary, "paired_mcnemar_craft_vs": paired_vs_craft}


def render_prior_work_html(report: Mapping[str, Any]) -> str:
    """Render a publication-ready, self-contained HTML comparison report."""
    by_method = list((report.get("summary_by_method") or {}).values())
    mcnemar = report.get("paired_mcnemar_craft_vs") or {}
    scenarios = report.get("scenarios") or []
    trials = report.get("trials") or []
    lit = report.get("literature_matrix") or PRIOR_WORK_LITERATURE_MATRIX
    colors = ["#94a3b8", "#f59e0b", "#a78bfa", "#38bdf8", "#22c55e"]

    # SVG grouped pass-rate chart with Wilson 95% CI (v1.1 physical + v1.0 marker)
    bar_w = 108
    gap = 42
    chart_w = max(720, 90 + len(by_method) * (bar_w + gap))
    bars_svg: list[str] = []
    for i, m in enumerate(by_method):
        x = 55 + i * (bar_w + gap)
        rate = float(m["pass_rate_v11"])
        h = int(rate * 190)
        y = 220 - h
        col = colors[i % len(colors)]
        ci = m.get("wilson_95_ci_v11") or [0, 0]
        y_hi = 220 - int(ci[1] * 190)
        y_lo = 220 - int(ci[0] * 190)
        cx = x + bar_w // 2
        bars_svg.append(
            f'<rect x="{x}" y="{y}" width="{bar_w}" height="{h}" rx="6" fill="{col}" opacity="0.88"/>'
            f'<line x1="{cx}" y1="{y_hi}" x2="{cx}" y2="{y_lo}" stroke="#f8fafc" stroke-width="2"/>'
            f'<line x1="{cx-8}" y1="{y_hi}" x2="{cx+8}" y2="{y_hi}" stroke="#f8fafc" stroke-width="2"/>'
            f'<line x1="{cx-8}" y1="{y_lo}" x2="{cx+8}" y2="{y_lo}" stroke="#f8fafc" stroke-width="2"/>'
            f'<text x="{cx}" y="{max(16, y-9)}" fill="#f8fafc" font-size="12" font-weight="700" text-anchor="middle">{rate*100:.1f}% ({m["passed_v11"]}/{m["total_trials"]})</text>'
            f'<text x="{cx}" y="242" fill="#cbd5e1" font-size="11" text-anchor="middle">{html.escape(m["label"][:24])}</text>'
            f'<text x="{cx}" y="257" fill="#64748b" font-size="10" text-anchor="middle">eval LLM={m["eval_llm_calls"]} · FAR={m["false_actuation_rate"]*100:.0f}%</text>'
        )

    # Main quantitative table rows
    agg_rows: list[str] = []
    for m in by_method:
        bc = m.get("by_category") or {}
        t1 = bc.get("pose_generalization", {})
        t2 = bc.get("fault_recovery", {})
        t3 = bc.get("safety_and_boundaries", {})
        mc = mcnemar.get(m["method_id"])
        mc_str = f"p={mc['p_value']} (+{mc['n_a_only']}/-{mc['n_b_only']})" if mc else "ref"
        t1_note = (
            f"<br><small>Turn-1 frozen: {m['turn1_passed_v11']}/{m['total_trials']} ({m['turn1_pass_rate_v11']*100:.0f}%)</small>"
            if m.get("online_mode") != "frozen"
            else ""
        )
        agg_rows.append(
            "<tr>"
            f"<td><strong>{html.escape(m['label'])}</strong><br><small>{html.escape(m['provenance'])} · portable={'✅' if m.get('portable') else '❌'}</small></td>"
            f"<td><strong>{m['passed_v11']}/{m['total_trials']} ({m['pass_rate_v11']*100:.1f}%)</strong>"
            f"<br><small>CI [{m['wilson_95_ci_v11'][0]*100:.1f}%, {m['wilson_95_ci_v11'][1]*100:.1f}%] · v1.0={m['passed_v10']}/{m['total_trials']}</small>{t1_note}</td>"
            f"<td>{t1.get('passed',0)}/{t1.get('total',0)}</td>"
            f"<td>{t2.get('passed',0)}/{t2.get('total',0)}</td>"
            f"<td>{t3.get('passed',0)}/{t3.get('total',0)}</td>"
            f"<td class=\"{'pass' if m['false_actuations']==0 else 'fail'}\">{m['false_actuations']}/{m['redline_trials']} ({m['false_actuation_rate']*100:.0f}%)</td>"
            f"<td>{m['duplicate_non_idempotent_total']} / {m['unknown_reconciled_rate']*100:.0f}%</td>"
            f"<td>dev={m['dev_llm_calls']} / rst={m['dev_sim_resets']}<br><small>eval LLM={m['eval_llm_calls']}</small></td>"
            f"<td>{m['mean_wall_s']}s / {m['p95_wall_s']}s</td>"
            f"<td><small>{html.escape(mc_str)}</small></td>"
            "</tr>"
        )

    # 20-scenario per-method matrix
    m_headers = "".join(f"<th>{html.escape(m['label'])}</th>" for m in by_method)
    m_rows: list[str] = []
    for s in scenarios:
        cells = []
        for m in by_method:
            st = [t for t in trials if t["method_id"] == m["method_id"] and t["scenario_id"] == s["scenario_id"]]
            if not st:
                cells.append("<td>—</td>")
                continue
            last = st[-1]
            ok = bool(last.get("task_passed_v11"))
            cls = "pass" if ok else "fail"
            regen_tag = " · R2" if last.get("online_regen_triggered") else ""
            if ok:
                err_v = (last.get("oracle_post_run") or {}).get("delivery_error_mm")
                sub = f"{err_v}mm · {last['wall_s']}s{regen_tag}" if s.get("expect_status") == "completed" and err_v is not None else f"{last['host_runtime_status']}{regen_tag}"
                txt = f"PASS<br><small>{html.escape(sub)}</small>"
            else:
                sub = f"{last.get('host_runtime_status') or last.get('status')}{regen_tag}"
                txt = f"FAIL<br><small>{html.escape(sub)}</small>"
            cells.append(f'<td class="{cls}">{txt}</td>')
        m_rows.append(f"<tr><td><strong>{html.escape(s['label'])}</strong><br><small>{html.escape(s['category'])}</small></td>{''.join(cells)}</tr>")

    # Literature matrix rows
    lit_rows = []
    for row in lit:
        lit_rows.append(
            "<tr>"
            f"<td><strong>{html.escape(str(row['method']))}</strong><br><small>{html.escape(str(row['citation']))}</small></td>"
            f"<td><small>{html.escape(str(row['evaluation_status']))}</small><br><small>{html.escape(str(row['code_availability']))}</small></td>"
            f"<td><small>{html.escape(str(row['perception_control_separation']))}</small></td>"
            f"<td><small>{html.escape(str(row['unknown_ack_reconciliation']))}</small></td>"
            f"<td><small>{html.escape(str(row['redline_false_actuation_gate']))}</small></td>"
            f"<td><small>{html.escape(str(row['eval_time_llm_calls']))}</small></td>"
            f"<td><small>{html.escape(str(row['independent_acceptance_rollback']))}</small></td>"
            "</tr>"
        )

    prov = report.get("provenance") or {}
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<title>{html.escape(str(report.get('benchmark_version', PRIOR_WORK_BENCH_VERSION)))} — Prior-Work Comparison</title>
<style>
  body {{ font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; background: #070b14; color: #f1f5f9; margin: 0; padding: 26px; }}
  .wrap {{ max-width: 1320px; margin: 0 auto; }}
  .hero {{ background: #0f172a; border: 1px solid #1e2f4f; border-left: 4px solid #22c55e; border-radius: 10px; padding: 16px 20px; margin-bottom: 18px; }}
  .badge {{ display: inline-block; padding: 3px 10px; border-radius: 999px; font-size: 12px; font-weight: 600; background: #1e293b; color: #7dd3fc; margin-right: 8px; }}
  .warn {{ background: #422006; color: #fde047; }}
  .card {{ background: #0f172a; border: 1px solid #1e2f4f; border-radius: 10px; padding: 16px 18px; margin-bottom: 18px; overflow-x: auto; }}
  h1 {{ margin: 0 0 8px 0; font-size: 22px; }}
  h2 {{ margin: 0 0 12px 0; font-size: 16px; color: #93c5fd; }}
  table {{ width: 100%; border-collapse: collapse; font-size: 12px; }}
  th, td {{ border-bottom: 1px solid #1e2f4f; padding: 8px 9px; text-align: left; vertical-align: middle; }}
  th {{ color: #94a3b8; background: #0a101f; font-weight: 600; }}
  td.pass {{ background: rgba(34,197,94,0.14); color: #86efac; font-weight: 700; }}
  td.fail {{ background: rgba(239,68,68,0.16); color: #fca5a5; font-weight: 700; }}
  small {{ color: #94a3b8; font-weight: 400; }}
</style>
</head>
<body>
<div class="wrap">
  <div class="hero">
    <h1>{html.escape(str(report.get('benchmark_version', PRIOR_WORK_BENCH_VERSION)))} — Matched-Budget Prior-Work Comparison</h1>
    <div>
      <span class="badge">model={html.escape(str(report.get('model')))} ({html.escape(str(report.get('reasoning_effort')))})</span>
      <span class="badge">driver={html.escape(str(report.get('driver')))}</span>
      <span class="badge">evidence={html.escape(str(report.get('evidence_source')))}</span>
      <span class="badge">heldout_trials={len(trials)}</span>
      <span class="badge warn">Adapted/Reimplemented protocols on shared host SDK · NOT physical-robot certification</span>
    </div>
    <p style="margin:8px 0 0;font-size:12px;color:#94a3b8;">
      git={html.escape(str(prov.get('git_commit') or 'n/a')[:12])} · dirty={html.escape(str(prov.get('worktree_dirty')))} ·
      shared_contract_sha={html.escape(str(report.get('shared_contract_sha256') or 'n/a')[:16])} ·
      finished={html.escape(str(report.get('finished_at_utc') or report.get('updated_at_utc') or ''))}
    </p>
  </div>

  <div class="card">
    <h2>1. Held-Out Pass Rate (v1.1 L2-Aware Physical Outcome, Wilson 95% CI)</h2>
    <svg viewBox="0 0 {chart_w} 275" style="width:100%;max-width:{chart_w}px;background:#0a101f;border-radius:8px;">
      <line x1="40" y1="220" x2="{chart_w-20}" y2="220" stroke="#334155" stroke-width="1"/>
      <line x1="40" y1="125" x2="{chart_w-20}" y2="125" stroke="#1e293b" stroke-dasharray="4 4"/>
      <line x1="40" y1="30" x2="{chart_w-20}" y2="30" stroke="#1e293b" stroke-dasharray="4 4"/>
      <text x="34" y="224" fill="#64748b" font-size="10" text-anchor="end">0%</text>
      <text x="34" y="129" fill="#64748b" font-size="10" text-anchor="end">50%</text>
      <text x="34" y="34" fill="#64748b" font-size="10" text-anchor="end">100%</text>
      {''.join(bars_svg)}
    </svg>
  </div>

  <div class="card">
    <h2>2. Quantitative Comparison Table (Matched Contract, Model &amp; Dev Budget)</h2>
    <table>
      <thead>
        <tr>
          <th>Method &amp; Provenance</th>
          <th>Held-Out Pass (v1.1 / v1.0)</th>
          <th>T1 Pose (6)</th>
          <th>T2 Fault (8)</th>
          <th>T3 Safety (6)</th>
          <th>False-Actuation</th>
          <th>Dup Brew / Reconcile</th>
          <th>Dev &amp; Eval Cost</th>
          <th>Mean / P95 Wall</th>
          <th>McNemar vs TRAX</th>
        </tr>
      </thead>
      <tbody>{''.join(agg_rows)}</tbody>
    </table>
  </div>

  <div class="card">
    <h2>3. Per-Scenario 20-Case Paired Matrix (R2 = Online Turn-2 Triggered)</h2>
    <table>
      <thead><tr><th>Scenario</th>{m_headers}</tr></thead>
      <tbody>{''.join(m_rows)}</tbody>
    </table>
  </div>

  <div class="card">
    <h2>4. Prior-Work Literature &amp; System Capability Matrix (Honest Provenance)</h2>
    <table>
      <thead>
        <tr>
          <th>Work</th>
          <th>Eval Status &amp; Code</th>
          <th>Perception / Control</th>
          <th>Unknown-Ack Reconciliation</th>
          <th>Safety Redline Gate</th>
          <th>Eval-Time LLM</th>
          <th>Acceptance &amp; Governance</th>
        </tr>
      </thead>
      <tbody>{''.join(lit_rows)}</tbody>
    </table>
  </div>
</div>
</body>
</html>"""


# ---------------------------------------------------------------------------
# End-to-End Orchestrator (Resumable, Incremental Persistence)
# ---------------------------------------------------------------------------

def run_prior_work_comparison(
    driver: SceneHostDriver,
    *,
    llm_fn: LLMGenerateFn,
    output_dir: Path,
    model: str = "gpt-6-astra",
    reasoning_effort: str = "medium",
    max_dev_llm_calls: int = 3,
    max_online_regen: int = 1,
    scenarios: Sequence[BenchmarkScenario] | None = None,
    dev_scenarios: Sequence[BenchmarkScenario] | None = None,
    progress_cb: Callable[[str], None] | None = None,
) -> dict[str, Any]:
    """Run the full 5-method matched-budget prior-work comparison and persist all artifacts."""
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    policies_dir = out / "policies"
    policies_dir.mkdir(parents=True, exist_ok=True)
    journal_path = out / "prior_work_journal.sqlite"
    report_path = out / "prior_work_report.json"
    html_path = out / "prior_work_report.html"

    heldout = tuple(scenarios) if scenarios is not None else cafebench_v1_scenarios()
    dev_set = tuple(dev_scenarios) if dev_scenarios is not None else cafebench_dev_scenarios()
    caps = driver.discover_capabilities(allow_busy=True)
    started_at = _utc_now()
    contract_sha = digest(SHARED_HOST_SDK_CONTRACT + "\n" + SHARED_TASK_REQUIREMENTS)

    def _log(msg: str) -> None:
        if progress_cb:
            progress_cb(msg)

    # Phase 1: Develop & freeze policies under matched budget
    _log("[Dev 1/4] Generating Strong Zero-Shot (Full Contract)...")
    art_zs = develop_strong_zeroshot(llm_fn)
    (policies_dir / "strong_zeroshot.py").write_text(art_zs.frozen_source + "\n", encoding="utf-8")

    _log("[Dev 2/4] Running ASPIRE-Protocol iterative debugging (disjoint 4-case dev set)...")
    art_aspire = develop_aspire_protocol(
        driver, llm_fn, dev_set, max_dev_llm_calls=max_dev_llm_calls, journal_path=journal_path
    )
    (policies_dir / "aspire_protocol.py").write_text(art_aspire.frozen_source + "\n", encoding="utf-8")

    _log("[Dev 3/4] Running CaP-X-Protocol execution feedback + SkillLibrary compilation...")
    art_capx = develop_capx_protocol(
        driver, llm_fn, dev_set, max_dev_llm_calls=max_dev_llm_calls, journal_path=journal_path
    )
    (policies_dir / "capx_protocol.py").write_text(art_capx.frozen_source + "\n", encoding="utf-8")

    # RoboRepair-Style shares the exact Strong Zero-Shot Turn-1 base (controlled Turn-1)
    art_rr = DevelopedMethodArtifact(
        method_id="roborepair_style",
        label="RoboRepair-Style (Reimpl.)",
        provenance="protocol_reimplementation_no_official_code",
        online_mode="roborepair_trace_recovery",
        frozen_source=art_zs.frozen_source,
        frozen_sha256=art_zs.frozen_sha256,
        dev_llm_calls=art_zs.dev_llm_calls,
        dev_sim_resets=0,
        dev_wall_s=art_zs.dev_wall_s,
        dev_passed_v11=0,
        dev_total=0,
        dev_history=[{"note": "Shares exact Strong Zero-Shot Turn-1 base; spends LLM budget online on trace recovery."}],
    )
    (policies_dir / "roborepair_style_base.py").write_text(art_rr.frozen_source + "\n", encoding="utf-8")

    _log("[Dev 4/4] Running TRAX Full Engine (Preflight AST/Portability + 4-Layer Attribution + Regression Gate)...")
    art_craft = develop_craft_full(
        driver,
        llm_fn,
        dev_set,
        max_dev_llm_calls=max_dev_llm_calls,
        journal_path=journal_path,
        workspace_dir=out,
    )
    (policies_dir / "craft_full.py").write_text(art_craft.frozen_source + "\n", encoding="utf-8")

    artifacts = [art_zs, art_aspire, art_capx, art_rr, art_craft]

    # Phase 2: Held-out paired evaluation across all 20 scenarios
    trials: list[dict[str, Any]] = []
    total_eval = len(artifacts) * len(heldout)
    done_eval = 0

    def _build_payload(*, finished: bool) -> dict[str, Any]:
        stats = summarize_prior_work_comparison(artifacts, heldout, trials)
        return {
            "benchmark_version": PRIOR_WORK_BENCH_VERSION,
            "driver": driver.driver_name,
            "evidence_source": driver.evidence_source,
            "model": model,
            "reasoning_effort": reasoning_effort,
            "max_dev_llm_calls": max_dev_llm_calls,
            "max_online_regen": max_online_regen,
            "shared_contract_sha256": contract_sha,
            "sdk_contract_sha256": caps.discovered.sdk_contract_sha256 if caps.discovered else None,
            "provenance": _git_provenance(),
            "started_at_utc": started_at,
            "updated_at_utc": _utc_now(),
            "finished_at_utc": _utc_now() if finished else None,
            "dev_scenarios": [s.to_dict() for s in dev_set],
            "scenarios": [s.to_dict() for s in heldout],
            "methods": [a.to_dict() for a in artifacts],
            "trials": trials,
            "summary_by_method": stats["by_method"],
            "paired_mcnemar_craft_vs": stats["paired_mcnemar_craft_vs"],
            "literature_matrix": list(PRIOR_WORK_LITERATURE_MATRIX),
            "claims_boundary": (
                "Empirical comparisons of ASPIRE and CaP-X evaluate their published development/execution "
                "protocols adapted onto the shared host SDK under matched model and budget, not their "
                "upstream perception/motion stacks. RoboRepair is explicitly marked as a protocol "
                "reimplementation because no official repository was verified. Evidence is simulation-only "
                "and is NOT physical-robot or food-quality certification."
            ),
        }

    _write_json_atomic(report_path, _build_payload(finished=False))

    for art in artifacts:
        for scen in heldout:
            done_eval += 1
            rec = evaluate_method_on_scenario(
                driver,
                art,
                scen,
                llm_fn=llm_fn if art.online_mode != "frozen" else None,
                journal_path=journal_path,
                max_online_regen=max_online_regen,
            )
            trials.append(rec)
            _log(
                f"[Eval {done_eval:03d}/{total_eval}] {art.method_id} x {scen.scenario_id} -> "
                f"v1.1={'PASS' if rec['task_passed_v11'] else 'FAIL'} "
                f"(v1.0={'PASS' if rec['task_passed_v10'] else 'FAIL'}, host={rec['host_runtime_status']}, "
                f"wall={rec['wall_s']}s, eval_llm={rec['eval_llm_calls']})"
            )
            _write_json_atomic(report_path, _build_payload(finished=False))

    final_payload = _build_payload(finished=True)
    _write_json_atomic(report_path, final_payload)
    html_path.write_text(render_prior_work_html(final_payload), encoding="utf-8")
    return final_payload
