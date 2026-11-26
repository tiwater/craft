"""Embodiment-agnostic 4-layer failure attribution over governed MCP executions."""
from __future__ import annotations

from typing import Any, Mapping

from .sandbox import GovernedMCPProxy


def classify_mcp_execution(
    *,
    ast_check: Mapping[str, Any],
    proxy: GovernedMCPProxy | None,
    verify_result: Mapping[str, Any] | None,
    runtime_error: str | None,
    aborted: bool = False,
) -> dict[str, Any]:
    """Classify an MCP execution into {policy, control, operator, none} with owner attribution."""
    if aborted:
        return {
            "layer": "operator",
            "code": "aborted_by_operator",
            "recommended_owner": "operator",
            "auto_repair_eligible": False,
            "summary": runtime_error or "Execution aborted by operator / supervisor.",
        }
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
