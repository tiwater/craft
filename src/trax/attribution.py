"""Embodiment-agnostic 4-layer failure attribution over governed MCP executions.

Restores the canonical `perception` attribution layer aligned with EvidenceScene v3.0
Spatial State Contract v1 (`docs/decisions/0004-spatial-state-contract-v1.md`) alongside
`policy`, `control`, `device`, `operator`, and `none`.
"""
from __future__ import annotations

from typing import Any, Mapping

from .sandbox import GovernedMCPProxy
from .spatial_contract import PERCEPTION_BLOCK_STATUSES


def _extract_field(obj: Any, key: str, default: Any = None) -> Any:
    if obj is None:
        return default
    if isinstance(obj, Mapping):
        return obj.get(key, default)
    return getattr(obj, key, default)


def classify_mcp_execution(
    *,
    ast_check: Mapping[str, Any],
    proxy: GovernedMCPProxy | None,
    verify_result: Mapping[str, Any] | None,
    runtime_error: str | None,
    aborted: bool = False,
    spatial_observation: Mapping[str, Any] | Any | None = None,
    revision_change: Mapping[str, Any] | Any | None = None,
) -> dict[str, Any]:
    """Classify an MCP execution into `{perception, policy, control, device, operator, none}`."""
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
        if getattr(proxy, "acted_after_perception_block", False):
            last_block = (getattr(proxy, "perception_blocks", None) or [{}])[-1]
            return {
                "layer": "policy",
                "code": "acted_on_unverified_perception",
                "recommended_owner": "trax_policy",
                "auto_repair_eligible": True,
                "summary": (
                    "Policy dispatched a non-idempotent actuation while a perception gate was unresolved "
                    f"({last_block.get('code', 'perception_block')}: {last_block.get('detail', '')})."
                ),
            }

    # Pre-actuation perception gate failure reported by Spatial State Runtime / EvidenceScene
    obs_status = str(_extract_field(spatial_observation, "status", "") or "").strip().lower()
    if obs_status in PERCEPTION_BLOCK_STATUSES:
        obs_detail = str(_extract_field(spatial_observation, "detail", "") or obs_status)
        return {
            "layer": "perception",
            "code": obs_status,
            "recommended_owner": "perception_owner",
            "auto_repair_eligible": False,
            "summary": obs_detail,
        }

    if proxy is not None and proxy.limit_refusals:
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

    # Distinguish post-run historical perception revision from control execution failure
    rev_cause = str(_extract_field(revision_change, "cause", "") or "").strip().lower()
    if rev_cause == "historical_correction":
        rev_detail = str(
            _extract_field(revision_change, "detail", "")
            or "Post-run EvidenceScene historical correction invalidated the prior spatial estimate."
        )
        return {
            "layer": "perception",
            "code": "historical_correction_invalidated",
            "recommended_owner": "perception_owner",
            "auto_repair_eligible": False,
            "summary": rev_detail,
        }
    if proxy is not None and getattr(proxy, "perception_blocks", None):
        last_block = proxy.perception_blocks[-1]
        return {
            "layer": "perception",
            "code": str(last_block.get("code") or "perception_block"),
            "recommended_owner": "perception_owner",
            "auto_repair_eligible": False,
            "summary": str(last_block.get("detail") or "Execution blocked by perception/observability gate."),
        }

    detail = str((verify_result or {}).get("detail") or "Task verification criteria not yet satisfied.")
    return {
        "layer": "control",
        "code": "task_verification_failed",
        "recommended_owner": "trax_policy",
        "auto_repair_eligible": True,
        "summary": detail,
    }
