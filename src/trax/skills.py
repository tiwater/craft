"""Verified skill experience & executable toolbox store with explicit applicability and drift invalidation.

Combines ASPIRE's independent skill verification discipline with SimEX's paired
``toolbox.py + skill.md`` persistence strictly inside TRAX's AST sandbox and
host-contract ownership boundaries:
- Textual guidance + ``measured_limits`` + ``known_failure_modes`` + ``regression_case_ids``
- Optional AST-sandbox-verified executable Python helper code (``code_snippet`` +
  ``entry_functions`` + ``code_sha256`` + ``preconditions`` + ``postconditions``)
- Strict binding to ``sdk_contract_sha256`` so contract drift automatically invalidates
  stale executable skills rather than silently reusing incompatible helpers.
"""
from __future__ import annotations

import ast
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

from .config import VALID_EVIDENCE_SOURCES
from .contracts import digest


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


_SKILL_FORBIDDEN_NODES = (
    ast.Import,
    ast.ImportFrom,
    ast.Try,
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


def validate_skill_code_snippet(code_snippet: str) -> dict[str, Any]:
    """Verify an executable skill helper snippet is AST-sandbox-safe and defines only pure helper functions."""
    src = code_snippet.strip()
    if not src:
        return {"valid": True, "entry_functions": [], "issues": [], "code_sha256": None}
    try:
        tree = ast.parse(src)
    except SyntaxError as exc:
        return {"valid": False, "entry_functions": [], "issues": [f"SyntaxError: {exc}"], "code_sha256": None}

    issues: list[str] = []
    entry_fns: list[str] = []
    for node in tree.body:
        if isinstance(node, ast.FunctionDef):
            if node.name.startswith("_"):
                issues.append(f"Forbidden leading-underscore function name: {node.name}")
            elif node.name == "run_policy":
                issues.append("Reusable skill helper must not define top-level `run_policy`")
            else:
                entry_fns.append(node.name)
        else:
            issues.append(f"Only top-level `def` helper functions are allowed in skill code (found {type(node).__name__})")

    for node in ast.walk(tree):
        if isinstance(node, _SKILL_FORBIDDEN_NODES):
            issues.append(f"Forbidden construct in skill code: {type(node).__name__}")
        elif isinstance(node, ast.Name) and node.id.startswith("_"):
            issues.append(f"Forbidden leading-underscore identifier: {node.id}")
        elif isinstance(node, ast.Attribute) and node.attr.startswith("_"):
            issues.append(f"Forbidden leading-underscore attribute: .{node.attr}")
        elif isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
            if node.func.id in {"exec", "eval", "compile", "open", "__import__", "getattr", "setattr", "delattr"}:
                issues.append(f"Forbidden builtin call in skill code: {node.func.id}()")

    if not entry_fns and not issues:
        issues.append("Skill code snippet defines no helper functions")

    return {
        "valid": len(issues) == 0,
        "entry_functions": entry_fns,
        "issues": issues,
        "code_sha256": digest(src) if not issues else None,
    }


@dataclass(frozen=True)
class SkillExperience:
    skill_id: str
    title: str
    guidance: str
    task_name: str
    required_methods: tuple[str, ...]
    sdk_contract_sha256: str | None
    evidence_source: str
    verification_report_id: str
    verified: bool
    verified_at_utc: str
    applicability_notes: str = ""
    measured_limits: Mapping[str, Any] = field(default_factory=dict)
    known_failure_modes: tuple[str, ...] = ()
    regression_case_ids: tuple[str, ...] = ()
    # P4-3: Optional verified executable helper toolbox fields (backward compatible)
    code_snippet: str = ""
    entry_functions: tuple[str, ...] = ()
    code_sha256: str | None = None
    preconditions: tuple[str, ...] = ()
    postconditions: tuple[str, ...] = ()
    invalidated: bool = False
    invalidation_reason: str = ""

    def __post_init__(self) -> None:
        if not self.skill_id.strip() or not self.title.strip() or not self.guidance.strip():
            raise ValueError("SkillExperience requires non-empty skill_id, title, and guidance")
        if self.evidence_source not in VALID_EVIDENCE_SOURCES:
            raise ValueError(f"evidence_source must be one of {sorted(VALID_EVIDENCE_SOURCES)}")
        if self.code_snippet.strip():
            chk = validate_skill_code_snippet(self.code_snippet)
            if not chk["valid"]:
                raise ValueError("Invalid skill code_snippet: " + "; ".join(chk["issues"]))
            expected_sha = chk["code_sha256"]
            if self.code_sha256 and self.code_sha256 != expected_sha:
                raise ValueError("Skill code_sha256 mismatch with code_snippet content")
            if not self.code_sha256:
                object.__setattr__(self, "code_sha256", expected_sha)
            if not self.entry_functions:
                object.__setattr__(self, "entry_functions", tuple(chk["entry_functions"]))

    def to_dict(self) -> dict[str, Any]:
        d: dict[str, Any] = {
            "skill_id": self.skill_id,
            "title": self.title,
            "guidance": self.guidance,
            "task_name": self.task_name,
            "required_methods": list(self.required_methods),
            "sdk_contract_sha256": self.sdk_contract_sha256,
            "evidence_source": self.evidence_source,
            "verification_report_id": self.verification_report_id,
            "verified": self.verified,
            "verified_at_utc": self.verified_at_utc,
            "applicability_notes": self.applicability_notes,
            "measured_limits": dict(self.measured_limits),
            "known_failure_modes": list(self.known_failure_modes),
            "regression_case_ids": list(self.regression_case_ids),
        }
        if self.code_snippet.strip():
            d["code_snippet"] = self.code_snippet
            d["entry_functions"] = list(self.entry_functions)
            d["code_sha256"] = self.code_sha256
        if self.preconditions:
            d["preconditions"] = list(self.preconditions)
        if self.postconditions:
            d["postconditions"] = list(self.postconditions)
        if self.invalidated:
            d["invalidated"] = True
            d["invalidation_reason"] = self.invalidation_reason
        return d

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> SkillExperience:
        return cls(
            skill_id=str(data["skill_id"]),
            title=str(data["title"]),
            guidance=str(data["guidance"]),
            task_name=str(data["task_name"]),
            required_methods=tuple(str(m) for m in (data.get("required_methods") or ())),
            sdk_contract_sha256=data.get("sdk_contract_sha256"),
            evidence_source=str(data["evidence_source"]),
            verification_report_id=str(data["verification_report_id"]),
            verified=bool(data["verified"]),
            verified_at_utc=str(data["verified_at_utc"]),
            applicability_notes=str(data.get("applicability_notes", "")),
            measured_limits=dict(data.get("measured_limits") or {}),
            known_failure_modes=tuple(str(x) for x in (data.get("known_failure_modes") or ())),
            regression_case_ids=tuple(str(x) for x in (data.get("regression_case_ids") or ())),
            code_snippet=str(data.get("code_snippet", "")),
            entry_functions=tuple(str(x) for x in (data.get("entry_functions") or ())),
            code_sha256=data.get("code_sha256"),
            preconditions=tuple(str(x) for x in (data.get("preconditions") or ())),
            postconditions=tuple(str(x) for x in (data.get("postconditions") or ())),
            invalidated=bool(data.get("invalidated", False)),
            invalidation_reason=str(data.get("invalidation_reason", "")),
        )


def compose_verified_toolbox_prelude(skills: Sequence[SkillExperience]) -> str:
    """Assemble deduplicated, AST-verified executable helper snippets for safe policy reuse.

    Only non-invalidated, verified skills with valid ``code_snippet`` are included.
    Duplicate function names are skipped (first verified entry wins) to avoid shadowing.
    """
    seen_fns: set[str] = set()
    blocks: list[str] = []
    for sk in skills:
        if not sk.verified or sk.invalidated or not sk.code_snippet.strip():
            continue
        if any(fn in seen_fns for fn in sk.entry_functions):
            continue
        chk = validate_skill_code_snippet(sk.code_snippet)
        if not chk["valid"]:
            continue
        seen_fns.update(chk["entry_functions"])
        blocks.append(f"# Verified Skill `{sk.skill_id}` ({sk.evidence_source}, report={sk.verification_report_id})\n{sk.code_snippet.strip()}")
    return "\n\n".join(blocks)


def inject_toolbox_into_policy(policy_source: str, skills: Sequence[SkillExperience]) -> str:
    """Prepend only those verified skill helper functions that are referenced but not already defined in ``policy_source``."""
    try:
        tree = ast.parse(policy_source)
    except SyntaxError:
        return policy_source
    defined = {n.name for n in tree.body if isinstance(n, ast.FunctionDef)}
    referenced = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    needed = [
        sk
        for sk in skills
        if sk.verified
        and not sk.invalidated
        and sk.code_snippet.strip()
        and any((fn in referenced and fn not in defined) for fn in sk.entry_functions)
    ]
    if not needed:
        return policy_source
    prelude = compose_verified_toolbox_prelude(needed)
    return f"{prelude}\n\n{policy_source.strip()}\n" if prelude else policy_source


class SkillExperienceStore:
    """Persistent JSON store of verified skill experiences & executable helper modules."""

    def __init__(self, store_path: str | Path) -> None:
        self.store_path = Path(store_path)
        if self.store_path.is_file():
            self._items: dict[str, dict[str, Any]] = json.loads(self.store_path.read_text(encoding="utf-8"))
        else:
            self._items = {}
            self._save()

    def _save(self) -> None:
        self.store_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.store_path.with_suffix(self.store_path.suffix + ".tmp")
        tmp.write_text(json.dumps(self._items, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        tmp.replace(self.store_path)

    def add(self, entry: SkillExperience) -> None:
        """Persist a skill experience entry. Unverified entries are retained but never queried as active."""
        self._items[entry.skill_id] = entry.to_dict()
        self._save()

    def invalidate_on_contract_drift(self, current_sdk_contract_sha256: str) -> list[str]:
        """Mark any stored skills bound to a different host SDK contract hash as invalidated."""
        invalidated_ids: list[str] = []
        changed = False
        for sid, raw in list(self._items.items()):
            bound_sha = raw.get("sdk_contract_sha256")
            if bound_sha and bound_sha != current_sdk_contract_sha256 and not raw.get("invalidated"):
                updated = dict(raw)
                updated["invalidated"] = True
                updated["invalidation_reason"] = (
                    f"Host SDK contract drift: skill bound to {bound_sha[:12]} != current {current_sdk_contract_sha256[:12]}"
                )
                self._items[sid] = updated
                invalidated_ids.append(sid)
                changed = True
        if changed:
            self._save()
        return invalidated_ids

    def list_verified(self, *, sdk_contract_sha256: str | None = None) -> list[SkillExperience]:
        """Return all active, verified skill experiences matching optional contract hash."""
        out: list[SkillExperience] = []
        for raw in self._items.values():
            skill = SkillExperience.from_dict(raw)
            if not skill.verified or skill.invalidated:
                continue
            if skill.sdk_contract_sha256 and sdk_contract_sha256 and skill.sdk_contract_sha256 != sdk_contract_sha256:
                continue
            out.append(skill)
        return out

    def query_applicable(
        self,
        *,
        task_name: str | None,
        available_methods: Sequence[str],
        sdk_contract_sha256: str | None = None,
        allow_cross_task: bool = False,
    ) -> list[SkillExperience]:
        """Return only active verified skills matching task (or cross-task if allowed), host capabilities, and contract hash."""
        avail = set(available_methods)
        matched: list[SkillExperience] = []
        for raw in self._items.values():
            skill = SkillExperience.from_dict(raw)
            if not skill.verified or skill.invalidated:
                continue
            if (not allow_cross_task) and task_name is not None and skill.task_name != task_name:
                continue
            if not set(skill.required_methods).issubset(avail):
                continue
            if skill.sdk_contract_sha256 and sdk_contract_sha256 and skill.sdk_contract_sha256 != sdk_contract_sha256:
                continue
            matched.append(skill)
        return matched

    def render_toolbox_markdown(self, skills: Sequence[SkillExperience] | None = None) -> str:
        """Render a concise SimEX-style paired skill+toolbox summary (guidance + limits + helper signatures)."""
        items = list(skills) if skills is not None else self.list_verified()
        if not items:
            return ""
        lines = ["Verified Skill Toolbox (applicable to current host SDK contract):"]
        for sk in items:
            lim = f" | measured_limits={json.dumps(sk.measured_limits, sort_keys=True)}" if sk.measured_limits else ""
            fm = f" | known_failure_modes={list(sk.known_failure_modes)}" if sk.known_failure_modes else ""
            fns = f" | helper_functions={list(sk.entry_functions)}" if sk.entry_functions else ""
            lines.append(f"- `{sk.skill_id}` ({sk.task_name}, {sk.evidence_source}): {sk.guidance}{lim}{fm}{fns}")
            if sk.code_snippet.strip():
                lines.append("```python")
                lines.append(sk.code_snippet.strip())
                lines.append("```")
        return "\n".join(lines)
