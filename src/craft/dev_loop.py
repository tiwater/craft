"""Persistent, budget-controlled, resumable policy development loop.

Combines ACE-inspired on-demand teaching/clarification with ASPIRE-inspired
failure-guided revision and verified skill experience hints. All candidates,
prompts, material sources, trials, budgets, and failures are persisted atomically
on disk. Software fixture runs and real model/simulator runs are explicitly tagged.
"""
from __future__ import annotations

import ast
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import json
from pathlib import Path
import time
from typing import Any, Callable, Mapping, Sequence

from .backends import invoke_backend
from .config import SceneConfig, VALID_EVIDENCE_SOURCES
from .contracts import canonical, digest
from .teaching import ClarificationRequest, TaskSpecification


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _write_json_atomic(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    tmp.replace(path)


def _strip_code_fence(raw: str) -> str:
    text = raw.strip()
    if text.startswith("```python\n") and text.endswith("```"):
        return text[len("```python\n") : -3].strip()
    if text.startswith("```\n") and text.endswith("```"):
        return text[len("```\n") : -3].strip()
    return text


def validate_host_policy_syntax(source: str) -> tuple[bool, str]:
    """Syntax-only check that source parses as Python and defines run_policy.

    Syntax validation is NOT hardware approval or safety verification.
    """
    try:
        tree = ast.parse(source)
    except SyntaxError as exc:
        return False, f"SyntaxError: {exc}"
    has_entry = any(
        isinstance(node, ast.FunctionDef) and node.name == "run_policy"
        for node in tree.body
    )
    if not has_entry:
        return False, "Policy must define top-level function def run_policy(sdk, params)"
    return True, ""


@dataclass(frozen=True)
class CandidateRecord:
    candidate_id: str
    iteration: int
    source_sha256: str
    source_path: str
    prompt_sha256: str
    material_ids: tuple[str, ...]
    skill_ids_used: tuple[str, ...]
    provider: str
    model: str
    reasoning_effort: str
    evidence_source: str
    syntax_valid: bool
    validation_error: str
    created_at_utc: str
    approved: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "candidate_id": self.candidate_id,
            "iteration": self.iteration,
            "source_sha256": self.source_sha256,
            "source_path": self.source_path,
            "prompt_sha256": self.prompt_sha256,
            "material_ids": list(self.material_ids),
            "skill_ids_used": list(self.skill_ids_used),
            "provider": self.provider,
            "model": self.model,
            "reasoning_effort": self.reasoning_effort,
            "evidence_source": self.evidence_source,
            "syntax_valid": self.syntax_valid,
            "validation_error": self.validation_error,
            "created_at_utc": self.created_at_utc,
            "approved": self.approved,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> CandidateRecord:
        return cls(
            candidate_id=str(data["candidate_id"]),
            iteration=int(data["iteration"]),
            source_sha256=str(data["source_sha256"]),
            source_path=str(data["source_path"]),
            prompt_sha256=str(data["prompt_sha256"]),
            material_ids=tuple(str(x) for x in (data.get("material_ids") or ())),
            skill_ids_used=tuple(str(x) for x in (data.get("skill_ids_used") or ())),
            provider=str(data["provider"]),
            model=str(data["model"]),
            reasoning_effort=str(data["reasoning_effort"]),
            evidence_source=str(data["evidence_source"]),
            syntax_valid=bool(data["syntax_valid"]),
            validation_error=str(data.get("validation_error", "")),
            created_at_utc=str(data["created_at_utc"]),
            approved=bool(data.get("approved", False)),
        )


@dataclass(frozen=True)
class TrialRecord:
    trial_id: str
    candidate_id: str
    condition: str
    evidence_source: str  # fixture | simulator | recorded_data | physical_robot
    status: str
    passed: bool
    wall_s: float
    non_idempotent_counts: Mapping[str, int] = field(default_factory=dict)
    duplicate_non_idempotent: bool = False
    unknown_events_observed: int = 0
    queried_after_unknown: bool = True
    human_interventions: int = 0
    delivery_error_mm: float | None = None
    failure_detail: str = ""
    needs_clarification: ClarificationRequest | None = None
    recorded_at_utc: str = field(default_factory=_utc_now)

    def __post_init__(self) -> None:
        if self.evidence_source not in VALID_EVIDENCE_SOURCES:
            raise ValueError(f"TrialRecord.evidence_source must be one of {sorted(VALID_EVIDENCE_SOURCES)}")

    def to_dict(self) -> dict[str, Any]:
        return {
            "trial_id": self.trial_id,
            "candidate_id": self.candidate_id,
            "condition": self.condition,
            "evidence_source": self.evidence_source,
            "status": self.status,
            "passed": self.passed,
            "wall_s": round(self.wall_s, 3),
            "non_idempotent_counts": dict(self.non_idempotent_counts),
            "duplicate_non_idempotent": self.duplicate_non_idempotent,
            "unknown_events_observed": self.unknown_events_observed,
            "queried_after_unknown": self.queried_after_unknown,
            "human_interventions": self.human_interventions,
            "delivery_error_mm": self.delivery_error_mm,
            "failure_detail": self.failure_detail,
            "needs_clarification": self.needs_clarification.to_dict() if self.needs_clarification else None,
            "recorded_at_utc": self.recorded_at_utc,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> TrialRecord:
        clar_raw = data.get("needs_clarification")
        return cls(
            trial_id=str(data["trial_id"]),
            candidate_id=str(data["candidate_id"]),
            condition=str(data["condition"]),
            evidence_source=str(data["evidence_source"]),
            status=str(data["status"]),
            passed=bool(data["passed"]),
            wall_s=float(data["wall_s"]),
            non_idempotent_counts=dict(data.get("non_idempotent_counts") or {}),
            duplicate_non_idempotent=bool(data.get("duplicate_non_idempotent", False)),
            unknown_events_observed=int(data.get("unknown_events_observed", 0)),
            queried_after_unknown=bool(data.get("queried_after_unknown", True)),
            human_interventions=int(data.get("human_interventions", 0)),
            delivery_error_mm=data.get("delivery_error_mm"),
            failure_detail=str(data.get("failure_detail", "")),
            needs_clarification=ClarificationRequest.from_dict(clar_raw) if clar_raw else None,
            recorded_at_utc=str(data.get("recorded_at_utc") or _utc_now()),
        )


def build_development_prompt(
    config: SceneConfig,
    spec: TaskSpecification,
    *,
    prior_trials: Sequence[TrialRecord] = (),
    skill_hints: Sequence[Mapping[str, Any]] = (),
) -> str:
    """Construct a traceable policy-generation prompt grounded in host capabilities and real feedback."""
    failed_feedback = [
        {
            "trial_id": t.trial_id,
            "candidate_id": t.candidate_id,
            "condition": t.condition,
            "status": t.status,
            "failure_detail": t.failure_detail,
            "duplicate_non_idempotent": t.duplicate_non_idempotent,
            "unknown_events_observed": t.unknown_events_observed,
            "queried_after_unknown": t.queried_after_unknown,
        }
        for t in prior_trials
        if not t.passed
    ]
    payload = {
        "task_name": spec.task_name,
        "instruction": spec.instruction,
        "allowed_host_methods": list(config.capabilities.required_methods),
        "non_idempotent_methods": list(config.capabilities.non_idempotent_methods),
        "object_bindings": [
            {"name": o.name, "semantic_classes": list(o.semantic_classes), "required_count": o.required_count}
            for o in config.objects
        ],
        "steps": [s.to_dict() for s in spec.steps],
        "teaching_materials": [m.to_dict() for m in spec.materials],
        "evidence_notes": list(spec.evidence_notes),
        "verified_skill_hints": list(skill_hints),
        "prior_failure_feedback": failed_feedback,
    }
    header = (
        "Write only Python defining def run_policy(sdk, params).\n"
        "Use only the allowed_host_methods on sdk. Never invent capability names or scene coordinates.\n"
        "Live scene state comes exclusively from sdk.observe_scene(); video materials are non-authoritative.\n"
        "For non-idempotent methods (e.g. start_coffee_machine), if an acknowledgement is unknown/timed out, "
        "always call sdk.query_command_status(command_id) and never blindly reissue the command.\n"
        "Generated code is an unapproved draft subject to independent acceptance verification.\n"
        "Task context (JSON):\n"
    )
    return header + canonical(payload)


def default_live_generator(config: SceneConfig, prompt: str) -> dict[str, Any]:
    """Invoke the configured live LLM backend (codex_cli or openai_api)."""
    res = invoke_backend(
        config.generation.provider,
        prompt,
        config.generation.timeout_s,
        model=config.generation.model or None,
        reasoning_effort=config.generation.reasoning_effort,
    )
    return {
        "source": _strip_code_fence(res["raw_output"]),
        "provider": config.generation.provider,
        "model": res["model"],
        "reasoning_effort": res["reasoning_effort"],
        "evidence_source": config.budget.deployment_scope,
    }


GeneratorFn = Callable[[SceneConfig, str], Mapping[str, Any]]
EvaluatorFn = Callable[[str, str, str], TrialRecord]


class DevelopmentLoop:
    """Durable, resumable policy development session manager."""

    def __init__(self, workspace_dir: str | Path, config: SceneConfig, state: dict[str, Any]) -> None:
        self.workspace_dir = Path(workspace_dir)
        self.config = config
        self.state = state

    @property
    def session_path(self) -> Path:
        return self.workspace_dir / "session.json"

    @classmethod
    def create_or_resume(
        cls,
        workspace_dir: str | Path,
        config: SceneConfig,
        spec: TaskSpecification,
        *,
        session_id: str | None = None,
    ) -> DevelopmentLoop:
        root = Path(workspace_dir)
        session_file = root / "session.json"
        if session_file.is_file():
            saved = json.loads(session_file.read_text(encoding="utf-8"))
            if saved.get("config_sha256") != config.config_sha256:
                raise ValueError(
                    "Session identity drift: workspace config_sha256 does not match supplied SceneConfig"
                )
            return cls(root, config, saved)

        initial_status = "initialized"
        if spec.status == "awaiting_teaching":
            initial_status = "awaiting_teaching"
        elif spec.status == "blocked_missing_capability":
            initial_status = "blocked"

        state = {
            "session_id": session_id or f"dev-{config.task_name}-{config.config_sha256[:12]}",
            "config_sha256": config.config_sha256,
            "spec_sha256": spec.spec_sha256,
            "spec": spec.to_dict(),
            "status": initial_status,
            "budget_used": {"model_calls": 0, "trials": 0, "wall_s": 0.0},
            "candidates": [],
            "trials": [],
            "pending_clarifications": [c.to_dict() for c in spec.clarifications],
            "active_candidate_id": None,
            "created_at_utc": _utc_now(),
            "updated_at_utc": _utc_now(),
        }
        loop = cls(root, config, state)
        loop._save()
        return loop

    def _save(self) -> None:
        self.state["updated_at_utc"] = _utc_now()
        _write_json_atomic(self.session_path, self.state)

    @property
    def status(self) -> str:
        return str(self.state["status"])

    @property
    def candidates(self) -> list[CandidateRecord]:
        return [CandidateRecord.from_dict(c) for c in self.state["candidates"]]

    @property
    def trials(self) -> list[TrialRecord]:
        return [TrialRecord.from_dict(t) for t in self.state["trials"]]

    def get_candidate_source(self, candidate_id: str) -> str:
        for c in self.candidates:
            if c.candidate_id == candidate_id:
                return (self.workspace_dir / c.source_path).read_text(encoding="utf-8")
        raise KeyError(f"Unknown candidate_id: {candidate_id}")

    def pause(self) -> None:
        if self.status in {"initialized", "running"}:
            self.state["status"] = "paused"
            self._save()

    def resume(self) -> None:
        if self.status == "paused":
            self.state["status"] = "running"
            self._save()

    def supply_teaching(self, updated_spec: TaskSpecification) -> None:
        """Resume an awaiting_teaching session with updated operator materials/steps."""
        self.state["spec"] = updated_spec.to_dict()
        self.state["spec_sha256"] = updated_spec.spec_sha256
        self.state["pending_clarifications"] = [c.to_dict() for c in updated_spec.clarifications]
        if updated_spec.status == "ready":
            self.state["status"] = "running"
        elif updated_spec.status == "blocked_missing_capability":
            self.state["status"] = "blocked"
        else:
            self.state["status"] = "awaiting_teaching"
        self._save()

    def _budget_exhausted(self, *, need_model_call: bool = False, need_trial: bool = False) -> bool:
        used = self.state["budget_used"]
        if used["wall_s"] >= self.config.budget.max_wall_s:
            return True
        if need_model_call and used["model_calls"] >= self.config.budget.max_model_calls:
            return True
        if need_trial and used["trials"] >= self.config.budget.max_trials:
            return True
        return False

    def run_iteration(
        self,
        *,
        generator_fn: GeneratorFn | None = None,
        evaluator_fn: EvaluatorFn,
        conditions: Sequence[str] = ("normal",),
        skill_hints: Sequence[Mapping[str, Any]] = (),
    ) -> dict[str, Any]:
        """Run one generate -> syntax-check -> evaluate iteration with full persistence."""
        if self.status in {"awaiting_teaching", "blocked", "paused", "budget_exhausted"}:
            return {"status": self.status, "candidate_id": None, "trials": []}

        if self._budget_exhausted(need_model_call=True):
            self.state["status"] = "budget_exhausted"
            self._save()
            return {"status": "budget_exhausted", "candidate_id": None, "trials": []}

        self.state["status"] = "running"
        spec = TaskSpecification.from_dict(self.state["spec"])
        prompt = build_development_prompt(
            self.config,
            spec,
            prior_trials=self.trials,
            skill_hints=skill_hints,
        )
        prompt_hash = digest(prompt)

        gen = generator_fn or default_live_generator
        started = time.monotonic()
        gen_result = gen(self.config, prompt)
        gen_wall = time.monotonic() - started
        self.state["budget_used"]["model_calls"] += 1
        self.state["budget_used"]["wall_s"] = round(self.state["budget_used"]["wall_s"] + gen_wall, 3)

        source = _strip_code_fence(str(gen_result["source"]))
        iteration = len(self.state["candidates"]) + 1
        candidate_id = f"cand_{iteration:03d}"
        rel_source_path = f"candidates/{candidate_id}.py"
        abs_source_path = self.workspace_dir / rel_source_path
        abs_source_path.parent.mkdir(parents=True, exist_ok=True)
        abs_source_path.write_text(source + "\n", encoding="utf-8")

        syntax_ok, val_error = validate_host_policy_syntax(source)
        cand = CandidateRecord(
            candidate_id=candidate_id,
            iteration=iteration,
            source_sha256=digest(source),
            source_path=rel_source_path,
            prompt_sha256=prompt_hash,
            material_ids=tuple(m.material_id for m in spec.materials),
            skill_ids_used=tuple(str(h.get("skill_id", "")) for h in skill_hints if h.get("skill_id")),
            provider=str(gen_result.get("provider", self.config.generation.provider)),
            model=str(gen_result.get("model", self.config.generation.model)),
            reasoning_effort=str(gen_result.get("reasoning_effort", self.config.generation.reasoning_effort)),
            evidence_source=str(gen_result.get("evidence_source", self.config.budget.deployment_scope)),
            syntax_valid=syntax_ok,
            validation_error=val_error,
            created_at_utc=_utc_now(),
            approved=False,
        )
        self.state["candidates"].append(cand.to_dict())
        _write_json_atomic(
            self.workspace_dir / f"candidates/{candidate_id}.json",
            {**cand.to_dict(), "prompt": prompt},
        )
        self._save()

        if not syntax_ok:
            # Retain syntax-failed candidate and record a synthetic trial entry for feedback
            trial_id = f"trial_{len(self.state['trials']) + 1:03d}"
            fail_trial = TrialRecord(
                trial_id=trial_id,
                candidate_id=candidate_id,
                condition="syntax_check",
                evidence_source=cand.evidence_source,
                status="syntax_error",
                passed=False,
                wall_s=0.0,
                failure_detail=val_error,
            )
            self.state["trials"].append(fail_trial.to_dict())
            _write_json_atomic(self.workspace_dir / f"trials/{trial_id}.json", fail_trial.to_dict())
            self._save()
            return {"status": "running", "candidate_id": candidate_id, "trials": [fail_trial.to_dict()]}

        iter_trials: list[TrialRecord] = []
        for condition in conditions:
            if self._budget_exhausted(need_trial=True):
                self.state["status"] = "budget_exhausted"
                self._save()
                return {
                    "status": "budget_exhausted",
                    "candidate_id": candidate_id,
                    "trials": [t.to_dict() for t in iter_trials],
                }
            trial = evaluator_fn(candidate_id, source, condition)
            self.state["budget_used"]["trials"] += 1
            self.state["budget_used"]["wall_s"] = round(self.state["budget_used"]["wall_s"] + trial.wall_s, 3)
            self.state["trials"].append(trial.to_dict())
            _write_json_atomic(self.workspace_dir / f"trials/{trial.trial_id}.json", trial.to_dict())
            iter_trials.append(trial)
            self._save()

            if trial.needs_clarification is not None:
                self.state["pending_clarifications"].append(trial.needs_clarification.to_dict())
                self.state["status"] = "awaiting_teaching"
                self._save()
                return {
                    "status": "awaiting_teaching",
                    "candidate_id": candidate_id,
                    "trials": [t.to_dict() for t in iter_trials],
                }

        all_passed = bool(iter_trials) and all(
            t.passed and not t.duplicate_non_idempotent and (t.unknown_events_observed == 0 or t.queried_after_unknown)
            for t in iter_trials
        )
        if all_passed:
            self.state["active_candidate_id"] = candidate_id
            self.state["status"] = "completed"
            self._save()

        return {
            "status": self.status,
            "candidate_id": candidate_id,
            "trials": [t.to_dict() for t in iter_trials],
        }
