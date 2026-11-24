"""Unified teaching interaction: text, video evidence, and host-contract trajectory demos.

Video inputs are strictly non-authoritative teaching evidence and never supply live
scene coordinates. Robot trajectory demonstrations must reference an existing host
contract ID. When required information or teaching materials are missing, TRAX
emits explicit ClarificationRequest objects and waits rather than fabricating demos.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Mapping, Sequence

from .config import SceneConfig
from .contracts import digest

VALID_MATERIAL_KINDS = frozenset({"text", "video", "robot_trajectory"})


@dataclass(frozen=True)
class TeachingMaterial:
    """One operator-supplied teaching artifact."""

    material_id: str
    kind: str  # text | video | robot_trajectory
    summary: str
    source_ref: str = ""
    contract_id: str | None = None
    notes: str = ""

    def __post_init__(self) -> None:
        if not self.material_id.strip():
            raise ValueError("material_id must be non-empty")
        if self.kind not in VALID_MATERIAL_KINDS:
            raise ValueError(f"TeachingMaterial.kind must be one of {sorted(VALID_MATERIAL_KINDS)}")
        if not self.summary.strip():
            raise ValueError("TeachingMaterial.summary must be non-empty")

    @property
    def authoritative_scene_state(self) -> bool:
        """Teaching materials are never live scene authority."""
        return False

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["authoritative_scene_state"] = False
        return data

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> TeachingMaterial:
        return cls(
            material_id=str(data["material_id"]),
            kind=str(data["kind"]),
            summary=str(data["summary"]),
            source_ref=str(data.get("source_ref", "")),
            contract_id=data.get("contract_id"),
            notes=str(data.get("notes", "")),
        )


@dataclass(frozen=True)
class TaskStep:
    """One structured step in a task specification."""

    step_id: str
    description: str
    action_binding: str
    preconditions: tuple[str, ...] = ()
    completion_evidence: tuple[str, ...] = ()
    failure_handling: str = "escalate_to_human_operator"
    non_idempotent: bool = False

    def __post_init__(self) -> None:
        if not self.step_id.strip() or not self.description.strip() or not self.action_binding.strip():
            raise ValueError("TaskStep requires non-empty step_id, description, and action_binding")

    def to_dict(self) -> dict[str, Any]:
        return {
            "step_id": self.step_id,
            "description": self.description,
            "action_binding": self.action_binding,
            "preconditions": list(self.preconditions),
            "completion_evidence": list(self.completion_evidence),
            "failure_handling": self.failure_handling,
            "non_idempotent": self.non_idempotent,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> TaskStep:
        return cls(
            step_id=str(data["step_id"]),
            description=str(data["description"]),
            action_binding=str(data["action_binding"]),
            preconditions=tuple(str(x) for x in (data.get("preconditions") or ())),
            completion_evidence=tuple(str(x) for x in (data.get("completion_evidence") or ())),
            failure_handling=str(data.get("failure_handling", "escalate_to_human_operator")),
            non_idempotent=bool(data.get("non_idempotent", False)),
        )


@dataclass(frozen=True)
class ClarificationRequest:
    """Explicit request for missing operator information or teaching materials."""

    request_id: str
    reason: str
    question: str
    required_kind: str  # text | video | robot_trajectory | capability | object_binding
    blocking: bool = True

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> ClarificationRequest:
        return cls(
            request_id=str(data["request_id"]),
            reason=str(data["reason"]),
            question=str(data["question"]),
            required_kind=str(data["required_kind"]),
            blocking=bool(data.get("blocking", True)),
        )


@dataclass(frozen=True)
class TaskSpecification:
    """Structured task plan derived from operator instruction and teaching materials."""

    task_name: str
    instruction: str
    materials: tuple[TeachingMaterial, ...]
    steps: tuple[TaskStep, ...]
    clarifications: tuple[ClarificationRequest, ...] = ()
    status: str = "ready"  # ready | awaiting_teaching | blocked_missing_capability
    evidence_notes: tuple[str, ...] = ()

    @property
    def spec_sha256(self) -> str:
        return digest(self.to_dict())

    def to_dict(self) -> dict[str, Any]:
        return {
            "task_name": self.task_name,
            "instruction": self.instruction,
            "materials": [m.to_dict() for m in self.materials],
            "steps": [s.to_dict() for s in self.steps],
            "clarifications": [c.to_dict() for c in self.clarifications],
            "status": self.status,
            "evidence_notes": list(self.evidence_notes),
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> TaskSpecification:
        return cls(
            task_name=str(data["task_name"]),
            instruction=str(data["instruction"]),
            materials=tuple(TeachingMaterial.from_dict(m) for m in (data.get("materials") or ())),
            steps=tuple(TaskStep.from_dict(s) for s in (data.get("steps") or ())),
            clarifications=tuple(ClarificationRequest.from_dict(c) for c in (data.get("clarifications") or ())),
            status=str(data.get("status", "ready")),
            evidence_notes=tuple(str(n) for n in (data.get("evidence_notes") or ())),
        )


def build_task_specification(
    config: SceneConfig,
    instruction: str,
    *,
    materials: Sequence[TeachingMaterial] = (),
    steps: Sequence[TaskStep] = (),
    discovered_methods: Sequence[str] | None = None,
) -> TaskSpecification:
    """Validate operator teaching inputs against scene config and discovered host capabilities.

    Never fabricates demonstrations, steps, or authoritative coordinates.
    """
    clarifications: list[ClarificationRequest] = []
    evidence_notes: list[str] = []
    missing_capability = False

    if not instruction or not instruction.strip():
        clarifications.append(
            ClarificationRequest(
                request_id="clarify-instruction",
                reason="missing_instruction",
                question="Please provide a text description of the task goal and expected sequence.",
                required_kind="text",
            )
        )

    for mat in materials:
        if mat.kind == "video":
            evidence_notes.append(
                f"Material {mat.material_id!r} (video) is teaching evidence only; "
                "live coordinates must come exclusively from EvidenceScene via sdk.observe_scene()."
            )
        elif mat.kind == "robot_trajectory":
            if not mat.contract_id or not mat.contract_id.strip():
                clarifications.append(
                    ClarificationRequest(
                        request_id=f"clarify-trajectory-contract-{mat.material_id}",
                        reason="missing_trajectory_contract",
                        question=(
                            f"Robot trajectory demonstration {mat.material_id!r} must specify "
                            "the existing host contract_id before it can be bound."
                        ),
                        required_kind="robot_trajectory",
                    )
                )

    if not steps:
        clarifications.append(
            ClarificationRequest(
                request_id="clarify-steps",
                reason="missing_task_steps",
                question=(
                    "Please supply structured task steps (action_binding, preconditions, "
                    "completion_evidence, failure_handling) or an approved demonstration."
                ),
                required_kind="text",
            )
        )

    available = set(discovered_methods) if discovered_methods is not None else set(config.capabilities.required_methods)
    for step in steps:
        if step.action_binding not in available:
            missing_capability = True
            clarifications.append(
                ClarificationRequest(
                    request_id=f"clarify-capability-{step.step_id}",
                    reason="missing_host_capability",
                    question=(
                        f"Step {step.step_id!r} requires host capability {step.action_binding!r}, "
                        "which is not present in the discovered host SDK."
                    ),
                    required_kind="capability",
                )
            )
        if not step.completion_evidence:
            clarifications.append(
                ClarificationRequest(
                    request_id=f"clarify-evidence-{step.step_id}",
                    reason="missing_completion_evidence",
                    question=f"Step {step.step_id!r} lacks explicit completion_evidence.",
                    required_kind="text",
                )
            )

    if missing_capability:
        status = "blocked_missing_capability"
    elif clarifications:
        status = "awaiting_teaching"
    else:
        status = "ready"

    return TaskSpecification(
        task_name=config.task_name,
        instruction=instruction.strip(),
        materials=tuple(materials),
        steps=tuple(steps),
        clarifications=tuple(clarifications),
        status=status,
        evidence_notes=tuple(evidence_notes),
    )
