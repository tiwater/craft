"""Verified skill experience store with explicit applicability conditions and provenance.

Informed by ASPIRE's independent skill verification discipline: proposed skill notes
are not promoted into active development hints until independently verified against
a concrete acceptance/trial record, and they remain scoped to their host capability
contract and applicability conditions.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

from .config import VALID_EVIDENCE_SOURCES


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


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

    def __post_init__(self) -> None:
        if not self.skill_id.strip() or not self.title.strip() or not self.guidance.strip():
            raise ValueError("SkillExperience requires non-empty skill_id, title, and guidance")
        if self.evidence_source not in VALID_EVIDENCE_SOURCES:
            raise ValueError(f"evidence_source must be one of {sorted(VALID_EVIDENCE_SOURCES)}")

    def to_dict(self) -> dict[str, Any]:
        return {
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
        }

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
        )


class SkillExperienceStore:
    """Persistent JSON store of verified skill experiences."""

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

    def query_applicable(
        self,
        *,
        task_name: str,
        available_methods: Sequence[str],
        sdk_contract_sha256: str | None = None,
    ) -> list[SkillExperience]:
        """Return only verified skills matching task, available host capabilities, and contract hash."""
        avail = set(available_methods)
        matched: list[SkillExperience] = []
        for raw in self._items.values():
            skill = SkillExperience.from_dict(raw)
            if not skill.verified:
                continue
            if skill.task_name != task_name:
                continue
            if not set(skill.required_methods).issubset(avail):
                continue
            if skill.sdk_contract_sha256 and sdk_contract_sha256 and skill.sdk_contract_sha256 != sdk_contract_sha256:
                continue
            matched.append(skill)
        return matched
