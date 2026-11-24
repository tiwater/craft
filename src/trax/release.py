"""Fixed-version release registry, rollback, regression gating, and optimization queue.

Published policies execute as frozen versions; silent in-service rewriting is forbidden.
Any policy edit requires a fresh passing independent AcceptanceReport before promotion.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any, Mapping

from .acceptance import AcceptanceReport, FrozenArtifact, compare_improvement_candidate
from .contracts import digest


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _write_json_atomic(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    tmp.replace(path)


@dataclass(frozen=True)
class ReleaseVersion:
    version: str
    policy_sha256: str
    config_sha256: str
    sdk_contract_sha256: str
    criteria_sha256: str
    deployment_scope: str
    report_id: str
    policy_path: str
    report_path: str
    published_at_utc: str
    notes: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "version": self.version,
            "policy_sha256": self.policy_sha256,
            "config_sha256": self.config_sha256,
            "sdk_contract_sha256": self.sdk_contract_sha256,
            "criteria_sha256": self.criteria_sha256,
            "deployment_scope": self.deployment_scope,
            "report_id": self.report_id,
            "policy_path": self.policy_path,
            "report_path": self.report_path,
            "published_at_utc": self.published_at_utc,
            "notes": self.notes,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> ReleaseVersion:
        return cls(
            version=str(data["version"]),
            policy_sha256=str(data["policy_sha256"]),
            config_sha256=str(data["config_sha256"]),
            sdk_contract_sha256=str(data["sdk_contract_sha256"]),
            criteria_sha256=str(data["criteria_sha256"]),
            deployment_scope=str(data["deployment_scope"]),
            report_id=str(data["report_id"]),
            policy_path=str(data["policy_path"]),
            report_path=str(data["report_path"]),
            published_at_utc=str(data["published_at_utc"]),
            notes=str(data.get("notes", "")),
        )


class ReleaseRegistry:
    """Durable fixed-version release registry with rollback and optimization queue."""

    def __init__(self, root_dir: str | Path) -> None:
        self.root_dir = Path(root_dir)
        self.index_path = self.root_dir / "registry.json"
        if self.index_path.is_file():
            self._state = json.loads(self.index_path.read_text(encoding="utf-8"))
        else:
            self._state = {
                "active_version": None,
                "releases": {},
                "history": [],
                "optimization_queue": [],
            }
            self._save()

    def _save(self) -> None:
        _write_json_atomic(self.index_path, self._state)

    @property
    def active_version_id(self) -> str | None:
        return self._state.get("active_version")

    @property
    def active_release(self) -> ReleaseVersion | None:
        vid = self.active_version_id
        if not vid:
            return None
        return ReleaseVersion.from_dict(self._state["releases"][vid])

    @property
    def optimization_queue(self) -> list[dict[str, Any]]:
        return list(self._state["optimization_queue"])

    @property
    def history(self) -> list[dict[str, Any]]:
        return list(self._state["history"])

    def get_policy_source(self, version: str | None = None) -> str:
        vid = version or self.active_version_id
        if not vid or vid not in self._state["releases"]:
            raise KeyError(f"Unknown release version: {vid!r}")
        rel = ReleaseVersion.from_dict(self._state["releases"][vid])
        source = (self.root_dir / rel.policy_path).read_text(encoding="utf-8")
        if digest(source) != rel.policy_sha256:
            raise ValueError(f"Frozen release {vid!r} tampered: policy_sha256 mismatch")
        return source

    def publish(
        self,
        artifact: FrozenArtifact,
        report: AcceptanceReport,
        *,
        version: str | None = None,
        notes: str = "",
    ) -> ReleaseVersion:
        """Publish a verified FrozenArtifact. Failed or mismatched reports are always rejected."""
        if not report.passed:
            raise ValueError(
                f"Cannot publish failed acceptance report {report.report_id!r}: "
                f"{'; '.join(report.failure_reasons)}"
            )
        if report.artifact.policy_sha256 != artifact.policy_sha256:
            raise ValueError("Acceptance report policy_sha256 does not match FrozenArtifact")
        if report.artifact.config_sha256 != artifact.config_sha256:
            raise ValueError("Acceptance report config_sha256 does not match FrozenArtifact")
        if report.artifact.sdk_contract_sha256 != artifact.sdk_contract_sha256:
            raise ValueError("Acceptance report sdk_contract_sha256 does not match FrozenArtifact")
        if report.artifact.deployment_scope != artifact.deployment_scope:
            raise ValueError("Acceptance report deployment_scope does not match FrozenArtifact")

        vid = version or f"v{len(self._state['releases']) + 1}"
        if vid in self._state["releases"]:
            raise ValueError(f"Release version {vid!r} already exists; publish a new version instead")

        rel_policy_path = f"releases/{vid}/policy.py"
        rel_report_path = f"releases/{vid}/acceptance_report.json"
        abs_policy = self.root_dir / rel_policy_path
        abs_policy.parent.mkdir(parents=True, exist_ok=True)
        abs_policy.write_text(artifact.policy_source, encoding="utf-8")
        _write_json_atomic(self.root_dir / rel_report_path, report.to_dict())

        rel = ReleaseVersion(
            version=vid,
            policy_sha256=artifact.policy_sha256,
            config_sha256=artifact.config_sha256,
            sdk_contract_sha256=artifact.sdk_contract_sha256,
            criteria_sha256=artifact.criteria_sha256,
            deployment_scope=artifact.deployment_scope,
            report_id=report.report_id,
            policy_path=rel_policy_path,
            report_path=rel_report_path,
            published_at_utc=_utc_now(),
            notes=notes,
        )
        self._state["releases"][vid] = rel.to_dict()
        self._state["active_version"] = vid
        self._state["history"].append(
            {
                "event": "publish",
                "version": vid,
                "report_id": report.report_id,
                "policy_sha256": rel.policy_sha256,
                "deployment_scope": rel.deployment_scope,
                "at_utc": _utc_now(),
            }
        )
        if report.queue_for_optimization:
            self._state["optimization_queue"].append(
                {
                    "queue_id": f"opt-{len(self._state['optimization_queue']) + 1:03d}",
                    "baseline_version": vid,
                    "baseline_report_id": report.report_id,
                    "reason": report.optimization_reason,
                    "status": "pending",
                    "created_at_utc": _utc_now(),
                }
            )
        self._save()
        return rel

    def verify_fixed_execution(
        self,
        policy_source: str,
        *,
        config_sha256: str,
        sdk_contract_sha256: str,
    ) -> ReleaseVersion:
        """Ensure runtime execution matches the exact frozen active release without silent edits."""
        active = self.active_release
        if active is None:
            raise ValueError("No active release published")
        if digest(policy_source) != active.policy_sha256:
            raise ValueError("Policy source differs from active frozen release; regression & republish required")
        if config_sha256 != active.config_sha256:
            raise ValueError("Config hash drift relative to active frozen release")
        if sdk_contract_sha256 != active.sdk_contract_sha256:
            raise ValueError("Host SDK contract hash drift relative to active frozen release")
        return active

    def rollback(self, target_version: str | None = None, *, reason: str = "") -> ReleaseVersion:
        """Roll back active release to a previously verified version."""
        if not self._state["releases"]:
            raise ValueError("No releases available for rollback")
        current = self.active_version_id
        if target_version is None:
            prior = [
                h["version"]
                for h in reversed(self._state["history"])
                if h["version"] != current and h["version"] in self._state["releases"]
            ]
            if not prior:
                raise ValueError("No prior version available for rollback")
            target_version = prior[0]
        if target_version not in self._state["releases"]:
            raise KeyError(f"Unknown rollback target version: {target_version!r}")

        self._state["active_version"] = target_version
        self._state["history"].append(
            {
                "event": "rollback",
                "from_version": current,
                "version": target_version,
                "reason": reason,
                "at_utc": _utc_now(),
            }
        )
        self._save()
        return ReleaseVersion.from_dict(self._state["releases"][target_version])

    def evaluate_improvement(
        self,
        *,
        queue_id: str,
        baseline_report: AcceptanceReport,
        candidate_artifact: FrozenArtifact,
        candidate_report: AcceptanceReport,
        new_version: str | None = None,
    ) -> dict[str, Any]:
        """Accept or reject an improvement candidate for a queued optimization item."""
        item = next((q for q in self._state["optimization_queue"] if q["queue_id"] == queue_id), None)
        if item is None:
            raise KeyError(f"Unknown optimization queue item: {queue_id!r}")

        verdict = compare_improvement_candidate(baseline_report, candidate_report)
        published_version: str | None = None
        if verdict["decision"] == "accepted":
            rel = self.publish(
                candidate_artifact,
                candidate_report,
                version=new_version,
                notes=f"Accepted improvement for {queue_id}: {verdict['reason']}",
            )
            published_version = rel.version
            item["status"] = "accepted"
        else:
            item["status"] = "rejected"

        item["candidate_report_id"] = candidate_report.report_id
        item["decision"] = verdict["decision"]
        item["decision_reason"] = verdict["reason"]
        item["published_version"] = published_version
        item["resolved_at_utc"] = _utc_now()
        self._save()
        return {**verdict, "queue_id": queue_id, "published_version": published_version}
