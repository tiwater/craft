"""Transport-independent Spatial State Contract v1 consumer records and validators.

Aligned with EvidenceScene v3.0 Schema v1 (`architecture-baseline-1.md`) while remaining
100% pure Python stdlib and backward-compatible with v2 snapshots.

Key invariants enforced here (zero inferred or fabricated scene state):
1. `observation_time` is strictly separated from `revision_time` and `published_at`.
   A late historical correction never makes an old observation fresh.
2. Freshness is only defined within the same explicit `clock_id` domain when
   `reference.ns >= observation.ns`; cross-clock or inverted timestamps return `unknown`.
3. Epistemic states are explicitly distinct:
   `unknown` (no supporting observation) != `unavailable` (known source failed) !=
   `stale` (previously accepted value expired) != `resnapshot_required` (epoch/gap discontinuity).
4. Unknown space is never implicitly treated as observed-free space.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Sequence

LEGACY_WALL_CLOCK_ID = "legacy_wall_ns"
VALID_FRESHNESS_STATUSES = frozenset({"fresh", "stale", "unknown"})
VALID_OBSERVABILITY_STATUSES = frozenset({"sufficient", "insufficient", "unknown"})
VALID_ASSESSMENT_STATUSES = frozenset({"satisfied", "unsatisfied", "unknown"})
VALID_CHANGE_CAUSES = frozenset(
    {"observation", "physical_motion", "historical_correction", "removal", "reset"}
)
PERCEPTION_BLOCK_STATUSES = frozenset(
    {
        "stale_evidence",
        "unknown_freshness",
        "unknown_evidence",
        "unavailable_evidence",
        "insufficient_observability",
        "resnapshot_required",
        "missing_object",
        "ambiguous_object",
        "unsatisfied",
    }
)


@dataclass(frozen=True)
class SceneCursor:
    """Unique immutable version identifier `(scene_id, epoch, revision)`."""

    scene_id: str
    epoch: int
    revision: int

    def __post_init__(self) -> None:
        if not isinstance(self.scene_id, str) or not self.scene_id.strip():
            raise ValueError("SceneCursor.scene_id must be a non-empty string")
        if self.epoch < 0 or self.revision < 0:
            raise ValueError("SceneCursor.epoch and revision must be nonnegative integers")

    def to_dict(self) -> dict[str, Any]:
        return {
            "scene_id": self.scene_id,
            "epoch": int(self.epoch),
            "revision": int(self.revision),
        }

    @classmethod
    def from_mapping(cls, raw: Mapping[str, Any]) -> "SceneCursor":
        return cls(
            scene_id=str(raw.get("scene_id") or "").strip(),
            epoch=int(raw.get("epoch", 0)),
            revision=int(raw.get("revision", 0)),
        )


@dataclass(frozen=True)
class ClockTimestamp:
    """Clock-domain-tagged nanosecond timestamp `(clock_id, ns)`."""

    clock_id: str
    ns: int

    def __post_init__(self) -> None:
        if not isinstance(self.clock_id, str) or not self.clock_id.strip():
            raise ValueError("ClockTimestamp.clock_id must be a non-empty string")
        if self.ns <= 0:
            raise ValueError("ClockTimestamp.ns must be a positive integer")

    def to_dict(self) -> dict[str, Any]:
        return {"clock_id": self.clock_id, "ns": int(self.ns)}


@dataclass(frozen=True)
class FreshnessResult:
    """Result of checking observation freshness within a single clock domain."""

    status: str  # fresh | stale | unknown
    age_ms: float | None = None
    reason: str = ""

    def __post_init__(self) -> None:
        if self.status not in VALID_FRESHNESS_STATUSES:
            raise ValueError(f"Invalid freshness status: {self.status!r}")

    def to_dict(self) -> dict[str, Any]:
        return {"status": self.status, "age_ms": self.age_ms, "reason": self.reason}


@dataclass(frozen=True)
class ObservabilitySummary:
    """Canonical observability summary attached to an entity or snapshot."""

    status: str  # sufficient | insufficient | unknown
    rank: int | None = None
    required_rank: int = 6
    reason: str = ""
    profile: str = ""

    def __post_init__(self) -> None:
        if self.status not in VALID_OBSERVABILITY_STATUSES:
            raise ValueError(f"Invalid observability status: {self.status!r}")

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "rank": self.rank,
            "required_rank": int(self.required_rank),
            "reason": self.reason,
            "profile": self.profile,
        }


@dataclass(frozen=True)
class SpatialChange:
    """Revision-ordered scene change emitted by the Spatial State Manager."""

    cursor: SceneCursor
    affected_entities: tuple[str, ...]
    cause: str  # observation | physical_motion | historical_correction | removal | reset
    prior_geometry_refs: Mapping[str, str] | None = None
    new_geometry_refs: Mapping[str, str] | None = None
    detail: str = ""

    def __post_init__(self) -> None:
        if self.cause not in VALID_CHANGE_CAUSES:
            raise ValueError(f"Invalid spatial change cause: {self.cause!r}")

    def to_dict(self) -> dict[str, Any]:
        return {
            "cursor": self.cursor.to_dict(),
            "affected_entities": list(self.affected_entities),
            "cause": self.cause,
            "prior_geometry_refs": dict(self.prior_geometry_refs or {}),
            "new_geometry_refs": dict(self.new_geometry_refs or {}),
            "detail": self.detail,
        }


@dataclass(frozen=True)
class SpatialRequirementAssessment:
    """Consumer view of EvidenceScene `assess_spatial_requirements(...)`."""

    status: str  # satisfied | unsatisfied | unknown
    entity_id: str
    cursor: SceneCursor | None = None
    observation_time: ClockTimestamp | None = None
    freshness: FreshnessResult = FreshnessResult(status="unknown", reason="not_evaluated")
    unobservable_directions: tuple[str, ...] = ()
    missing_information: tuple[str, ...] = ()
    limitations: tuple[str, ...] = ()
    reason: str = ""

    def __post_init__(self) -> None:
        if self.status not in VALID_ASSESSMENT_STATUSES:
            raise ValueError(f"Invalid assessment status: {self.status!r}")

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "entity_id": self.entity_id,
            "cursor": self.cursor.to_dict() if self.cursor else None,
            "observation_time": self.observation_time.to_dict() if self.observation_time else None,
            "freshness": self.freshness.to_dict(),
            "unobservable_directions": list(self.unobservable_directions),
            "missing_information": list(self.missing_information),
            "limitations": list(self.limitations),
            "reason": self.reason,
        }


@dataclass(frozen=True)
class ParsedSnapshotMeta:
    """Extracted v1/v2 metadata from a raw scene snapshot without modifying or inferring state."""

    schema_version: int
    cursor: SceneCursor | None
    observation_time: ClockTimestamp | None
    revision_time: ClockTimestamp | None
    published_at: ClockTimestamp | None
    availability: str  # available | unknown | unavailable | resnapshot_required
    observability: ObservabilitySummary
    detail: str = ""


def evaluate_freshness(
    observation_time: ClockTimestamp | None,
    reference_time: ClockTimestamp | None,
    max_age_ms: float,
) -> FreshnessResult:
    """Evaluate observation freshness strictly within a matching clock domain.

    Per EvidenceScene v3 Schema v1:
    - Sensor timestamps and reference clocks cannot be subtracted across different `clock_id`s.
    - If `reference_time.ns < observation_time.ns`, freshness is `unknown` (never clamped to 0ms fresh).
    """
    if observation_time is None:
        return FreshnessResult(status="unknown", age_ms=None, reason="missing_observation_timestamp")
    if reference_time is None:
        return FreshnessResult(status="unknown", age_ms=None, reason="missing_reference_timestamp")
    if observation_time.clock_id != reference_time.clock_id:
        return FreshnessResult(
            status="unknown",
            age_ms=None,
            reason=(
                f"clock_domain_mismatch: observation clock={observation_time.clock_id!r} "
                f"!= reference clock={reference_time.clock_id!r}"
            ),
        )
    delta_ns = reference_time.ns - observation_time.ns
    if delta_ns < 0:
        return FreshnessResult(
            status="unknown",
            age_ms=None,
            reason=(
                f"reference_precedes_observation: ref={reference_time.ns} < obs={observation_time.ns}"
            ),
        )
    age_ms = round(delta_ns / 1e6, 1)
    if age_ms > float(max_age_ms):
        return FreshnessResult(
            status="stale",
            age_ms=age_ms,
            reason=f"observation_age_exceeded ({age_ms} ms > {max_age_ms} ms)",
        )
    return FreshnessResult(status="fresh", age_ms=age_ms, reason="within_max_age")


def check_cursor_continuity(
    expected_cursor: SceneCursor | None,
    observed_cursor: SceneCursor | None,
    *,
    earliest_retained_revision: int | None = None,
) -> tuple[str, str]:
    """Validate whether `observed_cursor` is continuous with `expected_cursor`.

    Returns `(status, reason)` where status is one of:
    - `"head"`: exact same cursor
    - `"continuous"`: same scene & epoch, forward revision within retained history
    - `"resnapshot_required"`: scene mismatch, epoch reset, backward/future discontinuity, or evicted history
    """
    if expected_cursor is None or observed_cursor is None:
        return ("continuous", "no_prior_cursor_constraint")
    if observed_cursor.scene_id != expected_cursor.scene_id:
        return (
            "resnapshot_required",
            f"scene_id_changed ({expected_cursor.scene_id!r} -> {observed_cursor.scene_id!r})",
        )
    if observed_cursor.epoch != expected_cursor.epoch:
        return (
            "resnapshot_required",
            f"epoch_changed ({expected_cursor.epoch} -> {observed_cursor.epoch})",
        )
    if observed_cursor.revision == expected_cursor.revision:
        return ("head", "at_expected_revision")
    if observed_cursor.revision < expected_cursor.revision:
        return (
            "resnapshot_required",
            f"cursor_regressed ({expected_cursor.revision} -> {observed_cursor.revision})",
        )
    if earliest_retained_revision is not None and expected_cursor.revision < int(earliest_retained_revision):
        return (
            "resnapshot_required",
            f"history_evicted (expected rev={expected_cursor.revision} < earliest_retained={earliest_retained_revision})",
        )
    return (
        "continuous",
        f"advanced_revision ({expected_cursor.revision} -> {observed_cursor.revision})",
    )


def _parse_clock_ts(raw: Any, *, fallback_clock_id: str) -> ClockTimestamp | None:
    if isinstance(raw, Mapping):
        cid = str(raw.get("clock_id") or fallback_clock_id).strip()
        raw_ns = raw.get("ns") if "ns" in raw else raw.get("nanoseconds")
        try:
            ns = int(raw_ns) if raw_ns is not None else 0
        except (TypeError, ValueError):
            ns = 0
        if cid and ns > 0:
            return ClockTimestamp(clock_id=cid, ns=ns)
        return None
    try:
        ns = int(raw) if raw is not None else 0
    except (TypeError, ValueError):
        ns = 0
    if ns > 0 and fallback_clock_id:
        return ClockTimestamp(clock_id=fallback_clock_id, ns=ns)
    return None


def _parse_observability(raw: Any) -> ObservabilitySummary:
    if not isinstance(raw, Mapping):
        return ObservabilitySummary(status="unknown", reason="not_reported")
    raw_st = str(raw.get("status") or "").strip().lower()
    if raw_st in ("sufficient", "observable", "admitted", "ok"):
        status = "sufficient"
    elif raw_st in ("insufficient", "not_observable", "degenerate", "rejected"):
        status = "insufficient"
    else:
        status = "unknown"
    rank_val = raw.get("rank")
    try:
        rank = int(rank_val) if rank_val is not None else None
    except (TypeError, ValueError):
        rank = None
    req_val = raw.get("required_rank", 6)
    try:
        req = int(req_val)
    except (TypeError, ValueError):
        req = 6
    return ObservabilitySummary(
        status=status,
        rank=rank,
        required_rank=req,
        reason=str(raw.get("reason") or raw.get("detail") or ""),
        profile=str(raw.get("profile") or ""),
    )


def parse_spatial_snapshot_meta(snapshot: Mapping[str, Any]) -> ParsedSnapshotMeta:
    """Parse v1/v2 spatial metadata from a snapshot mapping without fabricating fields."""
    raw_sv = snapshot.get("schema_version")
    try:
        schema_version = int(raw_sv) if raw_sv is not None else 0
    except (TypeError, ValueError):
        schema_version = 0

    cursor: SceneCursor | None = None
    raw_cursor = snapshot.get("cursor")
    if isinstance(raw_cursor, Mapping) and str(raw_cursor.get("scene_id") or "").strip():
        try:
            cursor = SceneCursor.from_mapping(raw_cursor)
        except (TypeError, ValueError):
            cursor = None
    elif snapshot.get("scene_id") and ("epoch" in snapshot or "revision" in snapshot):
        try:
            cursor = SceneCursor(
                scene_id=str(snapshot["scene_id"]).strip(),
                epoch=int(snapshot.get("epoch", 0)),
                revision=int(snapshot.get("revision", 0)),
            )
        except (TypeError, ValueError):
            cursor = None

    snapshot_clock = str(snapshot.get("clock_id") or "").strip()
    fallback_clock = snapshot_clock or LEGACY_WALL_CLOCK_ID

    obs_time = _parse_clock_ts(
        snapshot.get("observation_time")
        if "observation_time" in snapshot
        else (snapshot.get("observation_timestamp_ns") if "observation_timestamp_ns" in snapshot else snapshot.get("timestamp_ns")),
        fallback_clock_id=fallback_clock,
    )
    rev_time = _parse_clock_ts(
        snapshot.get("revision_time") if "revision_time" in snapshot else snapshot.get("revision_timestamp_ns"),
        fallback_clock_id=fallback_clock,
    )
    pub_time = _parse_clock_ts(
        snapshot.get("published_at"),
        fallback_clock_id=snapshot_clock or "publisher_monotonic_ns",
    )

    raw_avail = str(snapshot.get("availability") or snapshot.get("state_status") or "").strip().lower()
    if raw_avail in ("unknown", "unavailable", "resnapshot_required"):
        availability = raw_avail
    elif bool(snapshot.get("resnapshot_required")):
        availability = "resnapshot_required"
    else:
        availability = "available"

    observability = _parse_observability(snapshot.get("observability"))
    return ParsedSnapshotMeta(
        schema_version=schema_version,
        cursor=cursor,
        observation_time=obs_time,
        revision_time=rev_time,
        published_at=pub_time,
        availability=availability,
        observability=observability,
        detail=str(snapshot.get("detail") or ""),
    )


def extract_entity_list(snapshot: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    """Return entity/object records from either v2 `objects` list or v3 `entities` mapping/list."""
    if isinstance(snapshot.get("entities"), Mapping):
        out: list[Mapping[str, Any]] = []
        for eid, ent in snapshot["entities"].items():
            if isinstance(ent, Mapping):
                row = dict(ent)
                row.setdefault("entity_id", str(eid))
                row.setdefault("object_id", str(eid))
                out.append(row)
        return out
    if isinstance(snapshot.get("entities"), Sequence) and not isinstance(snapshot.get("entities"), (str, bytes)):
        return [e for e in snapshot["entities"] if isinstance(e, Mapping)]
    raw_objs = snapshot.get("objects")
    if isinstance(raw_objs, Sequence) and not isinstance(raw_objs, (str, bytes)):
        return [o for o in raw_objs if isinstance(o, Mapping)]
    return []
