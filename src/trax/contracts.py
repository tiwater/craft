"""Transport-neutral contracts. Adapters own all observations and physical actions."""
from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass
from typing import Any, Mapping, Protocol


def canonical(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def digest(value: Any) -> str:
    return hashlib.sha256(canonical(value).encode()).hexdigest()


@dataclass(frozen=True)
class Receipt:
    command_id: str
    status: str
    evidence: Mapping[str, Any]

    def __post_init__(self):
        if self.status not in {"done", "running", "unknown", "rejected"}:
            raise ValueError("Invalid receipt status")
        if not self.command_id or not isinstance(self.evidence, Mapping):
            raise ValueError("Receipt requires an identity and evidence mapping")
        canonical(dict(self.evidence))

    def to_dict(self) -> dict:
        return asdict(self)


class Adapter(Protocol):
    """Query must not reissue commands. Verify must inspect current external state.

    execute must enforce robot/device interlocks independently of generated code.
    TRAX neither estimates geometry nor treats a dispatch acknowledgement as proof.
    """
    capabilities: frozenset[str]

    def observe(self) -> dict: ...
    def execute(self, action: str, arguments: dict, command_id: str) -> Receipt: ...
    def query(self, command_id: str) -> Receipt: ...
    def verify(self, goal: dict) -> dict: ...
