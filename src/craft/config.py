"""Declarative scene, capability, acceptance, budget, and generation configuration contracts.

All configuration objects are pure data, JSON-serializable, and content-hashed via
``craft.contracts.digest``. Private machine paths, credentials, and secret tokens
are rejected during validation so configs can be versioned and shared safely.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
import re
from typing import Any, Mapping

from .contracts import digest

VALID_PROVIDERS = frozenset({"codex_cli", "openai_api"})
VALID_EFFORTS = frozenset({"low", "medium", "high", "xhigh"})
VALID_EVIDENCE_SOURCES = frozenset({"fixture", "simulator", "recorded_data", "physical_robot"})

_SECRET_PATTERN = re.compile(
    r"(?:sk-[A-Za-z0-9_-]{12,}|bearer\s+\S+|ghp_[A-Za-z0-9]{12,}|hf_[A-Za-z0-9]{12,})",
    re.IGNORECASE,
)
_PRIVATE_PATH_PATTERN = re.compile(r"(?:^|[\s'\"=])(?:/home/[^/\s]+|/Users/[^/\s]+|[A-Za-z]:\\Users\\[^\\\s]+)")


def _check_sanitized(value: Any, path: str = "$") -> None:
    """Reject accidental secrets or private home paths in configuration data."""
    if isinstance(value, str):
        if _SECRET_PATTERN.search(value):
            raise ValueError(f"Configuration field {path} appears to contain a credential or token")
        if _PRIVATE_PATH_PATTERN.search(value):
            raise ValueError(f"Configuration field {path} contains a private machine path")
    elif isinstance(value, Mapping):
        for k, v in value.items():
            if any(tok in str(k).lower() for tok in ("api_key", "password", "secret", "token", "credential")):
                if v:
                    raise ValueError(f"Configuration must not embed secret key {path}.{k}")
            _check_sanitized(v, f"{path}.{k}")
    elif isinstance(value, (list, tuple)):
        for i, item in enumerate(value):
            _check_sanitized(item, f"{path}[{i}]")


@dataclass(frozen=True)
class HostConnectionConfig:
    """Explicit connection settings for an existing ticos-edge host."""

    edge_base_url: str
    scene_path: str = "/edge-scene"
    cap_state_path: str = "/cap/state"
    cap_execute_path: str = "/cap/execute"
    cap_reset_path: str = "/cap/reset"
    sim_base_url: str | None = None
    timeout_s: float = 10.0
    execute_timeout_s: float = 180.0
    max_evidence_age_ms: float = 2000.0

    def __post_init__(self) -> None:
        if not self.edge_base_url or not self.edge_base_url.startswith(("http://", "https://")):
            raise ValueError("edge_base_url must be an explicit http:// or https:// URL")
        if self.sim_base_url is not None and not self.sim_base_url.startswith(("http://", "https://")):
            raise ValueError("sim_base_url must be an explicit http:// or https:// URL when provided")
        if self.timeout_s <= 0 or self.execute_timeout_s <= 0 or self.max_evidence_age_ms <= 0:
            raise ValueError("Timeouts and max_evidence_age_ms must be positive")
        for p in (self.scene_path, self.cap_state_path, self.cap_execute_path, self.cap_reset_path):
            if not p.startswith("/"):
                raise ValueError(f"Endpoint path must start with '/': {p}")

    @property
    def normalized_edge_base(self) -> str:
        return self.edge_base_url.rstrip("/")

    @property
    def normalized_sim_base(self) -> str | None:
        return self.sim_base_url.rstrip("/") if self.sim_base_url else None


@dataclass(frozen=True)
class ObjectBindingConfig:
    """Binding rule for an object of interest consumed from EvidenceScene."""

    name: str
    semantic_classes: tuple[str, ...]
    required_count: int = 1
    allow_ambiguous: bool = False
    min_confidence: float = 0.0

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("ObjectBindingConfig.name must be non-empty")
        if not self.semantic_classes:
            raise ValueError(f"Object binding {self.name!r} requires at least one semantic_class")
        if self.required_count < 0:
            raise ValueError("required_count must be >= 0")
        if not (0.0 <= self.min_confidence <= 1.0):
            raise ValueError("min_confidence must be between 0.0 and 1.0")


@dataclass(frozen=True)
class CapabilityRequirementConfig:
    """Capabilities required from the host SDK and non-idempotent action markers."""

    required_methods: tuple[str, ...]
    non_idempotent_methods: tuple[str, ...] = ("start_coffee_machine",)
    expected_sdk_contract_sha256: str | None = None

    def __post_init__(self) -> None:
        if not self.required_methods:
            raise ValueError("required_methods must list at least one host capability")


@dataclass(frozen=True)
class AcceptanceCriteriaConfig:
    """Independent acceptance gate configuration locked outside policy generation."""

    min_completion_rate: float = 1.0
    max_unknown_outcomes: int = 0
    max_duplicate_non_idempotent: int = 0
    max_human_interventions: int = 0
    max_delivery_error_mm: float = 15.0
    target_wall_s: float = 90.0
    slow_step_threshold_s: float = 20.0
    min_heldout_trials: int = 2

    def __post_init__(self) -> None:
        if not (0.0 < self.min_completion_rate <= 1.0):
            raise ValueError("min_completion_rate must be in (0.0, 1.0]")
        if self.max_unknown_outcomes < 0 or self.max_duplicate_non_idempotent < 0 or self.max_human_interventions < 0:
            raise ValueError("Maximum failure/intervention counts must be >= 0")
        if self.max_delivery_error_mm <= 0 or self.target_wall_s <= 0 or self.slow_step_threshold_s <= 0:
            raise ValueError("Error and latency thresholds must be positive")
        if self.min_heldout_trials < 1:
            raise ValueError("min_heldout_trials must be >= 1")

    @property
    def criteria_sha256(self) -> str:
        return digest(asdict(self))


@dataclass(frozen=True)
class ExperimentBudgetConfig:
    """Explicit budget and execution permissions for policy development & verification."""

    max_model_calls: int = 6
    max_trials: int = 10
    max_wall_s: float = 600.0
    allow_live_actuation: bool = False
    allow_sim_reset: bool = False
    deployment_scope: str = "simulator"

    def __post_init__(self) -> None:
        if self.max_model_calls < 0 or self.max_trials < 0 or self.max_wall_s <= 0:
            raise ValueError("Budget limits must be non-negative (and max_wall_s > 0)")
        if self.deployment_scope not in VALID_EVIDENCE_SOURCES:
            raise ValueError(f"deployment_scope must be one of {sorted(VALID_EVIDENCE_SOURCES)}")


@dataclass(frozen=True)
class GenerationConfig:
    """Explicit, traceable LLM generation backend configuration."""

    provider: str = "codex_cli"
    model: str = ""
    reasoning_effort: str = "medium"
    timeout_s: float = 180.0

    def __post_init__(self) -> None:
        if self.provider not in VALID_PROVIDERS:
            raise ValueError(f"provider must be one of {sorted(VALID_PROVIDERS)}")
        if self.reasoning_effort not in VALID_EFFORTS:
            raise ValueError(f"reasoning_effort must be one of {sorted(VALID_EFFORTS)}")
        if self.timeout_s <= 0:
            raise ValueError("GenerationConfig.timeout_s must be positive")


@dataclass(frozen=True)
class SceneConfig:
    """Complete, content-hashed CRAFT scene configuration."""

    scene_id: str
    task_name: str
    host: HostConnectionConfig
    objects: tuple[ObjectBindingConfig, ...]
    capabilities: CapabilityRequirementConfig
    acceptance: AcceptanceCriteriaConfig = field(default_factory=AcceptanceCriteriaConfig)
    budget: ExperimentBudgetConfig = field(default_factory=ExperimentBudgetConfig)
    generation: GenerationConfig = field(default_factory=GenerationConfig)
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.scene_id.strip() or not self.task_name.strip():
            raise ValueError("scene_id and task_name must be non-empty")
        if not self.objects:
            raise ValueError("SceneConfig requires at least one ObjectBindingConfig")
        _check_sanitized(self.to_dict())

    def to_dict(self) -> dict[str, Any]:
        return {
            "scene_id": self.scene_id,
            "task_name": self.task_name,
            "host": asdict(self.host),
            "objects": [
                {
                    "name": o.name,
                    "semantic_classes": list(o.semantic_classes),
                    "required_count": o.required_count,
                    "allow_ambiguous": o.allow_ambiguous,
                    "min_confidence": o.min_confidence,
                }
                for o in self.objects
            ],
            "capabilities": {
                "required_methods": list(self.capabilities.required_methods),
                "non_idempotent_methods": list(self.capabilities.non_idempotent_methods),
                "expected_sdk_contract_sha256": self.capabilities.expected_sdk_contract_sha256,
            },
            "acceptance": asdict(self.acceptance),
            "budget": asdict(self.budget),
            "generation": asdict(self.generation),
            "metadata": dict(self.metadata),
        }

    @property
    def config_sha256(self) -> str:
        return digest(self.to_dict())

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> SceneConfig:
        _check_sanitized(data)
        host_raw = data["host"]
        host = HostConnectionConfig(
            edge_base_url=str(host_raw["edge_base_url"]),
            scene_path=str(host_raw.get("scene_path", "/edge-scene")),
            cap_state_path=str(host_raw.get("cap_state_path", "/cap/state")),
            cap_execute_path=str(host_raw.get("cap_execute_path", "/cap/execute")),
            cap_reset_path=str(host_raw.get("cap_reset_path", "/cap/reset")),
            sim_base_url=host_raw.get("sim_base_url"),
            timeout_s=float(host_raw.get("timeout_s", 10.0)),
            execute_timeout_s=float(host_raw.get("execute_timeout_s", 180.0)),
            max_evidence_age_ms=float(host_raw.get("max_evidence_age_ms", 2000.0)),
        )
        objects = tuple(
            ObjectBindingConfig(
                name=str(item["name"]),
                semantic_classes=tuple(str(c) for c in item["semantic_classes"]),
                required_count=int(item.get("required_count", 1)),
                allow_ambiguous=bool(item.get("allow_ambiguous", False)),
                min_confidence=float(item.get("min_confidence", 0.0)),
            )
            for item in data["objects"]
        )
        cap_raw = data["capabilities"]
        capabilities = CapabilityRequirementConfig(
            required_methods=tuple(str(m) for m in cap_raw["required_methods"]),
            non_idempotent_methods=tuple(
                str(m) for m in cap_raw.get("non_idempotent_methods", ("start_coffee_machine",))
            ),
            expected_sdk_contract_sha256=cap_raw.get("expected_sdk_contract_sha256"),
        )
        acc_raw = data.get("acceptance") or {}
        acceptance = AcceptanceCriteriaConfig(
            min_completion_rate=float(acc_raw.get("min_completion_rate", 1.0)),
            max_unknown_outcomes=int(acc_raw.get("max_unknown_outcomes", 0)),
            max_duplicate_non_idempotent=int(acc_raw.get("max_duplicate_non_idempotent", 0)),
            max_human_interventions=int(acc_raw.get("max_human_interventions", 0)),
            max_delivery_error_mm=float(acc_raw.get("max_delivery_error_mm", 15.0)),
            target_wall_s=float(acc_raw.get("target_wall_s", 90.0)),
            slow_step_threshold_s=float(acc_raw.get("slow_step_threshold_s", 20.0)),
            min_heldout_trials=int(acc_raw.get("min_heldout_trials", 2)),
        )
        bud_raw = data.get("budget") or {}
        budget = ExperimentBudgetConfig(
            max_model_calls=int(bud_raw.get("max_model_calls", 6)),
            max_trials=int(bud_raw.get("max_trials", 10)),
            max_wall_s=float(bud_raw.get("max_wall_s", 600.0)),
            allow_live_actuation=bool(bud_raw.get("allow_live_actuation", False)),
            allow_sim_reset=bool(bud_raw.get("allow_sim_reset", False)),
            deployment_scope=str(bud_raw.get("deployment_scope", "simulator")),
        )
        gen_raw = data.get("generation") or {}
        generation = GenerationConfig(
            provider=str(gen_raw.get("provider", "codex_cli")),
            model=str(gen_raw.get("model", "")),
            reasoning_effort=str(gen_raw.get("reasoning_effort", "medium")),
            timeout_s=float(gen_raw.get("timeout_s", 180.0)),
        )
        return cls(
            scene_id=str(data["scene_id"]),
            task_name=str(data["task_name"]),
            host=host,
            objects=objects,
            capabilities=capabilities,
            acceptance=acceptance,
            budget=budget,
            generation=generation,
            metadata=dict(data.get("metadata") or {}),
        )

    @classmethod
    def from_file(cls, path: str | Path) -> SceneConfig:
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls.from_dict(raw)

    def write_file(self, path: str | Path) -> None:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(self.to_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
