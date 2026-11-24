"""Tests for TRAX declarative scene configuration and sanitization rules."""
from pathlib import Path
import pytest

from trax.config import (
    AcceptanceCriteriaConfig,
    CapabilityRequirementConfig,
    ExperimentBudgetConfig,
    GenerationConfig,
    HostConnectionConfig,
    ObjectBindingConfig,
    SceneConfig,
)


def _sample_config(**overrides) -> SceneConfig:
    base = dict(
        scene_id="cafe-sim",
        task_name="single_cup_coffee",
        host=HostConnectionConfig(edge_base_url="http://127.0.0.1:8080"),
        objects=(ObjectBindingConfig(name="cup", semantic_classes=("dine_in_red", "takeout_green")),),
        capabilities=CapabilityRequirementConfig(
            required_methods=("observe_scene", "start_coffee_machine", "query_command_status"),
        ),
    )
    base.update(overrides)
    return SceneConfig(**base)


def test_example_single_cup_coffee_config_loads_cleanly():
    example_path = Path(__file__).resolve().parents[1] / "examples" / "single_cup_coffee_config.json"
    cfg = SceneConfig.from_file(example_path)
    assert cfg.task_name == "single_cup_coffee"
    assert cfg.generation.provider == "codex_cli"
    assert cfg.generation.reasoning_effort == "medium"
    assert cfg.budget.allow_live_actuation is False
    assert "start_coffee_machine" in cfg.capabilities.non_idempotent_methods
    assert len(cfg.config_sha256) == 64


def test_config_roundtrip_preserves_hash(tmp_path):
    cfg = _sample_config(
        generation=GenerationConfig(provider="openai_api", model="gpt-example", reasoning_effort="medium"),
    )
    out = tmp_path / "scene.json"
    cfg.write_file(out)
    loaded = SceneConfig.from_file(out)
    assert loaded.config_sha256 == cfg.config_sha256
    assert loaded.generation.provider == "openai_api"
    assert loaded.generation.reasoning_effort == "medium"


def test_rejects_secrets_and_private_paths():
    with pytest.raises(ValueError, match="private machine path"):
        _sample_config(metadata={"note": "/home/alice/secret/repo"})

    with pytest.raises(ValueError, match="credential or token"):
        _sample_config(metadata={"header": "Bearer sk-1234567890abcdef12345"})

    with pytest.raises(ValueError, match="secret key"):
        _sample_config(metadata={"openai_api_key": "not-empty"})


def test_generation_and_acceptance_validation():
    with pytest.raises(ValueError):
        GenerationConfig(provider="unknown_backend")
    with pytest.raises(ValueError):
        GenerationConfig(reasoning_effort="ultra")
    with pytest.raises(ValueError):
        AcceptanceCriteriaConfig(min_completion_rate=0.0)
    with pytest.raises(ValueError):
        ExperimentBudgetConfig(deployment_scope="magic_cloud")
