"""Versioned, headless CRAFT Engine API (/v1/...) designed for control-plane integration.

Control planes (such as ticos or any custom fleet console) integrate CRAFT through
either the in-process ``CraftEngineService`` Python facade or this stdlib HTTP/JSON
server (``craft serve-engine``). Zero UI coupling and zero assumption of any
specific host implementation.
"""
from __future__ import annotations

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time
from typing import Any, Mapping
from urllib.parse import parse_qs, urlparse

from . import __version__
from .autonomy import AutonomyPromotionLedger, StageEvidence
from .coffee_loop import OVEN_REQUIRED_METHODS, default_coffee_teaching_steps
from .config import SceneConfig
from .host_driver import SceneHostDriver, list_host_drivers
from .portability import check_policy_portability
from .profiler import aggregate_trial_profiles, attribute_trial, profile_steps
from .regression import RegressionSuite
from .release import ReleaseRegistry
from .skills import SkillExperienceStore
from .teaching import TeachingMaterial, build_task_specification


class CraftEngineService:
    """Headless core engine facade exposing all CRAFT orchestration & governance capabilities."""

    api_version = "v1"

    def __init__(
        self,
        config: SceneConfig,
        driver: SceneHostDriver,
        workspace_dir: str | Path,
    ) -> None:
        self.config = config
        self.driver = driver
        self.workspace_dir = Path(workspace_dir)
        self.workspace_dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self.registry = ReleaseRegistry(self.workspace_dir / "registry")
        self.regression = RegressionSuite(self.workspace_dir / "regression_suite.json")
        self.autonomy = AutonomyPromotionLedger(self.workspace_dir / "autonomy_ledger.json")
        self.skills = SkillExperienceStore(self.workspace_dir / "skills.json")

    def health(self) -> dict[str, Any]:
        return {
            "ok": True,
            "engine": "tiwater-craft",
            "version": __version__,
            "api_version": self.api_version,
            "driver": self.driver.driver_name,
            "evidence_source": self.driver.evidence_source,
            "available_drivers": list(list_host_drivers()),
            "active_release": self.registry.active_version_id,
        }

    def capabilities(self, *, check_missing: tuple[str, ...] = OVEN_REQUIRED_METHODS) -> dict[str, Any]:
        cap = self.driver.discover_capabilities(allow_busy=True)
        gap = self.driver.check_missing_capabilities(check_missing) if check_missing else None
        return {
            "driver": self.driver.driver_name,
            "evidence_source": self.driver.evidence_source,
            "capability_check": cap.to_dict(),
            "gap_check": gap,
        }

    def build_teaching_spec(self, body: Mapping[str, Any]) -> dict[str, Any]:
        instruction = str(body.get("instruction", ""))
        raw_mats = body.get("materials") or []
        mats = tuple(
            TeachingMaterial(
                material_id=str(m["material_id"]),
                kind=str(m["kind"]),
                summary=str(m["summary"]),
                source_ref=str(m.get("source_ref", "")),
                contract_id=m.get("contract_id"),
                notes=str(m.get("notes", "")),
            )
            for m in raw_mats
        )
        cap = self.driver.discover_capabilities(allow_busy=True)
        discovered = cap.discovered.allowed_methods if cap.discovered else None
        steps = default_coffee_teaching_steps() if body.get("use_default_steps", True) else ()
        spec = build_task_specification(
            self.config,
            instruction,
            materials=mats,
            steps=steps,
            discovered_methods=discovered,
        )
        return spec.to_dict()

    def releases_status(self) -> dict[str, Any]:
        active = self.registry.active_release
        return {
            "active_version": self.registry.active_version_id,
            "active_release": active.to_dict() if active else None,
            "history": self.registry.history,
            "optimization_queue": self.registry.optimization_queue,
        }

    def rollback_release(self, body: Mapping[str, Any]) -> dict[str, Any]:
        rel = self.registry.rollback(
            target_version=body.get("target_version"),
            reason=str(body.get("reason", "control-plane rollback")),
        )
        return {"ok": True, "active_release": rel.to_dict()}

    def applicable_skills(self) -> dict[str, Any]:
        cap = self.driver.discover_capabilities(allow_busy=True)
        if not cap.discovered:
            return {"skills": [], "detail": cap.detail}
        matched = self.skills.query_applicable(
            task_name=self.config.task_name,
            available_methods=cap.discovered.allowed_methods,
            sdk_contract_sha256=cap.discovered.sdk_contract_sha256,
        )
        return {"skills": [s.to_dict() for s in matched]}

    def list_regression_cases(self) -> dict[str, Any]:
        return {"cases": [c.to_dict() for c in self.regression.cases]}

    def capture_regression_case(self, body: Mapping[str, Any]) -> dict[str, Any]:
        trial = body.get("trial") or {}
        ctx = body.get("scenario_context")
        notes = str(body.get("notes", ""))
        case = self.regression.capture_failure(trial, scenario_context=ctx, notes=notes)
        return {"captured": case is not None, "case": case.to_dict() if case else None}

    def run_regression_gate(self, body: Mapping[str, Any]) -> dict[str, Any]:
        policy_source = str(body.get("policy_source") or "")
        if not policy_source.strip():
            raise ValueError("policy_source is required")
        with self._lock:
            return self.regression.evaluate_regression_gate(
                self.driver,
                policy_source,
                policy_id=str(body.get("policy_id") or "candidate"),
            )

    def profile_trials(self, body: Mapping[str, Any]) -> dict[str, Any]:
        trials = body.get("trials")
        if trials is None:
            # Default to workspace benchmark results if present
            bm_path = self.workspace_dir / "benchmark" / "benchmark_results.json"
            if bm_path.is_file():
                trials = json.loads(bm_path.read_text(encoding="utf-8")).get("trials") or []
            else:
                trials = []
        threshold = float(body.get("slow_step_threshold_s", 8.0))
        return aggregate_trial_profiles(trials, slow_step_threshold_s=threshold)

    def record_autonomy_evidence(self, body: Mapping[str, Any]) -> dict[str, Any]:
        ev = StageEvidence(
            stage=str(body["stage"]),
            policy_sha256=str(body["policy_sha256"]),
            config_sha256=str(body["config_sha256"]),
            sdk_contract_sha256=str(body["sdk_contract_sha256"]),
            report_id=str(body["report_id"]),
            passed=bool(body["passed"]),
            total_trials=int(body["total_trials"]),
            human_interventions=int(body.get("human_interventions", 0)),
            notes=str(body.get("notes", "")),
        )
        return {"recorded": self.autonomy.record_evidence(ev)}

    def check_autonomy_promotion(self, body: Mapping[str, Any]) -> dict[str, Any]:
        return self.autonomy.check_promotion(
            policy_sha256=str(body["policy_sha256"]),
            target_stage=str(body["target_stage"]),
            sdk_contract_sha256=body.get("sdk_contract_sha256"),
            require_recorded_data=bool(body.get("require_recorded_data", False)),
            max_live_intervention_rate=float(body.get("max_live_intervention_rate", 0.1)),
        )

    def check_portability(self, body: Mapping[str, Any]) -> dict[str, Any]:
        source = str(body.get("policy_source") or "")
        if not source.strip():
            raise ValueError("policy_source is required")
        return check_policy_portability(source)

    def execute_scenario(self, body: Mapping[str, Any]) -> dict[str, Any]:
        policy_source = body.get("policy_source")
        if not policy_source:
            policy_source = self.registry.get_policy_source()
        scenario = dict(body.get("scenario") or {"preset": "default"})
        cmd_id = str(body.get("command_id") or f"engine-{time.time_ns()}")
        with self._lock:
            if body.get("reset", True):
                self.driver.reset_scenario(scenario)
            res = self.driver.run_policy_scenario(str(policy_source), command_id=cmd_id, scenario=scenario)
        attr = attribute_trial(res).to_dict()
        steps_prof = profile_steps(
            res.get("step_records") or [{"method": m} for m in (res.get("methods") or ())],
            total_wall_s=res.get("wall_s"),
        )
        captured = None
        if body.get("auto_capture_regression", True):
            case = self.regression.capture_failure(res, scenario_context=scenario)
            if case is not None:
                captured = case.to_dict()
        return {
            "execution": res,
            "attribution": attr,
            "step_profile": steps_prof,
            "regression_captured": captured,
        }


def create_engine_server(
    service: CraftEngineService,
    *,
    host: str = "127.0.0.1",
    port: int = 8870,
) -> ThreadingHTTPServer:
    """Create a headless ThreadingHTTPServer exposing CRAFT Engine /v1/... REST API."""

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args: Any) -> None:  # noqa: A002
            return

        def _send_json(self, payload: Any, status: int = 200) -> None:
            raw = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def _read_json(self) -> dict[str, Any]:
            length = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(length) if length > 0 else b"{}"
            return json.loads(raw.decode("utf-8"))

        def do_GET(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            path = parsed.path
            qs = parse_qs(parsed.query)
            try:
                if path == "/v1/health":
                    self._send_json(service.health())
                elif path == "/v1/capabilities":
                    raw_miss = qs.get("missing", [])
                    check_missing = tuple(m for item in raw_miss for m in item.split(",") if m) if raw_miss else OVEN_REQUIRED_METHODS
                    self._send_json(service.capabilities(check_missing=check_missing))
                elif path == "/v1/releases":
                    self._send_json(service.releases_status())
                elif path == "/v1/skills":
                    self._send_json(service.applicable_skills())
                elif path == "/v1/regression/cases":
                    self._send_json(service.list_regression_cases())
                elif path == "/v1/telemetry/attribution":
                    self._send_json(service.profile_trials({}))
                else:
                    self._send_json({"error": f"Unknown endpoint: {path}"}, HTTPStatus.NOT_FOUND)
            except Exception as exc:  # noqa: BLE001
                self._send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)

        def do_POST(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            try:
                body = self._read_json()
                if path == "/v1/teaching/spec":
                    self._send_json(service.build_teaching_spec(body))
                elif path == "/v1/releases/rollback":
                    self._send_json(service.rollback_release(body))
                elif path == "/v1/regression/capture":
                    self._send_json(service.capture_regression_case(body))
                elif path == "/v1/regression/gate":
                    self._send_json(service.run_regression_gate(body))
                elif path == "/v1/telemetry/profile":
                    self._send_json(service.profile_trials(body))
                elif path == "/v1/autonomy/evidence":
                    self._send_json(service.record_autonomy_evidence(body))
                elif path == "/v1/autonomy/check":
                    self._send_json(service.check_autonomy_promotion(body))
                elif path == "/v1/policy/portability":
                    self._send_json(service.check_portability(body))
                elif path == "/v1/execute":
                    self._send_json(service.execute_scenario(body))
                else:
                    self._send_json({"error": f"Unknown endpoint: {path}"}, HTTPStatus.NOT_FOUND)
            except (KeyError, ValueError) as exc:
                self._send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
            except Exception as exc:  # noqa: BLE001
                self._send_json({"error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)

    return ThreadingHTTPServer((host, port), Handler)
