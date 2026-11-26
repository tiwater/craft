"""Standalone, host-independent web observer and operations interface (TRAX UI).

Built purely on Python's stdlib ``http.server`` with zero external web dependencies.
Consumes any ``SceneHostDriver`` (``fixture`` standalone or ``http`` live sim)
read-only for perception/cameras and delegates all orchestration through TRAX APIs.
"""
from __future__ import annotations

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time
from typing import Any, Mapping, Sequence
from urllib.parse import parse_qs, urlparse

from trax.agent_engine import TraxAgentEngine
from trax.autonomy import AutonomyPromotionLedger
from trax.benchmark import (
    PolicyEntry,
    builtin_reference_policy_source,
    default_coffee_benchmark_suite,
    render_benchmark_html,
    run_paired_benchmark,
)
from trax.coffee_loop import OVEN_REQUIRED_METHODS, default_coffee_teaching_steps
from trax.config import SceneConfig
from trax.embodiments import EmbodimentManager, load_archived_policy_comparison
from trax.evolution_lab import EvolutionLabController, load_m1_archive
from trax.host_driver import SceneHostDriver
from trax.mcp_protocol import MCPInspectorController
from trax.operations import run_continuous_shift
from trax.portability import check_policy_portability
from trax.profiler import aggregate_trial_profiles
from trax.regression import RegressionSuite
from trax.release import ReleaseRegistry
from trax.teaching import TeachingMaterial, build_task_specification


from .template import UI_HTML

def _read_optional_json(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except ValueError:
        return None


class TraxUIApp:
    """Thread-safe controller backing TRAX UI HTTP endpoints."""

    def __init__(
        self,
        config: SceneConfig,
        driver: SceneHostDriver,
        workspace_dir: str | Path,
        *,
        policies: Sequence[PolicyEntry] = (),
    ) -> None:
        self.config = config
        self.driver = driver
        self.workspace_dir = Path(workspace_dir)
        self.workspace_dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._bench_running = False
        self._run_state: dict[str, Any] = {
            "running": False,
            "label": "",
            "started_at": None,
            "elapsed_s": 0.0,
            "last_result": None,
            "error": None,
        }
        self._cam_cache: dict[str, tuple[float, bytes, str]] = {}
        self._policies: dict[str, PolicyEntry] = {}

        seen_hashes: set[str] = set()
        reg_dir = self.workspace_dir / "registry"
        if (reg_dir / "registry.json").is_file():
            reg = ReleaseRegistry(reg_dir)
            if reg.active_version_id:
                src = reg.get_policy_source()
                entry = PolicyEntry(
                    policy_id="active_release",
                    label=f"TRAX Frozen Release ({reg.active_version_id})",
                    source=src,
                    origin="craft_published",
                )
                self._policies[entry.policy_id] = entry
                seen_hashes.add(entry.source_sha256)

        for pol in policies:
            if pol.source_sha256 in seen_hashes:
                continue
            self._policies[pol.policy_id] = pol
            seen_hashes.add(pol.source_sha256)

        ref = PolicyEntry(
            policy_id="craft_reference",
            label="TRAX Reference Baseline",
            source=builtin_reference_policy_source(),
            origin="reference",
        )
        if ref.source_sha256 not in seen_hashes:
            self._policies[ref.policy_id] = ref

        seed_policies = [
            (pol.policy_id, pol.label, pol.source)
            for pol in self._policies.values()
        ]
        self.embodiments = EmbodimentManager(self.config, self.driver, self.workspace_dir)
        self.evolution = EvolutionLabController(
            self.config,
            self.driver,
            self.workspace_dir,
            self._lock,
            seed_policies=seed_policies,
        )
        self.mcp = MCPInspectorController(
            self.workspace_dir,
            lambda: self.driver,
            self._lock,
        )
        self._m1_archive = load_m1_archive(self.workspace_dir)

        def _resolve_active_mcp():
            client, label, _, _, _ = self.mcp._resolve_client({"transport": "auto"})
            return client, label

        self.agent = TraxAgentEngine(
            self.workspace_dir,
            _resolve_active_mcp,
            on_wire=self.mcp.record_external_wire,
        )

    def get_agent_state(self) -> dict[str, Any]:
        st = self.agent.get_state()
        st["m1_archive"] = self._m1_archive
        return st

    def _fetch_live_mcp_telemetry(self) -> dict[str, Any] | None:
        try:
            client, _, _, _, _ = self.mcp._resolve_client({"transport": "auto"})
            _, resp, _ = client.rpc("resources/read", {"uri": "trax://scene/state"})
            contents = ((resp.get("result") or {}).get("contents") or [])
            if contents and contents[0].get("text"):
                return json.loads(str(contents[0]["text"]))
        except Exception:  # noqa: BLE001
            return None
        return None

    def switch_embodiment(self, body: Mapping[str, Any]) -> dict[str, Any]:
        with self._lock:
            drv, meta = self.embodiments.switch(
                target_id=body.get("target_id"),
                host_url=body.get("host_url"),
                evidence_source=body.get("evidence_source"),
                label=body.get("label"),
            )
            self.driver = drv
            self._cam_cache.clear()
            lbl = str(meta.get("label") or drv.driver_name)
            self.evolution.set_driver(drv, label=lbl)
            self.agent.set_backend_changed(lbl)
        return {
            "status": "switched",
            "active_embodiment": meta,
            "embodiments": self.embodiments.snapshot(),
            "state": self.get_state(),
        }

    def compare_archived_policy(self, params: Mapping[str, list[str]]) -> dict[str, Any]:
        suite_key = (params.get("suite") or [""])[0]
        task_key = (params.get("task") or [""])[0]
        left_m = (params.get("left") or ["craft_full"])[0]
        right_m = (params.get("right") or ["rax_protocol"])[0]
        return load_archived_policy_comparison(
            self.workspace_dir,
            suite_key=suite_key,
            task_key=task_key,
            left_method=left_m,
            right_method=right_m,
        )

    def _active_release_id(self) -> str | None:
        reg_dir = self.workspace_dir / "registry"
        if (reg_dir / "registry.json").is_file():
            return ReleaseRegistry(reg_dir).active_version_id
        return None

    def _load_benchmark(self) -> dict[str, Any] | None:
        for candidate in (
            self.workspace_dir / "cafebench-v1" / "cafebench_results.json",
            self.workspace_dir / "benchmark" / "benchmark_results.json",
            self.workspace_dir / "benchmark_results.json",
        ):
            data = _read_optional_json(candidate)
            if data is not None:
                return data
        return None

    def _snapshot_run_state(self) -> dict[str, Any]:
        rs = dict(self._run_state)
        if rs["running"] and rs["started_at"] is not None:
            rs["elapsed_s"] = round(time.monotonic() - rs["started_at"], 1)
        rs.pop("started_at", None)
        return rs

    def fetch_camera_cached(self, camera_name: str, ttl_s: float = 0.08) -> tuple[bytes, str] | None:
        now = time.monotonic()
        cached = self._cam_cache.get(camera_name)
        if cached and (now - cached[0]) <= ttl_s:
            return cached[1], cached[2]
        img = self.driver.fetch_camera_image(camera_name)
        if img is not None:
            self._cam_cache[camera_name] = (now, img[0], img[1])
        return img

    def get_live(self) -> dict[str, Any]:
        """Lightweight high-frequency telemetry payload for smooth live HUD & step animation."""
        obs = self.driver.observe_scene()
        mcp_state = self._fetch_live_mcp_telemetry()
        return {
            "scene_observation": {
                **obs.to_dict(),
                "raw_snapshot": obs.raw_snapshot,
            },
            "mcp_telemetry": mcp_state,
            "live_runtime": self.driver.get_live_runtime(),
            "run_state": self._snapshot_run_state(),
            "evolution": self.get_agent_state(),
        }

    def get_state(self) -> dict[str, Any]:
        cap = self.driver.discover_capabilities(allow_busy=True)
        obs = self.driver.observe_scene()
        oven_gap = self.driver.check_missing_capabilities(OVEN_REQUIRED_METHODS)
        bm = self._load_benchmark()
        p2_tput = _read_optional_json(self.workspace_dir / "phase2_profile_and_throughput.json")
        if p2_tput and p2_tput.get("aggregate_profile"):
            profiler_data = p2_tput["aggregate_profile"]
        elif bm and bm.get("trials"):
            profiler_data = aggregate_trial_profiles(bm["trials"])
        else:
            profiler_data = aggregate_trial_profiles([])

        reg_path = self.workspace_dir / "regression_suite.json"
        reg_cases = [c.to_dict() for c in RegressionSuite(reg_path).cases] if reg_path.is_file() else []
        portability = {pid: check_policy_portability(pol.source) for pid, pol in self._policies.items()}

        promo_gate = None
        auto_path = self.workspace_dir / "autonomy_ledger.json"
        active_rel_id = self._active_release_id()
        if auto_path.is_file() and active_rel_id and "active_release" in self._policies:
            pol_sha = self._policies["active_release"].source_sha256
            promo_gate = AutonomyPromotionLedger(auto_path).check_promotion(
                policy_sha256=pol_sha,
                target_stage="physical_live",
            )

        scr_sessions: list[dict[str, Any]] = []
        scr_root = self.workspace_dir / "screening"
        if scr_root.is_dir():
            for p in sorted(scr_root.glob("*/session.json")):
                item = _read_optional_json(p)
                if item is not None:
                    scr_sessions.append(item)

        return {
            "driver": self.driver.driver_name,
            "evidence_source": self.driver.evidence_source,
            "active_release": active_rel_id,
            "capability_check": cap.to_dict(),
            "oven_gap": oven_gap,
            "scene_observation": {
                **obs.to_dict(),
                "raw_snapshot": obs.raw_snapshot,
            },
            "live_runtime": self.driver.get_live_runtime(),
            "run_state": self._snapshot_run_state(),
            "cameras": list(self.driver.list_cameras()),
            "policies": [p.to_meta() for p in self._policies.values()],
            "benchmark": bm,
            "benchmark_running": self._bench_running,
            "workspace_summary": _read_optional_json(self.workspace_dir / "summary.json"),
            "v1_acceptance": _read_optional_json(self.workspace_dir / "v1_acceptance_report.json"),
            "v2_acceptance": _read_optional_json(self.workspace_dir / "v2_acceptance_report.json"),
            "registry": _read_optional_json(self.workspace_dir / "registry" / "registry.json"),
            "skills": _read_optional_json(self.workspace_dir / "skills.json"),
            "phase2_profiler": profiler_data,
            "phase2_throughput": p2_tput,
            "regression_cases": reg_cases,
            "portability_checks": portability,
            "autonomy_physical_gate": promo_gate,
            "latest_shift_report": _read_optional_json(self.workspace_dir / "latest_shift_report.json"),
            "latest_nightly_evolution": _read_optional_json(self.workspace_dir / "latest_nightly_evolution.json"),
            "screening_sessions": scr_sessions,
        }

    def get_policy(self, policy_id: str) -> dict[str, Any]:
        pol = self._policies[policy_id]
        return {**pol.to_meta(), "source": pol.source}

    def preview_teaching_spec(self, instruction: str, video_ref: str = "") -> dict[str, Any]:
        cap = self.driver.discover_capabilities(allow_busy=True)
        discovered = cap.discovered.allowed_methods if cap.discovered else None
        mats = []
        if video_ref.strip():
            mats.append(
                TeachingMaterial(
                    material_id="ui-video-ref",
                    kind="video",
                    summary="Operator video reference (non-authoritative teaching evidence)",
                    source_ref=video_ref.strip(),
                )
            )
        spec = build_task_specification(
            self.config,
            instruction,
            materials=tuple(mats),
            steps=default_coffee_teaching_steps(),
            discovered_methods=discovered,
        )
        return spec.to_dict()

    def _build_scenario(self, body: Mapping[str, Any]) -> tuple[PolicyEntry, dict[str, Any]]:
        pol_id = str(body.get("policy_id") or "craft_reference")
        pol = self._policies[pol_id]
        preset = str(body.get("preset") or "default")
        slip = body.get("fault_inject_slip_xy_m")
        if slip is None and body.get("fault_slip"):
            slip = [0.038, -0.012]
        scenario: dict[str, Any] = {
            "preset": preset,
            "cup_a_xy": body.get("cup_a_xy"),
            "cup_b_xy": body.get("cup_b_xy"),
            "order_id": str(body.get("order_id") or "A042"),
            "target_slot": str(body.get("target_slot") or "$SLOT_PICKUP_1"),
            "fault_inject_slip_xy_m": slip,
            "simulate_timeout": bool(body.get("simulate_timeout")),
            "expect_status": "paused_for_operator" if preset == "sim_dual_cup" else "completed",
        }
        return pol, scenario

    def run_single_scenario(self, body: Mapping[str, Any]) -> dict[str, Any]:
        pol, scenario = self._build_scenario(body)
        with self._lock:
            self.driver.reset_scenario(scenario)
            cmd_id = f"ui-{pol.policy_id}-{time.time_ns()}"
            return self.driver.run_policy_scenario(pol.source, command_id=cmd_id, scenario=scenario)

    def start_scenario_async(self, body: Mapping[str, Any]) -> dict[str, Any]:
        if self._run_state["running"]:
            return {"status": "already_running"}
        pol, scenario = self._build_scenario(body)
        label = str(body.get("label") or scenario["preset"])
        self._run_state = {
            "running": True,
            "label": label,
            "started_at": time.monotonic(),
            "elapsed_s": 0.0,
            "last_result": None,
            "error": None,
        }

        def _worker() -> None:
            try:
                with self._lock:
                    self.driver.reset_scenario(scenario)
                    cmd_id = f"ui-{pol.policy_id}-{time.time_ns()}"
                    res = self.driver.run_policy_scenario(pol.source, command_id=cmd_id, scenario=scenario)
                self._run_state["last_result"] = res
            except Exception as exc:  # noqa: BLE001
                self._run_state["error"] = str(exc)
            finally:
                if self._run_state["started_at"] is not None:
                    self._run_state["elapsed_s"] = round(time.monotonic() - self._run_state["started_at"], 1)
                self._run_state["running"] = False

        threading.Thread(target=_worker, daemon=True).start()
        return {"status": "started", "label": label}

    def run_ui_shift(self, body: Mapping[str, Any]) -> dict[str, Any]:
        pol_id = str(body.get("policy_id") or "craft_reference")
        pol = self._policies[pol_id]
        reg_suite = RegressionSuite(self.workspace_dir / "regression_suite.json")
        with self._lock:
            return run_continuous_shift(
                self.driver,
                self.config,
                self.workspace_dir,
                policy_source=pol.source,
                orders=body.get("orders"),
                regression_suite=reg_suite,
                enable_journal_proxy=bool(body.get("enable_journal_proxy", True)),
            )

    def start_benchmark_async(self, repeats: int = 1) -> dict[str, Any]:
        if self._bench_running:
            return {"status": "already_running"}
        self._bench_running = True

        def _worker() -> None:
            try:
                with self._lock:
                    run_paired_benchmark(
                        self.driver,
                        list(self._policies.values()),
                        default_coffee_benchmark_suite(),
                        self.workspace_dir / "benchmark",
                        repeats=max(1, min(5, int(repeats))),
                    )
            finally:
                self._bench_running = False

        threading.Thread(target=_worker, daemon=True).start()
        return {"status": "started", "repeats": repeats}


def create_ui_server(
    app: TraxUIApp,
    *,
    host: str = "127.0.0.1",
    port: int = 8860,
) -> ThreadingHTTPServer:
    """Create a ThreadingHTTPServer serving TRAX UI."""

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

        def _send_bytes(self, data: bytes, content_type: str, status: int = 200) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _read_json(self) -> dict[str, Any]:
            length = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(length) if length > 0 else b"{}"
            return json.loads(raw.decode("utf-8"))

        def _strip_subpath(self, path: str) -> str:
            if path.startswith("/trax/"):
                return path[len("/trax") :]
            return path

        def do_GET(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            raw_path = parsed.path
            if raw_path == "/trax":
                self.send_response(HTTPStatus.FOUND)
                self.send_header("Location", "/trax/")
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            path = self._strip_subpath(raw_path)
            if path in ("/", "/index.html"):
                self._send_bytes(UI_HTML.encode("utf-8"), "text/html; charset=utf-8")
                return
            if path == "/api/state":
                self._send_json(app.get_state())
                return
            if path == "/api/live":
                self._send_json(app.get_live())
                return
            if path == "/api/evolve/state":
                self._send_json(app.get_agent_state())
                return
            if path == "/api/embodiments":
                self._send_json(app.embodiments.snapshot())
                return
            if path == "/api/mcp/state":
                self._send_json(app.mcp.get_state())
                return
            if path == "/api/kb/state":
                self._send_json(app.agent.get_kb_snapshot())
                return
            if path == "/api/kb/diff":
                qs = parse_qs(parsed.query)
                sha = (qs.get("sha") or [""])[0]
                parent = (qs.get("parent") or [None])[0]
                self._send_json({"sha": sha, "diff": app.agent.kb.git_diff(sha, parent)})
                return
            if path == "/api/kb/file":
                qs = parse_qs(parsed.query)
                rel = (qs.get("path") or [""])[0]
                self._send_json({"path": rel, "content": app.agent.kb.read_text_file(rel)})
                return
            if path == "/api/compare/policy":
                try:
                    self._send_json(app.compare_archived_policy(parse_qs(parsed.query)))
                except KeyError as exc:
                    self._send_json({"error": str(exc)}, HTTPStatus.NOT_FOUND)
                return
            def _find_report(*rel_candidates: str) -> Path | None:
                roots = [app.workspace_dir, app.workspace_dir.parent / "goal-v2-20261122", app.workspace_dir.parent / "goal-v3-simex-parity"]
                for root in roots:
                    for rel in rel_candidates:
                        p = root / rel
                        if p.is_file():
                            return p
                return None

            if path == "/robosuite_v2_report.html":
                rb2_html = _find_report("robosuite-parity-v3/robosuite_prior_work_report.html", "robosuite-parity-v2/robosuite_prior_work_report.html")
                if rb2_html is not None:
                    self._send_bytes(rb2_html.read_bytes(), "text/html; charset=utf-8")
                else:
                    self._send_bytes(b"<h3>No Robosuite Parity report generated yet.</h3>", "text/html; charset=utf-8", 404)
                return
            if path == "/cafebench_v2_report.html":
                cb2_html = _find_report("cafebench-v2/cafebench_v2_report.html")
                if cb2_html is not None:
                    self._send_bytes(cb2_html.read_bytes(), "text/html; charset=utf-8")
                else:
                    self._send_bytes(b"<h3>No CafeBench v2 report generated yet.</h3>", "text/html; charset=utf-8", 404)
                return
            if path == "/sim2sim_report.html":
                s2s_html = _find_report("sim2sim-stage2/sim2sim_report.html", "sim2sim_report.html")
                if s2s_html is not None:
                    self._send_bytes(s2s_html.read_bytes(), "text/html; charset=utf-8")
                else:
                    self._send_bytes(b"<h3>No Sim2Sim Stage-2 report generated yet.</h3>", "text/html; charset=utf-8", 404)
                return
            if path == "/robosuite_report.html":
                rb_html = app.workspace_dir / "robosuite-parity-v1" / "robosuite_prior_work_report.html"
                if rb_html.is_file():
                    self._send_bytes(rb_html.read_bytes(), "text/html; charset=utf-8")
                else:
                    self._send_bytes(b"<h3>No Robosuite parity report generated yet.</h3>", "text/html; charset=utf-8", 404)
                return
            if path == "/prior_work_report.html":
                pw_html = app.workspace_dir / "cafebench-v1-prior-work" / "prior_work_report.html"
                if pw_html.is_file():
                    self._send_bytes(pw_html.read_bytes(), "text/html; charset=utf-8")
                else:
                    self._send_bytes(b"<h3>No prior-work comparison report generated yet.</h3>", "text/html; charset=utf-8", 404)
                return
            if path == "/benchmark_report.html":
                cb_html = app.workspace_dir / "cafebench-v1" / "cafebench_report.html"
                if cb_html.is_file():
                    self._send_bytes(cb_html.read_bytes(), "text/html; charset=utf-8")
                    return
                bm = app._load_benchmark()
                if bm:
                    self._send_bytes(render_benchmark_html(bm).encode("utf-8"), "text/html; charset=utf-8")
                else:
                    self._send_bytes(b"<h3>No benchmark report generated yet.</h3>", "text/html; charset=utf-8", 404)
                return
            if path.startswith("/api/policy/"):
                pid = path[len("/api/policy/") :]
                try:
                    self._send_json(app.get_policy(pid))
                except KeyError:
                    self._send_json({"error": f"Unknown policy: {pid}"}, HTTPStatus.NOT_FOUND)
                return
            if path.startswith("/api/camera/"):
                cam = path[len("/api/camera/") :]
                img = app.fetch_camera_cached(cam)
                if img is None:
                    self._send_json({"error": f"Camera unavailable: {cam}"}, HTTPStatus.NOT_FOUND)
                else:
                    self._send_bytes(img[0], img[1])
                return
            self._send_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)

        def do_POST(self) -> None:  # noqa: N802
            path = self._strip_subpath(urlparse(self.path).path)
            try:
                body = self._read_json()
                if path == "/api/teaching/spec":
                    self._send_json(
                        app.preview_teaching_spec(
                            str(body.get("instruction", "")),
                            str(body.get("video_ref", "")),
                        )
                    )
                    return
                if path == "/api/confirm":
                    self._send_json(app.driver.confirm_object(str(body.get("object_id", ""))))
                    return
                if path == "/api/reset":
                    with app._lock:
                        self._send_json(app.driver.reset_scenario({"preset": str(body.get("preset", "default"))}))
                    return
                if path == "/api/run":
                    self._send_json(app.run_single_scenario(body))
                    return
                if path == "/api/run/start":
                    self._send_json(app.start_scenario_async(body))
                    return
                if path == "/api/benchmark/start":
                    self._send_json(app.start_benchmark_async(int(body.get("repeats", 1))))
                    return
                if path == "/api/shift/run":
                    self._send_json(app.run_ui_shift(body))
                    return
                if path == "/api/embodiment/switch":
                    self._send_json(app.switch_embodiment(body))
                    return
                if path == "/api/mcp/inspect":
                    self._send_json(app.mcp.inspect(body))
                    return
                if path == "/api/mcp/call":
                    self._send_json(app.mcp.call_tool(body))
                    return
                if path == "/api/mcp/read_resource":
                    self._send_json(app.mcp.read_resource(body))
                    return
                if path == "/api/kb/rollback":
                    self._send_json(app.agent.rollback_to_commit(str(body.get("sha") or "")))
                    return
                if path == "/api/evolve/probe":
                    self._send_json(app.agent.start_explore_async(body))
                    return
                if path == "/api/evolve/synthesize":
                    self._send_json(app.agent.start_synthesize_async(body))
                    return
                if path == "/api/evolve/execute":
                    self._send_json(app.agent.start_execute_async(body))
                    return
                if path == "/api/evolve/auto":
                    self._send_json(app.agent.start_closed_loop_async(body))
                    return
                if path == "/api/evolve/select":
                    self._send_json(app.agent.select_iteration(int(body.get("iter_index", 0))))
                    return
                if path == "/api/evolve/reset":
                    self._send_json(
                        app.agent.reset_session(
                            task_instruction=body.get("task_instruction"),
                            task_id=body.get("task_id") or body.get("scenario_key"),
                            explore_rounds=body.get("explore_rounds"),
                        )
                    )
                    return
                self._send_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)
            except Exception as exc:  # noqa: BLE001
                self._send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)

    return ThreadingHTTPServer((host, port), Handler)
