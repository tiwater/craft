"""Standalone, host-independent web observer and operations interface (`trax.ui`).

Built purely on Python's stdlib ``http.server`` with real static frontend files under
``trax/ui/static/`` (zero external web dependencies, zero core private-symbol coupling,
zero built-in robot URDFs—URDF and meshes are fetched dynamically from the active
embodiment host via MCP/HTTP with strict path-traversal and ETag handling).
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
import urllib.request

from trax.agent_engine import TraxAgentEngine
from trax.autonomy import AutonomyPromotionLedger
from trax.benchmark import (
    PolicyEntry,
    builtin_reference_policy_source,
    render_benchmark_html,
)
from trax.config import SceneConfig
from trax.embodiments import EmbodimentManager, load_archived_policy_comparison
from trax.host_driver import SceneHostDriver
from trax.mcp_protocol import MCPClient, MCPInspectorController
from trax.operations import run_continuous_shift
from trax.plugin import read_safe_robot_asset, rewrite_urdf_mesh_uris
from trax.portability import check_policy_portability
from trax.profiler import aggregate_trial_profiles
from trax.regression import RegressionSuite
from trax.release import ReleaseRegistry
from trax.self_audit import AuditLedger

_STATIC_DIR = Path(__file__).resolve().parent / "static"
_STATIC_MIME: dict[str, str] = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".svg": "image/svg+xml",
    ".png": "image/png",
}


def _read_optional_json(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except ValueError:
        return None


def _load_m1_archive_summary(workspace_dir: Path) -> dict[str, Any]:
    """Load optional archived cross-benchmark summary reports for UI comparison tab."""
    cand_exp = workspace_dir.parent / "experiments"
    exp_root = cand_exp if cand_exp.is_dir() else workspace_dir.parent
    out: dict[str, Any] = {}
    for key, rel in (
        ("libero_spatial", "trax-2.0-m1/libero-spatial-full/libero_report.json"),
        ("libero_object_blind", "trax-2.0-m1/libero-object-blind/libero_report.json"),
        ("robosuite_v4", "trax-2.0-m1/robosuite-v4/robosuite_prior_work_report.json"),
        ("cafebench_v2", "goal-v2-20261122/cafebench-v2/cafebench_v2_report.json"),
    ):
        data = _read_optional_json(exp_root / rel)
        if data:
            out[key] = {
                "evidence_source": data.get("evidence_source", "simulator"),
                "summary_by_method": data.get("summary_by_method") or {},
            }
    return out


class TraxUIApp:
    """Thread-safe controller backing optional TRAX UI HTTP endpoints."""

    def __init__(
        self,
        config: SceneConfig,
        driver: SceneHostDriver,
        workspace_dir: str | Path,
        *,
        policies: Sequence[PolicyEntry] = (),
        default_mcp_url: str | None = None,
        default_mcp_stdio: str | None = None,
    ) -> None:
        self.config = config
        self.driver = driver
        self.workspace_dir = Path(workspace_dir)
        self.workspace_dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
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
        self._default_mcp_url = (default_mcp_url or "").strip() or None
        self._default_mcp_stdio = [tok for tok in (default_mcp_stdio or "").split() if tok] or None
        self._stdio_client: MCPClient | None = (
            MCPClient(transport="stdio", command=self._default_mcp_stdio, timeout_s=15.0)
            if self._default_mcp_stdio
            else None
        )

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

        self.embodiments = EmbodimentManager(self.config, self.driver, self.workspace_dir)
        self.mcp = MCPInspectorController(
            self.workspace_dir,
            lambda: self.driver,
            self._lock,
        )
        self._m1_archive = _load_m1_archive_summary(self.workspace_dir)
        self.agent = TraxAgentEngine(
            self.workspace_dir,
            self._resolve_active_mcp_client,
            on_wire=self.mcp.record_external_wire,
        )

    def close(self) -> None:
        self.mcp.close()
        if self._stdio_client is not None:
            self._stdio_client.close()
            self._stdio_client = None

    def _resolve_active_mcp_client(self) -> tuple[MCPClient, str]:
        if self._stdio_client is not None and self._default_mcp_stdio:
            return self._stdio_client, " ".join(self._default_mcp_stdio)
        if self._default_mcp_url:
            return MCPClient(transport="http", url=self._default_mcp_url, timeout_s=15.0), self._default_mcp_url
        drv = self.driver
        if drv.driver_name == "http" and hasattr(drv, "config"):
            base = drv.config.host.normalized_base_url  # type: ignore[attr-defined]
            mcp_url = f"{base}/mcp"
            return MCPClient(transport="http", url=mcp_url, timeout_s=15.0), f"Active HTTP Host ({mcp_url})"
        from trax.mcp_protocol import build_driver_mcp_server

        return (
            MCPClient(transport="local", local_server=build_driver_mcp_server(drv, self._lock)),
            f"In-Process Backend ({drv.driver_name})",
        )

    def get_agent_state(self) -> dict[str, Any]:
        st = self.agent.get_state()
        st["m1_archive"] = self._m1_archive
        return st

    def _fetch_live_mcp_telemetry(self) -> dict[str, Any] | None:
        try:
            client, _ = self._resolve_active_mcp_client()
            _, resp, _ = client.rpc("resources/read", {"uri": "trax://scene/state"})
            contents = (resp.get("result") or {}).get("contents") or []
            if contents and contents[0].get("text"):
                return json.loads(str(contents[0]["text"]))
        except Exception:  # noqa: BLE001
            return None
        return None

    def fetch_embodiment_manifest(self) -> dict[str, Any] | None:
        try:
            client, _ = self._resolve_active_mcp_client()
            _, resp, _ = client.rpc("resources/read", {"uri": "trax://embodiment/manifest"})
            contents = (resp.get("result") or {}).get("contents") or []
            if contents and contents[0].get("text"):
                return json.loads(str(contents[0]["text"]))
        except Exception:  # noqa: BLE001
            return None
        return None

    def get_audit_ledger_snapshot(self) -> dict[str, Any]:
        ledger_path = self.workspace_dir / "traces" / "self_audit_ledger.jsonl"
        rows = AuditLedger(ledger_path).read_all()
        return {
            "ledger_path": "workspace/traces/self_audit_ledger.jsonl",
            "records": rows[-20:],
            "manifest": self.fetch_embodiment_manifest(),
        }

    def fetch_robot_urdf(self) -> str | None:
        """Fetch active embodiment URDF via MCP `trax://robot/urdf` and normalize mesh URLs for browser."""
        try:
            client, _ = self._resolve_active_mcp_client()
            _, resp, _ = client.rpc("resources/read", {"uri": "trax://robot/urdf"})
            contents = (resp.get("result") or {}).get("contents") or []
            if contents and contents[0].get("text"):
                raw_urdf = str(contents[0]["text"])
                return rewrite_urdf_mesh_uris(raw_urdf, asset_url_prefix="api/robot/assets")
        except Exception:  # noqa: BLE001
            return None
        return None

    def fetch_robot_asset(self, rel_path: str) -> tuple[bytes, str, str] | None:
        """Fetch validated binary robot asset (mesh/URDF) from local plugin or remote HTTP host."""
        resolver = getattr(self.driver, "resolve_robot_asset", None)
        if callable(resolver):
            return resolver(rel_path)
        if self.driver.driver_name == "http" and hasattr(self.driver, "config"):
            cleaned = (rel_path or "").strip().lstrip("/")
            if not cleaned or ".." in Path(cleaned).parts:
                return None
            base = self.driver.config.host.normalized_base_url  # type: ignore[attr-defined]
            url = f"{base}/assets/robot/{cleaned}"
            try:
                with self._opener.open(url, timeout=8.0) as resp:
                    data = resp.read()
                    mime = resp.headers.get("Content-Type") or "application/octet-stream"
                    etag = resp.headers.get("ETag") or ""
                    return data, mime, etag
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
            "scene_observation": {
                **obs.to_dict(),
                "raw_snapshot": obs.raw_snapshot,
            },
            "live_runtime": self.driver.get_live_runtime(),
            "run_state": self._snapshot_run_state(),
            "cameras": list(self.driver.list_cameras()),
            "policies": [p.to_meta() for p in self._policies.values()],
            "benchmark": bm,
            "workspace_summary": _read_optional_json(self.workspace_dir / "summary.json"),
            "registry": _read_optional_json(self.workspace_dir / "registry" / "registry.json"),
            "phase2_profiler": profiler_data,
            "regression_cases": reg_cases,
            "portability_checks": portability,
            "autonomy_physical_gate": promo_gate,
            "latest_shift_report": _read_optional_json(self.workspace_dir / "latest_shift_report.json"),
            "screening_sessions": scr_sessions,
        }

    def get_policy(self, policy_id: str) -> dict[str, Any]:
        pol = self._policies[policy_id]
        return {**pol.to_meta(), "source": pol.source}

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


def _read_static_asset(rel_name: str) -> tuple[bytes, str] | None:
    cleaned = (rel_name or "").strip().lstrip("/")
    if not cleaned or ".." in Path(cleaned).parts:
        return None
    root = _STATIC_DIR.resolve()
    cand = (root / cleaned).resolve()
    try:
        cand.relative_to(root)
    except ValueError:
        return None
    if not cand.is_file():
        return None
    mime = _STATIC_MIME.get(cand.suffix.lower(), "application/octet-stream")
    return cand.read_bytes(), mime


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

        def _send_bytes(self, data: bytes, content_type: str, status: int = 200, *, cache_control: str = "no-store", etag: str = "") -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", cache_control)
            if etag:
                self.send_header("ETag", etag)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _read_json(self) -> dict[str, Any]:
            length = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(length) if length > 0 else b"{}"
            return json.loads(raw.decode("utf-8")) if raw.strip() else {}

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
                asset = _read_static_asset("index.html")
                if asset is None:
                    self._send_json({"error": "index.html missing"}, HTTPStatus.INTERNAL_SERVER_ERROR)
                else:
                    self._send_bytes(asset[0], asset[1])
                return
            if path.startswith("/static/"):
                rel = path[len("/static/") :]
                asset = _read_static_asset(rel)
                if asset is None:
                    self._send_json({"error": f"Static asset not found: {rel}"}, HTTPStatus.NOT_FOUND)
                else:
                    self._send_bytes(asset[0], asset[1], cache_control="public, max-age=60")
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
            if path == "/api/audit/ledger":
                self._send_json(app.get_audit_ledger_snapshot())
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
            if path == "/api/robot/urdf":
                urdf_xml = app.fetch_robot_urdf()
                if urdf_xml is None:
                    self._send_json({"error": "Active embodiment does not expose a URDF resource"}, HTTPStatus.NOT_FOUND)
                else:
                    self._send_bytes(urdf_xml.encode("utf-8"), "application/xml; charset=utf-8")
                return
            if path.startswith("/api/robot/assets/"):
                rel = path[len("/api/robot/assets/") :]
                asset = app.fetch_robot_asset(rel)
                if asset is None:
                    self._send_json({"error": f"Robot asset not found: {rel}"}, HTTPStatus.NOT_FOUND)
                else:
                    data, mime, etag = asset
                    if etag and self.headers.get("If-None-Match") == etag:
                        self.send_response(HTTPStatus.NOT_MODIFIED)
                        self.send_header("ETag", etag)
                        self.end_headers()
                        return
                    self._send_bytes(data, mime, cache_control="public, max-age=3600", etag=etag)
                return
            if path == "/api/compare/policy":
                try:
                    self._send_json(app.compare_archived_policy(parse_qs(parsed.query)))
                except KeyError as exc:
                    self._send_json({"error": str(exc)}, HTTPStatus.NOT_FOUND)
                return
            if path == "/benchmark_report.html":
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
                if path == "/api/kb/run_skill":
                    self._send_json(app.agent.start_run_skill_async(body))
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
                if path == "/api/evolve/verify_seeds":
                    self._send_json(app.agent.start_verify_multiseed_async(body))
                    return
                if path == "/api/evolve/optimize_speed":
                    self._send_json(app.agent.start_optimize_speed_async(body))
                    return
                if path == "/api/evolve/auto":
                    self._send_json(app.agent.start_closed_loop_async(body))
                    return
                if path == "/api/evolve/abort":
                    self._send_json(app.agent.abort(str(body.get("reason") or "Aborted by operator")))
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
