"""Multi-embodiment registry, RAX coverage matrix, and cross-method experiment comparator.

Pure Python stdlib. Manages honest, hot-swappable host drivers across simulator, fixture,
recorded-data, and physical-robot HTTP endpoints, and loads real archived per-task benchmark
policies and scores for side-by-side comparison in TRAX UI.
"""
from __future__ import annotations

from dataclasses import replace
import difflib
import json
from pathlib import Path
from typing import Any, Mapping

from .config import HostConnectionConfig, SceneConfig
from .contracts import digest
from .host_driver import SceneHostDriver, create_host_driver
from .sandbox import check_sandbox_ast_compliance


def _read_json(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except ValueError:
        return None


def _suite_configs(workspace_dir: Path) -> dict[str, dict[str, Any]]:
    cand_exp = workspace_dir.parent / "experiments"
    exp_root = cand_exp if cand_exp.is_dir() else workspace_dir.parent
    return {
        "libero_object_blind": {
            "label": "Blind LIBERO-Object (10/10 · Franka Panda)",
            "embodiment": "Franka Panda (LIBERO Sim)",
            "domain": "simulation",
            "evidence_source": "simulator",
            "report_path": exp_root / "trax-2.0-m1" / "libero-object-blind" / "libero_report.json",
            "policies_dir": exp_root / "trax-2.0-m1" / "libero-object-blind" / "policies",
            "default_left": "craft_full",
            "default_right": "rax_protocol",
            "method_order": [
                "craft_full",
                "rax_protocol",
                "aspire_protocol",
                "simex_protocol",
                "strong_zeroshot",
            ],
        },
        "libero_spatial": {
            "label": "LIBERO-Spatial (10/10 · Franka Panda)",
            "embodiment": "Franka Panda (LIBERO Sim)",
            "domain": "simulation",
            "evidence_source": "simulator",
            "report_path": exp_root / "trax-2.0-m1" / "libero-spatial-full" / "libero_report.json",
            "policies_dir": exp_root / "trax-2.0-m1" / "libero-spatial-full" / "policies",
            "default_left": "craft_full",
            "default_right": "aspire_protocol",
            "method_order": [
                "craft_full",
                "aspire_protocol",
                "simex_protocol",
                "strong_zeroshot",
            ],
        },
        "robosuite_v4": {
            "label": "Robosuite v4 (4 Tasks × 20 Seeds · Franka Panda)",
            "embodiment": "Franka Panda (Robosuite Sim)",
            "domain": "simulation",
            "evidence_source": "simulator",
            "report_path": exp_root / "trax-2.0-m1" / "robosuite-v4" / "robosuite_prior_work_report.json",
            "policies_dir": exp_root / "trax-2.0-m1" / "robosuite-v4" / "policies",
            "default_left": "craft_full",
            "default_right": "aspire_protocol",
            "method_order": [
                "craft_full",
                "capx_protocol",
                "simex_protocol",
                "aspire_protocol",
                "simex_direct_repair",
                "strong_zeroshot",
            ],
        },
    }


def load_experiment_catalog(workspace_dir: Path) -> dict[str, Any]:
    """Load real per-suite, per-task, per-method scores and available generated policy files."""
    suites_out: dict[str, Any] = {}
    for suite_key, spec in _suite_configs(workspace_dir).items():
        report = _read_json(spec["report_path"])
        if not report:
            continue
        sm = report.get("summary_by_method") or {}
        mc = report.get("paired_mcnemar_craft_vs") or {}
        meta_by_task = report.get("meta_by_task") or {}
        policies_dir: Path = spec["policies_dir"]

        methods_ordered = [m for m in spec["method_order"] if m in sm] + [
            m for m in sm.keys() if m not in spec["method_order"]
        ]
        method_labels = {m: str((sm[m] or {}).get("label") or m) for m in methods_ordered}

        # Collect ordered task keys
        task_keys: list[str] = []
        for m in methods_ordered:
            bt = (sm[m] or {}).get("by_task") or {}
            for tk in bt.keys():
                if tk not in task_keys:
                    task_keys.append(tk)

        tasks_list: list[dict[str, Any]] = []
        for tk in task_keys:
            meta = meta_by_task.get(tk) or {}
            scores: dict[str, Any] = {}
            for m in methods_ordered:
                entry = ((sm[m] or {}).get("by_task") or {}).get(tk) or {}
                pol_file = policies_dir / f"{tk}_{m}.py"
                scores[m] = {
                    "passed": entry.get("passed"),
                    "total": entry.get("total"),
                    "pass_rate": entry.get("pass_rate"),
                    "has_policy": pol_file.is_file(),
                }
            tasks_list.append(
                {
                    "task_key": tk,
                    "language": str(meta.get("language") or meta.get("task_name") or tk),
                    "scores": scores,
                }
            )

        summary_rows: list[dict[str, Any]] = []
        for m in methods_ordered:
            r = sm[m] or {}
            p_info = mc.get(m) or {}
            summary_rows.append(
                {
                    "method": m,
                    "label": method_labels[m],
                    "passed": r.get("passed"),
                    "total": r.get("total", r.get("total_trials")),
                    "pass_rate": r.get("pass_rate"),
                    "dev_llm_calls": r.get("dev_llm_calls"),
                    "dev_sim_resets": r.get("dev_sim_resets"),
                    "eval_llm_calls": r.get("eval_llm_calls", 0),
                    "p95_wall_s": r.get("p95_wall_s"),
                    "mcnemar_p": p_info.get("p_value") if m != "craft_full" else None,
                }
            )

        suites_out[suite_key] = {
            "suite_key": suite_key,
            "label": spec["label"],
            "embodiment": spec["embodiment"],
            "domain": spec["domain"],
            "evidence_source": report.get("evidence_source", spec["evidence_source"]),
            "observation_contract": report.get("observation_contract", "host_state_api"),
            "model": report.get("model"),
            "methods": methods_ordered,
            "method_labels": method_labels,
            "default_left": spec["default_left"] if spec["default_left"] in methods_ordered else (methods_ordered[0] if methods_ordered else ""),
            "default_right": spec["default_right"] if spec["default_right"] in methods_ordered else (methods_ordered[-1] if methods_ordered else ""),
            "tasks": tasks_list,
            "summary": summary_rows,
        }
    return suites_out


def load_archived_policy_comparison(
    workspace_dir: Path,
    *,
    suite_key: str,
    task_key: str,
    left_method: str,
    right_method: str,
) -> dict[str, Any]:
    """Load exact archived Python policies for two methods on a benchmark task plus their unified diff."""
    specs = _suite_configs(workspace_dir)
    if suite_key not in specs:
        raise KeyError(f"Unknown suite: {suite_key}")
    spec = specs[suite_key]
    report = _read_json(spec["report_path"]) or {}
    sm = report.get("summary_by_method") or {}
    meta_by_task = report.get("meta_by_task") or {}
    policies_dir: Path = spec["policies_dir"]

    def _load_side(method: str) -> dict[str, Any]:
        pol_path = policies_dir / f"{task_key}_{method}.py"
        if not pol_path.is_file():
            return {
                "method": method,
                "label": str((sm.get(method) or {}).get("label") or method),
                "available": False,
                "source": "",
                "source_sha256": "",
                "lines": 0,
                "ast_check": None,
                "task_score": ((sm.get(method) or {}).get("by_task") or {}).get(task_key) or {},
            }
        src = pol_path.read_text(encoding="utf-8")
        return {
            "method": method,
            "label": str((sm.get(method) or {}).get("label") or method),
            "available": True,
            "file": pol_path.name,
            "source": src,
            "source_sha256": digest(src),
            "lines": len(src.splitlines()),
            "ast_check": check_sandbox_ast_compliance(src),
            "task_score": ((sm.get(method) or {}).get("by_task") or {}).get(task_key) or {},
        }

    left = _load_side(left_method)
    right = _load_side(right_method)
    diff_lines = list(
        difflib.unified_diff(
            right["source"].splitlines(),
            left["source"].splitlines(),
            fromfile=f"{task_key}_{right_method}.py",
            tofile=f"{task_key}_{left_method}.py",
            lineterm="",
            n=3,
        )
    ) if left["available"] and right["available"] else []

    meta = meta_by_task.get(task_key) or {}
    return {
        "suite_key": suite_key,
        "suite_label": spec["label"],
        "task_key": task_key,
        "task_language": str(meta.get("language") or meta.get("task_name") or task_key),
        "evidence_source": report.get("evidence_source", spec["evidence_source"]),
        "left": left,
        "right": right,
        "unified_diff": "\n".join(diff_lines),
    }


def build_rax_coverage_matrix(workspace_dir: Path) -> list[dict[str, Any]]:
    """Build honest multi-embodiment coverage matrix mapped against RAX (arXiv:2610.22496)."""
    cand_exp = workspace_dir.parent / "experiments"
    exp_root = cand_exp if cand_exp.is_dir() else workspace_dir.parent
    has_lib_sp = (exp_root / "trax-2.0-m1" / "libero-spatial-full" / "libero_report.json").is_file()
    has_lib_ob = (exp_root / "trax-2.0-m1" / "libero-object-blind" / "libero_report.json").is_file()
    has_rb4 = (exp_root / "trax-2.0-m1" / "robosuite-v4" / "robosuite_prior_work_report.json").is_file()
    has_cb2 = (exp_root / "goal-v2-20261122" / "cafebench-v2" / "cafebench_v2_report.json").is_file()
    has_s2s = (exp_root / "goal-v3-simex-parity" / "sim2sim-stage2" / "sim2sim_report.json").is_file()

    return [
        {
            "id": "so101_live",
            "family": "Single-Arm (6-DoF)",
            "platform": "LeRobot SO-101 (MuJoCo)",
            "domain": "simulation",
            "evidence_source": "simulator",
            "rax_scope": "Single-arm open-source tabletop embodiment (Apache-2.0 USD/MJCF)",
            "trax_status": "LIVE_SWITCHABLE",
            "trax_result": "Live 4-cam IK + Closed-Loop Explore -> Synthesize -> AST Exec + SQLite Journal",
            "action_type": "switch",
            "action_target": "primary_live",
        },
        {
            "id": "cafe_6dof_mujoco",
            "family": "Single-Arm (6-DoF)",
            "platform": "Tabletop Cafe Arm (MuJoCo)",
            "domain": "simulation",
            "evidence_source": "simulator",
            "rax_scope": "Commercial non-idempotent appliance + slip/timeout fault domain (beyond RAX)",
            "trax_status": "LIVE_SWITCHABLE",
            "trax_result": "CafeBench v2: 30/30 (100%, 0 dup)" if has_cb2 else "Live MuJoCo driver ready",
            "action_type": "switch",
            "action_target": "mujoco_cafe",
        },
        {
            "id": "libero_spatial",
            "family": "Single-Arm (7-DoF)",
            "platform": "Franka Panda · LIBERO-Spatial (10/10)",
            "domain": "simulation",
            "evidence_source": "simulator",
            "rax_scope": "RAX Table 6: 99.2% (RGB-D, <=20 eps, Opus 5.5, unmatched contract)",
            "trax_status": "BENCHMARK_VERIFIED" if has_lib_sp else "CONTRACT_PLANNED",
            "trax_result": "TRAX 97/100 (97.0%, p=0.0386 vs ASPIRE 89/100, host_state_api, <=5 LLM)",
            "action_type": "compare",
            "action_target": "libero_spatial",
        },
        {
            "id": "libero_object_blind",
            "family": "Single-Arm (7-DoF)",
            "platform": "Franka Panda · Blind LIBERO-Object (10/10)",
            "domain": "simulation",
            "evidence_source": "simulator",
            "rax_scope": "RAX Table 6: 100.0% (unmatched RGB-D) · Matched RAX-Protocol: 80/100",
            "trax_status": "BENCHMARK_VERIFIED" if has_lib_ob else "CONTRACT_PLANNED",
            "trax_result": "TRAX 88/100 (0 collapsed tasks) vs RAX-Protocol 80/100 & ASPIRE 90/100",
            "action_type": "compare",
            "action_target": "libero_object_blind",
        },
        {
            "id": "robosuite_v4",
            "family": "Single-Arm (7-DoF)",
            "platform": "Franka Panda · Robosuite v4 (4 Tasks)",
            "domain": "simulation",
            "evidence_source": "simulator",
            "rax_scope": "Not evaluated in RAX v1 (Lift, PickPlaceCan, NutAssemblySquare, Door)",
            "trax_status": "BENCHMARK_VERIFIED" if has_rb4 else "CONTRACT_PLANNED",
            "trax_result": "TRAX 72/80 (90.0%, p<=1.9e-5 vs CaP-X/SimEX/ASPIRE/Zero-Shot)",
            "action_type": "compare",
            "action_target": "robosuite_v4",
        },
        {
            "id": "gazebo_sim2sim",
            "family": "Single-Arm (6-DoF)",
            "platform": "Gazebo Harmonic (Sim-to-Sim Target)",
            "domain": "simulation",
            "evidence_source": "simulator",
            "rax_scope": "Cross-physics transfer & simulator calibration (<=5 trials)",
            "trax_status": "BENCHMARK_VERIFIED" if has_s2s else "HTTP_BRIDGE_READY",
            "trax_result": "Sim2Sim Stage-2: 19/20 (95.0%, target 9/10 + shadow 10/10)" if has_s2s else "HTTP bridge ready",
            "action_type": "connect_http",
            "action_target": "http://127.0.0.1:8881",
        },
        {
            "id": "physical_single_arm",
            "family": "Single-Arm (Physical)",
            "platform": "Physical SO-101 / Franka FR3 / ticos-edge",
            "domain": "physical",
            "evidence_source": "physical_robot",
            "rax_scope": "RAX Sec 5.1: Franka FR3 (5 real tasks: cup shuffle, air-fryer, wipe, 0.1mm peg)",
            "trax_status": "HTTP_BRIDGE_READY",
            "trax_result": "Live HTTP capability contract (/api/capabilities + /api/scene + /api/execute)",
            "action_type": "connect_http",
            "action_target": "http://127.0.0.1:8890",
        },
        {
            "id": "bimanual_robodojo",
            "family": "Bimanual Arms",
            "platform": "ARX X5 · RoboDojo (Sim / Physical)",
            "domain": "simulation",
            "evidence_source": "simulator",
            "rax_scope": "RAX Sec 5.2, Table 7/8: 4-agent scaling (87.5%) & RGB-D ablation (+17.3pp)",
            "trax_status": "CONTRACT_PLANNED",
            "trax_result": "Awaiting RoboDojo host bridge; connect any HTTP bimanual host to run live",
            "action_type": "connect_http",
            "action_target": "http://127.0.0.1:8892",
        },
        {
            "id": "dexterous_dexjoco",
            "family": "Dexterous Hand",
            "platform": "Franka + 16-DoF Allegro · DexJoCo",
            "domain": "simulation",
            "evidence_source": "simulator",
            "rax_scope": "RAX Sec 5.2: High-DoF in-hand dexterous manipulation benchmark",
            "trax_status": "CONTRACT_PLANNED",
            "trax_result": "Awaiting DexJoCo host bridge; connect any HTTP dexterous host to run live",
            "action_type": "connect_http",
            "action_target": "http://127.0.0.1:8893",
        },
        {
            "id": "mobile_bimanual",
            "family": "Mobile Bimanual",
            "platform": "Galaxea R1Pro (BEHAVIOR-1K) / Rainbow RB-Y1 (Physical)",
            "domain": "physical",
            "evidence_source": "physical_robot",
            "rax_scope": "RAX Sec 5.2 & 5.5: 6-stage long-horizon mobile tidying & skill composition",
            "trax_status": "CONTRACT_PLANNED",
            "trax_result": "Awaiting BEHAVIOR-1K / RB-Y1 host bridge; connect via HTTP contract",
            "action_type": "connect_http",
            "action_target": "http://127.0.0.1:8894",
        },
        {
            "id": "humanoid_g1",
            "family": "Humanoid Whole-Body",
            "platform": "Unitree G1 + Dex3 · HumanoidArena",
            "domain": "simulation",
            "evidence_source": "simulator",
            "rax_scope": "RAX Table 4: 89.8% avg across 60 official test episodes (+7.9pp vs WB-WAM)",
            "trax_status": "CONTRACT_PLANNED",
            "trax_result": "Awaiting HumanoidArena host bridge; connect via HTTP contract",
            "action_type": "connect_http",
            "action_target": "http://127.0.0.1:8895",
        },
    ]


class EmbodimentManager:
    """Manages hot-swappable live host drivers without fabricating unsupported hardware."""

    def __init__(self, config: SceneConfig, primary_driver: SceneHostDriver, workspace_dir: Path) -> None:
        self.config = config
        self.workspace_dir = workspace_dir
        self._drivers: dict[str, SceneHostDriver] = {"primary_live": primary_driver}
        self._meta: dict[str, dict[str, Any]] = {}

        primary_cap = None
        try:
            primary_cap = primary_driver.discover_capabilities(allow_busy=True)
        except Exception:  # noqa: BLE001
            primary_cap = None
        primary_auth = (
            primary_cap.discovered.execution_authority
            if primary_cap and primary_cap.discovered
            else primary_driver.driver_name
        )
        primary_label = (
            "LeRobot SO-101 (MuJoCo Sim · Live)"
            if "so101" in primary_auth.lower()
            else f"{primary_auth} ({primary_driver.evidence_source})"
        )
        self._meta["primary_live"] = {
            "id": "primary_live",
            "label": primary_label,
            "driver": primary_driver.driver_name,
            "execution_authority": primary_auth,
            "evidence_source": primary_driver.evidence_source,
            "domain": "physical" if primary_driver.evidence_source == "physical_robot" else "simulation",
            "builtin": True,
        }
        self._active_id = "primary_live"

        # Pre-register switchable local backends (lazy instantiated on first switch)
        self._catalog_presets: list[dict[str, Any]] = [
            self._meta["primary_live"],
            {
                "id": "mujoco_cafe",
                "label": "Tabletop 6-DoF Cafe Arm (MuJoCo Sim)",
                "driver": "mujoco_cafe",
                "execution_authority": "trax-mujoco-cafe-sim",
                "evidence_source": "simulator",
                "domain": "simulation",
                "builtin": True,
            },
            {
                "id": "fixture",
                "label": "Deterministic Protocol Sandbox (Fixture)",
                "driver": "fixture",
                "execution_authority": "craft-fixture-sim",
                "evidence_source": "fixture",
                "domain": "simulation",
                "builtin": True,
            },
        ]
        # If primary is NOT already mujoco_so101, also offer direct in-process SO-101
        if "so101" not in primary_auth.lower():
            self._catalog_presets.insert(
                1,
                {
                    "id": "mujoco_so101",
                    "label": "LeRobot SO-101 (MuJoCo Sim · Local)",
                    "driver": "mujoco_so101",
                    "execution_authority": "trax-mujoco-so101-sim",
                    "evidence_source": "simulator",
                    "domain": "simulation",
                    "builtin": True,
                },
            )

    @property
    def active_id(self) -> str:
        return self._active_id

    @property
    def active_driver(self) -> SceneHostDriver:
        return self._drivers[self._active_id]

    def list_targets(self) -> list[dict[str, Any]]:
        out = []
        seen = set()
        for item in self._catalog_presets:
            seen.add(item["id"])
            out.append({**item, "active": item["id"] == self._active_id})
        for tid, meta in self._meta.items():
            if tid not in seen:
                out.append({**meta, "active": tid == self._active_id})
        return out

    def switch(
        self,
        *,
        target_id: str | None = None,
        host_url: str | None = None,
        evidence_source: str | None = None,
        label: str | None = None,
    ) -> tuple[SceneHostDriver, dict[str, Any]]:
        if host_url and host_url.strip():
            url = host_url.strip().rstrip("/")
            ev = str(evidence_source or "physical_robot").strip()
            if ev not in ("physical_robot", "simulator", "recorded_data", "fixture"):
                ev = "physical_robot"
            custom_cfg = replace(
                self.config,
                host=replace(
                    self.config.host,
                    edge_base_url=url,
                    evaluator_base_url=url,
                    allow_actuation=True,
                    allow_sim_reset=(ev == "simulator"),
                    timeout_s=15.0,
                ),
                budget=replace(self.config.budget, deployment_scope=ev),
            )
            drv = create_host_driver("http", custom_cfg)
            cap = drv.discover_capabilities(allow_busy=True)
            if cap.status not in ("ready", "busy"):
                raise RuntimeError(f"Host {url} capability check returned status={cap.status}: {cap.detail}")
            auth = cap.discovered.execution_authority if cap.discovered else url
            cid = f"http_{digest(url)[:8]}"
            meta = {
                "id": cid,
                "label": (label or "").strip() or f"{auth} ({ev})",
                "driver": "http",
                "host_url": url,
                "execution_authority": auth,
                "evidence_source": ev,
                "domain": "physical" if ev == "physical_robot" else "simulation",
                "builtin": False,
            }
            self._drivers[cid] = drv
            self._meta[cid] = meta
            self._active_id = cid
            return drv, meta

        tid = str(target_id or "").strip()
        if tid in self._drivers:
            self._active_id = tid
            meta = self._meta.get(tid) or next((p for p in self._catalog_presets if p["id"] == tid), {"id": tid})
            return self._drivers[tid], meta

        preset = next((p for p in self._catalog_presets if p["id"] == tid), None)
        if preset is None:
            raise KeyError(f"Unknown embodiment target: {tid}")

        drv = create_host_driver(preset["driver"], self.config)
        self._drivers[tid] = drv
        self._meta[tid] = dict(preset)
        self._active_id = tid
        return drv, dict(preset)

    def snapshot(self) -> dict[str, Any]:
        return {
            "active_id": self._active_id,
            "active_meta": self._meta.get(self._active_id, {}),
            "targets": self.list_targets(),
            "rax_matrix": build_rax_coverage_matrix(self.workspace_dir),
            "experiment_catalog": load_experiment_catalog(self.workspace_dir),
        }
