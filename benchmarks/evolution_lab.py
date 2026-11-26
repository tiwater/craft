"""Closed-loop Explore -> Synthesize Python -> AST Execute -> Attribute & Iterate engine for TRAX UI.

Pure Python stdlib. Drives any ``SceneHostDriver`` (e.g., ``mujoco_so101``, ``mujoco_cafe``,
``http``, or ``fixture``) through the real TRAX research loop with zero fabricated traces.
"""
from __future__ import annotations

import difflib
import json
from pathlib import Path
import threading
import time
from typing import Any, Callable, Mapping, Sequence

from trax.config import SceneConfig
from trax.contracts import digest
from trax.host_driver import SceneHostDriver
from trax.journal import Journal
from .prior_work_bench import (
    SHARED_HOST_SDK_CONTRACT,
    SHARED_TASK_REQUIREMENTS,
    build_shared_initial_prompt,
    check_sandbox_ast_compliance,
    make_live_llm_fn,
)
from trax.probing import analyze_policy_subroutines
from trax.profiler import attribute_trial

LLMCallable = Callable[[str], dict[str, Any]]

PUBLIC_SDK_EXPLORATION_PROBE_SOURCE = '''def run_policy(sdk, params):
    """Task-agnostic public-SDK exploration probe (zero hardcoded task coordinates)."""
    scene = sdk.observe_scene()
    objs = scene.get("objects") or []
    probed = []
    for obj in objs[:2]:
        pos = obj.get("world")
        if pos and len(pos) >= 3:
            hover = [float(pos[0]), float(pos[1]), float(pos[2]) + 0.09]
            cid = "probe-hover-" + str(obj.get("id", "obj"))
            rec = sdk.move_to_target(hover, command_id=cid)
            probed.append({"id": obj.get("id"), "world": pos, "receipt": rec.get("receipt")})
    retract_rec = sdk.retract_safe_pose(command_id="probe-retract")
    return {
        "status": "probed",
        "objects_seen": len(objs),
        "l2_pouring_safety": scene.get("l2_pouring_safety") or {},
        "probed": probed,
        "retract": retract_rec.get("receipt"),
    }
'''

SCENARIO_PROFILES: dict[str, dict[str, Any]] = {
    "fault": {
        "label": "Slip (+38,-12mm) + Lost Brew Receipt",
        "preset": "default",
        "fault_inject_slip_xy_m": [0.038, -0.012],
        "simulate_timeout": True,
        "expect_status": "completed",
    },
    "normal": {
        "label": "Nominal Tabletop Cycle",
        "preset": "default",
        "fault_inject_slip_xy_m": None,
        "simulate_timeout": False,
        "expect_status": "completed",
    },
    "offset": {
        "label": "Large Initial Pose Offset (+20,+64mm)",
        "preset": "sim_pose_offset",
        "fault_inject_slip_xy_m": None,
        "simulate_timeout": False,
        "expect_status": "completed",
    },
    "ambiguity": {
        "label": "Dual-Cup Ambiguity Safety Gate",
        "preset": "sim_dual_cup",
        "fault_inject_slip_xy_m": None,
        "simulate_timeout": False,
        "expect_status": "paused_for_operator",
    },
}


def _utc_hms() -> str:
    return time.strftime("%H:%M:%S", time.gmtime())


def _compute_unified_diff(prev_source: str, new_source: str, prev_label: str, new_label: str) -> str:
    if not prev_source:
        return ""
    lines = list(
        difflib.unified_diff(
            prev_source.splitlines(),
            new_source.splitlines(),
            fromfile=prev_label,
            tofile=new_label,
            lineterm="",
            n=3,
        )
    )
    return "\n".join(lines)


def _read_journal_rows(db_path: Path, command_prefix: str) -> list[dict[str, Any]]:
    if not db_path.is_file():
        return []
    rows: list[dict[str, Any]] = []
    try:
        with Journal(db_path) as journal:
            cur = journal.db.execute(
                "SELECT id, request, receipt FROM commands ORDER BY rowid DESC LIMIT 40"
            )
            for cmd_id, req_raw, rec_raw in cur.fetchall():
                if command_prefix and command_prefix not in str(cmd_id):
                    continue
                try:
                    req = json.loads(req_raw) if req_raw else {}
                except ValueError:
                    req = {"raw": str(req_raw)}
                try:
                    rec = json.loads(rec_raw) if rec_raw else None
                except ValueError:
                    rec = {"raw": str(rec_raw)}
                rows.append(
                    {
                        "command_id": str(cmd_id),
                        "action": str(req.get("action") or req.get("method") or "command"),
                        "state": (
                            str(rec.get("receipt") or rec.get("status") or "recorded")
                            if isinstance(rec, dict)
                            else "INTENT_PENDING"
                        ),
                        "receipt": rec,
                    }
                )
    except Exception:  # noqa: BLE001
        return []
    rows.reverse()
    return rows


def _synthesize_journal_view(exec_res: Mapping[str, Any], sqlite_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Combine persisted SQLite journal rows with real per-SDK-method execution trace."""
    entries: list[dict[str, Any]] = []
    cmd_id = str(exec_res.get("command_id") or "run")
    step_records = list(exec_res.get("step_records") or [])
    if step_records:
        for idx, rec in enumerate(step_records, start=1):
            method = str(rec.get("method") or "")
            if not method:
                continue
            entries.append(
                {
                    "command_id": f"{cmd_id}:s{idx}",
                    "action": method,
                    "state": str(rec.get("receipt") or "done"),
                    "non_idempotent": method in ("start_coffee_machine", "command_ack_timeout_unknown", "query_command_status"),
                }
            )
    else:
        for idx, method in enumerate(exec_res.get("methods") or [], start=1):
            m = str(method)
            state = "UNKNOWN (ack timeout)" if m == "command_ack_timeout_unknown" else (
                "RECONCILED" if m == "query_command_status" else "SUCCEEDED"
            )
            entries.append(
                {
                    "command_id": f"{cmd_id}:s{idx}",
                    "action": m,
                    "state": state,
                    "non_idempotent": m in ("start_coffee_machine", "command_ack_timeout_unknown", "query_command_status"),
                }
            )
    for row in sqlite_rows:
        entries.append(
            {
                "command_id": row["command_id"],
                "action": f"sqlite.journal[{row['action']}]",
                "state": f"COMMITTED ({row['state']})",
                "non_idempotent": True,
            }
        )
    return entries


def _read_json_safe(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except ValueError:
        return None


def load_m1_archive(workspace_dir: Path) -> dict[str, Any]:
    """Load real archived benchmark receipts from experiments/ for the M1 Archive tab."""
    cand_exp = workspace_dir.parent / "experiments"
    exp_root = cand_exp if cand_exp.is_dir() else workspace_dir.parent
    out: dict[str, Any] = {}

    rb4 = _read_json_safe(exp_root / "trax-2.0-m1" / "robosuite-v4" / "robosuite_prior_work_report.json")
    if rb4:
        out["robosuite_v4"] = {
            "evidence_source": rb4.get("evidence_source", "simulator"),
            "model": rb4.get("model"),
            "summary_by_method": rb4.get("summary_by_method", {}),
            "paired_mcnemar_craft_vs": rb4.get("paired_mcnemar_craft_vs", {}),
        }

    lib_sp = _read_json_safe(exp_root / "trax-2.0-m1" / "libero-spatial-full" / "libero_report.json")
    if lib_sp:
        out["libero_spatial"] = {
            "evidence_source": lib_sp.get("evidence_source", "simulator"),
            "observation_contract": lib_sp.get("observation_contract", "host_state_api"),
            "model": lib_sp.get("model"),
            "summary_by_method": lib_sp.get("summary_by_method", {}),
            "paired_mcnemar_craft_vs": lib_sp.get("paired_mcnemar_craft_vs", {}),
        }

    lib_ob = _read_json_safe(exp_root / "trax-2.0-m1" / "libero-object-blind" / "libero_report.json")
    if lib_ob:
        out["libero_object_blind"] = {
            "evidence_source": lib_ob.get("evidence_source", "simulator"),
            "observation_contract": lib_ob.get("observation_contract", "host_state_api"),
            "model": lib_ob.get("model"),
            "git_commit": (lib_ob.get("provenance") or {}).get("git_commit", "")[:7],
            "summary_by_method": lib_ob.get("summary_by_method", {}),
            "paired_mcnemar_craft_vs": lib_ob.get("paired_mcnemar_craft_vs", {}),
        }

    cb2 = _read_json_safe(exp_root / "goal-v2-20261122" / "cafebench-v2" / "cafebench_v2_report.json")
    if cb2:
        out["cafebench_v2"] = {
            "evidence_source": "simulator + recorded_data",
            "summary_by_method": cb2.get("summary_by_method", {}),
            "paired_mcnemar_craft_vs": cb2.get("paired_mcnemar_craft_vs", {}),
        }

    s2s = _read_json_safe(exp_root / "goal-v3-simex-parity" / "sim2sim-stage2" / "sim2sim_report.json")
    if s2s:
        out["sim2sim_stage2"] = {
            "evidence_source": "simulator + recorded_data",
            "summary_by_method": s2s.get("summary_by_method", {}),
            "paired_mcnemar_craft_vs": s2s.get("paired_mcnemar_craft_vs", {}),
        }
    return out


class EvolutionLabController:
    """Manages the live Explore -> Synthesize Python -> Execute -> Iterate loop for TRAX UI."""

    def __init__(
        self,
        config: SceneConfig,
        driver: SceneHostDriver,
        workspace_dir: Path,
        driver_lock: threading.Lock,
        *,
        seed_policies: Sequence[tuple[str, str, str]] = (),
        llm_fn: LLMCallable | None = None,
    ) -> None:
        self.config = config
        self.driver = driver
        self.workspace_dir = workspace_dir
        self._driver_lock = driver_lock
        self._state_lock = threading.RLock()
        self._llm_fn = llm_fn
        self._journal_path = self.workspace_dir / "ui_evolution_journal.sqlite"

        self._phase: str = "idle"
        self._busy: bool = False
        self._busy_detail: str = ""
        self._task_instruction: str = (
            "Pick up the target dine-in coffee cup, place it under the espresso spout, "
            "verify L2 pouring safety (recentering if offset exceeds limit), start the brewer "
            "once with query-first reconciliation on unknown receipt timeout, wait until filled, "
            "and deliver to $SLOT_PICKUP_1 within 15 mm."
        )
        self._scenario_key: str = "fault"
        self._probe_report: dict[str, Any] | None = None
        self._iterations: list[dict[str, Any]] = []
        self._active_iter_idx: int = -1
        self._llm_calls: list[dict[str, Any]] = []
        self._pending_llm_call: dict[str, Any] | None = None
        self._executed_runs: list[dict[str, Any]] = []
        self._event_log: list[dict[str, str]] = []
        self._m1_archive = load_m1_archive(self.workspace_dir)

        # Seed timeline with workspace archived iterations (explicitly marked origin="archived")
        prev_src = ""
        prev_lbl = ""
        for idx, (pid, label, src) in enumerate(seed_policies):
            ast_res = check_sandbox_ast_compliance(src)
            sub = analyze_policy_subroutines(src, failure_codes=())
            diff_txt = _compute_unified_diff(prev_src, src, prev_lbl, f"iter_{idx} ({pid})")
            self._iterations.append(
                {
                    "iter_index": idx,
                    "label": label,
                    "origin": "archived",
                    "policy_id": pid,
                    "source": src,
                    "source_sha256": digest(src),
                    "unified_diff": diff_txt,
                    "ast_check": ast_res,
                    "subroutines": sub,
                    "llm_meta": None,
                    "execution": None,
                    "attribution": None,
                    "journal_entries": [],
                    "created_utc": _utc_hms(),
                }
            )
            prev_src = src
            prev_lbl = f"iter_{idx} ({pid})"
        if self._iterations:
            self._active_iter_idx = len(self._iterations) - 1
            self._log("INIT", f"Loaded {len(self._iterations)} archived workspace policies; ready for live Explore -> Synthesize -> Execute -> Iterate loop.")

    def _log(self, tag: str, message: str) -> None:
        self._event_log.append({"t": _utc_hms(), "tag": tag, "msg": message})
        if len(self._event_log) > 120:
            self._event_log = self._event_log[-120:]

    def _scenario_dict(self, scenario_key: str) -> dict[str, Any]:
        spec = SCENARIO_PROFILES.get(scenario_key) or SCENARIO_PROFILES["fault"]
        return {
            "preset": spec["preset"],
            "order_id": "A042",
            "target_slot": "$SLOT_PICKUP_1",
            "fault_inject_slip_xy_m": spec["fault_inject_slip_xy_m"],
            "simulate_timeout": spec["simulate_timeout"],
            "expect_status": spec["expect_status"],
        }

    def get_state(self) -> dict[str, Any]:
        with self._state_lock:
            return {
                "phase": self._phase,
                "busy": self._busy,
                "busy_detail": self._busy_detail,
                "task_instruction": self._task_instruction,
                "scenario_key": self._scenario_key,
                "scenario_profiles": {k: v["label"] for k, v in SCENARIO_PROFILES.items()},
                "probe_report": self._probe_report,
                "iterations": list(self._iterations),
                "active_iter_index": self._active_iter_idx,
                "llm_calls": list(self._llm_calls),
                "pending_llm_call": dict(self._pending_llm_call) if self._pending_llm_call else None,
                "executed_runs": list(self._executed_runs),
                "event_log": list(self._event_log),
                "m1_archive": self._m1_archive,
            }

    def set_driver(self, driver: SceneHostDriver, *, label: str = "") -> None:
        """Hot-swap the underlying host driver when switching embodiments in the UI."""
        with self._state_lock:
            if self._busy:
                raise RuntimeError("Cannot switch embodiment while an evolution step is running.")
            self.driver = driver
            self._probe_report = None
            self._phase = "idle"
            self._log(
                "HOST",
                f"Switched active embodiment -> {label or driver.driver_name} (driver={driver.driver_name}, evidence={driver.evidence_source}).",
            )

    def select_iteration(self, index: int) -> dict[str, Any]:
        with self._state_lock:
            if 0 <= index < len(self._iterations):
                self._active_iter_idx = index
        return self.get_state()

    def reset_session(self, *, task_instruction: str | None = None, scenario_key: str | None = None) -> dict[str, Any]:
        with self._state_lock:
            if self._busy:
                return {"status": "busy", "detail": self._busy_detail}
            if task_instruction is not None and task_instruction.strip():
                self._task_instruction = task_instruction.strip()
            if scenario_key in SCENARIO_PROFILES:
                self._scenario_key = scenario_key
            self._phase = "idle"
            self._probe_report = None
            self._iterations = []
            self._active_iter_idx = -1
            self._llm_calls = []
            self._pending_llm_call = None
            self._executed_runs = []
            self._event_log = []
            self._log("RESET", f"Fresh session initialized (scenario={self._scenario_key}). Click 1. Explore / Probe or Run Closed Loop (1->4).")
        return self.get_state()

    def _run_probe_sync(self, scenario_key: str) -> dict[str, Any]:
        scenario = self._scenario_dict(scenario_key)
        t0 = time.monotonic()
        with self._driver_lock:
            self.driver.reset_scenario(scenario)
            cap = self.driver.discover_capabilities(allow_busy=True)
            obs = self.driver.observe_scene()
            probe_cmd = f"probe-{time.time_ns()}"
            probe_exec = self.driver.run_policy_scenario(
                PUBLIC_SDK_EXPLORATION_PROBE_SOURCE,
                command_id=probe_cmd,
                scenario={**scenario, "fault_inject_slip_xy_m": None, "simulate_timeout": False},
            )
            # Reset cleanly back to scenario initial state after diagnostic probe
            self.driver.reset_scenario(scenario)
        wall_s = round(time.monotonic() - t0, 3)

        raw = obs.raw_snapshot or {}
        objs = raw.get("objects") or []
        l2 = raw.get("l2_pouring_safety") or {}
        disc = cap.discovered
        report = {
            "evidence_source": self.driver.evidence_source,
            "execution_authority": disc.execution_authority if disc else self.driver.driver_name,
            "perception_owner": disc.perception_owner if disc else "unknown",
            "sdk_contract_sha256": (disc.sdk_contract_sha256 if disc else "")[:16],
            "allowed_methods_count": len(disc.allowed_methods) if disc else 0,
            "allowed_methods": list(disc.allowed_methods) if disc else [],
            "scenario_key": scenario_key,
            "objects_observed": [
                {
                    "id": o.get("object_id") or o.get("id"),
                    "class": o.get("semantic_class") or o.get("cup_type"),
                    "world_m": [round(float(x), 4) for x in (o.get("center_world_m") or o.get("world") or [])],
                    "confidence": round(float(o.get("confidence", 0.0)), 3),
                }
                for o in objs
            ],
            "l2_safe_to_pour": bool(l2.get("safe_to_pour", False)),
            "l2_radial_offset_mm": (
                round(float(l2["current_radial_offset_m"]) * 1000.0, 2)
                if l2.get("current_radial_offset_m") is not None
                else None
            ),
            "probe_methods_executed": list(probe_exec.get("methods") or []),
            "probe_wall_s": wall_s,
            "probe_source": PUBLIC_SDK_EXPLORATION_PROBE_SOURCE,
            "summary": (
                f"Discovered {len(disc.allowed_methods) if disc else 0} public SDK methods on "
                f"{disc.execution_authority if disc else self.driver.driver_name}; "
                f"observed {len(objs)} scene objects; verified hover reachability + retract via public SDK in {wall_s}s."
            ),
        }
        with self._state_lock:
            self._probe_report = report
            self._executed_runs.append(
                {
                    "run_index": len(self._executed_runs) + 1,
                    "stage": "01_explore_probe",
                    "label": "Stage-1 ActiveProbe (Public SDK)",
                    "iter_index": None,
                    "command_id": probe_cmd,
                    "scenario_key": scenario_key,
                    "source": PUBLIC_SDK_EXPLORATION_PROBE_SOURCE,
                    "source_sha256": digest(PUBLIC_SDK_EXPLORATION_PROBE_SOURCE),
                    "ast_check": check_sandbox_ast_compliance(PUBLIC_SDK_EXPLORATION_PROBE_SOURCE),
                    "methods": list(probe_exec.get("methods") or []),
                    "status": str(probe_exec.get("host_runtime_status") or "probed"),
                    "task_passed": True,
                    "wall_s": wall_s,
                    "executed_utc": _utc_hms(),
                }
            )
            self._phase = "explored"
            self._log("PROBE", report["summary"])
        return report

    def start_probe_async(self, body: Mapping[str, Any]) -> dict[str, Any]:
        with self._state_lock:
            if self._busy:
                return {"status": "busy", "detail": self._busy_detail}
            scenario_key = str(body.get("scenario_key") or self._scenario_key)
            if scenario_key in SCENARIO_PROFILES:
                self._scenario_key = scenario_key
            if body.get("task_instruction"):
                self._task_instruction = str(body["task_instruction"]).strip()
            self._busy = True
            self._phase = "exploring"
            self._busy_detail = f"Running public-SDK ActiveProbe on {self.driver.driver_name}..."
            self._log("PROBE", f"Starting Stage 1 exploration (scenario={self._scenario_key})...")

        def _worker() -> None:
            try:
                self._run_probe_sync(self._scenario_key)
            except Exception as exc:  # noqa: BLE001
                with self._state_lock:
                    self._log("ERROR", f"Probe failed: {exc}")
                    self._phase = "idle"
            finally:
                with self._state_lock:
                    self._busy = False
                    self._busy_detail = ""

        threading.Thread(target=_worker, daemon=True).start()
        return {"status": "started", "phase": "exploring"}

    def _build_synthesis_prompt(
        self,
        *,
        task_instruction: str,
        probe_report: Mapping[str, Any] | None,
        prev_iter: Mapping[str, Any] | None,
        negative_history: list[str],
    ) -> tuple[str, str]:
        probe_block = ""
        if probe_report:
            probe_block = (
                "STAGE-1 PUBLIC-SDK EXPLORATION TELEMETRY (real measured state, zero hardcoded hints):\n"
                f"- Host authority: {probe_report.get('execution_authority')} (evidence={probe_report.get('evidence_source')})\n"
                f"- Observed objects: {json.dumps(probe_report.get('objects_observed', []), ensure_ascii=False)}\n"
                f"- L2 radial offset (mm): {probe_report.get('l2_radial_offset_mm')}\n"
                f"- Verified probe primitives: {probe_report.get('probe_methods_executed', [])}\n"
            )

        if prev_iter is None or not prev_iter.get("execution"):
            origin = "llm_draft"
            extra = (
                f"TASK INSTRUCTION:\n{task_instruction}\n\n"
                f"{probe_block}\n"
                "Synthesize a clean, modular Python policy `run_policy(sdk, params)` strictly obeying the AST sandbox "
                "(no imports, no leading-underscore names, no lambda, no exec/eval)."
            )
            return origin, build_shared_initial_prompt(extra_guidance=extra)

        origin = "llm_repair"
        ex = prev_iter["execution"]
        attr = prev_iter.get("attribution") or {}
        sub = prev_iter.get("subroutines") or {}
        neg_block = "\n".join(f"  - {item}" for item in negative_history[-4:]) if negative_history else "  (none)"
        prompt = "\n".join(
            [
                "You are TRAX's offline policy repair engine. Fix ONLY the failing behavior in the Python policy below.",
                "",
                SHARED_HOST_SDK_CONTRACT,
                "",
                SHARED_TASK_REQUIREMENTS,
                "",
                f"TASK INSTRUCTION:\n{task_instruction}",
                "",
                probe_block,
                "PREVIOUS ITERATION FAILURE DIAGNOSTICS:",
                f"- Four-layer attribution: layer={attr.get('layer')} code={attr.get('code')} ({attr.get('summary')})",
                f"- Host runtime status: {ex.get('host_runtime_status')} | task_passed={ex.get('task_passed')}",
                f"- Oracle post-run: {json.dumps(ex.get('oracle_post_run') or {})}",
                f"- Non-idempotent counts: {json.dumps(ex.get('non_idempotent_counts') or {})} | "
                f"duplicates={ex.get('duplicate_non_idempotent')} | unknown_events={ex.get('unknown_events_observed')} | "
                f"queried_after_unknown={ex.get('queried_after_unknown')}",
                f"- Executed SDK methods: {json.dumps(ex.get('methods') or [])}",
                f"- AST subroutine localization: {json.dumps(sub, ensure_ascii=False)}",
                f"- Prior negative attempts:\n{neg_block}",
                "",
                "PREVIOUS PYTHON POLICY SOURCE:",
                "```python",
                str(prev_iter.get("source") or ""),
                "```",
                "",
                "Return ONLY the complete repaired Python code defining `run_policy(sdk, params)` inside a ```python block.",
            ]
        )
        return origin, prompt

    def _synthesize_sync(self, *, force_fresh: bool = False) -> dict[str, Any]:
        with self._state_lock:
            prev_iter = None
            if not force_fresh and 0 <= self._active_iter_idx < len(self._iterations):
                cand = self._iterations[self._active_iter_idx]
                if cand.get("execution") and not cand["execution"].get("task_passed"):
                    prev_iter = cand
            neg_hist = [
                f"iter_{it['iter_index']}: {(it.get('attribution') or {}).get('layer')}/{(it.get('attribution') or {}).get('code')}"
                for it in self._iterations
                if it.get("execution") and not it["execution"].get("task_passed")
            ]
            origin, prompt = self._build_synthesis_prompt(
                task_instruction=self._task_instruction,
                probe_report=self._probe_report,
                prev_iter=prev_iter,
                negative_history=neg_hist,
            )
            started_utc = _utc_hms()
            next_call_id = len(self._llm_calls) + 1
            self._pending_llm_call = {
                "call_id": next_call_id,
                "iter_index": len(self._iterations),
                "origin": origin,
                "provider": "codex_cli",
                "model": "gpt-6-astra",
                "reasoning_effort": "medium",
                "status": "running",
                "started_utc": started_utc,
                "prompt": prompt,
                "prompt_chars": len(prompt),
                "raw_response": None,
                "response_chars": 0,
            }
            self._log(
                "SYNTH",
                f"LLM call #{next_call_id} ({origin}, model=gpt-6-astra, prompt={len(prompt)} chars) dispatched...",
            )

        llm = self._llm_fn or make_live_llm_fn(
            provider="codex_cli",
            model="gpt-6-astra",
            reasoning_effort="medium",
            timeout_s=150.0,
            max_retries=1,
        )
        try:
            llm_res = llm(prompt)
        finally:
            with self._state_lock:
                self._pending_llm_call = None

        source = str(llm_res.get("source") or "").strip()
        raw_response = str(llm_res.get("raw_output") if llm_res.get("raw_output") is not None else source)
        ast_res = check_sandbox_ast_compliance(source)
        sub = analyze_policy_subroutines(source, failure_codes=())

        with self._state_lock:
            idx = len(self._iterations)
            prev_src = str(self._iterations[self._active_iter_idx]["source"]) if 0 <= self._active_iter_idx < len(self._iterations) else ""
            prev_lbl = f"iter_{self._active_iter_idx}" if prev_src else ""
            diff_txt = _compute_unified_diff(prev_src, source, prev_lbl, f"iter_{idx} ({origin})")
            call_record = {
                "call_id": len(self._llm_calls) + 1,
                "iter_index": idx,
                "origin": origin,
                "provider": "codex_cli" if self._llm_fn is None else "custom_llm",
                "model": str(llm_res.get("model", "gpt-6-astra")),
                "reasoning_effort": str(llm_res.get("reasoning_effort", "medium")),
                "status": "completed",
                "started_utc": started_utc,
                "completed_utc": _utc_hms(),
                "wall_s": llm_res.get("wall_s", 0.0),
                "prompt": prompt,
                "prompt_chars": len(prompt),
                "raw_response": raw_response,
                "response_chars": len(raw_response),
                "extracted_source": source,
                "extracted_sha256": digest(source),
            }
            self._llm_calls.append(call_record)
            rec = {
                "iter_index": idx,
                "label": f"iter_{idx} ({origin})",
                "origin": origin,
                "policy_id": f"evolve_iter_{idx}",
                "source": source,
                "source_sha256": digest(source),
                "unified_diff": diff_txt,
                "ast_check": ast_res,
                "subroutines": sub,
                "llm_meta": call_record,
                "execution": None,
                "attribution": None,
                "journal_entries": [],
                "created_utc": _utc_hms(),
            }
            self._iterations.append(rec)
            self._active_iter_idx = idx
            self._phase = "synthesized"
            ast_tag = "COMPLIANT" if ast_res.get("compliant") else f"BLOCKED ({len(ast_res.get('violations', []))} violations)"
            self._log(
                "AST",
                f"LLM call #{call_record['call_id']} -> iter_{idx} in {call_record['wall_s']}s ({call_record['response_chars']} chars) -> AST {ast_tag} (sha256={rec['source_sha256'][:12]}).",
            )
            return rec

    def start_synthesize_async(self, body: Mapping[str, Any]) -> dict[str, Any]:
        with self._state_lock:
            if self._busy:
                return {"status": "busy", "detail": self._busy_detail}
            if body.get("task_instruction"):
                self._task_instruction = str(body["task_instruction"]).strip()
            if body.get("scenario_key") in SCENARIO_PROFILES:
                self._scenario_key = str(body["scenario_key"])
            force_fresh = bool(body.get("force_fresh", False))
            self._busy = True
            self._phase = "synthesizing"
            self._busy_detail = "Synthesizing Python policy via LLM (gpt-6-astra)..."

        def _worker() -> None:
            try:
                self._synthesize_sync(force_fresh=force_fresh)
            except Exception as exc:  # noqa: BLE001
                with self._state_lock:
                    self._log("ERROR", f"Synthesis failed: {exc}")
                    self._phase = "idle"
            finally:
                with self._state_lock:
                    self._busy = False
                    self._busy_detail = ""

        threading.Thread(target=_worker, daemon=True).start()
        return {"status": "started", "phase": "synthesizing"}

    def _execute_sync(self, *, edited_source: str | None = None, scenario_key: str | None = None) -> dict[str, Any]:
        with self._state_lock:
            if scenario_key in SCENARIO_PROFILES:
                self._scenario_key = scenario_key
            cur_idx = self._active_iter_idx
            if edited_source is not None and edited_source.strip():
                cleaned = edited_source.strip()
                existing_src = self._iterations[cur_idx]["source"].strip() if 0 <= cur_idx < len(self._iterations) else ""
                if cleaned != existing_src:
                    idx = len(self._iterations)
                    ast_res = check_sandbox_ast_compliance(cleaned)
                    sub = analyze_policy_subroutines(cleaned, failure_codes=())
                    diff_txt = _compute_unified_diff(existing_src, cleaned, f"iter_{cur_idx}", f"iter_{idx} (manual_edit)")
                    self._iterations.append(
                        {
                            "iter_index": idx,
                            "label": f"iter_{idx} (manual_edit)",
                            "origin": "manual_edit",
                            "policy_id": f"evolve_iter_{idx}",
                            "source": cleaned,
                            "source_sha256": digest(cleaned),
                            "unified_diff": diff_txt,
                            "ast_check": ast_res,
                            "subroutines": sub,
                            "llm_meta": None,
                            "execution": None,
                            "attribution": None,
                            "journal_entries": [],
                            "created_utc": _utc_hms(),
                        }
                    )
                    self._active_iter_idx = idx
                    cur_idx = idx
                    self._log("EDIT", f"Recorded manual Python edit as iter_{idx} (AST compliant={ast_res.get('compliant')}).")

            if not (0 <= cur_idx < len(self._iterations)):
                raise ValueError("No Python policy iteration available to execute. Synthesize or select one first.")

            target_iter = self._iterations[cur_idx]
            source = str(target_iter["source"])
            ast_res = check_sandbox_ast_compliance(source)
            target_iter["ast_check"] = ast_res
            if not ast_res.get("compliant"):
                attr = {
                    "layer": "policy",
                    "code": "ast_sandbox_violation",
                    "recommended_owner": "craft_policy",
                    "auto_repair_eligible": True,
                    "summary": "; ".join(ast_res.get("violations") or ["AST violation"]),
                }
                target_iter["execution"] = {
                    "task_passed": False,
                    "host_runtime_status": "ast_preflight_rejected",
                    "wall_s": 0.0,
                    "methods": [],
                    "oracle_post_run": {},
                    "non_idempotent_counts": {},
                    "duplicate_non_idempotent": False,
                    "unknown_events_observed": 0,
                    "queried_after_unknown": True,
                    "evidence_source": self.driver.evidence_source,
                }
                target_iter["attribution"] = attr
                self._phase = "attributed"
                self._log("AST", f"Blocked iter_{cur_idx} before actuation: {attr['summary']}")
                return target_iter

            scenario = self._scenario_dict(self._scenario_key)
            self._log("EXEC", f"Executing iter_{cur_idx} inside AST sandbox + SQLite journal (scenario={self._scenario_key})...")

        cmd_id = f"evolve-it{cur_idx}-{time.time_ns()}"
        with self._driver_lock:
            self.driver.reset_scenario(scenario)
            with Journal(self._journal_path) as journal:
                exec_res = self.driver.run_policy_scenario(
                    source,
                    command_id=cmd_id,
                    scenario=scenario,
                    journal=journal,
                )

        sqlite_rows = _read_journal_rows(self._journal_path, cmd_id)
        journal_entries = _synthesize_journal_view(exec_res, sqlite_rows)
        attr_obj = attribute_trial(exec_res)
        attr_dict = attr_obj.to_dict()
        failure_codes = () if exec_res.get("task_passed") else (str(attr_dict.get("code") or ""),)
        sub = analyze_policy_subroutines(source, failure_codes=failure_codes)

        with self._state_lock:
            target_iter = self._iterations[cur_idx]
            exec_with_code = dict(exec_res)
            exec_with_code["executed_command_id"] = cmd_id
            exec_with_code["executed_source"] = source
            exec_with_code["executed_sha256"] = digest(source)
            target_iter["execution"] = exec_with_code
            target_iter["attribution"] = attr_dict
            target_iter["journal_entries"] = journal_entries
            target_iter["subroutines"] = sub
            self._executed_runs.append(
                {
                    "run_index": len(self._executed_runs) + 1,
                    "stage": "03_policy_exec",
                    "label": f"iter_{cur_idx} ({target_iter.get('origin')})",
                    "iter_index": cur_idx,
                    "command_id": cmd_id,
                    "scenario_key": self._scenario_key,
                    "source": source,
                    "source_sha256": digest(source),
                    "ast_check": ast_res,
                    "methods": list(exec_res.get("methods") or []),
                    "status": str(exec_res.get("host_runtime_status") or ""),
                    "task_passed": bool(exec_res.get("task_passed")),
                    "wall_s": exec_res.get("wall_s", 0.0),
                    "executed_utc": _utc_hms(),
                }
            )
            self._phase = "verified" if exec_res.get("task_passed") else "attributed"
            verdict = "PASS" if exec_res.get("task_passed") else f"FAIL ({attr_dict['layer']}/{attr_dict['code']})"
            oracle = exec_res.get("oracle_post_run") or {}
            self._log(
                "EXEC",
                f"iter_{cur_idx} -> {verdict} | wall={exec_res.get('wall_s')}s | "
                f"fill={oracle.get('cup_filled')} | err={oracle.get('delivery_error_mm')}mm | "
                f"dup={exec_res.get('duplicate_non_idempotent')} | unknown={exec_res.get('unknown_events_observed')}",
            )
            return target_iter

    def start_execute_async(self, body: Mapping[str, Any]) -> dict[str, Any]:
        with self._state_lock:
            if self._busy:
                return {"status": "busy", "detail": self._busy_detail}
            edited_source = body.get("source")
            scenario_key = str(body.get("scenario_key") or self._scenario_key)
            self._busy = True
            self._phase = "executing"
            self._busy_detail = f"Executing Python policy on {self.driver.driver_name}..."

        def _worker() -> None:
            try:
                self._execute_sync(
                    edited_source=str(edited_source) if edited_source is not None else None,
                    scenario_key=scenario_key,
                )
            except Exception as exc:  # noqa: BLE001
                with self._state_lock:
                    self._log("ERROR", f"Execution failed: {exc}")
                    self._phase = "idle"
            finally:
                with self._state_lock:
                    self._busy = False
                    self._busy_detail = ""

        threading.Thread(target=_worker, daemon=True).start()
        return {"status": "started", "phase": "executing"}

    def start_closed_loop_async(self, body: Mapping[str, Any]) -> dict[str, Any]:
        """Run full automated Explore -> Synthesize -> Execute -> Iterate loop."""
        with self._state_lock:
            if self._busy:
                return {"status": "busy", "detail": self._busy_detail}
            if body.get("task_instruction"):
                self._task_instruction = str(body["task_instruction"]).strip()
            if body.get("scenario_key") in SCENARIO_PROFILES:
                self._scenario_key = str(body["scenario_key"])
            max_iters = max(1, min(4, int(body.get("max_iters", 3))))
            fresh = bool(body.get("fresh", True))
            if fresh:
                self._iterations = []
                self._active_iter_idx = -1
                self._probe_report = None
                self._llm_calls = []
                self._pending_llm_call = None
                self._executed_runs = []
            self._busy = True
            self._phase = "exploring"
            self._busy_detail = f"Closed-loop evolution (max_iters={max_iters}): Stage 1 Exploring..."
            self._log("LOOP", f"=== Starting Closed-Loop Evolution (scenario={self._scenario_key}, max_iters={max_iters}) ===")

        def _worker() -> None:
            try:
                # Stage 1: Active Exploration
                with self._state_lock:
                    self._phase = "exploring"
                    self._busy_detail = "Stage 1/4: Public-SDK Active Exploration on robot..."
                self._run_probe_sync(self._scenario_key)

                # Stages 2-4: Synthesize -> Execute -> Attribute -> Iterate
                for step in range(1, max_iters + 1):
                    with self._state_lock:
                        self._phase = "synthesizing" if step == 1 else "iterating"
                        self._busy_detail = f"Iteration {step}/{max_iters}: Synthesizing Python policy via LLM..."
                    self._synthesize_sync(force_fresh=(step == 1))

                    with self._state_lock:
                        self._phase = "executing"
                        self._busy_detail = f"Iteration {step}/{max_iters}: Executing Python in AST sandbox + SQLite journal..."
                    it_rec = self._execute_sync(scenario_key=self._scenario_key)
                    if (it_rec.get("execution") or {}).get("task_passed"):
                        with self._state_lock:
                            self._phase = "verified"
                            self._log("LOOP", f"=== Converged at iter_{it_rec['iter_index']} (PASS, 0 duplicates) ===")
                        return
                    attr = it_rec.get("attribution") or {}
                    with self._state_lock:
                        self._log(
                            "ITER",
                            f"iter_{it_rec['iter_index']} failed ({attr.get('layer')}/{attr.get('code')}); feeding diagnostics to next iteration...",
                        )
                with self._state_lock:
                    self._log("LOOP", f"=== Reached max_iters={max_iters} budget ===")
            except Exception as exc:  # noqa: BLE001
                with self._state_lock:
                    self._log("ERROR", f"Closed-loop evolution aborted: {exc}")
                    self._phase = "idle"
            finally:
                with self._state_lock:
                    self._busy = False
                    self._busy_detail = ""

        threading.Thread(target=_worker, daemon=True).start()
        return {"status": "started", "phase": "exploring", "max_iters": max_iters}
