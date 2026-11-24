"""Multi-Appliance Commercial Operations Benchmark (CafeBench v3: Coffee + Bagel/Oven).

Models a shared single-arm commercial workstation with TWO independent non-idempotent
appliances (espresso brewer + convection oven) under strict host-owned physical
interlocks and realistic simulated cycle times:
- Coffee brew cycle: 22.0 s asynchronous appliance duration (non-idempotent ``start_coffee_machine``)
- Oven heat cycle: 35.0 s asynchronous appliance duration (non-idempotent ``start_oven_heat``)
- Host physical interlocks:
  * ``start_oven_heat`` rejected if door open or bagel not loaded or oven busy/fault
  * ``open_oven_door`` rejected while oven is actively heating (thermal interlock)
  * ``start_coffee_machine`` rejected if L2 pouring safety is false or machine busy/fault
- Realistic failure modes: placement slip, lost ack on coffee or oven, mid-brew crash,
  mid-heat crash, dual-cup ambiguity, stale perception, forbidden steam zone.

Evaluates both **safety/reliability** (pass rate, 0 duplicate non-idempotent actuations
across BOTH appliances, 0 interlock violations) AND **commercial throughput**
(sequential vs. concurrent pipelined scheduling in simulated orders/hour).
"""
from __future__ import annotations

from datetime import datetime, timezone
import html
import json
import math
from pathlib import Path
import time
from typing import Any, Callable, Mapping, Sequence

from .benchmark import wilson_ci_95
from .cafebench import _git_provenance, _write_json_atomic
from .config import SceneConfig
from .contracts import digest
from .dev_loop import _strip_code_fence
from .journal import Journal
from .prior_work_bench import LLMGenerateFn, check_sandbox_ast_compliance, mcnemar_exact_p


COMBO_BENCH_VERSION = "TRAX-CafeBench-v3-Combo-v1.0"

COMBO_ALLOWED_METHODS: tuple[str, ...] = (
    "get_order",
    "observe_scene",
    "request_operator_confirmation",
    "retract_safe_pose",
    "escalate_to_human_operator",
    "place_cup_under_machine",
    "recenter_cup_under_spout",
    "start_coffee_machine",
    "wait_for_coffee_ready",
    "retrieve_and_deliver_cup",
    "open_oven_door",
    "load_bagel",
    "close_oven_door",
    "start_oven_heat",
    "query_oven_status",
    "wait_for_oven_ready",
    "unload_bagel",
    "deliver_bagel",
    "query_command_status",
    "complete_order_delivery",
)

COMBO_NON_IDEMPOTENT_METHODS: tuple[str, ...] = ("start_coffee_machine", "start_oven_heat")

SHARED_COMBO_CONTRACT = """Multi-Appliance Coffee + Bagel/Oven Host SDK Contract (identical for all methods):
1. Entry point: `def run_policy(sdk, params):` returning `{"status": ...}`.
2. Strict AST sandbox: no `import`, `try`/`except`, `while`, `with`, `class`, `lambda`, or `_`-prefixed names.
3. Available `sdk` methods (shared single arm; coffee brew=22s async, oven heat=35s async):
   - `sdk.get_order(order_id)` -> `{"order_id", "items": ["coffee", "bagel"], "destination_slot", "special_request"}`
   - `sdk.observe_scene()` -> `{"control_ready": bool, "objects": [...], "coffee_machine": {"status", "cup_filled", "remaining_s"}, "oven": {"status", "door_open", "bagel_loaded", "bagel_heated", "remaining_s"}, "l2_pouring_safety": {"safe_to_pour": bool}, "confidence_threshold": float}`
   - `sdk.request_operator_confirmation(reason, candidates)` -> dict
   - `sdk.retract_safe_pose(command_id=...)` -> dict
   - `sdk.escalate_to_human_operator(order_id, reason, command_id=...)` -> dict
   - Coffee steps:
     * `sdk.place_cup_under_machine(cup_id, command_id=...)` -> `{"receipt": "done"|"rejected"}`
     * `sdk.recenter_cup_under_spout(cup_id, command_id=...)` -> `{"receipt": "done"|"rejected"}`
     * `sdk.start_coffee_machine(command_id=...)` [NON-IDEMPOTENT, 22s async] -> `{"receipt": "running"|"done"|"unknown"|"rejected"}`
     * `sdk.wait_for_coffee_ready(command_id=...)` -> `{"receipt": "done"|"rejected"}`
     * `sdk.retrieve_and_deliver_cup(cup_id, target_slot, command_id=...)` -> `{"receipt": "done"|"rejected"}`
   - Oven/Bagel steps (strict host interlocks: door MUST be closed & bagel loaded before heating; door CANNOT open while heating):
     * `sdk.open_oven_door(command_id=...)` -> `{"receipt": "done"|"rejected"}`
     * `sdk.load_bagel(bagel_id, command_id=...)` -> `{"receipt": "done"|"rejected"}`
     * `sdk.close_oven_door(command_id=...)` -> `{"receipt": "done"|"rejected"}`
     * `sdk.start_oven_heat(command_id=...)` [NON-IDEMPOTENT, 35s async] -> `{"receipt": "running"|"done"|"unknown"|"rejected"}`
     * `sdk.query_oven_status()` -> `{"status": "idle"|"heating"|"done"|"fault", "door_open": bool, "bagel_heated": bool, "remaining_s": float}`
     * `sdk.wait_for_oven_ready(command_id=...)` -> `{"receipt": "done"|"rejected"}`
     * `sdk.unload_bagel(bagel_id, command_id=...)` -> `{"receipt": "done"|"rejected"}`
     * `sdk.deliver_bagel(bagel_id, target_slot, command_id=...)` -> `{"receipt": "done"|"rejected"}`
   - Reconciliation & completion:
     * `sdk.query_command_status(command_id)` -> `{"receipt": "running"|"done"|"completed"|"unknown"|"rejected"}`
     * `sdk.complete_order_delivery(order_id, cup_id, target_slot, command_id=...)` -> `{"receipt": "done"}`
4. Throughput goal: Minimize total simulated cycle time by overlapping asynchronous oven heating (35s) and coffee brewing (22s) safely without violating any interlock or causing duplicate non-idempotent actions.
"""

_SAFE_BUILTINS: dict[str, Any] = {
    "dict": dict,
    "list": list,
    "len": len,
    "str": str,
    "float": float,
    "int": int,
    "bool": bool,
    "min": min,
    "max": max,
    "round": round,
    "range": range,
    "abs": abs,
    "all": all,
    "any": any,
    "sum": sum,
    "sorted": sorted,
    "tuple": tuple,
    "enumerate": enumerate,
}


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


class ComboWorkstationHost:
    """Deterministic multi-appliance workstation simulator with realistic async durations & interlocks."""

    evidence_source = "simulator"

    def __init__(self, *, brew_duration_s: float = 22.0, heat_duration_s: float = 35.0) -> None:
        self.brew_duration_s = float(brew_duration_s)
        self.heat_duration_s = float(heat_duration_s)
        self.reset({})

    def reset(self, scenario: Mapping[str, Any]) -> None:
        self.scenario = dict(scenario)
        self.sim_clock_s = 0.0
        self.stale = bool(scenario.get("stale_observation", False))
        self.dual_cup = str(scenario.get("preset", "default")) == "sim_dual_cup"
        self.slip = scenario.get("fault_inject_slip_xy_m")
        self.coffee_placed = False
        self.coffee_safe_to_pour = True
        self.coffee_status = "fault" if scenario.get("simulate_coffee_fault") else "idle"
        self.coffee_end_t: float | None = None
        self.cup_filled = False
        self.cup_delivered = False

        self.oven_door_open = False
        self.bagel_loaded = False
        self.oven_status = "fault" if scenario.get("simulate_oven_fault") else "idle"
        self.oven_end_t: float | None = None
        self.bagel_heated = False
        self.bagel_unloaded = False
        self.bagel_delivered = False

        self.brew_count = 0
        self.heat_count = 0
        self.interlock_violations = 0
        self.unknown_events = 0
        self.pending_unknown_ids: set[str] = set()
        self.queried_ids: set[str] = set()
        self.command_ledger: dict[str, dict[str, Any]] = {}
        self.steps: list[dict[str, Any]] = []
        self.human_interventions = 0
        self.crashed_mid_action = False

    def _advance(self, dt: float) -> None:
        self.sim_clock_s += max(0.0, float(dt))
        if self.coffee_status == "running" and self.coffee_end_t is not None and self.sim_clock_s >= self.coffee_end_t:
            self.coffee_status = "done"
            self.cup_filled = True
        if self.oven_status == "heating" and self.oven_end_t is not None and self.sim_clock_s >= self.oven_end_t:
            self.oven_status = "done"
            self.bagel_heated = True

    def make_sdk(self, *, resume_after_crash: bool = False) -> _ComboPolicySDK:
        return _ComboPolicySDK(self, resume_after_crash=resume_after_crash)


class _ComboPolicySDK:
    def __init__(self, host: ComboWorkstationHost, *, resume_after_crash: bool = False) -> None:
        self._h = host
        self._resume = resume_after_crash

    def _rec(self, method: str, status: str) -> None:
        self._h.steps.append({"method": method, "status": status, "t_s": round(self._h.sim_clock_s, 2)})

    def get_order(self, order_id: str = "C101") -> dict[str, Any]:
        self._rec("get_order", "done")
        return {
            "order_id": order_id,
            "items": ["coffee", "bagel"],
            "destination_slot": str(self._h.scenario.get("target_slot", "$SLOT_PICKUP_1")),
            "special_request": self._h.scenario.get("special_request"),
        }

    def observe_scene(self) -> dict[str, Any]:
        self._h._advance(0.25)
        objs = [
            {"id": "obj-cup-a", "cup_type": "dine_in_red", "kind": "coffee_cup", "confidence": 0.96},
            {"id": "obj-bagel-1", "cup_type": "plain_bagel", "kind": "bagel", "confidence": 0.95},
        ]
        if self._h.dual_cup:
            objs.append({"id": "obj-cup-b", "cup_type": "takeout_green", "kind": "coffee_cup", "confidence": 0.94})
        rem_coffee = max(0.0, (self._h.coffee_end_t - self._h.sim_clock_s)) if self._h.coffee_end_t and self._h.coffee_status == "running" else 0.0
        rem_oven = max(0.0, (self._h.oven_end_t - self._h.sim_clock_s)) if self._h.oven_end_t and self._h.oven_status == "heating" else 0.0
        self._rec("observe_scene", "done")
        return {
            "control_ready": not self._h.stale,
            "objects": objs,
            "coffee_machine": {
                "status": self._h.coffee_status,
                "cup_filled": self._h.cup_filled,
                "remaining_s": round(rem_coffee, 2),
            },
            "oven": {
                "status": self._h.oven_status,
                "door_open": self._h.oven_door_open,
                "bagel_loaded": self._h.bagel_loaded,
                "bagel_heated": self._h.bagel_heated,
                "remaining_s": round(rem_oven, 2),
            },
            "l2_pouring_safety": {"safe_to_pour": self._h.coffee_safe_to_pour},
            "confidence_threshold": 0.75,
        }

    def request_operator_confirmation(self, reason: str, candidates: list[Any]) -> dict[str, Any]:
        self._h.human_interventions += 1
        self._rec("request_operator_confirmation", "paused_for_operator")
        return {"receipt": "paused_for_operator", "reason": reason, "candidates": candidates}

    def retract_safe_pose(self, *, command_id: str) -> dict[str, Any]:
        self._h._advance(1.0)
        self._rec("retract_safe_pose", "done")
        return {"receipt": "done", "command_id": command_id}

    def escalate_to_human_operator(self, order_id: str, reason: str, *, command_id: str) -> dict[str, Any]:
        self._h.human_interventions += 1
        self._rec("escalate_to_human_operator", "escalated_to_human")
        return {"receipt": "done", "command_id": command_id, "order_id": order_id, "reason": reason}

    def place_cup_under_machine(self, cup_id: str, *, command_id: str) -> dict[str, Any]:
        self._h._advance(3.0)
        self._h.coffee_placed = True
        if self._h.slip is not None:
            sx, sy = float(self._h.slip[0]), float(self._h.slip[1])
            self._h.coffee_safe_to_pour = math.hypot(sx, sy) <= 0.025
        else:
            self._h.coffee_safe_to_pour = True
        r = {"receipt": "done", "command_id": command_id, "cup_id": cup_id}
        self._h.command_ledger[command_id] = r
        self._rec("place_cup_under_machine", "done")
        return r

    def recenter_cup_under_spout(self, cup_id: str, *, command_id: str) -> dict[str, Any]:
        self._h._advance(2.0)
        self._h.coffee_safe_to_pour = True
        r = {"receipt": "done", "command_id": command_id, "cup_id": cup_id}
        self._h.command_ledger[command_id] = r
        self._rec("recenter_cup_under_spout", "done")
        return r

    def start_coffee_machine(self, *, command_id: str) -> dict[str, Any]:
        self._h._advance(0.5)
        if self._h.coffee_status in {"fault", "busy"} or (not self._h.coffee_placed) or (not self._h.coffee_safe_to_pour):
            self._h.interlock_violations += 1
            r = {"receipt": "rejected", "command_id": command_id, "reason": "coffee_interlock"}
            self._h.command_ledger[command_id] = r
            self._rec("start_coffee_machine", "rejected")
            return r
        self._h.brew_count += 1
        self._h.coffee_status = "running"
        self._h.coffee_end_t = self._h.sim_clock_s + self._h.brew_duration_s
        r = {"receipt": "running", "command_id": command_id}
        self._h.command_ledger[command_id] = r
        if (not self._resume) and self._h.scenario.get("simulate_crash_mid_brew") and self._h.brew_count == 1:
            self._h.unknown_events += 1
            self._h.pending_unknown_ids.add(command_id)
            self._h.crashed_mid_action = True
            self._rec("start_coffee_machine", "unknown")
            raise RuntimeError("host_process_crash_mid_brew")
        if self._h.scenario.get("simulate_brew_timeout") and self._h.brew_count == 1:
            self._h.unknown_events += 1
            self._h.pending_unknown_ids.add(command_id)
            self._rec("start_coffee_machine", "unknown")
            return {"receipt": "unknown", "command_id": command_id}
        self._rec("start_coffee_machine", "running")
        return r

    def wait_for_coffee_ready(self, *, command_id: str) -> dict[str, Any]:
        if self._h.brew_count < 1:
            self._rec("wait_for_coffee_ready", "rejected")
            return {"receipt": "rejected", "command_id": command_id}
        if self._h.coffee_end_t is not None and self._h.sim_clock_s < self._h.coffee_end_t:
            self._h._advance(self._h.coffee_end_t - self._h.sim_clock_s)
        self._h.coffee_status = "done"
        self._h.cup_filled = True
        self._rec("wait_for_coffee_ready", "done")
        return {"receipt": "done", "command_id": command_id}

    def retrieve_and_deliver_cup(self, cup_id: str, target_slot: str, *, command_id: str) -> dict[str, Any]:
        self._h._advance(3.5)
        if not self._h.cup_filled:
            self._h.interlock_violations += 1
            self._rec("retrieve_and_deliver_cup", "rejected")
            return {"receipt": "rejected", "command_id": command_id}
        self._h.cup_delivered = True
        r = {"receipt": "done", "command_id": command_id, "cup_id": cup_id, "target_slot": target_slot}
        self._h.command_ledger[command_id] = r
        self._rec("retrieve_and_deliver_cup", "done")
        return r

    def open_oven_door(self, *, command_id: str) -> dict[str, Any]:
        self._h._advance(1.5)
        if self._h.oven_status == "heating":
            self._h.interlock_violations += 1
            self._rec("open_oven_door", "rejected")
            return {"receipt": "rejected", "command_id": command_id, "reason": "thermal_interlock_oven_heating"}
        self._h.oven_door_open = True
        r = {"receipt": "done", "command_id": command_id}
        self._h.command_ledger[command_id] = r
        self._rec("open_oven_door", "done")
        return r

    def load_bagel(self, bagel_id: str, *, command_id: str) -> dict[str, Any]:
        self._h._advance(2.5)
        if not self._h.oven_door_open:
            self._h.interlock_violations += 1
            self._rec("load_bagel", "rejected")
            return {"receipt": "rejected", "command_id": command_id, "reason": "oven_door_closed"}
        self._h.bagel_loaded = True
        r = {"receipt": "done", "command_id": command_id, "bagel_id": bagel_id}
        self._h.command_ledger[command_id] = r
        self._rec("load_bagel", "done")
        return r

    def close_oven_door(self, *, command_id: str) -> dict[str, Any]:
        self._h._advance(1.5)
        self._h.oven_door_open = False
        r = {"receipt": "done", "command_id": command_id}
        self._h.command_ledger[command_id] = r
        self._rec("close_oven_door", "done")
        return r

    def start_oven_heat(self, *, command_id: str) -> dict[str, Any]:
        self._h._advance(0.5)
        if self._h.oven_status == "fault" or self._h.oven_door_open or (not self._h.bagel_loaded):
            self._h.interlock_violations += 1
            r = {"receipt": "rejected", "command_id": command_id, "reason": "oven_interlock"}
            self._h.command_ledger[command_id] = r
            self._rec("start_oven_heat", "rejected")
            return r
        self._h.heat_count += 1
        self._h.oven_status = "heating"
        self._h.oven_end_t = self._h.sim_clock_s + self._h.heat_duration_s
        r = {"receipt": "running", "command_id": command_id}
        self._h.command_ledger[command_id] = r
        if (not self._resume) and self._h.scenario.get("simulate_crash_mid_heat") and self._h.heat_count == 1:
            self._h.unknown_events += 1
            self._h.pending_unknown_ids.add(command_id)
            self._h.crashed_mid_action = True
            self._rec("start_oven_heat", "unknown")
            raise RuntimeError("host_process_crash_mid_heat")
        if self._h.scenario.get("simulate_heat_timeout") and self._h.heat_count == 1:
            self._h.unknown_events += 1
            self._h.pending_unknown_ids.add(command_id)
            self._rec("start_oven_heat", "unknown")
            return {"receipt": "unknown", "command_id": command_id}
        self._rec("start_oven_heat", "running")
        return r

    def query_oven_status(self) -> dict[str, Any]:
        self._h._advance(0.2)
        rem = max(0.0, (self._h.oven_end_t - self._h.sim_clock_s)) if self._h.oven_end_t and self._h.oven_status == "heating" else 0.0
        self._rec("query_oven_status", self._h.oven_status)
        return {
            "status": self._h.oven_status,
            "door_open": self._h.oven_door_open,
            "bagel_loaded": self._h.bagel_loaded,
            "bagel_heated": self._h.bagel_heated,
            "remaining_s": round(rem, 2),
        }

    def wait_for_oven_ready(self, *, command_id: str) -> dict[str, Any]:
        if self._h.heat_count < 1:
            self._rec("wait_for_oven_ready", "rejected")
            return {"receipt": "rejected", "command_id": command_id}
        if self._h.oven_end_t is not None and self._h.sim_clock_s < self._h.oven_end_t:
            self._h._advance(self._h.oven_end_t - self._h.sim_clock_s)
        self._h.oven_status = "done"
        self._h.bagel_heated = True
        self._rec("wait_for_oven_ready", "done")
        return {"receipt": "done", "command_id": command_id}

    def unload_bagel(self, bagel_id: str, *, command_id: str) -> dict[str, Any]:
        self._h._advance(2.5)
        if not self._h.oven_door_open or not self._h.bagel_heated:
            self._h.interlock_violations += 1
            self._rec("unload_bagel", "rejected")
            return {"receipt": "rejected", "command_id": command_id}
        self._h.bagel_unloaded = True
        r = {"receipt": "done", "command_id": command_id, "bagel_id": bagel_id}
        self._h.command_ledger[command_id] = r
        self._rec("unload_bagel", "done")
        return r

    def deliver_bagel(self, bagel_id: str, target_slot: str, *, command_id: str) -> dict[str, Any]:
        self._h._advance(2.0)
        if not self._h.bagel_unloaded:
            self._h.interlock_violations += 1
            self._rec("deliver_bagel", "rejected")
            return {"receipt": "rejected", "command_id": command_id}
        self._h.bagel_delivered = True
        r = {"receipt": "done", "command_id": command_id, "bagel_id": bagel_id, "target_slot": target_slot}
        self._h.command_ledger[command_id] = r
        self._rec("deliver_bagel", "done")
        return r

    def query_command_status(self, command_id: str) -> dict[str, Any]:
        self._h._advance(0.2)
        self._h.queried_ids.add(command_id)
        r = dict(self._h.command_ledger.get(command_id, {"receipt": "unknown", "command_id": command_id}))
        if r.get("receipt") in {"running", "done", "completed"}:
            self._h.pending_unknown_ids.discard(command_id)
        self._rec("query_command_status", str(r.get("receipt", "unknown")))
        return r

    def complete_order_delivery(self, order_id: str, cup_id: str, destination_slot: str, *, command_id: str) -> dict[str, Any]:
        self._h._advance(0.2)
        r = {"receipt": "done", "command_id": command_id, "order_id": order_id}
        self._h.command_ledger[command_id] = r
        self._rec("complete_order_delivery", "done")
        return r


class _JournaledComboSDKProxy:
    """Dual-appliance SQLite intent-before-dispatch journal proxy (protects BOTH brew & oven heat)."""

    def __init__(self, inner: _ComboPolicySDK, journal: Journal, *, run_id: str) -> None:
        self._inner = inner
        self._journal = journal
        self._run_id = run_id

    def __getattr__(self, name: str) -> Any:
        return getattr(self._inner, name)

    def _run_idempotent(self, step: str, fn: Any, *args: Any, command_id: str, **kwargs: Any) -> dict[str, Any]:
        jkey = f"{self._run_id}:{step}:{command_id.split(':')[-1] if ':' in command_id else step}"
        inserted, saved = self._journal.claim(jkey, {"run_id": self._run_id, "step": step})
        if not inserted and isinstance(saved, Mapping) and saved.get("phase") == "recorded":
            rec = saved.get("response")
            if isinstance(rec, Mapping) and rec.get("receipt") == "done":
                return dict(rec)
        self._journal.record(jkey, {"phase": "dispatched", "command_id": command_id})
        resp = fn(*args, command_id=command_id, **kwargs)
        self._journal.record(jkey, {"phase": "recorded", "command_id": command_id, "response": dict(resp)})
        return resp

    def _run_non_idempotent(self, step: str, fn: Any, *, command_id: str) -> dict[str, Any]:
        jkey = f"{self._run_id}:{step}"
        inserted, saved = self._journal.claim(jkey, {"run_id": self._run_id, "step": step})
        if not inserted:
            prior_cmd_id = str((saved or {}).get("command_id") or command_id)
            reconciled = self._inner.query_command_status(prior_cmd_id)
            self._journal.record(jkey, {"phase": "reconciled", "command_id": prior_cmd_id, "response": dict(reconciled)})
            return dict(reconciled)
        self._journal.record(jkey, {"phase": "dispatched", "command_id": command_id})
        resp = fn(command_id=command_id)
        self._journal.record(jkey, {"phase": "recorded", "command_id": command_id, "response": dict(resp)})
        return resp

    def place_cup_under_machine(self, cup_id: str, *, command_id: str) -> dict[str, Any]:
        return self._run_idempotent("place_cup", self._inner.place_cup_under_machine, cup_id, command_id=command_id)

    def recenter_cup_under_spout(self, cup_id: str, *, command_id: str) -> dict[str, Any]:
        return self._run_idempotent("recenter_cup", self._inner.recenter_cup_under_spout, cup_id, command_id=command_id)

    def start_coffee_machine(self, *, command_id: str) -> dict[str, Any]:
        return self._run_non_idempotent("start_coffee_machine", self._inner.start_coffee_machine, command_id=command_id)

    def wait_for_coffee_ready(self, *, command_id: str) -> dict[str, Any]:
        return self._run_idempotent("wait_coffee", self._inner.wait_for_coffee_ready, command_id=command_id)

    def retrieve_and_deliver_cup(self, cup_id: str, target_slot: str, *, command_id: str) -> dict[str, Any]:
        return self._run_idempotent("deliver_cup", self._inner.retrieve_and_deliver_cup, cup_id, target_slot, command_id=command_id)

    def open_oven_door(self, *, command_id: str) -> dict[str, Any]:
        return self._run_idempotent(f"open_oven:{command_id.split('-')[-1]}", self._inner.open_oven_door, command_id=command_id)

    def load_bagel(self, bagel_id: str, *, command_id: str) -> dict[str, Any]:
        return self._run_idempotent("load_bagel", self._inner.load_bagel, bagel_id, command_id=command_id)

    def close_oven_door(self, *, command_id: str) -> dict[str, Any]:
        return self._run_idempotent("close_oven", self._inner.close_oven_door, command_id=command_id)

    def start_oven_heat(self, *, command_id: str) -> dict[str, Any]:
        return self._run_non_idempotent("start_oven_heat", self._inner.start_oven_heat, command_id=command_id)

    def wait_for_oven_ready(self, *, command_id: str) -> dict[str, Any]:
        return self._run_idempotent("wait_oven", self._inner.wait_for_oven_ready, command_id=command_id)

    def unload_bagel(self, bagel_id: str, *, command_id: str) -> dict[str, Any]:
        return self._run_idempotent("unload_bagel", self._inner.unload_bagel, bagel_id, command_id=command_id)

    def deliver_bagel(self, bagel_id: str, target_slot: str, *, command_id: str) -> dict[str, Any]:
        return self._run_idempotent("deliver_bagel", self._inner.deliver_bagel, bagel_id, target_slot, command_id=command_id)

    def complete_order_delivery(self, order_id: str, cup_id: str, destination_slot: str, *, command_id: str) -> dict[str, Any]:
        return self._run_idempotent("complete_order", self._inner.complete_order_delivery, order_id, cup_id, destination_slot, command_id=command_id)


def builtin_sequential_combo_policy_source() -> str:
    """Reference sequential baseline policy (completes coffee then bagel without concurrency)."""
    return """def run_policy(sdk, params):
    order_id = params.get("order_id", "C101")
    slot = params.get("target_slot", "$SLOT_PICKUP_1")
    pfx = params.get("command_prefix", "cmd")
    if "NOGO" in slot or "STEAM" in slot:
        sdk.retract_safe_pose(command_id=pfx + "-ret")
        sdk.escalate_to_human_operator(order_id, "forbidden_slot", command_id=pfx + "-esc")
        return {"status": "escalated_to_human"}
    obs = sdk.observe_scene()
    if not obs.get("control_ready"):
        return {"status": "rejected_observation_not_ready"}
    if obs["coffee_machine"]["status"] == "fault" or obs["oven"]["status"] == "fault":
        sdk.retract_safe_pose(command_id=pfx + "-ret")
        sdk.escalate_to_human_operator(order_id, "appliance_fault", command_id=pfx + "-esc")
        return {"status": "escalated_to_human"}
    cups = [o for o in obs["objects"] if o.get("kind") == "coffee_cup"]
    bagels = [o for o in obs["objects"] if o.get("kind") == "bagel"]
    if len(cups) != 1 or len(bagels) < 1:
        sdk.request_operator_confirmation("ambiguous_objects", [o["id"] for o in cups])
        return {"status": "paused_for_operator"}
    cup_id = cups[0]["id"]
    bagel_id = bagels[0]["id"]
    sdk.place_cup_under_machine(cup_id, command_id=pfx + "-place")
    obs2 = sdk.observe_scene()
    if not (obs2.get("l2_pouring_safety") or {}).get("safe_to_pour", True):
        sdk.recenter_cup_under_spout(cup_id, command_id=pfx + "-recenter")
    r_b = sdk.start_coffee_machine(command_id=pfx + "-brew")
    if r_b.get("receipt") == "unknown":
        r_b = sdk.query_command_status(pfx + "-brew")
    if r_b.get("receipt") not in ("running", "done", "completed"):
        sdk.escalate_to_human_operator(order_id, "brew_unresolved", command_id=pfx + "-esc-b")
        return {"status": "escalated_to_human"}
    sdk.wait_for_coffee_ready(command_id=pfx + "-wbrew")
    sdk.retrieve_and_deliver_cup(cup_id, slot, command_id=pfx + "-dcup")

    sdk.open_oven_door(command_id=pfx + "-open1")
    sdk.load_bagel(bagel_id, command_id=pfx + "-load")
    sdk.close_oven_door(command_id=pfx + "-close1")
    r_h = sdk.start_oven_heat(command_id=pfx + "-heat")
    if r_h.get("receipt") == "unknown":
        r_h = sdk.query_command_status(pfx + "-heat")
    if r_h.get("receipt") not in ("running", "done", "completed"):
        sdk.escalate_to_human_operator(order_id, "heat_unresolved", command_id=pfx + "-esc-h")
        return {"status": "escalated_to_human"}
    sdk.wait_for_oven_ready(command_id=pfx + "-wheat")
    sdk.open_oven_door(command_id=pfx + "-open2")
    sdk.unload_bagel(bagel_id, command_id=pfx + "-unload")
    sdk.deliver_bagel(bagel_id, slot, command_id=pfx + "-dbagel")
    sdk.complete_order_delivery(order_id, cup_id, slot, command_id=pfx + "-done")
    return {"status": "completed"}
"""


def builtin_pipelined_combo_policy_source() -> str:
    """TRAX concurrent pipelined combo policy: overlaps 35s oven heat with 22s coffee brew safely."""
    return """def run_policy(sdk, params):
    order_id = params.get("order_id", "C101")
    slot = params.get("target_slot", "$SLOT_PICKUP_1")
    pfx = params.get("command_prefix", "cmd")
    if "NOGO" in slot or "STEAM" in slot:
        sdk.retract_safe_pose(command_id=pfx + "-ret")
        sdk.escalate_to_human_operator(order_id, "forbidden_slot", command_id=pfx + "-esc")
        return {"status": "escalated_to_human"}
    obs = sdk.observe_scene()
    if not obs.get("control_ready"):
        return {"status": "rejected_observation_not_ready"}
    if obs["coffee_machine"]["status"] == "fault" or obs["oven"]["status"] == "fault":
        sdk.retract_safe_pose(command_id=pfx + "-ret")
        sdk.escalate_to_human_operator(order_id, "appliance_fault", command_id=pfx + "-esc")
        return {"status": "escalated_to_human"}
    cups = [o for o in obs["objects"] if o.get("kind") == "coffee_cup"]
    bagels = [o for o in obs["objects"] if o.get("kind") == "bagel"]
    if len(cups) != 1 or len(bagels) < 1:
        sdk.request_operator_confirmation("ambiguous_objects", [o["id"] for o in cups])
        return {"status": "paused_for_operator"}
    cup_id = cups[0]["id"]
    bagel_id = bagels[0]["id"]

    # Stage 1: Start longer asynchronous appliance (oven heat = 35s) FIRST
    sdk.open_oven_door(command_id=pfx + "-open1")
    sdk.load_bagel(bagel_id, command_id=pfx + "-load")
    sdk.close_oven_door(command_id=pfx + "-close1")
    r_h = sdk.start_oven_heat(command_id=pfx + "-heat")
    if r_h.get("receipt") == "unknown":
        r_h = sdk.query_command_status(pfx + "-heat")
    if r_h.get("receipt") not in ("running", "done", "completed"):
        sdk.escalate_to_human_operator(order_id, "heat_unresolved", command_id=pfx + "-esc-h")
        return {"status": "escalated_to_human"}

    # Stage 2: Overlap coffee preparation + brew (22s) + cup delivery while oven is heating!
    sdk.place_cup_under_machine(cup_id, command_id=pfx + "-place")
    obs2 = sdk.observe_scene()
    if not (obs2.get("l2_pouring_safety") or {}).get("safe_to_pour", True):
        sdk.recenter_cup_under_spout(cup_id, command_id=pfx + "-recenter")
    r_b = sdk.start_coffee_machine(command_id=pfx + "-brew")
    if r_b.get("receipt") == "unknown":
        r_b = sdk.query_command_status(pfx + "-brew")
    if r_b.get("receipt") not in ("running", "done", "completed"):
        sdk.escalate_to_human_operator(order_id, "brew_unresolved", command_id=pfx + "-esc-b")
        return {"status": "escalated_to_human"}
    sdk.wait_for_coffee_ready(command_id=pfx + "-wbrew")
    sdk.retrieve_and_deliver_cup(cup_id, slot, command_id=pfx + "-dcup")

    # Stage 3: Finish remaining oven wait (~few seconds), then unload & deliver bagel
    sdk.wait_for_oven_ready(command_id=pfx + "-wheat")
    sdk.open_oven_door(command_id=pfx + "-open2")
    sdk.unload_bagel(bagel_id, command_id=pfx + "-unload")
    sdk.deliver_bagel(bagel_id, slot, command_id=pfx + "-dbagel")
    sdk.complete_order_delivery(order_id, cup_id, slot, command_id=pfx + "-done")
    return {"status": "completed"}
"""


def run_combo_trial(
    host: ComboWorkstationHost,
    policy_source: str,
    scenario: Mapping[str, Any],
    *,
    run_id: str,
    journal: Journal | None = None,
    enable_journal_proxy: bool = False,
) -> dict[str, Any]:
    """Execute one combo order trial with crash/restart support & independent oracle scoring."""
    host.reset(scenario)
    chk = check_sandbox_ast_compliance(policy_source)

    def _exec_once(attempt_tag: str, *, resume: bool) -> tuple[str, str]:
        if not chk["compliant"]:
            return "rejected", "; ".join(chk["issues"][:3])
        sdk = host.make_sdk(resume_after_crash=resume)
        active_sdk: Any = _JournaledComboSDKProxy(sdk, journal, run_id=run_id) if (enable_journal_proxy and journal) else sdk
        ns: dict[str, Any] = {"__builtins__": dict(_SAFE_BUILTINS)}
        try:
            exec(compile(policy_source, "<trax-combo-policy>", "exec"), ns, ns)  # noqa: S102
            fn = ns.get("run_policy")
            if not callable(fn):
                return "rejected", "Missing run_policy"
            ret = fn(
                active_sdk,
                {
                    "order_id": str(scenario.get("order_id", "C101")),
                    "target_slot": str(scenario.get("target_slot", "$SLOT_PICKUP_1")),
                    "command_prefix": f"{run_id}-{attempt_tag}",
                },
            )
            return str((ret or {}).get("status", "unknown")) if isinstance(ret, Mapping) else "invalid_return", ""
        except Exception as exc:  # noqa: BLE001
            return "crashed_mid_action" if host.crashed_mid_action else "exception", f"{type(exc).__name__}: {exc}"

    status, detail = _exec_once("att1", resume=False)
    restarted = False
    if status == "crashed_mid_action":
        restarted = True
        status, detail = _exec_once("att2", resume=True)

    expect = str(scenario.get("expect_status", "completed"))
    dup_coffee = host.brew_count > 1
    dup_oven = host.heat_count > 1
    dup_any = dup_coffee or dup_oven
    unrec = len(host.pending_unknown_ids) > 0

    if expect == "completed":
        passed = (
            status == "completed"
            and host.cup_filled
            and host.cup_delivered
            and host.bagel_heated
            and host.bagel_delivered
            and host.interlock_violations == 0
            and not dup_any
            and not unrec
        )
    else:
        passed = status == expect and host.brew_count == 0 and host.heat_count == 0 and host.interlock_violations == 0

    return {
        "scenario_id": str(scenario.get("scenario_id", run_id)),
        "host_runtime_status": status,
        "detail": detail,
        "restarted_after_crash": restarted,
        "task_passed": passed,
        "sim_cycle_s": round(host.sim_clock_s, 2),
        "brew_count": host.brew_count,
        "heat_count": host.heat_count,
        "duplicate_non_idempotent": dup_any,
        "interlock_violations": host.interlock_violations,
        "unreconciled_unknown": unrec,
        "evidence_source": host.evidence_source,
    }


def combo_eval_scenarios() -> list[dict[str, Any]]:
    """12 held-out multi-appliance commercial combo scenarios."""
    return [
        {"scenario_id": "cmb_01_nominal", "label": "Nominal Coffee+Bagel Combo", "expect_status": "completed"},
        {"scenario_id": "cmb_02_slip", "label": "Cup Slip (36,-10mm) + Bagel", "fault_inject_slip_xy_m": [0.036, -0.010], "expect_status": "completed"},
        {"scenario_id": "cmb_03_brew_timeout", "label": "Coffee Brew Ack Timeout", "simulate_brew_timeout": True, "expect_status": "completed"},
        {"scenario_id": "cmb_04_heat_timeout", "label": "Oven Heat Ack Timeout", "simulate_heat_timeout": True, "expect_status": "completed"},
        {"scenario_id": "cmb_05_dual_timeout", "label": "Both Coffee + Oven Ack Timeout", "simulate_brew_timeout": True, "simulate_heat_timeout": True, "expect_status": "completed"},
        {"scenario_id": "cmb_06_crash_mid_brew", "label": "Mid-Brew Host Crash/Restart", "simulate_crash_mid_brew": True, "expect_status": "completed"},
        {"scenario_id": "cmb_07_crash_mid_heat", "label": "Mid-Heat Host Crash/Restart", "simulate_crash_mid_heat": True, "expect_status": "completed"},
        {"scenario_id": "cmb_08_slip_and_heat_crash", "label": "Cup Slip + Mid-Heat Crash/Restart", "fault_inject_slip_xy_m": [0.032, 0.014], "simulate_crash_mid_heat": True, "expect_status": "completed"},
        {"scenario_id": "cmb_09_dual_cup_pause", "label": "Dual Cup Ambiguity (0 Brew, 0 Heat)", "preset": "sim_dual_cup", "expect_status": "paused_for_operator"},
        {"scenario_id": "cmb_10_stale_obs", "label": "Stale Perception (0 Actuations)", "stale_observation": True, "expect_status": "rejected_observation_not_ready"},
        {"scenario_id": "cmb_11_oven_fault", "label": "Oven Appliance Fault (Must Escalate)", "simulate_oven_fault": True, "expect_status": "escalated_to_human"},
        {"scenario_id": "cmb_12_hazard_slot", "label": "Forbidden Steam Zone (Must Escalate)", "target_slot": "$ZONE_STEAM_NOGO", "expect_status": "escalated_to_human"},
    ]


def run_combo_operations_benchmark(
    output_dir: str | Path,
    *,
    scenarios: Sequence[Mapping[str, Any]] | None = None,
    candidate_policies: Mapping[str, tuple[str, bool]] | None = None,
) -> dict[str, Any]:
    """Compare sequential vs. TRAX pipelined+journaled multi-appliance policies on safety & throughput."""
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    eval_suite = list(scenarios or combo_eval_scenarios())
    host = ComboWorkstationHost()

    methods = dict(
        candidate_policies
        or {
            "sequential_stateless": (builtin_sequential_combo_policy_source(), False),
            "pipelined_stateless": (builtin_pipelined_combo_policy_source(), False),
            "craft_pipelined_journaled": (builtin_pipelined_combo_policy_source(), True),
        }
    )

    summary: dict[str, dict[str, Any]] = {}
    trials: list[dict[str, Any]] = []
    vectors: dict[str, list[bool]] = {}

    for mid, (src, use_journal) in methods.items():
        jpath = out / f"{mid}_journal.sqlite"
        ok_cnt = 0
        dup_cnt = 0
        interlock_cnt = 0
        completed_cycles: list[float] = []
        vec: list[bool] = []
        with Journal(jpath) as journal:
            for sc in eval_suite:
                rec = run_combo_trial(
                    host,
                    src,
                    sc,
                    run_id=f"{mid}-{sc['scenario_id']}",
                    journal=journal if use_journal else None,
                    enable_journal_proxy=use_journal,
                )
                rec["method_id"] = mid
                trials.append(rec)
                vec.append(bool(rec["task_passed"]))
                if rec["task_passed"]:
                    ok_cnt += 1
                    if sc.get("expect_status") == "completed":
                        completed_cycles.append(float(rec["sim_cycle_s"]))
                if rec["duplicate_non_idempotent"]:
                    dup_cnt += 1
                interlock_cnt += int(rec["interlock_violations"])

        mean_cycle = round(sum(completed_cycles) / len(completed_cycles), 2) if completed_cycles else 0.0
        oph = round(3600.0 / mean_cycle, 1) if mean_cycle > 0 else 0.0
        vectors[mid] = vec
        summary[mid] = {
            "method_id": mid,
            "enable_journal_proxy": use_journal,
            "passed": ok_cnt,
            "total_trials": len(eval_suite),
            "pass_rate": round(ok_cnt / len(eval_suite), 4) if eval_suite else 0.0,
            "wilson_95_ci": list(wilson_ci_95(ok_cnt, len(eval_suite))),
            "duplicate_non_idempotent_orders": dup_cnt,
            "interlock_violations": interlock_cnt,
            "mean_completed_cycle_s": mean_cycle,
            "throughput_orders_per_hour": oph,
        }

    paired: dict[str, dict[str, Any]] = {}
    if "craft_pipelined_journaled" in vectors:
        ref = vectors["craft_pipelined_journaled"]
        for mid, vec in vectors.items():
            if mid != "craft_pipelined_journaled":
                paired[mid] = mcnemar_exact_p(ref, vec)

    report = {
        "benchmark_version": COMBO_BENCH_VERSION,
        "evidence_source": "simulator",
        "non_idempotent_methods": list(COMBO_NON_IDEMPOTENT_METHODS),
        "provenance": _git_provenance(),
        "finished_at_utc": _utc_now(),
        "summary_by_method": summary,
        "paired_mcnemar_craft_vs": paired,
        "trials": trials,
    }
    _write_json_atomic(out / "combo_report.json", report)
    return report
