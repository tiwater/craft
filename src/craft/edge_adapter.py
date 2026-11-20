"""Read-only perception pass-through and host-delegated execution adapter for ticos-edge.

Existing producers and state owners (verified against ticos-edge source):
- ``ticos-edge/tools/edge_cap_runtime.py``:
  - ``EdgeCapRuntime.get_state`` (``GET /cap/state``) owns runtime state,
    ``sdkAllowedMethods``, ``sdkContractSha256``, and ``deviceBundleSha256``.
  - ``EdgeCapRuntime._run_job`` (``POST /cap/execute``) constructs
    ``EdgeZeroGtCoffeeSDK`` and executes policies inside the subprocess sandbox.
- ``ticos-edge/tools/coffee_cap_sdk.py``:
  - ``ConstrainedCoffeeRobotSDK`` and ``SDKPublicProxy._ALLOWED_METHODS`` own
    the 13 allowed host capabilities and physical interlocks.
  - ``compute_sdk_contract_sha256`` owns the SDK contract digest.
  - ``execute_policy_in_subprocess_sandbox`` owns OS process isolation.
- ``EvidenceScene`` (``GET /edge-scene``) owns RGB-D perception and world-model
  snapshots.

This adapter consumes existing host state without adding any second detector,
world model, coordinate estimator, or web-side inference. Non-idempotent actions
with unknown outcomes are never blindly retried; callers must reconcile via
``reconcile_command`` (read-only query).
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import time
from typing import Any, Mapping, Protocol
import urllib.error
import urllib.request

from .config import SceneConfig
from .contracts import Receipt, digest
from .journal import Journal


class TransportError(RuntimeError):
    """Raised on network disconnect, HTTP error, or malformed JSON."""


class TransportTimeoutError(TransportError):
    """Raised when an HTTP request times out (outcome on host may be unknown)."""


class HttpTransport(Protocol):
    def get_json(self, url: str, timeout_s: float) -> dict[str, Any]: ...
    def post_json(self, url: str, body: Mapping[str, Any], timeout_s: float) -> dict[str, Any]: ...


class UrllibJsonTransport:
    """Stdlib-only HTTP JSON transport that bypasses system proxy env vars."""

    def __init__(self) -> None:
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def get_json(self, url: str, timeout_s: float) -> dict[str, Any]:
        req = urllib.request.Request(url, method="GET", headers={"Accept": "application/json"})
        return self._send(req, timeout_s)

    def post_json(self, url: str, body: Mapping[str, Any], timeout_s: float) -> dict[str, Any]:
        data = json.dumps(dict(body)).encode("utf-8")
        req = urllib.request.Request(
            url,
            data=data,
            method="POST",
            headers={"Accept": "application/json", "Content-Type": "application/json"},
        )
        return self._send(req, timeout_s)

    def _send(self, req: urllib.request.Request, timeout_s: float) -> dict[str, Any]:
        try:
            with self._opener.open(req, timeout=timeout_s) as resp:
                raw = resp.read().decode("utf-8")
                payload = json.loads(raw)
        except TimeoutError as exc:
            raise TransportTimeoutError(f"Request timed out after {timeout_s}s: {req.full_url}") from exc
        except urllib.error.HTTPError as exc:
            raise TransportError(f"HTTP {exc.code} from {req.full_url}") from exc
        except urllib.error.URLError as exc:
            if isinstance(exc.reason, TimeoutError):
                raise TransportTimeoutError(f"Request timed out after {timeout_s}s: {req.full_url}") from exc
            raise TransportError(f"Connection failed to {req.full_url}: {exc.reason}") from exc
        except (ValueError, UnicodeDecodeError) as exc:
            raise TransportError(f"Invalid JSON response from {req.full_url}: {exc}") from exc
        if not isinstance(payload, dict):
            raise TransportError(f"Expected JSON object from {req.full_url}")
        return payload


@dataclass(frozen=True)
class DiscoveredCapabilities:
    execution_authority: str
    perception_owner: str
    sdk_version: str
    allowed_methods: tuple[str, ...]
    sdk_contract_sha256: str
    device_bundle_id: str
    device_bundle_sha256: str
    runtime_status: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "execution_authority": self.execution_authority,
            "perception_owner": self.perception_owner,
            "sdk_version": self.sdk_version,
            "allowed_methods": list(self.allowed_methods),
            "sdk_contract_sha256": self.sdk_contract_sha256,
            "device_bundle_id": self.device_bundle_id,
            "device_bundle_sha256": self.device_bundle_sha256,
            "runtime_status": self.runtime_status,
        }


@dataclass(frozen=True)
class CapabilityCheckResult:
    status: str  # ready | missing_capabilities | contract_drift | host_busy | disconnected
    discovered: DiscoveredCapabilities | None = None
    missing_methods: tuple[str, ...] = ()
    detail: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "discovered": self.discovered.to_dict() if self.discovered else None,
            "missing_methods": list(self.missing_methods),
            "detail": self.detail,
        }


@dataclass(frozen=True)
class SceneObservationResult:
    status: str  # ok | stale_evidence | missing_object | ambiguous_object | disconnected
    evidence_source: str
    snapshot_sha256: str | None = None
    timestamp_ns: int | None = None
    age_ms: float | None = None
    bound_objects: Mapping[str, tuple[Mapping[str, Any], ...]] = field(default_factory=dict)
    raw_snapshot: Mapping[str, Any] | None = None
    detail: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "evidence_source": self.evidence_source,
            "snapshot_sha256": self.snapshot_sha256,
            "timestamp_ns": self.timestamp_ns,
            "age_ms": self.age_ms,
            "bound_objects": {k: [dict(o) for o in v] for k, v in self.bound_objects.items()},
            "detail": self.detail,
        }


@dataclass(frozen=True)
class EdgeExecutionResult:
    status: str  # completed | failed | blocked | unknown_outcome | permission_denied | reconciliation_required | disconnected
    command_id: str
    evidence_source: str
    wall_s: float
    host_runtime_status: str | None = None
    methods: tuple[str, ...] = ()
    non_idempotent_counts: Mapping[str, int] = field(default_factory=dict)
    duplicate_non_idempotent: bool = False
    unknown_events_observed: int = 0
    queried_after_unknown: bool = True
    human_interventions: int = 0
    sandbox_mode: str | None = None
    raw_runtime: Mapping[str, Any] | None = None
    detail: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "command_id": self.command_id,
            "evidence_source": self.evidence_source,
            "wall_s": round(self.wall_s, 3),
            "host_runtime_status": self.host_runtime_status,
            "methods": list(self.methods),
            "non_idempotent_counts": dict(self.non_idempotent_counts),
            "duplicate_non_idempotent": self.duplicate_non_idempotent,
            "unknown_events_observed": self.unknown_events_observed,
            "queried_after_unknown": self.queried_after_unknown,
            "human_interventions": self.human_interventions,
            "sandbox_mode": self.sandbox_mode,
            "detail": self.detail,
        }


@dataclass(frozen=True)
class ReconciliationResult:
    status: str  # reconciled_done | reconciled_running | reconciled_failed | still_unknown | disconnected
    command_id: str
    host_runtime_status: str | None = None
    non_idempotent_counts: Mapping[str, int] = field(default_factory=dict)
    detail: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "command_id": self.command_id,
            "host_runtime_status": self.host_runtime_status,
            "non_idempotent_counts": dict(self.non_idempotent_counts),
            "detail": self.detail,
        }


class EdgeHostAdapter:
    """Adapter for an existing ticos-edge CaP host and EvidenceScene endpoint."""

    def __init__(self, config: SceneConfig, *, transport: HttpTransport | None = None) -> None:
        self.config = config
        self.transport: HttpTransport = transport or UrllibJsonTransport()

    @property
    def _base(self) -> str:
        return self.config.host.normalized_edge_base

    def discover_capabilities(self, *, allow_busy: bool = False) -> CapabilityCheckResult:
        """Query GET /cap/state and verify required capabilities and contract hash."""
        url = self._base + self.config.host.cap_state_path
        try:
            state = self.transport.get_json(url, self.config.host.timeout_s)
        except TransportError as exc:
            return CapabilityCheckResult(status="disconnected", detail=str(exc))

        allowed = tuple(str(m) for m in (state.get("sdkAllowedMethods") or []))
        runtime_status = str((state.get("runtime") or {}).get("status", "unknown"))
        discovered = DiscoveredCapabilities(
            execution_authority=str(state.get("executionAuthority") or ""),
            perception_owner=str(state.get("perceptionOwner") or ""),
            sdk_version=str(state.get("sdkVersion") or ""),
            allowed_methods=allowed,
            sdk_contract_sha256=str(state.get("sdkContractSha256") or ""),
            device_bundle_id=str(state.get("deviceBundleId") or ""),
            device_bundle_sha256=str(state.get("deviceBundleSha256") or ""),
            runtime_status=runtime_status,
        )

        missing = tuple(m for m in self.config.capabilities.required_methods if m not in allowed)
        if missing:
            return CapabilityCheckResult(
                status="missing_capabilities",
                discovered=discovered,
                missing_methods=missing,
                detail=f"Host lacks required SDK methods: {', '.join(missing)}",
            )

        expected_hash = self.config.capabilities.expected_sdk_contract_sha256
        if expected_hash and discovered.sdk_contract_sha256 != expected_hash:
            return CapabilityCheckResult(
                status="contract_drift",
                discovered=discovered,
                detail=(
                    f"SDK contract hash mismatch: expected {expected_hash}, "
                    f"discovered {discovered.sdk_contract_sha256}"
                ),
            )

        if not allow_busy and runtime_status == "running":
            return CapabilityCheckResult(
                status="host_busy",
                discovered=discovered,
                detail="Host runtime is currently executing another job",
            )

        return CapabilityCheckResult(status="ready", discovered=discovered, detail="Host capabilities verified")

    def observe_scene(self, *, now_ns: int | None = None) -> SceneObservationResult:
        """Read GET /edge-scene verbatim; validate freshness and object bindings without inference."""
        url = self._base + self.config.host.scene_path
        scope = self.config.budget.deployment_scope
        try:
            snapshot = self.transport.get_json(url, self.config.host.timeout_s)
        except TransportError as exc:
            return SceneObservationResult(status="disconnected", evidence_source=scope, detail=str(exc))

        snap_hash = digest(snapshot)
        raw_ts = snapshot.get("timestamp_ns")
        try:
            ts_ns = int(raw_ts) if raw_ts is not None else 0
        except (TypeError, ValueError):
            ts_ns = 0

        current_ns = time.time_ns() if now_ns is None else int(now_ns)
        if ts_ns <= 0:
            return SceneObservationResult(
                status="stale_evidence",
                evidence_source=scope,
                snapshot_sha256=snap_hash,
                timestamp_ns=None,
                age_ms=None,
                raw_snapshot=snapshot,
                detail="EvidenceScene snapshot missing valid timestamp_ns",
            )

        age_ms = round(max(0.0, (current_ns - ts_ns) / 1e6), 1)
        if age_ms > self.config.host.max_evidence_age_ms:
            return SceneObservationResult(
                status="stale_evidence",
                evidence_source=scope,
                snapshot_sha256=snap_hash,
                timestamp_ns=ts_ns,
                age_ms=age_ms,
                raw_snapshot=snapshot,
                detail=(
                    f"EvidenceScene snapshot is stale ({age_ms} ms > "
                    f"{self.config.host.max_evidence_age_ms} ms)"
                ),
            )

        raw_objects = list(snapshot.get("objects") or [])
        bound: dict[str, tuple[Mapping[str, Any], ...]] = {}
        for binding in self.config.objects:
            matches = tuple(
                obj
                for obj in raw_objects
                if str(obj.get("semantic_class") or "") in binding.semantic_classes
                and float(obj.get("confidence", 1.0)) >= binding.min_confidence
            )
            bound[binding.name] = matches
            if len(matches) < binding.required_count:
                return SceneObservationResult(
                    status="missing_object",
                    evidence_source=scope,
                    snapshot_sha256=snap_hash,
                    timestamp_ns=ts_ns,
                    age_ms=age_ms,
                    bound_objects=bound,
                    raw_snapshot=snapshot,
                    detail=(
                        f"Object binding {binding.name!r} expected at least "
                        f"{binding.required_count} match(es), found {len(matches)}"
                    ),
                )
            if not binding.allow_ambiguous and len(matches) > binding.required_count:
                return SceneObservationResult(
                    status="ambiguous_object",
                    evidence_source=scope,
                    snapshot_sha256=snap_hash,
                    timestamp_ns=ts_ns,
                    age_ms=age_ms,
                    bound_objects=bound,
                    raw_snapshot=snapshot,
                    detail=(
                        f"Object binding {binding.name!r} is ambiguous: expected "
                        f"{binding.required_count}, found {len(matches)}"
                    ),
                )

        return SceneObservationResult(
            status="ok",
            evidence_source=scope,
            snapshot_sha256=snap_hash,
            timestamp_ns=ts_ns,
            age_ms=age_ms,
            bound_objects=bound,
            raw_snapshot=snapshot,
            detail="Fresh EvidenceScene snapshot bound without inference",
        )

    def execute_policy(
        self,
        policy_code: str,
        *,
        command_id: str,
        order_id: str = "A042",
        target_slot: str = "$SLOT_PICKUP_1",
        simulate_timeout: bool = False,
        inject_place_slip_xy_m: tuple[float, float] | list[float] | None = None,
        journal: Journal | None = None,
        run_id: str | None = None,
    ) -> EdgeExecutionResult:
        """Delegate policy execution to ticos-edge /cap/execute with intent journaling."""
        scope = self.config.budget.deployment_scope
        if not self.config.budget.allow_live_actuation:
            return EdgeExecutionResult(
                status="permission_denied",
                command_id=command_id,
                evidence_source=scope,
                wall_s=0.0,
                detail="budget.allow_live_actuation is False; execution blocked",
            )

        request_record = {
            "run_id": run_id or command_id,
            "action": "cap_execute",
            "order_id": order_id,
            "target_slot": target_slot,
            "policy_sha256": digest(policy_code),
        }
        if journal is not None:
            inserted, existing_receipt = journal.claim(command_id, request_record)
            if not inserted:
                prev_status = (existing_receipt or {}).get("status", "pending")
                if prev_status == "done":
                    return EdgeExecutionResult(
                        status="completed",
                        command_id=command_id,
                        evidence_source=scope,
                        wall_s=0.0,
                        host_runtime_status="completed",
                        detail="Reused completed receipt from local journal without redispatch",
                    )
                return EdgeExecutionResult(
                    status="reconciliation_required",
                    command_id=command_id,
                    evidence_source=scope,
                    wall_s=0.0,
                    detail=(
                        f"Command {command_id!r} already recorded with status "
                        f"{prev_status!r}; reconcile via reconcile_command() before any retry"
                    ),
                )

        body: dict[str, Any] = {
            "commandId": command_id,
            "policyCode": policy_code,
            "wait": True,
            "orderId": order_id,
            "targetSlot": target_slot,
            "simulateTimeout": bool(simulate_timeout),
            "injectPlaceSlipXyM": list(inject_place_slip_xy_m) if inject_place_slip_xy_m is not None else None,
        }

        url = self._base + self.config.host.cap_execute_path
        started = time.monotonic()
        try:
            response = self.transport.post_json(url, body, self.config.host.execute_timeout_s)
        except TransportTimeoutError as exc:
            wall_s = time.monotonic() - started
            if journal is not None:
                journal.record(command_id, Receipt(command_id, "unknown", {"error": str(exc)}).to_dict())
            return EdgeExecutionResult(
                status="unknown_outcome",
                command_id=command_id,
                evidence_source=scope,
                wall_s=wall_s,
                unknown_events_observed=1,
                queried_after_unknown=False,
                detail=f"Execute request timed out; physical/sim outcome unknown ({exc})",
            )
        except TransportError as exc:
            wall_s = time.monotonic() - started
            if journal is not None:
                journal.record(command_id, Receipt(command_id, "unknown", {"error": str(exc)}).to_dict())
            return EdgeExecutionResult(
                status="disconnected",
                command_id=command_id,
                evidence_source=scope,
                wall_s=wall_s,
                unknown_events_observed=1,
                queried_after_unknown=False,
                detail=str(exc),
            )

        wall_s = time.monotonic() - started
        rt = response.get("runtime") or {}
        host_status = str(rt.get("status") or "unknown")
        steps = list(rt.get("steps") or [])
        methods = tuple(str(s.get("method") or "") for s in steps)
        plan = list((rt.get("auditRecord") or {}).get("plan") or [])
        res = rt.get("result") or {}
        sandbox_mode = (res.get("sandbox_isolation") or {}).get("mode")

        non_idem_counts: dict[str, int] = {}
        for name in self.config.capabilities.non_idempotent_methods:
            count = sum(int(item.get("execution_count", 0)) for item in plan if item.get("skill_kind") == name)
            non_idem_counts[name] = count
        duplicate_non_idem = any(c > 1 for c in non_idem_counts.values())

        unknown_indices = [i for i, m in enumerate(methods) if m == "command_ack_timeout_unknown"]
        queried_after = all("query_command_status" in methods[i + 1 :] for i in unknown_indices)
        human_interventions = sum(
            1 for m in methods if m in {"request_operator_confirmation", "escalate_to_human_operator"}
        )

        if host_status in {"completed", "done"}:
            status = "completed"
            journal_status = "done"
        elif host_status in {"unknown", "running"}:
            status = "unknown_outcome"
            journal_status = "unknown"
        elif host_status in {"blocked", "escalated", "aborted"}:
            status = "blocked"
            journal_status = "rejected"
        else:
            status = "failed"
            journal_status = "rejected"

        if journal is not None:
            journal.record(
                command_id,
                Receipt(
                    command_id,
                    journal_status,
                    {
                        "host_runtime_status": host_status,
                        "non_idempotent_counts": non_idem_counts,
                        "unknown_events": len(unknown_indices),
                    },
                ).to_dict(),
            )

        return EdgeExecutionResult(
            status=status,
            command_id=command_id,
            evidence_source=scope,
            wall_s=wall_s,
            host_runtime_status=host_status,
            methods=methods,
            non_idempotent_counts=non_idem_counts,
            duplicate_non_idempotent=duplicate_non_idem,
            unknown_events_observed=len(unknown_indices),
            queried_after_unknown=queried_after,
            human_interventions=human_interventions,
            sandbox_mode=str(sandbox_mode) if sandbox_mode is not None else None,
            raw_runtime=rt,
            detail=f"Host runtime finished with status={host_status}",
        )

    def reconcile_command(self, command_id: str, *, journal: Journal | None = None) -> ReconciliationResult:
        """Read-only query of GET /cap/state to reconcile an unknown execution outcome."""
        url = self._base + self.config.host.cap_state_path
        try:
            state = self.transport.get_json(url, self.config.host.timeout_s)
        except TransportError as exc:
            return ReconciliationResult(status="disconnected", command_id=command_id, detail=str(exc))

        rt = state.get("runtime") or {}
        host_cmd = rt.get("commandId") or (rt.get("auditRecord") or {}).get("command_id")
        host_status = str(rt.get("status") or "unknown")
        plan = list((rt.get("auditRecord") or {}).get("plan") or [])
        non_idem_counts = {
            name: sum(int(item.get("execution_count", 0)) for item in plan if item.get("skill_kind") == name)
            for name in self.config.capabilities.non_idempotent_methods
        }

        if host_cmd != command_id:
            return ReconciliationResult(
                status="still_unknown",
                command_id=command_id,
                host_runtime_status=host_status,
                non_idempotent_counts=non_idem_counts,
                detail=f"Host runtime reports commandId={host_cmd!r}, not {command_id!r}; do not blindly retry",
            )

        if host_status in {"completed", "done"}:
            if journal is not None:
                journal.record(command_id, Receipt(command_id, "done", {"reconciled_from": "cap_state"}).to_dict())
            return ReconciliationResult(
                status="reconciled_done",
                command_id=command_id,
                host_runtime_status=host_status,
                non_idempotent_counts=non_idem_counts,
                detail="Host confirmed command completed",
            )
        if host_status == "running":
            return ReconciliationResult(
                status="reconciled_running",
                command_id=command_id,
                host_runtime_status=host_status,
                non_idempotent_counts=non_idem_counts,
                detail="Host reports command is still running",
            )
        if host_status in {"failed", "blocked", "aborted", "escalated", "error"}:
            if journal is not None:
                journal.record(
                    command_id,
                    Receipt(command_id, "rejected", {"reconciled_from": "cap_state", "host_status": host_status}).to_dict(),
                )
            return ReconciliationResult(
                status="reconciled_failed",
                command_id=command_id,
                host_runtime_status=host_status,
                non_idempotent_counts=non_idem_counts,
                detail=f"Host confirmed command ended with status={host_status}",
            )

        return ReconciliationResult(
            status="still_unknown",
            command_id=command_id,
            host_runtime_status=host_status,
            non_idempotent_counts=non_idem_counts,
            detail=f"Host status {host_status!r} does not resolve command outcome",
        )

    def reset_host_and_sim(self, *, preset: str = "default") -> dict[str, Any]:
        """Reset edge runtime and simulator testbed when explicitly permitted by config."""
        if not self.config.budget.allow_sim_reset:
            raise PermissionError("budget.allow_sim_reset is False")
        sim_base = self.config.host.normalized_sim_base
        if not sim_base:
            raise ValueError("host.sim_base_url is required for simulator reset")
        cap_reset = self.transport.post_json(
            self._base + self.config.host.cap_reset_path, {}, self.config.host.timeout_s
        )
        sim_setup = self.transport.post_json(
            sim_base + "/v1/sim/testbed/setup", {"preset": preset}, self.config.host.timeout_s
        )
        return {"cap_reset": cap_reset, "sim_setup": sim_setup}

    def read_sim_oracle(self) -> dict[str, Any]:
        """Read independent post-execution simulator oracle. Never pass to policy generation."""
        sim_base = self.config.host.normalized_sim_base
        if not sim_base:
            raise ValueError("host.sim_base_url is required to read simulator oracle")
        return self.transport.get_json(sim_base + "/v1/sim/testbed/oracle", self.config.host.timeout_s)
