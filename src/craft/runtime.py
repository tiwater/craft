"""Frozen policies and receipt-driven execution. No perception or motion estimator."""
from __future__ import annotations
import json
from dataclasses import asdict, dataclass
from typing import Mapping
from .contracts import Adapter, Receipt, canonical, digest
from .journal import Journal
from .policy import run, validate


@dataclass(frozen=True)
class PolicyBundle:
    source: str
    deployment: dict
    required_capabilities: tuple[str, ...]
    sha256: str
    schema: str = "craft.policy.v1"

    @classmethod
    def freeze(cls, source: str, deployment: dict, required_capabilities):
        validate(source)
        deployment = json.loads(canonical(deployment))
        caps = tuple(sorted(set(required_capabilities)))
        payload = dict(source=source, deployment=deployment, required_capabilities=caps, schema="craft.policy.v1")
        return cls(source, deployment, caps, digest(payload))

    def verify(self, deployment: dict):
        if self.schema != "craft.policy.v1": raise ValueError("Unknown policy schema")
        expected = self.freeze(self.source, self.deployment, self.required_capabilities)
        if expected.sha256 != self.sha256: raise ValueError("Policy content hash mismatch")
        if canonical(self.deployment) != canonical(deployment): raise ValueError("Deployment contract mismatch")

    def to_dict(self): return asdict(self)

    @classmethod
    def from_dict(cls, value):
        bundle = cls(**value)
        bundle.verify(bundle.deployment)
        return bundle


class Pending(Exception):
    def __init__(self, receipt): self.receipt = receipt


class Execution:
    def __init__(self, adapter: Adapter, journal: Journal, run_id: str, capabilities, goal):
        self.adapter, self.journal, self.run_id = adapter, journal, run_id
        self.capabilities = frozenset(capabilities)
        self.verified = False
        self.goal = goal
        self.events = []

    def observe(self):
        observation = json.loads(canonical(self.adapter.observe()))
        self.events.append({"kind": "observation", "value": observation})
        return observation

    def perform(self, action: str, arguments: dict, key: str):
        if action not in self.capabilities or action not in self.adapter.capabilities:
            raise ValueError("Action is outside the declared capability contract")
        if not isinstance(key, str) or not key or not isinstance(arguments, dict):
            raise ValueError("Action requires a stable key and JSON arguments")
        arguments = json.loads(canonical(arguments))
        command_id = digest([self.run_id, key])
        request = {"action": action, "arguments": arguments}
        fresh, saved = self.journal.claim(command_id, request)
        self.verified = False
        try:
            if fresh:
                receipt = self.adapter.execute(action, arguments, command_id)
            elif saved and saved["status"] in {"done", "rejected"}:
                receipt = Receipt(**saved)
            else:
                receipt = self.adapter.query(command_id)
        except Exception:
            # A transport exception cannot establish whether a physical effect occurred.
            receipt = Receipt(command_id, "unknown", {"reason": "adapter_exception"})
        if not isinstance(receipt, Receipt) or receipt.command_id != command_id:
            receipt = Receipt(command_id, "unknown", {"reason": "invalid_receipt_identity"})
        if receipt.status == "done" and not receipt.evidence:
            receipt = Receipt(command_id, "unknown", {"reason": "missing_completion_evidence"})
        self.journal.record(command_id, receipt.to_dict())
        self.events.append({"kind": "command", "action": action, "receipt": receipt.to_dict(), "dispatched": fresh})
        if receipt.status != "done": raise Pending(receipt)
        return receipt.to_dict()

    def verify(self, goal: dict):
        self.verified = False
        if canonical(goal) != canonical(self.goal):
            raise ValueError("Policy cannot redefine the requested goal")
        result = json.loads(canonical(self.adapter.verify(self.goal)))
        self.verified = result.get("satisfied") is True and bool(result.get("evidence"))
        self.events.append({"kind": "verification", "goal": goal, "result": result})
        return self.verified


def execute(bundle: PolicyBundle, adapter: Adapter, journal: Journal, *, run_id: str,
            parameters: dict, deployment: dict, goal: dict) -> dict:
    """Resume only the same frozen policy/run. Unknown commands are queried, never retried.

    A changed policy must not automatically get a new run ID and repeat side effects.
    Cross-policy recovery needs explicit adapter-side reconciliation (not implemented).
    """
    if not isinstance(run_id, str) or not run_id: raise ValueError("run_id is required")
    bundle.verify(deployment)
    if not set(bundle.required_capabilities) <= set(adapter.capabilities):
        raise ValueError("Adapter is missing required capabilities")
    parameters = json.loads(canonical(parameters))
    journal.bind(run_id, {"policy": bundle.sha256, "parameters": parameters, "deployment": deployment, "goal": goal})
    sdk = Execution(adapter, journal, run_id, bundle.required_capabilities, json.loads(canonical(goal)))
    try:
        result = run(bundle.source, sdk, parameters)
        if not isinstance(result, dict): raise ValueError("Policy must return a JSON object")
        result = json.loads(canonical(result))
        if result.get("status") == "completed" and not sdk.verified:
            result = {"status": "blocked", "reason": "goal_not_verified"}
    except Pending as exc:
        result = {"status": "blocked" if exc.receipt.status == "rejected" else "pending",
                  "receipt": exc.receipt.to_dict()}
    return {"result": result, "events": sdk.events, "policy_sha256": bundle.sha256, "run_id": run_id}
