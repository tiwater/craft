"""Deterministic contract fixture, NOT a simulator, robot driver or scientific benchmark."""
from .contracts import Receipt
from .runtime import PolicyBundle

DEPLOYMENT = {"adapter": "contract-fixture.v1", "equipment": "not-a-physical-device"}
CAPABILITIES = frozenset({"place_cup", "brew", "deliver_coffee", "load_bagel", "heat", "unload_bagel", "deliver_bagel"})


class FixtureAdapter:
    capabilities = CAPABILITIES

    def __init__(self, lose_receipt=None):
        self.completed = set()
        self.receipts = {}
        self.calls = []
        self.lose_receipt = lose_receipt

    def observe(self):
        return {"source": "contract_fixture", "control_ready": True, "completed": sorted(self.completed)}

    def execute(self, action, arguments, command_id):
        if command_id in self.receipts: return self.receipts[command_id]
        needs = {"brew": "place_cup", "deliver_coffee": "brew", "heat": "load_bagel",
                 "unload_bagel": "heat", "deliver_bagel": "unload_bagel"}
        self.calls.append(action)
        if action in needs and needs[action] not in self.completed:
            receipt = Receipt(command_id, "rejected", {"missing_step": needs[action]})
        else:
            self.completed.add(action)
            receipt = Receipt(command_id, "done", {"source": "contract_fixture", "action": action})
        self.receipts[command_id] = receipt
        if self.lose_receipt == action:
            self.lose_receipt = None
            return Receipt(command_id, "unknown", {"fault": "lost_receipt"})
        return receipt

    def query(self, command_id):
        return self.receipts.get(command_id, Receipt(command_id, "unknown", {"reason": "not_found"}))

    def verify(self, goal):
        required = {"coffee": {"deliver_coffee"}, "bagel": {"deliver_bagel"},
                    "combo": {"deliver_coffee", "deliver_bagel"}}[goal["task"]]
        return {"satisfied": required <= self.completed,
                "evidence": {"source": "contract_fixture", "completed": sorted(self.completed)}}


def example_policy(task: str) -> PolicyBundle:
    steps = {"coffee": ["place_cup", "brew", "deliver_coffee"],
             "bagel": ["load_bagel", "heat", "unload_bagel", "deliver_bagel"]}
    actions = steps[task] if task != "combo" else steps["coffee"] + steps["bagel"]
    source = '''def run_policy(sdk, params):
    observation = sdk.observe()
    if not observation["control_ready"]:
        return {"status": "blocked"}
'''
    source += ''.join(f'    sdk.perform("{action}", {{"order_id": params["order_id"]}}, "{action}")\n' for action in actions)
    source += f'    if sdk.verify({{"task": "{task}"}}):\n        return {{"status": "completed"}}\n    return {{"status": "blocked"}}\n'
    return PolicyBundle.freeze(source, DEPLOYMENT, actions)
