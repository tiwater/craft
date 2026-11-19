"""Local contract demo; never opens a robot connection."""
import argparse
import json
import tempfile
from pathlib import Path
from . import __version__
from .fixtures import DEPLOYMENT, FixtureAdapter, example_policy
from .journal import Journal
from .runtime import execute


def main():
    parser = argparse.ArgumentParser(description="CRAFT research baseline")
    parser.add_argument("--version", action="version", version=__version__)
    sub = parser.add_subparsers(dest="command", required=True)
    demo = sub.add_parser("demo", help="Deterministic contract fixture, not robot validation")
    demo.add_argument("--task", choices=["coffee", "bagel", "combo"], default="combo")
    demo.add_argument("--lost-receipt", action="store_true")
    args = parser.parse_args()
    adapter = FixtureAdapter(lose_receipt="heat" if args.task != "coffee" else "brew") if args.lost_receipt else FixtureAdapter()
    bundle = example_policy(args.task)
    with tempfile.TemporaryDirectory(prefix="craft-demo-") as tmp:
        path = Path(tmp) / "journal.sqlite"
        with Journal(path) as journal:
            first = execute(bundle, adapter, journal, run_id="fixture-order-1", parameters={"order_id": "example-1"}, deployment=DEPLOYMENT, goal={"task": args.task})
        with Journal(path) as journal:
            resumed = execute(bundle, adapter, journal, run_id="fixture-order-1", parameters={"order_id": "example-1"}, deployment=DEPLOYMENT, goal={"task": args.task})
    print(json.dumps({"scope": "software_contract_fixture_only", "first": first, "resumed": resumed,
                      "physical_commands": 0, "fixture_dispatches": adapter.calls}, indent=2))
    if resumed["result"]["status"] != "completed": raise SystemExit(1)

if __name__ == "__main__": main()
