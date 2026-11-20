"""Unified CRAFT CLI entry point: fixture demo, host discovery, and coffee closed loop."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile

from . import __version__
from .coffee_loop import check_oven_capability_gap, run_coffee_closed_loop
from .config import SceneConfig
from .edge_adapter import EdgeHostAdapter
from .fixtures import DEPLOYMENT, FixtureAdapter, example_policy
from .journal import Journal
from .runtime import execute


def _override_config(
    cfg: SceneConfig,
    *,
    edge_base: str | None = None,
    sim_base: str | None = None,
    provider: str | None = None,
    model: str | None = None,
    effort: str | None = None,
    allow_actuation: bool = False,
    allow_sim_reset: bool = False,
) -> SceneConfig:
    data = cfg.to_dict()
    if edge_base:
        data["host"]["edge_base_url"] = edge_base
    if sim_base:
        data["host"]["sim_base_url"] = sim_base
    if provider:
        data["generation"]["provider"] = provider
    if model:
        data["generation"]["model"] = model
    if effort:
        data["generation"]["reasoning_effort"] = effort
    if allow_actuation:
        data["budget"]["allow_live_actuation"] = True
    if allow_sim_reset:
        data["budget"]["allow_sim_reset"] = True
    return SceneConfig.from_dict(data)


def main() -> None:
    parser = argparse.ArgumentParser(description="CRAFT unified CLI")
    parser.add_argument("--version", action="version", version=__version__)
    sub = parser.add_subparsers(dest="command", required=True)

    demo = sub.add_parser("demo", help="Deterministic contract fixture, not robot validation")
    demo.add_argument("--task", choices=["coffee", "bagel", "combo"], default="combo")
    demo.add_argument("--lost-receipt", action="store_true")

    disc = sub.add_parser("discover", help="Read-only host capability discovery and EvidenceScene check")
    disc.add_argument("--config", required=True, help="Path to SceneConfig JSON file")
    disc.add_argument("--edge-base", help="Override edge_base_url at runtime")
    disc.add_argument("--check-oven-gap", action="store_true", help="Also report oven/bagel capability gap")

    loop = sub.add_parser("run-coffee-loop", help="Run Phase 1 single-cup coffee simulation closed loop")
    loop.add_argument("--config", required=True, help="Path to SceneConfig JSON file")
    loop.add_argument("--workspace", required=True, help="Output directory for persistent session & artifacts")
    loop.add_argument("--edge-base", help="Override edge_base_url at runtime")
    loop.add_argument("--sim-base", help="Override sim_base_url at runtime")
    loop.add_argument("--provider", choices=["codex_cli", "openai_api"], help="Override generation provider")
    loop.add_argument("--model", help="Override generation model")
    loop.add_argument("--effort", choices=["low", "medium", "high", "xhigh"], help="Override reasoning effort")
    loop.add_argument("--allow-actuation", action="store_true", help="Explicitly authorize /cap/execute dispatch")
    loop.add_argument("--allow-sim-reset", action="store_true", help="Explicitly authorize simulator reset")
    loop.add_argument("--skip-improvement", action="store_true", help="Skip Stage 6 improvement candidate step")

    args = parser.parse_args()

    if args.command == "demo":
        adapter = (
            FixtureAdapter(lose_receipt="heat" if args.task != "coffee" else "brew")
            if args.lost_receipt
            else FixtureAdapter()
        )
        bundle = example_policy(args.task)
        with tempfile.TemporaryDirectory(prefix="craft-demo-") as tmp:
            path = Path(tmp) / "journal.sqlite"
            with Journal(path) as journal:
                first = execute(
                    bundle,
                    adapter,
                    journal,
                    run_id="fixture-order-1",
                    parameters={"order_id": "example-1"},
                    deployment=DEPLOYMENT,
                    goal={"task": args.task},
                )
            with Journal(path) as journal:
                resumed = execute(
                    bundle,
                    adapter,
                    journal,
                    run_id="fixture-order-1",
                    parameters={"order_id": "example-1"},
                    deployment=DEPLOYMENT,
                    goal={"task": args.task},
                )
        print(
            json.dumps(
                {
                    "scope": "software_contract_fixture_only",
                    "first": first,
                    "resumed": resumed,
                    "physical_commands": 0,
                    "fixture_dispatches": adapter.calls,
                },
                indent=2,
            )
        )
        if resumed["result"]["status"] != "completed":
            raise SystemExit(1)
        return

    if args.command == "discover":
        cfg = _override_config(SceneConfig.from_file(args.config), edge_base=args.edge_base)
        host = EdgeHostAdapter(cfg)
        out = {
            "capability_check": host.discover_capabilities().to_dict(),
            "scene_observation": host.observe_scene().to_dict(),
        }
        if args.check_oven_gap:
            out["oven_gap"] = check_oven_capability_gap(cfg, host)
        print(json.dumps(out, indent=2))
        return

    if args.command == "run-coffee-loop":
        cfg = _override_config(
            SceneConfig.from_file(args.config),
            edge_base=args.edge_base,
            sim_base=args.sim_base,
            provider=args.provider,
            model=args.model,
            effort=args.effort,
            allow_actuation=args.allow_actuation,
            allow_sim_reset=args.allow_sim_reset,
        )
        summary = run_coffee_closed_loop(
            cfg,
            args.workspace,
            run_improvement_step=not args.skip_improvement,
        )
        print(json.dumps(summary, indent=2))
        if summary.get("status") != "completed":
            raise SystemExit(1)


if __name__ == "__main__":
    main()
