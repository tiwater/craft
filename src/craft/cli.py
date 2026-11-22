"""Unified CRAFT CLI: fixture demo, host discovery, closed loop, paired benchmark, and Studio."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile

from . import __version__
from .benchmark import (
    PolicyEntry,
    builtin_reference_policy_source,
    default_coffee_benchmark_suite,
    run_paired_benchmark,
)
from .cafebench import run_cafebench_v1
from .coffee_loop import check_oven_capability_gap, run_coffee_closed_loop
from .config import SceneConfig
from .edge_adapter import EdgeHostAdapter
from .engine_api import CraftEngineService, create_engine_server
from .fixtures import DEPLOYMENT, FixtureAdapter, example_policy
from .host_driver import create_host_driver
from .journal import Journal
from .runtime import execute
from .studio import CraftStudioApp, create_studio_server


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
    scope: str | None = None,
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
    if scope:
        data["budget"]["deployment_scope"] = scope
    return SceneConfig.from_dict(data)


def _parse_policy_specs(specs: list[str], *, include_builtin_ref: bool = True) -> list[PolicyEntry]:
    entries: list[PolicyEntry] = []
    if include_builtin_ref:
        entries.append(
            PolicyEntry(
                policy_id="craft_reference",
                label="CRAFT Reference Baseline",
                source=builtin_reference_policy_source(),
                origin="reference",
            )
        )
    for idx, spec in enumerate(specs, start=1):
        if "=" in spec:
            label, path_str = spec.split("=", 1)
        else:
            path_str = spec
            label = Path(path_str).stem
        pid = "".join(ch if ch.isalnum() or ch in ("_", "-") else "_" for ch in label.lower()) or f"policy_{idx}"
        source = Path(path_str).read_text(encoding="utf-8")
        entries.append(PolicyEntry(policy_id=pid, label=label, source=source, origin="file"))
    return entries


def main() -> None:
    parser = argparse.ArgumentParser(description="CRAFT unified CLI")
    parser.add_argument("--version", action="version", version=__version__)
    sub = parser.add_subparsers(dest="command", required=True)

    demo = sub.add_parser("demo", help="Deterministic contract fixture, not robot validation")
    demo.add_argument("--task", choices=["coffee", "bagel", "combo"], default="combo")
    demo.add_argument("--lost-receipt", action="store_true")

    disc = sub.add_parser("discover", help="Read-only host capability discovery and scene check")
    disc.add_argument("--config", required=True, help="Path to SceneConfig JSON file")
    disc.add_argument("--driver", default="fixture", help="Host driver name (fixture, http, or craft.drivers plugin)")
    disc.add_argument("--host-base", "--edge-base", dest="edge_base", help="Override host base URL at runtime")
    disc.add_argument("--check-oven-gap", action="store_true", help="Also report oven/bagel capability gap")

    loop = sub.add_parser("run-coffee-loop", help="Run Phase 1 single-cup coffee simulation closed loop")
    loop.add_argument("--config", required=True, help="Path to SceneConfig JSON file")
    loop.add_argument("--workspace", required=True, help="Output directory for persistent session & artifacts")
    loop.add_argument("--host-base", "--edge-base", dest="edge_base", help="Override host base URL at runtime")
    loop.add_argument("--evaluator-base", "--sim-base", dest="sim_base", help="Override evaluator/sim base URL at runtime")
    loop.add_argument("--provider", choices=["codex_cli", "openai_api"], help="Override generation provider")
    loop.add_argument("--model", help="Override generation model")
    loop.add_argument("--effort", choices=["low", "medium", "high", "xhigh"], help="Override reasoning effort")
    loop.add_argument("--allow-actuation", action="store_true", help="Explicitly authorize policy execution")
    loop.add_argument("--allow-sim-reset", action="store_true", help="Explicitly authorize scenario reset")
    loop.add_argument("--skip-improvement", action="store_true", help="Skip Stage 6 improvement candidate step")

    bench = sub.add_parser("benchmark", help="Run host-agnostic paired benchmark and generate HTML report")
    bench.add_argument("--config", required=True, help="Path to SceneConfig JSON file")
    bench.add_argument("--driver", default="fixture", help="Host driver name (fixture, http, or craft.drivers plugin)")
    bench.add_argument("--output", required=True, help="Output directory for JSON and HTML benchmark report")
    bench.add_argument("--repeats", type=int, default=1, help="Repeats per scenario")
    bench.add_argument("--policy", action="append", default=[], help="Policy spec as label=/path/to/policy.py")
    bench.add_argument("--no-builtin-ref", action="store_true", help="Exclude built-in reference policy")
    bench.add_argument("--host-base", "--edge-base", dest="edge_base", help="Override host base URL (for http/plugin drivers)")
    bench.add_argument("--evaluator-base", "--sim-base", dest="sim_base", help="Override evaluator/sim base URL")
    bench.add_argument("--allow-actuation", action="store_true")
    bench.add_argument("--allow-sim-reset", action="store_true")

    cb = sub.add_parser("cafebench", help="Run standardized 20-scenario CRAFT-CafeBench v1 suite")
    cb.add_argument("--config", required=True, help="Path to SceneConfig JSON file")
    cb.add_argument("--driver", default="fixture", help="Host driver name (fixture, http, recorded, or plugin)")
    cb.add_argument("--output", required=True, help="Output directory for sealed JSON & HTML report")
    cb.add_argument("--repeats", type=int, default=1, help="Repeats per scenario")
    cb.add_argument("--no-resume", action="store_true", help="Ignore prior partial results")
    cb.add_argument("--policy", action="append", default=[], help="Policy spec as label=/path/to/policy.py")
    cb.add_argument("--no-builtin-ref", action="store_true", help="Exclude built-in reference policy")
    cb.add_argument("--host-base", "--edge-base", dest="edge_base", help="Override host base URL")
    cb.add_argument("--evaluator-base", "--sim-base", dest="sim_base", help="Override evaluator/sim base URL")
    cb.add_argument("--allow-actuation", action="store_true")
    cb.add_argument("--allow-sim-reset", action="store_true")

    pw = sub.add_parser(
        "prior-work-bench",
        help="Run matched-budget prior-work comparison (Strong Zero-Shot / ASPIRE / CaP-X / RoboRepair / CRAFT)",
    )
    pw.add_argument("--config", required=True, help="Path to SceneConfig JSON file")
    pw.add_argument("--driver", default="fixture", help="Host driver name (fixture, http, or plugin)")
    pw.add_argument("--output", required=True, help="Output directory for artifacts, JSON & HTML report")
    pw.add_argument("--provider", default="codex_cli", choices=["codex_cli", "openai_api"])
    pw.add_argument("--model", default="gpt-6-astra", help="Shared LLM model name")
    pw.add_argument("--effort", default="medium", choices=["low", "medium", "high", "xhigh"])
    pw.add_argument("--max-dev-calls", type=int, default=3, help="Matched max dev LLM calls per offline method")
    pw.add_argument("--max-online-regen", type=int, default=1, help="Max online regen turns for CaP-X / RoboRepair")
    pw.add_argument("--host-base", "--edge-base", dest="edge_base", help="Override host base URL")
    pw.add_argument("--evaluator-base", "--sim-base", dest="sim_base", help="Override evaluator/sim base URL")
    pw.add_argument("--allow-actuation", action="store_true")
    pw.add_argument("--allow-sim-reset", action="store_true")

    rb = sub.add_parser(
        "robosuite-bench",
        help="Run Phase 3B public MuJoCo Robosuite parity benchmark (Zero-Shot / ASPIRE / CaP-X / CRAFT)",
    )
    rb.add_argument("--output", required=True, help="Output directory for Robosuite policies, JSON & HTML report")
    rb.add_argument("--tasks", default="Lift,PickPlaceCan,NutAssemblySquare,Door", help="Comma-separated Robosuite tasks")
    rb.add_argument("--dev-seeds", default="101,102,103,104,105", help="Comma-separated dev seeds")
    rb.add_argument("--eval-seeds", default="1-20", help="Seed range (e.g. 1-20) or comma list")
    rb.add_argument("--provider", default="codex_cli", choices=["codex_cli", "openai_api"])
    rb.add_argument("--model", default="gpt-6-astra")
    rb.add_argument("--effort", default="medium", choices=["low", "medium", "high", "xhigh"])
    rb.add_argument("--max-dev-calls", type=int, default=4)
    rb.add_argument("--no-online-regen", action="store_true", help="Disable CaP-X online Turn-2 regen during eval")

    studio = sub.add_parser("studio", help="Launch standalone interactive CRAFT Studio Web Demo")
    studio.add_argument("--config", required=True, help="Path to SceneConfig JSON file")
    studio.add_argument("--driver", default="fixture", help="Host driver name (fixture, http, or craft.drivers plugin)")
    studio.add_argument("--workspace", required=True, help="Workspace dir for releases & benchmark results")
    studio.add_argument("--host", default="127.0.0.1", help="Bind host (default 127.0.0.1)")
    studio.add_argument("--port", type=int, default=8860, help="Bind port (default 8860)")
    studio.add_argument("--policy", action="append", default=[], help="Extra policy as label=/path/to/policy.py")
    studio.add_argument("--host-base", "--edge-base", dest="edge_base", help="Override host base URL (for http/plugin drivers)")
    studio.add_argument("--evaluator-base", "--sim-base", dest="sim_base", help="Override evaluator/sim base URL")
    studio.add_argument("--allow-actuation", action="store_true")
    studio.add_argument("--allow-sim-reset", action="store_true")

    engine = sub.add_parser("serve-engine", help="Launch headless CRAFT Engine /v1/... REST API for control planes")
    engine.add_argument("--config", required=True, help="Path to SceneConfig JSON file")
    engine.add_argument("--driver", default="fixture", help="Host driver name (fixture, http, recorded, or plugin)")
    engine.add_argument("--workspace", required=True, help="Workspace dir for releases, regression & autonomy ledgers")
    engine.add_argument("--host", default="127.0.0.1", help="Bind host (default 127.0.0.1)")
    engine.add_argument("--port", type=int, default=8870, help="Bind port (default 8870)")
    engine.add_argument("--host-base", "--edge-base", dest="edge_base", help="Override host base URL")
    engine.add_argument("--evaluator-base", "--sim-base", dest="sim_base", help="Override evaluator/sim base URL")
    engine.add_argument("--allow-actuation", action="store_true")
    engine.add_argument("--allow-sim-reset", action="store_true")

    simhost = sub.add_parser(
        "serve-sim-host",
        help="Serve a local simulator driver (e.g., mujoco_cafe) as a standalone HTTP host for --driver http / ticos-edge",
    )
    simhost.add_argument("--config", required=True, help="Path to SceneConfig JSON file")
    simhost.add_argument("--driver", default="mujoco_cafe", help="Underlying sim driver (mujoco_cafe or fixture)")
    simhost.add_argument("--host", default="127.0.0.1", help="Bind host (default 127.0.0.1)")
    simhost.add_argument("--port", type=int, default=8880, help="Bind port (default 8880)")
    simhost.add_argument("--realtime-pacing", action="store_true", help="Enable smooth real-time arm motion pacing")

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
        cfg = _override_config(
            SceneConfig.from_file(args.config),
            edge_base=args.edge_base,
            scope="fixture" if args.driver == "fixture" else None,
        )
        drv = create_host_driver(args.driver, cfg)
        out = {
            "driver": drv.driver_name,
            "capability_check": drv.discover_capabilities().to_dict(),
            "scene_observation": drv.observe_scene().to_dict(),
        }
        if args.check_oven_gap:
            from .coffee_loop import OVEN_REQUIRED_METHODS
            out["oven_gap"] = drv.check_missing_capabilities(OVEN_REQUIRED_METHODS)
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
        return

    if args.command == "benchmark":
        cfg = _override_config(
            SceneConfig.from_file(args.config),
            edge_base=args.edge_base,
            sim_base=args.sim_base,
            allow_actuation=args.allow_actuation,
            allow_sim_reset=args.allow_sim_reset,
            scope="fixture" if args.driver == "fixture" else None,
        )
        driver = create_host_driver(args.driver, cfg)
        policies = _parse_policy_specs(args.policy, include_builtin_ref=not args.no_builtin_ref)
        if not policies:
            raise SystemExit("At least one policy is required for benchmark")

        def _log_trial(row: dict) -> None:
            print(
                json.dumps(
                    {
                        "repeat": row["repeat"],
                        "policy": row["policy_id"],
                        "scenario": row["scenario_id"],
                        "passed": row["task_passed"],
                        "status": row["host_runtime_status"],
                        "wall_s": row["wall_s"],
                        "err_mm": (row.get("oracle_post_run") or {}).get("delivery_error_mm"),
                    }
                ),
                flush=True,
            )

        report = run_paired_benchmark(
            driver,
            policies,
            default_coffee_benchmark_suite(),
            args.output,
            repeats=args.repeats,
            on_trial_complete=_log_trial,
        )
        print(json.dumps({"status": report["status"], "summary_by_policy": report["summary_by_policy"]}, indent=2))
        if report.get("status") != "completed":
            raise SystemExit(1)
        return

    if args.command == "cafebench":
        cfg = _override_config(
            SceneConfig.from_file(args.config),
            edge_base=args.edge_base,
            sim_base=args.sim_base,
            allow_actuation=args.allow_actuation,
            allow_sim_reset=args.allow_sim_reset,
            scope="fixture" if args.driver == "fixture" else None,
        )
        driver = create_host_driver(args.driver, cfg)
        policies = _parse_policy_specs(args.policy, include_builtin_ref=not args.no_builtin_ref)
        if not policies:
            raise SystemExit("At least one policy is required for cafebench")

        def _log_cb(row: dict) -> None:
            print(
                json.dumps(
                    {
                        "repeat": row["repeat"],
                        "policy": row["policy_id"],
                        "scenario": row["scenario_id"],
                        "tier": row["category"],
                        "passed": row["task_passed"],
                        "status": row["host_runtime_status"],
                        "wall_s": row["wall_s"],
                        "err_mm": (row.get("oracle_post_run") or {}).get("delivery_error_mm"),
                    }
                ),
                flush=True,
            )

        report = run_cafebench_v1(
            driver,
            policies,
            args.output,
            repeats=args.repeats,
            resume=not args.no_resume,
            on_trial_complete=_log_cb,
        )
        print(json.dumps({"status": report["status"], "summary_by_policy": report["summary_by_policy"]}, indent=2))
        if report.get("status") != "completed":
            raise SystemExit(1)
        return

    if args.command == "prior-work-bench":
        from .prior_work_bench import make_live_llm_fn, run_prior_work_comparison

        cfg = _override_config(
            SceneConfig.from_file(args.config),
            edge_base=args.edge_base,
            sim_base=args.sim_base,
            provider=args.provider,
            model=args.model,
            effort=args.effort,
            allow_actuation=args.allow_actuation,
            allow_sim_reset=args.allow_sim_reset,
            scope="fixture" if args.driver == "fixture" else None,
        )
        driver = create_host_driver(args.driver, cfg)
        llm_fn = make_live_llm_fn(
            provider=args.provider,
            model=args.model,
            reasoning_effort=args.effort,
        )
        report = run_prior_work_comparison(
            driver,
            llm_fn=llm_fn,
            output_dir=args.output,
            model=args.model,
            reasoning_effort=args.effort,
            max_dev_llm_calls=args.max_dev_calls,
            max_online_regen=args.max_online_regen,
            progress_cb=lambda msg: print(msg, flush=True),
        )
        print(
            json.dumps(
                {
                    "benchmark_version": report["benchmark_version"],
                    "summary_by_method": report["summary_by_method"],
                    "paired_mcnemar_craft_vs": report["paired_mcnemar_craft_vs"],
                },
                indent=2,
            )
        )
        return

    if args.command == "robosuite-bench":
        from .prior_work_bench import make_live_llm_fn
        from .robosuite_bench import run_robosuite_parity_benchmark

        tasks = tuple(x.strip() for x in args.tasks.split(",") if x.strip())
        dev_seeds = tuple(int(x.strip()) for x in args.dev_seeds.split(",") if x.strip())
        if "-" in args.eval_seeds and "," not in args.eval_seeds:
            lo, hi = args.eval_seeds.split("-", 1)
            eval_seeds = tuple(range(int(lo), int(hi) + 1))
        else:
            eval_seeds = tuple(int(x.strip()) for x in args.eval_seeds.split(",") if x.strip())

        llm_fn = make_live_llm_fn(
            provider=args.provider,
            model=args.model,
            reasoning_effort=args.effort,
        )
        report = run_robosuite_parity_benchmark(
            llm_fn=llm_fn,
            output_dir=args.output,
            tasks=tasks,
            dev_seeds=dev_seeds,
            eval_seeds=eval_seeds,
            model=args.model,
            reasoning_effort=args.effort,
            max_dev_llm_calls=args.max_dev_calls,
            enable_capx_online_regen=not args.no_online_regen,
            progress_cb=lambda msg: print(msg, flush=True),
        )
        print(
            json.dumps(
                {
                    "benchmark_version": report["benchmark_version"],
                    "summary_by_method": report["summary_by_method"],
                    "paired_mcnemar_craft_vs": report["paired_mcnemar_craft_vs"],
                },
                indent=2,
            )
        )
        return

    if args.command == "studio":
        cfg = _override_config(
            SceneConfig.from_file(args.config),
            edge_base=args.edge_base,
            sim_base=args.sim_base,
            allow_actuation=args.allow_actuation,
            allow_sim_reset=args.allow_sim_reset,
            scope="fixture" if args.driver == "fixture" else None,
        )
        driver = create_host_driver(args.driver, cfg)
        if getattr(driver, "driver_name", "") == "mujoco_cafe":
            driver._realtime_pacing = True  # smooth visible arm motion in live Studio video
        policies = _parse_policy_specs(args.policy, include_builtin_ref=False)
        app = CraftStudioApp(cfg, driver, args.workspace, policies=policies)
        server = create_studio_server(app, host=args.host, port=args.port)
        print(
            f"CRAFT Studio listening on http://{args.host}:{args.port} "
            f"(driver={driver.driver_name}, evidence_source={driver.evidence_source})",
            flush=True,
        )
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            server.shutdown()
        return

    if args.command == "serve-engine":
        cfg = _override_config(
            SceneConfig.from_file(args.config),
            edge_base=args.edge_base,
            sim_base=args.sim_base,
            allow_actuation=args.allow_actuation,
            allow_sim_reset=args.allow_sim_reset,
            scope="fixture" if args.driver == "fixture" else ("recorded_data" if args.driver == "recorded" else None),
        )
        driver = create_host_driver(args.driver, cfg)
        service = CraftEngineService(cfg, driver, args.workspace)
        server = create_engine_server(service, host=args.host, port=args.port)
        print(
            f"CRAFT Engine API (/v1) listening on http://{args.host}:{args.port} "
            f"(driver={driver.driver_name}, evidence_source={driver.evidence_source})",
            flush=True,
        )
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            server.shutdown()
        return

    if args.command == "serve-sim-host":
        from .sim_host_server import StandaloneSimHostApp, create_sim_host_server

        cfg = _override_config(
            SceneConfig.from_file(args.config),
            allow_actuation=True,
            allow_sim_reset=True,
            scope="fixture" if args.driver == "fixture" else "simulator",
        )
        driver = create_host_driver(args.driver, cfg)
        if args.realtime_pacing and hasattr(driver, "_realtime_pacing"):
            driver._realtime_pacing = True
        app = StandaloneSimHostApp(driver)
        server = create_sim_host_server(app, host=args.host, port=args.port)
        print(
            f"CRAFT Standalone Sim Host listening on http://{args.host}:{args.port} "
            f"(driver={driver.driver_name}, evidence_source={driver.evidence_source})",
            flush=True,
        )
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            server.shutdown()


if __name__ == "__main__":
    main()
