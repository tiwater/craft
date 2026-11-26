"""Standalone CLI entry point (`trax-ui`) for the optional TRAX web research console."""
from __future__ import annotations

import argparse
from typing import Sequence

from trax.cli import override_scene_config, parse_policy_specs
from trax.config import SceneConfig
from trax.host_driver import create_host_driver

from .server import TraxUIApp, create_ui_server


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        prog="trax-ui",
        description="Launch optional standalone TRAX Web Research Console & MCP Inspector",
    )
    parser.add_argument("--config", required=True, help="Path to SceneConfig JSON file")
    parser.add_argument("--driver", default="fixture", help="Host backend plugin name (fixture, http, mujoco_so101, or trax.backends plugin)")
    parser.add_argument("--workspace", required=True, help="Four-zone workspace dir (knowledge/runtime/traces/recordings)")
    parser.add_argument("--host", default="127.0.0.1", help="Bind host (default 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8860, help="Bind port (default 8860)")
    parser.add_argument("--policy", action="append", default=[], help="Extra policy as label=/path/to/policy.py")
    parser.add_argument("--host-base", "--edge-base", dest="edge_base", help="Override host base URL (for http/plugin drivers)")
    parser.add_argument("--evaluator-base", "--sim-base", dest="sim_base", help="Override evaluator/sim base URL")
    parser.add_argument("--mcp-url", help="Connect directly to an external Streamable HTTP MCP robot endpoint (zero-code unknown robot)")
    parser.add_argument("--mcp-stdio", help="Connect directly to an external persistent stdio MCP robot command")
    parser.add_argument("--allow-actuation", action="store_true")
    parser.add_argument("--allow-sim-reset", action="store_true")

    args = parser.parse_args(argv)
    cfg = override_scene_config(
        SceneConfig.from_file(args.config),
        edge_base=args.edge_base,
        sim_base=args.sim_base,
        allow_actuation=args.allow_actuation,
        allow_sim_reset=args.allow_sim_reset,
        scope="fixture" if args.driver == "fixture" else None,
    )
    driver = create_host_driver(args.driver, cfg)
    if hasattr(driver, "set_realtime_pacing"):
        driver.set_realtime_pacing(True)  # type: ignore[attr-defined]
    policies = parse_policy_specs(args.policy, include_builtin_ref=False)
    app = TraxUIApp(
        cfg,
        driver,
        args.workspace,
        policies=policies,
        default_mcp_url=args.mcp_url,
        default_mcp_stdio=args.mcp_stdio,
    )
    server = create_ui_server(app, host=args.host, port=args.port)
    print(
        f"TRAX UI listening on http://{args.host}:{args.port} "
        f"(driver={driver.driver_name}, evidence_source={driver.evidence_source})",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.shutdown()
    finally:
        app.close()


if __name__ == "__main__":
    main()
