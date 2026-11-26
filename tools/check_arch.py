#!/usr/bin/env python3
"""Static AST & lexical architecture boundary checker for TRAX (`make check-arch`).

Enforces production layer invariants on every CI run:
  1. Core pure-stdlib invariant (optional MuJoCo drivers explicitly allowlisted)
  2. Strict one-way UI isolation (`src/trax/*.py` never imports `trax.ui`)
  3. Clean UI boundary (`src/trax/ui/` never imports private `_symbols`, `coffee_loop`, or `benchmarks`)
  4. Embodiment- & domain-agnostic core (`mcp_protocol`, `plugin`, `sandbox`, `attribution`,
     `agent_engine`, `knowledge_base` contain zero hardcoded robot model or coffee terms
     and zero benchmark imports)
"""
from __future__ import annotations

import ast
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
TRAX_SRC = REPO_ROOT / "src" / "trax"
TRAX_UI = TRAX_SRC / "ui"

OPTIONAL_SIM_FILES = frozenset({"mujoco_cafe_driver.py", "mujoco_so101_driver.py"})
STDLIB_PLUS_LOCAL = set(sys.stdlib_module_names) | {"trax", "__future__"}

AGNOSTIC_CORE_FILES = (
    "plugin.py",
    "mcp_protocol.py",
    "spatial_contract.py",
    "sandbox.py",
    "attribution.py",
    "agent_engine.py",
    "knowledge_base.py",
    "llm.py",
    "self_audit.py",
)

FORBIDDEN_DOMAIN_TOKENS = (
    "so101",
    "shoulder_pan",
    "start_coffee_machine",
    "place_cup_under_machine",
    "recenter_cup_under_spout",
    "espresso_machine",
    "OVEN_REQUIRED_METHODS",
)


def _iter_imports(tree: ast.AST) -> list[tuple[int, str, list[str]]]:
    rows: list[tuple[int, str, list[str]]] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                rows.append((getattr(node, "lineno", 0), alias.name, []))
        elif isinstance(node, ast.ImportFrom):
            mod = node.module or ""
            names = [a.name for a in node.names]
            prefix = "." * node.level + mod
            rows.append((getattr(node, "lineno", 0), prefix, names))
    return rows


def main() -> int:
    errors: list[str] = []

    # 1 & 2: Check all files in src/trax/*.py
    for path in sorted(TRAX_SRC.glob("*.py")):
        src = path.read_text(encoding="utf-8")
        tree = ast.parse(src, filename=str(path))
        imports = _iter_imports(tree)

        for lineno, mod, names in imports:
            top = mod.lstrip(".").split(".")[0] if mod.lstrip(".") else ""
            if path.name not in OPTIONAL_SIM_FILES and not mod.startswith("."):
                if top and top not in STDLIB_PLUS_LOCAL:
                    # Allow lazy benchmark imports ONLY inside cli.py
                    if not (path.name == "cli.py" and top == "benchmarks"):
                        errors.append(f"[stdlib] {path.name}:{lineno} imports non-stdlib module `{mod}`")
            if mod in ("trax.ui", ".ui", "trax_ui") or mod.startswith(("trax.ui.", ".ui.", "trax_ui.")):
                errors.append(f"[ui-isolation] {path.name}:{lineno} core module must not import UI (`{mod}`)")

    # 3: Check src/trax/ui/*.py
    if TRAX_UI.is_dir():
        for path in sorted(TRAX_UI.glob("*.py")):
            src = path.read_text(encoding="utf-8")
            tree = ast.parse(src, filename=str(path))
            for lineno, mod, names in _iter_imports(tree):
                top = mod.lstrip(".").split(".")[0] if mod.lstrip(".") else ""
                if not mod.startswith(".") and top and top not in STDLIB_PLUS_LOCAL:
                    errors.append(f"[ui-stdlib] ui/{path.name}:{lineno} imports non-stdlib `{mod}`")
                if any(bad in mod for bad in ("coffee_loop", "evolution_lab", "benchmarks")):
                    errors.append(f"[ui-boundary] ui/{path.name}:{lineno} forbidden import `{mod}`")
                if mod.startswith("trax."):
                    for sym in names:
                        if sym.startswith("_"):
                            errors.append(
                                f"[ui-private] ui/{path.name}:{lineno} imports private core symbol `{mod}.{sym}`"
                            )

    # 4: Embodiment & domain purity of core protocol / engine / knowledge modules
    for fname in AGNOSTIC_CORE_FILES:
        path = TRAX_SRC / fname
        if not path.is_file():
            errors.append(f"[missing-core] expected core module {fname}")
            continue
        src = path.read_text(encoding="utf-8")
        tree = ast.parse(src, filename=str(path))
        for lineno, mod, _ in _iter_imports(tree):
            if "benchmarks" in mod or "prior_work_bench" in mod:
                errors.append(f"[core-bench] {fname}:{lineno} core module imports benchmark `{mod}`")
        lower = src.lower()
        for tok in FORBIDDEN_DOMAIN_TOKENS:
            if tok.lower() in lower:
                errors.append(f"[core-purity] {fname} contains hardcoded robot/domain token `{tok}`")

    if errors:
        print("TRAX architecture boundary check FAILED:", file=sys.stderr)
        for err in errors:
            print(f"  - {err}", file=sys.stderr)
        return 1

    print(
        f"TRAX architecture check OK: stdlib core ({len(list(TRAX_SRC.glob('*.py')))} files), "
        f"UI one-way isolation, zero hardcoded robot/domain tokens in core."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
