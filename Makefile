PYTHON ?= .venv/bin/python
TECTONIC ?= $(shell command -v tectonic 2>/dev/null || echo $(HOME)/.local/bin/tectonic)

.PHONY: test check-stdlib check-arch self-audit demo artifacts tables figures paper check-site

test:
	$(PYTHON) -m pytest -q

check-stdlib:
	$(PYTHON) -c "import ast, pathlib, sys; \
allowed_opt = {'mujoco_cafe_driver.py', 'mujoco_so101_driver.py'}; \
stdlib = set(sys.stdlib_module_names) | {'trax', '__future__'}; \
bad = []; \
[bad.append((p.name, mod)) for p in sorted(pathlib.Path('src/trax').glob('*.py')) if p.name not in allowed_opt \
 for node in ast.walk(ast.parse(p.read_text())) \
 for mod in ([a.name.split('.')[0] for a in node.names] if isinstance(node, ast.Import) else ([node.module.split('.')[0]] if isinstance(node, ast.ImportFrom) and node.module and node.level == 0 else [])) \
 if mod not in stdlib and not (p.name == 'cli.py' and mod == 'benchmarks')]; \
assert not bad, f'Non-stdlib imports in core: {bad}'; \
print('Core stdlib invariant OK (optional sim drivers:', sorted(allowed_opt), ')')"

check-arch:
	$(PYTHON) tools/check_arch.py

self-audit:
	$(PYTHON) tools/self_audit.py

demo:
	$(PYTHON) -m trax demo --task combo --lost-receipt

artifacts:
	$(PYTHON) tools/generate_paper_artifacts.py

tables: artifacts

figures: artifacts

paper: artifacts
	cd papers/trax && $(TECTONIC) main.tex
	mv papers/trax/main.pdf papers/trax/trax.pdf
	mkdir -p docs/assets/papers
	cp papers/trax/trax.pdf docs/assets/papers/trax.pdf

check-site:
	$(PYTHON) tools/generate_paper_artifacts.py --verify-site
