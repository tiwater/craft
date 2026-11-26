"""Git-versioned Executable Knowledge Base (`workspace/knowledge/`) and Audit Trace Store (`workspace/traces/`).

Pure Python stdlib (zero external dependencies). Implements TRAX's governed four-zone workspace:
  1. `knowledge/`   — Independent Git repository for versioned robot knowledge:
                      * `FACTS.md`      (Runtime-owned objective physical & MCP facts; LLM read-only)
                      * `FORBIDDEN.md`  (4-Layer attribution negative constraints; deduplicated)
                      * `probes/`       (Executed diagnostic exploration Python scripts)
                      * `policies/`     (Compiled & repaired task Python policies)
                      * `skills/<emb>/` (Contract-bound, evidence-tiered verified `SKILL.md` + `skill.py`)
  2. `runtime/`     — ACID SQLite WAL transactional ledgers (`agent_journal.sqlite`, `mcp_journal.sqlite`)
  3. `traces/`      — Append-only JSONL audit receipts (`llm_calls.jsonl`, `mcp_wire.jsonl`)
  4. `recordings/`  — Raw telemetry & verification snapshots (`<run_id>.json`)

Git Governance Invariant:
  All exploration and repair iterations commit on isolated `session/<embodiment>/<task>-<ts>` branches.
  Only policies passing AST preflight + independent `verify_task=PASS` + zero non-idempotent violations
  are promoted via `--no-ff` merge onto protected `main` with an immutable `verified/...` Git tag.
"""
from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import re
import subprocess
import threading
from typing import Any, Mapping, Sequence

from .contracts import digest


def _utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _utc_compact() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")


def _slug(text: str) -> str:
    cleaned = re.sub(r"[^a-zA-Z0-9._-]+", "-", str(text or "default").strip()).strip("-").lower()
    return cleaned[:48] or "default"


class AuditTraceStore:
    """Append-only JSONL audit store under `workspace/traces/` and `workspace/recordings/`."""

    def __init__(self, workspace_dir: Path) -> None:
        self.workspace_dir = Path(workspace_dir)
        self.traces_dir = self.workspace_dir / "traces"
        self.recordings_dir = self.workspace_dir / "recordings"
        self.runtime_dir = self.workspace_dir / "runtime"
        for d in (self.traces_dir, self.recordings_dir, self.runtime_dir):
            d.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()

    @property
    def llm_log_path(self) -> Path:
        return self.traces_dir / "llm_calls.jsonl"

    @property
    def mcp_wire_path(self) -> Path:
        return self.traces_dir / "mcp_wire.jsonl"

    def append_llm_call(self, record: Mapping[str, Any]) -> None:
        with self._lock:
            with self.llm_log_path.open("a", encoding="utf-8") as fh:
                fh.write(json.dumps({"recorded_utc": _utc_iso(), **dict(record)}, ensure_ascii=False) + "\n")

    def append_mcp_wire(self, record: Mapping[str, Any]) -> None:
        with self._lock:
            with self.mcp_wire_path.open("a", encoding="utf-8") as fh:
                fh.write(json.dumps({"recorded_utc": _utc_iso(), **dict(record)}, ensure_ascii=False) + "\n")

    def save_recording(self, run_id: str, payload: Mapping[str, Any]) -> Path:
        out = self.recordings_dir / f"{_slug(run_id)}.json"
        with self._lock:
            out.write_text(json.dumps({"recorded_utc": _utc_iso(), **dict(payload)}, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        return out

    def stats(self) -> dict[str, Any]:
        def _count_lines(p: Path) -> int:
            if not p.is_file():
                return 0
            return sum(1 for _ in p.open("r", encoding="utf-8", errors="ignore"))

        recs = list(self.recordings_dir.glob("*.json")) if self.recordings_dir.is_dir() else []
        sqlites = [p.name for p in sorted(self.runtime_dir.glob("*.sqlite"))] if self.runtime_dir.is_dir() else []
        return {
            "runtime_dir": str(self.runtime_dir.name),
            "sqlite_ledgers": sqlites,
            "llm_calls_logged": _count_lines(self.llm_log_path),
            "mcp_wire_logged": _count_lines(self.mcp_wire_path),
            "recordings_count": len(recs),
        }


class ExecutableKnowledgeBase:
    """Git-versioned Executable Robot Knowledge Base under `workspace/knowledge/`."""

    def __init__(self, workspace_dir: Path) -> None:
        self.workspace_dir = Path(workspace_dir)
        self.repo_dir = self.workspace_dir / "knowledge"
        self.repo_dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()

        self._facts_state_path = self.repo_dir / ".trax_facts_index.json"
        self._forbidden_state_path = self.repo_dir / ".trax_forbidden_index.json"
        self._facts: dict[str, dict[str, Any]] = {}
        self._forbidden: dict[str, dict[str, Any]] = {}
        self._active_branch: str = "main"

        self._ensure_git_repo()
        self._load_indexes()

    def _git(self, *args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
        proc = subprocess.run(
            ["git", *args],
            cwd=str(self.repo_dir),
            capture_output=True,
            text=True,
            check=False,
        )
        if check and proc.returncode != 0:
            raise RuntimeError(f"git {' '.join(args)} failed: {proc.stderr.strip() or proc.stdout.strip()}")
        return proc

    def _ensure_git_repo(self) -> None:
        with self._lock:
            git_dir = self.repo_dir / ".git"
            if not git_dir.is_dir():
                init = self._git("init", "-b", "main", check=False)
                if init.returncode != 0:
                    self._git("init")
                    self._git("checkout", "-B", "main")
                self._git("config", "user.name", "TRAX Runtime")
                self._git("config", "user.email", "trax-runtime@tiwater.local")
                self._git("config", "commit.gpgsign", "false")
                for sub in ("probes", "policies", "skills"):
                    (self.repo_dir / sub).mkdir(parents=True, exist_ok=True)
                self._write_facts_md()
                self._write_forbidden_md()
                (self.repo_dir / "README.md").write_text(
                    "# TRAX Executable Knowledge Base\n\n"
                    "- `FACTS.md`: Runtime-owned objective physical & MCP contract invariants (LLM read-only).\n"
                    "- `FORBIDDEN.md`: 4-Layer attribution deduplicated negative constraints.\n"
                    "- `probes/`: Executed diagnostic exploration scripts.\n"
                    "- `policies/`: Compiled task Python policies.\n"
                    "- `skills/<embodiment>/`: Contract-bound, evidence-tiered verified skills promoted to `main`.\n",
                    encoding="utf-8",
                )
                self._git("add", "-A")
                self._git("commit", "-m", "chore(kb): initialize TRAX executable knowledge base")
            else:
                br = self._git("rev-parse", "--abbrev-ref", "HEAD", check=False)
                if br.returncode == 0 and br.stdout.strip():
                    self._active_branch = br.stdout.strip()

    def _load_indexes(self) -> None:
        if self._facts_state_path.is_file():
            try:
                self._facts = json.loads(self._facts_state_path.read_text(encoding="utf-8"))
            except ValueError:
                self._facts = {}
        if self._forbidden_state_path.is_file():
            try:
                self._forbidden = json.loads(self._forbidden_state_path.read_text(encoding="utf-8"))
            except ValueError:
                self._forbidden = {}

    def _write_facts_md(self) -> None:
        lines = [
            "# TRAX Verified Physical & Interface Facts (`FACTS.md`)",
            "",
            "> **Ownership:** Runtime-written exclusively from live MCP telemetry and backend safeguards (read-only to LLM).",
            f"> **Updated:** {_utc_iso()}",
            "",
        ]
        if not self._facts:
            lines.append("_(No live embodiment facts recorded yet. Run Stage 1 Exploration.)_")
        else:
            for emb_id, rec in sorted(self._facts.items()):
                lines.append(f"## Embodiment: `{emb_id}`")
                lines.append(f"- **Evidence tier:** `{rec.get('evidence_source', 'simulator')}`")
                lines.append(f"- **MCP tools contract (`sha256`):** `{rec.get('mcp_tools_sha256', '')}` ({rec.get('tools_count', 0)} tools)")
                if rec.get("non_idempotent_tools"):
                    lines.append(f"- **Non-idempotent / journaled tools:** `{', '.join(rec['non_idempotent_tools'])}`")
                if rec.get("workspace_bounds_m"):
                    lines.append(f"- **Workspace bounds (m):** `{json.dumps(rec['workspace_bounds_m'], sort_keys=True)}`")
                if rec.get("zones"):
                    lines.append(f"- **Observed tabletop zones:** `{json.dumps(rec['zones'], sort_keys=True)}`")
                if rec.get("measured_observations"):
                    lines.append("- **Measured physical observations:**")
                    for item in rec["measured_observations"][-8:]:
                        lines.append(f"  - {item}")
                lines.append("")
        (self.repo_dir / "FACTS.md").write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")
        self._facts_state_path.write_text(json.dumps(self._facts, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def _write_forbidden_md(self) -> None:
        lines = [
            "# TRAX Negative Constraints & Failure Boundaries (`FORBIDDEN.md`)",
            "",
            "> **Ownership:** Written exclusively by TRAX 4-Layer Attribution (deduplicated by failure class).",
            f"> **Updated:** {_utc_iso()}",
            "",
        ]
        if not self._forbidden:
            lines.append("_(No negative constraints recorded yet.)_")
        else:
            for key, item in sorted(self._forbidden.items()):
                lines.append(
                    f"- **`[{item['layer']}/{item['code']}]`** (embodiment=`{item['embodiment_id']}`, "
                    f"occurrences={item['count']}, evidence=`{item['evidence_source']}`): {item['summary']}"
                )
        (self.repo_dir / "FORBIDDEN.md").write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")
        self._forbidden_state_path.write_text(json.dumps(self._forbidden, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def current_branch(self) -> str:
        with self._lock:
            res = self._git("rev-parse", "--abbrev-ref", "HEAD", check=False)
            if res.returncode == 0 and res.stdout.strip():
                self._active_branch = res.stdout.strip()
            return self._active_branch

    def begin_session_branch(self, embodiment_id: str, task_id: str) -> str:
        """Create and switch to an isolated `session/<embodiment>/<task>-<ts>` branch off `main`."""
        with self._lock:
            # Guard against dirty working tree before switching branches
            dirty = self._git("status", "--porcelain", check=False).stdout.strip()
            if dirty:
                self._git("add", "-A")
                self._git("commit", "-m", "chore(kb): auto-save uncommitted session files before branch switch", check=False)
            self._git("checkout", "main")
            branch = f"session/{_slug(embodiment_id)}/{_slug(task_id)}-{_utc_compact()}"
            # Ensure unique branch name if two sessions start within the same second
            if self._git("show-ref", "--verify", "--quiet", f"refs/heads/{branch}", check=False).returncode == 0:
                branch = f"{branch}-{digest(f'{branch}-{_utc_iso()}')[:4]}"
            self._git("checkout", "-b", branch)
            self._active_branch = branch
            return branch

    def commit_if_dirty(self, subject: str, *, trailers: Mapping[str, str] | None = None) -> str | None:
        with self._lock:
            self._git("add", "-A")
            st = self._git("status", "--porcelain")
            if not st.stdout.strip():
                return None
            msg_lines = [subject]
            if trailers:
                msg_lines.append("")
                for k, v in trailers.items():
                    msg_lines.append(f"{k}: {v}")
            self._git("commit", "-m", "\n".join(msg_lines))
            sha = self._git("rev-parse", "--short", "HEAD").stdout.strip()
            return sha

    def record_discovery_and_probe(
        self,
        *,
        embodiment_id: str,
        evidence_source: str,
        mcp_tools_sha256: str,
        tools: Sequence[Mapping[str, Any]],
        task_id: str,
        round_idx: int,
        probe_source: str,
        finding: Mapping[str, Any],
        obs_snapshot: Mapping[str, Any],
    ) -> str | None:
        """Runtime-write objective physical facts into `FACTS.md` + save probe script + commit on session branch."""
        with self._lock:
            emb = _slug(embodiment_id)
            rec = self._facts.setdefault(
                emb,
                {
                    "embodiment_id": emb,
                    "evidence_source": evidence_source,
                    "mcp_tools_sha256": mcp_tools_sha256[:16],
                    "tools_count": len(tools),
                    "non_idempotent_tools": [],
                    "workspace_bounds_m": {},
                    "zones": {},
                    "measured_observations": [],
                },
            )
            rec["evidence_source"] = evidence_source
            rec["mcp_tools_sha256"] = mcp_tools_sha256[:16]
            rec["tools_count"] = len(tools)
            rec["non_idempotent_tools"] = sorted(
                str(t["name"])
                for t in tools
                if ((t.get("annotations") or {}).get("idempotentHint") is False)
                or bool((t.get("annotations") or {}).get("destructiveHint"))
            )
            robot = obs_snapshot.get("get_robot_state") or {}
            if isinstance(robot, Mapping) and robot.get("workspace_bounds_m"):
                rec["workspace_bounds_m"] = dict(robot["workspace_bounds_m"])
            scene = obs_snapshot.get("get_scene_objects") or {}
            if isinstance(scene, Mapping) and scene.get("zones"):
                rec["zones"] = dict(scene["zones"])

            obs_line = (
                f"[{task_id} R{round_idx}] calls={len(finding.get('methods_called') or [])}, "
                f"wall={finding.get('wall_s')}s, refusals={len(finding.get('limit_refusals') or [])}"
            )
            if finding.get("return_value"):
                rv = json.dumps(finding["return_value"], ensure_ascii=False)
                obs_line += f", measured={rv[:160]}"
            if obs_line not in rec["measured_observations"]:
                rec["measured_observations"].append(obs_line)
                rec["measured_observations"] = rec["measured_observations"][-12:]

            for ref in finding.get("limit_refusals") or []:
                self._add_forbidden_unlocked(
                    embodiment_id=emb,
                    evidence_source=evidence_source,
                    layer="control",
                    code="backend_limit_refusal",
                    summary=f"{ref.get('tool')}: {ref.get('violated_limit')}",
                )

            probes_dir = self.repo_dir / "probes"
            probes_dir.mkdir(parents=True, exist_ok=True)
            (probes_dir / f"{_slug(task_id)}_r{round_idx}.py").write_text(probe_source.rstrip() + "\n", encoding="utf-8")
            self._write_facts_md()
            self._write_forbidden_md()
            return self.commit_if_dirty(
                f"explore({task_id}/r{round_idx}): record probe & measured facts on {emb}",
                trailers={
                    "TRAX-Stage": f"explore_r{round_idx}",
                    "TRAX-Embodiment": emb,
                    "TRAX-MCP-Contract": mcp_tools_sha256[:16],
                    "TRAX-Evidence": evidence_source,
                },
            )

    def _add_forbidden_unlocked(
        self,
        *,
        embodiment_id: str,
        evidence_source: str,
        layer: str,
        code: str,
        summary: str,
    ) -> None:
        key = f"{_slug(embodiment_id)}|{layer}|{code}|{summary[:90]}"
        entry = self._forbidden.get(key)
        if entry is None:
            self._forbidden[key] = {
                "embodiment_id": _slug(embodiment_id),
                "evidence_source": evidence_source,
                "layer": layer,
                "code": code,
                "summary": summary[:240],
                "count": 1,
                "last_utc": _utc_iso(),
            }
        else:
            entry["count"] = int(entry.get("count", 1)) + 1
            entry["last_utc"] = _utc_iso()

    def record_policy_iteration(
        self,
        *,
        embodiment_id: str,
        evidence_source: str,
        mcp_tools_sha256: str,
        task_id: str,
        iter_index: int,
        origin: str,
        source: str,
        attribution: Mapping[str, Any] | None,
        passed: bool,
    ) -> str | None:
        """Write compiled/repaired policy to `policies/<task_id>.py`, update `FORBIDDEN.md` on failure, and commit."""
        with self._lock:
            emb = _slug(embodiment_id)
            policies_dir = self.repo_dir / "policies"
            policies_dir.mkdir(parents=True, exist_ok=True)
            (policies_dir / f"{_slug(task_id)}.py").write_text(source.rstrip() + "\n", encoding="utf-8")

            if not passed and attribution and attribution.get("layer") not in (None, "none"):
                self._add_forbidden_unlocked(
                    embodiment_id=emb,
                    evidence_source=evidence_source,
                    layer=str(attribution.get("layer") or "policy"),
                    code=str(attribution.get("code") or "failed"),
                    summary=str(attribution.get("summary") or "execution failed"),
                )
                self._write_forbidden_md()

            verdict = "PASS" if passed else f"FAIL({(attribution or {}).get('code', 'pending')})"
            return self.commit_if_dirty(
                f"policy({task_id}/it{iter_index}): {origin} -> {verdict}",
                trailers={
                    "TRAX-Stage": origin,
                    "TRAX-Iter": str(iter_index),
                    "TRAX-Verdict": verdict,
                    "TRAX-Embodiment": emb,
                    "TRAX-MCP-Contract": mcp_tools_sha256[:16],
                    "TRAX-Evidence": evidence_source,
                },
            )

    def promote_verified_skill_to_main(
        self,
        *,
        embodiment_id: str,
        evidence_source: str,
        mcp_tools_sha256: str,
        task_id: str,
        task_instruction: str,
        policy_source: str,
        verify_result: Mapping[str, Any],
        urdf_sha256: str = "",
        verified_seeds: Sequence[int] = (0,),
        min_verified_seeds: int = 1,
    ) -> dict[str, Any]:
        """Stage a verified skill and merge `--no-ff` to `main` ONLY when the seed verification gate passes."""
        if not bool(verify_result.get("passed")):
            raise ValueError("Cannot promote unverified policy to main: independent verify_task did not pass.")

        unique_seeds = sorted({int(s) for s in verified_seeds})
        req_seeds = max(1, int(min_verified_seeds))
        meets_main_gate = len(unique_seeds) >= req_seeds

        with self._lock:
            emb = _slug(embodiment_id)
            sk_name = _slug(task_id)
            sk_dir = self.repo_dir / "skills" / emb / sk_name
            sk_dir.mkdir(parents=True, exist_ok=True)

            code_sha = digest(policy_source)[:16]
            urdf_short = (urdf_sha256 or "")[:16]
            meta = {
                "skill_name": sk_name,
                "embodiment_id": emb,
                "mcp_tools_sha256": mcp_tools_sha256[:16],
                "urdf_sha256": urdf_short,
                "evidence_source": evidence_source,
                "code_sha256": code_sha,
                "verified_utc": _utc_iso(),
                "verified_seeds": unique_seeds,
                "promotion_stage": "verified_main" if meets_main_gate else "candidate",
                "verify_metrics": verify_result.get("metrics") or {},
                "verify_detail": str(verify_result.get("detail") or ""),
            }
            skill_md = "\n".join(
                [
                    "---",
                    f"skill_name: {meta['skill_name']}",
                    f"embodiment_id: {meta['embodiment_id']}",
                    f"mcp_tools_sha256: {meta['mcp_tools_sha256']}",
                    *([f"urdf_sha256: {urdf_short}"] if urdf_short else []),
                    f"evidence_source: {meta['evidence_source']}",
                    f"code_sha256: {meta['code_sha256']}",
                    f"verified_seeds: {json.dumps(unique_seeds)}",
                    f"promotion_stage: {meta['promotion_stage']}",
                    f"verified_utc: {meta['verified_utc']}",
                    "---",
                    "",
                    f"# Verified Skill: `{sk_name}` (`{emb}`)",
                    "",
                    f"- **Task Scope:** {task_instruction}",
                    f"- **Independent Verification:** `{meta['verify_detail']}`",
                    f"- **Verified Seeds:** `{unique_seeds}` (required >= {req_seeds})",
                    f"- **Metrics:** `{json.dumps(meta['verify_metrics'], sort_keys=True)}`",
                    f"- **Contract Binding:** Valid only while MCP `tools/list` digest equals `{meta['mcp_tools_sha256']}`"
                    + (f" and URDF digest equals `{urdf_short}`." if urdf_short else "."),
                    "",
                ]
            )
            (sk_dir / "SKILL.md").write_text(skill_md, encoding="utf-8")
            (sk_dir / "skill.py").write_text(policy_source.rstrip() + "\n", encoding="utf-8")
            (sk_dir / "meta.json").write_text(json.dumps(meta, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            self._rebuild_skill_index_unlocked(emb)

            cur_branch = self.current_branch()
            stage_word = "promote verified executable skill" if meets_main_gate else "stage candidate skill (awaiting multi-seed gate)"
            commit_sha = self.commit_if_dirty(
                f"skill({emb}/{sk_name}): {stage_word} (sha={code_sha[:8]})",
                trailers={
                    "TRAX-Skill": sk_name,
                    "TRAX-MCP-Contract": mcp_tools_sha256[:16],
                    "TRAX-Evidence": evidence_source,
                    "TRAX-Verified-Seeds": ",".join(str(s) for s in unique_seeds),
                },
            )

            if not meets_main_gate:
                return {
                    "promoted": False,
                    "stage": "candidate",
                    "branch": cur_branch,
                    "commit": commit_sha or self._git("rev-parse", "--short", "HEAD").stdout.strip(),
                    "tag": None,
                    "verified_seeds": unique_seeds,
                    "required_seeds": req_seeds,
                    "skill_path": f"skills/{emb}/{sk_name}/SKILL.md",
                }

            tag_name = f"verified/{emb}/{sk_name}-{_utc_compact()}"
            if cur_branch != "main":
                self._git("checkout", "main")
                self._git("merge", "--no-ff", cur_branch, "-m", f"merge({cur_branch}): verified promotion of {sk_name}")
                self._active_branch = "main"
            self._git("tag", "-f", tag_name)
            merge_sha = self._git("rev-parse", "--short", "HEAD").stdout.strip()
            return {
                "promoted": True,
                "stage": "verified_main",
                "branch": "main",
                "merged_from": cur_branch,
                "commit": merge_sha,
                "tag": tag_name,
                "verified_seeds": unique_seeds,
                "required_seeds": req_seeds,
                "skill_path": f"skills/{emb}/{sk_name}/SKILL.md",
            }

    def _rebuild_skill_index_unlocked(self, emb: str) -> None:
        emb_dir = self.repo_dir / "skills" / emb
        if not emb_dir.is_dir():
            return
        lines = [f"# Verified Skills Index — `{emb}`", ""]
        for meta_file in sorted(emb_dir.glob("*/meta.json")):
            try:
                m = json.loads(meta_file.read_text(encoding="utf-8"))
            except ValueError:
                continue
            lines.append(
                f"- **`{m['skill_name']}`** | evidence=`{m['evidence_source']}` | "
                f"contract=`{m['mcp_tools_sha256']}` | {m.get('verify_detail', '')}"
            )
        (emb_dir / "INDEX.md").write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")

    def list_skills(
        self,
        *,
        current_embodiment: str | None = None,
        current_tools_sha: str | None = None,
        current_evidence: str | None = None,
        current_urdf_sha: str | None = None,
    ) -> list[dict[str, Any]]:
        """List all promoted/candidate skills with contract-drift, URDF-drift, and sim-to-real checks."""
        out: list[dict[str, Any]] = []
        skills_root = self.repo_dir / "skills"
        if not skills_root.is_dir():
            return out
        cur_emb = _slug(current_embodiment) if current_embodiment else None
        cur_sha = (current_tools_sha or "")[:16]
        cur_urdf = (current_urdf_sha or "")[:16]
        for meta_file in sorted(skills_root.glob("*/*/meta.json")):
            try:
                m = json.loads(meta_file.read_text(encoding="utf-8"))
            except ValueError:
                continue
            status = "active"
            if m.get("promotion_stage") == "candidate":
                status = "candidate"
            elif cur_emb and m.get("embodiment_id") != cur_emb:
                status = "other_embodiment"
            elif cur_sha and m.get("mcp_tools_sha256") != cur_sha:
                status = "stale_contract"
            elif cur_urdf and m.get("urdf_sha256") and m.get("urdf_sha256") != cur_urdf:
                status = "stale_kinematics"
            elif current_evidence == "physical_robot" and m.get("evidence_source") != "physical_robot":
                status = "advisory_sim_prior"
            rel_base = meta_file.parent.relative_to(self.repo_dir).as_posix()
            out.append(
                {
                    **m,
                    "status": status,
                    "skill_md_path": f"{rel_base}/SKILL.md",
                    "skill_py_path": f"{rel_base}/skill.py",
                }
            )
        return out

    def render_prompt_knowledge_block(
        self,
        *,
        embodiment_id: str,
        mcp_tools_sha256: str,
        evidence_source: str,
        urdf_sha256: str = "",
    ) -> str:
        """Render bounded, runtime-owned FACTS + FORBIDDEN + contract-valid skills for LLM grounding."""
        parts = []
        facts_md = self.read_text_file("FACTS.md")
        if facts_md and "(No live embodiment facts" not in facts_md:
            parts.append(facts_md[:1800])
        forb_md = self.read_text_file("FORBIDDEN.md")
        if forb_md and "(No negative constraints" not in forb_md:
            parts.append(forb_md[:1400])
        skills = self.list_skills(
            current_embodiment=embodiment_id,
            current_tools_sha=mcp_tools_sha256,
            current_evidence=evidence_source,
            current_urdf_sha=urdf_sha256 or None,
        )
        usable = [s for s in skills if s["status"] in ("active", "advisory_sim_prior")]
        if usable:
            sk_lines = ["CONTRACT-VERIFIED REUSABLE SKILLS IN KNOWLEDGE BASE:"]
            for s in usable[:4]:
                tier = "VERIFIED" if s["status"] == "active" else "ADVISORY_SIM_PRIOR (must re-verify on physical robot)"
                sk_lines.append(f"- `{s['skill_name']}` [{tier}] ({s['verify_detail']})")
                code = self.read_text_file(s["skill_py_path"])
                if code:
                    sk_lines.append("```python\n" + code[:1200] + "\n```")
            parts.append("\n".join(sk_lines))
        return "\n\n".join(parts)

    def read_text_file(self, rel_path: str) -> str:
        safe = (self.repo_dir / rel_path).resolve()
        if not str(safe).startswith(str(self.repo_dir.resolve())) or not safe.is_file():
            return ""
        return safe.read_text(encoding="utf-8", errors="replace")

    def git_log(self, limit: int = 25) -> list[dict[str, Any]]:
        with self._lock:
            fmt = "%h%x1f%H%x1f%ai%x1f%D%x1f%s"
            res = self._git("log", "--all", f"-n{max(1, min(60, limit))}", f"--pretty=format:{fmt}", check=False)
            if res.returncode != 0 or not res.stdout.strip():
                return []
            rows: list[dict[str, Any]] = []
            for line in res.stdout.splitlines():
                parts = line.split("\x1f")
                if len(parts) < 5:
                    continue
                rows.append(
                    {
                        "short_sha": parts[0],
                        "sha": parts[1],
                        "date": parts[2][:19],
                        "refs": parts[3].strip(),
                        "subject": parts[4].strip(),
                    }
                )
            return rows

    def git_diff(self, commit_sha: str, parent_sha: str | None = None) -> str:
        with self._lock:
            sha = re.sub(r"[^0-9a-fA-F]", "", str(commit_sha or ""))
            if not sha:
                return ""
            if parent_sha:
                p = re.sub(r"[^0-9a-fA-F]", "", str(parent_sha))
                res = self._git("diff", p, sha, check=False)
            else:
                res = self._git("show", "--format=fuller", "--stat", "--patch", sha, check=False)
            return res.stdout if res.returncode == 0 else (res.stderr or "")

    def rollback_policy_from_commit(self, commit_sha: str, task_id: str) -> dict[str, Any]:
        """Restore `policies/<task_id>.py` from any historical Git commit and commit the rollback cleanly."""
        with self._lock:
            sha = re.sub(r"[^0-9a-fA-F]", "", str(commit_sha or ""))
            if not sha:
                raise ValueError("Invalid commit SHA")
            rel = f"policies/{_slug(task_id)}.py"
            show = self._git("show", f"{sha}:{rel}", check=False)
            if show.returncode != 0:
                # Try finding any .py under policies/ in that commit
                ls = self._git("ls-tree", "-r", "--name-only", sha, "policies/", check=False)
                cands = [ln.strip() for ln in ls.stdout.splitlines() if ln.strip().endswith(".py")]
                if not cands:
                    raise FileNotFoundError(f"Commit {sha} does not contain any policy under policies/")
                rel = cands[0]
                show = self._git("show", f"{sha}:{rel}")
            restored_src = show.stdout
            dest = self.repo_dir / rel
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_text(restored_src, encoding="utf-8")
            new_sha = self.commit_if_dirty(
                f"rollback({rel}): restore policy from commit {sha[:8]}",
                trailers={"TRAX-Rollback-From": sha[:12]},
            )
            return {
                "restored_file": rel,
                "from_commit": sha[:12],
                "new_commit": new_sha,
                "source": restored_src,
                "branch": self.current_branch(),
            }

    def snapshot(
        self,
        *,
        current_embodiment: str | None = None,
        current_tools_sha: str | None = None,
        current_evidence: str | None = None,
        current_urdf_sha: str | None = None,
        trace_stats: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            branches_raw = self._git("branch", "--format=%(refname:short)", check=False).stdout.splitlines()
            tags_raw = self._git("tag", "--sort=-creatordate", check=False).stdout.splitlines()
            return {
                "repo_path": "workspace/knowledge",
                "branch": self.current_branch(),
                "branches": [b.strip() for b in branches_raw if b.strip()],
                "tags": [t.strip() for t in tags_raw[:20] if t.strip()],
                "facts_md": self.read_text_file("FACTS.md"),
                "forbidden_md": self.read_text_file("FORBIDDEN.md"),
                "skills": self.list_skills(
                    current_embodiment=current_embodiment,
                    current_tools_sha=current_tools_sha,
                    current_evidence=current_evidence,
                    current_urdf_sha=current_urdf_sha,
                ),
                "git_log": self.git_log(25),
                "workspace_stats": dict(trace_stats or {}),
            }
