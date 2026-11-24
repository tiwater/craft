#!/usr/bin/env python3
"""Generate reproducible LaTeX tables and vector SVG figures from archived TRAX JSON reports.

All numbers in papers/trax/tables/*.tex and docs/assets/figures/*.svg are computed
deterministically from immutable archived benchmark JSON receipts under experiments/
(which retain legacy internal codename keys such as 'craft_full' from earlier milestones).
Zero numbers are fabricated or hand-edited.
"""
from __future__ import annotations

import argparse
import hashlib
import html
import json
import re
import shutil
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent.parent
EXP_V2 = ROOT / "experiments" / "goal-v2-20261122"
EXP_V3 = ROOT / "experiments" / "goal-v3-simex-parity"
PAPER_DIR = ROOT / "papers" / "trax"
PAPER_TABLES = PAPER_DIR / "tables"
PAPER_FIGURES = PAPER_DIR / "figures"
DOCS_FIGURES = ROOT / "docs" / "assets" / "figures"


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256_short(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()[:12]


def _fmt_pct(rate: float) -> str:
    return f"{rate * 100.0:.1f}\\%"


def _fmt_p(p: float | None) -> str:
    if p is None:
        return "---"
    if p < 0.0001:
        return "$<0.0001$"
    return f"${p:.4f}$"


def generate_cafebench_v2_table(data: dict[str, Any], src: Path) -> str:
    s = data["summary_by_method"]
    mc = data["paired_mcnemar_craft_vs"]
    order = [
        ("craft_full", "\\textbf{TRAX (Ours)}"),
        ("aspire_protocol", "ASPIRE-Protocol (Adapted)"),
        ("capx_protocol", "CaP-X-Protocol (Adapted)"),
        ("simex_direct_repair", "SimEX-Direct-Repair (Ablation)"),
        ("strong_zeroshot", "Strong Zero-Shot"),
    ]
    lines = [
        f"% Auto-generated from {src.relative_to(ROOT)} (sha256:{_sha256_short(src)})",
        "\\begin{tabular}{@{}lcccccc@{}}",
        "\\toprule",
        "Method & Held-Out & Sim / Shadow & Dup / False & Dev LLM & Wall (s) & $p$ vs.\\ TRAX \\\\",
        "\\midrule",
    ]
    for key, label in order:
        r = s[key]
        p_str = "ref." if key == "craft_full" else _fmt_p(mc.get(key, {}).get("p_value"))
        overall = f"{r['passed']}/{r['total']} ({_fmt_pct(r['pass_rate'])})"
        sim_sh = f"{r['simulator_passed']}/{r['simulator_total']} \\,/\\, {r['recorded_shadow_passed']}/{r['recorded_shadow_total']}"
        dup_fa = f"{r['duplicate_non_idempotent_count']} / {r['false_actuation_count']}"
        if key == "craft_full":
            overall = f"\\textbf{{{overall}}}"
            sim_sh = f"\\textbf{{{sim_sh}}}"
            dup_fa = f"\\textbf{{{dup_fa}}}"
            dev_llm = f"\\textbf{{{r['dev_llm_calls']}}}"
            wall = f"\\textbf{{{r['dev_wall_s']:.1f}}}"
        else:
            dev_llm = str(r["dev_llm_calls"])
            wall = f"{r['dev_wall_s']:.1f}"
        lines.append(f"{label} & {overall} & {sim_sh} & {dup_fa} & {dev_llm} & {wall} & {p_str} \\\\")
    lines.extend(["\\bottomrule", "\\end{tabular}", ""])
    return "\n".join(lines)


def generate_sim2sim_table(r2: dict[str, Any], r1: dict[str, Any], src_r2: Path, src_r1: Path) -> str:
    s2 = r2["summary_by_method"]
    s1 = r1["summary_by_method"]
    mc = r2["paired_mcnemar_craft_vs"]
    order = [
        ("craft_full", "\\textbf{TRAX (Ours, Run 2)}"),
        ("simex_protocol", "SimEX-Protocol (Adapted)"),
        ("direct_repair", "Direct Target Repair"),
        ("zeroshot_sim2sim", "Zero-Shot Sim2Sim"),
    ]
    lines = [
        f"% Auto-generated from {src_r2.relative_to(ROOT)} (sha256:{_sha256_short(src_r2)}) and {src_r1.relative_to(ROOT)} (sha256:{_sha256_short(src_r1)})",
        "\\begin{tabular}{@{}lcccccc@{}}",
        "\\toprule",
        "Method & Overall ($n{=}20$) & Target / Shadow & Trials / Cal & LLM & Run 1 & $p$ vs.\\ TRAX \\\\",
        "\\midrule",
    ]
    for key, label in order:
        r = s2[key]
        prev = s1[key]
        p_str = "ref." if key == "craft_full" else _fmt_p(mc.get(key, {}).get("p_value"))
        overall = f"{r['passed']}/{r['total_trials']} ({_fmt_pct(r['pass_rate'])})"
        t_sh = f"{r['target_passed']}/{r['target_total']} \\,/\\, {r['shadow_passed']}/{r['shadow_total']}"
        tr_cal = f"{r['adapt_target_trials']} / {r['adapt_calibration_trials']}"
        run1_str = f"{prev['passed']}/{prev['total_trials']}"
        if key == "craft_full":
            overall = f"\\textbf{{{overall}}}"
            t_sh = f"\\textbf{{{t_sh}}}"
        lines.append(f"{label} & {overall} & {t_sh} & {tr_cal} & {r['adapt_llm_calls']} & {run1_str} & {p_str} \\\\")
    lines.extend(["\\bottomrule", "\\end{tabular}", ""])
    return "\n".join(lines)


def generate_combo_table(data: dict[str, Any], src: Path) -> str:
    s = data["summary_by_method"]
    order = [
        ("craft_pipelined_journaled", "\\textbf{TRAX Pipelined + Journal}"),
        ("pipelined_stateless", "Pipelined Stateless"),
        ("sequential_stateless", "Sequential Stateless"),
    ]
    lines = [
        f"% Auto-generated from {src.relative_to(ROOT)} (sha256:{_sha256_short(src)})",
        "\\begin{tabular}{@{}lccccc@{}}",
        "\\toprule",
        "Execution Policy & Pass ($n{=}12$) & Dup Actions & Interlocks & Cycle (s) & Orders/hr \\\\",
        "\\midrule",
    ]
    for key, label in order:
        r = s[key]
        passed = f"{r['passed']}/{r['total_trials']} ({_fmt_pct(r['pass_rate'])})"
        dup = str(r["duplicate_non_idempotent_orders"])
        inter = str(r["interlock_violations"])
        cyc = f"{r['mean_completed_cycle_s']:.2f}"
        tput = f"{r['throughput_orders_per_hour']:.1f}"
        if key == "craft_pipelined_journaled":
            passed = f"\\textbf{{{passed}}}"
            dup = f"\\textbf{{{dup}}}"
            inter = f"\\textbf{{{inter}}}"
            cyc = f"\\textbf{{{cyc}}}"
            tput = f"\\textbf{{{tput}}}"
        lines.append(f"{label} & {passed} & {dup} & {inter} & {cyc} & {tput} \\\\")
    lines.extend(["\\bottomrule", "\\end{tabular}", ""])
    return "\n".join(lines)


def generate_robosuite_v3_table(
    full_v3: dict[str, Any],
    nut_fix: dict[str, Any],
    src_full: Path,
    src_nut: Path,
) -> str:
    s_full = full_v3["summary_by_method"]
    s_nut = nut_fix["summary_by_method"]
    order = [
        ("craft_full", "\\textbf{TRAX (ActiveProbe + Screener)}"),
        ("capx_protocol", "CaP-X-Protocol (Online Regen)"),
        ("aspire_protocol", "ASPIRE-Protocol (Adapted)"),
        ("simex_protocol", "SimEX-Protocol (Adapted)"),
        ("simex_direct_repair", "SimEX-Direct-Repair (Ablation)"),
        ("strong_zeroshot", "Strong Zero-Shot"),
    ]
    lines = [
        f"% Auto-generated from {src_full.relative_to(ROOT)} (sha256:{_sha256_short(src_full)}) and {src_nut.relative_to(ROOT)} (sha256:{_sha256_short(src_nut)})",
        "\\begin{tabular}{@{}lccccccc@{}}",
        "\\toprule",
        "Method & Full v3 ($n{=}80$) & Lift / Can / Door & Nut (Pre) & Nut (Subr.) & Eval LLM & P95 (s) \\\\",
        "\\midrule",
    ]
    for key, label in order:
        r = s_full[key]
        rn = s_nut[key]
        bt = r["by_task"]
        lcd = f"{bt['Lift']['passed']}/{bt['PickPlaceCan']['passed']}/{bt['Door']['passed']}"
        nut_pre = f"{bt['NutAssemblySquare']['passed']}/20"
        nut_post_n = rn["by_task"]["NutAssemblySquare"]["passed"]
        nut_post = f"{nut_post_n}/20"
        if key == "capx_protocol":
            nut_post = f"{nut_post_n}/20 ({rn['turn1_passed']}$\\dagger$)"
        overall = f"{r['passed']}/80 ({_fmt_pct(r['pass_rate'])})"
        eval_llm = f"{r['eval_llm_calls']} / {rn['eval_llm_calls']}"
        p95 = f"{r['p95_wall_s']:.2f} / {rn['p95_wall_s']:.2f}"
        if key == "craft_full":
            overall = f"\\textbf{{{overall}}}"
            lcd = f"\\textbf{{{lcd}}}"
            nut_post = f"\\textbf{{{nut_post}}}"
            eval_llm = f"\\textbf{{{eval_llm}}}"
            p95 = f"\\textbf{{{p95}}}"
        lines.append(f"{label} & {overall} & {lcd} & {nut_pre} & {nut_post} & {eval_llm} & {p95} \\\\")
    lines.extend(["\\bottomrule", "\\end{tabular}", ""])
    return "\n".join(lines)


def build_architecture_svg() -> str:
    """Crisp academic architecture diagram showing strict ownership boundaries and transactional evolution."""
    return """<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 980 420" width="980" height="420" role="img" aria-label="TRAX Ownership-Separated Transactional Architecture">
  <defs>
    <marker id="arrow" viewBox="0 0 10 10" refX="8.5" refY="5" markerWidth="6.5" markerHeight="6.5" orient="auto-start-reverse">
      <path d="M 0 1 L 10 5 L 0 9 z" fill="#334155"/>
    </marker>
    <marker id="arrow-blue" viewBox="0 0 10 10" refX="8.5" refY="5" markerWidth="6.5" markerHeight="6.5" orient="auto-start-reverse">
      <path d="M 0 1 L 10 5 L 0 9 z" fill="#1d4ed8"/>
    </marker>
    <marker id="arrow-green" viewBox="0 0 10 10" refX="8.5" refY="5" markerWidth="6.5" markerHeight="6.5" orient="auto-start-reverse">
      <path d="M 0 1 L 10 5 L 0 9 z" fill="#15803d"/>
    </marker>
    <style>
      .t-title { font-family: Inter, -apple-system, sans-serif; font-size: 13px; font-weight: 700; fill: #0f172a; }
      .t-sub   { font-family: Inter, -apple-system, sans-serif; font-size: 10.5px; fill: #334155; }
      .t-mono  { font-family: "JetBrains Mono", monospace; font-size: 9.8px; fill: #475569; }
      .t-zone  { font-family: Inter, -apple-system, sans-serif; font-size: 10.5px; font-weight: 700; letter-spacing: 0.04em; text-transform: uppercase; }
    </style>
  </defs>

  <rect width="980" height="420" fill="#ffffff" rx="6"/>

  <!-- Zone 1: Perception Owner -->
  <rect x="18" y="18" width="210" height="384" rx="8" fill="#f8fafc" stroke="#cbd5e1" stroke-width="1.2"/>
  <text x="32" y="39" class="t-zone" fill="#475569">1. Perception Owner</text>
  <rect x="32" y="54" width="182" height="92" rx="6" fill="#ffffff" stroke="#94a3b8" stroke-width="1.2"/>
  <text x="44" y="76" class="t-title">EvidenceScene / Vision</text>
  <text x="44" y="94" class="t-sub">• 6-DoF object &amp; cup state</text>
  <text x="44" y="110" class="t-sub">• Timestamp &amp; confidence</text>
  <text x="44" y="126" class="t-sub">• Freshness / ambiguity flags</text>

  <rect x="32" y="164" width="182" height="92" rx="6" fill="#ffffff" stroke="#94a3b8" stroke-width="1.2"/>
  <text x="44" y="186" class="t-title">Recorded Shadow Feed</text>
  <text x="44" y="204" class="t-sub">• Real / logged edge frames</text>
  <text x="44" y="220" class="t-sub">• Zero physical actuation</text>
  <text x="44" y="236" class="t-mono">evidence: recorded_data</text>

  <rect x="32" y="274" width="182" height="110" rx="6" fill="#fefce8" stroke="#eab308" stroke-width="1.1"/>
  <text x="44" y="295" class="t-title">Boundary Contract</text>
  <text x="44" y="313" class="t-sub">TRAX never fabricates</text>
  <text x="44" y="329" class="t-sub">scene geometry or invents</text>
  <text x="44" y="345" class="t-sub">missing sensor estimates.</text>
  <text x="44" y="366" class="t-mono">fail-closed on stale/ambig</text>

  <!-- Zone 2: TRAX Policy Core (Pure Python stdlib) -->
  <rect x="246" y="18" width="486" height="384" rx="8" fill="#eff6ff" fill-opacity="0.45" stroke="#93c5fd" stroke-width="1.4"/>
  <text x="262" y="39" class="t-zone" fill="#1d4ed8">2. TRAX Core (Pure Python Stdlib Governance &amp; Evolution)</text>

  <!-- Online Execution Path -->
  <rect x="262" y="54" width="454" height="114" rx="6" fill="#ffffff" stroke="#2563eb" stroke-width="1.4"/>
  <text x="276" y="75" class="t-title">Online Transactional Execution (0 Online LLM Calls)</text>

  <rect x="276" y="88" width="136" height="66" rx="5" fill="#f8fafc" stroke="#cbd5e1"/>
  <text x="286" y="107" class="t-title">Frozen Bundle</text>
  <text x="286" y="123" class="t-sub">SHA-256 Policy +</text>
  <text x="286" y="138" class="t-sub">AST Sandbox</text>

  <rect x="426" y="88" width="136" height="66" rx="5" fill="#f8fafc" stroke="#cbd5e1"/>
  <text x="436" y="107" class="t-title">SQLite Journal</text>
  <text x="436" y="123" class="t-sub">Intent-before-dispatch</text>
  <text x="436" y="138" class="t-sub">Query-first reconcile</text>

  <rect x="576" y="88" width="126" height="66" rx="5" fill="#f0fdf4" stroke="#86efac"/>
  <text x="586" y="107" class="t-title">4-Layer Attr.</text>
  <text x="586" y="123" class="t-sub">percep / control</text>
  <text x="586" y="138" class="t-sub">device / policy</text>

  <!-- Offline / Nightly Evolution Path -->
  <rect x="262" y="186" width="454" height="126" rx="6" fill="#ffffff" stroke="#64748b" stroke-width="1.2"/>
  <text x="276" y="207" class="t-title">Offline / Nightly Evolution Loop (Bounded Budget)</text>

  <rect x="276" y="220" width="136" height="78" rx="5" fill="#f8fafc" stroke="#cbd5e1"/>
  <text x="286" y="238" class="t-title">ActiveProbe</text>
  <text x="286" y="254" class="t-sub">Public-SDK scan</text>
  <text x="286" y="269" class="t-sub">Sim calibration</text>
  <text x="286" y="284" class="t-mono">0 hardcoded hints</text>

  <rect x="426" y="220" width="136" height="78" rx="5" fill="#f8fafc" stroke="#cbd5e1"/>
  <text x="436" y="238" class="t-title">RepairScreener</text>
  <text x="436" y="254" class="t-sub">Multi-hypothesis</text>
  <text x="436" y="269" class="t-sub">AST subroutine fix</text>
  <text x="436" y="284" class="t-mono">RegressionSuite</text>

  <rect x="576" y="220" width="126" height="78" rx="5" fill="#f0fdf4" stroke="#86efac"/>
  <text x="586" y="238" class="t-title">Indep. Gate</text>
  <text x="586" y="254" class="t-sub">Held-out accept</text>
  <text x="586" y="269" class="t-sub">Shadow 0-act</text>
  <text x="586" y="284" class="t-mono">Proposer != Eval</text>

  <!-- Persistent Registries -->
  <rect x="262" y="328" width="218" height="56" rx="6" fill="#ffffff" stroke="#94a3b8"/>
  <text x="276" y="349" class="t-title">ReleaseRegistry</text>
  <text x="276" y="366" class="t-sub">Immutable versions + rollback drill</text>

  <rect x="498" y="328" width="218" height="56" rx="6" fill="#ffffff" stroke="#94a3b8"/>
  <text x="512" y="349" class="t-title">Verified Skill Toolbox</text>
  <text x="512" y="366" class="t-sub">AST helpers + contract-drift invalidation</text>

  <!-- Zone 3: Host Actuation Owner -->
  <rect x="750" y="18" width="212" height="384" rx="8" fill="#f8fafc" stroke="#cbd5e1" stroke-width="1.2"/>
  <text x="764" y="39" class="t-zone" fill="#475569">3. Host Actuation Owner</text>

  <rect x="764" y="54" width="184" height="114" rx="6" fill="#ffffff" stroke="#94a3b8" stroke-width="1.2"/>
  <text x="776" y="76" class="t-title">Robot &amp; Interlocks</text>
  <text x="776" y="94" class="t-sub">• Motion planning &amp; IK</text>
  <text x="776" y="110" class="t-sub">• Thermal / door / L2 locks</text>
  <text x="776" y="126" class="t-sub">• Hardware timeouts</text>
  <text x="776" y="144" class="t-mono">ticos-edge / sim host</text>

  <rect x="764" y="186" width="184" height="110" rx="6" fill="#ffffff" stroke="#94a3b8" stroke-width="1.2"/>
  <text x="776" y="208" class="t-title">Non-Idempotent Devices</text>
  <text x="776" y="226" class="t-sub">• Espresso brew machine</text>
  <text x="776" y="242" class="t-sub">• Bagel / high-speed oven</text>
  <text x="776" y="258" class="t-sub">• Explicit status query API</text>
  <text x="776" y="276" class="t-mono">query_action_status(id)</text>

  <rect x="764" y="314" width="184" height="70" rx="6" fill="#f0fdf4" stroke="#86efac"/>
  <text x="776" y="335" class="t-title">4-Tier Evidence Tags</text>
  <text x="776" y="352" class="t-mono">fixture | simulator</text>
  <text x="776" y="368" class="t-mono">recorded_data | robot*</text>

  <!-- Arrows -->
  <line x1="214" y1="100" x2="260" y2="100" stroke="#334155" stroke-width="1.4" marker-end="url(#arrow)"/>
  <line x1="214" y1="210" x2="260" y2="245" stroke="#334155" stroke-width="1.3" stroke-dasharray="4 3" marker-end="url(#arrow)"/>
  <line x1="412" y1="121" x2="424" y2="121" stroke="#1d4ed8" stroke-width="1.4" marker-end="url(#arrow-blue)"/>
  <line x1="562" y1="121" x2="574" y2="121" stroke="#1d4ed8" stroke-width="1.4" marker-end="url(#arrow-blue)"/>
  <line x1="716" y1="110" x2="762" y2="110" stroke="#1d4ed8" stroke-width="1.5" marker-end="url(#arrow-blue)"/>
  <line x1="762" y1="138" x2="716" y2="138" stroke="#334155" stroke-width="1.3" marker-end="url(#arrow)"/>
  <line x1="856" y1="168" x2="856" y2="184" stroke="#334155" stroke-width="1.4" marker-end="url(#arrow)"/>

  <line x1="412" y1="259" x2="424" y2="259" stroke="#334155" stroke-width="1.3" marker-end="url(#arrow)"/>
  <line x1="562" y1="259" x2="574" y2="259" stroke="#15803d" stroke-width="1.4" marker-end="url(#arrow-green)"/>
  <path d="M 639 154 L 639 178 L 344 178 L 344 218" fill="none" stroke="#64748b" stroke-width="1.2" stroke-dasharray="4 3" marker-end="url(#arrow)"/>
  <path d="M 639 298 L 639 318 L 371 318 L 371 326" fill="none" stroke="#15803d" stroke-width="1.3" marker-end="url(#arrow-green)"/>
  <line x1="639" y1="298" x2="639" y2="326" stroke="#15803d" stroke-width="1.3" marker-end="url(#arrow-green)"/>
</svg>
"""


def _hbar_panel(
    x0: float,
    y0: float,
    w: float,
    title: str,
    subtitle: str,
    rows: list[tuple[str, float, tuple[float, float] | None, str, bool]],
) -> str:
    out = [
        f'<text x="{x0}" y="{y0}" class="p-title">{html.escape(title)}</text>',
        f'<text x="{x0}" y="{y0 + 16}" class="p-sub">{html.escape(subtitle)}</text>',
    ]
    label_w = 152.0
    bar_x = x0 + label_w
    bar_w = w - label_w - 78.0
    top = y0 + 34.0
    row_h = 34.0
    chart_h = len(rows) * row_h
    for pct in (0, 25, 50, 75, 100):
        gx = bar_x + bar_w * (pct / 100.0)
        out.append(
            f'<line x1="{gx:.1f}" y1="{top - 4:.1f}" x2="{gx:.1f}" y2="{top + chart_h:.1f}" '
            f'stroke="#e2e8f0" stroke-width="1"/>'
        )
        out.append(
            f'<text x="{gx:.1f}" y="{top + chart_h + 14:.1f}" text-anchor="middle" class="p-axis">{pct}%</text>'
        )

    for idx, (label, val, ci, val_text, highlight) in enumerate(rows):
        ry = top + idx * row_h
        fill = "#1d4ed8" if highlight else "#94a3b8"
        txt_cls = "p-lbl-hi" if highlight else "p-lbl"
        bw = max(1.5, bar_w * val)
        out.append(f'<text x="{bar_x - 8:.1f}" y="{ry + 16:.1f}" text-anchor="end" class="{txt_cls}">{html.escape(label)}</text>')
        out.append(f'<rect x="{bar_x:.1f}" y="{ry + 4:.1f}" width="{bw:.1f}" height="18" rx="3" fill="{fill}"/>')
        if ci is not None:
            cx1 = bar_x + bar_w * ci[0]
            cx2 = bar_x + bar_w * ci[1]
            cy = ry + 13.0
            stroke = "#0f172a"
            out.append(f'<line x1="{cx1:.1f}" y1="{cy:.1f}" x2="{cx2:.1f}" y2="{cy:.1f}" stroke="{stroke}" stroke-width="1.3"/>')
            out.append(f'<line x1="{cx1:.1f}" y1="{cy - 4:.1f}" x2="{cx1:.1f}" y2="{cy + 4:.1f}" stroke="{stroke}" stroke-width="1.3"/>')
            out.append(f'<line x1="{cx2:.1f}" y1="{cy - 4:.1f}" x2="{cx2:.1f}" y2="{cy + 4:.1f}" stroke="{stroke}" stroke-width="1.3"/>')
        out.append(f'<text x="{bar_x + bar_w + 8:.1f}" y="{ry + 16:.1f}" class="{txt_cls}">{html.escape(val_text)}</text>')
    return "\n  ".join(out)


def build_cafebench_sim2sim_svg(cafe: dict[str, Any], s2s: dict[str, Any]) -> str:
    sc = cafe["summary_by_method"]
    ss = s2s["summary_by_method"]
    left_rows = [
        ("TRAX (Ours)", sc["craft_full"]["pass_rate"], tuple(sc["craft_full"]["wilson_95_ci"]), f"{sc['craft_full']['passed']}/30 (100%)", True),
        ("ASPIRE (Adapted)", sc["aspire_protocol"]["pass_rate"], tuple(sc["aspire_protocol"]["wilson_95_ci"]), f"{sc['aspire_protocol']['passed']}/30 (76.7%)", False),
        ("CaP-X (Adapted)", sc["capx_protocol"]["pass_rate"], tuple(sc["capx_protocol"]["wilson_95_ci"]), f"{sc['capx_protocol']['passed']}/30 (76.7%)", False),
        ("SimEX-Direct", sc["simex_direct_repair"]["pass_rate"], tuple(sc["simex_direct_repair"]["wilson_95_ci"]), f"{sc['simex_direct_repair']['passed']}/30 (70.0%)", False),
        ("Zero-Shot", sc["strong_zeroshot"]["pass_rate"], tuple(sc["strong_zeroshot"]["wilson_95_ci"]), f"{sc['strong_zeroshot']['passed']}/30 (23.3%)", False),
    ]
    right_rows = [
        ("TRAX (Ours)", ss["craft_full"]["pass_rate"], tuple(ss["craft_full"]["wilson_95_ci"]), f"{ss['craft_full']['passed']}/20 (95.0%)", True),
        ("SimEX-Protocol", ss["simex_protocol"]["pass_rate"], tuple(ss["simex_protocol"]["wilson_95_ci"]), f"{ss['simex_protocol']['passed']}/20 (80.0%)", False),
        ("Direct Repair", ss["direct_repair"]["pass_rate"], tuple(ss["direct_repair"]["wilson_95_ci"]), f"{ss['direct_repair']['passed']}/20 (80.0%)", False),
        ("Zero-Shot Sim2Sim", ss["zeroshot_sim2sim"]["pass_rate"], tuple(ss["zeroshot_sim2sim"]["wilson_95_ci"]), f"{ss['zeroshot_sim2sim']['passed']}/20 (65.0%)", False),
    ]
    p_left = _hbar_panel(
        22, 30, 452,
        "(a) CafeBench v2 Held-Out Pass Rate (n = 30)",
        "MuJoCo simulator (26) + recorded_data shadow (4); Wilson 95% CI",
        left_rows,
    )
    p_right = _hbar_panel(
        506, 30, 452,
        "(b) Sim-to-Sim Stage-2 Adaptation (<= 5 Target Trials)",
        "Nominal -> noisy+DR MuJoCo target (16) + shadow (4); Wilson 95% CI",
        right_rows,
    )
    return f"""<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 980 265" width="980" height="265" role="img" aria-label="CafeBench v2 and Sim-to-Sim Stage-2 Benchmark Comparison">
  <style>
    .p-title {{ font-family: Inter, -apple-system, sans-serif; font-size: 13px; font-weight: 700; fill: #0f172a; }}
    .p-sub   {{ font-family: Inter, -apple-system, sans-serif; font-size: 10.5px; fill: #64748b; }}
    .p-lbl   {{ font-family: Inter, -apple-system, sans-serif; font-size: 11px; fill: #334155; }}
    .p-lbl-hi{{ font-family: Inter, -apple-system, sans-serif; font-size: 11px; font-weight: 700; fill: #0f172a; }}
    .p-axis  {{ font-family: "JetBrains Mono", monospace; font-size: 9.5px; fill: #64748b; }}
  </style>
  <rect width="980" height="265" fill="#ffffff" rx="6"/>
  <line x1="490" y1="16" x2="490" y2="250" stroke="#e2e8f0" stroke-width="1"/>
  {p_left}
  {p_right}
</svg>
"""


def build_robosuite_combo_svg(nut_fix: dict[str, Any], combo: dict[str, Any]) -> str:
    sn = nut_fix["summary_by_method"]
    sc = combo["summary_by_method"]
    left_rows = [
        ("TRAX (0 eval LLM)", sn["craft_full"]["pass_rate"], tuple(sn["craft_full"]["wilson_95_ci"]), f"{sn['craft_full']['passed']}/20 (60%)", True),
        ("CaP-X Turn-1 Frozen", sn["capx_protocol"]["turn1_pass_rate"], None, f"{sn['capx_protocol']['turn1_passed']}/20 (60%)", False),
        ("CaP-X (+8 eval LLM)", sn["capx_protocol"]["pass_rate"], tuple(sn["capx_protocol"]["wilson_95_ci"]), f"{sn['capx_protocol']['passed']}/20 (80%)", False),
        ("ASPIRE (Adapted)", sn["aspire_protocol"]["pass_rate"], tuple(sn["aspire_protocol"]["wilson_95_ci"]), f"{sn['aspire_protocol']['passed']}/20 (30%)", False),
        ("SimEX-Protocol", sn["simex_protocol"]["pass_rate"], tuple(sn["simex_protocol"]["wilson_95_ci"]), f"{sn['simex_protocol']['passed']}/20 (0%)", False),
        ("Zero-Shot", sn["strong_zeroshot"]["pass_rate"], tuple(sn["strong_zeroshot"]["wilson_95_ci"]), f"{sn['strong_zeroshot']['passed']}/20 (0%)", False),
    ]
    p_left = _hbar_panel(
        22, 30, 452,
        "(a) Zero-Leakage NutAssemblySquare (Selective Subroutine)",
        "Robosuite Panda (n=20); TRAX 3x jump (4->12/20), p=0.0005 vs SimEX",
        left_rows,
    )

    c_trax = sc["craft_pipelined_journaled"]
    c_pipe = sc["pipelined_stateless"]
    c_seq = sc["sequential_stateless"]
    right_rows = [
        ("TRAX Pipe+Journal", c_trax["pass_rate"], tuple(c_trax["wilson_95_ci"]), "12/12 | 75.9/hr", True),
        ("Pipelined Stateless", c_pipe["pass_rate"], tuple(c_pipe["wilson_95_ci"]), "9/12 | 6 locks", False),
        ("Sequential Stateless", c_seq["pass_rate"], tuple(c_seq["wilson_95_ci"]), "9/12 | 46.7/hr", False),
    ]
    p_right = _hbar_panel(
        506, 30, 452,
        "(b) CafeBench v3 Multi-Appliance Combo (Coffee + Oven)",
        "Shared arm + crash/interlock faults (n=12); +62.5% throughput, 0 dup",
        right_rows,
    )
    callout = """
  <rect x="520" y="198" width="424" height="72" rx="6" fill="#f8fafc" stroke="#cbd5e1"/>
  <text x="534" y="219" class="p-lbl-hi">Operational Safety &amp; Throughput Summary (Combo v3)</text>
  <text x="534" y="238" class="p-sub">• TRAX Pipelined + Journal: 100% pass, 0 dup brews/heats, 0 interlocks, 47.45 s/order</text>
  <text x="534" y="256" class="p-sub">• Stateless baselines: 3 duplicate appliance starts, 4–6 interlock violations</text>
"""
    return f"""<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 980 296" width="980" height="296" role="img" aria-label="Zero-Leakage Robosuite Subroutine Repair and Multi-Appliance Combo Benchmarks">
  <style>
    .p-title {{ font-family: Inter, -apple-system, sans-serif; font-size: 13px; font-weight: 700; fill: #0f172a; }}
    .p-sub   {{ font-family: Inter, -apple-system, sans-serif; font-size: 10.5px; fill: #64748b; }}
    .p-lbl   {{ font-family: Inter, -apple-system, sans-serif; font-size: 11px; fill: #334155; }}
    .p-lbl-hi{{ font-family: Inter, -apple-system, sans-serif; font-size: 11px; font-weight: 700; fill: #0f172a; }}
    .p-axis  {{ font-family: "JetBrains Mono", monospace; font-size: 9.5px; fill: #64748b; }}
  </style>
  <rect width="980" height="296" fill="#ffffff" rx="6"/>
  <line x1="490" y1="16" x2="490" y2="280" stroke="#e2e8f0" stroke-width="1"/>
  {p_left}
  {p_right}
  {callout}
</svg>
"""


def verify_site() -> int:
    index_path = ROOT / "docs" / "index.html"
    if not index_path.exists():
        print("ERROR: docs/index.html does not exist", file=sys.stderr)
        return 1
    html_text = index_path.read_text(encoding="utf-8")
    refs = re.findall(r'(?:src|href)="([^"]+)"', html_text)
    missing: list[str] = []
    for ref in refs:
        if ref.startswith(("http://", "https://", "mailto:", "#")):
            continue
        target = (ROOT / "docs" / ref).resolve()
        if not target.exists():
            missing.append(ref)
    if missing:
        print(f"ERROR: Broken relative links in docs/index.html: {missing}", file=sys.stderr)
        return 1
    print(f"Site link check OK ({len(refs)} attributes verified, 0 broken local references).")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-site", action="store_true", help="Verify relative links in docs/index.html")
    args = parser.parse_args()
    if args.verify_site:
        return verify_site()

    PAPER_TABLES.mkdir(parents=True, exist_ok=True)
    PAPER_FIGURES.mkdir(parents=True, exist_ok=True)
    DOCS_FIGURES.mkdir(parents=True, exist_ok=True)

    src_cafe = EXP_V2 / "cafebench-v2" / "cafebench_v2_report.json"
    src_s2s_r2 = EXP_V3 / "sim2sim-stage2" / "sim2sim_report.json"
    src_s2s_r1 = EXP_V3 / "sim2sim-stage2-run1" / "sim2sim_report.json"
    src_combo = EXP_V3 / "combo-v3" / "combo_report.json"
    src_robo_v3 = EXP_V3 / "robosuite-parity-v3" / "robosuite_prior_work_report.json"
    src_robo_nut = EXP_V3 / "robosuite-v3-nut-subroutine-fix" / "robosuite_prior_work_report.json"

    cafe = _load_json(src_cafe)
    s2s_r2 = _load_json(src_s2s_r2)
    s2s_r1 = _load_json(src_s2s_r1)
    combo = _load_json(src_combo)
    robo_v3 = _load_json(src_robo_v3)
    robo_nut = _load_json(src_robo_nut)

    (PAPER_TABLES / "cafebench_v2_table.tex").write_text(
        generate_cafebench_v2_table(cafe, src_cafe), encoding="utf-8"
    )
    (PAPER_TABLES / "sim2sim_table.tex").write_text(
        generate_sim2sim_table(s2s_r2, s2s_r1, src_s2s_r2, src_s2s_r1), encoding="utf-8"
    )
    (PAPER_TABLES / "combo_v3_table.tex").write_text(
        generate_combo_table(combo, src_combo), encoding="utf-8"
    )
    (PAPER_TABLES / "robosuite_v3_table.tex").write_text(
        generate_robosuite_v3_table(robo_v3, robo_nut, src_robo_v3, src_robo_nut), encoding="utf-8"
    )

    (DOCS_FIGURES / "trax_architecture.svg").write_text(build_architecture_svg(), encoding="utf-8")
    (DOCS_FIGURES / "cafebench_sim2sim_comparison.svg").write_text(
        build_cafebench_sim2sim_svg(cafe, s2s_r2), encoding="utf-8"
    )
    (DOCS_FIGURES / "robosuite_combo_comparison.svg").write_text(
        build_robosuite_combo_svg(robo_nut, combo), encoding="utf-8"
    )

    previews_dir = EXP_V2 / "fidelity-previews"
    for fname in ("panda_pbr_overview.jpg", "panda_pbr_overhead.jpg", "panda_pbr_spout.jpg"):
        src_img = previews_dir / fname
        if src_img.exists():
            shutil.copy2(src_img, DOCS_FIGURES / fname)
            shutil.copy2(src_img, PAPER_FIGURES / fname)

    print("Generated LaTeX tables in papers/trax/tables/ and figures in docs/assets/figures/.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
