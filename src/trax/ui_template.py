"""Pure-English, publication-ready 3-Pane Closed-Loop Research IDE for TRAX UI.

Visualizes TRAX's real scientific workflow end-to-end:
  01 Explore (Public-SDK ActiveProbe)
  -> 02 Synthesize Python (LLM Policy + AST Compliance + Unified Diff)
  -> 03 Execute (AST Sandbox + Live Multi-Camera Robot + SQLite Intent Journal)
  -> 04 Attribute & Iterate (4-Layer Attribution + Selective Subroutine Repair)

Pure Python stdlib (zero external JS/CSS dependencies).
"""
from __future__ import annotations

UI_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>TRAX UI — Closed-Loop Policy Synthesis &amp; Transactional Execution Lab</title>
<style>
  :root {
    --bg: #070b13; --surface: #0d1423; --surface2: #131d31; --codebg: #060a12;
    --border: #1e2d48; --border2: #2b3f63;
    --text: #f1f5f9; --muted: #94a3b8; --dim: #64748b;
    --cyan: #38bdf8; --cyan-bg: rgba(56,189,248,0.12);
    --green: #22c55e; --green-bg: rgba(34,197,94,0.12);
    --amber: #f59e0b; --amber-bg: rgba(245,158,11,0.14);
    --red: #ef4444; --red-bg: rgba(239,68,68,0.14);
    --blue: #2563eb;
    --mono: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0;
    font-family: Inter, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    background: var(--bg); color: var(--text); line-height: 1.42;
    -webkit-font-smoothing: antialiased;
  }
  code, pre { font-family: var(--mono); }
  a { color: var(--cyan); text-decoration: none; }
  a:hover { text-decoration: underline; }

  /* Topbar + 4-Stage Pipeline */
  .topbar {
    background: rgba(13,20,35,0.97); backdrop-filter: blur(8px);
    border-bottom: 1px solid var(--border); padding: 9px 18px;
    position: sticky; top: 0; z-index: 40;
  }
  .topbar-inner {
    max-width: 1680px; margin: 0 auto;
    display: flex; align-items: center; justify-content: space-between;
    gap: 12px; flex-wrap: wrap;
  }
  .brand-row { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
  .brand-title { font-size: 16px; font-weight: 800; letter-spacing: 0.8px; color: #fff; }
  .badge {
    display: inline-flex; align-items: center; gap: 5px;
    padding: 2px 8px; border-radius: 5px;
    font-size: 11px; font-weight: 600; font-family: var(--mono);
    background: var(--surface2); color: var(--cyan); border: 1px solid var(--border);
  }
  .badge.ok { color: #86efac; background: var(--green-bg); border-color: rgba(34,197,94,0.35); }
  .badge.warn { color: #fde047; background: var(--amber-bg); border-color: rgba(245,158,11,0.38); }
  .badge.err { color: #fca5a5; background: var(--red-bg); border-color: rgba(239,68,68,0.40); }

  /* 4-Stage Scientific Pipeline Bar */
  .pipeline {
    display: flex; align-items: center; gap: 4px;
    background: #080d18; padding: 3px 6px; border-radius: 7px; border: 1px solid var(--border);
  }
  .pipe-step {
    font-family: var(--mono); font-size: 10.5px; font-weight: 700;
    padding: 3px 8px; border-radius: 5px; color: var(--dim);
    border: 1px solid transparent; transition: all 0.2s;
  }
  .pipe-step.done { color: #86efac; background: var(--green-bg); border-color: rgba(34,197,94,0.3); }
  .pipe-step.active { color: #e0f2fe; background: rgba(56,189,248,0.2); border-color: var(--cyan); }
  .pipe-step.warn { color: #fde047; background: var(--amber-bg); border-color: rgba(245,158,11,0.45); }
  .pipe-arr { color: var(--border2); font-size: 11px; font-family: var(--mono); }

  .tabs { display: flex; gap: 3px; background: #080d18; padding: 3px; border-radius: 7px; border: 1px solid var(--border); }
  .tab-btn {
    background: transparent; color: var(--muted); border: none;
    padding: 5px 12px; border-radius: 5px; font-size: 12px; font-weight: 600; cursor: pointer;
  }
  .tab-btn:hover { color: var(--text); }
  .tab-btn.active { background: var(--blue); color: #fff; }

  .container { max-width: 1680px; margin: 0 auto; padding: 12px 18px 28px; }
  .tab-panel { display: none; }
  .tab-panel.active { display: block; }

  /* 3-Pane Research IDE Grid */
  .ide-grid {
    display: grid;
    grid-template-columns: minmax(360px, 1.12fr) minmax(430px, 1.38fr) minmax(300px, 0.90fr);
    gap: 12px; align-items: start;
  }
  @media (max-width: 1220px) { .ide-grid { grid-template-columns: 1fr 1fr; } }
  @media (max-width: 860px) { .ide-grid { grid-template-columns: 1fr; } }

  .panel {
    background: var(--surface); border: 1px solid var(--border);
    border-radius: 9px; padding: 11px 13px; margin-bottom: 12px;
  }
  .panel-head {
    display: flex; align-items: center; justify-content: space-between; gap: 8px; flex-wrap: wrap;
    margin-bottom: 9px;
  }
  .panel-title {
    font-size: 11px; font-weight: 700; text-transform: uppercase; letter-spacing: 0.65px;
    color: #cbd5e1; font-family: var(--mono);
  }

  /* Camera Stage */
  .video-shell {
    position: relative; background: #02050c; border: 1px solid var(--border);
    border-radius: 7px; overflow: hidden; aspect-ratio: 16 / 9;
    display: flex; align-items: center; justify-content: center;
  }
  .video-shell canvas { width: 100%; height: 100%; object-fit: contain; display: block; }
  .hud-tl { position: absolute; top: 8px; left: 8px; display: flex; gap: 5px; flex-wrap: wrap; pointer-events: none; }
  .hud-tr { position: absolute; top: 8px; right: 8px; display: flex; gap: 5px; pointer-events: none; }
  .hud-pill {
    background: rgba(7,11,19,0.84); backdrop-filter: blur(4px);
    border: 1px solid rgba(148,163,184,0.22); color: #f1f5f9;
    padding: 2px 7px; border-radius: 4px; font-size: 10.5px; font-weight: 600; font-family: var(--mono);
  }
  .hud-pill.ok { border-color: rgba(34,197,94,0.55); color: #86efac; }
  .hud-pill.warn { border-color: rgba(245,158,11,0.65); color: #fde047; }
  .hud-bot { position: absolute; bottom: 0; left: 0; right: 0; padding: 6px 10px 5px; background: linear-gradient(transparent, rgba(2,6,23,0.92)); pointer-events: none; }
  .hud-bot-row { display: flex; justify-content: space-between; font-size: 10.5px; font-family: var(--mono); color: #cbd5e1; margin-bottom: 3px; }
  .bar-bg { height: 3px; background: rgba(255,255,255,0.16); border-radius: 99px; overflow: hidden; }
  .bar-fg { height: 100%; width: 0%; background: linear-gradient(90deg, var(--cyan), var(--green)); transition: width 0.2s; }

  .cam-grid-4 { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; }
  .cam-grid-4 .cam-cell { position: relative; background: #02050c; border: 1px solid var(--border); border-radius: 6px; overflow: hidden; aspect-ratio: 16 / 9; cursor: pointer; }
  .cam-grid-4 canvas { width: 100%; height: 100%; object-fit: cover; display: block; }

  .thumb-strip { display: grid; grid-template-columns: repeat(4, 1fr); gap: 6px; margin-top: 7px; }
  .thumb { position: relative; background: #02050c; border: 1.5px solid var(--border); border-radius: 6px; overflow: hidden; cursor: pointer; padding: 0; aspect-ratio: 16 / 9; }
  .thumb.active { border-color: var(--cyan); }
  .thumb canvas { width: 100%; height: 100%; object-fit: cover; display: block; }
  .thumb span { position: absolute; bottom: 3px; left: 4px; background: rgba(2,6,23,0.85); color: #e2e8f0; font-size: 9.5px; font-family: var(--mono); padding: 1px 5px; border-radius: 3px; }

  /* Radar + ActiveProbe Split */
  .explore-split { display: grid; grid-template-columns: 1.05fr 1fr; gap: 8px; margin-top: 9px; }
  @media (max-width: 680px) { .explore-split { grid-template-columns: 1fr; } }
  .sub-box { background: var(--codebg); border: 1px solid var(--border); border-radius: 7px; padding: 8px 9px; }
  .sub-head { display: flex; justify-content: space-between; align-items: center; font-size: 10px; font-weight: 700; text-transform: uppercase; letter-spacing: 0.45px; color: var(--muted); font-family: var(--mono); margin-bottom: 5px; }
  .radar-svg { width: 100%; height: 158px; display: block; }
  .probe-kv { font-family: var(--mono); font-size: 11px; display: flex; flex-direction: column; gap: 4px; }
  .probe-row { display: flex; justify-content: space-between; gap: 6px; border-bottom: 1px solid rgba(30,45,72,0.55); padding-bottom: 3px; }
  .probe-k { color: var(--muted); }
  .probe-v { color: #e2e8f0; font-weight: 600; text-align: right; }

  /* Controls & Inputs */
  .ctrl-row { display: flex; gap: 6px; flex-wrap: wrap; align-items: center; margin-bottom: 8px; }
  button {
    background: var(--surface2); color: var(--text); border: 1px solid var(--border2);
    border-radius: 6px; padding: 5px 10px; font-size: 11.5px; font-weight: 600;
    font-family: var(--mono); cursor: pointer; transition: all 0.14s;
  }
  button:hover { border-color: var(--cyan); background: #1a2742; }
  button:disabled { opacity: 0.42; cursor: not-allowed; }
  button.primary { background: var(--blue); border-color: #3b82f6; color: #fff; }
  button.primary:hover { background: #1d4ed8; }
  button.ok { background: #15803d; border-color: #22c55e; color: #fff; }
  button.warn { background: #b45309; border-color: #f59e0b; color: #fff; }
  button.sm { padding: 3px 7px; font-size: 10.5px; }

  select, input[type="text"], textarea.prompt-in {
    background: var(--codebg); color: var(--text); border: 1px solid var(--border);
    border-radius: 6px; padding: 6px 8px; font-size: 11.5px; font-family: var(--mono); width: 100%;
  }
  textarea.prompt-in { resize: vertical; min-height: 46px; line-height: 1.38; }

  /* Iteration Timeline Ribbon */
  .iter-strip { display: flex; gap: 6px; flex-wrap: wrap; margin: 8px 0; }
  .iter-chip {
    display: inline-flex; align-items: center; gap: 5px;
    padding: 4px 9px; border-radius: 6px; cursor: pointer;
    font-family: var(--mono); font-size: 11px; font-weight: 600;
    background: var(--surface2); border: 1px solid var(--border); color: var(--muted);
  }
  .iter-chip.active { border-color: var(--cyan); color: #fff; background: rgba(56,189,248,0.16); }
  .iter-chip .dot { width: 7px; height: 7px; border-radius: 50%; background: var(--dim); }
  .iter-chip.pass .dot { background: var(--green); }
  .iter-chip.fail .dot { background: var(--red); }

  /* Code / Diff / Subroutines IDE */
  .subtabs { display: flex; gap: 4px; border-bottom: 1px solid var(--border); padding-bottom: 6px; margin-bottom: 7px; align-items: center; justify-content: space-between; flex-wrap: wrap; }
  .subtab-group { display: flex; gap: 4px; }
  .subtab { background: transparent; border: 1px solid transparent; color: var(--muted); padding: 3px 9px; font-size: 11px; border-radius: 5px; }
  .subtab.active { background: var(--surface2); border-color: var(--border2); color: #fff; }

  textarea.code-editor {
    width: 100%; height: 390px; resize: vertical;
    background: var(--codebg); color: #e2e8f0;
    border: 1px solid var(--border); border-radius: 7px;
    padding: 10px 11px; font-family: var(--mono); font-size: 11.5px; line-height: 1.48;
    tab-size: 4; white-space: pre; overflow: auto;
  }
  .diff-view {
    height: 390px; overflow: auto; background: var(--codebg);
    border: 1px solid var(--border); border-radius: 7px;
    padding: 9px 10px; font-family: var(--mono); font-size: 11px; line-height: 1.46;
    white-space: pre;
  }
  .diff-add { color: #86efac; background: rgba(34,197,94,0.10); display: block; }
  .diff-del { color: #fca5a5; background: rgba(239,68,68,0.12); display: block; }
  .diff-hunk { color: var(--cyan); display: block; font-weight: 700; }
  .diff-ctx { color: var(--muted); display: block; }

  /* Full-Transparency LLM Prompt/Response & Executed Code Views */
  .run-strip { display: flex; gap: 5px; flex-wrap: wrap; margin-bottom: 6px; }
  .io-meta {
    display: flex; align-items: center; justify-content: space-between; gap: 6px; flex-wrap: wrap;
    background: var(--surface2); border: 1px solid var(--border); border-radius: 6px;
    padding: 5px 8px; margin-bottom: 6px; font-family: var(--mono); font-size: 10.5px; color: var(--muted);
  }
  .io-stack { display: grid; grid-template-columns: 1fr; gap: 7px; }
  .io-block { background: var(--codebg); border: 1px solid var(--border); border-radius: 7px; overflow: hidden; }
  .io-head {
    display: flex; align-items: center; justify-content: space-between; gap: 6px;
    padding: 5px 9px; background: #0a101d; border-bottom: 1px solid var(--border);
    font-family: var(--mono); font-size: 10.5px; font-weight: 700; color: var(--cyan);
  }
  .io-head.resp { color: #c084fc; }
  .io-head.exec { color: #86efac; }
  .io-pre {
    margin: 0; padding: 8px 10px; max-height: 175px; overflow: auto;
    font-family: var(--mono); font-size: 11px; line-height: 1.45; color: #e2e8f0;
    white-space: pre-wrap; word-break: break-word;
  }
  .io-pre.code-snap { max-height: 315px; white-space: pre; word-break: normal; }
  .ln-num { display: inline-block; width: 32px; color: var(--dim); user-select: none; text-align: right; margin-right: 10px; }

  /* Embodiment Switcher, RAX Matrix & Side-by-Side Policy Comparator */
  .emb-select { width: auto; min-width: 215px; max-width: 265px; padding: 3px 8px; font-size: 11px; border-color: rgba(56,189,248,0.45); background: #0a1222; color: #e0f2fe; }
  .http-bar {
    display: grid; grid-template-columns: 1.4fr 150px 1.1fr auto; gap: 7px; align-items: center;
    background: var(--codebg); border: 1px solid var(--border); border-radius: 7px; padding: 8px 10px; margin-top: 8px;
  }
  @media (max-width: 980px) { .http-bar { grid-template-columns: 1fr; } }
  input.txt-in {
    width: 100%; background: var(--surface2); color: var(--text);
    border: 1px solid var(--border2); border-radius: 6px; padding: 5px 8px;
    font-family: var(--mono); font-size: 11px;
  }
  .cmp-bar { display: flex; gap: 7px; flex-wrap: wrap; align-items: center; margin-bottom: 8px; }
  .cmp-bar select { width: auto; min-width: 170px; }
  .hm-cell { cursor: pointer; text-align: center; font-weight: 700; border-radius: 4px; transition: opacity 0.12s; }
  .hm-cell:hover { outline: 1px solid var(--cyan); }
  .hm-hi { background: rgba(34,197,94,0.18); color: #86efac; }
  .hm-mid { background: rgba(245,158,11,0.16); color: #fde047; }
  .hm-zero { background: rgba(239,68,68,0.22); color: #fca5a5; }
  .hm-sel { outline: 2px solid var(--cyan); }
  .cmp-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 8px; }
  @media (max-width: 1050px) { .cmp-grid { grid-template-columns: 1fr; } }
  .cmp-pre { max-height: 410px; }

  /* Right Pane: Oracle + Journal + Console */
  .metric-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; margin-bottom: 8px; }
  .metric { background: var(--codebg); border: 1px solid var(--border); border-radius: 6px; padding: 6px 8px; }
  .metric-k { font-size: 9.5px; text-transform: uppercase; letter-spacing: 0.45px; color: var(--muted); font-family: var(--mono); }
  .metric-v { font-size: 12.5px; font-weight: 700; font-family: var(--mono); margin-top: 2px; }

  .journal-box { max-height: 195px; overflow: auto; background: var(--codebg); border: 1px solid var(--border); border-radius: 7px; }
  table { width: 100%; border-collapse: collapse; font-family: var(--mono); font-size: 11px; }
  th, td { border-bottom: 1px solid rgba(30,45,72,0.7); padding: 5px 7px; text-align: left; vertical-align: middle; }
  th { color: var(--muted); font-size: 10px; text-transform: uppercase; background: #0a101d; position: sticky; top: 0; }
  tr.row-hi { background: rgba(56,189,248,0.10); }
  tr.row-warn { background: rgba(245,158,11,0.14); color: #fde047; }
  tr.row-ok { background: rgba(34,197,94,0.10); color: #86efac; }

  .console-box {
    height: 205px; overflow: auto; background: #04070e;
    border: 1px solid var(--border); border-radius: 7px;
    padding: 7px 9px; font-family: var(--mono); font-size: 10.5px; line-height: 1.48;
  }
  .log-ln { margin-bottom: 2px; word-break: break-word; }
  .log-t { color: var(--dim); margin-right: 5px; }
  .log-tag { font-weight: 700; margin-right: 5px; }
  .tag-PROBE { color: var(--cyan); }
  .tag-SYNTH, .tag-AST { color: #c084fc; }
  .tag-EXEC { color: #86efac; }
  .tag-ITER, .tag-LOOP { color: #fde047; }
  .tag-ERROR { color: #fca5a5; }
  .tag-INIT, .tag-EDIT, .tag-RESET { color: var(--muted); }

  /* Archive & Governance Tables */
  .two-col { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
  @media (max-width: 980px) { .two-col { grid-template-columns: 1fr; } }
  .chips { display: flex; flex-wrap: wrap; gap: 5px; margin-top: 6px; }
  .chip { font-size: 10.5px; padding: 2px 7px; border-radius: 4px; background: var(--surface2); color: #cbd5e1; border: 1px solid var(--border); font-family: var(--mono); }
  .chip.ok { background: var(--green-bg); color: #86efac; border-color: rgba(34,197,94,0.3); }
  .chip.miss { background: var(--red-bg); color: #fca5a5; border-color: rgba(239,68,68,0.35); }
  .alert-bar { margin-top: 8px; padding: 8px 10px; border-radius: 7px; background: var(--amber-bg); border: 1px solid rgba(245,158,11,0.5); display: none; align-items: center; justify-content: space-between; gap: 8px; flex-wrap: wrap; }
  .alert-bar.show { display: flex; }
  .pulse { display: inline-block; width: 7px; height: 7px; border-radius: 50%; background: var(--green); animation: blink 1.3s infinite; }
  .spin { display: inline-block; width: 10px; height: 10px; border: 2px solid rgba(56,189,248,0.3); border-top-color: var(--cyan); border-radius: 50%; animation: spin 0.8s linear infinite; vertical-align: -1px; margin-right: 4px; }
  @keyframes blink { 0%,100%{opacity:1} 50%{opacity:0.25} }
  @keyframes spin { to{transform:rotate(360deg)} }
</style>
</head>
<body>

<div class="topbar">
  <div class="topbar-inner">
    <div class="brand-row">
      <span class="brand-title">TRAX</span>
      <select id="emb-top-select" class="emb-select" onchange="switchEmbodiment(this.value)" title="Live robot embodiment / host target switcher"></select>
      <span id="pill-live" class="badge ok"><span class="pulse"></span>CONNECTING</span>
      <span id="pill-authority" class="badge">host=...</span>
      <span id="pill-evidence" class="badge">evidence=simulator</span>
    </div>

    <!-- Real 4-Stage Closed-Loop State Machine -->
    <div class="pipeline" id="pipeline-bar">
      <span id="ps-explore" class="pipe-step">01 EXPLORE</span>
      <span class="pipe-arr">&rarr;</span>
      <span id="ps-synth" class="pipe-step">02 SYNTHESIZE PY</span>
      <span class="pipe-arr">&rarr;</span>
      <span id="ps-exec" class="pipe-step">03 AST EXEC</span>
      <span class="pipe-arr">&rarr;</span>
      <span id="ps-iter" class="pipe-step">04 ITERATE</span>
    </div>

    <div class="tabs">
      <button class="tab-btn active" data-tab="lab" onclick="switchTab('lab')">Closed-Loop Lab</button>
      <button class="tab-btn" data-tab="m1" onclick="switchTab('m1')">Embodiments &amp; RAX Matrix</button>
      <button class="tab-btn" data-tab="gov" onclick="switchTab('gov')">Governance</button>
    </div>
  </div>
</div>

<div class="container">
  <!-- TAB 1: CLOSED-LOOP RESEARCH IDE -->
  <div id="tab-lab" class="tab-panel active">
    <div class="ide-grid">

      <!-- PANE 1: LIVE ROBOT EMBODIMENT & STAGE 1 EXPLORATION -->
      <div>
        <div class="panel">
          <div class="panel-head">
            <span class="panel-title">01 &middot; Embodiment &amp; Active Exploration</span>
            <div style="display:flex;gap:5px;align-items:center">
              <button id="btn-mode-single" class="sm" onclick="setCamMode('single')">Single</button>
              <button id="btn-mode-grid" class="sm" onclick="setCamMode('grid')">Quad</button>
              <button class="sm" onclick="resetEnv()">Reset Sim</button>
            </div>
          </div>

          <div id="cam-stage"></div>
          <div id="cam-thumbs" class="thumb-strip"></div>

          <div class="explore-split">
            <div class="sub-box">
              <div class="sub-head">
                <span>2D World Radar</span>
                <button class="sm" onclick="clearTrail()">Clear</button>
              </div>
              <svg id="radar-svg" class="radar-svg" viewBox="0 0 360 220"></svg>
            </div>
            <div class="sub-box">
              <div class="sub-head">
                <span>ActiveProbe Telemetry</span>
                <div style="display:flex;gap:5px;align-items:center">
                  <button class="sm" onclick="inspectProbeCode()">Probe Py</button>
                  <span id="probe-badge" class="badge">UNPROBED</span>
                </div>
              </div>
              <div id="probe-kv" class="probe-kv">
                <div class="probe-row"><span class="probe-k">authority</span><span id="pk-auth" class="probe-v">-</span></div>
                <div class="probe-row"><span class="probe-k">sdk_methods</span><span id="pk-sdk" class="probe-v">-</span></div>
                <div class="probe-row"><span class="probe-k">objects</span><span id="pk-objs" class="probe-v">-</span></div>
                <div class="probe-row"><span class="probe-k">cup_world_m</span><span id="pk-pos" class="probe-v">-</span></div>
                <div class="probe-row"><span class="probe-k">l2_offset</span><span id="pk-l2" class="probe-v">-</span></div>
                <div class="probe-row"><span class="probe-k">probe_wall</span><span id="pk-wall" class="probe-v">-</span></div>
              </div>
            </div>
          </div>

          <div id="confirm-alert" class="alert-bar">
            <span style="font-size:11.5px;font-family:var(--mono);color:#fde047"><strong>AMBIGUITY GATE:</strong> select target object_id</span>
            <div id="confirm-btns" style="display:flex;gap:6px"></div>
          </div>
        </div>
      </div>

      <!-- PANE 2: PYTHON SYNTHESIS, AST PREFLIGHT & ITERATION DIFF -->
      <div>
        <div class="panel">
          <div class="panel-head">
            <span class="panel-title">02 &middot; Python Synthesis, AST Gate &amp; Iteration</span>
            <span id="loop-status-pill" class="badge">IDLE</span>
          </div>

          <div style="display:grid;grid-template-columns:1fr 195px;gap:7px;margin-bottom:7px">
            <textarea id="task-instruction" class="prompt-in" rows="2"></textarea>
            <div style="display:flex;flex-direction:column;gap:5px">
              <select id="scenario-select"></select>
              <button class="sm" onclick="resetSession()">New Clean Session</button>
            </div>
          </div>

          <div class="ctrl-row">
            <button id="btn-probe" onclick="runProbe()">1. Explore / Probe</button>
            <button id="btn-synth" onclick="runSynthesize()">2. Synthesize Py (LLM)</button>
            <button id="btn-exec" class="ok" onclick="runExecute()">3. Execute Py</button>
            <button id="btn-auto" class="primary" onclick="runClosedLoop()">Run Closed Loop (1&rarr;4)</button>
          </div>

          <div id="iter-strip" class="iter-strip"></div>

          <div class="subtabs">
            <div class="subtab-group">
              <button class="subtab active" data-sub="code" onclick="switchCodeTab('code')">Python Policy</button>
              <button class="subtab" data-sub="llm" onclick="switchCodeTab('llm')">LLM Prompt / Response <span id="llm-tab-cnt" style="opacity:0.8">(0)</span></button>
              <button class="subtab" data-sub="exec" onclick="switchCodeTab('exec')">Executed Code <span id="exec-tab-cnt" style="opacity:0.8">(0)</span></button>
              <button class="subtab" data-sub="diff" onclick="switchCodeTab('diff')">Unified Diff</button>
              <button class="subtab" data-sub="subr" onclick="switchCodeTab('subr')">Subroutines</button>
            </div>
            <div style="display:flex;gap:5px;align-items:center">
              <span id="ast-badge" class="badge">-</span>
              <span id="sha-badge" class="badge">sha256:-</span>
            </div>
          </div>

          <div id="pane-code">
            <textarea id="code-editor" class="code-editor" spellcheck="false"></textarea>
          </div>
          <div id="pane-llm" style="display:none">
            <div id="llm-call-strip" class="run-strip"></div>
            <div id="llm-meta-bar" class="io-meta">No LLM call recorded in this session yet. Click 2. Synthesize Py (LLM) or Run Closed Loop.</div>
            <div class="io-stack">
              <div class="io-block">
                <div class="io-head">
                  <span>FULL PROMPT SENT TO LLM</span>
                  <div style="display:flex;gap:6px;align-items:center">
                    <span id="llm-prompt-len" style="color:var(--muted);font-weight:500">0 chars</span>
                    <button class="sm" onclick="copyElem('llm-prompt-pre')">Copy</button>
                  </div>
                </div>
                <pre id="llm-prompt-pre" class="io-pre">(no LLM prompt yet)</pre>
              </div>
              <div class="io-block">
                <div class="io-head resp">
                  <span>RAW RESPONSE RETURNED BY LLM</span>
                  <div style="display:flex;gap:6px;align-items:center">
                    <span id="llm-resp-len" style="color:var(--muted);font-weight:500">0 chars</span>
                    <button class="sm" onclick="copyElem('llm-resp-pre')">Copy</button>
                  </div>
                </div>
                <pre id="llm-resp-pre" class="io-pre">(no LLM response yet)</pre>
              </div>
            </div>
          </div>
          <div id="pane-exec" style="display:none">
            <div id="exec-run-strip" class="run-strip"></div>
            <div id="exec-meta-bar" class="io-meta">No Python code has been executed on the host in this session yet.</div>
            <div class="io-block">
              <div class="io-head exec">
                <span id="exec-code-title">EXACT EXECUTED PYTHON SNAPSHOT</span>
                <div style="display:flex;gap:6px;align-items:center">
                  <span id="exec-code-sha" style="color:var(--muted);font-weight:500">-</span>
                  <button class="sm" onclick="copyExecCode()">Copy</button>
                </div>
              </div>
              <pre id="exec-code-pre" class="io-pre code-snap">(no executed Python snapshot yet)</pre>
            </div>
          </div>
          <div id="pane-diff" style="display:none">
            <div id="diff-view" class="diff-view"></div>
          </div>
          <div id="pane-subr" style="display:none">
            <div id="subr-view" class="diff-view"></div>
          </div>
        </div>
      </div>

      <!-- PANE 3: SQLITE JOURNAL, 4-LAYER ATTRIBUTION & LOOP CONSOLE -->
      <div>
        <div class="panel">
          <div class="panel-head">
            <span class="panel-title">03 &middot; Oracle &amp; 4-Layer Attribution</span>
            <span id="verdict-pill" class="badge">PENDING</span>
          </div>
          <div class="metric-grid">
            <div class="metric"><div class="metric-k">Attribution Layer</div><div id="m-layer" class="metric-v">-</div></div>
            <div class="metric"><div class="metric-k">Failure Code</div><div id="m-code" class="metric-v">-</div></div>
            <div class="metric"><div class="metric-k">Oracle Fill / Err</div><div id="m-oracle" class="metric-v">-</div></div>
            <div class="metric"><div class="metric-k">Brew / Dup / Rec</div><div id="m-brew" class="metric-v">-</div></div>
          </div>

          <div class="panel-head" style="margin-top:10px;margin-bottom:6px">
            <span class="panel-title">SQLite Intent Journal &amp; SDK Trace</span>
            <span id="journal-count" class="badge">0 ops</span>
          </div>
          <div class="journal-box">
            <table>
              <thead><tr><th>#</th><th>Action / Method</th><th>State / Receipt</th></tr></thead>
              <tbody id="journal-tbody">
                <tr><td colspan="3" style="color:var(--dim)">No execution yet.</td></tr>
              </tbody>
            </table>
          </div>

          <div class="panel-head" style="margin-top:10px;margin-bottom:6px">
            <span class="panel-title">04 &middot; Closed-Loop Console</span>
            <span id="iter-meta" class="badge">0 iters</span>
          </div>
          <div id="console-box" class="console-box"></div>
        </div>
      </div>

    </div>
  </div>

  <!-- TAB 2: MULTI-EMBODIMENT RAX COVERAGE MATRIX & CROSS-METHOD COMPARATOR -->
  <div id="tab-m1" class="tab-panel">
    <div class="panel">
      <div class="panel-head">
        <span class="panel-title">RAX (arXiv:2610.22496) Multi-Embodiment Coverage Matrix &mdash; Simulation &amp; Physical Robots</span>
        <div style="display:flex;gap:5px;align-items:center">
          <button id="mf-all" class="sm primary" onclick="setMatrixFilter('all')">All (11)</button>
          <button id="mf-sim" class="sm" onclick="setMatrixFilter('simulation')">Simulation</button>
          <button id="mf-phys" class="sm" onclick="setMatrixFilter('physical')">Physical Robot</button>
        </div>
      </div>
      <div style="overflow:auto">
        <table>
          <thead>
            <tr>
              <th>Embodiment Family</th>
              <th>Robot / Suite</th>
              <th>Domain / Evidence</th>
              <th>RAX Paper Scope</th>
              <th>TRAX Status &amp; Verified Result</th>
              <th>Action</th>
            </tr>
          </thead>
          <tbody id="rax-matrix-tbody"></tbody>
        </table>
      </div>

      <div class="http-bar">
        <input id="http-host-url" class="txt-in" placeholder="Physical robot or remote sim HTTP host (e.g. http://192.168.1.50:8890)" value="http://127.0.0.1:8880"/>
        <select id="http-evidence">
          <option value="physical_robot">physical_robot</option>
          <option value="simulator">simulator</option>
          <option value="recorded_data">recorded_data</option>
        </select>
        <input id="http-label" class="txt-in" placeholder="Embodiment label (e.g. Franka FR3 / Real SO-101 / RB-Y1)"/>
        <button class="primary" onclick="connectCustomHost()">Probe &amp; Switch Live</button>
      </div>
      <div id="http-connect-msg" style="font-family:var(--mono);font-size:11px;color:var(--muted);margin-top:5px"></div>
    </div>

    <div class="panel">
      <div class="panel-head">
        <span class="panel-title">Interactive Cross-Method Task &amp; Generated Python Policy Comparator (Real Archived Runs)</span>
        <div style="display:flex;gap:5px;align-items:center">
          <button id="cmp-mode-split" class="sm primary" onclick="setCompareMode('split')">Side-by-Side Code</button>
          <button id="cmp-mode-diff" class="sm" onclick="setCompareMode('diff')">Unified Diff (B &rarr; A)</button>
        </div>
      </div>

      <div class="cmp-bar">
        <select id="cmp-suite" onchange="onCompareSuiteChange()"></select>
        <select id="cmp-task" onchange="loadPolicyComparison()"></select>
        <span style="font-family:var(--mono);font-size:11px;color:var(--cyan)">Method A:</span>
        <select id="cmp-left" onchange="loadPolicyComparison()"></select>
        <span style="font-family:var(--mono);font-size:11px;color:#c084fc">vs Method B:</span>
        <select id="cmp-right" onchange="loadPolicyComparison()"></select>
      </div>

      <div id="cmp-heatmap" style="overflow:auto;margin-bottom:8px"></div>
      <div id="cmp-task-banner" class="io-meta">Select a benchmark suite and task above to compare exact generated Python policies side-by-side.</div>

      <div id="cmp-split-view" class="cmp-grid">
        <div class="io-block">
          <div class="io-head">
            <span id="cmp-left-head">Method A</span>
            <span id="cmp-left-meta" style="color:var(--muted);font-weight:500">-</span>
          </div>
          <pre id="cmp-left-pre" class="io-pre code-snap cmp-pre"></pre>
        </div>
        <div class="io-block">
          <div class="io-head resp">
            <span id="cmp-right-head">Method B</span>
            <span id="cmp-right-meta" style="color:var(--muted);font-weight:500">-</span>
          </div>
          <pre id="cmp-right-pre" class="io-pre code-snap cmp-pre"></pre>
        </div>
      </div>

      <div id="cmp-diff-view" style="display:none;margin-top:8px">
        <div id="cmp-diff-box" class="diff-view" style="height:410px"></div>
      </div>
    </div>

    <div class="panel">
      <div class="panel-head">
        <span class="panel-title">Zero-Leakage Robosuite v4 (4 Tasks &times; 20 Held-Out Seeds = 480 Rollouts &middot; evidence=simulator)</span>
        <a href="robosuite_v2_report.html" target="_blank" class="badge">HTML Report &nearr;</a>
      </div>
      <div id="m1-robosuite"></div>
    </div>

    <div class="two-col">
      <div class="panel">
        <div class="panel-head">
          <span class="panel-title">Official LIBERO-Spatial 10/10 (host_state_api &middot; n=100)</span>
          <span class="badge ok">TRAX 97/100 (p=0.0386)</span>
        </div>
        <div id="m1-libero-spatial"></div>
      </div>
      <div class="panel">
        <div class="panel-head">
          <span class="panel-title">Zero-Tuning Blind LIBERO-Object 10/10 (Frozen 108648a &middot; n=100)</span>
          <span class="badge ok">0 Collapsed Tasks</span>
        </div>
        <div id="m1-libero-object"></div>
      </div>
    </div>

    <div class="two-col">
      <div class="panel">
        <div class="panel-head">
          <span class="panel-title">CafeBench v2 (n=30 &middot; simulator + recorded_data)</span>
          <a href="cafebench_v2_report.html" target="_blank" class="badge">HTML Report &nearr;</a>
        </div>
        <div id="m1-cafebench"></div>
      </div>
      <div class="panel">
        <div class="panel-head">
          <span class="panel-title">Sim-to-Sim Stage-2 Adaptation (&le;5 Trials &middot; n=20)</span>
          <a href="sim2sim_report.html" target="_blank" class="badge">HTML Report &nearr;</a>
        </div>
        <div id="m1-sim2sim"></div>
      </div>
    </div>
  </div>

  <!-- TAB 3: GOVERNANCE -->
  <div id="tab-gov" class="tab-panel">
    <div class="two-col">
      <div class="panel">
        <div class="panel-head">
          <span class="panel-title">Host Capability Contract</span>
          <span id="cap-pill" class="badge">-</span>
        </div>
        <div id="cap-meta" style="font-family:var(--mono);font-size:11px;color:var(--muted);margin-bottom:8px"></div>
        <div style="font-size:10.5px;font-weight:700;font-family:var(--mono);color:#86efac">READY SDK METHODS</div>
        <div id="cap-ok-chips" class="chips"></div>
        <div style="font-size:10.5px;font-weight:700;font-family:var(--mono);color:#fca5a5;margin-top:11px">MISSING CAPABILITIES (OVEN GATE)</div>
        <div id="cap-miss-chips" class="chips"></div>
      </div>

      <div class="panel">
        <div class="panel-head">
          <span class="panel-title">Verified Skill Toolbox &amp; Release Ledger</span>
        </div>
        <div id="gov-skills"></div>
      </div>
    </div>
  </div>
</div>

<script>
let camList = [];
let activeCam = 'overview';
let camMode = 'single';
let pumpRunning = false;
let frameTimestamps = [];
let cupTrail = [];
let lastLiveScene = null;
let evoState = null;
let codeSubTab = 'code';
let editorDirty = false;
let lastLoadedIterSha = '';
let selectedLlmCallIdx = -1;
let selectedExecRunIdx = -1;
let lastLlmCount = 0;
let lastExecCount = 0;
let currentExecRawSource = '';
let embSnapshot = null;
let matrixFilter = 'all';
let compareMode = 'split';

async function api(path, opts) {
  const r = await fetch(path, opts);
  const data = await r.json();
  if (!r.ok) throw new Error(data.error || ('HTTP ' + r.status));
  return data;
}

function esc(s) {
  return String(s ?? '').replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}

function copyElem(id) {
  const el = document.getElementById(id);
  if (el && navigator.clipboard) navigator.clipboard.writeText(el.textContent || '');
}

function copyExecCode() {
  if (currentExecRawSource && navigator.clipboard) navigator.clipboard.writeText(currentExecRawSource);
}

function switchTab(name) {
  document.querySelectorAll('.tab-btn').forEach(b => b.classList.toggle('active', b.dataset.tab === name));
  document.querySelectorAll('.tab-panel').forEach(p => p.classList.toggle('active', p.id === 'tab-' + name));
}

function switchCodeTab(name) {
  codeSubTab = name;
  document.querySelectorAll('.subtab').forEach(b => b.classList.toggle('active', b.dataset.sub === name));
  for (const id of ['code', 'llm', 'exec', 'diff', 'subr']) {
    const el = document.getElementById('pane-' + id);
    if (el) el.style.display = name === id ? 'block' : 'none';
  }
}

function selectLlmCall(idx) {
  selectedLlmCallIdx = idx;
  if (evoState) renderLlmInspector(evoState);
}

function selectExecRun(idx) {
  selectedExecRunIdx = idx;
  if (evoState) renderExecutedCodeInspector(evoState);
}

function inspectProbeCode() {
  if (evoState && (evoState.executed_runs || []).length) {
    const idx = evoState.executed_runs.findIndex(r => r.stage === '01_explore_probe');
    if (idx >= 0) selectedExecRunIdx = idx;
  }
  switchCodeTab('exec');
  if (evoState) renderExecutedCodeInspector(evoState);
}

document.addEventListener('DOMContentLoaded', () => {
  const ed = document.getElementById('code-editor');
  if (ed) ed.addEventListener('input', () => { editorDirty = true; });
});

function setCamMode(mode) { camMode = mode; renderCameraShells(); }
function selectCam(name) { activeCam = name; camMode = 'single'; renderCameraShells(); }
function clearTrail() { cupTrail = []; if (lastLiveScene) renderSpatialRadar(lastLiveScene); }

function renderCameraShells() {
  const stage = document.getElementById('cam-stage');
  const thumbs = document.getElementById('cam-thumbs');
  if (!camList.length) {
    stage.innerHTML = '<div class="video-shell" style="color:var(--muted);font-family:var(--mono);font-size:11.5px">No camera stream</div>';
    thumbs.innerHTML = '';
    return;
  }
  if (!camList.includes(activeCam)) activeCam = camList[0];
  document.getElementById('btn-mode-single').style.borderColor = camMode === 'single' ? 'var(--cyan)' : 'var(--border2)';
  document.getElementById('btn-mode-grid').style.borderColor = camMode === 'grid' ? 'var(--cyan)' : 'var(--border2)';

  if (camMode === 'grid' && camList.length > 1) {
    stage.innerHTML = '<div class="cam-grid-4">' + camList.map(c => `
      <div class="cam-cell" onclick="selectCam('${c}')">
        <canvas data-cam-canvas="${c}" width="640" height="360"></canvas>
        <span class="hud-pill" style="position:absolute;top:6px;left:6px">${c}</span>
      </div>
    `).join('') + '</div>';
    thumbs.style.display = 'none';
  } else {
    stage.innerHTML = `
      <div class="video-shell">
        <canvas id="main-cam-canvas" data-cam-canvas="${activeCam}" width="800" height="450"></canvas>
        <div class="hud-tl">
          <span class="hud-pill" id="hud-cam">cam: ${activeCam}</span>
          <span class="hud-pill" id="hud-fps">-- FPS</span>
          <span class="hud-pill" id="hud-phase">phase: idle</span>
        </div>
        <div class="hud-tr"><span class="hud-pill" id="hud-l2">L2: --</span></div>
        <div class="hud-bot">
          <div class="hud-bot-row">
            <span id="hud-action">idle</span>
            <span id="hud-brew-txt">brew 0%</span>
          </div>
          <div class="bar-bg"><div id="hud-brew-bar" class="bar-fg"></div></div>
        </div>
      </div>`;
    if (camList.length > 1) {
      thumbs.style.display = 'grid';
      thumbs.innerHTML = camList.map(c => `
        <button class="thumb ${c === activeCam ? 'active' : ''}" onclick="selectCam('${c}')">
          <canvas data-thumb-canvas="${c}" width="320" height="180"></canvas>
          <span>${c}</span>
        </button>
      `).join('');
    } else {
      thumbs.style.display = 'none';
    }
  }
  if (!pumpRunning) startVideoPump();
}

function drawCamToCanvas(cam, canvas) {
  return new Promise(resolve => {
    const img = new Image();
    img.onload = () => {
      if (canvas && canvas.isConnected) {
        if (canvas.width !== img.naturalWidth && img.naturalWidth > 0) {
          canvas.width = img.naturalWidth;
          canvas.height = img.naturalHeight;
        }
        const ctx = canvas.getContext('2d');
        if (ctx) ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
      }
      resolve(true);
    };
    img.onerror = () => resolve(false);
    img.src = 'api/camera/' + encodeURIComponent(cam) + '?t=' + Date.now();
  });
}

async function startVideoPump() {
  pumpRunning = true;
  let tick = 0;
  while (true) {
    const t0 = performance.now();
    const onLab = document.getElementById('tab-lab').classList.contains('active');
    if (onLab && camList.length) {
      if (camMode === 'grid' && camList.length > 1) {
        await Promise.all(camList.map(c => {
          const cv = document.querySelector(`canvas[data-cam-canvas="${c}"]`);
          return cv ? drawCamToCanvas(c, cv) : Promise.resolve();
        }));
      } else {
        const mainCv = document.getElementById('main-cam-canvas');
        if (mainCv) {
          const ok = await drawCamToCanvas(activeCam, mainCv);
          if (ok) {
            const now = performance.now();
            frameTimestamps.push(now);
            frameTimestamps = frameTimestamps.filter(t => now - t <= 2000);
            const el = document.getElementById('hud-fps');
            if (el) el.textContent = (frameTimestamps.length / 2).toFixed(1) + ' FPS';
          }
        }
        tick = (tick + 1) % 5;
        if (tick === 0 && camList.length > 1) {
          Promise.all(camList.map(c => {
            const tv = document.querySelector(`canvas[data-thumb-canvas="${c}"]`);
            return tv ? drawCamToCanvas(c, tv) : Promise.resolve();
          }));
        }
      }
    }
    const dt = performance.now() - t0;
    const busy = evoState && evoState.busy;
    await new Promise(r => setTimeout(r, Math.max(25, (busy ? 95 : 150) - dt)));
  }
}

function worldToRadar(x, y) {
  const px = 30 + ((x - 0.22) / 0.43) * 300;
  const py = 110 - (y / 0.32) * 95;
  return [Math.max(12, Math.min(348, px)), Math.max(12, Math.min(208, py))];
}

function renderSpatialRadar(obs) {
  const svg = document.getElementById('radar-svg');
  if (!svg) return;
  const raw = (obs && obs.raw_snapshot) || {};
  const objs = raw.objects || [];
  const l2 = raw.l2_pouring_safety || {};
  const filled = !!(raw.cup_filled || (raw.coffee_machine && raw.coffee_machine.cup_filled));

  const [sx, sy] = worldToRadar(0.45, 0.0);
  const [p1x, p1y] = worldToRadar(0.38, -0.17);
  const [p2x, p2y] = worldToRadar(0.38, 0.17);
  const [nzx, nzy] = worldToRadar(0.56, 0.22);

  const cup = objs.find(o => o.semantic_class === 'dine_in_red') || objs.find(o => o.semantic_class === 'takeout_green');
  if (cup && Array.isArray(cup.center_world_m)) {
    const [cx, cy] = worldToRadar(cup.center_world_m[0], cup.center_world_m[1]);
    const prev = cupTrail[cupTrail.length - 1];
    if (!prev || Math.hypot(cx - prev[0], cy - prev[1]) > 2.5) {
      cupTrail.push([cx, cy]);
      if (cupTrail.length > 60) cupTrail.shift();
    }
  }
  const trail = cupTrail.length > 1
    ? `<polyline fill="none" stroke="#38bdf8" stroke-width="2" stroke-dasharray="4 3" opacity="0.75" points="${cupTrail.map(p => p[0].toFixed(1)+','+p[1].toFixed(1)).join(' ')}"/>`
    : '';

  const marks = objs
    .filter(o => o.semantic_class === 'dine_in_red' || o.semantic_class === 'takeout_green')
    .map(o => {
      const pos = o.center_world_m || o.world || [0.33, 0];
      const [px, py] = worldToRadar(pos[0], pos[1]);
      const col = o.semantic_class === 'dine_in_red' ? '#f43f5e' : '#10b981';
      const ring = filled && o.semantic_class === 'dine_in_red' ? '#facc15' : '#f8fafc';
      return `<g>
        <circle cx="${px.toFixed(1)}" cy="${py.toFixed(1)}" r="8" fill="${col}" stroke="${ring}" stroke-width="2"/>
        <text x="${px.toFixed(1)}" y="${(py+17).toFixed(1)}" fill="#e2e8f0" font-size="8.5" font-family="monospace" text-anchor="middle">${o.object_id || o.id} (${(pos[0]*100).toFixed(1)},${(pos[1]*100).toFixed(1)}cm)</text>
      </g>`;
    }).join('');

  const spCol = l2.safe_to_pour ? '#22c55e' : '#f59e0b';
  svg.innerHTML = `
    <rect x="4" y="4" width="352" height="212" rx="6" fill="#070c18" stroke="#1e2d48"/>
    <line x1="24" y1="110" x2="336" y2="110" stroke="#17233b" stroke-dasharray="2 4"/>
    <line x1="${sx.toFixed(1)}" y1="14" x2="${sx.toFixed(1)}" y2="206" stroke="#17233b" stroke-dasharray="2 4"/>
    <rect x="${(nzx-26).toFixed(1)}" y="${(nzy-15).toFixed(1)}" width="60" height="30" rx="4" fill="rgba(239,68,68,0.12)" stroke="#ef4444" stroke-dasharray="3 2"/>
    <text x="${(nzx+4).toFixed(1)}" y="${(nzy+3).toFixed(1)}" fill="#fca5a5" font-size="8.5" font-family="monospace" text-anchor="middle">NO-GO</text>
    <circle cx="${sx.toFixed(1)}" cy="${sy.toFixed(1)}" r="16" fill="rgba(56,189,248,0.08)" stroke="${spCol}" stroke-width="1.7" stroke-dasharray="3 2"/>
    <circle cx="${sx.toFixed(1)}" cy="${sy.toFixed(1)}" r="2.2" fill="${spCol}"/>
    <text x="${sx.toFixed(1)}" y="${(sy-21).toFixed(1)}" fill="${spCol}" font-size="8.5" font-weight="600" font-family="monospace" text-anchor="middle">SPOUT (L2)</text>
    <circle cx="${p1x.toFixed(1)}" cy="${p1y.toFixed(1)}" r="14" fill="rgba(34,197,94,0.08)" stroke="#22c55e" stroke-width="1.4" stroke-dasharray="3 2"/>
    <text x="${p1x.toFixed(1)}" y="${(p1y+24).toFixed(1)}" fill="#86efac" font-size="8.5" font-family="monospace" text-anchor="middle">SLOT_1</text>
    <circle cx="${p2x.toFixed(1)}" cy="${p2y.toFixed(1)}" r="12" fill="none" stroke="#475569" stroke-dasharray="2 3"/>
    <text x="${p2x.toFixed(1)}" y="${(p2y-15).toFixed(1)}" fill="#64748b" font-size="8" font-family="monospace" text-anchor="middle">SLOT_2</text>
    ${trail}${marks}
  `;
}

function renderDiffHtml(diffText) {
  if (!diffText || !diffText.trim()) {
    return '<span class="diff-ctx">No diff vs previous iteration (initial policy or identical source).</span>';
  }
  return diffText.split('\n').map(ln => {
    if (ln.startsWith('+++') || ln.startsWith('---') || ln.startsWith('@@')) return `<span class="diff-hunk">${esc(ln)}</span>`;
    if (ln.startsWith('+')) return `<span class="diff-add">${esc(ln)}</span>`;
    if (ln.startsWith('-')) return `<span class="diff-del">${esc(ln)}</span>`;
    return `<span class="diff-ctx">${esc(ln)}</span>`;
  }).join('');
}

function updatePipelineBar(evo) {
  const phase = evo.phase || 'idle';
  const hasProbe = !!evo.probe_report;
  const iters = evo.iterations || [];
  const cur = (evo.active_iter_index >= 0 && evo.active_iter_index < iters.length) ? iters[evo.active_iter_index] : null;
  const hasExec = !!(cur && cur.execution);
  const passed = !!(hasExec && cur.execution.task_passed);

  const setP = (id, cls) => { document.getElementById(id).className = 'pipe-step ' + cls; };
  setP('ps-explore', phase === 'exploring' ? 'active' : (hasProbe ? 'done' : ''));
  setP('ps-synth', phase === 'synthesizing' ? 'active' : (cur ? 'done' : ''));
  setP('ps-exec', phase === 'executing' ? 'active' : (passed ? 'done' : (hasExec ? 'warn' : '')));
  setP('ps-iter', phase === 'iterating' ? 'active' : (passed ? 'done' : (hasExec && !passed ? 'warn' : '')));
}

function renderEvolution(evo) {
  evoState = evo;
  updatePipelineBar(evo);

  const busy = !!evo.busy;
  for (const id of ['btn-probe', 'btn-synth', 'btn-exec', 'btn-auto']) {
    const el = document.getElementById(id);
    if (el) el.disabled = busy;
  }

  const lp = document.getElementById('loop-status-pill');
  if (busy) {
    lp.innerHTML = `<span class="spin"></span>${esc(evo.phase.toUpperCase())}`;
    lp.className = 'badge warn';
  } else {
    lp.textContent = (evo.phase || 'idle').toUpperCase();
    lp.className = 'badge ' + (evo.phase === 'verified' ? 'ok' : (evo.phase === 'attributed' ? 'warn' : ''));
  }

  const ti = document.getElementById('task-instruction');
  if (ti && !ti.matches(':focus') && evo.task_instruction && !ti.value) {
    ti.value = evo.task_instruction;
  }
  const sel = document.getElementById('scenario-select');
  if (sel && evo.scenario_profiles && sel.options.length === 0) {
    sel.innerHTML = Object.entries(evo.scenario_profiles)
      .map(([k, lbl]) => `<option value="${esc(k)}">${esc(lbl)}</option>`).join('');
    sel.value = evo.scenario_key || 'fault';
  }

  // Probe telemetry panel
  const pr = evo.probe_report;
  const pb = document.getElementById('probe-badge');
  if (pr) {
    pb.textContent = 'PROBED';
    pb.className = 'badge ok';
    document.getElementById('pk-auth').textContent = pr.execution_authority || '-';
    document.getElementById('pk-sdk').textContent = `${pr.allowed_methods_count} methods`;
    document.getElementById('pk-objs').textContent = `${(pr.objects_observed || []).length} observed`;
    const first = (pr.objects_observed || [])[0];
    document.getElementById('pk-pos').textContent = first && first.world_m ? `[${first.world_m.slice(0,3).join(', ')}]` : '-';
    document.getElementById('pk-l2').textContent = pr.l2_radial_offset_mm != null ? `${pr.l2_radial_offset_mm} mm` : '-';
    document.getElementById('pk-wall').textContent = `${pr.probe_wall_s}s`;
  }

  // Iteration timeline strip
  const iters = evo.iterations || [];
  document.getElementById('iter-meta').textContent = `${iters.length} iters`;
  document.getElementById('iter-strip').innerHTML = iters.map((it, idx) => {
    const ex = it.execution;
    const stCls = ex ? (ex.task_passed ? 'pass' : 'fail') : '';
    const actCls = idx === evo.active_iter_index ? 'active' : '';
    const tag = ex ? (ex.task_passed ? 'PASS' : 'FAIL') : it.origin;
    return `<button class="iter-chip ${stCls} ${actCls}" onclick="selectIter(${idx})">
      <span class="dot"></span>it=${idx} [${esc(tag)}]
    </button>`;
  }).join('') || '<span style="font-family:var(--mono);font-size:11px;color:var(--dim)">No iterations yet. Click 1. Explore / Probe or Run Closed Loop.</span>';

  const cur = (evo.active_iter_index >= 0 && evo.active_iter_index < iters.length) ? iters[evo.active_iter_index] : null;
  if (cur) {
    const ed = document.getElementById('code-editor');
    if (ed && (!editorDirty || cur.source_sha256 !== lastLoadedIterSha)) {
      ed.value = cur.source || '';
      lastLoadedIterSha = cur.source_sha256 || '';
      editorDirty = false;
    }
    const ast = cur.ast_check || {};
    const ab = document.getElementById('ast-badge');
    ab.textContent = ast.compliant ? 'AST: COMPLIANT' : `AST: ${(ast.violations||[]).length} VIOLATIONS`;
    ab.className = 'badge ' + (ast.compliant ? 'ok' : 'err');
    document.getElementById('sha-badge').textContent = `sha256:${(cur.source_sha256||'').slice(0,10)}`;
    document.getElementById('diff-view').innerHTML = renderDiffHtml(cur.unified_diff || '');

    const subPayload = {
      iteration: cur.label,
      origin: cur.origin,
      llm_meta: cur.llm_meta,
      ast_compliance: cur.ast_check,
      subroutines: cur.subroutines,
      stage1_probe_report: pr || null,
    };
    document.getElementById('subr-view').textContent = JSON.stringify(subPayload, null, 2);

    // Right pane: Oracle & Attribution & Journal
    const ex = cur.execution;
    const attr = cur.attribution || {};
    const vp = document.getElementById('verdict-pill');
    if (ex) {
      vp.textContent = ex.task_passed ? 'PASS' : 'FAIL';
      vp.className = 'badge ' + (ex.task_passed ? 'ok' : 'err');
      document.getElementById('m-layer').textContent = attr.layer || 'none';
      document.getElementById('m-code').textContent = attr.code || ex.host_runtime_status || '-';
      const o = ex.oracle_post_run || {};
      document.getElementById('m-oracle').textContent = `${o.cup_filled ? 'FILLED' : 'EMPTY'} / ${o.delivery_error_mm != null ? o.delivery_error_mm + 'mm' : '-'}`;
      const bc = (ex.non_idempotent_counts || {}).start_coffee_machine ?? 0;
      document.getElementById('m-brew').textContent = `${bc}x / dup=${ex.duplicate_non_idempotent ? 1 : 0} / unk=${ex.unknown_events_observed ?? 0}`;
    } else {
      vp.textContent = 'UNEXECUTED';
      vp.className = 'badge';
      document.getElementById('m-layer').textContent = '-';
      document.getElementById('m-code').textContent = '-';
      document.getElementById('m-oracle').textContent = '-';
      document.getElementById('m-brew').textContent = '-';
    }

    const jrows = cur.journal_entries || [];
    document.getElementById('journal-count').textContent = `${jrows.length} ops`;
    document.getElementById('journal-tbody').innerHTML = jrows.length
      ? jrows.map((r, i) => {
          const st = String(r.state || '');
          const cls = st.includes('UNKNOWN') || r.action === 'command_ack_timeout_unknown'
            ? 'row-warn'
            : (r.action === 'query_command_status' || st.includes('RECONCILED') ? 'row-ok' : (r.non_idempotent ? 'row-hi' : ''));
          return `<tr class="${cls}"><td>${i+1}</td><td>${esc(r.action)}</td><td>${esc(st)}</td></tr>`;
        }).join('')
      : '<tr><td colspan="3" style="color:var(--dim)">Click 3. Execute Py to run inside AST sandbox + SQLite journal.</td></tr>';
  }

  // Console log
  const cbox = document.getElementById('console-box');
  const logs = evo.event_log || [];
  cbox.innerHTML = logs.map(l => `
    <div class="log-ln"><span class="log-t">[${esc(l.t)}]</span><span class="log-tag tag-${esc(l.tag)}">[${esc(l.tag)}]</span><span>${esc(l.msg)}</span></div>
  `).join('');
  cbox.scrollTop = cbox.scrollHeight;

  renderLlmInspector(evo);
  renderExecutedCodeInspector(evo);

  if (evo.m1_archive) renderM1Archive(evo.m1_archive);
}

function renderNumberedCode(src) {
  const lines = String(src || '').split('\n');
  return lines.map((ln, idx) => `<span class="ln-num">${idx + 1}</span>${esc(ln)}`).join('\n');
}

function renderLlmInspector(evo) {
  const completed = evo.llm_calls || [];
  const pending = evo.pending_llm_call || null;
  const allCalls = pending ? [...completed, pending] : completed;
  const cntEl = document.getElementById('llm-tab-cnt');
  if (cntEl) cntEl.textContent = pending ? `(${completed.length}+1 LIVE)` : `(${completed.length})`;

  if (allCalls.length !== lastLlmCount) {
    selectedLlmCallIdx = allCalls.length - 1;
    lastLlmCount = allCalls.length;
  }
  if (selectedLlmCallIdx < 0 || selectedLlmCallIdx >= allCalls.length) {
    selectedLlmCallIdx = allCalls.length - 1;
  }

  const strip = document.getElementById('llm-call-strip');
  const metaBar = document.getElementById('llm-meta-bar');
  const pPre = document.getElementById('llm-prompt-pre');
  const rPre = document.getElementById('llm-resp-pre');
  const pLen = document.getElementById('llm-prompt-len');
  const rLen = document.getElementById('llm-resp-len');

  if (!allCalls.length) {
    strip.innerHTML = '';
    metaBar.textContent = 'No LLM call recorded yet. Click 2. Synthesize Py (LLM) or Run Closed Loop to inspect full prompt & response.';
    pPre.textContent = '(no LLM prompt yet)';
    rPre.textContent = '(no LLM response yet)';
    pLen.textContent = '0 chars';
    rLen.textContent = '0 chars';
    return;
  }

  strip.innerHTML = allCalls.map((c, idx) => {
    const isRun = c.status === 'running';
    const act = idx === selectedLlmCallIdx ? 'active' : '';
    const dotCls = isRun ? '' : 'pass';
    const lbl = isRun
      ? `call #${c.call_id} [${c.origin} · RUNNING]`
      : `call #${c.call_id} [it=${c.iter_index} · ${c.origin} · ${c.wall_s}s]`;
    return `<button class="iter-chip ${dotCls} ${act}" onclick="selectLlmCall(${idx})"><span class="dot"></span>${esc(lbl)}</button>`;
  }).join('');

  const cur = allCalls[selectedLlmCallIdx];
  metaBar.innerHTML = `
    <span><strong>call #${esc(cur.call_id)}</strong> &middot; origin=<code>${esc(cur.origin)}</code> &middot; model=<code>${esc(cur.model)}</code> (${esc(cur.reasoning_effort||'-')}) &middot; provider=<code>${esc(cur.provider||'-')}</code></span>
    <span>${cur.status === 'running' ? '<span class="spin"></span>awaiting LLM response...' : `wall=${esc(cur.wall_s)}s · ${esc(cur.started_utc||'')} -> ${esc(cur.completed_utc||'')}`}</span>
  `;
  const promptTxt = cur.prompt || '';
  const respTxt = cur.status === 'running' ? '(waiting for LLM response...)' : (cur.raw_response || cur.extracted_source || '');
  pPre.textContent = promptTxt;
  rPre.textContent = respTxt;
  pLen.textContent = `${promptTxt.length} chars`;
  rLen.textContent = cur.status === 'running' ? 'pending' : `${respTxt.length} chars`;
}

function renderExecutedCodeInspector(evo) {
  const runs = evo.executed_runs || [];
  const cntEl = document.getElementById('exec-tab-cnt');
  if (cntEl) cntEl.textContent = `(${runs.length})`;

  if (runs.length !== lastExecCount) {
    selectedExecRunIdx = runs.length - 1;
    lastExecCount = runs.length;
  }
  if (selectedExecRunIdx < 0 || selectedExecRunIdx >= runs.length) {
    selectedExecRunIdx = runs.length - 1;
  }

  const strip = document.getElementById('exec-run-strip');
  const metaBar = document.getElementById('exec-meta-bar');
  const titleEl = document.getElementById('exec-code-title');
  const shaEl = document.getElementById('exec-code-sha');
  const codePre = document.getElementById('exec-code-pre');

  if (!runs.length) {
    currentExecRawSource = '';
    strip.innerHTML = '';
    metaBar.textContent = 'No Python code has been executed on the host yet. Click 1. Explore / Probe or 3. Execute Py.';
    titleEl.textContent = 'EXACT EXECUTED PYTHON SNAPSHOT';
    shaEl.textContent = '-';
    codePre.textContent = '(no executed Python snapshot yet)';
    return;
  }

  strip.innerHTML = runs.map((r, idx) => {
    const act = idx === selectedExecRunIdx ? 'active' : '';
    const stCls = r.stage === '01_explore_probe' ? 'pass' : (r.task_passed ? 'pass' : 'fail');
    const tag = r.stage === '01_explore_probe' ? 'PROBE' : (r.task_passed ? 'PASS' : 'FAIL');
    return `<button class="iter-chip ${stCls} ${act}" onclick="selectExecRun(${idx})"><span class="dot"></span>#${r.run_index} ${esc(r.label)} [${tag}]</button>`;
  }).join('');

  const cur = runs[selectedExecRunIdx];
  currentExecRawSource = cur.source || '';
  const methodsStr = (cur.methods || []).join(' -> ');
  metaBar.innerHTML = `
    <span><strong>#${esc(cur.run_index)} ${esc(cur.stage)}</strong> &middot; cmd=<code>${esc(cur.command_id)}</code> &middot; scenario=<code>${esc(cur.scenario_key)}</code> &middot; wall=${esc(cur.wall_s)}s &middot; @${esc(cur.executed_utc)}</span>
    <span>calls=${(cur.methods||[]).length} (${esc(methodsStr.slice(0, 90))}${methodsStr.length > 90 ? '...' : ''})</span>
  `;
  titleEl.textContent = `EXACT EXECUTED PYTHON — ${cur.label}`;
  shaEl.textContent = `sha256:${(cur.source_sha256 || '').slice(0, 14)} · ${(cur.source || '').split('\n').length} lines`;
  codePre.innerHTML = renderNumberedCode(cur.source || '');
}

function renderM1Archive(arc) {
  const fmtTable = (headers, rows) => `<table><thead><tr>${headers.map(h => `<th>${h}</th>`).join('')}</tr></thead><tbody>${rows.join('')}</tbody></table>`;

  if (arc.robosuite_v4) {
    const sm = arc.robosuite_v4.summary_by_method || {};
    const mc = arc.robosuite_v4.paired_mcnemar_craft_vs || {};
    const order = ['craft_full','capx_protocol','simex_protocol','aspire_protocol','simex_direct_repair','strong_zeroshot'];
    const rows = order.filter(k => sm[k]).map(k => {
      const r = sm[k]; const bt = r.by_task || {};
      const p = k === 'craft_full' ? 'ref' : (mc[k] ? (mc[k].p_value < 1e-4 ? '<1e-4' : mc[k].p_value.toFixed(4)) : '-');
      return `<tr class="${k==='craft_full'?'row-hi':''}">
        <td><strong>${esc(r.label||k)}</strong></td>
        <td>${r.passed}/80 (${(r.pass_rate*100).toFixed(1)}%)</td>
        <td>${bt.Lift?.passed??'-'}/${bt.PickPlaceCan?.passed??'-'}/${bt.NutAssemblySquare?.passed??'-'}/${bt.Door?.passed??'-'}</td>
        <td>${r.dev_llm_calls} / ${r.dev_sim_resets}</td>
        <td>${r.eval_llm_calls}</td>
        <td>${r.p95_wall_s.toFixed(2)}s</td>
        <td>${p}</td>
      </tr>`;
    });
    document.getElementById('m1-robosuite').innerHTML = fmtTable(['Method','Total (80)','Lift/Can/Nut/Door','Dev LLM/Resets','Eval LLM','P95','McNemar p'], rows);
  }

  const renderLibero = (elId, sec, includeRax) => {
    if (!sec) return;
    const sm = sec.summary_by_method || {};
    const mc = sec.paired_mcnemar_craft_vs || {};
    const order = includeRax
      ? ['aspire_protocol','craft_full','simex_protocol','rax_protocol','strong_zeroshot']
      : ['craft_full','aspire_protocol','simex_protocol','strong_zeroshot'];
    const rows = order.filter(k => sm[k]).map(k => {
      const r = sm[k];
      const zeros = Object.values(r.by_task || {}).filter(t => t.passed === 0).length;
      const p = k === 'craft_full' ? 'ref' : (mc[k] ? (mc[k].p_value < 1e-4 ? '<1e-4' : mc[k].p_value.toFixed(4)) : '-');
      return `<tr class="${k==='craft_full'?'row-hi':''}">
        <td><strong>${esc(r.label||k)}</strong></td>
        <td>${r.passed}/100 (${(r.pass_rate*100).toFixed(1)}%)</td>
        <td>${zeros}/10</td>
        <td>${r.dev_llm_calls}/${r.dev_sim_resets}</td>
        <td>${p}</td>
      </tr>`;
    });
    rows.push(`<tr style="color:var(--dim)"><td><em>RAX (arXiv:2610.22496 Table 6, unmatched RGB-D)</em></td><td>${includeRax?'100.0%':'99.2%'} (n=500)</td><td>-</td><td>&le;20 eps (Opus 5.5)</td><td>unmatched</td></tr>`);
    document.getElementById(elId).innerHTML = fmtTable(['Method','Pass (100)','0/10 Tasks','Dev LLM/Res','p vs TRAX'], rows);
  };
  renderLibero('m1-libero-spatial', arc.libero_spatial, false);
  renderLibero('m1-libero-object', arc.libero_object_blind, true);

  if (arc.cafebench_v2) {
    const sm = arc.cafebench_v2.summary_by_method || {};
    const rows = Object.entries(sm).map(([k, r]) => `<tr class="${k==='craft_full'?'row-hi':''}">
      <td><strong>${esc(r.label||k)}</strong></td><td>${r.passed}/${r.total} (${(r.pass_rate*100).toFixed(1)}%)</td>
      <td>${r.duplicate_non_idempotent_count}/${r.false_actuation_count}</td><td>${r.dev_llm_calls}</td></tr>`);
    document.getElementById('m1-cafebench').innerHTML = fmtTable(['Method','Held-Out (30)','Dup/False','Dev LLM'], rows);
  }
  if (arc.sim2sim_stage2) {
    const sm = arc.sim2sim_stage2.summary_by_method || {};
    const rows = Object.entries(sm).map(([k, r]) => `<tr class="${k==='craft_full'?'row-hi':''}">
      <td><strong>${esc(r.label||k)}</strong></td><td>${r.passed}/${r.total_trials} (${(r.pass_rate*100).toFixed(1)}%)</td>
      <td>${r.target_passed}/${r.target_total} &middot; ${r.shadow_passed}/${r.shadow_total}</td><td>${r.adapt_llm_calls}</td></tr>`);
    document.getElementById('m1-sim2sim').innerHTML = fmtTable(['Method','Overall (20)','Target / Shadow','LLM'], rows);
  }
}

function updateLiveUI(live) {
  const obs = live.scene_observation || {};
  lastLiveScene = obs;
  const raw = obs.raw_snapshot || {};
  const cups = (obs.bound_objects && obs.bound_objects.coffee_cup) || [];
  const l2 = raw.l2_pouring_safety || {};
  const brewProg = Number(raw.brew_progress ?? (raw.cup_filled ? 1 : 0));
  const radialMm = l2.current_radial_offset_m != null ? (Number(l2.current_radial_offset_m) * 1000).toFixed(1) : null;

  const hp = document.getElementById('hud-phase');
  if (hp) hp.textContent = `phase: ${raw.phase || 'idle'}` + (raw.in_hand_cup ? ' [grasped]' : '');
  const hl2 = document.getElementById('hud-l2');
  if (hl2) {
    hl2.textContent = l2.safe_to_pour ? `L2: SAFE (${radialMm ?? '0.0'}mm)` : `L2: ${radialMm ?? '-'}mm`;
    hl2.className = 'hud-pill ' + (l2.safe_to_pour ? 'ok' : 'warn');
  }
  const bb = document.getElementById('hud-brew-bar');
  const bt = document.getElementById('hud-brew-txt');
  if (bb) bb.style.width = `${Math.max(0, Math.min(100, Math.round(brewProg * 100)))}%`;
  if (bt) bt.textContent = `brew ${Math.round(brewProg * 100)}%`;

  const ha = document.getElementById('hud-action');
  const evo = live.evolution;
  if (ha && evo) ha.textContent = evo.busy ? evo.busy_detail : `phase=${evo.phase} | iters=${(evo.iterations||[]).length}`;

  if (evo) renderEvolution(evo);
  renderSpatialRadar(obs);

  const ca = document.getElementById('confirm-alert');
  if (cups.length >= 2) {
    ca.classList.add('show');
    document.getElementById('confirm-btns').innerHTML = cups.map(c => {
      const oid = c.object_id || c.id;
      return `<button class="warn sm" onclick="confirmCup('${oid}')">Confirm ${oid}</button>`;
    }).join('');
  } else {
    ca.classList.remove('show');
  }
}

async function refreshState() {
  try {
    const st = await api('api/state');
    const ok = st.capability_check && st.capability_check.status === 'ready';
    const d = (st.capability_check && st.capability_check.discovered) || {};
    const lp = document.getElementById('pill-live');
    lp.innerHTML = `<span class="pulse"></span>${ok ? 'ONLINE' : (st.capability_check||{}).status}`;
    lp.className = 'badge ' + (ok ? 'ok' : 'err');
    document.getElementById('pill-authority').textContent = d.execution_authority || st.driver;
    document.getElementById('pill-evidence').textContent = `evidence=${st.evidence_source}`;

    if (JSON.stringify(st.cameras || []) !== JSON.stringify(camList)) {
      camList = st.cameras || [];
      activeCam = camList.includes('overview') ? 'overview' : (camList[0] || '');
      renderCameraShells();
    }

    // Governance tab
    document.getElementById('cap-pill').textContent = (st.capability_check||{}).status || '-';
    document.getElementById('cap-pill').className = 'badge ' + (ok ? 'ok' : 'err');
    document.getElementById('cap-meta').textContent = `authority=${d.execution_authority||'-'} · perception=${d.perception_owner||'-'} · contract_sha256=${(d.sdk_contract_sha256||'').slice(0,16)}`;
    document.getElementById('cap-ok-chips').innerHTML = (d.allowed_methods || []).map(m => `<span class="chip ok">${esc(m)}</span>`).join('');
    document.getElementById('cap-miss-chips').innerHTML = ((st.oven_gap || {}).missing_methods || []).map(m => `<span class="chip miss">${esc(m)}</span>`).join('') || '<span class="chip ok">none</span>';

    const skills = Object.values(st.skills || {});
    const hist = (st.registry && st.registry.history) || [];
    document.getElementById('gov-skills').innerHTML = `
      <div style="font-size:10.5px;font-family:var(--mono);color:var(--cyan);font-weight:700">VERIFIED SKILLS (${skills.length})</div>
      ${skills.map(s => `<div style="padding:5px 0;border-bottom:1px solid var(--border);font-size:11.5px;font-family:var(--mono)"><strong>${esc(s.title)}</strong> <span class="badge ok">${esc(s.evidence_source)}</span></div>`).join('') || '<div style="font-size:11px;color:var(--dim)">None</div>'}
      <div style="font-size:10.5px;font-family:var(--mono);color:var(--cyan);font-weight:700;margin-top:10px">RELEASE LEDGER (${hist.length})</div>
      ${hist.map(h => `<div style="padding:4px 0;border-bottom:1px solid var(--border);font-size:11px;font-family:var(--mono);color:var(--muted)"><strong>${esc(h.event)}</strong> &rarr; <code>${esc(h.version)}</code> (${esc(h.deployment_scope||'-')}) &middot; ${esc(h.at_utc||'')}</div>`).join('') || '<div style="font-size:11px;color:var(--dim)">None</div>'}
    `;

    const evo = await api('api/evolve/state');
    updateLiveUI({ scene_observation: st.scene_observation, evolution: evo });
    await loadEmbodiments();
  } catch (e) {
    document.getElementById('pill-live').textContent = 'OFFLINE';
    document.getElementById('pill-live').className = 'badge err';
  }
}

async function loadEmbodiments() {
  try {
    embSnapshot = await api('api/embodiments');
    const topSel = document.getElementById('emb-top-select');
    if (topSel) {
      topSel.innerHTML = (embSnapshot.targets || []).map(t =>
        `<option value="${esc(t.id)}" ${t.active ? 'selected' : ''}>${esc(t.label)}</option>`
      ).join('') + '<option value="__connect__">+ Connect Physical / Remote HTTP...</option>';
      topSel.value = embSnapshot.active_id || '';
    }
    renderRaxMatrix();
    renderComparatorCatalog();
  } catch (_) {}
}

function setMatrixFilter(f) {
  matrixFilter = f;
  for (const [id, key] of [['mf-all','all'], ['mf-sim','simulation'], ['mf-phys','physical']]) {
    const el = document.getElementById(id);
    if (el) el.className = 'sm' + (matrixFilter === key ? ' primary' : '');
  }
  renderRaxMatrix();
}

function renderRaxMatrix() {
  if (!embSnapshot) return;
  const rows = (embSnapshot.rax_matrix || []).filter(r => matrixFilter === 'all' || r.domain === matrixFilter);
  const statusCls = s => s === 'LIVE_SWITCHABLE' ? 'ok' : (s === 'BENCHMARK_VERIFIED' ? 'ok' : (s === 'HTTP_BRIDGE_READY' ? 'warn' : ''));
  document.getElementById('rax-matrix-tbody').innerHTML = rows.map(r => {
    let actBtn = '';
    if (r.action_type === 'switch') {
      const isAct = embSnapshot.active_id === r.action_target;
      actBtn = `<button class="sm ${isAct ? 'ok' : 'primary'}" onclick="switchEmbodiment('${esc(r.action_target)}', true)">${isAct ? 'Active' : 'Switch Live'}</button>`;
    } else if (r.action_type === 'compare') {
      actBtn = `<button class="sm" onclick="jumpCompareSuite('${esc(r.action_target)}')">Compare Code</button>`;
    } else {
      actBtn = `<button class="sm" onclick="prefillHttpHost('${esc(r.action_target)}','${esc(r.evidence_source)}','${esc(r.platform)}')">Connect HTTP</button>`;
    }
    return `<tr>
      <td><strong>${esc(r.family)}</strong></td>
      <td>${esc(r.platform)}</td>
      <td><span class="badge ${r.domain==='physical'?'warn':''}">${esc(r.domain)} / ${esc(r.evidence_source)}</span></td>
      <td style="color:var(--muted)">${esc(r.rax_scope)}</td>
      <td><span class="badge ${statusCls(r.trax_status)}">${esc(r.trax_status)}</span> <span style="margin-left:4px">${esc(r.trax_result)}</span></td>
      <td>${actBtn}</td>
    </tr>`;
  }).join('');
}

function prefillHttpHost(url, ev, label) {
  document.getElementById('http-host-url').value = url || '';
  document.getElementById('http-evidence').value = ev || 'physical_robot';
  document.getElementById('http-label').value = label || '';
  document.getElementById('http-host-url').focus();
  document.getElementById('http-connect-msg').textContent = `Prefilled HTTP bridge for ${label}. Start the host bridge and click Probe & Switch Live.`;
}

async function switchEmbodiment(targetId, goToLab = false) {
  if (targetId === '__connect__') {
    switchTab('m1');
    document.getElementById('http-host-url').focus();
    if (embSnapshot) document.getElementById('emb-top-select').value = embSnapshot.active_id;
    return;
  }
  const msgEl = document.getElementById('http-connect-msg');
  try {
    if (msgEl) msgEl.textContent = `Switching embodiment -> ${targetId}...`;
    cupTrail = [];
    camList = [];
    const res = await api('api/embodiment/switch', {
      method: 'POST', headers: {'Content-Type':'application/json'},
      body: JSON.stringify({ target_id: targetId }),
    });
    if (msgEl) msgEl.textContent = `Switched live to ${(res.active_embodiment||{}).label} (${(res.active_embodiment||{}).evidence_source}).`;
    await refreshState();
    if (goToLab) switchTab('lab');
  } catch (e) {
    if (msgEl) msgEl.textContent = `Switch failed: ${e.message}`;
    if (embSnapshot) document.getElementById('emb-top-select').value = embSnapshot.active_id;
  }
}

async function connectCustomHost() {
  const url = document.getElementById('http-host-url').value.trim();
  const ev = document.getElementById('http-evidence').value;
  const lbl = document.getElementById('http-label').value.trim();
  const msgEl = document.getElementById('http-connect-msg');
  if (!url) { msgEl.textContent = 'Enter a host HTTP base URL first.'; return; }
  msgEl.textContent = `Probing ${url}/api/capabilities ...`;
  try {
    cupTrail = [];
    camList = [];
    const res = await api('api/embodiment/switch', {
      method: 'POST', headers: {'Content-Type':'application/json'},
      body: JSON.stringify({ host_url: url, evidence_source: ev, label: lbl }),
    });
    msgEl.textContent = `Connected & switched live to ${(res.active_embodiment||{}).label} (authority=${(res.active_embodiment||{}).execution_authority}, evidence=${(res.active_embodiment||{}).evidence_source}).`;
    await refreshState();
    switchTab('lab');
  } catch (e) {
    msgEl.textContent = `Connection failed: ${e.message}`;
  }
}

function setCompareMode(mode) {
  compareMode = mode;
  document.getElementById('cmp-mode-split').className = 'sm' + (mode === 'split' ? ' primary' : '');
  document.getElementById('cmp-mode-diff').className = 'sm' + (mode === 'diff' ? ' primary' : '');
  document.getElementById('cmp-split-view').style.display = mode === 'split' ? 'grid' : 'none';
  document.getElementById('cmp-diff-view').style.display = mode === 'diff' ? 'block' : 'none';
}

function jumpCompareSuite(suiteKey) {
  switchTab('m1');
  const sel = document.getElementById('cmp-suite');
  if (sel && [...sel.options].some(o => o.value === suiteKey)) {
    sel.value = suiteKey;
    onCompareSuiteChange();
    sel.scrollIntoView({ behavior: 'smooth', block: 'center' });
  }
}

function renderComparatorCatalog() {
  if (!embSnapshot) return;
  const cat = embSnapshot.experiment_catalog || {};
  const keys = Object.keys(cat);
  const sSel = document.getElementById('cmp-suite');
  if (!sSel || !keys.length) return;
  const prev = sSel.value;
  if (sSel.options.length === 0) {
    sSel.innerHTML = keys.map(k => `<option value="${esc(k)}">${esc(cat[k].label)}</option>`).join('');
    sSel.value = prev && cat[prev] ? prev : keys[0];
    onCompareSuiteChange();
  }
}

function onCompareSuiteChange() {
  if (!embSnapshot) return;
  const cat = embSnapshot.experiment_catalog || {};
  const suite = cat[document.getElementById('cmp-suite').value];
  if (!suite) return;
  const tSel = document.getElementById('cmp-task');
  tSel.innerHTML = (suite.tasks || []).map(t =>
    `<option value="${esc(t.task_key)}">${esc(t.task_key)} — ${esc(t.language.slice(0, 62))}</option>`
  ).join('');
  const mOpts = (suite.methods || []).map(m =>
    `<option value="${esc(m)}">${esc(suite.method_labels[m] || m)}</option>`
  ).join('');
  const lSel = document.getElementById('cmp-left');
  const rSel = document.getElementById('cmp-right');
  lSel.innerHTML = mOpts;
  rSel.innerHTML = mOpts;
  lSel.value = suite.default_left;
  rSel.value = suite.default_right;
  renderTaskHeatmap(suite);
  loadPolicyComparison();
}

function selectHeatmapCell(taskKey, methodKey) {
  document.getElementById('cmp-task').value = taskKey;
  if (methodKey !== document.getElementById('cmp-left').value) {
    document.getElementById('cmp-right').value = methodKey;
  }
  loadPolicyComparison();
}

function renderTaskHeatmap(suite) {
  const methods = suite.methods || [];
  const head = `<tr><th>Task</th>${methods.map(m => `<th>${esc(suite.method_labels[m] || m)}</th>`).join('')}</tr>`;
  const curTask = document.getElementById('cmp-task').value;
  const body = (suite.tasks || []).map(t => {
    const cells = methods.map(m => {
      const sc = (t.scores || {})[m] || {};
      const pr = sc.pass_rate != null ? Number(sc.pass_rate) : null;
      const cls = pr == null ? '' : (pr >= 0.8 ? 'hm-hi' : (pr > 0 ? 'hm-mid' : 'hm-zero'));
      const selCls = (t.task_key === curTask) ? ' hm-sel' : '';
      const txt = sc.passed != null ? `${sc.passed}/${sc.total}` : '-';
      return `<td class="hm-cell ${cls}${selCls}" onclick="selectHeatmapCell('${esc(t.task_key)}','${esc(m)}')" title="Click to compare ${esc(t.task_key)} · ${esc(m)}">${txt}</td>`;
    }).join('');
    return `<tr><td style="white-space:nowrap"><strong>${esc(t.task_key)}</strong> <span style="color:var(--dim)">${esc(t.language.slice(0, 42))}</span></td>${cells}</tr>`;
  }).join('');
  document.getElementById('cmp-heatmap').innerHTML = `<table><thead>${head}</thead><tbody>${body}</tbody></table>`;
}

async function loadPolicyComparison() {
  const suiteKey = document.getElementById('cmp-suite').value;
  const taskKey = document.getElementById('cmp-task').value;
  const leftM = document.getElementById('cmp-left').value;
  const rightM = document.getElementById('cmp-right').value;
  if (!suiteKey || !taskKey) return;
  if (embSnapshot && embSnapshot.experiment_catalog[suiteKey]) {
    renderTaskHeatmap(embSnapshot.experiment_catalog[suiteKey]);
  }
  try {
    const q = new URLSearchParams({ suite: suiteKey, task: taskKey, left: leftM, right: rightM });
    const data = await api('api/compare/policy?' + q.toString());
    document.getElementById('cmp-task-banner').innerHTML = `
      <span><strong>${esc(data.task_key)}</strong> &middot; ${esc(data.task_language)}</span>
      <span>evidence=<code>${esc(data.evidence_source)}</code></span>
    `;
    const fillSide = (prefix, side) => {
      const sc = side.task_score || {};
      const scTxt = sc.passed != null ? `${sc.passed}/${sc.total} (${(Number(sc.pass_rate||0)*100).toFixed(0)}%)` : '-';
      const astTxt = side.ast_check ? (side.ast_check.compliant ? 'AST OK' : 'AST ERR') : '-';
      document.getElementById(prefix + '-head').textContent = `${side.label} (${scTxt})`;
      document.getElementById(prefix + '-meta').textContent = side.available
        ? `${side.file} · ${side.lines} lines · ${astTxt} · sha:${(side.source_sha256||'').slice(0,8)}`
        : 'Policy file unavailable';
      document.getElementById(prefix + '-pre').innerHTML = side.available
        ? renderNumberedCode(side.source)
        : '(no archived policy file for this task/method)';
    };
    fillSide('cmp-left', data.left);
    fillSide('cmp-right', data.right);
    document.getElementById('cmp-diff-box').innerHTML = renderDiffHtml(data.unified_diff || '');
  } catch (e) {
    document.getElementById('cmp-task-banner').textContent = `Compare error: ${e.message}`;
  }
}

async function runProbe() {
  cupTrail = [];
  await api('api/evolve/probe', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({
      scenario_key: document.getElementById('scenario-select').value,
      task_instruction: document.getElementById('task-instruction').value,
    }),
  });
  await pollLiveOnce();
}

async function runSynthesize() {
  editorDirty = false;
  switchCodeTab('llm');
  await api('api/evolve/synthesize', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({
      scenario_key: document.getElementById('scenario-select').value,
      task_instruction: document.getElementById('task-instruction').value,
    }),
  });
  await pollLiveOnce();
}

async function runExecute() {
  cupTrail = [];
  const src = document.getElementById('code-editor').value;
  await api('api/evolve/execute', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({
      scenario_key: document.getElementById('scenario-select').value,
      source: src,
    }),
  });
  editorDirty = false;
  await pollLiveOnce();
}

async function runClosedLoop() {
  cupTrail = [];
  editorDirty = false;
  await api('api/evolve/auto', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({
      scenario_key: document.getElementById('scenario-select').value,
      task_instruction: document.getElementById('task-instruction').value,
      max_iters: 3,
      fresh: true,
    }),
  });
  await pollLiveOnce();
}

async function selectIter(idx) {
  editorDirty = false;
  const evo = await api('api/evolve/select', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({ iter_index: idx }),
  });
  renderEvolution(evo);
}

async function resetSession() {
  cupTrail = [];
  editorDirty = false;
  lastLoadedIterSha = '';
  document.getElementById('code-editor').value = '';
  const evo = await api('api/evolve/reset', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({
      scenario_key: document.getElementById('scenario-select').value,
      task_instruction: document.getElementById('task-instruction').value,
    }),
  });
  renderEvolution(evo);
}

async function resetEnv() {
  cupTrail = [];
  await api('api/reset', { method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify({preset:'default'}) });
  await pollLiveOnce();
}

async function confirmCup(oid) {
  await api('api/confirm', { method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify({object_id: oid}) });
  await pollLiveOnce();
}

async function pollLiveOnce() {
  try {
    const live = await api('api/live');
    updateLiveUI(live);
  } catch (_) {}
}

refreshState();
(async function loop() {
  while (true) {
    const busy = evoState && evoState.busy;
    await new Promise(r => setTimeout(r, busy ? 400 : 1600));
    if (document.getElementById('tab-lab').classList.contains('active')) {
      await pollLiveOnce();
    }
  }
})();
</script>
</body>
</html>
"""
