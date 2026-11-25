"""Pure-English, publication-ready HTML/CSS/JS template for TRAX UI.

Pure Python stdlib (string constant only; zero external web/JS dependencies).
Designed for clean research demos and direct paper screenshots.
"""
from __future__ import annotations

UI_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>TRAX UI — Transactional Robot Action Execution</title>
<style>
  :root {
    --bg: #080c15; --surface: #0f1626; --surface2: #151f33; --border: #202e4a;
    --text: #f1f5f9; --muted: #94a3b8; --dim: #64748b;
    --blue: #38bdf8; --blue-bg: rgba(56,189,248,0.12);
    --green: #22c55e; --green-bg: rgba(34,197,94,0.12);
    --amber: #f59e0b; --amber-bg: rgba(245,158,11,0.14);
    --red: #ef4444; --red-bg: rgba(239,68,68,0.14);
    --mono: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0;
    font-family: Inter, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    background: var(--bg);
    color: var(--text);
    line-height: 1.45;
    -webkit-font-smoothing: antialiased;
  }
  a { color: var(--blue); text-decoration: none; }
  a:hover { text-decoration: underline; }
  code { font-family: var(--mono); font-size: 0.92em; }

  /* Topbar */
  .topbar {
    background: rgba(15,22,38,0.96);
    backdrop-filter: blur(8px);
    border-bottom: 1px solid var(--border);
    padding: 10px 22px;
    position: sticky; top: 0; z-index: 30;
  }
  .topbar-inner {
    max-width: 1440px; margin: 0 auto;
    display: flex; align-items: center; justify-content: space-between;
    gap: 14px; flex-wrap: wrap;
  }
  .brand-row { display: flex; align-items: center; gap: 9px; flex-wrap: wrap; }
  .brand-title { font-size: 17px; font-weight: 800; letter-spacing: 0.6px; color: #fff; }
  .brand-sep { color: var(--border); font-weight: 300; }
  .badge {
    display: inline-flex; align-items: center; gap: 5px;
    padding: 2px 9px; border-radius: 6px;
    font-size: 11.5px; font-weight: 600; font-family: var(--mono);
    background: var(--surface2); color: var(--blue); border: 1px solid var(--border);
  }
  .badge.ok { color: #86efac; background: var(--green-bg); border-color: rgba(34,197,94,0.35); }
  .badge.warn { color: #fde047; background: var(--amber-bg); border-color: rgba(245,158,11,0.35); }
  .badge.err { color: #fca5a5; background: var(--red-bg); border-color: rgba(239,68,68,0.35); }

  .tabs { display: flex; gap: 4px; background: #090e1b; padding: 3px; border-radius: 8px; border: 1px solid var(--border); }
  .tab-btn {
    background: transparent; color: var(--muted); border: none;
    padding: 6px 14px; border-radius: 6px; font-size: 12.5px; font-weight: 600;
    cursor: pointer; transition: all 0.15s;
  }
  .tab-btn:hover { color: var(--text); }
  .tab-btn.active { background: #1d4ed8; color: #fff; }

  .container { max-width: 1440px; margin: 0 auto; padding: 14px 22px 32px; }

  /* KPI Strip */
  .kpi-grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px; margin-bottom: 14px; }
  @media (max-width: 960px) { .kpi-grid { grid-template-columns: repeat(2, 1fr); } }
  .kpi { background: var(--surface); border: 1px solid var(--border); border-radius: 10px; padding: 10px 14px; }
  .kpi-label { font-size: 11px; font-weight: 600; text-transform: uppercase; letter-spacing: 0.5px; color: var(--muted); }
  .kpi-val { font-size: 20px; font-weight: 800; font-family: var(--mono); margin-top: 2px; letter-spacing: -0.3px; }
  .kpi-note { font-size: 11px; color: var(--dim); margin-top: 2px; font-family: var(--mono); }

  .tab-panel { display: none; }
  .tab-panel.active { display: block; }

  /* Stage Layout (Tab 1) */
  .stage-grid { display: grid; grid-template-columns: minmax(0, 1.55fr) minmax(350px, 1fr); gap: 14px; align-items: start; }
  @media (max-width: 1080px) { .stage-grid { grid-template-columns: 1fr; } }

  .card { background: var(--surface); border: 1px solid var(--border); border-radius: 10px; padding: 13px 15px; margin-bottom: 14px; }
  .card-title {
    font-size: 12.5px; font-weight: 700; text-transform: uppercase; letter-spacing: 0.55px; color: #cbd5e1;
    margin: 0 0 10px 0; display: flex; align-items: center; justify-content: space-between; gap: 8px; flex-wrap: wrap;
  }

  /* Smooth Canvas Video Stage + HUD */
  .video-shell {
    position: relative; background: #020617; border: 1px solid var(--border);
    border-radius: 8px; overflow: hidden; aspect-ratio: 16 / 9;
    display: flex; align-items: center; justify-content: center;
  }
  .video-shell canvas { width: 100%; height: 100%; object-fit: contain; display: block; }
  .hud-top-left { position: absolute; top: 9px; left: 9px; display: flex; gap: 6px; flex-wrap: wrap; pointer-events: none; }
  .hud-top-right { position: absolute; top: 9px; right: 9px; display: flex; gap: 6px; flex-wrap: wrap; pointer-events: none; }
  .hud-pill {
    background: rgba(8,12,21,0.82); backdrop-filter: blur(4px);
    border: 1px solid rgba(148,163,184,0.22); color: #f1f5f9;
    padding: 2px 8px; border-radius: 5px; font-size: 11px; font-weight: 600; font-family: var(--mono);
  }
  .hud-pill.ok { border-color: rgba(34,197,94,0.55); color: #86efac; }
  .hud-pill.warn { border-color: rgba(245,158,11,0.65); color: #fde047; }
  .hud-bottom { position: absolute; bottom: 0; left: 0; right: 0; padding: 7px 11px 6px; background: linear-gradient(transparent, rgba(2,6,23,0.9)); pointer-events: none; }
  .hud-bottom-row { display: flex; justify-content: space-between; align-items: center; font-size: 11px; font-family: var(--mono); color: #e2e8f0; margin-bottom: 4px; }
  .brew-bar-bg { height: 4px; background: rgba(255,255,255,0.16); border-radius: 99px; overflow: hidden; }
  .brew-bar-fg { height: 100%; width: 0%; background: linear-gradient(90deg, #38bdf8, #22c55e); transition: width 0.25s ease; }

  .cam-grid-4 { display: grid; grid-template-columns: 1fr 1fr; gap: 7px; }
  .cam-grid-4 .cam-cell {
    position: relative; background: #020617; border: 1px solid var(--border);
    border-radius: 7px; overflow: hidden; aspect-ratio: 16 / 9; cursor: pointer;
  }
  .cam-grid-4 canvas { width: 100%; height: 100%; object-fit: cover; display: block; }

  .thumb-strip { display: grid; grid-template-columns: repeat(4, 1fr); gap: 7px; margin-top: 8px; }
  .thumb {
    position: relative; background: #020617; border: 1.5px solid var(--border);
    border-radius: 7px; overflow: hidden; cursor: pointer; padding: 0; aspect-ratio: 16 / 9;
  }
  .thumb.active { border-color: var(--blue); box-shadow: 0 0 0 1px rgba(56,189,248,0.4); }
  .thumb canvas { width: 100%; height: 100%; object-fit: cover; display: block; }
  .thumb span {
    position: absolute; bottom: 4px; left: 5px;
    background: rgba(2,6,23,0.85); color: #e2e8f0;
    font-size: 10px; font-family: var(--mono); padding: 1px 6px; border-radius: 4px; font-weight: 600;
  }

  /* Spatial Radar + Telemetry */
  .spatial-grid { display: grid; grid-template-columns: 1.2fr 1fr; gap: 10px; margin-top: 10px; align-items: stretch; }
  @media (max-width: 780px) { .spatial-grid { grid-template-columns: 1fr; } }
  .radar-box { background: #070d1b; border: 1px solid var(--border); border-radius: 8px; padding: 8px 10px; display: flex; flex-direction: column; }
  .radar-head { display: flex; justify-content: space-between; align-items: center; font-size: 11px; font-weight: 600; text-transform: uppercase; letter-spacing: 0.4px; color: var(--muted); margin-bottom: 5px; }
  .radar-svg { width: 100%; height: 175px; display: block; }

  .tel-stack { display: grid; grid-template-columns: 1fr 1fr; gap: 7px; }
  .tel-item { background: var(--surface2); border: 1px solid var(--border); border-radius: 7px; padding: 8px 10px; display: flex; flex-direction: column; justify-content: center; }
  .tel-k { font-size: 10.5px; text-transform: uppercase; letter-spacing: 0.4px; color: var(--muted); }
  .tel-v { font-size: 13px; font-weight: 700; margin-top: 3px; font-family: var(--mono); }

  /* Ambiguity Alert */
  .alert-box {
    margin-top: 10px; padding: 10px 12px; border-radius: 8px;
    background: var(--amber-bg); border: 1px solid rgba(245,158,11,0.5);
    display: none; align-items: center; justify-content: space-between; gap: 10px; flex-wrap: wrap;
  }
  .alert-box.show { display: flex; }

  /* Compact Scenario List */
  .scen-list { display: flex; flex-direction: column; gap: 7px; }
  .scen-card { background: var(--surface2); border: 1px solid var(--border); border-radius: 8px; padding: 9px 11px; transition: border-color 0.15s; }
  .scen-card:hover { border-color: #3b82f6; }
  .scen-head { display: flex; align-items: center; justify-content: space-between; gap: 8px; }
  .scen-title { font-size: 13px; font-weight: 700; color: #f8fafc; }
  .scen-meta { font-size: 11px; color: var(--muted); font-family: var(--mono); margin-top: 2px; }

  /* Buttons & Inputs */
  button {
    background: #1d4ed8; color: #fff; border: none; border-radius: 6px;
    padding: 5px 11px; font-size: 12px; font-weight: 600; cursor: pointer; transition: all 0.15s;
  }
  button:hover { background: #2563eb; }
  button:disabled { opacity: 0.45; cursor: not-allowed; }
  button.sec { background: var(--surface2); color: var(--text); border: 1px solid var(--border); }
  button.sec:hover { border-color: var(--blue); background: #1c2a46; }
  button.ok { background: #15803d; }
  button.ok:hover { background: #16a34a; }
  button.warn { background: #b45309; }
  button.warn:hover { background: #d97706; }
  select, input[type="text"], input[type="number"], textarea {
    background: #090e1b; color: var(--text); border: 1px solid var(--border); border-radius: 6px;
    padding: 6px 9px; font-size: 12.5px; font-family: var(--mono); width: 100%;
  }
  input[type="range"] { width: 100%; accent-color: #38bdf8; }
  textarea { resize: vertical; min-height: 62px; }

  /* Execution Trace */
  .verdict-banner { padding: 9px 12px; border-radius: 8px; margin-bottom: 9px; font-size: 12.5px; display: flex; align-items: center; justify-content: space-between; gap: 8px; flex-wrap: wrap; }
  .verdict-banner.idle { background: var(--surface2); border: 1px solid var(--border); color: var(--muted); }
  .verdict-banner.running { background: var(--blue-bg); border: 1px solid rgba(56,189,248,0.5); color: #bae6fd; }
  .verdict-banner.pass { background: var(--green-bg); border: 1px solid rgba(34,197,94,0.45); color: #bbf7d0; }
  .verdict-banner.fail { background: var(--red-bg); border: 1px solid rgba(239,68,68,0.45); color: #fecaca; }

  .step-flow { display: flex; flex-direction: column; gap: 5px; }
  .step-row {
    display: flex; align-items: center; justify-content: space-between; gap: 10px;
    padding: 7px 10px; border-radius: 7px; background: var(--surface2); border: 1px solid var(--border);
    font-size: 12px; transition: all 0.2s;
  }
  .step-row.active { border-color: rgba(56,189,248,0.75); background: rgba(56,189,248,0.1); }
  .step-row.done { border-color: rgba(34,197,94,0.38); }
  .step-row.warn { border-color: rgba(245,158,11,0.55); background: rgba(245,158,11,0.09); }
  .step-row.skip { opacity: 0.42; }
  .step-left { display: flex; align-items: center; gap: 8px; }
  .step-dot { width: 8px; height: 8px; border-radius: 50%; background: var(--dim); flex-shrink: 0; }
  .step-row.done .step-dot { background: var(--green); }
  .step-row.active .step-dot { background: var(--blue); }
  .step-row.warn .step-dot { background: var(--amber); }
  .step-title { font-weight: 600; color: #f1f5f9; }
  .step-sub { font-size: 11px; color: var(--muted); font-family: var(--mono); text-align: right; }

  /* Tables & Bars */
  table { width: 100%; border-collapse: collapse; font-size: 12.5px; }
  th, td { border-bottom: 1px solid var(--border); padding: 8px 9px; text-align: left; vertical-align: middle; }
  th { color: var(--muted); font-size: 11px; text-transform: uppercase; letter-spacing: 0.4px; font-weight: 600; background: #0c1425; }
  td { font-family: var(--mono); font-size: 12px; }
  .cell-pass { background: rgba(34,197,94,0.14); color: #86efac; font-weight: 700; }
  .cell-partial { background: rgba(245,158,11,0.15); color: #fde047; font-weight: 700; }
  .cell-fail { background: rgba(239,68,68,0.16); color: #fca5a5; font-weight: 700; }
  .bar-track { height: 8px; background: #1e293b; border-radius: 99px; overflow: hidden; margin-top: 4px; }
  .bar-fill { height: 100%; border-radius: 99px; background: linear-gradient(90deg, #38bdf8, #22c55e); }
  .chips { display: flex; flex-wrap: wrap; gap: 5px; margin-top: 7px; }
  .chip { font-size: 11px; padding: 2px 7px; border-radius: 5px; background: var(--surface2); color: #cbd5e1; border: 1px solid var(--border); font-family: var(--mono); }
  .chip.ok { background: var(--green-bg); color: #86efac; border-color: rgba(34,197,94,0.3); }
  .chip.miss { background: var(--red-bg); color: #fca5a5; border-color: rgba(239,68,68,0.35); }
  .two-col { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; }
  @media (max-width: 960px) { .two-col { grid-template-columns: 1fr; } }
  details { margin-top: 8px; font-size: 11.5px; color: var(--muted); }
  summary { cursor: pointer; font-weight: 600; color: var(--blue); }
  pre { background: #050912; border: 1px solid var(--border); border-radius: 7px; padding: 10px; font-size: 11.5px; font-family: var(--mono); overflow: auto; max-height: 360px; color: #cbd5e1; margin: 7px 0 0; }
  .ev-strip { padding: 8px 12px; border-radius: 7px; background: var(--surface2); border: 1px solid var(--border); color: var(--muted); font-size: 11.5px; font-family: var(--mono); margin-bottom: 14px; }
  .pulse { display: inline-block; width: 7px; height: 7px; border-radius: 50%; background: var(--green); margin-right: 4px; animation: blink 1.4s infinite; }
  .spin { display: inline-block; width: 11px; height: 11px; border: 2px solid rgba(56,189,248,0.3); border-top-color: #38bdf8; border-radius: 50%; animation: spin 0.8s linear infinite; vertical-align: -2px; margin-right: 5px; }
  @keyframes blink { 0%,100% { opacity: 1; } 50% { opacity: 0.25; } }
  @keyframes spin { to { transform: rotate(360deg); } }
</style>
</head>
<body>

<div class="topbar">
  <div class="topbar-inner">
    <div class="brand-row">
      <span class="brand-title">TRAX</span>
      <span class="brand-sep">|</span>
      <span id="pill-live" class="badge ok"><span class="pulse"></span>CONNECTING</span>
      <span id="pill-authority" class="badge">host=...</span>
      <span id="pill-driver" class="badge">evidence=...</span>
      <span id="pill-release" class="badge">release: -</span>
    </div>
    <div class="tabs">
      <button class="tab-btn active" data-tab="live" onclick="switchTab('live')">Live</button>
      <button class="tab-btn" data-tab="bench" onclick="switchTab('bench')">Benchmarks</button>
      <button class="tab-btn" data-tab="gov" onclick="switchTab('gov')">Governance</button>
    </div>
  </div>
</div>

<div class="container">
  <!-- KPI Strip -->
  <div class="kpi-grid">
    <div class="kpi">
      <div class="kpi-label">Paired Pass Rate</div>
      <div id="kpi-pass" class="kpi-val">-</div>
      <div id="kpi-pass-note" class="kpi-note">loading...</div>
    </div>
    <div class="kpi">
      <div class="kpi-label">Delivery Error</div>
      <div id="kpi-err" class="kpi-val">-</div>
      <div id="kpi-err-note" class="kpi-note">gate &le; 15.0 mm</div>
    </div>
    <div class="kpi">
      <div class="kpi-label">Non-Idempotent Duplicates</div>
      <div id="kpi-safe" class="kpi-val">-</div>
      <div id="kpi-safe-note" class="kpi-note">query-first journal</div>
    </div>
    <div class="kpi">
      <div class="kpi-label">Frozen Release</div>
      <div id="kpi-rel" class="kpi-val">-</div>
      <div id="kpi-rel-note" class="kpi-note">SHA-256 AST sandbox</div>
    </div>
  </div>

  <!-- TAB 1: LIVE -->
  <div id="tab-live" class="tab-panel active">
    <div class="stage-grid">
      <!-- Left: Multi-Camera Viewport + Spatial Radar -->
      <div>
        <div class="card">
          <div class="card-title">
            <span>Simulation Viewport</span>
            <div style="display:flex;gap:6px;align-items:center">
              <button id="btn-mode-single" class="sec" onclick="setCamMode('single')">Single</button>
              <button id="btn-mode-grid" class="sec" onclick="setCamMode('grid')">Quad</button>
            </div>
          </div>

          <div id="cam-stage"></div>
          <div id="cam-thumbs" class="thumb-strip"></div>

          <div class="spatial-grid">
            <div class="radar-box">
              <div class="radar-head">
                <span>2D Spatial Radar (World Frame)</span>
                <button class="sec" onclick="clearTrail()" style="padding:2px 7px;font-size:10.5px">Clear</button>
              </div>
              <svg id="radar-svg" class="radar-svg" viewBox="0 0 360 220"></svg>
            </div>
            <div class="tel-stack">
              <div class="tel-item"><div class="tel-k">Observation</div><div id="tel-obs" class="tel-v">-</div></div>
              <div class="tel-item"><div class="tel-k">Bound Objects</div><div id="tel-cups" class="tel-v">-</div></div>
              <div class="tel-item"><div class="tel-k">L2 Spout Gate</div><div id="tel-l2" class="tel-v">-</div></div>
              <div class="tel-item"><div class="tel-k">Appliance</div><div id="tel-cm" class="tel-v">-</div></div>
            </div>
          </div>

          <div id="confirm-alert" class="alert-box">
            <div>
              <strong style="color:#fde047;font-size:12.5px">Ambiguity Gate Triggered</strong>
              <div style="font-size:11.5px;color:#fef3c7;font-family:var(--mono)">Select target object_id to resume:</div>
            </div>
            <div id="confirm-btns" style="display:flex;gap:7px;flex-wrap:wrap"></div>
          </div>
        </div>
      </div>

      <!-- Right: Scenarios + Live Execution Trace -->
      <div>
        <div class="card">
          <div class="card-title">
            <span>Scenarios</span>
            <button class="sec" onclick="resetDefault()">Reset</button>
          </div>
          <div style="margin-bottom:9px">
            <select id="policy-select" onchange="loadPolicyCode()"></select>
          </div>

          <div class="scen-list">
            <div class="scen-card">
              <div class="scen-head">
                <div>
                  <div class="scen-title">A. Nominal Cycle</div>
                  <div class="scen-meta">default · slot_1 · gate &le; 15 mm</div>
                </div>
                <button class="scen-btn" onclick="runPresetScenario('normal')">Run</button>
              </div>
            </div>

            <div class="scen-card" style="border-color:rgba(245,158,11,0.45)">
              <div class="scen-head">
                <div>
                  <div class="scen-title">B. Slip + Lost Receipt</div>
                  <div class="scen-meta">slip (+38, -12 mm) · ack_timeout (unknown)</div>
                </div>
                <button class="scen-btn warn" onclick="runPresetScenario('fault')">Run</button>
              </div>
            </div>

            <div class="scen-card">
              <div class="scen-head">
                <div>
                  <div class="scen-title">C. Large Pose Offset</div>
                  <div class="scen-meta">cup_offset (+20, +64 mm) · live 3D pose</div>
                </div>
                <button class="scen-btn" onclick="runPresetScenario('offset')">Run</button>
              </div>
            </div>

            <div class="scen-card">
              <div class="scen-head">
                <div>
                  <div class="scen-title">D. Dual-Cup Ambiguity</div>
                  <div class="scen-meta">2 cups · fail-closed operator gate · 0 brew</div>
                </div>
                <button class="scen-btn sec" onclick="runPresetScenario('ambiguity')">Run</button>
              </div>
            </div>
          </div>

          <details>
            <summary>Custom Perturbation</summary>
            <div style="margin-top:8px;padding:9px;background:#090e1b;border-radius:7px;border:1px solid var(--border)">
              <div style="display:grid;grid-template-columns:1fr 1fr;gap:8px;font-family:var(--mono);font-size:11px">
                <div>
                  <label>cup X (m): <strong id="val-cx">0.33</strong></label>
                  <input type="range" id="rng-cx" min="0.30" max="0.37" step="0.005" value="0.33" oninput="document.getElementById('val-cx').textContent=this.value"/>
                </div>
                <div>
                  <label>cup Y (m): <strong id="val-cy">0.00</strong></label>
                  <input type="range" id="rng-cy" min="-0.08" max="0.08" step="0.005" value="0.00" oninput="document.getElementById('val-cy').textContent=this.value"/>
                </div>
                <div>
                  <label>slip dX (mm): <strong id="val-sx">32</strong></label>
                  <input type="range" id="rng-sx" min="-45" max="45" step="2" value="32" oninput="document.getElementById('val-sx').textContent=this.value"/>
                </div>
                <div>
                  <label>slip dY (mm): <strong id="val-sy">-10</strong></label>
                  <input type="range" id="rng-sy" min="-30" max="30" step="2" value="-10" oninput="document.getElementById('val-sy').textContent=this.value"/>
                </div>
              </div>
              <label style="display:flex;align-items:center;gap:6px;margin-top:7px;font-size:11.5px;color:var(--text)">
                <input type="checkbox" id="chk-custom-timeout" checked/> Inject brew receipt timeout (<code>unknown</code>)
              </label>
              <button class="scen-btn" style="margin-top:8px;width:100%" onclick="runCustomScenario()">Run Custom</button>
            </div>
          </details>
        </div>

        <div class="card">
          <div class="card-title">
            <span>Execution Trace &amp; Oracle</span>
            <span id="run-badge" class="badge">IDLE</span>
          </div>
          <div id="run-banner" class="verdict-banner idle">
            <span>Select a scenario to execute frozen AST policy.</span>
          </div>
          <div id="run-metrics" class="tel-stack" style="margin-bottom:9px;display:none">
            <div class="tel-item"><div class="tel-k">Oracle Fill</div><div id="rm-filled" class="tel-v">-</div></div>
            <div class="tel-item"><div class="tel-k">Delivery Err</div><div id="rm-err" class="tel-v">-</div></div>
            <div class="tel-item"><div class="tel-k">Brew Count</div><div id="rm-brew" class="tel-v">-</div></div>
            <div class="tel-item"><div class="tel-k">Wall / Reconcile</div><div id="rm-wall" class="tel-v">-</div></div>
          </div>
          <div id="step-flow" class="step-flow"></div>
          <details id="raw-trace-details" style="display:none">
            <summary>SDK Call Sequence</summary>
            <div id="raw-trace-chips" class="chips"></div>
          </details>
        </div>
      </div>
    </div>
  </div>

  <!-- TAB 2: BENCHMARKS -->
  <div id="tab-bench" class="tab-panel">
    <div class="ev-strip">
      evidence_source=<code id="bm-scope-text">simulator</code> &middot; paired held-out evaluation &middot; simulator evidence is not physical hardware qualification
    </div>

    <div class="card">
      <div class="card-title">
        <span>Paired Benchmark Summary</span>
        <div style="display:flex;align-items:center;gap:7px;flex-wrap:wrap">
          <span id="bench-status-pill" class="badge">idle</span>
          <span style="font-size:11px;color:var(--muted);text-transform:none">repeats</span>
          <input id="bench-repeats" type="number" min="1" max="5" value="1" style="width:52px"/>
          <button id="btn-start-bench" class="ok" onclick="startBenchmark()">Run Paired</button>
          <a href="benchmark_report.html" target="_blank" class="badge">CafeBench v1</a>
          <a href="cafebench_v2_report.html" target="_blank" class="badge ok">CafeBench v2</a>
          <a href="sim2sim_report.html" target="_blank" class="badge ok">Sim2Sim</a>
          <a href="robosuite_v2_report.html" target="_blank" class="badge ok">Robosuite</a>
        </div>
      </div>
      <div id="bench-bars" style="margin-bottom:12px"></div>
      <div id="bench-summary-table"></div>
    </div>

    <div class="card">
      <div class="card-title"><span>Per-Scenario Pass Matrix</span></div>
      <div id="bench-matrix"></div>
    </div>

    <div class="card">
      <div class="card-title"><span>Independent Acceptance &amp; Release Gate</span></div>
      <div id="gov-evidence-cards" class="two-col"></div>
    </div>

    <div class="two-col">
      <div class="card">
        <div class="card-title"><span>Four-Layer Attribution &amp; Step Latency</span></div>
        <div id="profiler-panel" style="font-size:12px;color:var(--muted)">loading...</div>
      </div>
      <div class="card">
        <div class="card-title"><span>Regression Suite &amp; Portability</span></div>
        <div id="regression-portability-panel" style="font-size:12px;color:var(--muted)">loading...</div>
      </div>
    </div>
  </div>

  <!-- TAB 3: GOVERNANCE -->
  <div id="tab-gov" class="tab-panel">
    <div class="two-col">
      <div class="card">
        <div class="card-title">
          <span>Host Capability Discovery</span>
          <span id="cap-pill" class="badge">...</span>
        </div>
        <div id="cap-meta" style="font-size:11.5px;font-family:var(--mono);color:var(--muted);margin-bottom:9px"></div>
        <div style="font-size:11px;font-weight:700;text-transform:uppercase;color:#86efac">Ready SDK Surface</div>
        <div id="cap-ok-chips" class="chips"></div>
        <div style="font-size:11px;font-weight:700;text-transform:uppercase;color:#fca5a5;margin-top:12px">Missing Capabilities (Oven / Bagel Gate)</div>
        <div id="cap-miss-chips" class="chips"></div>
      </div>

      <div class="card">
        <div class="card-title"><span>Task Specification Binding</span></div>
        <textarea id="teach-instruction">Brew one dine-in hot americano and deliver to $SLOT_PICKUP_1; recenter cup if L2 offset exceeds limit; reconcile unknown brew receipt via query_command_status without retrying.</textarea>
        <input type="text" id="teach-video" value="teaching://single-cup-coffee-ref" style="margin-top:7px"/>
        <div style="margin-top:8px"><button onclick="previewSpec()">Validate Specification</button></div>
        <pre id="spec-out" style="max-height:175px">Ready.</pre>
      </div>
    </div>

    <div class="two-col">
      <div class="card">
        <div class="card-title">
          <span>Day-Shift &amp; RepairScreener</span>
          <button onclick="runStudioShift()">Run 6-Order Shift</button>
        </div>
        <div id="ops-shift-panel" style="font-size:12px;color:var(--muted)">No shift reports yet.</div>
      </div>
      <div class="card">
        <div class="card-title"><span>Verified Skills &amp; Release Ledger</span></div>
        <div id="skills-and-history"></div>
      </div>
    </div>

    <div class="card">
      <div class="card-title">
        <span>Frozen Policy Source (Read-Only)</span>
        <select id="code-policy-select" style="width:auto" onchange="loadCodeTabPolicy()"></select>
      </div>
      <pre id="policy-code-view">loading...</pre>
    </div>
  </div>
</div>

<script>
let stateData = null;
let camList = [];
let activeCam = 'overview';
let camMode = 'single';
let isRunningScenario = false;
let activeScenarioLabel = '';
let cupTrail = [];
let frameTimestamps = [];
let lastLiveScene = null;
let pumpRunning = false;

const SCENARIO_PRESETS = {
  normal:    { label: 'A. Nominal Cycle', preset: 'default', fault_slip: false, simulate_timeout: false },
  fault:     { label: 'B. Slip (+38,-12mm) + Lost Receipt', preset: 'default', fault_slip: true, simulate_timeout: true },
  offset:    { label: 'C. Large Pose Offset (+20,+64mm)', preset: 'sim_pose_offset', fault_slip: false, simulate_timeout: false },
  ambiguity: { label: 'D. Dual-Cup Ambiguity Gate', preset: 'sim_dual_cup', fault_slip: false, simulate_timeout: false },
};

async function api(path, opts) {
  const r = await fetch(path, opts);
  const data = await r.json();
  if (!r.ok) throw new Error(data.error || ('HTTP ' + r.status));
  return data;
}

function switchTab(name) {
  document.querySelectorAll('.tab-btn').forEach(b => b.classList.toggle('active', b.dataset.tab === name));
  document.querySelectorAll('.tab-panel').forEach(p => p.classList.toggle('active', p.id === 'tab-' + name));
}

function setCamMode(mode) { camMode = mode; renderCameraShells(); }
function selectCam(name) { activeCam = name; camMode = 'single'; renderCameraShells(); }
function clearTrail() { cupTrail = []; if (lastLiveScene) renderSpatialRadar(lastLiveScene); }

function renderCameraShells() {
  const stage = document.getElementById('cam-stage');
  const thumbs = document.getElementById('cam-thumbs');
  if (!camList.length) {
    stage.innerHTML = '<div class="video-shell" style="color:var(--muted);font-family:var(--mono);font-size:12px">No camera stream</div>';
    thumbs.innerHTML = '';
    return;
  }
  if (!camList.includes(activeCam)) activeCam = camList[0];
  document.getElementById('btn-mode-single').style.borderColor = camMode === 'single' ? 'var(--blue)' : 'var(--border)';
  document.getElementById('btn-mode-grid').style.borderColor = camMode === 'grid' ? 'var(--blue)' : 'var(--border)';

  if (camMode === 'grid' && camList.length > 1) {
    stage.innerHTML = '<div class="cam-grid-4">' + camList.map(c => `
      <div class="cam-cell" onclick="selectCam('${c}')">
        <canvas data-cam-canvas="${c}" width="640" height="360"></canvas>
        <span class="hud-pill" style="position:absolute;top:7px;left:7px">${c}</span>
      </div>
    `).join('') + '</div>';
    thumbs.style.display = 'none';
  } else {
    stage.innerHTML = `
      <div class="video-shell">
        <canvas id="main-cam-canvas" data-cam-canvas="${activeCam}" width="1280" height="720"></canvas>
        <div class="hud-top-left">
          <span class="hud-pill" id="hud-cam">cam: ${activeCam}</span>
          <span class="hud-pill" id="hud-fps">-- FPS</span>
          <span class="hud-pill" id="hud-phase">phase: idle</span>
        </div>
        <div class="hud-top-right">
          <span class="hud-pill" id="hud-l2">L2: --</span>
        </div>
        <div class="hud-bottom">
          <div class="hud-bottom-row">
            <span id="hud-action">state: idle</span>
            <span id="hud-brew-txt">brew 0%</span>
          </div>
          <div class="brew-bar-bg"><div id="hud-brew-bar" class="brew-bar-fg"></div></div>
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
  let thumbTick = 0;
  while (true) {
    const t0 = performance.now();
    const onLiveTab = document.getElementById('tab-live').classList.contains('active');
    if (onLiveTab && camList.length) {
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
            const fpsEl = document.getElementById('hud-fps');
            if (fpsEl) fpsEl.textContent = (frameTimestamps.length / 2).toFixed(1) + ' FPS';
          }
        }
        thumbTick = (thumbTick + 1) % 5;
        if (thumbTick === 0 && camList.length > 1) {
          Promise.all(camList.map(c => {
            const tv = document.querySelector(`canvas[data-thumb-canvas="${c}"]`);
            return tv ? drawCamToCanvas(c, tv) : Promise.resolve();
          }));
        }
      }
    }
    const elapsed = performance.now() - t0;
    const targetMs = isRunningScenario ? 95 : 140;
    await new Promise(r => setTimeout(r, Math.max(25, targetMs - elapsed)));
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

  const primaryCup = objs.find(o => o.semantic_class === 'dine_in_red') || objs.find(o => o.semantic_class === 'takeout_green');
  if (primaryCup && Array.isArray(primaryCup.center_world_m)) {
    const [cx, cy] = worldToRadar(primaryCup.center_world_m[0], primaryCup.center_world_m[1]);
    const prev = cupTrail[cupTrail.length - 1];
    if (!prev || Math.hypot(cx - prev[0], cy - prev[1]) > 2.5) {
      cupTrail.push([cx, cy]);
      if (cupTrail.length > 60) cupTrail.shift();
    }
  }

  const trailPath = cupTrail.length > 1
    ? `<polyline fill="none" stroke="#38bdf8" stroke-width="2" stroke-dasharray="4 3" opacity="0.75" points="${cupTrail.map(p => p[0].toFixed(1)+','+p[1].toFixed(1)).join(' ')}"/>`
    : '';

  const cupMarks = objs
    .filter(o => o.semantic_class === 'dine_in_red' || o.semantic_class === 'takeout_green')
    .map(o => {
      const pos = o.center_world_m || o.world || [0.33, 0];
      const [px, py] = worldToRadar(pos[0], pos[1]);
      const col = o.semantic_class === 'dine_in_red' ? '#f43f5e' : '#10b981';
      const ring = filled && o.semantic_class === 'dine_in_red' ? '#facc15' : '#f8fafc';
      return `<g>
        <circle cx="${px.toFixed(1)}" cy="${py.toFixed(1)}" r="8.5" fill="${col}" stroke="${ring}" stroke-width="2.2"/>
        <text x="${px.toFixed(1)}" y="${(py + 18).toFixed(1)}" fill="#e2e8f0" font-size="9" font-family="monospace" text-anchor="middle">${o.object_id || o.id} (${(pos[0]*100).toFixed(1)},${(pos[1]*100).toFixed(1)}cm)</text>
      </g>`;
    }).join('');

  const spoutColor = l2.safe_to_pour ? '#22c55e' : '#f59e0b';
  svg.innerHTML = `
    <rect x="4" y="4" width="352" height="212" rx="7" fill="#091122" stroke="#1e2f4f"/>
    <line x1="30" y1="110" x2="330" y2="110" stroke="#1e293b" stroke-dasharray="2 4"/>
    <line x1="${sx.toFixed(1)}" y1="15" x2="${sx.toFixed(1)}" y2="205" stroke="#1e293b" stroke-dasharray="2 4"/>
    <rect x="${(nzx-28).toFixed(1)}" y="${(nzy-16).toFixed(1)}" width="64" height="32" rx="4" fill="rgba(239,68,68,0.12)" stroke="#ef4444" stroke-dasharray="3 2"/>
    <text x="${(nzx+4).toFixed(1)}" y="${(nzy+3).toFixed(1)}" fill="#fca5a5" font-size="8.5" font-family="monospace" text-anchor="middle">NO-GO</text>
    <circle cx="${sx.toFixed(1)}" cy="${sy.toFixed(1)}" r="17" fill="rgba(56,189,248,0.08)" stroke="${spoutColor}" stroke-width="1.8" stroke-dasharray="3 2"/>
    <circle cx="${sx.toFixed(1)}" cy="${sy.toFixed(1)}" r="2.5" fill="${spoutColor}"/>
    <text x="${sx.toFixed(1)}" y="${(sy - 22).toFixed(1)}" fill="${spoutColor}" font-size="9" font-weight="600" font-family="monospace" text-anchor="middle">SPOUT (L2 15mm)</text>
    <circle cx="${p1x.toFixed(1)}" cy="${p1y.toFixed(1)}" r="15" fill="rgba(34,197,94,0.08)" stroke="#22c55e" stroke-width="1.5" stroke-dasharray="3 2"/>
    <text x="${p1x.toFixed(1)}" y="${(p1y + 25).toFixed(1)}" fill="#86efac" font-size="9" font-family="monospace" text-anchor="middle">SLOT_1 (&le;15mm)</text>
    <circle cx="${p2x.toFixed(1)}" cy="${p2y.toFixed(1)}" r="13" fill="none" stroke="#475569" stroke-dasharray="2 3"/>
    <text x="${p2x.toFixed(1)}" y="${(p2y - 16).toFixed(1)}" fill="#64748b" font-size="8.5" font-family="monospace" text-anchor="middle">SLOT_2</text>
    ${trailPath}
    ${cupMarks}
  `;
}

function updateLiveTelemetryUI(live) {
  const obs = live.scene_observation || {};
  lastLiveScene = obs;
  const raw = obs.raw_snapshot || {};
  const cups = (obs.bound_objects && obs.bound_objects.coffee_cup) || [];
  const l2 = raw.l2_pouring_safety || {};
  const brewStatus = raw.brew_status || (raw.coffee_machine && raw.coffee_machine.status) || 'idle';
  const cupFilled = !!(raw.cup_filled || (raw.coffee_machine && raw.coffee_machine.cup_filled));
  const brewProg = Number(raw.brew_progress !== undefined ? raw.brew_progress : (cupFilled ? 1.0 : 0.0));
  const radialMm = l2.current_radial_offset_m != null ? (Number(l2.current_radial_offset_m) * 1000).toFixed(1) : null;

  document.getElementById('tel-obs').textContent = `${obs.status || '-'} (${obs.age_ms ?? '-'}ms)`;
  document.getElementById('tel-cups').textContent = `${cups.length} cup${cups.length === 1 ? '' : 's'}`;
  document.getElementById('tel-l2').textContent = l2.safe_to_pour
    ? `SAFE (${radialMm ?? '0.0'}mm)`
    : (radialMm != null ? `OFFSET ${radialMm}mm` : '-');
  document.getElementById('tel-cm').textContent = `${brewStatus} · ${Math.round(brewProg * 100)}%`;

  const hudPhase = document.getElementById('hud-phase');
  if (hudPhase) hudPhase.textContent = `phase: ${raw.phase || 'idle'}` + (raw.in_hand_cup ? ' [grasped]' : '');
  const hudL2 = document.getElementById('hud-l2');
  if (hudL2) {
    hudL2.textContent = l2.safe_to_pour ? `L2: SAFE (${radialMm ?? '0.0'}mm)` : `L2: OFFSET ${radialMm ?? '-'}mm`;
    hudL2.className = 'hud-pill ' + (l2.safe_to_pour ? 'ok' : 'warn');
  }
  const brewBar = document.getElementById('hud-brew-bar');
  const brewTxt = document.getElementById('hud-brew-txt');
  if (brewBar) brewBar.style.width = `${Math.max(0, Math.min(100, Math.round(brewProg * 100)))}%`;
  if (brewTxt) brewTxt.textContent = `brew ${Math.round(brewProg * 100)}%` + (cupFilled ? ' · filled' : '');

  const calert = document.getElementById('confirm-alert');
  if (cups.length >= 2) {
    calert.classList.add('show');
    document.getElementById('confirm-btns').innerHTML = cups.map(c => {
      const oid = c.object_id || c.id;
      return `<button class="warn" onclick="confirmCup('${oid}')">Confirm ${oid}</button>`;
    }).join('');
  } else {
    calert.classList.remove('show');
  }

  renderSpatialRadar(obs);

  const rs = live.run_state || {};
  const rt = live.live_runtime || {};
  if (rs.running) {
    const methods = (rt.steps || []).map(s => s.method).filter(Boolean);
    const lastM = methods.length ? methods[methods.length - 1] : 'resetting_scene';
    const hudAct = document.getElementById('hud-action');
    if (hudAct) hudAct.textContent = `exec (${rs.elapsed_s}s): ${lastM}`;
    const banner = document.getElementById('run-banner');
    banner.className = 'verdict-banner running';
    banner.innerHTML = `<div><span class="spin"></span><strong>${rs.label || activeScenarioLabel}</strong> (${rs.elapsed_s}s) &middot; <code>${lastM}</code></div>`;
    renderHumanSteps({ methods, running: true, host_runtime_status: rt.status });
  }
}

function renderHumanSteps(res) {
  const methods = res.methods || [];
  const running = !!res.running;
  const has = m => methods.includes(m);

  const step1Done = has('observe_scene');
  const step2Done = has('place_cup_under_machine');
  const step3Hit  = has('recenter_cup_under_spout');
  const step4Hit  = has('start_coffee_machine') || has('command_ack_timeout_unknown');
  const step5Done = has('complete_order_delivery');

  function stateCls(done, isActive, isWarn) {
    if (isWarn) return 'warn';
    if (isActive && running) return 'active';
    return done ? 'done' : 'skip';
  }

  const steps = [
    {
      cls: stateCls(step1Done, !step2Done, has('request_operator_confirmation')),
      title: '1. Observe & Bind',
      sub: has('request_operator_confirmation') ? 'ambiguity_gate' : (step1Done ? 'bound' : 'pending'),
    },
    {
      cls: stateCls(step2Done, step1Done && !step2Done, false),
      title: '2. Pick & Place',
      sub: step2Done ? 'under_spout' : (running && step1Done ? 'moving' : 'pending'),
    },
    {
      cls: stateCls(step3Hit, false, step3Hit),
      title: '3. L2 Recenter',
      sub: step3Hit ? 'recentered' : (step4Hit ? 'not_needed' : 'pending'),
    },
    {
      cls: stateCls(step4Hit, step2Done && !step4Hit, has('command_ack_timeout_unknown')),
      title: '4. Brew & Reconcile',
      sub: has('command_ack_timeout_unknown') ? 'reconciled_unknown' : (step4Hit ? 'brew_1x' : 'pending'),
    },
    {
      cls: stateCls(step5Done, step4Hit && !step5Done, false),
      title: '5. Deliver & Verify',
      sub: step5Done
        ? (res.oracle_post_run ? `err=${res.oracle_post_run.delivery_error_mm ?? '-'}mm` : 'delivered')
        : (running && step4Hit ? 'delivering' : 'pending'),
    },
  ];

  document.getElementById('step-flow').innerHTML = steps.map(s => `
    <div class="step-row ${s.cls}">
      <div class="step-left"><span class="step-dot"></span><span class="step-title">${s.title}</span></div>
      <span class="step-sub">${s.sub}</span>
    </div>
  `).join('');

  const det = document.getElementById('raw-trace-details');
  det.style.display = methods.length ? 'block' : 'none';
  document.getElementById('raw-trace-chips').innerHTML = methods.map((m, i) => {
    let cls = 'chip';
    if (m === 'command_ack_timeout_unknown') cls = 'chip miss';
    else if (m === 'recenter_cup_under_spout' || m === 'query_command_status') cls = 'chip ok';
    return `<span class="${cls}">${i+1}. ${m}</span>`;
  }).join('');
}

function renderKPIs(st) {
  const bm = st.benchmark;
  const pols = bm && bm.summary_by_policy ? Object.values(bm.summary_by_policy) : [];
  const craftPol = pols.find(p => p.policy_id.includes('v1')) || pols[0];
  const refPol = pols.find(p => p.policy_id === 'craft_reference');

  if (craftPol) {
    document.getElementById('kpi-pass').textContent = `${craftPol.passed_trials}/${craftPol.total_trials} (${(craftPol.pass_rate*100).toFixed(0)}%)`;
    document.getElementById('kpi-pass').style.color = craftPol.pass_rate >= 1 ? '#86efac' : '#fde047';
    document.getElementById('kpi-pass-note').textContent = refPol
      ? `ref ${refPol.passed_trials}/${refPol.total_trials} · ${(bm.trials||[]).length} trials`
      : `CI [${(craftPol.wilson_95_ci[0]*100).toFixed(0)}%, ${(craftPol.wilson_95_ci[1]*100).toFixed(0)}%]`;

    const err = craftPol.mean_delivery_error_mm;
    document.getElementById('kpi-err').textContent = err != null ? `${err.toFixed(2)} mm` : 'N/A';
    document.getElementById('kpi-err-note').textContent = craftPol.max_delivery_error_mm != null
      ? `max ${craftPol.max_delivery_error_mm.toFixed(2)} mm (gate <=15)`
      : 'oracle post-run';

    const dup = pols.reduce((a, p) => a + (p.duplicate_non_idempotent_total || 0), 0);
    const rec = craftPol.unknown_reconciled_rate !== undefined ? (craftPol.unknown_reconciled_rate * 100).toFixed(0) + '%' : '-';
    document.getElementById('kpi-safe').textContent = `${dup}`;
    document.getElementById('kpi-safe').style.color = dup === 0 ? '#86efac' : '#fca5a5';
    document.getElementById('kpi-safe-note').textContent = `reconciled ${rec}`;
  }

  const rel = st.active_release || 'none';
  const sum = st.workspace_summary || {};
  const imp = sum.improvement_result;
  document.getElementById('kpi-rel').textContent = rel;
  document.getElementById('kpi-rel').style.color = st.active_release ? '#38bdf8' : '#94a3b8';
  document.getElementById('kpi-rel-note').textContent = imp
    ? `v2 candidate: ${imp.decision}`
    : 'SHA-256 frozen AST';
}

function renderBenchmarkTab(st) {
  const bm = st.benchmark;
  document.getElementById('bm-scope-text').textContent = (bm && bm.evidence_source) || st.evidence_source || 'unknown';
  const statusPill = document.getElementById('bench-status-pill');

  if (!bm || !bm.summary_by_policy || Object.keys(bm.summary_by_policy).length === 0) {
    statusPill.textContent = st.benchmark_running ? 'running' : 'idle';
    document.getElementById('bench-bars').innerHTML = '<div style="color:var(--muted);font-size:12px">No archived benchmark run.</div>';
    document.getElementById('bench-summary-table').innerHTML = '';
    document.getElementById('bench-matrix').innerHTML = '';
  } else {
    const totalExpected = (bm.repeats || 1) * (bm.scenarios || []).length * (bm.policies || []).length;
    const done = (bm.trials || []).length;
    statusPill.textContent = bm.status === 'running' ? `${done}/${totalExpected}` : `${done} trials`;
    statusPill.className = 'badge ' + (bm.status === 'completed' ? 'ok' : 'warn');

    const pols = Object.values(bm.summary_by_policy);
    document.getElementById('bench-bars').innerHTML = pols.map(p => `
      <div style="margin-bottom:9px">
        <div style="display:flex;justify-content:space-between;font-size:12px;font-family:var(--mono)">
          <span><strong>${p.label}</strong></span>
          <span><strong>${(p.pass_rate*100).toFixed(1)}%</strong> (${p.passed_trials}/${p.total_trials}) · CI [${(p.wilson_95_ci[0]*100).toFixed(0)}%, ${(p.wilson_95_ci[1]*100).toFixed(0)}%]</span>
        </div>
        <div class="bar-track"><div class="bar-fill" style="width:${(p.pass_rate*100).toFixed(1)}%"></div></div>
      </div>
    `).join('');

    document.getElementById('bench-summary-table').innerHTML = `<table>
      <thead><tr><th>Policy</th><th>Pass Rate</th><th>Wall (Mean / P95)</th><th>Err (Mean / Max)</th><th>Reconciled</th><th>Dup</th><th>Failures</th></tr></thead>
      <tbody>${pols.map(p => {
        const f = Object.entries(p.failure_breakdown || {}).map(([k,v]) => `${k} x${v}`).join(', ') || 'none';
        const err = p.mean_delivery_error_mm != null ? `${p.mean_delivery_error_mm} / ${p.max_delivery_error_mm} mm` : '-';
        return `<tr>
          <td><strong>${p.label}</strong></td>
          <td>${p.passed_trials}/${p.total_trials} (${(p.pass_rate*100).toFixed(1)}%)</td>
          <td>${p.mean_wall_s}s / ${p.p95_wall_s ?? '-'}s</td>
          <td>${err}</td>
          <td>${(p.unknown_reconciled_rate*100).toFixed(0)}% (${p.unknown_events_observed})</td>
          <td>${p.duplicate_non_idempotent_total}</td>
          <td>${f}</td>
        </tr>`;
      }).join('')}</tbody>
    </table>`;

    const scenarios = bm.scenarios || [];
    const trials = bm.trials || [];
    document.getElementById('bench-matrix').innerHTML = `<table>
      <thead><tr><th>Scenario</th>${pols.map(p => `<th>${p.label}</th>`).join('')}</tr></thead>
      <tbody>${scenarios.map(s => {
        const cells = pols.map(p => {
          const ts = trials.filter(t => t.policy_id === p.policy_id && t.scenario_id === s.scenario_id);
          const ok = ts.filter(t => t.task_passed).length;
          const cls = ts.length === 0 ? '' : (ok === ts.length ? 'cell-pass' : (ok > 0 ? 'cell-partial' : 'cell-fail'));
          const avgW = ts.length ? (ts.reduce((a,b) => a + Number(b.wall_s||0), 0) / ts.length).toFixed(1) + 's' : '-';
          const last = ts[ts.length - 1];
          const extra = last && !last.task_passed ? `<br><small>${last.host_runtime_status}</small>` : (last && s.expect_status === 'completed' && last.oracle_post_run ? `<br><small>${last.oracle_post_run.delivery_error_mm}mm · ${avgW}</small>` : `<br><small>${avgW}</small>`);
          return `<td class="${cls}">${ok}/${ts.length}${extra}</td>`;
        }).join('');
        return `<tr><td><strong>${s.label}</strong><br><small style="color:var(--muted)">${s.category}</small></td>${cells}</tr>`;
      }).join('')}</tbody>
    </table>`;
  }

  const sum = st.workspace_summary;
  const v1 = st.v1_acceptance;
  const v2 = st.v2_acceptance;
  const cards = [];
  if (v1) {
    cards.push(`<div class="scen-card">
      <div class="scen-title">Release v1 Acceptance</div>
      <div class="scen-meta">completion=${(v1.completion_rate*100).toFixed(0)}% · max_err=${v1.max_delivery_error_mm}mm · mean_wall=${v1.mean_wall_s}s · dup=0</div>
    </div>`);
  }
  if (sum && sum.improvement_result) {
    const ir = sum.improvement_result;
    cards.push(`<div class="scen-card">
      <div class="scen-title">Candidate v2 Gate: ${ir.decision.toUpperCase()}</div>
      <div class="scen-meta">${v2 ? `v2=${(v2.completion_rate*100).toFixed(0)}% (${v2.mean_wall_s}s) · ` : ''}reason=${ir.reason}</div>
    </div>`);
  }
  document.getElementById('gov-evidence-cards').innerHTML = cards.join('') || '<div style="color:var(--muted);font-size:12px">No acceptance records.</div>';

  const prof = st.phase2_profiler || {};
  const attrCounts = prof.attribution_counts || {};
  const bns = (prof.method_bottlenecks || []).slice(0, 6);
  const pp = st.phase2_throughput;
  document.getElementById('profiler-panel').innerHTML = `
    <div class="chips" style="margin-top:0;margin-bottom:9px">
      <span class="chip">perception: ${attrCounts.perception || 0}</span>
      <span class="chip">control: ${attrCounts.control || 0}</span>
      <span class="chip">device: ${attrCounts.device || 0}</span>
      <span class="chip">policy: ${attrCounts.policy || 0}</span>
    </div>
    ${bns.length ? `<table>
      <thead><tr><th>Method</th><th>Calls</th><th>Mean (s)</th><th>Max (s)</th></tr></thead>
      <tbody>${bns.map(b => `<tr><td><code>${b.method}</code></td><td>${b.mean_calls_per_trial}</td><td>${b.mean_s_per_timed_trial ?? '-'}</td><td>${b.max_step_s ?? '-'}</td></tr>`).join('')}</tbody>
    </table>` : '<div style="font-size:12px">No profiler data.</div>'}
    ${pp && pp.pipeline_projection_2slot ? `<div style="margin-top:8px;font-size:11.5px;font-family:var(--mono);color:#86efac">mean=${pp.throughput.mean_order_s}s (${pp.throughput.throughput_orders_per_hour}/hr) · 2-slot model: +${pp.pipeline_projection_2slot.projected_throughput_gain_pct}% (${pp.pipeline_projection_2slot.pipelined_orders_per_hour}/hr)</div>` : ''}
  `;

  const regCases = st.regression_cases || [];
  const ports = st.portability_checks || {};
  const promo = st.autonomy_physical_gate;
  document.getElementById('regression-portability-panel').innerHTML = `
    <div style="font-size:11px;font-weight:700;text-transform:uppercase;color:var(--blue);margin-bottom:6px">Regression Bank (${regCases.length})</div>
    ${regCases.length ? regCases.map(c => `<div class="scen-card" style="margin-bottom:5px;padding:7px 9px">
      <div style="font-size:12px;font-weight:700;font-family:var(--mono)">${c.case_id} <span class="badge warn">${c.attribution_layer}/${c.attribution_code}</span></div>
      <div class="scen-meta">${c.scenario.label} (hits=${c.hit_count})</div>
    </div>`).join('') : '<div style="font-size:11.5px;color:var(--muted);margin-bottom:8px">No regression cases.</div>'}
    <div style="font-size:11px;font-weight:700;text-transform:uppercase;color:var(--blue);margin:9px 0 5px">Portability Static Check</div>
    <div class="chips" style="margin-top:0">${Object.entries(ports).map(([pid, r]) => `<span class="chip ${r.portable ? 'ok' : 'miss'}">${pid}: ${r.portable ? 'portable' : 'hardcoded'}</span>`).join('')}</div>
    ${promo ? `<div style="margin-top:9px;font-size:11.5px;font-family:var(--mono);color:${promo.allowed ? '#86efac' : '#fde047'}">physical_live gate: <strong>${promo.allowed ? 'OPEN' : 'BLOCKED'}</strong>${promo.missing_prerequisites && promo.missing_prerequisites.length ? ' [' + promo.missing_prerequisites.join(', ') + ']' : ''}</div>` : ''}
  `;
}

function renderGovTab(st) {
  const cap = st.capability_check || {};
  const d = cap.discovered || {};
  const cp = document.getElementById('cap-pill');
  cp.textContent = cap.status || '-';
  cp.className = 'badge ' + (cap.status === 'ready' ? 'ok' : 'err');
  document.getElementById('cap-meta').innerHTML = `
    authority=<code>${d.execution_authority || '-'}</code> &middot;
    perception=<code>${d.perception_owner || '-'}</code> &middot;
    sha256=<code>${(d.sdk_contract_sha256 || '').slice(0, 14)}</code>
  `;
  document.getElementById('cap-ok-chips').innerHTML = (d.allowed_methods || []).map(m => `<span class="chip ok">${m}</span>`).join('');
  document.getElementById('cap-miss-chips').innerHTML = ((st.oven_gap || {}).missing_methods || []).map(m => `<span class="chip miss">${m}</span>`).join('') || '<span class="chip ok">none</span>';

  const skills = Object.values(st.skills || {});
  const hist = (st.registry && st.registry.history) || [];
  document.getElementById('skills-and-history').innerHTML = `
    <div style="font-size:11px;font-weight:700;text-transform:uppercase;color:var(--blue);margin-bottom:5px">Verified Skills (${skills.length})</div>
    ${skills.length ? skills.map(s => `<div class="scen-card" style="margin-bottom:6px">
      <div class="scen-title">${s.title} <span class="badge ok">${s.evidence_source}</span></div>
      <div class="scen-meta">${s.verification_report_id}${s.measured_limits && Object.keys(s.measured_limits).length ? ' · limits=' + JSON.stringify(s.measured_limits) : ''}</div>
    </div>`).join('') : '<div style="font-size:11.5px;color:var(--muted)">None.</div>'}
    <div style="font-size:11px;font-weight:700;text-transform:uppercase;color:var(--blue);margin:10px 0 5px">Release History</div>
    ${hist.length ? hist.map(h => `<div style="font-size:11.5px;font-family:var(--mono);color:var(--muted);padding:3px 0;border-bottom:1px solid var(--border)">
      <strong>${h.event}</strong> &rarr; <code>${h.version}</code> (${h.deployment_scope || '-'}) · ${h.at_utc || ''}
    </div>`).join('') : '<div style="font-size:11.5px;color:var(--muted)">None.</div>'}
  `;

  const opsEl = document.getElementById('ops-shift-panel');
  if (opsEl) {
    const sh = st.latest_shift_report;
    const scs = st.screening_sessions || [];
    const shiftHtml = sh
      ? `<div class="scen-card" style="margin-bottom:7px">
          <div class="scen-title">${sh.shift_id} <span class="badge ${sh.passed_orders === sh.total_orders && sh.duplicate_non_idempotent_orders === 0 ? 'ok' : 'warn'}">${sh.passed_orders}/${sh.total_orders}</span></div>
          <div class="scen-meta">dup=${sh.duplicate_non_idempotent_orders} · false_act=${sh.false_actuation_orders} · ${sh.throughput_orders_per_hour}/hr · ${sh.evidence_source}</div>
        </div>`
      : '<div style="font-size:11.5px;color:var(--muted);margin-bottom:7px">No shift executed.</div>';
    const scrHtml = scs.length
      ? scs.slice(-3).map(s => `<div class="scen-card" style="margin-bottom:5px">
          <div class="scen-title">${s.screening_id} <span class="badge">${s.status}</span></div>
          <div class="scen-meta">candidates=${(s.candidates || []).length} · rec=${s.recommended_candidate_id || 'none'}</div>
        </div>`).join('')
      : '';
    opsEl.innerHTML = shiftHtml + scrHtml;
  }
}

async function runStudioShift() {
  const pid = document.getElementById('policy-select').value;
  try {
    await api('api/shift/run', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({policy_id: pid}),
    });
    await refreshState();
  } catch (e) {
    alert('Shift run failed: ' + e.message);
  }
}

async function refreshState() {
  try {
    const st = await api('api/state');
    stateData = st;

    const livePill = document.getElementById('pill-live');
    const ok = st.capability_check && st.capability_check.status === 'ready';
    const disc = (st.capability_check && st.capability_check.discovered) || {};
    livePill.innerHTML = `<span class="pulse"></span>${ok ? 'ONLINE' : (st.capability_check||{}).status}`;
    livePill.className = 'badge ' + (ok ? 'ok' : 'err');
    document.getElementById('pill-authority').textContent = disc.execution_authority || st.driver;
    document.getElementById('pill-driver').textContent = `evidence=${st.evidence_source}`;
    document.getElementById('pill-release').textContent = `release=${st.active_release || 'none'}`;

    if (JSON.stringify(st.cameras || []) !== JSON.stringify(camList)) {
      camList = st.cameras || [];
      if (camList.includes('overview')) activeCam = 'overview';
      else if (camList.length) activeCam = camList[0];
      renderCameraShells();
    }

    const pols = st.policies || [];
    for (const selId of ['policy-select', 'code-policy-select']) {
      const sel = document.getElementById(selId);
      const prev = sel.value;
      sel.innerHTML = pols.map(p => `<option value="${p.policy_id}">${p.label}</option>`).join('');
      if (prev && pols.some(p => p.policy_id === prev)) {
        sel.value = prev;
      } else {
        const pref = pols.find(p => p.policy_id === 'active_release') || pols.find(p => p.policy_id.includes('v1')) || pols[0];
        if (pref) sel.value = pref.policy_id;
      }
    }
    loadCodeTabPolicy();
    renderKPIs(st);
    renderBenchmarkTab(st);
    renderGovTab(st);
    updateLiveTelemetryUI({ scene_observation: st.scene_observation, run_state: st.run_state, live_runtime: st.live_runtime });
  } catch (e) {
    document.getElementById('pill-live').textContent = 'OFFLINE: ' + e.message;
    document.getElementById('pill-live').className = 'badge err';
  }
}

async function pollLiveOnce() {
  try {
    const live = await api('api/live');
    updateLiveTelemetryUI(live);
    const rs = live.run_state || {};
    if (isRunningScenario && !rs.running) {
      finishScenarioUI(rs);
    }
  } catch (_) {}
}

function finishScenarioUI(rs) {
  isRunningScenario = false;
  document.querySelectorAll('.scen-btn').forEach(b => b.disabled = false);
  const badge = document.getElementById('run-badge');
  const banner = document.getElementById('run-banner');
  const hudAct = document.getElementById('hud-action');

  if (rs.error) {
    badge.textContent = 'ERROR';
    badge.className = 'badge err';
    banner.className = 'verdict-banner fail';
    banner.textContent = 'Error: ' + rs.error;
    if (hudAct) hudAct.textContent = 'state: error';
    return;
  }
  const res = rs.last_result;
  if (!res) return;
  const ok = !!res.task_passed;
  badge.textContent = ok ? 'PASS' : 'FAIL';
  badge.className = 'badge ' + (ok ? 'ok' : 'err');
  banner.className = 'verdict-banner ' + (ok ? 'pass' : 'fail');
  banner.innerHTML = `<div><strong>${ok ? 'PASS' : 'FAIL'} &mdash; ${rs.label || activeScenarioLabel}</strong><br><span style="font-size:11.5px;font-family:var(--mono)">status=${res.host_runtime_status} · evidence=${res.evidence_source}</span></div>`;
  if (hudAct) hudAct.textContent = ok ? 'result: PASS' : 'result: FAIL (' + res.host_runtime_status + ')';

  const oracle = res.oracle_post_run || {};
  document.getElementById('run-metrics').style.display = 'grid';
  document.getElementById('rm-filled').textContent = oracle.cup_filled ? 'FILLED' : 'EMPTY';
  document.getElementById('rm-err').textContent = oracle.delivery_error_mm != null ? `${oracle.delivery_error_mm} mm` : '-';
  document.getElementById('rm-brew').textContent = `${(res.non_idempotent_counts||{}).start_coffee_machine ?? 0}x`;
  document.getElementById('rm-wall').textContent = `${res.wall_s}s / rec=${res.unknown_events_observed}`;
  renderHumanSteps(res);
  refreshState();
}

async function startScenarioPayload(payload, label) {
  if (isRunningScenario) return;
  isRunningScenario = true;
  activeScenarioLabel = label;
  cupTrail = [];
  document.querySelectorAll('.scen-btn').forEach(b => b.disabled = true);

  const badge = document.getElementById('run-badge');
  const banner = document.getElementById('run-banner');
  badge.textContent = 'RUNNING';
  badge.className = 'badge warn';
  banner.className = 'verdict-banner running';
  banner.innerHTML = `<div><span class="spin"></span><strong>${label}</strong></div>`;
  document.getElementById('run-metrics').style.display = 'none';
  renderHumanSteps({ methods: [], running: true });

  try {
    await api('api/run/start', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({ ...payload, label }),
    });
  } catch (e) {
    finishScenarioUI({ error: e.message });
  }
}

function runPresetScenario(key) {
  const spec = SCENARIO_PRESETS[key];
  startScenarioPayload({
    policy_id: document.getElementById('policy-select').value,
    preset: spec.preset,
    fault_slip: spec.fault_slip,
    simulate_timeout: spec.simulate_timeout,
  }, spec.label);
}

function runCustomScenario() {
  const cx = parseFloat(document.getElementById('rng-cx').value);
  const cy = parseFloat(document.getElementById('rng-cy').value);
  const sx = parseFloat(document.getElementById('rng-sx').value) / 1000.0;
  const sy = parseFloat(document.getElementById('rng-sy').value) / 1000.0;
  const timeout = document.getElementById('chk-custom-timeout').checked;
  const label = `Custom: cup(${(cx*100).toFixed(1)},${(cy*100).toFixed(1)}cm) slip(${(sx*1000).toFixed(0)},${(sy*1000).toFixed(0)}mm)`;
  startScenarioPayload({
    policy_id: document.getElementById('policy-select').value,
    preset: 'default',
    cup_a_xy: [cx, cy],
    fault_inject_slip_xy_m: (Math.abs(sx) > 0.001 || Math.abs(sy) > 0.001) ? [sx, sy] : null,
    simulate_timeout: timeout,
  }, label);
}

async function loadCodeTabPolicy() {
  const pid = document.getElementById('code-policy-select').value;
  if (!pid) return;
  try {
    const res = await api('api/policy/' + encodeURIComponent(pid));
    document.getElementById('policy-code-view').textContent = `# policy_id=${res.policy_id} | sha256=${res.source_sha256}\n\n` + (res.source || '');
  } catch (_) {}
}

function loadPolicyCode() {
  document.getElementById('code-policy-select').value = document.getElementById('policy-select').value;
  loadCodeTabPolicy();
}

async function confirmCup(oid) {
  await api('api/confirm', {
    method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({object_id: oid}),
  });
  await pollLiveOnce();
}

async function resetDefault() {
  cupTrail = [];
  await api('api/reset', {
    method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({preset: 'default'}),
  });
  await pollLiveOnce();
}

async function previewSpec() {
  const instruction = document.getElementById('teach-instruction').value;
  const video_ref = document.getElementById('teach-video').value;
  const res = await api('api/teaching/spec', {
    method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({instruction, video_ref}),
  });
  document.getElementById('spec-out').textContent = JSON.stringify(res, null, 2);
}

async function startBenchmark() {
  const repeats = parseInt(document.getElementById('bench-repeats').value || '1', 10);
  document.getElementById('btn-start-bench').disabled = true;
  try {
    await api('api/benchmark/start', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({repeats}),
    });
    const poll = setInterval(async () => {
      await refreshState();
      const st = stateData && stateData.benchmark && stateData.benchmark.status;
      if (st === 'completed' || st === 'failed' || st === 'blocked_host_not_ready') {
        clearInterval(poll);
        document.getElementById('btn-start-bench').disabled = false;
      }
    }, 3000);
  } catch (e) {
    alert('Benchmark failed: ' + e.message);
    document.getElementById('btn-start-bench').disabled = false;
  }
}

refreshState();
(async function livePollLoop() {
  while (true) {
    await new Promise(r => setTimeout(r, isRunningScenario ? 350 : 1500));
    if (document.getElementById('tab-live').classList.contains('active')) {
      await pollLiveOnce();
    }
  }
})();
</script>
</body>
</html>
"""
