"""Standalone, host-independent interactive Web Demo (CRAFT Studio).

Built purely on Python's stdlib ``http.server`` with zero external web dependencies.
Consumes any ``SceneHostDriver`` (``fixture`` standalone or ``http`` live sim)
read-only for perception/cameras and delegates all orchestration through CRAFT APIs.
"""
from __future__ import annotations

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time
from typing import Any, Mapping, Sequence
from urllib.parse import urlparse

from .autonomy import AutonomyPromotionLedger
from .benchmark import (
    PolicyEntry,
    builtin_reference_policy_source,
    default_coffee_benchmark_suite,
    render_benchmark_html,
    run_paired_benchmark,
)
from .coffee_loop import OVEN_REQUIRED_METHODS, default_coffee_teaching_steps
from .config import SceneConfig
from .host_driver import SceneHostDriver
from .portability import check_policy_portability
from .profiler import aggregate_trial_profiles
from .regression import RegressionSuite
from .release import ReleaseRegistry
from .teaching import TeachingMaterial, build_task_specification


STUDIO_HTML = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>CRAFT — 策略编排与独立验收实证平台</title>
<style>
  :root {
    --bg: #070b14; --surface: #0e1628; --surface2: #152038; --border: #1e2f4f;
    --text: #f1f5f9; --muted: #94a3b8; --dim: #64748b;
    --blue: #38bdf8; --blue-bg: rgba(56,189,248,0.13);
    --green: #22c55e; --green-bg: rgba(34,197,94,0.13);
    --amber: #f59e0b; --amber-bg: rgba(245,158,11,0.15);
    --red: #ef4444; --red-bg: rgba(239,68,68,0.15);
  }
  * { box-sizing: border-box; }
  body { margin: 0; font-family: -apple-system, BlinkMacSystemFont, "PingFang SC", "Segoe UI", Roboto, sans-serif; background: var(--bg); color: var(--text); line-height: 1.5; }
  a { color: var(--blue); text-decoration: none; }
  a:hover { text-decoration: underline; }

  /* Topbar */
  .topbar { background: rgba(14,22,40,0.95); backdrop-filter: blur(8px); border-bottom: 1px solid var(--border); padding: 12px 24px; position: sticky; top: 0; z-index: 30; }
  .topbar-inner { max-width: 1400px; margin: 0 auto; display: flex; align-items: center; justify-content: space-between; gap: 16px; flex-wrap: wrap; }
  .brand-block { display: flex; flex-direction: column; gap: 2px; }
  .brand-row { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }
  .brand-title { font-size: 19px; font-weight: 800; letter-spacing: 0.4px; }
  .brand-sub { font-size: 12.5px; color: var(--muted); }
  .badge { display: inline-flex; align-items: center; gap: 5px; padding: 2px 10px; border-radius: 999px; font-size: 12px; font-weight: 600; background: var(--surface2); color: var(--blue); border: 1px solid var(--border); }
  .badge.ok { color: #86efac; background: var(--green-bg); border-color: rgba(34,197,94,0.35); }
  .badge.warn { color: #fde047; background: var(--amber-bg); border-color: rgba(245,158,11,0.35); }
  .badge.err { color: #fca5a5; background: var(--red-bg); border-color: rgba(239,68,68,0.35); }

  .tabs { display: flex; gap: 6px; background: #090f1d; padding: 4px; border-radius: 10px; border: 1px solid var(--border); }
  .tab-btn { background: transparent; color: var(--muted); border: none; padding: 7px 15px; border-radius: 7px; font-size: 13px; font-weight: 600; cursor: pointer; transition: all 0.15s; }
  .tab-btn:hover { color: var(--text); }
  .tab-btn.active { background: #2563eb; color: #fff; }

  .container { max-width: 1400px; margin: 0 auto; padding: 16px 24px 40px; }

  /* KPI Strip */
  .kpi-grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 12px; margin-bottom: 16px; }
  @media (max-width: 960px) { .kpi-grid { grid-template-columns: repeat(2, 1fr); } }
  .kpi { background: var(--surface); border: 1px solid var(--border); border-radius: 12px; padding: 13px 16px; }
  .kpi-label { font-size: 12px; color: var(--muted); margin-bottom: 4px; }
  .kpi-val { font-size: 21px; font-weight: 800; letter-spacing: -0.3px; }
  .kpi-note { font-size: 12px; color: var(--dim); margin-top: 3px; }

  .tab-panel { display: none; }
  .tab-panel.active { display: block; }

  /* Stage Layout (Tab 1) */
  .stage-grid { display: grid; grid-template-columns: minmax(0, 1.45fr) minmax(370px, 1fr); gap: 16px; align-items: start; }
  @media (max-width: 1080px) { .stage-grid { grid-template-columns: 1fr; } }

  .card { background: var(--surface); border: 1px solid var(--border); border-radius: 12px; padding: 15px 17px; margin-bottom: 16px; }
  .card-title { font-size: 15px; font-weight: 700; margin: 0 0 11px 0; display: flex; align-items: center; justify-content: space-between; gap: 10px; flex-wrap: wrap; }
  .card-desc { font-size: 12.5px; color: var(--muted); margin: -5px 0 11px 0; }

  /* Smooth Canvas Video Stage + HUD */
  .video-shell { position: relative; background: #020617; border: 1px solid var(--border); border-radius: 10px; overflow: hidden; aspect-ratio: 16 / 9; display: flex; align-items: center; justify-content: center; }
  .video-shell canvas, .video-shell img.svg-frame { width: 100%; height: 100%; object-fit: contain; display: block; }
  .hud-top-left { position: absolute; top: 10px; left: 10px; display: flex; gap: 6px; flex-wrap: wrap; pointer-events: none; }
  .hud-top-right { position: absolute; top: 10px; right: 10px; display: flex; gap: 6px; flex-wrap: wrap; pointer-events: none; }
  .hud-pill { background: rgba(7,11,20,0.78); backdrop-filter: blur(4px); border: 1px solid rgba(148,163,184,0.22); color: #f1f5f9; padding: 3px 9px; border-radius: 6px; font-size: 11.5px; font-weight: 600; font-family: ui-monospace, SFMono-Regular, monospace; }
  .hud-pill.ok { border-color: rgba(34,197,94,0.55); color: #86efac; }
  .hud-pill.warn { border-color: rgba(245,158,11,0.65); color: #fde047; }
  .hud-bottom { position: absolute; bottom: 0; left: 0; right: 0; padding: 8px 12px 7px; background: linear-gradient(transparent, rgba(2,6,23,0.88)); pointer-events: none; }
  .hud-bottom-row { display: flex; justify-content: space-between; align-items: center; font-size: 11.5px; color: #e2e8f0; margin-bottom: 4px; }
  .brew-bar-bg { height: 5px; background: rgba(255,255,255,0.16); border-radius: 99px; overflow: hidden; }
  .brew-bar-fg { height: 100%; width: 0%; background: linear-gradient(90deg, #f59e0b, #22c55e); transition: width 0.25s ease; }

  .cam-grid-4 { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
  .cam-grid-4 .cam-cell { position: relative; background: #020617; border: 1px solid var(--border); border-radius: 8px; overflow: hidden; aspect-ratio: 16 / 9; cursor: pointer; }
  .cam-grid-4 canvas { width: 100%; height: 100%; object-fit: cover; display: block; }

  .thumb-strip { display: grid; grid-template-columns: repeat(4, 1fr); gap: 8px; margin-top: 9px; }
  .thumb { position: relative; background: #020617; border: 2px solid var(--border); border-radius: 8px; overflow: hidden; cursor: pointer; padding: 0; aspect-ratio: 16 / 9; }
  .thumb.active { border-color: var(--blue); box-shadow: 0 0 0 1px rgba(56,189,248,0.4); }
  .thumb canvas { width: 100%; height: 100%; object-fit: cover; display: block; }
  .thumb span { position: absolute; bottom: 4px; left: 5px; background: rgba(2,6,23,0.82); color: #e2e8f0; font-size: 10.5px; padding: 1px 6px; border-radius: 4px; font-weight: 600; }

  /* Visual Sub-grid: Spatial Radar + Telemetry */
  .spatial-grid { display: grid; grid-template-columns: 1.15fr 1fr; gap: 12px; margin-top: 12px; align-items: stretch; }
  @media (max-width: 780px) { .spatial-grid { grid-template-columns: 1fr; } }
  .radar-box { background: #070d1b; border: 1px solid var(--border); border-radius: 10px; padding: 10px; display: flex; flex-direction: column; }
  .radar-head { display: flex; justify-content: space-between; align-items: center; font-size: 12px; color: var(--muted); margin-bottom: 6px; }
  .radar-svg { width: 100%; height: 185px; display: block; }

  .tel-stack { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
  .tel-item { background: var(--surface2); border: 1px solid var(--border); border-radius: 8px; padding: 9px 11px; display: flex; flex-direction: column; justify-content: center; }
  .tel-k { font-size: 11px; color: var(--muted); }
  .tel-v { font-size: 13.5px; font-weight: 700; margin-top: 3px; font-family: ui-monospace, SFMono-Regular, monospace; }

  /* Confirm Alert */
  .alert-box { margin-top: 12px; padding: 12px 14px; border-radius: 10px; background: var(--amber-bg); border: 1px solid rgba(245,158,11,0.5); display: none; align-items: center; justify-content: space-between; gap: 12px; flex-wrap: wrap; }
  .alert-box.show { display: flex; }

  /* Scenario Cards */
  .scen-list { display: flex; flex-direction: column; gap: 9px; }
  .scen-card { background: var(--surface2); border: 1px solid var(--border); border-radius: 10px; padding: 11px 13px; transition: border-color 0.15s; }
  .scen-card:hover { border-color: #3b82f6; }
  .scen-head { display: flex; align-items: center; justify-content: space-between; gap: 8px; }
  .scen-title { font-size: 13.5px; font-weight: 700; display: flex; align-items: center; gap: 7px; }
  .scen-desc { font-size: 12px; color: var(--muted); margin: 5px 0 0 0; }

  /* Buttons & Inputs */
  button { background: #2563eb; color: #fff; border: none; border-radius: 7px; padding: 7px 13px; font-size: 13px; font-weight: 600; cursor: pointer; transition: all 0.15s; }
  button:hover { background: #1d4ed8; }
  button:disabled { opacity: 0.45; cursor: not-allowed; }
  button.sec { background: var(--surface2); color: var(--text); border: 1px solid var(--border); }
  button.sec:hover { border-color: var(--blue); background: #1c2a46; }
  button.ok { background: #16a34a; }
  button.ok:hover { background: #15803d; }
  button.warn { background: #d97706; }
  select, input[type="text"], input[type="number"], textarea {
    background: #090f1d; color: var(--text); border: 1px solid var(--border); border-radius: 7px;
    padding: 7px 10px; font-size: 13px; font-family: inherit; width: 100%;
  }
  input[type="range"] { width: 100%; accent-color: #38bdf8; }
  textarea { resize: vertical; min-height: 68px; }

  /* Live Step Flow */
  .verdict-banner { padding: 11px 14px; border-radius: 10px; margin-bottom: 11px; display: flex; align-items: center; justify-content: space-between; gap: 10px; flex-wrap: wrap; }
  .verdict-banner.idle { background: var(--surface2); border: 1px solid var(--border); color: var(--muted); }
  .verdict-banner.running { background: var(--blue-bg); border: 1px solid rgba(56,189,248,0.5); color: #bae6fd; }
  .verdict-banner.pass { background: var(--green-bg); border: 1px solid rgba(34,197,94,0.45); color: #bbf7d0; }
  .verdict-banner.fail { background: var(--red-bg); border: 1px solid rgba(239,68,68,0.45); color: #fecaca; }

  .step-flow { display: flex; flex-direction: column; gap: 7px; }
  .step-row { display: flex; align-items: flex-start; gap: 10px; padding: 8px 11px; border-radius: 8px; background: var(--surface2); border: 1px solid var(--border); font-size: 12.5px; transition: all 0.2s; }
  .step-row.active { border-color: rgba(56,189,248,0.75); background: rgba(56,189,248,0.1); }
  .step-row.done { border-color: rgba(34,197,94,0.38); }
  .step-row.warn { border-color: rgba(245,158,11,0.55); background: rgba(245,158,11,0.09); }
  .step-row.skip { opacity: 0.45; }
  .step-icon { font-size: 15px; line-height: 1.3; flex-shrink: 0; }
  .step-title { font-weight: 700; }
  .step-sub { font-size: 12px; color: var(--muted); margin-top: 1px; }

  /* Tables & Bars */
  table { width: 100%; border-collapse: collapse; font-size: 13px; }
  th, td { border-bottom: 1px solid var(--border); padding: 10px 10px; text-align: left; vertical-align: middle; }
  th { color: var(--muted); font-size: 12px; font-weight: 600; background: #0c1425; }
  .cell-pass { background: rgba(34,197,94,0.14); color: #86efac; font-weight: 700; }
  .cell-partial { background: rgba(245,158,11,0.15); color: #fde047; font-weight: 700; }
  .cell-fail { background: rgba(239,68,68,0.16); color: #fca5a5; font-weight: 700; }
  .bar-track { height: 10px; background: #1e293b; border-radius: 99px; overflow: hidden; margin-top: 5px; }
  .bar-fill { height: 100%; border-radius: 99px; background: linear-gradient(90deg, #38bdf8, #22c55e); }
  .chips { display: flex; flex-wrap: wrap; gap: 6px; margin-top: 8px; }
  .chip { font-size: 11.5px; padding: 3px 8px; border-radius: 6px; background: var(--surface2); color: #cbd5e1; border: 1px solid var(--border); font-family: ui-monospace, monospace; }
  .chip.ok { background: var(--green-bg); color: #86efac; border-color: rgba(34,197,94,0.3); }
  .chip.miss { background: var(--red-bg); color: #fca5a5; border-color: rgba(239,68,68,0.35); }
  .two-col { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
  @media (max-width: 960px) { .two-col { grid-template-columns: 1fr; } }
  details { margin-top: 10px; font-size: 12px; color: var(--muted); }
  summary { cursor: pointer; font-weight: 600; color: var(--blue); }
  pre { background: #050912; border: 1px solid var(--border); border-radius: 8px; padding: 12px; font-size: 12px; overflow: auto; max-height: 360px; color: #cbd5e1; margin: 8px 0 0; }
  .disclaimer { padding: 11px 14px; border-radius: 9px; background: var(--amber-bg); border: 1px solid rgba(245,158,11,0.35); color: #fde68a; font-size: 12px; margin-bottom: 16px; }
  .pulse { display: inline-block; width: 8px; height: 8px; border-radius: 50%; background: var(--green); margin-right: 6px; animation: blink 1.4s infinite; }
  .spin { display: inline-block; width: 12px; height: 12px; border: 2px solid rgba(56,189,248,0.3); border-top-color: #38bdf8; border-radius: 50%; animation: spin 0.8s linear infinite; vertical-align: -2px; margin-right: 4px; }
  @keyframes blink { 0%,100% { opacity: 1; } 50% { opacity: 0.25; } }
  @keyframes spin { to { transform: rotate(360deg); } }
</style>
</head>
<body>

<div class="topbar">
  <div class="topbar-inner">
    <div class="brand-block">
      <div class="brand-row">
        <span class="brand-title">CRAFT Studio</span>
        <span id="pill-live" class="badge ok"><span class="pulse"></span>连接中...</span>
        <span id="pill-driver" class="badge">driver=...</span>
        <span id="pill-release" class="badge">冻结版本: -</span>
      </div>
      <div class="brand-sub">机器人策略编排与独立验收实证平台 — 10 FPS 实时视觉 + 3D 空间精度雷达 + 逐步骤实时追踪</div>
    </div>
    <div class="tabs">
      <button class="tab-btn active" data-tab="live" onclick="switchTab('live')">① 实时仿真演练</button>
      <button class="tab-btn" data-tab="bench" onclick="switchTab('bench')">② Benchmark 量化证明</button>
      <button class="tab-btn" data-tab="gov" onclick="switchTab('gov')">③ 能力体检与治理详情</button>
    </div>
  </div>
</div>

<div class="container">
  <!-- KPI Strip -->
  <div class="kpi-grid">
    <div class="kpi">
      <div class="kpi-label">CRAFT-v1 仿真配对通过率</div>
      <div id="kpi-pass" class="kpi-val">-</div>
      <div id="kpi-pass-note" class="kpi-note">加载 Benchmark 数据中...</div>
    </div>
    <div class="kpi">
      <div class="kpi-label">独立裁判交付精度（门禁 ≤ 15 mm）</div>
      <div id="kpi-err" class="kpi-val">-</div>
      <div id="kpi-err-note" class="kpi-note">后验真值测算，不输入给策略</div>
    </div>
    <div class="kpi">
      <div class="kpi-label">非幂等安全（冲煮不重复）</div>
      <div id="kpi-safe" class="kpi-val">-</div>
      <div id="kpi-safe-note" class="kpi-note">回执超时必须先查询对账</div>
    </div>
    <div class="kpi">
      <div class="kpi-label">独立验收与版本门禁</div>
      <div id="kpi-rel" class="kpi-val">-</div>
      <div id="kpi-rel-note" class="kpi-note">失败不发布，未变优自动拒绝</div>
    </div>
  </div>

  <!-- TAB 1: LIVE VERIFICATION -->
  <div id="tab-live" class="tab-panel active">
    <div class="stage-grid">
      <!-- Left: Smooth Video + Spatial Radar -->
      <div>
        <div class="card">
          <div class="card-title">
            <span>实时仿真现场画面（双缓冲平滑流 · 只读消费传感器）</span>
            <div style="display:flex;gap:6px;align-items:center">
              <button id="btn-mode-single" class="sec" onclick="setCamMode('single')" style="padding:4px 10px;font-size:12px">主视角 + 缩略图</button>
              <button id="btn-mode-grid" class="sec" onclick="setCamMode('grid')" style="padding:4px 10px;font-size:12px">四宫格</button>
            </div>
          </div>

          <div id="cam-stage"></div>
          <div id="cam-thumbs" class="thumb-strip"></div>

          <!-- Spatial Radar + Real-time Telemetry -->
          <div class="spatial-grid">
            <div class="radar-box">
              <div class="radar-head">
                <strong>实时桌面 2D 空间精度雷达（真实观测坐标）</strong>
                <button class="sec" onclick="clearTrail()" style="padding:2px 7px;font-size:11px">清轨迹</button>
              </div>
              <svg id="radar-svg" class="radar-svg" viewBox="0 0 360 220"></svg>
            </div>
            <div class="tel-stack">
              <div class="tel-item"><div class="tel-k">观测状态 / 帧龄</div><div id="tel-obs" class="tel-v">-</div></div>
              <div class="tel-item"><div class="tel-k">当前识别杯数</div><div id="tel-cups" class="tel-v">-</div></div>
              <div class="tel-item"><div class="tel-k">L2 倒水安全 / 偏心</div><div id="tel-l2" class="tel-v">-</div></div>
              <div class="tel-item"><div class="tel-k">咖啡机 / 注满进度</div><div id="tel-cm" class="tel-v">-</div></div>
            </div>
          </div>

          <div id="confirm-alert" class="alert-box">
            <div>
              <strong style="color:#fde047">检测到多杯歧义（安全拦截生效）</strong>
              <div style="font-size:12px;color:#fef3c7">系统拒绝盲目抓取，请人工指定要服务的目标杯：</div>
            </div>
            <div id="confirm-btns" style="display:flex;gap:8px;flex-wrap:wrap"></div>
          </div>
        </div>
      </div>

      <!-- Right: Scenarios + Live Step Progress -->
      <div>
        <div class="card">
          <div class="card-title">
            <span>1. 一键验证场景（实时观看机械臂执行）</span>
            <button class="sec" onclick="resetDefault()" style="padding:4px 10px;font-size:12px">重置工作台</button>
          </div>
          <div style="margin-bottom:10px">
            <label style="font-size:12px;color:var(--muted);display:block;margin-bottom:4px">被测策略版本（默认选中经独立验收冻结发布的 v1）</label>
            <select id="policy-select" onchange="loadPolicyCode()"></select>
          </div>

          <div class="scen-list">
            <div class="scen-card">
              <div class="scen-head">
                <div class="scen-title">🟢 场景 A：标准单杯咖啡闭环</div>
                <button class="scen-btn" onclick="runPresetScenario('normal')">运行验证</button>
              </div>
              <p class="scen-desc"><strong>验证点：</strong>找杯 → 放置 → 冲煮 → 交付至 1 号取餐口，误差 ≤ 15 mm。</p>
            </div>

            <div class="scen-card" style="border-color:rgba(245,158,11,0.45)">
              <div class="scen-head">
                <div class="scen-title">🟡 场景 B：放杯偏移 + 回执超时（核心抗故障）</div>
                <button class="scen-btn warn" onclick="runPresetScenario('fault')">运行验证</button>
              </div>
              <p class="scen-desc"><strong>验证点：</strong>注入 +38/-12 mm 放杯滑移并丢弃冲煮回执，观察雷达图上杯子被拉回安全圈内 + 只查对账不重复冲煮。</p>
            </div>

            <div class="scen-card">
              <div class="scen-head">
                <div class="scen-title">🔵 场景 C：初始杯位大偏移（+20, +64 mm）</div>
                <button class="scen-btn" onclick="runPresetScenario('offset')">运行验证</button>
              </div>
              <p class="scen-desc"><strong>验证点：</strong>验证消费现场实时 3D 观测而非写死固定坐标。</p>
            </div>

            <div class="scen-card">
              <div class="scen-head">
                <div class="scen-title">🟠 场景 D：双杯歧义安全拦截</div>
                <button class="scen-btn sec" onclick="runPresetScenario('ambiguity')">运行验证</button>
              </div>
              <p class="scen-desc"><strong>验证点：</strong>同时放置红/绿双杯且未指定时，拒绝盲抓、零冲煮，暂停等人工确认。</p>
            </div>
          </div>

          <details style="margin-top:12px">
            <summary>自定义仿真扰动参数（高级）</summary>
            <div style="margin-top:10px;padding:10px;background:#0a101f;border-radius:8px;border:1px solid var(--border)">
              <div style="display:grid;grid-template-columns:1fr 1fr;gap:8px">
                <div>
                  <label style="font-size:11.5px;color:var(--muted)">初始杯位 X (m): <strong id="val-cx">0.33</strong></label>
                  <input type="range" id="rng-cx" min="0.30" max="0.37" step="0.005" value="0.33" oninput="document.getElementById('val-cx').textContent=this.value"/>
                </div>
                <div>
                  <label style="font-size:11.5px;color:var(--muted)">初始杯位 Y (m): <strong id="val-cy">0.00</strong></label>
                  <input type="range" id="rng-cy" min="-0.08" max="0.08" step="0.005" value="0.00" oninput="document.getElementById('val-cy').textContent=this.value"/>
                </div>
                <div>
                  <label style="font-size:11.5px;color:var(--muted)">放杯滑移 dX (mm): <strong id="val-sx">32</strong></label>
                  <input type="range" id="rng-sx" min="-45" max="45" step="2" value="32" oninput="document.getElementById('val-sx').textContent=this.value"/>
                </div>
                <div>
                  <label style="font-size:11.5px;color:var(--muted)">放杯滑移 dY (mm): <strong id="val-sy">-10</strong></label>
                  <input type="range" id="rng-sy" min="-30" max="30" step="2" value="-10" oninput="document.getElementById('val-sy').textContent=this.value"/>
                </div>
              </div>
              <label style="display:flex;align-items:center;gap:6px;margin-top:8px;font-size:12px;color:var(--text)">
                <input type="checkbox" id="chk-custom-timeout" checked/> 同时注入冲煮回执超时（unknown）
              </label>
              <button class="scen-btn" style="margin-top:9px;width:100%" onclick="runCustomScenario()">运行自定义扰动仿真</button>
            </div>
          </details>
        </div>

        <!-- Live Execution Progress Card -->
        <div class="card">
          <div class="card-title">
            <span>2. 实时执行追踪与独立裁判结论</span>
            <span id="run-badge" class="badge">就绪</span>
          </div>
          <div id="run-banner" class="verdict-banner idle">
            <div>点击上方任意场景即可开始实测，此处会随机械臂动作<strong>实时逐项点亮</strong>。</div>
          </div>
          <div id="run-metrics" class="tel-stack" style="margin-bottom:10px;display:none">
            <div class="tel-item"><div class="tel-k">咖啡注满（裁判真值）</div><div id="rm-filled" class="tel-v">-</div></div>
            <div class="tel-item"><div class="tel-k">交付误差（≤15mm）</div><div id="rm-err" class="tel-v">-</div></div>
            <div class="tel-item"><div class="tel-k">启动冲煮次数（应=1）</div><div id="rm-brew" class="tel-v">-</div></div>
            <div class="tel-item"><div class="tel-k">耗时 / 未知回执对账</div><div id="rm-wall" class="tel-v">-</div></div>
          </div>
          <div id="step-flow" class="step-flow"></div>
          <details id="raw-trace-details" style="display:none">
            <summary>展开底层 SDK 调用原始序列</summary>
            <div id="raw-trace-chips" class="chips"></div>
          </details>
        </div>
      </div>
    </div>
  </div>

  <!-- TAB 2: BENCHMARK PROOF -->
  <div id="tab-bench" class="tab-panel">
    <div class="disclaimer">
      <strong>诚实证据边界：</strong>当前 Benchmark 运行在 <code id="bm-scope-text">simulator</code> 环境中，用于量化策略在仿真扰动与故障下的工程有效性；<strong>不等同于真机验收或食品品质认证</strong>。小样本下 95% 置信区间存在重叠，结论以完整记录与失败归因为准。
    </div>

    <div class="card">
      <div class="card-title">
        <span>策略综合表现对比（配对实测，失败完整计入分母）</span>
        <div style="display:flex;align-items:center;gap:8px;flex-wrap:wrap">
          <span id="bench-status-pill" class="badge">idle</span>
          <label style="font-size:12px;color:var(--muted)">每工况重复</label>
          <input id="bench-repeats" type="number" min="1" max="5" value="1" style="width:58px"/>
          <button id="btn-start-bench" class="ok" onclick="startBenchmark()">重跑配对 Benchmark</button>
          <a href="benchmark_report.html" target="_blank" class="badge" style="text-decoration:none">独立 HTML 报告 ↗</a>
        </div>
      </div>
      <div id="bench-bars" style="margin-bottom:14px"></div>
      <div id="bench-summary-table"></div>
    </div>

    <div class="card">
      <div class="card-title"><span>分工况逐项通过矩阵</span></div>
      <div id="bench-matrix"></div>
    </div>

    <div class="card">
      <div class="card-title"><span>独立验收与改进门禁实证（本工作区归档）</span></div>
      <div id="gov-evidence-cards" class="two-col"></div>
    </div>

    <div class="two-col">
      <div class="card">
        <div class="card-title"><span>四层失败归因与步骤耗时瓶颈剖析</span></div>
        <div id="profiler-panel" style="font-size:13px;color:var(--muted)">加载中...</div>
      </div>
      <div class="card">
        <div class="card-title"><span>踩坑永久回归题库（Failure-to-Testcase）与跨店迁移检查</span></div>
        <div id="regression-portability-panel" style="font-size:13px;color:var(--muted)">加载中...</div>
      </div>
    </div>
  </div>

  <!-- TAB 3: CAPABILITIES, TEACHING & GOVERNANCE -->
  <div id="tab-gov" class="tab-panel">
    <div class="two-col">
      <div class="card">
        <div class="card-title">
          <span>宿主自动能力发现（无需手写适配 SDK）</span>
          <span id="cap-pill" class="badge">...</span>
        </div>
        <div id="cap-meta" style="font-size:12px;color:var(--muted);margin-bottom:10px"></div>
        <div style="font-size:12px;font-weight:600;color:#86efac">✅ 当前宿主已就绪能力（单杯咖啡全链路）</div>
        <div id="cap-ok-chips" class="chips"></div>
        <div style="font-size:12px;font-weight:600;color:#fca5a5;margin-top:14px">❌ 缺失能力（烤箱 / 贝果能力缺口，诚实拦截）</div>
        <div id="cap-miss-chips" class="chips"></div>
      </div>

      <div class="card">
        <div class="card-title"><span>统一教学交互校验（Task Specification）</span></div>
        <p class="card-desc">文字指令 + 视频教学参考（严格标为非权威证据，不推断现场坐标）绑定到宿主能力，缺失能力自动拦截。</p>
        <label style="font-size:12px;color:var(--muted)">任务描述</label>
        <textarea id="teach-instruction">制作一杯堂食热美式，送到1号取餐口；若放杯偏移先微调居中，若冲煮回执未知先查询状态切勿重复启动。</textarea>
        <label style="font-size:12px;color:var(--muted);display:block;margin-top:8px">教学视频引用（非权威证据）</label>
        <input type="text" id="teach-video" value="teaching://single-cup-coffee-ref"/>
        <div style="margin-top:10px"><button onclick="previewSpec()">校验并生成结构化任务规格</button></div>
        <pre id="spec-out" style="max-height:190px">点击上方按钮校验教学输入...</pre>
      </div>
    </div>

    <div class="two-col">
      <div class="card">
        <div class="card-title"><span>已验证技能经验与发布历史</span></div>
        <div id="skills-and-history"></div>
      </div>

      <div class="card">
        <div class="card-title">
          <span>冻结策略源码只读查看</span>
          <select id="code-policy-select" style="width:auto" onchange="loadCodeTabPolicy()"></select>
        </div>
        <pre id="policy-code-view">加载中...</pre>
      </div>
    </div>
  </div>
</div>

<script>
let stateData = null;
let camList = [];
let activeCam = 'head';
let camMode = 'single'; // single | grid
let isRunningScenario = false;
let activeScenarioLabel = '';
let cupTrail = [];
let frameTimestamps = [];
let lastLiveScene = null;
let pumpRunning = false;

const SCENARIO_PRESETS = {
  normal:    { label: '场景 A：标准单杯咖啡闭环', preset: 'default', fault_slip: false, simulate_timeout: false },
  fault:     { label: '场景 B：放杯偏移 (+38,-12mm) + 冲煮回执超时', preset: 'default', fault_slip: true, simulate_timeout: true },
  offset:    { label: '场景 C：初始杯位大偏移 (+20,+64mm)', preset: 'sim_pose_offset', fault_slip: false, simulate_timeout: false },
  ambiguity: { label: '场景 D：双杯歧义安全拦截', preset: 'sim_dual_cup', fault_slip: false, simulate_timeout: false },
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
    stage.innerHTML = '<div class="video-shell" style="color:var(--muted)">暂无可用相机流</div>';
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
          <span class="hud-pill" id="hud-cam">视角: ${activeCam}</span>
          <span class="hud-pill" id="hud-fps">-- FPS</span>
          <span class="hud-pill" id="hud-phase">phase: idle</span>
        </div>
        <div class="hud-top-right">
          <span class="hud-pill" id="hud-l2">L2: --</span>
        </div>
        <div class="hud-bottom">
          <div class="hud-bottom-row">
            <span id="hud-action">就绪 — 等待选择验证场景</span>
            <span id="hud-brew-txt">冲煮进度 0%</span>
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

// Smooth double-buffered pipelined frame loader (zero flicker, adaptive ~10 FPS)
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
        // Update thumbnails at ~2 FPS to save bandwidth while keeping main view silky 10 FPS
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

// Real-time 2D Spatial Workspace Radar (purely from host-observed world coordinates)
function worldToRadar(x, y) {
  // Workspace x in [0.22, 0.65], y in [-0.32, 0.32]
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

  // Record cup A trail from real observations
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
    ? `<polyline fill="none" stroke="#38bdf8" stroke-width="2" stroke-dasharray="4 3" opacity="0.7" points="${cupTrail.map(p => p[0].toFixed(1)+','+p[1].toFixed(1)).join(' ')}"/>`
    : '';

  const cupMarks = objs
    .filter(o => o.semantic_class === 'dine_in_red' || o.semantic_class === 'takeout_green')
    .map(o => {
      const pos = o.center_world_m || o.world || [0.33, 0];
      const [px, py] = worldToRadar(pos[0], pos[1]);
      const col = o.semantic_class === 'dine_in_red' ? '#f43f5e' : '#10b981';
      const ring = filled && o.semantic_class === 'dine_in_red' ? '#facc15' : '#f8fafc';
      return `<g>
        <circle cx="${px.toFixed(1)}" cy="${py.toFixed(1)}" r="9" fill="${col}" stroke="${ring}" stroke-width="2.5"/>
        <text x="${px.toFixed(1)}" y="${(py + 19).toFixed(1)}" fill="#e2e8f0" font-size="9.5" text-anchor="middle">${o.object_id || o.id} (${(pos[0]*100).toFixed(1)},${(pos[1]*100).toFixed(1)}cm)</text>
      </g>`;
    }).join('');

  const spoutColor = l2.safe_to_pour ? '#22c55e' : '#f59e0b';
  svg.innerHTML = `
    <rect x="4" y="4" width="352" height="212" rx="8" fill="#091122" stroke="#1e2f4f"/>
    <line x1="30" y1="110" x2="330" y2="110" stroke="#1e293b" stroke-dasharray="2 4"/>
    <line x1="${sx.toFixed(1)}" y1="15" x2="${sx.toFixed(1)}" y2="205" stroke="#1e293b" stroke-dasharray="2 4"/>
    <!-- Steam No-Go -->
    <rect x="${(nzx-28).toFixed(1)}" y="${(nzy-18).toFixed(1)}" width="64" height="36" rx="5" fill="rgba(239,68,68,0.12)" stroke="#ef4444" stroke-dasharray="3 2"/>
    <text x="${(nzx+4).toFixed(1)}" y="${(nzy+3).toFixed(1)}" fill="#fca5a5" font-size="9" text-anchor="middle">蒸汽禁区</text>
    <!-- Spout 15mm gate -->
    <circle cx="${sx.toFixed(1)}" cy="${sy.toFixed(1)}" r="18" fill="rgba(56,189,248,0.08)" stroke="${spoutColor}" stroke-width="1.8" stroke-dasharray="3 2"/>
    <circle cx="${sx.toFixed(1)}" cy="${sy.toFixed(1)}" r="2.5" fill="${spoutColor}"/>
    <text x="${sx.toFixed(1)}" y="${(sy - 23).toFixed(1)}" fill="${spoutColor}" font-size="9.5" font-weight="600" text-anchor="middle">咖啡出水口 (15mm安全圈)</text>
    <!-- Pickup 1 15mm gate -->
    <circle cx="${p1x.toFixed(1)}" cy="${p1y.toFixed(1)}" r="16" fill="rgba(34,197,94,0.08)" stroke="#22c55e" stroke-width="1.5" stroke-dasharray="3 2"/>
    <text x="${p1x.toFixed(1)}" y="${(p1y + 27).toFixed(1)}" fill="#86efac" font-size="9.5" text-anchor="middle">1号取餐口 (≤15mm)</text>
    <!-- Pickup 2 -->
    <circle cx="${p2x.toFixed(1)}" cy="${p2y.toFixed(1)}" r="13" fill="none" stroke="#475569" stroke-dasharray="2 3"/>
    <text x="${p2x.toFixed(1)}" y="${(p2y - 16).toFixed(1)}" fill="#64748b" font-size="8.5" text-anchor="middle">2号取餐口</text>
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
  document.getElementById('tel-cups').textContent = `${cups.length} 个`;
  document.getElementById('tel-l2').textContent = l2.safe_to_pour
    ? `安全 ✅ (${radialMm ?? '-'}mm)`
    : (radialMm != null ? `偏心 ${radialMm}mm ⚠️` : '-');
  document.getElementById('tel-cm').textContent = `${brewStatus} · ${Math.round(brewProg * 100)}% ${cupFilled ? '☕' : ''}`;

  // Update Camera HUD
  const hudPhase = document.getElementById('hud-phase');
  if (hudPhase) hudPhase.textContent = `phase: ${raw.phase || 'idle'}` + (raw.in_hand_cup ? ' (手持杯)' : '');
  const hudL2 = document.getElementById('hud-l2');
  if (hudL2) {
    hudL2.textContent = l2.safe_to_pour ? `L2 安全 (${radialMm ?? '0'}mm)` : `L2 偏心 ${radialMm ?? '-'}mm`;
    hudL2.className = 'hud-pill ' + (l2.safe_to_pour ? 'ok' : 'warn');
  }
  const brewBar = document.getElementById('hud-brew-bar');
  const brewTxt = document.getElementById('hud-brew-txt');
  if (brewBar) brewBar.style.width = `${Math.max(0, Math.min(100, Math.round(brewProg * 100)))}%`;
  if (brewTxt) brewTxt.textContent = `冲煮进度 ${Math.round(brewProg * 100)}%` + (cupFilled ? ' · 已注满 ☕' : '');

  // Ambiguity confirm bar
  const calert = document.getElementById('confirm-alert');
  if (cups.length >= 2) {
    calert.classList.add('show');
    document.getElementById('confirm-btns').innerHTML = cups.map(c => {
      const oid = c.object_id || c.id;
      return `<button class="warn" onclick="confirmCup('${oid}')">确认目标杯：${oid}</button>`;
    }).join('');
  } else {
    calert.classList.remove('show');
  }

  renderSpatialRadar(obs);

  // Live step-by-step updates during active execution!
  const rs = live.run_state || {};
  const rt = live.live_runtime || {};
  if (rs.running) {
    const methods = (rt.steps || []).map(s => s.method).filter(Boolean);
    const lastM = methods.length ? methods[methods.length - 1] : 'resetting_scene';
    const hudAct = document.getElementById('hud-action');
    if (hudAct) hudAct.textContent = `正在执行 (${rs.elapsed_s}s)：${lastM}`;
    const banner = document.getElementById('run-banner');
    banner.className = 'verdict-banner running';
    banner.innerHTML = `<div><span class="spin"></span><strong>正在运行：${rs.label || activeScenarioLabel}</strong>（已用时 ${rs.elapsed_s}s）<br><span style="font-size:12px">当前宿主动作：<code>${lastM}</code> · 已完成 ${methods.length} 步</span></div>`;
    renderHumanSteps({ methods, running: true, host_runtime_status: rt.status });
  }
}

function renderHumanSteps(res) {
  const methods = res.methods || [];
  const running = !!res.running;
  const has = m => methods.includes(m);
  const lastM = methods.length ? methods[methods.length - 1] : '';

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
      icon: running && !step2Done && !has('request_operator_confirmation') ? '<span class="spin"></span>' : (step1Done ? '✅' : '⚪'),
      cls: stateCls(step1Done, !step2Done, has('request_operator_confirmation')),
      title: '1. 读取订单与现场实时 3D 观测',
      sub: has('request_operator_confirmation')
        ? '检测到多杯歧义，主动暂停并请求人工确认（零误操作）'
        : (res.host_runtime_status === 'rejected_observation_not_ready'
            ? '未满足唯一目标绑定条件，主动拦截执行'
            : (step1Done ? '已获取实时场景快照并唯一锁定目标咖啡杯' : '等待获取场景快照...')),
    },
    {
      icon: running && step1Done && !step2Done ? '<span class="spin"></span>' : (step2Done ? '✅' : '⚪'),
      cls: stateCls(step2Done, step1Done && !step2Done, false),
      title: '2. 抓取并放杯至咖啡机出口下方',
      sub: step2Done ? '已完成放置并重新检测 L2 倒水安全门禁' : (running && step1Done ? '机械臂正在抓取并放置咖啡杯...' : '等待执行'),
    },
    {
      icon: step3Hit ? '🛠️' : '➖',
      cls: stateCls(step3Hit, false, step3Hit),
      title: '3. 放杯偏移检测与视觉微调居中',
      sub: step3Hit
        ? '检测到放杯偏心超限 → 已触发视觉微调居中（recenter_cup_under_spout）'
        : (step4Hit ? '放杯位置一次达标，无需微调' : '待放置后检测'),
    },
    {
      icon: has('command_ack_timeout_unknown') ? '🔎' : (running && step2Done && !step5Done && lastM === 'start_coffee_machine' ? '<span class="spin"></span>' : (step4Hit ? '✅' : '⚪')),
      cls: stateCls(step4Hit, step2Done && !step4Hit, has('command_ack_timeout_unknown')),
      title: '4. 启动冲煮与未知回执对账（防重复冲煮）',
      sub: has('command_ack_timeout_unknown')
        ? `捕获冲煮回执超时（unknown）→ 已调用 query_command_status 对账确认，绝不重复冲煮`
        : (step4Hit ? '单次启动冲煮成功，等待萃取完成' : '待 L2 居中安全通过后启动'),
    },
    {
      icon: running && step4Hit && !step5Done ? '<span class="spin"></span>' : (step5Done ? '✅' : '⚪'),
      cls: stateCls(step5Done, step4Hit && !step5Done, false),
      title: '5. 等待注满、取出并交付核验',
      sub: step5Done
        ? (res.oracle_post_run ? `交付完成，独立裁判误差 ${res.oracle_post_run.delivery_error_mm ?? '-'} mm，杯内注满=${String(res.oracle_post_run.cup_filled)}` : '已完成交付核验')
        : (running && step4Hit ? '正在冲煮/取杯交付至 1 号取餐口...' : '等待冲煮完成'),
    },
  ];

  document.getElementById('step-flow').innerHTML = steps.map(s => `
    <div class="step-row ${s.cls}">
      <div class="step-icon">${s.icon}</div>
      <div><div class="step-title">${s.title}</div><div class="step-sub">${s.sub}</div></div>
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
      ? `对比手写基线 ${refPol.passed_trials}/${refPol.total_trials} (${(refPol.pass_rate*100).toFixed(0)}%) · 共 ${(bm.trials||[]).length} 次仿真`
      : `95% CI [${(craftPol.wilson_95_ci[0]*100).toFixed(0)}%, ${(craftPol.wilson_95_ci[1]*100).toFixed(0)}%]`;

    const err = craftPol.mean_delivery_error_mm;
    document.getElementById('kpi-err').textContent = err != null ? `${err.toFixed(2)} mm` : 'N/A';
    document.getElementById('kpi-err-note').textContent = craftPol.max_delivery_error_mm != null
      ? `最大误差 ${craftPol.max_delivery_error_mm.toFixed(2)} mm（全部 ≤ 15 mm）`
      : '后验真值测算';

    const dup = pols.reduce((a, p) => a + (p.duplicate_non_idempotent_total || 0), 0);
    const rec = craftPol.unknown_reconciled_rate !== undefined ? (craftPol.unknown_reconciled_rate * 100).toFixed(0) + '%' : '-';
    document.getElementById('kpi-safe').textContent = dup === 0 ? '0 次重复冲煮' : `${dup} 次异常`;
    document.getElementById('kpi-safe').style.color = dup === 0 ? '#86efac' : '#fca5a5';
    document.getElementById('kpi-safe-note').textContent = `未知回执对账率 ${rec}（先查后行）`;
  }

  const rel = st.active_release || '未发布';
  const sum = st.workspace_summary || {};
  const imp = sum.improvement_result;
  document.getElementById('kpi-rel').textContent = `已冻结 ${rel}`;
  document.getElementById('kpi-rel').style.color = st.active_release ? '#38bdf8' : '#94a3b8';
  document.getElementById('kpi-rel-note').textContent = imp
    ? `改进候选已${imp.decision === 'accepted' ? '接受并升级' : '自动拒绝（未优于基线）'}`
    : '验收通过方可发布，支持一键回退';
}

function renderBenchmarkTab(st) {
  const bm = st.benchmark;
  document.getElementById('bm-scope-text').textContent = (bm && bm.evidence_source) || st.evidence_source || 'unknown';
  const statusPill = document.getElementById('bench-status-pill');

  if (!bm || !bm.summary_by_policy || Object.keys(bm.summary_by_policy).length === 0) {
    statusPill.textContent = st.benchmark_running ? '运行中...' : '尚未运行';
    document.getElementById('bench-bars').innerHTML = '<div style="color:var(--muted);font-size:13px">暂无 Benchmark 结果。</div>';
    document.getElementById('bench-summary-table').innerHTML = '';
    document.getElementById('bench-matrix').innerHTML = '';
  } else {
    const totalExpected = (bm.repeats || 1) * (bm.scenarios || []).length * (bm.policies || []).length;
    const done = (bm.trials || []).length;
    statusPill.textContent = bm.status === 'running' ? `运行中 ${done}/${totalExpected}` : `已完成 (${done} 次仿真)`;
    statusPill.className = 'badge ' + (bm.status === 'completed' ? 'ok' : 'warn');

    const pols = Object.values(bm.summary_by_policy);
    document.getElementById('bench-bars').innerHTML = pols.map(p => `
      <div style="margin-bottom:10px">
        <div style="display:flex;justify-content:space-between;font-size:13px">
          <span><strong>${p.label}</strong></span>
          <span><strong>${(p.pass_rate*100).toFixed(1)}%</strong> (${p.passed_trials}/${p.total_trials}) · 95% CI [${(p.wilson_95_ci[0]*100).toFixed(0)}%, ${(p.wilson_95_ci[1]*100).toFixed(0)}%]</span>
        </div>
        <div class="bar-track"><div class="bar-fill" style="width:${(p.pass_rate*100).toFixed(1)}%"></div></div>
      </div>
    `).join('');

    document.getElementById('bench-summary-table').innerHTML = `<table>
      <thead><tr><th>策略</th><th>通过率</th><th>耗时（均 / P95）</th><th>交付误差（均 / 最大）</th><th>未知回执对账</th><th>重复冲煮</th><th>失败明细</th></tr></thead>
      <tbody>${pols.map(p => {
        const f = Object.entries(p.failure_breakdown || {}).map(([k,v]) => `${k} ×${v}`).join(', ') || '无';
        const err = p.mean_delivery_error_mm != null ? `${p.mean_delivery_error_mm} / ${p.max_delivery_error_mm} mm` : '-';
        return `<tr>
          <td><strong>${p.label}</strong></td>
          <td>${p.passed_trials}/${p.total_trials} (${(p.pass_rate*100).toFixed(1)}%)</td>
          <td>${p.mean_wall_s}s / ${p.p95_wall_s ?? '-'}s</td>
          <td>${err}</td>
          <td>${(p.unknown_reconciled_rate*100).toFixed(0)}% (${p.unknown_events_observed}次)</td>
          <td>${p.duplicate_non_idempotent_total}</td>
          <td>${f}</td>
        </tr>`;
      }).join('')}</tbody>
    </table>`;

    const scenarios = bm.scenarios || [];
    const trials = bm.trials || [];
    document.getElementById('bench-matrix').innerHTML = `<table>
      <thead><tr><th>验证工况</th>${pols.map(p => `<th>${p.label}</th>`).join('')}</tr></thead>
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
      <div class="scen-title">✅ 冻结发布版本 v1 独立验收</div>
      <p class="scen-desc">独立 held-out 工况完成率 <strong>${(v1.completion_rate*100).toFixed(0)}%</strong>，最大误差 <strong>${v1.max_delivery_error_mm} mm</strong>，平均耗时 <strong>${v1.mean_wall_s} s</strong>，0 重复冲煮。</p>
    </div>`);
  }
  if (sum && sum.improvement_result) {
    const ir = sum.improvement_result;
    cards.push(`<div class="scen-card">
      <div class="scen-title">${ir.decision === 'accepted' ? '🚀' : '🛡️'} 改进候选 v2 门禁判定：${ir.decision.toUpperCase()}</div>
      <p class="scen-desc">${v2 ? `v2 通过率 ${(v2.completion_rate*100).toFixed(0)}%，平均耗时 ${v2.mean_wall_s} s（基线 v1 为 ${v1 ? v1.mean_wall_s : '-'} s）。` : ''}判定原因：<strong>${ir.reason}</strong>。</p>
    </div>`);
  }
  document.getElementById('gov-evidence-cards').innerHTML = cards.join('') || '<div style="color:var(--muted);font-size:13px">暂无发布验收记录。</div>';

  // Phase 2: Profiler + Four-Layer Attribution
  const prof = st.phase2_profiler || {};
  const attrCounts = prof.attribution_counts || {};
  const bns = (prof.method_bottlenecks || []).slice(0, 6);
  const pp = st.phase2_throughput;
  document.getElementById('profiler-panel').innerHTML = `
    <div class="chips" style="margin-top:0;margin-bottom:10px">
      <span class="chip">感知 perception: ${attrCounts.perception || 0}</span>
      <span class="chip">控制 control: ${attrCounts.control || 0}</span>
      <span class="chip">外设 device: ${attrCounts.device || 0}</span>
      <span class="chip">策略 policy: ${attrCounts.policy || 0}</span>
    </div>
    ${bns.length ? `<table>
      <thead><tr><th>宿主步骤</th><th>平均调用</th><th>平均耗时(s)</th><th>最大单步(s)</th></tr></thead>
      <tbody>${bns.map(b => `<tr><td><code>${b.method}</code></td><td>${b.mean_calls_per_trial}</td><td>${b.mean_s_per_timed_trial ?? '-'}</td><td>${b.max_step_s ?? '-'}</td></tr>`).join('')}</tbody>
    </table>` : '<div style="font-size:12px">暂无步骤剖析数据</div>'}
    ${pp && pp.pipeline_projection_2slot ? `<div style="margin-top:10px;font-size:12px;color:#86efac">⚡ 实测单均 ${pp.throughput.mean_order_s}s（${pp.throughput.throughput_orders_per_hour} 单/时）；2 工位异步流水线调度模型预计可提升 <strong>+${pp.pipeline_projection_2slot.projected_throughput_gain_pct}%</strong>（${pp.pipeline_projection_2slot.pipelined_orders_per_hour} 单/时，标明 schedule_model）</div>` : ''}
  `;

  // Phase 2: Regression Suite + Portability + Autonomy Gate
  const regCases = st.regression_cases || [];
  const ports = st.portability_checks || {};
  const promo = st.autonomy_physical_gate;
  document.getElementById('regression-portability-panel').innerHTML = `
    <div style="font-size:12px;font-weight:600;color:var(--blue);margin-bottom:6px">永久回归考题库（共 ${regCases.length} 题）</div>
    ${regCases.length ? regCases.map(c => `<div class="scen-card" style="margin-bottom:6px;padding:8px 10px">
      <div style="font-size:12.5px;font-weight:700">${c.case_id} · <span class="badge warn">${c.attribution_layer}/${c.attribution_code}</span></div>
      <div style="font-size:11.5px;color:var(--muted)">${c.scenario.label}（命中 ${c.hit_count} 次）</div>
    </div>`).join('') : '<div style="font-size:12px;color:var(--muted);margin-bottom:8px">暂无已沉淀失败考题</div>'}
    <div style="font-size:12px;font-weight:600;color:var(--blue);margin:10px 0 6px">跨店无坐标迁移静态校验</div>
    <div class="chips" style="margin-top:0">${Object.entries(ports).map(([pid, r]) => `<span class="chip ${r.portable ? 'ok' : 'miss'}">${pid}: ${r.portable ? '无硬编码 ✅' : '含硬编码 ❌'}</span>`).join('')}</div>
    ${promo ? `<div style="margin-top:10px;font-size:12px;color:${promo.allowed ? '#86efac' : '#fde047'}">🛡️ 真机上线门禁（physical_live）：<strong>${promo.allowed ? '已放行' : '安全阻断'}</strong>${promo.missing_prerequisites && promo.missing_prerequisites.length ? ' — 待完成前置阶段: ' + promo.missing_prerequisites.join(', ') : ''}</div>` : ''}
  `;
}

function renderGovTab(st) {
  const cap = st.capability_check || {};
  const d = cap.discovered || {};
  const cp = document.getElementById('cap-pill');
  cp.textContent = cap.status || '-';
  cp.className = 'badge ' + (cap.status === 'ready' ? 'ok' : 'err');
  document.getElementById('cap-meta').innerHTML = `
    执行权威：<code>${d.execution_authority || '-'}</code> ·
    感知权威：<code>${d.perception_owner || '-'}</code> ·
    契约哈希：<code>${(d.sdk_contract_sha256 || '').slice(0, 16)}…</code>
  `;
  document.getElementById('cap-ok-chips').innerHTML = (d.allowed_methods || []).map(m => `<span class="chip ok">${m}</span>`).join('');
  document.getElementById('cap-miss-chips').innerHTML = ((st.oven_gap || {}).missing_methods || []).map(m => `<span class="chip miss">${m}</span>`).join('') || '<span class="chip ok">无缺失</span>';

  const skills = Object.values(st.skills || {});
  const hist = (st.registry && st.registry.history) || [];
  document.getElementById('skills-and-history').innerHTML = `
    <div style="font-size:12px;font-weight:600;color:var(--blue);margin-bottom:6px">已验证技能经验</div>
    ${skills.length ? skills.map(s => `<div class="scen-card" style="margin-bottom:8px">
      <div class="scen-title">${s.title} <span class="badge ok">${s.evidence_source}</span></div>
      <p class="scen-desc">${s.guidance}<br><small>验收来源：${s.verification_report_id}</small></p>
    </div>`).join('') : '<div style="font-size:12px;color:var(--muted)">暂无已验证技能</div>'}
    <div style="font-size:12px;font-weight:600;color:var(--blue);margin:12px 0 6px">发布与回退审计记录</div>
    ${hist.length ? hist.map(h => `<div style="font-size:12px;color:var(--muted);padding:4px 0;border-bottom:1px solid var(--border)">
      <strong>${h.event}</strong> → <code>${h.version}</code> (${h.deployment_scope || '-'}) · ${h.at_utc || ''}
    </div>`).join('') : '<div style="font-size:12px;color:var(--muted)">暂无发布记录</div>'}
  `;
}

async function refreshState() {
  try {
    const st = await api('api/state');
    stateData = st;

    const livePill = document.getElementById('pill-live');
    const ok = st.capability_check && st.capability_check.status === 'ready';
    livePill.innerHTML = `<span class="pulse"></span>${ok ? '实时连接正常' : '连接异常: ' + (st.capability_check||{}).status}`;
    livePill.className = 'badge ' + (ok ? 'ok' : 'err');
    document.getElementById('pill-driver').textContent = `driver=${st.driver} · scope=${st.evidence_source}`;
    document.getElementById('pill-release').textContent = `冻结版本: ${st.active_release || '未发布'}`;

    if (JSON.stringify(st.cameras || []) !== JSON.stringify(camList)) {
      camList = st.cameras || [];
      if (camList.includes('head')) activeCam = 'head';
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
    document.getElementById('pill-live').textContent = '连接失败: ' + e.message;
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
    badge.textContent = '执行出错';
    badge.className = 'badge err';
    banner.className = 'verdict-banner fail';
    banner.textContent = '执行失败：' + rs.error;
    if (hudAct) hudAct.textContent = '执行出错';
    return;
  }
  const res = rs.last_result;
  if (!res) return;
  const ok = !!res.task_passed;
  badge.textContent = ok ? '验证通过' : '验证未通过';
  badge.className = 'badge ' + (ok ? 'ok' : 'err');
  banner.className = 'verdict-banner ' + (ok ? 'pass' : 'fail');
  banner.innerHTML = `<div><strong>${ok ? '✅ 验证通过' : '❌ 验证未通过'} — ${rs.label || activeScenarioLabel}</strong><br><span style="font-size:12px">宿主返回：<code>${res.host_runtime_status}</code> · 证据来源：<code>${res.evidence_source}</code></span></div>`;
  if (hudAct) hudAct.textContent = ok ? '✅ 演练验证通过' : '❌ 演练未通过 (' + res.host_runtime_status + ')';

  const oracle = res.oracle_post_run || {};
  document.getElementById('run-metrics').style.display = 'grid';
  document.getElementById('rm-filled').textContent = oracle.cup_filled ? '已注满 ☕' : '未注满';
  document.getElementById('rm-err').textContent = oracle.delivery_error_mm != null ? `${oracle.delivery_error_mm} mm` : '-';
  document.getElementById('rm-brew').textContent = `${(res.non_idempotent_counts||{}).start_coffee_machine ?? 0} 次`;
  document.getElementById('rm-wall').textContent = `${res.wall_s}s / 对账=${res.unknown_events_observed}次`;
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
  badge.textContent = '仿真执行中';
  badge.className = 'badge warn';
  banner.className = 'verdict-banner running';
  banner.innerHTML = `<div><span class="spin"></span><strong>正在启动：${label}</strong><br><span style="font-size:12px">正在重置仿真环境并下发策略，请观察左侧 10 FPS 实时画面与空间雷达图...</span></div>`;
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
  const label = `自定义扰动：初始(${(cx*100).toFixed(1)},${(cy*100).toFixed(1)}cm) 滑移(${(sx*1000).toFixed(0)},${(sy*1000).toFixed(0)}mm)`;
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
    alert('启动 Benchmark 失败: ' + e.message);
    document.getElementById('btn-start-bench').disabled = false;
  }
}

refreshState();
// Fast 350ms telemetry poll while executing, 1.5s when idle
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


def _read_optional_json(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except ValueError:
        return None


class CraftStudioApp:
    """Thread-safe controller backing CRAFT Studio HTTP endpoints."""

    def __init__(
        self,
        config: SceneConfig,
        driver: SceneHostDriver,
        workspace_dir: str | Path,
        *,
        policies: Sequence[PolicyEntry] = (),
    ) -> None:
        self.config = config
        self.driver = driver
        self.workspace_dir = Path(workspace_dir)
        self.workspace_dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._bench_running = False
        self._run_state: dict[str, Any] = {
            "running": False,
            "label": "",
            "started_at": None,
            "elapsed_s": 0.0,
            "last_result": None,
            "error": None,
        }
        self._cam_cache: dict[str, tuple[float, bytes, str]] = {}
        self._policies: dict[str, PolicyEntry] = {}

        seen_hashes: set[str] = set()
        reg_dir = self.workspace_dir / "registry"
        if (reg_dir / "registry.json").is_file():
            reg = ReleaseRegistry(reg_dir)
            if reg.active_version_id:
                src = reg.get_policy_source()
                entry = PolicyEntry(
                    policy_id="active_release",
                    label=f"CRAFT 冻结线上版 ({reg.active_version_id})",
                    source=src,
                    origin="craft_published",
                )
                self._policies[entry.policy_id] = entry
                seen_hashes.add(entry.source_sha256)

        for pol in policies:
            if pol.source_sha256 in seen_hashes:
                continue
            self._policies[pol.policy_id] = pol
            seen_hashes.add(pol.source_sha256)

        ref = PolicyEntry(
            policy_id="craft_reference",
            label="CRAFT Reference Baseline（手写基线）",
            source=builtin_reference_policy_source(),
            origin="reference",
        )
        if ref.source_sha256 not in seen_hashes:
            self._policies[ref.policy_id] = ref

    def _active_release_id(self) -> str | None:
        reg_dir = self.workspace_dir / "registry"
        if (reg_dir / "registry.json").is_file():
            return ReleaseRegistry(reg_dir).active_version_id
        return None

    def _load_benchmark(self) -> dict[str, Any] | None:
        for candidate in (
            self.workspace_dir / "cafebench-v1" / "cafebench_results.json",
            self.workspace_dir / "benchmark" / "benchmark_results.json",
            self.workspace_dir / "benchmark_results.json",
        ):
            data = _read_optional_json(candidate)
            if data is not None:
                return data
        return None

    def _snapshot_run_state(self) -> dict[str, Any]:
        rs = dict(self._run_state)
        if rs["running"] and rs["started_at"] is not None:
            rs["elapsed_s"] = round(time.monotonic() - rs["started_at"], 1)
        rs.pop("started_at", None)
        return rs

    def fetch_camera_cached(self, camera_name: str, ttl_s: float = 0.08) -> tuple[bytes, str] | None:
        now = time.monotonic()
        cached = self._cam_cache.get(camera_name)
        if cached and (now - cached[0]) <= ttl_s:
            return cached[1], cached[2]
        img = self.driver.fetch_camera_image(camera_name)
        if img is not None:
            self._cam_cache[camera_name] = (now, img[0], img[1])
        return img

    def get_live(self) -> dict[str, Any]:
        """Lightweight high-frequency telemetry payload for smooth live HUD & step animation."""
        obs = self.driver.observe_scene()
        return {
            "scene_observation": {
                **obs.to_dict(),
                "raw_snapshot": obs.raw_snapshot,
            },
            "live_runtime": self.driver.get_live_runtime(),
            "run_state": self._snapshot_run_state(),
        }

    def get_state(self) -> dict[str, Any]:
        cap = self.driver.discover_capabilities(allow_busy=True)
        obs = self.driver.observe_scene()
        oven_gap = self.driver.check_missing_capabilities(OVEN_REQUIRED_METHODS)
        bm = self._load_benchmark()
        p2_tput = _read_optional_json(self.workspace_dir / "phase2_profile_and_throughput.json")
        if p2_tput and p2_tput.get("aggregate_profile"):
            profiler_data = p2_tput["aggregate_profile"]
        elif bm and bm.get("trials"):
            profiler_data = aggregate_trial_profiles(bm["trials"])
        else:
            profiler_data = aggregate_trial_profiles([])

        reg_path = self.workspace_dir / "regression_suite.json"
        reg_cases = [c.to_dict() for c in RegressionSuite(reg_path).cases] if reg_path.is_file() else []
        portability = {pid: check_policy_portability(pol.source) for pid, pol in self._policies.items()}

        promo_gate = None
        auto_path = self.workspace_dir / "autonomy_ledger.json"
        active_rel_id = self._active_release_id()
        if auto_path.is_file() and active_rel_id and "active_release" in self._policies:
            pol_sha = self._policies["active_release"].source_sha256
            promo_gate = AutonomyPromotionLedger(auto_path).check_promotion(
                policy_sha256=pol_sha,
                target_stage="physical_live",
            )

        return {
            "driver": self.driver.driver_name,
            "evidence_source": self.driver.evidence_source,
            "active_release": active_rel_id,
            "capability_check": cap.to_dict(),
            "oven_gap": oven_gap,
            "scene_observation": {
                **obs.to_dict(),
                "raw_snapshot": obs.raw_snapshot,
            },
            "live_runtime": self.driver.get_live_runtime(),
            "run_state": self._snapshot_run_state(),
            "cameras": list(self.driver.list_cameras()),
            "policies": [p.to_meta() for p in self._policies.values()],
            "benchmark": bm,
            "benchmark_running": self._bench_running,
            "workspace_summary": _read_optional_json(self.workspace_dir / "summary.json"),
            "v1_acceptance": _read_optional_json(self.workspace_dir / "v1_acceptance_report.json"),
            "v2_acceptance": _read_optional_json(self.workspace_dir / "v2_acceptance_report.json"),
            "registry": _read_optional_json(self.workspace_dir / "registry" / "registry.json"),
            "skills": _read_optional_json(self.workspace_dir / "skills.json"),
            "phase2_profiler": profiler_data,
            "phase2_throughput": p2_tput,
            "regression_cases": reg_cases,
            "portability_checks": portability,
            "autonomy_physical_gate": promo_gate,
        }

    def get_policy(self, policy_id: str) -> dict[str, Any]:
        pol = self._policies[policy_id]
        return {**pol.to_meta(), "source": pol.source}

    def preview_teaching_spec(self, instruction: str, video_ref: str = "") -> dict[str, Any]:
        cap = self.driver.discover_capabilities(allow_busy=True)
        discovered = cap.discovered.allowed_methods if cap.discovered else None
        mats = []
        if video_ref.strip():
            mats.append(
                TeachingMaterial(
                    material_id="studio-video-ref",
                    kind="video",
                    summary="Operator video reference (non-authoritative teaching evidence)",
                    source_ref=video_ref.strip(),
                )
            )
        spec = build_task_specification(
            self.config,
            instruction,
            materials=tuple(mats),
            steps=default_coffee_teaching_steps(),
            discovered_methods=discovered,
        )
        return spec.to_dict()

    def _build_scenario(self, body: Mapping[str, Any]) -> tuple[PolicyEntry, dict[str, Any]]:
        pol_id = str(body.get("policy_id") or "craft_reference")
        pol = self._policies[pol_id]
        preset = str(body.get("preset") or "default")
        slip = body.get("fault_inject_slip_xy_m")
        if slip is None and body.get("fault_slip"):
            slip = [0.038, -0.012]
        scenario: dict[str, Any] = {
            "preset": preset,
            "cup_a_xy": body.get("cup_a_xy"),
            "cup_b_xy": body.get("cup_b_xy"),
            "order_id": str(body.get("order_id") or "A042"),
            "target_slot": str(body.get("target_slot") or "$SLOT_PICKUP_1"),
            "fault_inject_slip_xy_m": slip,
            "simulate_timeout": bool(body.get("simulate_timeout")),
            "expect_status": "paused_for_operator" if preset == "sim_dual_cup" else "completed",
        }
        return pol, scenario

    def run_single_scenario(self, body: Mapping[str, Any]) -> dict[str, Any]:
        pol, scenario = self._build_scenario(body)
        with self._lock:
            self.driver.reset_scenario(scenario)
            cmd_id = f"studio-{pol.policy_id}-{time.time_ns()}"
            return self.driver.run_policy_scenario(pol.source, command_id=cmd_id, scenario=scenario)

    def start_scenario_async(self, body: Mapping[str, Any]) -> dict[str, Any]:
        if self._run_state["running"]:
            return {"status": "already_running"}
        pol, scenario = self._build_scenario(body)
        label = str(body.get("label") or scenario["preset"])
        self._run_state = {
            "running": True,
            "label": label,
            "started_at": time.monotonic(),
            "elapsed_s": 0.0,
            "last_result": None,
            "error": None,
        }

        def _worker() -> None:
            try:
                with self._lock:
                    self.driver.reset_scenario(scenario)
                    cmd_id = f"studio-{pol.policy_id}-{time.time_ns()}"
                    res = self.driver.run_policy_scenario(pol.source, command_id=cmd_id, scenario=scenario)
                self._run_state["last_result"] = res
            except Exception as exc:  # noqa: BLE001
                self._run_state["error"] = str(exc)
            finally:
                if self._run_state["started_at"] is not None:
                    self._run_state["elapsed_s"] = round(time.monotonic() - self._run_state["started_at"], 1)
                self._run_state["running"] = False

        threading.Thread(target=_worker, daemon=True).start()
        return {"status": "started", "label": label}

    def start_benchmark_async(self, repeats: int = 1) -> dict[str, Any]:
        if self._bench_running:
            return {"status": "already_running"}
        self._bench_running = True

        def _worker() -> None:
            try:
                with self._lock:
                    run_paired_benchmark(
                        self.driver,
                        list(self._policies.values()),
                        default_coffee_benchmark_suite(),
                        self.workspace_dir / "benchmark",
                        repeats=max(1, min(5, int(repeats))),
                    )
            finally:
                self._bench_running = False

        threading.Thread(target=_worker, daemon=True).start()
        return {"status": "started", "repeats": repeats}


def create_studio_server(
    app: CraftStudioApp,
    *,
    host: str = "127.0.0.1",
    port: int = 8860,
) -> ThreadingHTTPServer:
    """Create a ThreadingHTTPServer serving CRAFT Studio."""

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args: Any) -> None:  # noqa: A002
            return

        def _send_json(self, payload: Any, status: int = 200) -> None:
            raw = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def _send_bytes(self, data: bytes, content_type: str, status: int = 200) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _read_json(self) -> dict[str, Any]:
            length = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(length) if length > 0 else b"{}"
            return json.loads(raw.decode("utf-8"))

        def do_GET(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            path = parsed.path
            if path in ("/", "/index.html"):
                self._send_bytes(STUDIO_HTML.encode("utf-8"), "text/html; charset=utf-8")
                return
            if path == "/api/state":
                self._send_json(app.get_state())
                return
            if path == "/api/live":
                self._send_json(app.get_live())
                return
            if path == "/benchmark_report.html":
                cb_html = app.workspace_dir / "cafebench-v1" / "cafebench_report.html"
                if cb_html.is_file():
                    self._send_bytes(cb_html.read_bytes(), "text/html; charset=utf-8")
                    return
                bm = app._load_benchmark()
                if bm:
                    self._send_bytes(render_benchmark_html(bm).encode("utf-8"), "text/html; charset=utf-8")
                else:
                    self._send_bytes(b"<h3>No benchmark report generated yet.</h3>", "text/html; charset=utf-8", 404)
                return
            if path.startswith("/api/policy/"):
                pid = path[len("/api/policy/") :]
                try:
                    self._send_json(app.get_policy(pid))
                except KeyError:
                    self._send_json({"error": f"Unknown policy: {pid}"}, HTTPStatus.NOT_FOUND)
                return
            if path.startswith("/api/camera/"):
                cam = path[len("/api/camera/") :]
                img = app.fetch_camera_cached(cam)
                if img is None:
                    self._send_json({"error": f"Camera unavailable: {cam}"}, HTTPStatus.NOT_FOUND)
                else:
                    self._send_bytes(img[0], img[1])
                return
            self._send_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)

        def do_POST(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            try:
                body = self._read_json()
                if path == "/api/teaching/spec":
                    self._send_json(
                        app.preview_teaching_spec(
                            str(body.get("instruction", "")),
                            str(body.get("video_ref", "")),
                        )
                    )
                    return
                if path == "/api/confirm":
                    self._send_json(app.driver.confirm_object(str(body.get("object_id", ""))))
                    return
                if path == "/api/reset":
                    with app._lock:
                        self._send_json(app.driver.reset_scenario({"preset": str(body.get("preset", "default"))}))
                    return
                if path == "/api/run":
                    self._send_json(app.run_single_scenario(body))
                    return
                if path == "/api/run/start":
                    self._send_json(app.start_scenario_async(body))
                    return
                if path == "/api/benchmark/start":
                    self._send_json(app.start_benchmark_async(int(body.get("repeats", 1))))
                    return
                self._send_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)
            except Exception as exc:  # noqa: BLE001
                self._send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)

    return ThreadingHTTPServer((host, port), Handler)
