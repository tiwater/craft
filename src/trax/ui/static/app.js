let camList = [];
let activeCam = 'front_top';
let camMode = 'split';
let focusedCam = null;
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
let esViewerFrame = null;
let lastMcpTelemetry = null;
let lastReconStepSeq = -1;

async function api(path, opts) {
  const r = await fetch(path, opts);
  const data = await r.json();
  if (!r.ok) throw new Error(data.error || ('HTTP ' + r.status));
  return data;
}

function esc(s) {
  return String(s ?? '').replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}

const PY_KEYWORDS = new Set([
  'def','return','if','elif','else','for','in','while','not','and','or','is',
  'try','except','finally','raise','pass','break','continue','with','as',
  'import','from','class','yield','assert','lambda','None','True','False',
]);
const PY_BUILTINS = new Set([
  'len','float','int','str','dict','list','tuple','set','bool','range',
  'min','max','abs','round','sum','sorted','enumerate','any','all','isinstance','zip','print',
]);
const PY_TOK_RE = /(#[^\n]*)|([frbuFRBU]{0,2}(?:"{3}[\s\S]*?"{3}|'{3}[\s\S]*?'{3}|"(?:\\.|[^"\\\n])*"|'(?:\\.|[^'\\\n])*'))|(\bdef\s+)([A-Za-z_]\w*)|(\b\d+(?:\.\d+)?(?:[eE][+-]?\d+)?\b)|(\b[A-Za-z_]\w*\b)(?=\s*\()|(\b[A-Za-z_]\w*\b)/g;

function highlightPython(src) {
  const s = String(src ?? '');
  let out = '';
  let last = 0;
  PY_TOK_RE.lastIndex = 0;
  let m;
  while ((m = PY_TOK_RE.exec(s)) !== null) {
    if (m.index > last) out += esc(s.slice(last, m.index));
    if (m[1]) {
      out += `<span class="tok-cm">${esc(m[1])}</span>`;
    } else if (m[2]) {
      out += m[2].split('\n').map(l => `<span class="tok-str">${esc(l)}</span>`).join('\n');
    } else if (m[3] && m[4]) {
      out += `<span class="tok-kw">${esc(m[3])}</span><span class="tok-fn">${esc(m[4])}</span>`;
    } else if (m[5]) {
      out += `<span class="tok-num">${esc(m[5])}</span>`;
    } else if (m[6]) {
      const id = m[6];
      const cls = PY_KEYWORDS.has(id) ? 'tok-kw' : (PY_BUILTINS.has(id) ? 'tok-bi' : 'tok-call');
      out += `<span class="${cls}">${esc(id)}</span>`;
    } else if (m[7]) {
      const id = m[7];
      if (PY_KEYWORDS.has(id)) out += `<span class="tok-kw">${esc(id)}</span>`;
      else if (id === 'sdk') out += `<span class="tok-sdk">${esc(id)}</span>`;
      else if (PY_BUILTINS.has(id)) out += `<span class="tok-bi">${esc(id)}</span>`;
      else out += esc(id);
    }
    last = PY_TOK_RE.lastIndex;
  }
  if (last < s.length) out += esc(s.slice(last));
  return out;
}

function syncCodeEditorHighlight() {
  const ed = document.getElementById('code-editor');
  const hl = document.getElementById('code-highlight');
  const gt = document.getElementById('code-gutter');
  if (!ed || !hl || !gt) return;
  const val = ed.value || '';
  hl.innerHTML = highlightPython(val) + '\n';
  const nLines = val.split('\n').length;
  let nums = '';
  for (let i = 1; i <= nLines; i++) nums += i + '\n';
  gt.textContent = nums;
  hl.scrollTop = ed.scrollTop;
  hl.scrollLeft = ed.scrollLeft;
  gt.scrollTop = ed.scrollTop;
}

const JSON_TOK_RE = /("(?:\\.|[^"\\])*")(\s*:)?|(\btrue\b|\bfalse\b)|(\bnull\b)|(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)/g;

function highlightJson(val) {
  const raw = typeof val === 'string' ? val : JSON.stringify(val, null, 2);
  if (raw == null) return '';
  let out = '';
  let last = 0;
  JSON_TOK_RE.lastIndex = 0;
  let m;
  while ((m = JSON_TOK_RE.exec(raw)) !== null) {
    if (m.index > last) out += esc(raw.slice(last, m.index));
    if (m[1]) {
      if (m[2]) out += `<span class="tok-jkey">${esc(m[1])}</span>${esc(m[2])}`;
      else out += `<span class="tok-str">${esc(m[1])}</span>`;
    } else if (m[3]) {
      out += `<span class="tok-jbool">${esc(m[3])}</span>`;
    } else if (m[4]) {
      out += `<span class="tok-jnull">${esc(m[4])}</span>`;
    } else if (m[5]) {
      out += `<span class="tok-num">${esc(m[5])}</span>`;
    }
    last = JSON_TOK_RE.lastIndex;
  }
  if (last < raw.length) out += esc(raw.slice(last));
  return out;
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
  if (name === 'mcp' && (!mcpState || !(mcpState.tools || []).length)) {
    inspectMcp();
  }
  if (name === 'kb') {
    loadKnowledgeBase();
  }
  if (name === 'gov') {
    loadGovernanceLedger();
  }
}

let kbState = null;
let selectedKbSkill = null;
let selectedKbCommit = '';

async function loadKnowledgeBase() {
  try {
    kbState = await api('api/kb/state');
    document.getElementById('kb-branch-pill').textContent = `branch: ${kbState.branch || 'main'}`;
    document.getElementById('kb-branch-pill').className = 'badge ' + (kbState.branch === 'main' ? 'ok' : 'warn');
    document.getElementById('kb-tags-pill').textContent = `${(kbState.tags || []).length} verified tags`;

    const ws = kbState.workspace_stats || {};
    document.getElementById('kb-zones-bar').innerHTML = `
      <span><strong>knowledge/</strong> Git (${(kbState.git_log||[]).length} commits)</span>
      <span><strong>runtime/</strong> SQLite (${(ws.sqlite_ledgers||[]).length} WAL DBs)</span>
      <span><strong>traces/</strong> JSONL (${ws.llm_calls_logged||0} LLM · ${ws.mcp_wire_logged||0} MCP)</span>
      <span><strong>recordings/</strong> (${ws.recordings_count||0} episodes)</span>
    `;
    document.getElementById('kb-facts-pre').textContent = kbState.facts_md || '(empty)';
    document.getElementById('kb-forbidden-pre').textContent = kbState.forbidden_md || '(empty)';

    const skills = kbState.skills || [];
    document.getElementById('kb-skills-cnt').textContent = `${skills.length} skills`;
    document.getElementById('kb-skills-list').innerHTML = skills.map((s, idx) => {
      const cls = s.status === 'active' ? 'ok' : (s.status === 'stale_contract' ? 'err' : 'warn');
      const act = selectedKbSkill && selectedKbSkill.skill_md_path === s.skill_md_path ? 'active' : '';
      return `<button class="mcp-item ${act}" onclick="selectKbSkill(${idx})">
        <div class="mcp-item-top">
          <span class="mcp-item-name">${esc(s.embodiment_id)} / ${esc(s.skill_name)}</span>
          <span class="badge ${cls}">${esc(s.status.toUpperCase())}</span>
        </div>
        <div class="mcp-item-desc">evidence=${esc(s.evidence_source)} · contract=${esc(s.mcp_tools_sha256)} · ${esc(s.verify_detail||'')}</div>
      </button>`;
    }).join('') || '<div style="font-family:var(--mono);font-size:11px;color:var(--dim)">No promoted skills yet. Run a Closed Loop task until verify_task passes to promote onto main.</div>';

    if (!selectedKbSkill && skills.length) selectKbSkill(0);

    const commits = kbState.git_log || [];
    document.getElementById('kb-git-log').innerHTML = commits.map(c => {
      const act = c.sha === selectedKbCommit ? 'active' : '';
      const isMerge = c.subject.startsWith('merge(');
      return `<button class="mcp-item ${act}" onclick="selectKbCommit('${esc(c.sha)}','${esc(c.short_sha)}')">
        <div class="mcp-item-top">
          <span class="mcp-item-name"><code>${esc(c.short_sha)}</code> ${esc(c.subject.slice(0, 52))}</span>
          ${c.refs ? `<span class="badge ${isMerge ? 'ok' : ''}">${esc(c.refs.slice(0, 32))}</span>` : ''}
        </div>
        <div class="mcp-item-desc">${esc(c.date)}</div>
      </button>`;
    }).join('') || '<div style="font-family:var(--mono);font-size:11px;color:var(--dim)">No commits yet.</div>';

    if (!selectedKbCommit && commits.length) selectKbCommit(commits[0].sha, commits[0].short_sha);
  } catch (e) {
    document.getElementById('kb-facts-pre').textContent = `KB load error: ${e.message}`;
  }
}

function selectKbSkill(idx) {
  if (!kbState || !kbState.skills || !kbState.skills[idx]) return;
  selectedKbSkill = kbState.skills[idx];
  const runBtn = document.getElementById('btn-kb-run-skill');
  if (runBtn) runBtn.disabled = false;
  openKbSkillFile('py');
}

async function runSkillDirect(skillPyPath, taskId) {
  if (!skillPyPath) return;
  try {
    cupTrail = [];
    await api('api/kb/run_skill', {
      method: 'POST', headers: {'Content-Type':'application/json'},
      body: JSON.stringify({ skill_py_path: skillPyPath, task_id: taskId || '' }),
    });
    switchTab('lab');
    await pollLiveOnce();
  } catch (e) {
    alert('Run skill failed: ' + e.message);
  }
}

async function runSelectedKbSkill() {
  if (!selectedKbSkill) return;
  await runSkillDirect(selectedKbSkill.skill_py_path, selectedKbSkill.skill_name);
}

async function openKbSkillFile(kind) {
  if (!selectedKbSkill) return;
  const path = kind === 'md' ? selectedKbSkill.skill_md_path : selectedKbSkill.skill_py_path;
  document.getElementById('kb-file-title').textContent = path;
  const data = await api('api/kb/file?path=' + encodeURIComponent(path));
  const pre = document.getElementById('kb-file-pre');
  if (kind === 'py') pre.innerHTML = renderNumberedCode(data.content || '');
  else pre.textContent = data.content || '';
}

async function selectKbCommit(sha, shortSha) {
  selectedKbCommit = sha;
  document.getElementById('kb-diff-sha').textContent = shortSha || sha.slice(0, 7);
  document.getElementById('btn-kb-rollback').disabled = !sha;
  const data = await api('api/kb/diff?sha=' + encodeURIComponent(sha));
  document.getElementById('kb-diff-view').innerHTML = renderDiffHtml(data.diff || '');
}

async function rollbackSelectedCommit() {
  if (!selectedKbCommit) return;
  try {
    const res = await api('api/kb/rollback', {
      method: 'POST', headers: {'Content-Type':'application/json'},
      body: JSON.stringify({ sha: selectedKbCommit }),
    });
    if (res.state) renderEvolution(res.state);
    await loadKnowledgeBase();
    switchTab('lab');
  } catch (e) {
    alert('Rollback failed: ' + e.message);
  }
}

function switchCodeTab(name) {
  codeSubTab = name;
  document.querySelectorAll('#tab-lab .subtab').forEach(b => b.classList.toggle('active', b.dataset.sub === name));
  for (const id of ['code', 'llm', 'exec', 'diff', 'subr']) {
    const el = document.getElementById('pane-' + id);
    if (el) el.style.display = name === id ? 'flex' : 'none';
  }
}

let mcpState = null;
let selectedMcpTool = '';

function switchMcpSub(name) {
  document.querySelectorAll('#tab-mcp .subtab').forEach(b => b.classList.toggle('active', b.dataset.mcpsub === name));
  for (const id of ['tool', 'res', 'prompt']) {
    const el = document.getElementById('mcp-pane-' + id);
    if (el) el.style.display = name === id ? 'flex' : 'none';
  }
}

function onMcpTransportChange() {
  const tr = document.getElementById('mcp-transport').value;
  const inp = document.getElementById('mcp-target-input');
  if (tr === 'stdio') {
    inp.placeholder = 'e.g. python3 -m my_robot_mcp_server';
  } else if (tr === 'http') {
    inp.placeholder = 'e.g. http://192.168.1.50:8890/mcp';
  } else {
    inp.placeholder = 'Auto-detects active host /mcp (or in-process MCP bridge)';
  }
}

function defaultArgsFromSchema(schema) {
  const out = {};
  const props = (schema && schema.properties) || {};
  for (const [k, def] of Object.entries(props)) {
    if (def.default !== undefined) out[k] = def.default;
    else if (def.type === 'string') out[k] = '';
    else if (def.type === 'boolean') out[k] = false;
    else if (def.type === 'number' || def.type === 'integer') out[k] = 0;
    else if (def.type === 'array') out[k] = [];
    else if (def.type === 'object') out[k] = {};
  }
  return out;
}

async function inspectMcp() {
  const tr = document.getElementById('mcp-transport').value;
  const raw = document.getElementById('mcp-target-input').value.trim();
  const badge = document.getElementById('mcp-srv-badge');
  badge.textContent = 'DISCOVERING...';
  badge.className = 'badge warn';
  try {
    const payload = { transport: tr };
    if (tr === 'http' && raw) payload.url = raw;
    if (tr === 'stdio' && raw) payload.command = raw;
    mcpState = await api('api/mcp/inspect', {
      method: 'POST', headers: {'Content-Type':'application/json'},
      body: JSON.stringify(payload),
    });
    renderMcpState();
  } catch (e) {
    badge.textContent = 'ERR';
    badge.className = 'badge err';
    document.getElementById('mcp-tool-meta').textContent = `MCP discovery failed: ${e.message}`;
  }
}

function renderMcpState() {
  if (!mcpState) return;
  const info = mcpState.server_info || {};
  const tools = mcpState.tools || [];
  const res = mcpState.resources || [];
  const b = document.getElementById('mcp-srv-badge');
  b.textContent = info.name ? `${info.name} (${info.protocolVersion || 'mcp'})` : 'CONNECTED';
  b.className = 'badge ok';
  document.getElementById('mcp-counts-badge').textContent = `${tools.length} tools · ${res.length} res`;
  document.getElementById('mcp-prompt-preview').textContent = mcpState.prompt_contract_preview || '(no tools discovered)';
  if (!selectedMcpTool && tools.length) selectedMcpTool = tools[0].name;
  renderMcpCatalog();
  renderMcpSelectedTool();
  renderMcpWire();
}

function renderMcpCatalog() {
  if (!mcpState) return;
  const q = (document.getElementById('mcp-filter').value || '').toLowerCase();
  const tools = (mcpState.tools || []).filter(t => !q || (t.name || '').toLowerCase().includes(q) || (t.description || '').toLowerCase().includes(q));
  document.getElementById('mcp-tools-list').innerHTML = tools.map(t => {
    const ann = t.annotations || {};
    let tag = '<span class="badge">IDEMPOTENT</span>';
    if (ann.destructiveHint || ann.idempotentHint === false) {
      tag = '<span class="badge warn">NON-IDEMPOTENT · JOURNAL</span>';
    } else if (ann.readOnlyHint) {
      tag = '<span class="badge ok">READ-ONLY</span>';
    }
    const act = t.name === selectedMcpTool ? 'active' : '';
    return `<button class="mcp-item ${act}" onclick="selectMcpTool('${esc(t.name)}')">
      <div class="mcp-item-top"><span class="mcp-item-name">${esc(t.name)}</span>${tag}</div>
      <div class="mcp-item-desc">${esc((t.description || '').slice(0, 110))}</div>
    </button>`;
  }).join('') || '<div style="font-family:var(--mono);font-size:11px;color:var(--dim)">No tools match filter.</div>';

  const resources = mcpState.resources || [];
  document.getElementById('mcp-res-list').innerHTML = resources.map(r => `
    <button class="mcp-item" onclick="readMcpResource('${esc(r.uri)}')">
      <div class="mcp-item-top"><span class="mcp-item-name">${esc(r.uri)}</span><span class="badge">${esc(r.mimeType||'')}</span></div>
      <div class="mcp-item-desc">${esc(r.name || r.description || '')}</div>
    </button>
  `).join('') || '<div style="font-family:var(--mono);font-size:11px;color:var(--dim)">No resources advertised.</div>';
}

function selectMcpTool(name) {
  selectedMcpTool = name;
  switchMcpSub('tool');
  renderMcpCatalog();
  renderMcpSelectedTool();
}

function renderMcpSelectedTool() {
  if (!mcpState) return;
  const t = (mcpState.tools || []).find(x => x.name === selectedMcpTool);
  if (!t) return;
  const ann = t.annotations || {};
  document.getElementById('mcp-sel-badge').textContent = t.name;
  document.getElementById('mcp-tool-meta').innerHTML = `
    <span><strong>${esc(t.name)}</strong> &middot; ${esc(t.description || '')}</span>
    <span>endpoint=<code>${esc(mcpState.endpoint_label || '-')}</code></span>
  `;
  document.getElementById('mcp-schema-pre').innerHTML = highlightJson({
    name: t.name,
    annotations: ann,
    inputSchema: t.inputSchema || {},
  });
  document.getElementById('mcp-args-input').value = JSON.stringify(defaultArgsFromSchema(t.inputSchema), null, 2);
}

async function invokeSelectedMcpTool() {
  if (!selectedMcpTool) return;
  let args = {};
  try {
    args = JSON.parse(document.getElementById('mcp-args-input').value || '{}');
  } catch (e) {
    document.getElementById('mcp-call-result').textContent = `Invalid JSON arguments: ${e.message}`;
    return;
  }
  document.getElementById('mcp-call-meta').textContent = 'invoking...';
  try {
    const res = await api('api/mcp/call', {
      method: 'POST', headers: {'Content-Type':'application/json'},
      body: JSON.stringify({
        tool_name: selectedMcpTool,
        arguments: args,
        use_journal: document.getElementById('mcp-journal-chk').checked,
      }),
    });
    mcpState = res.state;
    const jTxt = res.journal_id ? ` · SQLite journal=${res.journal_id}` : '';
    document.getElementById('mcp-call-meta').textContent = `${res.latency_ms}ms${jTxt}`;
    document.getElementById('mcp-call-result').innerHTML = highlightJson(res.response);
    renderMcpWire();
  } catch (e) {
    document.getElementById('mcp-call-meta').textContent = 'error';
    document.getElementById('mcp-call-result').textContent = `tools/call failed: ${e.message}`;
  }
}

async function readMcpResource(uri) {
  switchMcpSub('res');
  document.getElementById('mcp-res-meta').textContent = `Reading ${uri} ...`;
  try {
    const res = await api('api/mcp/read_resource', {
      method: 'POST', headers: {'Content-Type':'application/json'},
      body: JSON.stringify({ uri }),
    });
    mcpState = res.state;
    document.getElementById('mcp-res-meta').textContent = `${uri} · ${res.latency_ms}ms`;
    const contents = ((res.response || {}).result || {}).contents || [];
    const box = document.getElementById('mcp-res-body');
    if (contents.length && contents[0].blob && String(contents[0].mimeType || '').startsWith('image/')) {
      box.innerHTML = `<img class="mcp-res-img" src="data:${esc(contents[0].mimeType)};base64,${contents[0].blob}"/>`;
    } else if (contents.length && contents[0].text != null) {
      const isJson = String(contents[0].mimeType || '').includes('json') || String(contents[0].text).trim().startsWith('{');
      box.innerHTML = `<pre class="io-pre" style="padding:0">${isJson ? highlightJson(contents[0].text) : esc(contents[0].text)}</pre>`;
    } else {
      box.innerHTML = `<pre class="io-pre" style="padding:0">${highlightJson(res.response)}</pre>`;
    }
    renderMcpWire();
  } catch (e) {
    document.getElementById('mcp-res-meta').textContent = `Read failed: ${e.message}`;
  }
}

function renderMcpWire() {
  if (!mcpState) return;
  const logs = [...(mcpState.wire_log || [])].reverse();
  document.getElementById('mcp-wire-count').textContent = `${logs.length} msgs`;
  document.getElementById('mcp-wire-list').innerHTML = logs.map(w => `
    <div class="mcp-wire-card">
      <div class="mcp-item-top">
        <span><strong style="color:var(--cyan)">${esc(w.method)}</strong> <span style="color:var(--dim)">[${esc(w.t)}]</span></span>
        <span style="display:flex;gap:4px">
          ${w.journal_id ? `<span class="badge warn">JOURNALED</span>` : ''}
          <span class="badge ${w.ok ? 'ok' : 'err'}">${w.ok ? 'OK' : 'ERR'} · ${esc(w.latency_ms)}ms</span>
        </span>
      </div>
      <pre class="mcp-wire-pre"><span style="color:var(--dim)">REQ:</span> ${highlightJson(JSON.stringify(w.request))}\n<span style="color:var(--dim)">RES:</span> ${highlightJson(w.response)}</pre>
    </div>
  `).join('') || '<div style="font-family:var(--mono);font-size:11px;color:var(--dim)">No JSON-RPC messages yet.</div>';
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
    const idx = evoState.executed_runs.findIndex(r => String(r.stage || '').startsWith('01_explore'));
    if (idx >= 0) selectedExecRunIdx = idx;
  }
  switchCodeTab('exec');
  if (evoState) renderExecutedCodeInspector(evoState);
}

function applyTheme(theme) {
  const mode = theme === 'dark' ? 'dark' : 'light';
  document.documentElement.dataset.theme = mode;
  try { localStorage.setItem('trax_theme', mode); } catch (_) {}
  const btn = document.getElementById('theme-toggle');
  if (btn) btn.textContent = mode === 'dark' ? 'Light' : 'Dark';
}

function toggleTheme() {
  const cur = document.documentElement.dataset.theme || 'light';
  applyTheme(cur === 'dark' ? 'light' : 'dark');
}

(function initTheme() {
  let saved = 'light';
  try { saved = localStorage.getItem('trax_theme') || 'light'; } catch (_) {}
  applyTheme(saved);
})();

document.addEventListener('DOMContentLoaded', () => {
  applyTheme(document.documentElement.dataset.theme || 'light');
  const ed = document.getElementById('code-editor');
  if (ed) {
    ed.addEventListener('input', () => { editorDirty = true; syncCodeEditorHighlight(); });
    ed.addEventListener('scroll', () => {
      const hl = document.getElementById('code-highlight');
      const gt = document.getElementById('code-gutter');
      if (hl) { hl.scrollTop = ed.scrollTop; hl.scrollLeft = ed.scrollLeft; }
      if (gt) { gt.scrollTop = ed.scrollTop; }
    });
    ed.addEventListener('keydown', (e) => {
      if (e.key === 'Tab') {
        e.preventDefault();
        const st = ed.selectionStart, en = ed.selectionEnd;
        ed.value = ed.value.slice(0, st) + '    ' + ed.value.slice(en);
        ed.selectionStart = ed.selectionEnd = st + 4;
        editorDirty = true;
        syncCodeEditorHighlight();
      }
    });
  }
});

function setCamMode(mode) {
  camMode = mode;
  if (mode !== 'cams') focusedCam = null;
  renderCameraShells();
}
function selectCam(name) {
  activeCam = name;
  focusedCam = (focusedCam === name) ? null : name;
  renderCameraShells();
}
function clearTrail() {
  cupTrail = [];
  if (esViewerFrame && esViewerFrame.contentWindow) {
    esViewerFrame.contentWindow.postMessage({ type: 'evidencescene:clearTrail' }, '*');
  }
  if (lastLiveScene) renderSpatialRadar(lastLiveScene);
}

window.addEventListener('message', (ev) => {
  const d = ev.data;
  if (!d || typeof d !== 'object') return;
  if (d.type === 'evidencescene:cursor' && d.cursor) {
    const curBadge = document.getElementById('pk-cursor');
    if (curBadge) {
      const ok = d.snapshot && d.snapshot.observability && d.snapshot.observability.status === 'observable';
      curBadge.textContent = `ep${d.cursor.epoch}:r${d.cursor.revision}`;
      curBadge.className = 'badge ' + (ok ? 'ok' : 'warn');
    }
  }
});

function refreshSceneReconstruction(force) {
  if (!esViewerFrame || !esViewerFrame.contentWindow) return;
  const seq = (lastMcpTelemetry && lastMcpTelemetry.robot && lastMcpTelemetry.robot.step_seq != null)
    ? Number(lastMcpTelemetry.robot.step_seq) : 0;
  if (!force && seq === lastReconStepSeq) return;
  lastReconStepSeq = seq;
  esViewerFrame.contentWindow.postMessage({ type: 'evidencescene:sync', force: !!force }, '*');
}

function mountTwinViewer() {
  const twinStage = document.getElementById('twin-stage');
  if (!twinStage) return;
  if (esViewerFrame && document.getElementById('es-viewer-iframe')) return;
  twinStage.innerHTML = `
    <div class="urdf-shell">
      <iframe id="es-viewer-iframe" class="es-viewer-frame" src="evidencescene/viewer?embed=1" title="EvidenceScene v3 Human Visual Interface"></iframe>
    </div>`;
  esViewerFrame = document.getElementById('es-viewer-iframe');
  lastReconStepSeq = -1;
}

function renderCameraShells() {
  const twinStage = document.getElementById('twin-stage');
  const camStage = document.getElementById('cam-stage');
  if (!twinStage || !camStage) return;

  const setBtn = (id, on) => {
    const el = document.getElementById(id);
    if (el) el.classList.toggle('active', !!on);
  };
  setBtn('btn-mode-split', camMode === 'split');
  setBtn('btn-mode-twin', camMode === 'twin');
  setBtn('btn-mode-cams', camMode === 'cams');

  const showTwin = camMode === 'split' || camMode === 'twin';
  const showCams = camMode === 'split' || camMode === 'cams';

  // 1. Always keep 3D Digital Twin alive in Split and Twin modes
  twinStage.style.display = showTwin ? 'flex' : 'none';
  if (showTwin) {
    mountTwinViewer();
  } else if (esViewerFrame) {
    esViewerFrame = null;
    twinStage.innerHTML = '';
  }

  // 2. Always show Live Cameras simultaneously underneath in Split (or expanded in Cams)
  camStage.style.display = showCams ? 'flex' : 'none';
  camStage.classList.toggle('expanded', camMode === 'cams');
  if (!showCams) {
    camStage.innerHTML = '';
    return;
  }
  if (!camList.length) {
    camStage.innerHTML = '<div class="video-shell" style="color:var(--muted);font-family:var(--mono);font-size:11px">Waiting for camera stream...</div>';
    return;
  }
  if (!camList.includes(activeCam)) activeCam = camList[0];
  const visibleCams = (focusedCam && camList.includes(focusedCam)) ? [focusedCam] : camList;
  const singleCls = visibleCams.length === 1 ? 'single-focus' : '';
  camStage.innerHTML = `
    <div class="cam-dual-row ${singleCls}">
      ${visibleCams.map((c, idx) => `
        <div class="cam-tile" onclick="selectCam('${c}')" title="Click to ${focusedCam === c ? 'restore dual cameras' : 'focus ' + c}">
          <canvas data-cam-canvas="${c}" width="640" height="360"></canvas>
          <div class="hud-tl">
            <span class="hud-pill">${c}</span>
            ${idx === 0 ? '<span class="hud-pill" id="hud-fps">-- FPS</span>' : ''}
          </div>
          ${idx === 0 ? '<div class="hud-tr"><span class="hud-pill" id="hud-l2">gripper: --</span></div>' : ''}
          ${idx === 0 ? '<div class="hud-bot"><div class="hud-bot-row"><span id="hud-action">idle</span><span id="hud-brew-txt">step 0</span></div><div class="bar-bg"><div id="hud-brew-bar" class="bar-fg"></div></div></div>' : ''}
        </div>
      `).join('')}
    </div>`;
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
  while (true) {
    const t0 = performance.now();
    const onLab = document.getElementById('tab-lab').classList.contains('active');
    if (onLab && camList.length && (camMode === 'split' || camMode === 'cams')) {
      const canvases = [...document.querySelectorAll('#cam-stage canvas[data-cam-canvas]')];
      if (canvases.length) {
        const results = await Promise.all(canvases.map(cv => drawCamToCanvas(cv.dataset.camCanvas, cv)));
        if (results[0]) {
          const now = performance.now();
          frameTimestamps.push(now);
          frameTimestamps = frameTimestamps.filter(t => now - t <= 2000);
          const el = document.getElementById('hud-fps');
          if (el) el.textContent = (frameTimestamps.length / 2).toFixed(1) + ' FPS';
        }
      }
    }
    const dt = performance.now() - t0;
    const busy = evoState && evoState.busy;
    await new Promise(r => setTimeout(r, Math.max(25, (busy ? 95 : 150) - dt)));
  }
}

function worldToRadar(x, y) {
  const px = 36 + ((x - 0.10) / 0.42) * 290;
  const py = 110 - (y / 0.30) * 92;
  return [Math.max(14, Math.min(346, px)), Math.max(14, Math.min(206, py))];
}

function renderSpatialRadar(obs, mcpTel) {
  const svg = document.getElementById('radar-svg');
  if (!svg) return;
  const scene = (mcpTel && mcpTel.scene) || null;
  const robot = (mcpTel && mcpTel.robot) || null;

  // Dynamic MCP Tabletop Map (embodiment-agnostic: renders whatever zones, objects & EE exist)
  if (scene && robot) {
    const [bx, by] = worldToRadar((robot.base_pos_m || [0.13, 0])[0], (robot.base_pos_m || [0.13, 0])[1]);
    const ee = robot.ee_pos_m || [0.32, 0.0, 1.15];
    const [ex, ey] = worldToRadar(ee[0], ee[1]);
    const prev = cupTrail[cupTrail.length - 1];
    if (!prev || Math.hypot(ex - prev[0], ey - prev[1]) > 2.0) {
      cupTrail.push([ex, ey]);
      if (cupTrail.length > 80) cupTrail.shift();
    }
    const trail = cupTrail.length > 1
      ? `<polyline fill="none" stroke="var(--cyan)" stroke-width="2" stroke-dasharray="4 3" opacity="0.85" points="${cupTrail.map(p => p[0].toFixed(1)+','+p[1].toFixed(1)).join(' ')}"/>`
      : '';

    const zones = scene.zones || {};
    const zoneSvg = Object.entries(zones).map(([zName, zPos]) => {
      const [zx, zy] = worldToRadar(zPos[0], zPos[1]);
      const col = zName.includes('station') ? 'var(--amber)' : 'var(--green)';
      const bg = zName.includes('station') ? 'var(--amber-bg)' : 'var(--green-bg)';
      return `<g>
        <circle cx="${zx.toFixed(1)}" cy="${zy.toFixed(1)}" r="15" fill="${bg}" stroke="${col}" stroke-width="1.5" stroke-dasharray="3 2"/>
        <text x="${zx.toFixed(1)}" y="${(zy + 25).toFixed(1)}" fill="${col}" font-size="8" font-weight="600" font-family="monospace" text-anchor="middle">${esc(zName)}</text>
      </g>`;
    }).join('');

    const objSvg = (scene.objects || []).map(o => {
      const pos = o.world_m || [0.33, 0, 1.03];
      const [ox, oy] = worldToRadar(pos[0], pos[1]);
      const col = o.id.includes('red') ? '#e11d48' : '#78716c';
      const ring = o.grasped ? '#d97706' : 'var(--surface)';
      return `<g>
        <circle cx="${ox.toFixed(1)}" cy="${oy.toFixed(1)}" r="8" fill="${col}" stroke="${ring}" stroke-width="2"/>
        <text x="${ox.toFixed(1)}" y="${(oy - 12).toFixed(1)}" fill="var(--radar-txt)" font-size="8" font-family="monospace" text-anchor="middle">${esc(o.id)} (${(pos[0]*100).toFixed(0)},${(pos[1]*100).toFixed(0)}cm)</text>
      </g>`;
    }).join('');

    svg.innerHTML = `
      <rect x="4" y="4" width="352" height="212" rx="6" fill="var(--radar-bg)" stroke="var(--border)"/>
      <line x1="20" y1="110" x2="340" y2="110" stroke="var(--radar-grid)" stroke-dasharray="2 4"/>
      <line x1="${bx.toFixed(1)}" y1="14" x2="${bx.toFixed(1)}" y2="206" stroke="var(--radar-grid)" stroke-dasharray="2 4"/>
      <rect x="${(bx-9).toFixed(1)}" y="${(by-9).toFixed(1)}" width="18" height="18" rx="3" fill="var(--surface2)" stroke="var(--muted)" stroke-width="1.5"/>
      <text x="${bx.toFixed(1)}" y="${(by+21).toFixed(1)}" fill="var(--muted)" font-size="7.5" font-family="monospace" text-anchor="middle">BASE</text>
      ${zoneSvg}${trail}${objSvg}
      <line x1="${bx.toFixed(1)}" y1="${by.toFixed(1)}" x2="${ex.toFixed(1)}" y2="${ey.toFixed(1)}" stroke="var(--cyan)" stroke-width="1.4" opacity="0.45"/>
      <circle cx="${ex.toFixed(1)}" cy="${ey.toFixed(1)}" r="5" fill="var(--cyan)" stroke="var(--surface)" stroke-width="1.5"/>
      <text x="${ex.toFixed(1)}" y="${(ey+14).toFixed(1)}" fill="var(--cyan)" font-size="8" font-weight="700" font-family="monospace" text-anchor="middle">EE z=${ee[2].toFixed(2)}m</text>
    `;
    return;
  }

  // Generic fallback when connected to a legacy driver without trax://scene/state
  const raw = (obs && obs.raw_snapshot) || {};
  const objs = raw.objects || [];
  const objSvg = objs.map(o => {
    const pos = o.center_world_m || o.world || [0.33, 0];
    const [ox, oy] = worldToRadar(pos[0], pos[1]);
    return `<g>
      <circle cx="${ox.toFixed(1)}" cy="${oy.toFixed(1)}" r="7" fill="var(--cyan)" stroke="var(--surface)" stroke-width="1.5"/>
      <text x="${ox.toFixed(1)}" y="${(oy+16).toFixed(1)}" fill="var(--radar-txt)" font-size="8" font-family="monospace" text-anchor="middle">${esc(o.object_id || o.id)}</text>
    </g>`;
  }).join('');
  svg.innerHTML = `
    <rect x="4" y="4" width="352" height="212" rx="6" fill="var(--radar-bg)" stroke="var(--border)"/>
    <line x1="20" y1="110" x2="340" y2="110" stroke="var(--radar-grid)" stroke-dasharray="2 4"/>
    ${objSvg}
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
  const hasExplore = (evo.exploration_findings || []).length > 0;
  const iters = evo.iterations || [];
  const cur = (evo.active_iter_index >= 0 && evo.active_iter_index < iters.length) ? iters[evo.active_iter_index] : null;
  const hasExec = !!(cur && cur.execution);
  const passed = !!(hasExec && cur.execution.task_passed);

  const setP = (id, cls) => { document.getElementById(id).className = 'pipe-step ' + cls; };
  setP('ps-explore', phase === 'exploring' ? 'active' : (hasExplore ? 'done' : ''));
  setP('ps-synth', phase === 'synthesizing' ? 'active' : (cur ? 'done' : ''));
  setP('ps-exec', phase === 'executing' ? 'active' : (passed ? 'done' : (hasExec ? 'warn' : '')));
  setP('ps-iter', phase === 'iterating' ? 'active' : (passed ? 'done' : (hasExec && !passed ? 'warn' : '')));
}

function onTaskProfileChange() {
  const sel = document.getElementById('scenario-select');
  const ti = document.getElementById('task-instruction');
  if (evoState && evoState.task_instructions && sel && ti) {
    const inst = evoState.task_instructions[sel.value];
    if (inst) ti.value = inst;
  }
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
  const profiles = evo.task_profiles || evo.scenario_profiles;
  if (sel && profiles && sel.options.length === 0) {
    sel.innerHTML = Object.entries(profiles)
      .map(([k, lbl]) => `<option value="${esc(k)}">${esc(lbl)}</option>`).join('');
    sel.value = evo.task_id || evo.scenario_key || Object.keys(profiles)[0];
  }

  // MCP Exploration telemetry panel
  const findings = evo.exploration_findings || [];
  const srvInfo = evo.server_info || {};
  const pb = document.getElementById('probe-badge');
  if (findings.length > 0) {
    pb.textContent = `EXPLORED (${findings.length}R)`;
    pb.className = 'badge ok';
  } else if (evo.phase === 'exploring') {
    pb.textContent = 'EXPLORING...';
    pb.className = 'badge warn';
  } else {
    pb.textContent = 'UNEXPLORED';
    pb.className = 'badge';
  }
  document.getElementById('pk-auth').textContent = srvInfo.name || evo.mcp_endpoint || '-';
  document.getElementById('pk-sdk').textContent = evo.discovered_tools_count ? `${evo.discovered_tools_count} tools` : '-';
  document.getElementById('pk-objs').textContent = `${findings.length} / ${evo.explore_rounds || 2} rounds`;

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
  }).join('') || '<span style="font-family:var(--mono);font-size:11px;color:var(--dim)">No compiled policy yet. Click 1. Agent Explore (MCP+LLM) or Run Closed Loop (1->4).</span>';

  const cur = (evo.active_iter_index >= 0 && evo.active_iter_index < iters.length) ? iters[evo.active_iter_index] : null;
  if (cur) {
    const ed = document.getElementById('code-editor');
    if (ed && (!editorDirty || cur.source_sha256 !== lastLoadedIterSha)) {
      ed.value = cur.source || '';
      lastLoadedIterSha = cur.source_sha256 || '';
      editorDirty = false;
      syncCodeEditorHighlight();
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
      ast_compliance: cur.ast_check,
      subroutines: cur.subroutines,
      discovered_mcp_tools: evo.discovered_tools || [],
      exploration_findings: findings,
    };
    document.getElementById('subr-view').innerHTML = highlightJson(subPayload);

    // Right pane: Independent Verifier & Attribution & Governed MCP Journal
    const ex = cur.execution;
    const attr = cur.attribution || {};
    const vp = document.getElementById('verdict-pill');
    if (ex) {
      vp.textContent = ex.task_passed ? 'PASS' : 'FAIL';
      vp.className = 'badge ' + (ex.task_passed ? 'ok' : 'err');
      document.getElementById('m-layer').textContent = attr.layer || 'none';
      document.getElementById('m-code').textContent = attr.code || ex.host_runtime_status || '-';
      const vm = (ex.verify_result && ex.verify_result.metrics) || {};
      const vmParts = Object.entries(vm).slice(0, 2).map(([k, v]) => `${k}=${v}`);
      document.getElementById('m-oracle').textContent = vmParts.length ? vmParts.join(' · ') : (ex.verify_result ? ex.verify_result.detail : '-');
      const niTotal = Object.values(ex.non_idempotent_counts || {}).reduce((a, b) => a + Number(b || 0), 0);
      document.getElementById('m-brew').textContent = `${(ex.methods||[]).length} calls · ni=${niTotal} · dup=${ex.duplicate_non_idempotent ? 1 : 0}`;
      const prof = ex.profile || null;
      const ps = document.getElementById('prof-summary');
      if (ps) {
        if (prof) {
          const hint = (prof.optimization_hints && prof.optimization_hints[0]) ? ` · ${prof.optimization_hints[0]}` : '';
          ps.textContent = `wall=${ex.wall_s}s · mcp=${prof.total_mcp_ms}ms · p95=${prof.p95_call_ms}ms · bottleneck=${prof.bottleneck_tool || '-'}${hint}`;
        } else {
          ps.textContent = `wall=${ex.wall_s}s · ${(ex.methods||[]).length} MCP calls`;
        }
      }
    } else {
      vp.textContent = 'UNEXECUTED';
      vp.className = 'badge';
      document.getElementById('m-layer').textContent = '-';
      document.getElementById('m-code').textContent = '-';
      document.getElementById('m-oracle').textContent = '-';
      document.getElementById('m-brew').textContent = '-';
      const ps = document.getElementById('prof-summary');
      if (ps) ps.textContent = 'No execution profile yet.';
    }

    const jrows = cur.journal_entries || [];
    document.getElementById('journal-count').textContent = `${jrows.length} ops`;
    document.getElementById('journal-tbody').innerHTML = jrows.length
      ? jrows.map((r, i) => {
          const st = String(r.state || '');
          const cls = st.includes('UNKNOWN') || st.includes('LIMIT_REFUSAL')
            ? 'row-warn'
            : (st.includes('RECONCILED') || st.includes('COMMITTED') ? 'row-ok' : (r.non_idempotent ? 'row-hi' : ''));
          return `<tr class="${cls}"><td>${i+1}</td><td>${esc(r.action)}</td><td>${esc(st)}</td></tr>`;
        }).join('')
      : '<tr><td colspan="3" style="color:var(--dim)">Click 3. Governed Execute to run via MCP + SQLite Journal.</td></tr>';
  } else {
    const ed = document.getElementById('code-editor');
    if (ed && !editorDirty) {
      ed.value = '';
      lastLoadedIterSha = '';
      syncCodeEditorHighlight();
    }
    const ab = document.getElementById('ast-badge');
    ab.textContent = 'IDLE';
    ab.className = 'badge';
    document.getElementById('sha-badge').textContent = 'sha256:-';
    document.getElementById('diff-view').innerHTML = '<span class="diff-ctx">Ready. Click 1. Agent Explore (MCP+LLM) or Run Closed Loop (1->4).</span>';
    document.getElementById('subr-view').innerHTML = highlightJson({
      discovered_mcp_tools: evo.discovered_tools || [],
      exploration_findings: findings,
    });
    const vp = document.getElementById('verdict-pill');
    vp.textContent = 'IDLE';
    vp.className = 'badge';
    for (const id of ['m-layer','m-code','m-oracle','m-brew']) document.getElementById(id).textContent = '-';
    document.getElementById('journal-count').textContent = '0 ops';
    document.getElementById('journal-tbody').innerHTML = '<tr><td colspan="3" style="color:var(--dim)">Click 1. Agent Explore (MCP+LLM) or Run Closed Loop (1->4).</td></tr>';
  }

  // Enable/disable Abort, Speed Opt, and Multi-Seed buttons honestly
  const busyNow = Boolean(evo.busy);
  const btnAbort = document.getElementById('btn-abort');
  if (btnAbort) btnAbort.disabled = !busyNow;
  const itersAll = evo.iterations || [];
  const activeIt = (evo.active_iter_index >= 0 && evo.active_iter_index < itersAll.length) ? itersAll[evo.active_iter_index] : null;
  const activePassed = Boolean(activeIt && activeIt.execution && activeIt.execution.task_passed);
  const btnOpt = document.getElementById('btn-opt-speed');
  if (btnOpt) btnOpt.disabled = busyNow || !activePassed;
  const btnMseed = document.getElementById('btn-verify-seeds');
  if (btnMseed) btnMseed.disabled = busyNow || !activeIt;

  // Step-by-Step Value Progression Strip
  const sv = evo.step_value_summary || {};
  const setVal = (id, text, cls) => {
    const el = document.getElementById(id);
    if (!el) return;
    el.className = 'val-cell' + (cls ? ' ' + cls : '');
    const v = el.querySelector('.val-v');
    if (v) v.textContent = text;
  };
  setVal(
    'vs-explore',
    sv.explore_rounds_done ? `${sv.explore_rounds_done}R · ${sv.explore_mcp_calls} calls · ${sv.explore_refusals_found} lim` : (evo.phase === 'exploring' ? 'probing...' : 'idle'),
    sv.explore_rounds_done ? 'done' : (evo.phase === 'exploring' ? 'active' : '')
  );
  setVal(
    'vs-compile',
    itersAll.length ? `${itersAll.length} iters (${sv.ast_blocked_iters || 0} AST blk)` : (evo.phase === 'synthesizing' ? 'compiling...' : '-'),
    itersAll.length ? 'done' : (evo.phase === 'synthesizing' ? 'active' : '')
  );
  setVal(
    'vs-pass',
    sv.first_pass_wall_s != null ? `it=${sv.first_pass_iter} · ${sv.first_pass_wall_s}s (${sv.first_pass_calls}c)` : (evo.phase === 'executing' ? 'running...' : '-'),
    sv.first_pass_wall_s != null ? 'done' : (evo.phase === 'executing' ? 'active' : '')
  );
  setVal(
    'vs-opt',
    sv.optimized_wall_s != null
      ? `${sv.optimized_wall_s}s (${sv.optimized_speedup_pct != null ? (sv.optimized_speedup_pct >= 0 ? '-' + sv.optimized_speedup_pct + '%' : '+' + Math.abs(sv.optimized_speedup_pct) + '%') : 'opt'})`
      : (activePassed ? 'ready' : '-'),
    sv.optimized_wall_s != null ? 'done' : ''
  );
  const lsr = sv.last_skill_replay;
  const availSkills = evo.available_skills || [];
  setVal(
    'vs-skill',
    lsr ? `${lsr.task_passed ? 'PASS' : 'FAIL'} · ${lsr.wall_s}s (0 LLM)` : (availSkills.length ? `${availSkills.length} ready (0 LLM)` : '-'),
    lsr && lsr.task_passed ? 'done' : (availSkills.length ? 'active' : '')
  );

  // Quick One-Click 0-LLM Verified Action Library Bar right inside Closed-Loop Lab
  const sbar = document.getElementById('lab-skills-bar');
  if (sbar) {
    const usable = availSkills.filter(s => s.status === 'active' || s.status === 'advisory_sim_prior' || s.status === 'candidate');
    if (usable.length) {
      sbar.style.display = 'flex';
      sbar.innerHTML = `<strong style="color:var(--green-txt)">ACTION LIBRARY (0-LLM):</strong>` + usable.map(s =>
        `<button class="sm ok" ${busyNow ? 'disabled' : ''} onclick="runSkillDirect('${esc(s.skill_py_path)}','${esc(s.skill_name)}')" title="${esc(s.verify_detail || '')}">&#9654; ${esc(s.skill_name)} [${esc(s.status)}]</button>`
      ).join('');
    } else {
      sbar.style.display = 'none';
    }
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
  const highlighted = highlightPython(src || '');
  const lines = highlighted.split('\n');
  return lines.map((ln, idx) => `<span class="ln-num">${idx + 1}</span>${ln}`).join('\n');
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
  const mcpTel = live.mcp_telemetry || null;
  if (mcpTel) lastMcpTelemetry = mcpTel;
  const robot = (mcpTel && mcpTel.robot) || null;
  const evo = live.evolution;

  const hp = document.getElementById('hud-phase');
  const hl2 = document.getElementById('hud-l2');
  const bb = document.getElementById('hud-brew-bar');
  const bt = document.getElementById('hud-brew-txt');
  const ha = document.getElementById('hud-action');

  if (robot) {
    const ee = robot.ee_pos_m || [0, 0, 0];
    const gRatio = Number(robot.gripper_open_ratio ?? 1.0);
    if (hp) hp.textContent = `ee: [${ee.map(v => Number(v).toFixed(2)).join(', ')}]`;
    if (hl2) {
      hl2.textContent = robot.grasped_object_id ? `GRASPED: ${robot.grasped_object_id}` : `gripper: ${Math.round(gRatio * 100)}%`;
      hl2.className = 'hud-pill ' + (robot.grasped_object_id ? 'ok' : '');
    }
    if (bb) bb.style.width = `${Math.round(gRatio * 100)}%`;
    if (bt) bt.textContent = `step ${robot.step_seq || 0}`;
    document.getElementById('pk-pos').textContent = `[${ee.map(v => Number(v).toFixed(3)).join(', ')}]`;
    document.getElementById('pk-l2').textContent = `${Math.round(gRatio * 100)}% open`;
    document.getElementById('pk-wall').textContent = robot.grasped_object_id || 'none';
  } else {
    const raw = obs.raw_snapshot || {};
    if (hp) hp.textContent = `phase: ${raw.phase || 'idle'}`;
    if (hl2) { hl2.textContent = `step ${raw.step_seq || 0}`; hl2.className = 'hud-pill'; }
  }

  if (ha && evo) ha.textContent = evo.busy ? evo.busy_detail : `phase=${evo.phase} | iters=${(evo.iterations||[]).length} | llm_calls=${(evo.llm_calls||[]).length}`;

  if (evo) renderEvolution(evo);
  if (esViewerFrame && mcpTel) {
    refreshSceneReconstruction(false);
  }
  renderSpatialRadar(obs, mcpTel);
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

    if (JSON.stringify(st.cameras || []) !== JSON.stringify(camList) || !esViewerFrame) {
      camList = st.cameras || [];
      activeCam = camList.includes('front_top') ? 'front_top' : (camList.includes('overview') ? 'overview' : (camList[0] || ''));
      renderCameraShells();
    }

    // Governance tab
    document.getElementById('cap-pill').textContent = (st.capability_check||{}).status || '-';
    document.getElementById('cap-pill').className = 'badge ' + (ok ? 'ok' : 'err');
    document.getElementById('cap-meta').textContent = `authority=${d.execution_authority||'-'} · perception=${d.perception_owner||'-'} · contract_sha256=${(d.sdk_contract_sha256||'').slice(0,16)}`;
    document.getElementById('cap-ok-chips').innerHTML = (d.allowed_methods || []).map(m => `<span class="chip ok">${esc(m)}</span>`).join('');
    loadGovernanceLedger();

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
  switchCodeTab('llm');
  await api('api/evolve/probe', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({
      task_id: document.getElementById('scenario-select').value,
      task_instruction: document.getElementById('task-instruction').value,
      explore_rounds: Number(document.getElementById('explore-rounds').value || 2),
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
      task_id: document.getElementById('scenario-select').value,
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
      task_id: document.getElementById('scenario-select').value,
      source: src,
    }),
  });
  editorDirty = false;
  await pollLiveOnce();
}

async function runClosedLoop() {
  cupTrail = [];
  editorDirty = false;
  switchCodeTab('llm');
  await api('api/evolve/auto', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({
      task_id: document.getElementById('scenario-select').value,
      task_instruction: document.getElementById('task-instruction').value,
      explore_rounds: Number(document.getElementById('explore-rounds').value || 2),
      max_iters: 3,
      fresh: true,
    }),
  });
  await pollLiveOnce();
}

async function runOptimizeSpeed() {
  cupTrail = [];
  editorDirty = false;
  switchCodeTab('diff');
  try {
    await api('api/evolve/optimize_speed', {
      method: 'POST', headers: {'Content-Type':'application/json'},
      body: JSON.stringify({}),
    });
    await pollLiveOnce();
  } catch (e) {
    alert('Speed optimization: ' + e.message);
  }
}

async function runVerifyMultiSeed() {
  cupTrail = [];
  try {
    await api('api/evolve/verify_seeds', {
      method: 'POST', headers: {'Content-Type':'application/json'},
      body: JSON.stringify({
        task_id: document.getElementById('scenario-select').value,
        seeds: [0, 1, 2],
      }),
    });
    await pollLiveOnce();
  } catch (e) {
    alert('Multi-seed verification: ' + e.message);
  }
}

async function abortRun() {
  try {
    await api('api/evolve/abort', {
      method: 'POST', headers: {'Content-Type':'application/json'},
      body: JSON.stringify({ reason: 'Operator E-Stop from TRAX UI' }),
    });
    await pollLiveOnce();
  } catch (_) {}
}

async function loadGovernanceLedger() {
  try {
    const data = await api('api/audit/ledger');
    const mp = document.getElementById('gov-manifest-pre');
    if (mp) {
      mp.innerHTML = data.manifest ? highlightJson(data.manifest) : '(active backend exposes no trax://embodiment/manifest resource)';
    }
    const lb = document.getElementById('gov-audit-ledger');
    if (lb) {
      const rows = (data.records || []).slice().reverse();
      if (!rows.length) {
        lb.innerHTML = '<div style="font-family:var(--mono);font-size:11px;color:var(--dim)">No self-audit ledger records yet. Run <code>make self-audit</code>.</div>';
      } else {
        lb.innerHTML = `<table><thead><tr><th>UTC</th><th>SHA</th><th>Suite</th><th>Acc</th><th>p95 (ms)</th><th>Verdict</th></tr></thead><tbody>` +
          rows.map(r => {
            const d = r.decision || {};
            const cls = d.accepted ? 'ok' : 'err';
            return `<tr>
              <td>${esc((r.recorded_utc || '').slice(11, 19))}</td>
              <td><code>${esc(r.git_sha || '-')}</code></td>
              <td>${esc(r.suite || '-')}</td>
              <td>${r.passed}/${r.total} (${((r.accuracy || 0) * 100).toFixed(0)}%)</td>
              <td>${Number(r.p95_ms || 0).toFixed(1)}</td>
              <td><span class="badge ${cls}">${esc(d.reason_code || '-')}</span></td>
            </tr>`;
          }).join('') + '</tbody></table>';
      }
    }
  } catch (_) {}
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
  syncCodeEditorHighlight();
  const evo = await api('api/evolve/reset', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({
      task_id: document.getElementById('scenario-select').value,
      task_instruction: document.getElementById('task-instruction').value,
      explore_rounds: Number(document.getElementById('explore-rounds').value || 2),
    }),
  });
  renderEvolution(evo);
}

async function resetEnv() {
  cupTrail = [];
  if (esViewerFrame && esViewerFrame.contentWindow) {
    esViewerFrame.contentWindow.postMessage({ type: 'evidencescene:clearTrail' }, '*');
  }
  await api('api/reset', { method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify({preset:'default'}) });
  await pollLiveOnce();
  lastReconStepSeq = -1;
  refreshSceneReconstruction(true);
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
