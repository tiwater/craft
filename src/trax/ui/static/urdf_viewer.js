/* TRAX 3D Digital Twin Viewer (Pure WebGL, Zero External CDN)
 * Combines 4 engineering layers that 2D cameras cannot show:
 *   1. Robot URDF + live joint forward kinematics (`/api/robot/urdf` + `/api/robot/assets/*.stl`)
 *   2. EvidenceScene 3D metric surfel reconstruction & 6-DoF Schur observability (`/api/scene/reconstruction`)
 *   3. 3D Workspace Safety Envelope (radial min/max, z_min/z_max, zone tolerance rings)
 *   4. Live 3D End-Effector Trajectory, Joint/TCP TF coordinate triads & Limit-Refusal markers
 */
(function () {
  'use strict';

  function mat4Identity() {
    return new Float32Array([1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]);
  }
  function mat4Mul(a, b) {
    const out = new Float32Array(16);
    for (let c = 0; c < 4; c++) {
      const b0 = b[c*4], b1 = b[c*4+1], b2 = b[c*4+2], b3 = b[c*4+3];
      out[c*4]   = a[0]*b0 + a[4]*b1 + a[8]*b2  + a[12]*b3;
      out[c*4+1] = a[1]*b0 + a[5]*b1 + a[9]*b2  + a[13]*b3;
      out[c*4+2] = a[2]*b0 + a[6]*b1 + a[10]*b2 + a[14]*b3;
      out[c*4+3] = a[3]*b0 + a[7]*b1 + a[11]*b2 + a[15]*b3;
    }
    return out;
  }
  function mat4Translate(x, y, z) { const m = mat4Identity(); m[12] = x; m[13] = y; m[14] = z; return m; }
  function mat4Scale(sx, sy, sz) { const m = mat4Identity(); m[0] = sx; m[5] = sy; m[10] = sz; return m; }
  function mat4FromRpy(r, p, y) {
    const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
    return new Float32Array([
      cy*cp, sy*cp, -sp, 0,
      cy*sp*sr - sy*cr, sy*sp*sr + cy*cr, cp*sr, 0,
      cy*sp*cr + sy*sr, sy*sp*cr - cy*sr, cp*cr, 0,
      0, 0, 0, 1,
    ]);
  }
  function mat4AxisAngle(ax, ay, az, angle) {
    const len = Math.hypot(ax, ay, az) || 1;
    const x = ax / len, y = ay / len, z = az / len;
    const c = Math.cos(angle), s = Math.sin(angle), t = 1 - c;
    return new Float32Array([
      t*x*x + c, t*x*y + s*z, t*x*z - s*y, 0,
      t*x*y - s*z, t*y*y + c, t*y*z + s*x, 0,
      t*x*z + s*y, t*y*z - s*x, t*z*z + c, 0,
      0, 0, 0, 1,
    ]);
  }
  function mat4Origin(xyz, rpy) { return mat4Mul(mat4Translate(xyz[0], xyz[1], xyz[2]), mat4FromRpy(rpy[0], rpy[1], rpy[2])); }
  function mat4Perspective(fovyRad, aspect, near, far) {
    const f = 1.0 / Math.tan(fovyRad / 2), nf = 1.0 / (near - far);
    return new Float32Array([f / aspect, 0, 0, 0, 0, f, 0, 0, 0, 0, (far + near) * nf, -1, 0, 0, 2 * far * near * nf, 0]);
  }
  function mat4LookAt(eye, target, up) {
    let zx = eye[0] - target[0], zy = eye[1] - target[1], zz = eye[2] - target[2];
    const zlen = Math.hypot(zx, zy, zz) || 1; zx /= zlen; zy /= zlen; zz /= zlen;
    let xx = up[1]*zz - up[2]*zy, xy = up[2]*zx - up[0]*zz, xz = up[0]*zy - up[1]*zx;
    const xlen = Math.hypot(xx, xy, xz) || 1; xx /= xlen; xy /= xlen; xz /= xlen;
    const yx = zy*xz - zz*xy, yy = zz*xx - zx*xz, yz = zx*xy - zy*xx;
    return new Float32Array([
      xx, yx, zx, 0, xy, yy, zy, 0, xz, yz, zz, 0,
      -(xx*eye[0] + xy*eye[1] + xz*eye[2]),
      -(yx*eye[0] + yy*eye[1] + yz*eye[2]),
      -(zx*eye[0] + zy*eye[1] + zz*eye[2]), 1,
    ]);
  }

  function parseFloats(str, fallback) {
    if (!str) return fallback.slice();
    const parts = str.trim().split(/\s+/).map(Number);
    return parts.length >= fallback.length && parts.every(Number.isFinite) ? parts : fallback.slice();
  }

  function parseBinaryStl(buf) {
    if (buf.byteLength < 84) return null;
    const dv = new DataView(buf);
    const count = dv.getUint32(80, true);
    if (84 + count * 50 !== buf.byteLength) return parseAsciiStl(new TextDecoder().decode(buf));
    const pos = new Float32Array(count * 9), nrm = new Float32Array(count * 9);
    let off = 84;
    for (let i = 0; i < count; i++) {
      let nx = dv.getFloat32(off, true), ny = dv.getFloat32(off + 4, true), nz = dv.getFloat32(off + 8, true);
      const ax = dv.getFloat32(off + 12, true), ay = dv.getFloat32(off + 16, true), az = dv.getFloat32(off + 20, true);
      const bx = dv.getFloat32(off + 24, true), by = dv.getFloat32(off + 28, true), bz = dv.getFloat32(off + 32, true);
      const cx = dv.getFloat32(off + 36, true), cy = dv.getFloat32(off + 40, true), cz = dv.getFloat32(off + 44, true);
      off += 50;
      if (Math.hypot(nx, ny, nz) < 1e-6) {
        const ux = bx - ax, uy = by - ay, uz = bz - az, vx = cx - ax, vy = cy - ay, vz = cz - az;
        nx = uy * vz - uz * vy; ny = uz * vx - ux * vz; nz = ux * vy - uy * vx;
        const nl = Math.hypot(nx, ny, nz) || 1; nx /= nl; ny /= nl; nz /= nl;
      }
      const p = i * 9;
      pos[p] = ax; pos[p+1] = ay; pos[p+2] = az;
      pos[p+3] = bx; pos[p+4] = by; pos[p+5] = bz;
      pos[p+6] = cx; pos[p+7] = cy; pos[p+8] = cz;
      for (let k = 0; k < 3; k++) { nrm[p + k*3] = nx; nrm[p + k*3 + 1] = ny; nrm[p + k*3 + 2] = nz; }
    }
    return { pos, nrm, vertexCount: count * 3 };
  }

  function parseAsciiStl(txt) {
    const verts = [], norms = [];
    const re = /facet\s+normal\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)[\s\S]*?vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)[\s\S]*?vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)[\s\S]*?vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)/g;
    let m;
    while ((m = re.exec(txt)) !== null) {
      const nx = +m[1], ny = +m[2], nz = +m[3];
      for (let k = 0; k < 3; k++) { verts.push(+m[4 + k*3], +m[5 + k*3], +m[6 + k*3]); norms.push(nx, ny, nz); }
    }
    return { pos: new Float32Array(verts), nrm: new Float32Array(norms), vertexCount: verts.length / 3 };
  }

  function pushRing(pos, col, cx, cy, z, r, segs, rgb) {
    for (let i = 0; i < segs; i++) {
      const a0 = (i / segs) * Math.PI * 2, a1 = ((i + 1) / segs) * Math.PI * 2;
      pos.push(cx + r * Math.cos(a0), cy + r * Math.sin(a0), z, cx + r * Math.cos(a1), cy + r * Math.sin(a1), z);
      col.push(rgb[0], rgb[1], rgb[2], rgb[0], rgb[1], rgb[2]);
    }
  }

  function pushTriad(pos, col, mat, size) {
    const ox = mat[12], oy = mat[13], oz = mat[14];
    const axes = [
      [[mat[0]*size, mat[1]*size, mat[2]*size], [0.92, 0.22, 0.22]],
      [[mat[4]*size, mat[5]*size, mat[6]*size], [0.16, 0.80, 0.34]],
      [[mat[8]*size, mat[9]*size, mat[10]*size], [0.22, 0.52, 0.96]],
    ];
    for (const [v, rgb] of axes) {
      pos.push(ox, oy, oz, ox + v[0], oy + v[1], oz + v[2]);
      col.push(rgb[0], rgb[1], rgb[2], rgb[0], rgb[1], rgb[2]);
    }
  }

  class TraxURDFViewer {
    constructor(canvas, statusEl) {
      this.canvas = canvas;
      this.statusEl = statusEl;
      this.gl = canvas.getContext('webgl', { antialias: true, alpha: false });
      this.robot = null;
      this.meshBuffers = new Map();
      this.jointAngles = {};
      this.sceneZones = {};
      this.workspaceBounds = { radial_from_base_min_m: 0.10, radial_from_base_max_m: 0.34, z_min_m: 1.025, z_max_m: 1.28, table_top_z_m: 1.0 };
      this.baseWorld = [0.13, 0.0, 1.0];
      this.eeTrail = [];
      this.refusalPoints = [];
      this.lastEeLocal = null;

      this.layers = { surfels: true, env: true, traj: true, tf: true, cameras: true, urdf: true };
      this.surfelBuffer = null;
      this.surfelCount = 0;
      this.rawSurfels = [];
      this.reconMeta = null;
      this.selectedTarget = null;
      this.pickMode = false;
      this.onSurfacePick = null;
      this.lastProj = null;
      this.lastView = null;
      this.cameraMeta = [];
      this.camBuf = null;
      this.camCount = 0;
      this.envBuf = null;
      this.envCount = 0;

      this.azimuth = -0.82;
      this.elevation = 0.48;
      this.distance = 0.68;
      this.target = [0.18, 0.0, 0.09];
      this.dragging = false;
      this.lastX = 0;
      this.lastY = 0;
      this.rafId = 0;
      this.disposed = false;

      if (this.gl) {
        this._initGL();
        this._rebuildEnvelopeBuffer();
        this._bindPointer();
        this._startLoop();
      } else if (this.statusEl) {
        this.statusEl.textContent = 'WebGL unavailable in this browser';
      }
    }

    toggleLayer(key) {
      if (key in this.layers) this.layers[key] = !this.layers[key];
      return this.layers[key];
    }

    setPickMode(enabled) {
      this.pickMode = !!enabled;
      if (this.canvas) this.canvas.style.cursor = this.pickMode ? 'crosshair' : 'grab';
      this._updateStatusText();
      return this.pickMode;
    }

    setSelectedTarget(sel) {
      this.selectedTarget = sel || null;
      this._updateStatusText();
    }

    _compile(type, src) {
      const gl = this.gl;
      const sh = gl.createShader(type);
      gl.shaderSource(sh, src);
      gl.compileShader(sh);
      return sh;
    }

    _initGL() {
      const gl = this.gl;
      const vs = `
        attribute vec3 aPos; attribute vec3 aNrm;
        uniform mat4 uProj, uView, uModel;
        varying vec3 vNrm;
        void main() { gl_Position = uProj * uView * uModel * vec4(aPos, 1.0); vNrm = normalize(mat3(uModel) * aNrm); }`;
      const fs = `
        precision mediump float; uniform vec4 uColor; varying vec3 vNrm;
        void main() {
          vec3 l1 = normalize(vec3(0.45, -0.55, 0.85));
          vec3 l2 = normalize(vec3(-0.6, 0.4, 0.5));
          float d1 = max(dot(normalize(vNrm), l1), 0.0);
          float d2 = max(dot(normalize(vNrm), l2), 0.0);
          gl_FragColor = vec4(uColor.rgb * (0.40 + 0.50 * d1 + 0.20 * d2), uColor.a);
        }`;
      const prog = gl.createProgram();
      gl.attachShader(prog, this._compile(gl.VERTEX_SHADER, vs));
      gl.attachShader(prog, this._compile(gl.FRAGMENT_SHADER, fs));
      gl.linkProgram(prog);
      this.prog = prog;
      this.loc = {
        aPos: gl.getAttribLocation(prog, 'aPos'), aNrm: gl.getAttribLocation(prog, 'aNrm'),
        uProj: gl.getUniformLocation(prog, 'uProj'), uView: gl.getUniformLocation(prog, 'uView'),
        uModel: gl.getUniformLocation(prog, 'uModel'), uColor: gl.getUniformLocation(prog, 'uColor'),
      };

      const pvs = `
        attribute vec3 aPos; attribute vec3 aCol;
        uniform mat4 uProj, uView; uniform float uPointSize;
        varying vec3 vCol;
        void main() { gl_Position = uProj * uView * vec4(aPos, 1.0); gl_PointSize = uPointSize; vCol = aCol; }`;
      const pfs = `precision mediump float; varying vec3 vCol; void main() { gl_FragColor = vec4(vCol, 1.0); }`;
      const pprog = gl.createProgram();
      gl.attachShader(pprog, this._compile(gl.VERTEX_SHADER, pvs));
      gl.attachShader(pprog, this._compile(gl.FRAGMENT_SHADER, pfs));
      gl.linkProgram(pprog);
      this.pprog = pprog;
      this.ploc = {
        aPos: gl.getAttribLocation(pprog, 'aPos'), aCol: gl.getAttribLocation(pprog, 'aCol'),
        uProj: gl.getUniformLocation(pprog, 'uProj'), uView: gl.getUniformLocation(pprog, 'uView'),
        uPointSize: gl.getUniformLocation(pprog, 'uPointSize'),
      };

      // Subtle tabletop coordinate grid
      const gPos = [], gCol = [];
      for (let i = -4; i <= 4; i++) {
        const y = i * 0.08, c = i === 0 ? 0.45 : 0.72;
        gPos.push(-0.08, y, 0, 0.56, y, 0);
        gCol.push(c, c, c, c, c, c);
      }
      for (let i = -1; i <= 7; i++) {
        const x = i * 0.08, c = i === 0 ? 0.45 : 0.72;
        gPos.push(x, -0.32, 0, x, 0.32, 0);
        gCol.push(c, c, c, c, c, c);
      }
      this.gridBuf = this._uploadPoints(new Float32Array(gPos), new Float32Array(gCol));
      this.gridCount = gPos.length / 3;

      this.dynPosBuf = gl.createBuffer();
      this.dynColBuf = gl.createBuffer();
    }

    _uploadMesh(m) {
      const gl = this.gl;
      const pb = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, pb); gl.bufferData(gl.ARRAY_BUFFER, m.pos, gl.STATIC_DRAW);
      const nb = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, nb); gl.bufferData(gl.ARRAY_BUFFER, m.nrm, gl.STATIC_DRAW);
      return { posBuf: pb, nrmBuf: nb, count: m.vertexCount };
    }

    _uploadPoints(posArr, colArr) {
      const gl = this.gl;
      const pb = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, pb); gl.bufferData(gl.ARRAY_BUFFER, posArr, gl.STATIC_DRAW);
      const cb = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, cb); gl.bufferData(gl.ARRAY_BUFFER, colArr, gl.STATIC_DRAW);
      return { posBuf: pb, colBuf: cb };
    }

    _rebuildEnvelopeBuffer() {
      if (!this.gl) return;
      const b = this.workspaceBounds || {};
      const rMin = Number(b.radial_from_base_min_m ?? 0.10);
      const rMax = Number(b.radial_from_base_max_m ?? 0.34);
      const zMin = Number(b.z_min_m ?? 1.025) - this.baseWorld[2];
      const zMax = Number(b.z_max_m ?? 1.28) - this.baseWorld[2];
      const pos = [], col = [];
      const cyan = [0.05, 0.65, 0.92], amber = [0.92, 0.56, 0.08], green = [0.12, 0.78, 0.42];

      // Reachability cylindrical shell (rMax & rMin at zMin & zMax)
      pushRing(pos, col, 0, 0, zMin, rMax, 48, cyan);
      pushRing(pos, col, 0, 0, zMax, rMax, 48, cyan);
      pushRing(pos, col, 0, 0, zMin, rMin, 28, amber);
      for (let i = 0; i < 8; i++) {
        const a = (i / 8) * Math.PI * 2;
        const x = rMax * Math.cos(a), y = rMax * Math.sin(a);
        pos.push(x, y, zMin, x, y, zMax);
        col.push(cyan[0], cyan[1], cyan[2], cyan[0], cyan[1], cyan[2]);
      }

      // Target zone acceptance tolerance rings (32mm / 28mm)
      for (const [zname, xyz] of Object.entries(this.sceneZones || {})) {
        const lx = Number(xyz[0]) - this.baseWorld[0];
        const ly = Number(xyz[1]) - this.baseWorld[1];
        const rad = zname === 'station_pad' ? 0.028 : 0.032;
        const c = zname === 'station_pad' ? green : (zname === 'target_zone_b' ? amber : cyan);
        pushRing(pos, col, lx, ly, 0.004, rad, 28, c);
        pushRing(pos, col, lx, ly, 0.020, rad, 28, c);
      }

      this.envBuf = this._uploadPoints(new Float32Array(pos), new Float32Array(col));
      this.envCount = pos.length / 3;
    }

    _rebuildCameraFrustums() {
      if (!this.gl) return;
      const cams = Array.isArray(this.cameraMeta) ? this.cameraMeta : [];
      if (!cams.length) { this.camBuf = null; this.camCount = 0; return; }
      const pos = [], col = [];
      const [bx, by, bz] = this.baseWorld;
      const xf = (T, cx, cy, cz) => [
        T[0][0] * cx + T[0][1] * cy + T[0][2] * cz + T[0][3] - bx,
        T[1][0] * cx + T[1][1] * cy + T[1][2] * cz + T[1][3] - by,
        T[2][0] * cx + T[2][1] * cy + T[2][2] * cz + T[2][3] - bz,
      ];
      const pushSeg = (a, b, rgb) => {
        pos.push(a[0], a[1], a[2], b[0], b[1], b[2]);
        col.push(rgb[0], rgb[1], rgb[2], rgb[0], rgb[1], rgb[2]);
      };
      for (const cam of cams) {
        const T = cam.T_world_cam;
        if (!Array.isArray(T) || T.length < 3) continue;
        const isWrist = cam.mount === 'eye_in_hand' || cam.name === 'wrist';
        const rgb = isWrist ? [0.98, 0.62, 0.12] : [0.06, 0.72, 0.98];
        const rayRgb = isWrist ? [0.98, 0.74, 0.34] : [0.35, 0.82, 0.98];
        const fovy = ((Number(cam.fovy_deg) || 58) * Math.PI) / 180;
        const aspect = Number(cam.aspect) || (16 / 9);
        const d = isWrist ? 0.065 : 0.135;
        const hh = d * Math.tan(fovy * 0.5);
        const hw = hh * aspect;
        const o = xf(T, 0, 0, 0);
        const tl = xf(T, -hw, -hh, d);
        const tr = xf(T,  hw, -hh, d);
        const br = xf(T,  hw,  hh, d);
        const bl = xf(T, -hw,  hh, d);
        const upTip = xf(T, 0, -hh * 1.28, d);
        const rayEnd = xf(T, 0, 0, isWrist ? 0.16 : 0.44);
        // Pyramid rays & image plane rectangle
        pushSeg(o, tl, rgb); pushSeg(o, tr, rgb); pushSeg(o, br, rgb); pushSeg(o, bl, rgb);
        pushSeg(tl, tr, rgb); pushSeg(tr, br, rgb); pushSeg(br, bl, rgb); pushSeg(bl, tl, rgb);
        // Top-edge orientation chevron & optical axis
        pushSeg(tl, upTip, rgb); pushSeg(tr, upTip, rgb);
        pushSeg(o, rayEnd, rayRgb);
      }
      this.camBuf = this._uploadPoints(new Float32Array(pos), new Float32Array(col));
      this.camCount = pos.length / 3;
    }

    _pickSurfelAtClient(clientX, clientY) {
      if (!this.lastProj || !this.lastView || !this.rawSurfels.length) return null;
      const rect = this.canvas.getBoundingClientRect();
      if (!rect.width || !rect.height) return null;
      const nx = ((clientX - rect.left) / rect.width) * 2 - 1;
      const ny = 1 - ((clientY - rect.top) / rect.height) * 2;
      const vp = mat4Mul(this.lastProj, this.lastView);
      const [bx, by, bz] = this.baseWorld;
      let best = null;
      let bestScore = 1e9;
      for (let i = 0; i < this.rawSurfels.length; i++) {
        const pt = this.rawSurfels[i];
        const lx = Number(pt[0]) - bx, ly = Number(pt[1]) - by, lz = Number(pt[2]) - bz;
        const cx = vp[0]*lx + vp[4]*ly + vp[8]*lz + vp[12];
        const cy = vp[1]*lx + vp[5]*ly + vp[9]*lz + vp[13];
        const cz = vp[2]*lx + vp[6]*ly + vp[10]*lz + vp[14];
        const cw = vp[3]*lx + vp[7]*ly + vp[11]*lz + vp[15];
        if (cw <= 1e-4) continue;
        const sx = cx / cw, sy = cy / cw, sz = cz / cw;
        const d2 = Math.hypot(sx - nx, sy - ny);
        if (d2 > 0.065) continue;
        const score = d2 + Math.max(0, sz) * 0.015;
        if (score < bestScore) {
          bestScore = score;
          best = [Number(pt[0]), Number(pt[1]), Number(pt[2])];
        }
      }
      return best;
    }

    _bindPointer() {
      let downX = 0, downY = 0;
      this.canvas.addEventListener('pointerdown', (e) => {
        downX = e.clientX; downY = e.clientY;
        if (this.pickMode || e.shiftKey) return;
        this.dragging = true; this.lastX = e.clientX; this.lastY = e.clientY;
        this.canvas.setPointerCapture(e.pointerId);
      });
      this.canvas.addEventListener('pointermove', (e) => {
        if (!this.dragging) return;
        const dx = e.clientX - this.lastX, dy = e.clientY - this.lastY;
        this.lastX = e.clientX; this.lastY = e.clientY;
        this.azimuth -= dx * 0.008;
        this.elevation = Math.max(0.08, Math.min(1.48, this.elevation + dy * 0.007));
      });
      this.canvas.addEventListener('pointerup', (e) => {
        this.dragging = false;
        if ((this.pickMode || e.shiftKey) && Math.hypot(e.clientX - downX, e.clientY - downY) < 6) {
          const hit = this._pickSurfelAtClient(e.clientX, e.clientY);
          if (hit && typeof this.onSurfacePick === 'function') this.onSurfacePick(hit);
        }
      });
      this.canvas.addEventListener('wheel', (e) => {
        e.preventDefault();
        this.distance = Math.max(0.28, Math.min(1.6, this.distance * (1 + e.deltaY * 0.001)));
      }, { passive: false });
      this.canvas.addEventListener('dblclick', () => {
        this.azimuth = -0.82; this.elevation = 0.48; this.distance = 0.68;
      });
    }

    async loadUrdf(url) {
      if (!this.gl) return;
      if (this.statusEl) this.statusEl.textContent = 'Loading URDF & EvidenceScene 3D surfels...';
      try {
        const r = await fetch(url);
        if (!r.ok) {
          this.robot = null;
          this._updateStatusText();
          return;
        }
        const doc = new DOMParser().parseFromString(await r.text(), 'application/xml');
        const robotEl = doc.querySelector('robot');
        if (!robotEl) throw new Error('Invalid URDF XML');

        const materials = {};
        robotEl.querySelectorAll(':scope > material').forEach((m) => {
          const name = m.getAttribute('name'), c = m.querySelector('color');
          if (name && c) materials[name] = parseFloats(c.getAttribute('rgba'), [0.75, 0.78, 0.82, 1]);
        });
        const links = {}, meshUrls = new Set();
        robotEl.querySelectorAll(':scope > link').forEach((l) => {
          const name = l.getAttribute('name'), visuals = [];
          l.querySelectorAll(':scope > visual').forEach((v) => {
            const orig = v.querySelector('origin');
            const xyz = parseFloats(orig && orig.getAttribute('xyz'), [0, 0, 0]);
            const rpy = parseFloats(orig && orig.getAttribute('rpy'), [0, 0, 0]);
            const meshEl = v.querySelector('geometry > mesh');
            if (!meshEl) return;
            const filename = meshEl.getAttribute('filename') || '';
            const scale = parseFloats(meshEl.getAttribute('scale'), [1, 1, 1]);
            const matEl = v.querySelector('material');
            const rgba = matEl?.querySelector('color')
              ? parseFloats(matEl.querySelector('color').getAttribute('rgba'), [0.78, 0.80, 0.84, 1])
              : (materials[matEl?.getAttribute('name') || ''] || [0.78, 0.80, 0.84, 1]);
            if (filename) meshUrls.add(filename);
            visuals.push({ filename, scale, origin: mat4Origin(xyz, rpy), rgba });
          });
          links[name] = { name, visuals };
        });

        const joints = [], childSet = new Set();
        robotEl.querySelectorAll(':scope > joint').forEach((j) => {
          const name = j.getAttribute('name') || '', type = j.getAttribute('type') || 'fixed';
          const parent = j.querySelector('parent')?.getAttribute('link') || '';
          const child = j.querySelector('child')?.getAttribute('link') || '';
          const orig = j.querySelector('origin');
          const xyz = parseFloats(orig && orig.getAttribute('xyz'), [0, 0, 0]);
          const rpy = parseFloats(orig && orig.getAttribute('rpy'), [0, 0, 0]);
          const axis = parseFloats(j.querySelector('axis')?.getAttribute('xyz'), [0, 0, 1]);
          childSet.add(child);
          joints.push({ name, type, parent, child, origin: mat4Origin(xyz, rpy), axis });
        });

        const rootLink = Object.keys(links).find((n) => !childSet.has(n)) || Object.keys(links)[0];
        const childrenByParent = {};
        for (const j of joints) (childrenByParent[j.parent] = childrenByParent[j.parent] || []).push(j);

        await Promise.all([...meshUrls].map(async (u) => {
          if (this.meshBuffers.has(u)) return;
          try {
            const mr = await fetch(u);
            if (!mr.ok) return;
            const parsed = parseBinaryStl(await mr.arrayBuffer());
            if (parsed && parsed.vertexCount > 0) this.meshBuffers.set(u, this._uploadMesh(parsed));
          } catch (_) {}
        }));

        this.robot = { name: robotEl.getAttribute('name') || 'robot', links, childrenByParent, rootLink };
        this._updateStatusText();
      } catch (e) {
        if (this.statusEl) this.statusEl.textContent = `URDF error: ${e.message}`;
      }
    }

    setReconstruction(recon) {
      if (!recon || !Array.isArray(recon.surfels)) return;
      this.reconMeta = recon;
      this.rawSurfels = recon.surfels;
      if ('selected_surface_target' in recon) this.selectedTarget = recon.selected_surface_target || null;
      this.cameraMeta = Array.isArray(recon.cameras) ? recon.cameras : [];
      this._rebuildCameraFrustums();
      const pts = recon.surfels;
      if (!this.gl || !pts.length) { this.surfelCount = 0; this._updateStatusText(); return; }
      const pos = new Float32Array(pts.length * 3);
      const col = new Float32Array(pts.length * 3);
      for (let i = 0; i < pts.length; i++) {
        const pt = pts[i];
        pos[i*3]   = Number(pt[0]) - this.baseWorld[0];
        pos[i*3+1] = Number(pt[1]) - this.baseWorld[1];
        pos[i*3+2] = Number(pt[2]) - this.baseWorld[2];
        const camCnt = Number(pt[6] || 1);
        // Subtle brightness boost for multi-camera corroborated surfels (cam_count >= 2)
        const boost = camCnt >= 2 ? 1.08 : 0.85;
        col[i*3]   = Math.min(1.0, Number(pt[3] ?? 0.15) * boost);
        col[i*3+1] = Math.min(1.0, Number(pt[4] ?? 0.72) * boost);
        col[i*3+2] = Math.min(1.0, Number(pt[5] ?? 0.92) * boost);
      }
      this.surfelBuffer = this._uploadPoints(pos, col);
      this.surfelCount = pts.length;
      this._updateStatusText();
    }

    _updateStatusText() {
      if (!this.statusEl) return;
      const parts = [];
      if (this.robot) parts.push(`URDF: ${this.robot.name}`);
      if (this.reconMeta) {
        const obs = this.reconMeta.observability || {};
        const cur = this.reconMeta.cursor;
        if (cur) parts.push(`SceneStore ep=${cur.epoch} rev=${cur.revision}`);
        const pct = Math.round((this.reconMeta.corroborated_ratio || 0) * 100);
        const cams = (this.reconMeta.cameras_used || []).join('+');
        parts.push(`${this.reconMeta.total_surfels} surfels (${pct}% 2-cam${cams ? ' · ' + cams : ''})`);
        if (obs.rank != null) parts.push(`Schur ${obs.rank}/${obs.required_rank || 6}`);
      }
      if (this.selectedTarget && this.selectedTarget.scene_point_m) {
        const p = this.selectedTarget.scene_point_m.map((v) => Number(v).toFixed(3)).join(',');
        parts.push(`target=${this.selectedTarget.entity_id}@(${p})`);
      }
      parts.push(this.pickMode ? 'CLICK surfel to commit target' : 'drag=orbit · shift+click=pick target');
      this.statusEl.textContent = parts.join('  ·  ');
    }

    setTelemetry(mcpTel) {
      if (!mcpTel) return;
      const robot = mcpTel.robot;
      let envDirty = false;
      if (robot) {
        if (Array.isArray(robot.base_pos_m) && robot.base_pos_m.length === 3) {
          this.baseWorld = robot.base_pos_m.map(Number);
        }
        if (robot.workspace_bounds_m) {
          this.workspaceBounds = robot.workspace_bounds_m;
          envDirty = true;
        }
        const names = robot.joint_names || [], vals = robot.joint_pos_rad || [];
        for (let i = 0; i < names.length; i++) this.jointAngles[names[i]] = Number(vals[i] || 0);
        if (robot.gripper_open_ratio != null) {
          this.jointAngles['gripper'] = 0.10 + Number(robot.gripper_open_ratio) * (0.88 - 0.10);
        }
        if (Array.isArray(robot.ee_pos_m) && robot.ee_pos_m.length === 3) {
          const lx = Number(robot.ee_pos_m[0]) - this.baseWorld[0];
          const ly = Number(robot.ee_pos_m[1]) - this.baseWorld[1];
          const lz = Number(robot.ee_pos_m[2]) - this.baseWorld[2];
          this.lastEeLocal = [lx, ly, lz];
          const prev = this.eeTrail[this.eeTrail.length - 1];
          if (!prev || Math.hypot(lx - prev[0], ly - prev[1], lz - prev[2]) > 0.004) {
            this.eeTrail.push([lx, ly, lz]);
            if (this.eeTrail.length > 160) this.eeTrail.shift();
          }
        }
      }
      if (mcpTel.scene && mcpTel.scene.zones) {
        this.sceneZones = mcpTel.scene.zones;
        envDirty = true;
      }
      if (envDirty) this._rebuildEnvelopeBuffer();
    }

    clearTrail() { this.eeTrail = []; this.refusalPoints = []; }

    _drawMesh(buf, modelMat, rgba) {
      const gl = this.gl;
      gl.uniformMatrix4fv(this.loc.uModel, false, modelMat);
      gl.uniform4f(this.loc.uColor, rgba[0], rgba[1], rgba[2], rgba[3] ?? 1.0);
      gl.bindBuffer(gl.ARRAY_BUFFER, buf.posBuf);
      gl.enableVertexAttribArray(this.loc.aPos);
      gl.vertexAttribPointer(this.loc.aPos, 3, gl.FLOAT, false, 0, 0);
      gl.bindBuffer(gl.ARRAY_BUFFER, buf.nrmBuf);
      gl.enableVertexAttribArray(this.loc.aNrm);
      gl.vertexAttribPointer(this.loc.aNrm, 3, gl.FLOAT, false, 0, 0);
      gl.drawArrays(gl.TRIANGLES, 0, buf.count);
    }

    _drawDynLines(posArr, colArr, mode) {
      if (!posArr.length) return;
      const gl = this.gl;
      gl.bindBuffer(gl.ARRAY_BUFFER, this.dynPosBuf);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(posArr), gl.DYNAMIC_DRAW);
      gl.enableVertexAttribArray(this.ploc.aPos);
      gl.vertexAttribPointer(this.ploc.aPos, 3, gl.FLOAT, false, 0, 0);
      gl.bindBuffer(gl.ARRAY_BUFFER, this.dynColBuf);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(colArr), gl.DYNAMIC_DRAW);
      gl.enableVertexAttribArray(this.ploc.aCol);
      gl.vertexAttribPointer(this.ploc.aCol, 3, gl.FLOAT, false, 0, 0);
      gl.drawArrays(mode, 0, posArr.length / 3);
    }

    _renderFrame() {
      if (this.disposed || !this.gl || !this.canvas.isConnected) return;
      const gl = this.gl;
      const dpr = Math.min(window.devicePixelRatio || 1, 2);
      const w = Math.max(320, Math.floor(this.canvas.clientWidth * dpr));
      const h = Math.max(180, Math.floor(this.canvas.clientHeight * dpr));
      if (this.canvas.width !== w || this.canvas.height !== h) { this.canvas.width = w; this.canvas.height = h; }
      gl.viewport(0, 0, w, h);
      const isDark = document.documentElement.dataset.theme === 'dark';
      if (isDark) gl.clearColor(0.04, 0.06, 0.11, 1.0);
      else gl.clearColor(0.95, 0.97, 0.99, 1.0);
      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
      gl.enable(gl.DEPTH_TEST);

      const eye = [
        this.target[0] + this.distance * Math.cos(this.elevation) * Math.cos(this.azimuth),
        this.target[1] + this.distance * Math.cos(this.elevation) * Math.sin(this.azimuth),
        this.target[2] + this.distance * Math.sin(this.elevation),
      ];
      const proj = mat4Perspective((42 * Math.PI) / 180, w / h, 0.02, 10.0);
      const view = mat4LookAt(eye, this.target, [0, 0, 1]);
      this.lastProj = proj;
      this.lastView = view;

      gl.useProgram(this.pprog);
      gl.uniformMatrix4fv(this.ploc.uProj, false, proj);
      gl.uniformMatrix4fv(this.ploc.uView, false, view);

      // 1. Grid
      gl.bindBuffer(gl.ARRAY_BUFFER, this.gridBuf.posBuf);
      gl.enableVertexAttribArray(this.ploc.aPos);
      gl.vertexAttribPointer(this.ploc.aPos, 3, gl.FLOAT, false, 0, 0);
      gl.bindBuffer(gl.ARRAY_BUFFER, this.gridBuf.colBuf);
      gl.enableVertexAttribArray(this.ploc.aCol);
      gl.vertexAttribPointer(this.ploc.aCol, 3, gl.FLOAT, false, 0, 0);
      gl.drawArrays(gl.LINES, 0, this.gridCount);

      // 2. Workspace Safety Envelope + Target Tolerance Cylinders
      if (this.layers.env && this.envBuf && this.envCount > 0) {
        gl.bindBuffer(gl.ARRAY_BUFFER, this.envBuf.posBuf);
        gl.vertexAttribPointer(this.ploc.aPos, 3, gl.FLOAT, false, 0, 0);
        gl.bindBuffer(gl.ARRAY_BUFFER, this.envBuf.colBuf);
        gl.vertexAttribPointer(this.ploc.aCol, 3, gl.FLOAT, false, 0, 0);
        gl.drawArrays(gl.LINES, 0, this.envCount);
      }

      // 2b. Calibrated Camera Frustums (Global Front-Top + Moving Wrist Eye-in-Hand)
      if (this.layers.cameras && this.camBuf && this.camCount > 0) {
        gl.bindBuffer(gl.ARRAY_BUFFER, this.camBuf.posBuf);
        gl.vertexAttribPointer(this.ploc.aPos, 3, gl.FLOAT, false, 0, 0);
        gl.bindBuffer(gl.ARRAY_BUFFER, this.camBuf.colBuf);
        gl.vertexAttribPointer(this.ploc.aCol, 3, gl.FLOAT, false, 0, 0);
        gl.drawArrays(gl.LINES, 0, this.camCount);
      }

      // 3. EvidenceScene Reconstructed 3D Surfels (Metric Point Cloud)
      if (this.layers.surfels && this.surfelBuffer && this.surfelCount > 0) {
        gl.uniform1f(this.ploc.uPointSize, 4.2 * dpr);
        gl.bindBuffer(gl.ARRAY_BUFFER, this.surfelBuffer.posBuf);
        gl.vertexAttribPointer(this.ploc.aPos, 3, gl.FLOAT, false, 0, 0);
        gl.bindBuffer(gl.ARRAY_BUFFER, this.surfelBuffer.colBuf);
        gl.vertexAttribPointer(this.ploc.aCol, 3, gl.FLOAT, false, 0, 0);
        gl.drawArrays(gl.POINTS, 0, this.surfelCount);
      }

      // 4. Live 3D EE Trajectory Polyline
      if (this.layers.traj && this.eeTrail.length >= 2) {
        const tp = [], tc = [];
        for (let i = 0; i < this.eeTrail.length; i++) {
          const p = this.eeTrail[i], a = 0.35 + 0.65 * (i / this.eeTrail.length);
          tp.push(p[0], p[1], p[2]);
          tc.push(0.05 * a, 0.72 * a, 0.98 * a);
        }
        this._drawDynLines(tp, tc, gl.LINE_STRIP);
      }

      // 5. Robot URDF + collect Joint TF frames
      const tfPos = [], tfCol = [];
      if (this.layers.tf) {
        pushTriad(tfPos, tfCol, mat4Identity(), 0.08);
        if (this.lastEeLocal) {
          pushTriad(tfPos, tfCol, mat4Translate(this.lastEeLocal[0], this.lastEeLocal[1], this.lastEeLocal[2]), 0.045);
        }
      }
      if (this.robot && this.robot.rootLink) {
        gl.useProgram(this.prog);
        gl.uniformMatrix4fv(this.loc.uProj, false, proj);
        gl.uniformMatrix4fv(this.loc.uView, false, view);
        const drawLink = (linkName, worldMat) => {
          const link = this.robot.links[linkName];
          if (this.layers.urdf && link) {
            for (const v of link.visuals) {
              const buf = this.meshBuffers.get(v.filename);
              if (!buf) continue;
              const visMat = mat4Mul(worldMat, mat4Mul(v.origin, mat4Scale(v.scale[0], v.scale[1], v.scale[2])));
              this._drawMesh(buf, visMat, v.rgba);
            }
          }
          for (const j of this.robot.childrenByParent[linkName] || []) {
            let childMat = mat4Mul(worldMat, j.origin);
            if (j.type === 'revolute' || j.type === 'continuous') {
              const q = Number(this.jointAngles[j.name] || 0);
              childMat = mat4Mul(childMat, mat4AxisAngle(j.axis[0], j.axis[1], j.axis[2], q));
              if (this.layers.tf) pushTriad(tfPos, tfCol, childMat, 0.032);
            }
            drawLink(j.child, childMat);
          }
        };
        drawLink(this.robot.rootLink, mat4Identity());
      }

      // 6. Draw Joint & TCP TF triads + Operator Selected Surface Target marker on top
      if (this.selectedTarget && Array.isArray(this.selectedTarget.scene_point_m)) {
        const sp = this.selectedTarget.scene_point_m;
        const sx = Number(sp[0]) - this.baseWorld[0];
        const sy = Number(sp[1]) - this.baseWorld[1];
        const sz = Number(sp[2]) - this.baseWorld[2];
        const mag = [0.95, 0.16, 0.62];
        pushRing(tfPos, tfCol, sx, sy, sz + 0.003, 0.018, 24, mag);
        pushRing(tfPos, tfCol, sx, sy, sz + 0.012, 0.010, 20, mag);
        const s = 0.028;
        tfPos.push(sx - s, sy, sz + 0.004, sx + s, sy, sz + 0.004, sx, sy - s, sz + 0.004, sx, sy + s, sz + 0.004, sx, sy, sz, sx, sy, sz + 0.045);
        for (let k = 0; k < 6; k++) tfCol.push(mag[0], mag[1], mag[2]);
      }
      if (tfPos.length) {
        gl.useProgram(this.pprog);
        gl.uniformMatrix4fv(this.ploc.uProj, false, proj);
        gl.uniformMatrix4fv(this.ploc.uView, false, view);
        this._drawDynLines(tfPos, tfCol, gl.LINES);
      }
    }

    _startLoop() {
      const tick = () => {
        if (this.disposed) return;
        this._renderFrame();
        this.rafId = requestAnimationFrame(tick);
      };
      this.rafId = requestAnimationFrame(tick);
    }

    dispose() { this.disposed = true; if (this.rafId) cancelAnimationFrame(this.rafId); }
  }

  window.TraxURDFViewer = TraxURDFViewer;
})();
