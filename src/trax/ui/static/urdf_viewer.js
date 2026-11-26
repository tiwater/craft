/* Zero-dependency pure-WebGL URDF + STL + Live Joint Kinematics + Surfel Point-Cloud Viewer.
 * Fetches any embodiment's URDF dynamically from `api/robot/urdf` and binary/ASCII STL meshes
 * from `api/robot/assets/...` (no external CDN, works offline & behind reverse proxies).
 * Includes a `setSceneSurfels(points)` layer ready for EvidenceScene 3D reconstruction.
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

  function mat4Translate(x, y, z) {
    const m = mat4Identity();
    m[12] = x; m[13] = y; m[14] = z;
    return m;
  }

  function mat4Scale(sx, sy, sz) {
    const m = mat4Identity();
    m[0] = sx; m[5] = sy; m[10] = sz;
    return m;
  }

  function mat4FromRpy(r, p, y) {
    const cr = Math.cos(r), sr = Math.sin(r);
    const cp = Math.cos(p), sp = Math.sin(p);
    const cy = Math.cos(y), sy = Math.sin(y);
    // R = Rz(y) * Ry(p) * Rx(r), column-major
    return new Float32Array([
      cy*cp,                sy*cp,                -sp,    0,
      cy*sp*sr - sy*cr,     sy*sp*sr + cy*cr,     cp*sr,  0,
      cy*sp*cr + sy*sr,     sy*sp*cr - cy*sr,     cp*cr,  0,
      0,                    0,                    0,      1,
    ]);
  }

  function mat4AxisAngle(ax, ay, az, angle) {
    const len = Math.hypot(ax, ay, az) || 1;
    const x = ax / len, y = ay / len, z = az / len;
    const c = Math.cos(angle), s = Math.sin(angle), t = 1 - c;
    return new Float32Array([
      t*x*x + c,     t*x*y + s*z,   t*x*z - s*y,   0,
      t*x*y - s*z,   t*y*y + c,     t*y*z + s*x,   0,
      t*x*z + s*y,   t*y*z - s*x,   t*z*z + c,     0,
      0,             0,             0,             1,
    ]);
  }

  function mat4Origin(xyz, rpy) {
    return mat4Mul(mat4Translate(xyz[0], xyz[1], xyz[2]), mat4FromRpy(rpy[0], rpy[1], rpy[2]));
  }

  function mat4Perspective(fovyRad, aspect, near, far) {
    const f = 1.0 / Math.tan(fovyRad / 2);
    const nf = 1.0 / (near - far);
    return new Float32Array([
      f / aspect, 0, 0, 0,
      0, f, 0, 0,
      0, 0, (far + near) * nf, -1,
      0, 0, 2 * far * near * nf, 0,
    ]);
  }

  function mat4LookAt(eye, target, up) {
    let zx = eye[0] - target[0], zy = eye[1] - target[1], zz = eye[2] - target[2];
    const zlen = Math.hypot(zx, zy, zz) || 1;
    zx /= zlen; zy /= zlen; zz /= zlen;
    let xx = up[1]*zz - up[2]*zy, xy = up[2]*zx - up[0]*zz, xz = up[0]*zy - up[1]*zx;
    const xlen = Math.hypot(xx, xy, xz) || 1;
    xx /= xlen; xy /= xlen; xz /= xlen;
    const yx = zy*xz - zz*xy, yy = zz*xx - zx*xz, yz = zx*xy - zy*xx;
    return new Float32Array([
      xx, yx, zx, 0,
      xy, yy, zy, 0,
      xz, yz, zz, 0,
      -(xx*eye[0] + xy*eye[1] + xz*eye[2]),
      -(yx*eye[0] + yy*eye[1] + yz*eye[2]),
      -(zx*eye[0] + zy*eye[1] + zz*eye[2]),
      1,
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
    if (84 + count * 50 !== buf.byteLength) {
      return parseAsciiStl(new TextDecoder().decode(buf));
    }
    const pos = new Float32Array(count * 9);
    const nrm = new Float32Array(count * 9);
    let off = 84;
    for (let i = 0; i < count; i++) {
      let nx = dv.getFloat32(off, true);
      let ny = dv.getFloat32(off + 4, true);
      let nz = dv.getFloat32(off + 8, true);
      const ax = dv.getFloat32(off + 12, true), ay = dv.getFloat32(off + 16, true), az = dv.getFloat32(off + 20, true);
      const bx = dv.getFloat32(off + 24, true), by = dv.getFloat32(off + 28, true), bz = dv.getFloat32(off + 32, true);
      const cx = dv.getFloat32(off + 36, true), cy = dv.getFloat32(off + 40, true), cz = dv.getFloat32(off + 44, true);
      off += 50;
      if (Math.hypot(nx, ny, nz) < 1e-6) {
        const ux = bx - ax, uy = by - ay, uz = bz - az;
        const vx = cx - ax, vy = cy - ay, vz = cz - az;
        nx = uy * vz - uz * vy;
        ny = uz * vx - ux * vz;
        nz = ux * vy - uy * vx;
        const nl = Math.hypot(nx, ny, nz) || 1;
        nx /= nl; ny /= nl; nz /= nl;
      }
      const p = i * 9;
      pos[p] = ax; pos[p+1] = ay; pos[p+2] = az;
      pos[p+3] = bx; pos[p+4] = by; pos[p+5] = bz;
      pos[p+6] = cx; pos[p+7] = cy; pos[p+8] = cz;
      for (let k = 0; k < 3; k++) {
        nrm[p + k*3] = nx; nrm[p + k*3 + 1] = ny; nrm[p + k*3 + 2] = nz;
      }
    }
    return { pos, nrm, vertexCount: count * 3 };
  }

  function parseAsciiStl(txt) {
    const verts = [];
    const norms = [];
    const re = /facet\s+normal\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)[\s\S]*?vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)[\s\S]*?vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)[\s\S]*?vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)/g;
    let m;
    while ((m = re.exec(txt)) !== null) {
      const nx = +m[1], ny = +m[2], nz = +m[3];
      for (let k = 0; k < 3; k++) {
        verts.push(+m[4 + k*3], +m[5 + k*3], +m[6 + k*3]);
        norms.push(nx, ny, nz);
      }
    }
    return { pos: new Float32Array(verts), nrm: new Float32Array(norms), vertexCount: verts.length / 3 };
  }

  function makeBoxMesh(sx, sy, sz) {
    const hx = sx/2, hy = sy/2, hz = sz/2;
    const faces = [
      [[0,0,1], [[-hx,-hy,hz],[hx,-hy,hz],[hx,hy,hz],[-hx,hy,hz]]],
      [[0,0,-1],[[-hx,hy,-hz],[hx,hy,-hz],[hx,-hy,-hz],[-hx,-hy,-hz]]],
      [[0,1,0], [[-hx,hy,-hz],[-hx,hy,hz],[hx,hy,hz],[hx,hy,-hz]]],
      [[0,-1,0],[[-hx,-hy,hz],[-hx,-hy,-hz],[hx,-hy,-hz],[hx,-hy,hz]]],
      [[1,0,0], [[hx,-hy,-hz],[hx,hy,-hz],[hx,hy,hz],[hx,-hy,hz]]],
      [[-1,0,0],[[-hx,-hy,hz],[-hx,hy,hz],[-hx,hy,-hz],[-hx,-hy,-hz]]],
    ];
    const pos = [], nrm = [];
    for (const [n, q] of faces) {
      for (const idx of [0,1,2, 0,2,3]) {
        pos.push(q[idx][0], q[idx][1], q[idx][2]);
        nrm.push(n[0], n[1], n[2]);
      }
    }
    return { pos: new Float32Array(pos), nrm: new Float32Array(nrm), vertexCount: 36 };
  }

  function makeCylinderMesh(r, h, segs) {
    const pos = [], nrm = [];
    const hz = h / 2;
    for (let i = 0; i < segs; i++) {
      const a0 = (i / segs) * Math.PI * 2;
      const a1 = ((i + 1) / segs) * Math.PI * 2;
      const c0 = Math.cos(a0), s0 = Math.sin(a0), c1 = Math.cos(a1), s1 = Math.sin(a1);
      // Side quad
      const p00 = [r*c0, r*s0, -hz], p10 = [r*c1, r*s1, -hz], p11 = [r*c1, r*s1, hz], p01 = [r*c0, r*s0, hz];
      const n0 = [c0, s0, 0], n1 = [c1, s1, 0];
      pos.push(...p00, ...p10, ...p11, ...p00, ...p11, ...p01);
      nrm.push(...n0, ...n1, ...n1, ...n0, ...n1, ...n0);
      // Top cap
      pos.push(0,0,hz, ...p01, ...p11);
      nrm.push(0,0,1, 0,0,1, 0,0,1);
    }
    return { pos: new Float32Array(pos), nrm: new Float32Array(nrm), vertexCount: pos.length / 3 };
  }

  class TraxURDFViewer {
    constructor(canvas, statusEl) {
      this.canvas = canvas;
      this.statusEl = statusEl;
      this.gl = canvas.getContext('webgl', { antialias: true, alpha: false });
      this.robot = null;
      this.meshBuffers = new Map();
      this.jointAngles = {};
      this.sceneObjects = [];
      this.sceneZones = {};
      this.surfelBuffer = null;
      this.surfelCount = 0;
      this.baseWorld = [0.13, 0.0, 1.0];
      this.azimuth = -0.95;
      this.elevation = 0.52;
      this.distance = 0.72;
      this.target = [0.28, 0.0, 0.10];
      this.dragging = false;
      this.lastX = 0;
      this.lastY = 0;
      this.rafId = 0;
      this.disposed = false;
      this.loadedKey = '';

      if (this.gl) {
        this._initGL();
        this._bindPointer();
        this._startLoop();
      } else if (this.statusEl) {
        this.statusEl.textContent = 'WebGL unavailable in this browser';
      }
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
        attribute vec3 aPos;
        attribute vec3 aNrm;
        uniform mat4 uProj;
        uniform mat4 uView;
        uniform mat4 uModel;
        varying vec3 vNrm;
        void main() {
          gl_Position = uProj * uView * uModel * vec4(aPos, 1.0);
          vNrm = normalize(mat3(uModel) * aNrm);
        }`;
      const fs = `
        precision mediump float;
        uniform vec4 uColor;
        varying vec3 vNrm;
        void main() {
          vec3 l1 = normalize(vec3(0.45, -0.55, 0.85));
          vec3 l2 = normalize(vec3(-0.6, 0.4, 0.5));
          float d1 = max(dot(normalize(vNrm), l1), 0.0);
          float d2 = max(dot(normalize(vNrm), l2), 0.0);
          vec3 col = uColor.rgb * (0.38 + 0.54 * d1 + 0.18 * d2);
          gl_FragColor = vec4(col, uColor.a);
        }`;
      const prog = gl.createProgram();
      gl.attachShader(prog, this._compile(gl.VERTEX_SHADER, vs));
      gl.attachShader(prog, this._compile(gl.FRAGMENT_SHADER, fs));
      gl.linkProgram(prog);
      this.prog = prog;
      this.loc = {
        aPos: gl.getAttribLocation(prog, 'aPos'),
        aNrm: gl.getAttribLocation(prog, 'aNrm'),
        uProj: gl.getUniformLocation(prog, 'uProj'),
        uView: gl.getUniformLocation(prog, 'uView'),
        uModel: gl.getUniformLocation(prog, 'uModel'),
        uColor: gl.getUniformLocation(prog, 'uColor'),
      };

      const pvs = `
        attribute vec3 aPos;
        attribute vec3 aCol;
        uniform mat4 uProj;
        uniform mat4 uView;
        uniform float uPointSize;
        varying vec3 vCol;
        void main() {
          gl_Position = uProj * uView * vec4(aPos, 1.0);
          gl_PointSize = uPointSize;
          vCol = aCol;
        }`;
      const pfs = `
        precision mediump float;
        varying vec3 vCol;
        void main() { gl_FragColor = vec4(vCol, 1.0); }`;
      const pprog = gl.createProgram();
      gl.attachShader(pprog, this._compile(gl.VERTEX_SHADER, pvs));
      gl.attachShader(pprog, this._compile(gl.FRAGMENT_SHADER, pfs));
      gl.linkProgram(pprog);
      this.pprog = pprog;
      this.ploc = {
        aPos: gl.getAttribLocation(pprog, 'aPos'),
        aCol: gl.getAttribLocation(pprog, 'aCol'),
        uProj: gl.getUniformLocation(pprog, 'uProj'),
        uView: gl.getUniformLocation(pprog, 'uView'),
        uPointSize: gl.getUniformLocation(pprog, 'uPointSize'),
      };

      // Build tabletop grid lines
      const gPos = [], gCol = [];
      for (let i = -5; i <= 5; i++) {
        const v = i * 0.08;
        const c = i === 0 ? 0.52 : 0.78;
        gPos.push(-0.2, v, 0,  0.6, v, 0);
        gCol.push(c, c, c,  c, c, c);
        const x = 0.2 + i * 0.08;
        gPos.push(x, -0.4, 0,  x, 0.4, 0);
        gCol.push(c, c, c,  c, c, c);
      }
      // XYZ origin axes (10cm)
      gPos.push(0,0,0.002, 0.10,0,0.002,  0,0,0.002, 0,0.10,0.002,  0,0,0.002, 0,0,0.102);
      gCol.push(0.85,0.15,0.15, 0.85,0.15,0.15,  0.12,0.72,0.28, 0.12,0.72,0.28,  0.15,0.42,0.92, 0.15,0.42,0.92);
      this.gridBuf = this._uploadPoints(new Float32Array(gPos), new Float32Array(gCol));
      this.gridCount = gPos.length / 3;

      this.cylMesh = this._uploadMesh(makeCylinderMesh(0.026, 0.072, 20));
      this.zoneMesh = this._uploadMesh(makeCylinderMesh(0.042, 0.004, 24));
      this.boxMesh = this._uploadMesh(makeBoxMesh(0.045, 0.045, 0.045));
    }

    _uploadMesh(m) {
      const gl = this.gl;
      const pb = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, pb);
      gl.bufferData(gl.ARRAY_BUFFER, m.pos, gl.STATIC_DRAW);
      const nb = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, nb);
      gl.bufferData(gl.ARRAY_BUFFER, m.nrm, gl.STATIC_DRAW);
      return { posBuf: pb, nrmBuf: nb, count: m.vertexCount };
    }

    _uploadPoints(posArr, colArr) {
      const gl = this.gl;
      const pb = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, pb);
      gl.bufferData(gl.ARRAY_BUFFER, posArr, gl.STATIC_DRAW);
      const cb = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, cb);
      gl.bufferData(gl.ARRAY_BUFFER, colArr, gl.STATIC_DRAW);
      return { posBuf: pb, colBuf: cb };
    }

    _bindPointer() {
      this.canvas.addEventListener('pointerdown', (e) => {
        this.dragging = true;
        this.lastX = e.clientX;
        this.lastY = e.clientY;
        this.canvas.setPointerCapture(e.pointerId);
      });
      this.canvas.addEventListener('pointermove', (e) => {
        if (!this.dragging) return;
        const dx = e.clientX - this.lastX;
        const dy = e.clientY - this.lastY;
        this.lastX = e.clientX;
        this.lastY = e.clientY;
        this.azimuth -= dx * 0.008;
        this.elevation = Math.max(0.08, Math.min(1.48, this.elevation + dy * 0.007));
      });
      this.canvas.addEventListener('pointerup', () => { this.dragging = false; });
      this.canvas.addEventListener('wheel', (e) => {
        e.preventDefault();
        this.distance = Math.max(0.28, Math.min(1.8, this.distance * (1 + e.deltaY * 0.001)));
      }, { passive: false });
      this.canvas.addEventListener('dblclick', () => {
        this.azimuth = -0.95; this.elevation = 0.52; this.distance = 0.72;
      });
    }

    async loadUrdf(url) {
      if (!this.gl) return;
      if (this.statusEl) this.statusEl.textContent = 'Loading URDF & STL meshes...';
      try {
        const r = await fetch(url);
        if (!r.ok) {
          if (this.statusEl) this.statusEl.textContent = 'Active embodiment exposes no URDF resource';
          this.robot = null;
          return;
        }
        const xmlText = await r.text();
        const doc = new DOMParser().parseFromString(xmlText, 'application/xml');
        const robotEl = doc.querySelector('robot');
        if (!robotEl) throw new Error('Invalid URDF XML');

        const materials = {};
        robotEl.querySelectorAll(':scope > material').forEach((m) => {
          const name = m.getAttribute('name');
          const c = m.querySelector('color');
          if (name && c) materials[name] = parseFloats(c.getAttribute('rgba'), [0.75, 0.78, 0.82, 1]);
        });

        const links = {};
        const meshUrls = new Set();
        robotEl.querySelectorAll(':scope > link').forEach((l) => {
          const name = l.getAttribute('name');
          const visuals = [];
          l.querySelectorAll(':scope > visual').forEach((v) => {
            const orig = v.querySelector('origin');
            const xyz = parseFloats(orig && orig.getAttribute('xyz'), [0, 0, 0]);
            const rpy = parseFloats(orig && orig.getAttribute('rpy'), [0, 0, 0]);
            const meshEl = v.querySelector('geometry > mesh');
            if (!meshEl) return;
            const filename = meshEl.getAttribute('filename') || '';
            const scale = parseFloats(meshEl.getAttribute('scale'), [1, 1, 1]);
            const matEl = v.querySelector('material');
            const matName = matEl ? matEl.getAttribute('name') : '';
            const colorEl = matEl ? matEl.querySelector('color') : null;
            const rgba = colorEl
              ? parseFloats(colorEl.getAttribute('rgba'), [0.78, 0.80, 0.84, 1])
              : (materials[matName] || [0.78, 0.80, 0.84, 1]);
            if (filename) meshUrls.add(filename);
            visuals.push({ filename, scale, origin: mat4Origin(xyz, rpy), rgba });
          });
          links[name] = { name, visuals };
        });

        const joints = [];
        const childSet = new Set();
        robotEl.querySelectorAll(':scope > joint').forEach((j) => {
          const name = j.getAttribute('name') || '';
          const type = j.getAttribute('type') || 'fixed';
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
        for (const j of joints) {
          (childrenByParent[j.parent] = childrenByParent[j.parent] || []).push(j);
        }

        let loadedMeshes = 0;
        await Promise.all([...meshUrls].map(async (u) => {
          if (this.meshBuffers.has(u)) { loadedMeshes++; return; }
          try {
            const mr = await fetch(u);
            if (!mr.ok) return;
            const parsed = parseBinaryStl(await mr.arrayBuffer());
            if (parsed && parsed.vertexCount > 0) {
              this.meshBuffers.set(u, this._uploadMesh(parsed));
              loadedMeshes++;
            }
          } catch (_) {}
        }));

        this.robot = { name: robotEl.getAttribute('name') || 'robot', links, childrenByParent, rootLink };
        if (this.statusEl) {
          this.statusEl.textContent = `URDF: ${this.robot.name} · ${Object.keys(links).length} links · ${loadedMeshes} STL meshes (drag to orbit, scroll to zoom)`;
        }
      } catch (e) {
        if (this.statusEl) this.statusEl.textContent = `URDF load error: ${e.message}`;
      }
    }

    setTelemetry(mcpTel) {
      if (!mcpTel) return;
      const robot = mcpTel.robot;
      if (robot) {
        if (Array.isArray(robot.base_pos_m) && robot.base_pos_m.length === 3) {
          this.baseWorld = robot.base_pos_m.map(Number);
        }
        const names = robot.joint_names || [];
        const vals = robot.joint_pos_rad || [];
        for (let i = 0; i < names.length; i++) {
          this.jointAngles[names[i]] = Number(vals[i] || 0);
        }
        if (robot.gripper_open_ratio != null) {
          this.jointAngles['gripper'] = 0.10 + Number(robot.gripper_open_ratio) * (0.88 - 0.10);
        }
      }
      const scene = mcpTel.scene;
      if (scene) {
        this.sceneObjects = scene.objects || [];
        this.sceneZones = scene.zones || {};
      }
    }

    /* Standard hook for Step B EvidenceScene 3D reconstructed surfels / point clouds */
    setSceneSurfels(points) {
      if (!this.gl || !Array.isArray(points) || !points.length) {
        this.surfelCount = 0;
        return;
      }
      const pos = new Float32Array(points.length * 3);
      const col = new Float32Array(points.length * 3);
      for (let i = 0; i < points.length; i++) {
        const pt = points[i];
        pos[i*3]   = Number(pt[0]) - this.baseWorld[0];
        pos[i*3+1] = Number(pt[1]) - this.baseWorld[1];
        pos[i*3+2] = Number(pt[2]) - this.baseWorld[2];
        col[i*3]   = pt[3] != null ? Number(pt[3]) : 0.08;
        col[i*3+1] = pt[4] != null ? Number(pt[4]) : 0.72;
        col[i*3+2] = pt[5] != null ? Number(pt[5]) : 0.92;
      }
      this.surfelBuffer = this._uploadPoints(pos, col);
      this.surfelCount = points.length;
    }

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

    _renderFrame() {
      if (this.disposed || !this.gl || !this.canvas.isConnected) return;
      const gl = this.gl;
      const dpr = Math.min(window.devicePixelRatio || 1, 2);
      const w = Math.max(320, Math.floor(this.canvas.clientWidth * dpr));
      const h = Math.max(180, Math.floor(this.canvas.clientHeight * dpr));
      if (this.canvas.width !== w || this.canvas.height !== h) {
        this.canvas.width = w; this.canvas.height = h;
      }
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

      // 1. Draw tabletop grid & axes
      gl.useProgram(this.pprog);
      gl.uniformMatrix4fv(this.ploc.uProj, false, proj);
      gl.uniformMatrix4fv(this.ploc.uView, false, view);
      gl.uniform1f(this.ploc.uPointSize, 3.0);
      gl.bindBuffer(gl.ARRAY_BUFFER, this.gridBuf.posBuf);
      gl.enableVertexAttribArray(this.ploc.aPos);
      gl.vertexAttribPointer(this.ploc.aPos, 3, gl.FLOAT, false, 0, 0);
      gl.bindBuffer(gl.ARRAY_BUFFER, this.gridBuf.colBuf);
      gl.enableVertexAttribArray(this.ploc.aCol);
      gl.vertexAttribPointer(this.ploc.aCol, 3, gl.FLOAT, false, 0, 0);
      gl.drawArrays(gl.LINES, 0, this.gridCount);

      // 2. Draw EvidenceScene reconstructed surfels if attached
      if (this.surfelBuffer && this.surfelCount > 0) {
        gl.uniform1f(this.ploc.uPointSize, 4.0);
        gl.bindBuffer(gl.ARRAY_BUFFER, this.surfelBuffer.posBuf);
        gl.vertexAttribPointer(this.ploc.aPos, 3, gl.FLOAT, false, 0, 0);
        gl.bindBuffer(gl.ARRAY_BUFFER, this.surfelBuffer.colBuf);
        gl.vertexAttribPointer(this.ploc.aCol, 3, gl.FLOAT, false, 0, 0);
        gl.drawArrays(gl.POINTS, 0, this.surfelCount);
      }

      // 3. Draw URDF links + live scene objects & target zones
      gl.useProgram(this.prog);
      gl.uniformMatrix4fv(this.loc.uProj, false, proj);
      gl.uniformMatrix4fv(this.loc.uView, false, view);

      const zoneColors = {
        target_zone_a: [0.10, 0.72, 0.92, 0.85],
        target_zone_b: [0.95, 0.68, 0.14, 0.85],
        station_pad:   [0.16, 0.78, 0.45, 0.85],
      };
      for (const [zname, xyz] of Object.entries(this.sceneZones || {})) {
        const m = mat4Translate(xyz[0] - this.baseWorld[0], xyz[1] - this.baseWorld[1], 0.002);
        this._drawMesh(this.zoneMesh, m, zoneColors[zname] || [0.5, 0.6, 0.7, 0.8]);
      }
      for (const obj of this.sceneObjects || []) {
        const wpos = obj.world_m || [0, 0, 0];
        const m = mat4Translate(wpos[0] - this.baseWorld[0], wpos[1] - this.baseWorld[1], wpos[2] - this.baseWorld[2]);
        const isRed = String(obj.id || '').includes('red');
        this._drawMesh(isRed ? this.cylMesh : this.boxMesh, m, isRed ? [0.90, 0.18, 0.22, 1] : [0.68, 0.52, 0.36, 1]);
      }

      if (this.robot && this.robot.rootLink) {
        const drawLink = (linkName, worldMat) => {
          const link = this.robot.links[linkName];
          if (link) {
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
            }
            drawLink(j.child, childMat);
          }
        };
        drawLink(this.robot.rootLink, mat4Identity());
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

    dispose() {
      this.disposed = true;
      if (this.rafId) cancelAnimationFrame(this.rafId);
    }
  }

  window.TraxURDFViewer = TraxURDFViewer;
})();
