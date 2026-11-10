/* EchoMap PLY viewer: binary little-endian PLY with float x,y,z + uchar r,g,b, drawn as WebGL points. */
(function () {
  'use strict';
  const $ = id => document.getElementById(id);
  const cv = $('view');
  const gl = cv.getContext('webgl', { antialias: true, alpha: false });
  const st = { n: 0, yaw: 0.7, pitch: 0.75, dist: 1, center: [0, 0, 0], extent: 1, zmin: 0, zmax: 1, top: false, cut: false, height: true };
  let prog, loc = {}, bufPos, bufCol;

  function warn(m) { $('warn').hidden = !m; $('warn').textContent = m || ''; }

  function parsePly(buf) {
    const bytes = new Uint8Array(buf);
    const headEnd = (() => { const key = 'end_header\n'; for (let i = 0; i < Math.min(bytes.length, 4096); i++) { let ok = true; for (let j = 0; j < key.length; j++) if (bytes[i + j] !== key.charCodeAt(j)) { ok = false; break; } if (ok) return i + key.length; } return -1; })();
    if (headEnd < 0) throw new Error('Not a PLY file.');
    const head = new TextDecoder().decode(bytes.subarray(0, headEnd));
    if (!/format binary_little_endian/.test(head)) throw new Error('Only binary little-endian PLY files are supported.');
    const n = parseInt((/element vertex (\d+)/.exec(head) || [])[1], 10);
    const props = [...head.matchAll(/property (\w+) (\w+)/g)].map(m => m[1] + ' ' + m[2]).join(',');
    if (props !== 'float x,float y,float z,uchar red,uchar green,uchar blue') throw new Error('Unsupported PLY layout: expected x y z (float) and red green blue (uchar).');
    if (headEnd + n * 15 > bytes.length) throw new Error('The PLY file is cut short.');
    const pos = new Float32Array(n * 3), col = new Uint8Array(n * 3), dv = new DataView(buf, headEnd);
    for (let i = 0, o = 0; i < n; i++, o += 15) {
      pos[i * 3] = dv.getFloat32(o, true); pos[i * 3 + 1] = dv.getFloat32(o + 4, true); pos[i * 3 + 2] = dv.getFloat32(o + 8, true);
      col[i * 3] = bytes[headEnd + o + 12]; col[i * 3 + 1] = bytes[headEnd + o + 13]; col[i * 3 + 2] = bytes[headEnd + o + 14];
    }
    return { n, pos, col };
  }

  function initGL() {
    const sh = (type, src) => { const s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s); if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s)); return s; };
    prog = gl.createProgram();
    gl.attachShader(prog, sh(gl.VERTEX_SHADER, `
      attribute vec3 aPos; attribute vec3 aCol;
      uniform mat4 uMvp; uniform float uSize, uMode, uZ0, uZ1, uCut;
      varying vec3 vCol;
      vec3 ramp(float t) {
        vec3 a = vec3(0.10, 0.20, 0.75), b = vec3(0.10, 0.80, 0.55), c = vec3(0.98, 0.85, 0.25);
        return t < 0.5 ? mix(a, b, t * 2.0) : mix(b, c, t * 2.0 - 1.0);
      }
      void main() {
        gl_Position = uMvp * vec4(aPos, 1.0);
        if (aPos.z > uCut) gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        gl_PointSize = clamp(uSize / gl_Position.w, 1.0, 12.0);
        vCol = uMode > 0.5 ? ramp(clamp((aPos.z - uZ0) / (uZ1 - uZ0), 0.0, 1.0)) : aCol;
      }`));
    gl.attachShader(prog, sh(gl.FRAGMENT_SHADER, 'precision mediump float; varying vec3 vCol; void main() { gl_FragColor = vec4(vCol, 1.0); }'));
    gl.linkProgram(prog);
    for (const n of ['uMvp', 'uSize', 'uMode', 'uZ0', 'uZ1', 'uCut']) loc[n] = gl.getUniformLocation(prog, n);
    loc.aPos = gl.getAttribLocation(prog, 'aPos'); loc.aCol = gl.getAttribLocation(prog, 'aCol');
    bufPos = gl.createBuffer(); bufCol = gl.createBuffer();
    gl.enable(gl.DEPTH_TEST);
  }

  function show(cloud, name) {
    let lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    for (let i = 0; i < cloud.n; i++) for (let k = 0; k < 3; k++) { const v = cloud.pos[i * 3 + k]; if (v < lo[k]) lo[k] = v; if (v > hi[k]) hi[k] = v; }
    st.n = cloud.n; st.center = [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2];
    st.extent = Math.max(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]) || 1; st.zmin = lo[2]; st.zmax = hi[2]; st.dist = 1.1;
    gl.bindBuffer(gl.ARRAY_BUFFER, bufPos); gl.bufferData(gl.ARRAY_BUFFER, cloud.pos, gl.STATIC_DRAW);
    gl.bindBuffer(gl.ARRAY_BUFFER, bufCol); gl.bufferData(gl.ARRAY_BUFFER, cloud.col, gl.STATIC_DRAW);
    $('info').textContent = `${cloud.n.toLocaleString()} points · ${(hi[0] - lo[0]).toFixed(1)} × ${(hi[1] - lo[1]).toFixed(1)} × ${(hi[2] - lo[2]).toFixed(1)} m`;
    $('status').textContent = name;
    requestDraw();
  }

  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const norm = a => { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
  function mul(a, b) { const o = new Float32Array(16); for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) { let s = 0; for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k]; o[c * 4 + r] = s; } return o; }
  function perspective(fovy, aspect, n, f) { const t = 1 / Math.tan(fovy / 2); return new Float32Array([t / aspect, 0, 0, 0, 0, t, 0, 0, 0, 0, (f + n) / (n - f), -1, 0, 0, 2 * f * n / (n - f), 0]); }
  function ortho(hw, hh, n, f) { return new Float32Array([1 / hw, 0, 0, 0, 0, 1 / hh, 0, 0, 0, 0, -2 / (f - n), 0, 0, 0, -(f + n) / (f - n), 1]); }
  function lookAt(e, c, up) {
    const z = norm([e[0] - c[0], e[1] - c[1], e[2] - c[2]]), x = norm(cross(up, z)), y = cross(z, x);
    return new Float32Array([x[0], y[0], z[0], 0, x[1], y[1], z[1], 0, x[2], y[2], z[2], 0, -dot(x, e), -dot(y, e), -dot(z, e), 1]);
  }

  let queued = false;
  function requestDraw() { if (!queued) { queued = true; requestAnimationFrame(draw); } }
  function draw() {
    queued = false;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = Math.round(cv.clientWidth * dpr), h = Math.round(cv.clientHeight * dpr);
    if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
    gl.viewport(0, 0, w, h);
    gl.clearColor(0.027, 0.043, 0.051, 1); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    if (!st.n) return;
    const c = st.center, d = st.dist * st.extent;
    let mvp, size;
    if (st.top) {
      const hw = d * 0.55 * Math.max(1, w / h), hh = hw * h / w;
      const up = [-Math.sin(st.yaw), Math.cos(st.yaw), 0];
      mvp = mul(ortho(hw, hh, 0.01, 4 * st.extent), lookAt([c[0], c[1], c[2] + 2 * st.extent], c, up));
      size = 2.2 * dpr;
    } else {
      const eye = [c[0] + d * Math.cos(st.pitch) * Math.sin(st.yaw), c[1] - d * Math.cos(st.pitch) * Math.cos(st.yaw), c[2] + d * Math.sin(st.pitch)];
      mvp = mul(perspective(50 * Math.PI / 180, w / h, 0.02 * st.extent, 10 * st.extent), lookAt(eye, c, [0, 0, 1]));
      size = 0.012 * st.extent * h / 2;     // roughly 2.5 cm points at the map's scale
    }
    gl.useProgram(prog);
    gl.uniformMatrix4fv(loc.uMvp, false, mvp);
    gl.uniform1f(loc.uSize, size); gl.uniform1f(loc.uMode, st.height ? 1 : 0);
    gl.uniform1f(loc.uZ0, st.zmin); gl.uniform1f(loc.uZ1, st.zmax);
    gl.uniform1f(loc.uCut, st.cut ? st.zmin + 0.8 * (st.zmax - st.zmin) : 1e9);
    gl.bindBuffer(gl.ARRAY_BUFFER, bufPos); gl.enableVertexAttribArray(loc.aPos); gl.vertexAttribPointer(loc.aPos, 3, gl.FLOAT, false, 0, 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, bufCol); gl.enableVertexAttribArray(loc.aCol); gl.vertexAttribPointer(loc.aCol, 3, gl.UNSIGNED_BYTE, true, 0, 0);
    gl.drawArrays(gl.POINTS, 0, st.n);
  }

  const ptrs = new Map(); let pinch0 = 0;
  cv.addEventListener('pointerdown', e => { cv.setPointerCapture(e.pointerId); ptrs.set(e.pointerId, [e.clientX, e.clientY]); pinch0 = 0; });
  cv.addEventListener('pointermove', e => {
    if (!ptrs.has(e.pointerId)) return;
    const prev = ptrs.get(e.pointerId); ptrs.set(e.pointerId, [e.clientX, e.clientY]);
    if (ptrs.size === 1) { st.yaw -= (e.clientX - prev[0]) * 0.008; st.pitch = Math.max(-1.5, Math.min(1.5, st.pitch + (e.clientY - prev[1]) * 0.008)); }
    else if (ptrs.size === 2) { const [a, b] = [...ptrs.values()]; const dd = Math.hypot(a[0] - b[0], a[1] - b[1]); if (pinch0) st.dist = Math.max(0.05, Math.min(5, st.dist * pinch0 / dd)); pinch0 = dd; }
    requestDraw();
  });
  for (const ev of ['pointerup', 'pointercancel']) cv.addEventListener(ev, e => { ptrs.delete(e.pointerId); pinch0 = 0; });
  cv.addEventListener('wheel', e => { e.preventDefault(); st.dist = Math.max(0.05, Math.min(5, st.dist * Math.exp(e.deltaY * 0.001))); requestDraw(); }, { passive: false });
  window.addEventListener('resize', requestDraw);

  const toggle = (id, key) => $(id).addEventListener('click', () => { st[key] = !st[key]; $(id).setAttribute('aria-pressed', String(st[key])); requestDraw(); });
  toggle('mHeight', 'height'); toggle('mTop', 'top'); toggle('mCut', 'cut');

  $('file').addEventListener('change', async e => {
    const f = e.target.files[0]; if (!f) return;
    try { warn(''); show(parsePly(await f.arrayBuffer()), f.name); $('report').hidden = true; $('caption').hidden = true; }
    catch (err) { warn(err.message); }
  });

  function renderReport(r) {
    const items = [['Map error, median', r.map_error_cm.median + ' cm'], ['Map error, 99% within', r.map_error_cm.p99 + ' cm'],
      ['Path error (RMS)', r.path_error_cm.rmse + ' cm'], ['Start-to-end drift', r.start_to_end_drift_cm + ' cm'], ['Walk', r.path_length_m + ' m in ' + r.seconds + ' s']];
    const dl = $('report'); dl.innerHTML = '';
    for (const [k, v] of items) { const d = document.createElement('div'), dt = document.createElement('dt'), dd = document.createElement('dd'); dt.textContent = k; dd.textContent = v; d.append(dt, dd); dl.append(d); }
    dl.hidden = false;
  }

  (async () => {
    try {
      if (!gl) throw new Error('This browser cannot draw 3D (WebGL is unavailable).');
      initGL();
      const file = new URLSearchParams(location.search).get('file') || 'livox/sample/sim-map.ply';
      const res = await fetch(file);
      if (!res.ok) throw new Error('Could not load ' + file);
      show(parsePly(await res.arrayBuffer()), 'Sample: simulated Livox walk');
      if (file === 'livox/sample/sim-map.ply') { const r = await fetch('livox/sample/report.json'); if (r.ok) renderReport(await r.json()); }
    } catch (err) { $('status').textContent = 'Error'; warn(err.message); }
  })();
  window.__viewer = st;
})();
