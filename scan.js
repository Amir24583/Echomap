/* EchoMap 3D scan: camera frames -> AI depth -> gyroscope-registered point cloud, drawn with WebGL. */
(function () {
  'use strict';
  const G = window.ScanGeom;
  const $ = id => document.getElementById(id);

  const MODEL = 'onnx-community/depth-anything-v2-small';
  const LIB = 'https://cdn.jsdelivr.net/npm/@huggingface/transformers@3.7.6';
  const CAPACITY = 1200000;          // max points
  const FRAME_LONG = 448;            // long side of the frame sent to the depth model
  const NEW_VIEW_DEG = 12;           // capture when facing this far from every captured view
  const MAX_SPEED = 35;              // deg/s; faster than this the frame would be blurred / mis-registered

  const st = {
    estimator: null, RawImage: null, running: false, busy: false, demo: false,
    R: null, lastR: null, lastT: 0, speed: 0, orientationOk: false,
    views: [], pano: G.createPano(3),
    pos: new Float32Array(CAPACITY * 3), col: new Uint8Array(CAPACITY * 3), count: 0, uploaded: 0,
    yaw: 0.6, pitch: 0.55, dist: 3.2, frames: 0,
  };
  const cap = document.createElement('canvas');
  const capCtx = cap.getContext('2d', { willReadFrequently: true });

  function setStatus(t, cls) { const s = $('status'); s.textContent = t; s.className = 'status' + (cls ? ' ' + cls : ''); }
  function warn(m) { const w = $('warn'); w.hidden = !m; w.textContent = m || ''; }
  function coach(m) { $('coach').textContent = m; }

  // ---------- WebGL point cloud ----------
  const cv = $('view');
  const gl = cv.getContext('webgl', { antialias: true, alpha: false });
  let prog, buf = {}, loc = {};
  function initGL() {
    if (!gl) { warn('This browser cannot draw 3D (WebGL is unavailable).'); return; }
    const sh = (type, src) => { const s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s); if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s)); return s; };
    prog = gl.createProgram();
    gl.attachShader(prog, sh(gl.VERTEX_SHADER, `
      attribute vec3 aPos; attribute vec3 aCol;
      uniform mat4 uMvp; uniform float uScale, uSize, uMode, uZ0, uZ1;
      varying vec3 vCol;
      vec3 ramp(float t) {
        vec3 a = vec3(0.10, 0.20, 0.75), b = vec3(0.10, 0.80, 0.55), c = vec3(0.98, 0.85, 0.25);
        return t < 0.5 ? mix(a, b, t * 2.0) : mix(b, c, t * 2.0 - 2.0 + 1.0);
      }
      void main() {
        vec3 p = aPos * uScale;
        gl_Position = uMvp * vec4(p, 1.0);
        gl_PointSize = clamp(uSize * 3.0 / gl_Position.w, 1.0, 24.0);
        vCol = uMode > 0.5 ? ramp(clamp((p.z - uZ0) / (uZ1 - uZ0), 0.0, 1.0)) : aCol;
      }`));
    gl.attachShader(prog, sh(gl.FRAGMENT_SHADER, `precision mediump float; varying vec3 vCol; void main() { gl_FragColor = vec4(vCol, 1.0); }`));
    gl.linkProgram(prog);
    if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(prog));
    for (const n of ['uMvp', 'uScale', 'uSize', 'uMode', 'uZ0', 'uZ1']) loc[n] = gl.getUniformLocation(prog, n);
    loc.aPos = gl.getAttribLocation(prog, 'aPos'); loc.aCol = gl.getAttribLocation(prog, 'aCol');
    buf.pos = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, buf.pos); gl.bufferData(gl.ARRAY_BUFFER, st.pos.byteLength, gl.DYNAMIC_DRAW);
    buf.col = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, buf.col); gl.bufferData(gl.ARRAY_BUFFER, st.col.byteLength, gl.DYNAMIC_DRAW);
    // viewer marker + compass ring (drawn as extra points at the origin plane)
    const ring = [];
    for (let i = 0; i < 72; i++) { const a = i / 72 * Math.PI * 2; ring.push(Math.cos(a), Math.sin(a), 0); }
    ring.push(0, 0, 0);
    buf.ring = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, buf.ring); gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(ring), gl.STATIC_DRAW);
    buf.ringN = ring.length / 3;
    gl.enable(gl.DEPTH_TEST);
  }

  function mul(a, b) { const o = new Float32Array(16); for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) { let s = 0; for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k]; o[c * 4 + r] = s; } return o; }
  function perspective(fovy, aspect, n, f) { const t = 1 / Math.tan(fovy / 2); return new Float32Array([t / aspect, 0, 0, 0, 0, t, 0, 0, 0, 0, (f + n) / (n - f), -1, 0, 0, 2 * f * n / (n - f), 0]); }
  function lookAt(e, c, up) {
    const z = norm([e[0] - c[0], e[1] - c[1], e[2] - c[2]]), x = norm(cross(up, z)), y = cross(z, x);
    return new Float32Array([x[0], y[0], z[0], 0, x[1], y[1], z[1], 0, x[2], y[2], z[2], 0, -dot(x, e), -dot(y, e), -dot(z, e), 1]);
  }
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const norm = a => { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };

  let drawQueued = false;
  function requestDraw() { if (!drawQueued) { drawQueued = true; requestAnimationFrame(draw); } }
  function draw() {
    drawQueued = false;
    if (!gl || !prog) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = Math.round(cv.clientWidth * dpr), h = Math.round(cv.clientHeight * dpr);
    if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
    gl.viewport(0, 0, w, h);
    gl.clearColor(0.027, 0.043, 0.051, 1); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    const scale = parseFloat($('scale').value) || 2.5;
    const d = st.dist * scale;
    const eye = [d * Math.cos(st.pitch) * Math.sin(st.yaw), -d * Math.cos(st.pitch) * Math.cos(st.yaw), d * Math.sin(st.pitch)];
    const mvp = mul(perspective(55 * Math.PI / 180, w / h, 0.05 * scale, 60 * scale), lookAt(eye, [0, 0, 0], [0, 0, 1]));
    gl.useProgram(prog);
    gl.uniformMatrix4fv(loc.uMvp, false, mvp);
    gl.uniform1f(loc.uSize, (parseFloat($('psize').value) || 3) * dpr);
    gl.uniform1f(loc.uZ0, -1.4); gl.uniform1f(loc.uZ1, 1.4);
    // upload new points
    if (st.uploaded < st.count) {
      gl.bindBuffer(gl.ARRAY_BUFFER, buf.pos); gl.bufferSubData(gl.ARRAY_BUFFER, st.uploaded * 12, st.pos.subarray(st.uploaded * 3, st.count * 3));
      gl.bindBuffer(gl.ARRAY_BUFFER, buf.col); gl.bufferSubData(gl.ARRAY_BUFFER, st.uploaded * 3, st.col.subarray(st.uploaded * 3, st.count * 3));
      st.uploaded = st.count;
    }
    // ring at 1 m and viewer marker
    gl.bindBuffer(gl.ARRAY_BUFFER, buf.ring);
    gl.enableVertexAttribArray(loc.aPos); gl.vertexAttribPointer(loc.aPos, 3, gl.FLOAT, false, 0, 0);
    gl.disableVertexAttribArray(loc.aCol); gl.vertexAttrib3f(loc.aCol, 0.37, 0.77, 0.85);
    gl.uniform1f(loc.uScale, 1); gl.uniform1f(loc.uMode, 0);
    gl.drawArrays(gl.POINTS, 0, buf.ringN);
    if (st.count) {
      gl.bindBuffer(gl.ARRAY_BUFFER, buf.pos); gl.vertexAttribPointer(loc.aPos, 3, gl.FLOAT, false, 0, 0);
      gl.bindBuffer(gl.ARRAY_BUFFER, buf.col); gl.enableVertexAttribArray(loc.aCol); gl.vertexAttribPointer(loc.aCol, 3, gl.UNSIGNED_BYTE, true, 0, 0);
      gl.uniform1f(loc.uScale, scale); gl.uniform1f(loc.uMode, parseFloat($('color').value));
      gl.drawArrays(gl.POINTS, 0, st.count);
    }
  }

  // orbit controls
  const ptrs = new Map(); let pinch0 = 0;
  cv.addEventListener('pointerdown', e => { cv.setPointerCapture(e.pointerId); ptrs.set(e.pointerId, [e.clientX, e.clientY]); pinch0 = 0; });
  cv.addEventListener('pointermove', e => {
    if (!ptrs.has(e.pointerId)) return;
    const prev = ptrs.get(e.pointerId); ptrs.set(e.pointerId, [e.clientX, e.clientY]);
    if (ptrs.size === 1) {
      st.yaw -= (e.clientX - prev[0]) * 0.008;
      st.pitch = Math.max(-1.5, Math.min(1.5, st.pitch + (e.clientY - prev[1]) * 0.008));
    } else if (ptrs.size === 2) {
      const [a, b] = [...ptrs.values()]; const dd = Math.hypot(a[0] - b[0], a[1] - b[1]);
      if (pinch0) st.dist = Math.max(0.2, Math.min(12, st.dist * pinch0 / dd));
      pinch0 = dd;
    }
    requestDraw();
  });
  for (const ev of ['pointerup', 'pointercancel']) cv.addEventListener(ev, e => { ptrs.delete(e.pointerId); pinch0 = 0; });
  cv.addEventListener('wheel', e => { e.preventDefault(); st.dist = Math.max(0.2, Math.min(12, st.dist * Math.exp(e.deltaY * 0.001))); requestDraw(); }, { passive: false });
  window.addEventListener('resize', requestDraw);
  for (const id of ['color', 'scale', 'psize']) $(id).addEventListener('input', requestDraw);

  // ---------- orientation ----------
  function onOrientation(alpha, beta, gamma) {
    const R = G.rotation(alpha, beta, gamma), now = performance.now();
    if (st.R) {
      const dt = (now - st.lastT) / 1000;
      if (dt > 0) st.speed = 0.7 * st.speed + 0.3 * (G.angleBetween(G.forward(st.R), G.forward(R)) / dt);
    }
    st.R = R; st.lastT = now; st.orientationOk = true;
    maybeCapture();
  }
  async function startOrientation() {
    if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
      const r = await DeviceOrientationEvent.requestPermission();
      if (r !== 'granted') throw new Error('Motion access was declined. The scan needs it to know which way the camera points. Reload and allow it.');
    }
    window.addEventListener('deviceorientation', e => { if (e.alpha != null && !st.demo) onOrientation(e.alpha, e.beta, e.gamma); });
  }

  // ---------- camera + model ----------
  async function startCamera() {
    if (!window.isSecureContext) throw new Error('This page must be opened over https:// for the camera to work.');
    const stream = await navigator.mediaDevices.getUserMedia({ audio: false, video: { facingMode: { ideal: 'environment' }, width: { ideal: 1280 }, height: { ideal: 720 } } });
    const v = $('cam'); v.srcObject = stream; v.hidden = false;
    await v.play();
  }

  async function loadModel() {
    setStatus('Loading model…');
    $('prog').hidden = false;
    const T = await import(LIB);
    st.RawImage = T.RawImage;
    const tries = [];
    if (navigator.gpu) {
      let f16 = false;
      try { const ad = await navigator.gpu.requestAdapter(); f16 = !!(ad && ad.features.has('shader-f16')); } catch (e) { /* fall through to wasm */ }
      tries.push({ device: 'webgpu', dtype: f16 ? 'fp16' : 'fp32' });
    }
    tries.push({ device: 'wasm', dtype: 'q8' });
    const files = {};
    const progress_callback = p => {
      if (p.status !== 'progress' || !p.total) return;
      files[p.file] = [p.loaded, p.total];
      let l = 0, t = 0; for (const k in files) { l += files[k][0]; t += files[k][1]; }
      $('prog').value = 100 * l / t;
      setStatus(`Downloading model ${Math.round(l / 1e6)} / ${Math.round(t / 1e6)} MB`);
    };
    let lastErr;
    for (const cfg of tries) {
      try {
        st.estimator = await T.pipeline('depth-estimation', MODEL, { ...cfg, progress_callback });
        st.backend = cfg.device;
        $('prog').hidden = true;
        return;
      } catch (e) { lastErr = e; console.warn('model load failed for', cfg, e); }
    }
    $('prog').hidden = true;
    throw new Error('The depth model could not be loaded: ' + (lastErr && lastErr.message || lastErr));
  }

  // returns { disp: Float32Array, w, h, rgba }
  async function estimate(R) {
    if (st.demo) return demoFrame(R);
    const v = $('cam');
    const s = FRAME_LONG / Math.max(v.videoWidth, v.videoHeight);
    const w = Math.round(v.videoWidth * s), h = Math.round(v.videoHeight * s);
    cap.width = w; cap.height = h;
    capCtx.drawImage(v, 0, 0, w, h);
    const rgba = capCtx.getImageData(0, 0, w, h).data;
    const out = await st.estimator(st.RawImage.fromCanvas(cap));
    const t = out.predicted_depth;
    const [th, tw] = t.dims.slice(-2);
    if (tw !== w || th !== h) throw new Error(`unexpected depth size ${tw}x${th} for ${w}x${h}`);
    return { disp: t.data, w, h, rgba };
  }

  function maybeCapture() {
    if (!st.running || st.busy || !st.R) return;
    if (st.count >= CAPACITY - 30000) { coach('Scan is full. Save it or reset.'); return; }
    const f = G.forward(st.R);
    let nearest = 180;
    for (const v of st.views) nearest = Math.min(nearest, G.angleBetween(f, v));
    if (nearest < NEW_VIEW_DEG) { coach(st.frames < 4 ? 'Keep turning slowly' : 'Turn to a part you have not scanned yet'); return; }
    if (st.speed > MAX_SPEED) { coach('Slow down a little'); return; }
    capture(st.R, f);
  }

  async function capture(R, f) {
    st.busy = true;
    coach('Hold still…');
    try {
      const t0 = performance.now();
      const fr = await estimate(R);
      const out = G.frameToPoints(fr.disp, fr.w, fr.h, R, fr.rgba, st.pano, {
        longFovDeg: parseFloat($('fov').value) || 65, stride: Math.max(2, Math.round(fr.w / 110)), edge: 0.12, minPairs: 150,
      });
      const n = Math.min(out.positions.length / 3, CAPACITY - st.count);
      st.pos.set(out.positions.subarray(0, n * 3), st.count * 3);
      st.col.set(out.colors.subarray(0, n * 3), st.count * 3);
      st.count += n; st.frames++; st.views.push(f);
      $('nFrames').textContent = st.frames; $('nPoints').textContent = st.count.toLocaleString();
      $('save').disabled = false; $('reset').disabled = false;
      setStatus(`Scanning · ${Math.round(performance.now() - t0)} ms/frame${st.backend ? ' · ' + st.backend : ''}`, 'live');
      coach('Keep turning slowly');
      requestDraw();
    } catch (e) {
      console.error(e);
      warn('A frame failed: ' + e.message);
      st.views.push(f); // do not retry the same direction forever
    } finally {
      st.busy = false;
    }
  }

  // ---------- demo room (no camera needed; also used by the automated check) ----------
  function demoRange(d) {
    let t = Infinity;
    const hit = (num, den) => { if (den > 1e-6) t = Math.min(t, num / den); };
    hit(2.2, d[0]); hit(1.6, -d[0]); hit(3.0, d[1]); hit(1.4, -d[1]); hit(1.4, -d[2]); hit(1.1, d[2]); // box room
    // a table: box 0.8..1.6 x, 0.6..1.6 y, top at z=-0.65
    if (d[2] < 0) { const tt = 0.65 / -d[2]; const x = d[0] * tt, y = d[1] * tt; if (x > 0.8 && x < 1.6 && y > 0.6 && y < 1.6) t = Math.min(t, tt); }
    return t;
  }
  function demoFrame(R) {
    const w = 168, h = 224, f = G.focalPx(w, h, parseFloat($('fov').value) || 65);
    const disp = new Float32Array(w * h), rgba = new Uint8ClampedArray(w * h * 4);
    const s = 0.6 + Math.random() * 2, sh = st.frames ? Math.random() * 0.05 : 0;
    for (let v = 0; v < h; v++) for (let u = 0; u < w; u++) {
      const ray = G.pixelRay(u, v, w, h, f), d = G.apply(R, ray), t = demoRange(d);
      disp[v * w + u] = s / t + sh;
      const p = [d[0] * t, d[1] * t, d[2] * t], i = (v * w + u) * 4;
      const floor = p[2] < -1.39, ceil = p[2] > 1.09, table = Math.abs(p[2] + 0.65) < 0.01;
      const chk = (Math.floor(p[0] * 2) + Math.floor(p[1] * 2) + Math.floor(p[2] * 2)) & 1;
      const base = floor ? [120, 96, 70] : ceil ? [200, 200, 195] : table ? [170, 110, 60] : [150, 165, 175];
      for (let c = 0; c < 3; c++) rgba[i + c] = base[c] * (chk ? 1 : 0.8);
      rgba[i + 3] = 255;
    }
    return { disp, w, h, rgba };
  }
  function runDemo() {
    reset();
    st.demo = true; st.running = true; st.speed = 0;
    setStatus('Demo room', 'live');
    const poses = [];
    for (const beta of [90, 55, 125]) for (let a = 0; a < 360; a += 20) poses.push([a, beta, 0]);
    let i = 0;
    const step = async () => {
      if (!st.demo || i >= poses.length) { if (st.demo) coach('Demo room: drag to look around'); st.running = false; return; }
      const R = G.rotation(...poses[i++]);
      st.R = R;
      await capture(R, G.forward(R));
      setTimeout(step, 30);
    };
    step();
  }

  // ---------- controls ----------
  function reset() {
    st.count = 0; st.uploaded = 0; st.frames = 0; st.views = []; st.pano = G.createPano(3); st.demo = false;
    $('nFrames').textContent = '0'; $('nPoints').textContent = '0';
    $('save').disabled = true;
    requestDraw();
  }

  $('start').addEventListener('click', async () => {
    if (st.running && !st.demo) { st.running = false; $('start').textContent = 'Resume scan'; setStatus('Paused'); return; }
    try {
      warn('');
      if (st.demo) reset();
      if (!st.estimator) {
        const o = startOrientation(); // permission must be requested inside the tap
        o.catch(() => {});            // reported below via `await o`
        await startCamera();
        await o;
        await loadModel();
      }
      st.running = true;
      $('start').textContent = 'Pause scan';
      setStatus('Scanning', 'live');
      coach(st.orientationOk ? 'Turn slowly on the spot' : 'Waiting for the motion sensors…');
      maybeCapture();
    } catch (e) {
      console.error(e);
      setStatus('Error', 'err');
      warn(e.name === 'NotAllowedError' ? 'Camera access was declined. Allow it in the browser settings for this site and reload.' : e.message);
    }
  });
  $('reset').addEventListener('click', () => { const was = st.running && !st.demo; reset(); st.running = was; coach('Cleared. Turn slowly on the spot'); });
  $('demo').addEventListener('click', runDemo);
  $('save').addEventListener('click', async () => {
    const scale = parseFloat($('scale').value) || 2.5;
    const file = new File([G.encodePly(st.pos, st.col, st.count, scale)], `echomap-scan-${new Date().toISOString().slice(0, 19).replace(/[:]/g, '-')}.ply`, { type: 'application/octet-stream' });
    try { if (navigator.canShare && navigator.canShare({ files: [file] })) { await navigator.share({ files: [file], title: 'EchoMap 3D scan' }); return; } } catch (e) { if (e.name === 'AbortError') return; }
    const a = document.createElement('a'); a.href = URL.createObjectURL(file); a.download = file.name; document.body.appendChild(a); a.click(); a.remove();
  });

  try { initGL(); } catch (e) { warn('3D view failed to start: ' + e.message); }
  requestDraw();
  window.__scan = st; // for the automated check
})();
