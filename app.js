/* EchoMap app: audio I/O, ping scheduling, displays, test mode, raw export. DSP is in dsp.js. */
(function () {
  'use strict';
  const D = window.EchoDSP;
  const $ = id => document.getElementById(id);

  const BANDS = { quiet: [18000, 22000], audible: [4000, 10000] };
  const CHIRP_SEC = 0.004;
  const PING_INTERVAL = 0.2;      // s; longer than window so pings never overlap
  const WINDOW = 8192;            // samples analysed per ping (~170 ms at 48 kHz)
  const RING_SEC = 30;            // raw audio kept for export
  const ANG_BINS = 72, RANGE_BIN_M = 0.05;

  const state = {
    ctx: null, stream: null, node: null, src: null, analyzer: null, fs: 48000,
    ring: null, written: 0,           // absolute frame count written into ring
    pending: [],                      // scheduled pings awaiting enough audio
    pingLog: [],                      // for export: {frame, t, heading}
    group: [],                        // profiles awaiting averaging
    heading: null, heading0: null, orientationOk: false,
    bg: null, bgCollect: null,
    lastEnv: null, lastMotion: null, lastHits: [], lastC: 343, lastNBins: 0,
    map: new Float32Array(ANG_BINS * 120),
    test: null, testRows: [],
    timer: null, running: false, clipCount: 0,
  };

  const settings = () => ({
    band: BANDS[$('band').value],
    tempC: parseFloat($('temp').value) || 21,
    maxRange: Math.min(6, Math.max(1, parseFloat($('maxRange').value) || 5)),
    avg: Math.min(8, Math.max(1, parseInt($('avg').value, 10) || 3)),
    alpha: parseFloat($('alpha').value) || 5,
    blind: parseFloat($('blind').value) || 0.15,
    offset: parseFloat($('offset').value) || 0,
  });

  function setStatus(text, cls) {
    const s = $('status');
    s.textContent = text;
    s.className = 'status' + (cls ? ' ' + cls : '');
  }
  function warn(msg) {
    const w = $('warn');
    w.hidden = !msg;
    w.textContent = msg || '';
  }

  // ---------- audio capture ----------
  const WORKLET = `
    class Tap extends AudioWorkletProcessor {
      constructor() { super(); this.buf = new Float32Array(2048); this.n = 0; this.start = 0; }
      process(inputs) {
        const ch = inputs[0] && inputs[0][0];
        if (!ch) return true;
        if (this.n === 0) this.start = currentFrame;
        this.buf.set(ch, this.n); this.n += ch.length;
        if (this.n >= 2048) { this.port.postMessage({ frame: this.start, data: this.buf.slice(0, this.n) }); this.n = 0; }
        return true;
      }
    }
    registerProcessor('echomap-tap', Tap);`;

  function onAudio(frame, data) {
    const ring = state.ring, L = ring.length;
    // place samples at their absolute frame position (handles gaps if any)
    for (let i = 0; i < data.length; i++) {
      const v = data[i];
      if (v > 0.98 || v < -0.98) state.clipCount++;
      ring[(frame + i) % L] = v;
    }
    state.written = Math.max(state.written, frame + data.length);
    processPending();
  }

  async function startAudio() {
    if (!window.isSecureContext) throw new Error('This page must be opened over https:// for the microphone to work.');
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) throw new Error('This browser does not allow microphone access.');
    const AC = window.AudioContext || window.webkitAudioContext;
    const ctx = new AC({ latencyHint: 'interactive' });
    await ctx.resume();
    const stream = await navigator.mediaDevices.getUserMedia({
      audio: { echoCancellation: false, noiseSuppression: false, autoGainControl: false, channelCount: 1 }
    });
    // Voice processing would delete the very echoes we want; warn if the browser refused to turn it off.
    const tr = stream.getAudioTracks()[0];
    const st = tr.getSettings ? tr.getSettings() : {};
    if (st.echoCancellation === true) warn('The browser kept echo cancellation on. Echoes may be weakened.');
    state.ctx = ctx; state.stream = stream; state.fs = ctx.sampleRate;
    state.ring = new Float32Array(Math.round(ctx.sampleRate * RING_SEC));
    state.written = 0;
    const src = ctx.createMediaStreamSource(stream);
    state.src = src;
    let node;
    if (ctx.audioWorklet) {
      const url = URL.createObjectURL(new Blob([WORKLET], { type: 'application/javascript' }));
      await ctx.audioWorklet.addModule(url);
      node = new AudioWorkletNode(ctx, 'echomap-tap', { numberOfInputs: 1, numberOfOutputs: 1, channelCount: 1 });
      node.port.onmessage = e => onAudio(e.data.frame, e.data.data);
    } else {
      node = ctx.createScriptProcessor(2048, 1, 1);
      node.onaudioprocess = e => onAudio(Math.round(e.playbackTime * ctx.sampleRate), e.inputBuffer.getChannelData(0).slice());
    }
    // the tap must reach the destination to be pulled; route through a muted gain
    const mute = ctx.createGain(); mute.gain.value = 0;
    src.connect(node); node.connect(mute); mute.connect(ctx.destination);
    state.node = node;
  }

  function buildAnalyzer() {
    const s = settings();
    state.analyzer = D.createAnalyzer({ fs: state.fs, f0: s.band[0], f1: s.band[1], durSec: CHIRP_SEC, windowLen: WINDOW, maxRange: s.maxRange });
    const b = state.ctx.createBuffer(1, state.analyzer.chirp.length, state.fs);
    b.copyToChannel(state.analyzer.chirp, 0);
    state.chirpBuf = b;
    state.group = []; state.bg = null; state.bgCollect = null;
  }

  function schedulePing() {
    const ctx = state.ctx;
    const t = ctx.currentTime + 0.05;
    const s = ctx.createBufferSource();
    s.buffer = state.chirpBuf;
    s.connect(ctx.destination);
    s.start(t);
    const frame = Math.round(t * state.fs);
    const p = { frame, t, heading: state.heading };
    state.pending.push(p);
    state.pingLog.push(p);
    if (state.pingLog.length > RING_SEC / PING_INTERVAL + 10) state.pingLog.shift();
  }

  function processPending() {
    const L = state.ring.length;
    while (state.pending.length && state.written >= state.pending[0].frame + WINDOW) {
      const p = state.pending.shift();
      if (state.written - p.frame > L) continue; // fell out of the ring
      const rec = new Float32Array(WINDOW);
      for (let i = 0; i < WINDOW; i++) rec[i] = state.ring[(p.frame + i) % L];
      analyzePing(rec, p);
    }
  }

  // ---------- analysis ----------
  function analyzePing(rec, p) {
    const s = settings();
    const r = state.analyzer.analyze(rec, { tempC: s.tempC, cancelDirect: true });
    // direct-path SNR: peak vs median envelope before it would be ideal; use the far tail instead
    state.lastDirect = r.direct;
    state.lastC = r.c; state.lastNBins = r.nBins;
    state.group.push({ prof: r.profile, heading: p.heading });
    if (state.group.length < s.avg) return;
    const prof = D.averageProfiles(state.group.map(g => g.prof));
    const heading = state.group[state.group.length - 1].heading;
    state.group = [];
    frame(prof, r, heading, s);
  }

  function frame(prof, r, heading, s) {
    const fs = state.fs, c = r.c;
    const env = D.envelope(prof);
    const start = D.metersToBin(s.blind, fs, c);
    // guard must cover the matched-filter main lobe (~fs/bandwidth samples) or a peak's own
    // shoulders inflate its noise estimate and real echoes get missed
    const guard = Math.ceil(1.5 * fs / Math.abs(s.band[1] - s.band[0]));
    const hits = D.cfar(env, { start, alpha: s.alpha, guard, train: 48 });
    const toM = k => D.binToMeters(k, fs, c) - s.offset;

    // background / motion
    if (state.bgCollect) {
      state.bgCollect.push(prof);
      if (state.bgCollect.length >= 8) { state.bg = D.averageProfiles(state.bgCollect); state.bgCollect = null; setStatus('Live · background set', 'live'); }
    }
    let motion = null;
    if (state.bg && state.bg.re.length === prof.re.length) {
      motion = new Float64Array(env.length);
      for (let i = 0; i < env.length; i++) motion[i] = Math.hypot(prof.re[i] - state.bg.re[i], prof.im[i] - state.bg.im[i]);
    }

    state.lastEnv = env; state.lastMotion = motion; state.lastHits = hits;
    state.lastRef = r.direct.value;

    // noise floor from the last 15% of the profile
    const tail = env.slice(Math.floor(env.length * 0.85)).sort();
    const floor = tail[Math.floor(tail.length / 2)] || 1e-9;
    const snrDb = 20 * Math.log10(r.direct.value / floor);
    $('snr').textContent = isFinite(snrDb) ? snrDb.toFixed(0) + ' dB' : '-- dB';
    const near = hits[0];
    $('nearest').textContent = near ? toM(near.index).toFixed(2) + ' m' : '-- m';

    // map accumulation
    if (heading != null && state.heading0 != null) {
      const rel = ((heading - state.heading0) % 360 + 360) % 360;
      const a = Math.floor(rel / (360 / ANG_BINS)) % ANG_BINS;
      for (const h of hits) {
        const m = toM(h.index);
        const rb = Math.floor(m / RANGE_BIN_M);
        if (rb >= 0 && rb < 120) state.map[a * 120 + rb] += Math.log10(h.snr);
      }
    }

    // test mode
    if (state.test) {
      state.test.readings.push(near ? toM(near.index) : null);
      $('measureBtn').textContent = `Measuring… ${state.test.readings.length}/20`;
      if (state.test.readings.length >= 20) finishTest();
    }

    if (state.clipCount > 50) { warn('The microphone is clipping. Lower the volume a little or move away from hard surfaces right next to the phone.'); state.clipCount = 0; }
    requestDraw();
  }

  // ---------- test mode ----------
  function finishTest() {
    const t = state.test; state.test = null;
    const vals = t.readings.filter(v => v != null).sort((a, b) => a - b);
    const row = { truth: t.truth, n: vals.length };
    if (vals.length) {
      row.median = vals[Math.floor(vals.length / 2)];
      const mean = vals.reduce((a, b) => a + b, 0) / vals.length;
      row.sd = Math.sqrt(vals.reduce((a, b) => a + (b - mean) ** 2, 0) / vals.length);
      row.err = row.median - t.truth;
    }
    state.testRows.push(row);
    renderTest();
    $('measureBtn').textContent = 'Measure (20 readings)';
    $('measureBtn').disabled = false;
  }
  function renderTest() {
    const tb = $('testRows');
    tb.innerHTML = '';
    for (const r of state.testRows) {
      const tr = document.createElement('tr');
      const cells = [r.truth.toFixed(2) + ' m',
        r.median != null ? r.median.toFixed(3) + ' m' : '—',
        r.err != null ? (r.err >= 0 ? '+' : '') + (r.err * 100).toFixed(1) + ' cm' : '—',
        r.sd != null ? '±' + (r.sd * 100).toFixed(1) + ' cm' : '—',
        r.n + '/20'];
      for (const c of cells) { const td = document.createElement('td'); td.textContent = c; tr.appendChild(td); }
      tb.appendChild(tr);
    }
    const errs = state.testRows.filter(r => r.err != null);
    $('applyOffset').disabled = !errs.length;
  }

  // ---------- drawing ----------
  let drawQueued = false;
  function requestDraw() { if (!drawQueued) { drawQueued = true; requestAnimationFrame(draw); } }
  // match the canvas bitmap to its on-screen size so text stays crisp and readable
  function fit(cv, aspect) {
    const dpr = window.devicePixelRatio || 1;
    const w = Math.round(cv.clientWidth * dpr) || 600;
    const h = Math.round(w / aspect);
    if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
    return dpr;
  }
  function css(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }

  function draw() {
    drawQueued = false;
    if (!$('scopeView').hidden) drawScope();
    if (!$('mapView').hidden) drawMap();
  }

  function drawScope() {
    const cv = $('scope'), g = cv.getContext('2d');
    const k = fit(cv, 16 / 10);
    const W = cv.width, H = cv.height, padL = 48 * k, padB = 28 * k, padT = 12 * k, padR = 14 * k;
    const s = settings();
    g.clearRect(0, 0, W, H);
    g.font = (11 * k) + 'px ' + css('--mono'); g.fillStyle = css('--muted'); g.strokeStyle = css('--line'); g.lineWidth = 1;
    const x = m => padL + (m / s.maxRange) * (W - padL - padR);
    const DBMIN = -70;
    const y = db => padT + (Math.max(DBMIN, Math.min(0, db)) / DBMIN) * (H - padT - padB);
    for (let m = 0; m <= s.maxRange + 1e-9; m += 1) {
      g.beginPath(); g.moveTo(x(m), padT); g.lineTo(x(m), H - padB); g.stroke();
      g.textAlign = 'center'; g.fillText(m + ' m', x(m), H - 8 * k);
    }
    for (let db = 0; db >= DBMIN; db -= 20) {
      g.beginPath(); g.moveTo(padL, y(db)); g.lineTo(W - padR, y(db)); g.stroke();
      g.textAlign = 'right'; g.fillText(db + ' dB', padL - 6 * k, y(db) + 4 * k);
    }
    // blind zone
    g.fillStyle = 'rgba(127,148,155,0.12)';
    g.fillRect(padL, padT, x(s.blind) - padL, H - padT - padB);
    if (!state.lastEnv) {
      g.fillStyle = css('--muted'); g.textAlign = 'center';
      g.fillText('Press Start sonar to see echoes', W / 2, H / 2);
      return;
    }
    const ref = state.lastRef || 1, fs = state.fs, c = state.lastC;
    const trace = (arr, color) => {
      g.strokeStyle = color; g.lineWidth = 1.5 * k; g.beginPath();
      for (let k = 0; k < arr.length; k++) {
        const m = D.binToMeters(k, fs, c) - s.offset;
        const px = x(m), py = y(20 * Math.log10((arr[k] + 1e-12) / ref));
        if (k === 0) g.moveTo(px, py); else g.lineTo(px, py);
      }
      g.stroke();
    };
    trace(state.lastEnv, css('--trace'));
    if (state.lastMotion) trace(state.lastMotion, css('--motion'));
    g.fillStyle = css('--hit'); g.textAlign = 'center';
    for (const h of state.lastHits.slice(0, 8)) {
      const m = D.binToMeters(h.index, fs, c) - s.offset;
      const py = y(20 * Math.log10(h.value / ref));
      g.beginPath(); g.arc(x(m), py, 4 * k, 0, Math.PI * 2); g.fill();
      g.fillText(m.toFixed(2), x(m), py - 8 * k);
    }
  }

  function drawMap() {
    const cv = $('map'), g = cv.getContext('2d');
    const k = fit(cv, 1);
    const W = cv.width, cx = W / 2, cy = W / 2, R = W / 2 - 16 * k;
    const s = settings();
    const scale = R / s.maxRange;
    g.clearRect(0, 0, W, W);
    g.strokeStyle = css('--line'); g.fillStyle = css('--muted'); g.font = (11 * k) + 'px ' + css('--mono'); g.lineWidth = 1;
    for (let m = 1; m <= s.maxRange; m++) {
      g.beginPath(); g.arc(cx, cy, m * scale, 0, Math.PI * 2); g.stroke();
      g.fillText(m + ' m', cx + 4 * k, cy - m * scale + 13 * k);
    }
    for (let a = 0; a < 360; a += 45) {
      const t = a * Math.PI / 180;
      g.beginPath(); g.moveTo(cx, cy); g.lineTo(cx - Math.sin(t) * R, cy - Math.cos(t) * R); g.stroke();
    }
    let max = 0;
    for (const v of state.map) if (v > max) max = v;
    const col = css('--trace');
    if (max > 0) {
      const da = 2 * Math.PI / ANG_BINS;
      for (let a = 0; a < ANG_BINS; a++) for (let rb = 0; rb < 120; rb++) {
        const v = state.map[a * 120 + rb];
        if (!v) continue;
        const m = (rb + 0.5) * RANGE_BIN_M;
        if (m > s.maxRange) continue;
        const t = (a + 0.5) * da;
        g.globalAlpha = Math.min(1, 0.15 + 0.85 * v / max);
        g.fillStyle = col;
        g.beginPath(); g.arc(cx - Math.sin(t) * m * scale, cy - Math.cos(t) * m * scale, 3 * k, 0, Math.PI * 2); g.fill();
      }
      g.globalAlpha = 1;
    } else {
      g.fillStyle = css('--muted'); g.textAlign = 'center';
      g.fillText(state.orientationOk ? 'Turn slowly to draw the room' : 'Start sonar and allow motion access', cx, cy + 40 * k);
      g.textAlign = 'start';
    }
    // current beam
    if (state.heading != null && state.heading0 != null) {
      const t = ((state.heading - state.heading0) * Math.PI) / 180;
      g.strokeStyle = css('--motion'); g.lineWidth = 2 * k;
      g.beginPath(); g.moveTo(cx, cy); g.lineTo(cx - Math.sin(t) * R, cy - Math.cos(t) * R); g.stroke();
    }
    g.fillStyle = css('--text'); g.beginPath(); g.arc(cx, cy, 5 * k, 0, Math.PI * 2); g.fill();
  }

  // ---------- orientation ----------
  async function startOrientation() {
    try {
      if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
        const r = await DeviceOrientationEvent.requestPermission();
        if (r !== 'granted') { warn('Motion access was declined, so the Map view cannot know which way you face. Scope and Tape test still work.'); return; }
      }
      window.addEventListener('deviceorientation', e => {
        if (e.alpha == null) return;
        state.orientationOk = true;
        state.heading = e.alpha; // gyro-integrated yaw, counter-clockwise; relative use only
        if (state.heading0 == null) state.heading0 = e.alpha;
        $('heading').textContent = Math.round(((e.alpha - state.heading0) % 360 + 360) % 360) + '°';
        if (!$('mapView').hidden) requestDraw();
      });
    } catch (e) { warn('Could not read the motion sensors: ' + e.message); }
  }

  // ---------- export ----------
  async function exportRaw() {
    const L = state.ring.length;
    const n = Math.min(state.written, L);
    const first = state.written - n;
    const pcm = new Float32Array(n);
    for (let i = 0; i < n; i++) pcm[i] = state.ring[(first + i) % L];
    const s = settings();
    const meta = {
      app: 'EchoMap', created: new Date().toISOString(), sampleRate: state.fs,
      firstFrame: first, chirp: { f0: s.band[0], f1: s.band[1], durationSec: CHIRP_SEC, taper: 'tukey 0.25' },
      windowSamples: WINDOW, tempC: s.tempC, speedOfSound: D.speedOfSound(s.tempC), rangeOffsetM: s.offset,
      note: 'ping.frame is the scheduled output frame; sample index in the WAV = frame - firstFrame. Headings are relative yaw in degrees (counter-clockwise).',
      pings: state.pingLog.filter(p => p.frame >= first).map(p => ({ frame: p.frame, sample: p.frame - first, heading: p.heading })),
      tapeTest: state.testRows,
    };
    const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
    const files = [
      new File([D.encodeWav(pcm, state.fs)], `echomap-${stamp}.wav`, { type: 'audio/wav' }),
      new File([JSON.stringify(meta, null, 1)], `echomap-${stamp}.json`, { type: 'application/json' }),
    ];
    try {
      if (navigator.canShare && navigator.canShare({ files })) { await navigator.share({ files, title: 'EchoMap raw data' }); return; }
    } catch (e) { if (e.name === 'AbortError') return; }
    for (const f of files) {
      const a = document.createElement('a');
      a.href = URL.createObjectURL(f); a.download = f.name;
      document.body.appendChild(a); a.click(); a.remove();
      setTimeout(() => URL.revokeObjectURL(a.href), 5000);
    }
  }

  // ---------- controls ----------
  function startPinging() {
    buildAnalyzer();
    state.pending = [];
    clearInterval(state.timer);
    state.timer = setInterval(schedulePing, PING_INTERVAL * 1000);
    state.running = true;
    setStatus('Live', 'live');
  }

  $('start').addEventListener('click', async () => {
    if (state.running) {
      clearInterval(state.timer); state.running = false;
      state.ctx.suspend();
      $('start').textContent = 'Resume sonar';
      setStatus('Paused');
      return;
    }
    try {
      if (!state.ctx) {
        setStatus('Starting…');
        const o = startOrientation(); // must be requested inside the tap, before any await on iOS
        await startAudio();
        await o;
      } else {
        await state.ctx.resume();
      }
      startPinging();
      $('start').textContent = 'Pause sonar';
      for (const id of ['exportBtn', 'bgBtn', 'bgClear', 'measureBtn']) $(id).disabled = false;
    } catch (e) {
      setStatus('Error', 'err');
      warn(e.name === 'NotAllowedError' ? 'Microphone access was declined. Allow it in Safari’s settings for this site and reload.' : e.message);
    }
  });

  for (const id of ['band', 'maxRange']) $(id).addEventListener('change', () => { if (state.running) startPinging(); });
  $('bgBtn').addEventListener('click', () => { state.bgCollect = []; setStatus('Hold still · capturing background', 'live'); });
  $('bgClear').addEventListener('click', () => { state.bg = null; state.lastMotion = null; requestDraw(); });
  $('mapClear').addEventListener('click', () => { state.map.fill(0); state.heading0 = state.heading; requestDraw(); });
  $('exportBtn').addEventListener('click', exportRaw);
  $('measureBtn').addEventListener('click', () => {
    const truth = parseFloat($('trueDist').value);
    if (!(truth > 0)) { warn('Enter the true distance in meters first.'); return; }
    warn('');
    state.test = { truth, readings: [] };
    $('measureBtn').disabled = true;
  });
  $('applyOffset').addEventListener('click', () => {
    const errs = state.testRows.filter(r => r.err != null).map(r => r.err);
    const mean = errs.reduce((a, b) => a + b, 0) / errs.length;
    $('offset').value = ((parseFloat($('offset').value) || 0) + mean).toFixed(3);
    state.testRows = state.testRows.map(r => ({ ...r, note: 'before offset' }));
  });
  document.querySelectorAll('[role=tab]').forEach(b => b.addEventListener('click', () => {
    document.querySelectorAll('[role=tab]').forEach(o => {
      o.setAttribute('aria-selected', String(o === b));
      $(o.dataset.view).hidden = o !== b;
    });
    requestDraw();
  }));

  requestDraw();
})();
