// Simulated echoes: checks the pipeline recovers known distances despite unknown latency.
const D = require('../dsp.js');
const assert = require('assert');
let seed = 7; const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
const gauss = () => Math.sqrt(-2 * Math.log(rnd() + 1e-12)) * Math.cos(2 * Math.PI * rnd());

function run(f0, f1, label) {
  const fs = 48000, windowLen = 8192, tempC = 21, c = D.speedOfSound(tempC);
  const A = D.createAnalyzer({ fs, f0, f1, durSec: 0.004, windowLen, maxRange: 5 });
  const truth = [0.6, 1.8, 3.2];
  const amps = [0.05, 0.02, 0.01];
  const errs = [];
  for (let trial = 0; trial < 20; trial++) {
    const profs = [];
    for (let ping = 0; ping < 4; ping++) {
    const rec = new Float32Array(windowLen);
    const lat = 800 + Math.floor(rnd() * 2000); // unknown output->input latency, differs per ping
    const add = (delaySamp, a) => {
      const i0 = Math.floor(delaySamp), fr = delaySamp - i0;
      for (let i = 0; i < A.chirp.length; i++) {
        const v = A.chirp[i] * a;
        if (i0 + i < windowLen) rec[i0 + i] += v * (1 - fr);
        if (i0 + i + 1 < windowLen) rec[i0 + i + 1] += v * fr;
      }
    };
    add(lat, 1);
    truth.forEach((m, j) => add(lat + (2 * m / c) * fs, amps[j]));
    for (let i = 0; i < windowLen; i++) rec[i] += 0.01 * gauss();
    const r = A.analyze(rec, { tempC, cancelDirect: true });
    assert(Math.abs(r.direct.index - lat) <= 1, `direct ${r.direct.index} vs ${lat}`);
    profs.push(r.profile);
    }
    const env = D.envelope(D.averageProfiles(profs));
    const start = D.metersToBin(0.2, fs, c);
    const hits = D.cfar(env, { start, alpha: 4 }).map(h => D.binToMeters(h.index, fs, c));
    truth.forEach(m => {
      const best = hits.reduce((b, h) => Math.abs(h - m) < Math.abs(b - m) ? h : b, Infinity);
      errs.push(Math.abs(best - m));
    });
  }
  const max = Math.max(...errs);
  console.log(`${label}: max range error ${(max * 100).toFixed(1)} cm over ${errs.length} echoes`);
  assert(max < 0.03, 'range error too large');
}
run(18000, 22000, 'quiet 18-22 kHz');
run(4000, 10000, 'audible 4-10 kHz');

// WAV header sanity
const w = new DataView(D.encodeWav(new Float32Array([0, 0.5, -0.5]), 48000));
assert.strictEqual(w.getUint32(24, true), 48000);
assert(Math.abs(w.getInt16(46, true) - 16383) <= 1);
console.log('ok');
