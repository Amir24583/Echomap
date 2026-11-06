/* EchoMap signal processing. Pure functions, no DOM, so they can be tested in Node.
 *
 * Pipeline for one ping:
 *   recording window -> matched filter (FFT cross-correlation with the chirp, analytic output)
 *   -> find the direct speaker-to-mic peak (the "0 m" reference, removes unknown audio latency)
 *   -> subtract the ideal direct-path response (its sidelobes would mask near echoes)
 *   -> range profile r = c * (k - d) / (2 * fs)
 *   -> CA-CFAR detection of echo peaks
 */
(function (root) {
  'use strict';

  function speedOfSound(tempC) {
    return 331.3 + 0.606 * tempC; // m/s, dry air
  }

  function nextPow2(n) {
    let p = 1;
    while (p < n) p <<= 1;
    return p;
  }

  // In-place iterative radix-2 FFT. re/im are Float64Array of length n (power of 2).
  function fft(re, im, inverse) {
    const n = re.length;
    for (let i = 1, j = 0; i < n; i++) {
      let bit = n >> 1;
      for (; j & bit; bit >>= 1) j ^= bit;
      j ^= bit;
      if (i < j) {
        let t = re[i]; re[i] = re[j]; re[j] = t;
        t = im[i]; im[i] = im[j]; im[j] = t;
      }
    }
    for (let len = 2; len <= n; len <<= 1) {
      const ang = (2 * Math.PI / len) * (inverse ? 1 : -1);
      const wr = Math.cos(ang), wi = Math.sin(ang);
      const half = len >> 1;
      for (let i = 0; i < n; i += len) {
        let cr = 1, ci = 0;
        for (let k = 0; k < half; k++) {
          const a = i + k, b = a + half;
          const xr = re[b] * cr - im[b] * ci;
          const xi = re[b] * ci + im[b] * cr;
          re[b] = re[a] - xr; im[b] = im[a] - xi;
          re[a] += xr; im[a] += xi;
          const ncr = cr * wr - ci * wi;
          ci = cr * wi + ci * wr;
          cr = ncr;
        }
      }
    }
    if (inverse) {
      for (let i = 0; i < n; i++) { re[i] /= n; im[i] /= n; }
    }
  }

  // Linear FM chirp f0 -> f1 over durSec, Tukey-tapered (25% edges) to limit spectral splatter
  // and the audible click a hard-edged burst makes.
  function makeChirp(fs, f0, f1, durSec) {
    const n = Math.round(fs * durSec);
    const out = new Float32Array(n);
    const k = (f1 - f0) / durSec;
    const taper = 0.25;
    for (let i = 0; i < n; i++) {
      const t = i / fs;
      const x = i / (n - 1);
      let w = 1;
      if (x < taper / 2) w = 0.5 * (1 - Math.cos((2 * Math.PI * x) / taper));
      else if (x > 1 - taper / 2) w = 0.5 * (1 - Math.cos((2 * Math.PI * (1 - x)) / taper));
      out[i] = w * Math.sin(2 * Math.PI * (f0 * t + 0.5 * k * t * t));
    }
    return out;
  }

  // Matched filter. Returns the analytic cross-correlation (re, im) of rec with tmpl,
  // lag 0..rec.length-1. |c| is the echo envelope; complex values allow coherent averaging
  // and background subtraction. Optional band-pass keeps only [fLo, fHi].
  function makeMatchedFilter(tmpl, recLen, fs, fLo, fHi) {
    const n = nextPow2(recLen + tmpl.length);
    const tr = new Float64Array(n), ti = new Float64Array(n);
    for (let i = 0; i < tmpl.length; i++) tr[i] = tmpl[i];
    fft(tr, ti, false);
    // energy normalisation so a perfect copy of the chirp correlates to ~1
    let e = 0;
    for (let i = 0; i < tmpl.length; i++) e += tmpl[i] * tmpl[i];
    const binLo = Math.floor((fLo / fs) * n), binHi = Math.ceil((fHi / fs) * n);
    const re = new Float64Array(n), im = new Float64Array(n);
    return function apply(rec) {
      re.fill(0); im.fill(0);
      const L = Math.min(rec.length, recLen);
      for (let i = 0; i < L; i++) re[i] = rec[i];
      fft(re, im, false);
      // multiply by conj(T); keep positive frequencies only (x2) -> analytic signal
      for (let k = 0; k < n; k++) {
        if (k === 0 || k > n / 2 || k < binLo || k > binHi) { re[k] = 0; im[k] = 0; continue; }
        const a = re[k], b = im[k], c = tr[k], d = -ti[k];
        re[k] = 2 * (a * c - b * d) / e;
        im[k] = 2 * (a * d + b * c) / e;
      }
      fft(re, im, true);
      return { re: re.slice(0, L), im: im.slice(0, L) };
    };
  }

  function envelope(c) {
    const out = new Float64Array(c.re.length);
    for (let i = 0; i < out.length; i++) out[i] = Math.hypot(c.re[i], c.im[i]);
    return out;
  }

  // Strongest peak in [from, to): the direct speaker->mic path, always the loudest arrival.
  function findDirect(env, from, to) {
    let best = from, bv = -1;
    for (let i = from; i < Math.min(to, env.length); i++) if (env[i] > bv) { bv = env[i]; best = i; }
    return { index: best, value: bv };
  }

  // Ideal response of the chirp to itself (analytic), centred at lag 0. Used to cancel the
  // direct path: scaled copy placed at the direct-path index and subtracted.
  function selfResponse(chirp, fs, fLo, fHi, half) {
    const L = 2 * half + chirp.length;
    const rec = new Float32Array(L);
    rec.set(chirp, half);
    const mf = makeMatchedFilter(chirp, L, fs, fLo, fHi);
    const c = mf(rec);
    return { re: c.re.slice(0, 2 * half + 1), im: c.im.slice(0, 2 * half + 1), half };
  }

  // Extract the range profile starting at the direct path. Returns complex arrays aligned so
  // index 0 = direct path. If selfResp is given, the direct path's ideal response is removed.
  function rangeProfile(c, d, nBins, selfResp) {
    const re = new Float64Array(nBins), im = new Float64Array(nBins);
    for (let k = 0; k < nBins; k++) {
      const j = d + k;
      if (j >= c.re.length) break;
      re[k] = c.re[j]; im[k] = c.im[j];
    }
    if (selfResp) {
      // complex amplitude of the direct path relative to the ideal response peak
      const pr = selfResp.re[selfResp.half], pi = selfResp.im[selfResp.half];
      const p2 = pr * pr + pi * pi;
      const ar = (re[0] * pr + im[0] * pi) / p2;
      const ai = (im[0] * pr - re[0] * pi) / p2;
      for (let k = 0; k < nBins && selfResp.half + k < selfResp.re.length; k++) {
        const sr = selfResp.re[selfResp.half + k], si = selfResp.im[selfResp.half + k];
        re[k] -= ar * sr - ai * si;
        im[k] -= ar * si + ai * sr;
      }
    }
    return { re, im };
  }

  // Coherent average of several aligned complex profiles (each already referenced to its own
  // direct path, so latency jitter between pings does not smear the echoes). Noise drops by
  // sqrt(N) while static echoes add up.
  function averageProfiles(list) {
    const n = list[0].re.length;
    const re = new Float64Array(n), im = new Float64Array(n);
    for (const p of list) for (let i = 0; i < n; i++) { re[i] += p.re[i]; im[i] += p.im[i]; }
    for (let i = 0; i < n; i++) { re[i] /= list.length; im[i] /= list.length; }
    return { re, im };
  }

  // Cell-averaging CFAR. Returns indices of local maxima above alpha * local noise mean.
  function cfar(env, opts) {
    const guard = opts.guard || 12, train = opts.train || 48, alpha = opts.alpha || 4;
    const start = opts.start || 0;
    const hits = [];
    for (let i = start; i < env.length; i++) {
      let s = 0, n = 0;
      for (let j = i - guard - train; j < i - guard; j++) if (j >= start && j >= 0) { s += env[j]; n++; }
      for (let j = i + guard + 1; j <= i + guard + train; j++) if (j < env.length) { s += env[j]; n++; }
      if (!n) continue;
      const noise = s / n;
      const v = env[i];
      if (v > alpha * noise && v >= env[i - 1] && v >= (env[i + 1] ?? 0)) {
        hits.push({ index: i, value: v, snr: v / (noise || 1e-12) });
      }
    }
    // keep only the strongest within +-guard (one peak per echo)
    hits.sort((a, b) => b.value - a.value);
    const kept = [];
    for (const h of hits) if (!kept.some(k => Math.abs(k.index - h.index) <= guard)) kept.push(h);
    return kept.sort((a, b) => a.index - b.index);
  }

  function binToMeters(k, fs, c) {
    return (k / fs) * c / 2;
  }
  function metersToBin(m, fs, c) {
    return Math.round((2 * m / c) * fs);
  }

  // Full single-ping analysis. rec: Float32Array window that contains the transmitted chirp
  // (direct path) somewhere in [0, searchLen).
  function createAnalyzer(cfg) {
    const { fs, f0, f1, durSec, windowLen, maxRange } = cfg;
    const chirp = makeChirp(fs, f0, f1, durSec);
    const fLo = Math.min(f0, f1) - 500, fHi = Math.max(f0, f1) + 500;
    const mf = makeMatchedFilter(chirp, windowLen, fs, fLo, fHi);
    const selfResp = selfResponse(chirp, fs, fLo, fHi, 512);
    return {
      chirp,
      analyze(rec, opts) {
        const c = speedOfSound(opts.tempC);
        const nBins = metersToBin(maxRange, fs, c);
        const corr = mf(rec);
        const env = envelope(corr);
        const searchTo = Math.max(1, windowLen - nBins);
        const direct = findDirect(env, 0, searchTo);
        const prof = rangeProfile(corr, direct.index, nBins, opts.cancelDirect ? selfResp : null);
        return { direct, profile: prof, c, nBins };
      }
    };
  }

  // WAV (16-bit PCM mono) encoder for raw-data export
  function encodeWav(samples, fs) {
    const buf = new ArrayBuffer(44 + samples.length * 2);
    const v = new DataView(buf);
    const w = (o, s) => { for (let i = 0; i < s.length; i++) v.setUint8(o + i, s.charCodeAt(i)); };
    w(0, 'RIFF'); v.setUint32(4, 36 + samples.length * 2, true); w(8, 'WAVE');
    w(12, 'fmt '); v.setUint32(16, 16, true); v.setUint16(20, 1, true); v.setUint16(22, 1, true);
    v.setUint32(24, fs, true); v.setUint32(28, fs * 2, true); v.setUint16(32, 2, true); v.setUint16(34, 16, true);
    w(36, 'data'); v.setUint32(40, samples.length * 2, true);
    for (let i = 0; i < samples.length; i++) {
      const s = Math.max(-1, Math.min(1, samples[i]));
      v.setInt16(44 + i * 2, s < 0 ? s * 0x8000 : s * 0x7fff, true);
    }
    return buf;
  }

  const api = {
    speedOfSound, nextPow2, fft, makeChirp, makeMatchedFilter, envelope, findDirect,
    selfResponse, rangeProfile, averageProfiles, cfar, binToMeters, metersToBin, createAnalyzer, encodeWav
  };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  root.EchoDSP = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);
