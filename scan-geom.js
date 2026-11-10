/* EchoMap 3D scan geometry. Pure functions (no DOM) so they can be tested in Node.
 *
 * Model: you stand on one spot and turn the phone. Every pixel's viewing ray depends only on the
 * phone's rotation (from the gyroscope), so each depth frame becomes a patch of one spherical
 * depth panorama. The depth network returns relative inverse depth (bigger = closer) with an
 * unknown scale and shift per frame; each new frame is aligned to the panorama built so far with
 * a least-squares affine fit over the overlapping directions.
 *
 * World frame (W3C deviceorientation): x east, y north, z up. Units: 1 = the room-scale setting.
 */
(function (root) {
  'use strict';
  const RAD = Math.PI / 180;

  // Rotation device -> world: R = Rz(alpha) * Rx(beta) * Ry(gamma), row-major 3x3.
  function rotation(alphaDeg, betaDeg, gammaDeg) {
    const a = alphaDeg * RAD, b = betaDeg * RAD, g = gammaDeg * RAD;
    const cA = Math.cos(a), sA = Math.sin(a), cB = Math.cos(b), sB = Math.sin(b), cG = Math.cos(g), sG = Math.sin(g);
    return [
      cA * cG - sA * sB * sG, -cB * sA, cA * sG + cG * sA * sB,
      cG * sA + cA * sB * sG, cA * cB, sA * sG - cA * cG * sB,
      -cB * sG, sB, cB * cG,
    ];
  }
  function apply(R, v) {
    return [R[0] * v[0] + R[1] * v[1] + R[2] * v[2], R[3] * v[0] + R[4] * v[1] + R[5] * v[2], R[6] * v[0] + R[7] * v[1] + R[8] * v[2]];
  }
  // Rear camera looks along device -z; image x = device x, image y (down) = device -y.
  function forward(R) { return apply(R, [0, 0, -1]); }
  function angleBetween(a, b) {
    const d = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (Math.hypot(...a) * Math.hypot(...b));
    return Math.acos(Math.max(-1, Math.min(1, d))) / RAD;
  }
  // Focal length in pixels from the field of view across the image's long side.
  function focalPx(w, h, longFovDeg) {
    return Math.max(w, h) / (2 * Math.tan((longFovDeg * RAD) / 2));
  }
  // Unit-z ray in device coords for pixel (u, v); multiply by z-depth to get the point.
  function pixelRay(u, v, w, h, f) {
    return [(u - w / 2) / f, -(v - h / 2) / f, -1];
  }

  function percentile(arr, p) {
    const s = Float32Array.from(arr).sort();
    return s[Math.min(s.length - 1, Math.max(0, Math.floor(p * (s.length - 1))))];
  }

  // Least-squares y = a*x + b
  function fitAffine(xs, ys) {
    const n = xs.length;
    let sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (let i = 0; i < n; i++) { sx += xs[i]; sy += ys[i]; sxx += xs[i] * xs[i]; sxy += xs[i] * ys[i]; }
    const den = n * sxx - sx * sx;
    if (Math.abs(den) < 1e-12) return null;
    const a = (n * sxy - sx * sy) / den;
    return { a, b: (sy - a * sx) / n };
  }

  // Spherical panorama of inverse *range* (1 / distance from the viewer), in cells of cellDeg.
  function createPano(cellDeg) {
    const cols = Math.round(360 / cellDeg), rows = Math.round(180 / cellDeg);
    return { cellDeg, cols, rows, sum: new Float32Array(cols * rows), n: new Uint16Array(cols * rows) };
  }
  function panoIndex(p, d) {
    const len = Math.hypot(d[0], d[1], d[2]);
    const az = Math.atan2(d[0], d[1]) / RAD;             // -180..180, 0 = north
    const el = Math.asin(d[2] / len) / RAD;              // -90..90
    const c = Math.min(p.cols - 1, Math.floor((az + 180) / p.cellDeg));
    const r = Math.min(p.rows - 1, Math.floor((el + 90) / p.cellDeg));
    return r * p.cols + c;
  }

  const FAR_CAP = 6; // farthest allowed point, in room-scale units

  /* Turn one depth frame into world points.
   *  disp: Float32Array (w*h) relative inverse depth; R: rotation at capture; rgba: Uint8ClampedArray (w*h*4)
   *  Returns { positions: Float32Array(3n), colors: Uint8Array(3n), fit: {a,b,aligned,pairs} }.
   *  Mutates pano (adds this frame). */
  function frameToPoints(disp, w, h, R, rgba, pano, opts) {
    const f = focalPx(w, h, opts.longFovDeg);
    const stride = opts.stride;
    // sample grid
    const us = [], vs = [], ds = [];
    for (let v = stride >> 1; v < h - 1; v += stride) for (let u = stride >> 1; u < w - 1; u += stride) {
      const i = v * w + u, d = disp[i];
      // drop depth edges: points there are smeared between foreground and background
      const gx = Math.abs(disp[i + 1] - d), gy = Math.abs(disp[i + w] - d);
      us.push(u); vs.push(v); ds.push(d);
      if ((gx + gy) > opts.edge * (Math.abs(d) + 1e-6)) ds[ds.length - 1] = NaN;
    }
    const n = ds.length;
    const dirs = new Float32Array(n * 3), zscale = new Float32Array(n), cells = new Int32Array(n);
    for (let k = 0; k < n; k++) {
      const ray = pixelRay(us[k], vs[k], w, h, f);
      const wd = apply(R, ray);
      dirs[k * 3] = wd[0]; dirs[k * 3 + 1] = wd[1]; dirs[k * 3 + 2] = wd[2];
      zscale[k] = Math.hypot(ray[0], ray[1], ray[2]); // range = zdepth * zscale
      cells[k] = panoIndex(pano, wd);
    }
    // affine alignment: inverse z-depth = a*disp + b
    let fit = null;
    const xs = [], ys = [];
    for (let k = 0; k < n; k++) {
      const c = cells[k];
      if (pano.n[c] && !Number.isNaN(ds[k])) { xs.push(ds[k]); ys.push((pano.sum[c] / pano.n[c]) * zscale[k]); }
    }
    if (xs.length >= opts.minPairs) {
      const r = fitAffine(xs, ys);
      if (r && r.a > 0) fit = { a: r.a, b: r.b, aligned: true, pairs: xs.length };
    }
    if (!fit) {
      // No usable overlap (first frame, or a jump to a new direction). The network's output is
      // close to true inverse depth up to scale, so assume zero shift and put the median at 1 unit.
      const valid = ds.filter(d => !Number.isNaN(d));
      const pMid = percentile(valid, 0.5);
      fit = { a: 1 / Math.max(pMid, 1e-6), b: 0, aligned: false, pairs: xs.length };
    }
    const positions = new Float32Array(n * 3), colors = new Uint8Array(n * 3);
    let m = 0;
    for (let k = 0; k < n; k++) {
      if (Number.isNaN(ds[k])) continue;
      const inv = Math.max(fit.a * ds[k] + fit.b, 1 / FAR_CAP);
      if (inv <= 1 / FAR_CAP) continue; // at or beyond the cap: unreliable, skip
      const z = 1 / inv;
      positions[m * 3] = dirs[k * 3] * z; positions[m * 3 + 1] = dirs[k * 3 + 1] * z; positions[m * 3 + 2] = dirs[k * 3 + 2] * z;
      const pi = (vs[k] * w + us[k]) * 4;
      colors[m * 3] = rgba[pi]; colors[m * 3 + 1] = rgba[pi + 1]; colors[m * 3 + 2] = rgba[pi + 2];
      const c = cells[k];
      if (pano.n[c] < 65535) { pano.sum[c] += inv / zscale[k]; pano.n[c]++; }
      m++;
    }
    return { positions: positions.subarray(0, m * 3), colors: colors.subarray(0, m * 3), fit };
  }

  // Binary little-endian PLY point cloud
  function encodePly(positions, colors, count, scale) {
    const head = `ply\nformat binary_little_endian 1.0\ncomment EchoMap 3D scan, units: meters (approximate)\nelement vertex ${count}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n`;
    const buf = new ArrayBuffer(head.length + count * 15);
    const u8 = new Uint8Array(buf), dv = new DataView(buf);
    for (let i = 0; i < head.length; i++) u8[i] = head.charCodeAt(i);
    let o = head.length;
    for (let i = 0; i < count; i++) {
      dv.setFloat32(o, positions[i * 3] * scale, true); dv.setFloat32(o + 4, positions[i * 3 + 1] * scale, true); dv.setFloat32(o + 8, positions[i * 3 + 2] * scale, true);
      u8[o + 12] = colors[i * 3]; u8[o + 13] = colors[i * 3 + 1]; u8[o + 14] = colors[i * 3 + 2];
      o += 15;
    }
    return buf;
  }

  const api = { rotation, apply, forward, angleBetween, focalPx, pixelRay, percentile, fitAffine, createPano, panoIndex, frameToPoints, encodePly, FAR_CAP };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  root.ScanGeom = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);
