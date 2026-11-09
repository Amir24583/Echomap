// Geometry checks for the 3D scan: orientation conventions, and that frames with different
// unknown depth scale/shift are aligned into one consistent room.
const G = require('../scan-geom.js');
const assert = require('assert');
const near = (a, b, tol = 1e-6) => a.every((v, i) => Math.abs(v - b[i]) < tol);

// phone upright (beta 90): camera looks north; turning counter-clockwise 90 deg looks west
assert(near(G.forward(G.rotation(0, 90, 0)), [0, 1, 0]));
assert(near(G.forward(G.rotation(90, 90, 0)), [-1, 0, 0]));
// phone flat on a table, screen up: camera looks down
assert(near(G.forward(G.rotation(0, 0, 0)), [0, 0, -1]));
// top of the image is up when upright
assert(near(G.apply(G.rotation(0, 90, 0), G.pixelRay(50, 0, 100, 100, 50)), [0, 1, 1]));

// Synthetic room: a cylinder wall of radius 3 m around the viewer plus floor 1.4 m below, ceiling 1.1 m above.
function sceneRange(d) {
  const h = Math.hypot(d[0], d[1]);
  let t = 3 / h;
  if (d[2] < 0) t = Math.min(t, 1.4 / -d[2]);
  if (d[2] > 0) t = Math.min(t, 1.1 / d[2]);
  return t; // multiplier on d (d has z-depth 1)
}
const w = 96, h = 128, fov = 65, pano = G.createPano(3);
const rgba = new Uint8ClampedArray(w * h * 4).fill(128);
let seed = 3; const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
const all = [];
let aligned = 0, frames = 0;
for (let alpha = 0; alpha < 360; alpha += 15) {
  const R = G.rotation(alpha, 90, 0);
  const f = G.focalPx(w, h, fov);
  const disp = new Float32Array(w * h);
  // each frame gets its own unknown scale and shift, like a relative-depth network
  // (the first frame has zero shift: its shift cannot be recovered from one view)
  const s = 0.5 + rnd() * 3, sh = alpha === 0 ? 0 : rnd() * 0.5;
  for (let v = 0; v < h; v++) for (let u = 0; u < w; u++) {
    const z = sceneRange(G.apply(R, G.pixelRay(u, v, w, h, f)));
    disp[v * w + u] = s * (1 / z) + sh;
  }
  const out = G.frameToPoints(disp, w, h, R, rgba, pano, { longFovDeg: fov, stride: 4, edge: 0.5, minPairs: 30 });
  frames++; if (out.fit.aligned) aligned++;
  for (let i = 0; i < out.positions.length; i += 3) all.push([out.positions[i], out.positions[i + 1], out.positions[i + 2]]);
}
assert.strictEqual(aligned, frames - 1, 'every frame after the first should align to the panorama');
// The whole cloud should be the true room times ONE global scale (first frame sets it).
// ratio of reconstructed range to true range along the same direction
const rr = all.map(p => { const n = Math.hypot(...p); const d = p.map(x => x / n); const tr = sceneRange(d) * 1; return n / (tr * Math.hypot(...d)); });
const med = G.percentile(rr, 0.5), lo = G.percentile(rr, 0.02), hi = G.percentile(rr, 0.98);
console.log(`points ${all.length}, frames ${frames}, scale ${med.toFixed(3)}, spread ${(lo / med).toFixed(3)}..${(hi / med).toFixed(3)}`);
assert(lo / med > 0.9 && hi / med < 1.1, 'room shape should be consistent to within 10% all the way round');

// PLY
const ply = G.encodePly(new Float32Array([1, 2, 3]), new Uint8Array([9, 8, 7]), 1, 2);
const txt = Buffer.from(ply).toString('latin1');
assert(txt.startsWith('ply\n') && txt.includes('element vertex 1'));
assert.strictEqual(new DataView(ply).getFloat32(ply.byteLength - 15, true), 2);
console.log('ok');
