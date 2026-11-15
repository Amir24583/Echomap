// PLY reader: EchoMap's own files plus the layouts iPhone LiDAR apps export (ascii, float colours,
// extra properties, mesh faces, big-endian), and the Y-up -> Z-up turn.
const fs = require('fs');
const path = require('path');
const assert = require('assert');
const { parsePly, yUpToZUp, isYUp } = require('../ply.js');
const ab = b => b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength);

// 1. The Livox sample map: Z-up EchoMap file
const sample = parsePly(ab(fs.readFileSync(path.join(__dirname, '../livox/sample/sim-map.ply'))));
assert(sample.n > 100000 && sample.hasColor && sample.faces === 0);
assert(!isYUp(sample));

// 2. ascii mesh with normals and float colours, CRLF line ends, faces after vertices (3d Scanner App / Polycam style)
const ascii = ['ply', 'format ascii 1.0', 'comment exported by a phone app', 'element vertex 3',
  'property float x', 'property float y', 'property float z', 'property float nx', 'property float ny', 'property float nz',
  'property float red', 'property float green', 'property float blue', 'element face 1', 'property list uchar int vertex_indices',
  'end_header', '0 0 0 0 1 0 1 0 0', '1 0 0 0 1 0 0 1 0', '0 2 -3 0 1 0 0 0 0.5', '3 0 1 2', ''].join('\r\n');
const a = parsePly(ab(Buffer.from(ascii)));
assert.strictEqual(a.n, 3); assert.strictEqual(a.faces, 1); assert(isYUp(a));
assert.deepStrictEqual([...a.pos], [0, 0, 0, 1, 0, 0, 0, 2, -3]);
assert.deepStrictEqual([...a.col], [255, 0, 0, 0, 255, 0, 0, 0, 128]);
yUpToZUp(a.pos);  // y-up (0,2,-3): 2 m up, 3 m in front -> z-up (0,3,2)
assert.deepStrictEqual([...a.pos.slice(6)], [0, 3, 2]);

// 3. binary big-endian, double xyz, uchar r/g/b named r,g,b, a face element BEFORE the vertices, no colour on one file
function bin(le, faceFirst, withColor) {
  const head = ['ply', `format binary_${le ? 'little' : 'big'}_endian 1.0`];
  const face = ['element face 2', 'property list uchar uint vertex_indices'];
  const vert = ['element vertex 2', 'property double x', 'property double y', 'property double z', 'property uchar confidence']
    .concat(withColor ? ['property uchar r', 'property uchar g', 'property uchar b'] : []);
  const txt = head.concat(faceFirst ? face.concat(vert) : vert.concat(face), ['end_header', '']).join('\n');
  const parts = [Buffer.from(txt)];
  const faceBytes = () => { const f = Buffer.alloc(2 * 13); for (let k = 0; k < 2; k++) { f.writeUInt8(3, k * 13); for (let j = 0; j < 3; j++) le ? f.writeUInt32LE(j, k * 13 + 1 + 4 * j) : f.writeUInt32BE(j, k * 13 + 1 + 4 * j); } return f; };
  if (faceFirst) parts.push(faceBytes());
  for (const [x, y, z, c] of [[1.5, -2, 3, [10, 20, 30]], [4, 5, 6.25, [40, 50, 60]]]) {
    const v = Buffer.alloc(25 + (withColor ? 3 : 0));
    for (let k = 0; k < 3; k++) le ? v.writeDoubleLE([x, y, z][k], 8 * k) : v.writeDoubleBE([x, y, z][k], 8 * k);
    v.writeUInt8(2, 24); if (withColor) c.forEach((cc, k) => v.writeUInt8(cc, 25 + k));
    parts.push(v);
  }
  if (!faceFirst) parts.push(faceBytes());
  return ab(Buffer.concat(parts));
}
for (const le of [true, false]) for (const faceFirst of [true, false]) {
  const b = parsePly(bin(le, faceFirst, true));
  assert.deepStrictEqual([...b.pos], [1.5, -2, 3, 4, 5, 6.25]);
  assert.deepStrictEqual([...b.col], [10, 20, 30, 40, 50, 60]);
  assert.strictEqual(b.faces, 2);
}
const nc = parsePly(bin(true, false, false));
assert(!nc.hasColor && nc.col.every(c => c === 200));

// 4. errors
assert.throws(() => parsePly(ab(Buffer.from('hello'))), /Not a PLY/);
const cut = Buffer.from(bin(true, false, true)); assert.throws(() => parsePly(ab(cut.subarray(0, cut.length - 40))), /cut short/);

console.log('ply ok');
