/* EchoMap PLY reader. Reads the vertex positions and colours of any common PLY file:
   ascii, binary little-endian or binary big-endian, any property order and types, extra
   properties (normals, confidence, classification) and other elements such as mesh faces.
   That covers the Livox mapper's output and what iPhone LiDAR apps export.
   Returns { n, pos: Float32Array(n*3), col: Uint8Array(n*3), hasColor, faces, comments }. */
(function (root) {
  'use strict';
  const SIZE = { char: 1, int8: 1, uchar: 1, uint8: 1, short: 2, int16: 2, ushort: 2, uint16: 2,
    int: 4, int32: 4, uint: 4, uint32: 4, float: 4, float32: 4, double: 8, float64: 8 };
  const GET = { char: 'getInt8', int8: 'getInt8', uchar: 'getUint8', uint8: 'getUint8', short: 'getInt16', int16: 'getInt16',
    ushort: 'getUint16', uint16: 'getUint16', int: 'getInt32', int32: 'getInt32', uint: 'getUint32', uint32: 'getUint32',
    float: 'getFloat32', float32: 'getFloat32', double: 'getFloat64', float64: 'getFloat64' };
  const COLOR = [['red', 'r', 'diffuse_red'], ['green', 'g', 'diffuse_green'], ['blue', 'b', 'diffuse_blue']];

  function findHeaderEnd(bytes) {
    const lim = Math.min(bytes.length, 65536);
    for (let i = 0; i < lim; i++) {
      if (bytes[i] !== 101) continue; // 'e'
      let s = ''; for (let j = 0; j < 10 && i + j < bytes.length; j++) s += String.fromCharCode(bytes[i + j]);
      if (s !== 'end_header') continue;
      let k = i + 10; if (bytes[k] === 13) k++; if (bytes[k] === 10) return k + 1;
    }
    return -1;
  }

  function parseHeader(text) {
    const lines = text.split(/\r?\n/).map(l => l.trim());
    if (lines[0] !== 'ply') throw new Error('Not a PLY file.');
    let format = null; const elements = [], comments = [];
    for (const l of lines.slice(1)) {
      const t = l.split(/\s+/);
      if (t[0] === 'format') format = t[1];
      else if (t[0] === 'comment' || t[0] === 'obj_info') comments.push(l.slice(t[0].length).trim());
      else if (t[0] === 'element') elements.push({ name: t[1], count: parseInt(t[2], 10), props: [] });
      else if (t[0] === 'property') {
        const el = elements[elements.length - 1]; if (!el) throw new Error('PLY property outside an element.');
        if (t[1] === 'list') el.props.push({ list: true, countType: t[2], type: t[3], name: t[4] });
        else el.props.push({ type: t[1], name: t[2] });
      }
    }
    if (!['ascii', 'binary_little_endian', 'binary_big_endian'].includes(format)) throw new Error('Unknown PLY format: ' + format);
    for (const el of elements) for (const p of el.props) {
      if (!SIZE[p.type] || (p.list && !SIZE[p.countType])) throw new Error('Unknown PLY property type in ' + el.name + '.' + p.name);
    }
    return { format, elements, comments };
  }

  function parsePly(buf) {
    const bytes = new Uint8Array(buf);
    const headEnd = findHeaderEnd(bytes);
    if (headEnd < 0) throw new Error('Not a PLY file (no end_header).');
    const { format, elements, comments } = parseHeader(new TextDecoder().decode(bytes.subarray(0, headEnd)));
    const vEl = elements.find(e => e.name === 'vertex');
    if (!vEl) throw new Error('The PLY file has no vertex element.');
    const idx = name => vEl.props.findIndex(p => !p.list && p.name === name);
    const ix = idx('x'), iy = idx('y'), iz = idx('z');
    if (ix < 0 || iy < 0 || iz < 0) throw new Error('The PLY vertices have no x, y, z.');
    const ic = COLOR.map(names => { for (const nm of names) { const i = idx(nm); if (i >= 0) return i; } return -1; });
    const hasColor = ic.every(i => i >= 0);
    const scale = ic.map(i => (i >= 0 && /float|double/.test(vEl.props[i].type) ? 255 : 1));
    const n = vEl.count, pos = new Float32Array(n * 3), col = new Uint8Array(n * 3).fill(200);
    const fEl = elements.find(e => e.name === 'face'), faces = fEl ? fEl.count : 0;
    const store = (i, vals) => {
      pos[i * 3] = vals[ix]; pos[i * 3 + 1] = vals[iy]; pos[i * 3 + 2] = vals[iz];
      if (hasColor) for (let k = 0; k < 3; k++) col[i * 3 + k] = Math.max(0, Math.min(255, Math.round(vals[ic[k]] * scale[k])));
    };

    if (format === 'ascii') {
      const words = new TextDecoder().decode(bytes.subarray(headEnd)).split(/\s+/).filter(Boolean);
      let w = 0;
      const next = () => { if (w >= words.length) throw new Error('The PLY file is cut short.'); return parseFloat(words[w++]); };
      for (const el of elements) {
        for (let r = 0; r < el.count; r++) {
          const vals = [];
          for (const p of el.props) { if (p.list) { const c = next(); for (let j = 0; j < c; j++) next(); vals.push(c); } else vals.push(next()); }
          if (el === vEl) store(r, vals);
        }
        if (el === vEl) break;               // later elements (faces) are not needed
      }
    } else {
      const le = format === 'binary_little_endian';
      const dv = new DataView(buf, headEnd); let o = 0;
      const read = type => { if (o + SIZE[type] > dv.byteLength) throw new Error('The PLY file is cut short.'); const v = dv[GET[type]](o, le); o += SIZE[type]; return v; };
      for (const el of elements) {
        const fixed = el.props.every(p => !p.list);
        if (el !== vEl && fixed) {          // skip whole element without reading it
          o += el.count * el.props.reduce((s, p) => s + SIZE[p.type], 0);
        } else {
          for (let r = 0; r < el.count; r++) {
            const vals = [];
            for (const p of el.props) {
              if (p.list) { const c = read(p.countType); if (el === vEl) vals.push(c); o += c * SIZE[p.type]; if (o > dv.byteLength) throw new Error('The PLY file is cut short.'); }
              else vals.push(read(p.type));
            }
            if (el === vEl) store(r, vals);
          }
        }
        if (el === vEl) break;
      }
    }
    return { n, pos, col, hasColor, faces, comments };
  }

  // Many phone apps (ARKit, RoomPlan, 3d Scanner App, Polycam) write Y-up coordinates.
  // The viewer uses Z-up. This rotates Y-up to Z-up in place: (x, y, z) -> (x, -z, y).
  function yUpToZUp(pos) {
    for (let i = 0; i < pos.length; i += 3) { const y = pos[i + 1], z = pos[i + 2]; pos[i + 1] = -z; pos[i + 2] = y; }
    return pos;
  }

  // EchoMap's own files (Livox mapper, camera scan) say so in a header comment and are Z-up.
  // Anything else, from iPhone LiDAR apps or ARKit, is assumed Y-up.
  const isYUp = cloud => !cloud.comments.some(c => /^EchoMap\b/.test(c) && !/y-up/i.test(c));

  const api = { parsePly, yUpToZUp, isYUp };
  if (typeof module !== 'undefined' && module.exports) module.exports = api; else root.EchoPly = api;
})(typeof self !== 'undefined' ? self : this);
