// SPDX-License-Identifier: MIT
// meshes.js: reads meshes.bin (written by tools/pack_meshes.py; the file layout is described there). Plain ES module, no
// dependencies. loadMeshes() returns key -> { nv, nt, idx_type, pos: Float32Array, idx: Uint16Array or Uint32Array },
// the same arrays meshes.json holds as base64, positions within half a quantization step (about 0.001 mm).
// The file is gzip on its own (hosts do not compress unknown binary files); the browser's DecompressionStream opens it.

export async function loadMeshes(url = "./meshes.bin") {
  const r = await fetch(url);
  if (!r.ok) throw new Error(url + " " + r.status);
  const ds = r.body.pipeThrough(new DecompressionStream("gzip"));
  return decodeMeshes(new Uint8Array(await new Response(ds).arrayBuffer()));
}

// The decompressed bytes -> the meshes.
function decodeMeshes(b) {
  const n = b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24);
  const header = JSON.parse(new TextDecoder().decode(b.subarray(4, 4 + n)));
  let p = 4 + n;
  const out = {};
  for (const h of header) {
    const pos = new Float32Array(h.nv * 3);
    for (let a = 0; a < 3; a++) {
      const mn = h.min[a], sc = h.scale[a];
      let q = 0;
      for (let i = a; i < pos.length; i += 3) {
        // one zigzag varint: the delta to the previous vertex on this axis
        let z = 0, s = 0, c;
        do { c = b[p++]; z += (c & 0x7f) * 2 ** s; s += 7; } while (c & 0x80);
        q += z % 2 ? -(z + 1) / 2 : z / 2;
        pos[i] = mn + q * sc;
      }
    }
    const idx = h.idx_type === "u32" ? new Uint32Array(h.nt * 3) : new Uint16Array(h.nt * 3);
    let v = 0;
    for (let i = 0; i < idx.length; i++) {
      let z = 0, s = 0, c;
      do { c = b[p++]; z += (c & 0x7f) * 2 ** s; s += 7; } while (c & 0x80);
      v += z % 2 ? -(z + 1) / 2 : z / 2;
      idx[i] = v;
    }
    out[h.key] = { nv: h.nv, nt: h.nt, idx_type: h.idx_type, pos, idx };
  }
  if (p !== b.length) throw new Error("meshes.bin: " + (b.length - p) + " bytes left over");
  return out;
}
