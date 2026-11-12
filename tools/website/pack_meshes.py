#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# pack_meshes.py: meshes.json -> meshes.bin, the same 19 meshes in about a quarter of the size. Standard library only.
# Run: python3 tools/website/pack_meshes.py [SOURCE [TARGET]]   (meshes.js reads the file, meshes-roundtrip.mjs checks it)
#   SOURCE defaults to cad/viewer/meshes.json and TARGET to docs/meshes.bin, both in the repository this file sits in
#   (tools/website/ is two folders below its root). Relative paths are taken from the current folder.
#
# File layout, the whole file gzip level 9 (hosts do not compress unknown binary files, so it carries its own):
#   4 bytes   header length n, uint32 little-endian
#   n bytes   header, UTF-8 JSON: [{ key, nv, nt, idx_type, min: [x, y, z], scale: [x, y, z], pos_len, idx_len }, ...]
#   then per mesh, in header order: pos_len bytes of positions, idx_len bytes of indices
# Positions: each axis quantized to 16 bits per mesh, q = round((v - min) / scale), min and scale exact float32 values;
#   stored axis by axis (all x, then all y, then all z), each as zigzag varints of the delta to the previous vertex.
#   Decoded v = min + q * scale; off by at most scale / 2 (about 0.001 mm on the largest part).
# Indices: unchanged order, zigzag varints of the delta to the previous index.
import base64, gzip, json, os, struct, sys

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
SRC = os.path.join(REPO, "cad", "viewer", "meshes.json")
OUT = os.path.join(REPO, "docs", "meshes.bin")


def f32(x):
  return struct.unpack("<f", struct.pack("<f", x))[0]


def varints(values, out):
  prev = 0
  for v in values:
    d = v - prev
    prev = v
    z = (d << 1) ^ (d >> 63)   # zigzag: small negative and positive deltas both become small numbers
    while z >= 0x80:
      out.append((z & 0x7F) | 0x80)
      z >>= 7
    out.append(z)


def main(argv):
  if len(argv) > 2:
    sys.exit("usage: pack_meshes.py [SOURCE [TARGET]]")
  src = argv[0] if len(argv) > 0 else SRC
  out = argv[1] if len(argv) > 1 else OUT
  meshes = json.load(open(src))
  header, streams = [], []
  for key, m in meshes.items():
    nv, nt = m["nv"], m["nt"]
    raw = base64.b64decode(m["pos"])
    pos = struct.unpack("<%df" % (nv * 3), raw)
    fmt = "<%dI" % (nt * 3) if m["idx_type"] == "u32" else "<%dH" % (nt * 3)
    idx = struct.unpack(fmt, base64.b64decode(m["idx"]))
    lo, sc, ps = [], [], bytearray()
    for a in range(3):
      col = pos[a::3]
      mn = f32(min(col))
      s = f32((max(col) - mn) / 65535) if max(col) > mn else 0.0
      q = [min(65535, max(0, round((v - mn) / s))) if s else 0 for v in col]
      varints(q, ps)
      lo.append(mn)
      sc.append(s)
    ix = bytearray()
    varints(idx, ix)
    header.append({"key": key, "nv": nv, "nt": nt, "idx_type": m["idx_type"], "min": lo, "scale": sc,
                   "pos_len": len(ps), "idx_len": len(ix)})
    streams += [ps, ix]
  head = json.dumps(header, separators=(",", ":")).encode()
  body = struct.pack("<I", len(head)) + head + b"".join(streams)
  data = gzip.compress(body, 9, mtime=0)
  open(out, "wb").write(data)
  print("%s %d bytes -> %s %d bytes (%d before gzip), %d meshes"
        % (src, os.path.getsize(src), out, len(data), len(body), len(header)))


if __name__ == "__main__":
  sys.exit(main(sys.argv[1:]))
