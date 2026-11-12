// SPDX-License-Identifier: MIT
// meshes-roundtrip.mjs: SITE/meshes.bin, read back through SITE/meshes.js, against MESHES_JSON. Exits 1 on any mismatch.
// Run: bun checks/website/meshes-roundtrip.mjs [SITE [MESHES_JSON]]   (after tools/website/pack_meshes.py)
//   SITE defaults to docs/ and MESHES_JSON to cad/viewer/meshes.json, both in the repository this file sits in
//   (checks/website/ is two folders below its root). Relative paths are taken from the current folder.
import { readFileSync, statSync } from "node:fs";
import { resolve } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const repo = new URL("../../", import.meta.url);
const site = process.argv[2] ? pathToFileURL(resolve(process.argv[2]) + "/") : new URL("docs/", repo);
const json = process.argv[3] ? resolve(process.argv[3]) : fileURLToPath(new URL("cad/viewer/meshes.json", repo));
const { loadMeshes } = await import(new URL("meshes.js", site).href);
const bin = new URL("meshes.bin", site);
const ref = JSON.parse(readFileSync(json, "utf8"));
const got = await loadMeshes(bin.href);   // the same path the browser takes, DecompressionStream included
const b64 = (s) => new Uint8Array(Buffer.from(s, "base64")).buffer;

let bad = 0, worst = 0, worstKey = "";
const fail = (msg) => { bad++; console.log("FAIL " + msg); };
const keys = Object.keys(ref);
if (Object.keys(got).join() !== keys.join()) fail("mesh keys differ: " + Object.keys(got).join());
for (const k of keys) {
  const r = ref[k], g = got[k];
  if (!g) continue;
  for (const f of ["nv", "nt", "idx_type"]) if (r[f] !== g[f]) fail(`${k}.${f} ${g[f]} != ${r[f]}`);
  const pos = new Float32Array(b64(r.pos));
  const idx = r.idx_type === "u32" ? new Uint32Array(b64(r.idx)) : new Uint16Array(b64(r.idx));
  if (g.pos.length !== pos.length || g.pos.length !== r.nv * 3) fail(`${k} pos length ${g.pos.length} != ${pos.length}`);
  if (g.idx.length !== idx.length || g.idx.length !== r.nt * 3) fail(`${k} idx length ${g.idx.length} != ${idx.length}`);
  if (g.idx.constructor !== idx.constructor) fail(`${k} idx is ${g.idx.constructor.name}, want ${idx.constructor.name}`);
  for (let i = 0; i < idx.length; i++) if (g.idx[i] !== idx[i]) { fail(`${k} idx[${i}] ${g.idx[i]} != ${idx[i]}`); break; }
  let e = 0;
  for (let i = 0; i < pos.length; i++) e = Math.max(e, Math.abs(g.pos[i] - pos[i]));
  if (!(e < 0.01)) fail(`${k} position error ${e} mm`);
  if (e > worst) { worst = e; worstKey = k; }
}
console.log(`meshes.json ${statSync(json).size} bytes, meshes.bin ${statSync(bin).size} bytes, ${keys.length} meshes`);
console.log(`largest position error ${worst.toFixed(6)} mm (${worstKey}), indices identical: ${bad ? "see FAIL lines" : "yes"}`);
console.log(bad ? `${bad} mismatch(es)` : "OK");
process.exit(bad ? 1 : 0);
