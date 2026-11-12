// SPDX-License-Identifier: MIT
// guide-covers-bom.mjs: every line of the parts list is used by the build guide, and the guide never takes more than
// the parts list buys. Exits 1 if a parts list id is used by no step, if a step names an id that does not exist, or if
// the pieces the steps take from a line are more than it buys (n x pack). Prints a table id -> steps.
// Run: bun checks/website/guide-covers-bom.mjs [SITE]   SITE (the folder with bom-data.js and guide-steps.js) defaults to
// docs/ in the repository this file sits in (checks/website/ is two folders below its root).
import { pathToFileURL } from "node:url";
import { resolve } from "node:path";

const dir = process.argv[2] ? pathToFileURL(resolve(process.argv[2]) + "/").href : new URL("../../docs/", import.meta.url).href;
const { ITEMS } = await import(dir + "bom-data.js");
const { STEPS, TOOLS, TOOL_BOM } = await import(dir + "guide-steps.js");

let bad = 0;
const fail = (msg) => { bad++; console.log("FAIL " + msg); };
const LINE = new Map();
for (const it of ITEMS) {
  if (!it.id) { fail(`parts list line without an id: ${it.part}`); continue; }
  if (LINE.has(it.id)) fail(`parts list id used twice: ${it.id}`);
  LINE.set(it.id, { it, buys: it.n * (it.pack || 1), took: 0, open: false, tool: false, steps: [] });
}
const use = (id, k, what) => {
  const L = LINE.get(id);
  if (!L) { fail(`step ${k} names "${id}", which is not in the parts list (${what})`); return null; }
  if (!L.steps.includes(k)) L.steps.push(k);
  return L;
};
for (const k in TOOL_BOM) if (!(k in TOOLS)) fail(`TOOL_BOM names the tool "${k}", which is not in TOOLS`);
STEPS.forEach((s, i) => {
  const k = i + 1;
  for (const p of s.parts || []) {
    if (!p.bom) continue;
    const L = use(p.bom, k, p.name);
    if (!L || p.reuse) continue;
    if (p.n == null) L.open = true;
    else L.took += p.n + (p.spare || 0);
  }
  for (const t of s.tools || []) {
    if (!(t in TOOLS)) fail(`step ${k} names the tool "${t}", which is not in TOOLS`);
    const L = TOOL_BOM[t] && use(TOOL_BOM[t], k, "tool " + t);
    if (L) L.tool = true;
  }
});

const rows = [["id", "steps", "takes", "buys", "part"]];
for (const [id, L] of LINE) {
  if (!L.steps.length) fail(`"${id}" (${L.it.part}) is used by no step`);
  if (L.took > L.buys) fail(`"${id}": the steps take ${L.took}, the parts list buys ${L.buys} (${L.it.n} x ${L.it.pack || 1})`);
  const takes = L.took ? L.took + (L.open ? " + ?" : "") : L.open ? "?" : L.tool ? "tool" : "0";
  rows.push([id, L.steps.join(", ") || "-", takes, String(L.buys), L.it.part.slice(0, 52)]);
}
const w = rows[0].map((_, c) => Math.max(...rows.map((r) => String(r[c]).length)));
for (const r of rows) console.log(r.map((x, c) => String(x).padEnd(w[c])).join("  ").trimEnd());
console.log(`\n${LINE.size} parts list lines, ${STEPS.length} steps. "?": a count the design does not give yet. Tools and reused pieces are not counted as taken.`);
if (bad) { console.log(`\n${bad} problem(s).`); process.exit(1); }
console.log("OK: every line is used, no step takes more than the parts list buys.");
