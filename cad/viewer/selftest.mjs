// SPDX-License-Identifier: MIT
// selftest.mjs: proves fk.js reproduces the CAD script (cad/takt.py) in FreeCAD. Run: node selftest.mjs (from any folder).
// Reference: reference.json from export_viewer.py (8 poses, 48 gait frames, 8 odd poses, and at every height the height study
// checked: the stand, the page's four other poses and all 48 gait frames). Exit 1 on any miss.
// Tolerances: joint angles 1e-7 deg; placement translations and foot points 1e-6 mm; placement rotation entries
// times 300 mm (larger than any part) 1e-6 mm; mass 1e-6 g; centre of mass 1e-6 mm; margins 1e-6 mm; push to tip
// 1e-8 g; femur and knee torques 1e-6 kg.cm. Rounded report.json values must match to their printed digits.
import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import * as fk from "./fk.js";

const HERE = dirname(fileURLToPath(import.meta.url));
const read = (p) => JSON.parse(readFileSync(p, "utf8"));
const model = read(join(HERE, "model.json"));
const ref = read(join(HERE, "reference.json"));
const meshes = read(join(HERE, "meshes.json"));
const report = read(join(HERE, "..", "report.json"));

const fails = [];
const check = (ok, msg) => { if (!ok) fails.push(msg); };
const W = { joint: 0, trans: 0, rot: 0, foot: 0, mass: 0, com: 0, margin: 0, push: 0, torque: 0, rounded: 0 };
const worst = (k, v) => { if (v > W[k]) W[k] = v; return v; };
const TOL = { joint: 1e-7, trans: 1e-6, rot: 1e-6, foot: 1e-6, mass: 1e-6, com: 1e-6, margin: 1e-6, push: 1e-8, torque: 1e-6 };
const PART_RADIUS = 300;

// 1. constants
const C = model.C;
for (const k of ["R_HIP", "L_COXA", "Z_F", "L_FEMUR", "L_TIBIA", "R_TIP", "S_MID", "Z_S1", "S3_DIR", "COXA_RANGE", "STAND_HB", "STAND_R", "GAIT_R", "RATED", "STALL"]) {
  check(fk[k] === C[k], `constant ${k}: fk.js ${fk[k]} vs FreeCAD ${C[k]}`);
}
check(JSON.stringify(fk.HIPS) === JSON.stringify(C.HIPS), "HIPS differ");
check(JSON.stringify(fk.TH2_RANGE) === JSON.stringify(C.TH2_RANGE) && JSON.stringify(fk.BETA_RANGE) === JSON.stringify(C.BETA_RANGE), "ranges differ");
check(JSON.stringify(fk.TRI_A) === JSON.stringify(C.TRI_A) && JSON.stringify(fk.TRI_B) === JSON.stringify(C.TRI_B), "tripods differ");
check(JSON.stringify(fk.GAIT) === JSON.stringify(C.GAIT), "gait parameters differ");
check(JSON.stringify(fk.BELLY) === JSON.stringify(C.BELLY), "belly plate differs");
for (const inst of model.instances) if (inst.render) check(meshes[inst.key], `no mesh for ${inst.key}`);

function cmpJoints(tag, got, want) {
  for (const n of fk.LEGS) for (let i = 0; i < 3; i++) check(worst("joint", Math.abs(got[n][i] - want[n][i])) < TOL.joint, `${tag}: ${n} joint ${i} ${got[n][i]} vs ${want[n][i]}`);
}
function cmpMatrix(tag, got, want) {
  for (let i = 0; i < 16; i++) {
    const d = Math.abs(got[i] - want[i]);
    if (i === 3 || i === 7 || i === 11) check(worst("trans", d) < TOL.trans, `${tag}: translation ${i} off by ${d}`);
    else check(worst("rot", d * PART_RADIUS) < TOL.rot, `${tag}: rotation ${i} off by ${d}`);
  }
}
function cmpSupport(tag, got, want) {
  check(worst("margin", Math.abs(got.margin - want.margin)) < TOL.margin, `${tag}: margin ${got.margin} vs ${want.margin}`);
  check(worst("push", Math.abs(got.push_g - want.push_g)) < TOL.push, `${tag}: push ${got.push_g} vs ${want.push_g}`);
  for (const j of ["femur", "knee"]) check(worst("torque", Math.abs(got[j] - want[j])) < TOL.torque, `${tag}: ${j} ${got[j]} vs ${want[j]}`);
  for (const n of fk.LEGS) for (const j of ["femur", "knee"]) check(worst("torque", Math.abs(got.per_leg[n][j] - want.per_leg[n][j])) < TOL.torque, `${tag}: ${n} ${j}`);
  check(Math.abs(got.max_foot_load_g - want.max_foot_load_g) < 1e-6, `${tag}: foot load ${got.max_foot_load_g} vs ${want.max_foot_load_g}`);
  check(JSON.stringify(got.legs) === JSON.stringify(want.legs), `${tag}: support legs differ`);
}

// 2. every reference pose: rebuild it the way the page does, then compare everything
const presets = Object.fromEntries(model.presets.map((p) => [p.id, p]));
const rows = [];
for (const r of ref.poses) {
  let pose, stance = null;
  if (r.kind === "pose") {
    const pr = presets[r.id.slice(5)];
    check(pr, `${r.id}: no preset`);
    pose = fk.presetPose(model, pr);
    const raw = fk.posture(r.B, r.targets, r.contact, r.on);      // the exported targets give the same legs from the FreeCAD body frame
    for (const [n, j] of Object.entries(pr.overrides || {})) raw.joints[n] = [...j];
    cmpJoints(r.id + " (raw targets)", raw.joints, r.joints);
  } else if (r.kind === "gait") {
    const g = fk.gaitPose(r.gait.kind, r.gait.k);
    for (const n of fk.LEGS) check(worst("foot", Math.max(...g.feet[n].map((v, i) => Math.abs(v - r.gait.targets[n][i])))) < TOL.foot, `${r.id}: gait target ${n}`);
    check(JSON.stringify(g.stance) === JSON.stringify(r.gait.stance), `${r.id}: stance ${g.stance} vs ${r.gait.stance}`);
    check(JSON.stringify(g.pose.on) === JSON.stringify(r.on), `${r.id}: legs on floor differ`);
    pose = g.pose; stance = g.stance;
  } else if (r.kind === "hstand" || r.kind === "hpose") {
    pose = fk.presetPose(model, fk.presetAt(presets[r.kind === "hstand" ? "stand" : r.preset], r.hb));   // the page's preset at that height
  } else if (r.kind === "hgait") {
    const ga = fk.gaitAt(r.hb, model.height);
    check(ga.checked && ga.checked.hb === r.hb, `${r.id}: ${r.hb} mm is not a grid height`);
    const g = fk.gaitPose(r.gait.kind, r.gait.k, ga.g, r.hb, ga.gaitR);
    for (const n of fk.LEGS) check(worst("foot", Math.max(...g.feet[n].map((v, i) => Math.abs(v - r.gait.targets[n][i])))) < TOL.foot, `${r.id}: gait target ${n}`);
    check(JSON.stringify(g.stance) === JSON.stringify(r.gait.stance), `${r.id}: stance ${g.stance} vs ${r.gait.stance}`);
    check(JSON.stringify(g.pose.on) === JSON.stringify(r.on), `${r.id}: legs on floor differ`);
    pose = g.pose; stance = g.stance;
  } else if (r.targets) {
    pose = fk.posture(fk.bodyFrame(r.body), r.targets, r.contact, r.on);
  } else {
    pose = { B: fk.bodyFrame(r.body), joints: r.joints, on: r.on };       // forward kinematics only
  }
  cmpMatrix(r.id + " body", pose.B, r.B);
  cmpJoints(r.id, pose.joints, r.joints);
  const contact = fk.contactOf(model, r.variant);
  const pf = fk.poseFrames(pose, contact);
  for (const n of fk.LEGS) check(worst("foot", Math.max(...pf.feet[n].map((v, i) => Math.abs(v - r.feet[n][i])))) < TOL.foot, `${r.id}: foot ${n}`);
  const pl = fk.placements(model, r.variant, pf);
  const hasPl = Object.keys(r.placements).length > 0;         // height poses and frames carry no placements (file size)
  check(!hasPl || Object.keys(pl).length === Object.keys(r.placements).length, `${r.id}: ${Object.keys(pl).length} vs ${Object.keys(r.placements).length} placements`);
  for (const [name, m] of Object.entries(r.placements)) {
    if (!pl[name]) { fails.push(`${r.id}: no placement ${name}`); continue; }
    cmpMatrix(`${r.id} ${name}`, pl[name], m);
  }
  const L = fk.loads(model, r.variant, pf);
  check(worst("mass", Math.abs(L.total - r.mass)) < TOL.mass, `${r.id}: mass ${L.total} vs ${r.mass}`);
  check(worst("com", Math.max(...L.com.map((v, i) => Math.abs(v - r.com[i])))) < TOL.com, `${r.id}: com ${L.com} vs ${r.com}`);
  cmpSupport(r.id, L.support(stance || r.on), r.support);
  if (r.tripod_A) { cmpSupport(r.id + " tripod A", L.support(fk.TRI_A), r.tripod_A); cmpSupport(r.id + " tripod B", L.support(fk.TRI_B), r.tripod_B); }
  if (r.resting) {
    const R = L.resting();
    check(worst("margin", Math.abs(R.margin - r.resting.margin)) < TOL.margin, `${r.id}: resting margin`);
    for (const j of ["femur", "knee"]) check(worst("torque", Math.abs(R[j] - r.resting[j])) < TOL.torque, `${r.id}: resting ${j}`);
  }
  if (r.limits) check(JSON.stringify(fk.limitsBad(pose.joints).map(([n, j]) => n + j)) === JSON.stringify(r.limits.map(([n, j]) => n + j)), `${r.id}: page limits ${JSON.stringify(fk.limitsBad(pose.joints))} vs FreeCAD ${JSON.stringify(r.limits)}`);
  else check(JSON.stringify(fk.limitsBad(pose.joints)) === "[]" || r.kind === "odd", `${r.id}: joints outside the ranges`);
  if (r.kind === "pose" || r.kind === "gait") rows.push([r.id, L.total, L.support(stance || r.on), r.resting ? L.resting() : null]);
}

// 3. the rounded numbers in report.json (the CAD script's main) come out of fk.js too
const near = (a, b, digits) => Math.abs(a - b) <= 0.5 * 10 ** -digits + 1e-9;
for (const [name, rp] of Object.entries(report.poses)) {
  const pr = presets[name];
  const pose = fk.presetPose(model, pr);
  const st = fk.statics(model, pr.variant, pose, { rest: pr.rest_on_belly });
  const s = rp.statics;
  const chk = (ok, what) => { check(ok, `report ${name}: ${what}`); };
  chk(Math.round(st.total) === s.mass_g, `mass ${st.total} vs ${s.mass_g}`);
  chk(near(st.com[0], s.com_mm[0], 1) && near(st.com[1], s.com_mm[1], 1) && near(st.com[2], s.com_mm[2], 1), `com ${st.com} vs ${s.com_mm}`);
  for (const [k, v] of Object.entries(rp.joints)) for (let i = 0; i < 3; i++) chk(near(pose.joints[k][i], v[i], 1), `joint ${k} ${i}`);
  const L = st.L;
  const blocks = pr.rest_on_belly ? [["stand_up_six", L.support(pr.on)]] : [["support", L.support(pr.on)]].concat(s.tripod_A ? [["tripod_A", L.support(fk.TRI_A)], ["tripod_B", L.support(fk.TRI_B)]] : []);
  for (const [k, sup] of blocks) {
    chk(near(sup.margin, s[k].margin_mm, 1), `${k} margin ${sup.margin} vs ${s[k].margin_mm}`);
    chk(near(sup.push_g, s[k].push_g, 2), `${k} push ${sup.push_g} vs ${s[k].push_g}`);
    chk(near(sup.femur, s[k].femur.kgcm, 2) && near(sup.knee, s[k].knee.kgcm, 2), `${k} torques ${sup.femur}/${sup.knee} vs ${s[k].femur.kgcm}/${s[k].knee.kgcm}`);
    // the CAD script's pct(): the torque already rounded to 0.01 kg.cm, then Python's round() (half to even)
    const pyRound = (x) => { const f = Math.floor(x), d = x - f; return d === 0.5 ? (f % 2 === 0 ? f : f + 1) : Math.round(x); };   // Python's round(): ties only when exact
    const pct = (v) => pyRound(Math.round(v * 100) / 100 / fk.RATED * 100);
    chk(pct(sup.femur) === s[k].femur.pct_rated_6v && pct(sup.knee) === s[k].knee.pct_rated_6v, `${k} percent of rated ${pct(sup.femur)}/${pct(sup.knee)} vs ${s[k].femur.pct_rated_6v}/${s[k].knee.pct_rated_6v}`);
    W.rounded = Math.max(W.rounded, Math.abs(sup.margin - s[k].margin_mm), Math.abs(sup.femur - s[k].femur.kgcm));
  }
  if (pr.rest_on_belly) {
    chk(near(L.resting().margin, s.resting.margin_mm, 1) && near(L.resting().femur, s.resting.femur.kgcm, 2), "resting");
  }
}
for (const kind of ["walk", "turn"]) {
  const sum = report.gait[kind].summary;
  let mf = 0, mk = 0, mm = Infinity, mp = Infinity;
  for (let k = 0; k < 24; k++) {
    const g = fk.gaitPose(kind, k);
    const st = fk.statics(model, "claw", g.pose, { stance: g.stance });
    mf = Math.max(mf, st.sup.femur); mk = Math.max(mk, st.sup.knee); mm = Math.min(mm, st.sup.margin); mp = Math.min(mp, st.sup.push_g);
    check(fk.limitsBad(g.pose.joints).length === 0, `${kind} ${k}: joints outside the ranges`);
  }
  check(near(mf, sum.femur.kgcm, 2) && near(mk, sum.knee.kgcm, 2), `report ${kind}: torques ${mf}/${mk} vs ${sum.femur.kgcm}/${sum.knee.kgcm}`);
  check(near(mm, sum.min_margin_mm, 1) && near(mp, sum.min_push_g, 2), `report ${kind}: margin ${mm} / push ${mp}`);
  rows.push([`${kind} worst of 24`, null, { margin: mm, push_g: mp, femur: mf, knee: mk }, null]);
}

// 3a. at every height the height study checked, the page's 24 walk and 24 turn frames give the study's rounded worst numbers
const pyRound = (x) => { const f = Math.floor(x), d = x - f; return d === 0.5 ? (f % 2 === 0 ? f : f + 1) : Math.round(x); };   // Python's round(): ties only when exact
const pctOf = (v) => pyRound(Math.round(v * 100) / 100 / fk.RATED * 100);
const hrows = [];
for (const row of model.height.grid) {
  const ga = fk.gaitAt(row.hb, model.height);
  check(ga.gaitR === row.gait_r && ga.g.lift === row.lift && ga.g.out === row.out, `height ${row.hb}: gaitAt gives ${ga.gaitR}/${ga.g.lift}/${ga.g.out}`);
  for (const kind of ["walk", "turn"]) {
    let mf = 0, mk = 0, mm = Infinity, mp = Infinity;
    for (let k = 0; k < 24; k++) {
      const g = fk.gaitPose(kind, k, ga.g, row.hb, ga.gaitR);
      const st = fk.statics(model, "claw", g.pose, { stance: g.stance });
      mf = Math.max(mf, st.sup.femur); mk = Math.max(mk, st.sup.knee); mm = Math.min(mm, st.sup.margin); mp = Math.min(mp, st.sup.push_g);
    }
    check(pctOf(mf) === row[kind].femur && pctOf(mk) === row[kind].knee, `height ${row.hb} ${kind}: ${pctOf(mf)}/${pctOf(mk)} % vs the height study ${row[kind].femur}/${row[kind].knee}`);
    check(near(mm, row[kind].margin, 1) && near(mp, row[kind].push, 2), `height ${row.hb} ${kind}: margin ${mm} / push ${mp} vs ${row[kind].margin} / ${row[kind].push}`);
    hrows.push([`${row.hb} mm ${kind}`, mm, mp, pctOf(mf), pctOf(mk)]);
  }
}
{ // between grid heights the gait is the straight blend of its neighbours; outside the grid the ends hold
  const [a, b] = model.height.grid.slice(1, 3), mid = (a.hb + b.hb) / 2, ga = fk.gaitAt(mid, model.height);
  check(!ga.checked && Math.abs(ga.gaitR - (a.gait_r + b.gait_r) / 2) < 1e-9 && Math.abs(ga.g.out - (a.out + b.out) / 2) < 1e-9, `gaitAt(${mid}) is not the blend`);
  const lo = model.height.grid[0], top = model.height.grid.at(-1);
  check(fk.gaitAt(lo.hb - 10, model.height).gaitR === lo.gait_r && fk.gaitAt(top.hb + 10, model.height).g.lift === top.lift, "gaitAt does not hold the ends");
  check(fk.presetAt(presets.sit, 95) === presets.sit && fk.presetAt(presets.stand, 95).body.hb === 95, "presetAt: sit must stay on its belly, stand must follow");
}

// 3b. the page's floor contact finds exactly the standing legs of every preset (claw and sneaker)
for (const pr of model.presets) {
  const pf = fk.poseFrames(fk.presetPose(model, pr), fk.contactOf(model, pr.variant));
  const on = fk.legsOnFloor(pf, fk.contactZ(model, pr.variant));
  check(JSON.stringify(on) === JSON.stringify(pr.on), `${pr.id}: page sees ${on} on the floor, FreeCAD ${pr.on}`);
  const st = fk.statics(model, pr.variant, fk.presetPose(model, pr));
  check(st.below.length === 0 && st.limits.length === 0, `${pr.id}: page flags ${st.below} below the floor or ${JSON.stringify(st.limits)}`);
  check((st.kind === "belly") === pr.rest_on_belly, `${pr.id}: belly support ${st.kind}`);
}

// 4. IK refuses an unreachable foot; moving the body by zero keeps the legs; the sneaker preset carries report.json's height
let threw = false;
try { fk.legIk(1, [400, 0, 0], fk.T(0, 0, 60)); } catch (e) { threw = e instanceof fk.IKError; }
check(threw, "legIk accepted an unreachable foot");
{
  const pr = presets.stand, pose = fk.presetPose(model, pr);
  const moved = fk.moveBody(pose, pr.body, pr.contact, fk.LEGS, fk.contactZ(model, pr.variant));
  cmpJoints("moveBody identity", moved.joints, pose.joints);
  check(Math.round(presets.sneaker_stand.body.hb * 10) / 10 === report.sneaker_hb_mm, `sneaker body height ${presets.sneaker_stand.body.hb} vs report ${report.sneaker_hb_mm}`);
}

console.log("pose                 mass g    margin mm   push g   femur kg.cm   knee kg.cm");
for (const [id, m, s, rest] of rows) {
  const tag = rest ? `${id} (belly ${rest.margin.toFixed(1)})` : id;
  console.log(`${tag.padEnd(22)} ${m ? m.toFixed(1).padStart(7) : "       "}  ${s.margin.toFixed(2).padStart(9)}  ${s.push_g.toFixed(3).padStart(7)}  ${s.femur.toFixed(3).padStart(12)}  ${s.knee.toFixed(3).padStart(11)}`);
}
console.log("\nheight      worst of 24    margin mm   push g   femur %   knee %");
for (const [id, m, p, f, k] of hrows) console.log(`${id.padEnd(22)}  ${m.toFixed(2).padStart(9)}  ${p.toFixed(3).padStart(7)}  ${String(f).padStart(7)}  ${String(k).padStart(7)}`);
const nh = (k) => ref.poses.filter((p) => p.kind === k).length;
console.log(`\nreference poses: ${ref.poses.length} (8 poses, 48 gait frames, ${nh("odd")} odd; at ${model.height.grid.length} heights ${nh("hstand")} stands, ${nh("hpose")} poses, ${nh("hgait")} gait frames), instances: ${model.instances.length}`);
console.log(`worst joint angle   ${W.joint.toExponential(2)} deg   (limit ${TOL.joint})`);
console.log(`worst translation   ${W.trans.toExponential(2)} mm    (limit ${TOL.trans})`);
console.log(`worst rotation      ${W.rot.toExponential(2)} mm at 300 mm (limit ${TOL.rot})`);
console.log(`worst foot point    ${W.foot.toExponential(2)} mm    (limit ${TOL.foot})`);
console.log(`worst mass          ${W.mass.toExponential(2)} g     (limit ${TOL.mass})`);
console.log(`worst centre of mass ${W.com.toExponential(2)} mm   (limit ${TOL.com})`);
console.log(`worst margin        ${W.margin.toExponential(2)} mm    (limit ${TOL.margin})`);
console.log(`worst push to tip   ${W.push.toExponential(2)} g     (limit ${TOL.push})`);
console.log(`worst torque        ${W.torque.toExponential(2)} kg.cm (limit ${TOL.torque})`);
console.log(`report.json         ${Object.keys(report.poses).length} poses and both gait summaries match to their printed digits`);
if (fails.length) {
  console.error(`\nSELFTEST FAIL (${fails.length})`);
  for (const f of fails.slice(0, 40)) console.error("  " + f);
  process.exit(1);
}
console.log("\nSELFTEST PASS");
