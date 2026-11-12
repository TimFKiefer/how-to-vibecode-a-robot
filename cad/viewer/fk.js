// SPDX-License-Identifier: MIT
// fk.js: port of the CAD script's (cad/takt.py) leg frames, leg IK, posture, gait, centre of mass and statics (class Loads, hull_margin,
// share_load). Plain ES module, no DOM. Matrices are row-major arrays of 16 numbers, the layout of FreeCAD's Matrix.A.
// Lengths mm, angles deg, masses g, torques kg.cm. Body frame: X right, Y front, Z up, floor z = 0.
// selftest.mjs checks every function here against FreeCAD (reference.json, written by export_viewer.py).

// --- CAD script constants (selftest compares them with model.json C) ---
export const R_HIP = 74.0, L_COXA = 33.2, Z_F = 44.0, L_FEMUR = 66.0, L_TIBIA = 120.0, R_TIP = 4.0;
export const S_MID = 14.5, Z_S1 = 12.0, S3_DIR = 20.0;
export const HIPS = [["RF", 60.0, 15.0], ["RM", 0.0, 0.0], ["RR", -60.0, -15.0], ["LF", 120.0, -15.0], ["LM", 180.0, 0.0], ["LR", 240.0, 15.0]];
export const LEGS = HIPS.map((h) => h[0]);
export const TRI_A = ["RF", "LM", "RR"], TRI_B = ["LF", "RM", "LR"];
export const COXA_RANGE = 40.0, TH2_RANGE = [-20.0, 100.0], BETA_RANGE = [70.0, 155.0];
export const STAND_HB = 62.0, STAND_R = 68.2, GAIT_R = 83.2;
export const RATED = 4.0, STALL = 16.5;         // kg.cm at 6 V (Feetech STS3215 spec 5-8 and 5-4)
export const GAIT = { n: 24, stride: 50.0, lift: 50.0, out: 0.0, turn: 16.0 };
export const BELLY = [40.0, 27.0, -2.0];        // belly plate corners (+-x, +-y, z) in the body frame
const DEG = Math.PI / 180;

// --- 4x4 matrices ---
export function I4() { return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]; }
export function mul(a, b) {
  const o = new Array(16);
  for (let r = 0; r < 4; r++) {
    for (let c = 0; c < 4; c++) o[r * 4 + c] = a[r * 4] * b[c] + a[r * 4 + 1] * b[4 + c] + a[r * 4 + 2] * b[8 + c] + a[r * 4 + 3] * b[12 + c];
  }
  return o;
}
export function chain(...ms) { let o = I4(); for (const m of ms) o = mul(o, m); return o; }
export function T(x = 0, y = 0, z = 0) { return [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1]; }
export function Rx(d) { const c = Math.cos(d * DEG), s = Math.sin(d * DEG); return [1, 0, 0, 0, 0, c, -s, 0, 0, s, c, 0, 0, 0, 0, 1]; }
export function Rz(d) { const c = Math.cos(d * DEG), s = Math.sin(d * DEG); return [c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]; }
export function apply(m, v) {
  return [m[0] * v[0] + m[1] * v[1] + m[2] * v[2] + m[3], m[4] * v[0] + m[5] * v[1] + m[6] * v[2] + m[7], m[8] * v[0] + m[9] * v[1] + m[10] * v[2] + m[11]];
}
/** Inverse of a rigid transform, as FreeCAD's Matrix.inverse() gives for these. */
export function invRigid(m) {
  const r = [m[0], m[4], m[8], m[1], m[5], m[9], m[2], m[6], m[10]], t = [m[3], m[7], m[11]];
  return [r[0], r[1], r[2], -(r[0] * t[0] + r[1] * t[1] + r[2] * t[2]), r[3], r[4], r[5], -(r[3] * t[0] + r[4] * t[1] + r[5] * t[2]),
    r[6], r[7], r[8], -(r[6] * t[0] + r[7] * t[1] + r[8] * t[2]), 0, 0, 0, 1];
}
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

// --- body and legs ---
/** pitch_about(): body frame for height hb, nose-up pitch and yaw about a point zc above the belly, shifted by (x, y). */
export function pitchAbout(hb, pitch = 0, yaw = 0, zc = 40.0, shift = [0, 0]) {
  return chain(T(shift[0], shift[1], hb + zc), Rz(yaw), Rx(pitch), T(0, 0, -zc));
}
export function bodyFrame(body) { return pitchAbout(body.hb, body.pitch || 0, body.yaw || 0, 40.0, [body.x || 0, body.y || 0]); }

/** leg_frames(): every frame of leg i for coxa gamma, femur th2 (up from horizontal) and knee bend beta. */
export function legFrames(i, gamma, th2, beta, B) {
  const [, phi, delta] = HIPS[i];
  const H = chain(B, Rz(phi), T(R_HIP, 0, 0));
  const C = chain(H, Rz(gamma));
  const fa = chain(C, T(L_COXA, 0, Z_F), Rx(-90));
  const Fr = chain(fa, Rz(-th2));
  return {
    s1: chain(H, Rz(delta), T(0, 0, Z_S1)), coxa: C, s2: chain(fa, T(0, 0, -S_MID), Rz(-90)), femur: Fr,
    s3: chain(Fr, T(L_FEMUR, 0, 0), Rz(beta), Rz(S3_DIR), T(0, 0, -S_MID)), tibia: chain(Fr, T(L_FEMUR, 0, 0), Rz(beta)),
  };
}

export class IKError extends Error {}

/** leg_ik(): joint angles that put the foot contact point (tibia frame) on `foot` (world). */
export function legIk(i, foot, B, contact = [L_TIBIA, 0]) {
  const [name, phi] = HIPS[i];
  const H = chain(B, Rz(phi), T(R_HIP, 0, 0));
  const p = apply(invRigid(H), foot);
  const gamma = Math.atan2(p[1], p[0]) / DEG;
  const r = Math.hypot(p[0], p[1]);
  const dx = r - L_COXA, dz = p[2] - Z_F;
  const D = Math.hypot(dx, dz);
  const L2 = Math.hypot(contact[0], contact[1]);
  const off = Math.atan2(contact[1], contact[0]) / DEG;
  const c = (L_FEMUR ** 2 + D ** 2 - L2 ** 2) / (2 * L_FEMUR * D);
  if (!(Math.abs(c) <= 1)) throw new IKError(`${name} außer Reichweite`);
  const th2 = (Math.atan2(dz, dx) + Math.acos(c)) / DEG;
  const kx = L_COXA + L_FEMUR * Math.cos(th2 * DEG), kz = Z_F + L_FEMUR * Math.sin(th2 * DEG);
  const tib = Math.atan2(p[2] - kz, r - kx) / DEG;
  return [gamma, th2, th2 - tib - off];
}

/** posture(): legs by IK so each contact point sits on its target. Returns { B, joints: {leg: [g, t, b]}, on }. */
export function posture(B, feet, contact = [L_TIBIA, 0], on = null) {
  const joints = {};
  HIPS.forEach((h, i) => { joints[h[0]] = legIk(i, feet[h[0]], B, contact); });
  return { B, joints, on: on ? [...on] : [...LEGS] };
}

/** spots(): contact targets straight out from each hip, foot_r mm beyond the hip, tip centre at R_TIP. */
export function spots(footR) {
  const out = {};
  for (const [n, phi] of HIPS) {
    const a = phi * DEG;
    out[n] = [(R_HIP + footR) * Math.cos(a), (R_HIP + footR) * Math.sin(a), R_TIP];
  }
  return out;
}

/** gait_feet(): tripod gait frame k (any real k; FreeCAD checked the integers 0..23). Returns { feet, stance }. */
export function gaitFeet(kind, k, g = GAIT, base = spots(GAIT_R)) {
  const p = k / g.n;
  const feet = {}, stance = [];
  for (const [name] of HIPS) {
    const ph = (p + (TRI_A.includes(name) ? 0.0 : 0.5)) % 1.0;
    let s, z, u = 0;
    if (ph < 0.5) { s = 0.5 - ph / 0.5; z = 0.0; stance.push(name); }
    else { u = (ph - 0.5) / 0.5; s = -0.5 + (0.5 - 0.5 * Math.cos(Math.PI * u)); z = g.lift * Math.sin(Math.PI * u) ** 2; }
    let h = base[name];
    if (ph >= 0.5) {
      const r = Math.hypot(h[0], h[1]);
      const o = g.out * Math.sin(Math.PI * u) ** 0.7;
      h = [h[0] + h[0] / r * o, h[1] + h[1] / r * o, h[2]];
    }
    if (kind === "walk") feet[name] = [h[0], h[1] + s * g.stride, h[2] + z];
    else {
      const a = s * g.turn * DEG;
      feet[name] = [h[0] * Math.cos(a) - h[1] * Math.sin(a), h[0] * Math.sin(a) + h[1] * Math.cos(a), h[2] + z];
    }
  }
  return { feet, stance };
}

/** limits_ok(): joints outside the swept ranges, [leg, "coxa"|"femur"|"knee", value]. */
export function limitsBad(joints) {
  const bad = [];
  HIPS.forEach(([n, , delta]) => {
    const [g, t, b] = joints[n];
    if (Math.abs(g - delta) > COXA_RANGE + 1e-6) bad.push([n, "coxa", g]);
    if (!(TH2_RANGE[0] - 1e-6 <= t && t <= TH2_RANGE[1] + 1e-6)) bad.push([n, "femur", t]);
    if (!(BETA_RANGE[0] - 1e-6 <= b && b <= BETA_RANGE[1] + 1e-6)) bad.push([n, "knee", b]);
  });
  return bad;
}

// --- frames, feet, placements ---
export function contactOf(model, variant) { return variant === "sneaker" ? model.C.SNEAKER.contact : [L_TIBIA, 0]; }
/** Height of the contact point when the foot stands: the claw tip centre, or the settled sneaker sole point. */
export function contactZ(model, variant) { return variant === "sneaker" ? model.C.SNEAKER.contact_z : R_TIP; }

export function poseFrames(pose, contact) {
  const frames = {}, feet = {};
  HIPS.forEach(([n], i) => {
    const [g, t, b] = pose.joints[n];
    frames[n] = legFrames(i, g, t, b, pose.B);
    feet[n] = apply(frames[n].tibia, [contact[0], contact[1], 0]);
  });
  return { B: pose.B, frames, feet };
}

export function active(inst, variant) { return inst.variant === null || inst.variant === variant; }

/** Placement of every instance of the variant: its frame times its fixed offset. */
export function placements(model, variant, pf) {
  const out = {};
  for (const inst of model.instances) {
    if (!active(inst, variant)) continue;
    let F;
    if (inst.frame === "body") F = pf.B;
    else { const [, leg, link] = inst.frame.split(":"); F = pf.frames[leg][link]; }
    out[inst.name] = inst.offset ? mul(F, inst.offset) : F;
  }
  return out;
}

// --- balance ---
const round6 = (x) => Number(x.toFixed(6));
/** hull2d(): counter-clockwise hull of points rounded to 1e-6 mm, the same monotone chain as the CAD script. */
export function hull2d(points) {
  const seen = new Set(), pts = [];
  for (const q of points) {
    const p = [round6(q[0]), round6(q[1])], key = p[0] + "," + p[1];
    if (!seen.has(key)) { seen.add(key); pts.push(p); }
  }
  pts.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const half = (seq) => {
    const out = [];
    for (const p of seq) {
      while (out.length >= 2 && cr(out[out.length - 2], out[out.length - 1], p) <= 0) out.pop();
      out.push(p);
    }
    return out.slice(0, -1);
  };
  return half(pts).concat(half(pts.slice().reverse()));
}

/** Distance from p to the edge of the foot polygon; negative means it would tip. */
export function hullMargin(points, p) {
  const hull = hull2d(points);
  let m = Infinity;
  for (let i = 0; i < hull.length; i++) {
    const a = hull[i], b = hull[(i + 1) % hull.length];
    const v = ((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])) / Math.hypot(b[0] - a[0], b[1] - a[1]);
    if (v < m) m = v;
  }
  return m;
}

/** share_load(): least-effort split of the weight over the feet in contact, no foot pulling. Returns {index: share}. */
export function shareLoad(points, com) {
  const act = points.map((_, i) => i);
  for (;;) {
    const n = act.length;
    const xs = act.map((i) => points[i][0]), ys = act.map((i) => points[i][1]);
    let sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0;
    for (const x of xs) sx += x;
    for (const y of ys) sy += y;
    for (const x of xs) sxx += x * x;
    for (const y of ys) syy += y * y;
    for (let i = 0; i < n; i++) sxy += xs[i] * ys[i];
    const A = [[n, sx, sy], [sx, sxx, sxy], [sy, sxy, syy]];
    const b = [1.0, com[0], com[1]];
    for (let i = 0; i < 3; i++) {
      let piv = i;
      for (let r = i + 1; r < 3; r++) if (Math.abs(A[r][i]) > Math.abs(A[piv][i])) piv = r;
      [A[i], A[piv]] = [A[piv], A[i]];
      [b[i], b[piv]] = [b[piv], b[i]];
      for (let r = 0; r < 3; r++) {
        if (r !== i && Math.abs(A[i][i]) > 1e-12) {
          const k = A[r][i] / A[i][i];
          A[r] = A[r].map((u, j) => u - k * A[i][j]);
          b[r] -= k * b[i];
        }
      }
    }
    const lam = [0, 1, 2].map((i) => (Math.abs(A[i][i]) > 1e-12 ? b[i] / A[i][i] : 0.0));
    const w = xs.map((x, i) => lam[0] + lam[1] * x + lam[2] * ys[i]);
    const wmin = Math.min(...w);
    if (wmin >= -1e-9 || n <= 3) return Object.fromEntries(w.map((v, i) => [act[i], Math.max(v, 0.0)]));
    act.splice(w.indexOf(wmin), 1);
  }
}

/** Loads: masses in the CAD script's order, total, centre of mass, gravity torques and support figures. */
export function loads(model, variant, pf, rest = false) {
  const P = model.physics, C = model.C;
  const pl = placements(model, variant, pf);
  const masses = [];
  const add = (name, phys) => { const ph = P[phys]; masses.push([name, ph.mass, apply(pl[name], ph.c)]); };
  for (const inst of model.instances) if (inst.frame === "body" && inst.phys) add(inst.name, inst.phys);
  const fkey = variant === "sneaker" ? "foot_sneaker" : "foot";
  for (const [n] of HIPS) for (const k of ["s1", "coxa", "s2", "femur", "s3", "knee", "blade", fkey]) {
    const name = `${n}_${k}`, inst = model.instances.find((q) => q.name === name);
    add(name, inst.phys);
  }
  if (variant === "sneaker") for (const [n] of HIPS) for (const k of ["sneaker_white", "sneaker_red", "sneaker_stripe"]) add(`${n}_${k}`, k);
  for (const [name, p, m] of C.LUMPS_BODY) masses.push([name, m, apply(pf.B, p)]);
  for (const [n] of HIPS) for (const link of ["coxa", "femur", "tibia"]) masses.push([`${n}_hw_${link}`, C.LEG_HARDWARE / 3, apply(pf.frames[n][link], [0, 0, 0])]);
  let total = 0;
  for (const [, m] of masses) total += m;
  let cx = 0, cy = 0, cz = 0;
  for (const [, m, p] of masses) { cx += p[0] * m; cy += p[1] * m; cz += p[2] * m; }
  const inv = 1.0 / total;
  const com = [cx * inv, cy * inv, cz * inv];

  const BEYOND = { femur: ["_femur", "_s3", "_knee", "_blade", "_foot", "_sneaker", "_hw_femur", "_hw_tibia"], knee: ["_knee", "_s3", "_blade", "_foot", "_sneaker", "_hw_tibia"] };
  function torque(leg, joint, wKg) {
    const f = pf.frames[leg][joint === "femur" ? "femur" : "tibia"];
    const p = apply(f, [0, 0, 0]);
    const a = sub(apply(f, [0, 0, 1]), p);
    let t = 0.0;
    for (const [n, m, c] of masses) {
      if (n.startsWith(leg + "_") && BEYOND[joint].some((b) => n.startsWith(leg + b))) t += dot(a, cross(sub(c, p), [0, 0, -m / 1000.0]));
    }
    t += dot(a, cross(sub(pf.feet[leg], p), [0, 0, wKg]));
    return Math.abs(t) / 10.0;
  }
  function support(legs) {
    const pts = legs.map((n) => [pf.feet[n][0], pf.feet[n][1]]);
    const c2 = [com[0], com[1]];
    const margin = hullMargin(pts, c2);
    const shares = shareLoad(pts, c2);
    const w = {};
    legs.forEach((n, i) => { w[n] = (shares[i] ?? 0.0) * total / 1000.0; });
    const per = {};
    let femur = 0, knee = 0;
    for (const [n] of HIPS) {
      per[n] = { femur: torque(n, "femur", w[n] ?? 0.0), knee: torque(n, "knee", w[n] ?? 0.0) };
      femur = Math.max(femur, per[n].femur); knee = Math.max(knee, per[n].knee);
    }
    const h = com[2] - Math.min(...legs.map((n) => pf.feet[n][2] - R_TIP));
    return { legs: [...legs], margin, push_g: margin / h, com_height: h, femur, knee, max_foot_load_g: Math.max(...Object.values(w)) * 1000, per_leg: per,
      w: Object.fromEntries(Object.entries(w).map(([k, v]) => [k, v * 1000])), hull: hull2d(pts) };
  }
  function resting() {
    const belly = [];
    for (const sx of [1, -1]) for (const sy of [1, -1]) belly.push(apply(pf.B, [sx * BELLY[0], sy * BELLY[1], BELLY[2]]));
    const margin = hullMargin(belly.map((p) => [p[0], p[1]]), [com[0], com[1]]);
    let femur = 0, knee = 0;
    for (const [n] of HIPS) { femur = Math.max(femur, torque(n, "femur", 0.0)); knee = Math.max(knee, torque(n, "knee", 0.0)); }
    return { margin, femur, knee, belly, hull: hull2d(belly.map((p) => [p[0], p[1]])) };
  }
  return { masses, total, com, torque, support, resting };
}

// --- page helpers ---
/** Pose of a preset (model.json presets): body frame from its parameters, legs by IK on its exact targets, overrides. */
export function presetPose(model, pr) {
  const B = bodyFrame(pr.body);
  const pose = posture(B, pr.targets, pr.contact, pr.on);
  for (const [n, j] of Object.entries(pr.overrides || {})) pose.joints[n] = [...j];
  return pose;
}

/** Gait frame k as a pose: body level at belly height hb, feet on a circle gaitR beyond the hips (the CAD script's
 * gait_report at hb = STAND_HB, gaitR = GAIT_R; the height study checked the other heights with their own gaitR, lift, out). */
export function gaitPose(kind, k, g = GAIT, hb = STAND_HB, gaitR = GAIT_R) {
  const { feet, stance } = gaitFeet(kind, k, g, spots(gaitR));
  const on = LEGS.filter((n) => Math.abs(feet[n][2] - R_TIP) < 1e-6);
  const pose = posture(T(0, 0, hb), feet, [L_TIBIA, 0], on);
  return { pose, feet, stance };
}

/** Gait at belly height hb from the heights FreeCAD checked (model.height.grid, the height study): walking radius, lift and
 * outward swing, linear between grid heights, held at the ends. `checked` is the grid row when hb sits on one. */
export function gaitAt(hb, height) {
  const G = height.grid;
  const exact = G.find((r) => Math.abs(r.hb - hb) < 1e-6) || null;
  let i = 0;
  while (i < G.length - 2 && hb > G[i + 1].hb) i++;
  const a = G[i], b = G[i + 1] || a;
  const f = exact ? 0 : Math.min(1, Math.max(0, (hb - a.hb) / (b.hb - a.hb || 1)));
  const at = exact || { gait_r: a.gait_r + (b.gait_r - a.gait_r) * f, lift: a.lift + (b.lift - a.lift) * f, out: a.out + (b.out - a.out) * f };
  return { hb, gaitR: at.gait_r, g: { ...GAIT, lift: at.lift, out: at.out }, checked: exact, between: exact ? [hb, hb] : [a.hb, b.hb] };
}

/** Presets that follow the belly height: their body moves by (hb - STAND_HB), feet stay on their floor targets. */
export const HEIGHT_PRESETS = ["stand", "look_up", "look_around", "wave", "wave_2"];
export function presetAt(pr, hb) {
  if (!HEIGHT_PRESETS.includes(pr.id)) return pr;
  return { ...pr, body: { ...pr.body, hb: pr.body.hb + (hb - STAND_HB) } };
}

/** Body moved on planted feet: legs in `legs` keep their current contact point (dropped to the floor), others keep joints. */
export function moveBody(pose, body, contact, legs, z0 = R_TIP) {
  const pf = poseFrames(pose, contact);
  const B = bodyFrame(body);
  const joints = {};
  HIPS.forEach(([n], i) => {
    if (legs.includes(n)) {
      const f = pf.feet[n];
      joints[n] = legIk(i, [f[0], f[1], z0], B, contact);
    } else joints[n] = [...pose.joints[n]];
  });
  return { B, joints, on: [...legs] };
}

/** Legs whose contact point is on the floor: within tol of its standing height z0. 1.5 mm covers the settled
 * tilted poses (the claw tip touches with its edge when the body pitches, up to 0.8 mm higher). */
export function legsOnFloor(pf, z0 = R_TIP, tol = 1.5) { return LEGS.filter((n) => Math.abs(pf.feet[n][2] - z0) <= tol); }

/** Statics for a page state: belly support when the belly plate touches the floor, else the legs on the floor. */
export function statics(model, variant, pose, opts = {}) {
  const contact = contactOf(model, variant);
  const pf = poseFrames(pose, contact);
  const L = loads(model, variant, pf);
  const bellyZ = Math.min(...[[1, 1], [1, -1], [-1, 1], [-1, -1]].map(([sx, sy]) => apply(pf.B, [sx * BELLY[0], sy * BELLY[1], BELLY[2]])[2]));
  const z0 = contactZ(model, variant);
  const on = opts.stance || legsOnFloor(pf, z0);
  const out = { pf, L, total: L.total, com: L.com, on, bellyZ, limits: limitsBad(pose.joints) };
  out.below = LEGS.filter((n) => pf.feet[n][2] < z0 - 1.5);
  if (opts.rest || bellyZ <= 0.5) { out.kind = "belly"; out.rest = L.resting(); if (on.length >= 3) out.standUp = L.support(on); }
  else if (on.length >= 3) {
    out.kind = "legs"; out.sup = L.support(on);
    if (on.length === 6 && !opts.stance) out.tri = [L.support(TRI_A), L.support(TRI_B)];
  } else out.kind = "none";
  return out;
}

/** Linear blend of two page states (body values and joints). */
export function blend(a, b, f) {
  const mix = (x, y) => x + (y - x) * f;
  const body = Object.fromEntries(["hb", "pitch", "yaw", "x", "y"].map((k) => [k, mix(a.body[k] || 0, b.body[k] || 0)]));
  const joints = Object.fromEntries(LEGS.map((n) => [n, a.joints[n].map((v, i) => mix(v, b.joints[n][i]))]));
  return { body, joints };
}
