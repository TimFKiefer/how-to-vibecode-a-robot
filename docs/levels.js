// SPDX-License-Identifier: MIT AND CC-BY-SA-4.0
// levels.js: the 41 training rooms of Puls and a
// sketch renderer for them. Plain ES module, no dependencies. Sketches only: heights drawn taller, not to scale.
// The SVG markers and gradients it references (ah, ahr, ball, pit) are defined once in index.html.
// Fallback only: the page reads the real progress from status.json and uses these values when that file cannot be read.
// Every level up to `done` is passed, `active` is the one being worked on. 0 means "only the Hall is built".
export const STATUS = { done: 0, active: 1, note: "Puls's learning core is built and passed its checks. No training attempt has run yet." };

const u = 34, C = Math.cos(Math.PI / 6), S = 0.5, W = 8, D = 5, HW = 2.3;
const P = (x, y, z = 0) => [(x - y) * C * u, (x + y) * S * u - z * u];
const f = n => n.toFixed(1);
const pts = a => a.map(p => f(p[0]) + "," + f(p[1])).join(" ");
const poly = (a, fill, extra = "") => `<polygon points="${pts(a)}" fill="${fill}" ${extra}/>`;
const line = (a, b, stroke, w = 1, extra = "") => `<line x1="${f(a[0])}" y1="${f(a[1])}" x2="${f(b[0])}" y2="${f(b[1])}" stroke="${stroke}" stroke-width="${w}" ${extra}/>`;
const edge = 'stroke="#1b1e23" stroke-opacity=".35" stroke-width=".7" stroke-linejoin="round"';

const COL = {
  grey: { top: "#d8dbe0", r: "#a9aeb6", l: "#8b9199" },
  podium: { top: "#5fd640", r: "#a9aeb6", l: "#8b9199" },
  wood: { top: "#e6c38e", r: "#c69c60", l: "#a88047" },
  heavy: { top: "#6a6f78", r: "#4e525a", l: "#3b3e45" },
  loose: { top: "#ffd43b", r: "#e2b100", l: "#c49800" },
  pen: { top: "#ffd43b", r: "#e2b100", l: "#c49800" },
  bump: { top: "#7d828b", r: "#5d6169", l: "#4a4e55" },
  post: { top: "#e9ecef", r: "#c3c8cf", l: "#a7adb5" },
  bar: { top: "#f07d3c", r: "#d0622a", l: "#b2511f" },
};
const ROBOT = {
  blue: { top: "#3d86f7", in: "#5c9bff", s1: "#2668d4", s2: "#1c52ab", leg: "#2f74ea" },
  orange: { top: "#f28a2e", in: "#ffa552", s1: "#d46d18", s2: "#ad560f", leg: "#ec7d22" },
};

function rectF(x0, y0, x1, y1, z = 0.004) { return [P(x0, y0, z), P(x1, y0, z), P(x1, y1, z), P(x0, y1, z)]; }
function box(x0, y0, x1, y1, z0, z1, c) {
  return poly(rectF(x0, y0, x1, y1, z1), c.top, edge)
    + poly([P(x1, y0, z0), P(x1, y1, z0), P(x1, y1, z1), P(x1, y0, z1)], c.r, edge)
    + poly([P(x0, y1, z0), P(x1, y1, z0), P(x1, y1, z1), P(x0, y1, z1)], c.l, edge);
}
function ramp(x0, y0, x1, y1, h, c) {
  return poly([P(x0, y0, 0), P(x1, y0, h), P(x1, y1, h), P(x0, y1, 0)], c.top, edge)
    + poly([P(x0, y1, 0), P(x1, y1, 0), P(x1, y1, h)], c.l, edge);
}
function cyl(cx, cy, r, h, top, side, inner) {
  const b = P(cx, cy, 0), t = P(cx, cy, h), rx = Math.SQRT2 * C * u * r, ry = Math.SQRT2 * S * u * r;
  let s = `<path d="M${f(b[0] - rx)} ${f(b[1])} A${f(rx)} ${f(ry)} 0 0 0 ${f(b[0] + rx)} ${f(b[1])} L${f(t[0] + rx)} ${f(t[1])} L${f(t[0] - rx)} ${f(t[1])} Z" fill="${side}" ${edge}/>`;
  s += `<ellipse cx="${f(t[0])}" cy="${f(t[1])}" rx="${f(rx)}" ry="${f(ry)}" fill="${top}" ${edge}/>`;
  if (inner) s += `<ellipse cx="${f(t[0])}" cy="${f(t[1] + ry * 0.08)}" rx="${f(rx * 0.8)}" ry="${f(ry * 0.8)}" fill="${inner}"/>`;
  return s;
}
function ellipseF(cx, cy, r, z, attrs) {
  const c = P(cx, cy, z);
  return `<ellipse cx="${f(c[0])}" cy="${f(c[1])}" rx="${f(Math.SQRT2 * C * u * r)}" ry="${f(Math.SQRT2 * S * u * r)}" ${attrs}/>`;
}
function robot(x, y, o = {}) {
  const ground = o.zg || 0, lift = o.air || 0, zg = ground + lift, k = ROBOT[o.col || "blue"], yaw = o.yaw || 0, low = !!o.low;
  const R = 0.24, zb = zg + (low ? 0.01 : 0.17), zt = zb + 0.2;
  let back = "", front = "";
  for (let i = 0; i < 6; i++) {
    const a = yaw + (i * 60 + 30) * Math.PI / 180, ca = Math.cos(a), sa = Math.sin(a);
    const hip = P(x + R * ca, y + R * sa, zb + 0.1);
    const knee = low ? P(x + 0.52 * ca, y + 0.52 * sa, zg + 0.12) : P(x + 0.47 * ca, y + 0.47 * sa, zb + 0.32);
    const foot = low ? P(x + 0.74 * ca, y + 0.74 * sa, zg) : P(x + 0.6 * ca, y + 0.6 * sa, zg);
    const leg = `<polyline points="${pts([hip, knee, foot])}" fill="none" stroke="${k.leg}" stroke-width="4.6" stroke-linecap="round" stroke-linejoin="round"/>`
      + `<circle cx="${f(knee[0])}" cy="${f(knee[1])}" r="2.3" fill="#15181d"/>`
      + `<circle cx="${f(foot[0])}" cy="${f(foot[1])}" r="1.6" fill="#15181d"/>`;
    if (ca + sa > 0) front += leg; else back += leg;
  }
  const v = [];
  for (let i = 0; i < 6; i++) { const a = yaw + i * Math.PI / 3; v.push([x + R * Math.cos(a), y + R * Math.sin(a)]); }
  let body = "";
  for (let i = 0; i < 6; i++) {
    const m = yaw + (i * 60 + 30) * Math.PI / 180;
    if (Math.cos(m) + Math.sin(m) > 0) {
      const p = v[i], q = v[(i + 1) % 6];
      body += poly([P(p[0], p[1], zb), P(q[0], q[1], zb), P(q[0], q[1], zt), P(p[0], p[1], zt)], i % 2 ? k.s1 : k.s2, edge);
    }
  }
  body += poly(v.map(p => P(p[0], p[1], zt)), k.top, edge);
  body += poly(v.map(p => P(x + (p[0] - x) * 0.6, y + (p[1] - y) * 0.6, zt)), k.in);
  const eye = P(x + R * 0.78 * Math.cos(yaw), y + R * 0.78 * Math.sin(yaw), zt);
  body += `<circle cx="${f(eye[0])}" cy="${f(eye[1])}" r="2.2" fill="#0f1114"/>`;
  if (o.load) body += box(x - 0.09, y - 0.09, x + 0.09, y + 0.09, zt, zt + 0.12, COL.heavy);
  if (o.loose) body += box(x - 0.12, y - 0.08, x + 0.12, y + 0.08, zt, zt + 0.1, COL.loose);
  const shadow = ellipseF(x, y, lift ? 0.45 : 0.62, ground + 0.003, `fill="#000" fill-opacity="${lift ? ".14" : ".22"}"`);
  return shadow + back + body + front;
}
function label(p, text, kind) {
  const w = text.length * 5.9 + 12, color = kind === "t" ? "#c2410c" : kind === "r" ? "#c62828" : "#16191e";
  return `<g transform="translate(${f(p[0])},${f(p[1])})"><rect x="${f(-w / 2)}" y="-9" width="${f(w)}" height="16" rx="8" fill="#ffffff" fill-opacity=".93"/>`
    + `<text x="0" y="3" text-anchor="middle" font-family="Inter, Arial, sans-serif" font-size="10.5" font-weight="600" fill="${color}">${text}</text></g>`;
}
function scene() {
  const F = [], O = [], T = [];
  const s = {
    floor: svg => F.push(svg),
    obj: (d, svg) => O.push([d, svg]),
    top: svg => T.push(svg),
    start: (x, y) => {
      F.push(poly(rectF(x, y, x + 1, y + 1), "#2b2d31"));
      for (let i = 0; i < 4; i++) F.push(poly(rectF(x + 0.12, y + 0.12 + i * 0.21, x + 0.88, y + 0.22 + i * 0.21, 0.006), "#f59f0b"));
    },
    goal: (x, y, w = 1, h = 1) => F.push(poly(rectF(x, y, x + w, y + h, 0.006), "#5fd640")),
    zone: (x0, y0, x1, y1) => F.push(poly(rectF(x0, y0, x1, y1, 0.006), "#5fd640", 'fill-opacity=".45" stroke="#5fd640" stroke-width="1.6" stroke-dasharray="5 3"')),
    path: pts3 => T.push(`<polyline points="${pts(pts3.map(p => P(p[0], p[1], p[2] || 0.01)))}" fill="none" stroke="#ffffff" stroke-width="1.8" stroke-dasharray="5 4" marker-end="url(#ah)"/>`),
    push: (a, b) => T.push(`<line x1="${f(P(...a)[0])}" y1="${f(P(...a)[1])}" x2="${f(P(...b)[0])}" y2="${f(P(...b)[1])}" stroke="#e5484d" stroke-width="3.2" marker-end="url(#ahr)"/>`),
    robot: (x, y, o = {}) => O.push([o.depth != null ? o.depth : x + y, robot(x, y, o)]),
    box: (x0, y0, x1, y1, z0, z1, c, depth) => O.push([depth != null ? depth : (x0 + x1) / 2 + (y0 + y1) / 2, box(x0, y0, x1, y1, z0, z1, c)]),
    label: (x, y, z, text, kind) => T.push(label(P(x, y, z), text, kind)),
    pit: (x0, x1) => {
      F.push(poly(rectF(x0, 0, x1, D, 0.002), "url(#pit)"));
      F.push(poly([P(x0, D, 0), P(x1, D, 0), P(x1, D, -0.25), P(x0, D, -0.25)], "#050506"));
    },
    bumps: () => [[2.6, 1.8, 0.4, 0.35, 0.1], [3.3, 2.7, 0.3, 0.4, 0.14], [3.9, 1.6, 0.5, 0.3, 0.08], [4.4, 2.5, 0.35, 0.35, 0.12], [5.1, 2.0, 0.3, 0.5, 0.1], [3.0, 3.4, 0.4, 0.3, 0.09], [4.7, 3.3, 0.3, 0.3, 0.13], [2.4, 2.6, 0.3, 0.3, 0.07]]
      .forEach(([x, y, w, h, z]) => O.push([x + w / 2 + y + h / 2, box(x, y, x + w, y + h, 0, z, COL.bump)])),
    ice: () => {
      F.push(poly(rectF(2.2, 1.0, 5.8, 4.0, 0.005), "#a5dcf5", 'fill-opacity=".5"'));
      [[2.6, 1.4, 3.4, 1.6], [3.8, 2.6, 4.9, 2.9], [4.2, 1.3, 5.2, 1.5], [2.8, 3.3, 3.6, 3.5]].forEach(([a, b, c, d]) => F.push(line(P(a, b, 0.006), P(c, d, 0.006), "#ffffff", 1.4, 'stroke-opacity=".8" stroke-linecap="round"')));
    },
  };
  return { s, F, O, T };
}

/** The room of one level as an SVG string (board shows the level's time and attempt 00: no attempt is drawn in a sketch). */
export function roomSVG(lv) {
  const { s, F, O, T } = scene();
  lv.scene(s);
  O.sort((a, b) => a[0] - b[0]);
  let g = "";
  g += poly([P(0, 0, 0), P(0, D, 0), P(0, D, HW), P(0, 0, HW)], "#c9ccd2");
  g += poly([P(0, 0, 0), P(W, 0, 0), P(W, 0, HW), P(0, 0, HW)], "#eef0f3");
  for (let i = 1; i < W; i++) g += line(P(i, 0, 0), P(i, 0, HW), "#9ea3ab", 0.7);
  for (let j = 1; j < D; j++) g += line(P(0, j, 0), P(0, j, HW), "#8f949c", 0.7);
  for (const z of [0.8, 1.6]) { g += line(P(0, 0, z), P(W, 0, z), "#9ea3ab", 0.7); g += line(P(0, 0, z), P(0, D, z), "#8f949c", 0.7); }
  for (const x of [1.5, 4, 6.5]) g += ellipseF(x, 0.02, 0.5, HW - 0.25, 'fill="#ffffff" fill-opacity=".55"');
  const tt = String(lv.time).padStart(2, "0");
  g += `<g transform="matrix(${f(C * u)},${f(S * u)},0,${u},0,0)" font-family="JetBrains Mono, Menlo, monospace" font-weight="700" text-anchor="middle">`
    + `<rect x="${tt.length > 2 ? 3.45 : 3.62}" y="-2.08" width="${tt.length > 2 ? 1.1 : 0.76}" height="0.42" fill="#0b0c0e"/>`
    + `<text x="4" y="-1.74" font-size="0.36" fill="#f4f4f4">${tt}</text>`
    + `<rect x="3.3" y="-1.58" width="1.4" height="0.42" fill="#0b0c0e"/>`
    + `<text x="4" y="-1.24" font-size="0.36"><tspan fill="#2a2c30">88</tspan><tspan fill="#f4f4f4">00</tspan></text></g>`;
  g += poly([P(0, 0), P(W, 0), P(W, D), P(0, D)], "#3a3c41");
  for (let i = 1; i < W; i++) g += line(P(i, 0), P(i, D), "#d9dbe0", 0.8, 'stroke-opacity=".55"');
  for (let j = 1; j < D; j++) g += line(P(0, j), P(W, j), "#d9dbe0", 0.8, 'stroke-opacity=".55"');
  g += poly([P(W, 0, 0), P(W, D, 0), P(W, D, -0.25), P(W, 0, -0.25)], "#2a2c30");
  g += poly([P(0, D, 0), P(W, D, 0), P(W, D, -0.25), P(0, D, -0.25)], "#1f2125");
  g += F.join("") + O.map(o => o[1]).join("") + T.join("");
  return `<svg class="room" viewBox="-160 -92 410 332" role="img" aria-label="Room sketch, level ${lv.n}: ${lv.name}">${g}</svg>`;
}

const pen = s => s.box(2.6, 2.44, 3.05, 2.5, 0, 0.04, COL.pen);
const podiumBig = (s, h) => s.box(5.8, 1.2, 7.6, 3.8, 0, h, COL.podium);
const board = (s, x = 2.4, y = 3.6) => s.box(x, y, x + 0.9, y + 0.54, 0, 0.05, COL.wood);   // plain plywood, 30 x 18 cm

export const WORLDS = [
  { w: 1, name: "Know your body", line: "Find a rhythm alone, then walk, turn and hold on." },
  { w: 2, name: "Find your limits", line: "How high, how far, how steady. Measured, not guessed." },
  { w: 3, name: "Use the world", line: "Push things. Front legs become hands. Move a board and build your own way up." },
  { w: 4, name: "Watch and understand", line: "Read the goal from a sentence or from someone showing it." },
];

/** Level 0 is not a training level: it is the Hall itself, built and checked first. */
export const HALL = {
  n: 0, w: 1, name: "The Hall", time: 30,
  happens: "The simulated Takt stands in its training room: tiles, walls, the board, the start tile.",
  knows: "Nothing yet. Puls has not moved a single joint.",
  why: "An honest simulation: the body matches the CAD model in 324 poses and runs over 100 times faster than real time.",
  scene: s => { s.start(1, 2); s.robot(1.5, 2.5); },
};

export const LEVELS = [
  { w: 1, name: "Walk Off", time: 30, scene: s => { s.start(1, 2); s.floor(ellipseF(1.5, 2.5, 0.9, 0.01, 'fill="none" stroke="#ffffff" stroke-width="1.6" stroke-dasharray="5 4"')); s.robot(1.5, 2.5); s.label(1.5, 3.6, 0.02, "30 cm"); } },
  { w: 1, name: "Stand Up", time: 30, scene: s => { s.start(3, 2); s.robot(3.5, 2.5, { low: true }); s.label(3.5, 2.5, 0.9, "on its belly"); } },
  { w: 1, name: "Straight Ahead", time: 60, scene: s => { s.start(1, 2); s.goal(4, 2); s.path([[2.1, 2.5], [3.9, 2.5]]); s.robot(1.5, 2.5); s.label(3, 2.5, 0.5, "1 m"); } },
  { w: 1, name: "Turn", time: 30, scene: s => { s.start(3, 2); const a = []; for (let t = 0; t <= 90; t += 9) { const r = t * Math.PI / 180; a.push([3.5 + 0.95 * Math.cos(r), 2.5 + 0.95 * Math.sin(r)]); } s.path(a); s.robot(3.5, 2.5); s.label(4.6, 3.6, 0.02, "90°"); } },
  { w: 1, name: "Sidestep", time: 60, scene: s => { s.start(3, 1); s.goal(3, 2.5); s.path([[3.5, 2.0], [3.5, 2.6]]); s.robot(3.5, 1.5); s.label(4.4, 3.2, 0.02, "0.5 m"); } },
  { w: 1, name: "Hold the Height", time: 30, scene: s => { s.start(3, 2); s.robot(3.5, 2.5); s.box(4.55, 1.55, 4.65, 1.65, 0, 0.82, COL.post, 4.4);
      [[0.225, "30"], [0.465, "62"], [0.71, "95 mm"]].forEach(([z, t]) => { s.box(4.5, 1.5, 4.7, 1.7, z, z + 0.025, COL.bar, 4.41); s.label(5.25, 1.6, z, t); }); } },
  { w: 1, name: "Shove", time: 60, scene: s => { s.start(1, 2); s.goal(4, 2); s.path([[3.0, 2.5], [3.9, 2.5]]); s.robot(2.5, 2.5); s.push([2.5, 4.0, 0.32], [2.5, 3.05, 0.32]); s.label(2.5, 4.3, 0.32, "push", "r"); } },
  { w: 2, name: "Ledge", time: 60, scene: s => { s.start(1, 2); s.box(3.2, 1.2, 6.8, 3.8, 0, 0.1, COL.podium); s.robot(1.5, 2.5); s.label(5, 3.8, 0.1, "15 mm"); } },
  { w: 2, name: "Tall Box", time: 90, scene: s => { s.start(1, 2); s.box(3.8, 1.4, 6.6, 3.6, 0, 0.4, COL.podium); s.robot(1.5, 2.5); s.label(5.2, 3.6, 0.4, "60 mm"); } },
  { w: 2, name: "Limit Stairs", time: 120, scene: s => { s.start(1, 2); for (let i = 0; i < 7; i++) { const x0 = 2.6 + i * 0.72; s.box(x0, 1.3, x0 + 0.72, 3.7, 0, 0.12 * (i + 1), COL.grey); } s.robot(1.5, 2.5);
      for (let i = 0; i < 7; i++) s.label(2.96 + i * 0.72, 1.7, 0.12 * (i + 1) + 0.12, String(2 * (i + 1))); } },
  { w: 2, name: "Staircase", time: 120, scene: s => { s.start(1, 2); [[3.6, 0.15], [4.3, 0.3], [5.0, 0.45]].forEach(([x, h]) => s.box(x, 1.4, x + 0.7, 3.6, 0, h, COL.grey)); s.box(5.7, 1.4, 7.5, 3.6, 0, 0.6, COL.podium); s.robot(1.5, 2.5); } },
  { w: 2, name: "Bumpy Floor", time: 90, scene: s => { s.start(1, 2); s.goal(6, 2); s.bumps(); s.robot(1.5, 2.5); } },
  { w: 2, name: "Slippery Floor", time: 90, scene: s => { s.start(1, 2); s.goal(6, 2); s.ice(); s.robot(1.5, 2.5); s.label(4, 2.5, 0.02, "slippery"); } },
  { w: 2, name: "Backpack", time: 60, scene: s => { s.start(1, 2); s.goal(4, 2); s.path([[2.1, 2.5], [3.9, 2.5]]); s.robot(1.5, 2.5, { load: true }); s.label(1.5, 2.5, 0.85, "200 g, fixed"); } },
  { w: 2, name: "Steady Carry", time: 60, scene: s => { s.start(1, 2); s.goal(4, 2); s.path([[2.1, 2.5], [3.9, 2.5]]); s.robot(1.5, 2.5, { loose: true }); s.label(1.5, 2.5, 0.85, "loose"); } },
  { w: 2, name: "Carry over Bumps", time: 90, scene: s => { s.start(1, 2); s.goal(6, 2); s.bumps(); s.robot(1.5, 2.5, { loose: true }); s.label(1.5, 2.5, 0.85, "loose"); } },
  { w: 2, name: "Carry on Ice", time: 90, scene: s => { s.start(1, 2); s.goal(6, 2); s.ice(); s.robot(1.5, 2.5, { loose: true }); s.label(1.5, 2.5, 0.85, "loose"); s.label(4, 2.5, 0.02, "slippery"); } },
  { w: 2, name: "Ramp", time: 90, scene: s => { s.start(1, 2); s.obj(5.0, ramp(2.8, 1.5, 5.3, 3.5, 0.5, COL.grey)); s.box(5.3, 1.5, 7.3, 3.5, 0, 0.5, COL.podium); s.robot(1.5, 2.5); } },
  { w: 2, name: "Gap", time: 90, scene: s => { s.start(1, 2); s.pit(3.6, 4.2); s.goal(5.8, 2); s.robot(1.5, 2.5); s.label(3.9, 4.6, 0.02, "gap"); } },
  { w: 2, name: "Duck", time: 90, scene: s => { s.start(1, 2); s.goal(5.8, 2); s.box(3.95, 0.8, 4.05, 0.9, 0, 0.55, COL.post); s.robot(1.5, 2.5); s.box(3.95, 0.8, 4.05, 4.2, 0.5, 0.56, COL.bar, 6.0); s.box(3.95, 4.1, 4.05, 4.2, 0, 0.55, COL.post, 8.2); } },
  { w: 2, name: "Hop", time: 30, scene: s => { s.start(3, 2); s.robot(3.5, 2.5, { air: 0.3 }); s.label(3.5, 2.5, 1.2, "nothing touches the floor"); } },
  { w: 2, name: "Long Jump", time: 60, scene: s => { s.start(1, 2); for (let k = 1; k <= 8; k++) { const x = 2.1 + 0.3 * k; s.floor(line(P(x, 1.4, 0.006), P(x, 3.6, 0.006), "#ffffff", k % 2 ? 0.8 : 1.4, 'stroke-opacity=".8"')); if (k % 2 === 0) s.label(x, 3.9, 0.02, String(k * 10)); } s.robot(1.5, 2.5); s.label(3.3, 1.2, 0.02, "cm from the line"); } },
  { w: 2, name: "High Jump", time: 60, scene: s => { s.start(1, 2); [[2.8, 0.15], [3.9, 0.3], [5.0, 0.45], [6.1, 0.6]].forEach(([x, h]) => s.box(x, 1.9, x + 0.7, 3.1, 0, h, COL.grey)); s.robot(1.5, 2.5); s.label(5.0, 1.6, 0.9, "higher and higher"); } },
  { w: 2, name: "Big Gap", time: 90, scene: s => { s.start(1, 2); s.pit(3.0, 5.0); s.goal(6.2, 2); s.robot(1.6, 2.5); s.label(4.0, 4.6, 0.02, "as wide as its best jump"); } },
  { w: 3, name: "Push the Block", time: 120, scene: s => { s.start(1, 2); s.zone(5.2, 2.0, 6.2, 3.0); s.box(3.0, 2.2, 3.6, 2.8, 0, 0.45, COL.wood); s.path([[3.9, 2.5], [5.1, 2.5]]); s.robot(1.6, 2.5); } },
  { w: 3, name: "Push the Pen", time: 120, scene: s => { s.start(1, 2); s.zone(5.0, 1.7, 6.6, 2.9); pen(s); s.robot(1.5, 2.5); s.label(2.85, 2.1, 0.1, "pen"); } },
  { w: 3, name: "Pen in the Tray", time: 120, scene: s => { s.start(1, 2); pen(s); s.obj(6.6, box(5.0, 1.8, 6.5, 2.8, 0, 0.07, COL.post) + poly(rectF(5.07, 1.87, 6.43, 2.73, 0.071), "#a3a9b1")); s.robot(1.5, 2.5); } },
  { w: 3, name: "Pen in the Bowl", time: 180, scene: s => { s.start(1, 2); pen(s); s.obj(8.2, cyl(5.8, 2.4, 0.36, 0.16, "#eceff2", "#b9bec6", "#7c838c")); s.robot(1.5, 2.5); } },
  { w: 3, name: "Balance Beam", time: 90, scene: s => { s.start(1.85, 2); s.box(2.0, 2.23, 3.8, 2.77, 0, 0.09, COL.grey, 3.0); s.box(3.5, 2.23, 3.8, 2.77, 0.09, 0.096, COL.podium, 3.1); s.robot(2.35, 2.5, { zg: 0.09, depth: 5.4 }); s.label(4.8, 1.6, 0.02, "18 cm wide"); s.label(2.6, 3.6, 0.02, "foot on the floor = fall", "r"); } },
  { w: 3, name: "Up the Plank", time: 120, scene: s => { s.start(1, 2); s.obj(5.1, ramp(4.1, 2.23, 5.0, 2.77, 0.4, COL.wood)); s.box(5.0, 1.4, 7.2, 3.6, 0, 0.4, COL.podium); s.robot(1.5, 2.5); s.label(4.3, 3.3, 0.02, "board, fixed"); s.label(6.1, 2.5, 1.1, "too high to climb"); } },
  { w: 3, name: "Front Legs Free", time: 60, scene: s => { s.start(3, 2); s.robot(3.5, 2.5); s.label(3.5, 2.5, 0.95, "both front feet up, 2 s"); } },
  { w: 3, name: "Lift the Board", time: 120, scene: s => { s.start(1, 2); board(s, 2.4, 2.23); s.robot(1.5, 2.5); s.label(2.85, 2.5, 0.35, "one end 5 cm up"); } },
  { w: 3, name: "Fetch the Board", time: 180, scene: s => { s.start(1, 2); board(s); s.zone(5.0, 1.6, 6.4, 2.6); s.path([[3.5, 3.6], [4.9, 2.4]]); s.robot(1.5, 2.5); s.label(2.85, 3.87, 0.25, "board"); } },
  { w: 3, name: "Lay the Board", time: 180, scene: s => { s.start(1, 2); board(s); s.box(5.2, 1.3, 7.6, 3.7, 0, 0.2, COL.grey); s.robot(1.5, 2.5); s.label(2.85, 3.87, 0.25, "board"); s.label(6.4, 2.5, 0.35, "4 cm edge"); } },
  { w: 3, name: "Board as a Ramp", time: 180, scene: s => { s.start(1, 1.5); board(s); s.box(5.6, 1.2, 7.6, 3.8, 0, 0.4, COL.podium); s.robot(1.5, 2.0); s.label(2.85, 3.87, 0.25, "board"); s.label(6.6, 2.5, 0.75, "too high to climb"); } },
  { w: 3, name: "Block Stairs", time: 240, scene: s => { s.start(1, 2); s.box(2.8, 3.6, 3.5, 4.3, 0, 0.3, COL.wood); s.box(3.6, 0.8, 4.3, 1.5, 0, 0.6, COL.wood); podiumBig(s, 0.9); s.robot(1.5, 2.5); } },
  { w: 4, name: "Go to the Cup", time: 60, scene: s => { s.start(1, 2); s.obj(6.8, cyl(5.2, 1.6, 0.13, 0.3, "#f3f4f6", "#e0483e", "#3a1513")); s.obj(8.8, `<circle cx="${f(P(5.4, 3.4, 0.15)[0])}" cy="${f(P(5.4, 3.4, 0.15)[1])}" r="${f(0.15 * u)}" fill="url(#ball)"/>`); s.robot(1.6, 2.5);
      s.top(`<g transform="translate(-118,-58)"><rect width="110" height="26" rx="13" fill="#ffffff"/><text x="55" y="17" text-anchor="middle" font-family="Inter, Arial, sans-serif" font-size="12" font-weight="600" fill="#16191e">“Go to the cup.”</text></g>`); } },
  { w: 4, name: "Copy the Pen Trick", time: 180, scene: s => { s.start(1, 2.7); s.obj(7.6, box(5.6, 1.4, 6.8, 2.2, 0, 0.07, COL.post) + poly(rectF(5.67, 1.47, 6.73, 2.13, 0.071), "#a3a9b1") + box(5.9, 1.78, 6.35, 1.84, 0.07, 0.11, COL.pen)); s.robot(4.9, 1.8, { col: "orange" }); s.robot(1.6, 3.2, { yaw: -0.5 }); s.label(4.9, 1.8, 0.95, "teacher shows it", "t"); s.label(1.6, 3.2, 0.95, "student watches"); } },
  { w: 4, name: "Summit: Teacher Practises", time: 180, scene: s => { s.start(1.5, 2); s.box(3.6, 2.2, 4.3, 2.9, 0, 0.5, COL.wood); podiumBig(s, 0.9); s.robot(2.0, 2.5, { col: "orange" }); s.label(2.0, 2.5, 0.95, "teacher practises", "t"); } },
  { w: 4, name: "Summit: Student", time: 180, scene: s => { s.start(1, 3); s.box(4.9, 1.3, 5.6, 2.0, 0, 0.5, COL.wood, 6.4); s.box(5.6, 0.6, 7.4, 2.6, 0, 0.9, COL.podium, 8.0); s.robot(6.5, 1.6, { col: "orange", zg: 0.9, depth: 8.1, yaw: Math.PI }); s.robot(1.5, 3.5, { yaw: -0.6 }); s.label(6.5, 1.6, 1.85, "teacher is up", "t"); s.label(1.5, 3.5, 0.95, "student"); } },
  { w: 4, name: "Summit: Hard Mode", time: 240, scene: s => { s.start(1, 1.7); s.box(3.4, 2.0, 4.1, 2.7, 0, 0.5, COL.heavy); board(s, 2.4, 3.8); podiumBig(s, 0.9); s.robot(1.5, 2.2); s.label(3.75, 2.35, 0.8, "too heavy", "r"); s.label(2.85, 4.07, 0.25, "board"); } },
];

// [what happens, what Puls knows, what the level is for]
const INFO = [
  ["Puls starts from zero. It wiggles, feels how its body swings, and has to get off the striped tile on its own.", "Only its own body. No goal, no hint, no rhythm given.", "Prove it finds a walking rhythm by itself: 30 cm away after 30 s, no fall. Hopping does not count."],
  ["The robot lies on its belly. All six legs have to push at once.", "How high it should stand.", "Belly 55 mm up, level, held for 2 s."],
  ["Walk to the green tile, 1 m ahead.", "Where the tile is and how fast to go.", "Turn its own rhythm into a direction. Within 10 cm of the tile."],
  ["Turn a quarter circle on the spot, then stand still.", "The angle to reach.", "Within 10° of the target, held for 2 s."],
  ["Walk sideways like a crab, without turning.", "Where the target is.", "A way of walking it never needs for going forward."],
  ["Crouch low, stand normal, stand tall: 30, 62 and 95 mm.", "The height asked for.", "Control its own height. Each one held 5 s within 8 mm."],
  ["While it walks, a push hits it from the side at a random moment.", "Where the goal is. Not when the push comes.", "Catch itself and still reach the tile."],
  ["A flat 15 mm board lies in the way.", "The goal is on the board.", "Lift the feet instead of dragging them. All six feet on the board."],
  ["A 60 mm box it can only step onto when standing tall.", "The goal is on top. Nobody tells it to stand taller.", "Notice on its own that it has to rise first."],
  ["Steps from 2 to 14 cm, each one higher.", "The top step is the goal.", "Measure how high it really gets. Above about 7 cm one step is not enough: it has to climb."],
  ["Real stairs up to a platform, every step below its measured limit.", "The goal is on the platform.", "Chain many climbing steps, each at a different height."],
  ["1.5 m of bumpy floor.", "Where the goal is.", "Every foot lands at a different height, and it must not fall. The body stays calm, bobbing 10 mm at most."],
  ["A slippery patch between start and goal.", "Where the goal is.", "Find a way of stepping that does not slip. The body stays calm, bobbing 10 mm at most."],
  ["200 g strapped to its back.", "Where the goal is. Not that it got heavier.", "Adapt to a body that suddenly weighs more. The body stays calm, bobbing 10 mm at most."],
  ["A weight lies loose on its back. If the body tilts, it slides off.", "Where the goal is.", "Keep the body level while walking. The body stays calm, bobbing 10 mm at most."],
  ["Loose weight on its back, bumps under its feet.", "Where the goal is.", "The legs go up and down, the body stays level. The body stays calm, bobbing 10 mm at most."],
  ["Loose weight on its back, slippery floor.", "Where the goal is.", "No jerky moves, or the weight falls. The body stays calm, bobbing 10 mm at most."],
  ["A slope up onto a platform.", "The goal is on top.", "Push against sliding back down."],
  ["A trench across the room. Falling in counts as a fall.", "The goal is on the other side.", "Reach far enough and pull the body across. The body stays calm, bobbing 10 mm at most."],
  ["A bar hangs lower than the robot stands.", "The goal is behind the bar.", "Find a flatter way to walk. If this body cannot, that is the result. The body stays calm, bobbing 10 mm at most."],
  ["Jump on the spot. It counts only if, for a moment, nothing of the robot touches anything.", "That the feet should leave the floor.", "The first step to jumping. The servos are slow, so it may be impossible, and we will say so."],
  ["Lines on the floor every 10 cm.", "A goal far ahead.", "Measure its longest jump."],
  ["Boxes that get higher and higher.", "The goal is on a box.", "Measure how high it can jump, not just how far."],
  ["A trench just under its best jump.", "The goal is on the other side.", "Use the jump it learned. Half a jump means falling in."],
  ["A block and a green field.", "Where the block is and where it should go.", "Move something other than itself. The body stays calm, bobbing 10 mm at most."],
  ["A thin pen that rolls away.", "Where the pen is and where it should go.", "Use the front legs like hands. The body stays calm, bobbing 10 mm at most."],
  ["A tray with a 10 mm rim.", "Where the pen is and where it should go.", "Lift or roll the pen over an edge. The body stays calm, bobbing 10 mm at most."],
  ["A round bowl with a 30 mm rim.", "Where the pen is and where it should go.", "Fine control with legs that were never built for this. The body stays calm, bobbing 10 mm at most."],
  ["A beam 18 cm wide and 3 cm high, as narrow as the board. Puls starts on it. A foot on the floor beside it counts as a fall.", "Where the goal on the beam is.", "Walk a strip narrower than its normal stance, 0.5 m along it. The body stays calm, bobbing 10 mm at most."],
  ["A platform too high to climb. The board is fixed against it as a ramp.", "Where the goal is and how high.", "Walk up a narrow, sloping board. All six feet on top."],
  ["A flat floor. Stand on four legs and lift both front feet.", "Nothing.", "Free the front legs to use them as hands: 2 s, belly off the floor, no fall."],
  ["The board lies flat on the floor. No arms, only legs.", "Where the board is.", "Lift one end at least 5 cm and hold it 2 s. The board may touch only the floor and the robot."],
  ["The board and a marked field on the floor.", "Where the board and the field are.", "Move the whole board into the field and leave it there 2 s. The body stays calm, bobbing 10 mm at most."],
  ["The board and a 4 cm high edge.", "Where the goal is and where the board is.", "One end on the edge, the other on the floor. It stays there 3 s without the robot touching it."],
  ["A platform too high to climb, and the board lying loose somewhere else.", "The goal is on top, and where the board is. Nobody says what the board is for.", "Put it all together: fetch the board, lay it as a ramp and walk up."],
  ["An even higher platform and two blocks of different heights.", "The goal is on top, and where the blocks are.", "One block is not enough. Push both into a row as stairs: it has to plan two moves ahead."],
  ["A cup and a ball stand in the room. Puls hears one sentence.", "The sentence “Go to the cup.” and its camera picture.", "Connect a word with what it sees. The body stays calm, bobbing 10 mm at most."],
  ["The teacher, a second Takt, puts a pen into a tray. Puls only watches.", "Only what it saw. No goal is given.", "Understand the goal from watching, then do it in a new room. The body stays calm, bobbing 10 mm at most."],
  ["A copy of Puls trains to reach the platform with the help of the block.", "The goal is on top, and where the block is.", "Become a teacher that really learned it. No scripted movement."],
  ["The teacher shows it once. Then Puls tries in rooms it has never seen.", "Only what it watched.", "Beat a copy of itself that never saw the demo."],
  ["The block it saw is too heavy now. A board lies nearby.", "Only what it watched, with the block.", "Not copying: finding its own way up."],
];
LEVELS.forEach((l, i) => { l.n = i + 1; [l.happens, l.knows, l.why] = INFO[i]; });

export const two = n => String(n).padStart(2, "0");
