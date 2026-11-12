// SPDX-License-Identifier: MIT
// hall.js: the Hall furnished for one level, in 3D, on
// three.js 0.160: the room, the board, Takt and the teacher, and the furniture of each level that rises out of the
// floor while the furniture of the level before sinks into it. One scene, drawn by two renderers: the page's own
// canvas for the full roadmap, and one small renderer for the pictures in the level cards.
// Units: one tile (a third of a metre) is 1. Room coordinates: x to the right, y into the room, z up.

import { ROOMS } from "./rooms.js";

const clamp = (x, a = 0, b = 1) => Math.min(b, Math.max(a, x));
const lerp = (a, b, t) => a + (b - a) * t;
const smooth = t => { t = clamp(t); return t * t * (3 - 2 * t); };
const easeOut = t => 1 - Math.pow(1 - clamp(t), 3);
const pad = n => String(n).padStart(2, "0");

/**
 * THREE: the three.js module. geo: the robot's geometries by mesh key. model, fk: the CAD model and its kinematics.
 * boardFor(n): { time, attempt } shown on the board in level n. reduced: no motion in Level 06.
 * dark: the Hall at night (setDark switches it later; the board stays black either way).
 */
export function createHall(THREE, { geo, model, fk, boardFor, reduced = false, dark = false }) {
  const N = ROOMS.length;
  const V = (x, y, z) => new THREE.Vector3(x, y, z);
  const Wp = (x, y, z = 0) => V(x - 4, z, y - 2.5);
  const C = c => (c && c.isColor ? c : new THREE.Color(c));
  const std = (color, o = {}) => new THREE.MeshStandardMaterial(Object.assign({ color: C(color), roughness: 0.85, metalness: 0 }, o));

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x101216);
  const hemi = new THREE.HemisphereLight(0xffffff, 0x4f545e, 0.78 * Math.PI);
  scene.add(hemi);
  const sun = new THREE.DirectionalLight(0xffffff, 0.8 * Math.PI);
  sun.position.set(2.6, 7, 4.2);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, { left: -5.5, right: 5.5, top: 5.5, bottom: -5.5, near: 1, far: 20 });
  sun.shadow.bias = -0.0004; sun.shadow.normalBias = 0.012;
  scene.add(sun);
  const fill = new THREE.DirectionalLight(0xffffff, 0);   // a little front light at night, so the blue robot reads
  fill.position.set(-1.5, 3, 8);
  scene.add(fill);
  let DK = dark;
  const ROOM = {};                          // the room's materials, recoloured by setDark

  // ---------- the room
  const hall = new THREE.Group();
  scene.add(hall);
  (function buildHall() {
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(8, 5), ROOM.floor = std(0x3a3c41, { roughness: 0.78 }));
    floor.rotation.x = -Math.PI / 2; floor.receiveShadow = true; hall.add(floor);
    const slab = new THREE.Mesh(new THREE.BoxGeometry(8, 0.25, 5), std(0x25272b));
    slab.position.y = -0.127; hall.add(slab);
    const wall = (w, color) => new THREE.Mesh(new THREE.PlaneGeometry(w, 2.3), std(color, { roughness: 0.95 }));
    const back = wall(8, 0xe3e6ea); back.position.set(0, 1.15, -2.5); hall.add(back);
    const left = wall(5, 0xc4c8ce); left.position.set(-4, 1.15, 0); left.rotation.y = Math.PI / 2; hall.add(left);
    const right = wall(5, 0xc4c8ce); right.position.set(4, 1.15, 0); right.rotation.y = -Math.PI / 2; hall.add(right);
    ROOM.back = back.material; ROOM.sides = [left.material, right.material];
    const seg = (pts, color, opacity) => {
      const g = new THREE.BufferGeometry(); g.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3));
      const m = new THREE.LineBasicMaterial({ color: C(color), transparent: true, opacity });
      hall.add(new THREE.LineSegments(g, m)); return m;
    };
    const fl = [], wl = [];
    for (let i = 0; i <= 8; i++) fl.push(i - 4, 0.002, -2.5, i - 4, 0.002, 2.5);
    for (let j = 0; j <= 5; j++) fl.push(-4, 0.002, j - 2.5, 4, 0.002, j - 2.5);
    ROOM.tiles = seg(fl, 0xd9dbe0, 0.6);
    for (let i = 1; i < 8; i++) wl.push(i - 4, 0, -2.497, i - 4, 2.3, -2.497);
    for (let j = 1; j < 5; j++) wl.push(-3.997, 0, j - 2.5, -3.997, 2.3, j - 2.5, 3.997, 0, j - 2.5, 3.997, 2.3, j - 2.5);
    for (const z of [0.8, 1.6]) wl.push(-4, z, -2.497, 4, z, -2.497, -3.997, z, -2.5, -3.997, z, 2.5, 3.997, z, -2.5, 3.997, z, 2.5);
    ROOM.panels = seg(wl, 0x7d838c, 0.7);
    const c = document.createElement("canvas"); c.width = c.height = 128;
    const g2 = c.getContext("2d"), gr = g2.createRadialGradient(64, 64, 4, 64, 64, 64);
    gr.addColorStop(0, "rgba(255,255,255,0.95)"); gr.addColorStop(1, "rgba(255,255,255,0)");
    g2.fillStyle = gr; g2.fillRect(0, 0, 128, 128);
    const tex = new THREE.CanvasTexture(c);
    ROOM.lamps = [-2.5, 0, 2.5].map(x => {
      const lamp = new THREE.Mesh(new THREE.PlaneGeometry(1.5, 0.75), new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false }));
      lamp.position.set(x, 2.02, -2.49); hall.add(lamp); return lamp.material;
    });
  })();

  // ---------- the board on the wall: countdown of the level on top, the attempt below
  const board = (function () {
    const c = document.createElement("canvas"); c.width = 560; c.height = 400;
    const g = c.getContext("2d"), tex = new THREE.CanvasTexture(c);
    tex.colorSpace = THREE.SRGBColorSpace;
    const mesh = new THREE.Mesh(new THREE.PlaneGeometry(1.54, 1.1), new THREE.MeshBasicMaterial({ map: tex, transparent: true }));
    mesh.position.set(0, 1.63, -2.47); hall.add(mesh);
    const SEG = { 0: "abcdef", 1: "bc", 2: "abged", 3: "abgcd", 4: "fgbc", 5: "afgcd", 6: "afgedc", 7: "abc", 8: "abcdefg", 9: "abfgcd" };
    function digit(x, y, w, h, ch, color) {
      const t = w * 0.2, on = SEG[ch] || "";
      const R = { a: [x + t, y, w - 2 * t, t], d: [x + t, y + h - t, w - 2 * t, t], g: [x + t, y + h / 2 - t / 2, w - 2 * t, t],
        f: [x, y + t, t, h / 2 - t * 1.5], b: [x + w - t, y + t, t, h / 2 - t * 1.5], e: [x, y + h / 2 + t / 2, t, h / 2 - t * 1.5], c: [x + w - t, y + h / 2 + t / 2, t, h / 2 - t * 1.5] };
      for (const k in R) { g.fillStyle = on.includes(k) ? color : "#1d1f23"; g.fillRect(...R[k]); }
    }
    function panel(cx, y, text) {
      const dw = 62, dh = 110, gap = 22, w = text.length * dw + (text.length - 1) * gap + 56, x0 = cx - w / 2;
      g.fillStyle = "#0b0c0e"; g.fillRect(x0, y, w, dh + 56);
      [...text].forEach((ch, i) => digit(x0 + 28 + i * (dw + gap), y + 28, dw, dh, ch.c, ch.dim ? "#2a2c30" : "#f4f4f4"));
    }
    let key = "";
    return function (time, attempt) {
      const k = time + "|" + attempt; if (k === key) return; key = k;
      g.clearRect(0, 0, c.width, c.height);
      panel(280, 14, [...String(time).padStart(2, "0")].map(ch => ({ c: ch })));
      panel(280, 210, [{ c: "8", dim: true }, { c: "8", dim: true }, ...[...pad(attempt % 100)].map(ch => ({ c: ch }))]);
      tex.needsUpdate = true;
    };
  })();

  // ---------- Takt from the FreeCAD export (the same parts as the assembly on the page)
  const TILE = 1000 / 3;
  const PRESET = Object.fromEntries(model.presets.map(p => [p.id, p]));
  const contact = fk.contactOf(model, "claw");
  const plOf = pr => fk.placements(model, "claw", fk.poseFrames(fk.presetPose(model, pr), contact));
  const PL_STAND = plOf(PRESET.stand), PL_SIT = plOf(PRESET.sit || PRESET.stand);
  const INST = model.instances.filter(i => i.render && fk.active(i, "claw") && PL_STAND[i.name]);
  function robotMats(printed) {
    const M = {};
    for (const [k, c] of Object.entries(model.colors)) {
      const color = k === "black" ? new THREE.Color(printed) : new THREE.Color().setRGB(c[0] / 255, c[1] / 255, c[2] / 255, THREE.SRGBColorSpace);
      M[k] = k === "eye" ? new THREE.MeshBasicMaterial({ color }) : new THREE.MeshStandardMaterial({ color, roughness: { glass: 0.12, lens: 0.2, servo: 0.45, black: 0.5 }[k] ?? 0.68, metalness: { glass: 0.2, servo: 0.15 }[k] ?? 0.02 });
    }
    return M;
  }
  function Robot(printed) {
    const M = robotMats(printed);
    this.holder = new THREE.Group();
    const body = new THREE.Group(); body.rotation.x = -Math.PI / 2; body.scale.setScalar(1 / TILE); this.holder.add(body);
    this.meshes = INST.map(inst => {
      const m = new THREE.Mesh(geo[inst.key], M[model.keycolor[inst.key]] || M.black);
      m.matrixAutoUpdate = false; m.castShadow = inst.key !== "eyes" && inst.key !== "window"; m.receiveShadow = true;
      body.add(m); return m;
    });
    const mk = (w, h, d, color) => { const b = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), std(color, { roughness: 0.7 })); b.castShadow = true; b.visible = false; this.holder.add(b); return b; };
    this.load = mk(0.2, 0.13, 0.2, 0x5a5f68);
    this.loose = mk(0.26, 0.1, 0.18, 0xffd43b);
    this.key = "";
  }
  /** Pose between two placement sets, t = 0 is a, 1 is b. */
  Robot.prototype.pose = function (key, a, b, t) {
    if (key === this.key) return;
    this.key = key;
    INST.forEach((inst, i) => {
      const A = a[inst.name], B = b[inst.name], e = this.meshes[i].matrix.elements;
      // fk gives row-major 4x4, three stores column-major
      for (let r = 0; r < 4; r++) for (let c = 0; c < 4; c++) e[c * 4 + r] = lerp(A[r * 4 + c], B[r * 4 + c], t);
    });
  };
  const robot = new Robot(0x2f74ea), teacher = new Robot(0xec7d22);
  scene.add(robot.holder, teacher.holder);
  teacher.holder.visible = false;

  // ---------- level furniture
  const COL = {
    grey: [0xd8dbe0, 0xa9aeb6, 0x8b9199], podium: [0x5fd640, 0xa9aeb6, 0x8b9199], wood: [0xe6c38e, 0xc69c60, 0xa88047], heavy: [0x6a6f78, 0x4e525a, 0x3b3e45],
    pen: [0xffd43b, 0xe2b100, 0xc49800], bump: [0x7d828b, 0x5d6169, 0x4a4e55], post: [0xe9ecef, 0xc3c8cf, 0xa7adb5], bar: [0xf07d3c, 0xd0622a, 0xb2511f],
  };
  const boxMats = {};
  function matsFor(name) {
    if (!boxMats[name]) { const [t, r, l] = COL[name] || COL.grey; const T = std(t), R = std(r), L = std(l); boxMats[name] = [R, R, T, L, L, L]; }
    return boxMats[name];
  }
  const flatMat = (color, opacity = 1) => new THREE.MeshBasicMaterial({ color: C(color), transparent: true, opacity, depthWrite: false });
  const LABELS = [];                        // redrawn by setDark: light labels by day, dark ones at night
  function labelSprite(text, kind) {
    const c = document.createElement("canvas"), g = c.getContext("2d"), font = '600 44px "Inter", -apple-system, "Segoe UI", Arial, sans-serif';
    g.font = font; const w = Math.ceil(g.measureText(text).width) + 48; c.width = w; c.height = 72;
    const tex = new THREE.CanvasTexture(c); tex.colorSpace = THREE.SRGBColorSpace;
    const draw = () => {
      g.clearRect(0, 0, w, 72);
      g.font = font; g.fillStyle = DK ? "#23272e" : "rgba(255,255,255,0.95)";
      const r = 36; g.beginPath(); g.moveTo(r, 0); g.lineTo(w - r, 0); g.arc(w - r, r, r, -Math.PI / 2, Math.PI / 2); g.lineTo(r, 72); g.arc(r, r, r, Math.PI / 2, Math.PI * 1.5); g.fill();
      g.fillStyle = kind === "t" ? (DK ? "#ff9a52" : "#c2410c") : kind === "r" ? (DK ? "#ff8a8a" : "#c62828") : DK ? "#e8ebf0" : "#16191e";
      g.textAlign = "center"; g.textBaseline = "middle"; g.fillText(text, w / 2, 38);
      tex.needsUpdate = true;
    };
    draw(); LABELS.push(draw);
    const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false }));
    s.scale.set(0.24 * w / 72, 0.24, 1); s.renderOrder = 6;
    return s;
  }
  function buildLevel(l) {
    const group = new THREE.Group(), items = [];
    const rise = (obj, top) => { obj.userData.y0 = obj.position.y; obj.userData.top = top; obj.traverse(o => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } }); group.add(obj); items.push({ obj, kind: "rise" }); };
    const flat = (obj, mats) => { group.add(obj); items.push({ obj, kind: "flat", mats: mats.map(m => [m, m.opacity]) }); };
    const plane = (x0, y0, x1, y1, z, mat) => { const m = new THREE.Mesh(new THREE.PlaneGeometry(x1 - x0, y1 - y0), mat); m.rotation.x = -Math.PI / 2; m.position.copy(Wp((x0 + x1) / 2, (y0 + y1) / 2, z)); return m; };
    const box = (x0, y0, x1, y1, z0, z1, c) => { const m = new THREE.Mesh(new THREE.BoxGeometry(x1 - x0, z1 - z0, y1 - y0), matsFor(c)); m.position.copy(Wp((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)); return m; };
    const dashed = (pts, color) => { const g = new THREE.BufferGeometry().setFromPoints(pts); const ln = new THREE.Line(g, new THREE.LineDashedMaterial({ color: C(color), dashSize: 0.12, gapSize: 0.09, transparent: true, depthWrite: false })); ln.computeLineDistances(); return ln; };
    for (const p of l.props) {
      const k = p[0];
      if (k === "start") {
        const g = new THREE.Group(), base = flatMat(0x2b2d31), st = flatMat(0xf59f0b);
        g.add(plane(p[1], p[2], p[1] + 1, p[2] + 1, 0.004, base));
        for (let i = 0; i < 4; i++) g.add(plane(p[1] + 0.12, p[2] + 0.12 + i * 0.21, p[1] + 0.88, p[2] + 0.22 + i * 0.21, 0.007, st));
        flat(g, [base, st]);
      } else if (k === "goal") { const m = flatMat(0x5fd640); flat(plane(p[1], p[2], p[1] + 1, p[2] + 1, 0.006, m), [m]); }
      else if (k === "zone") {
        const m = flatMat(0x5fd640, 0.42), g = new THREE.Group(); g.add(plane(p[1], p[2], p[3], p[4], 0.006, m));
        const ln = dashed([Wp(p[1], p[2], 0.01), Wp(p[3], p[2], 0.01), Wp(p[3], p[4], 0.01), Wp(p[1], p[4], 0.01), Wp(p[1], p[2], 0.01)], 0x5fd640); g.add(ln);
        flat(g, [m, ln.material]);
      } else if (k === "ring") {
        const pts = []; for (let a = 0; a <= 64; a++) pts.push(Wp(p[1] + p[3] * Math.cos(a / 64 * 6.2832), p[2] + p[3] * Math.sin(a / 64 * 6.2832), 0.01));
        const ln = dashed(pts, 0xffffff); flat(ln, [ln.material]);
      } else if (k === "path") {
        const pts = p[1].map(q => Wp(q[0], q[1], 0.012)), g = new THREE.Group(), ln = dashed(pts, 0xffffff); g.add(ln);
        const a = pts[pts.length - 2], b = pts[pts.length - 1], m = flatMat(0xffffff);
        const tip = new THREE.Mesh(new THREE.ConeGeometry(0.055, 0.17, 14), m); tip.position.copy(b); tip.position.y = 0.03;
        tip.quaternion.setFromUnitVectors(V(0, 1, 0), b.clone().sub(a).normalize()); g.add(tip);
        flat(g, [ln.material, m]);
      } else if (k === "box") rise(box(p[1], p[2], p[3], p[4], p[5], p[6], p[7]), p[6]);
      else if (k === "ramp") {
        const [, x0, y0, x1, y1, h, c] = p, [t, , s] = COL[c] || COL.grey, g = new THREE.Group();
        const gm = v => { const b = new THREE.BufferGeometry(); b.setAttribute("position", new THREE.Float32BufferAttribute(v.flatMap(q => [q.x, q.y, q.z]), 3)); b.computeVertexNormals(); return b; };
        const A = Wp(x0, y0, 0), B = Wp(x0, y1, 0), Cc = Wp(x1, y0, 0), D = Wp(x1, y1, 0), E = Wp(x1, y0, h), F = Wp(x1, y1, h);
        g.add(new THREE.Mesh(gm([A, B, F, A, F, E]), std(t, { side: THREE.DoubleSide })));
        g.add(new THREE.Mesh(gm([B, D, F, A, E, Cc]), std(s, { side: THREE.DoubleSide })));
        rise(g, h);
      } else if (k === "cyl") {
        const [, cx, cy, r, h, side, inner] = p, g = new THREE.Group();
        const body = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, 40), [std(side), std(0xeceff2), std(0x888888)]);
        body.position.copy(Wp(cx, cy, h / 2)); g.add(body);
        const hole = new THREE.Mesh(new THREE.CircleGeometry(r * 0.82, 40), std(inner));
        hole.rotation.x = -Math.PI / 2; hole.position.copy(Wp(cx, cy, h + 0.002)); g.add(hole);
        rise(g, h);
      } else if (k === "ball") { const m = new THREE.Mesh(new THREE.SphereGeometry(p[3], 32, 20), std(0x7c3aed, { roughness: 0.4 })); m.position.copy(Wp(p[1], p[2], p[3])); rise(m, p[3] * 2); }
      else if (k === "pit") { const m = flatMat(0x040405); flat(plane(p[1], 0, p[2], 5, 0.003, m), [m]); }
      else if (k === "bumps") {
        [[2.6, 1.8, 0.4, 0.35, 0.1], [3.3, 2.7, 0.3, 0.4, 0.14], [3.9, 1.6, 0.5, 0.3, 0.08], [4.4, 2.5, 0.35, 0.35, 0.12], [5.1, 2.0, 0.3, 0.5, 0.1], [3.0, 3.4, 0.4, 0.3, 0.09], [4.7, 3.3, 0.3, 0.3, 0.13], [2.4, 2.6, 0.3, 0.3, 0.07]]
          .forEach(([x, y, w, h, z]) => rise(box(x, y, x + w, y + h, 0, z, "bump"), z));
      } else if (k === "ice") {
        const m = flatMat(0xa5dcf5, 0.5), w = flatMat(0xffffff, 0.8), g = new THREE.Group(); g.add(plane(2.2, 1.0, 5.8, 4.0, 0.005, m));
        [[2.6, 1.4, 3.4, 1.6], [3.8, 2.6, 4.9, 2.9], [4.2, 1.3, 5.2, 1.5], [2.8, 3.3, 3.6, 3.5]].forEach(([a, b, c, d]) => {
          const s = plane(a, b - 0.012, c, b + 0.012, 0.008, w); s.rotation.z = -Math.atan2(d - b, c - a); g.add(s);
        });
        flat(g, [m, w]);
      } else if (k === "line") { const m = flatMat(0xffffff, 0.8); flat(plane(p[1] - (p[4] ? 0.022 : 0.012), p[2], p[1] + (p[4] ? 0.022 : 0.012), p[3], 0.007, m), [m]); }
      else if (k === "tray") {
        const [, x0, y0, x1, y1, h] = p, g = new THREE.Group(); g.add(box(x0, y0, x1, y1, 0, h, "post"));
        g.add(plane(x0 + 0.07, y0 + 0.07, x1 - 0.07, y1 - 0.07, h + 0.002, std(0xa3a9b1)));
        rise(g, h);
      } else if (k === "push") {
        const a = Wp(...p[1]), b = Wp(...p[2]), d = b.clone().sub(a), len = d.length(), g = new THREE.Group(), m = flatMat(0xe5484d);
        const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, len - 0.2, 12), m); shaft.position.y = (len - 0.2) / 2;
        const tip = new THREE.Mesh(new THREE.ConeGeometry(0.1, 0.22, 16), m); tip.position.y = len - 0.11;
        g.add(shaft, tip); g.position.copy(a); g.quaternion.setFromUnitVectors(V(0, 1, 0), d.normalize());
        flat(g, [m]);
      } else if (k === "label") { const s = labelSprite(p[4], p[5]); s.position.copy(Wp(p[1], p[2], p[3] + 0.14)); group.add(s); items.push({ obj: s, kind: "label" }); }
    }
    const n = items.length;
    // k = 1: the level stands in the room. Going down to 0 the furniture sinks into the floor, one piece after another.
    function setK(k) {
      group.visible = k > 0.001;
      if (!group.visible) return;
      items.forEach((it, i) => {
        const ki = clamp(k * 1.7 - 0.7 * (i / Math.max(1, n - 1)));
        if (it.kind === "rise") { it.obj.position.y = it.obj.userData.y0 - (1 - easeOut(ki)) * (it.obj.userData.top + 0.04); it.obj.visible = ki > 0.001; }
        else if (it.kind === "flat") it.mats.forEach(([m, o]) => { m.opacity = o * ki; });
        else { it.obj.material.opacity = clamp(k * 3 - 2); it.obj.visible = k > 0.67; }
      });
    }
    setK(0); hall.add(group);
    return { setK };
  }
  const levelCache = {};
  const levelOf = n => levelCache[n] || (levelCache[n] = buildLevel(ROOMS[n - 1]));

  // ---------- where the robots stand in each level
  const thetaOf = yaw => Math.atan2(-Math.cos(yaw), -Math.sin(yaw));
  function spot(spec, fallback) {
    const s = spec || fallback || { x: 1.5, y: 2.5 };
    return { x: s.x, y: s.y, th: thetaOf(s.yaw || 0), zg: (s.zg || 0) + (s.air || 0), low: s.low ? 1 : 0, heights: s.heights ? 1 : 0, load: s.load ? 1 : 0, loose: s.loose ? 1 : 0, here: spec ? 1 : 0 };
  }
  const SPOT = ROOMS.map(l => spot(l.robot, l.teacher)), TSPOT = ROOMS.map(l => spot(l.teacher, l.robot));
  function mixSpot(arr, i0, i1, u) {
    const a = arr[i0 - 1], b = arr[i1 - 1], t = smooth((u - 0.2) / 0.6), o = {};
    for (const k in a) o[k] = lerp(a[k], b[k], t);
    return o;
  }
  function place(rb, s, lidTop) {
    rb.holder.position.set(s.x - 4, s.zg - (1 - s.here) * 0.95, s.y - 2.5);
    rb.holder.rotation.y = s.th;
    rb.holder.visible = s.here > 0.02;
    for (const [m, k, h] of [[rb.load, s.load, 0.065], [rb.loose, s.loose, 0.05]]) { m.visible = k > 0.02; m.scale.setScalar(Math.max(0.001, k)); m.position.y = lidTop + h * k; }
  }
  let heightKey = -1, PL_H = PL_STAND;

  /** Furnish the room for f (1..35; between two levels it is half way through the change) and pose the robots. */
  let bare = null;
  function setLevel(f, time = 0) {
    if (f < 1) {
      // Level 00, the Hall itself: only the start tile, the robot standing on it
      for (const n in levelCache) levelCache[n].setK(0);
      (bare || (bare = buildLevel({ props: [["start", 1, 2]] }))).setK(1);
      board(30, 0);
      robot.pose("s0", PL_STAND, PL_STAND, 0);
      place(robot, spot({ x: 1.5, y: 2.5 }), 0.425);
      teacher.holder.visible = false;
      return false;
    }
    f = clamp(f, 1, N);
    const lo = Math.floor(f);
    return furnish(lo, Math.min(N, lo + 1), f - lo, time);
  }
  /** The change from level a straight to level b (t = 0 is a, 1 is b), with no rooms in between. */
  function setPair(a, b, t, time = 0) {
    a = clamp(Math.round(a), 1, N); b = clamp(Math.round(b), 1, N);
    return a === b ? furnish(a, a, 0, time) : furnish(a, b, clamp(t), time);
  }
  function furnish(lo, hi, u, time) {
    if (bare) bare.setK(0);
    for (const n in levelCache) if (Number(n) !== lo && Number(n) !== hi) levelCache[n].setK(0);
    if (lo === hi) levelOf(lo).setK(1);
    else { levelOf(lo).setK(clamp(1.28 - u * 2.56)); levelOf(hi).setK(clamp(1.28 - (1 - u) * 2.56)); }
    const shown = u < 0.5 ? lo : hi, b = boardFor(shown);
    board(b.time, b.attempt);
    const s = mixSpot(SPOT, lo, hi, u), ts = mixSpot(TSPOT, lo, hi, u);
    if (s.heights > 0.5 && !reduced) {
      // Level 06: the belly goes down and up between 30 and 95 mm while the visitor scrolls through the level, and
      // stands at the middle height when the level is reached (a CAD pose, not a recorded attempt; no clock)
      const hb = Math.round(clamp(62 + 33 * Math.sin(2 * Math.PI * (lo + u)), 30, 95));
      if (hb !== heightKey) { heightKey = hb; PL_H = plOf(fk.presetAt(PRESET.stand, hb)); }
      robot.pose("h" + hb, PL_H, PL_H, 0);
    } else robot.pose("s" + s.low.toFixed(3), PL_STAND, PL_SIT, s.low);
    place(robot, s, lerp(0.425, 0.245, s.low));
    const tHere = ROOMS[lo - 1].teacher || ROOMS[hi - 1].teacher;
    if (tHere) { teacher.pose("s", PL_STAND, PL_STAND, 0); place(teacher, ts, 0.425); } else teacher.holder.visible = false;
    return false;                           // nothing in a level moves on its own
  }
  /** Where the blue robot stands in level n, in the page's frame (mm, z up, the same room centre): x, y, z, turn. */
  function robotAt(n) {
    const s = SPOT[clamp(Math.round(n), 1, N) - 1];
    return { x: (s.x - 4) * TILE, y: (2.5 - s.y) * TILE, z: s.zg * TILE, th: s.th };
  }

  /** Camera of the full roadmap: the room from the front, a little to the side. */
  function roadCamera(camera, f, W, H, phone) {
    const P = [0.5 + Math.sin(f * 0.9) * 0.55, 3.25, 6.9], T = [0, 0.4, -0.2], need = 1.62, left = phone ? 0 : 0.25, bottom = phone ? -0.75 : -0.75;
    camera.aspect = W / H;
    const pull = clamp(need / camera.aspect, 1, 3.2);
    camera.position.set(T[0] + (P[0] - T[0]) * pull, T[1] + (P[1] - T[1]) * pull, T[2] + (P[2] - T[2]) * pull);
    camera.lookAt(T[0], T[1], T[2]);
    const ox = phone ? 0 : 0.14 * left, oy = phone ? 0.2 * Math.abs(bottom) : 0.12 * bottom;
    camera.setViewOffset(W, H, -ox * W, oy * H, W, H);
    camera.updateProjectionMatrix();
  }
  /** Camera of a card picture: the whole room, straight on. */
  function cardCamera(camera, W, H) {
    const P = [0.3, 2.55, 5.5], T = [0, 0.4, -0.2];
    camera.aspect = W / H;
    const pull = clamp(1.55 / camera.aspect, 0.95, 3);
    camera.position.set(T[0] + (P[0] - T[0]) * pull, T[1] + (P[1] - T[1]) * pull, T[2] + (P[2] - T[2]) * pull);
    camera.lookAt(T[0], T[1], T[2]);
    camera.clearViewOffset();
    camera.updateProjectionMatrix();
  }
  /** Build these levels' rooms ahead of time, so a change never has to build one. */
  const prebuild = ns => ns.forEach(n => levelOf(clamp(n, 1, N)));

  /** Light or dark: the Hall by day, or the Hall at night. Walls, floor, light and the labels change; the board, the
   *  furniture and the robots keep their colours. The caller draws the next frame. */
  function setDark(d) {
    DK = !!d;
    scene.background.set(DK ? 0x15171b : 0x101216);
    ROOM.back.color.set(DK ? 0x2a2f37 : 0xe3e6ea);
    ROOM.sides.forEach(m => m.color.set(DK ? 0x252a31 : 0xc4c8ce));
    ROOM.floor.color.set(DK ? 0x1c1e22 : 0x3a3c41);
    ROOM.tiles.color.set(DK ? 0xc8ccd3 : 0xd9dbe0); ROOM.tiles.opacity = DK ? 0.16 : 0.6;
    ROOM.panels.color.set(DK ? 0x3c434e : 0x7d838c); ROOM.panels.opacity = DK ? 0.9 : 0.7;
    ROOM.lamps.forEach(m => { m.opacity = DK ? 0.22 : 1; });
    hemi.color.set(DK ? 0xa9bbd8 : 0xffffff); hemi.groundColor.set(DK ? 0x15171b : 0x4f545e); hemi.intensity = (DK ? 0.47 : 0.78) * Math.PI;
    sun.intensity = (DK ? 0.56 : 0.8) * Math.PI; fill.intensity = DK ? 0.32 * Math.PI : 0;
    LABELS.forEach(r => r());
  }
  setDark(dark);
  return { scene, setLevel, setPair, prebuild, roadCamera, cardCamera, robotAt, setDark, N };
}
