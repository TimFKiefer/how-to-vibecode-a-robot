// SPDX-License-Identifier: MIT
// guide.js: the Takt build guide. The step text works on its own; the 3D view (three.js 0.160.0, the FreeCAD
// export in model.json and meshes.bin, the leg frames from fk.js) is added when it loads.
// Coordinates: the floor frame of the CAD script (cad/takt.py) in mm (x right, y front, z up, floor z = 0). One group turns it into
// three.js's y-up world.
//
// The page is scroll-driven. One "unit" of scroll is one step. Inside a step:
//   0 .. A        the camera moves from the last step's view to this one, the step's new parts appear lying on the
//                 table beside the build (a change of pose, like a part laid on its side, crossfades here)
//   A .. B_END    the new parts are lifted along an arc to the start of their fit and slide in, in the order and
//                 overlap of their `delay` and `dur`
//   B_END .. 1    they are in place; their highlight fades
// Every picture is a function of the scroll position only: nothing moves on its own, scrolling back takes the
// parts back out. Dragging turns the view; scrolling on eases that turn back to the guide's view.

import { STEPS, COVER, ICON, TOOLS, VIRTUAL, LEGS, FOCUS, DRAFT } from "./guide-steps.js";

const $ = (id) => document.getElementById(id);
const N = STEPS.length;
const pad = (n) => String(n).padStart(2, "0");
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
const clamp = (x, a = 0, b = 1) => Math.min(b, Math.max(a, x));
const lerp = (a, b, t) => a + (b - a) * t;
const easeIO = (t) => { t = clamp(t); return t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2; };
const reduced = matchMedia("(prefers-reduced-motion: reduce)").matches;
const LIFT = 40;           // the robot rests on a box while it is built: 40 mm higher than its stand
const COVER_LEN = 0.3;     // scroll units of the start page before step 1 begins
const A = 0.18;            // see the top of the file
const B_END = 0.9;
const REST_T = 0.98;       // a step's end rest point: all parts in place, the highlight almost gone
const REST_NEAR = 0.12;    // a stop closer than this (in units) to a rest point glides onto it; a farther stop stays put
const END_U = COVER_LEN + N;
const FADE0 = 0.3;         // how strong a part that fades in (no start place) is when its step begins

let cur = 0;               // 0 = start page, 1..N = steps
let u = 0;                 // the scroll position in units
let stepPx = 700;          // pixels of scroll per step
let trackTop = 0;
let viewer = null;

const segStart = (k) => (k === 0 ? 0 : COVER_LEN + (k - 1));
const landing = (k) => (k <= 0 ? 0 : k > N ? END_U : segStart(k) + A);
// where the page settles when the visitor stops near one: the cover, each step's landing and end, the finished robot
const restPoints = () => [0, ...STEPS.flatMap((s, i) => [landing(i + 1), segStart(i + 1) + REST_T]), END_U];
function locate(x) {
  if (x < COVER_LEN) return { k: 0, t: clamp(x / COVER_LEN) };
  const k = Math.min(N, Math.floor(x - COVER_LEN) + 1);
  return { k, t: clamp(x - segStart(k)) };
}

// ---------------------------------------------------------------- the text side
const TOOL_SVG = {
  screwdriver: '<path d="M6 26l8-8M14 18l3 3M17 21l9-9a2.5 2.5 0 0 0-3.5-3.5l-9 9" /><path d="M4 28l3-3" />',
  hex: '<path d="M8 6v14a4 4 0 0 0 4 4h12" /><path d="M8 6h4v14h12v4" />',
  iron: '<path d="M4 28l6-6" /><rect x="9" y="13" width="7" height="12" rx="2" transform="rotate(45 12.5 19)" /><path d="M19 13l8-8" />',
  insertTip: '<path d="M10 22l12-12" /><path d="M20 8l4 4" /><circle cx="8" cy="24" r="3" />',
  computer: '<rect x="5" y="7" width="22" height="14" rx="2" /><path d="M3 25h26" />',
  tape: '<circle cx="14" cy="16" r="9" /><circle cx="14" cy="16" r="4" /><path d="M23 16h6" />',
  shrink: '<rect x="5" y="12" width="22" height="8" rx="4" /><path d="M11 12v8M21 12v8" />',
  flat: '<path d="M5 27l9-9" /><rect x="13" y="9" width="6" height="12" rx="2" transform="rotate(45 16 15)" /><path d="M20 12l7-7M25 3l4 4" />',
  nut: '<path d="M6 26l11-11" /><path d="M17 15a6 6 0 1 1 6 3l-3-3 3-5" />',
};
const toolIcon = (k) => `<svg viewBox="0 0 32 32" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">${TOOL_SVG[k] || ""}</svg>`;
const NOTE_LABEL = { rec: "Recommended", confirm: "To be confirmed in the first real build", info: "Good to know" };

function partsHTML(parts, legs) {
  if (!parts.length) return "";
  return `<h3 class="sub">You need</h3><ul class="parts">${parts.map((p) => `
    <li><span class="pic" data-icon="${p.icon}"><img alt="" hidden></span>
      ${p.n == null ? "" : `<span class="n">x${p.n}</span>`}
      <span class="nm">${esc(p.name)}${p.per ? `<small>${esc(p.per)}</small>` : ""}</span></li>`).join("")}</ul>`;
}

function stepHTML(i) {
  const s = STEPS[i - 1];
  const notes = (s.notes || []).map((n) => `<p class="note ${n.k}"><b>${NOTE_LABEL[n.k]}</b>${esc(n.t)}</p>`).join("");
  const tools = s.tools.length ? `<h3 class="sub">Tools</h3><ul class="tools">${s.tools.map((t) => `<li>${toolIcon(t)}<span>${esc(TOOLS[t])}</span></li>`).join("")}</ul>` : "";
  return `
    <div class="head">
      <div class="num" aria-hidden="true">${i}</div>
      <div><p class="kicker">Step ${i} of ${N}${s.sub ? " · on the table" : ""}</p><h2>${esc(s.title)}</h2></div>
    </div>
    ${s.legs ? `<p class="six"><svg viewBox="-14 -15 28 30" aria-hidden="true"><polygon points="0,-13 11.3,-6.5 11.3,6.5 0,13 -11.3,6.5 -11.3,-6.5" fill="none" stroke="currentColor" stroke-width="2.4"/>${[0, 60, 120, 180, 240, 300].map((a) => `<circle cx="${(10 * Math.cos((a - 90) * Math.PI / 180)).toFixed(2)}" cy="${(10 * Math.sin((a - 90) * Math.PI / 180)).toFixed(2)}" r="2.6" fill="currentColor"/>`).join("")}</svg>Shown on one leg. Do this for all 6 legs.</p>` : ""}
    ${partsHTML(s.parts, s.legs)}
    ${tools}
    <ol class="say">${s.say.map((t) => `<li>${esc(t)}</li>`).join("")}</ol>
    ${notes}
    <p class="draftline">${esc(DRAFT)} This step will be corrected after the first real build.</p>
    <p class="src">Based on: ${esc(s.src)}</p>`;
}

function coverHTML() {
  return `
    <p class="kicker">Step-by-step build guide</p>
    <h2 class="big">${esc(COVER.title)}</h2>
    <p class="lead">${esc(COVER.lead)}</p>
    <ul class="facts">${COVER.facts.map(([a, b]) => `<li><b>${esc(a)}</b><span>${esc(b)}</span></li>`).join("")}</ul>
    <p class="note confirm"><b>Read this first</b>This is a draft from the CAD model. Nothing is bought, printed or built yet, so nobody has followed these steps. The guide will be corrected after the first real build.</p>
    <p>Every picture uses the real parts from the FreeCAD model. The printed parts are drawn blue; the parts list prints them in black PETG. Electronics without a CAD model are drawn as simple boxes. A part the design gives no place yet lies on the table beside the build, as a simple stand-in.</p>
    <p>Like the whole robot, this guide is built in public and fully vibe-codable: it was made by talking to AI coding agents, and you can do it yourself.</p>
    <p class="cta"><button type="button" class="go" data-go="1">Start with step 1</button><a class="ghostbtn" href="bom.html">Open the parts list</a></p>`;
}

const FOOT = `<p class="foot">Takt is built in public. <a href="index.html">Home</a> · <a href="bom.html">Parts list</a> · Scroll to build. The arrow keys jump between steps.</p>`;

function showCard(k, dir) {
  const card = $("card");
  card.innerHTML = `<div class="cin">${k === 0 ? coverHTML() : stepHTML(k)}${FOOT}</div>`;
  card.scrollTop = 0;
  card.querySelectorAll("[data-go]").forEach((b) => b.addEventListener("click", () => go(+b.dataset.go)));
  fillThumbs();
  const cin = card.firstElementChild;
  if (cin.animate) cin.animate(reduced ? [{ opacity: 0 }, { opacity: 1 }]
    : [{ opacity: 0, transform: `translateY(${dir * 18}px)` }, { opacity: 1, transform: "none" }],
  { duration: reduced ? 150 : 340, easing: "cubic-bezier(0.2, 0.8, 0.2, 1)" });
}

function setStep(k, dir) {
  cur = k;
  showCard(k, dir);
  const lbl = $("next").querySelector(".lbl");
  const word = k === 0 ? "Start" : k === N ? "Done" : "Next";
  if (lbl.textContent !== word) {
    lbl.textContent = word;
    if (lbl.animate) lbl.animate([{ opacity: 0, transform: reduced ? "none" : "translateY(6px)" }, { opacity: 1, transform: "none" }], { duration: 220, easing: "ease-out" });
  }
  document.querySelectorAll("#prog button").forEach((b, j) => {
    b.classList.toggle("done", j < k); b.classList.toggle("on", j === k);
    if (j === k) b.setAttribute("aria-current", "step"); else b.removeAttribute("aria-current");
    if (j !== k) b.firstElementChild.style.removeProperty("--f");
  });
  $("count").textContent = k === 0 ? `${N} steps` : `Step ${k} / ${N}`;
  try { history.replaceState(null, "", k === 0 ? location.pathname + location.search : `#step-${k}`); } catch (e) { /* file or sandbox */ }
  document.title = k === 0 ? "Build Takt" : `Step ${k}: ${STEPS[k - 1].title} · Build Takt`;
}

function updateControls(k, t) {
  const bar = document.querySelector("#prog button.on i");
  if (bar) bar.style.setProperty("--f", k === 0 ? t.toFixed(3) : clamp((t - A) / (B_END - A)).toFixed(3));
  $("prev").disabled = u <= 0.001;
  $("next").disabled = u >= END_U - 0.004;
  $("replay").disabled = k === 0 || u <= landing(k) + 0.01;
}

function scrollToU(x) {
  const top = Math.round(clamp(x, 0, END_U) * stepPx + trackTop);
  scrollTo({ top, behavior: reduced ? "auto" : "smooth" });
  if (reduced) { const st = $("stage"); if (st.animate) st.animate([{ opacity: 0.55 }, { opacity: 1 }], { duration: 160, easing: "ease-out" }); }
}
// when the scroll stops near a rest point, the page settles exactly there. Not CSS scroll snapping: its proximity
// range is a third of the view in Chrome, so nearly every stop inside a step jumped to the landing or the end.
function settle() {
  aimK = null;
  const p = restPoints().reduce((a, b) => (Math.abs(b - u) < Math.abs(a - u) ? b : a));
  const d = Math.abs(p - u) * stepPx;
  if (d >= 1 && d < REST_NEAR * stepPx) scrollToU(p);
}
let settleT = 0;
// quick presses while the page is still gliding count from the step it is gliding to
let aimK = null, aimAt = 0;
const baseK = () => (aimK !== null && performance.now() - aimAt < 1200 ? aimK : locate(u).k);
function go(k) {
  k = clamp(k, 0, N + 1);
  aimK = Math.min(k, N); aimAt = performance.now();
  if (k > N) return scrollToU(END_U);
  scrollToU(landing(k));
}

function onScroll() {
  const nu = clamp((scrollY - trackTop) / stepPx, 0, END_U);
  const du = nu - u; u = nu;
  const { k, t } = locate(u);
  if (k !== cur) setStep(k, du < 0 ? -1 : 1);
  updateControls(k, t);
  if (viewer) { viewer.scrolled(du); viewer.request(); }
}

let lastW = 0;
function layout() {
  const barH = document.querySelector(".bar").offsetHeight, ctrlH = $("ctrl").offsetHeight;
  const pinH = Math.max(340, innerHeight - barH - ctrlH);
  const keep = u;
  let changed = false;
  if (innerWidth !== lastW) {   // the phone's address bar changes only the height: keep the step length then
    lastW = innerWidth;
    const px = Math.round(clamp(innerHeight * 0.8, 480, 860));
    changed = px !== stepPx; stepPx = px;
  }
  const r = document.documentElement.style;
  r.setProperty("--barh", barH + "px"); r.setProperty("--pinh", pinH + "px");
  document.body.style.paddingBottom = ctrlH + "px";
  $("track").style.height = Math.round(END_U * stepPx + pinH) + "px";
  trackTop = $("track").offsetTop - barH;
  if (changed) scrollTo({ top: Math.round(keep * stepPx + trackTop), behavior: "auto" });
}

function fillThumbs() {
  if (!viewer) return;
  document.querySelectorAll("#card .pic[data-icon]").forEach((el) => {
    const img = el.querySelector("img");
    const url = viewer.thumb(el.dataset.icon);
    if (url) { img.src = url; img.hidden = false; el.classList.add("has"); }
  });
}

function buildProgress() {
  const prog = $("prog");
  const labels = ["Start", ...STEPS.map((s, k) => `Step ${k + 1}: ${s.title}`)];
  prog.innerHTML = labels.map((l) => `<button type="button" title="${esc(l)}" aria-label="${esc(l)}"><i></i></button>`).join("");
  prog.querySelectorAll("button").forEach((b, k) => b.addEventListener("click", () => go(k)));
}

// a short spring on every press, also for the keyboard (CSS :active covers only the pointer)
function pressFeedback() {
  document.addEventListener("click", (e) => {
    const b = e.target.closest && e.target.closest("button, .go, .ghostbtn, .links a, .back, .brand");
    if (!b || b.disabled) return;
    b.classList.remove("pressed"); void b.offsetWidth; b.classList.add("pressed");
  });
  document.addEventListener("animationend", (e) => { if (e.animationName === "press" || e.animationName === "fadepress") e.target.classList.remove("pressed"); });
}

// back to where the visitor came from on the home page; a visitor who came from elsewhere goes to the project part
function backButton() {
  $("back").addEventListener("click", (e) => {
    let same = false;
    try { same = !!document.referrer && new URL(document.referrer).origin === location.origin; } catch (err) { same = false; }
    if (same && history.length > 1) { e.preventDefault(); history.back(); }
  });
}

const fromHash = () => { const m = /^#step-(\d+)$/.exec(location.hash); return m ? clamp(+m[1], 0, N) : 0; };

function startUI() {
  buildProgress();

  pressFeedback();
  backButton();
  $("prev").addEventListener("click", () => go(baseK() - 1));
  $("next").addEventListener("click", () => go(baseK() + 1));
  $("replay").addEventListener("click", () => scrollToU(landing(locate(u).k)));
  addEventListener("keydown", (e) => {
    if (e.target.closest && e.target.closest("input, textarea")) return;
    if (e.metaKey || e.ctrlKey || e.altKey) return;
    const k = baseK();
    if (e.key === "ArrowRight" || e.key === "ArrowDown" || e.key === "PageDown") { e.preventDefault(); go(k + 1); }
    else if (e.key === "ArrowLeft" || e.key === "ArrowUp" || e.key === "PageUp") { e.preventDefault(); go(k - 1); }
    else if (e.key === "Home") { e.preventDefault(); go(0); }
    else if (e.key === "End") { e.preventDefault(); scrollToU(END_U); }
  });
  addEventListener("hashchange", () => { const k = fromHash(); if (k !== locate(u).k) go(k); });
  addEventListener("scroll", onScroll, { passive: true });
  if ("onscrollend" in window) addEventListener("scrollend", settle);
  else addEventListener("scroll", () => { clearTimeout(settleT); settleT = setTimeout(settle, 180); }, { passive: true });
  for (const ev of ["wheel", "touchstart"]) addEventListener(ev, () => { aimK = null; }, { passive: true });
  addEventListener("resize", () => { layout(); onScroll(); });
  // a link to #step-N opens that step (the browser's own scroll memory would put it elsewhere)
  const want = fromHash();
  if (want && "scrollRestoration" in history) history.scrollRestoration = "manual";
  layout();
  u = clamp((scrollY - trackTop) / stepPx, 0, END_U);
  if (want && locate(u).k !== want) scrollTo({ top: Math.round(landing(want) * stepPx + trackTop), behavior: "auto" });
  u = clamp((scrollY - trackTop) / stepPx, 0, END_U);
  setStep(locate(u).k, 1);
  onScroll();
}

// ---------------------------------------------------------------- the 3D side
async function startViewer() {
  let THREE, U, fk, model, meshes;
  try {
    THREE = await import("three");
    U = await import("three/addons/utils/BufferGeometryUtils.js");
    fk = await import("./fk.js");
    const { loadMeshes } = await import("./meshes.js");
    [model, meshes] = await Promise.all([
      fetch("./model.json").then((r) => { if (!r.ok) throw new Error("./model.json " + r.status); return r.json(); }),
      loadMeshes(),
    ]);
  } catch (err) { return no3d(err); }

  const canvas = $("gl"), stage = $("stage");
  let renderer;
  try { renderer = new THREE.WebGLRenderer({ canvas, antialias: true }); } catch (err) { return no3d(err); }
  renderer.setPixelRatio(Math.min(devicePixelRatio || 1, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0xdfe2e7);
  const hemi = new THREE.HemisphereLight(0xffffff, 0x4f545e, 0.85 * Math.PI); scene.add(hemi);
  const sun = new THREE.DirectionalLight(0xffffff, 0.75 * Math.PI);
  sun.position.set(380, 900, 520);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, { left: -450, right: 450, top: 450, bottom: -450, near: 100, far: 2200 });
  sun.shadow.bias = -0.0005; sun.shadow.normalBias = 0.6;
  scene.add(sun);
  const fill = new THREE.DirectionalLight(0xffffff, 0.25 * Math.PI); fill.position.set(-500, 300, -400); scene.add(fill);

  const camera = new THREE.PerspectiveCamera(34, 1, 5, 8000);

  // floor frame (z up, mm) inside three's y-up world
  const root = new THREE.Group(); root.rotation.x = -Math.PI / 2; scene.add(root);
  const w = (x, y, z) => new THREE.Vector3(x, z, -y);   // floor frame to world

  // ---------- the Hall: light panel walls, dark tiled floor (a tile is a third of a metre), the black board
  const hallM = (function hall() {
    // walls are drawn from both sides, so the camera never looks through one
    const S = 3000, H = 1500, TILE = 1000 / 3;
    const std = (c, o = {}) => new THREE.MeshStandardMaterial(Object.assign({ color: c, roughness: 0.85, metalness: 0 }, o));
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(S, S), std(0x3a3c41, { roughness: 0.78 })), wallM = [];
    floor.receiveShadow = true; root.add(floor);
    const lines = [];
    for (let k = -4; k <= 4; k++) { lines.push(k * TILE, -S / 2, 0.6, k * TILE, S / 2, 0.6, -S / 2, k * TILE, 0.6, S / 2, k * TILE, 0.6); }
    const lg = new THREE.BufferGeometry(); lg.setAttribute("position", new THREE.Float32BufferAttribute(lines, 3));
    const tileL = new THREE.LineBasicMaterial({ color: 0xd9dbe0, transparent: true, opacity: 0.35 });
    root.add(new THREE.LineSegments(lg, tileL));
    const walls = [[0, -S / 2, 0, 0xe3e6ea], [S / 2, 0, -Math.PI / 2, 0xc9cdd3], [-S / 2, 0, Math.PI / 2, 0xc9cdd3], [0, S / 2, Math.PI, 0xd6d9de]];
    const wl = [];
    for (const [x, y, rot, c] of walls) {
      const m = new THREE.Mesh(new THREE.PlaneGeometry(S, H), std(c, { roughness: 0.95, side: THREE.DoubleSide })); wallM.push(m.material);
      m.rotation.x = Math.PI / 2; m.rotation.y = rot; m.position.set(x, y, H / 2); root.add(m);
    }
    for (let k = -4; k <= 4; k++) wl.push(k * TILE, -S / 2 + 1, 0, k * TILE, -S / 2 + 1, H);
    for (const z of [400, 800, 1200]) wl.push(-S / 2, -S / 2 + 1, z, S / 2, -S / 2 + 1, z);
    const wg = new THREE.BufferGeometry(); wg.setAttribute("position", new THREE.Float32BufferAttribute(wl, 3));
    const wallL = new THREE.LineBasicMaterial({ color: 0x7d838c, transparent: true, opacity: 0.55 });
    root.add(new THREE.LineSegments(wg, wallL));
    return { floor: floor.material, tileL, wallM, wallL };
  })();
  const board = (function () {
    const c = document.createElement("canvas"); c.width = 512; c.height = 320;
    const g = c.getContext("2d"), tex = new THREE.CanvasTexture(c); tex.colorSpace = THREE.SRGBColorSpace;
    const m = new THREE.Mesh(new THREE.PlaneGeometry(640, 400), new THREE.MeshBasicMaterial({ map: tex }));
    m.rotation.set(Math.PI / 2, 0, Math.PI, "ZXY"); m.position.set(0, -1497, 820); root.add(m);   // on the back wall, facing the room
    const SEG = { 0: "abcdef", 1: "bc", 2: "abged", 3: "abgcd", 4: "fgbc", 5: "afgcd", 6: "afgedc", 7: "abc", 8: "abcdefg", 9: "abfgcd" };
    function digit(x, y, wd, h, ch) {
      const t = wd * 0.2, on = SEG[ch] || "";
      const R = { a: [x + t, y, wd - 2 * t, t], d: [x + t, y + h - t, wd - 2 * t, t], g: [x + t, y + h / 2 - t / 2, wd - 2 * t, t],
        f: [x, y + t, t, h / 2 - t * 1.5], b: [x + wd - t, y + t, t, h / 2 - t * 1.5], e: [x, y + h / 2 + t / 2, t, h / 2 - t * 1.5], c: [x + wd - t, y + h / 2 + t / 2, t, h / 2 - t * 1.5] };
      for (const k in R) { g.fillStyle = on.includes(k) ? "#f4f4f4" : "#1d1f23"; g.fillRect(...R[k]); }
    }
    return (n) => {
      g.fillStyle = "#0b0c0e"; g.fillRect(0, 0, 512, 320);
      const txt = pad(n);
      [...txt].forEach((ch, k) => digit(150 + k * 120, 40, 90, 170, ch));
      g.fillStyle = "#aab1bb"; g.font = "600 34px Inter, Arial, sans-serif"; g.textAlign = "center";
      g.fillText(n === 0 ? "BUILD GUIDE" : `OF ${N}`, 256, 280);
      tex.needsUpdate = true;
    };
  })();

  // ---------- materials and geometries
  const GEO = {};
  for (const [key, m] of Object.entries(meshes)) {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(m.pos, 3));
    g.setIndex(new THREE.BufferAttribute(m.idx, 1));
    GEO[key] = U.toCreasedNormals(g, (32 * Math.PI) / 180); GEO[key].computeBoundingSphere();
  }
  const MAT = {};
  for (const [k, c] of Object.entries(model.colors)) {
    const color = k === "black" ? new THREE.Color(0x2f74ea) : new THREE.Color().setRGB(c[0] / 255, c[1] / 255, c[2] / 255, THREE.SRGBColorSpace);
    MAT[k] = k === "eye" ? new THREE.MeshBasicMaterial({ color }) : new THREE.MeshStandardMaterial({ color, roughness: { glass: 0.12, lens: 0.2, servo: 0.45, black: 0.5 }[k] ?? 0.68, metalness: { glass: 0.2, servo: 0.15 }[k] ?? 0.02 });
  }
  const brass = new THREE.MeshStandardMaterial({ color: 0xc9a227, roughness: 0.35, metalness: 0.75 });
  const steel = new THREE.MeshStandardMaterial({ color: 0xb9bec6, roughness: 0.32, metalness: 0.8 });
  const boxMat = (c) => new THREE.MeshStandardMaterial({ color: c, roughness: 0.6, metalness: 0.05 });
  const HI = new Map();
  const hiOf = (m) => {
    if (!HI.has(m)) { const h = m.clone(); if (h.emissive) { h.emissive = new THREE.Color(0xf59f0b); h.emissiveIntensity = 0.25;
      // a dark part (the servos) only needs a little glow to stand out
      const c = m.color || new THREE.Color(1, 1, 1); h.userData.k = c.r + c.g + c.b < 0.3 ? 0.35 : 1; } HI.set(m, h); }
    return HI.get(m);
  };

  function screwGeo(d, len, head, rings = true) {
    const parts = [];
    const shaft = new THREE.CylinderGeometry(d / 2, d / 2, len, rings ? 16 : 8); shaft.translate(0, -len / 2, 0); parts.push(shaft);
    let h;
    if (head === "cs") { h = new THREE.CylinderGeometry(d, d / 2, d * 0.55, 20); h.translate(0, d * 0.275, 0); }
    else if (head === "pan") { h = new THREE.CylinderGeometry(d * 0.95, d * 0.95, d * 0.6, 20); h.translate(0, d * 0.3, 0); }
    else if (head === "cheese") { h = new THREE.CylinderGeometry(d * 0.85, d * 0.85, d * 0.65, 20); h.translate(0, d * 0.33, 0); }
    else { h = new THREE.CylinderGeometry(d * 0.8, d * 0.8, d, 20); h.translate(0, d * 0.5, 0); }
    parts.push(h);
    if (rings) for (let k = 1; k < len / (d * 0.45); k++) { const r = new THREE.TorusGeometry(d / 2, d * 0.06, 4, 16); r.rotateX(Math.PI / 2); r.translate(0, -k * d * 0.45, 0); parts.push(r); }
    return U.mergeGeometries(parts.map((p) => p.toNonIndexed()));
  }
  // fit_gauge and fit_collar of cad/takt.py, in the servo's frame (shaft on z, x from tail to nose, mm). No hole patterns.
  function gaugeGeo() {
    const HORN_FACE = 33.5, IDLER_FACE = -3.7, T = 2.5;
    const plate = (r, z0) => {
      const s = new THREE.Shape(); s.moveTo(17.5, -12); s.lineTo(0, -r); s.absarc(0, 0, r, -Math.PI / 2, Math.PI / 2, true); s.lineTo(17.5, 12); s.lineTo(17.5, -12);
      const g = new THREE.ExtrudeGeometry(s, { depth: T, bevelEnabled: false, curveSegments: 20 }); g.translate(0, 0, z0); return g;   // an extruded shape has no index already
    };
    const web = new THREE.BoxGeometry(2.5, 24, HORN_FACE - IDLER_FACE + 2 * T); web.translate(16.25, 0, (HORN_FACE + IDLER_FACE) / 2);
    return U.mergeGeometries([plate(12, HORN_FACE), plate(10.2, IDLER_FACE - T), web.toNonIndexed()]);
  }
  function collarGeo() {
    const FIT = 0.25, W = 24.73 / 2 + FIT, Z0 = -1.5 - FIT, Z1 = 30.5 + FIT, X0 = -35.03 - FIT, X1 = -24, t = 1.6;
    const bx = (x0, x1, y0, y1, z0, z1) => { const g = new THREE.BoxGeometry(x1 - x0, y1 - y0, z1 - z0); g.translate((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2); return g.toNonIndexed(); };
    return U.mergeGeometries([bx(X0 - t, X1, -W - t, W + t, Z1, Z1 + t), bx(X0 - t, X1, -W - t, W + t, Z0 - t, Z0),
      bx(X0 - t, X1, W, W + t, Z0, Z1), bx(X0 - t, X1, -W - t, -W, Z0, Z1), bx(X0 - t, X0, -W, W, Z0, Z1)]);
  }
  function insertGeo(r, len) { const g = new THREE.CylinderGeometry(r, r, len, 10); return g; }
  function nutGeo(d) {
    const s = new THREE.Shape(); for (let k = 0; k < 6; k++) { const a = (k / 6) * Math.PI * 2; s[k ? "lineTo" : "moveTo"](Math.cos(a) * d * 1.05, Math.sin(a) * d * 1.05); }
    const hole = new THREE.Path(); hole.absarc(0, 0, d / 2, 0, Math.PI * 2, true); s.holes.push(hole);
    const g = new THREE.ExtrudeGeometry(s, { depth: d * 0.8, bevelEnabled: false, curveSegments: 16 }); g.rotateX(-Math.PI / 2); return g;
  }
  // a group of screws (or nuts) as one merged geometry, the same for every leg: shared, so many screws stay cheap
  const HW = new Map();
  function hwGeo(v) {
    const key = JSON.stringify([v.kind, v.d, v.len, v.head, v.at, v.dir, v.axis]);
    if (!HW.has(key)) {
      let one;
      if (v.kind === "nuts") one = nutGeo(v.d).rotateX(Math.PI / 2);   // along +z, from its place up
      else { one = screwGeo(v.d, v.len, v.head, false);                  // the shaft along -y from the head
        if (v.axis === "y") { if (v.dir > 0) one.rotateX(Math.PI); } else one.rotateX(v.dir < 0 ? Math.PI / 2 : -Math.PI / 2); }
      HW.set(key, U.mergeGeometries(v.at.map(([x, y, z]) => one.clone().translate(x, y, z))));
    }
    return HW.get(key);
  }
  function boxWithEdges(wd, dp, h, color) {
    const g = new THREE.Group();
    const m = new THREE.Mesh(new THREE.BoxGeometry(wd, dp, h), boxMat(color)); m.castShadow = true; m.receiveShadow = true; g.add(m);
    g.add(new THREE.LineSegments(new THREE.EdgesGeometry(m.geometry), new THREE.LineBasicMaterial({ color: 0x15181d, transparent: true, opacity: 0.55 })));
    return g;
  }

  // ---------- the robot in its stand pose (model.json preset "stand"), lifted onto the build box
  const PRESET = Object.fromEntries(model.presets.map((p) => [p.id, p]));
  const pf = fk.poseFrames(fk.presetPose(model, PRESET.stand), fk.contactOf(model, "claw"));
  const PL = fk.placements(model, "claw", pf);
  const M4 = (a) => new THREE.Matrix4().set(...a);
  const robotG = new THREE.Group(); root.add(robotG);
  const HIPDIR = Object.fromEntries(fk.HIPS.map(([n, phi]) => [n, [Math.cos(phi * Math.PI / 180), Math.sin(phi * Math.PI / 180)]]));
  const ITEMS = new Map();   // id -> { obj, M, leg, meshes: [mesh], base: [material] }
  const legOf = (id) => (LEGS.includes(id.slice(0, 2)) && id[2] === "_" ? id.slice(0, 2) : null);
  function addItem(id, obj, M, leg, parent = robotG) {
    obj.matrixAutoUpdate = false; obj.matrix.copy(M); obj.visible = false; parent.add(obj);
    const ms = []; obj.traverse((o) => { if (o.isMesh || o.isLine) ms.push(o); });
    const it = { id, obj, M: M.clone(), leg, meshes: ms, base: ms.map((m) => m.material), cast: ms.map((m) => !!m.castShadow), parent };
    ITEMS.set(id, it); return it;
  }
  for (const inst of model.instances) {
    if (!inst.render || !fk.active(inst, "claw") || !PL[inst.name]) continue;
    const mesh = new THREE.Mesh(GEO[inst.key], MAT[model.keycolor[inst.key]] || MAT.black);
    mesh.castShadow = inst.key !== "eyes" && inst.key !== "window"; mesh.receiveShadow = true;
    const g = new THREE.Group(); g.add(mesh);
    addItem(inst.name, g, M4(PL[inst.name]), legOf(inst.name));
  }
  const B = pf.B;
  const plug = (leg, s) => fk.apply(pf.frames[leg][s], [-14.25, 0, -3.5]);
  // the fit test on the table: the servo lies with its shaft up, its plugs on the table (z: the height of its frame)
  const fitM = ([x, y, rz], z = 8.5) => new THREE.Matrix4().makeTranslation(x, y, z).multiply(new THREE.Matrix4().makeRotationZ(rz * Math.PI / 180));
  for (const v of VIRTUAL) {
    let obj, M;
    if (v.kind === "insert" || v.kind === "disc") {
      const len = v.z1 - v.z0;
      const geo = v.kind === "insert" ? insertGeo(v.r, len) : new THREE.CylinderGeometry(v.r, v.r, len, 32);
      geo.rotateX(Math.PI / 2);
      obj = new THREE.Group(); const m = new THREE.Mesh(geo, v.kind === "insert" ? brass : boxMat(v.color)); m.castShadow = true; obj.add(m);
      M = M4(fk.mul(B, fk.T(v.x, v.y, (v.z0 + v.z1) / 2)));
    } else if (v.kind === "insertX") {
      const geo = insertGeo(v.r, v.x1 - v.x0); geo.rotateZ(Math.PI / 2);
      obj = new THREE.Group(); obj.add(new THREE.Mesh(geo, brass));
      M = M4(fk.mul(pf.frames[v.leg][v.frame], fk.T((v.x0 + v.x1) / 2, v.y, 0)));
    } else if (v.kind === "screwX") {
      const geo = screwGeo(3, v.len, "cs"); geo.rotateZ(Math.PI / 2);   // head at x0, shaft into the blade
      obj = new THREE.Group(); obj.add(new THREE.Mesh(geo, steel));
      M = M4(fk.mul(pf.frames[v.leg][v.frame], fk.T(v.x0, v.y, 0)));
    } else if (v.kind === "cable") {
      const [a, b] = v.link === "32" ? [plug(v.leg, "s3"), plug(v.leg, "s2")] : v.link === "21" ? [plug(v.leg, "s2"), plug(v.leg, "s1")]
        : [plug(v.leg, "s1"), fk.apply(B, [HIPDIR[v.leg][0] * 30, HIPDIR[v.leg][1] * 30, 40])];
      // a loose loop over the joint: drawn, not routed (hardware/DESIGN.md: cable paths are not designed yet)
      const [hx, hy] = HIPDIR[v.leg];
      const mid = [(a[0] + b[0]) / 2 + hx * 6, (a[1] + b[1]) / 2 + hy * 6, Math.max(a[2], b[2]) + 16];
      const curve = new THREE.CatmullRomCurve3([new THREE.Vector3(...a), new THREE.Vector3(...mid), new THREE.Vector3(...b)]);
      const geo = new THREE.TubeGeometry(curve, 40, 2.0, 8, false);
      obj = new THREE.Group(); const m = new THREE.Mesh(geo, boxMat(0xe9ecef)); m.castShadow = true; obj.add(m);
      M = new THREE.Matrix4();
    } else if (v.kind === "box") {
      const [x0, x1, y0, y1, z0, z1] = v.b;
      obj = boxWithEdges(x1 - x0, y1 - y0, z1 - z0, v.color);
      M = M4(fk.mul(B, fk.T((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)));
    } else if (v.kind === "screws" || v.kind === "nuts") {
      obj = new THREE.Group(); const m = new THREE.Mesh(hwGeo(v), steel); m.castShadow = true; obj.add(m);
      if (v.frame === "fit") { addItem(v.id, obj, fitM(v.fit), null, root); continue; }
      M = M4(v.frame === "body" ? B : pf.frames[v.leg][v.frame]);
    } else if (v.kind === "loose" || v.kind === "looseCable") {
      // no place in the design: it only ever lies on the table (its step adds it with { stay: true })
      if (v.kind === "loose" && v.disc) {   // a roll: a flat disc
        const geo = new THREE.CylinderGeometry(v.disc[0] / 2, v.disc[0] / 2, v.disc[1], 32); geo.rotateX(Math.PI / 2);
        obj = new THREE.Group(); const m = new THREE.Mesh(geo, boxMat(v.color)); m.castShadow = true; obj.add(m);
      } else if (v.kind === "loose") obj = boxWithEdges(...v.box, v.color);
      else {
        obj = new THREE.Group();
        v.colors.forEach((c, j) => {
          const curve = new THREE.CatmullRomCurve3([[-40, 0, 0], [-15, 12, 0], [15, -12, 0], [40, 0, 0]].map(([x, y]) => new THREE.Vector3(x, y + j * 5, 0)));
          const m = new THREE.Mesh(new THREE.TubeGeometry(curve, 32, 1.6, 6), boxMat(c)); m.castShadow = true; obj.add(m);
        });
      }
      M = M4(fk.mul(B, fk.T(0, 0, 30)));
    } else if (v.kind === "stand") {
      obj = boxWithEdges(90, 90, LIFT + 60, 0xd8dbe0);
      M = new THREE.Matrix4().makeTranslation(0, 0, (LIFT + 60) / 2);
      addItem(v.id, obj, M, null, root); continue;
    } else if (v.kind === "fitServo" || v.kind === "fitGauge" || v.kind === "fitCollar") {
      // on the table: the servo lies with its shaft up, its idler horn on the table
      obj = new THREE.Group();
      if (v.kind === "fitServo") obj.add(new THREE.Mesh(GEO.servo, MAT.servo), new THREE.Mesh(GEO.label, MAT.red));
      else obj.add(new THREE.Mesh(v.kind === "fitGauge" ? gaugeGeo() : collarGeo(), MAT.black));
      obj.children.forEach((m) => { m.castShadow = true; m.receiveShadow = true; });
      addItem(v.id, obj, fitM(v.at, v.z), null, root); continue;
    } else if (v.kind === "bladeKit" || v.kind === "bladeKitInserts") {
      // one blade lying flat on the table, in the frame of the focus leg's tibia (its inserts go in along x)
      const tib = M4(pf.frames[FOCUS].tibia), inv = tib.clone().invert();
      const blade = new THREE.Mesh(GEO.blade, MAT[model.keycolor.blade] || MAT.black);
      blade.applyMatrix4(inv.clone().multiply(M4(PL[`${FOCUS}_blade`])));
      const box = new THREE.Box3().setFromObject(blade), sz = box.getSize(new THREE.Vector3());
      const thin = sz.x <= sz.y && sz.x <= sz.z ? "x" : sz.y <= sz.z ? "y" : "z";   // its thinnest side goes down
      const L = new THREE.Matrix4();
      if (thin === "x") L.makeRotationY(Math.PI / 2); else if (thin === "y") L.makeRotationX(Math.PI / 2);
      const fb = box.clone().applyMatrix4(L), c = fb.getCenter(new THREE.Vector3());
      L.premultiply(new THREE.Matrix4().makeTranslation(v.at[0] - c.x, v.at[1] - c.y, -fb.min.z));
      obj = new THREE.Group();
      if (v.kind === "bladeKit") { blade.castShadow = true; blade.receiveShadow = true; obj.add(blade); }
      else for (const y of [6, -6]) { const g = insertGeo(2.3, 5.7); g.rotateZ(Math.PI / 2); g.translate(26.75, y, 0); obj.add(new THREE.Mesh(g, brass)); }
      addItem(v.id, obj, L, null, root); continue;
    } else if (v.kind === "servoGrid" || v.kind === "servoSpare") {
      // the 18 servos in three rows of six; the 2 spares lie in front of them
      obj = new THREE.Group();
      const at = v.kind === "servoGrid" ? [0, 1, 2].flatMap((r) => [0, 1, 2, 3, 4, 5].map((c) => [(c - 2.5) * 62, (1 - r) * 48])) : [[-31, -106], [31, -106]];
      for (const [x, y] of at) {
        const s = new THREE.Group(); s.add(new THREE.Mesh(GEO.servo, MAT.servo), new THREE.Mesh(GEO.label, MAT.red));
        s.children[0].castShadow = true;
        s.position.set(x, y, 7.5); obj.add(s);
      }
      addItem(v.id, obj, new THREE.Matrix4(), null, root); continue;
    } else continue;
    addItem(v.id, obj, M, v.leg || legOf(v.id));
  }

  // ---------- where each part sits when it is built: its box in the robot's own frame (not lifted, not turned)
  root.updateMatrixWorld(true);
  const rootInv = root.matrixWorld.clone().invert();
  const BOX = new Map();
  for (const [id, it] of ITEMS) {
    it.obj.matrix.copy(it.M); it.obj.updateMatrixWorld(true);
    const b = new THREE.Box3().setFromObject(it.obj, true);
    if (!b.isEmpty()) BOX.set(id, b.applyMatrix4(rootInv));
  }
  robotG.matrixAutoUpdate = false;
  const inRobot = (it) => it.parent === robotG;

  // the belly step: two blocks under the front and the back of the body hold the robot up, the belly stays open
  const JIG = STEPS.find((s) => s.jig);
  if (JIG && BOX.has("base")) {
    const bb = BOX.get("base"), H = (JIG.view.lift || LIFT) + bb.min.z;
    const g = new THREE.Group();
    for (const y of [1, -1]) { const b = boxWithEdges(60, 22, H, 0xd8dbe0); b.position.set(0, y * (bb.max.y - 15), H / 2); g.add(b); }
    addItem("jig", g, new THREE.Matrix4(), null, root);
  }

  function offsetVec(id, off) {
    const it = ITEMS.get(id), leg = it.leg;
    if (off.body) return new THREE.Vector3(...off.body);
    if (off.radial !== undefined) { const d = HIPDIR[leg] || [0, 0]; return new THREE.Vector3(d[0] * off.radial, d[1] * off.radial, off.up || 0); }
    if (off.frame) {
      const F = off.frame === "self" ? it.M : M4(pf.frames[leg][off.frame]);
      return new THREE.Vector3(...off.v).transformDirection(F).multiplyScalar(Math.hypot(...off.v));
    }
    return new THREE.Vector3();
  }

  // ---------- how the robot sits in a step: turned about a pivot (a part laid on its side), then raised by `lift`
  const ID_Q = new THREE.Quaternion();
  function poseM(S, lift) {
    const p = S.pivot;
    return new THREE.Matrix4().makeTranslation(p.x, p.y, p.z + lift)
      .multiply(new THREE.Matrix4().makeRotationFromQuaternion(S.rot))
      .multiply(new THREE.Matrix4().makeTranslation(-p.x, -p.y, -p.z));
  }
  const worldBox = (id, M) => { const b = BOX.get(id); if (!b) return null; const it = ITEMS.get(id); return inRobot(it) ? b.clone().applyMatrix4(M) : b.clone(); };

  // ---------- a part's way in: it lies on the table beside the build, is lifted along an arc to its start of
  // the fit (`off`), then slides in. `s` runs from 1 (on the table) to 0 (in place). Offsets are in the part's
  // parent frame; `up` is the world's up in that frame.
  const SPLIT = 0.42;
  const _o = new THREE.Vector3(), _d = new THREE.Vector3();
  function pathAt(f, s, out = _o) {
    if (s <= SPLIT) return out.copy(f.vec).multiplyScalar(s / SPLIT);
    const r = 1 - (s - SPLIT) / (1 - SPLIT);   // 0 on the table, 1 at the start of the fit
    _d.copy(f.vec).sub(f.rest);
    const dv = _d.dot(f.up);
    const hc = r * r * (3 - 2 * r), vc = 1 - (1 - r) * (1 - r);
    out.copy(f.rest).addScaledVector(_d, hc).addScaledVector(f.up, dv * (vc - hc) + f.bump * Math.sin(Math.PI * r));
    return out;
  }

  // ---------- the plan: what is on screen at the end of each step, and how the step gets there
  const ids = (s) => (s.add || []).flatMap((a) => a.ids);
  const kept = (s) => (s.add || []).filter((a) => !a.off.stay).flatMap((a) => a.ids);   // a part that only lies beside the build goes with its step
  const builtAt = [];   // ids on the robot before step k
  { let acc = new Set(); for (let k = 1; k <= N; k++) { builtAt[k] = new Set(acc); const s = STEPS[k - 1];
      if (!s.view.only) kept(s).forEach((x) => acc.add(x));
      if (s.keep) [...(s.builtBefore || []), ...kept(s)].forEach((x) => acc.add(x)); }
    builtAt[N + 1] = acc; }
  const GOAL = builtAt[N + 1];   // the finished robot, drawn faintly as the goal (the picture on the box)

  const PLAN = [{ vis: new Set(["stand_box"]), ghost: new Set(), fresh: new Map(), T: 1, lift: LIFT, liftEnd: LIFT, goal: 0.2,
    rot: ID_Q.clone(), pivot: new THREE.Vector3(), robot: false }];
  for (let k = 1; k <= N; k++) {
    const s = STEPS[k - 1], prev = PLAN[k - 1];
    const vis = s.view.only ? new Set(s.view.only) : new Set([...builtAt[k], ...ids(s), ...(s.ghost || [])]);
    if (!s.view.only && !s.nobox && !s.jig) vis.add("stand_box");
    if (s.jig) vis.add("jig");
    const robotIds = [...vis].filter((id) => ITEMS.has(id) && BOX.has(id) && inRobot(ITEMS.get(id)));
    const Q = { vis, ghost: new Set(s.ghost || []), fresh: new Map(), goal: s.view.only ? 0 : 0.07, finale: !!s.finale, robot: robotIds.length > 0 };
    // the pose: a step with no robot parts on screen keeps the last one, so nothing turns while it fades
    if (!Q.robot) { Q.rot = prev.rot; Q.pivot = prev.pivot; Q.lift = prev.liftEnd; }
    else {
      Q.rot = s.view.side ? new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(HIPDIR[FOCUS][0], HIPDIR[FOCUS][1], 0), s.view.side * Math.PI / 180) : ID_Q.clone();
      Q.pivot = new THREE.Vector3();
      if (s.view.side) { const u3 = new THREE.Box3(); robotIds.forEach((id) => u3.union(BOX.get(id))); u3.getCenter(Q.pivot); }
      if (s.view.only) {   // a part built on its own rests on the table: lower it until its lowest point touches it
        const M0 = poseM(Q, 0); let minZ = Infinity;
        robotIds.forEach((id) => { minZ = Math.min(minZ, worldBox(id, M0).min.z); });
        Q.lift = -minZ;
      } else Q.lift = s.view.lift || LIFT;
    }
    Q.liftEnd = s.finale ? 0 : Q.lift;
    // a change of pose (laid on its side, onto the table, onto the blocks) crossfades: the old picture fades out,
    // the new one fades in, so nothing swings through the air
    Q.cutIn = Q.rot.angleTo(prev.rot) > 1e-3 || Q.pivot.distanceTo(prev.pivot) > 1 && Q.rot.angleTo(ID_Q) > 1e-3 || Math.abs(Q.lift - prev.liftEnd) > 30;
    const M = poseM(Q, Q.lift), R = new THREE.Matrix4().makeRotationFromQuaternion(Q.rot), Rinv = R.clone().invert();

    // what is already on the table at the landing point: the new parts are laid outside it
    const freshIds = new Set(ids(s));
    const here = [...vis].filter((id) => !freshIds.has(id) && BOX.has(id) && id !== "grid");
    const C0 = new THREE.Vector3(); let foot = 0;
    { const all = new THREE.Box3(); (here.length ? here : [...vis].filter((id) => BOX.has(id))).forEach((id) => all.union(worldBox(id, M)));
      if (!all.isEmpty()) all.getCenter(C0); C0.z = 0;
      if (!s.view.only) C0.set(0, 0, 0);
      for (const id of here) { const b = worldBox(id, M); for (const x of [b.min.x, b.max.x]) for (const y of [b.min.y, b.max.y]) foot = Math.max(foot, Math.hypot(x - C0.x, y - C0.y)); } }
    const camDir = s.view.az * Math.PI / 180;

    let T = 0;
    const groups = [];
    for (const a of s.add || []) {
      const gids = a.ids.filter((id) => ITEMS.has(id));
      if (!gids.length) continue;
      const kind = a.off.grow ? "grow" : a.off.fade ? "fade" : a.off.stay ? "stay" : "move";
      const f = { kind, delay: a.delay || 0, dur: a.dur || 1.0, vec: null, rest: null, up: null, bump: 0, leg: ITEMS.get(gids[0]).leg };
      if (kind === "move" || kind === "stay") {
        const robot = inRobot(ITEMS.get(gids[0]));
        const vec = offsetVec(gids[0], a.off);
        const wb = new THREE.Box3(); gids.forEach((id) => { const b = worldBox(id, M); if (b) wb.union(b); });
        const pc = wb.getCenter(new THREE.Vector3()), sz = wb.getSize(new THREE.Vector3());
        const vecW = robot ? vec.clone().applyMatrix4(R) : vec.clone();
        groups.push({ f, robot, vec, vecW, pc, sz, half: Math.hypot(sz.x, sz.y) / 2, rest: new THREE.Vector3() });
      }
      for (const id of gids) Q.fresh.set(id, f);
      T = Math.max(T, f.delay + f.dur);
    }
    if (s.view.only) {
      // built on the table: the parts lie in a row in front of the build, in the order they go in (like a manual)
      const tan = new THREE.Vector3(Math.cos(camDir), -Math.sin(camDir), 0), toCam = new THREE.Vector3(Math.sin(camDir), Math.cos(camDir), 0);
      let rAll = foot; for (const id of vis) { const b = worldBox(id, M); if (!b) continue; for (const x of [b.min.x, b.max.x]) for (const y of [b.min.y, b.max.y]) rAll = Math.max(rAll, (x - C0.x) * toCam.x + (y - C0.y) * toCam.y); }
      const GAP = 16, ROW = 400;
      const rows = [[]]; let wsum = 0;
      for (const gr of groups) {
        gr.w = Math.abs(gr.sz.x * tan.x) + Math.abs(gr.sz.y * tan.y); gr.dp = Math.abs(gr.sz.x * toCam.x) + Math.abs(gr.sz.y * toCam.y);
        if (wsum && wsum + gr.w > ROW) { rows.push([]); wsum = 0; }
        rows[rows.length - 1].push(gr); wsum += gr.w + GAP;
      }
      let out = rAll + 26;
      for (const row of rows) {
        const total = row.reduce((x, gr) => x + gr.w, 0) + GAP * (row.length - 1), depth = Math.max(...row.map((gr) => gr.dp));
        let at = -total / 2;
        for (const gr of row) { gr.rest.copy(C0).addScaledVector(toCam, out + depth / 2).addScaledVector(tan, at + gr.w / 2); at += gr.w + GAP; }
        out += depth + GAP;
      }
    } else {
      // on the box: each part lies on the floor in the first free spot out along its leg, or (a body part) beside
      // the body on the camera's side, clear of the legs, the box and the parts laid out before it
      const busy = here.map((id) => worldBox(id, M)).filter(Boolean).map((b) => [b.min.x, b.max.x, b.min.y, b.max.y]);
      const hits = (c, hx, hy) => busy.some((r) => c.x + hx + 8 > r[0] && c.x - hx - 8 < r[1] && c.y + hy + 8 > r[2] && c.y - hy - 8 < r[3]);
      const walk = (start, dir, hx, hy) => { const c = start.clone(); let n = 0; while (n < 120 && hits(c, hx, hy)) { c.addScaledVector(dir, 5); n++; } return { c, n }; };
      for (const gr of groups) {
        const hx = gr.sz.x / 2, hy = gr.sz.y / 2;
        const start = gr.pc.clone().add(gr.vecW); start.z = 0;
        let best;
        if (gr.f.leg) {
          const dir = new THREE.Vector3(HIPDIR[gr.f.leg][0], HIPDIR[gr.f.leg][1], 0).applyMatrix4(R);
          dir.z = 0; dir.normalize();
          best = walk(start.addScaledVector(dir, Math.max(hx, hy) + 10), dir, hx, hy);
        } else {
          for (const da of gr.f.kind === "stay" ? [35, -35, 70, -70, 0] : [35, -35, 70, -70, 0, 105, -105]) {   // a loose part stays in sight
            const ang = camDir + da * Math.PI / 180, dir = new THREE.Vector3(Math.sin(ang), Math.cos(ang), 0);
            const r = walk(new THREE.Vector3(C0.x, C0.y, 0).addScaledVector(dir, Math.max(hx, hy)), dir, hx, hy);
            if (!best || r.n < best.n - 2) best = r;
          }
        }
        gr.rest.copy(best.c);
        busy.push([best.c.x - hx, best.c.x + hx, best.c.y - hy, best.c.y + hy]);
      }
    }
    for (const gr of groups) {
      const f = gr.f, rest = gr.rest;
      rest.z = gr.sz.z / 2 + 0.5;
      f.span = [(f.kind === "stay" ? rest : gr.pc).clone(), rest.clone(), gr.half];   // for the camera: where it is and where it lies, in the world
      const restW = rest.clone().sub(gr.pc);
      f.vec = gr.vec;
      f.rest = gr.robot ? restW.applyMatrix4(Rinv) : restW;
      f.up = gr.robot ? new THREE.Vector3(0, 0, 1).applyMatrix4(Rinv) : new THREE.Vector3(0, 0, 1);
      f.bump = 22 + 0.12 * Math.hypot(f.rest.x - f.vec.x, f.rest.y - f.vec.y);
    }
    for (const [id, f] of Q.fresh) if (f.kind === "stay") vis.add(id);   // so a loose part fades out where it lay as the next step begins
    Q.T = T || 1; Q.M = M;
    PLAN.push(Q);
  }

  // ---------- the camera of each step: a target, an angle and a distance (aimed at the new parts of the focus leg)
  const VIEW = PLAN.map((p, k) => {
    const s = k ? STEPS[k - 1] : null;
    const v = s ? s.view : { az: 28, el: 18, d: 820, t: [0, 0, 85] };
    let T;
    if (v.t) T = new THREE.Vector3(...v.t);
    else {
      const cs = [];
      for (const [id, f] of p.fresh) { const b = worldBox(id, p.M || new THREE.Matrix4()); if (!b || f.kind === "stay" || (s.legs && f.leg && f.leg !== FOCUS)) continue; cs.push(b.getCenter(new THREE.Vector3())); }
      T = cs.length ? cs.reduce((a, c) => a.add(c), new THREE.Vector3()).multiplyScalar(1 / cs.length) : new THREE.Vector3(0, 0, 100);
    }
    const T0 = T.clone();
    // widen the view so the parts waiting on the table are in it too
    const span = new THREE.Box3(), seen = new Set();
    for (const [, f] of p.fresh) { if (!f.span || seen.has(f) || (s && s.legs && f.leg && f.leg !== FOCUS)) continue; seen.add(f);
      for (const c of f.span.slice(0, 2)) span.expandByPoint(c.clone().addScalar(f.span[2])).expandByPoint(c.clone().addScalar(-f.span[2])); }
    let d = v.d;
    if (!span.isEmpty()) {
      span.expandByPoint(T); T.lerp(span.getCenter(new THREE.Vector3()), v.t ? 0.35 : 0.7);
      let r = 0; for (const x of [span.min.x, span.max.x]) for (const y of [span.min.y, span.max.y]) for (const z of [span.min.z, span.max.z]) r = Math.max(r, T.distanceTo(new THREE.Vector3(x, y, z)));
      d = Math.max(d, r * 2.3);
    }
    // view.close: once every part is in, the camera comes in to v.d on the build (the table is empty again)
    return { T, az: v.az, el: v.el, d, close: v.close ? { T: T0, az: v.az, el: v.el, d: v.d } : null };
  });
  const endView = (k) => VIEW[k].close || VIEW[k];
  const angLerp = (a, b, t) => { let d = ((b - a + 540) % 360) - 180; return a + d * t; };
  const viewLerp = (p, q, t) => ({ T: p.T.clone().lerp(q.T, t), az: angLerp(p.az, q.az, t), el: lerp(p.el, q.el, t), d: p.d * Math.pow(q.d / p.d, t) });
  const coverEnd = { ...VIEW[0], d: VIEW[0].d * 0.9 };

  // ---------- materials: every part has its own copies, so parts never fade together
  function matOf(it, j, kind) {
    const c = it.mc || (it.mc = {}); const arr = c[kind] || (c[kind] = []);
    let m = arr[j];
    if (!m) {
      const b = it.base[j], src = kind === "hi" || kind === "hiT" ? hiOf(b) : b;
      m = src.clone();
      m.userData = { o0: b.opacity ?? 1, k: src.userData.k || 1 };
      if (kind !== "hi") m.transparent = true;
      if (kind === "goal" || kind === "ghost") m.depthWrite = false;
      arr[j] = m;
    }
    return m;
  }

  // dashed paths from where a part lies to its place (the focus leg only), one set per step
  const arrowMat = new THREE.LineDashedMaterial({ color: 0xf59f0b, dashSize: 6, gapSize: 5, transparent: true, depthTest: false });
  const arrowsR = new THREE.Group(), arrowsF = new THREE.Group(); robotG.add(arrowsR); root.add(arrowsF);
  const ARROWS = PLAN.map((p, k) => {
    const s = k ? STEPS[k - 1] : null, list = [], seen = new Set();
    for (const [id, f] of p.fresh) {
      const it = ITEMS.get(id), b = BOX.get(id);
      if (!f.vec || !b || seen.has(f) || f.kind === "stay" || (s && s.legs && it.leg !== FOCUS)) continue;
      seen.add(f);
      const c = b.getCenter(new THREE.Vector3()), pts = [];
      for (let i = 24; i >= 0; i--) pts.push(c.clone().add(pathAt(f, i / 24, new THREE.Vector3())));
      const ln = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), arrowMat);
      ln.computeLineDistances(); ln.userData = { f, robot: inRobot(it) }; list.push(ln);
    }
    return list;
  });

  // ---------- one picture for one scroll position
  let shownK = -1, dAz = 0, dEl = 0;
  const tmpM = new THREE.Matrix4();
  function frame() {
    const { k, t } = locate(u);
    const Q = PLAN[k], P = k ? PLAN[k - 1] : null;
    const a = k ? easeIO(t / A) : 1;                          // the move into this step
    const half = A / 2, cut = !!(k && Q.cutIn), phaseP = cut && t < half;
    const aIn = !k ? 1 : cut ? easeIO((t - half) / half) : a;
    const aOut = !k ? 0 : cut ? 1 - easeIO(t / half) : 1 - a;
    const tau = clamp((t - A) / (B_END - A)) * Q.T;            // the parts' own timeline
    const hi = t < B_END ? 1 : 1 - easeIO((t - B_END) / (1 - B_END));
    if (k !== shownK) {
      shownK = k; board(k);
      arrowsR.clear(); arrowsF.clear();
      for (const ln of ARROWS[k]) (ln.userData.robot ? arrowsR : arrowsF).add(ln);
    }

    // the robot's pose: on its box, on the blocks, or (a part built on its own) resting on the table
    const S = phaseP ? P : Q;
    let lift = phaseP ? P.liftEnd : cut || !k ? Q.lift : lerp(P.liftEnd, Q.lift, a);
    let boxSlide = 0, boxFade = 1;
    if (Q.finale) {
      const tn = clamp((t - A) / (1 - A));
      const r1 = easeIO(tn / 0.3), r2 = easeIO((tn - 0.3) / 0.3), r3 = easeIO((tn - 0.6) / 0.4);
      lift = lerp(lift, LIFT + 50, r1) - (LIFT + 50) * r3;     // lift it off its box, the box goes, set it down
      boxSlide = r2; boxFade = 1 - r2;
    }
    robotG.matrix.copy(poseM(S, lift));
    const g = !k ? Q.goal : phaseP ? P.goal * aOut : cut ? Q.goal * aIn : lerp(P.goal, Q.goal, a);

    for (const [id, it] of ITEMS) {
      let mode = "hidden", op = 1, s = 0, draw = 1, f = null;
      if (phaseP) {
        if (P.vis.has(id)) { mode = P.ghost.has(id) ? "ghost" : "T"; op = aOut; }
        else if (GOAL.has(id) && g > 0.004) mode = "goal";
      } else {
        f = k ? Q.fresh.get(id) : null;
        if (f) {
          const e = easeIO((tau - f.delay) / f.dur);
          const wasGhost = !cut && P.vis.has(id) && P.ghost.has(id);
          const was = !cut && P.vis.has(id) && !P.ghost.has(id);
          mode = "fresh";
          if (t < A) {
            if (wasGhost) {   // it was drawn see-through for orientation: that fades, then the real part lies ready
              if (t < half) { mode = "ghost"; op = 1 - easeIO(t / half); } else { s = f.vec ? 1 : 0; op = easeIO((t - half) / half); }
            } else if (f.kind === "move" || f.kind === "stay") { s = was ? a : 1; op = was ? 1 : aIn; }
            else if (f.kind === "grow") draw = 0;
            else op = FADE0 * aIn;                               // a part that fades in shows faintly first
          } else if (f.kind === "stay") s = 1;                   // it stays where it lies
          else if (f.kind === "move") s = 1 - e;
          else if (f.kind === "grow") draw = e;
          else op = lerp(FADE0, 1, e);
          if (mode === "fresh" && hi < 0.002 && op >= 1) mode = "solid";
        } else if (Q.ghost.has(id)) { mode = "ghost"; op = !cut && k && P.vis.has(id) ? 1 : aIn; }
        else if (Q.vis.has(id)) {
          if (!k || (!cut && P.vis.has(id) && !P.ghost.has(id))) mode = "solid";
          else { mode = "T"; op = aIn; }
        } else if (!cut && k && P.vis.has(id) && aOut > 0.004) { mode = P.ghost.has(id) ? "ghost" : "T"; op = aOut; }
        else if (GOAL.has(id) && g > 0.004) mode = "goal";
      }
      if (id === "stand_box" && Q.finale && boxSlide > 0) { mode = "T"; op = boxFade; }
      // a part that only lay beside the build in the last step fades out where it lay
      if (!f && P && mode !== "hidden" && mode !== "goal" && P.fresh.get(id) && P.fresh.get(id).kind === "stay") { f = P.fresh.get(id); s = 1; }
      if (mode === "T" && op >= 0.999) mode = "solid";

      const show = mode !== "hidden" && op > 0.004 && draw > 0.002;
      it.obj.visible = show;
      if (!show) continue;
      it.meshes.forEach((m, j) => {
        let mat;
        if (mode === "solid") mat = it.base[j];
        else if (mode === "fresh") { mat = matOf(it, j, op < 1 ? "hiT" : "hi"); if (op < 1) mat.opacity = op * mat.userData.o0; }
        else if (mode === "T") { mat = matOf(it, j, "T"); mat.opacity = op * mat.userData.o0; }
        else if (mode === "ghost") { mat = matOf(it, j, "ghost"); mat.opacity = 0.2 * op; }
        else { mat = matOf(it, j, "goal"); mat.opacity = g; }
        if (mode === "fresh" && mat.emissive) mat.emissiveIntensity = 0.25 * hi * mat.userData.k;
        m.material = mat;
        m.castShadow = it.cast[j] && (mode === "solid" || ((mode === "fresh" || mode === "T") && op > 0.5));
        if (m.geometry && m.geometry.index && it.id.includes("_cable_")) m.geometry.setDrawRange(0, f && f.kind === "grow" ? Math.floor(m.geometry.index.count * draw / 3) * 3 : Infinity);
      });
      if (s > 0 && f && f.vec) { const o = pathAt(f, s); it.obj.matrix.multiplyMatrices(tmpM.makeTranslation(o.x, o.y, o.z), it.M); }
      else it.obj.matrix.copy(it.M);
      if (id === "stand_box" && Q.finale && boxSlide > 0) it.obj.matrix.multiplyMatrices(tmpM.makeTranslation(0, -320 * boxSlide, 0), it.M);
    }

    // the paths: they come with the parts and go when each part has arrived
    arrowMat.opacity = 0.9 * (t < A ? aIn : hi);
    for (const ln of [...arrowsR.children, ...arrowsF.children]) {
      const f = ln.userData.f;
      ln.visible = t < A || easeIO((tau - f.delay) / f.dur) < 0.98;
    }
    arrowsR.visible = arrowsF.visible = k > 0 && !phaseP && arrowMat.opacity > 0.01;

    // the camera: from the last step's view to this one, plus what the person turned
    const v1 = VIEW[k];
    let v = k === 0 ? viewLerp(VIEW[0], coverEnd, easeIO(t)) : viewLerp(k === 1 ? coverEnd : endView(k - 1), v1, a);
    if (k && v1.close) v = viewLerp(v, v1.close, easeIO((t - 0.7) / (B_END - 0.7)));
    const az = (v.az + dAz) * Math.PI / 180, el = clamp(v.el + dEl, -35, 82) * Math.PI / 180;
    const aspect = camera.aspect || 1;
    const d = v.d * 1.15 * (aspect < 1.25 ? Math.min(1.7, 1.25 / aspect) : 1);
    const pos = v.T.clone().add(new THREE.Vector3(Math.sin(az) * Math.cos(el), Math.cos(az) * Math.cos(el), Math.sin(el)).multiplyScalar(d));
    pos.z = Math.max(pos.z, 18);
    // never through a wall of the room
    const lim = 1380, far = Math.max(Math.abs(pos.x), Math.abs(pos.y));
    if (far > lim) { const off = pos.clone().sub(v.T), n = Math.max(Math.abs(off.x) ? (Math.sign(off.x) * lim - v.T.x) / off.x : 1, 0);
      const m = Math.max(Math.abs(off.y) ? (Math.sign(off.y) * lim - v.T.y) / off.y : 1, 0); pos.copy(v.T).addScaledVector(off, Math.min(1, n, m)); }
    camera.position.copy(w(pos.x, pos.y, pos.z));
    camera.lookAt(w(v.T.x, v.T.y, v.T.z));
  }

  // ---------- drawing on demand: after a scroll, a drag or a resize, never on a clock
  let pending = false;
  function draw() { pending = false; frame(); renderer.render(scene, camera); }
  function request() {
    if (pending) return; pending = true;
    requestAnimationFrame(() => { if (pending) draw(); });
    setTimeout(() => { if (pending) draw(); }, 120);   // a hidden tab throttles frames: still show the right picture
  }
  // scrolling eases a turned view back to the guide's own view, in step with the scroll
  function scrolled(du) { const f = Math.exp(-Math.abs(du) * 2.4); dAz *= f; dEl *= f; if (Math.abs(dAz) < 0.05) dAz = 0; if (Math.abs(dEl) < 0.05) dEl = 0; }

  // drag sideways to turn, up and down to tilt (a touch screen scrolls with up and down)
  let drag = null;
  canvas.addEventListener("pointerdown", (e) => {
    if (e.button !== 0) return;
    drag = { id: e.pointerId, x: e.clientX, y: e.clientY };
    try { canvas.setPointerCapture(e.pointerId); } catch (err) { /* old browser */ }
    canvas.classList.add("drag");
  });
  canvas.addEventListener("pointermove", (e) => {
    if (!drag || e.pointerId !== drag.id) return;
    const dx = e.clientX - drag.x, dy = e.clientY - drag.y; drag.x = e.clientX; drag.y = e.clientY;
    dAz -= dx * 0.4;
    if (e.pointerType !== "touch") dEl = clamp(dEl + dy * 0.3, -40, 55);
    if (dx || dy) stage.classList.add("turned");
    request();
  });
  const endDrag = () => { drag = null; canvas.classList.remove("drag"); };
  canvas.addEventListener("pointerup", endDrag); canvas.addEventListener("pointercancel", endDrag); canvas.addEventListener("lostpointercapture", endDrag);

  function size() {
    const r = stage.getBoundingClientRect();
    renderer.setSize(r.width, r.height, false);
    camera.aspect = r.width / Math.max(1, r.height); camera.updateProjectionMatrix();
  }
  new ResizeObserver(() => { size(); draw(); }).observe(stage);
  size();

  // ---------- part pictures for the "You need" boxes, from the same meshes
  const thumbR = new THREE.WebGLRenderer({ antialias: true, alpha: true, preserveDrawingBuffer: true });
  thumbR.setPixelRatio(1); thumbR.setSize(160, 160, false); thumbR.outputColorSpace = THREE.SRGBColorSpace; thumbR.setClearColor(0x000000, 0);
  const tScene = new THREE.Scene();
  tScene.add(new THREE.HemisphereLight(0xffffff, 0x5a5f68, 0.95 * Math.PI));
  const tSun = new THREE.DirectionalLight(0xffffff, 0.8 * Math.PI); tSun.position.set(1, 2, 1.5); tScene.add(tSun);
  const tCam = new THREE.PerspectiveCamera(30, 1, 0.1, 5000);
  const cache = new Map();
  function iconObj(spec) {
    const g = new THREE.Group();
    if (spec.mesh) { const m = new THREE.Mesh(GEO[spec.mesh], MAT[model.keycolor[spec.mesh]] || MAT.black); g.add(m); if (spec.mesh === "servo") g.add(new THREE.Mesh(GEO.label, MAT.red)); g.rotation.x = -Math.PI / 2; }
    else if (spec.screw) { g.add(new THREE.Mesh(screwGeo(spec.screw.d, spec.screw.len, spec.screw.head), steel)); g.rotation.z = -0.75; }
    else if (spec.insert) { const r = spec.insert === 3 ? 2.3 : 1.9; const m = new THREE.Mesh(insertGeo(r, 5.7), brass); g.add(m); const hole = new THREE.Mesh(new THREE.CylinderGeometry(r * 0.55, r * 0.55, 5.8, 16), new THREE.MeshStandardMaterial({ color: 0x3a3220 })); g.add(hole); g.rotation.x = 0.5; }
    else if (spec.nut) { g.add(new THREE.Mesh(nutGeo(spec.nut), steel)); g.rotation.x = 0.7; }
    else if (spec.box) { const [a, b, c] = spec.box; const bx = boxWithEdges(a, c, b, spec.color); g.add(bx); }
    else if (spec.disc) { const [dd, h] = spec.disc; g.add(new THREE.Mesh(new THREE.CylinderGeometry(dd / 2, dd / 2, h, 32), boxMat(spec.color))); g.rotation.x = 0.5; }
    else if (spec.cable) {
      const c = new THREE.CatmullRomCurve3([[-40, 0, -10], [-15, 0, 12], [15, 0, -12], [40, 0, 10]].map((p) => new THREE.Vector3(...p)));
      g.add(new THREE.Mesh(new THREE.TubeGeometry(c, 48, 4, 10), boxMat(spec.cable)));
      for (const x of [-44, 44]) { const p = new THREE.Mesh(new THREE.BoxGeometry(9, 6, 7), boxMat(0xece9e0)); p.position.set(x, 0, x < 0 ? -10 : 10); g.add(p); }
    } else if (spec.shape === "gauge") {
      g.add(new THREE.Mesh(gaugeGeo(), MAT.black)); g.rotation.set(-Math.PI / 2, 0, 1.1);
    } else if (spec.shape === "collar") {
      g.add(new THREE.Mesh(collarGeo(), MAT.black)); g.rotation.set(-Math.PI / 2, 0, -0.6);
    }
    return g;
  }
  function thumb(key) {
    if (cache.has(key)) return cache.get(key);
    const spec = ICON[key]; if (!spec) return null;
    const obj = iconObj(spec); tScene.add(obj); obj.updateMatrixWorld(true);
    const sph = new THREE.Box3().setFromObject(obj).getBoundingSphere(new THREE.Sphere());
    const dir = new THREE.Vector3(0.62, 0.5, 0.78).normalize();
    tCam.position.copy(sph.center).addScaledVector(dir, sph.radius / Math.sin((15 * Math.PI) / 180) * 1.02);
    tCam.lookAt(sph.center); tCam.near = sph.radius * 0.1; tCam.far = sph.radius * 20; tCam.updateProjectionMatrix();
    thumbR.render(tScene, tCam);
    const url = thumbR.domElement.toDataURL("image/png");
    tScene.remove(obj);
    cache.set(key, url); return url;
  }

  // ---------- light and dark: the Hall by day, the Hall at night (stones, robot and board keep their colors)
  const DAY = { bg: 0xdfe2e7, walls: [0xe3e6ea, 0xc9cdd3, 0xc9cdd3, 0xd6d9de], wallL: [0x7d838c, 0.55], floor: 0x3a3c41, tileL: [0xd9dbe0, 0.35],
    sky: 0xffffff, ground: 0x4f545e, hemi: 0.85, sun: 0.75, fill: 0.25, box: 0xd8dbe0, edge: 0x15181d };
  const NIGHT = { bg: 0x15171b, walls: [0x2a2f37, 0x252a31, 0x252a31, 0x282d34], wallL: [0x3c434e, 1], floor: 0x1c1e22, tileL: [0xc8ccd3, 0.16],
    sky: 0xa9bbd8, ground: 0x15171b, hemi: 0.85 * 0.6, sun: 0.75 * 0.7, fill: 0.4, box: 0x3b414b, edge: 0xc8ccd3 };
  function paint(dark) {
    const P = dark ? NIGHT : DAY;
    scene.background.setHex(P.bg);
    hallM.wallM.forEach((m, i) => m.color.setHex(P.walls[i]));
    hallM.wallL.color.setHex(P.wallL[0]); hallM.wallL.opacity = P.wallL[1];
    hallM.floor.color.setHex(P.floor);
    hallM.tileL.color.setHex(P.tileL[0]); hallM.tileL.opacity = P.tileL[1];
    hemi.color.setHex(P.sky); hemi.groundColor.setHex(P.ground); hemi.intensity = P.hemi * Math.PI;
    sun.intensity = P.sun * Math.PI; fill.intensity = P.fill * Math.PI;
    // the stand box and the belly blocks: every copy of their materials (the fades use their own copies)
    for (const id of ["stand_box", "jig"]) {
      const it = ITEMS.get(id); if (!it) continue;
      it.base.forEach((b, j) => {
        const all = [b, HI.get(b), ...Object.values(it.mc || {}).map((a) => a[j])];
        for (const m of all) if (m && m.color) m.color.setHex(m.isLineBasicMaterial ? P.edge : P.box);
      });
    }
  }
  paint(!!window.theme && theme.dark());
  // a press crossfades the page inside a View Transition: draw the new picture right away, not in the next frame
  addEventListener("themechange", (e) => { paint(e.detail.dark); draw(); });

  document.body.classList.add("has3d");
  viewer = { request, scrolled, thumb };
  draw();
  fillThumbs();
}

function no3d(err) {
  console.warn("3D view off:", err && err.message ? err.message : err);
  const n = $("nogl"); if (n) n.hidden = false;
  document.body.classList.add("no3d");
}

startUI();
startViewer();
