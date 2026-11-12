// SPDX-License-Identifier: MIT AND CC-BY-SA-4.0
// guide-steps.js: the steps of the Takt build guide, as data. guide.js draws them.
//
// Where the content comes from (paths in the repository):
//   cad/takt.py            the CAD model: parts, frames, insert holes, deck posts, hatch and lid screws, electronics
//   hardware/DESIGN.md     "Assembly order per leg", the hardware list, the UNVERIFIED list, "Not designed yet"
//   hardware/BOM.md        the parts list (made from bom-data.js): which screws, inserts, servo version, battery, bus hub,
//                          CAD changes
//   cad/viewer/model.json  the FreeCAD export used here (model.json in this folder is the same file)
// Rule: no screw size, count or setting that is not in those files. Where they say nothing, the step says "see the
// parts list" or carries a "confirm" note. Common practice that is not in the sources carries a "rec" note.
//
// Step fields
//   title, legs (true: shown once, do it for all 6 legs), parts [{ icon, name, n, per, bom?, spare?, reuse? }],
//   tools [tool keys], say [plain sentences], notes [{ k: "rec" | "confirm" | "info", t }], src (where it comes from),
//   view { only?: ids shown alone, az, el, d, t?: [x, y, z] target in mm (floor frame), else the new parts,
//          lift?: mm the robot is held up in this step (default: on its build box),
//          side?: degrees a part built on its own is laid over, about the focus leg's hip direction (it lies on the table),
//          close?: once every part is in, the camera comes in to d on the build (for a step with many parts on the table) },
//   add  [{ ids, off, delay?, dur? }]: the parts this step puts in place, and how they go in. Each part waits on the
//        table beside the build; scrolling lifts it to the start of its fit (`off` from its place), then slides it in.
//   builtBefore: ids shown as already done in a step shown on its own; keep: its parts stay on the robot after it.
//   nobox: the build box is not drawn (the robot is lifted off it); jig: two blocks hold the robot up instead.
//   ghost: ids drawn see-through for orientation; sub: built on the table; finale: the robot steps off its box.
//   off  { body: [dx, dy, dz] } in the floor frame, { radial: out, up } along the leg's hip direction,
//        { frame: link | "self", v: [x, y, z] } along a frame of the same leg, { grow: true }, { fade: true },
//        { stay: true }: a part the design gives no place yet; it lies on the table beside the build and stays there.
// Parts list links (checked by checks/guide-covers-bom.mjs)
//   bom: the id of the line in bom-data.js this part comes from; its n is the pieces taken in this step, plus `spare`
//   pieces kept in reserve. reuse: the same pieces as another step (set up, tested or charged here, counted where they
//   go in). A part without `bom` comes in a box of another part (the servo box: horns, horn screws, cables).
//   Tools: TOOL_BOM names the line a tool key comes from; a tool is used, not used up.

export const LEGS = ["RF", "RM", "RR", "LR", "LM", "LF"];   // the order the legs are animated in (RF is in front)
export const FOCUS = "RF";                                  // the leg the camera looks at

const perLeg = (keys, off, gap = 0.32, dur = 1.0, start = 0) => LEGS.map((n, i) => ({ ids: keys.map((k) => `${n}_${k}`), off, delay: start + i * gap, dur }));

// Icons for the parts boxes. "mesh:" draws the real CAD mesh; the others are simple stand-ins.
export const ICON = {
  servo: { mesh: "servo" },
  coxa: { mesh: "coxa" }, femur: { mesh: "femur" }, knee: { mesh: "knee" }, blade: { mesh: "blade" }, foot: { mesh: "foot" },
  base: { mesh: "base" }, lid: { mesh: "lid" }, deck: { mesh: "deck" }, hatch: { mesh: "hatch" }, window: { mesh: "window" },
  camera: { mesh: "e_camera" }, led: { mesh: "e_led" },
  gauge: { shape: "gauge" }, collar: { shape: "collar" },
  st2x6: { screw: { d: 2, len: 6, head: "pan" } }, st2x8: { screw: { d: 2, len: 8, head: "pan" } },
  m3x6: { screw: { d: 3, len: 6, head: "cap" } }, m3x6cs: { screw: { d: 3, len: 6, head: "cs" } },
  m3x12cs: { screw: { d: 3, len: 12, head: "cs" } }, m3x20: { screw: { d: 3, len: 20, head: "cheese" } },
  m25x6: { screw: { d: 2.5, len: 6, head: "cap" } },
  ins3: { insert: 3 }, ins25: { insert: 2.5 }, nut3: { nut: 3 },
  battery: { box: [72, 35, 18], color: 0x59606b }, adapter: { box: [42, 33, 8], color: 0x2e7d4f },
  hub: { box: [50, 32, 10], color: 0xb08a4a }, pi: { box: [65, 30, 5], color: 0x2e7d4f }, imu: { box: [26, 23, 5], color: 0x2a4fa8 },
  oled: { box: [35, 34, 5], color: 0x1f2a44 }, reg: { box: [18, 20, 9], color: 0x2a4fa8 }, speaker: { disc: [28, 4.5], color: 0x2b2d31 },
  amp: { box: [19, 18, 3], color: 0x2a4fa8 }, mic: { box: [17, 13, 2], color: 0x2a4fa8 }, sd: { box: [15, 11, 1], color: 0x2b2d31 },
  header: { box: [8, 5, 6], color: 0xece9e0 }, xt60: { box: [16, 8, 16], color: 0xe2b100 }, fuse: { box: [19, 5, 12], color: 0x6fb3e8 },
  fuseholder: { box: [24, 10, 14], color: 0x2b2d31 }, wire: { cable: 0xc8282e }, cable: { cable: 0x3a3d44 }, jumper: { cable: 0x5fd640 },
  ribbon: { cable: 0xd8d2c4 },
  stripboard: { box: [100, 50, 1.6], color: 0xb08a4a }, acrylic: { mesh: "window" },
  printer: { box: [60, 60, 60], color: 0x5b6068 }, spool: { disc: [60, 24], color: 0x2b2d31 },
  charger: { box: [40, 30, 14], color: 0x2b2d31 }, bag: { box: [40, 26, 14], color: 0x9aa1ab }, checker: { box: [30, 16, 10], color: 0x6b7280 },
};

export const TOOLS = {
  screwdriver: "Small cross-head screwdriver (not in the parts list)",
  flat: "Small flat-blade screwdriver, for the slotted M3 x 20 screws (not in the parts list)",
  nut: "5.5 mm wrench or pliers, for the lock nut (not in the parts list)",
  hex: "Small screwdrivers, hex 2.0 and 2.5 mm",
  iron: "Soldering iron and solder",
  insertTip: "Insert tip set for the iron",
  computer: "A computer",
  tape: "Double-sided tape",
  shrink: "Heat-shrink tubing",
};
// the parts list line each tool comes from (the others are not in the parts list)
export const TOOL_BOM = { hex: "drivers", iron: "iron", insertTip: "tip_set", shrink: "shrink" };

const DRAFT = "Draft from the CAD model. Nobody has built this robot yet.";
export { DRAFT };

// Things the guide draws that have no CAD mesh of their own. Positions in the body frame of cad/takt.py (mm),
// unless `leg` and `frame` say otherwise. Boxes are the keep-out boxes of cad/takt.py's electronics list.
const COLUMNS = [[42, 22], [-42, 22], [42, -22], [-42, -22]];     // lid screws, M3 inserts (cad/takt.py COLUMNS)
const HATCH = [[14, 23], [-14, 23], [14, -23], [-14, -23]];        // hatch screws, M3 inserts from below (HATCH_SCREWS)
const POSTS = [[24, 23], [-24, 23], [24, -23], [-24, -23]];        // deck posts, M2.5 inserts (POSTS)
const PI_HOLES = [[29, -45.5], [-29, -45.5], [29, -22.5], [-29, -22.5]];   // the Pi's 4 bosses under the lid (PI_HOLES)

// Screws and nuts at the holes cad/takt.py gives. One item is one group that goes in together; `at` is in the frame
// named (a leg's s1, coxa, s2, femur, s3, tibia; "body"; "fit": the fit test servo on the table). The head sits at `at`;
// dir -1: the shaft points down the frame's z, +1: up. axis "y": along the frame's y instead.
const HORN = [45, 135, 225, 315].map((a) => [7 * Math.cos(a * Math.PI / 180), 7 * Math.sin(a * Math.PI / 180)]);   // 4 x M3 on the horn's 14 mm circle
const CASE_TOP = [[-28.8, 10.4], [-28.8, -10.4]], CASE_BACK = [[-32.7, 10.4], [-32.7, -10.4]];   // the case holes the brackets use (HOLE_TOP, HOLE_BOT)
const FOOT_HOLE = [108.4, -7.7];   // the foot's one M3 across, 3.5 mm past where blade and foot meet (tibia frame)
const ST2X6 = { d: 2, len: 6, head: "pan" }, ST2X8 = { d: 2, len: 8, head: "pan" }, M3X6 = { d: 3, len: 6, head: "cap" };
const M3X20 = { d: 3, len: 20, head: "cheese" }, M3X6CS = { d: 3, len: 6, head: "cs" }, M25X6 = { d: 2.5, len: 6, head: "cap" };
const grp = (id, frame, size, pts, z, dir, more = {}) => ({ id, kind: "screws", frame, ...size, at: pts.map(([x, y]) => [x, y, z]), dir, ...more });
// outer faces of the plates the screws go through, in the servo frame (S1: base cover and pedestal counterbore; S2, S3:
// the coxa cheeks and the knee plates, 4 mm each), in the coxa frame (its horn plates) and the femur frame (its plates)
const legHardware = (n) => [
  grp(`${n}_cs1_t`, "s1", ST2X6, CASE_TOP, 33, -1), grp(`${n}_cs1_b`, "s1", ST2X6, CASE_BACK, -4.75, 1),
  grp(`${n}_hc_t`, "coxa", M3X6, HORN, 49.5, -1), grp(`${n}_hc_b`, "coxa", M3X6, HORN, 4.3, 1),
  grp(`${n}_cs2_t`, "s2", ST2X8, CASE_TOP, 34.75, -1), grp(`${n}_cs2_b`, "s2", ST2X8, CASE_BACK, -5.75, 1),
  grp(`${n}_hf2_t`, "femur", M3X6, HORN, 23, -1), grp(`${n}_hf2_b`, "femur", M3X6, HORN, -22.2, 1),
  grp(`${n}_cs3_t`, "s3", ST2X8, CASE_TOP, 34.75, -1), grp(`${n}_cs3_b`, "s3", ST2X8, CASE_BACK, -5.75, 1),
  grp(`${n}_hf3_t`, "femur", M3X6, HORN.map(([x, y]) => [x + 66, y]), 23, -1), grp(`${n}_hf3_b`, "femur", M3X6, HORN.map(([x, y]) => [x + 66, y]), -22.2, 1),
  grp(`${n}_fscrew`, "tibia", M3X20, [FOOT_HOLE], 7.2, -1), { id: `${n}_fnut`, kind: "nuts", frame: "tibia", d: 3, at: [[...FOOT_HOLE, -9.6]] },
].map((v) => ({ ...v, leg: n }));

export const VIRTUAL = [
  ...COLUMNS.map(([x, y], i) => ({ id: `ins_lid_${i}`, kind: "insert", r: 2.3, z0: 55.3, z1: 61, x, y })),
  ...HATCH.map(([x, y], i) => ({ id: `ins_hatch_${i}`, kind: "insert", r: 2.3, z0: 0.2, z1: 5.9, x, y })),
  ...POSTS.map(([x, y], i) => ({ id: `ins_deck_${i}`, kind: "insert", r: 1.9, z0: 12.8, z1: 18.5, x, y })),
  // two M3 inserts in each blade (tibia frame, along x from 23.9 mm) and the two countersunk screws from the knee side
  ...LEGS.flatMap((n) => [6, -6].map((y, i) => ({ id: `${n}_bins_${i}`, kind: "insertX", leg: n, frame: "tibia", r: 2.3, x0: 23.9, x1: 29.6, y }))),
  ...LEGS.flatMap((n) => [6, -6].map((y, i) => ({ id: `${n}_bscrew_${i}`, kind: "screwX", leg: n, frame: "tibia", x0: 17, len: 12, y }))),
  ...LEGS.flatMap(legHardware),
  grp("lid_s", "body", M3X20, COLUMNS, 76.5, -1),            // in the lid's counterbores, into the column inserts
  grp("hatch_s", "body", M3X6CS, HATCH, -0.35, 1),           // countersunk, flush with the hatch's outer face
  grp("deck_s", "body", M25X6, POSTS, 20.5, -1),             // through the deck into the post inserts
  grp("pi_s", "body", M25X6, PI_HOLES, 66, 1),               // up through the Pi into the lid bosses
  { id: "cam_s", kind: "screws", frame: "body", ...ST2X6, axis: "y", dir: 1,                           // 4 x M2 into the camera standoffs
    at: [[10.5, 52.68, 20.75], [-10.5, 52.68, 20.75], [10.5, 52.68, 8.25], [-10.5, 52.68, 8.25]] },
  grp("fit_cs_t", "fit", ST2X6, CASE_TOP, 32.35, -1, { fit: [150, -10, 25] }),     // the fit collar's 4 screws
  grp("fit_cs_b", "fit", ST2X6, CASE_BACK, -3.35, 1, { fit: [150, -10, 25] }),
  // each leg is one chain: S3 to S2 to S1, then into the body
  ...LEGS.flatMap((n) => ["32", "21", "1h"].map((c) => ({ id: `${n}_cable_${c}`, kind: "cable", leg: n, link: c }))),
  { id: "e_battery", kind: "box", b: [-36, 36, -17.5, 17.5, 0, 18], color: 0x59606b, label: "Battery" },
  { id: "e_adapter", kind: "box", b: [-27.9, 27.9, -18.3, 18.3, 20.5, 32], color: 0x2e7d4f, label: "Bus adapter" },
  { id: "e_hub", kind: "box", b: [-21.6, 21.6, 22.1, 53.9, 63.5, 74.5], color: 0xb08a4a, label: "Bus hub" },
  { id: "e_pi", kind: "box", b: [-32.5, 32.5, -49, -19, 66, 71], color: 0x2e7d4f, label: "Pi Zero 2 W" },
  { id: "e_oled", kind: "box", b: [-17.7, 17.7, 57.2, 62.2, 27, 60.5], color: 0x1f2a44, label: "Display" },
  { id: "e_speaker", kind: "disc", r: 14, z0: 72, z1: 76.5, x: 0, y: 0, color: 0x2b2d31, label: "Speaker" },
  { id: "e_amp", kind: "box", b: [18.3, 37.7, -8.9, 8.9, 73, 76], color: 0x2a4fa8, label: "Amplifier" },
  { id: "e_mic", kind: "box", b: [-36.7, -20, -6.35, 6.35, 74.2, 76], color: 0x2a4fa8, label: "Microphone" },
  { id: "e_reg5", kind: "box", b: [-8.9, 8.9, -64.2, -60, 14, 31.8], color: 0x2a4fa8, label: "5 V regulator" },
  // parts the design gives no place yet: they lie on the table beside the build in their step (add with { stay: true }).
  // The motion sensor's size is from the parts list; the others are stand-ins, their sizes are not in the design.
  { id: "loose_imu", kind: "loose", box: [25.6, 22.7, 4.6], color: 0x2a4fa8 },
  { id: "loose_sd", kind: "loose", box: [15, 11, 1], color: 0x2b2d31 },
  { id: "loose_xt60", kind: "loose", box: [16, 16, 8], color: 0xe2b100 },
  { id: "loose_fuse", kind: "loose", box: [19, 12, 5], color: 0x6fb3e8 },
  { id: "loose_fuseholder", kind: "loose", box: [24, 14, 10], color: 0x2b2d31 },
  { id: "loose_wire", kind: "looseCable", colors: [0xc8282e, 0x2b2d31] },
  { id: "loose_jumpers", kind: "looseCable", colors: [0x5fd640, 0xe2b100, 0x2a4fa8] },
  { id: "loose_ribbon", kind: "looseCable", colors: [0xd8d2c4] },
  { id: "loose_cable", kind: "looseCable", colors: [0x9aa1ab] },
  // simple stand-ins for things that never go into the robot (the sizes of the parts box icons, not real sizes)
  { id: "loose_printer", kind: "loose", box: [60, 60, 60], color: 0x5b6068 },
  { id: "loose_petg", kind: "loose", disc: [60, 24], color: 0x2b2d31 },
  { id: "loose_tpu", kind: "loose", disc: [60, 24], color: 0x8a929c },
  { id: "loose_charger", kind: "loose", box: [40, 30, 14], color: 0x2b2d31 },
  { id: "loose_bag", kind: "loose", box: [40, 26, 14], color: 0x9aa1ab },
  { id: "loose_checker", kind: "loose", box: [30, 16, 10], color: 0x6b7280 },
  grp("loose_adapter_s", "body", ST2X6, [[-6, 0], [-2, 0], [2, 0], [6, 0]], 30, -1),   // the adapter's 4 screws: its holes are not drawn yet
  { id: "stand_box", kind: "stand" },
  { id: "grid", kind: "servoGrid" },
  { id: "grid_spare", kind: "servoSpare" },
  // step 2's fit test: one servo with both horns, the U gauge (fit_gauge in cad/takt.py) and the tail collar (fit_collar),
  // drawn from cad/takt.py's numbers, on the table beside the test leg (floor frame: x, y, turn about z in degrees)
  { id: "fit_servo", kind: "fitServo", at: [150, -10, 25] },
  { id: "fit_gauge", kind: "fitGauge", at: [150, -10, 25] },
  { id: "fit_collar", kind: "fitCollar", at: [150, -10, 25] },
  // step 1: the same two pieces, freshly printed, lying flat on the table (z: their lowest point on the table)
  { id: "pr_gauge", kind: "fitGauge", at: [-40, 0, 0], z: 6.2 },
  { id: "pr_collar", kind: "fitCollar", at: [40, 0, 0], z: 3.35 },
  // step 5: one of the six blades, flat on the table beside the body, and its 2 inserts (floor frame: x, y)
  { id: "kit_blade", kind: "bladeKit", at: [20, 150] },
  { id: "kit_bins", kind: "bladeKitInserts", at: [20, 150] },
];

export const COVER = {
  title: "Build Takt",
  lead: "An easy step-by-step tutorial in 3D for a six-legged robot with 18 servos.",
  facts: [["18", "Feetech STS3215 servos, 3 in each leg"], ["34", "printed parts: 4 for the body, 24 for the legs, 6 soft feet"], ["2,051 g", "in the CAD model"], ["22", "steps"]],
};

const LOOSE = "It has no place in the model yet, so it lies beside the build here. Its place will be fixed in the first real build.";

export const STEPS = [
  {
    title: "Print the parts",
    parts: [{ icon: "printer", name: "3D printer (the parts list holds a Bambu Lab A1 mini as a placeholder)", n: 1, bom: "printer" },
      { icon: "spool", name: "PETG filament, 1 kg roll", n: 1, per: "every hard part", bom: "petg" },
      { icon: "spool", name: "TPU 95A filament, 750 g roll", n: 1, per: "the 6 soft feet", bom: "tpu" }],
    tools: ["computer"],
    say: ["Every size in the model is untested, so print in two rounds. First the fit test: the fit gauge, the fit collar and one whole leg. Print the full set only once the fit test in the next step works.",
      "With your own printer: the hard parts in PETG, the feet in TPU 95A. The files already lie the way they print. The body bottom, the hip parts, the thighs and a little of the knee parts need supports.",
      "With a print service instead: order 1 is the fit test (7 parts), order 2 the full set (34 parts). Ask the service not to turn the parts."],
    notes: [{ k: "info", t: "The printer needs a bed of about 160 x 160 mm, about 80 mm of height, a nozzle up to 255 °C, a bed up to 75 °C, a direct extruder and 1.75 mm filament. The PETG roll wants 230 to 255 °C at the nozzle and a 60 to 75 °C bed; the TPU roll 200 to 230 °C and a bed from 60 °C." },
      { k: "info", t: "On a 160 mm bed, turn the body bottom and the lid by 15°. From 180 mm they fit as they are." },
      { k: "rec", t: "Print PETG on a textured PEI plate, or use glue stick on a smooth one. TPU takes up water from the air: keep the roll dry." },
      { k: "confirm", t: "Weights are model numbers, not weighed: about 83 g PETG and 2 g TPU for the fit test, about 650 g PETG and 11 g TPU for the full set, before supports. The print service prices are estimates, not quotes." }],
    src: "hardware/BOM.md (printed parts, the two print orders, what the printer needs, the filament rolls), hardware/DESIGN.md (printed parts and supports)",
    view: { only: ["pr_gauge", "pr_collar"], az: 20, el: 40, d: 330 },
    add: [{ ids: ["loose_printer"], off: { stay: true } }, { ids: ["loose_petg"], off: { stay: true } }, { ids: ["loose_tpu"], off: { stay: true } },
      { ids: ["pr_gauge"], off: { fade: true }, dur: 0.9 }, { ids: ["pr_collar"], off: { fade: true }, delay: 0.6, dur: 0.9 }],
  },
  {
    title: "Test one leg first",
    parts: [{ icon: "gauge", name: "Fit gauge", n: 1, bom: "p1_gauge" }, { icon: "collar", name: "Fit collar", n: 1, bom: "p1_collar" },
      { icon: "servo", name: "Servo", n: 3, per: "3 of the 18, they go back into the build", bom: "servo", reuse: true },
      { icon: "st2x6", name: "Self-tapping screw 2.0 x 6", n: 4, per: "for the collar", bom: "st2x6" },
      { icon: "coxa", name: "Hip bracket", n: 1, bom: "p1_coxa" }, { icon: "femur", name: "Thigh link", n: 1, bom: "p1_femur" },
      { icon: "knee", name: "Knee frame", n: 1, bom: "p1_knee" }, { icon: "blade", name: "Blade", n: 1, bom: "p1_blade" },
      { icon: "foot", name: "Soft foot", n: 1, bom: "p1_foot" },
      { icon: "ins3", name: "Insert M3", n: 2, per: "for the blade", bom: "ins_m3" }, { icon: "m3x12cs", name: "Countersunk screw M3 x 12", n: 2, bom: "m3x12cs" },
      { icon: "st2x8", name: "Self-tapping screw 2.0 x 8", n: 8, per: "4 per servo, thigh and knee", bom: "st2x8" },
      { icon: "m3x6", name: "Screw M3 x 6", n: 24, per: "8 per joint, from the servo boxes" },
      { icon: "m3x20", name: "Screw M3 x 20", n: 1, per: "for the foot", bom: "m3x20" }, { icon: "nut3", name: "Lock nut M3", n: 1, bom: "nut_m3" }],
    tools: ["screwdriver", "hex", "flat", "nut", "iron", "insertTip"],
    say: ["One servo with both horns must slide snugly into the fit gauge. The 4 screws of the fit collar must find the servo's case holes.",
      "Then build the test leg the way steps 7 to 13 show it. Melt its blade's 2 inserts in first, as in step 5."],
    notes: [{ k: "confirm", t: "If something does not fit, the CAD model is changed and printed again. Only then print the full set." },
      { k: "confirm", t: "The design gives no length for the collar screws. The short 2.0 x 6 screws from the parts list are used here." },
      { k: "info", t: "If the test leg fits, it can be one of the six legs. The parts list still buys a full set of six." }],
    src: "hardware/DESIGN.md (the unverified list, assembly order per leg), cad/takt.py (fit gauge, fit collar, horn and case holes), hardware/BOM.md (fit test first)",
    view: { only: ["fit_servo", "fit_gauge", "fit_collar", "fit_cs_t", "fit_cs_b", "RF_s1", "RF_label_s1", "RF_coxa", "RF_hc_t", "RF_hc_b", "RF_s2", "RF_label_s2",
      "RF_cs2_t", "RF_cs2_b", "RF_femur", "RF_hf2_t", "RF_hf2_b", "RF_s3", "RF_label_s3", "RF_cs3_t", "RF_cs3_b", "RF_knee", "RF_blade", "RF_bins_0", "RF_bins_1",
      "RF_bscrew_0", "RF_bscrew_1", "RF_hf3_t", "RF_hf3_b", "RF_foot", "RF_fscrew", "RF_fnut"], az: 125, el: 34, d: 430, side: 90, close: true },
    add: [
      { ids: ["fit_servo"], off: { body: [0, 0, 0] }, delay: 0, dur: 0.8 },
      { ids: ["fit_gauge"], off: { frame: "self", v: [70, 0, 0] }, delay: 0.7 },
      { ids: ["fit_collar"], off: { frame: "self", v: [-70, 0, 0] }, delay: 1.4 },
      { ids: ["fit_cs_t"], off: { frame: "self", v: [0, 0, 30] }, delay: 2.0, dur: 0.7 },
      { ids: ["fit_cs_b"], off: { frame: "self", v: [0, 0, -4] }, delay: 2.2, dur: 0.7 },
      { ids: ["RF_s1", "RF_label_s1"], off: { radial: 0, up: 70 }, delay: 2.7 },
      { ids: ["RF_coxa"], off: { radial: 70, up: 0 }, delay: 3.2 },
      { ids: ["RF_hc_t"], off: { frame: "self", v: [0, 0, 25] }, delay: 3.8, dur: 0.6 }, { ids: ["RF_hc_b"], off: { frame: "self", v: [0, 0, -20] }, delay: 3.9, dur: 0.6 },
      { ids: ["RF_s2", "RF_label_s2"], off: { body: [0, 0, 70] }, delay: 4.2 },
      { ids: ["RF_cs2_t"], off: { frame: "self", v: [0, 0, 22] }, delay: 4.8, dur: 0.6 }, { ids: ["RF_cs2_b"], off: { frame: "self", v: [0, 0, -22] }, delay: 4.9, dur: 0.6 },
      { ids: ["RF_femur"], off: { frame: "femur", v: [70, 0, 0] }, delay: 5.2 },
      { ids: ["RF_hf2_t"], off: { frame: "self", v: [0, 0, 22] }, delay: 5.8, dur: 0.6 }, { ids: ["RF_hf2_b"], off: { frame: "self", v: [0, 0, -22] }, delay: 5.9, dur: 0.6 },
      { ids: ["RF_s3", "RF_label_s3", "RF_cs3_t", "RF_cs3_b", "RF_knee", "RF_blade", "RF_bins_0", "RF_bins_1", "RF_bscrew_0", "RF_bscrew_1"], off: { frame: "femur", v: [70, 0, 0] }, delay: 6.2 },
      { ids: ["RF_hf3_t"], off: { frame: "self", v: [0, 0, 22] }, delay: 6.8, dur: 0.6 }, { ids: ["RF_hf3_b"], off: { frame: "self", v: [0, 0, -22] }, delay: 6.9, dur: 0.6 },
      { ids: ["RF_foot"], off: { frame: "tibia", v: [35, -12, 0] }, delay: 7.2 },
      { ids: ["RF_fscrew"], off: { frame: "self", v: [0, 0, 25] }, delay: 7.8, dur: 0.6 }, { ids: ["RF_fnut"], off: { frame: "self", v: [0, 0, -20] }, delay: 7.9, dur: 0.6 },
    ],
  },
  {
    title: "Charge the battery and keep it safe",
    parts: [{ icon: "battery", name: "Battery 3S 1000 mAh, XT60", n: 1, per: "it goes in at step 20", bom: "battery", reuse: true },
      { icon: "charger", name: "Charger M4 Pocket", n: 1, bom: "charger" }, { icon: "bag", name: "LiPo fire-safe bag", n: 1, bom: "bag" },
      { icon: "checker", name: "Battery checker", n: 1, bom: "checker" }],
    tools: [],
    say: ["Charge the 3S battery with the charger, with its balance lead plugged in, so all three cells are charged evenly.",
      "Charge it and store it inside the fire-safe bag. Before every run, plug the battery checker onto the balance lead: it beeps before the battery runs too low."],
    notes: [{ k: "confirm", t: "Check on the charger's product page which power supply it needs. The parts list has none." },
      { k: "info", t: "The battery comes out through the belly hatch to charge. There is no charge port in the design." },
      { k: "rec", t: "Stay close while it charges, and never charge a swollen or damaged battery." }],
    src: "hardware/BOM.md (charger, fire-safe bag, battery checker, the 3S battery), hardware/DESIGN.md (not designed yet: charging)",
    view: { only: ["e_battery"], az: 30, el: 38, d: 300 },
    add: [{ ids: ["e_battery"], off: { fade: true }, dur: 0.8 },
      { ids: ["loose_charger"], off: { stay: true } }, { ids: ["loose_bag"], off: { stay: true } }, { ids: ["loose_checker"], off: { stay: true } }],
  },
  {
    title: "Get the servos ready",
    parts: [{ icon: "servo", name: "Servo, 12 V version (ST-3215-C018)", n: 18, per: "set up now, they go in at steps 6, 8 and 11", bom: "servo", reuse: true },
      { icon: "servo", name: "Spare servo", n: 2, per: "kept in reserve, in case one fails", bom: "servo_spare" },
      { icon: "adapter", name: "Bus servo adapter", n: 1, per: "it goes in at step 16", bom: "adapter", reuse: true },
      { icon: "battery", name: "Battery, 3S 1000 mAh, charged", n: 1, bom: "battery", reuse: true },
      { icon: "m3x6", name: "Screw M3 x 6, reserve pack", n: 30, per: "only if horn screws are missing or do not fit", bom: "m3x6" }],
    tools: ["computer"],
    say: ["Connect one servo at a time to the bus adapter. Give it its own ID number and turn it to its middle position.",
      "Put both metal horns on each servo. They come in the servo box, with 8 small and 2 bigger screws and a 15 cm cable."],
    notes: [{ k: "rec", t: "Setting an ID and centring every servo before it goes in is common practice. The design notes do not cover it." },
      { k: "confirm", t: "Which ID goes to which joint, and how the horn is fixed to the shaft: to be confirmed in the first real build." },
      { k: "confirm", t: "How the battery powers the adapter for this set-up, and the USB cable to the computer, are not in the parts list yet." },
      { k: "confirm", t: "One shop says the small screws are M3 x 6, the size the horns need (8 per joint). That is not sure. If they do not fit, the parts list says to buy 4 more reserve packs." }],
    src: "hardware/DESIGN.md (the horns go onto the servo first), hardware/BOM.md (servo version, spare servos, what is in the servo box)",
    view: { only: ["grid", "grid_spare"], az: 15, el: 48, d: 560, t: [0, -20, 10] },
    add: [{ ids: ["grid"], off: { fade: true }, dur: 0.6 }, { ids: ["grid_spare"], off: { fade: true }, delay: 0.5, dur: 0.6 },
      { ids: ["e_adapter"], off: { stay: true }, delay: 0.3 }, { ids: ["e_battery"], off: { stay: true }, delay: 0.3 }],
  },
  {
    title: "Melt in the threaded inserts",
    parts: [{ icon: "base", name: "Body base", n: 1, bom: "p2_base" }, { icon: "ins3", name: "Insert M3", n: 8, per: "4 for the lid, 4 for the hatch", bom: "ins_m3" },
      { icon: "ins25", name: "Insert M2.5", n: 4, per: "for the deck posts", bom: "ins_m25" }, { icon: "ins3", name: "Insert M3, in the blades", n: 12, per: "2 per blade", bom: "ins_m3" }],
    tools: ["iron", "insertTip"],
    say: ["Heat each insert with the soldering iron and press it straight into its hole. Let it cool.",
      "The body gets 12: four on top for the lid, four inside for the deck, four from below for the belly hatch. Each blade gets two."],
    notes: [{ k: "rec", t: "Press gently and keep the insert straight. Let the heat do the work." },
      { k: "confirm", t: "The hole for the M2.5 inserts (3.6 mm) is not checked. Measure an insert before you print." }],
    src: "cad/takt.py (lid columns, deck posts, hatch screws, blade inserts), hardware/DESIGN.md hardware list, hardware/BOM.md",
    view: { only: ["base", "ins_lid_0", "ins_lid_1", "ins_lid_2", "ins_lid_3", "ins_hatch_0", "ins_hatch_1", "ins_hatch_2", "ins_hatch_3",
      "ins_deck_0", "ins_deck_1", "ins_deck_2", "ins_deck_3", "kit_blade", "kit_bins"], az: 25, el: 55, d: 360 },
    add: [
      { ids: ["ins_lid_0", "ins_lid_1", "ins_lid_2", "ins_lid_3"], off: { body: [0, 0, 60] } },
      { ids: ["ins_deck_0", "ins_deck_1", "ins_deck_2", "ins_deck_3"], off: { body: [0, 0, 80] }, delay: 0.5 },
      { ids: ["ins_hatch_0", "ins_hatch_1", "ins_hatch_2", "ins_hatch_3"], off: { fade: true }, delay: 1.0 },
      { ids: ["kit_bins"], off: { frame: "self", v: [45, 0, 0] }, delay: 1.4 },   // one blade shown, its 2 inserts go in
    ],
    builtBefore: ["base"],
    keep: true,
  },
  {
    title: "Hip servos into the body",
    legs: true,
    parts: [{ icon: "servo", name: "Servo (hip)", n: 6, per: "1 per leg", bom: "servo" }, { icon: "st2x6", name: "Self-tapping screw 2.0 x 6", n: 24, per: "4 per servo", bom: "st2x6" }],
    tools: ["screwdriver"],
    say: ["Slide the hip servo into its pocket in the body, from the outside. The horn points up.",
      "Screw it in with 4 screws through the body into the servo's case: 2 from above, 2 from below."],
    notes: [{ k: "rec", t: "Rest the body on a small box while you build. The legs can then hang free." }],
    src: "cad/takt.py (base: servo pockets and case screw holes), hardware/BOM.md (2.0 x 6 for the hip servos)",
    view: { az: 35, el: 30, d: 520, t: [40, 40, 100] },
    add: [...perLeg(["s1", "label_s1"], { radial: 90, up: 0 }),
      ...perLeg(["cs1_t"], { frame: "self", v: [0, 0, 25] }, 0.32, 0.7, 0.8), ...perLeg(["cs1_b"], { frame: "self", v: [0, 0, -5] }, 0.32, 0.7, 1.0)],
  },
  {
    title: "Hip bracket",
    legs: true,
    parts: [{ icon: "coxa", name: "Hip bracket", n: 6, per: "1 per leg", bom: "p2_coxa" }, { icon: "m3x6", name: "Screw M3 x 6", n: 48, per: "8 per bracket, from the servo boxes" }],
    tools: ["hex"],
    say: ["Slide the hip bracket over the servo's two horns, from the outside.",
      "Fix it with 4 screws into each horn, 8 in all."],
    notes: [{ k: "confirm", t: "The horn screws may come in the servo box. That is not sure: see the parts list." }],
    src: "hardware/DESIGN.md (assembly order per leg; each horn has 4 M3 holes), cad/takt.py (horn hole circle), hardware/BOM.md",
    view: { az: 38, el: 26, d: 430 },
    add: [...perLeg(["coxa"], { radial: 80, up: 0 }),
      ...perLeg(["hc_t"], { frame: "self", v: [0, 0, 25] }, 0.32, 0.7, 0.8), ...perLeg(["hc_b"], { frame: "self", v: [0, 0, -20] }, 0.32, 0.7, 1.0)],
  },
  {
    title: "Thigh servo",
    legs: true,
    parts: [{ icon: "servo", name: "Servo (thigh)", n: 6, per: "1 per leg", bom: "servo" }, { icon: "st2x8", name: "Self-tapping screw 2.0 x 8", n: 24, per: "4 per servo", bom: "st2x8" }],
    tools: ["screwdriver"],
    say: ["Set the thigh servo into the cradle of the hip bracket, from above.", "Screw it in with 4 screws through the cradle's two cheeks."],
    src: "hardware/DESIGN.md (assembly order per leg), cad/takt.py (coxa cradle screw holes), hardware/BOM.md (2.0 x 8)",
    view: { az: 40, el: 30, d: 430 },
    add: [...perLeg(["s2", "label_s2"], { body: [0, 0, 80] }),
      ...perLeg(["cs2_t"], { frame: "self", v: [0, 0, 22] }, 0.32, 0.7, 0.8), ...perLeg(["cs2_b"], { frame: "self", v: [0, 0, -22] }, 0.32, 0.7, 1.0)],
  },
  {
    title: "Thigh link",
    legs: true,
    parts: [{ icon: "femur", name: "Thigh link", n: 6, per: "1 per leg", bom: "p2_femur" }, { icon: "m3x6", name: "Screw M3 x 6", n: 48, per: "8 per link, from the servo boxes" }],
    tools: ["hex"],
    say: ["Slide the thigh link along its length onto the thigh servo's two horns.", "Fix it with 4 screws into each horn."],
    src: "hardware/DESIGN.md (assembly order per leg), cad/takt.py (femur horn holes)",
    view: { az: 50, el: 28, d: 460 },
    add: [...perLeg(["femur"], { frame: "femur", v: [80, 0, 0] }),
      ...perLeg(["hf2_t"], { frame: "self", v: [0, 0, 22] }, 0.32, 0.7, 0.8), ...perLeg(["hf2_b"], { frame: "self", v: [0, 0, -22] }, 0.32, 0.7, 1.0)],
  },
  {
    title: "Blade into the knee frame",
    legs: true,
    sub: true,
    parts: [{ icon: "knee", name: "Knee frame", n: 6, per: "1 per leg", bom: "p2_knee" }, { icon: "blade", name: "Blade, with its 2 inserts", n: 6, per: "1 per leg", bom: "p2_blade" },
      { icon: "m3x12cs", name: "Countersunk screw M3 x 12", n: 12, per: "2 per blade", bom: "m3x12cs" }],
    tools: ["hex"],
    say: ["Build this on the table, next to the robot. Push the top of the blade into the block of the knee frame.",
      "Fix it with 2 countersunk screws into the blade's inserts. Do this before the knee servo goes in."],
    src: "hardware/DESIGN.md (bolt the blade to the knee frame first; M3 x 12), cad/takt.py (knee block, blade inserts)",
    view: { only: ["RF_knee", "RF_blade", "RF_bins_0", "RF_bins_1", "RF_bscrew_0", "RF_bscrew_1"], az: 75, el: 34, d: 440, side: 90 },
    add: [
      { ids: ["RF_blade", "RF_bins_0", "RF_bins_1"], off: { frame: "tibia", v: [55, 0, 0] } },
      { ids: ["RF_bscrew_0", "RF_bscrew_1"], off: { frame: "tibia", v: [-40, 0, 0] }, delay: 0.9 },
    ],
    builtBefore: ["RF_knee"],
  },
  {
    title: "Knee servo into the knee frame",
    legs: true,
    sub: true,
    parts: [{ icon: "servo", name: "Servo (knee)", n: 6, per: "1 per leg", bom: "servo" }, { icon: "st2x8", name: "Self-tapping screw 2.0 x 8", n: 24, per: "4 per servo", bom: "st2x8" }],
    tools: ["screwdriver"],
    say: ["Put the knee servo into the knee frame from the open inner side. The servo's label side faces the window.",
      "Screw it in with 4 screws through the frame's two plates."],
    src: "hardware/DESIGN.md (put the knee servo into the knee frame from the open inner side), cad/takt.py (knee window and plate holes), hardware/BOM.md (2.0 x 8)",
    view: { only: ["RF_knee", "RF_blade", "RF_bins_0", "RF_bins_1", "RF_bscrew_0", "RF_bscrew_1", "RF_s3", "RF_label_s3", "RF_cs3_t", "RF_cs3_b"], az: 95, el: 34, d: 440, side: 90 },
    add: [{ ids: ["RF_s3", "RF_label_s3"], off: { frame: "s3", v: [0, 0, 60] } },
      { ids: ["RF_cs3_t"], off: { frame: "self", v: [0, 0, 22] }, delay: 0.9, dur: 0.7 }, { ids: ["RF_cs3_b"], off: { frame: "self", v: [0, 0, -22] }, delay: 1.2, dur: 0.7 }],
    builtBefore: ["RF_knee", "RF_blade", "RF_bins_0", "RF_bins_1", "RF_bscrew_0", "RF_bscrew_1"],
  },
  {
    title: "Knee onto the thigh",
    legs: true,
    parts: [{ icon: "knee", name: "Knee with blade and servo (from steps 10 and 11)", n: 6, per: "1 per leg" }, { icon: "m3x6", name: "Screw M3 x 6", n: 48, per: "8 per knee, from the servo boxes" }],
    tools: ["hex"],
    say: ["Slide the thigh link onto the knee servo's two horns.", "Fix it with 4 screws into each horn."],
    src: "hardware/DESIGN.md (slide the thigh link onto the knee servo's horns; every U bracket slides on sideways), cad/takt.py (femur horn holes)",
    view: { az: 55, el: 22, d: 560, t: [90, 110, 90] },
    add: [...perLeg(["knee", "blade", "bins_0", "bins_1", "bscrew_0", "bscrew_1", "s3", "label_s3", "cs3_t", "cs3_b"], { frame: "femur", v: [90, 0, 0] }),
      ...perLeg(["hf3_t"], { frame: "self", v: [0, 0, 22] }, 0.32, 0.7, 0.8), ...perLeg(["hf3_b"], { frame: "self", v: [0, 0, -22] }, 0.32, 0.7, 1.0)],
  },
  {
    title: "Soft feet",
    legs: true,
    parts: [{ icon: "foot", name: "Soft foot (TPU)", n: 6, per: "1 per leg", bom: "p2_foot" }, { icon: "m3x20", name: "Screw M3 x 20", n: 6, per: "1 per foot", bom: "m3x20" },
      { icon: "nut3", name: "Lock nut M3", n: 6, per: "1 per foot", bom: "nut_m3" }],
    tools: ["flat", "nut"],
    say: ["Slip the soft foot over the tip of the blade.", "Push one screw through foot and blade and tighten the lock nut."],
    notes: [{ k: "confirm", t: "The design notes say M3 x 16. The parts list buys M3 x 20, so the nut grips fully." }],
    src: "cad/takt.py (foot: slips over the blade's tongue, one M3 across), hardware/DESIGN.md, hardware/BOM.md",
    view: { az: 45, el: 12, d: 520, t: [100, 140, 40] },
    add: [...perLeg(["foot"], { frame: "tibia", v: [40, -14, 0] }),
      ...perLeg(["fscrew"], { frame: "self", v: [0, 0, 25] }, 0.32, 0.7, 0.8), ...perLeg(["fnut"], { frame: "self", v: [0, 0, -20] }, 0.32, 0.7, 1.0)],
  },
  {
    title: "Leg cables",
    legs: true,
    parts: [{ icon: "cable", name: "Servo cable, 15 cm", n: 18, per: "3 per leg, in the servo boxes" }],
    tools: [],
    say: ["Each leg is one chain. Plug the knee servo into the thigh servo, the thigh servo into the hip servo.",
      "One more cable runs from the hip servo into the body."],
    notes: [{ k: "rec", t: "Leave some slack at every joint, so the leg can move all the way." },
      { k: "confirm", t: "Cable paths are not designed yet, and nobody has checked that 15 cm reaches everywhere." }],
    src: "hardware/DESIGN.md (each leg chains the knee, thigh and hip servos to the body), hardware/BOM.md (cables in the servo box)",
    view: { az: 60, el: 46, d: 700, t: [50, 50, 110] },
    add: perLeg(["cable_32", "cable_21", "cable_1h"], { grow: true }, 0.3, 1.3),
  },
  {
    title: "Bus hub and power",
    parts: [{ icon: "stripboard", name: "Strip board 50 x 100 mm", n: 1, bom: "stripboard" },
      { icon: "header", name: "Molex Mini-SPOX header, 3 pins", n: 7, spare: 1, per: "plus 1 spare", bom: "molex" },
      { icon: "xt60", name: "XT60 plug and socket", n: 1, spare: 1, per: "plus 1 spare", bom: "xt60" },
      { icon: "fuse", name: "Blade fuse 15 A", n: 1, spare: 2, per: "plus 2 spare", bom: "fuse" },
      { icon: "fuseholder", name: "Fuse holder", n: 1, bom: "fuse_holder" },
      { icon: "wire", name: "Silicone wire 1.5 mm², red and black", n: 1, per: "1 m each", bom: "wire" }],
    tools: ["iron", "shrink"],
    say: ["Solder the bus hub: 7 headers on the strip board, one for each leg and one for the bus adapter.",
      "Battery power runs through the XT60 plug and the 15 A fuse to the hub. Pulling the plug switches the robot off. Cover every solder joint with heat-shrink tubing."],
    notes: [{ k: "confirm", t: "There is no wiring plan yet, and no place for the hub in the model. The parts list suggests the free space under the lid: it goes there in step 18." },
      { k: "confirm", t: "The XT60 plug, the fuse, the fuse holder and the power wire have no place in the model yet, so they lie beside the build in this step only. Their places inside the body will be fixed in the first real build." },
      { k: "info", t: "The small slot in the back of the body was drawn for a switch. A small switch cannot carry the servo current, so the XT60 plug is the switch and the slot stays empty." }],
    src: "hardware/BOM.md (bus hub, XT60 as the main switch, fuse, spares, CAD changes), hardware/DESIGN.md (not designed yet: power switch)",
    view: { az: 20, el: 50, d: 420, t: [0, 0, 120] },
    add: [{ ids: ["e_hub"], off: { stay: true } },
      { ids: ["loose_xt60"], off: { stay: true }, delay: 0.3 }, { ids: ["loose_fuse"], off: { stay: true }, delay: 0.3 },
      { ids: ["loose_fuseholder"], off: { stay: true }, delay: 0.3 }, { ids: ["loose_wire"], off: { stay: true }, delay: 0.3 }],
  },
  {
    title: "Deck and bus adapter",
    parts: [{ icon: "deck", name: "Deck", n: 1, bom: "p2_deck" }, { icon: "m25x6", name: "Screw M2.5 x 6", n: 4, per: "into the post inserts", bom: "m25x6" },
      { icon: "adapter", name: "Bus servo adapter", n: 1, bom: "adapter" },
      { icon: "st2x6", name: "Self-tapping screw 2.0 x 6", n: 4, per: "for the adapter, like the 4 board holes in the deck", bom: "st2x6" },
      { icon: "cable", name: "Servo cable, 15 cm", n: 1, per: "the 19th, from a servo box: hub to adapter" }],
    tools: ["hex", "screwdriver"],
    say: ["Lay the deck on its four posts above the battery space. Screw it down.",
      "Screw the bus adapter onto the deck and plug in the one cable from the hub."],
    notes: [{ k: "confirm", t: "The adapter in the parts list has other holes (37 x 28 mm) than the deck drawn now. The deck needs a CAD change first. Until then the adapter's screws lie beside the build." }],
    src: "cad/takt.py (deck on the four posts, bus board space and holes), hardware/BOM.md (bus servo adapter: hole spacing 37 x 28 mm, CAD changes, 2.0 mm screws for the adapter)",
    view: { az: 15, el: 55, d: 400, t: [0, 0, 110] },
    add: [{ ids: ["deck"], off: { body: [0, 0, 90] } }, { ids: ["deck_s"], off: { body: [0, 0, 30] }, delay: 0.6, dur: 0.7 },
      { ids: ["e_adapter"], off: { body: [0, 0, 120] }, delay: 1.0 }, { ids: ["loose_adapter_s"], off: { stay: true }, delay: 1.0 },
      { ids: ["loose_cable"], off: { stay: true }, delay: 1.0 }],
  },
  {
    title: "The face",
    parts: [{ icon: "acrylic", name: "Smoked window 34 x 28 x 2 mm", n: 1, bom: "acrylic" }, { icon: "oled", name: "Display 1.3 inch, SH1106", n: 1, bom: "oled" },
      { icon: "camera", name: "Camera Module 3", n: 1, bom: "camera" }, { icon: "st2x6", name: "Self-tapping screw 2.0 x 6", n: 4, per: "for the camera", bom: "st2x6" },
      { icon: "led", name: "LED 3 mm with resistor", n: 1, bom: "led" }],
    tools: ["screwdriver"],
    say: ["Set the window into its frame from inside. Slide the display down its rails from above.",
      "Screw the camera onto its 4 posts behind the small hole. Push the LED into its hole."],
    notes: [{ k: "confirm", t: "The display is a little bigger than drawn. Camera depth and lens position are not checked." },
      { k: "info", t: "These parts belong to the \"Face & voice\" set of the parts list. The robot walks without them." }],
    src: "cad/takt.py (window, display rails, camera standoffs, LED), hardware/DESIGN.md (electronics), hardware/BOM.md",
    view: { az: 4, el: 10, d: 190 },
    add: [{ ids: ["window"], off: { body: [0, -40, 30] } }, { ids: ["e_oled"], off: { body: [0, 0, 90] }, delay: 0.6 },
      { ids: ["e_camera"], off: { body: [0, -50, 0] }, delay: 1.2 }, { ids: ["cam_s"], off: { body: [0, -25, 0] }, delay: 1.7, dur: 0.7 },
      { ids: ["e_led"], off: { body: [0, -40, 0] }, delay: 1.9 }],
  },
  {
    title: "Brain and sound under the lid",
    sub: true,
    parts: [{ icon: "lid", name: "Lid", n: 1, bom: "p2_lid" }, { icon: "pi", name: "Raspberry Pi Zero 2 W", n: 1, bom: "pi" }, { icon: "sd", name: "microSD card 32 GB", n: 1, bom: "sd" },
      { icon: "m25x6", name: "Screw M2.5 x 6", n: 4, per: "for the Pi", bom: "m25x6" },
      { icon: "speaker", name: "Speaker", n: 1, bom: "speaker" }, { icon: "amp", name: "Amplifier", n: 1, bom: "amp" }, { icon: "mic", name: "Microphone", n: 1, bom: "mic" },
      { icon: "hub", name: "Bus hub (from step 15)", n: 1, per: "soldered in step 15", bom: "stripboard", reuse: true },
      { icon: "imu", name: "Motion sensor BNO085", n: 1, bom: "imu" },
      { icon: "ribbon", name: "Camera cable, 200 mm", n: 1, per: "from the camera to the Pi", bom: "cam_cable" },
      { icon: "jumper", name: "Jumper wires", n: null, per: "from the pack of 40; how many is not designed yet", bom: "jumpers" }],
    tools: ["hex", "tape"],
    say: ["Build this on the table: turn the lid over. Put the microSD card into the Pi. Screw the Pi onto its four posts, parts facing down. Put the speaker under the grille.",
      "Tape the amplifier and the microphone beside it, and the bus hub into the free space at the front. Connect the camera cable and the jumper wires."],
    notes: [{ k: "confirm", t: "The motion sensor, the microSD card, the camera cable and the jumper wires have no place in the model yet, so they lie beside the build in this step only. Where they go, and the wire paths, will be fixed in the first real build." },
      { k: "confirm", t: "The Pi posts have no inserts: the screws go straight into the plastic." }],
    src: "cad/takt.py (electronics: Pi under the lid, speaker, amplifier, microphone; the Pi's lid bosses), hardware/DESIGN.md (tape for now), hardware/BOM.md (the hub under the lid)",
    view: { only: ["lid", "e_pi", "pi_s", "e_speaker", "e_amp", "e_mic", "e_hub"], az: 30, el: 45, d: 290, side: 180, close: true },
    add: [{ ids: ["e_pi"], off: { body: [0, 0, -50] } }, { ids: ["pi_s"], off: { body: [0, 0, -30] }, delay: 0.5, dur: 0.7 },
      { ids: ["e_speaker"], off: { body: [0, 0, -40] }, delay: 0.6 },
      { ids: ["e_amp", "e_mic"], off: { body: [0, 0, -40] }, delay: 1.0 }, { ids: ["e_hub"], off: { body: [0, 0, -40] }, delay: 1.4 },
      { ids: ["loose_imu"], off: { stay: true }, delay: 0.2 }, { ids: ["loose_sd"], off: { stay: true }, delay: 0.2 },
      { ids: ["loose_ribbon"], off: { stay: true }, delay: 0.2 }, { ids: ["loose_jumpers"], off: { stay: true }, delay: 0.2 }],
  },
  {
    title: "Close the lid",
    parts: [{ icon: "lid", name: "Lid with the Pi, sound and the bus hub (from step 18)", n: 1 }, { icon: "reg", name: "5 V regulator", n: 1, bom: "reg5" },
      { icon: "m3x20", name: "Screw M3 x 20", n: 4, bom: "m3x20" }],
    tools: ["tape", "flat"],
    say: ["Tape the 5 V regulator inside the back wall of the body.", "Put the lid with everything on it onto the body. Screw it down with 4 screws into the inserts."],
    notes: [{ k: "rec", t: "Before you close it, check that no cable is pinched." }],
    src: "cad/takt.py (lid screws on the four columns, the 5 V regulator at the back wall), hardware/DESIGN.md (4 x M3 x 20 for the lid, tape for now)",
    view: { az: 30, el: 40, d: 560, t: [0, 0, 120] },
    add: [{ ids: ["e_reg5"], off: { body: [0, 40, 0] } },
      { ids: ["lid", "e_pi", "pi_s", "e_speaker", "e_amp", "e_mic", "e_hub"], off: { body: [0, 0, 130] }, delay: 0.6 },
      { ids: ["lid_s"], off: { body: [0, 0, 40] }, delay: 1.4, dur: 0.7 }],
  },
  {
    title: "Battery and belly hatch",
    parts: [{ icon: "battery", name: "Battery 3S 1000 mAh, XT60, charged", n: 1, bom: "battery" }, { icon: "hatch", name: "Belly hatch", n: 1, bom: "p2_hatch" },
      { icon: "m3x6cs", name: "Countersunk screw M3 x 6", n: 4, bom: "m3x6cs" }],
    tools: ["hex"],
    say: ["Lift the robot off its box and hold it up. Slide the battery in through the opening in the belly.", "Close the belly hatch with 4 screws. Put the robot back on its box."],
    notes: [{ k: "confirm", t: "Where the battery's XT60 plug comes out, so you can reach it with the hatch closed, is not designed yet. To be confirmed in the first real build." }, { k: "confirm", t: "This battery is 3 mm longer and 1 mm thicker than the space drawn. The battery space needs a CAD change first." },
      { k: "rec", t: "Charge and store the battery in the fire-safe bag (step 3)." },
      { k: "rec", t: "Two tall blocks under the front and the back of the body hold it up for you, as drawn. The belly stays open." }],
    src: "cad/takt.py (hatch, hatch screws, battery space), hardware/DESIGN.md, hardware/BOM.md (battery, CAD changes)",
    view: { az: 35, el: -4, d: 560, t: [0, 0, 150], lift: 110 },
    jig: true,
    add: [{ ids: ["e_battery"], off: { body: [0, 0, -70] } }, { ids: ["hatch"], off: { body: [0, 0, -90] }, delay: 0.6 },
      { ids: ["hatch_s"], off: { body: [0, 0, -30] }, delay: 1.3, dur: 0.7 }],
  },
  {
    title: "First power-on",
    parts: [{ icon: "checker", name: "Battery checker", n: 1, per: "from step 3, on the balance lead", bom: "checker", reuse: true }],
    tools: ["computer"],
    say: ["Keep the robot on its box, legs free. Plug the battery checker onto the balance lead. Then plug in the battery: the XT60 plug is the main switch.",
      "Check that every servo answers. Pull the plug to switch off."],
    notes: [{ k: "rec", t: "This whole step is common practice, not from the design notes." },
      { k: "confirm", t: "The XT60 plug must be reachable with the belly hatch closed. Where it comes out is not designed yet (see step 20)." },
      { k: "confirm", t: "The software that runs the robot is not written yet." }],
    src: "hardware/BOM.md (XT60 as the main switch, battery checker), hardware/DESIGN.md (firmware not designed yet)",
    view: { az: 0, el: 14, d: 520, t: [0, 0, 110] },
    add: [{ ids: ["eyes"], off: { fade: true }, dur: 0.8 }, { ids: ["loose_checker"], off: { stay: true }, delay: 0.2 }],
  },
  {
    title: "Stand it up",
    parts: [],
    tools: [],
    say: ["Take the robot off its box and set it on its six feet. That is Takt.",
      "Next comes Puls, the learning model that will teach it to walk. No training attempt has run yet."],
    notes: [{ k: "confirm", t: "This guide will be corrected after the first real build." }],
    src: "The stand pose from the CAD model (model.json, the \"stand\" pose)",
    view: { az: 25, el: 18, d: 820, t: [0, 0, 80] },
    finale: true,
    add: [],
  },
];
