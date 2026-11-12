/* SPDX-License-Identifier: MIT
 * rooms.js: how the Hall is furnished for each of the 35 levels, in 3D.
 * Names and texts live in levels.js; this file only holds the furniture and where the robots stand.
 * Content follows levels.js. Rooms are 8 x 5 tiles; one tile is a third of a metre.
 * Heights are drawn taller than real so they show. Progress does NOT live here: see status.json.
 *
 * Room props (x to the right, y into the room, z up, in tiles):
 *   ["start", x, y]                      striped start tile
 *   ["goal", x, y]                       green goal tile
 *   ["zone", x0, y0, x1, y1]             green field for a thing
 *   ["ring", cx, cy, r]                  dashed circle on the floor
 *   ["path", [[x, y], ...]]              dashed arrow on the floor
 *   ["box", x0, y0, x1, y1, z0, z1, colour]
 *   ["ramp", x0, y0, x1, y1, h, colour]
 *   ["cyl", cx, cy, r, h, side, inner]   cup or bowl
 *   ["ball", x, y, r]  ["pit", x0, x1]  ["bumps"]  ["ice"]  ["line", x, y0, y1, bold]
 *   ["tray", x0, y0, x1, y1, h]  ["push", [x, y, z], [x, y, z]]
 *   ["label", x, y, z, text, kind]       kind: "t" teacher, "r" warning
 */
  const arc = [];
  for (let t = 0; t <= 90; t += 9) { const r = t * Math.PI / 180; arc.push([3.5 + 0.95 * Math.cos(r), 2.5 + 0.95 * Math.sin(r)]); }
  const pen = ["box", 2.6, 2.44, 3.05, 2.5, 0, 0.04, "pen"];
  const walk = ["path", [[2.1, 2.5], [3.9, 2.5]]];
  const stairs10 = [], labels10 = [];
  for (let i = 0; i < 7; i++) {
    const x0 = 2.6 + i * 0.72;
    stairs10.push(["box", x0, 1.3, x0 + 0.72, 3.7, 0, 0.12 * (i + 1), "grey"]);
    labels10.push(["label", x0 + 0.36, 1.7, 0.12 * (i + 1) + 0.14, String(2 * (i + 1))]);
  }
  const jump = [];
  for (let k = 1; k <= 8; k++) {
    const x = 2.1 + 0.3 * k;
    jump.push(["line", x, 1.4, 3.6, k % 2 === 0]);
    if (k % 2 === 0) jump.push(["label", x, 3.9, 0.02, String(k * 10)]);
  }

const L = [
    { w: 1, name: "Walk Off", time: 30, robot: { x: 1.5, y: 2.5 },
      props: [["start", 1, 2], ["ring", 1.5, 2.5, 0.9], ["label", 1.5, 3.7, 0.02, "30 cm"]], },
    { w: 1, name: "Stand Up", time: 30, robot: { x: 3.5, y: 2.5, low: true },
      props: [["start", 3, 2], ["label", 3.5, 2.5, 0.75, "on its belly"]], },
    { w: 1, name: "Straight Ahead", time: 60, robot: { x: 1.5, y: 2.5 },
      props: [["start", 1, 2], ["goal", 4, 2], walk, ["label", 3, 2.5, 0.5, "1 m"]], },
    { w: 1, name: "Turn", time: 30, robot: { x: 3.5, y: 2.5 },
      props: [["start", 3, 2], ["path", arc], ["label", 4.6, 3.6, 0.02, "90°"]], },
    { w: 1, name: "Sidestep", time: 60, robot: { x: 3.5, y: 1.5 },
      props: [["start", 3, 1], ["goal", 3, 2.5], ["path", [[3.5, 2.05], [3.5, 2.6]]], ["label", 4.5, 3.2, 0.02, "0.5 m"]], },
    { w: 1, name: "Hold the Height", time: 30, robot: { x: 3.5, y: 2.5, heights: true },
      props: [["start", 3, 2], ["box", 4.75, 1.55, 4.85, 1.65, 0, 0.5, "post"],
        ["box", 4.7, 1.5, 4.9, 1.7, 0.09, 0.11, "bar"], ["box", 4.7, 1.5, 4.9, 1.7, 0.186, 0.206, "bar"],
        ["box", 4.7, 1.5, 4.9, 1.7, 0.285, 0.305, "bar"], ["label", 5.75, 1.6, 0.42, "30 / 62 / 95 mm"]], },
    { w: 1, name: "Shove", time: 60, robot: { x: 2.5, y: 2.5 },
      props: [["start", 1, 2], ["goal", 4, 2], ["path", [[3.2, 2.5], [3.9, 2.5]]], ["push", [2.5, 4.0, 0.32], [2.5, 3.2, 0.32]], ["label", 2.5, 4.3, 0.34, "push", "r"]], },

    { w: 2, name: "Ledge", time: 60, robot: { x: 1.5, y: 2.5 },
      props: [["start", 1, 2], ["box", 3.2, 1.2, 6.8, 3.8, 0, 0.1, "podium"], ["label", 5, 3.8, 0.24, "15 mm"]], },
    { w: 2, name: "Tall Box", time: 90, robot: { x: 1.5, y: 2.5 },
      props: [["start", 1, 2], ["box", 3.8, 1.4, 6.6, 3.6, 0, 0.4, "podium"], ["label", 5.2, 3.6, 0.55, "60 mm"]], },
    { w: 2, name: "Limit Stairs", time: 120, robot: { x: 1.5, y: 2.5 }, measure: "cm",
      props: [["start", 1, 2], ...stairs10, ...labels10], },
    { w: 2, name: "Staircase", time: 120, robot: { x: 1.5, y: 2.5 },
      props: [["start", 1, 2], ["box", 3.6, 1.4, 4.3, 3.6, 0, 0.15, "grey"], ["box", 4.3, 1.4, 5.0, 3.6, 0, 0.3, "grey"], ["box", 5.0, 1.4, 5.7, 3.6, 0, 0.45, "grey"], ["box", 5.7, 1.4, 7.5, 3.6, 0, 0.6, "podium"]], },
    { w: 2, name: "Bumpy Floor", time: 90, robot: { x: 1.5, y: 2.5 },
      props: [["start", 1, 2], ["goal", 6, 2], ["bumps"]], },
    { w: 2, name: "Slippery Floor", time: 90, robot: { x: 1.5, y: 2.5 },
      props: [["start", 1, 2], ["goal", 6, 2], ["ice"], ["label", 4, 3.6, 0.2, "slippery"]], },
    { w: 2, name: "Backpack", time: 60, robot: { x: 1.5, y: 2.5, load: true },
      props: [["start", 1, 2], ["goal", 4, 2], walk, ["label", 1.5, 2.5, 0.95, "200 g, fixed"]], },
    { w: 2, name: "Steady Carry", time: 60, robot: { x: 1.5, y: 2.5, loose: true },
      props: [["start", 1, 2], ["goal", 4, 2], walk, ["label", 1.5, 2.5, 0.95, "loose"]], },
    { w: 2, name: "Carry over Bumps", time: 90, robot: { x: 1.5, y: 2.5, loose: true },
      props: [["start", 1, 2], ["goal", 6, 2], ["bumps"], ["label", 1.5, 2.5, 0.95, "loose"]], },
    { w: 2, name: "Carry on Ice", time: 90, robot: { x: 1.5, y: 2.5, loose: true },
      props: [["start", 1, 2], ["goal", 6, 2], ["ice"], ["label", 1.5, 2.5, 0.95, "loose"], ["label", 4, 3.6, 0.2, "slippery"]], },
    { w: 2, name: "Ramp", time: 90, robot: { x: 1.5, y: 2.5 },
      props: [["start", 1, 2], ["ramp", 2.8, 1.5, 5.3, 3.5, 0.5, "grey"], ["box", 5.3, 1.5, 7.3, 3.5, 0, 0.5, "podium"]], },
    { w: 2, name: "Gap", time: 90, robot: { x: 1.5, y: 2.5 },
      props: [["start", 1, 2], ["pit", 3.6, 4.2], ["goal", 5.8, 2], ["label", 3.9, 4.6, 0.2, "gap"]], },
    { w: 2, name: "Duck", time: 90, robot: { x: 1.5, y: 2.5 }, mayFail: true,
      props: [["start", 1, 2], ["goal", 5.8, 2], ["box", 3.95, 0.8, 4.05, 0.9, 0, 0.44, "post"], ["box", 3.95, 0.8, 4.05, 4.2, 0.38, 0.43, "bar"], ["box", 3.95, 4.1, 4.05, 4.2, 0, 0.44, "post"]], },
    { w: 2, name: "Hop", time: 30, robot: { x: 3.5, y: 2.5, air: 0.3 }, mayFail: true,
      props: [["start", 3, 2], ["label", 3.5, 2.5, 1.2, "all six feet in the air"]], },
    { w: 2, name: "Long Jump", time: 60, robot: { x: 1.5, y: 2.5 }, measure: "cm", mayFail: true,
      props: [["start", 1, 2], ...jump, ["label", 3.3, 1.1, 0.02, "cm from the line"]], },
    { w: 2, name: "High Jump", time: 60, robot: { x: 1.5, y: 2.5 }, measure: "cm", mayFail: true,
      props: [["start", 1, 2], ["box", 2.8, 1.9, 3.5, 3.1, 0, 0.15, "grey"], ["box", 3.9, 1.9, 4.6, 3.1, 0, 0.3, "grey"], ["box", 5.0, 1.9, 5.7, 3.1, 0, 0.45, "grey"], ["box", 6.1, 1.9, 6.8, 3.1, 0, 0.6, "grey"], ["label", 5.0, 1.6, 0.95, "higher and higher"]], },
    { w: 2, name: "Big Gap", time: 90, robot: { x: 1.6, y: 2.5 }, mayFail: true,
      props: [["start", 1, 2], ["pit", 3.0, 5.0], ["goal", 6.2, 2], ["label", 4.0, 4.6, 0.2, "as wide as its best jump"]], },

    { w: 3, name: "Push the Block", time: 120, robot: { x: 1.6, y: 2.5 },
      props: [["start", 1, 2], ["zone", 5.2, 2.0, 6.2, 3.0], ["box", 3.0, 2.2, 3.6, 2.8, 0, 0.45, "wood"], ["path", [[3.9, 2.5], [5.1, 2.5]]]], },
    { w: 3, name: "Push the Pen", time: 120, robot: { x: 1.5, y: 2.5 },
      props: [["start", 1, 2], ["zone", 5.0, 1.7, 6.6, 2.9], pen, ["label", 2.85, 2.1, 0.22, "pen"]], },
    { w: 3, name: "Pen in the Tray", time: 120, robot: { x: 1.5, y: 2.5 },
      props: [["start", 1, 2], pen, ["tray", 5.0, 1.8, 6.5, 2.8, 0.07]], },
    { w: 3, name: "Pen in the Bowl", time: 180, robot: { x: 1.5, y: 2.5 },
      props: [["start", 1, 2], pen, ["cyl", 5.8, 2.4, 0.36, 0.16, "#b9bec6", "#7c838c"]], },
    { w: 3, name: "Board as a Ramp", time: 180, robot: { x: 1.5, y: 2.0 },
      props: [["start", 1, 1.5], ["box", 2.4, 3.7, 4.6, 4.3, 0, 0.05, "wood"], ["box", 5.6, 1.2, 7.6, 3.8, 0, 0.9, "podium"], ["label", 3.5, 4.0, 0.3, "board"], ["label", 6.6, 2.5, 1.2, "too high to climb"]], },
    { w: 3, name: "Stack the Blocks", time: 240, robot: { x: 1.5, y: 2.5 },
      props: [["start", 1, 2], ["box", 2.8, 3.6, 3.5, 4.3, 0, 0.5, "wood"], ["box", 3.6, 0.8, 4.3, 1.5, 0, 0.5, "wood"], ["box", 5.8, 1.2, 7.6, 3.8, 0, 1.2, "podium"]], },

    { w: 4, name: "Go to the Cup", time: 60, robot: { x: 1.6, y: 2.5 },
      props: [["start", 1, 2], ["cyl", 5.2, 1.6, 0.13, 0.3, "#e0483e", "#3a1513"], ["ball", 5.4, 3.4, 0.15], ["label", 1.6, 2.5, 0.95, "“Go to the cup.”"]], },
    { w: 4, name: "Copy the Pen Trick", time: 180, robot: { x: 1.6, y: 3.2, yaw: -0.5 }, teacher: { x: 4.9, y: 1.8 },
      props: [["start", 1, 2.7], ["tray", 5.6, 1.4, 6.8, 2.2, 0.07], ["box", 5.9, 1.78, 6.35, 1.84, 0.07, 0.11, "pen"], ["label", 4.9, 1.8, 0.95, "teacher shows it", "t"], ["label", 1.6, 3.2, 0.95, "student watches"]], },
    { w: 4, name: "Summit: Teacher Practises", time: 180, robot: null, teacher: { x: 2.0, y: 2.5 },
      props: [["start", 1.5, 2], ["box", 3.6, 2.2, 4.3, 2.9, 0, 0.5, "wood"], ["box", 5.8, 1.2, 7.6, 3.8, 0, 0.9, "podium"], ["label", 2.0, 2.5, 0.95, "teacher practises", "t"]], },
    { w: 4, name: "Summit: Student", time: 180, robot: { x: 1.5, y: 3.5, yaw: -0.6 }, teacher: { x: 6.5, y: 1.6, zg: 0.9, yaw: Math.PI },
      props: [["start", 1, 3], ["box", 4.9, 1.3, 5.6, 2.0, 0, 0.5, "wood"], ["box", 5.6, 0.6, 7.4, 2.6, 0, 0.9, "podium"], ["label", 6.5, 1.6, 1.85, "teacher is up", "t"], ["label", 1.5, 3.5, 0.95, "student"]], },
    { w: 4, name: "Summit: Hard Mode", time: 240, robot: { x: 1.5, y: 2.2 },
      props: [["start", 1, 1.7], ["box", 3.4, 2.0, 4.1, 2.7, 0, 0.5, "heavy"], ["box", 2.2, 3.8, 4.4, 4.4, 0, 0.05, "wood"], ["box", 5.8, 1.2, 7.6, 3.8, 0, 0.9, "podium"], ["label", 3.75, 2.35, 0.8, "too heavy", "r"], ["label", 3.3, 4.1, 0.3, "board"]], },
  ];
  L.forEach((l, i) => { l.n = i + 1; });
export const ROOMS = L;
