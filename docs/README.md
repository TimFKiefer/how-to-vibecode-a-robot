# Takt website

The home page of the open-source project "How to vibecode a robot". Takt is a six-legged walking robot, and Puls is
its brain, a small learning model. Takt is built in public and fully vibecoded: Tim says what he wants, AI coding agents
(Claude Code) write the code, check it and fix it. The page keeps the title "How to vibecode a robot".

Every public word on the page is written for three readers: a curious visitor, a maker who wants to build Takt, and
someone who wants to vibecode their own robot. Internal plans and notes do not go on the page.

What the page does, top to bottom:

1. The robot assembles part by part while you scroll (the real CAD parts). Five captions, one at a time, tell how it
   is made: "How to vibecode a robot" (meet Takt and Puls), "Fully vibecoded" (no code by hand, you can do it yourself),
   "Built in public" (plans, parts and prices are here, every training run will be filmed in full), "Digital first"
   (nothing is bought yet, Puls learns in simulation first) and "Now it has to learn to walk." The details of the
   robot and its brain are further down, in the project section. The last caption links to the parts list and the
   build guide; it and the parts meter fade out before the road begins, so nothing slides up.
2. The road. The road chapter starts on the last screen of the assembly, so there is no empty screen in between. The
   camera flows from the assembled robot straight to the level Puls is on, while the stones rise: the levels from
   three before Puls to three after it solid and bright, every other level up to the Summit as a faint ghost stone.
   The level card on the left starts at Puls's level ("Puls is here") and carries the live status in a compact line
   (the note and the update date from `status.json`). Scrolling on goes one level per step up to the last solid stone
   (Puls's level + 3, fewer at the end of the road). When the card switches to level n, a grey, see-through ghost of
   Takt walks from stone n - 1 to stone n, its legs stepping with the CAD tripod gait, driven by the scroll alone
   (no clock, no loop), and stands there until the next level. Scrolling back walks it back. It is labelled
   "Preview · the way ahead"; the blue robot stays on Puls's stone. The end shot shows the whole road from the Hall
   to the Summit, the ghost on the last solid stone, and the button into the full roadmap in the middle. Two small
   tags name the stones there ("Puls · Level 01", "Ghost · Level 04"), because from that far the robots are tiny.
   Nothing on the page moves on a clock: the ring round Puls's stone glows and the Summit flag waves with the scroll,
   and when nobody scrolls nothing is drawn.
3. The project: one short lead, then two full-width bands that preview the parts list
   and the build guide. The parts list band (`bom.html`) shows the black board with the total the parts list opens
   on (everything, own printer, with shipping) and the groups' goods, which add up to that total without shipping;
   the core total is in the small print. It counts up once, in 0.8 s, when the board comes into view, and always ends
   on the real total. The build guide band (`build.html`) shows the CAD robot built in the guide's own order
   (`guide-steps.js`): the scroll goes from the first step to the last, the parts of earlier steps stand built, the step's own
   parts come in, later parts are not there yet. The band calls the guide an easy step-by-step tutorial in 3D. Then
   three cards: the robot (servos, legs, height, what is inside, weight), the brain Puls, and "Your turn" (watch every
   training, build your own, vibe-code your own). Then the repository (github.com/TimFKiefer/how-to-vibecode-a-robot).
4. Videos, from `status.json`. With no video listed it says that none is online yet and that every training run
   will be filmed in full.
5. The honest footer.

The top bar links to the road, the videos, the parts list, the build guide and the full roadmap.

The full roadmap ("Roadmap" in the top bar or the button at the end of the road) shows all 35 levels: the Hall
furnished for each level, the level card on the left and a rail with all 35 on the right. It opens at the level
Puls is on. Scroll, use the arrow keys, Home and End, or click a level on the rail. Escape or "Close the roadmap"
takes you back to where you were. The link `#roadmap` opens it directly, also on a page that is already open.

Opening it plays a move: the hex stones in view fly to their places on the rail, and the page's camera flies into
the small Hall until it stands exactly where the roadmap's camera stands. On the way the page's labels, ring and
ghost fade, a half-built robot finishes coming together, and a blue robot standing in the small Hall moves with its
start tile to the spot where the roadmap's robot stands, so only one robot is there when the rooms cross (`hall.js` uses 1 unit = 1 tile = 1000/3 mm
with the same room centre and back wall, so the roadmap camera times 1000/3 is the same view on the page). The
furnished Hall is drawn from the same flying camera and crossfades in, so the small room becomes the big one. Closing
plays it backwards. With `prefers-reduced-motion` it is a short crossfade only.

Going to the parts list or the build guide morphs the clicked band into the top of that page (cross-document View
Transitions, names `bom-band` and `guide-band`), and the back button there morphs it back to where you were. A band
that is not in view does not morph, and only the band of the page you go to (or come back from) carries a name; the
page just crossfades. Coming back from a sub-page lands where you left, also when the address had a `#section` (the
page drops a plain `#section` from the address after its jump). Browsers without cross-document View Transitions fade
the page in. Every button and link answers a press with a short squeeze, and in-page jumps glide.

## Keep it up to date

Edit **`status.json`** whenever training moves on. Nothing else needs to change:

| Field | Meaning |
|---|---|
| `updated` | date of the change, `YYYY-MM-DD` |
| `done` | highest level passed (0 = only the Hall is built) |
| `active` | the level Puls is working on now |
| `attempt` | the attempt number on the board: the running or last attempt, `0` while no attempt has run (the board shows 00) |
| `note` | one plain sentence about where things stand |
| `results` | optional, per level, shown as "Result" in the level cards. Only list levels that have one. |
| `videos` | optional list of videos, newest first on the page: `{ "level": 1, "title": "...", "url": "https://...", "date": "YYYY-MM-DD" }`. Only real, published videos. An empty list (or none) shows "No video is out yet". Entries without a title or an http(s) link are skipped. With a `level` (1 to 35) and a YouTube link, that level's card gets "▶ Watch the training": in the full roadmap the video plays in the card, from the road card or a click on a passed level's stone the roadmap opens at that level and plays it. The player loads from youtube-nocookie.com only after that press, so a visit sends nothing to YouTube before it. |

A number that cannot be read falls back to the default, so a typo does not stop the page.

Example only, not real results (the shape of the field, with placeholders):

```json
"results": {
  "1":  { "note": "<what the test with learning switched off showed>" },
  "10": { "state": "measured", "value": "<number> cm", "note": "<what was measured>" },
  "20": { "state": "cannot", "note": "<what was tried>" }
}
```

`state` may be `measured` (levels 10, 22, 23 measure a limit) or `cannot` ("this body cannot do it", levels 20 to
24). Without a `state` the level counts as done or not from `done`. A level counts as passed only after its test
with learning switched off (the levels and their texts are in `levels.js`). Write only real results here.

## Preview: how the page looks at another level

Add `#preview-N` to the address, N from 1 to 35, for example `index.html#preview-7`. Any other number is ignored
and the real status shows. The page then shows itself
as if Puls were at Level N (levels below N done, N active, attempt 01 on the board, the board counts that level's
time). The top-bar links keep the preview in the address. A fixed banner says "Preview: this is how the page looks at Level 07. Not the real status." with a link back
to the real status. A preview never changes `status.json` and shows no results.

## Files

- `index.html`: the page (three.js 0.160.0 from `vendor/three/`, no build step)
- `levels.js`: the 35 levels (names, texts) and 2D room sketches (used only without 3D)
- `rooms.js`: how each level's room is furnished in 3D, and where the robot and the teacher stand
- `hall.js`: the 3D Hall per level (room, board, robot, teacher, furniture that sinks and rises)
- `bom.html`, `build.html` and their files: the parts list and the build guide, separate pages (not part of this
  page's code; the bands only read `bom-data.js` and `guide-steps.js`, and keep plain text when those cannot be read)
- `fk.js`, `model.json`: copies of `cad/viewer/fk.js` and `cad/viewer/model.json` (byte for byte); `meshes.bin` is
  packed from `cad/viewer/meshes.json`
- `status.json`: progress, the one file to update
- `theme.js`: light and dark mode for every page. It follows the system until a toggle is pressed; that pick is kept
  under `takt-theme` in this browser and wins on every page. `index.html` has two toggles (the top bar and the full roadmap's header).
- `fonts/`: the three fonts (Fredoka, Inter, JetBrains Mono; latin and latin-ext), `fonts/fonts.css` with their
  `@font-face` rules, and each family's license (`*-OFL.txt`, SIL Open Font License 1.1).
- `vendor/three/`: three.js 0.160.0, unchanged from its npm package: `three.module.min.js`,
  `examples/jsm/utils/BufferGeometryUtils.js` and its `LICENSE` (MIT). The pages load no file from another server.

The pages do not load `meshes.json` (1.2 MB of base64). `pack_meshes.py` packs it into `meshes.bin`, about a
quarter of the size: positions rounded to 16 bits per axis (off by about 0.001 mm at most), indices unchanged, the whole
file gzipped. `meshes.js` reads it back in the browser (`loadMeshes()`). `meshes-roundtrip.mjs` compares every mesh in
`meshes.bin` with `meshes.json` and fails on a changed index or a position more than 0.01 mm off. `guide-covers-bom.mjs`
checks that the build guide uses every line of the parts list and never takes more than it buys.

## Tools and checks

In the repository the site is `docs/`, `meshes.json` is `cad/viewer/meshes.json`, the tool is in `tools/website/` and
the checks are in `checks/website/`. Their defaults are those paths, so from the repository root:

```sh
python3 tools/website/pack_meshes.py          # cad/viewer/meshes.json -> docs/meshes.bin
bun checks/website/meshes-roundtrip.mjs       # docs/meshes.bin (read through docs/meshes.js) against cad/viewer/meshes.json
bun checks/website/guide-covers-bom.mjs       # docs/bom-data.js against docs/guide-steps.js
```

After the CAD export changes `cad/viewer/meshes.json`, pack it again, then run the round trip. The round trip needs `bun`
(it reads `meshes.bin` with `fetch`, the way the browser does, and Node's `fetch` does not open local files);
`guide-covers-bom.mjs` also runs under `node`.

The main page and the full roadmap share one WebGL canvas. Two extra small renderers draw the road card's picture
and the build guide band.

Run locally: `python3 -m http.server` in this folder, then open http://localhost:8000. Opening the file directly
does not work, because the page loads its data with `fetch`.
