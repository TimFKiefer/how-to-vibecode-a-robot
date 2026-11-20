# How to vibecode a robot

**Takt** is a six-legged walking robot, built in public and fully vibecoded: AI coding agents write the code, check
it and fix it. **Puls** is its brain: a small model with learned time in two speeds, a brake on surprise, and commands
as channels. Puls learns in simulation first; nothing is bought or built yet.

Website: https://timfkiefer.github.io/how-to-vibecode-a-robot/

## Licences

This repository holds software, hardware and texts, so it has more than one licence (GitHub shows "Other"). Each
file's own SPDX line wins; files without one are covered by [`REUSE.toml`](REUSE.toml). Details in
[`LICENSE.md`](LICENSE.md), the texts in [`LICENSES/`](LICENSES/).

| What | Licence |
|---|---|
| Puls (`sim/puls/`) | AGPL-3.0-only |
| Simulation, checks, website code, tools, `cad/render.py`, `cad/viewer/*.mjs` and `*.py` | MIT |
| CAD source and print files: `cad/takt.py`, `cad/parts/*.step` and `*.stl`, `cad/report.json`, `cad/blade120_report.json` | CERN-OHL-S-2.0 |
| Files with printable part geometry for viewing: `docs/model.json`, `docs/meshes.bin`, `cad/viewer/*.json`, `sim/assets/meshes/`, `sim/bodies/` | CERN-OHL-S-2.0 |
| Fonts (`docs/fonts/`) | OFL-1.1, by their authors, unchanged |
| three.js (`docs/vendor/three/`) | MIT, by the three.js authors, unchanged |
| Texts (every README, `hardware/*.md`, the step texts of the build guide, the part descriptions, `docs/status.json`) and pictures (`cad/renders/`) | CC-BY-SA-4.0 |
| Mixed website files (`guide-steps.js`, `bom-data.js`, `levels.js`, `index.html`, `bom.html`, `build.html`) | MIT AND CC-BY-SA-4.0 |
| The licence texts in `LICENSES/` and `sim/puls/LICENSE` | unchanged, under their own terms |

Attribution for CC BY-SA: "Takt by TimFKiefer, https://github.com/TimFKiefer/how-to-vibecode-a-robot".

## Status

Puls is working on level 3. Levels passed: 2. Attempt on the board: 0. Updated 2026-11-20.

Levels 1 and 2 are passed: Puls walks off on 4 of 5 unseen simulated bodies and stands up from its belly on 20 of 20. Level 3 is next.

Progress lives in [`docs/status.json`](docs/status.json); the website reads it.

### Videos

No video is out yet. Every training run will be filmed in full.

## Quick start

You need Python 3.14 for the simulation, bun (or node) for the website and viewer checks, and FreeCAD only if you
change the CAD.

**1. Look at the website.** Online at https://timfkiefer.github.io/how-to-vibecode-a-robot/, or locally:

```bash
python3 -m http.server -d docs
```

then open http://localhost:8000/.

**2. Print the parts or change the CAD.** The print files are in [`cad/parts/`](cad/parts/), one STEP and one STL per
part, already in print orientation (the file name says how many you need, `_x6` = six). `foot_sneaker_x6` is an
alternative to `foot_x6`: print one of the two. Print `fit_gauge` and `fit_collar` first, before the full set. To
change the robot, edit [`cad/takt.py`](cad/takt.py) and run it with FreeCAD; [`cad/README.md`](cad/README.md) has the
order of the scripts.

**3. Run the simulation: the hall, the tasks, the fixed gait.**

```bash
python3.14 -m venv sim/.venv
sim/.venv/bin/pip install -r sim/requirements.txt
cd sim
.venv/bin/python -m robot2sim.run --brain scripted --task walk_1m --nominal
```

[`sim/README.md`](sim/README.md) explains every command, the tasks and the replay.

**4. Check Puls.** Puls is not connected to the simulated robot yet (that is the next stage); these checks run its
learning core on a rhythm.

```bash
sim/.venv/bin/python sim/checks/check_e1_all.py   # the simulation, about 2 minutes
sim/.venv/bin/python sim/checks/check_e2_all.py   # Puls, about 6 minutes
```

Two rows measure speed and can fail on a busy or slow computer; run them again when it is quiet.

**5. Build Takt.** The build guide is on the website (`docs/build.html`), the parts list in
[`hardware/BOM.md`](hardware/BOM.md) and on the [parts list page](https://timfkiefer.github.io/how-to-vibecode-a-robot/bom.html), with shops and prices for Germany.
Elsewhere, give the [prompt on that page](https://timfkiefer.github.io/how-to-vibecode-a-robot/bom.html#prompt) to your own AI agent and it finds the same parts in
your country. The design notes are in [`hardware/DESIGN.md`](hardware/DESIGN.md).

## What is where

| Folder | What |
|---|---|
| `docs/` | the website (GitHub Pages), with its fonts and three.js |
| `cad/` | the FreeCAD script, the parts, the renders, the viewer data and its self-test |
| `hardware/` | the design notes and the parts list |
| `sim/` | the MuJoCo simulation, its checks, and Puls in `sim/puls/` |
| `checks/website/`, `tools/website/` | the website's checks and its mesh packer |

## Third-party parts

| Part | Licence | How |
|---|---|---|
| three.js | MIT | in `docs/vendor/three/`, unchanged |
| Fredoka, Inter, JetBrains Mono (fonts) | OFL-1.1 | in `docs/fonts/`, unchanged |
| MuJoCo | Apache-2.0 | installed by `pip`, not included |
| NumPy | BSD-3-Clause | installed by `pip`, not included |
| Pillow | HPND | installed by `pip`, not included |

FreeCAD is only a tool for the CAD and is not included.

## Contributing

Bug reports are welcome everywhere. Pull requests for the website, the simulation and the CAD are welcome under the
licence of their folder, with a sign-off. Puls (`sim/puls/`) takes no outside code. See
[`CONTRIBUTING.md`](CONTRIBUTING.md).

Source location: https://github.com/TimFKiefer/how-to-vibecode-a-robot
