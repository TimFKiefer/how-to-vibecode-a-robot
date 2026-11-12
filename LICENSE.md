# Licences

Copyright holder: TimFKiefer. Software and hardware: "Copyright (c) 2026 TimFKiefer and contributors".

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

**Mixed files.** A file that holds both program code and readable text carries `MIT AND CC-BY-SA-4.0`: the program
code is under MIT, the readable texts (step texts, part descriptions, captions) are under CC BY-SA 4.0.

**Which licence a file has.** The SPDX line at the top of a file wins. Files that have no room for one (JSON, STL, STEP,
OBJ, PNG, `meshes.bin`) and generated files are listed in [`REUSE.toml`](REUSE.toml). The licence texts are in
[`LICENSES/`](LICENSES/), unchanged and named after their SPDX identifiers.

**CERN-OHL-S-2.0.** Source location: https://github.com/TimFKiefer/how-to-vibecode-a-robot. If you make and share a changed version of the hardware, share its
source under the same licence and say what you changed.

**CC BY-SA 4.0.** Attribution: "Takt by TimFKiefer, https://github.com/TimFKiefer/how-to-vibecode-a-robot".

**Third-party parts** keep their own licences: three.js (MIT) in `docs/vendor/three/`, the fonts (OFL-1.1) in
`docs/fonts/`.
