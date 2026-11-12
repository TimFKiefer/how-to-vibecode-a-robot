# CAD

[`takt.py`](takt.py) is the whole robot as a FreeCAD script: every part, the leg maths, the masses and the load
estimates. The numbers in [`report.json`](report.json) come from it, and the simulation checks its body against them.

FreeCAD version: `takt.py` and the files in `parts/` were built with FreeCAD 1.1.4 (Open CASCADE 7.8). Other versions
can give slightly different meshes.

## Run, in this order

```bash
freecadcmd cad/takt.py                     # writes cad/parts/ (STEP + STL), cad/meshes/, cad/report.json
python cad/render.py                       # with FreeCAD's Python (numpy + Pillow): reads cad/meshes/, writes cad/renders/
freecadcmd cad/viewer/export_viewer.py     # writes cad/viewer/meshes.json, model.json, reference.json
node cad/viewer/selftest.mjs               # checks the leg maths in cad/viewer/fk.js against FreeCAD
```

`STAGE=parts freecadcmd cad/takt.py` stops after the parts. `cad/meshes/` is a build product and not in the repository:
`render.py` needs it, so run `takt.py` first.

After a change, copy `cad/viewer/model.json` and `cad/viewer/fk.js` to `docs/` (they must stay byte for byte the same),
pack the meshes for the website with `python3 tools/website/pack_meshes.py` and run the simulation checks again.

## Files

| File | What |
|---|---|
| `takt.py` | the CAD source |
| `parts/` | 12 parts as STEP and STL, in print orientation; `_x6` = print six |

`foot_sneaker_x6` is an alternative to `foot_x6`: print one of the two. Print `fit_gauge` and `fit_collar` first,
before the full set.
| `report.json` | masses, loads, clearances from `takt.py` |
| `blade120_report.json` | the height study of the 120 mm blade (read by `export_viewer.py`) |
| `render.py`, `renders/` | the pictures |
| `viewer/` | the data for the website's 3D view and its self-test |

Source location: https://github.com/TimFKiefer/how-to-vibecode-a-robot
