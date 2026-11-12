# SPDX-License-Identifier: MIT
"""Paths inside this checkout. Anchored on this file, never a machine path.

The CAD sits beside `sim/` in one of two layouts, decided once by which CAD script exists:
repository layout `cad/takt.py` with `cad/viewer/`, or the older layout `robot2.py` with `viewer/`.
When both exist the repository layout wins.
"""

import functools
import json
from pathlib import Path


def sim_dir():
    return Path(__file__).resolve().parents[1]


def root_dir():
    return sim_dir().parent


def cad_layout(parent):
    """(CAD script, viewer folder) under `parent`; FileNotFoundError naming both scripts tried when neither exists."""
    parent = Path(parent)
    repo_script, older_script = parent / "cad" / "takt.py", parent / "robot2.py"
    if repo_script.exists():
        return repo_script, parent / "cad" / "viewer"
    if older_script.exists():
        return older_script, parent / "viewer"
    raise FileNotFoundError(f"no CAD script: tried {repo_script} and {older_script}")


@functools.cache
def _layout():
    return cad_layout(root_dir())


def robot2_py():
    return _layout()[0]


def model_json():
    return _layout()[1] / "model.json"


def reference_json():
    return _layout()[1] / "reference.json"


def meshes_json():
    return _layout()[1] / "meshes.json"


def body_json(name):
    return sim_dir() / "bodies" / f"{name}.json"


def body_xml(name):
    return sim_dir() / "bodies" / f"{name}.xml"


def mesh_dir():
    return sim_dir() / "assets" / "meshes"


def read_json(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


# Names the step 2–4 modules already import.
SIM = sim_dir()
ROOT = root_dir()
RUNS = SIM / "runs"


def __getattr__(name):
    """MODEL_JSON, REFERENCE_JSON, ROBOT2_PY: resolved on first use, so importing this module never needs the CAD."""
    lazy = {"MODEL_JSON": model_json, "REFERENCE_JSON": reference_json, "ROBOT2_PY": robot2_py}
    if name in lazy:
        return lazy[name]()
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def body_path(name="v0"):
    return body_json(name)
