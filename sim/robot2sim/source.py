# SPDX-License-Identifier: MIT
"""Read the CAD script's constants without importing it (it imports FreeCAD)."""

import ast
import math

from robot2sim.paths import model_json, read_json, robot2_py


def _names(node):
    if isinstance(node, ast.Name):
        return [node.id]
    if isinstance(node, (ast.Tuple, ast.List)):
        out = []
        for elt in node.elts:
            out.extend(_names(elt))
        return out
    return []


def load_robot2_namespace():
    """Evaluate module-level assignments that do not need FreeCAD."""
    text = robot2_py().read_text(encoding="utf-8")
    tree = ast.parse(text)
    ns = {"math": math, "V": lambda x, y, z: (float(x), float(y), float(z))}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if not all(_names(target) for target in node.targets):
            continue
        src = ast.get_source_segment(text, node)
        if not src:
            continue
        try:
            exec(src, ns)  # noqa: S102 — assignments copied from the CAD script
        except Exception:
            continue
    return ns


def require(ns, *names):
    missing = [n for n in names if n not in ns]
    if missing:
        raise RuntimeError(f"the CAD script did not yield {missing}")
    return {n: ns[n] for n in names}


def load_model():
    return read_json(model_json())


def constants():
    """The kinematic and mass constants the body file is built from."""
    ns = load_robot2_namespace()
    names = (
        "R_HIP", "L_COXA", "Z_F", "L_FEMUR", "L_TIBIA", "R_TIP",
        "S_MID", "Z_S1", "S3_DIR", "COXA_RANGE", "TH2_RANGE", "BETA_RANGE",
        "HIPS", "STAND_HB", "STAND_R", "R_V", "APO", "WALL", "LID_TOP",
        "BASE_TOP", "T_BLADE", "CAM_Z", "CAM_Y", "LEG_HARDWARE", "LUMPS_BODY",
        "STALL", "RATED",
    )
    raw = require(ns, *names)
    model = load_model()
    c = model["C"]
    # Cross-check the two read-only sources. A mismatch means one of them moved.
    for key in ("R_HIP", "L_COXA", "Z_F", "L_FEMUR", "L_TIBIA", "R_TIP", "S_MID", "Z_S1", "S3_DIR", "COXA_RANGE", "STAND_HB", "STAND_R", "LEG_HARDWARE"):
        if float(raw[key]) != float(c[key]):
            raise RuntimeError(f"{key}: the CAD script {raw[key]} vs model.json {c[key]}")
    if list(raw["TH2_RANGE"]) != list(c["TH2_RANGE"]) or list(raw["BETA_RANGE"]) != list(c["BETA_RANGE"]):
        raise RuntimeError("joint ranges differ between the CAD script and model.json")
    if [list(h) for h in raw["HIPS"]] != [list(h) for h in c["HIPS"]]:
        raise RuntimeError("HIPS differ between the CAD script and model.json")
    return raw, model
