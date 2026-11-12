# SPDX-License-Identifier: MIT
"""Stage 1: contacts and the CAD mesh count. Headless. Exit 0 only when every row passes.

    cd <repository root> && sim/.venv/bin/python sim/checks/check_e1_body.py
"""

import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import mujoco  # noqa: E402

from _rows import run  # noqa: E402
from robot2sim.contacts import floor_force_n, on_floor, penetration_mm, self_hits  # noqa: E402
from robot2sim.kinematics import stand_pose  # noqa: E402
from robot2sim.paths import cad_layout, meshes_json, model_json, read_json, reference_json, robot2_py, root_dir  # noqa: E402
from robot2sim.scene import hold, load_model, load_spec, set_pose  # noqa: E402
from robot2sim.spec import iter_parts  # noqa: E402
from robot2sim.units import GRAVITY  # noqa: E402

VIS_GROUP = 2
COXA_TURN_DEG = 55.0
SWEEP_FROM_DEG, SWEEP_TO_DEG, SWEEP_STEP_DEG = 100.0, 130.0, 0.5
SWEEP_KNEE_DEG = 70.0
BODY_HIT_DEG, BODY_HIT_TOL_DEG = 121.0, 2.0
FOLD_FEMUR_DEG, FOLD_KNEE_DEG = -20.0, 155.0
HOLD_S = 1.0
FORCE_TOL = 0.02
PENETRATION_MAX_MM = 0.5

SPEC = load_spec("v0")
MODEL = load_model("v0")
DATA = mujoco.MjData(MODEL)
STAND_B, STAND_JOINTS = stand_pose(SPEC["hips"], SPEC["constants"])


def _stand_with(changes):
    """Stand pose with some leg angles replaced: changes maps a leg to {index: degrees}."""
    joints = {leg: list(angles) for leg, angles in STAND_JOINTS.items()}
    for leg, slots in changes.items():
        for index, angle in slots.items():
            joints[leg][index] = angle
    set_pose(MODEL, DATA, SPEC, STAND_B, joints)
    return self_hits(MODEL, DATA)


def row_cad_meshes():
    with_mesh, without = [], []
    for part in iter_parts(SPEC):
        (with_mesh if part["mesh"] else without).append(part["name"])
    bad = []
    for name in with_mesh:
        gid = mujoco.mj_name2id(MODEL, mujoco.mjtObj.mjOBJ_GEOM, f"{name}_vis")
        if gid < 0:
            bad.append(f"{name}_vis missing")
            continue
        mesh = int(MODEL.geom_dataid[gid])
        if int(MODEL.geom_type[gid]) != mujoco.mjtGeom.mjGEOM_MESH or int(MODEL.geom_group[gid]) != VIS_GROUP:
            bad.append(f"{name}_vis not a group-{VIS_GROUP} mesh")
        elif mesh < 0 or int(MODEL.mesh_vertnum[mesh]) < 1:
            bad.append(f"{name}_vis has no vertex")
    for gid in range(MODEL.ngeom):
        on_robot = int(MODEL.geom_bodyid[gid]) != 0
        if on_robot and int(MODEL.geom_group[gid]) == VIS_GROUP and int(MODEL.geom_type[gid]) != mujoco.mjtGeom.mjGEOM_MESH:
            bad.append(f"group-{VIS_GROUP} geom {mujoco.mj_id2name(MODEL, mujoco.mjtObj.mjOBJ_GEOM, gid)} is not a mesh")
    assert not bad, bad[:6]
    drawn = len(with_mesh) - len(bad)
    hardware = [n for n in without if "_hw_" in n]
    named = [n for n in without if "_hw_" not in n]
    return (
        f"{drawn} of {len(with_mesh)} parts with a mesh; {len(without)} without: "
        f"{', '.join(named)}, {len(hardware)} × {{leg}}_hw_{{coxa,femur,tibia}}"
    )


def row_cad_poses_clear():
    poses = [p for p in read_json(reference_json())["poses"] if p["variant"] == "claw"]
    hit = []
    for pose in poses:
        set_pose(MODEL, DATA, SPEC, pose["B"], pose["joints"])
        pairs = self_hits(MODEL, DATA)
        if pairs:
            hit.append(f"{pose['id']} {pairs[:2]}")
    assert len(poses) == 324, f"{len(poses)} claw poses"
    assert not hit, hit[:4]
    return f"{len(poses)} poses, no self_hit"


def row_legs_collide():
    both = _stand_with({"RF": {0: COXA_TURN_DEG}, "LF": {0: -COXA_TURN_DEG}})
    rf_alone = _stand_with({"RF": {0: COXA_TURN_DEG}})
    lf_alone = _stand_with({"LF": {0: -COXA_TURN_DEG}})
    cross = [p for p in both if {p[0][:3], p[1][:3]} == {"RF_", "LF_"}]
    assert cross, f"no RF/LF pair in {both}"
    assert rf_alone == [] and lf_alone == [], f"alone RF {rf_alone} LF {lf_alone}"
    return f"both: {len(cross)} RF/LF pairs, e.g. {cross[0]}; each alone: []"


def row_leg_hits_body():
    first = None
    steps = int(round((SWEEP_TO_DEG - SWEEP_FROM_DEG) / SWEEP_STEP_DEG))
    for i in range(steps + 1):
        femur = SWEEP_FROM_DEG + i * SWEEP_STEP_DEG
        pairs = _stand_with({"RF": {1: femur, 2: SWEEP_KNEE_DEG}})
        hit = [p for p in pairs if "body_hex" in p and any(n.startswith("RF_") for n in p)]
        if hit:
            first = (femur, hit[0])
            break
    assert first is not None, f"no body_hex/RF_ pair up to {SWEEP_TO_DEG}°"
    assert abs(first[0] - BODY_HIT_DEG) <= BODY_HIT_TOL_DEG, f"first at {first[0]:.1f}° {first[1]}"
    return f"first at {first[0]:.1f}° (expect {BODY_HIT_DEG} ±{BODY_HIT_TOL_DEG}) {first[1]}"


def row_folded_leg_reported():
    pairs = _stand_with({"RF": {1: FOLD_FEMUR_DEG, 2: FOLD_KNEE_DEG}})
    hit = [p for p in pairs if "RF_tibia_cap" in p]
    assert hit, f"no RF_tibia_cap pair in {pairs}"
    return f"{len(pairs)} pairs, with RF_tibia_cap: {hit}"


def row_floor_contact():
    set_pose(MODEL, DATA, SPEC, STAND_B, STAND_JOINTS)
    hold(MODEL, DATA, HOLD_S)
    touching = sorted(on_floor(MODEL, DATA))
    feet = sorted(f"{hip['name']}_foot_tip" for hip in SPEC["hips"])
    weight = float(MODEL.body_mass.sum()) * GRAVITY
    force = floor_force_n(MODEL, DATA)
    depth = penetration_mm(MODEL, DATA)
    err = abs(force - weight) / weight
    assert touching == feet, f"on floor {touching}"
    assert err <= FORCE_TOL, f"force {force:.3f} N vs weight {weight:.3f} N ({err * 100:.2f}%)"
    assert depth <= PENETRATION_MAX_MM, f"penetration {depth:.3f} mm"
    return f"six foot tips; force {force:.3f}/{weight:.3f} N ({err * 100:.2f}%); penetration {depth:.3f} mm"


def row_cad_layouts():
    """paths.cad_layout on empty files in a temp folder: older layout, repository layout, both, neither; then the
    four public path functions of the live tree equal what cad_layout returns for it."""
    older = ("robot2.py", "viewer/model.json", "viewer/reference.json", "viewer/meshes.json")
    repo = ("cad/takt.py", "cad/viewer/model.json", "cad/viewer/reference.json", "cad/viewer/meshes.json")
    with tempfile.TemporaryDirectory() as tmp:
        roots = {name: Path(tmp) / name for name in ("older", "repo", "both", "neither")}
        for name, files in (("older", older), ("repo", repo), ("both", older + repo)):
            for f in files:
                (roots[name] / f).parent.mkdir(parents=True, exist_ok=True)
                (roots[name] / f).touch()
        roots["neither"].mkdir()
        for name, script, viewer in (("older", "robot2.py", "viewer"), ("repo", "cad/takt.py", "cad/viewer"),
                                     ("both", "cad/takt.py", "cad/viewer")):
            got = cad_layout(roots[name])
            assert got == (roots[name] / script, roots[name] / viewer), f"{name}: {got}"
            assert all((got[1] / j).is_file() for j in ("model.json", "reference.json", "meshes.json")), name
        try:
            cad_layout(roots["neither"])
        except FileNotFoundError as e:
            tried = (roots["neither"] / "cad" / "takt.py", roots["neither"] / "robot2.py")
            assert all(str(t) in str(e) for t in tried), f"message {e}"
        else:
            raise AssertionError("neither layout: no FileNotFoundError")
    script, viewer = cad_layout(root_dir())
    live = (robot2_py(), model_json(), reference_json(), meshes_json())
    want = (script, viewer / "model.json", viewer / "reference.json", viewer / "meshes.json")
    assert live == want, f"the live tree's paths do not go through cad_layout: {live} != {want}"
    return (f"older: robot2.py + viewer/; repository: cad/takt.py + cad/viewer/; both: repository; "
            f"neither: FileNotFoundError naming both; live tree: the four path functions equal cad_layout")


if __name__ == "__main__":
    sys.exit(run([
        row_cad_meshes,
        row_cad_poses_clear,
        row_legs_collide,
        row_leg_hits_body,
        row_folded_leg_reported,
        row_floor_contact,
        row_cad_layouts,
    ]))
