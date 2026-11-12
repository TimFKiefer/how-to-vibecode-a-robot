# SPDX-License-Identifier: MIT
"""Step 1 check. Headless. Exit 0 only when every row passes.

    cd <repository root> && sim/.venv/bin/python sim/checks/check_s1_body.py
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import mujoco  # noqa: E402

from robot2sim.frames import mat16  # noqa: E402
from robot2sim.paths import reference_json, read_json  # noqa: E402
from robot2sim.scene import hold, load_model, load_spec, set_pose  # noqa: E402
from robot2sim.source import constants  # noqa: E402
from robot2sim.units import GRAVITY, deg_to_rad, kg_to_g, m_to_mm, nm_to_kgcm  # noqa: E402

MASS_TOL = 0.005
COM_TOL_MM = 2.0
POS_TOL_MM = 0.5
ROT_TOL_DEG = 0.1
RANGE_TOL_RAD = 1e-6
FORCE_TOL = 0.02
TORQUE_FRAC = 0.15
TORQUE_ABS_KGCM = 0.1
BELLY_MM = 62.0
BELLY_TOL_MM = 3.0
HOLD_S = 2.0


def _rot_deg(got, want):
    err = got.T @ want
    cosine = float(np.clip((np.trace(err) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def _row(ok, name, detail):
    print(f"{'PASS' if ok else 'FAIL'}  {name:<22} {detail}")
    return ok


def _ranges(model, raw, hips):
    bad = []
    for hip in hips:
        name = hip["name"]
        delta = hip["delta_deg"]
        expect = {
            "coxa": (delta - raw["COXA_RANGE"], delta + raw["COXA_RANGE"]),
            "femur": (raw["TH2_RANGE"][0], raw["TH2_RANGE"][1]),
            "knee": (raw["BETA_RANGE"][0], raw["BETA_RANGE"][1]),
        }
        for kind, (lo, hi) in expect.items():
            jname = f"{name}_{kind}"
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, jname)
            if jid < 0 or aid < 0:
                bad.append(f"missing {jname}")
                continue
            got = model.jnt_range[jid]
            want = np.array([deg_to_rad(lo), deg_to_rad(hi)])
            if int(model.jnt_limited[jid]) != 1 or np.max(np.abs(got - want)) > RANGE_TOL_RAD:
                bad.append(f"{jname} range {got} vs {want}")
    nu = int(model.nu)
    ok = nu == 18 and not bad
    detail = f"{nu} actuators" if ok else f"{nu} actuators; {bad[:4]}"
    return _row(ok, "1 actuators, ranges", detail)


def _fk(model, data, spec, poses):
    worst_pos, worst_rot = 0.0, 0.0
    worst_pos_at, worst_rot_at = "", ""
    missing = []
    n = 0
    for pose in poses:
        set_pose(model, data, spec, pose["B"], pose["joints"])
        n += 1
        for name, m in pose["placements"].items():
            sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
            if sid < 0:
                missing.append(name)
                continue
            matrix = mat16(m)
            pos = float(np.linalg.norm(np.array(data.site_xpos[sid]) * 1000.0 - matrix[:3, 3]))
            rot = _rot_deg(np.array(data.site_xmat[sid]).reshape(3, 3), matrix[:3, :3])
            if pos > worst_pos:
                worst_pos, worst_pos_at = pos, f"{pose['id']} {name}"
            if rot > worst_rot:
                worst_rot, worst_rot_at = rot, f"{pose['id']} {name}"
    ok = not missing and worst_pos <= POS_TOL_MM and worst_rot <= ROT_TOL_DEG
    detail = (
        f"{n} claw poses  pos {worst_pos:.4f} mm ({worst_pos_at})  "
        f"rot {worst_rot:.4f} deg ({worst_rot_at})"
    )
    if missing:
        detail += f"  missing sites {sorted(set(missing))[:6]}"
    return _row(ok, "2 FK vs FreeCAD", detail)


def _mass_com(model, data, spec, stand):
    """Mass within MASS_TOL of reference.json (the FreeCAD mass of this pose), CoM within COM_TOL_MM."""
    mass_g = kg_to_g(float(model.body_mass.sum()))
    set_pose(model, data, spec, stand["B"], stand["joints"])
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body")
    com = np.array(data.subtree_com[bid]) * 1000.0
    err = float(np.linalg.norm(com - np.array(stand["com"])))
    ref_mass = float(stand["mass"])
    near_ref = abs(mass_g - ref_mass) / ref_mass <= MASS_TOL
    ok = near_ref and err <= COM_TOL_MM
    detail = (
        f"mass {mass_g:.3f} g  reference.json {ref_mass:.3f} g "
        f"({abs(mass_g - ref_mass) / ref_mass * 100:.4f}%, {'ok' if near_ref else 'OUT'})  "
        f"CoM {err:.3f} mm (limit {COM_TOL_MM})"
    )
    return _row(ok, "3 mass and CoM", detail), mass_g


def _foot_forces(model, data, on):
    """Normal force per stance foot, and the sum over every contact."""
    per = {leg: 0.0 for leg in on}
    total = 0.0
    touch = []
    force = np.zeros(6)
    for i in range(data.ncon):
        mujoco.mj_contactForce(model, data, i, force)
        total += float(force[0])
        g1 = int(data.contact[i].geom1)
        g2 = int(data.contact[i].geom2)
        n1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g1) or ""
        n2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g2) or ""
        touch.append(n1 if n1 != "floor" else n2)
        for leg in on:
            if f"{leg}_foot_tip" in (n1, n2) and float(force[0]) > 0.0:
                per[leg] += float(force[0])
    return per, total, touch


def _statics(model, data, spec, frames, mass_g):
    saved = np.array(model.geom_friction, copy=True)
    model.geom_friction[:] = 0.0
    weight = (mass_g / 1000.0) * GRAVITY
    lines = []
    ok = True
    try:
        for pose in frames:
            set_pose(model, data, spec, pose["B"], pose["joints"])
            hold(model, data, HOLD_S)
            per, total, touch = _foot_forces(model, data, pose["on"])
            force_err = abs(total - weight) / weight
            missing = [leg for leg, f in per.items() if f <= 0.0]
            torque_bad = []
            worst = 0.0
            for leg in spec["hips"]:
                name = leg["name"]
                for kind, slot in (("femur", "femur"), ("knee", "knee")):
                    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_{kind}")
                    got = abs(nm_to_kgcm(float(data.actuator_force[aid])))
                    want = float(pose["support"]["per_leg"][name][slot])
                    tol = max(TORQUE_FRAC * want, TORQUE_ABS_KGCM)
                    err = abs(got - want)
                    worst = max(worst, err)
                    if err > tol:
                        torque_bad.append(f"{name}_{slot} {got:.3f} vs {want:.3f} (tol {tol:.3f})")
            frame_ok = force_err <= FORCE_TOL and not missing and not torque_bad
            ok = ok and frame_ok
            lines.append(
                f"    {pose['id']:<10} on {','.join(pose['on']):<12} "
                f"F {total:.3f}/{weight:.3f} N ({force_err * 100:.2f}%)  "
                f"torque worst {worst:.3f} kg·cm  feet {missing or 'all'}  "
                f"contacts {sorted(set(touch))}"
            )
            for item in torque_bad[:6]:
                lines.append(f"      {item}")
    finally:
        model.geom_friction[:] = saved
    detail = f"{len(frames)} frictionless frames, hold {HOLD_S:.0f} s"
    passed = _row(ok, "4 statics", detail)
    print("\n".join(lines))
    return passed


def _stand(model, data, spec, stand):
    set_pose(model, data, spec, stand["B"], stand["joints"])
    hold(model, data, HOLD_S)
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body")
    height = m_to_mm(float(data.xpos[bid][2]))
    ok = abs(height - BELLY_MM) <= BELLY_TOL_MM
    return _row(ok, "5 stand belly", f"{height:.2f} mm (target {BELLY_MM:.0f} ±{BELLY_TOL_MM:.0f})")


def main():
    spec = load_spec("v0")
    raw, _model = constants()
    ref = read_json(reference_json())
    claw = [p for p in ref["poses"] if p["variant"] == "claw"]
    stand = next(p for p in claw if p["id"] == "pose_stand")
    tripod = [p for p in claw if p["kind"] == "gait" and len(p["on"]) == 3]
    if len(tripod) < 3:
        print("FAIL  4 statics             fewer than 3 gait frames with 3 stance feet")
        return 1
    pick = [tripod[0], tripod[len(tripod) // 2], tripod[-1]]
    model = load_model("v0")
    data = mujoco.MjData(model)
    hips = [{"name": n, "delta_deg": float(d)} for n, _phi, d in raw["HIPS"]]
    results = [
        _ranges(model, raw, hips),
        _fk(model, data, spec, claw),
    ]
    mass_ok, mass_g = _mass_com(model, data, spec, stand)
    results.append(mass_ok)
    results.append(_statics(model, data, spec, pick, mass_g))
    results.append(_stand(model, data, spec, stand))
    print("PASS" if all(results) else "FAIL")
    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
