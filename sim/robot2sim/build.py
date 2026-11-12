# SPDX-License-Identifier: MIT
"""Write bodies/v0.json, the visual meshes and bodies/v0.xml.

Run from sim/:  python -m robot2sim.build
"""

import json

import numpy as np

from robot2sim.collision import fit_capsules
from robot2sim.frames import apply, mat16
from robot2sim.kinematics import leg_frames
from robot2sim.meshes import centroids_mm, write_hex, write_used
from robot2sim.mjcf import write_xml
from robot2sim.paths import body_json, reference_json, read_json
from robot2sim.scene import load_model, set_pose
from robot2sim.spec import force_point_masses, iter_parts, make_spec
from robot2sim.units import kg_to_g


def _world_com(spec, B, joints):
    acc = np.zeros(3)
    mass = spec["body"]["mass_g"]
    acc += mass * apply(B, spec["body"]["com_mm"])
    c = spec["constants"]
    for hip in spec["hips"]:
        frames = leg_frames(B, hip, joints[hip["name"]], c)
        for link, frame in frames.items():
            block = spec["legs"][hip["name"]][link]
            acc += block["mass_g"] * apply(frame, block["com_mm"])
            mass += block["mass_g"]
    return float(mass), acc / mass


def _check_reference(spec):
    ref = read_json(reference_json())
    stand = next(p for p in ref["poses"] if p["id"] == "pose_stand")
    B = mat16(spec["poses"]["stand"]["B"])
    joints = spec["poses"]["stand"]["joints_deg"]
    mass, com = _world_com(spec, B, joints)
    mass_err = abs(mass - stand["mass"])
    com_err = float(np.linalg.norm(com - np.array(stand["com"])))
    joint_err = 0.0
    for leg, want in stand["joints"].items():
        got = joints[leg]
        joint_err = max(joint_err, max(abs(a - b) for a, b in zip(got, want)))
    link_mass = spec["body"]["mass_g"] + sum(
        spec["legs"][h["name"]][link]["mass_g"]
        for h in spec["hips"] for link in ("coxa", "femur", "tibia")
    )
    print(f"analytical mass {mass:.6f} g  reference {stand['mass']:.6f} g  err {mass_err:.3e} g")
    print(f"analytical CoM err {com_err:.3e} mm   stand joint err {joint_err:.3e} deg")
    print(f"link masses sum {link_mass:.6f} g  declared {spec['mass_g']['total']:.6f} g")
    if mass_err > 1e-3 or com_err > 1e-3 or joint_err > 1e-3 or abs(link_mass - spec["mass_g"]["total"]) > 1e-3:
        raise SystemExit("body file does not match viewer/reference.json stand pose")
    # An earlier design figure is 1944.6 g of parts and 2092.6 g total. The live export is
    # the source of the numbers; a mismatch is reported, not patched.
    if abs(spec["mass_g"]["parts"] - 1944.6) > 0.1 or abs(spec["mass_g"]["total"] - 2092.6) > 0.1:
        print(
            f"NOTE an earlier design figure is parts 1944.6 g and total 2092.6 g; "
            f"model.json parts are {spec['mass_g']['parts']:.4f} g and with lumps "
            f"and leg hardware the total is {spec['mass_g']['total']:.4f} g"
        )


def _mujoco_com_mm(spec):
    import mujoco
    model = load_model(spec["name"])
    data = mujoco.MjData(model)
    pose = spec["poses"]["stand"]
    set_pose(model, data, spec, pose["B"], pose["joints_deg"])
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body")
    com = np.array(data.subtree_com[bid]) * 1000.0
    ref = read_json(reference_json())
    stand = next(p for p in ref["poses"] if p["id"] == "pose_stand")
    err = float(np.linalg.norm(com - np.array(stand["com"])))
    mass_g = kg_to_g(float(model.body_mass.sum()))
    print(f"MuJoCo mass {mass_g:.4f} g  stand CoM err {err:.4f} mm  nq {model.nq} nu {model.nu}")
    return err, mass_g, model.nq, model.nu


def _write(spec):
    path = body_json(spec["name"])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(spec, indent=2) + "\n", encoding="utf-8")
    keys = sorted({p["mesh"] for p in iter_parts(spec) if p["mesh"]})
    write_used(keys)
    hx = spec["collision"]["hex"]
    write_hex(hx["circumradius_mm"], hx["z0_mm"], hx["z1_mm"])
    write_xml(spec, path.with_suffix(".xml"))
    print(f"wrote {path}")
    print(f"wrote {path.with_suffix('.xml')}")


def main():
    used = set()
    # Centroids for every render key the claw instances can ask for.
    from robot2sim.source import load_model as load_mj
    model = load_mj()
    for inst in model["instances"]:
        if inst["variant"] in (None, "claw") and inst["render"]:
            used.add(inst["key"])
    spec = make_spec(centroids_mm(used))
    _check_reference(spec)
    fit_capsules(spec)
    col = spec["collision"]
    print(
        f"capsules r {col['capsule_radius_mm']} mm  "
        f"stand/gait clearance {col['min_clearance_stand_gait_mm']:.2f} mm at {col['min_clearance_where']}"
    )
    _write(spec)
    err, _mass, nq, nu = _mujoco_com_mm(spec)
    if nq != 25 or nu != 18:
        raise SystemExit(f"expected 25 qpos and 18 actuators, got nq {nq} nu {nu}")
    if err > 0.5:
        print(f"mesh CoM is {err:.3f} mm off; moving every mass onto its authored CoM")
        force_point_masses(spec)
        _write(spec)
        err, _mass, nq, nu = _mujoco_com_mm(spec)
        if err > 0.5:
            raise SystemExit(f"MuJoCo CoM still {err:.3f} mm from the reference")
    print("build ok")


if __name__ == "__main__":
    main()
