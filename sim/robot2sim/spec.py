# SPDX-License-Identifier: MIT
"""Build the V0 body description from the CAD script and viewer/model.json.

Mass is the model.json parts (physics × claw instances) plus LUMPS_BODY on the
body and LEG_HARDWARE/3 at each of the coxa, femur and tibia origins. The
rounded model.json numbers.mass.total (2088 g) is not used.
"""

import numpy as np

from robot2sim.frames import apply, eye
from robot2sim.kinematics import frame_pose, hips_of, link_of, part_frame, stand_pose
from robot2sim.source import constants


# Mesh volume-centroid may replace the authored CoM only inside this band.
# The lid mesh (grille) is further from lid_proxy than this, so its mass is a
# point at the proxy CoM instead.
MESH_COM_TOL_MM = 0.05

# Radius of the invisible sphere that carries a mass with no usable mesh.
# The sphere centre is the authored CoM; the radius is not a part size.
POINT_MASS_R_MM = 1.0  # UNVERIFIED


def _f(x):
    return float(x)


def _vec(v):
    return [_f(v[0]), _f(v[1]), _f(v[2])]


def _part(name, mesh, mass_g, com_link, frame, mass_on_mesh, rgba, site=True):
    pos, quat = frame_pose(frame)
    return {
        "name": name,
        "mesh": mesh,
        "mass_g": _f(mass_g),
        "com_mm": _vec(com_link),
        "pos_mm": pos,
        "quat": quat,
        "mass_on_mesh": bool(mass_on_mesh and mesh and mass_g > 0),
        "rgba": rgba,
        "site": site,
    }


def _rgba(model, key):
    if key is None:
        return [0.2, 0.2, 0.22, 1.0]
    name = model["keycolor"].get(key, "black")
    rgb = model["colors"].get(name, [40, 41, 45])
    alpha = 0.45 if name == "glass" else 1.0
    return [rgb[0] / 255.0, rgb[1] / 255.0, rgb[2] / 255.0, alpha]


def _combine(parts):
    mass = sum(p["mass_g"] for p in parts)
    if mass <= 0:
        return 0.0, [0.0, 0.0, 0.0]
    acc = np.zeros(3)
    for p in parts:
        acc += p["mass_g"] * np.array(p["com_mm"])
    return _f(mass), _vec(acc / mass)


def make_spec(centroids=None):
    """centroids: mesh key -> volume centroid in the part frame, mm, or None."""
    raw, model = constants()
    c = {
        "R_HIP": _f(raw["R_HIP"]),
        "L_COXA": _f(raw["L_COXA"]),
        "Z_F": _f(raw["Z_F"]),
        "L_FEMUR": _f(raw["L_FEMUR"]),
        "L_TIBIA": _f(raw["L_TIBIA"]),
        "R_TIP": _f(raw["R_TIP"]),
        "S_MID": _f(raw["S_MID"]),
        "Z_S1": _f(raw["Z_S1"]),
        "S3_DIR": _f(raw["S3_DIR"]),
        "COXA_RANGE": _f(raw["COXA_RANGE"]),
        "TH2_RANGE": [_f(raw["TH2_RANGE"][0]), _f(raw["TH2_RANGE"][1])],
        "BETA_RANGE": [_f(raw["BETA_RANGE"][0]), _f(raw["BETA_RANGE"][1])],
        "STAND_HB": _f(raw["STAND_HB"]),
        "STAND_R": _f(raw["STAND_R"]),
        "R_V": _f(raw["R_V"]),
        "APO": _f(raw["APO"]),
        "WALL": _f(raw["WALL"]),
        "LID_TOP": _f(raw["LID_TOP"]),
        "BASE_TOP": _f(raw["BASE_TOP"]),
        "T_BLADE": _f(raw["T_BLADE"]),
        "CAM_Z": _f(raw["CAM_Z"]),
        "LEG_HARDWARE": _f(raw["LEG_HARDWARE"]),
        "BELLY": [_f(v) for v in model["C"]["BELLY"]],
        "LUMPS_BODY": [[n, _vec(p), _f(m)] for n, p, m in raw["LUMPS_BODY"]],
        "STALL_KGCM": _f(raw["STALL"][6.0]),
        "RATED_KGCM": _f(raw["RATED"][6.0]),
    }
    hips = hips_of(raw["HIPS"])
    by_name = {h["name"]: h for h in hips}
    phys = model["physics"]
    centroids = centroids or {}
    groups = {"body": []}
    for hip in hips:
        groups[hip["name"]] = {"coxa": [], "femur": [], "tibia": []}

    parts_mass = 0.0
    for inst in model["instances"]:
        if inst["variant"] not in (None, "claw"):
            continue
        link, leg, kind = link_of(inst["frame"])
        hip = by_name[leg] if leg else None
        frame = part_frame(kind, hip, c) if kind != "body" else eye()
        # Body-frame parts (kind "body") sit at the body origin.
        if kind == "body":
            frame = eye()
        mass = 0.0
        com_local = [0.0, 0.0, 0.0]
        if inst["phys"]:
            entry = phys[inst["phys"]]
            mass = _f(entry["mass"])
            com_local = entry["c"]
            parts_mass += mass
        com_link = apply(frame, com_local)
        mesh = inst["key"] if inst["render"] else None
        use_mesh_mass = False
        if mesh and mass > 0 and mesh in centroids:
            err = float(np.linalg.norm(np.array(centroids[mesh]) - np.array(com_local)))
            use_mesh_mass = err <= MESH_COM_TOL_MM
        part = _part(inst["name"], mesh, mass, com_link, frame, use_mesh_mass, _rgba(model, inst["key"]))
        if link == "body":
            groups["body"].append(part)
        else:
            groups[leg][link].append(part)

    hw = c["LEG_HARDWARE"] / 3.0
    for hip in hips:
        for link in ("coxa", "femur", "tibia"):
            groups[hip["name"]][link].append(_part(
                f"{hip['name']}_hw_{link}", None, hw, [0.0, 0.0, 0.0], eye(), False, [0, 0, 0, 0], site=False,
            ))
    lump_mass = 0.0
    for name, pos, mass in c["LUMPS_BODY"]:
        lump_mass += mass
        groups["body"].append(_part(name, None, mass, pos, T_at(pos), False, [0, 0, 0, 0], site=False))

    body_mass, body_com = _combine(groups["body"])
    legs = {}
    for hip in hips:
        legs[hip["name"]] = {}
        for link in ("coxa", "femur", "tibia"):
            mass, com = _combine(groups[hip["name"]][link])
            legs[hip["name"]][link] = {"mass_g": mass, "com_mm": com, "parts": groups[hip["name"]][link]}

    hw_total = 6.0 * c["LEG_HARDWARE"]
    total = parts_mass + lump_mass + hw_total
    B, joints = stand_pose(hips, c)
    spec = {
        "name": "v0",
        "variant": "claw",
        "units": "mm, deg, g — MuJoCo converts these in robot2sim.units",
        "servo": "STS3215",
        "joints_per_leg": 3,
        "joint_order": ["coxa", "femur", "knee"],
        "constants": c,
        "hips": hips,
        "foot_contact_mm": [c["L_TIBIA"], 0.0, 0.0],
        "camera": {
            "pos_mm": [0.0, c["APO"], c["CAM_Z"]],
            "look": [0.0, 1.0, 0.0],
            "up": [0.0, 0.0, 1.0],
        },
        "mass_g": {
            "parts": parts_mass,
            "lumps_body": lump_mass,
            "leg_hardware": hw_total,
            "total": total,
        },
        "body": {"mass_g": body_mass, "com_mm": body_com, "parts": groups["body"]},
        "legs": legs,
        "poses": {
            "stand": {"B": [float(x) for x in B.reshape(-1)], "joints_deg": {n: [_f(a) for a in ang] for n, ang in joints.items()}},
        },
        "point_mass_radius_mm": POINT_MASS_R_MM,
    }
    return spec


def T_at(pos):
    from robot2sim.frames import T
    return T(pos[0], pos[1], pos[2])


def iter_parts(spec):
    yield from spec["body"]["parts"]
    for leg in spec["legs"].values():
        for link in leg.values():
            yield from link["parts"]


def force_point_masses(spec):
    """Put every gram on a sphere at the authored CoM. Visual meshes stay."""
    for part in iter_parts(spec):
        part["mass_on_mesh"] = False
    return spec
