# SPDX-License-Identifier: MIT
"""Capsule shells that stay off the floor in the standing poses.

The radius is half the blade thickness (T_BLADE), a CAD dimension: the capsule
is a round stand-in for the link, not a scanned solid. Ends are inset, and the
tibia capsule stops short of the foot, because the tip centre is only R_TIP
above the floor and a full-length capsule would cut the ground.
"""

import numpy as np

from robot2sim.frames import apply, mat16
from robot2sim.kinematics import leg_frames
from robot2sim.paths import read_json, reference_json

# How far the capsule surface must stay above z = 0 in the poses we simulate.
# Not a measured clearance.
FLOOR_CLEARANCE_MM = 2.0  # UNVERIFIED


def _segment(p0, p1):
    return np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)


def _inset(p0, p1, inset):
    d = p1 - p0
    n = float(np.linalg.norm(d))
    if n <= 2.0 * inset + 2.0:
        mid = (p0 + p1) / 2.0
        u = d / max(n, 1e-9)
        half = max(2.0, n * 0.2)
        return mid - u * half, mid + u * half
    u = d / n
    return p0 + u * inset, p1 - u * inset


def _min_z(world, p0, p1, radius):
    lowest = 1e9
    for t in np.linspace(0.0, 1.0, 17):
        z = float(apply(world, p0 + t * (p1 - p0))[2])
        if z < lowest:
            lowest = z
    return lowest - radius


def _poses(spec):
    """Standing claw poses: the stand, every gait frame, and other poses whose belly is up."""
    ref = read_json(reference_json())
    required, extra = [], []
    for pose in ref["poses"]:
        if pose["variant"] != "claw":
            continue
        B = mat16(pose["B"])
        item = (pose["id"], B, pose["joints"])
        if pose["kind"] == "gait" or pose["id"] == "pose_stand":
            required.append(item)
        elif float(B[2, 3]) >= 30.0 and pose["id"] != "pose_sit":
            extra.append(item)
    return required, extra


def _clearance(spec, poses, capsules, radius):
    worst = 1e9
    worst_id = None
    c = spec["constants"]
    for pid, B, joints in poses:
        for hip in spec["hips"]:
            frames = leg_frames(B, hip, joints[hip["name"]], c)
            for link, (p0, p1) in capsules.items():
                z = _min_z(frames[link], p0, p1, radius)
                if z < worst:
                    worst, worst_id = z, f"{pid} {hip['name']} {link}"
    return worst, worst_id


def fit_capsules(spec):
    c = spec["constants"]
    radius = c["T_BLADE"] / 2.0
    required, extra = _poses(spec)
    coxa_far = np.array([c["L_COXA"], 0.0, c["Z_F"]])
    femur_far = np.array([c["L_FEMUR"], 0.0, 0.0])
    tibia_far = np.array([c["L_TIBIA"], 0.0, 0.0])
    origin = np.zeros(3)

    def pack(radius, tibia_to):
        return {
            "coxa": _inset(origin, coxa_far, radius),
            "femur": _inset(origin, femur_far, radius),
            "tibia": _inset(origin, np.array([tibia_to, 0.0, 0.0]), min(radius, tibia_to * 0.25)),
        }

    chosen = None
    for radius in (c["T_BLADE"] / 2.0, 6.0, 5.0, 4.0):
        for tibia_to in np.arange(c["L_TIBIA"] - c["R_TIP"] - radius, 30.0, -2.0):
            capsules = pack(float(radius), float(tibia_to))
            clearance, where = _clearance(spec, required, capsules, radius)
            if clearance >= FLOOR_CLEARANCE_MM:
                chosen = (float(radius), capsules, clearance, where)
                break
        if chosen:
            break
    if chosen is None:
        raise RuntimeError("no capsule shell stays off the floor in the standing poses")
    radius, capsules, clearance, where = chosen
    extra_clearance, extra_where = _clearance(spec, extra, capsules, radius) if extra else (None, None)
    spec["collision"] = {
        "capsule_radius_mm": radius,
        "floor_clearance_mm": FLOOR_CLEARANCE_MM,
        "min_clearance_stand_gait_mm": clearance,
        "min_clearance_where": where,
        "min_clearance_other_mm": extra_clearance,
        "min_clearance_other_where": extra_where,
        "capsules_mm": {link: {"from": [float(v) for v in p0], "to": [float(v) for v in p1]} for link, (p0, p1) in capsules.items()},
        "hex": {"z0_mm": c["BELLY"][2], "z1_mm": c["LID_TOP"], "circumradius_mm": c["R_V"]},
        "foot_radius_mm": c["R_TIP"],
    }
    return spec
