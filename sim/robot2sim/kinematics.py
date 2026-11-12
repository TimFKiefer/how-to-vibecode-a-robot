# SPDX-License-Identifier: MIT
"""Forward kinematics of the four links, matching viewer/fk.js legFrames.

Link frames, as a function of the body frame B and the CAD script's joint angles
(degrees):

    coxa  = B · Rz(phi) · T(R_HIP, 0, 0) · Rz(gamma)
    femur = coxa · T(L_COXA, 0, Z_F) · Rx(-90) · Rz(-th2)
    tibia = femur · T(L_FEMUR, 0, 0) · Rz(beta)

s1 is fixed on the body: Rz(phi) · T(R_HIP, 0, 0) · Rz(delta) · T(0, 0, Z_S1).
s2 is fixed on the coxa. s3 is fixed on the tibia. Joint coordinates in the
sim are gamma, th2 and beta themselves.
"""

import math

import numpy as np

from robot2sim.frames import Rx, Rz, T, apply, chain, eye, quat_of, translation


def hips_of(raw_hips):
    return [{"name": n, "phi_deg": float(phi), "delta_deg": float(delta)} for n, phi, delta in raw_hips]


def part_frame(kind, hip, c):
    """Part frame in its link frame. Identity when the part frame is the link."""
    if kind in ("body", "coxa", "femur", "tibia"):
        return eye()
    if kind == "s1":
        return chain(Rz(hip["phi_deg"]), T(c["R_HIP"], 0, 0), Rz(hip["delta_deg"]), T(0, 0, c["Z_S1"]))
    if kind == "s2":
        return chain(T(c["L_COXA"], 0, c["Z_F"]), Rx(-90), T(0, 0, -c["S_MID"]), Rz(-90))
    if kind == "s3":
        return chain(Rz(c["S3_DIR"]), T(0, 0, -c["S_MID"]))
    raise ValueError(kind)


def link_of(frame):
    """Which moving link an instance frame rides on. s1 is on the body."""
    if frame == "body":
        return "body", None, "body"
    _leg, name, kind = frame.split(":")
    if kind == "s1":
        return "body", name, "s1"
    if kind in ("coxa", "s2"):
        return "coxa", name, kind
    if kind == "femur":
        return "femur", name, "femur"
    if kind in ("s3", "tibia"):
        return "tibia", name, kind
    raise ValueError(frame)


def leg_frames(B, hip, angles, c):
    """World frames of one leg. angles is (gamma, th2, beta) in degrees."""
    gamma, th2, beta = angles
    coxa = chain(B, Rz(hip["phi_deg"]), T(c["R_HIP"], 0, 0), Rz(gamma))
    femur = chain(coxa, T(c["L_COXA"], 0, c["Z_F"]), Rx(-90), Rz(-th2))
    tibia = chain(femur, T(c["L_FEMUR"], 0, 0), Rz(beta))
    return {"coxa": coxa, "femur": femur, "tibia": tibia}


def leg_ik(hip, foot, B, contact, c):
    """The CAD script's leg_ik. foot and contact are millimetres. Returns degrees."""
    H = chain(B, Rz(hip["phi_deg"]), T(c["R_HIP"], 0, 0))
    p = apply(np.linalg.inv(H), foot)
    gamma = math.degrees(math.atan2(p[1], p[0]))
    r = math.hypot(float(p[0]), float(p[1]))
    dx, dz = r - c["L_COXA"], float(p[2]) - c["Z_F"]
    reach = math.hypot(dx, dz)
    length = math.hypot(contact[0], contact[1])
    off = math.degrees(math.atan2(contact[1], contact[0]))
    cos_a = (c["L_FEMUR"] ** 2 + reach ** 2 - length ** 2) / (2 * c["L_FEMUR"] * reach)
    if abs(cos_a) > 1.0:
        raise ValueError(f"{hip['name']} out of reach")
    th2 = math.degrees(math.atan2(dz, dx) + math.acos(cos_a))
    kx = c["L_COXA"] + c["L_FEMUR"] * math.cos(math.radians(th2))
    kz = c["Z_F"] + c["L_FEMUR"] * math.sin(math.radians(th2))
    tib = math.degrees(math.atan2(float(p[2]) - kz, r - kx))
    return [gamma, th2, th2 - tib - off]


def stand_pose(hips, c):
    """Level body at STAND_HB, feet on spots(STAND_R)."""
    B = T(0, 0, c["STAND_HB"])
    joints = {}
    for hip in hips:
        a = math.radians(hip["phi_deg"])
        radius = c["R_HIP"] + c["STAND_R"]
        foot = (radius * math.cos(a), radius * math.sin(a), c["R_TIP"])
        joints[hip["name"]] = leg_ik(hip, foot, B, (c["L_TIBIA"], 0.0), c)
    return B, joints


def frame_pose(m):
    return [float(v) for v in translation(m)], [float(v) for v in quat_of(m)]
