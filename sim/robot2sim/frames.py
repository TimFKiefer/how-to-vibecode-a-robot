# SPDX-License-Identifier: MIT
"""Rigid frames, the same convention as viewer/fk.js (FreeCAD Matrix.A).

A 4×4 matrix is row-major. Applying it to a point is M · (x, y, z, 1).
chain(A, B, C) = A @ B @ C, so the first frame is closest to the world.
"""

import math

import numpy as np


def eye():
    return np.eye(4)


def T(x=0.0, y=0.0, z=0.0):
    m = np.eye(4)
    m[0, 3], m[1, 3], m[2, 3] = x, y, z
    return m


def Rx(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    m = np.eye(4)
    m[1, 1], m[1, 2] = c, -s
    m[2, 1], m[2, 2] = s, c
    return m


def Rz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    m = np.eye(4)
    m[0, 0], m[0, 1] = c, -s
    m[1, 0], m[1, 1] = s, c
    return m


def chain(*ms):
    out = np.eye(4)
    for m in ms:
        out = out @ m
    return out


def apply(m, v):
    p = np.array([v[0], v[1], v[2], 1.0], dtype=float)
    return (m @ p)[:3]


def mat16(values):
    return np.array(values, dtype=float).reshape(4, 4)


def as16(m):
    return [float(x) for x in np.asarray(m, dtype=float).reshape(-1)]


def translation(m):
    return np.array([m[0, 3], m[1, 3], m[2, 3]], dtype=float)


def rotation(m):
    return np.array(m[:3, :3], dtype=float)


def mat_to_quat(rot):
    """Rotation matrix to MuJoCo quaternion (w, x, y, z)."""
    m = np.asarray(rot, dtype=float)
    t = float(np.trace(m))
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=float)
    n = np.linalg.norm(q)
    if n == 0.0:
        raise ValueError("rotation matrix is not a rotation")
    q /= n
    if q[0] < 0.0:
        q = -q
    return q


def quat_of(m):
    return mat_to_quat(rotation(m))


def leg_frames(phi, delta, gamma, th2, beta, c, B=None):
    """World frames keyed the way viewer/fk.js legFrames is. For the step 2–4 mass code."""
    if B is None:
        B = eye()
    coxa = chain(B, Rz(phi), T(c["R_HIP"], 0, 0), Rz(gamma))
    fa = chain(coxa, T(c["L_COXA"], 0, c["Z_F"]), Rx(-90))
    femur = chain(fa, Rz(-th2))
    tibia = chain(femur, T(c["L_FEMUR"], 0, 0), Rz(beta))
    s1 = chain(B, Rz(phi), T(c["R_HIP"], 0, 0), Rz(delta), T(0, 0, c["Z_S1"]))
    s2 = chain(fa, T(0, 0, -c["S_MID"]), Rz(-90))
    s3 = chain(tibia, Rz(c["S3_DIR"]), T(0, 0, -c["S_MID"]))
    return {"s1": s1, "coxa": coxa, "s2": s2, "femur": femur, "s3": s3, "tibia": tibia}


def part_in_link(phi, delta, frame, c):
    """Part frame expressed in its link frame."""
    if frame == "body":
        return eye()
    kind = frame.split(":")[2]
    hip = {"phi_deg": phi, "delta_deg": delta}
    if kind == "s1":
        return chain(Rz(phi), T(c["R_HIP"], 0, 0), Rz(delta), T(0, 0, c["Z_S1"]))
    if kind in ("coxa", "femur", "tibia"):
        return eye()
    if kind == "s2":
        return chain(T(c["L_COXA"], 0, c["Z_F"]), Rx(-90), T(0, 0, -c["S_MID"]), Rz(-90))
    if kind == "s3":
        return chain(Rz(c["S3_DIR"]), T(0, 0, -c["S_MID"]))
    raise ValueError(frame)


def frame_name_to_link(frame):
    """(link key used by masses.py, part kind). s1 rides on the trunk."""
    if frame == "body" or frame.endswith(":s1"):
        return "trunk", "body" if frame == "body" else "s1"
    _tag, leg, kind = frame.split(":")
    if kind in ("coxa", "s2"):
        return f"{leg}_coxa", kind
    if kind == "femur":
        return f"{leg}_femur", kind
    if kind in ("s3", "tibia"):
        return f"{leg}_tibia", kind
    raise ValueError(frame)
