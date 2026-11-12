# SPDX-License-Identifier: MIT
"""Load a generated body and put it in a pose."""

import numpy as np

import mujoco

from robot2sim.frames import mat16, quat_of, translation
from robot2sim.paths import body_json, body_xml, read_json
from robot2sim.units import deg_to_rad, mm_to_m


def load_spec(name):
    return read_json(body_json(name))


def load_model(name):
    return mujoco.MjModel.from_xml_path(str(body_xml(name)))


def _joint_names(spec):
    names = []
    for hip in spec["hips"]:
        for kind in ("coxa", "femur", "knee"):
            names.append(f"{hip['name']}_{kind}")
    return names


def set_pose(model, data, spec, B, joints_deg):
    """B is a 16-list or 4×4 in millimetres. joints_deg maps a leg to (γ, θ2, β)."""
    mujoco.mj_resetData(model, data)
    matrix = B if isinstance(B, np.ndarray) and B.shape == (4, 4) else mat16(B)
    root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    adr = int(model.jnt_qposadr[root])
    data.qpos[adr:adr + 3] = [mm_to_m(v) for v in translation(matrix)]
    data.qpos[adr + 3:adr + 7] = quat_of(matrix)
    for name in _joint_names(spec):
        leg, kind = name.split("_", 1)
        # knee is the third angle; split keeps RF_coxa as leg RF, kind coxa.
        angle = joints_deg[leg][{"coxa": 0, "femur": 1, "knee": 2}[kind]]
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        qadr = int(model.jnt_qposadr[jid])
        data.qpos[qadr] = deg_to_rad(angle)
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if aid >= 0:
            data.ctrl[aid] = deg_to_rad(angle)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


def hold(model, data, seconds):
    steps = int(round(seconds / float(model.opt.timestep)))
    for _ in range(steps):
        mujoco.mj_step(model, data)
    return steps
