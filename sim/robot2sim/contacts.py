# SPDX-License-Identifier: MIT
"""What touches what: read data.contact as arrays. Read-only on data."""

import numpy as np

import mujoco

from robot2sim.units import m_to_mm

ROBOT_GROUP = 3


def robot_geoms(model):
    """Ids of the robot's collision geoms (group 3)."""
    return np.flatnonzero(model.geom_group == ROBOT_GROUP)


def _floor_id(model):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")


def _name(model, gid):
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(gid))


def _pairs(data):
    n = int(data.ncon)
    return data.contact.geom[:n], data.contact.dist[:n]


def _floor_rows(model, data):
    """Contact indices robot vs floor, and the robot geom of each."""
    geom, _dist = _pairs(data)
    floor = _floor_id(model)
    robot = np.isin(geom, robot_geoms(model))
    rows = np.flatnonzero((geom == floor).any(axis=1) & robot.any(axis=1))
    other = np.where(geom[rows, 0] == floor, geom[rows, 1], geom[rows, 0])
    return rows, other


def scan(data, robot, floor):
    """One pass over data.contact for the step path, in plain Python (a handful of contacts per step):
    (robot geom id of every robot-vs-floor contact, deepest robot-vs-floor penetration in mm, any robot-vs-robot
    overlap). `robot` is a frozenset of robot_geoms(model), `floor` the floor geom id, both looked up once by the
    caller. The same values as on_floor (as ids), penetration_mm and bool(self_hits)."""
    n = int(data.ncon)
    touching = []
    deepest = None
    hit = False
    for (g1, g2), dist in zip(data.contact.geom[:n].tolist(), data.contact.dist[:n].tolist()):
        if g1 == floor and g2 in robot:
            touching.append(g2)
        elif g2 == floor and g1 in robot:
            touching.append(g1)
        else:
            hit = hit or (g1 in robot and g2 in robot and dist < 0.0)
            continue
        deepest = dist if deepest is None else min(deepest, dist)
    depth = 0.0 if deepest is None else max(0.0, m_to_mm(-deepest))
    return touching, depth, hit


def self_hits(model, data):
    """Name pairs of robot geoms that overlap (dist < 0), each pair once."""
    geom, dist = _pairs(data)
    robot = np.isin(geom, robot_geoms(model)).all(axis=1)
    hits = []
    for g1, g2 in geom[robot & (dist < 0.0)]:
        pair = (_name(model, g1), _name(model, g2))
        if pair not in hits:
            hits.append(pair)
    return hits


def on_floor(model, data):
    """Names of the robot geoms that touch the floor, each once."""
    _rows, other = _floor_rows(model, data)
    return [_name(model, gid) for gid in dict.fromkeys(other.tolist())]


def penetration_mm(model, data):
    """Deepest robot-vs-floor contact in mm, positive; 0.0 without one."""
    rows, _other = _floor_rows(model, data)
    if rows.size == 0:
        return 0.0
    return max(0.0, m_to_mm(-float(data.contact.dist[rows].min())))


def floor_force_n(model, data):
    """Sum of the normal forces robot vs floor, N. For rows and summaries, not the step path."""
    rows, _other = _floor_rows(model, data)
    force = np.zeros(6)
    total = 0.0
    for i in rows:
        mujoco.mj_contactForce(model, data, int(i), force)
        total += float(force[0])
    return total
