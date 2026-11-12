# SPDX-License-Identifier: MIT
"""The four stage-1 tasks: start pose, goal, wish, time limit, and `judge` as a pure function of one truth dict.

No simulation here. A truth dict has the keys of World.truth(). Forward is body +Y: at yaw ψ the forward direction in
the world is (−sin ψ, cos ψ). Ledge tasks are not built in stage 1.
"""

import math
from dataclasses import dataclass

import numpy as np

from robot2sim.randomise import rng
from robot2sim.sensors import wrap
from robot2sim.units import deg_to_rad, mm_to_m

TASKS = ("stand_up", "walk_1m", "turn_90", "push_walk")


@dataclass(frozen=True)
class TaskSpec:
    name: str
    limit_s: float
    pose: str
    goal_kind: str
    walking: bool


SPECS = {
    "stand_up": TaskSpec("stand_up", 30.0, "sit", "none", False),
    "walk_1m": TaskSpec("walk_1m", 60.0, "stand", "point", True),
    "turn_90": TaskSpec("turn_90", 30.0, "stand", "yaw", True),
    "push_walk": TaskSpec("push_walk", 60.0, "stand", "point", True),
}
STAND_BELLY_M = 0.062
POINT_DISTANCE_M = 1.0
TURN_RAD = deg_to_rad(90.0)
WALK_MPS = 0.05  # the fixed gait: 100 mm per 2 s cycle
TURN_RAD_S = 0.279  # the fixed gait: 32° per 2 s cycle
PUSH_N = 3.0
PUSH_S = 0.2
PUSH_AT_S = (2.0, 15.0)
FALL_TILT_RAD = deg_to_rad(45.0)
POINT_TOL_M = 0.10
YAW_TOL_RAD = deg_to_rad(10.0)
UPRIGHT_BELLY_M = 0.055
UPRIGHT_TILT_RAD = deg_to_rad(10.0)
HOLD_S = 2.0
CONTROL_DT_S = 0.02  # one judge call per control step
HOLD_TOL_S = CONTROL_DT_S / 2.0  # float sums of 0.02 s: the 100th call reaches 2.0 s within half a step


def spec(name: str) -> TaskSpec:
    if name not in SPECS:
        raise KeyError(f"unknown task {name!r}; valid: {', '.join(TASKS)}")
    return SPECS[name]


def start(name: str, seed: int, belly_mm: float | None = None) -> dict:
    """What World.reset and World.push need: pose, belly height, and the seeded push of push_walk (else None)."""
    sp = spec(name)
    belly_m = STAND_BELLY_M if belly_mm is None else float(mm_to_m(belly_mm))
    push = None
    if name == "push_walk":
        r = rng(seed, "task")
        at_s = float(r.uniform(*PUSH_AT_S))
        sign = 1.0 if r.integers(0, 2) else -1.0
        push = {"at_s": at_s, "fx_n": sign * PUSH_N, "fy_n": 0.0, "seconds": PUSH_S}
    return {"pose": sp.pose, "belly_m": belly_m, "push": push}


def goal(name: str, truth0: dict) -> dict:
    """The goal argument of Sensors.reset, from the start truth: the body origin plus 1 m forward, or yaw + 90°."""
    kind = spec(name).goal_kind
    yaw = float(truth0["yaw_rad"])
    if kind == "point":
        x, y = float(truth0["pos_m"][0]), float(truth0["pos_m"][1])
        return {"kind": "point", "xy_m": (x - POINT_DISTANCE_M * math.sin(yaw), y + POINT_DISTANCE_M * math.cos(yaw))}
    if kind == "yaw":
        return {"kind": "yaw", "yaw_rad": float(wrap(yaw + TURN_RAD))}
    return {"kind": "none"}


def wish(name: str, belly_m: float) -> np.ndarray:
    """(forward m/s, sideways m/s, turn rad/s, belly height m)."""
    kind = spec(name).goal_kind
    if kind == "point":
        return np.array([WALK_MPS, 0.0, 0.0, belly_m])
    if kind == "yaw":
        return np.array([0.0, 0.0, TURN_RAD_S, belly_m])
    return np.array([0.0, 0.0, 0.0, STAND_BELLY_M])


def judge(spec: TaskSpec, goal: dict, truth: dict, held_s: float) -> tuple[str | None, float]:
    """(reason or None, new held_s). First match wins: cutout, fall, success, timeout."""
    if np.any(truth["cut"]):
        return "cutout", held_s
    if truth["tilt_rad"] > FALL_TILT_RAD:
        return "fall", held_s
    if spec.walking and truth["body_on_floor"]:
        return "fall", held_s
    if spec.goal_kind == "point":
        dx = truth["com_m"][0] - goal["xy_m"][0]
        dy = truth["com_m"][1] - goal["xy_m"][1]
        if math.hypot(dx, dy) <= POINT_TOL_M:
            return "success", held_s
    elif spec.goal_kind == "yaw":
        level_on_heading = (
            abs(float(wrap(goal["yaw_rad"] - truth["yaw_rad"]))) < YAW_TOL_RAD
            and abs(truth["roll_rad"]) < UPRIGHT_TILT_RAD
            and abs(truth["pitch_rad"]) < UPRIGHT_TILT_RAD
        )
        held_s = held_s + CONTROL_DT_S if level_on_heading else 0.0
        if held_s >= HOLD_S - HOLD_TOL_S:
            return "success", held_s
    else:
        upright = (
            truth["belly_m"] >= UPRIGHT_BELLY_M
            and abs(truth["roll_rad"]) < UPRIGHT_TILT_RAD
            and abs(truth["pitch_rad"]) < UPRIGHT_TILT_RAD
        )
        held_s = held_s + CONTROL_DT_S if upright else 0.0
        if held_s >= HOLD_S - HOLD_TOL_S:
            return "success", held_s
    if truth["time_s"] >= spec.limit_s:
        return "timeout", held_s
    return None, held_s
