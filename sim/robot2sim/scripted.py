# SPDX-License-Identifier: MIT
"""The scripted rival: the CAD script's tripod gait, open loop, the first opponent every learned brain is measured
against.

It is no scored model: it uses IK and link lengths. The gait numbers come from the CAD script through `source`, so a
changed gait there changes the rival. At its first `act` it reads `obs["wish"]` once: a turn rate → turn, a forward
speed → walk, else it holds its first `servo_pos`. The walk and the turn run only at the three NAMED_HEIGHTS.

Time line from the first act (50 Hz): the stand pose of World.reset; tripod A moves from its stand spot to its gait
spot at phase 0 in TRANSITION_S (cosine blend, lifted TRANSITION_LIFT_MM × sin²); then tripod B the same; then the
gait from phase 0 with period PERIOD_S, forever. Feet are body-frame millimetres, B = T(0, 0, belly height).
"""

import math

import numpy as np

from robot2sim.brains import Brain, BodySpec
from robot2sim.frames import T
from robot2sim.kinematics import leg_ik
from robot2sim.scene import load_spec
from robot2sim.source import load_robot2_namespace, require
from robot2sim.units import deg_to_rad, m_to_mm

PERIOD_S = 2.0  # one gait cycle; not a knob
STRIDE_MM = 50.0  # the CAD script's gait_feet defaults
TURN_DEG = 16.0
TRANSITION_S = 0.6  # per tripod
TRANSITION_LIFT_MM = 15.0
CONTROL_DT_S = 0.02  # 50 Hz control
HEIGHT_TOL_MM = 0.5
MIRROR = {"RF": "LF", "RM": "LM", "RR": "LR", "LF": "RF", "LM": "RM", "LR": "RR"}  # x → −x

_GAIT = require(load_robot2_namespace(), "GAIT_R", "GAIT_LIFT", "GAIT_OUT", "TRI_A", "TRI_B", "NAMED_HEIGHTS")
TRI_A, TRI_B = tuple(_GAIT["TRI_A"]), tuple(_GAIT["TRI_B"])
NAMED_HEIGHTS = _GAIT["NAMED_HEIGHTS"]


def heights_mm() -> tuple[float, ...]:
    return tuple(float(e["hb"]) for e in NAMED_HEIGHTS.values())


def spots(r_mm: float, hips, c) -> dict[str, tuple]:
    """Foot rest points straight out from each hip at radius R_HIP + r_mm, z = R_TIP."""
    out = {}
    for hip in hips:
        a = math.radians(hip["phi_deg"])
        out[hip["name"]] = ((c["R_HIP"] + r_mm) * math.cos(a), (c["R_HIP"] + r_mm) * math.sin(a), c["R_TIP"])
    return out


def gait_feet(kind: str, p: float, base: dict, lift: float, out: float) -> dict[str, tuple]:
    """The CAD script's gait_feet with a continuous phase p in [0, 1). Tripod A at p, tripod B at p + 0.5."""
    feet = {}
    for name, (hx, hy, hz) in base.items():
        ph = (p + (0.0 if name in TRI_A else 0.5)) % 1.0
        if ph < 0.5:
            s, z = 0.5 - ph / 0.5, 0.0
        else:
            u = (ph - 0.5) / 0.5
            s, z = -0.5 + (0.5 - 0.5 * math.cos(math.pi * u)), lift * math.sin(math.pi * u) ** 2
            r = math.hypot(hx, hy)
            o = out * math.sin(math.pi * u) ** 0.7
            hx, hy = hx + hx / r * o, hy + hy / r * o
        if kind == "walk":
            feet[name] = (hx, hy + s * STRIDE_MM, hz + z)
        elif kind == "turn":
            a = math.radians(s * TURN_DEG)
            feet[name] = (hx * math.cos(a) - hy * math.sin(a), hx * math.sin(a) + hy * math.cos(a), hz + z)
        else:
            raise ValueError(f"gait kind {kind!r}; valid: walk, turn")
    return feet


def _named(belly_mm: float) -> dict:
    for entry in NAMED_HEIGHTS.values():
        if abs(belly_mm - entry["hb"]) <= HEIGHT_TOL_MM:
            return entry
    valid = ", ".join(f"{h:g}" for h in heights_mm())
    raise ValueError(f"scripted walks only at belly {valid} mm (±{HEIGHT_TOL_MM:g}); got {belly_mm:.2f} mm")


class Scripted(Brain):
    def __init__(self, body: str = "v0"):
        spec = load_spec(body)
        self.hips, self.c, self.joint_order = spec["hips"], spec["constants"], spec["joint_order"]
        assert tuple(self.joint_order) == ("coxa", "femur", "knee"), self.joint_order
        self.transition_steps = int(round(TRANSITION_S / CONTROL_DT_S))
        self.period_steps = int(round(PERIOD_S / CONTROL_DT_S))

    def reset(self, body: BodySpec, task: str, seed: int) -> None:
        self.body = body
        self.steps = 0
        self.mode = None
        self.hold = None
        self.ik_failures = 0

    def _start(self, obs: dict) -> None:
        wish = np.array(obs["wish"], dtype=float)
        if wish[2] != 0.0:
            self.mode, self.kind, self.mirror = "gait", "turn", wish[2] < 0.0
        elif wish[0] > 0.0:
            self.mode, self.kind, self.mirror = "gait", "walk", False
        else:
            self.mode = "hold"
            self.hold = np.array(obs["servo_pos"], dtype=float)
            return
        entry = _named(m_to_mm(float(wish[3])))
        self.B = T(0, 0, entry["hb"])
        self.stand = spots(self.c["STAND_R"], self.hips, self.c)
        self.base = spots(entry["gait_r"], self.hips, self.c)
        self.lift, self.out = entry["lift"], entry["out"]
        self.first = self._feet_of_gait(0.0)
        self.last = {}

    def _feet_of_gait(self, p: float) -> dict:
        feet = gait_feet(self.kind, p, self.base, self.lift, self.out)
        if self.mirror:
            feet = {MIRROR[n]: (-x, y, z) for n, (x, y, z) in feet.items()}
        return feet

    def _blend(self, name: str, k: int) -> tuple:
        u = k / self.transition_steps
        w = 0.5 - 0.5 * math.cos(math.pi * u)
        a, b = self.stand[name], self.first[name]
        lift = TRANSITION_LIFT_MM * math.sin(math.pi * u) ** 2
        return (a[0] + w * (b[0] - a[0]), a[1] + w * (b[1] - a[1]), a[2] + w * (b[2] - a[2]) + lift)

    def _feet(self, k: int) -> dict:
        n = self.transition_steps
        if k < n:
            return {h: self._blend(h, k) if h in TRI_A else self.stand[h] for h in self.stand}
        if k < 2 * n:
            return {h: self.first[h] if h in TRI_A else self._blend(h, k - n) for h in self.stand}
        return self._feet_of_gait(((k - 2 * n) % self.period_steps) / self.period_steps)

    def act(self, obs: dict) -> np.ndarray:
        if self.mode is None:
            self._start(obs)
        k = self.steps
        self.steps += 1
        if self.mode == "hold":
            return self.hold.copy()
        feet = self._feet(k)
        angles = []
        for hip in self.hips:
            name = hip["name"]
            try:
                self.last[name] = leg_ik(hip, feet[name], self.B, (self.c["L_TIBIA"], 0.0), self.c)
            except ValueError:
                if name not in self.last:
                    raise
                self.ik_failures += 1
            angles.extend(self.last[name])
        return deg_to_rad(np.array(angles, dtype=float))

    def info(self) -> dict:
        return {"ik_failures": self.ik_failures}
