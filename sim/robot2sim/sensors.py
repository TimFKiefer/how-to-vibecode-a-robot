# SPDX-License-Identifier: MIT
"""The sensor list and nothing else: servo bus, IMU, outside eye, goal, wish, last command.

Read-only on MuJoCo state. The root comes from data.qpos / data.qvel / data.time, never from derived arrays, so a
caller that writes qpos without stepping still gets the pose it wrote. Time is counted in whole physics ticks
(round(data.time / timestep)), so the floating-point dust in data.time never moves an eye sample.
"""

import math
from fractions import Fraction

import numpy as np

import mujoco

from robot2sim import randomise

SENSOR_CHANNELS = {
    "servo_pos": 18, "servo_speed": 18, "servo_load": 18, "servo_voltage": 18, "imu_gravity": 3,
    "imu_gyro": 3, "eye": 4, "goal": 3, "wish": 4, "last_command": 18,
}
EYE = {"rate_hz": 30.0, "delay_s": 0.10, "sigma_xy_m": 0.003, "sigma_z_m": 0.003, "sigma_yaw_rad": 0.02}  # UNVERIFIED
IMU = {"sigma_gravity": 0.02, "sigma_gyro": 0.01}  # UNVERIFIED
NOISE_DRAW = 10  # standard normals per read: 4 eye (x, y, yaw, z), 3 gravity, 3 gyro
DOWN = np.array([0.0, 0.0, -1.0])  # gravity in the world frame
RATE_DENOMINATOR_MAX = 1000  # eye rate × timestep as an exact fraction (30 Hz × 2 ms = 3/50)


def wrap(a):
    """Angle or array to (−π, π]."""
    return math.pi - np.mod(math.pi - np.asarray(a, dtype=float), 2.0 * math.pi)


class Sensors:
    def __init__(self, world, seed: int):
        self.world, self.seed = world, seed
        m = world.model
        root = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "root")
        self._q = int(m.jnt_qposadr[root])
        self._v = int(m.jnt_dofadr[root])
        self._dt = float(m.opt.timestep)
        self.goal = self.wish = self.rng = None
        self._eye = self._pending = self._instant = self._rate = self._delay = None

    def _tick(self):
        return int(round(self.world.data.time / self._dt))

    def _instant_at(self, tick):
        """Index of the last 1/30 s instant at or before this tick, in exact integer arithmetic."""
        return tick * self._rate.numerator // self._rate.denominator

    def _delay_ticks(self):
        ticks = EYE["delay_s"] / self._dt
        assert abs(ticks - round(ticks)) < 1e-9, f"eye delay {EYE['delay_s']} s is not a whole number of ticks"
        return int(round(ticks))

    def _rotation(self):
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, self.world.data.qpos[self._q + 3:self._q + 7])
        return R.reshape(3, 3)

    def _measure_eye(self, noise):
        """(x, y, yaw, height) of the body origin now, plus noise."""
        q = self.world.data.qpos[self._q:self._q + 3]
        R = self._rotation()
        true = np.array([q[0], q[1], math.atan2(R[1, 0], R[0, 0]), q[2]])
        sigma = np.array([EYE["sigma_xy_m"], EYE["sigma_xy_m"], EYE["sigma_yaw_rad"], EYE["sigma_z_m"]])
        eye = true + noise * sigma
        eye[2] = wrap(eye[2])
        return eye

    def reset(self, goal: dict, wish: np.ndarray) -> None:
        """Call after world.reset: new noise stream, one eye measurement of the start pose, visible at once."""
        assert goal["kind"] in ("none", "point", "yaw"), f"goal kind {goal['kind']}"
        self.goal = dict(goal)
        self.wish = np.array(wish, dtype=float)
        assert self.wish.shape == (SENSOR_CHANNELS["wish"],), f"wish shape {self.wish.shape}"
        self.rng = randomise.rng(self.seed, "sensors")
        # Eye rate and delay in ticks, fixed per episode: worked out here once, not on every read.
        self._rate = (Fraction(EYE["rate_hz"]) * Fraction(self._dt)).limit_denominator(RATE_DENOMINATOR_MAX)
        self._delay = self._delay_ticks()
        noise = self.rng.standard_normal(NOISE_DRAW)
        self._eye = self._measure_eye(noise[:4])
        self._pending = []
        self._instant = self._instant_at(self._tick())

    def _update_eye(self, noise):
        tick = self._tick()
        instant = self._instant_at(tick)
        if instant > self._instant:
            self._pending.append((tick, self._measure_eye(noise)))
            self._instant = instant
        while self._pending and self._pending[0][0] + self._delay <= tick:
            self._eye = self._pending.pop(0)[1]

    def _goal(self):
        x, y, yaw, _h = self._eye
        kind = self.goal["kind"]
        if kind == "point":
            gx, gy = self.goal["xy_m"]
            dx, dy = gx - x, gy - y
            right = dx * math.cos(yaw) + dy * math.sin(yaw)
            front = -dx * math.sin(yaw) + dy * math.cos(yaw)
            return np.array([right, front, 0.0])
        if kind == "yaw":
            return np.array([0.0, 0.0, wrap(self.goal["yaw_rad"] - yaw)])
        return np.zeros(3)

    def read(self, last_command: np.ndarray) -> dict[str, np.ndarray]:
        d = self.world.data
        noise = self.rng.standard_normal(NOISE_DRAW)
        self._update_eye(noise[:4])
        R = self._rotation()
        gravity = R.T @ DOWN + noise[4:7] * IMU["sigma_gravity"]
        gyro = d.qvel[self._v + 3:self._v + 6] + noise[7:10] * IMU["sigma_gyro"]
        bus = self.world.bus.read()
        command = np.array(last_command, dtype=float)
        assert command.shape == (SENSOR_CHANNELS["last_command"],), f"last_command shape {command.shape}"
        out = {
            "servo_pos": bus["pos"], "servo_speed": bus["speed"], "servo_load": bus["load"],
            "servo_voltage": bus["voltage"],
            "imu_gravity": gravity / np.sqrt(gravity.dot(gravity)),  # np.linalg.norm's own sqrt(x·x)
            "imu_gyro": gyro,
            "eye": self._eye, "goal": self._goal(), "wish": self.wish, "last_command": command,
        }
        return {k: np.array(v, dtype=np.float32) for k, v in out.items()}
