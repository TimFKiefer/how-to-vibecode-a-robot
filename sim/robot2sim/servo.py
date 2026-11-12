# SPDX-License-Identifier: MIT
"""Servo table, the motor model on a loaded MjModel, and the bus (latency, encoder, overload cut-out).

Motor line: a position actuator clipped to ± stall plus joint damping stall / no-load speed, so at full
drive the torque falls linearly to zero at the no-load speed. Armature is compiled into the body file
(mjcf.ARMATURE); MuJoCo derives the contact softness from it at compile, so it is checked here, never written.
"""

import math

import numpy as np

from robot2sim.mjcf import ARMATURE
from robot2sim.units import kgcm_to_nm, rpm_to_rad_s

SERVOS = {
    "C018_12V": {"stall_kgcm": 30.0, "noload_rpm": 45.0, "volts": 12.0},  # UNVERIFIED
    "C001_6V": {"stall_kgcm": 16.5, "noload_rpm": 42.0, "volts": 6.0},  # UNVERIFIED
}
DEFAULT_SERVO = "C018_12V"

KP = 80.0  # N·m/rad  # UNVERIFIED
KV = 2.0  # N·m·s/rad  # UNVERIFIED
LIMP_DAMPING = 0.05  # N·m·s/rad, a cut servo back-driven through its gears  # UNVERIFIED
OVERLOAD_FRACTION = 0.8
OVERLOAD_S = 2.0
ENCODER_STEPS = 4096
SUBSTEPS = 10  # physics steps per control step (2 ms physics, 50 Hz control)


def _stall_nm(servo):
    return kgcm_to_nm(SERVOS[servo]["stall_kgcm"])


def _noload_rad_s(servo):
    return rpm_to_rad_s(SERVOS[servo]["noload_rpm"])


def _servo_dofs(model):
    assert model.nu == 18, f"{model.nu} actuators, expected 18"
    joints = model.actuator_trnid[:, 0]
    return model.jnt_dofadr[joints].astype(int), model.jnt_qposadr[joints].astype(int), joints


def apply_motor(model, servo: str, kp: float = KP, kv: float = KV) -> None:
    """Write the motor line on the 18 actuators and their dofs. Clears any cut-out of an earlier bus."""
    dofs, _, _ = _servo_dofs(model)
    armature = model.dof_armature[dofs]
    assert np.allclose(armature, ARMATURE, atol=1e-12, rtol=0), (
        f"compiled armature {armature.min()}…{armature.max()} ≠ mjcf.ARMATURE {ARMATURE}: rebuild the body file "
        "(python -m robot2sim.build); an in-memory write leaves the contact constants stale"
    )
    stall = _stall_nm(servo)
    model.actuator_gainprm[:, 0] = kp
    model.actuator_biasprm[:, 1] = -kp
    model.actuator_biasprm[:, 2] = -kv
    model.actuator_forcelimited[:] = 1
    model.actuator_forcerange[:] = [-stall, stall]
    model.dof_damping[dofs] = stall / _noload_rad_s(servo)


class Bus:
    """The servo bus of one episode. Built after apply_motor and after the reset pose is set."""

    def __init__(self, model, data, servo: str, latency_steps: int, offsets_rad: np.ndarray, noise: dict, rng):
        assert latency_steps >= 1, f"latency {latency_steps}"
        assert np.all(model.actuator_gainprm[:, 0] > 0), "a servo is still limp: call apply_motor before Bus"
        self.model, self.data, self.rng = model, data, rng
        self.dofs, self.qadr, joints = _servo_dofs(model)
        self.lo, self.hi = model.jnt_range[joints].T.copy()
        self.stall = _stall_nm(servo)
        self.volts = float(SERVOS[servo]["volts"])
        self.offsets = np.asarray(offsets_rad, dtype=float).copy()
        self.sigma = np.array([
            noise["encoder_sigma_rad"], noise["speed_sigma_rad_s"], noise["load_sigma_pct"] / 100.0,
            noise["voltage_sigma_v"],
        ])[:, None]
        reset = data.qpos[self.qadr] - self.offsets
        self.queue = [reset.copy() for _ in range(latency_steps)]
        self.overload_steps = int(round(OVERLOAD_S / (SUBSTEPS * float(model.opt.timestep))))
        self._overload_nm = OVERLOAD_FRACTION * self.stall  # per bus, not per step
        self._grid = 2.0 * math.pi / ENCODER_STEPS
        self.over = np.zeros(model.nu, dtype=int)
        self.cut = np.zeros(model.nu, dtype=bool)

    def command(self, targets_rad: np.ndarray) -> None:
        self.queue.append(np.clip(np.asarray(targets_rad, dtype=float), self.lo, self.hi))
        out = self.queue.pop(0) + self.offsets
        live = ~self.cut
        self.data.ctrl[live] = out[live]

    def after_step(self) -> None:
        force = np.abs(self.data.actuator_force)
        self.over = np.where(force > self._overload_nm, self.over + 1, 0)
        if self.over.max() < self.overload_steps:  # no servo has been overloaded long enough: nothing to cut
            return
        for a in np.flatnonzero((self.over >= self.overload_steps) & ~self.cut):
            self.cut[a] = True
            self.model.actuator_gainprm[a, 0] = 0.0
            self.model.actuator_biasprm[a, 1:3] = 0.0
            self.model.dof_damping[self.dofs[a]] = LIMP_DAMPING

    def read(self) -> dict[str, np.ndarray]:
        noise = self.rng.standard_normal((4, self.model.nu)) * self.sigma
        grid = self._grid
        pos = self.data.qpos[self.qadr] - self.offsets + noise[0]
        return {
            "pos": np.rint(pos / grid) * grid,  # np.round with 0 decimals is rint, without the wrapper
            "speed": self.data.qvel[self.dofs] + noise[1],
            "load": self.data.actuator_force / self.stall + noise[2],
            "voltage": self.volts + noise[3],
        }
