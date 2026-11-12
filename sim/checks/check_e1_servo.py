# SPDX-License-Identifier: MIT
"""Stage 1: the servo model and the bus. Headless. Exit 0 only when every row passes.

    cd <repository root> && sim/.venv/bin/python sim/checks/check_e1_servo.py

Rig: its own model, gravity 0, the root lifted 0.3 m and held by damping 1e4 on the six root dofs.
The joint under test is RM_coxa (range ±40°); outside loads go in through data.qfrc_applied.
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import mujoco  # noqa: E402

from _rows import run  # noqa: E402
from robot2sim.kinematics import stand_pose  # noqa: E402
from robot2sim.scene import load_model, load_spec, set_pose  # noqa: E402
from robot2sim.servo import (  # noqa: E402
    ARMATURE, DEFAULT_SERVO, ENCODER_STEPS, LIMP_DAMPING, OVERLOAD_FRACTION, OVERLOAD_S, SERVOS, SUBSTEPS, Bus,
    apply_motor,
)
from robot2sim.units import kgcm_to_nm  # noqa: E402

# The chosen servo table and its derived numbers, typed here so the check does not trust servo.SERVOS.
BRIEF_SERVOS = {
    "C018_12V": {"stall_kgcm": 30.0, "noload_rpm": 45.0, "volts": 12.0},
    "C001_6V": {"stall_kgcm": 16.5, "noload_rpm": 42.0, "volts": 6.0},
}
BRIEF_DERIVED = {  # stall N·m, no-load rad/s, damping N·m·s/rad
    "C018_12V": (2.941995, 4.712389, 0.624310),
    "C001_6V": (1.618097, 4.398230, 0.367897),
}
LITERAL_TOL = 1e-6

ROOT_LIFT_M = 0.3
ROOT_DAMPING = 1e4
JOINT = "RM_coxa"
STEP_FROM_DEG, STEP_TO_DEG = -40.0, 40.0
FREE_SPIN_S = 1.0
FREE_SPIN_BAND = (0.90, 1.00)
STALL_LOAD = 1.5
STALL_S, STALL_SETTLED_S = 1.0, 0.5
STALL_TOL = 0.02
TORQUE_SPEED_LOAD = 0.5
TORQUE_SPEED_S = 1.5
TORQUE_SPEED_TOL = 0.10
LATENCIES = (1, 2, 3)
LATENCY_COMMANDS = 12
OUT_OF_RANGE_RAD = 0.3
ABOVE_AT, BELOW_AT = 4, 7  # commands sent beyond the joint limits
OFFSET_DEG = 1.0
ENCODER_SAMPLES = 200
CUT_LOAD, CUT_WINDOW_S = 0.85, (2.00, 2.10)
SAFE_LOAD, SAFE_S = 0.70, 5.0
PULSE_ON_S, PULSE_OFF_S = 1.9, 0.1
FORCE_ZERO = 1e-12
AFTER_CUT_NUDGE_RAD = 0.2
ZERO_NOISE = {"encoder_sigma_rad": 0.0, "speed_sigma_rad_s": 0.0, "load_sigma_pct": 0.0, "voltage_sigma_v": 0.0}

SPEC = load_spec("v0")
MODEL = load_model("v0")
MODEL.opt.gravity[:] = 0.0  # gravity and dof_damping feed no mj_setConst constant
ROOT_DOF = int(MODEL.jnt_dofadr[mujoco.mj_name2id(MODEL, mujoco.mjtObj.mjOBJ_JOINT, "root")])
MODEL.dof_damping[ROOT_DOF:ROOT_DOF + 6] = ROOT_DAMPING
DATA = mujoco.MjData(MODEL)
STAND_B, STAND_JOINTS = stand_pose(SPEC["hips"], SPEC["constants"])
JID = mujoco.mj_name2id(MODEL, mujoco.mjtObj.mjOBJ_JOINT, JOINT)
AID = mujoco.mj_name2id(MODEL, mujoco.mjtObj.mjOBJ_ACTUATOR, JOINT)
QADR, DADR = int(MODEL.jnt_qposadr[JID]), int(MODEL.jnt_dofadr[JID])
SERVO_DOFS = [int(MODEL.jnt_dofadr[j]) for j in MODEL.actuator_trnid[:, 0]]
SERVO_QADR = [int(MODEL.jnt_qposadr[j]) for j in MODEL.actuator_trnid[:, 0]]
CONTROL_DT = SUBSTEPS * float(MODEL.opt.timestep)


def stall_nm(servo):
    return BRIEF_DERIVED[servo][0]


def noload_rad_s(servo):
    return BRIEF_DERIVED[servo][1]


def _rig(coxa_deg, latency=1, offsets=None, servo=DEFAULT_SERVO):
    """Stand pose with RM_coxa at coxa_deg, lifted, motor applied, a fresh bus."""
    apply_motor(MODEL, servo)
    joints = {leg: list(angles) for leg, angles in STAND_JOINTS.items()}
    joints["RM"][0] = coxa_deg
    set_pose(MODEL, DATA, SPEC, STAND_B, joints)
    DATA.qpos[2] += ROOT_LIFT_M
    offsets = np.zeros(MODEL.nu) if offsets is None else offsets
    DATA.qpos[SERVO_QADR] += offsets
    mujoco.mj_forward(MODEL, DATA)
    bus = Bus(MODEL, DATA, servo, latency, offsets, ZERO_NOISE, np.random.default_rng(0))
    return bus, DATA.qpos[SERVO_QADR] - offsets


def _control_step(bus, targets, on_physics_step=None):
    bus.command(targets)
    for _ in range(SUBSTEPS):
        mujoco.mj_step(MODEL, DATA)
        if on_physics_step:
            on_physics_step()
    bus.after_step()


def row_motor_line():
    assert SERVOS == BRIEF_SERVOS, f"servo table {SERVOS}"
    for name, servo in BRIEF_SERVOS.items():  # the literals follow from the table
        stall, w0, damping = BRIEF_DERIVED[name]
        assert abs(kgcm_to_nm(servo["stall_kgcm"]) - stall) <= LITERAL_TOL, f"{name} stall literal"
        assert abs(servo["noload_rpm"] * 2 * np.pi / 60 - w0) <= LITERAL_TOL, f"{name} no-load literal"
        assert abs(stall / w0 - damping) <= LITERAL_TOL, f"{name} damping literal"
    out = []
    for name in BRIEF_SERVOS:
        apply_motor(MODEL, name)
        stall, w0, want_damping = BRIEF_DERIVED[name]
        assert np.all(MODEL.actuator_forcelimited == 1), "force limit off"
        forcerange = MODEL.actuator_forcerange
        assert np.allclose(forcerange, [[-stall, stall]] * MODEL.nu, atol=LITERAL_TOL, rtol=0), f"force range {forcerange[0]}"
        damping = MODEL.dof_damping[SERVO_DOFS]
        assert np.allclose(damping, want_damping, atol=LITERAL_TOL, rtol=0), f"damping {damping[:3]}"
        armature = MODEL.dof_armature[SERVO_DOFS]
        assert np.allclose(armature, ARMATURE, atol=1e-12, rtol=0), f"armature {armature[:3]}"
        out.append(f"{name} ±{stall:.6f} N·m, damping {damping[0]:.6f} = {stall:.6f}/{w0:.6f}")
    return "; ".join(out) + f"; armature {ARMATURE} on 18 dofs"


def row_free_spin():
    bus, targets = _rig(STEP_FROM_DEG)
    targets[AID] = np.radians(STEP_TO_DEG)
    speeds = []
    for _ in range(int(round(FREE_SPIN_S / CONTROL_DT))):
        _control_step(bus, targets, lambda: speeds.append(float(DATA.qvel[DADR])))
    ratio = max(speeds) / noload_rad_s(DEFAULT_SERVO)
    assert FREE_SPIN_BAND[0] <= ratio <= FREE_SPIN_BAND[1], f"peak {max(speeds):.4f} rad/s = {ratio:.4f} × no load"
    return f"peak {max(speeds):.4f} rad/s = {ratio:.4f} × no load; end {np.degrees(DATA.qpos[QADR]):.2f}°"


def row_stall():
    bus, targets = _rig(0.0)
    stall = stall_nm(DEFAULT_SERVO)
    forces = []
    DATA.qfrc_applied[DADR] = STALL_LOAD * stall
    for _ in range(int(round(STALL_S / CONTROL_DT))):
        _control_step(bus, targets, lambda: forces.append(abs(float(DATA.actuator_force[AID]))))
    DATA.qfrc_applied[DADR] = 0.0
    settled = forces[-int(round(STALL_SETTLED_S / float(MODEL.opt.timestep))):]
    lo, hi = min(settled) / stall, max(settled) / stall
    assert max(forces) <= stall * (1 + 1e-12), f"peak {max(forces):.6f} N·m above stall {stall:.6f}"
    assert 1 - STALL_TOL <= lo and hi <= 1 + STALL_TOL, f"settled {lo:.4f}…{hi:.4f} × stall"
    assert not bus.cut[AID], "cut inside the 1 s stall"
    return f"settled |force| {lo:.4f}…{hi:.4f} × stall, peak {max(forces):.6f} ≤ {stall:.6f} N·m"


def row_torque_speed():
    bus, targets = _rig(STEP_FROM_DEG)
    targets[AID] = np.radians(STEP_TO_DEG)
    DATA.qfrc_applied[DADR] = -TORQUE_SPEED_LOAD * stall_nm(DEFAULT_SERVO)
    lo_q = np.radians(STEP_FROM_DEG + (STEP_TO_DEG - STEP_FROM_DEG) / 3)
    hi_q = np.radians(STEP_TO_DEG - (STEP_TO_DEG - STEP_FROM_DEG) / 3)
    middle = []

    def sample():
        if lo_q <= DATA.qpos[QADR] <= hi_q:
            middle.append(float(DATA.qvel[DADR]))

    for _ in range(int(round(TORQUE_SPEED_S / CONTROL_DT))):
        _control_step(bus, targets, sample)
    DATA.qfrc_applied[DADR] = 0.0
    assert middle, f"never reached the middle third, end {np.degrees(DATA.qpos[QADR]):.2f}°"
    want = 0.5 * noload_rad_s(DEFAULT_SERVO)
    lo, hi = min(middle) / want, max(middle) / want
    assert 1 - TORQUE_SPEED_TOL <= lo and hi <= 1 + TORQUE_SPEED_TOL, f"middle third {lo:.4f}…{hi:.4f} × 0.5 no load"
    return f"middle third {min(middle):.4f}…{max(middle):.4f} rad/s = {lo:.4f}…{hi:.4f} × 0.5 no load ({len(middle)} samples)"


def row_latency():
    rng = np.random.default_rng(3)
    offsets = np.radians(rng.uniform(-OFFSET_DEG, OFFSET_DEG, MODEL.nu))
    out = []
    for latency in LATENCIES:
        bus, reset = _rig(0.0, latency, offsets)
        lo, hi = MODEL.jnt_range[MODEL.actuator_trnid[:, 0]].T
        sent = [reset + np.radians(rng.uniform(-5, 5, MODEL.nu)) for _ in range(LATENCY_COMMANDS)]
        sent[ABOVE_AT] = hi + OUT_OF_RANGE_RAD
        sent[BELOW_AT] = lo - OUT_OF_RANGE_RAD
        expected = [np.clip(target, lo, hi) for target in sent]
        for t, target in enumerate(sent, 1):
            bus.command(target)
            if t > latency:
                assert np.array_equal(DATA.ctrl, expected[t - 1 - latency] + offsets), f"L={latency} command {t}"
            else:
                assert np.allclose(DATA.ctrl, reset + offsets, atol=1e-15, rtol=0), f"L={latency} fill {t}"
        out.append(f"L={latency} exact")
    return (
        ", ".join(out) + f" over {LATENCY_COMMANDS} commands, offsets ±{OFFSET_DEG}°, "
        f"2 commands {OUT_OF_RANGE_RAD} rad beyond the limits arrive clipped"
    )


def row_encoder():
    rng = np.random.default_rng(5)
    offsets = np.radians(rng.uniform(-OFFSET_DEG, OFFSET_DEG, MODEL.nu))
    assert np.all(offsets != 0)
    bus, targets = _rig(STEP_FROM_DEG, 1, offsets)
    targets[AID] = np.radians(STEP_TO_DEG)
    grid = 2 * np.pi / ENCODER_STEPS
    worst_off_grid = worst_err = 0.0
    for _ in range(ENCODER_SAMPLES):
        _control_step(bus, targets)
        pos = bus.read()["pos"]
        steps = pos / grid
        worst_off_grid = max(worst_off_grid, float(np.max(np.abs(steps - np.round(steps)))))
        worst_err = max(worst_err, float(np.max(np.abs(pos - (DATA.qpos[SERVO_QADR] - offsets)))))
    assert worst_off_grid < 1e-9, f"off grid by {worst_off_grid:.3g} steps"
    assert worst_err <= grid / 2 + 1e-12, f"error {worst_err:.3g} rad > half step {grid / 2:.3g}"
    return f"{ENCODER_SAMPLES} reads × 18: off grid ≤ {worst_off_grid:.1e} steps, |pos − (q − offset)| ≤ {worst_err:.6f} ≤ {grid / 2:.6f} rad"


def _run_load(schedule):
    """schedule: list of (seconds, load × stall). Returns the cut time (s) or None, the |force| after it, the bus
    and the targets it held."""
    bus, targets = _rig(0.0)
    stall = stall_nm(DEFAULT_SERVO)
    t, cut_at, after = 0.0, None, []
    for seconds, load in schedule:
        DATA.qfrc_applied[DADR] = load * stall
        for _ in range(int(round(seconds / CONTROL_DT))):
            _control_step(bus, targets)
            t += CONTROL_DT
            if cut_at is not None:
                after.append(abs(float(DATA.actuator_force[AID])))
            elif bus.cut[AID]:
                cut_at = t
    DATA.qfrc_applied[DADR] = 0.0
    assert not np.any(np.delete(bus.cut, AID)), "another servo cut"
    return cut_at, after, bus, targets


def row_cutout():
    cut_at, after, bus, targets = _run_load([(SAFE_S, CUT_LOAD)])
    assert cut_at is not None, f"{CUT_LOAD} × stall: no cut in {SAFE_S} s"
    assert CUT_WINDOW_S[0] - 1e-9 <= cut_at <= CUT_WINDOW_S[1] + 1e-9, f"{CUT_LOAD} × stall: cut at {cut_at:.3f} s"
    assert after and max(after) <= FORCE_ZERO, f"force after cut {max(after):.3g} N·m"
    gains = (MODEL.actuator_gainprm[AID, 0], *MODEL.actuator_biasprm[AID, 1:3])
    assert all(g == 0.0 for g in gains), f"cut gains {gains}"
    assert MODEL.dof_damping[DADR] == LIMP_DAMPING, f"cut damping {MODEL.dof_damping[DADR]}"
    held = float(DATA.ctrl[AID])
    nudged = targets.copy()
    nudged[AID] += AFTER_CUT_NUDGE_RAD
    for _ in range(3):  # past the latency queue
        bus.command(nudged)
    assert DATA.ctrl[AID] == held, f"cut servo took a command: ctrl {held} → {DATA.ctrl[AID]}"
    safe_at, *_ = _run_load([(SAFE_S, SAFE_LOAD)])
    assert safe_at is None, f"{SAFE_LOAD} × stall: cut at {safe_at:.3f} s"
    pulse_at, *_ = _run_load([(PULSE_ON_S, CUT_LOAD), (PULSE_OFF_S, 0.0), (PULSE_ON_S, CUT_LOAD)])
    assert pulse_at is None, f"1.9/0.1/1.9 s: cut at {pulse_at:.3f} s"
    return (
        f"{CUT_LOAD} × stall: cut at {cut_at:.2f} s (> {OVERLOAD_FRACTION} × stall for {OVERLOAD_S} s), "
        f"force after ≤ {max(after):.1g}, gains 0, damping {LIMP_DAMPING}, ctrl held under a new command; {SAFE_LOAD} × stall: none in {SAFE_S} s; {PULSE_ON_S}/{PULSE_OFF_S}/{PULSE_ON_S} s: none"
    )


if __name__ == "__main__":
    sys.exit(run([
        row_motor_line,
        row_free_spin,
        row_stall,
        row_torque_speed,
        row_latency,
        row_encoder,
        row_cutout,
    ]))
