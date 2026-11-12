# SPDX-License-Identifier: MIT
"""Stage 1: the sensor list, the only view a brain gets of the robot. Headless. Exit 0 only when every row
passes.

    cd <repository root> && sim/.venv/bin/python sim/checks/check_e1_sensors.py

One World for the whole check. Rows 2–6 write qpos / qvel / data.time directly and never step the physics.
"""

import contextlib
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import mujoco  # noqa: E402

from _rows import run  # noqa: E402
from robot2sim.sensors import EYE, IMU, SENSOR_CHANNELS, Sensors, wrap  # noqa: E402
from robot2sim.servo import ENCODER_STEPS  # noqa: E402
from robot2sim.world import World  # noqa: E402

SEED = 3
N_SERVOS = 18
NO_GOAL = {"kind": "none"}
WISH = np.array([0.05, 0.0, 0.0, 0.062])
READ_DT_S = 0.02  # one control step between reads
RATE_WINDOW_S = 3.0
RATE_EXPECTED = 90
RATE_TOL = 1
MOVE_MPS = (0.05, -0.03)  # root speed in rows 2 and 3, so every instant has its own truth
MOVE_YAW_RAD_S = 0.3
MOVE_Z_MPS = 0.004
STILL_READS = 2000
STILL_READ_DT_S = 0.04  # longer than the eye period: every read shows a new sample
SPREAD_TOL = 0.30
GOAL_TOL = 1e-6
YAW_GOAL_DEG = 170.0
YAW_SWEEP_DEG = list(range(160, 181)) + list(range(-179, -169))
YAW_SETTLE_READS = 10  # reads after the sweep, so the delayed eye catches up
YAW_JUMP_MAX_RAD = 0.2
PITCH_DEG = 30.0
IMU_TOL = 0.1
SPIN_RAD_S = 1.0
TURNED_YAW_DEG = 90.0  # the body-x spin case: a world-frame gyro would read (0, 1, 0) here
GRID_RAD = 2.0 * math.pi / ENCODER_STEPS

WORLD = World()
ROOT = mujoco.mj_name2id(WORLD.model, mujoco.mjtObj.mjOBJ_JOINT, "root")
RQ = int(WORLD.model.jnt_qposadr[ROOT])
RV = int(WORLD.model.jnt_dofadr[ROOT])
COMMAND = np.zeros(N_SERVOS)


@contextlib.contextmanager
def _no_noise():
    """Eye and IMU sigmas 0 on the module dicts, restored after. The bus is new at each reset: see _reset."""
    saved = {"EYE": dict(EYE), "IMU": dict(IMU)}
    EYE.update({k: 0.0 for k in EYE if k.startswith("sigma")})
    IMU.update({k: 0.0 for k in IMU})
    try:
        yield
    finally:
        EYE.update(saved["EYE"])
        IMU.update(saved["IMU"])


def _reset(quiet):
    WORLD.reset(SEED, nominal=False)
    if quiet:
        WORLD.bus.sigma[:] = 0.0


def _fresh(quiet, goal=NO_GOAL):
    _reset(quiet)
    s = Sensors(WORLD, SEED)
    s.reset(goal, WISH)
    return s


def _set_root(xyz, yaw_rad, pitch_rad=0.0):
    """Root at xyz, turned yaw about world z, then pitched nose up about body x."""
    qz = np.array([math.cos(yaw_rad / 2.0), 0.0, 0.0, math.sin(yaw_rad / 2.0)])
    qx = np.array([math.cos(pitch_rad / 2.0), math.sin(pitch_rad / 2.0), 0.0, 0.0])
    q = np.zeros(4)
    mujoco.mju_mulQuat(q, qz, qx)
    WORLD.data.qpos[RQ:RQ + 3] = xyz
    WORLD.data.qpos[RQ + 3:RQ + 7] = q


def _true_eye():
    d = WORLD.data
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, d.qpos[RQ + 3:RQ + 7])
    yaw = math.atan2(R[3], R[0])
    return np.array([d.qpos[RQ], d.qpos[RQ + 1], yaw, d.qpos[RQ + 2]], dtype=np.float32)


def row_sensor_list():
    with _no_noise():
        s = _fresh(quiet=True)
        first = s.read(COMMAND)
        assert list(first) == list(SENSOR_CHANNELS), f"keys {list(first)}"
        for k, n in SENSOR_CHANNELS.items():
            assert first[k].shape == (n,), f"{k} shape {first[k].shape}"
            assert first[k].dtype == np.float32, f"{k} dtype {first[k].dtype}"
        kept = {k: v.copy() for k, v in first.items()}
        for v in first.values():
            v[:] = 999.0
        second = s.read(COMMAND)
        changed = [k for k in kept if not np.array_equal(second[k], kept[k])]
        assert not changed, f"writing into a returned array changed the next read: {changed}"
    total = sum(SENSOR_CHANNELS.values())
    assert total == 107, f"{total} numbers"
    return f"{len(SENSOR_CHANNELS)} channels, {total} float32 numbers, returned arrays are copies"


def _moving_truth(t):
    x0, y0 = MOVE_MPS
    return [x0 * t, y0 * t, 0.06 + MOVE_Z_MPS * t], MOVE_YAW_RAD_S * t


def row_eye_rate_delay():
    """Count the eye changes in a 3 s window that starts once the first delay is over."""
    delay, period = EYE["delay_s"], 1.0 / EYE["rate_hz"]
    with _no_noise():
        _reset(quiet=True)
        d = WORLD.data
        d.time = 0.0
        _set_root(*_moving_truth(0.0))
        s = Sensors(WORLD, SEED)
        s.reset(NO_GOAL, WISH)
        history = [(0.0, _true_eye())]
        previous = s.read(COMMAND)["eye"]
        changes, worst_lo, worst_hi, n = 0, math.inf, -math.inf, 0
        while d.time < delay + RATE_WINDOW_S - 1e-9:
            d.time += READ_DT_S
            t = d.time
            _set_root(*_moving_truth(t))
            history.append((t, _true_eye()))
            eye = s.read(COMMAND)["eye"]
            if t > delay + 1e-9 and not np.array_equal(eye, previous):
                changes += 1
            previous = eye
            if t >= delay - 1e-9:
                seen = [ts for ts, e in history if np.array_equal(e, eye)]
                assert len(seen) == 1, f"t {t:.3f}: eye matches {len(seen)} truths"
                age = t - seen[0]
                assert delay - 1e-9 <= age <= delay + period + READ_DT_S + 1e-9, f"t {t:.3f}: eye {age:.4f} s old"
                worst_lo, worst_hi, n = min(worst_lo, age), max(worst_hi, age), n + 1
    assert abs(changes - RATE_EXPECTED) <= RATE_TOL, f"{changes} changes in {RATE_WINDOW_S} s"
    return f"{changes} changes in {RATE_WINDOW_S:.0f} s, eye age {worst_lo:.3f}…{worst_hi:.3f} s over {n} reads"


def row_eye_noise():
    s = _fresh(quiet=False)
    d = WORLD.data
    truth = _true_eye().astype(float)
    eyes = []
    while len(eyes) < STILL_READS:
        d.time += STILL_READ_DT_S
        eye = s.read(COMMAND)["eye"].astype(float)
        if d.time > EYE["delay_s"] + STILL_READ_DT_S:  # before that the reset measurement is still shown
            eyes.append(eye)
    err = np.array(eyes) - truth
    err[:, 2] = wrap(err[:, 2])
    spread = err.std(axis=0)
    sigma = np.array([EYE["sigma_xy_m"], EYE["sigma_xy_m"], EYE["sigma_yaw_rad"], EYE["sigma_z_m"]])
    ratio = spread / sigma
    assert np.all(np.abs(ratio - 1.0) <= SPREAD_TOL), f"spread / sigma {np.round(ratio, 3)}"
    assert len({tuple(e) for e in eyes}) == STILL_READS, "two reads showed the same sample"
    with _no_noise():
        s = _fresh(quiet=True)
        truth = _true_eye()
        exact = 0
        for _ in range(STILL_READS):
            d.time += STILL_READ_DT_S
            exact += bool(np.array_equal(s.read(COMMAND)["eye"], truth))
    assert exact == STILL_READS, f"sigmas 0: {exact} of {STILL_READS} reads equal the truth"
    return f"spread / sigma (x, y, yaw, z) {', '.join(f'{r:.3f}' for r in ratio)}; sigmas 0: {exact} exact"


def row_goal_frame():
    with _no_noise():
        _reset(quiet=True)
        xyz = WORLD.data.qpos[RQ:RQ + 3].copy()
        target = {"kind": "point", "xy_m": (float(xyz[0]), float(xyz[1]) + 1.0)}
        out = []
        for yaw_deg in (0.0, 90.0):
            _set_root(xyz, math.radians(yaw_deg))
            s = Sensors(WORLD, SEED)
            s.reset(target, WISH)
            out.append(s.read(COMMAND)["goal"].astype(float))
    ahead, turned = out
    assert np.allclose(ahead, [0.0, 1.0, 0.0], atol=GOAL_TOL), f"yaw 0: {ahead}"
    assert np.allclose(turned, [1.0, 0.0, 0.0], atol=GOAL_TOL), f"yaw 90°: {turned}"
    return f"yaw 0 → {np.round(ahead, 6).tolist()}, yaw +90° → {np.round(turned, 6).tolist()}"


def row_yaw_wrap():
    """Noise on: a wrap bug near ±180° would show as a jump of about 2π, noise alone stays far below 0.2 rad."""
    _reset(quiet=False)
    d = WORLD.data
    xyz = d.qpos[RQ:RQ + 3].copy()
    _set_root(xyz, math.radians(YAW_SWEEP_DEG[0]))
    s = Sensors(WORLD, SEED)
    s.reset({"kind": "yaw", "yaw_rad": math.radians(YAW_GOAL_DEG)}, WISH)
    dyaw, from_eye = [], []
    for n, deg in enumerate(YAW_SWEEP_DEG + [YAW_SWEEP_DEG[-1]] * YAW_SETTLE_READS):
        if n:
            d.time += READ_DT_S
            _set_root(xyz, math.radians(deg))
        out = s.read(COMMAND)
        dyaw.append(float(out["goal"][2]))
        from_eye.append(float(wrap(math.radians(YAW_GOAL_DEG) - float(out["eye"][2]))))
    dyaw = np.array(dyaw)
    leak = float(np.abs(dyaw - np.array(from_eye)).max())
    assert leak <= GOAL_TOL, f"goal differs from the eye's own value by {leak:.6f} rad (computed from truth?)"
    biggest = float(np.abs(dyaw).max())
    jump = float(np.abs(np.diff(dyaw)).max())
    assert biggest <= math.pi, f"|dyaw| {biggest:.4f}"
    assert jump <= YAW_JUMP_MAX_RAD, f"step {jump:.4f} rad"
    end = math.degrees(dyaw[-1])
    return f"{len(dyaw)} reads, |dyaw| ≤ {biggest:.3f} rad, largest step {jump:.3f} rad, ends at {end:.1f}°"


def row_imu():
    _reset(quiet=False)
    d = WORLD.data
    xyz = d.qpos[RQ:RQ + 3].copy()
    s = Sensors(WORLD, SEED)
    s.reset(NO_GOAL, WISH)
    _set_root(xyz, 0.0)
    d.qvel[:] = 0.0
    level = s.read(COMMAND)["imu_gravity"].astype(float)
    _set_root(xyz, 0.0, math.radians(PITCH_DEG))
    pitched = s.read(COMMAND)["imu_gravity"].astype(float)
    _set_root(xyz, 0.0)
    d.qvel[RV + 5] = SPIN_RAD_S
    gyro = s.read(COMMAND)["imu_gyro"].astype(float)
    d.qvel[:] = 0.0
    _set_root(xyz, math.radians(TURNED_YAW_DEG))
    d.qvel[RV + 3] = SPIN_RAD_S  # the free joint's angular velocity is in the body frame: about body x
    body_x = s.read(COMMAND)["imu_gyro"].astype(float)
    d.qvel[:] = 0.0
    assert np.allclose(level, [0.0, 0.0, -1.0], atol=IMU_TOL), f"level {level}"
    assert abs(pitched[1] + 0.5) <= IMU_TOL, f"pitched {pitched}"
    assert abs(gyro[2] - SPIN_RAD_S) <= IMU_TOL, f"gyro {gyro}"
    assert np.allclose(body_x, [SPIN_RAD_S, 0.0, 0.0], atol=IMU_TOL), (
        f"yaw {TURNED_YAW_DEG:.0f}°, body x spin: {body_x}"
    )
    return (
        f"level {np.round(level, 3).tolist()}, pitched {PITCH_DEG:.0f}° y {pitched[1]:.3f}, "
        f"spin gyro z {gyro[2]:.3f}; yaw {TURNED_YAW_DEG:.0f}°, spin about body x: gyro "
        f"{np.round(body_x, 3).tolist()} (noise on)"
    )


def row_offsets_hidden():
    _reset(quiet=True)
    offsets = WORLD.jitter["joint_offsets_rad"]
    assert np.abs(offsets).max() > GRID_RAD, "offsets too small to test"
    with _no_noise():
        s = Sensors(WORLD, SEED)
        s.reset(NO_GOAL, WISH)
        pos = s.read(COMMAND)["servo_pos"].astype(float)
    off = np.abs(pos - WORLD.home_rad).max()
    assert off <= GRID_RAD, f"servo_pos off home by {off:.6f} rad (step {GRID_RAD:.6f})"
    return (
        f"servo_pos − home ≤ {off / GRID_RAD:.3f} encoder steps, true offsets up to "
        f"{np.degrees(np.abs(offsets).max()):.3f}°"
    )


if __name__ == "__main__":
    sys.exit(run([
        row_sensor_list, row_eye_rate_delay, row_eye_noise, row_goal_frame, row_yaw_wrap, row_imu,
        row_offsets_hidden,
    ]))
