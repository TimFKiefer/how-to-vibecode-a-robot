# SPDX-License-Identifier: MIT
"""Stage 1: the World (reset, step, push, truth) and the parameter draw. Headless. Exit 0 only when every
row passes.

    cd <repository root> && sim/.venv/bin/python sim/checks/check_e1_world.py

One World for the whole check, reset per seed; the body file is loaded once.
"""

import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import mujoco  # noqa: E402

from _rows import run  # noqa: E402
from robot2sim import tasks  # noqa: E402
from robot2sim.contacts import floor_force_n, robot_geoms  # noqa: E402
from robot2sim.episode import EVAL_SEEDS  # noqa: E402
from robot2sim.randomise import JITTER, RANGES, jitter, params  # noqa: E402
from robot2sim.servo import KP, SUBSTEPS  # noqa: E402
from robot2sim.units import GRAVITY  # noqa: E402
from robot2sim.world import World  # noqa: E402

SEEDS = range(1, 21)
STAND_HEIGHTS_MM = (30.0, 62.0, 95.0)
HEIGHT_RANGE_MM = (30.0, 95.0)
PENETRATION_MAX_MM = 0.5
BELLY_TOL_MM = 3.0
OFFSET_SPREAD_MIN_DEG = 0.2
MASS_G = 2051.23
MASS_TOL_KG = 1e-6
SETTLE_S = 0.5  # hold before reading a resting foot contact
DROP_MM = 20.0
DROP_S = 3.0
REBOUND_MAX_MM = 5.0
DROP_SETTLED_AFTER_S = 1.0
DROP_SETTLED_TOL_MM = 1.0
QUIET_BELLY_MM = 62.0
QUIET_S = 5.0
QUIET_FROM_S = 1.0
FEET_MIN = 3
FORCE_TOL = 0.25
YAW_DRIFT_MAX_DEG = 1.0
XY_DRIFT_MAX_MM = 2.0
SPEED_STEPS = 3000
PRESS_S = 3.0
PRESS_DEG = {"RF_coxa": 55.0, "LF_coxa": -55.0}
PUSH_STEPS = 10  # tasks.PUSH_S 0.2 s at 50 Hz
PUSH_AFTER_S = 0.5  # steps run past the push's start

WORLD = World()
CONTROL_DT = SUBSTEPS * float(WORLD.model.opt.timestep)
GEOM = {mujoco.mj_id2name(WORLD.model, mujoco.mjtObj.mjOBJ_GEOM, int(g)): int(g) for g in robot_geoms(WORLD.model)}
ROOT_BODY = int(WORLD.model.jnt_bodyid[mujoco.mj_name2id(WORLD.model, mujoco.mjtObj.mjOBJ_JOINT, "root")])
_QUIET = {}


def _weight_n():
    return float(WORLD.model.body_mass.sum()) * GRAVITY


def _steps(seconds):
    return int(round(seconds / CONTROL_DT))


def _yaw_deg_diff(a, b):
    return math.degrees(math.atan2(math.sin(a - b), math.cos(a - b)))


def row_reset_clean():
    worst_pen, worst_belly, n = 0.0, 0.0, 0
    for seed in SEEDS:
        drawn = float(np.random.default_rng(seed).uniform(*HEIGHT_RANGE_MM))
        for pose, belly_mm in [("stand", h) for h in (*STAND_HEIGHTS_MM, drawn)] + [("sit", None)]:
            if pose == "stand":
                WORLD.reset(seed, nominal=False, pose="stand", belly_m=belly_mm / 1000.0)
            else:
                WORLD.reset(seed, nominal=False, pose="sit")
            t = WORLD.truth()
            where = f"seed {seed} {pose} {belly_mm}"
            assert t["penetration_mm"] <= PENETRATION_MAX_MM, f"{where}: {t['penetration_mm']:.3f} mm"
            assert not t["self_hit"], f"{where}: self hit"
            worst_pen = max(worst_pen, t["penetration_mm"])
            if pose == "stand":
                off = abs(t["belly_m"] * 1000.0 - belly_mm)
                assert off <= BELLY_TOL_MM, f"{where}: belly {t['belly_m'] * 1000.0:.2f} mm"
                worst_belly = max(worst_belly, off)
            n += 1
    return f"{n} resets: deepest {worst_pen:.3f} mm, belly off the request ≤ {worst_belly:.2f} mm, no self hit"


def row_variation_applied():
    yaws, xys, offsets = [], [], []
    for seed in SEEDS:
        j = jitter(seed)
        WORLD.reset(seed, nominal=True)
        t = WORLD.truth()
        assert abs(_yaw_deg_diff(t["yaw_rad"], j["yaw_rad"])) < 1e-6, f"seed {seed}: yaw not applied"
        assert np.hypot(t["pos_m"][0] - j["dx_m"], t["pos_m"][1] - j["dy_m"]) < 1e-9, f"seed {seed}: xy not applied"
        true_rad = t["qpos"][7:]
        assert np.allclose(true_rad, WORLD.home_rad + j["joint_offsets_rad"], atol=1e-12), f"seed {seed}: offsets"
        yaws.append(math.degrees(t["yaw_rad"]))
        xys.append(np.array(t["pos_m"][:2]) * 1000.0)
        offsets.append(np.degrees(j["joint_offsets_rad"]))
    yaws, xys, offsets = np.array(yaws), np.array(xys), np.array(offsets)
    assert np.all(np.abs(yaws) <= JITTER["yaw_deg"]), f"yaw {np.abs(yaws).max():.3f}°"
    assert np.all(np.abs(xys) <= JITTER["pos_mm"]), f"xy {np.abs(xys).max():.3f} mm"
    assert offsets.shape == (len(SEEDS), 18)
    assert np.all(np.abs(offsets) <= JITTER["joint_deg"]), f"offset {np.abs(offsets).max():.3f}°"
    spread = float(offsets.std())
    assert spread > OFFSET_SPREAD_MIN_DEG, f"offset spread {spread:.3f}°"
    assert len({round(y, 9) for y in yaws}) == len(SEEDS), "seeds share a yaw"
    WORLD.reset(7, nominal=False)
    first = WORLD.truth()["qpos"]
    WORLD.reset(7, nominal=False)
    assert np.array_equal(first, WORLD.truth()["qpos"]), "same seed, other qpos"
    return (
        f"yaw ≤ {np.abs(yaws).max():.2f}°, xy ≤ {np.abs(xys).max():.2f} mm, offsets ≤ {np.abs(offsets).max():.3f}° "
        f"(spread {spread:.3f}°), {len(SEEDS)} different seeds, same seed → same qpos"
    )


def _model_values():
    m = WORLD.model
    return [a.copy() for a in (
        m.body_mass, m.body_inertia, m.geom_friction, m.actuator_gainprm, m.actuator_biasprm, m.dof_damping,
        m.body_invweight0, m.dof_invweight0, m.body_subtreemass, m.actuator_acc0,
    )]


def _shows(p):
    m = WORLD.model
    mass_kg = float(m.body_mass.sum())
    assert abs(mass_kg - MASS_G / 1000.0 * p["mass_scale"]) <= MASS_TOL_KG, f"mass {mass_kg * 1000:.4f} g"
    assert abs(m.body_subtreemass[0] - mass_kg) <= MASS_TOL_KG, "stale mass constants (mj_setConst missing)"
    assert m.geom_friction[GEOM["RF_foot_tip"], 0] == p["foot_mu"], "foot friction"
    assert m.geom_friction[GEOM["LM_femur_cap"], 0] == p["blade_mu"], "capsule friction"
    assert m.geom_friction[GEOM["body_hex"], 0] == p["blade_mu"], "belly friction"
    assert np.allclose(m.actuator_gainprm[:, 0], KP * p["kp_scale"], rtol=0, atol=1e-12), "actuator gain"


def row_randomisation_applied():
    drawn = {k: [] for k in RANGES}
    for seed in SEEDS:
        WORLD.reset(seed, nominal=False)
        p = WORLD.params
        assert p == params(seed, nominal=False), f"seed {seed}: World.params is not params(seed)"
        for k, (lo, hi) in RANGES.items():
            assert lo <= p[k] <= hi, f"seed {seed}: {k} {p[k]} outside {lo}…{hi}"
            drawn[k].append(p[k])
        _shows(p)
    flat = [k for k, v in drawn.items() if len(set(v)) < 2]
    assert not flat, f"constant: {flat}"
    assert set(drawn["latency_steps"]) == {1, 2, 3}, f"latency {sorted(set(drawn['latency_steps']))}"
    WORLD.reset(3, nominal=False)
    first = _model_values()
    WORLD.reset(4, nominal=False)
    WORLD.reset(3, nominal=False)
    again = _model_values()
    assert all(np.array_equal(a, b) for a, b in zip(first, again)), "reset A, B, A: model differs from the first A"
    WORLD.reset(3, nominal=True)
    for k, (lo, hi) in RANGES.items():
        mid = 2 if k == "latency_steps" else (lo + hi) / 2.0
        assert WORLD.params[k] == mid, f"nominal {k} {WORLD.params[k]}"
    _shows(WORLD.params)
    WORLD.reset(5, nominal=False)
    for _ in range(_steps(SETTLE_S)):
        WORLD.step(WORLD.home_rad)
    n = int(WORLD.data.ncon)
    geom, fric = WORLD.data.contact.geom[:n], WORLD.data.contact.friction[:n, 0]
    feet = [GEOM[f"{h['name']}_foot_tip"] for h in WORLD.spec["hips"]]
    rows = np.flatnonzero(np.isin(geom, feet).any(axis=1))
    assert rows.size, "no resting foot contact"
    assert np.all(fric[rows] == WORLD.params["foot_mu"]), f"contact friction {fric[rows]} ≠ {WORLD.params['foot_mu']}"
    return (
        f"{len(SEEDS)} seeds inside every range, none constant; model mass, foot/capsule/belly friction, gain match; "
        f"A,B,A identical; nominal = middle; {rows.size} foot contacts at μ {WORLD.params['foot_mu']:.4f}"
    )


def row_drop():
    WORLD.reset(1, nominal=True)
    data, z = WORLD.data, WORLD.model.jnt_qposadr[0] + 2
    data.qpos[z] += DROP_MM / 1000.0
    mujoco.mj_forward(WORLD.model, data)
    belly, touch = [], None
    for i in range(_steps(DROP_S)):
        WORLD.step(WORLD.home_rad)
        t = WORLD.truth()
        belly.append(t["belly_m"] * 1000.0)
        if touch is None and t["feet_down"] + t["body_on_floor"] > 0:
            touch = i
    assert touch is not None, "never touched"
    after = np.array(belly[touch:])
    rebound = float(np.max(after - np.minimum.accumulate(after)))
    assert rebound < REBOUND_MAX_MM, f"rebound {rebound:.2f} mm"
    final = belly[-1]
    late = np.array(belly[touch + _steps(DROP_SETTLED_AFTER_S):])
    assert late.size, "run too short"
    wander = float(np.abs(late - final).max())
    assert wander <= DROP_SETTLED_TOL_MM, f"from 1 s after touch {wander:.2f} mm off final {final:.2f} mm"
    return (
        f"touch at {(touch + 1) * CONTROL_DT:.2f} s, rebound {rebound:.2f} mm, final belly {final:.2f} mm, "
        f"≤ {wander:.3f} mm off it from 1 s after"
    )


def row_physics_options():
    m = WORLD.model
    assert m.opt.timestep == 0.002, f"timestep {m.opt.timestep}"
    assert m.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC, f"cone {m.opt.cone}"
    assert m.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST, f"integrator {m.opt.integrator}"
    root_dof = int(m.jnt_dofadr[0])
    assert m.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE
    assert np.all(m.dof_damping[root_dof:root_dof + 6] == 0.0), f"root damping {m.dof_damping[:6]}"
    head = ["RF_coxa", "RF_femur", "RF_knee", "RM_coxa"]
    assert WORLD.joint_names[:4] == head, f"joint_names {WORLD.joint_names[:4]}"
    assert WORLD.n_servos == 18 and len(WORLD.joint_names) == 18
    return "dt 0.002, elliptic, implicitfast, root damping 0, joints RF_coxa, RF_femur, RF_knee, RM_coxa, …"


def _quiet_runs():
    if _QUIET:
        return _QUIET
    weight = _weight_n()
    for seed in SEEDS:
        WORLD.reset(seed, nominal=True, pose="stand", belly_m=QUIET_BELLY_MM / 1000.0)
        start = WORLD.truth()
        feet, force, pen = [], [], []
        for i in range(_steps(QUIET_S)):
            WORLD.step(WORLD.home_rad)
            if (i + 1) * CONTROL_DT <= QUIET_FROM_S + 1e-9:
                continue
            t = WORLD.truth()
            feet.append(t["feet_down"])
            force.append(floor_force_n(WORLD.model, WORLD.data) / weight)
            pen.append(t["penetration_mm"])
        end = WORLD.truth()
        _QUIET[seed] = {
            "feet_min": min(feet), "force_min": min(force), "force_max": max(force), "pen_max": max(pen),
            "yaw_deg": abs(_yaw_deg_diff(end["yaw_rad"], start["yaw_rad"])),
            "xy_mm": float(np.hypot(*(np.array(end["pos_m"][:2]) - start["pos_m"][:2]))) * 1000.0,
        }
    return _QUIET


def row_stand_quiet():
    runs = _quiet_runs()
    bad = []
    for seed, r in runs.items():
        if (r["feet_min"] < FEET_MIN or r["force_min"] < 1 - FORCE_TOL or r["force_max"] > 1 + FORCE_TOL
                or r["yaw_deg"] > YAW_DRIFT_MAX_DEG or r["xy_mm"] > XY_DRIFT_MAX_MM):
            bad.append(
                f"seed {seed}: feet {r['feet_min']}, force {r['force_min']:.3f}…{r['force_max']:.3f} × weight, "
                f"yaw {r['yaw_deg']:.3f}°, xy {r['xy_mm']:.3f} mm"
            )
    assert not bad, "S4 " + "; ".join(bad)
    v = list(runs.values())
    return (
        f"{len(v)} seeds: feet ≥ {min(r['feet_min'] for r in v)}, force {min(r['force_min'] for r in v):.3f}…"
        f"{max(r['force_max'] for r in v):.3f} × weight, yaw drift ≤ {max(r['yaw_deg'] for r in v):.3f}°, "
        f"xy drift ≤ {max(r['xy_mm'] for r in v):.3f} mm"
    )


def row_hold_penetration():
    runs = _quiet_runs()
    worst = max(r["pen_max"] for r in runs.values())
    assert worst <= PENETRATION_MAX_MM, f"{worst:.3f} mm"
    return f"{len(runs)} seeds, deepest after 1 s {worst:.3f} mm"


def info_speed():
    WORLD.reset(1, nominal=True)
    start = time.perf_counter()
    for _ in range(SPEED_STEPS):
        WORLD.step(WORLD.home_rad)
    wall = time.perf_counter() - start
    return f"{SPEED_STEPS * CONTROL_DT / wall:.0f}× real time, {wall / SPEED_STEPS * 1e6:.0f} µs per control step"


def _deepest_overlap_mm():
    data = WORLD.data
    n = int(data.ncon)
    robot = np.isin(data.contact.geom[:n], robot_geoms(WORLD.model)).all(axis=1)
    if not robot.any():
        return 0.0
    return max(0.0, -float(data.contact.dist[:n][robot].min()) * 1000.0)


def row_legs_driven_together():
    WORLD.reset(1, nominal=True)
    targets = WORLD.home_rad.copy()
    pressed = [WORLD.joint_names.index(j) for j in PRESS_DEG]
    for j, deg in PRESS_DEG.items():
        targets[WORLD.joint_names.index(j)] = math.radians(deg)
    first, deepest, cut_at, lost = None, 0.0, None, []
    for i in range(_steps(PRESS_S)):
        WORLD.step(targets)
        t = WORLD.truth()
        now = (i + 1) * CONTROL_DT
        if cut_at is None and t["cut"][pressed].any():
            cut_at = now
        if first is None and t["self_hit"]:
            first = now
        if first is not None:
            deepest = max(deepest, _deepest_overlap_mm())
            if not t["self_hit"] and cut_at is None:
                lost.append(round(now, 2))
    assert first is not None, f"no self hit in {PRESS_S} s"
    assert not lost, f"self hit lost while pressing at {lost[:5]} s"
    cut = f"cut at {cut_at:.2f} s" if cut_at is not None else "no servo cut"
    return f"first hit {first:.2f} s, held to the end, deepest overlap {deepest:.2f} mm, {cut}"


def row_push_steps():
    counts = {}
    for seed in EVAL_SEEDS:
        start = tasks.start("push_walk", seed)
        push = start["push"]
        WORLD.reset(seed, nominal=False, pose=start["pose"], belly_m=start["belly_m"])
        pending, pushed = True, 0
        for _ in range(_steps(push["at_s"] + PUSH_AFTER_S)):
            if pending and WORLD.data.time >= push["at_s"]:  # the episode loop's rule
                WORLD.push(push["fx_n"], push["fy_n"], push["seconds"])
                pending = False
            pushed += bool(np.any(WORLD.data.xfrc_applied[ROOT_BODY] != 0.0))  # the force acts during this step
            WORLD.step(WORLD.home_rad)
        counts[seed] = pushed
    wrong = {s: n for s, n in counts.items() if n != PUSH_STEPS}
    assert not wrong, f"push not {PUSH_STEPS} control steps on seeds {wrong}"
    return f"{len(counts)} evaluation seeds: the force acts {PUSH_STEPS} control steps on every one"


if __name__ == "__main__":
    sys.exit(run([
        row_reset_clean,
        row_variation_applied,
        row_randomisation_applied,
        row_drop,
        row_physics_options,
        row_stand_quiet,
        row_hold_penetration,
        info_speed,
        row_legs_driven_together,
        row_push_steps,
    ]))
