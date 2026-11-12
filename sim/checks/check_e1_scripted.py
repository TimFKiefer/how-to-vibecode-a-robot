# SPDX-License-Identifier: MIT
"""Stage 1: the scripted rival and the gate run. Headless. Row 5 is the only row that writes into runs/:
it prints its folder. Every other evaluate writes into a temp folder. Exit 0 only when every row passes.

    cd <repository root> && sim/.venv/bin/python sim/checks/check_e1_scripted.py
"""

import json
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import mujoco  # noqa: E402

from _rows import run  # noqa: E402
from robot2sim import contacts, tasks  # noqa: E402
from robot2sim.brains import BodySpec  # noqa: E402
from robot2sim.episode import EVAL_SEEDS, evaluate, run_episode  # noqa: E402
from robot2sim.frames import mat16  # noqa: E402
from robot2sim.kinematics import leg_ik  # noqa: E402
from robot2sim.paths import read_json, reference_json  # noqa: E402
from robot2sim.scene import load_spec  # noqa: E402
from robot2sim.scripted import Scripted, gait_feet, heights_mm, spots  # noqa: E402
from robot2sim.sensors import SENSOR_CHANNELS  # noqa: E402
from robot2sim.source import load_robot2_namespace, require  # noqa: E402
from robot2sim.units import m_to_mm, mm_to_m, rad_to_deg  # noqa: E402
from robot2sim.world import World  # noqa: E402

FRAMES = 24  # reference gait records per cycle (walk_00 … walk_23)
FEET_TOL_MM = 1e-6
JOINTS_TOL_DEG = 1e-6
HOME_TOL_RAD = 1e-9
JUMP_MAX_RAD = 0.094  # the C018's no-load speed per 20 ms control step
NO_PHYSICS_STEPS = 500  # 10 s at 50 Hz
SPY_STEPS = 200
BAD_BELLY_MM = 50.0
GATE_MIN = 18
TURN_SEED = 1001
TURN_AFTER_STEPS = 200  # 4 s
TURN_MIN_DEG = 20.0
CONTROL_DT_S = 0.02
GATE_OUT = None  # None = runs/: the gate run is the one row that writes there
GATE = {}  # row 5's run, read by row 7


def _records():
    """(id, kind, NAMED_HEIGHTS entry, record) of every walk/turn gait record at 30, 62 and 95 mm."""
    named = require(load_robot2_namespace(), "NAMED_HEIGHTS")["NAMED_HEIGHTS"]
    prefixes = {"": named["normal"], "h30_": named["spinne"], "h95_": named["hoch"]}
    out = []
    for rec in read_json(reference_json())["poses"]:
        for prefix, entry in prefixes.items():
            for kind in ("walk", "turn"):
                rest = rec["id"].removeprefix(prefix + kind + "_")
                if rec["id"].startswith(prefix + kind + "_") and rest.isdigit():
                    out.append((rec["id"], kind, entry, rec))
    return out


def row_feet_equal_cad():
    spec = load_spec("v0")
    recs = _records()
    assert len(recs) == 3 * 2 * FRAMES, len(recs)
    worst = 0.0
    for rid, kind, entry, rec in recs:
        base = spots(entry["gait_r"], spec["hips"], spec["constants"])
        feet = gait_feet(kind, int(rid[-2:]) / FRAMES, base, entry["lift"], entry["out"])
        for leg, target in rec["targets"].items():
            err = max(abs(a - b) for a, b in zip(feet[leg], target))
            worst = max(worst, err)
            assert err <= FEET_TOL_MM, f"{rid} {leg}: {feet[leg]} vs {target}"
    return f"{len(recs)} records (walk/turn at 30, 62, 95 mm): worst foot error {worst:.1e} mm"


def row_joints_equal_cad():
    spec = load_spec("v0")
    c, hips = spec["constants"], spec["hips"]
    recs = _records()
    worst = 0.0
    for rid, kind, entry, rec in recs:
        assert list(rec["contact"]) == [c["L_TIBIA"], 0.0], f"{rid} contact {rec['contact']}"
        base = spots(entry["gait_r"], hips, c)
        feet = gait_feet(kind, int(rid[-2:]) / FRAMES, base, entry["lift"], entry["out"])
        B = mat16(rec["B"])
        for hip in hips:
            got = leg_ik(hip, feet[hip["name"]], B, (c["L_TIBIA"], 0.0), c)
            err = max(abs(a - b) for a, b in zip(got, rec["joints"][hip["name"]]))
            worst = max(worst, err)
            assert err <= JOINTS_TOL_DEG, f"{rid} {hip['name']}: {got} vs {rec['joints'][hip['name']]}"
    return f"{len(recs)} records: worst joint error {worst:.1e} deg"


def _obs(wish, servo_pos=None):
    obs = {k: np.zeros(n, dtype=np.float32) for k, n in SENSOR_CHANNELS.items()}
    obs["wish"] = np.asarray(wish, dtype=float)
    if servo_pos is not None:
        obs["servo_pos"] = np.asarray(servo_pos, dtype=np.float32)
    return obs


def _body(world):
    return BodySpec(world.n_servos, world.lower_rad.copy(), world.upper_rad.copy())


def row_no_jump():
    world = World()
    parts = []
    worst_home = worst_jump = 0.0
    for belly_mm in heights_mm():
        belly_m = mm_to_m(belly_mm)
        world.reset(EVAL_SEEDS[0], True, "stand", belly_m)
        for task in ("walk_1m", "turn_90"):
            brain = Scripted()
            brain.reset(_body(world), task, EVAL_SEEDS[0])
            obs = _obs(tasks.wish(task, belly_m))
            actions = np.array([brain.act(obs) for _ in range(NO_PHYSICS_STEPS)])
            home = float(np.max(np.abs(actions[0] - world.home_rad)))
            jump = float(np.max(np.abs(np.diff(actions, axis=0))))
            worst_home, worst_jump = max(worst_home, home), max(worst_jump, jump)
            assert home <= HOME_TOL_RAD, f"{belly_mm} mm {task}: first action {home:.2e} rad from home"
            assert jump <= JUMP_MAX_RAD, f"{belly_mm} mm {task}: jump {jump:.4f} rad"
            assert brain.info() == {"ik_failures": 0}, (belly_mm, task, brain.info())
            parts.append(f"{belly_mm:.0f}/{task.split('_')[0]} {jump:.4f}")
    return f"first action ≤ {worst_home:.1e} rad from home; largest step (rad): {', '.join(parts)}"


class Recording(dict):
    """A sensor dict that logs every key read, with the act number it was read at."""

    def __init__(self, data, log, act):
        super().__init__(data)
        self.log, self.act = log, act

    def __getitem__(self, key):
        self.log.append((self.act, key))
        return super().__getitem__(key)

    def get(self, key, default=None):
        self.log.append((self.act, key))
        return super().get(key, default)

    def __contains__(self, key):
        self.log.append((self.act, key))
        return super().__contains__(key)

    def __iter__(self):
        self.log.append((self.act, "<iter>"))
        return super().__iter__()

    def keys(self):
        self.log.append((self.act, "<keys>"))
        return super().keys()

    def values(self):
        self.log.append((self.act, "<values>"))
        return super().values()

    def items(self):
        self.log.append((self.act, "<items>"))
        return super().items()


def _reads(wish, world):
    brain = Scripted()
    brain.reset(_body(world), "walk_1m", EVAL_SEEDS[0])
    log = []
    for k in range(SPY_STEPS):
        brain.act(Recording(_obs(wish, world.home_rad), log, k))
    return log


def row_open_loop():
    world = World()
    world.reset(EVAL_SEEDS[0], True, "stand", tasks.STAND_BELLY_M)
    walk = _reads(tasks.wish("walk_1m", tasks.STAND_BELLY_M), world)
    turn = _reads(tasks.wish("turn_90", tasks.STAND_BELLY_M), world)
    hold = _reads(np.zeros(SENSOR_CHANNELS["wish"]), world)
    assert walk == [(0, "wish")], walk[:10]
    assert turn == [(0, "wish")], turn[:10]
    assert sorted(hold) == [(0, "servo_pos"), (0, "wish")], hold[:10]
    brain = Scripted()
    brain.reset(_body(world), "walk_1m", EVAL_SEEDS[0])
    try:
        brain.act(_obs(tasks.wish("walk_1m", mm_to_m(BAD_BELLY_MM))))
    except ValueError as e:
        message = str(e)
    else:
        raise AssertionError(f"belly {BAD_BELLY_MM} mm: no ValueError")
    assert all(f"{h:g}" in message for h in heights_mm()), message
    return f"{SPY_STEPS} steps: walk/turn read {walk}, zero wish read {sorted(hold)}; {BAD_BELLY_MM:g} mm → {message}"


class Diagnosis:
    """on_step for the gate run: per episode the last CoM, per foot the stances, and the floor force range."""

    def __init__(self, feet_names):
        self.names = feet_names
        self.ids = None
        self.episodes = []
        self.forces = []
        self.slips = []
        self.open = {}

    def _close(self):
        for start, last in self.open.values():
            self.slips.append(float(np.hypot(*(last - start))))
        self.open = {}

    def __call__(self, world):
        m, d = world.model, world.data
        if round(d.time / CONTROL_DT_S) == 1:
            self._close()
            self.episodes.append(None)
        self.episodes[-1] = d.subtree_com[world._root_body].copy()
        self.forces.append(contacts.floor_force_n(m, d))
        if self.ids is None:
            self.ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n) for n in self.names]
        down = set(contacts.on_floor(m, d))
        for name, gid in zip(self.names, self.ids):
            xy = d.geom_xpos[gid, :2].copy()
            if name in down:
                start = self.open.get(name, (xy, xy))[0]
                self.open[name] = (start, xy)
            elif name in self.open:
                start, last = self.open.pop(name)
                self.slips.append(float(np.hypot(*(last - start))))
        return True


def _read_run(run_dir):
    run_dir = Path(run_dir)
    metrics = [json.loads(line) for line in (run_dir / "metrics.jsonl").read_text().splitlines()]
    with np.load(run_dir / "motion.npz") as z:
        qpos, episode = z["qpos"], z["episode"]
    starts = [qpos[np.flatnonzero(episode == n)[0]] for n in range(len(metrics))]
    return metrics, starts


def _yaw(q):
    w, x, y, z = (float(v) for v in q[3:7])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def row_gate_walk_1m():
    spec = load_spec("v0")
    diag = Diagnosis([f"{h['name']}_foot_tip" for h in spec["hips"]])
    summary = evaluate("scripted", "walk_1m", EVAL_SEEDS, nominal=True, belly_mm=62, out_dir=GATE_OUT, on_step=diag)
    diag._close()
    metrics, starts = _read_run(summary["run_dir"])
    GATE.update(summary=summary, metrics=metrics, starts=starts, diag=diag)
    reasons = {r: n for r, n in summary["reasons"].items() if n}
    hits = [m["self_hit_steps"] for m in metrics]
    ik = [m["ik_failures"] for m in metrics]
    assert len(diag.episodes) == len(metrics) == len(EVAL_SEEDS), (len(diag.episodes), len(metrics))
    assert summary["successes"] >= GATE_MIN, f"{summary['successes']}/20 {reasons}: S1, {summary['run_dir']}"
    assert not any(hits), f"self_hit_steps {hits}"
    assert not any(ik), f"ik_failures {ik}"
    assert summary["eval_seeds"] is True, summary["eval_seeds"]
    return (f"{summary['successes']}/{len(metrics)} {reasons}, self_hit 0, ik_failures 0, eval_seeds True, "
            f"speed {summary['speed_x']:.0f}x, run {summary['run_dir']}")


def row_turn_direction():
    world = World()
    metrics, trace = run_episode(world, Scripted(), "turn_90", TURN_SEED, True,
                                 on_step=lambda w: round(w.data.time / CONTROL_DT_S) < TURN_AFTER_STEPS)
    assert metrics["steps"] == TURN_AFTER_STEPS, (metrics["steps"], metrics["reason"])
    moved = rad_to_deg(math.remainder(_yaw(trace[TURN_AFTER_STEPS]) - _yaw(trace[0]), 2.0 * math.pi))
    assert moved >= TURN_MIN_DEG, f"yaw moved {moved:.1f} deg towards the goal (+90)"
    return f"seed {TURN_SEED}: after {TURN_AFTER_STEPS * CONTROL_DT_S:.0f} s yaw +{moved:.1f} deg towards the goal"


def info_diagnosis():
    assert GATE, "row 5 did not run"
    metrics, starts, diag = GATE["metrics"], GATE["starts"], GATE["diag"]
    times = [m["time_s"] for m in metrics if m["success"]]
    side = []
    for m, q0, com in zip(metrics, starts, diag.episodes):
        if m["success"]:
            psi = _yaw(q0)
            d = com[:2] - np.asarray(q0[:2], dtype=float)
            side.append(abs(float(d[0] * math.cos(psi) + d[1] * math.sin(psi))))
    airborne = sum(m["airborne_steps"] for m in metrics) / sum(m["steps"] for m in metrics)
    depth = max(m["max_penetration_mm"] for m in metrics)
    ends = ", ".join(f"{m['seed']}:{m['reason']}@{m['time_s']:.1f}s" for m in metrics if not m["success"]) or "none"
    time_part = f"time to goal mean {np.mean(times):.2f} s, worst {max(times):.2f} s" if times else "no success"
    side_part = f"sideways at goal mean {m_to_mm(np.mean(side)):.1f} mm, worst {m_to_mm(max(side)):.1f} mm" if side \
        else "no sideways error"
    return (f"{time_part}; {side_part}; airborne {airborne:.2%} of steps; max penetration {depth:.3f} mm; "
            f"stance slip mean {m_to_mm(np.mean(diag.slips)):.2f} mm over {len(diag.slips)} stances; "
            f"floor force {min(diag.forces):.1f}…{max(diag.forces):.1f} N; failed: {ends}")


def info_other_runs():
    runs = (
        ("walk_1m random", "walk_1m", False, None),
        ("walk_1m 30 mm", "walk_1m", True, 30.0),
        ("walk_1m 95 mm", "walk_1m", True, 95.0),
        ("turn_90", "turn_90", True, None),
        ("push_walk", "push_walk", True, None),
    )
    parts = []
    for label, task, nominal, belly in runs:
        with tempfile.TemporaryDirectory() as tmp:
            s = evaluate("scripted", task, EVAL_SEEDS, nominal=nominal, belly_mm=belly, out_dir=Path(tmp))
        reasons = "/".join(f"{r} {n}" for r, n in s["reasons"].items() if n)
        parts.append(f"{label} {s['successes']}/{s['episodes']} ({reasons})")
    return "; ".join(parts)


if __name__ == "__main__":
    sys.exit(run([
        row_feet_equal_cad, row_joints_equal_cad, row_no_jump, row_open_loop, row_gate_walk_1m,
        row_turn_direction, info_diagnosis, info_other_runs,
    ]))
