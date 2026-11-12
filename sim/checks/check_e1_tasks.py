# SPDX-License-Identifier: MIT
"""Stage 1: the four tasks and the judge. Pure: no World, every truth is a hand-made dict with the keys of
World.truth(). Exit 0 only when every row passes.

    cd <repository root> && sim/.venv/bin/python sim/checks/check_e1_tasks.py
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _rows import run  # noqa: E402
from robot2sim.sensors import wrap  # noqa: E402
from robot2sim.tasks import CONTROL_DT_S, TASKS, TaskSpec, goal, judge, spec, start, wish  # noqa: E402
from robot2sim.units import deg_to_rad, rad_to_deg  # noqa: E402

N_SERVOS = 18
EXPECTED_SPECS = {
    "stand_up": TaskSpec("stand_up", 30, "sit", "none", False),
    "walk_1m": TaskSpec("walk_1m", 60, "stand", "point", True),
    "turn_90": TaskSpec("turn_90", 30, "stand", "yaw", True),
    "push_walk": TaskSpec("push_walk", 60, "stand", "point", True),
}
HOLD_CALLS = 100  # 2.0 s at 0.02 s per call
PUSH_SEEDS = range(200)
GOAL_TOL = 1e-12
BELLY_M = 0.062


def _truth(**over) -> dict:
    """A quiet stand at the origin facing +Y, nothing reached; `over` replaces keys."""
    t = {
        "time_s": 1.0, "pos_m": np.array([0.0, 0.0, 0.1]), "com_m": np.array([0.0, 0.0, 0.1]), "yaw_rad": 0.0,
        "roll_rad": 0.0, "pitch_rad": 0.0, "tilt_rad": 0.0, "belly_m": 0.062, "body_on_floor": False,
        "feet_down": 6, "self_hit": False, "penetration_mm": 0.0, "cut": np.zeros(N_SERVOS, dtype=bool),
        "qpos": np.zeros(25),
    }
    t.update(over)
    return t


def _cut_one() -> np.ndarray:
    cut = np.zeros(N_SERVOS, dtype=bool)
    cut[7] = True
    return cut


def row_specs():
    assert TASKS == ("stand_up", "walk_1m", "turn_90", "push_walk"), TASKS
    for name in TASKS:
        assert spec(name) == EXPECTED_SPECS[name], (name, spec(name))
    try:
        spec("fly")
    except KeyError as e:
        assert all(name in str(e) for name in TASKS), f"KeyError does not list the names: {e}"
    else:
        raise AssertionError('spec("fly") did not raise')
    return f"{len(TASKS)} specs as the table, spec('fly') → KeyError"


def row_judge_table():
    walk = spec("walk_1m")
    far = {"kind": "point", "xy_m": (0.0, 1.0)}
    reached = {"kind": "point", "xy_m": (0.0, 0.05)}  # com at the origin is 0.05 m away
    cases = [
        ("cutout", walk, far, _truth(cut=_cut_one())),
        ("fall", walk, far, _truth(tilt_rad=deg_to_rad(46.0))),
        ("fall", walk, far, _truth(body_on_floor=True)),
        ("success", walk, reached, _truth()),
        ("timeout", walk, far, _truth(time_s=60.0)),
        (None, walk, far, _truth()),
        ("cutout", walk, reached, _truth(cut=_cut_one())),  # a cut servo beats a reached goal
        ("success", walk, reached, _truth(time_s=60.0)),  # a reached goal beats the limit at the same step
        (None, spec("stand_up"), {"kind": "none"}, _truth(body_on_floor=True, belly_m=0.002)),  # sitting: no fall
        ("fall", walk, reached, _truth(tilt_rad=deg_to_rad(46.0))),  # a fall beats a reached goal
        ("fall", walk, reached, _truth(body_on_floor=True)),
        ("cutout", walk, far, _truth(cut=_cut_one(), tilt_rad=deg_to_rad(46.0))),  # a cut servo beats a fall
        ("fall", walk, far, _truth(tilt_rad=deg_to_rad(46.0), time_s=60.0)),  # a fall beats the limit
        (None, walk, far, _truth(tilt_rad=deg_to_rad(44.0))),  # 44° is no fall
        ("success", walk, {"kind": "point", "xy_m": (0.0, 0.09)}, _truth()),  # 0.09 m from the goal
        (None, walk, {"kind": "point", "xy_m": (0.0, 0.11)}, _truth()),  # 0.11 m from the goal
    ]
    turn = spec("turn_90")
    heading = {"kind": "yaw", "yaw_rad": 0.0}
    one_short = (HOLD_CALLS - 1) * CONTROL_DT_S
    held_cases = [
        ("success", turn, heading, _truth(), one_short),  # on the heading and level, the 100th call
        (None, turn, heading, _truth(), 0.0),  # the same truth on the first call
        ("cutout", turn, heading, _truth(cut=_cut_one()), one_short),  # a cut servo beats a held heading
        ("success", turn, heading, _truth(time_s=30.0), one_short),  # a held heading beats the limit
    ]
    for n, (want, sp, g, t, held) in enumerate([c + (0.0,) for c in cases] + held_cases, 1):
        got, _ = judge(sp, g, t, held)
        assert got == want, f"case {n}: {got!r}, want {want!r}"
    return f"{len(cases) + len(held_cases)} hand-made truths give their reason (turn_90: held 1.98 s → success, 0 s → None)"


def row_stand_up_hold():
    sp = spec("stand_up")
    none = {"kind": "none"}
    up = _truth(belly_m=0.056, roll_rad=deg_to_rad(9.0), pitch_rad=deg_to_rad(-9.0))
    down = _truth(belly_m=0.050)
    held = 0.0
    for k in range(HOLD_CALLS - 1):
        reason, held = judge(sp, none, up, held)
        assert reason is None, f"call {k + 1}: {reason!r} at held {held:.4f} s"
    reason, held = judge(sp, none, up, held)
    assert reason == "success", f"call {HOLD_CALLS}: {reason!r} at held {held:.4f} s"
    held_at_success = held
    held = 0.0
    for _ in range(50):
        _, held = judge(sp, none, up, held)
    reason, held = judge(sp, none, down, held)
    assert reason is None and held == 0.0, (reason, held)
    for key in ("roll_rad", "pitch_rad"):
        for tilt_deg in (10.0, 11.0, -11.0):
            held = 0.0
            for _ in range(50):
                _, held = judge(sp, none, up, held)
            reason, held = judge(sp, none, _truth(belly_m=0.056, **{key: deg_to_rad(tilt_deg)}), held)
            assert reason is None and held == 0.0, f"{key} {tilt_deg}°: {reason!r}, held {held:.4f} s"
    held = 0.0
    for k in range(HOLD_CALLS - 1):
        reason, held = judge(sp, none, up, held)
        assert reason is None, f"after the break, call {k + 1}: {reason!r}"
    reason, held = judge(sp, none, up, held)
    assert reason == "success", reason
    return f"99 calls → None, call 100 → success (held {held_at_success:.4f} s); a break (belly 50 mm, roll or pitch 10°/±11°) restarts the count"


def _calls_to_success(sp, g, truth, calls):
    """The call number (1-based) at which a truth repeated `calls` times succeeds, else None; and the last held_s."""
    held = 0.0
    for k in range(1, calls + 1):
        reason, held = judge(sp, g, truth, held)
        if reason is not None:
            assert reason == "success", f"call {k}: {reason!r}"
            return k, held
    return None, held


def row_yaw_wrap():
    sp = spec("turn_90")
    g = goal("turn_90", _truth(yaw_rad=deg_to_rad(175.0)))
    assert g["kind"] == "yaw", g
    goal_deg = rad_to_deg(g["yaw_rad"])
    assert abs(goal_deg - (-95.0)) < 1e-9, goal_deg
    for d in range(-9, 10):
        at, _ = _calls_to_success(sp, g, _truth(yaw_rad=float(wrap(deg_to_rad(-95.0 + d)))), 2 * HOLD_CALLS)
        assert at == HOLD_CALLS, f"−95° {d:+d}°: success at call {at}"
    for miss_deg in (175.0, 85.0, -95.0 - 11.0, -95.0 + 11.0):
        at, held = _calls_to_success(sp, g, _truth(yaw_rad=deg_to_rad(miss_deg)), 2 * HOLD_CALLS)
        assert at is None and held == 0.0, f"{miss_deg}°: success at call {at}, held {held}"
    return (
        f"goal {goal_deg:.3f}°, level at −95° ± 9° succeeds at call {HOLD_CALLS}; 175°, 85°, −106°, −84°: "
        f"held 0 over {2 * HOLD_CALLS} calls"
    )


def row_goal_and_wish():
    t0 = _truth(pos_m=np.array([0.3, -0.2, 0.1]), com_m=np.array([0.31, -0.19, 0.1]), yaw_rad=deg_to_rad(90.0))
    for name in ("walk_1m", "push_walk"):
        g = goal(name, t0)
        assert g["kind"] == "point", g
        err = math.dist(g["xy_m"], (-0.7, -0.2))
        assert err < GOAL_TOL, (name, g)
    assert goal("stand_up", t0) == {"kind": "none"}
    belly = 0.07
    want = {
        "stand_up": (0.0, 0.0, 0.0, 0.062), "walk_1m": (0.05, 0.0, 0.0, belly),
        "turn_90": (0.0, 0.0, 0.279, belly), "push_walk": (0.05, 0.0, 0.0, belly),
    }
    for name, w in want.items():
        got = wish(name, belly)
        assert isinstance(got, np.ndarray) and got.shape == (4,), (name, got)
        assert np.allclose(got, w, rtol=0.0, atol=1e-12), (name, got)
    return f"point goal {tuple(round(v, 6) for v in goal('walk_1m', t0)['xy_m'])}; 4 wishes as listed"


def row_push_seeded():
    signs = set()
    at = []
    for seed in PUSH_SEEDS:
        s = start("push_walk", seed)
        p = s["push"]
        assert s["pose"] == "stand" and s["belly_m"] == BELLY_M, s
        assert set(p) == {"at_s", "fx_n", "fy_n", "seconds"}, p
        assert 2.0 <= p["at_s"] <= 15.0, p
        assert p["fx_n"] in (3.0, -3.0) and p["fy_n"] == 0.0 and p["seconds"] == 0.2, p
        assert start("push_walk", seed) == s, f"seed {seed} not repeatable"
        signs.add(p["fx_n"])
        at.append(p["at_s"])
    assert signs == {3.0, -3.0}, signs
    for name in ("stand_up", "walk_1m", "turn_90"):
        assert start(name, 7)["push"] is None, name
    assert start("stand_up", 7)["pose"] == "sit"
    assert abs(start("walk_1m", 7, belly_mm=70.0)["belly_m"] - 0.07) < 1e-12
    n_pos = sum(1 for seed in PUSH_SEEDS if start("push_walk", seed)["push"]["fx_n"] > 0)
    return f"{len(PUSH_SEEDS)} seeds: at_s {min(at):.2f}…{max(at):.2f} s, +3 N {n_pos}×, −3 N {len(at) - n_pos}×"


def row_turn_hold():
    sp = spec("turn_90")
    g = {"kind": "yaw", "yaw_rad": 0.0}
    on = _truth(roll_rad=deg_to_rad(9.0), pitch_rad=deg_to_rad(-9.0))
    breaks = {
        "outside the window": _truth(yaw_rad=deg_to_rad(11.0)),
        "roll 11°": _truth(roll_rad=deg_to_rad(11.0)),
        "pitch 11°": _truth(pitch_rad=deg_to_rad(11.0)),
    }
    held = 0.0
    for k in range(HOLD_CALLS - 1):
        reason, held = judge(sp, g, on, held)
        assert reason is None, f"call {k + 1}: {reason!r}"
    reason, held = judge(sp, g, on, held)
    assert reason == "success", f"call {HOLD_CALLS}: {reason!r}"
    for label, broken in breaks.items():
        held = 0.0
        for _ in range(HOLD_CALLS - 1):
            _, held = judge(sp, g, on, held)
        reason, held = judge(sp, g, broken, held)
        assert reason is None and held == 0.0, f"{label}: {reason!r}, held {held}"
        at, _ = _calls_to_success(sp, g, on, 2 * HOLD_CALLS)
        assert at == HOLD_CALLS, f"after {label}: success at call {at}"
    at, held = _calls_to_success(sp, g, breaks["roll 11°"], 2 * HOLD_CALLS)
    assert at is None and held == 0.0, f"roll 11° held {2 * HOLD_CALLS} calls: success at {at}"
    return (
        f"99 calls → None, call {HOLD_CALLS} → success; one call outside the window, roll 11° or pitch 11° restarts "
        f"the count; {2 * HOLD_CALLS} calls at roll 11° never succeed"
    )


if __name__ == "__main__":
    sys.exit(run([row_specs, row_judge_table, row_stand_up_hold, row_yaw_wrap, row_goal_and_wish, row_push_seeded,
                 row_turn_hold]))
