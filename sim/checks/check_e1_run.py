# SPDX-License-Identifier: MIT
"""Stage 1: speed, replay without physics, the CAD-only picture, the command's errors and headless run.
Headless; every evaluate and every CLI call writes into a temp folder, never runs/. Exit 0 only when every row passes.

    cd <repository root> && sim/.venv/bin/python sim/checks/check_e1_run.py

Row 1 needs a quiet machine (load average below 4): on a busy machine it measures the other jobs too.
"""

import json
import math
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import mujoco  # noqa: E402

from _rows import run  # noqa: E402
from robot2sim import brains, replay, stage, tasks  # noqa: E402
from robot2sim.brains import Brain  # noqa: E402
from robot2sim.episode import EVAL_SEEDS, evaluate, run_episode  # noqa: E402
from robot2sim.paths import SIM  # noqa: E402
from robot2sim.scene import load_model  # noqa: E402
from robot2sim.world import World, twin_model  # noqa: E402

SPEED_MIN_X = 100.0
SPEED_TRIES = 3
SPEED_SEED = 1001
CAD_GROUP = 2
COLLISION_GROUP = 3
WIDTH, HEIGHT = 1280, 720
EVERY = 2
FILES = ("config.json", "metrics.jsonl", "motion.npz", "summary.json")
CHILD_TIMEOUT_S = 120
CONTROL_DT_S = 0.02
PNG_STEPS = 50  # the run row 3 pictures: one hold episode cut after 50 steps
VIS_GEOMS = 74
TWIN_EQUAL_ARRAYS = ("body_mass", "body_inertia", "body_ipos", "body_iquat")
TWIN_RUNS = (("scripted", "walk_1m"), ("random", "push_walk"))
TIMING_KEYS = ("wall_s", "brain_s")
TWIN_STATS = ("meaninertia", "meanmass", "meansize")
STATE_FIELDS = ("qpos", "qvel")
SHORT_STEPS = 5  # the run the --episode error case replays: one hold episode cut after 5 steps
CLI_SECONDS = 40.0  # --seconds of the headless CLI row: longer than stand_up's 30 s, so it cuts nothing


def _stop_after(n):
    """on_step that ends the episode after n control steps."""
    return lambda world: round(world.data.time / CONTROL_DT_S) < n


class Recorder(Brain):
    """Wraps a brain and keeps every action it returns, clipped as the episode clips it."""

    def __init__(self, inner):
        self.inner = inner
        self.actions = []

    def reset(self, body, task, seed):
        self.body = body
        self.inner.reset(body, task, seed)

    def act(self, obs):
        action = self.inner.act(obs)
        self.actions.append(np.clip(np.asarray(action, dtype=float), self.body.lower_rad, self.body.upper_rad))
        return action

    def observe(self, obs, action, next_obs, done):
        self.inner.observe(obs, action, next_obs, done)


def _physics_only_x(world, actions):
    """Robot time / wall time of the recorded actions through world.step alone, from the same reset."""
    start = tasks.start("walk_1m", SPEED_SEED)
    world.reset(SPEED_SEED, True, start["pose"], start["belly_m"])
    t = time.perf_counter()
    for action in actions:
        world.step(action)
    wall = time.perf_counter() - t
    return len(actions) * CONTROL_DT_S / wall


def row_speed_100x():
    world = World(twin=True)  # what evaluate and run --headless take
    recorder = Recorder(brains.make("scripted"))
    first, _trace = run_episode(world, recorder, "walk_1m", SPEED_SEED, True)
    assert first["reason"] == "success", first["reason"]
    best = None
    for _ in range(SPEED_TRIES):
        metrics, _trace = run_episode(world, brains.make("scripted"), "walk_1m", SPEED_SEED, True)
        speed = metrics["time_s"] / (metrics["wall_s"] - metrics["brain_s"])
        if best is None or speed > best[0]:
            best = (speed, metrics)
    physics = max(_physics_only_x(world, recorder.actions) for _ in range(SPEED_TRIES))
    speed, metrics = best
    load = ", ".join(f"{x:.2f}" for x in os.getloadavg())
    text = (
        f"{speed:.1f}x (time {metrics['time_s']:.2f} s / (wall {metrics['wall_s']:.3f} − brain "
        f"{metrics['brain_s']:.3f}) s), physics only {physics:.1f}x, {metrics['steps']} steps, load {load}"
    )
    assert speed >= SPEED_MIN_X, f"below {SPEED_MIN_X:g}x: {text}"
    return text


def _patched_mj_step(*_args, **_kwargs):
    raise AssertionError("mj_step called during replay")


def row_replay_no_physics():
    with tempfile.TemporaryDirectory() as tmp:
        evaluate("hold", "stand_up", [1, 2], False, out_dir=Path(tmp))
        with np.load(Path(tmp) / "motion.npz") as z:
            recorded = z["qpos"][z["episode"] == 1]
        real_step = mujoco.mj_step
        mujoco.mj_step = _patched_mj_step
        try:
            model, qpos, time_s = replay.load(Path(tmp), 1)
            shapes = [frame.shape for frame in replay.frames(model, qpos)]
        finally:
            mujoco.mj_step = real_step
    n = recorded.shape[0]
    assert np.array_equal(qpos, recorded) and qpos.dtype == recorded.dtype, "load changed the recorded qpos"
    assert time_s.shape == (n,), time_s.shape
    assert len(shapes) == math.ceil(n / EVERY), (len(shapes), n)
    assert set(shapes) == {(HEIGHT, WIDTH, 3)}, set(shapes)
    return f"episode 1: {n} rows equal the file; {len(shapes)} frames of {HEIGHT}×{WIDTH}×3 with mj_step raising"


def _frame(model, qpos, option):
    """The first frame of `frames` with cad_only swapped for `option`."""
    real = replay.cad_only
    replay.cad_only = lambda: option
    try:
        return next(iter(replay.frames(model, qpos[:1])))
    finally:
        replay.cad_only = real


def row_png_cad_only():
    groups = [int(g) for g in replay.cad_only().geomgroup]
    assert groups == [int(g == CAD_GROUP) for g in range(len(groups))], groups
    with tempfile.TemporaryDirectory() as tmp:
        evaluate("hold", "walk_1m", [1], False, out_dir=Path(tmp), on_step=_stop_after(PNG_STEPS))
        path = Path(tmp) / "pose.png"
        replay.png(Path(tmp), 0, path)
        image = np.asarray(Image.open(path).convert("RGB"))
        model, qpos, _time_s = replay.load(Path(tmp), 0)
    assert image.shape == (HEIGHT, WIDTH, 3), image.shape
    colours = len(np.unique(image.reshape(-1, 3), axis=0))
    assert colours > 1, "the PNG is one flat colour"
    cad = _frame(model, qpos, replay.cad_only())
    both = replay.cad_only()
    both.geomgroup[COLLISION_GROUP] = 1
    with_capsules = _frame(model, qpos, both)
    changed = int(np.count_nonzero(np.any(cad != with_capsules, axis=2)))
    assert changed > 0, "group 3 on changes nothing: the capsules are drawn anyway"
    return f"geomgroup {groups}; png {WIDTH}×{HEIGHT}, {colours} colours; group 3 on changes {changed} pixels"


def _cli(*args):
    return subprocess.run(
        [sys.executable, "-m", "robot2sim.run", *args], cwd=SIM, capture_output=True, text=True,
        timeout=CHILD_TIMEOUT_S,
    )


def row_cli_errors():
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        empty = tmp / "empty"
        empty.mkdir()
        short = tmp / "short"
        evaluate("hold", "walk_1m", [1], False, out_dir=short, on_step=_stop_after(SHORT_STEPS))
        base = ("--task", "walk_1m", "--headless", "--episodes", "1", "--out", str(tmp / "never"))
        cases = {
            "brain": ("--brain", "nobody", *base),
            "task": ("--brain", "hold", "--task", "fly", "--headless", "--out", str(tmp / "never")),
            "servo": ("--brain", "hold", *base, "--servo", "X999"),
            "body": ("--brain", "hold", *base, "--body", "v9"),
            "belly": ("--brain", "scripted", *base, "--belly-mm", "50"),
            "belly_range": ("--brain", "hold", *base, "--belly-mm", "200"),
            "out_not_empty": ("--brain", "hold", "--task", "walk_1m", "--headless", "--episodes", "1",
                              "--out", str(short)),
            "no_motion": ("--replay", str(empty)),
            "episode": ("--replay", str(short), "--episode", "1", "--png", str(tmp / "x.png")),
        }
        out = []
        for label, args in cases.items():
            child = _cli(*args)
            lines = child.stderr.splitlines()
            assert child.returncode == 2, f"{label}: exit {child.returncode}, stderr {child.stderr[-300:]!r}"
            assert len(lines) == 1 and "Traceback" not in child.stderr, f"{label}: stderr {child.stderr!r}"
            out.append(f"{label}: {lines[0]}")
        assert not (tmp / "never").exists(), "a refused run wrote its folder"
        assert sorted(p.name for p in short.iterdir()) == sorted(FILES), "the refused --out changed the run folder"
    return "exit 2, one line each — " + " | ".join(out)


def row_cli_headless():
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "run"
        child = _cli("--brain", "hold", "--task", "stand_up", "--headless", "--episodes", "2", "--seed", "5",
                     "--seconds", str(CLI_SECONDS), "--out", str(out))
        assert child.returncode == 0, f"exit {child.returncode}, stderr {child.stderr[-300:]!r}"
        missing = [name for name in FILES if not (out / name).is_file()]
        assert not missing, f"missing {missing}"
        summary = json.loads((out / "summary.json").read_text())
        assert summary["episodes"] == 2, summary["episodes"]
        cli = json.loads((out / "config.json").read_text())["cli"]
        assert cli == {"seconds": CLI_SECONDS, "headless": True}, cli
        seeds = [json.loads(line)["seed"] for line in (out / "metrics.jsonl").read_text().splitlines()]
        assert seeds == [5, 6], seeds
        png = Path(tmp) / "ep1.png"
        child2 = _cli("--replay", str(out), "--episode", "1", "--png", str(png))
        assert child2.returncode == 0, f"replay exit {child2.returncode}, stderr {child2.stderr[-300:]!r}"
        assert png.is_file(), "no PNG"
        lines = child.stdout.splitlines()
    return f"exit 0, 4 files, 2 episodes (seeds {seeds}), config cli {cli}, png written; stdout: {' / '.join(lines)}"


def row_twin_equal():
    full = load_model("v0")
    twin = twin_model("v0", load_model("v0"))
    for name in ("nq", "nv", "nu", "nbody"):
        assert getattr(full, name) == getattr(twin, name), (name, getattr(full, name), getattr(twin, name))
    for name in TWIN_EQUAL_ARRAYS:
        assert np.array_equal(getattr(full, name), getattr(twin, name)), f"{name} differs"
    assert full.ngeom - twin.ngeom == VIS_GEOMS, (full.ngeom, twin.ngeom)
    names = [mujoco.mj_id2name(twin, mujoco.mjtObj.mjOBJ_GEOM, g) or "" for g in range(twin.ngeom)]
    assert not [n for n in names if n.endswith("_vis")], "a _vis geom is left in the twin"
    worst = worst64 = 0.0
    out = []
    for brain, task in TWIN_RUNS:
        seed = EVAL_SEEDS[0]
        w_full, w_twin = World(), World(twin=True)
        m_full, t_full = run_episode(w_full, brains.make(brain), task, seed, False, brain_seed=0)
        m_twin, t_twin = run_episode(w_twin, brains.make(brain), task, seed, False, brain_seed=0)
        for name in TWIN_STATS:  # after the randomised reset inside run_episode
            a, b = getattr(w_full.model.stat, name), getattr(w_twin.model.stat, name)
            assert a == b, f"{brain} {task}: stat.{name} {a} full, {b} twin"
        assert t_full.shape == t_twin.shape, (task, t_full.shape, t_twin.shape)
        worst = max(worst, float(np.max(np.abs(t_full.astype(float) - t_twin.astype(float)))))
        assert np.array_equal(t_full, t_twin), f"{brain} {task}: traces differ"
        same = {k: v for k, v in m_full.items() if k not in TIMING_KEYS}
        assert same == {k: v for k, v in m_twin.items() if k not in TIMING_KEYS}, (m_full, m_twin)
        for name in STATE_FIELDS:  # float64, after the same steps
            a, b = getattr(w_full.data, name), getattr(w_twin.data, name)
            worst64 = max(worst64, float(np.max(np.abs(a - b))))
            assert np.array_equal(a, b), f"{brain} {task}: data.{name} differs in float64"
        out.append(f"{brain}/{task} {m_full['steps']} steps {m_full['reason']}")
    assert worst == 0.0 and worst64 == 0.0, (worst, worst64)
    return (
        f"nq {full.nq}, nv {full.nv}, nu {full.nu}, nbody {full.nbody} and mass, inertia, ipos, iquat equal; "
        f"ngeom {full.ngeom} → {twin.ngeom}, no _vis; nominal=False seed {EVAL_SEEDS[0]}: {', '.join(out)}, "
        f"stat {', '.join(TWIN_STATS)} equal after the reset, identical trace and metrics, worst difference {worst}; "
        f"float64 qpos, qvel at the end equal, worst difference {worst64}"
    )


SEVEN_SEGMENTS = {  # the standard 7-segment digits, a top, b top right, c bottom right, d bottom, e, f, g middle
    0: "abcdef", 1: "bc", 2: "abdeg", 3: "abcdg", 4: "bcfg", 5: "acdfg", 6: "acdefg", 7: "abc", 8: "abcdefg",
    9: "abcdfg",
}
STAGE_SEED = 1003  # a jittered start (nominal False), so the goal is not at the nominal (0, 1)
STAGE_ROW = 400  # a row the board is read at: 8 s into the scripted walk
STAGE_EPISODE = 1  # the stage run's second episode: the board's bottom line must read 01
CORNER_PX = 8
DARK_SUM = 30  # an RGB sum below this is black (the old picture's corners: 0; the room's dark slab face: about 50)
GOAL_GREEN_MIN_PX = 200


def _lit_digits(model, line):
    """What the board's line shows, read back from the segment colours of the model: a digit per slot, " " dark."""
    out = ""
    for k in range(stage.DIGITS[line]):
        lit = ""
        for seg in "abcdefg":
            g = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"stage_{line}{k}_{seg}")
            assert g >= 0, f"no segment {line}{k}_{seg}"
            if np.allclose(model.geom_rgba[g], stage.SEGMENT_ON):
                lit += seg
            else:
                assert np.allclose(model.geom_rgba[g], stage.SEGMENT_OFF), (line, k, seg, model.geom_rgba[g])
        digit = [str(d) for d, segs in SEVEN_SEGMENTS.items() if segs == lit]
        assert lit == "" or len(digit) == 1, f"line {line} slot {k}: lit {lit!r} is no digit"
        out += digit[0] if digit else " "
    return out


def _stage_run(tmp):
    evaluate("scripted", "walk_1m", [STAGE_SEED - 1, STAGE_SEED], False, out_dir=tmp, brain_seed=0)
    return tmp


def row_stage_digits():
    for d, segs in SEVEN_SEGMENTS.items():
        assert stage.segments(d) == segs, (d, stage.segments(d), segs)
    limit = tasks.spec("walk_1m").limit_s
    seen = []
    with tempfile.TemporaryDirectory() as tmp:
        run_dir = _stage_run(Path(tmp))
        for episode, row in ((STAGE_EPISODE, STAGE_ROW), (STAGE_EPISODE, -1), (0, 0)):  # -1: the clock stops at success
            model, qpos, time_s = replay.load(run_dir, episode)
            board = replay.board(run_dir, episode, time_s)
            k = row % len(qpos)
            t = k * CONTROL_DT_S  # the row's time from its index, not from the file
            assert abs(float(time_s[k]) - t) < 1e-4, (k, time_s[k])
            want = (f"{max(0, math.ceil(limit - t)):02d}", f"{episode:02d}".rjust(stage.DIGITS["bottom"]))
            frames = replay.frames(model, qpos[k:k + 1], board=board[k:k + 1])
            next(frames)  # renders row k; the board's segments stay as drawn
            got = (_lit_digits(model, "top"), _lit_digits(model, "bottom"))
            frames.close()
            assert got == want, (episode, k, got, want)
            seen.append(f"episode {episode} row {k} ({t:.2f} s): top {got[0]!r} bottom {got[1]!r}")
    for line, value in (("top", 10 ** stage.DIGITS["top"]), ("bottom", 10 ** stage.DIGITS["bottom"])):
        try:
            stage.show(model, *((value, 0) if line == "top" else (0, value)))
        except ValueError as e:
            assert str(value) in str(e) and line in str(e), e
        else:
            raise AssertionError(f"{line} {value} was drawn, not refused")
    return (
        f"digit → segments equal the 7-segment table for 0…9; limit {limit:g} s: {'; '.join(seen)} "
        f"(top = ceil(limit − t), bottom = episode index); top {10 ** stage.DIGITS['top']} and bottom "
        f"{10 ** stage.DIGITS['bottom']} raise ValueError"
    )


def row_stage_goal_tile():
    with tempfile.TemporaryDirectory() as tmp:
        run_dir = _stage_run(Path(tmp))
        out = []
        for episode, seed in ((0, STAGE_SEED - 1), (1, STAGE_SEED)):
            model, _qpos, _time_s = replay.load(run_dir, episode)
            world = World(twin=True)
            st = tasks.start("walk_1m", seed)
            world.reset(seed, False, st["pose"], st["belly_m"])
            truth0 = world.truth()
            goal = np.array(tasks.goal("walk_1m", truth0)["xy_m"])
            tile = model.geom_pos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "stage_goal")][:2]
            start = model.body_pos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "stage_start")][:2]
            err = float(np.linalg.norm(tile - goal))
            err0 = float(np.linalg.norm(start - truth0["pos_m"][:2]))
            assert err < 1e-5 and err0 < 1e-5, (seed, tile, goal, start, truth0["pos_m"][:2])
            out.append(f"seed {seed} goal ({goal[0]:+.4f}, {goal[1]:+.4f}) m: tile off {err:.1e} m, start tile off "
                       f"{err0:.1e} m")
    return "; ".join(out)


def row_stage_room():
    physics = {"full": World().model, "twin": World(twin=True).model, "file": load_model("v0")}
    names = [mujoco.mj_id2name(physics["file"], mujoco.mjtObj.mjOBJ_GEOM, g) for g in range(physics["file"].ngeom)]
    for label, m in physics.items():
        geoms = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "" for g in range(m.ngeom)]
        assert not [n for n in geoms if n.startswith("stage_")], f"{label}: a stage geom in the physics model"
        assert m.nlight == physics["file"].nlight, (label, m.nlight)
    assert physics["file"].ngeom == 131, physics["file"].ngeom
    with tempfile.TemporaryDirectory() as tmp:
        run_dir = _stage_run(Path(tmp))
        display, qpos, time_s = replay.load(run_dir, 0)
        shown = [mujoco.mj_id2name(display, mujoco.mjtObj.mjOBJ_GEOM, g) for g in range(display.ngeom)]
        assert shown[:len(names)] == names, "the display model reorders or drops a physics geom"
        path = Path(tmp) / "room.png"
        replay.png(run_dir, 0, path)
        image = np.asarray(Image.open(path).convert("RGB")).astype(int)
    c = CORNER_PX
    corners = [image[:c, :c], image[:c, -c:], image[-c:, :c], image[-c:, -c:]]
    sums = [int(x.sum(axis=2).mean()) for x in corners]
    assert min(sums) > DARK_SUM, f"a corner is black: RGB sums {sums}"
    blue = np.array(stage.PRINTED_RGBA[:3]) * 255.0
    robot = int(np.count_nonzero(np.linalg.norm(image - blue, axis=2) < 80.0))
    green = int(np.count_nonzero((image[..., 1] > 170) & (image[..., 0] < 140) & (image[..., 2] < 140)))
    amber = int(np.count_nonzero((image[..., 0] > 170) & (image[..., 1] > 80) & (image[..., 1] < 170)
                                 & (image[..., 2] < 90)))
    colours = len(np.unique(image.reshape(-1, 3), axis=0))
    total = image.shape[0] * image.shape[1]
    assert 0 < robot < total // 10, f"robot-blue pixels {robot} of {total}"
    assert green > GOAL_GREEN_MIN_PX and amber > 0, (green, amber)
    return (
        f"physics models (World full, twin, body file) have no stage_ geom, ngeom {physics['file'].ngeom}, "
        f"display model keeps the {len(names)} physics geoms first in order (+{display.ngeom - len(names)} stage); "
        f"png corners RGB sums {sums}, {colours} colours, robot-blue {robot} px, goal-green {green} px, "
        f"start-amber {amber} px of {total}"
    )


if __name__ == "__main__":
    sys.exit(run([
        row_speed_100x, row_replay_no_physics, row_png_cad_only, row_cli_errors, row_cli_headless, row_twin_equal,
        row_stage_digits, row_stage_goal_tile, row_stage_room,
    ]))
