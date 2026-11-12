# SPDX-License-Identifier: MIT
"""Stage 1: brains and the episode loop. Headless; every evaluate writes into a temp folder, never runs/.
Exit 0 only when every row passes.

    cd <repository root> && sim/.venv/bin/python sim/checks/check_e1_episode.py

Row 4 runs 160 episodes (a few minutes).
"""

import ast
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile
from collections import Counter
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _rows import run  # noqa: E402
from robot2sim import brains, episode, randomise  # noqa: E402
from robot2sim.brains import Brain  # noqa: E402
from robot2sim.episode import EVAL_SEEDS, METRIC_KEYS, REASONS, evaluate, run_episode  # noqa: E402
from robot2sim.paths import SIM  # noqa: E402
from robot2sim.sensors import SENSOR_CHANNELS  # noqa: E402
from robot2sim.tasks import TASKS, spec  # noqa: E402
from robot2sim.world import World  # noqa: E402

SPY_STEPS = 50
SPY_JOINT = 1  # RF femur
SPY_MOVE_AT = 10  # the step at which the spy drives its joint past the upper limit
SPY_MOVED_RAD = 0.01  # a real move, well above the encoder noise (σ ≤ 0.002 rad)
SPY_SETTLE_STEPS = 6  # latency (≤ 3 steps) plus a few steps of motion
CTRL_TOL_RAD = 0.02  # joint range ± the largest mounting offset (1° = 0.01745 rad)
OVER_LIMIT_RAD = 10.0
OVER_LIMIT_STEPS = 50
BAD_AT_ACT = 6
NAN_AT_STEP = 10
NAN_QPOS_INDEX = 7
ABORT_AT_STEP = 20
HOLD_PENETRATION_MAX_MM = 0.5
CONTROL_DT_S = 0.02
EIGHT_MODULES = ("servo", "contacts", "randomise", "world", "sensors", "tasks", "brains", "episode")
STATE_FIELDS = ("qpos", "qvel")
MUJOCO_LOG = "MUJOCO_LOG.TXT"
FILES = ("config.json", "metrics.jsonl", "motion.npz", "summary.json")
CHILD_TIMEOUT_S = 120
SEED_RUN_SEEDS = (1, 2, 3)  # the two evaluate calls with one brain seed root
TIMING_KEYS = ("wall_s", "brain_s")
SAME_SECOND = datetime(2026, 11, 11, 12, 0, 0)
FOLDER_STEPS = 2
RAISING_STDERR_SCRIPT = f"""
import sys
sys.path.insert(0, sys.argv[1])
import numpy as np
from robot2sim import brains
from robot2sim.episode import run_episode
from robot2sim.world import World


class Raising:
    def write(self, text):
        raise OSError("stderr is gone")

    def flush(self):
        raise OSError("stderr is gone")


def poison(world):
    if round(world.data.time / {CONTROL_DT_S}) == {NAN_AT_STEP}:
        world.data.qpos[{NAN_QPOS_INDEX}] = np.nan
    return True


sys.stderr = Raising()
metrics, _trace = run_episode(World(), brains.make("hold"), "turn_90", 1, False, on_step=poison)
sys.stderr = sys.__stderr__  # a raising stderr at shutdown is exit 120, whatever the episode did
print(metrics["reason"])
"""


def _steps(world):
    return int(round(world.data.time / CONTROL_DT_S))


def _stop_after(n):
    """on_step that ends the episode after n control steps."""
    return lambda world: _steps(world) < n


def _episode_counter(action):
    """on_step that tracks the episode index (data.time falls at a reset) and calls action(episode, step, world)."""
    state = {"episode": 0, "last": -1}

    def on_step(world):
        step = _steps(world)
        if step < state["last"]:
            state["episode"] += 1
        state["last"] = step
        return action(state["episode"], step, world)
    return on_step


def _load_run(out):
    out = Path(out)
    for name in FILES:
        assert (out / name).is_file(), f"{name} missing"
    config = json.loads((out / "config.json").read_text())
    summary = json.loads((out / "summary.json").read_text())
    metrics = [json.loads(line) for line in (out / "metrics.jsonl").read_text().splitlines()]
    with np.load(out / "motion.npz") as z:
        motion = {k: z[k] for k in z.files}
    return config, summary, metrics, motion


class Spy(Brain):
    """Holds the first servo_pos, then drives one joint past its upper limit; records everything it is given,
    the seed of every reset included."""

    def __init__(self):
        self.seeds = []

    def reset(self, body, task, seed):
        self.seeds.append(seed)
        self.body = body
        self.acts, self.observed, self.clipped = [], [], []
        self.target = None

    def act(self, obs):
        self.acts.append(obs)
        if self.target is None:
            self.target = obs["servo_pos"].astype(float)
        if len(self.acts) == SPY_MOVE_AT:
            self.target = self.target.copy()
            self.target[SPY_JOINT] = self.body.upper_rad[SPY_JOINT] + 1.0
        self.clipped.append(np.clip(self.target, self.body.lower_rad, self.body.upper_rad))
        return self.target.copy()

    def observe(self, obs, action, next_obs, done):
        self.observed.append((obs, action, next_obs))


class Broken(Brain):
    """Holds, then returns `bad` at act number `at`."""

    def __init__(self, bad, at=BAD_AT_ACT):
        self.bad, self.at = bad, at

    def reset(self, body, task, seed):
        self.n = 0

    def act(self, obs):
        self.n += 1
        if self.n == self.at:
            return self.bad
        return obs["servo_pos"].astype(float)


class OverLimit(Brain):
    def reset(self, body, task, seed):
        self.target = body.upper_rad + OVER_LIMIT_RAD

    def act(self, obs):
        return self.target.copy()


def row_brain_sees_sensors_only():
    world = World()
    spy = Spy()
    metrics, trace = run_episode(world, spy, "walk_1m", 1, False, on_step=_stop_after(SPY_STEPS))
    assert metrics["reason"] == "aborted" and metrics["steps"] == SPY_STEPS, metrics
    assert len(spy.acts) == len(spy.observed) == SPY_STEPS, (len(spy.acts), len(spy.observed))
    keys = set(SENSOR_CHANNELS)
    for k, (obs, action, next_obs) in enumerate(spy.observed):
        assert set(obs) == keys and set(next_obs) == keys, f"step {k}: {sorted(set(obs) ^ keys)}"
        assert obs is spy.acts[k], f"step {k}: observe got another obs than act"
        if k + 1 < SPY_STEPS:
            assert next_obs is spy.acts[k + 1], f"step {k}: next_obs is not the next act's obs"
        assert np.array_equal(action, spy.clipped[k]), f"step {k}: observe got an unclipped action"
        assert np.array_equal(next_obs["last_command"], spy.clipped[k].astype(np.float32)), f"step {k} last_command"
    moved = [
        abs(float(nxt["servo_pos"][SPY_JOINT] - obs["servo_pos"][SPY_JOINT]))
        for obs, _a, nxt in spy.observed[SPY_MOVE_AT - 1:SPY_MOVE_AT - 1 + SPY_SETTLE_STEPS]
    ]
    assert max(moved) > SPY_MOVED_RAD, f"servo_pos of joint {SPY_JOINT} did not move: {moved}"
    assert trace.shape == (SPY_STEPS + 1, 25) and trace.dtype == np.float32, (trace.shape, trace.dtype)
    seeds = _spy_seeds(world)
    root = _same_root_metrics()
    return (
        f"{SPY_STEPS} steps: 10 keys every obs, next_obs is the next obs, last_command = clipped action, "
        f"joint {SPY_JOINT} servo_pos moved up to {max(moved):.4f} rad per step after the command; "
        f"{seeds}; {root}"
    )


def _spy_seeds(world):
    """Without brain_seed the brain never gets the episode seed: not on any evaluation seed, not twice the same."""
    spy = Spy()
    for seed in EVAL_SEEDS:
        run_episode(world, spy, "push_walk", seed, False, on_step=_stop_after(1))
    run_episode(world, spy, "push_walk", EVAL_SEEDS[0], False, on_step=_stop_after(1))
    for seed, got in zip(EVAL_SEEDS, spy.seeds):
        assert got != seed, f"episode {seed}: the brain got the episode seed"
        mine = randomise.jitter(got)["joint_offsets_rad"]
        assert not np.array_equal(mine, randomise.jitter(seed)["joint_offsets_rad"]), f"episode {seed}: offsets"
    assert spy.seeds[0] != spy.seeds[-1], f"two runs of seed {EVAL_SEEDS[0]} gave the brain {spy.seeds[0]} twice"
    return (f"{len(EVAL_SEEDS)} evaluation seeds: brain seed ≠ episode seed, its jitter ≠ the episode's; "
            "a rerun: new seed")


def _same_root_metrics():
    """Two evaluate calls of the random brain with brain_seed 0: equal metrics, the root in config.json."""
    runs = []
    for _ in range(2):
        with tempfile.TemporaryDirectory() as tmp:
            evaluate("random", "walk_1m", SEED_RUN_SEEDS, False, out_dir=Path(tmp), brain_seed=0)
            config, _s, metrics, _m = _load_run(tmp)
        assert config["arguments"]["brain_seed"] == 0, config["arguments"]
        runs.append([{k: v for k, v in m.items() if k not in TIMING_KEYS} for m in metrics])
    assert runs[0] == runs[1], "brain_seed 0 twice: metrics differ"
    return f"evaluate random brain_seed 0 twice: equal metrics over {len(SEED_RUN_SEEDS)} seeds, config root 0"


def row_deterministic():
    runs = []
    for seed in (7, 7, 8):
        metrics, trace = run_episode(World(), brains.make("random"), "walk_1m", seed, False, brain_seed=0)
        runs.append((metrics, trace))
    (m1, t1), (m2, t2), (m3, t3) = runs
    assert np.array_equal(t1, t2), "seed 7 traces differ"
    same = {k: v for k, v in m1.items() if k not in ("wall_s", "brain_s")}
    assert same == {k: v for k, v in m2.items() if k not in ("wall_s", "brain_s")}, (m1, m2)
    assert t1.shape != t3.shape or not np.array_equal(t1, t3), "seed 8 gives the seed-7 trace"
    return f"seed 7 twice: {m1['steps']} steps, {m1['reason']}, identical; seed 8: {m3['steps']} steps, {m3['reason']}"


def row_files():
    with tempfile.TemporaryDirectory() as tmp:
        summary = evaluate("hold", "walk_1m", [1, 2, 3], False, out_dir=Path(tmp))
        config, written, metrics, motion = _load_run(tmp)
    assert written == summary, "summary.json differs from the returned dict"
    assert len(metrics) == 3, len(metrics)
    for line in metrics:
        assert tuple(line) == METRIC_KEYS, f"keys {tuple(line)}"
        assert line["reason"] in REASONS, line["reason"]
    assert set(motion) == {"qpos", "episode", "time_s", "body", "rate_hz"}, sorted(motion)
    n = motion["qpos"].shape[0]
    assert motion["qpos"].shape == (n, 25) and motion["qpos"].dtype == np.float32, motion["qpos"].shape
    assert motion["episode"].shape == (n,) and motion["time_s"].shape == (n,), "first dimensions differ"
    assert n == sum(line["steps"] + 1 for line in metrics), (n, [line["steps"] for line in metrics])
    assert str(motion["body"]) == "v0" and float(motion["rate_hz"]) == 50.0, (motion["body"], motion["rate_hz"])
    assert summary["episodes"] == 3, summary["episodes"]
    assert summary["eval_seeds"] is False, summary["eval_seeds"]
    assert all(r in REASONS for r in summary["reasons"]), summary["reasons"]
    assert {"arguments", "ranges", "eye", "imu", "servo"} <= set(config), sorted(config)
    reasons = {r: c for r, c in summary["reasons"].items() if c}
    return f"4 files, 3 metric lines × {len(METRIC_KEYS)} keys, motion {n} rows, reasons {reasons}, eval_seeds False"


def row_baselines_zero():
    out = []
    worst_random = 0.0
    worst_hold = 0.0
    hold_steps = []
    for name in ("hold", "random"):
        for task in TASKS:
            with tempfile.TemporaryDirectory() as tmp:
                summary = evaluate(name, task, EVAL_SEEDS, False, out_dir=Path(tmp), brain_seed=0)
                _c, _s, metrics, _m = _load_run(tmp)
            pen = max(line["max_penetration_mm"] for line in metrics)
            if name == "hold":
                worst_hold = max(worst_hold, pen)
                limit_steps = round(spec(task).limit_s / CONTROL_DT_S)
                timeout_steps = {line["steps"] for line in metrics if line["reason"] == "timeout"}
                assert timeout_steps <= {limit_steps}, f"hold {task}: timeout after {timeout_steps}, not {limit_steps}"
                hold_steps.append(f"{task} {','.join(str(n) for n in sorted(timeout_steps))}")
            else:
                worst_random = max(worst_random, pen)
            counts = Counter(line["reason"] for line in metrics)
            out.append(f"{name}/{task} " + ",".join(f"{r}{c}" for r, c in sorted(counts.items())))
            assert summary["episodes"] == len(EVAL_SEEDS), summary["episodes"]
            assert summary["successes"] == 0, f"{name} {task} {summary['successes']} successes: {counts}"
    assert worst_hold <= HOLD_PENETRATION_MAX_MM, f"hold penetration {worst_hold:.3f} mm"
    return (
        "; ".join(out) + f"; hold timeout steps {'; '.join(hold_steps)}; hold pen ≤ {worst_hold:.3f} mm, "
        f"random pen ≤ {worst_random:.3f} mm"
    )


def row_bad_action():
    world = World()
    n = world.n_servos
    bad = {"nan": np.full(n, np.nan), "inf": np.full(n, np.inf), "17": np.zeros(n - 1)}
    for label, action in bad.items():
        metrics, trace = run_episode(world, Broken(action), "walk_1m", 1, False)
        assert metrics["reason"] == "bad_action" and metrics["steps"] == BAD_AT_ACT - 1, (label, metrics)
        assert trace.shape == (BAD_AT_ACT, 25), (label, trace.shape)
    worst = []

    def watch(w):
        ctrl = w.data.ctrl
        worst.append(max(float(np.max(ctrl - w.upper_rad)), float(np.max(w.lower_rad - ctrl))))
        return _steps(w) < OVER_LIMIT_STEPS

    metrics, _trace = run_episode(world, OverLimit(), "walk_1m", 1, False, on_step=watch)
    assert metrics["reason"] in ("aborted", "fall", "cutout"), metrics["reason"]
    assert max(worst) <= CTRL_TOL_RAD, f"ctrl {max(worst):.4f} rad outside the joint range"
    return (
        f"NaN, inf, 17 numbers → bad_action at steps 5; upper + 10 rad: {metrics['steps']} steps, "
        f"{metrics['reason']}, ctrl at most {max(worst):.4f} rad past a limit"
    )


def row_sim_error():
    def poison(ep, step, world):
        if ep == 0 and step == NAN_AT_STEP:
            world.data.qpos[NAN_QPOS_INDEX] = np.nan
        return True

    captured = io.StringIO()
    with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stderr(captured):
        evaluate("hold", "turn_90", [1, 2], False, out_dir=Path(tmp), on_step=_episode_counter(poison))
        _c, _s, metrics, motion = _load_run(tmp)
    text = captured.getvalue()
    assert "MuJoCo" in text and "QPOS" in text, f"stderr during the NaN: {text!r}"
    first, second = metrics
    assert first["reason"] == "sim_error" and first["steps"] == NAN_AT_STEP + 1, first
    assert second["reason"] != "sim_error" and second["reason"] in REASONS, second
    trace2 = motion["qpos"][motion["episode"] == 1]
    assert trace2.shape[0] == second["steps"] + 1 and np.all(np.isfinite(trace2)), trace2.shape
    log = Path(os.getcwd()) / MUJOCO_LOG
    assert not log.exists(), f"{log} written"
    child = subprocess.run(
        [sys.executable, "-c", RAISING_STDERR_SCRIPT, str(SIM)], capture_output=True, text=True,
        timeout=CHILD_TIMEOUT_S,
    )
    assert child.returncode == 0 and child.stdout.split() == ["sim_error"], (
        f"raising stderr: exit {child.returncode}, stdout {child.stdout!r}, stderr {child.stderr[-300:]!r}"
    )
    return (
        f"first: sim_error at step {first['steps']}; second: {second['reason']} after {second['steps']} steps, "
        f"finite trace; warning on stderr names MuJoCo and QPOS; no {MUJOCO_LOG} in {os.getcwd()}; "
        f"a raising stderr: child exit 0, sim_error"
    )


def _writes_state(target):
    """True when an assignment target writes into MuJoCo state: x.qpos, x.qvel, or a subscript or slice of one."""
    while isinstance(target, ast.Subscript):
        target = target.value
    if isinstance(target, (ast.Tuple, ast.List)):
        return any(_writes_state(t) for t in target.elts)
    return isinstance(target, ast.Attribute) and target.attr in STATE_FIELDS


def row_state_writes():
    found = []
    for module in EIGHT_MODULES:
        path = SIM / "robot2sim" / f"{module}.py"
        tree = ast.parse(path.read_text(), str(path))
        scopes = {}
        for cls in (n for n in tree.body if isinstance(n, ast.ClassDef)):
            for fn in (n for n in cls.body if isinstance(n, ast.FunctionDef)):
                for node in ast.walk(fn):
                    scopes[id(node)] = f"{cls.name}.{fn.name}"
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign):
                targets = node.targets
            elif isinstance(node, (ast.AugAssign, ast.AnnAssign)):
                targets = [node.target]
            else:
                continue
            if any(_writes_state(t) for t in targets):
                found.append((module, scopes.get(id(node), "<module>"), node.lineno))
    outside = [f for f in found if (f[0], f[1]) != ("world", "World.reset")]
    assert not outside, f"state writes outside World.reset: {outside}"
    assert found, "no state write found at all: the walk sees nothing"
    lines = ", ".join(str(f[2]) for f in found)
    return f"{len(EIGHT_MODULES)} modules: {len(found)} writes, all in World.reset (world.py lines {lines})"


def row_aborted():
    def stop(ep, step, world):
        return not (ep == 1 and step == ABORT_AT_STEP)

    with tempfile.TemporaryDirectory() as tmp:
        summary = evaluate("hold", "turn_90", [1, 2, 3], False, out_dir=Path(tmp), on_step=_episode_counter(stop))
        _c, written, metrics, motion = _load_run(tmp)
    assert summary["episodes"] == 2 and written == summary, summary
    assert len(metrics) == 2, len(metrics)
    assert metrics[0]["reason"] not in ("aborted",) and metrics[1]["reason"] == "aborted", metrics
    assert metrics[1]["steps"] == ABORT_AT_STEP, metrics[1]["steps"]
    assert motion["qpos"].shape[0] == sum(m["steps"] + 1 for m in metrics), motion["qpos"].shape
    assert sum(summary["reasons"].values()) == 2 and summary["reasons"]["aborted"] == 1, summary["reasons"]
    return f"2 episodes: {metrics[0]['reason']} + aborted at step {metrics[1]['steps']}, 4 files load"


class _FixedClock:
    """Stands in for episode.datetime: every evaluate call starts in the same second."""

    @staticmethod
    def now():
        return SAME_SECOND


def row_run_folders():
    real_runs, real_clock = episode.RUNS, episode.datetime
    with tempfile.TemporaryDirectory() as tmp:
        episode.RUNS, episode.datetime = Path(tmp), _FixedClock
        try:
            dirs = [
                Path(evaluate("hold", "walk_1m", [1], False, on_step=_stop_after(FOLDER_STEPS))["run_dir"])
                for _ in range(2)
            ]
        finally:
            episode.RUNS, episode.datetime = real_runs, real_clock
        assert dirs[0] != dirs[1], f"both runs in {dirs[0]}"
        for d in dirs:
            assert d.parent == Path(tmp), d
            lines = (d / "metrics.jsonl").read_text().splitlines()
            assert len(lines) == 1 and json.loads(lines[0])["steps"] == FOLDER_STEPS, (d, lines)
        names = [d.name for d in dirs]
        full = Path(tmp) / "full"
        full.mkdir()
        (full / "keep.txt").write_text("keep\n")
        try:
            evaluate("hold", "walk_1m", [1], False, out_dir=full, on_step=_stop_after(FOLDER_STEPS))
            raise AssertionError("evaluate wrote into a non-empty out_dir")
        except ValueError as e:
            refused = str(e)
        left = sorted(p.name for p in full.iterdir())
        assert left == ["keep.txt"] and (full / "keep.txt").read_text() == "keep\n", left
    return f"same second: {names[0]} and {names[1]}, each with its own metrics.jsonl; non-empty out_dir: {refused}"


if __name__ == "__main__":
    sys.exit(run([
        row_brain_sees_sensors_only, row_deterministic, row_files, row_baselines_zero, row_bad_action,
        row_sim_error, row_state_writes, row_aborted, row_run_folders,
    ]))
