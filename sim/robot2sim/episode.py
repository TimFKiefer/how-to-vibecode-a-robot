# SPDX-License-Identifier: MIT
"""The episode loop and the evaluation: the only place that sees true state. Metrics, motion file, run folder.

A brain gets only the sensor dict. One control step, in this order: act (not 18 finite numbers → bad_action, else
clipped to the limits), start the push when its time has come, world.step, physics check (MuJoCo's bad-qpos,
bad-qvel, bad-qacc counters or a non-finite qpos → sim_error), one truth, judge, read the next obs and let the brain
observe, on_step (False → aborted). An exception from the brain is not caught.

The brain's `reset` gets a brain seed, never the episode seed: with the episode seed a brain could recompute the
hidden joint offsets, the parameter draw, the noise streams and the push. Without one it is secrets.randbits(32).

At import, MuJoCo's warning text goes to stderr instead of MUJOCO_LOG.TXT in the working directory; the detection
reads data.warning, never the text.
"""

import json
import secrets
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

import mujoco

from robot2sim import brains, randomise, sensors, tasks
from robot2sim.brains import Brain, BodySpec
from robot2sim.paths import RUNS, SIM, read_json
from robot2sim.servo import DEFAULT_SERVO, SERVOS, SUBSTEPS
from robot2sim.units import m_to_mm
from robot2sim.world import World

REASONS = ("success", "fall", "cutout", "timeout", "bad_action", "sim_error", "aborted")
METRIC_KEYS = (
    "episode", "seed", "task", "brain", "body", "servo", "nominal", "success", "reason", "steps", "robot_minutes",
    "time_s", "wall_s", "brain_s", "belly_min_mm", "belly_mean_mm", "belly_max_mm", "cutouts", "self_hit_steps",
    "airborne_steps", "max_penetration_mm", "ik_failures",
)
EVAL_SEEDS: tuple[int, ...] = tuple(int(s) for s in read_json(SIM / "seeds" / "eval.json")["eval"])
ROBOT_MINUTE_STEPS = 3000
RATE_HZ = 50.0
BAD_PHYSICS = (mujoco.mjtWarning.mjWARN_BADQPOS, mujoco.mjtWarning.mjWARN_BADQVEL, mujoco.mjtWarning.mjWARN_BADQACC)
_BAD_PHYSICS_IDS = np.array([int(w) for w in BAD_PHYSICS])  # rows of data.warning.number, read as one array


def _warning_to_stderr(message: str) -> None:
    """Never raises: an exception inside MuJoCo's C callback aborts the whole process (libc++abi, exit 134)."""
    try:
        sys.stderr.write(f"WARNING: MuJoCo: {message}\n")
    except BaseException:  # noqa: BLE001 — losing one warning line beats a dead process
        pass


mujoco.set_mju_user_warning(_warning_to_stderr)


def _physics_broken(data) -> bool:
    return bool(np.any(data.warning.number[_BAD_PHYSICS_IDS] > 0)) or not np.all(np.isfinite(data.qpos))


def run_episode(world: World, brain: Brain, task: str, seed: int, nominal: bool, belly_mm: float | None = None,
                on_step=None, brain_seed: int | None = None) -> tuple[dict, np.ndarray]:
    """(metrics without "episode", qpos trace of shape (steps + 1, 25), float32). brain_seed None: a fresh one."""
    if brain_seed is None:
        brain_seed = secrets.randbits(32)
    wall0 = time.perf_counter()
    sp = tasks.spec(task)
    start = tasks.start(task, seed, belly_mm)
    world.reset(seed, nominal, start["pose"], start["belly_m"])
    truth = world.truth()
    goal = tasks.goal(task, truth)
    eyes = sensors.Sensors(world, seed)
    eyes.reset(goal, tasks.wish(task, start["belly_m"]))
    obs = eyes.read(world.home_rad)
    lower, upper = world.lower_rad.copy(), world.upper_rad.copy()
    brain.reset(BodySpec(world.n_servos, lower.copy(), upper.copy()), task, brain_seed)
    control_dt = float(world.model.opt.timestep) * SUBSTEPS
    push = start["push"]
    trace = [truth["qpos"]]
    bellies = [truth["belly_m"]]
    depth = truth["penetration_mm"]
    self_hit = airborne = steps = 0
    brain_s = held = 0.0
    reason = None
    while reason is None:
        t = time.perf_counter()
        action = brain.act(obs)
        brain_s += time.perf_counter() - t
        action = np.asarray(action, dtype=float)
        if action.shape != (world.n_servos,) or not np.all(np.isfinite(action)):
            reason = "bad_action"
            break
        action = np.clip(action, lower, upper)
        if push is not None and world.data.time >= push["at_s"]:
            world.push(push["fx_n"], push["fy_n"], push["seconds"])
            push = None
        world.step(action)
        steps += 1
        if _physics_broken(world.data):
            trace.append(world.data.qpos.copy())
            reason = "sim_error"
            break
        truth = world.truth()
        truth["time_s"] = steps * control_dt  # the episode's step clock, not the summed float data.time
        trace.append(truth["qpos"])
        bellies.append(truth["belly_m"])
        depth = max(depth, truth["penetration_mm"])
        self_hit += truth["self_hit"]
        airborne += truth["feet_down"] == 0
        reason, held = tasks.judge(sp, goal, truth, held)
        next_obs = eyes.read(action)
        t = time.perf_counter()
        brain.observe(obs, action, next_obs, reason is not None)
        brain_s += time.perf_counter() - t
        obs = next_obs
        if on_step is not None and on_step(world) is False and reason is None:
            reason = "aborted"
    belly_mm_arr = m_to_mm(np.asarray(bellies))
    metrics = {
        "seed": int(seed), "task": task, "brain": type(brain).__name__.lower(), "body": world.spec["name"],
        "servo": world.servo, "nominal": bool(nominal), "success": reason == "success", "reason": reason,
        "steps": steps, "robot_minutes": steps / ROBOT_MINUTE_STEPS, "time_s": steps * control_dt,
        "wall_s": time.perf_counter() - wall0, "brain_s": brain_s,
        "belly_min_mm": float(belly_mm_arr.min()), "belly_mean_mm": float(belly_mm_arr.mean()),
        "belly_max_mm": float(belly_mm_arr.max()), "cutouts": int(np.count_nonzero(world.bus.cut)),
        "self_hit_steps": int(self_hit), "airborne_steps": int(airborne), "max_penetration_mm": float(depth),
        "ik_failures": int(brain.info().get("ik_failures", 0)),
    }
    return metrics, np.asarray(trace, dtype=np.float32)


def _jsonable(x):
    """Tuples, numpy scalars and arrays, paths → plain JSON values."""
    if isinstance(x, dict):
        return {k: _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple, np.ndarray)):
        return [_jsonable(v) for v in x]
    if isinstance(x, np.generic):
        return x.item()
    if isinstance(x, Path):
        return str(x)
    return x


def _write_json(path: Path, value) -> None:
    path.write_text(json.dumps(_jsonable(value), indent=2) + "\n", encoding="utf-8")


def _new_run_dir(name: str) -> Path:
    """runs/<name>, or <name>-2, <name>-3, … when it exists; created here, never shared with another run."""
    RUNS.mkdir(parents=True, exist_ok=True)
    n = 1
    while True:
        run_dir = RUNS / (name if n == 1 else f"{name}-{n}")
        try:
            run_dir.mkdir(exist_ok=False)
            return run_dir
        except FileExistsError:
            n += 1


def _brain_seeds(root: int, n: int) -> list[int]:
    """The brain seed of episode k (0-based position in the run's seeds) under one root."""
    return [int(np.random.SeedSequence([root, k]).generate_state(1)[0]) for k in range(n)]


def evaluate(brain_name: str, task: str, seeds, nominal: bool, body: str = "v0", servo: str = DEFAULT_SERVO,
             belly_mm: float | None = None, out_dir: Path | None = None, on_step=None, twin: bool = True,
             brain_seed: int | None = None, cli: dict | None = None) -> dict:
    """One World, one brain, one episode per seed; stops after an "aborted" episode. Returns summary.json.
    twin: the headless twin without the CAD mesh geoms (the default); False for a run shown in the window.
    out_dir: an existing folder must be empty (ValueError before anything is written); None: a new folder in runs/.
    brain_seed: the root of the brain seeds, secrets.randbits(32) when None; config.json records the root used.
    cli: the command's own settings (--seconds, --headless), written to config.json as given; None from code."""
    seeds = [int(s) for s in seeds]
    if out_dir is not None and Path(out_dir).exists() and any(Path(out_dir).iterdir()):
        raise ValueError(f"{out_dir} exists and is not empty; a run folder is never overwritten")
    root = secrets.randbits(32) if brain_seed is None else int(brain_seed)
    world = World(body, servo, twin=twin)
    brain = brains.make(brain_name, body)
    if out_dir is None:
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        run_dir = _new_run_dir(f"{stamp}_{brain_name}_{body}_{task}")
    else:
        run_dir = Path(out_dir)
        run_dir.mkdir(parents=True, exist_ok=True)
    arguments = {
        "brain_name": brain_name, "task": task, "seeds": seeds, "nominal": nominal, "body": body, "servo": servo,
        "belly_mm": belly_mm, "out_dir": run_dir, "twin": twin, "brain_seed": root,
    }
    _write_json(run_dir / "config.json", {
        "arguments": arguments, "cli": cli, "model": "twin (no _vis geoms)" if twin else "full",
        "ranges": randomise.RANGES, "eye": sensors.EYE, "imu": sensors.IMU,
        "servo": {"name": servo, **SERVOS[servo]},
    })
    rows, traces = [], []
    with open(run_dir / "metrics.jsonl", "w", encoding="utf-8") as f:
        for n, (seed, episode_brain_seed) in enumerate(zip(seeds, _brain_seeds(root, len(seeds)))):
            metrics, trace = run_episode(world, brain, task, seed, nominal, belly_mm, on_step, episode_brain_seed)
            metrics = {"episode": n, **metrics}
            assert tuple(metrics) == METRIC_KEYS, tuple(metrics)
            f.write(json.dumps(_jsonable(metrics)) + "\n")
            f.flush()
            rows.append(metrics)
            traces.append(trace)
            if metrics["reason"] == "aborted":
                break
    np.savez_compressed(
        run_dir / "motion.npz",
        qpos=np.concatenate(traces).astype(np.float32),
        episode=np.concatenate([np.full(len(t), n, dtype=int) for n, t in enumerate(traces)]),
        time_s=np.concatenate([np.arange(len(t)) / RATE_HZ for t in traces]),
        body=np.array(body),
        rate_hz=np.array(RATE_HZ),
    )
    robot_s = sum(m["time_s"] for m in rows)
    wall_s = sum(m["wall_s"] for m in rows)
    brain_s = sum(m["brain_s"] for m in rows)
    summary = {
        "brain": brain_name, "task": task, "body": body, "servo": servo, "nominal": nominal, "episodes": len(rows),
        "successes": sum(m["success"] for m in rows),
        "reasons": {r: sum(m["reason"] == r for m in rows) for r in REASONS},
        "robot_minutes": sum(m["robot_minutes"] for m in rows), "wall_s": wall_s,
        "speed_x": robot_s / (wall_s - brain_s), "run_dir": str(run_dir),
        "eval_seeds": seeds == list(EVAL_SEEDS),
    }
    _write_json(run_dir / "summary.json", summary)
    return summary
