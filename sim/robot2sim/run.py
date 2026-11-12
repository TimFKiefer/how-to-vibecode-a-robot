# SPDX-License-Identifier: MIT
"""The command: a run live in the MuJoCo window (default), a headless evaluation, or a replay of a motion file.

From sim/:

    .venv/bin/python -m robot2sim.run --brain scripted --task walk_1m --nominal
    .venv/bin/python -m robot2sim.run --brain hold --task stand_up --headless
    .venv/bin/python -m robot2sim.run --replay runs/<folder> --episode 0 --png pose.png
    .venv/bin/python -m robot2sim.run --replay runs/<folder> --episode 0 --mp4 walk.mp4

With a window: one episode, seed 1001 (the first evaluation seed); on macOS the plain interpreter re-executes
itself under mjpython. Closing the window or reaching --seconds ends the episode as "aborted"; the files are still
written. --headless: the 20 evaluation seeds. --seed S --episodes N: seeds S … S + N − 1 in both modes.
--brain-seed N: the root of the brain seeds (a fresh one without it). A refused argument (an unknown name, a belly
outside the body's range, a non-empty --out, …) is one line on stderr and exit code 2.
"""

import argparse
import json
import shutil
import sys
import time
from pathlib import Path

from robot2sim.paths import body_json, body_xml

WINDOW_EPISODES = 1
CONTROL_DT_S = 0.02  # 50 Hz control
EXIT_REFUSED = 2


def _refuse(message: str):
    print(f"error: {message}", file=sys.stderr)
    raise SystemExit(EXIT_REFUSED)


def _parser():
    p = argparse.ArgumentParser(prog="python -m robot2sim.run", description="Run, evaluate or replay Takt")
    p.error = _refuse  # one line on stderr instead of argparse's usage block
    p.add_argument("--brain")
    p.add_argument("--task")
    p.add_argument("--episodes", type=int)
    p.add_argument("--seed", type=int)
    p.add_argument("--nominal", action="store_true")
    p.add_argument("--belly-mm", type=float)
    p.add_argument("--servo")
    p.add_argument("--body", default="v0")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--seconds", type=float)
    p.add_argument("--out", type=Path)
    p.add_argument("--brain-seed", type=int)
    p.add_argument("--replay", type=Path)
    p.add_argument("--episode", type=int, default=0)
    p.add_argument("--png", type=Path)
    p.add_argument("--at", type=float, default=0.5)
    p.add_argument("--mp4", type=Path)
    return p


def _replay(args, argv) -> int:
    from robot2sim import replay
    from robot2sim.view import _ensure_mjpython

    try:
        replay.load(args.replay, args.episode)
    except (FileNotFoundError, ValueError) as e:
        _refuse(str(e))
    if args.png is not None:
        if not 0.0 <= args.at <= 1.0:
            _refuse(f"--at {args.at} is not within 0 … 1")
        replay.png(args.replay, args.episode, args.png, args.at)
        print(f"png: {args.png}")
        return 0
    if args.mp4 is not None:
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is None:
            _refuse("--mp4 needs ffmpeg on PATH; none found")
        replay.mp4(args.replay, args.episode, args.mp4, ffmpeg)
        print(f"mp4: {args.mp4}")
        return 0
    _ensure_mjpython(argv, "robot2sim.run")
    replay.show(args.replay, args.episode)
    return 0


def _check_run_args(args) -> None:
    from robot2sim import brains, scripted, tasks
    from robot2sim.servo import SERVOS

    if args.brain not in brains.NAMES:
        _refuse(f"unknown brain {args.brain!r}; valid: {', '.join(brains.NAMES)}")
    if args.task not in tasks.TASKS:
        _refuse(f"unknown task {args.task!r}; valid: {', '.join(tasks.TASKS)}")
    if args.servo not in SERVOS:
        _refuse(f"unknown servo {args.servo!r}; valid: {', '.join(SERVOS)}")
    if not (body_json(args.body).is_file() and body_xml(args.body).is_file()):
        _refuse(f"unknown body {args.body!r}: no {body_json(args.body).name} and {body_xml(args.body).name}")
    heights = scripted.heights_mm()  # the CAD script's NAMED_HEIGHTS: the lowest and highest are the belly range
    if args.belly_mm is not None and not min(heights) <= args.belly_mm <= max(heights):
        _refuse(f"--belly-mm {args.belly_mm:g} is outside the belly range {min(heights):g} … {max(heights):g} mm")
    if args.brain == "scripted" and args.belly_mm is not None:
        if not any(abs(args.belly_mm - h) <= scripted.HEIGHT_TOL_MM for h in heights):
            named = ", ".join(f"{h:g}" for h in heights)
            _refuse(f"the scripted rival cannot walk at belly {args.belly_mm:g} mm; it walks at {named} mm")
    if args.episodes is not None and args.episodes < 1:
        _refuse(f"--episodes {args.episodes} is below 1")
    if args.seconds is not None and args.seconds <= 0.0:
        _refuse(f"--seconds {args.seconds} is not above 0")
    if args.out is not None and args.out.exists() and any(args.out.iterdir()):
        _refuse(f"--out {args.out} exists and is not empty; a run folder is never overwritten")


def _steps(world) -> int:
    return int(round(world.data.time / CONTROL_DT_S))


def _window_on_step(seconds, clocks, state):
    """on_step for a run in the window: opens it at the first call, paces to real time, stops at `seconds` or when
    the window is closed. clocks gets [robot s, wall s] per episode, from the first step on."""
    from robot2sim import replay

    def on_step(world):
        if state.get("viewer") is None:
            state["viewer"], groups = replay.open_window(world.model, world.data)
            print("window: open")
            print(f"visible groups: {groups}", flush=True)
        viewer = state["viewer"]
        if not viewer.is_running():
            return False
        steps = _steps(world)
        now = time.perf_counter()
        if steps == 1:
            state["t0"] = now
            clocks.append([0.0, 0.0])
        viewer.sync()
        wait = state["t0"] + steps * CONTROL_DT_S - time.perf_counter()
        if wait > 0.0:
            time.sleep(wait)
        clocks[-1] = [steps * CONTROL_DT_S, time.perf_counter() - state["t0"]]
        return seconds is None or steps < round(seconds / CONTROL_DT_S)

    return on_step


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    args = _parser().parse_args(argv)
    if args.replay is not None:
        return _replay(args, argv)
    from robot2sim.servo import DEFAULT_SERVO

    args.servo = DEFAULT_SERVO if args.servo is None else args.servo
    _check_run_args(args)
    if not args.headless:
        from robot2sim.view import _ensure_mjpython

        _ensure_mjpython(argv, "robot2sim.run")
    from robot2sim.episode import EVAL_SEEDS, evaluate

    n = args.episodes if args.episodes is not None else (len(EVAL_SEEDS) if args.headless else WINDOW_EPISODES)
    seeds = list(EVAL_SEEDS[:n]) if args.seed is None else [args.seed + k for k in range(n)]
    clocks, state = [], {}
    if args.headless:
        limit = None if args.seconds is None else round(args.seconds / CONTROL_DT_S)
        on_step = None if limit is None else (lambda world: _steps(world) < limit)
    else:
        on_step = _window_on_step(args.seconds, clocks, state)
    try:
        summary = evaluate(args.brain, args.task, seeds, args.nominal, args.body, args.servo, args.belly_mm,
                           args.out, on_step, twin=args.headless,  # the window shows the full model
                           brain_seed=args.brain_seed, cli={"seconds": args.seconds, "headless": args.headless})
    finally:
        if state.get("viewer") is not None:
            state["viewer"].close()
    run_dir = Path(summary["run_dir"])
    for line in (run_dir / "metrics.jsonl").read_text(encoding="utf-8").splitlines():
        m = json.loads(line)
        print(f"episode {m['episode']} seed {m['seed']} {m['reason']} {m['time_s']:.1f} s")
    for robot_s, wall_s in clocks:
        print(f"real time: robot {robot_s:.2f} s, wall {wall_s:.2f} s")
    reasons = ", ".join(f"{r} {c}" for r, c in summary["reasons"].items() if c)
    print(
        f"summary: {summary['brain']} {summary['task']} {summary['episodes']} episodes, {summary['successes']} "
        f"successes ({reasons}), {summary['robot_minutes']:.2f} robot minutes, speed {summary['speed_x']:.0f}x"
    )
    print(f"run: {run_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
