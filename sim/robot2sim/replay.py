# SPDX-License-Identifier: MIT
"""Show and picture a recorded run from its motion.npz, without physics: poses come from the file, mj_forward places
the meshes, mj_step is never called. Only geom group 2 is drawn: the CAD meshes and the room (`robot2sim.stage`, a
display-only model with a fixed camera, a board, goal and start tiles; the physics model is not touched).

Rows whose qpos is not finite (the last row of a sim_error episode can be) are skipped by frames, png, mp4 and show.
"""

import json
import subprocess
import time
from pathlib import Path

import numpy as np

import mujoco

from robot2sim import stage

CAD_GROUP = 2
CAMERA_DISTANCE_M = 1.0  # UNVERIFIED: the live run's camera (no room there): the whole robot and a little floor
CAMERA_ELEVATION_DEG = -25.0  # UNVERIFIED
CAMERA_AZIMUTH_DEG = 135.0  # UNVERIFIED
MOTION_FILE = "motion.npz"
CONFIG_FILE = "config.json"
FPS = 25  # frames at every = 2 of the 50 Hz motion: real time


def _task(run_dir: Path) -> str:
    return json.loads((Path(run_dir) / CONFIG_FILE).read_text(encoding="utf-8"))["arguments"]["task"]


def load(run_dir: Path, episode: int = 0) -> tuple[mujoco.MjModel, np.ndarray, np.ndarray]:
    """(display model of the recorded body in the room, qpos n × 25 as recorded, time_s n) of one episode."""
    path = Path(run_dir) / MOTION_FILE
    if not path.is_file():
        raise FileNotFoundError(f"{run_dir}: no {MOTION_FILE}")
    with np.load(path) as z:
        episodes = z["episode"]
        if episode not in set(episodes.tolist()):
            raise ValueError(f"episode {episode} is not in {path}: it holds 0…{int(episodes.max())}")
        mask = episodes == episode
        qpos, time_s, body = z["qpos"][mask], z["time_s"][mask], str(z["body"])
    model = stage.build(body, _task(run_dir), qpos[0])
    assert qpos.shape[1] == model.nq, f"qpos width {qpos.shape[1]}, body {body} has nq {model.nq}"
    return model, qpos, time_s


def board(run_dir: Path, episode: int, time_s) -> np.ndarray:
    """Board numbers per row of the episode, n × 2: (seconds left, attempts already had)."""
    task = _task(run_dir)
    return np.array([stage.numbers(task, episode, float(t)) for t in time_s], dtype=int).reshape(-1, 2)


def cad_only() -> mujoco.MjvOption:
    """Every geom group off except the CAD meshes and the room (group 2)."""
    option = mujoco.MjvOption()
    option.geomgroup[:] = 0
    option.geomgroup[CAD_GROUP] = 1
    return option


def follow(model, cam) -> None:
    """The live run's camera (the physics model, no room): track the robot's root body."""
    root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = int(model.jnt_bodyid[root])
    cam.distance = CAMERA_DISTANCE_M
    cam.elevation = CAMERA_ELEVATION_DEG
    cam.azimuth = CAMERA_AZIMUTH_DEG


def _finite(qpos):
    return np.all(np.isfinite(qpos), axis=1)


def frames(model, qpos, width: int = 1280, height: int = 720, every: int = 2, board=None):
    """RGB arrays (height × width × 3) of every `every`-th finite row from the fixed room camera; `board` (n × 2,
    parallel to qpos) lights the board per row, else it stays as it is."""
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), width)
    model.vis.global_.offheight = max(int(model.vis.global_.offheight), height)
    data = mujoco.MjData(model)
    cam = mujoco.MjvCamera()
    stage.aim(model, cam)
    option = cad_only()
    renderer = mujoco.Renderer(model, height, width)
    rows = np.arange(len(qpos))[::every]
    try:
        for k in rows[_finite(qpos[rows])]:
            if board is not None:
                stage.show(model, *board[k])
            data.qpos[:] = qpos[k]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=cam, scene_option=option)
            yield renderer.render()
    finally:
        renderer.close()


def png(run_dir: Path, episode: int, path: Path, at: float = 0.5) -> None:
    """Write the frame at share `at` (0 … 1) of the episode's finite rows as a PNG."""
    from PIL import Image

    if not 0.0 <= at <= 1.0:
        raise ValueError(f"at {at} is not within 0 … 1")
    model, qpos, time_s = load(run_dir, episode)
    keep = _finite(qpos)
    rows, numbers = qpos[keep], board(run_dir, episode, time_s[keep])
    k = int(round(at * (len(rows) - 1)))
    frame = next(frames(model, rows[k:k + 1], board=numbers[k:k + 1]))
    Image.fromarray(frame).save(Path(path))


def mp4(run_dir: Path, episode: int, path: Path, ffmpeg: str) -> None:
    """The whole episode in real time as H.264, raw frames piped into ffmpeg."""
    model, qpos, time_s = load(run_dir, episode)
    width, height = 1280, 720
    cmd = [ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}",
           "-r", str(FPS), "-i", "-", "-pix_fmt", "yuv420p", "-c:v", "libx264", str(path)]
    child = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    for frame in frames(model, qpos, width, height, board=board(run_dir, episode, time_s)):
        child.stdin.write(frame.tobytes())
    child.stdin.close()
    if child.wait() != 0:
        raise RuntimeError(f"ffmpeg exit {child.returncode}")


def open_window(model, data, aim=follow):
    """The live window of a run and of a replay: launch_passive, geom group 2 only, camera set by `aim` (the live
    run follows the robot, the replay takes the room camera). Returns the viewer and the visible geom groups as
    read back from it. Needs mjpython on macOS."""
    import mujoco.viewer

    viewer = mujoco.viewer.launch_passive(model, data)
    with viewer.lock():
        viewer.opt.geomgroup[:] = cad_only().geomgroup
        aim(model, viewer.cam)
    return viewer, [int(g) for g in np.flatnonzero(viewer.opt.geomgroup)]


def show(run_dir: Path, episode: int = 0) -> None:
    """Play one episode once in the live window at real time; ends at the last row or when the window is closed."""
    model, qpos, time_s = load(run_dir, episode)
    finite = _finite(qpos)
    numbers = board(run_dir, episode, time_s)
    data = mujoco.MjData(model)
    viewer, _groups = open_window(model, data, stage.aim)
    try:
        t0 = time.perf_counter()
        for row, t, n in zip(qpos[finite], time_s[finite] - time_s[0], numbers[finite]):
            if not viewer.is_running():
                break
            with viewer.lock():
                stage.show(model, *n)
                data.qpos[:] = row
                mujoco.mj_forward(model, data)
            viewer.sync()
            wait = t0 + float(t) - time.perf_counter()
            if wait > 0.0:
                time.sleep(wait)
    finally:
        viewer.close()
