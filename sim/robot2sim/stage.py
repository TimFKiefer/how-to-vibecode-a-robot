# SPDX-License-Identifier: MIT
"""The room the replay draws the robot in: a display-only model (tiled floor slab, three panel walls, a two-line
7-segment board, goal and start tiles, spot lights), a fixed camera, and the robot recoloured.

Decoration only. `build` loads the body file into an MjSpec, adds static geoms that collide with nothing (the body
file's default: contype 0, conaffinity 0, group 2) and compiles a second model; the physics model World and
scene.load_model build is not touched. Every look constant is a choice, not a measurement.
"""

import math

import mujoco
import numpy as np

from robot2sim import tasks
from robot2sim.paths import body_xml
from robot2sim.units import deg_to_rad

TILE_M = 1.0 / 3.0  # UNVERIFIED: the robot (about 0.4 m across) covers 1.2 tiles; start and goal sit on tile centres
TILES_Y = 8  # UNVERIFIED: room width along y, in tiles
TILES_X = 5  # UNVERIFIED: room depth along x, in tiles
ROOM_CENTRE_XY = (0.0, 0.5)  # between the nominal start (0, 0) and the nominal goal (0, 1)
SLAB_M = 0.7  # UNVERIFIED: floor slab thickness below z = 0, its front face is seen
WALL_M = 1.7  # UNVERIFIED: wall height above the floor
WALL_T_M = 0.04  # UNVERIFIED
PANEL_H_M = 0.5  # UNVERIFIED: wall panel height; a panel is one tile wide
SEAM_M = 0.006  # UNVERIFIED: seam width
SEAM_RISE_M = 0.0008  # UNVERIFIED: floor seams stand this far above the tiles, wall seams out of the wall
MARK_RISE_M = 0.0012  # UNVERIFIED: goal and start tiles above the floor (over the seams)
MARK_SIDE = 0.92  # UNVERIFIED: goal and start squares as a share of a tile
STRIPE_W_M = 0.028  # UNVERIFIED
STRIPE_PITCH_M = 0.07  # UNVERIFIED: stripe to stripe, across the stripes
TILE_RGBA = (0.31, 0.31, 0.33, 1.0)  # UNVERIFIED
FLOOR_SEAM_RGBA = (0.85, 0.85, 0.87, 1.0)  # UNVERIFIED
WALL_RGBA = (0.82, 0.82, 0.84, 1.0)  # UNVERIFIED
WALL_SEAM_RGBA = (0.42, 0.42, 0.45, 1.0)  # UNVERIFIED
GOAL_RGBA = (0.35, 0.95, 0.20, 1.0)  # UNVERIFIED
STRIPE_RGBA = (0.95, 0.55, 0.08, 1.0)  # UNVERIFIED: amber
PRINTED_RGBA = (0.10, 0.45, 0.95, 1.0)  # UNVERIFIED: every printed part
SERVO_RGBA = (0.08, 0.08, 0.08, 1.0)  # UNVERIFIED: the 18 servos ({leg}_s1, _s2, _s3)
KEEP_COLOUR = ("eyes", "label_", "window", "e_")  # part name starts: keep the body file's colour (not printed)
LEGS = ("RF", "RM", "RR", "LF", "LM", "LR")
SERVO_PARTS = ("s1_vis", "s2_vis", "s3_vis")
BOARD_RGBA = (0.03, 0.03, 0.035, 1.0)  # UNVERIFIED
SEGMENT_ON = (0.97, 0.97, 0.97, 1.0)  # UNVERIFIED
SEGMENT_OFF = (0.07, 0.07, 0.075, 1.0)  # UNVERIFIED: an unlit segment, a little lighter than the panel
DIGITS = {"top": 2, "bottom": 4}  # slots: seconds left in the attempt; attempts already had (the episode index)
SHOWN = {"top": 2, "bottom": 2}  # digits always lit ("00"); slots left of them stay dark
CLOCK_ROUND_DIGITS = 6  # limit − time_s rounded first, so float32 dust in time_s never moves the ceil
BOARD_Z_M = {"top": 0.9, "bottom": 0.67}  # UNVERIFIED: panel centres on the back wall
DIGIT_W_M, DIGIT_H_M, SEG_T_M = 0.075, 0.14, 0.016  # UNVERIFIED
DIGIT_GAP_M = 0.035  # UNVERIFIED
BOARD_PAD_M = 0.04  # UNVERIFIED: panel margin around the digits
SEGMENTS = ("abcdef", "bc", "abdeg", "abcdg", "bcfg", "acdfg", "acdefg", "abc", "abcdefg", "abcdfg")
CAMERA_LOOKAT = (0.05, 0.5, 0.28)  # UNVERIFIED
CAMERA_DISTANCE_M = 2.3  # UNVERIFIED
CAMERA_AZIMUTH_DEG = 180.0  # UNVERIFIED: from +x looking toward −x, so +y runs left to right
CAMERA_ELEVATION_DEG = -27.0  # UNVERIFIED
SPOTS_Y = (-0.4, 0.5, 1.4)  # UNVERIFIED: three pools of light on the back wall
SPOT = {"pos_z": 1.75, "pos_x": 0.5, "aim_z": 0.95, "cutoff": 70.0, "exponent": 24.0, "diffuse": 0.85}  # UNVERIFIED
KEY_LIGHT = {"pos": (1.2, 0.5, 2.6), "diffuse": 0.7, "cutoff": 50.0}  # UNVERIFIED: casts the robot's shadow
NUMQUADS = 40  # UNVERIFIED: quads per box face; with MuJoCo's default 4 the spot pools on the walls did not show
SIDE_LIGHT = {"back": 0.45, "down": 0.5, "diffuse": 0.42}  # UNVERIFIED: direction (−back, ±1, −down) onto each side wall
HEADLIGHT = {"ambient": 0.22, "diffuse": 0.15}  # UNVERIFIED


def segments(digit: int) -> str:
    """The lit segments of one digit: a top, b top right, c bottom right, d bottom, e bottom left, f top left,
    g middle."""
    return SEGMENTS[digit]


def goal_xy(task: str, start_qpos) -> np.ndarray | None:
    """The run's point goal from the recorded start pose (the reset pose is the start truth), via tasks.goal."""
    if tasks.spec(task).goal_kind != "point":
        return None
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(start_qpos[3:7], dtype=float))
    truth0 = {"pos_m": np.asarray(start_qpos[:3], dtype=float), "yaw_rad": math.atan2(R[3], R[0])}
    return np.array(tasks.goal(task, truth0)["xy_m"])


def numbers(task: str, episode: int, time_s: float) -> tuple[int, int]:
    """(top, bottom) of the board: whole seconds left in the attempt, ceil(limit − elapsed), never below 0 (it stops
    where it is when the attempt ends early); the attempts already had = the episode index (the first shows 00)."""
    left = round(tasks.spec(task).limit_s - float(time_s), CLOCK_ROUND_DIGITS)
    return max(0, math.ceil(left)), episode


def _text(line: str, value: int) -> str:
    """The digits of one line, right-aligned in its slots; leading slots beyond SHOWN are blank (dark)."""
    text = f"{value:0{SHOWN[line]}d}"
    if value < 0 or len(text) > DIGITS[line]:
        raise ValueError(f"{value} does not fit the board's {line} line ({DIGITS[line]} digits)")
    return text.rjust(DIGITS[line])


def show(model, top: int, bottom: int) -> None:
    """Light the board's segments for the two numbers (model.geom_rgba of the display model)."""
    for line, value in (("top", top), ("bottom", bottom)):
        for k, ch in enumerate(_text(line, value)):
            lit = "" if ch == " " else segments(int(ch))
            for seg in "abcdefg":
                g = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"stage_{line}{k}_{seg}")
                model.geom_rgba[g] = SEGMENT_ON if seg in lit else SEGMENT_OFF


def aim(_model, cam) -> None:
    """The fixed room camera."""
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = CAMERA_LOOKAT
    cam.distance = CAMERA_DISTANCE_M
    cam.azimuth = CAMERA_AZIMUTH_DEG
    cam.elevation = CAMERA_ELEVATION_DEG


def _box(parent, name, pos, half, rgba, quat=(1.0, 0.0, 0.0, 0.0)):
    g = parent.add_geom()
    g.name, g.type = name, mujoco.mjtGeom.mjGEOM_BOX
    g.pos, g.size, g.rgba, g.quat = pos, half, rgba, quat
    g.contype = g.conaffinity = 0
    g.group = 2
    g.mass = 0.0
    return g


def _floor(wb, x0, x1, y0, y1):
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    hx, hy, s, r = (x1 - x0) / 2.0, (y1 - y0) / 2.0, SEAM_M / 2.0, SEAM_RISE_M / 2.0
    _box(wb, "stage_slab", (cx, cy, -SLAB_M / 2.0), (hx, hy, SLAB_M / 2.0), TILE_RGBA)
    for i in range(TILES_X + 1):  # seams along y on the top
        _box(wb, f"stage_seam_x{i}", (x0 + i * TILE_M, cy, r), (s, hy, r), FLOOR_SEAM_RGBA)
    for j in range(TILES_Y + 1):  # seams along x on the top, and down the front face
        y = y0 + j * TILE_M
        _box(wb, f"stage_seam_y{j}", (cx, y, r), (hx, s, r), FLOOR_SEAM_RGBA)
        _box(wb, f"stage_front_{j}", (x1 + r, y, -SLAB_M / 2.0), (r, s, SLAB_M / 2.0), FLOOR_SEAM_RGBA)
    _box(wb, "stage_front_lip", (x1 + r, cy, -s), (r, hy, s), FLOOR_SEAM_RGBA)


def _walls(wb, x0, x1, y0, y1):
    t, h, s, r = WALL_T_M / 2.0, WALL_M / 2.0, SEAM_M / 2.0, SEAM_RISE_M
    cx, cy, hx, hy = (x0 + x1) / 2.0, (y0 + y1) / 2.0, (x1 - x0) / 2.0, (y1 - y0) / 2.0
    _box(wb, "stage_wall_back", (x0 - t, cy, h), (t, hy + 2 * t, h), WALL_RGBA)
    zs, hs = (WALL_M - SLAB_M) / 2.0, (WALL_M + SLAB_M) / 2.0  # side walls reach down past the slab
    _box(wb, "stage_wall_left", (cx, y0 - t, zs), (hx, t, hs), WALL_RGBA)
    _box(wb, "stage_wall_right", (cx, y1 + t, zs), (hx, t, hs), WALL_RGBA)
    rows = int(round(WALL_M / PANEL_H_M))
    for j in range(1, TILES_Y):
        _box(wb, f"stage_wseam_back_v{j}", (x0 + r, y0 + j * TILE_M, h), (r, s, h), WALL_SEAM_RGBA)
    for i in range(1, TILES_X + 1):
        for side, y in (("left", y0 + r), ("right", y1 - r)):
            _box(wb, f"stage_wseam_{side}_v{i}", (x0 + i * TILE_M, y, h), (s, r, h), WALL_SEAM_RGBA)
    for k in range(0, rows):
        z = max(k * PANEL_H_M, s)  # the bottom seam marks where wall meets floor
        _box(wb, f"stage_wseam_back_h{k}", (x0 + r, cy, z), (r, hy, s), WALL_SEAM_RGBA)
        for side, y in (("left", y0 + r), ("right", y1 - r)):
            _box(wb, f"stage_wseam_{side}_h{k}", (cx, y, z), (hx, r, s), WALL_SEAM_RGBA)


def _segment_boxes(u, v):
    """(name, du, dv, half_u, half_v) of the seven segments of a digit centred at (u, v) on the wall."""
    w, hh, t = DIGIT_W_M / 2.0, DIGIT_H_M / 2.0, SEG_T_M / 2.0
    return (
        ("a", u, v + hh, w - t, t), ("g", u, v, w - t, t), ("d", u, v - hh, w - t, t),
        ("f", u - w, v + hh / 2.0, t, hh / 2.0 - t), ("b", u + w, v + hh / 2.0, t, hh / 2.0 - t),
        ("e", u - w, v - hh / 2.0, t, hh / 2.0 - t), ("c", u + w, v - hh / 2.0, t, hh / 2.0 - t),
    )


def _board(wb, x0):
    cy = ROOM_CENTRE_XY[1]
    for line, n in DIGITS.items():
        z = BOARD_Z_M[line]
        width = n * DIGIT_W_M + (n - 1) * DIGIT_GAP_M
        x_panel = x0 + 0.01
        _box(wb, f"stage_board_{line}", (x_panel, cy, z),
             (0.01, width / 2.0 + BOARD_PAD_M, DIGIT_H_M / 2.0 + BOARD_PAD_M), BOARD_RGBA)
        for k in range(n):
            u = cy - width / 2.0 + DIGIT_W_M / 2.0 + k * (DIGIT_W_M + DIGIT_GAP_M)
            for seg, su, sv, hu, hv in _segment_boxes(u, z):
                _box(wb, f"stage_{line}{k}_{seg}", (x_panel + 0.011, su, sv), (0.002, hu, hv), SEGMENT_OFF)


def _start_tile(wb, xy):
    """Diagonal amber stripes clipped to a square around the start point (a body, so its origin is the point)."""
    body = wb.add_body()
    body.name, body.pos = "stage_start", (float(xy[0]), float(xy[1]), 0.0)
    h = MARK_SIDE * TILE_M / 2.0
    step = STRIPE_PITCH_M * math.sqrt(2.0)  # lines u + v = c, this far apart in c
    q = (math.cos(deg_to_rad(-22.5)), 0.0, 0.0, math.sin(deg_to_rad(-22.5)))  # −45° about z
    for i, c in enumerate(np.arange(-2.0 * h + step / 2.0, 2.0 * h, step)):
        u0, u1 = max(-h, c - h), min(h, c + h)
        length = (u1 - u0) * math.sqrt(2.0)
        if length < STRIPE_W_M:
            continue
        um = (u0 + u1) / 2.0
        _box(body, f"stage_stripe_{i}", (um, c - um, MARK_RISE_M / 2.0),
             (length / 2.0 - STRIPE_W_M / 2.0, STRIPE_W_M / 2.0, MARK_RISE_M / 2.0), STRIPE_RGBA, q)


def _lights(spec, x0):
    for light in spec.worldbody.lights:  # the body file's directional light: the room brings its own
        light.active = False
    for i, y in enumerate(SPOTS_Y):
        li = spec.worldbody.add_light()
        li.name, li.type, li.castshadow = f"stage_spot_{i}", mujoco.mjtLightType.mjLIGHT_SPOT, False
        pos = np.array([x0 + SPOT["pos_x"], y, SPOT["pos_z"]])
        d = np.array([x0, y, SPOT["aim_z"]]) - pos
        li.pos, li.dir = pos, d / np.linalg.norm(d)
        li.cutoff, li.exponent = SPOT["cutoff"], SPOT["exponent"]
        li.diffuse, li.specular = [SPOT["diffuse"]] * 3, [0.0] * 3
    for i, sign in enumerate((-1.0, 1.0)):  # one soft directional light onto each side wall, no shadow
        side = spec.worldbody.add_light()
        side.name, side.type, side.castshadow = f"stage_side_{i}", mujoco.mjtLightType.mjLIGHT_DIRECTIONAL, False
        side.dir = (-SIDE_LIGHT["back"], sign, -SIDE_LIGHT["down"])
        side.diffuse, side.specular = [SIDE_LIGHT["diffuse"]] * 3, [0.0] * 3
    key = spec.worldbody.add_light()
    key.name, key.type, key.castshadow = "stage_key", mujoco.mjtLightType.mjLIGHT_SPOT, True
    pos = np.array(KEY_LIGHT["pos"])
    d = np.array([ROOM_CENTRE_XY[0], ROOM_CENTRE_XY[1], 0.0]) - pos
    key.pos, key.dir = pos, d / np.linalg.norm(d)
    key.cutoff, key.diffuse, key.specular = KEY_LIGHT["cutoff"], [KEY_LIGHT["diffuse"]] * 3, [0.1] * 3


def _part(name):
    """The part name without a leg prefix: RF_blade_vis → blade_vis, e_camera_vis stays."""
    head, _, rest = name.partition("_")
    return rest if head in LEGS else name


def _recolour(spec):
    for g in spec.geoms:
        name = g.name or ""
        if name == "floor":
            g.group = 5  # the physics floor plane: not drawn, the slab's top is at the same height
        elif _part(name) in SERVO_PARTS:
            g.rgba = SERVO_RGBA
        elif name.endswith("_vis") and not _part(name).startswith(KEEP_COLOUR):
            g.rgba = PRINTED_RGBA


def build(body: str, task: str, start_qpos) -> mujoco.MjModel:
    """The display model of `body` in the room; goal and start tiles for a point-goal task, at the run's points."""
    _text("top", math.ceil(tasks.spec(task).limit_s))  # a time limit the clock cannot show raises here
    spec = mujoco.MjSpec.from_file(str(body_xml(body)))
    _recolour(spec)
    wb = spec.worldbody.add_body()  # one static body after the robot: the physics geoms keep their ids
    wb.name = "stage"
    cx, cy = ROOM_CENTRE_XY
    x0, x1 = cx - TILES_X * TILE_M / 2.0, cx + TILES_X * TILE_M / 2.0
    y0, y1 = cy - TILES_Y * TILE_M / 2.0, cy + TILES_Y * TILE_M / 2.0
    _floor(wb, x0, x1, y0, y1)
    _walls(wb, x0, x1, y0, y1)
    _board(wb, x0)
    goal = goal_xy(task, start_qpos)
    if goal is not None:
        h = MARK_SIDE * TILE_M / 2.0
        _box(wb, "stage_goal", (float(goal[0]), float(goal[1]), MARK_RISE_M / 2.0), (h, h, MARK_RISE_M / 2.0),
             GOAL_RGBA)
        _start_tile(wb, start_qpos[:2])
    _lights(spec, x0)
    spec.visual.headlight.ambient = [HEADLIGHT["ambient"]] * 3
    spec.visual.headlight.diffuse = [HEADLIGHT["diffuse"]] * 3
    spec.visual.map.zfar = 50.0
    spec.visual.quality.numquads = NUMQUADS
    return spec.compile()
