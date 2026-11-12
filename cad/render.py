#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Perspective z-buffer renders of Takt on a light floor, with shadows.
Run with FreeCAD's python (numpy + Pillow): python render.py [shot ...]   (no args: every shot)
Reads meshes/parts.npz and meshes/scenes.json written by the CAD script (takt.py); writes renders/*.png."""
import json, os, sys, math
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
SS = 2   # supersampling factor

# Linear-light albedo, specular strength and exponent per material.
MAT = dict(black=((0.040, 0.040, 0.044), 0.10, 18), servo=((0.026, 0.026, 0.029), 0.16, 30), red=((0.55, 0.035, 0.04), 0.12, 30),
           eye=((1.0, 1.0, 0.97), 0.0, 1), glass=((0.010, 0.010, 0.013), 0.55, 160), tpu=((0.030, 0.030, 0.032), 0.06, 10),
           white=((0.80, 0.80, 0.78), 0.10, 20), lens=((0.02, 0.02, 0.03), 0.6, 120), cable=((0.018, 0.018, 0.02), 0.25, 40),
           floor=((0.60, 0.56, 0.50), 0.03, 8))
EMISSIVE = {"eye": 0.95}
NAMES = list(MAT)
LIGHT = np.array([-0.30, 0.22, 0.93])
LIGHT /= np.linalg.norm(LIGHT)
FILL = np.array([0.6, -0.3, 0.45])
FILL /= np.linalg.norm(FILL)
BG_TOP, BG_LOW = np.array([0.80, 0.80, 0.79]), np.array([0.90, 0.89, 0.87])


def load():
    parts = dict(np.load(os.path.join(HERE, "meshes", "parts.npz")))
    scenes = json.load(open(os.path.join(HERE, "meshes", "scenes.json")))
    return parts, scenes


def cable_mesh(a, b, r=1.3, sides=8, segs=24):
    """Sagging tube between two plug points: a loop of black servo cable (drawn, not routed)."""
    a, b = np.array(a, float), np.array(b, float)
    d = np.linalg.norm(b - a)
    mid = (a + b) / 2 + np.array([0, 0, -(8.0 + 0.35 * d)])
    out = (a + b) / 2
    out[2] = 0
    n = np.linalg.norm(out[:2])
    if n > 1e-6:
        mid[:2] += out[:2] / n * (6.0 + 0.15 * d)
    t = np.linspace(0, 1, segs + 1)[:, None]
    pts = (1 - t) ** 2 * a + 2 * (1 - t) * t * mid + t ** 2 * b
    tris = []
    for i in range(segs):
        p0, p1 = pts[i], pts[i + 1]
        tan = p1 - p0
        tan /= np.linalg.norm(tan) + 1e-9
        u = np.cross(tan, [0, 0, 1.0])
        if np.linalg.norm(u) < 1e-3:
            u = np.cross(tan, [1.0, 0, 0])
        u /= np.linalg.norm(u)
        v = np.cross(tan, u)
        ring = lambda p: [p + r * (math.cos(2 * math.pi * k / sides) * u + math.sin(2 * math.pi * k / sides) * v) for k in range(sides)]
        r0, r1 = ring(p0), ring(p1)
        for k in range(sides):
            k2 = (k + 1) % sides
            tris.append([r0[k], r0[k2], r1[k]])
            tris.append([r0[k2], r1[k2], r1[k]])
    return np.array(tris, dtype=np.float32)


def build(scene, parts, cables=True):
    tris, mats = [], []
    for key, color, m in scene["items"]:
        if key not in parts:
            continue
        M = np.array(m, dtype=np.float64).reshape(3, 4)
        t = parts[key].reshape(-1, 3) @ M[:, :3].T + M[:, 3]
        tris.append(t.reshape(-1, 3, 3))
        mats.append(np.full(len(t) // 3, NAMES.index(color), dtype=np.int16))
    if cables:
        for a, b in scene["cables"]:
            c = cable_mesh(a, b)
            tris.append(c)
            mats.append(np.full(len(c), NAMES.index("cable"), dtype=np.int16))
    return np.concatenate(tris).astype(np.float64), np.concatenate(mats)


def look_at(eye, target, up=(0, 0, 1)):
    f = np.array(target, float) - np.array(eye, float)
    f /= np.linalg.norm(f)
    r = np.cross(f, up)
    if np.linalg.norm(r) < 1e-6:
        r = np.cross(f, (0, 1, 0))
    r /= np.linalg.norm(r)
    u = np.cross(r, f)
    return np.array([r, u, f])


def raster(xy, z, W, H, ids=None, zbuf=None, idbuf=None, persp=True):
    """Fill a depth buffer (and triangle ids). xy: (N,3,2) screen coords, z: (N,3) depth."""
    zbuf = np.full((H, W), np.inf, np.float32) if zbuf is None else zbuf
    idbuf = np.full((H, W), -1, np.int32) if idbuf is None else idbuf
    x0 = np.clip(np.floor(xy[:, :, 0].min(1)), 0, W - 1).astype(int)
    x1 = np.clip(np.ceil(xy[:, :, 0].max(1)), 0, W - 1).astype(int)
    y0 = np.clip(np.floor(xy[:, :, 1].min(1)), 0, H - 1).astype(int)
    y1 = np.clip(np.ceil(xy[:, :, 1].max(1)), 0, H - 1).astype(int)
    keep = (x1 >= x0) & (y1 >= y0) & (xy[:, :, 0].max(1) >= 0) & (xy[:, :, 0].min(1) < W) & (xy[:, :, 1].max(1) >= 0) & (xy[:, :, 1].min(1) < H)
    for i in np.nonzero(keep)[0]:
        a, b, c = xy[i]
        d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(d) < 1e-9:
            continue
        X, Y = np.meshgrid(np.arange(x0[i], x1[i] + 1) + 0.5, np.arange(y0[i], y1[i] + 1) + 0.5)
        w0 = ((b[1] - c[1]) * (X - c[0]) + (c[0] - b[0]) * (Y - c[1])) / d
        w1 = ((c[1] - a[1]) * (X - c[0]) + (a[0] - c[0]) * (Y - c[1])) / d
        w2 = 1 - w0 - w1
        m = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6)
        if not m.any():
            continue
        if persp:
            zz = 1.0 / (w0[m] / z[i, 0] + w1[m] / z[i, 1] + w2[m] / z[i, 2])
        else:
            zz = w0[m] * z[i, 0] + w1[m] * z[i, 1] + w2[m] * z[i, 2]
        ys, xs = (Y[m] - 0.5).astype(int), (X[m] - 0.5).astype(int)
        near = zz < zbuf[ys, xs]
        zbuf[ys[near], xs[near]] = zz[near]
        idbuf[ys[near], xs[near]] = i if ids is None else ids[i]
    return zbuf, idbuf


def shadow_map(tris, res=2600):
    """Orthographic depth from the key light over the robot's footprint."""
    R = look_at(LIGHT * 1000, (0, 0, 0))
    p = tris.reshape(-1, 3) @ R.T
    lo, hi = p[:, :2].min(0) - 5, p[:, :2].max(0) + 5
    s = (res - 1) / max(hi - lo)
    xy = (p[:, :2] - lo) * s
    xy[:, 1] = res - 1 - xy[:, 1]
    zb, _ = raster(xy.reshape(-1, 3, 2), p[:, 2].reshape(-1, 3), res, res, persp=False)
    return R, lo, s, res, zb


def render(scene, parts, cam, size=(1600, 1100), name="out", cables=True, floor=True):
    W, H = size[0] * SS, size[1] * SS
    tris, mats = build(scene, parts, cables)
    smap = shadow_map(tris)
    eye, target, fov = np.array(cam["eye"], float), np.array(cam["target"], float), cam["fov"]
    R = look_at(eye, target)
    if floor:
        # Floor as a grid; cells behind the camera are dropped, so no triangle straddles the eye plane.
        g = np.linspace(-2000.0, 2000.0, 41)
        cells = []
        for i in range(40):
            for j in range(40):
                a0, a1, b0, b1 = g[i], g[i + 1], g[j], g[j + 1]
                cells += [[[a0, b0, 0], [a1, b0, 0], [a1, b1, 0]], [[a0, b0, 0], [a1, b1, 0], [a0, b1, 0]]]
        fl = np.array(cells, float)
        front = (((fl.reshape(-1, 3) - eye) @ R.T)[:, 2].reshape(-1, 3) > 20.0).all(1)
        fl = fl[front]
        tris = np.concatenate([tris, fl])
        mats = np.concatenate([mats, np.full(len(fl), NAMES.index("floor"), np.int16)])
    f = (W / 2) / math.tan(math.radians(fov) / 2)
    p = (tris.reshape(-1, 3) - eye) @ R.T
    zc = np.maximum(p[:, 2], 1.0)
    xy = np.stack([W / 2 + f * p[:, 0] / zc, H / 2 - f * p[:, 1] / zc], 1).reshape(-1, 3, 2)
    zb, ib = raster(xy, zc.reshape(-1, 3), W, H)
    # Per-pixel world position from depth.
    X, Y = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    dirs = np.stack([(X - W / 2) / f, -(Y - H / 2) / f, np.ones_like(X)], -1)
    hit = ib >= 0
    world = eye + (dirs * np.where(hit, zb, 0)[..., None]) @ R
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    N = n[np.where(hit, ib, 0)]
    V = eye - world
    V /= np.linalg.norm(V, axis=-1, keepdims=True) + 1e-9
    flip = (N * V).sum(-1) < 0
    N[flip] *= -1
    M = mats[np.where(hit, ib, 0)]
    # Shadows from the key light.
    LR, lo, s, res, lz = smap
    lp = world @ LR.T
    u = np.clip(((lp[..., 0] - lo[0]) * s).astype(int), 0, res - 1)
    v = np.clip((res - 1 - (lp[..., 1] - lo[1]) * s).astype(int), 0, res - 1)
    bias = 1.2 + 2.5 * (1 - np.abs((N * LIGHT).sum(-1)))
    uf = (lp[..., 0] - lo[0]) * s
    vf = res - 1 - (lp[..., 1] - lo[1]) * s
    inside = (uf >= 0) & (uf <= res - 1) & (vf >= 0) & (vf <= res - 1)
    # Soften: 3x3 percentage-closer filter; outside the map nothing casts a shadow.
    acc = np.zeros(hit.shape, np.float32)
    for du in (-2, 0, 2):
        for dv in (-2, 0, 2):
            acc += lp[..., 2] <= lz[np.clip(v + dv, 0, res - 1), np.clip(u + du, 0, res - 1)] + bias
    lit = np.where(inside, acc / 9.0, 1.0)
    alb = np.array([MAT[k][0] for k in NAMES])[M]
    ks = np.array([MAT[k][1] for k in NAMES])[M]
    ex = np.array([MAT[k][2] for k in NAMES])[M]
    ndl = np.clip((N * LIGHT).sum(-1), 0, 1)
    ndf = np.clip((N * FILL).sum(-1), 0, 1)
    Hh = LIGHT + V
    Hh /= np.linalg.norm(Hh, axis=-1, keepdims=True) + 1e-9
    spec = ks * np.clip((N * Hh).sum(-1), 0, 1) ** ex * lit
    rim = 0.05 * (1 - np.clip((N * V).sum(-1), 0, 1)) ** 3
    sky = 0.30 + 0.20 * N[..., 2]
    col = alb * (sky + 1.05 * ndl * lit + 0.30 * ndf)[..., None] + (spec + rim)[..., None]
    em = np.array([EMISSIVE.get(k, 0.0) for k in NAMES])[M]
    col = col * (1 - em[..., None]) + em[..., None] * np.array(MAT["eye"][0])
    # Floor fades into the background far from the robot.
    is_floor = M == NAMES.index("floor")
    dist = np.linalg.norm(world[..., :2], axis=-1)
    fade = np.clip((dist - 650) / 1100, 0, 1)
    sky_col = BG_TOP + (BG_LOW - BG_TOP) * np.clip(Y / H, 0, 1)[..., None]
    col = np.where((is_floor & hit)[..., None], col * (1 - fade[..., None]) + sky_col * fade[..., None], col)
    col = np.where(hit[..., None], col, sky_col)
    # Outline on depth jumps and creases so black parts read against each other.
    zz = np.where(hit, zb, 1e5)
    jump = np.zeros_like(hit)
    for dy, dx in ((0, 1), (1, 0)):
        a, b = zz, np.roll(np.roll(zz, dy, 0), dx, 1)
        jump |= np.abs(a - b) > 0.02 * np.minimum(a, b) + 2.0
    crease = np.zeros_like(hit)
    for dy, dx in ((0, 1), (1, 0)):
        Nb = np.roll(np.roll(N, dy, 0), dx, 1)
        crease |= (N * Nb).sum(-1) < 0.75
    robot = hit & ~is_floor
    col = np.where((jump & robot)[..., None], col * 0.35, col)
    col = np.where((crease & robot & ~jump)[..., None], col + 0.018, col)
    img = np.clip(col, 0, 1) ** (1 / 2.2)
    img = (img * 255).astype(np.float32)
    img = img.reshape(size[1], SS, size[0], SS, 3).mean((1, 3))
    out = os.path.join(HERE, "renders", name + ".png")
    Image.fromarray(img.astype(np.uint8)).save(out)
    print("wrote", out, len(tris), "triangles")
    return out


def orbit(target, dist, az, el, fov):
    """Camera `dist` from target; az from the robot's front (+y) toward its right (+x), el above the floor."""
    a, e = math.radians(az), math.radians(el)
    eye = np.array(target) + dist * np.array([math.sin(a) * math.cos(e), math.cos(a) * math.cos(e), math.sin(e)])
    return dict(eye=eye.tolist(), target=list(target), fov=fov)


SHOTS = {
    "stand_front34": ("stand", orbit((0, 0, 78), 720, 10, 14, 34), (1600, 1100)),
    "face": ("stand", orbit((0, 66, 98), 230, 14, 9, 24), (1400, 1000)),
    "side": ("stand", orbit((0, 0, 95), 1500, -90, 6, 20), (1600, 900)),
    "top": ("stand", dict(eye=[0.001, -0.5, 950], target=[0, 0, 0], fov=26), (1300, 1300)),
    "sit": ("sit", orbit((0, 0, 50), 700, 28, 18, 38), (1600, 1100)),
    "low_stand": ("low_stand", orbit((0, 0, 55), 700, 28, 16, 38), (1600, 1100)),
    "wave": ("wave", orbit((-40, 30, 115), 900, 4, 10, 36), (1600, 1100)),
    "wave_2": ("wave_2", orbit((-40, 30, 115), 900, 4, 10, 36), (1600, 1100)),
    "look_up": ("look_up", orbit((0, 0, 80), 660, 30, 12, 36), (1600, 1100)),
    "look_around": ("look_around", orbit((0, 0, 70), 660, 24, 16, 36), (1600, 1100)),
    "walk_00": ("walk_00", orbit((0, 0, 70), 700, 55, 14, 36), (1400, 1000)),
    "walk_03": ("walk_03", orbit((0, 0, 70), 700, 55, 14, 36), (1400, 1000)),
    "walk_06": ("walk_06", orbit((0, 0, 70), 700, 55, 14, 36), (1400, 1000)),
    "walk_09": ("walk_09", orbit((0, 0, 70), 700, 55, 14, 36), (1400, 1000)),
    "turn_03": ("turn_03", orbit((0, 0, 70), 700, 35, 30, 36), (1400, 1000)),
    "sneaker": ("sneaker_stand", orbit((0, 0, 85), 740, 12, 13, 34), (1600, 1100)),
}


def sheet(names, out, cols=2):
    ims = [Image.open(os.path.join(HERE, "renders", n + ".png")) for n in names]
    w, h = ims[0].size
    rows = (len(ims) + cols - 1) // cols
    s = Image.new("RGB", (w * cols, h * rows), (230, 230, 228))
    for i, im in enumerate(ims):
        s.paste(im, ((i % cols) * w, (i // cols) * h))
    s.save(os.path.join(HERE, "renders", out + ".png"))


if __name__ == "__main__":
    os.makedirs(os.path.join(HERE, "renders"), exist_ok=True)
    parts, scenes = load()
    todo = sys.argv[1:] or list(SHOTS)
    for shot in todo:
        sc, cam, size = SHOTS[shot]
        render(scenes[sc], parts, cam, size, shot)
    if not sys.argv[1:]:
        sheet(["walk_00", "walk_03", "walk_06", "walk_09"], "gait_walk_sheet")
