# Takt: a six-legged walking robot
# SPDX-License-Identifier: CERN-OHL-S-2.0
# Source location: https://github.com/TimFKiefer/how-to-vibecode-a-robot
# Run: nice -n 15 freecadcmd cad/takt.py      (STAGE=parts stops after the parts)
# Writes parts/ (STL + STEP, already in print orientation), meshes/ (for render.py), report.json, robot2.log.
# Frames: body X right, Y front, Z up, origin at the belly centre (z=0 is the underside of the base floor).
#   Coxa frame: x out along the leg, y tangential (counter-clockwise), z up.
#   Femur frame: x from the femur axis to the knee, y toward the lower side of the femur, z = coxa y.
#   Tibia frame: the femur frame turned about z by the knee angle; x is the chord from the knee to the foot tip.
# Deliberate simplifications (each one is named again where it happens):
#   - loads are static: gravity only, no acceleration, no foot friction, no cable drag;
#   - electronics are keep-out boxes of the real parts' outlines, with their listed masses;
#   - cables are drawn as loops in the renders but not routed and not collision-checked;
#   - the servo case is boxes: nose block, tail block, lip, bump, horns and plugs, from the spec sheet drawing.
import json, math, os, sys, time
import FreeCAD as App, Part
from FreeCAD import Vector as V, Matrix

OUT = os.environ.get("TAKT_OUT") or os.path.dirname(os.path.abspath(__file__ if "__file__" in globals() else sys.argv[-1]))
STAGE = os.environ.get("STAGE", "all")
LOG = open(os.path.join(OUT, "robot2.log"), "w")
T0 = time.time()


def log(*a):
    LOG.write("%7.1fs  %s\n" % (time.time() - T0, " ".join(str(x) for x in a)))
    LOG.flush()


# --- print tuning: change these after the fit test, not the geometry below ---
FIT = 0.25          # gap per side around a servo case
WALL = 2.5
PLATE = 4.0         # bracket plate thickness
M2_CLEAR = 2.3      # M2 / PA2.0 screw through our part into the servo case
M3_CLEAR = 3.3      # M3 through hole (horn screws, blade bolts)
M3_INSERT = 4.0     # hole for an M3 heat-set insert
M25_INSERT = 3.6    # hole for an M2.5 heat-set insert
INFILL = 0.75       # effective density factor
PETG, TPU, ACRYLIC = 1.27, 1.21, 1.19   # g/cm3

# --- Feetech STS3215, from the spec sheet A/0 2020-04-10, section 9 (outline) and 10 (horns) ---
S_NOSE = 10.2       # case end beyond the shaft axis; 45.23 overall.  # UNVERIFIED: read off the drawing raster
S_TAIL = 45.23 - S_NOSE
S_W = 24.73 / 2
S_STEP = -5.5       # where the 29 mm nose block meets the 32 mm tail block.  # UNVERIFIED: raster
S_H_NOSE = 29.0     # back face to output face around the shaft (drawing: 29)
S_MID = S_H_NOSE / 2
S_TAIL_Z0, S_TAIL_Z1 = -1.5, 30.5    # tail block 32 tall (drawing), 1.5 proud of both faces.  # UNVERIFIED split
S_LIP = 31.5        # top lip at the tail end; with the bump this gives the 35 mm overall height.  # UNVERIFIED
S_BUMP = (-29.7, -17.8, -3.5)        # x range and depth of the bump under the tail.  # UNVERIFIED: raster
SPLINE_D, SPLINE_H = 5.9, 3.4        # 25T, OD 5.9, 3.4 proud
BOSS_D, BOSS_H = 6.0, 4.1            # rear dummy shaft, 6 mm, 4.1 proud
HORN_D = 19.95                       # both aluminium horns: OD 19.95, 4 x M3 tapped on 14 mm
HORN_PCD = 14.0
HORN_FACE = S_H_NOSE + 4.5           # output horn face above the back face.  # UNVERIFIED: horn is 4.5 thick, seat height not drawn
IDLER_FACE = -3.7                    # idler horn face below the back face.  # UNVERIFIED: horn is 3.1 thick on a 4.1 boss
HF = HORN_FACE - S_MID               # 19.0: horn face from the case mid plane
IF = IDLER_FACE - S_MID              # -18.2
S_FACE = S_TAIL_Z1 - S_MID           # 16.0: tail faces from the mid plane
HOLE_TOP = [(-8.4, 10.4), (-28.8, 10.4)]   # PA2.0 holes in the output face, (x, +-y).  # UNVERIFIED: raster
HOLE_BOT = [(-8.3, 10.4), (-32.7, 10.4)]   # PA2.0 holes in the back face.  # UNVERIFIED: raster
PLUG = (-18.0, -10.5, 9.0, 7.0)    # two 5264 sockets on the back face: x range, half width, plug + bend depth.  # UNVERIFIED depth
MASS_SERVO = 60.0                  # 55 +-1 g (spec 6-8) plus both horns and screws, about 5 g (estimate)
STALL = {6.0: 16.5, 7.4: 19.5}     # kg.cm, spec 5-4
RATED = {6.0: 4.0, 7.4: 5.0}       # kg.cm, spec 5-8
V_SERVO = 6.0                      # the servos run behind a 6 V regulator: 8.4 V full 2S is above the 4-7.4 V input range

# --- geometry ---
R_V = 77.0          # hex circumradius
APO = R_V * math.cos(math.radians(30))
R_CORNER = 10.0     # rounding of the vertical body edges
R_HIP = 74.0        # coxa servo axis radius
Z_S1 = 12.0         # coxa servo back face above the belly
FLOOR = 2.5
BASE_TOP = 61.0
LID_TOP = 79.0
LID_TOP_APO = 62.0 * math.cos(math.radians(30))
L_COXA = 33.2       # coxa axis to femur axis
Z_F = 44.0          # femur axis height
L_FEMUR = 66.0
L_TIBIA = 120.0     # knee axis to the centre of the tip rounding
R_TIP = 4.0
T_BLADE = 16.0      # blade thickness along the knee axis
HIPS = [("RF", 60.0, 15.0), ("RM", 0.0, 0.0), ("RR", -60.0, -15.0),
        ("LF", 120.0, -15.0), ("LM", 180.0, 0.0), ("LR", 240.0, 15.0)]   # name, hip angle, coxa case turned away from the face
TRI_A, TRI_B = ("RF", "LM", "RR"), ("LF", "RM", "LR")
COXA_RANGE = 40.0   # coxa bracket vs its own servo case, +- degrees (checked by the sweep)
TH2_RANGE = (-20.0, 100.0)    # at 110 the femur reaches the lid edge (sweep)
BETA_RANGE = (70.0, 155.0)    # 65: femur plate meets the cradle; 160: blade block reaches the femur rib (sweep)
BLADE_CURVE = [(30.0, 0.0), (67.5, 0.0), (96.0, -22.5), (L_TIBIA, 0.0)]   # sickle centreline (tibia frame), the 150 mm curve scaled
S_JOINT = 0.8       # where the blade hands over to the foot tip


# --- helpers ---
def box(x0, x1, y0, y1, z0, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl(r, z0, z1, x=0.0, y=0.0):
    return Part.makeCylinder(r, z1 - z0, V(x, y, z0))


def cylx(r, x0, x1, y=0.0, z=0.0):
    return Part.makeCylinder(r, x1 - x0, V(x0, y, z), V(1, 0, 0))


def cyly(r, y0, y1, x=0.0, z=0.0):
    return Part.makeCylinder(r, y1 - y0, V(x, y0, z), V(0, 1, 0))


def poly_face(pts, z=0.0):
    return Part.Face(Part.makePolygon([V(x, y, z) for x, y in pts] + [V(pts[0][0], pts[0][1], z)]))


def prism(pts, z0, z1):
    return poly_face(pts, z0).extrude(V(0, 0, z1 - z0))


def single(s):
    s = s.removeSplitter()
    if s.ShapeType != "Solid" and len(s.Solids) == 1:
        return s.Solids[0]
    return s


def fuse(*shapes):
    shapes = [s for s in shapes if s is not None]
    if len(shapes) == 1:
        return single(shapes[0])
    return single(shapes[0].multiFuse(list(shapes[1:])))


def cut(base, *shapes):
    shapes = [s for s in shapes if s is not None]
    return single(base.cut(list(shapes))) if shapes else base


def T(x=0.0, y=0.0, z=0.0):
    m = Matrix()
    m.move(V(x, y, z))
    return m


def Rx(d):
    m = Matrix()
    m.rotateX(math.radians(d))
    return m


def Ry(d):
    m = Matrix()
    m.rotateY(math.radians(d))
    return m


def Rz(d):
    m = Matrix()
    m.rotateZ(math.radians(d))
    return m


def chain(*ms):
    out = Matrix()
    for m in ms:
        out = out.multiply(m)
    return out


def frame(o, ex, ey, ez):
    return Matrix(ex.x, ey.x, ez.x, o.x, ex.y, ey.y, ez.y, o.y, ex.z, ey.z, ez.z, o.z, 0, 0, 0, 1)


# Faces in the x-z plane extruded along +y: local y of the face maps to -z (keeps the frame right-handed).
def to_xz(x, y0, z):
    return frame(V(x, y0, z), V(1, 0, 0), V(0, 0, -1), V(0, 1, 0))


def rrect_face(w, h, r):
    f = poly_face([(-w / 2 + r, -h / 2 + r), (w / 2 - r, -h / 2 + r), (w / 2 - r, h / 2 - r), (-w / 2 + r, h / 2 - r)])
    return f.makeOffset2D(r)


def rrect_y(w, h, r, x, z, y0, y1):
    """Rounded rectangle in the x-z plane, centred on (x, z), from y0 to y1."""
    return rrect_face(w, h, r).extrude(V(0, 0, y1 - y0)).transformed(to_xz(x, y0, z))


def rhex_face(apothem, r, z=0.0):
    """Hexagon with a vertex on +x, flat faces front and back, corners rounded with radius r."""
    a = apothem - r
    R = a / math.cos(math.radians(30))
    f = poly_face([(R * math.cos(math.radians(60 * k)), R * math.sin(math.radians(60 * k))) for k in range(6)], z)
    return f.makeOffset2D(r) if r > 0 else f


def circle_pts(cx, cy, r, n=48):
    return [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def hull2d(pts):
    pts = sorted(set((round(x, 6), round(y, 6)) for x, y in pts))
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and cross(out[-2], out[-1], p) <= 0:
                out.pop()
            out.append(p)
        return out[:-1]
    return half(pts) + half(pts[::-1])


def rect_pts(x0, x1, y0, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def horn_holes(z0, z1, idler=False, x=0.0, y=0.0):
    """4 x M3 into the horn's tapped holes, plus the centre: screwdriver access, or room for the idler screw head."""
    r = HORN_PCD / 2
    holes = [cyl(M3_CLEAR / 2, z0, z1, x + r * math.cos(math.radians(a)), y + r * math.sin(math.radians(a))) for a in (45, 135, 225, 315)]
    return holes + [cyl(4.0 if idler else 3.25, z0, z1, x, y)]


# --- the servo ---
def servo():
    """STS3215 with both horns and the plugs. Shaft on +z at (0,0), back face of the nose block at z=0, nose toward +x."""
    return fuse(box(S_STEP, S_NOSE, -S_W, S_W, 0, S_H_NOSE),
                box(-S_TAIL, S_STEP + 0.01, -S_W, S_W, S_TAIL_Z0, S_TAIL_Z1),
                box(-S_TAIL, -S_TAIL + 3, -S_W, S_W, S_TAIL_Z1 - 0.01, S_LIP),
                box(S_BUMP[0], S_BUMP[1], -9, 9, S_BUMP[2], S_TAIL_Z0 + 0.01),
                cyl(SPLINE_D / 2, S_H_NOSE - 0.01, S_H_NOSE + SPLINE_H),
                cyl(4.5, S_H_NOSE + 0.5, HORN_FACE - 2.49), cyl(HORN_D / 2, HORN_FACE - 2.5, HORN_FACE),
                cyl(BOSS_D / 2, -BOSS_H, 0.01), cyl(4.5, IDLER_FACE + 2.09, -0.6), cyl(HORN_D / 2, IDLER_FACE, IDLER_FACE + 2.1),
                cyl(2.75, -BOSS_H - 2.0, -BOSS_H + 0.01),
                box(PLUG[0], PLUG[1], -PLUG[2], PLUG[2], S_TAIL_Z0 - PLUG[3], S_TAIL_Z0 + 0.01))


def servo_label():
    """Red sticker on the side of the case (render only)."""
    return fuse(box(-27.0, -17.0, S_W - 0.01, S_W + 0.3, 12.0, 20.0), box(-27.0, -17.0, -S_W - 0.3, -S_W + 0.01, 12.0, 20.0))


# --- leg parts ---
COXA_W0, COXA_W1 = 16.5, 20.5      # web of the coxa bracket, clear of the S1 nose corners (16.0) at any yaw
CRADLE_Y = S_FACE + FIT + PLATE    # 20.25


def coxa():
    """U bracket on S1 (horn on top, idler below) with the cradle that holds S2 by its tail. Coxa frame."""
    zt0, zt1 = Z_S1 + HORN_FACE, Z_S1 + HORN_FACE + PLATE
    zb0, zb1 = Z_S1 + IDLER_FACE - PLATE, Z_S1 + IDLER_FACE
    top = fuse(prism(hull2d(circle_pts(0, 0, 12.5) + rect_pts(10.0, COXA_W0 + 0.1, -12.0, 12.0)), zt0, zt1),
               box(COXA_W0, COXA_W1, -14.0, 14.0, zt0, zt1))
    bot = fuse(prism(hull2d(circle_pts(0, 0, 10.2) + rect_pts(10.0, COXA_W0, -8.0, 8.0)), zb0, zb1),
               prism(hull2d(rect_pts(10.0, COXA_W0 + 0.01, -8.0, 8.0) + rect_pts(COXA_W0, COXA_W1, -14.0, 14.0)), zb0, zb1))   # narrow neck: S1's plugs
    z_cheek = Z_F - 20.0           # below the femur plates down to -20 deg (sweep)
    web = fuse(box(COXA_W0, COXA_W1, -14.0, 14.0, zb0, zt1), box(COXA_W0, COXA_W1, -CRADLE_Y, CRADLE_Y, zb0, z_cheek))
    xo = L_COXA + S_W + FIT
    floor = box(COXA_W0, xo + WALL, -CRADLE_Y, CRADLE_Y, zb0, Z_F - S_TAIL - FIT)
    outer = box(xo, xo + WALL, -CRADLE_Y, CRADLE_Y, zb0, z_cheek)
    cheek_p = box(COXA_W0, xo + WALL, S_FACE + FIT, CRADLE_Y, zb0, z_cheek)
    cheek_m = box(COXA_W0, xo + WALL, -CRADLE_Y, -(S_FACE + FIT), zb0, min(z_cheek, Z_F + PLUG[0] - 1.5))   # under S2's plugs
    out = fuse(top, bot, web, floor, outer, cheek_p, cheek_m)
    cuts = horn_holes(zt0 - 1, zt1 + 1) + horn_holes(zb0 - 1, zb1 + 1, idler=True)
    # S2 screws: output face holes through the +y cheek, back face holes through the -y cheek.
    for hx, hy in HOLE_TOP[1:]:
        for s in (1, -1):
            cuts.append(cyly(M2_CLEAR / 2, S_FACE - 1, CRADLE_Y + 1, L_COXA + s * hy, Z_F + hx))
    for hx, hy in HOLE_BOT[1:]:
        for s in (1, -1):
            cuts.append(cyly(M2_CLEAR / 2, -CRADLE_Y - 1, -S_FACE + 1, L_COXA + s * hy, Z_F + hx))
    # Reliefs for S2's tail lip (output side) and bump (back side).
    cuts.append(box(xo - 2 * S_W - 0.5, xo + 0.1, S_FACE - 0.1, S_LIP - S_MID + FIT, Z_F - S_TAIL - FIT - 0.1, Z_F - S_TAIL + 3 + FIT))
    cuts.append(box(L_COXA - 9.25, L_COXA + 9.25, -S_MID + S_BUMP[2] - FIT, -S_FACE + 0.1, Z_F + S_BUMP[0] - FIT, Z_F + S_BUMP[1] + FIT))
    return cut(out, *cuts)


def femur():
    """Link from S2's horns (femur axis) to S3's horns (knee): two plates and a rib along the upper edge. Femur frame.
    Printed on that upper edge. The idler-side plate is narrow where S2's and S3's plugs pass."""
    K = L_FEMUR
    plus = prism(hull2d(circle_pts(0, 0, 12.2) + circle_pts(K, 0, 12.2)), HF, HF + PLATE)
    minus = fuse(prism(circle_pts(0, 0, 10.2), IF - PLATE, IF), prism(circle_pts(K, 0, 10.2), IF - PLATE, IF),
                 prism(rect_pts(0.0, 45.0, -12.2, 6.0), IF - PLATE, IF), prism(rect_pts(44.9, K, -6.0, 6.0), IF - PLATE, IF))
    rib = box(18.0, 38.0, -12.2, -4.0, IF - 0.01, HF + 0.01)   # ends early: the blade block passes it at full knee bend
    out = fuse(plus, minus, rib)
    cuts = horn_holes(HF - 1, HF + PLATE + 1) + horn_holes(IF - PLATE - 1, IF + 1, idler=True)
    cuts += horn_holes(HF - 1, HF + PLATE + 1, x=K) + horn_holes(IF - PLATE - 1, IF + 1, idler=True, x=K)
    return cut(out, *cuts)


KNEE_WEB = (17.0, 30.0, 15.0)      # x0, x1, half width of the block that carries the blade
TONGUE = (24.0, 12.5, T_BLADE / 2) # blade tongue: top x, half depth, half thickness
S3_DIR = 20.0      # S3's nose in the tibia frame; its tail, and the frame around it, lean 20 deg outboard of the blade


def knee():
    """Top of the tibia: a frame around S3 (cradle on its tail, outboard wall with a window, cap) and the block the
    blade plugs into. The femur bolts to S3's horns from the open inboard side. Tibia frame."""
    xe = -S_TAIL - FIT
    zp0, zp1 = S_TAIL_Z1 + FIT, S_TAIL_Z1 + FIT + PLATE
    zm0, zm1 = S_TAIL_Z0 - FIT - PLATE, S_TAIL_Z0 - FIT
    yo1 = -(S_W + FIT)
    yo0 = yo1 - WALL
    yi = S_W + FIT + 1.0
    plate_p = box(xe - PLATE, -13.0, yo0, yi, zp0, zp1)
    plate_m = box(xe - PLATE, -20.0, yo0, yi, zm0, zm1)          # stops short of the plugs
    cap = box(xe - PLATE, xe, yo0, yi, zm0, zp1)
    wall = fuse(box(xe - PLATE, -12.9, yo0, yo1, zm0, zp1), box(-13.0, 17.0, yo0, yo1, S_TAIL_Z0, S_TAIL_Z1))
    frame_s = fuse(plate_p, plate_m, cap, wall)
    cuts = [box(xe - 0.1, -S_TAIL + 3 + FIT, -S_W - FIT, S_W + FIT, S_TAIL_Z1 - 0.1, S_LIP + FIT),          # tail lip
            box(S_BUMP[0] - FIT, S_BUMP[1] + FIT, -9.25, 9.25, S_BUMP[2] - FIT, S_TAIL_Z0 + 0.1),        # bump
            box(-28.5, -15.5, yo0 - 1, yo1 + 1, 10.0, 22.0)]                                             # window: the red label shows
    for hx, hy in HOLE_TOP[1:]:
        cuts += [cyl(M2_CLEAR / 2, zp0 - 1, zp1 + 1, hx, s * hy) for s in (1, -1)]
    for hx, hy in HOLE_BOT[1:]:
        cuts += [cyl(M2_CLEAR / 2, zm0 - 1, zm1 + 1, hx, s * hy) for s in (1, -1)]
    frame_t = cut(frame_s, *cuts).transformed(chain(Rz(S3_DIR), T(0, 0, -S_MID)))
    x0, x1, w = KNEE_WEB
    block = box(x0, x1, -w, w, -12.0, 12.0)
    block = block.cut(prism([(x0 - 1, w - 6.0), (x0 + 6.0, w + 1), (x0 - 1, w + 1)], -13, 13))   # chamfer where the femur rib passes
    out = fuse(frame_t, block)
    cuts = [box(TONGUE[0], x1 + 1, -TONGUE[1] - 0.3, TONGUE[1] + 0.3, -TONGUE[2] - 0.3, TONGUE[2] + 0.3)]
    for y in (6.0, -6.0):     # countersunk M3 x 16 into inserts in the blade, fitted before S3 goes in
        cuts.append(cylx(M3_CLEAR / 2, x0 - 1, TONGUE[0] + 0.5, y, 0))
        cuts.append(Part.makeCone(3.3, 1.65, 1.7, V(x0 - 0.01, y, 0), V(1, 0, 0)))
    return cut(out, *cuts)


def bezier(s, p=BLADE_CURVE):
    a, b, c, d = (1 - s) ** 3, 3 * (1 - s) ** 2 * s, 3 * (1 - s) * s * s, s ** 3
    return (a * p[0][0] + b * p[1][0] + c * p[2][0] + d * p[3][0], a * p[0][1] + b * p[1][1] + c * p[2][1] + d * p[3][1])


def bezier_d(s, p=BLADE_CURVE):
    a, b, c = 3 * (1 - s) ** 2, 6 * (1 - s) * s, 3 * s * s
    return (a * (p[1][0] - p[0][0]) + b * (p[2][0] - p[1][0]) + c * (p[3][0] - p[2][0]),
            a * (p[1][1] - p[0][1]) + b * (p[2][1] - p[1][1]) + c * (p[3][1] - p[2][1]))


def half_depth(s):
    return R_TIP + (TONGUE[1] - R_TIP) * (1 - s) ** 0.85


def sickle(s0, s1, n=40, extra=0.0):
    left, right = [], []
    for i in range(n + 1):
        s = s0 + (s1 - s0) * i / n
        (x, y), (dx, dy) = bezier(s), bezier_d(s)
        L = math.hypot(dx, dy)
        nx, ny = -dy / L, dx / L
        h = half_depth(s) + extra
        left.append((x + nx * h, y + ny * h))
        right.append((x - nx * h, y - ny * h))
    return left + right[::-1]


def joint_frame():
    """Where the blade hands over to the foot: origin on the centreline, x along the tangent."""
    (x, y), (dx, dy) = bezier(S_JOINT), bezier_d(S_JOINT)
    a = math.degrees(math.atan2(dy, dx))
    return chain(T(x, y, 0), Rz(a))


FOOT_TONGUE = (7.0, 3.5, 4.5)      # length, half depth, half thickness of the blade's tongue into the foot


def blade():
    """Sickle blade between the knee web and the foot tip. Printed flat, 16 mm thick. Tibia frame."""
    t = T_BLADE / 2
    body = prism(sickle(0.0, S_JOINT, 48), -t, t)
    top = box(TONGUE[0], 30.5, -TONGUE[1], TONGUE[1], -t, t)
    J = joint_frame()
    L, hd, ht = FOOT_TONGUE
    tongue = box(-0.5, L, -hd, hd, -ht, ht).transformed(J)
    out = fuse(body, top, tongue)
    # Lightening slot through the blade, along the centreline (visible from the side).
    pts = []
    for s in [0.10 + 0.40 * i / 20 for i in range(21)]:
        (x, y), (dx, dy) = bezier(s), bezier_d(s)
        l = math.hypot(dx, dy)
        pts.append((x - dy / l * 3.0, y + dx / l * 3.0))
    for s in [0.50 - 0.40 * i / 20 for i in range(21)]:
        (x, y), (dx, dy) = bezier(s), bezier_d(s)
        l = math.hypot(dx, dy)
        pts.append((x + dy / l * 3.0, y - dx / l * 3.0))
    cuts = [prism(pts, -t - 1, t + 1)]
    cuts += [cylx(M3_INSERT / 2, TONGUE[0] - 0.1, TONGUE[0] + 7.0, y, 0) for y in (6.0, -6.0)]
    cuts.append(cyl(M3_CLEAR / 2, -ht - 1, ht + 1).transformed(chain(J, T(L / 2, 0, 0))))
    return cut(out, *cuts)


def foot_socket():
    L, hd, ht = FOOT_TONGUE
    J = joint_frame()
    return [box(-0.1, L + 0.4, -hd - 0.25, hd + 0.25, -ht - 0.25, ht + 0.25).transformed(J),
            cyl(M3_CLEAR / 2, -T_BLADE, T_BLADE).transformed(chain(J, T(L / 2, 0, 0)))]


def foot():
    """TPU claw tip: grips the floor and keeps the tap noise down. Slips over the blade's tongue, one M3 across."""
    t = T_BLADE / 2
    body = fuse(prism(sickle(S_JOINT, 1.0, 16), -t, t), cyl(R_TIP, -t, t, L_TIBIA, 0.0))
    J = joint_frame()
    # Taper the thickness from 16 mm at the joint to 8 mm at the tip.
    jx, jy = bezier(S_JOINT)
    d = math.hypot(L_TIBIA - jx, -jy) + R_TIP
    a = math.degrees(math.atan2(-jy, L_TIBIA - jx))
    wedge_pts = [(-2, -t - 0.01), (d + 2, -4.0), (d + 2, 4.0), (-2, t + 0.01)]
    wedge = prism(wedge_pts, -30, 30).transformed(chain(T(jx, jy, 0), Rz(a), Rx(90)))
    body = body.common(wedge)
    return cut(single(body), *foot_socket())


SNEAKER_PEG = (14.0, 2.4, 12.0)    # length of the straight foot, peg radius, peg length


def foot_sneaker(direction):
    """Straight foot with a peg for a bought mini sneaker. `direction` is the peg axis in the tibia frame (degrees)."""
    t = T_BLADE / 2
    J = joint_frame()
    jx, jy = bezier(S_JOINT)
    L, pr, pl = SNEAKER_PEG
    seg = box(-1.0, L, -6.0, 6.0, -t + 1.5, t - 1.5).transformed(chain(T(jx, jy, 0), Rz(direction)))
    collar = cylx(6.0, L - 0.01, L + 2.0).transformed(chain(T(jx, jy, 0), Rz(direction)))
    peg = cylx(pr, L + 1.99, L + 2.0 + pl).transformed(chain(T(jx, jy, 0), Rz(direction)))
    root = box(-0.5, 9.5, -6.5, 6.5, -t, t).transformed(J)
    return cut(fuse(root, seg, collar, peg), *foot_socket())


def sneaker():
    """Stand-in for a bought keychain sneaker (render only): sole, upper, toe cap. Frame: origin on the peg tip,
    z up along the peg, x toward the toe. No logos."""
    sole = box(-16.0, 40.0, -11.0, 11.0, -14.0, -8.0)
    upper = box(-14.0, 26.0, -10.0, 10.0, -8.0, 13.0)
    toe = box(26.0, 38.0, -10.0, 10.0, -8.0, -1.0)
    stripe = box(-6.0, 22.0, -10.3, 10.3, -2.0, 4.0).cut(box(-7, 23, -10.0, 10.0, -3, 5))
    return dict(sneaker_white=fuse(sole, toe), sneaker_red=cut(upper, cyl(6.5, -2.0, 14.0, 0, 0)), sneaker_stripe=stripe)


# --- body ---
def hip_frame(i):
    """Origin on the coxa axis at belly height, x along the coxa servo case (hip angle + its turn)."""
    _, phi, delta = HIPS[i]
    return chain(Rz(phi), T(R_HIP, 0, 0), Rz(delta))


DECK_Z = (18.5, 20.5)
POSTS = [(24.0, 23.0), (-24.0, 23.0), (24.0, -23.0), (-24.0, -23.0)]       # deck posts, M2.5 inserts
HATCH_SCREWS = [(14.0, 23.0), (-14.0, 23.0), (14.0, -23.0), (-14.0, -23.0)]
COLUMNS = [(42.0, 22.0), (-42.0, 22.0), (42.0, -22.0), (-42.0, -22.0)]    # lid screws, M3 inserts
WIN = dict(w=30.0, h=24.0, r=7.0, z=40.5)      # face window opening
ACRYL = dict(w=34.0, h=28.0, r=8.0, y0=APO - WALL - 2.0, y1=APO - WALL)
CAM_Z = 14.5
LED = (16.0, CAM_Z)


def deck_outline():
    """Polygon behind all six coxa servo sockets (the end walls are at -37.8 along each case)."""
    poly = rect_pts(-60, 60, -60, 60)
    for i in range(6):
        P = hip_frame(i)
        o = P.multiply(V(0, 0, 0))
        u = P.multiply(V(1, 0, 0)) - o
        lim = -(S_TAIL + FIT + WALL + 1.0)
        def inside(p):
            return (p[0] - o.x) * u.x + (p[1] - o.y) * u.y <= lim
        def inter(p, q):
            fp = (p[0] - o.x) * u.x + (p[1] - o.y) * u.y - lim
            fq = (q[0] - o.x) * u.x + (q[1] - o.y) * u.y - lim
            k = fp / (fp - fq)
            return (p[0] + k * (q[0] - p[0]), p[1] + k * (q[1] - p[1]))
        new = []
        for j in range(len(poly)):
            p, q = poly[j], poly[(j + 1) % len(poly)]
            if inside(p):
                new.append(p)
                if not inside(q):
                    new.append(inter(p, q))
            elif inside(q):
                new.append(inter(p, q))
        poly = new
    return poly


def base():
    """Lower hex prism: the six coxa servo sockets, the face, and mounts for the electronics. Printed floor down."""
    out = rhex_face(APO, R_CORNER).extrude(V(0, 0, BASE_TOP))
    out = out.cut(rhex_face(APO - WALL, R_CORNER - WALL, FLOOR).extrude(V(0, 0, BASE_TOP)))
    adds, cuts = [], []
    x_end = -S_TAIL - FIT
    xs0 = x_end - WALL
    side = S_W + FIT
    z_ped = Z_S1 + S_TAIL_Z0 - FIT
    z_cov = Z_S1 + S_TAIL_Z1 + FIT
    z_top = Z_S1 + HORN_FACE - 0.5
    for i in range(6):
        P = hip_frame(i)
        loc = [box(xs0, x_end + 0.01, -15.1, 15.1, FLOOR - 0.01, z_top),
               box(x_end, -26.0, side, side + WALL, FLOOR - 0.01, z_top),
               box(x_end, -26.0, -side - WALL, -side, FLOOR - 0.01, z_top),
               box(x_end, -13.5, -13.0, 13.0, FLOOR - 0.01, z_ped),
               box(x_end, -13.5, -13.0, 13.0, z_cov, z_top)]
        adds += [s.transformed(P) for s in loc]
        c = [box(x_end, 40.0, -side, side, z_ped, z_cov),
             box(x_end - 0.01, -S_TAIL + 3 + FIT, -side, side, z_cov - 0.1, Z_S1 + S_LIP + FIT),
             box(S_BUMP[0] - FIT, S_BUMP[1] + FIT, -9.25, 9.25, Z_S1 + S_BUMP[2] - FIT, z_ped + 0.1),
             box(PLUG[0] - 2.0, -13.4, -PLUG[2] - 1.0, PLUG[2] + 1.0, FLOOR, z_ped + 0.1),
             box(xs0 - 1.0, PLUG[0] - 1.9, -4.0, 4.0, FLOOR, 8.0),
             cyl(13.5, 3.3, 50.5),
             cyl(25.5, 3.3, 50.5).common(box(0, 26, -26, 26, 3.3, 50.5))]   # bracket sweep, outward half
        for hx, hy in HOLE_TOP[1:]:
            c += [cyl(M2_CLEAR / 2, z_cov - 1, z_top + 1, hx, s * hy) for s in (1, -1)]
        for hx, hy in HOLE_BOT[1:]:
            c += [cyl(M2_CLEAR / 2, -1, z_ped + 1, hx, s * hy) for s in (1, -1)]
            c += [cyl(2.3, -1, z_ped - 3.0, hx, s * hy) for s in (1, -1)]
        cuts += [s.transformed(P) for s in c]
    # Deck posts, hatch bosses, lid columns with ribs to the nearest side face.
    for x, y in POSTS:
        adds.append(cyl(3.5, FLOOR - 0.01, DECK_Z[0], x, y))
        cuts.append(cyl(M25_INSERT / 2, DECK_Z[0] - 5.5, DECK_Z[0] + 0.1, x, y))
    for x, y in HATCH_SCREWS:
        adds.append(cyl(3.5, FLOOR - 0.01, FLOOR + 5.0, x, y))
        cuts.append(cyl(M3_INSERT / 2, -1, FLOOR + 4.5, x, y))
    for x, y in COLUMNS:
        adds.append(cyl(4.0, FLOOR - 0.01, BASE_TOP, x, y))
        psi = math.degrees(math.atan2(y, x))
        psi = min((30, 150, 210, 330, -30, -150), key=lambda a: abs(((psi - a) + 180) % 360 - 180))
        n = x * math.cos(math.radians(psi)) + y * math.sin(math.radians(psi))
        p = -x * math.sin(math.radians(psi)) + y * math.cos(math.radians(psi))
        adds.append(box(n, APO - WALL + 0.1, p - 1.0, p + 1.0, FLOOR - 0.01, BASE_TOP - 4.0).transformed(Rz(psi)))
        cuts.append(cyl(M3_INSERT / 2, BASE_TOP - 6.5, BASE_TOP + 1, x, y))
    cuts.append(box(-36.0, 36.0, -19.0, 19.0, -1, FLOOR + 1))          # battery hatch opening
    # Face: smoked window in a raised bezel, camera hole with a ring, status LED.
    yw = APO
    adds.append(rrect_y(ACRYL["w"] + 6, ACRYL["h"] + 6, ACRYL["r"] + 2, 0, WIN["z"], ACRYL["y0"], ACRYL["y1"] + 0.01))
    adds.append(rrect_y(WIN["w"] + 8, WIN["h"] + 8, WIN["r"] + 3, 0, WIN["z"], yw - 0.01, yw + 1.5))
    adds.append(cyly(6.0, yw - 0.01, yw + 0.8, 0, CAM_Z))
    cuts.append(rrect_y(WIN["w"], WIN["h"], WIN["r"], 0, WIN["z"], ACRYL["y1"] - 0.5, yw + 2))
    cuts.append(rrect_y(ACRYL["w"] + 0.4, ACRYL["h"] + 0.4, ACRYL["r"] + 0.2, 0, WIN["z"], ACRYL["y0"] - 0.1, ACRYL["y1"] + 0.05))
    cuts.append(cyly(4.0, ACRYL["y1"] - 1, yw + 2, 0, CAM_Z))     # 8 mm: the lens sits right behind the wall
    cuts.append(cyly(1.6, ACRYL["y1"] - 1, yw + 2, LED[0], LED[1]))
    # OLED module rails behind the window: the module slides down from above with the lid off.
    for s in (1, -1):
        adds.append(box(min(s * 17.95, s * 19.45), max(s * 17.95, s * 19.45), OLED_Y[0] - 1.0, ACRYL["y1"] + 0.01, 24.0, 58.0))
        adds.append(box(min(s * 16.6, s * 19.45), max(s * 16.6, s * 19.45), OLED_Y[0] - 1.0, OLED_Y[0] - 0.08, 24.0, 58.0))
    # Camera standoffs: 4 x M2 on the module's 21 x 12.5 pattern.  # UNVERIFIED lens offset on the board
    for sx in (1, -1):
        for sz in (1, -1):
            adds.append(cyly(2.4, CAM_Y[0] + 1.0, ACRYL["y1"] + 0.01, sx * 10.5, CAM_Z + sz * 6.25))
            cuts.append(cyly(0.85, CAM_Y[0], ACRYL["y1"] - 1.0, sx * 10.5, CAM_Z + sz * 6.25))
    # Back face: power switch slot.
    cuts.append(box(-6.0, 6.0, -yw - 2, -yw + WALL + 1, 36.75, 43.25))
    out = out.fuse(adds).removeSplitter()
    return cut(out, *cuts)


def deck():
    """Plate on the four posts above the battery; the bus driver board screws onto it."""
    pts = deck_outline()
    out = prism(pts, DECK_Z[0], DECK_Z[1])
    cuts = [cyl(1.4, DECK_Z[0] - 1, DECK_Z[1] + 1, x, y) for x, y in POSTS]
    cuts += [cyl(1.1, DECK_Z[0] - 1, DECK_Z[1] + 1, sx * 24.4, sy * 14.4) for sx in (1, -1) for sy in (1, -1)]
    cuts += [box(-8, 8, sy * 19.5 - 4, sy * 19.5 + 4, DECK_Z[0] - 1, DECK_Z[1] + 1) for sy in (1, -1)]   # cable slots
    for x, y in COLUMNS:
        cuts.append(cyl(5.0, DECK_Z[0] - 1, DECK_Z[1] + 1, x, y))
    return cut(out, *cuts)


def hatch():
    """Belly plate under the battery; four M3 into inserts. It is the lowest point of the body."""
    out = rrect_face(80.0, 54.0, 6.0).extrude(V(0, 0, 2.0)).transformed(T(0, 0, -2.0))
    cuts = []
    for x, y in HATCH_SCREWS:
        cuts.append(cyl(M3_CLEAR / 2, -3, 1, x, y))
        cuts.append(Part.makeCone(3.2, 1.65, 1.55, V(x, y, -2.01), V(0, 0, 1)))
    return cut(out, *cuts)


GRILLE = dict(d=3.2, pitch=5.8, apo=46.0)
PI_HOLES = [(29.0, -45.5), (-29.0, -45.5), (29.0, -22.5), (-29.0, -22.5)]
REG_HOLES = [(18.6, 25.1), (-18.6, 25.1), (18.6, 50.9), (-18.6, 50.9)]     # UNVERIFIED: Pololu hole positions


def lid():
    """Tapered hex lid with the honeycomb speaker grille. Printed top down; walls lean 36 degrees."""
    w0 = rhex_face(APO, R_CORNER, BASE_TOP).OuterWire
    w1 = rhex_face(LID_TOP_APO, 8.0, LID_TOP).OuterWire
    outer = Part.makeLoft([w0, w1], True, True)
    slope = (APO - LID_TOP_APO) / (LID_TOP - BASE_TOP)
    th = WALL * math.sqrt(1 + slope ** 2)
    zi = LID_TOP - WALL
    i0 = rhex_face(APO - th, R_CORNER - 2.5, BASE_TOP).OuterWire
    i1 = rhex_face(APO - slope * (zi - BASE_TOP) - th, 5.5, zi).OuterWire
    inner = Part.makeLoft([i0, i1], True, True)
    out = outer.cut([inner, rhex_face(APO - th, R_CORNER - 2.5, BASE_TOP - 1).extrude(V(0, 0, 1.01))])
    lip = rhex_face(APO - WALL - 0.3, R_CORNER - 2.8, BASE_TOP - 2.5).extrude(V(0, 0, 2.51)).cut(
        rhex_face(APO - WALL - 1.8, R_CORNER - 4.3, BASE_TOP - 3).extrude(V(0, 0, 4)))
    lip = lip.cut(box(-22.0, 22.0, 40.0, 80.0, BASE_TOP - 4, BASE_TOP + 1))     # room for the OLED at the front
    adds = [lip]
    for x, y in COLUMNS:
        adds.append(cyl(4.0, BASE_TOP + 0.2, zi + 0.01, x, y))
    for x, y in PI_HOLES:
        adds.append(cyl(3.0, PI_Z[1], zi + 0.01, x, y))
    for x, y in REG_HOLES:
        adds.append(cyl(2.5, REG_Z[1], zi + 0.01, x, y))
    adds.append(cyl(16.0, SPK_Z[0], zi + 0.01).cut(cyl(14.2, SPK_Z[0] - 1, zi + 1)))
    out = out.fuse(adds).removeSplitter()
    cuts = []
    for x, y in COLUMNS:
        cuts += [cyl(M3_CLEAR / 2, BASE_TOP - 1, LID_TOP + 1, x, y), cyl(3.0, LID_TOP - 2.5, LID_TOP + 1, x, y)]
    for x, y in PI_HOLES:
        cuts.append(cyl(1.1, PI_Z[1] - 1, zi - 1.0, x, y))
    for x, y in REG_HOLES:
        cuts.append(cyl(0.85, REG_Z[1] - 1, zi - 1.0, x, y))
    return cut(out, *cuts)


def grille_points():
    g, pts = GRILLE, []
    rows = int(g["apo"] / (g["pitch"] * 0.866)) + 2
    keep_out = [(x, y, 7.0) for x, y in COLUMNS]   # holes only go through the 2.5 mm top, so bosses below can stay under them
    for j in range(-rows, rows + 1):
        y = j * g["pitch"] * 0.866
        for i in range(-rows - 2, rows + 3):
            x = (i + 0.5 * (j % 2)) * g["pitch"]
            if max(x * math.cos(math.radians(a)) + y * math.sin(math.radians(a)) for a in (30, 90, 150, 210, 270, 330)) > g["apo"] - g["d"] / 2:
                continue
            if any(math.hypot(x - kx, y - ky) < kr for kx, ky, kr in keep_out):
                continue
            pts.append((x, y))
    return pts


def grille_cut(shape):
    zi = LID_TOP - WALL
    holes = Part.makeCompound([cyl(GRILLE["d"] / 2, zi - 0.01, LID_TOP + 0.5, x, y) for x, y in grille_points()])
    return single(shape.cut(holes))


def window():
    """2 mm smoked acrylic, cut to size; sits behind the window opening."""
    return rrect_y(ACRYL["w"], ACRYL["h"], ACRYL["r"], 0, WIN["z"], ACRYL["y0"], ACRYL["y1"])


def eyes():
    """The two pill eyes drawn by the OLED, shown on the window surface for the renders."""
    return fuse(*[rrect_y(5.5, 9.0, 2.7, sx * 5.0, WIN["z"], ACRYL["y1"] + 0.02, ACRYL["y1"] + 0.3) for sx in (1, -1)])


def fit_gauge():
    """Print first: a U gauge for the horn-to-idler distance and both horn hole patterns. If one servo with both
    horns on does not slide snugly between the plates, fix HORN_FACE or IDLER_FACE."""
    zt0 = HORN_FACE
    zb1 = IDLER_FACE
    top = prism(hull2d(circle_pts(0, 0, 12.0) + rect_pts(8.0, 17.5, -12.0, 12.0)), zt0, zt0 + 2.5)
    bot = prism(hull2d(circle_pts(0, 0, 10.2) + rect_pts(8.0, 17.5, -12.0, 12.0)), zb1 - 2.5, zb1)
    web = box(15.0, 17.5, -12.0, 12.0, zb1 - 2.5, zt0 + 2.5)
    return cut(fuse(top, bot, web), *(horn_holes(zt0 - 1, zt0 + 4) + horn_holes(zb1 - 4, zb1 + 1, idler=True)))


def fit_collar():
    """Print first: a collar around the tail with the four tail holes used by the brackets, and a window for the
    plugs and the bump. Screw it on: if a screw misses its hole, fix HOLE_TOP / HOLE_BOT."""
    xc0, xc1 = -S_TAIL - FIT - 1.6, -24.0
    collar = box(xc0, xc1, -S_W - FIT - 1.6, S_W + FIT + 1.6, S_TAIL_Z0 - FIT - 1.6, S_TAIL_Z1 + FIT + 1.6)
    collar = cut(collar, box(-S_TAIL - FIT, xc1 + 1, -S_W - FIT, S_W + FIT, S_TAIL_Z0 - FIT, S_TAIL_Z1 + FIT),
                 box(-S_TAIL + 3 + FIT, xc1 + 1, -S_W - FIT, S_W + FIT, S_TAIL_Z0 - 3, S_TAIL_Z0),
                 *[cyl(M2_CLEAR / 2, S_TAIL_Z1 - 1, S_TAIL_Z1 + 3, HOLE_TOP[1][0], s * HOLE_TOP[1][1]) for s in (1, -1)],
                 *[cyl(M2_CLEAR / 2, S_TAIL_Z0 - 3, S_TAIL_Z0 + 1, HOLE_BOT[1][0], s * HOLE_BOT[1][1]) for s in (1, -1)])
    return collar


# --- electronics: real parts, keep-out boxes with their masses (g) ---
OLED_Y = (ACRYL["y0"] - 5.0, ACRYL["y0"])    # 1.3" SH1106 module: 35.4 x 33.5 mm board, about 5 mm deep with the header
CAM_Y = (APO - WALL - 11.5, APO - WALL)       # Camera Module 3: 25 x 24 mm, 11.5 mm deep with the lens
PI_Z = (66.0, 71.0)                           # Pi Zero 2 W hangs under the lid, parts facing down
REG_Z = (63.5, 74.5)
SPK_Z = (72.0, LID_TOP - WALL)
ELECTRONICS = [
    ("battery", box(-34.5, 34.5, -17.5, 17.5, 0.0, 17.0), 86.0),              # Gens ace 2S 1300 mAh 25C, 69 x 35 x 17
    ("bus_board", box(-27.9, 27.9, -18.3, 18.3, DECK_Z[1], DECK_Z[1] + 11.5), 15.0),   # Feetech FE-URT-1/-2, 55.8 x 36.6 x 11.5; mass UNVERIFIED
    ("regulator", box(-21.6, 21.6, 22.1, 53.9, REG_Z[0], REG_Z[1]), 19.0),   # Pololu D24V150F6, 43.2 x 31.8 x 11, 19 g
    ("pi_zero", box(-32.5, 32.5, -49.0, -19.0, PI_Z[0], PI_Z[1]), 11.0),     # 65 x 30; mass UNVERIFIED
    ("camera", box(-12.5, 12.5, CAM_Y[0], CAM_Y[0] + 1.0, CAM_Z - 12.0, CAM_Z + 12.0).fuse(
        box(-4.5, 4.5, CAM_Y[0] + 0.99, CAM_Y[1], CAM_Z - 4.5, CAM_Z + 4.5)), 4.0),   # Camera Module 3, 25 x 24 board, lens barrel
    ("oled", box(-17.7, 17.7, OLED_Y[0], OLED_Y[1], WIN["z"] - 13.5, WIN["z"] + 20.0), 8.0),   # mass UNVERIFIED
    ("speaker", cyl(14.0, SPK_Z[0], SPK_Z[1] - 0.01), 6.0),                    # Adafruit 1890, 28 x 4.5 mm, 6 g
    ("amp", box(18.3, 37.7, -8.9, 8.9, 73.0, 76.0), 2.0),                     # MAX98357A breakout 19.4 x 17.8 x 3
    ("mic", box(-36.7, -20.0, -6.35, 6.35, 74.2, 76.0), 1.0),                 # SPH0645 breakout 16.7 x 12.7 x 1.8
    ("switch", box(-7.0, 7.0, -APO + WALL, -APO + WALL + 8.0, 36.0, 44.0), 2.0),
    ("led", cyly(2.5, APO - WALL - 8.0, APO - WALL - 0.01, LED[0], LED[1]), 0.5),
    ("reg_5v", box(-8.9, 8.9, -APO + WALL + 0.01, -APO + WALL + 4.2, 14.0, 31.8), 3.0),   # 5 V for the Pi: Pololu D24V22F5 class, 17.8 x 17.8; taped to the back wall; mass UNVERIFIED
]
LUMPS_BODY = [("wiring", V(0, 0, 35), 50.0), ("body_hardware", V(0, 0, 30), 20.0)]
MASS_SNEAKER = 10.0  # one keychain mini sneaker.  # UNVERIFIED: not weighed
LEG_HARDWARE = 13.0   # 24 x M3 horn screws and 12 x M2 case screws per leg, spread over the three joints


def build_parts():
    log("building parts")
    p = dict(servo=servo(), label=servo_label(), coxa=coxa(), femur=femur(), knee=knee(), blade=blade(), foot=foot(),
             base=base(), deck=deck(), hatch=hatch(), window=window(), eyes=eyes(), fit_gauge=fit_gauge(), fit_collar=fit_collar())
    log("base done")
    lid_plain = lid()
    p["lid_proxy"] = lid_plain                 # collision proxy without the 200+ grille holes
    p["lid"] = grille_cut(lid_plain)
    log("lid done, grille holes:", len(grille_points()))
    for name, shape, _ in ELECTRONICS:
        p["e_" + name] = shape
    return p


PRINTED = dict(base=1, lid=1, deck=1, hatch=1, coxa=6, femur=6, knee=6, blade=6, foot=6, fit_gauge=1, fit_collar=1, foot_sneaker=6)
MATERIAL = dict(foot=TPU)
ALTERNATIVES = ("fit_gauge", "fit_collar", "foot_sneaker")   # printed, but not part of the robot's mass


def part_mass(key, shape):
    if key == "servo":
        return MASS_SERVO
    if key == "window":
        return shape.Volume / 1000 * ACRYLIC
    if key.startswith("sneaker"):
        return MASS_SNEAKER / 3
    if key.startswith("e_"):
        return dict((n, m) for n, _, m in ELECTRONICS)[key[2:]]
    return shape.Volume / 1000 * MATERIAL.get(key, PETG) * INFILL


# ===================== assembly =====================
def leg_frames(i, gamma, th2, beta, B):
    _, phi, delta = HIPS[i]
    H = chain(B, Rz(phi), T(R_HIP, 0, 0))
    C = chain(H, Rz(gamma))
    fa = chain(C, T(L_COXA, 0, Z_F), Rx(-90))
    Fr = chain(fa, Rz(-th2))
    return dict(s1=chain(H, Rz(delta), T(0, 0, Z_S1)), coxa=C, s2=chain(fa, T(0, 0, -S_MID), Rz(-90)), femur=Fr,
                s3=chain(Fr, T(L_FEMUR, 0, 0), Rz(beta), Rz(S3_DIR), T(0, 0, -S_MID)), tibia=chain(Fr, T(L_FEMUR, 0, 0), Rz(beta)))


def body_frame(pose):
    return pose.get("B") or T(0, 0, pose["hb"])


FOOT_KEY = "foot"


def assemble(pose, parts, render=False):
    """[(name, key, shape, group)], feet {leg: contact point}, frames. Render adds labels, eyes, sneakers."""
    B = body_frame(pose)
    out = [("base", "base", parts["base"], "B"), ("lid", "lid_proxy", parts["lid_proxy"], "B"),
           ("deck", "deck", parts["deck"], "B"), ("hatch", "hatch", parts["hatch"], "B"), ("window", "window", parts["window"], "B")]
    out += [("e_" + n, "e_" + n, parts["e_" + n], "B") for n, _, _ in ELECTRONICS]
    out = [(n, k, s.transformed(B), g, B) for n, k, s, g in out]
    feet, frames = {}, {}
    fkey = pose.get("foot", FOOT_KEY)
    contact = pose.get("contact_pt", (L_TIBIA, 0.0))
    for i, (n, _, _) in enumerate(HIPS):
        f = leg_frames(i, pose["gamma"][n], pose["th2"][n], pose["beta"][n], B)
        frames[n] = f
        for link, key, grp in (("s1", "servo", "B"), ("coxa", "coxa", "C" + n), ("s2", "servo", "C" + n), ("femur", "femur", "F" + n),
                               ("s3", "servo", "T" + n), ("tibia", "knee", "T" + n), ("tibia", "blade", "T" + n), ("tibia", fkey, "T" + n)):
            out.append(("%s_%s" % (n, key if link == "tibia" else link), key, parts[key].transformed(f[link]), grp, f[link]))
        feet[n] = f["tibia"].multiply(V(contact[0], contact[1], 0))
    if render:
        extra = [("eyes", "eyes", B)]
        for n, _, _ in HIPS:
            for link in ("s1", "s2", "s3"):
                extra.append(("%s_label_%s" % (n, link), "label", frames[n][link]))
        out += [(n, k, parts[k].transformed(m), "R", m) for n, k, m in extra]
    if fkey == "foot_sneaker":
        for n, _, _ in HIPS:
            m = chain(frames[n]["tibia"], pose["sneaker_place"])
            for k in ("sneaker_white", "sneaker_red", "sneaker_stripe"):
                out.append(("%s_%s" % (n, k), k, parts[k].transformed(m), "T" + n, m))
    return out, feet, frames


def collisions(items, mode="cross", limit=0.5, skip=()):
    hits = []
    bbs = [it[2].BoundBox for it in items]
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            gi, gj = items[i][3], items[j][3]
            if "R" in (gi, gj) or (mode == "cross") == (gi == gj):
                continue
            if (items[i][0], items[j][0]) in skip:
                continue
            if not bbs[i].intersect(bbs[j]):
                continue
            v = items[i][2].common(items[j][2]).Volume
            if v > limit:
                hits.append((items[i][0], items[j][0], round(v, 1)))
    return sorted(hits, key=lambda h: -h[2])


def mesh_min_z(shape):
    pts, _ = shape.tessellate(0.05)
    return min(p.z for p in pts)


def floor_check(items, feet_on):
    """Lowest point of every part; feet that should stand must touch (|z| < 0.5), nothing may go below -0.5."""
    lows, worst = {}, 1e9
    for n, k, s, g, _ in items:
        if g == "R":
            continue
        z = s.BoundBox.ZMin
        if z < 1.0:
            z = mesh_min_z(s)
        if k.startswith("foot") or k.startswith("sneaker"):
            leg = n.split("_")[0]
            lows[leg] = round(min(z, lows.get(leg, 1e9)), 2)
        else:
            worst = min(worst, z)
    stand_ok = all(abs(lows[n]) < 0.5 for n in feet_on)
    air_ok = all(lows[n] > 0.5 for n in lows if n not in feet_on)
    return dict(foot_low=lows, lowest_other=round(worst, 2), feet_down_ok=stand_ok, feet_up_ok=air_ok,
                no_penetration=worst > -0.5 and min(lows.values()) > -0.5)


def envelope(items):
    b = None
    for _, _, s, g, _ in items:
        if g == "R":
            continue
        bb = s.BoundBox
        if b is None:
            b = App.BoundBox(bb)
        else:
            b.add(bb)
    return dict(x=round(b.XLength, 1), y=round(b.YLength, 1), zmax=round(b.ZMax, 1), zmin=round(b.ZMin, 1))


# ===================== kinematics =====================
def leg_ik(i, foot, B, contact=(L_TIBIA, 0.0)):
    """Joint angles (coxa, femur up from horizontal, knee bend) that put the contact point on `foot` (world)."""
    name, phi, _ = HIPS[i]
    H = chain(B, Rz(phi), T(R_HIP, 0, 0))
    p = H.inverse().multiply(foot)
    gamma = math.degrees(math.atan2(p.y, p.x))
    r = math.hypot(p.x, p.y)
    dx, dz = r - L_COXA, p.z - Z_F
    D = math.hypot(dx, dz)
    L2 = math.hypot(*contact)
    off = math.degrees(math.atan2(contact[1], contact[0]))
    c = (L_FEMUR ** 2 + D ** 2 - L2 ** 2) / (2 * L_FEMUR * D)
    if abs(c) > 1:
        raise ValueError("%s out of reach (D=%.1f)" % (name, D))
    th2 = math.degrees(math.atan2(dz, dx) + math.acos(c))
    kx = L_COXA + L_FEMUR * math.cos(math.radians(th2))
    kz = Z_F + L_FEMUR * math.sin(math.radians(th2))
    tib = math.degrees(math.atan2(p.z - kz, r - kx))
    return gamma, th2, th2 - tib - off


def limits_ok(pose):
    bad = []
    for n, _, delta in HIPS:
        g, t, b = pose["gamma"][n], pose["th2"][n], pose["beta"][n]
        if abs(g - delta) > COXA_RANGE + 1e-6:
            bad.append((n, "coxa", round(g, 1)))
        if not TH2_RANGE[0] - 1e-6 <= t <= TH2_RANGE[1] + 1e-6:
            bad.append((n, "femur", round(t, 1)))
        if not BETA_RANGE[0] - 1e-6 <= b <= BETA_RANGE[1] + 1e-6:
            bad.append((n, "knee", round(b, 1)))
    return bad


def spots(foot_r):
    """Foot contact points (world, tip centre) straight out from each hip."""
    out = {}
    for n, phi, _ in HIPS:
        a = math.radians(phi)
        out[n] = V((R_HIP + foot_r) * math.cos(a), (R_HIP + foot_r) * math.sin(a), R_TIP)
    return out


STAND_HB, STAND_R = 62.0, 68.2   # "Normal": belly 62 mm, feet 35 mm outside the femur axis
GAIT_R = 83.2       # walking stance at Normal: feet 50 mm out, the lowest worst-frame load
GAIT_LIFT, GAIT_OUT = 50.0, 0.0    # at Normal the knee stands at 117 deg, so the foot lifts straight up
# Belly height is a software setting: the body rises or sinks on feet planted at STAND_R (checked every 5 mm, 30..100).
# Three named heights, each with its own walking radius, lift and outward swing (the blade120 height report).
NAMED_HEIGHTS = dict(spinne=dict(hb=30.0, gait_r=73.2, lift=25.0, out=45.0), normal=dict(hb=62.0, gait_r=83.2, lift=50.0, out=0.0),
                     hoch=dict(hb=95.0, gait_r=83.2, lift=50.0, out=0.0))


def posture(B, feet, contact=(L_TIBIA, 0.0), on=None, extra=None):
    pose = dict(B=B, hb=B.multiply(V(0, 0, 0)).z, gamma={}, th2={}, beta={}, on=list(on or [n for n, _, _ in HIPS]))
    for i, (n, _, _) in enumerate(HIPS):
        pose["gamma"][n], pose["th2"][n], pose["beta"][n] = leg_ik(i, feet[n], B, contact)
    pose["targets"] = {n: [round(feet[n].x, 2), round(feet[n].y, 2), round(feet[n].z, 2)] for n in feet}
    pose["_args"] = (B, dict(feet), contact, on, extra)
    if extra:
        pose.update(extra)
    return pose


def pitch_about(hb, pitch=0.0, yaw=0.0, zc=40.0, shift=(0.0, 0.0)):
    return chain(T(shift[0], shift[1], hb + zc), Rz(yaw), Rx(pitch), T(0, 0, -zc))


def gait_feet(kind, k, n=24, stride=50.0, lift=GAIT_LIFT, out=GAIT_OUT, turn=16.0, base_spots=None):
    """Tripod gait, both tripods half a cycle apart. Stance: the foot slides back under the body at constant speed.
    Swing: cosine forward, sine lift, and the tip swings `out` mm outward. At Normal the lift is straight; at Spinne
    (knee folded to 143 deg) the foot must swing out while it lifts. Returns world feet and the stance legs."""
    base_spots = base_spots or spots(GAIT_R)
    p = k / float(n)
    feet, stance = {}, []
    for name, _, _ in HIPS:
        ph = (p + (0.0 if name in TRI_A else 0.5)) % 1.0
        if ph < 0.5:
            s, z = 0.5 - ph / 0.5, 0.0
            stance.append(name)
        else:
            u = (ph - 0.5) / 0.5
            s, z = -0.5 + (0.5 - 0.5 * math.cos(math.pi * u)), lift * math.sin(math.pi * u) ** 2   # out first, then up
        h = base_spots[name]
        if ph >= 0.5:
            r = math.hypot(h.x, h.y)
            o = out * math.sin(math.pi * u) ** 0.7
            h = V(h.x + h.x / r * o, h.y + h.y / r * o, h.z)
        if kind == "walk":
            feet[name] = V(h.x, h.y + s * stride, h.z + z)
        else:
            a = math.radians(s * turn)
            feet[name] = V(h.x * math.cos(a) - h.y * math.sin(a), h.x * math.sin(a) + h.y * math.cos(a), h.z + z)
    return feet, stance


# ===================== statics =====================
def centroid(shape):
    return shape.CenterOfMass if hasattr(shape, "CenterOfMass") else shape.Solids[0].CenterOfMass


def hull_margin(points, p):
    """Distance from p to the edge of the foot polygon; negative means it would tip."""
    hull = hull2d(points)
    m = []
    for i in range(len(hull)):
        a, b = hull[i], hull[(i + 1) % len(hull)]
        m.append(((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])) / math.hypot(b[0] - a[0], b[1] - a[1]))
    return min(m)


def share_load(points, com):
    """Least-effort split of the weight over the feet in contact, no foot pulling."""
    active = list(range(len(points)))
    while True:
        n = len(active)
        xs = [points[i][0] for i in active]
        ys = [points[i][1] for i in active]
        sx, sy = sum(xs), sum(ys)
        sxx, syy, sxy = sum(x * x for x in xs), sum(y * y for y in ys), sum(x * y for x, y in zip(xs, ys))
        A = [[n, sx, sy], [sx, sxx, sxy], [sy, sxy, syy]]
        b = [1.0, com[0], com[1]]
        for i in range(3):
            piv = max(range(i, 3), key=lambda r: abs(A[r][i]))
            A[i], A[piv], b[i], b[piv] = A[piv], A[i], b[piv], b[i]
            for r in range(3):
                if r != i and abs(A[i][i]) > 1e-12:
                    k = A[r][i] / A[i][i]
                    A[r] = [u - k * v for u, v in zip(A[r], A[i])]
                    b[r] -= k * b[i]
        lam = [b[i] / A[i][i] if abs(A[i][i]) > 1e-12 else 0.0 for i in range(3)]
        w = [lam[0] + lam[1] * x + lam[2] * y for x, y in zip(xs, ys)]
        if min(w) >= -1e-9 or n <= 3:
            return {active[i]: max(v, 0.0) for i, v in enumerate(w)}
        active.pop(w.index(min(w)))


class Loads:
    """Mass, centre of mass, tipping margin and gravity torques for one pose."""
    def __init__(self, pose, parts, items=None):
        items, self.feet, self.frames = items or assemble(pose, parts)
        self.masses = []
        for n, k, s, g, m in items:
            if g == "R":
                continue
            self.masses.append((n, part_mass(k, s), centroid(s)))
        B = body_frame(pose)
        for n, p, m in LUMPS_BODY:
            self.masses.append((n, m, B.multiply(p)))
        for leg, f in self.frames.items():
            for link in ("coxa", "femur", "tibia"):
                self.masses.append(("%s_hw_%s" % (leg, link), LEG_HARDWARE / 3, f[link].multiply(V(0, 0, 0))))
        self.total = sum(m for _, m, _ in self.masses)
        c = V(0, 0, 0)
        for _, m, p in self.masses:
            c = c + p * m
        self.com = c * (1.0 / self.total)

    def torque(self, leg, joint, w_kg):
        """|gravity torque| in kg.cm at the femur or knee of `leg`: foot reaction w_kg plus the weight of the links beyond."""
        f = self.frames[leg]["femur" if joint == "femur" else "tibia"]
        p = f.multiply(V(0, 0, 0))
        a = f.multiply(V(0, 0, 1)) - p
        beyond = ("_femur", "_s3", "_knee", "_blade", "_foot", "_sneaker", "_hw_femur", "_hw_tibia") if joint == "femur" else ("_knee", "_s3", "_blade", "_foot", "_sneaker", "_hw_tibia")
        t = 0.0
        for n, m, c in self.masses:
            if n.startswith(leg + "_") and any(n.startswith(leg + b) for b in beyond):
                t += a.dot((c - p).cross(V(0, 0, -m / 1000.0)))
        t += a.dot((self.feet[leg] - p).cross(V(0, 0, w_kg)))
        return abs(t) / 10.0

    def support(self, legs):
        pts = [(self.feet[n].x, self.feet[n].y) for n in legs]
        c2 = (self.com.x, self.com.y)
        margin = hull_margin(pts, c2)
        shares = share_load(pts, c2)
        w = {n: shares.get(i, 0.0) * self.total / 1000.0 for i, n in enumerate(legs)}
        tq = {"femur": 0.0, "knee": 0.0}
        for n, _, _ in HIPS:
            for j in ("femur", "knee"):
                tq[j] = max(tq[j], self.torque(n, j, w.get(n, 0.0)))
        h = self.com.z - min(self.feet[n].z - R_TIP for n in legs)
        return dict(margin_mm=round(margin, 1), push_g=round(margin / h, 2), com_height_mm=round(h, 1),
                    femur_kgcm=round(tq["femur"], 2), knee_kgcm=round(tq["knee"], 2),
                    max_foot_load_g=round(max(w.values()) * 1000))


def pct(kgcm):
    return dict(kgcm=round(kgcm, 2), pct_rated_6v=round(kgcm / RATED[6.0] * 100), pct_stall_6v=round(kgcm / STALL[6.0] * 100),
                pct_rated_7v4=round(kgcm / RATED[7.4] * 100), pct_stall_7v4=round(kgcm / STALL[7.4] * 100))


def statics(pose, parts, items=None):
    L = Loads(pose, parts, items)
    out = dict(mass_g=round(L.total), com_mm=[round(L.com.x, 1), round(L.com.y, 1), round(L.com.z, 1)])
    on = pose.get("on")
    if pose.get("rest_on_belly"):
        # Resting: the belly plate carries the weight and the legs only hold themselves up.
        B = body_frame(pose)
        belly = [B.multiply(V(sx * 40.0, sy * 27.0, -2.0)) for sx in (1, -1) for sy in (1, -1)]
        m = hull_margin([(p.x, p.y) for p in belly], (L.com.x, L.com.y))
        tq = {j: max(L.torque(n, j, 0.0) for n, _, _ in HIPS) for j in ("femur", "knee")}
        out["resting"] = dict(margin_mm=round(m, 1), femur=pct(tq["femur"]), knee=pct(tq["knee"]))
        out["stand_up_six"] = L.support(on)          # the push to lift the body off the floor on six legs
        for k in ("femur_kgcm", "knee_kgcm"):
            out["stand_up_six"][k.split("_")[0]] = pct(out["stand_up_six"].pop(k))
        return out
    out["support"] = L.support(on)
    if len(on) == 6:
        out["tripod_A"] = L.support(list(TRI_A))
        out["tripod_B"] = L.support(list(TRI_B))
    for k in ("support", "tripod_A", "tripod_B"):
        if k in out:
            out[k]["femur"] = pct(out[k].pop("femur_kgcm"))
            out[k]["knee"] = pct(out[k].pop("knee_kgcm"))
    return out


# ===================== export =====================
ORIENTS = dict(zdown=Matrix(), zup=Rx(180), xdown=Ry(90), xup=Ry(-90), ydown=Rx(-90), yup=Rx(90))


def print_layout(shape):
    """Try the six axis-down orientations; keep the one with the least overhang that fits the 180 mm bed."""
    best = None
    for name, m in ORIENTS.items():
        s = shape.transformed(m)
        s = s.transformed(T(0, 0, -s.BoundBox.ZMin))
        pts, tris = s.tessellate(0.1)
        over = 0.0
        for a, b, c in tris:
            pa, pb, pc = pts[a], pts[b], pts[c]
            n = (pb - pa).cross(pc - pa)
            area = n.Length / 2
            if area < 1e-9:
                continue
            if n.z / n.Length < -0.7071 and min(pa.z, pb.z, pc.z) > 0.3:
                over += area
        bb = s.BoundBox
        foot = None
        for ang in range(0, 91, 5):
            r = s.transformed(Rz(ang)).BoundBox
            f = (max(r.XLength, r.YLength), min(r.XLength, r.YLength), ang)
            if foot is None or f[0] < foot[0]:
                foot = f
        fits = foot[0] <= 180.0 and bb.ZLength <= 180.0
        key = (not fits, round(over / 100, 1), bb.ZLength)
        if best is None or key < best[0]:
            best = (key, name, m, dict(orientation=name, bed_mm=[round(foot[0], 1), round(foot[1], 1), round(bb.ZLength, 1)],
                                       turn_on_bed_deg=foot[2], fits_180=fits, overhang_cm2=round(over / 100, 1)))
    return best[2], best[3]


def export_parts(parts):
    folder = os.path.join(OUT, "parts")
    os.makedirs(folder, exist_ok=True)
    for old in os.listdir(folder):
        if old.endswith((".stl", ".step")):
            os.remove(os.path.join(folder, old))
    import MeshPart
    report = {}
    for key, count in PRINTED.items():
        shape = parts[key]
        m, info = print_layout(shape if key != "lid" else parts["lid_proxy"])
        s = shape.transformed(m)
        s = s.transformed(T(-s.BoundBox.Center.x, -s.BoundBox.Center.y, -s.BoundBox.ZMin))
        mesh = MeshPart.meshFromShape(Shape=s, LinearDeflection=0.05, AngularDeflection=0.25)
        ok = len(s.Solids) == 1 and s.isValid() and mesh.isSolid() and not mesh.hasNonManifolds() and mesh.countComponents() == 1
        mat = "TPU" if MATERIAL.get(key) == TPU else "PETG"
        g = part_mass(key, shape)
        info.update(count=count, material=mat, volume_cm3=round(shape.Volume / 1000, 1), mass_g_each=round(g, 1),
                    mass_g_total=round(g * count, 1), solid_ok=ok)
        report[key] = info
        stem = "%s_x%d" % (key, count)
        mesh.write(os.path.join(folder, stem + ".stl"))
        s.exportStep(os.path.join(folder, stem + ".step"))
        log("exported", stem, info)
    return report


def dump_meshes(parts, keys):
    """Triangles of every part in its own frame, for render.py (numpy, no FreeCAD)."""
    import numpy as np
    folder = os.path.join(OUT, "meshes")
    os.makedirs(folder, exist_ok=True)
    data = {}
    for k in keys:
        pts, tris = parts[k].tessellate(0.12 if k not in ("lid",) else 0.2)
        P = np.array([[p.x, p.y, p.z] for p in pts], dtype=np.float32)
        I = np.array(tris, dtype=np.int32)
        data[k] = P[I]
    np.savez_compressed(os.path.join(folder, "parts.npz"), **data)


RENDER_KEYS = {"base": "black", "lid": "black", "hatch": "black", "window": "glass", "eyes": "eye", "servo": "servo", "label": "red",
               "coxa": "black", "femur": "black", "knee": "black", "blade": "black", "foot": "tpu", "foot_sneaker": "tpu",
               "sneaker_white": "white", "sneaker_red": "red", "sneaker_stripe": "white", "e_camera": "lens", "e_led": "lens"}


def mat_list(m):
    return [m.A11, m.A12, m.A13, m.A14, m.A21, m.A22, m.A23, m.A24, m.A31, m.A32, m.A33, m.A34]


def scene(pose, parts):
    """What render.py draws: part key, colour, placement; plus cable anchors."""
    items, feet, frames = assemble(pose, parts, render=True)
    out = []
    for n, k, s, g, m in items:
        key = "lid" if k == "lid_proxy" else k
        if key in RENDER_KEYS:
            out.append([key, RENDER_KEYS[key], mat_list(m)])
    cables = []
    plug = V((PLUG[0] + PLUG[1]) / 2, 0, S_TAIL_Z0 - 2.0)
    for n, f in frames.items():
        p1 = f["s1"].multiply(V(S_NOSE - 2, 0, -1.0))
        p2 = f["s2"].multiply(plug)
        p3 = f["s3"].multiply(plug)
        cables.append([[p3.x, p3.y, p3.z], [p2.x, p2.y, p2.z]])
        cables.append([[p2.x, p2.y, p2.z], [p1.x, p1.y, p1.z]])
    return dict(items=out, cables=cables)


# ===================== poses =====================
LOOKUP_PITCH = 10.0


def all_poses(parts):
    P = {}
    sp = spots(STAND_R)
    P["stand"] = posture(T(0, 0, STAND_HB), sp)
    P["low_stand"] = posture(T(0, 0, NAMED_HEIGHTS["spinne"]["hb"]), sp)      # "Spinne": belly 30 on the stand feet
    P["sit"] = posture(T(0, 0, 2.0), spots(140.0), extra=dict(rest_on_belly=True))
    # Look up: nose up about the rear hip line, so the rear knees do not fold past their limit. 10 deg, not 12: with
    # the 120 mm blade the front femurs reach -24 deg at 12 (range ends at -20).
    rear = R_HIP * math.sin(math.radians(60))
    P["look_up"] = posture(chain(T(0, -rear, STAND_HB + 40.0), Rx(LOOKUP_PITCH), T(0, rear, -40.0)), sp)
    # Look around: body yaw on planted feet. The coxa turns about 2.2x the body yaw, so 10 deg is the most.
    P["look_around"] = posture(pitch_about(STAND_HB, yaw=10.0), sp)
    # Wave: left front leg up and out, the other five stand where they were; body leans a little right and back.
    B = pitch_about(STAND_HB, shift=(6.0, -6.0))
    w = posture(B, sp, on=[n for n, _, _ in HIPS if n != "LF"])
    w["gamma"]["LF"], w["th2"]["LF"], w["beta"]["LF"] = 5.0, 95.0, 75.0
    P["wave"] = w
    w2 = dict(w, gamma=dict(w["gamma"]), th2=dict(w["th2"]), beta=dict(w["beta"]))
    w2["gamma"]["LF"], w2["beta"]["LF"] = -20.0, 90.0
    P["wave_2"] = w2
    return P


def sneaker_setup(parts, stand):
    """Sneaker foot: the peg points straight down in the stand pose, the toe points out along the leg."""
    f = leg_frames(1, stand["gamma"]["RM"], stand["th2"]["RM"], stand["beta"]["RM"], body_frame(stand))["tibia"]
    o = f.multiply(V(0, 0, 0))
    down = f.inverse().multiply(o + V(0, 0, -1))
    direction = math.degrees(math.atan2(down.y, down.x))
    parts["foot_sneaker"] = foot_sneaker(direction)
    jx, jy = bezier(S_JOINT)
    L, pr, pl = SNEAKER_PEG
    a = math.radians(direction)
    tip = (jx + (L + 2.0 + pl) * math.cos(a), jy + (L + 2.0 + pl) * math.sin(a))
    out = f.inverse().multiply(o + V(1, 0, 0))     # body +x is "out" for RM
    ex = V(out.x, out.y, 0)
    ezv = V(-math.cos(a), -math.sin(a), 0)          # sneaker z is up along the peg
    ex = ex - ezv * ex.dot(ezv)
    ex.normalize()
    ey = ezv.cross(ex)
    place = frame(V(tip[0], tip[1], 0), ex, ey, ezv)
    sole = place.multiply(V(12.0, 0, -14.0))
    sn = sneaker()
    parts.update(sn)
    return direction, place, (sole.x, sole.y)


# ===================== checks =====================
def sweep_checks(parts):
    """Each joint through its whole range with the others neutral, body in the air: brackets vs their own servos."""
    out = {}
    B = T(0, 0, 400.0)
    i = 1   # RM: same parts as every leg
    n = HIPS[i][0]
    def single_leg(g, t, b):
        pose = dict(B=B, hb=400.0, gamma={h[0]: h[2] for h in HIPS}, th2={h[0]: 45.0 for h in HIPS}, beta={h[0]: 120.0 for h in HIPS})
        pose["gamma"][n], pose["th2"][n], pose["beta"][n] = g, t, b
        items = [it for it in assemble(pose, parts)[0] if it[3] in ("B", "C" + n, "F" + n, "T" + n)]
        return collisions(items)
    rows = []
    for g in [-COXA_RANGE + 10 * k for k in range(int(2 * COXA_RANGE / 10) + 1)]:
        rows.append((g, single_leg(g, 45.0, 120.0)))
    out["coxa_deg"] = {str(round(g)): h for g, h in rows}
    def steps(lo, hi, d=15.0):
        v = [lo + d * k for k in range(int((hi - lo) / d) + 1)]
        return v + ([hi] if hi - v[-1] > 1e-6 else [])
    out["femur_deg"] = {str(round(t)): single_leg(0.0, t, 100.0) for t in steps(*TH2_RANGE)}
    out["knee_deg"] = {str(round(b)): single_leg(0.0, 45.0, b) for b in steps(*BETA_RANGE)}
    out["knee_outside_range_deg"] = {str(b): single_leg(0.0, 45.0, b) for b in (60.0, 65.0, 160.0, 165.0)}
    out["pass"] = all(not h for d in ("coxa_deg", "femur_deg", "knee_deg") for h in out[d].values())   # inside the ranges only
    return out


def settle(pose, parts, rounds=2):
    """Tilted body or a sneaker sole: the lowest point of a foot is not where the IK put it. Lift or lower each
    standing foot's target by its error and solve again."""
    if "_args" not in pose:
        return pose
    for _ in range(rounds):
        items = assemble(pose, parts)[0]
        low = floor_check(items, pose["on"])["foot_low"]
        if all(abs(low[n]) < 0.1 for n in pose["on"]):
            break
        B, feet, contact, on, extra = pose["_args"]
        feet = {n: (V(p.x, p.y, p.z - low[n]) if n in pose["on"] else p) for n, p in feet.items()}
        pose = posture(B, feet, contact, on, extra)
    return pose


def pose_report(name, pose, parts, skip=()):
    items, feet, frames = assemble(pose, parts)
    hits = collisions(items, skip=skip)
    fl = floor_check(items, pose["on"])
    r = dict(body_height_mm=round(pose["hb"], 1), envelope=envelope(items), hits=hits, floor=fl, limits_bad=limits_ok(pose),
             joints={n: [round(pose["gamma"][n], 1), round(pose["th2"][n], 1), round(pose["beta"][n], 1)] for n, _, _ in HIPS})
    r["statics"] = statics(pose, parts, (items, feet, frames))
    log("pose", name, "hits", hits, "floor", fl["feet_down_ok"], fl["no_penetration"], "limits", r["limits_bad"])
    return r, items


def gait_report(kind, parts, n=24):
    frames, worst = [], dict(margin_mm=1e9, push_g=1e9, femur=0.0, knee=0.0, min_swing_clear=1e9)
    ok = True
    for k in range(n):
        feet, stance = gait_feet(kind, k, n)
        try:
            pose = posture(T(0, 0, STAND_HB), feet, on=[f for f in feet if abs(feet[f].z - R_TIP) < 1e-6])
        except ValueError as e:
            frames.append(dict(k=k, ik=str(e)))
            ok = False
            continue
        items, ft, fr = assemble(pose, parts)
        hits = collisions(items)
        fl = floor_check(items, pose["on"])
        L = Loads(pose, parts, (items, ft, fr))
        sup = L.support(stance)
        bad = limits_ok(pose)
        swing = [fl["foot_low"][f] for f in fl["foot_low"] if f not in pose["on"]]
        row = dict(k=k, stance=stance, hits=hits, limits_bad=bad, margin_mm=sup["margin_mm"], push_g=sup["push_g"],
                   femur_kgcm=sup["femur_kgcm"], knee_kgcm=sup["knee_kgcm"], foot_low=fl["foot_low"],
                   joints={f: [round(pose["gamma"][f], 1), round(pose["th2"][f], 1), round(pose["beta"][f], 1)] for f in feet})
        frames.append(row)
        ok = ok and not hits and not bad and fl["feet_down_ok"] and fl["no_penetration"] and sup["margin_mm"] > 0
        worst["margin_mm"] = min(worst["margin_mm"], sup["margin_mm"])
        worst["push_g"] = min(worst["push_g"], sup["push_g"])
        worst["femur"] = max(worst["femur"], sup["femur_kgcm"])
        worst["knee"] = max(worst["knee"], sup["knee_kgcm"])
        if swing:
            worst["min_swing_clear"] = min(worst["min_swing_clear"], min(swing))
        log(kind, k, "hits", hits, "margin", sup["margin_mm"], "femur", sup["femur_kgcm"], "knee", sup["knee_kgcm"], "bad", bad)
    summary = dict(frames=n, all_ok=ok, min_margin_mm=worst["margin_mm"], min_push_g=worst["push_g"],
                   femur=pct(worst["femur"]), knee=pct(worst["knee"]), min_swing_foot_height_mm=round(worst["min_swing_clear"], 1))
    return dict(summary=summary, frames=frames)


def main():
    parts = build_parts()
    report = {}
    keys = ["servo", "coxa", "femur", "knee", "blade", "foot", "base", "lid", "lid_proxy", "deck", "hatch", "window", "fit_gauge", "fit_collar"]
    report["valid"] = {k: dict(valid=parts[k].isValid(), solids=len(parts[k].Solids), volume_cm3=round(parts[k].Volume / 1000, 2)) for k in keys}
    log("valid", report["valid"])
    stand = posture(T(0, 0, STAND_HB), spots(STAND_R))
    direction, place, sole = sneaker_setup(parts, stand)
    report["valid"]["foot_sneaker"] = dict(valid=parts["foot_sneaker"].isValid(), solids=len(parts["foot_sneaker"].Solids))
    # Static fit: parts that are screwed together must not overlap (servo in its socket, cradle, housing, blade in web).
    items = assemble(stand, parts)[0]
    report["static_fit_hits"] = collisions(items, mode="same")
    log("static fit", report["static_fit_hits"])
    if STAGE == "parts":
        report["print"] = export_parts(parts)
        json.dump(report, open(os.path.join(OUT, "report.json"), "w"), indent=1)
        log("stage parts done")
        return
    report["joint_sweeps"] = sweep_checks(parts)
    log("sweeps", json.dumps(report["joint_sweeps"]))
    json.dump(report, open(os.path.join(OUT, "report.json"), "w"), indent=1)
    poses = all_poses(parts)
    hb_sn = STAND_HB + math.hypot(*sole) - L_TIBIA + 6.0    # the sneaker makes the leg longer: stand higher
    for _ in range(2):      # aim the peg straight down in the sneaker pose itself, then solve again
        sn = posture(T(0, 0, hb_sn), spots(STAND_R), contact=sole, extra=dict(foot="foot_sneaker", sneaker_place=place, contact_pt=sole))
        direction, place, sole = sneaker_setup(parts, sn)
    sn = posture(T(0, 0, hb_sn), spots(STAND_R), contact=sole, extra=dict(foot="foot_sneaker", sneaker_place=place, contact_pt=sole))
    report["sneaker_hb_mm"] = round(hb_sn, 1)
    poses["sneaker_stand"] = sn
    report["poses"] = {}
    scenes = {}
    for name in list(poses):
        if name not in ("wave", "wave_2"):
            poses[name] = settle(poses[name], parts)
        pose = poses[name]
        r, items = pose_report(name, pose, parts)
        report["poses"][name] = r
        scenes[name] = scene(pose, parts)
    report["gait"] = {}
    for kind in ("walk", "turn"):
        report["gait"][kind] = gait_report(kind, parts)
        for k in (0, 3, 6, 9, 12, 18):
            feet, stance = gait_feet(kind, k)
            try:
                scenes["%s_%02d" % (kind, k)] = scene(posture(T(0, 0, STAND_HB), feet), parts)
            except ValueError as e:
                log("no scene", kind, k, e)
    report["print"] = export_parts(parts)
    pr = report["print"]
    printed = sum(v["mass_g_total"] for k, v in pr.items() if k not in ALTERNATIVES)
    petg = sum(v["mass_g_total"] for k, v in pr.items() if v["material"] == "PETG" and k not in ALTERNATIVES)
    tpu = sum(v["mass_g_total"] for k, v in pr.items() if v["material"] == "TPU")
    elec = sum(m for _, _, m in ELECTRONICS) + parts["window"].Volume / 1000 * ACRYLIC
    hw = sum(m for _, _, m in LUMPS_BODY) + 6 * LEG_HARDWARE
    report["mass_g"] = dict(printed=round(printed), servos=round(18 * MASS_SERVO), electronics=round(elec), wiring_and_hardware=round(hw),
                            total=round(printed + 18 * MASS_SERVO + elec + hw))
    report["filament"] = dict(petg_g=round(petg), tpu_g=round(tpu), petg_m_1_75=round(petg / (PETG * math.pi * 0.0875 ** 2) / 100, 1),
                              note="at %.0f%% effective density, without supports or purge" % (INFILL * 100))
    report["sneaker"] = dict(peg_direction_deg=round(direction, 1), sole_contact_in_tibia_frame=[round(sole[0], 1), round(sole[1], 1)])
    report["params"] = dict(R_HIP=R_HIP, L_COXA=L_COXA, Z_F=Z_F, L_FEMUR=L_FEMUR, L_TIBIA=L_TIBIA, STAND_HB=STAND_HB, STAND_R=STAND_R, GAIT_R=GAIT_R,
                            GAIT=dict(frames=24, stride_mm=50, lift_mm=GAIT_LIFT, swing_out_mm=GAIT_OUT, turn_deg_per_cycle=16),
                            NAMED_HEIGHTS=NAMED_HEIGHTS,
                            COXA_RANGE=COXA_RANGE, TH2_RANGE=TH2_RANGE, BETA_RANGE=BETA_RANGE, V_SERVO=V_SERVO,
                            RATED=RATED, STALL=STALL, MASS_SERVO=MASS_SERVO, FIT=FIT, INFILL=INFILL)
    json.dump(report, open(os.path.join(OUT, "report.json"), "w"), indent=1)
    dump_meshes(parts, [k for k in RENDER_KEYS if k in parts])
    json.dump(scenes, open(os.path.join(OUT, "meshes", "scenes.json"), "w"))
    log("done")


try:
    main()
except Exception:
    import traceback
    log("FAILED\n" + traceback.format_exc())
    raise
