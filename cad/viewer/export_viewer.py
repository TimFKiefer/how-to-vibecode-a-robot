# SPDX-License-Identifier: MIT
# Data for the Takt viewer, taken from the CAD script one folder up (takt.py) with FreeCAD. Writes only into this folder:
#   meshes.json     one mesh per part key in the part's own frame (base64 float32 positions, uint16/32 indices)
#   model.json      constants, instance map, masses and centroids, presets, gait parameters, numbers from report.json,
#                   and the height grid from blade120_report.json (read only)
#   reference.json  FreeCAD results for selftest.mjs: joints, placements, feet, centre of mass, margins, torques;
#                   also the stand, the page's poses and all 48 gait frames at every height the height study checked
#   export_viewer.log
# Run: nice -n 15 freecadcmd cad/viewer/export_viewer.py
import base64, json, math, os, random, sys
import numpy as np

VIEW = os.path.dirname(os.path.abspath(__file__ if "__file__" in globals() else sys.argv[-1]))
CAD = os.path.dirname(VIEW)     # the CAD folder: takt.py, or robot2.py under its earlier name
CAD_PY = next(p for p in (CAD + "/takt.py", CAD + "/robot2.py") if os.path.exists(p))
H120 = next(p for p in (CAD + "/blade120_report.json", CAD + "/variants/H_hoehe/blade120/report.json") if os.path.exists(p))
SRC = open(CAD_PY).read().rsplit("\ntry:\n    main()", 1)[0]
# The CAD script opens its own log at import; point it here so robot2.log stays untouched, and keep OUT on the CAD folder.
for a, b in (('OUT = os.environ.get("TAKT_OUT") or os.path.dirname(os.path.abspath(__file__ if "__file__" in globals() else sys.argv[-1]))', "OUT = %r" % CAD),
             ('LOG = open(os.path.join(OUT, "robot2.log"), "w")', 'LOG = open(%r, "w")' % (VIEW + "/export_viewer.log"))):
    assert SRC.count(a) == 1, a
    SRC = SRC.replace(a, b)
assert 'robot2.log"), "w")' not in SRC
exec(SRC)
import MeshPart

log("viewer export: building parts")
parts = build_parts()

# --- the same pose set main() builds, sneaker included ---
stand0 = posture(T(0, 0, STAND_HB), spots(STAND_R))
direction, place, sole = sneaker_setup(parts, stand0)
hb_sn = STAND_HB + math.hypot(*sole) - L_TIBIA + 6.0
for _ in range(2):
    sn = posture(T(0, 0, hb_sn), spots(STAND_R), contact=sole, extra=dict(foot="foot_sneaker", sneaker_place=place, contact_pt=sole))
    direction, place, sole = sneaker_setup(parts, sn)
sn = posture(T(0, 0, hb_sn), spots(STAND_R), contact=sole, extra=dict(foot="foot_sneaker", sneaker_place=place, contact_pt=sole))
POSES = all_poses(parts)
POSES["sneaker_stand"] = sn
for name in list(POSES):
    if name not in ("wave", "wave_2"):
        POSES[name] = settle(POSES[name], parts)
log("poses", list(POSES), "sneaker hb", round(hb_sn, 3))


def M16(m):
    return [float(v) for v in m.A]


# --- instance map, in assemble() order: frame "body" or "leg:<leg>:<link>", optional fixed offset ---
BODY_ITEMS = [("base", "base", "base"), ("lid", "lid", "lid_proxy"), ("deck", "deck", "deck"), ("hatch", "hatch", "hatch"), ("window", "window", "window")]
BODY_ITEMS += [("e_" + n, "e_" + n, "e_" + n) for n, _, _ in ELECTRONICS]
RENDER_ONLY = {"label", "eyes"}
SHOWN_E = {"e_camera", "e_led"}      # the other electronics sit inside the closed body
INST = []
for n, mesh, phys in BODY_ITEMS:
    INST.append(dict(name=n, key=mesh, phys=phys, frame="body", offset=None, variant=None, render=(not n.startswith("e_")) or n in SHOWN_E))
for leg, _, _ in HIPS:
    for link, key in (("s1", "servo"), ("coxa", "coxa"), ("s2", "servo"), ("femur", "femur"), ("s3", "servo"), ("tibia", "knee"), ("tibia", "blade")):
        INST.append(dict(name="%s_%s" % (leg, key if link == "tibia" else link), key=key, phys=key, frame="leg:%s:%s" % (leg, link), offset=None, variant=None, render=True))
    INST.append(dict(name=leg + "_foot", key="foot", phys="foot", frame="leg:%s:tibia" % leg, offset=None, variant="claw", render=True))
    INST.append(dict(name=leg + "_foot_sneaker", key="foot_sneaker", phys="foot_sneaker", frame="leg:%s:tibia" % leg, offset=None, variant="sneaker", render=True))
    for k in ("sneaker_white", "sneaker_red", "sneaker_stripe"):
        INST.append(dict(name="%s_%s" % (leg, k), key=k, phys=k, frame="leg:%s:tibia" % leg, offset=M16(place), variant="sneaker", render=True))
INST.append(dict(name="eyes", key="eyes", phys=None, frame="body", offset=None, variant=None, render=True))
for leg, _, _ in HIPS:
    for link in ("s1", "s2", "s3"):
        INST.append(dict(name="%s_label_%s" % (leg, link), key="label", phys=None, frame="leg:%s:%s" % (leg, link), offset=None, variant=None, render=True))


def frame_of(inst, B, frames):
    if inst["frame"] == "body":
        F = B
    else:
        _, leg, link = inst["frame"].split(":")
        F = frames[leg][link]
    if inst["offset"] is not None:
        F = F.multiply(Matrix(*inst["offset"]))
    return F


def variant_of(pose):
    return "sneaker" if pose.get("foot") == "foot_sneaker" else "claw"


def active(inst, var):
    return inst["variant"] in (None, var)


# Check the map against assemble(render=True): same items, same matrices.
for pname in ("stand", "sneaker_stand", "wave"):
    p = POSES[pname]
    items, feet, frames = assemble(p, parts, render=True)
    B = body_frame(p)
    var = variant_of(p)
    mine = [i for i in INST if active(i, var)]
    got = {n: m for n, k, s, g, m in items}
    assert sorted(got) == sorted(i["name"] for i in mine), (pname, sorted(set(got) ^ set(i["name"] for i in mine)))
    for inst in mine:
        F = frame_of(inst, B, frames)
        d = max(abs(a - b) for a, b in zip(F.A, got[inst["name"]].A))
        assert d < 1e-9, (pname, inst["name"], d)
log("instance map matches assemble()")

# --- meshes, each once in its own frame ---
def mesh_entry(shape, defl=0.3, ang=0.3):
    mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=defl, AngularDeflection=ang)
    pts, fac = mesh.Topology
    pos = np.array([[p.x, p.y, p.z] for p in pts], dtype="<f4")
    big = len(pts) > 65535
    idx = np.array(fac, dtype="<u4" if big else "<u2")
    return dict(nv=len(pts), nt=len(fac), idx_type="u32" if big else "u16",
                pos=base64.b64encode(pos.tobytes()).decode(), idx=base64.b64encode(idx.tobytes()).decode())


MESHES = {}
for k in sorted({i["key"] for i in INST if i["render"]}):
    MESHES[k] = mesh_entry(parts[k], 0.25 if k in ("eyes", "label", "e_led") else 0.3)
    log("mesh", k, MESHES[k]["nv"], MESHES[k]["nt"])
json.dump(MESHES, open(VIEW + "/meshes.json", "w"), separators=(",", ":"))

# --- physics: what Loads uses (part_mass and the centroid of each part in its own frame) ---
PHYS = {}
for k in sorted({i["phys"] for i in INST if i["phys"]}):
    c = centroid(parts[k])
    PHYS[k] = dict(mass=part_mass(k, parts[k]), c=[c.x, c.y, c.z])


# --- unrounded statics, the same arithmetic as Loads.support / statics (asserted against the rounded versions) ---
def support_raw(L, legs):
    pts = [(L.feet[n].x, L.feet[n].y) for n in legs]
    c2 = (L.com.x, L.com.y)
    margin = hull_margin(pts, c2)
    shares = share_load(pts, c2)
    w = {n: shares.get(i, 0.0) * L.total / 1000.0 for i, n in enumerate(legs)}
    per = {n: {j: L.torque(n, j, w.get(n, 0.0)) for j in ("femur", "knee")} for n, _, _ in HIPS}
    h = L.com.z - min(L.feet[n].z - R_TIP for n in legs)
    out = dict(legs=list(legs), margin=margin, push_g=margin / h, com_height=h, femur=max(v["femur"] for v in per.values()),
               knee=max(v["knee"] for v in per.values()), max_foot_load_g=max(w.values()) * 1000, per_leg=per, w={n: v * 1000 for n, v in w.items()})
    r = L.support(legs)
    assert round(out["margin"], 1) == r["margin_mm"] and round(out["push_g"], 2) == r["push_g"], (out["margin"], r)
    assert round(out["femur"], 2) == r["femur_kgcm"] and round(out["knee"], 2) == r["knee_kgcm"], (out["femur"], r)
    return out


def resting_raw(L, pose):
    B = body_frame(pose)
    belly = [B.multiply(V(sx * 40.0, sy * 27.0, -2.0)) for sx in (1, -1) for sy in (1, -1)]
    m = hull_margin([(p.x, p.y) for p in belly], (L.com.x, L.com.y))
    tq = {j: max(L.torque(n, j, 0.0) for n, _, _ in HIPS) for j in ("femur", "knee")}
    return dict(margin=m, femur=tq["femur"], knee=tq["knee"], belly=[[p.x, p.y, p.z] for p in belly])


def record(pid, pose, kind, stance=None, with_placements=True):
    items, feet, frames = assemble(pose, parts, render=True)
    B = body_frame(pose)
    var = variant_of(pose)
    pl = {}
    for inst in INST:
        if active(inst, var) and with_placements:
            pl[inst["name"]] = M16(frame_of(inst, B, frames))
    L = Loads(pose, parts, (items, feet, frames))
    rec = dict(id=pid, kind=kind, variant=var, B=M16(B),
               joints={n: [pose["gamma"][n], pose["th2"][n], pose["beta"][n]] for n, _, _ in HIPS},
               feet={n: [v.x, v.y, v.z] for n, v in feet.items()}, placements=pl,
               mass=L.total, com=[L.com.x, L.com.y, L.com.z], on=list(pose.get("on") or []))
    if pose.get("rest_on_belly"):
        rec["resting"] = resting_raw(L, pose)
        rec["support"] = support_raw(L, pose["on"])
    else:
        rec["support"] = support_raw(L, stance or pose["on"])
        if stance is None and len(pose["on"]) == 6:
            rec["tripod_A"] = support_raw(L, list(TRI_A))
            rec["tripod_B"] = support_raw(L, list(TRI_B))
    if kind.startswith("h"):
        rec["limits"] = [[n, j, v] for n, j, v in limits_ok(pose)]
    if "_args" in pose:
        Bm, ft, contact, on, extra = pose["_args"]
        rec["targets"] = {n: [p.x, p.y, p.z] for n, p in ft.items()}
        rec["contact"] = list(contact)
    return rec


REF = []
for name, pose in POSES.items():
    REF.append(record("pose_" + name, pose, "pose"))
    log("ref pose", name, "margin", round(REF[-1]["support"]["margin"], 3))
for kind in ("walk", "turn"):
    for k in range(24):
        feet, stance = gait_feet(kind, k)
        pose = posture(T(0, 0, STAND_HB), feet, on=[f for f in feet if abs(feet[f].z - R_TIP) < 1e-6])
        r = record("%s_%02d" % (kind, k), pose, "gait", stance)
        r["gait"] = dict(kind=kind, k=k, stance=stance, targets={n: [p.x, p.y, p.z] for n, p in feet.items()})
        REF.append(r)
log("ref gait frames", 48)

# Odd poses: random body placement with IK on the stand spots, and random joints with no IK, both feet variants.
rnd = random.Random(20261109)
odd = 0
while odd < 8:
    hb = rnd.uniform(45, 85)
    pitch, yaw = rnd.uniform(-9, 9), rnd.uniform(-9, 9)
    sx, sy = rnd.uniform(-10, 10), rnd.uniform(-10, 10)
    B = pitch_about(hb, pitch=pitch, yaw=yaw, shift=(sx, sy))
    try:
        if odd % 2 == 0:
            extra = None if odd % 4 == 0 else dict(foot="foot_sneaker", sneaker_place=place, contact_pt=sole)
            contact = (L_TIBIA, 0.0) if extra is None else sole
            pose = posture(B, spots(STAND_R + rnd.uniform(-5, 20)), contact=contact, extra=extra)
            body = dict(hb=hb, pitch=pitch, yaw=yaw, x=sx, y=sy)
        else:
            pose = dict(B=B, hb=hb, gamma={}, th2={}, beta={}, on=[])
            for n, _, d in HIPS:
                pose["gamma"][n] = d + rnd.uniform(-COXA_RANGE, COXA_RANGE)
                pose["th2"][n] = rnd.uniform(*TH2_RANGE)
                pose["beta"][n] = rnd.uniform(*BETA_RANGE)
            pose["on"] = [n for n, _, _ in HIPS][: rnd.randint(3, 6)]
            if odd % 4 == 3:
                pose.update(foot="foot_sneaker", sneaker_place=place, contact_pt=sole)
            body = dict(hb=hb, pitch=pitch, yaw=yaw, x=sx, y=sy)
    except ValueError:
        continue
    r = record("odd_%d" % odd, pose, "odd")
    r["body"] = body
    REF.append(r)
    odd += 1
log("ref odd poses", odd)

# --- belly height as a free parameter: the heights the height study checked with V0's parts ---
HREP = json.load(open(H120))
assert abs(HREP["stand_r"] - STAND_R) < 1e-9 and abs(HREP["blade_mm"] - L_TIBIA) < 1e-9
REAR = R_HIP * math.sin(math.radians(60))
BASE_BODY = dict(stand=dict(hb=STAND_HB), look_up=dict(hb=STAND_HB + REAR * math.sin(math.radians(LOOKUP_PITCH)), pitch=LOOKUP_PITCH,
                 y=-REAR * (1 - math.cos(math.radians(LOOKUP_PITCH)))), look_around=dict(hb=STAND_HB, yaw=10.0),
                 wave=dict(hb=STAND_HB, x=6.0, y=-6.0), wave_2=dict(hb=STAND_HB, x=6.0, y=-6.0))
HEIGHTS, DROPPED = [], []


def outside_ranges(hb, g):
    """The CAD script's own joint ranges at this height: the stand and all 48 gait frames (the height study used the wider swept ranges)."""
    bad = [("stand",) + tuple(b) for b in limits_ok(posture(T(0, 0, hb), spots(STAND_R)))]
    for kind in ("walk", "turn"):
        for k in range(24):
            feet, _ = gait_feet(kind, k, 24, 50.0, g["lift"], g["swing_out"], 16.0, spots(g["gait_r"]))
            bad += [("%s_%02d" % (kind, k),) + tuple(b) for b in limits_ok(posture(T(0, 0, hb), feet))]
    return bad


for key, h in sorted(HREP["heights"].items(), key=lambda kv: float(kv[0])):
    if not h.get("gait"):
        continue
    g, hb = h["gait"], h["hb"]
    bad = outside_ranges(hb, g)
    if bad or DROPPED:          # the slider ends below the first height the CAD script's ranges reject
        DROPPED.append(dict(hb=hb, why=[[str(x) for x in b] for b in bad[:4]] or "above a rejected height"))
        log("height dropped", hb, bad[:4])
        continue
    tri = h["stand"]["statics"]["tripod_A"]
    row = dict(hb=hb, gait_r=g["gait_r"], lift=g["lift"], out=g["swing_out"],
               walk=dict(femur=g["walk"]["femur"]["pct_rated"], knee=g["walk"]["knee"]["pct_rated"], margin=g["walk"]["min_margin_mm"], push=g["walk"]["min_push_g"], ok=g["walk"]["all_ok"]),
               turn=dict(femur=g["turn"]["femur"]["pct_rated"], knee=g["turn"]["knee"]["pct_rated"], margin=g["turn"]["min_margin_mm"], push=g["turn"]["min_push_g"], ok=g["turn"]["all_ok"]),
               stand=dict(hits=len(h["stand"]["hits"]), tripod_margin=tri["margin_mm"], tripod_push=tri["push_g"],
                          tripod_pct=max(tri["femur"]["pct_rated"], tri["knee"]["pct_rated"])),
               step_h=h["step"]["h"], shuffle_ok=bool(h.get("shuffle", {}).get("ok")), knee_vs_lid=h.get("knee_vs_lid"), poses={})
    # stand at this height, with placements
    sp = posture(T(0, 0, hb), spots(STAND_R))
    REF.append(record("h%g_stand" % hb, sp, "hstand"))
    REF[-1]["hb"] = hb
    # the page's poses at this height: Takt's preset targets, body moved by (hb - 62); FreeCAD collision check
    for name, b0 in BASE_BODY.items():
        if name == "stand":
            continue
        _, ft, contact, on, extra = POSES[name]["_args"]
        b = dict(hb=0.0, pitch=0.0, yaw=0.0, x=0.0, y=0.0)
        b.update(b0)
        b["hb"] += hb - STAND_HB
        Bp = pitch_about(b["hb"], pitch=b["pitch"], yaw=b["yaw"], shift=(b["x"], b["y"]))
        try:
            pz = posture(Bp, ft, contact=contact, on=on)
        except ValueError as e:
            row["poses"][name] = dict(ik=str(e))
            continue
        if name.startswith("wave"):
            for j, v in zip(("gamma", "th2", "beta"), (POSES[name][q]["LF"] for q in ("gamma", "th2", "beta"))):
                pz[j]["LF"] = v
        items = assemble(pz, parts)[0]
        fl = floor_check(items, pz["on"])
        lim = limits_ok(pz)
        row["poses"][name] = dict(hits=len(collisions(items)), floor_ok=bool(fl["feet_down_ok"] and fl["no_penetration"]), limits=len(lim))
        r = record("h%g_%s" % (hb, name), pz, "hpose", with_placements=False)
        r.update(hb=hb, preset=name, body=b)
        REF.append(r)
    # all 24 walk and 24 turn frames at this height, at its walking radius, lift and swing
    for kind in ("walk", "turn"):
        for k in range(24):
            feet, stance = gait_feet(kind, k, 24, 50.0, g["lift"], g["swing_out"], 16.0, spots(g["gait_r"]))
            pose = posture(T(0, 0, hb), feet, on=[f for f in feet if abs(feet[f].z - R_TIP) < 1e-6])
            r = record("h%g_%s_%02d" % (hb, kind, k), pose, "hgait", stance, with_placements=False)
            r["gait"] = dict(kind=kind, k=k, stance=stance, targets={n: [p.x, p.y, p.z] for n, p in feet.items()})
            r["hb"] = hb
            REF.append(r)
    HEIGHTS.append(row)
    log("ref height", hb, "poses", row["poses"])
log("ref heights", [r["hb"] for r in HEIGHTS])

# --- presets for the page: body parameters that rebuild the FreeCAD body frame, exact foot targets, overrides ---
REAR = R_HIP * math.sin(math.radians(60))
LOOKUP_HB = STAND_HB + REAR * math.sin(math.radians(LOOKUP_PITCH))
LOOKUP_Y = -REAR * (1 - math.cos(math.radians(LOOKUP_PITCH)))
PRESET_BODY = dict(stand=dict(hb=STAND_HB), low_stand=dict(hb=NAMED_HEIGHTS["spinne"]["hb"]), sit=dict(hb=2.0),
                   look_up=dict(hb=LOOKUP_HB, pitch=LOOKUP_PITCH, y=LOOKUP_Y), look_around=dict(hb=STAND_HB, yaw=10.0),
                   wave=dict(hb=STAND_HB, x=6.0, y=-6.0), wave_2=dict(hb=STAND_HB, x=6.0, y=-6.0), sneaker_stand=dict(hb=hb_sn))
LABELS = dict(stand="Stand", low_stand="Low stand", sit="Sit", look_up="Look up", look_around="Look around", wave="Wave",
              wave_2="Wave (2)", sneaker_stand="Stand with sneakers")
PRESETS = []
for name, pose in POSES.items():
    b = dict(hb=0.0, pitch=0.0, yaw=0.0, x=0.0, y=0.0)
    b.update(PRESET_BODY[name])
    Bp = pitch_about(b["hb"], pitch=b["pitch"], yaw=b["yaw"], shift=(b["x"], b["y"]))
    d = max(abs(a - c) for a, c in zip(Bp.A, body_frame(pose).A))
    assert d < 1e-9, (name, d)
    _, ft, contact, on, extra = pose["_args"]
    over = {}
    if name.startswith("wave"):
        over = {"LF": [pose["gamma"]["LF"], pose["th2"]["LF"], pose["beta"]["LF"]]}
    PRESETS.append(dict(id=name, label=LABELS[name], body=b, targets={n: [p.x, p.y, p.z] for n, p in ft.items()}, contact=list(contact),
                        on=list(pose["on"]), variant=variant_of(pose), overrides=over, rest_on_belly=bool(pose.get("rest_on_belly"))))

# --- numbers from report.json for the panel ---
rep = json.load(open(CAD + "/report.json"))
P = rep["poses"]
NUM = dict(
    mass_total_g=rep["mass_g"]["total"], mass=rep["mass_g"],
    stand_joints_RM=P["stand"]["joints"]["RM"], beta_range=rep["params"]["BETA_RANGE"], th2_range=rep["params"]["TH2_RANGE"], coxa_range=rep["params"]["COXA_RANGE"],
    gait=rep["params"]["GAIT"], gait_r=rep["params"]["GAIT_R"], stand_r=rep["params"]["STAND_R"],
    walk=rep["gait"]["walk"]["summary"], turn=rep["gait"]["turn"]["summary"],
    low_tripod=P["low_stand"]["statics"]["tripod_A"], low_six=P["low_stand"]["statics"]["support"],
    sit_up=P["sit"]["statics"]["stand_up_six"], stand=P["stand"]["statics"],
    sneaker_hb=rep["sneaker_hb_mm"], rated=rep["params"]["RATED"], stall=rep["params"]["STALL"],
    sweeps_outside=rep["joint_sweeps"]["knee_outside_range_deg"], sweeps_pass=rep["joint_sweeps"]["pass"],
    hits=dict((k, len(v["hits"])) for k, v in P.items()))
UNVERIFIED = [
    dict(name="S_NOSE", value=S_NOSE, text="Shaft axis to the nose end, from the drawing raster"),
    dict(name="S_STEP", value=S_STEP, text="Step where the 29 mm nose block meets the 32 mm tail block, from the raster"),
    dict(name="S_TAIL_Z0 / S_TAIL_Z1", value=[S_TAIL_Z0, S_TAIL_Z1], text="Split of the 32 mm tail block height"),
    dict(name="S_LIP / S_BUMP", value=[S_LIP, list(S_BUMP)], text="The lip and the bump that make the 35 mm overall height"),
    dict(name="HORN_FACE", value=HORN_FACE, text="Where the output horn sits above the back face (not in the drawing)"),
    dict(name="IDLER_FACE", value=IDLER_FACE, text="Where the idler horn sits below the back face (not in the drawing)"),
    dict(name="HOLE_TOP / HOLE_BOT", value=[HOLE_TOP, HOLE_BOT], text="The PA2.0 case holes, from the raster"),
    dict(name="PLUG", value=list(PLUG), text="Plug depth 7 mm, for the plug plus the cable bend"),
]

C = dict(R_HIP=R_HIP, L_COXA=L_COXA, Z_F=Z_F, L_FEMUR=L_FEMUR, L_TIBIA=L_TIBIA, R_TIP=R_TIP, S_MID=S_MID, Z_S1=Z_S1, S3_DIR=S3_DIR,
         HIPS=[list(h) for h in HIPS], TRI_A=list(TRI_A), TRI_B=list(TRI_B), COXA_RANGE=COXA_RANGE, TH2_RANGE=list(TH2_RANGE),
         BETA_RANGE=list(BETA_RANGE), STAND_HB=STAND_HB, STAND_R=STAND_R, GAIT_R=GAIT_R, RATED=RATED["6.0"] if "6.0" in RATED else RATED[6.0],
         STALL=STALL[6.0], LEG_HARDWARE=LEG_HARDWARE, LUMPS_BODY=[[n, [p.x, p.y, p.z], m] for n, p, m in LUMPS_BODY],
         GAIT=dict(n=24, stride=50.0, lift=GAIT_LIFT, out=GAIT_OUT, turn=16.0), BELLY=[40.0, 27.0, -2.0],
         SNEAKER=dict(place=M16(place), contact=list(sole), hb=hb_sn, direction=direction,
                      contact_z=POSES["sneaker_stand"]["_args"][1]["RM"].z))   # settled height of the sole contact point
COLORS = dict(black=[40, 41, 45], servo=[26, 27, 29], red=[200, 40, 46], eye=[250, 250, 245], glass=[12, 12, 16], tpu=[34, 34, 37],
              white=[236, 236, 232], lens=[10, 10, 14])
KEYCOL = dict(base="black", lid="black", deck="black", hatch="black", window="glass", eyes="eye", servo="servo", label="red", coxa="black",
              femur="black", knee="black", blade="black", foot="tpu", foot_sneaker="black", sneaker_white="white", sneaker_red="red",
              sneaker_stripe="white", e_camera="lens", e_led="lens")
HEIGHT = dict(source="cad/blade120_report.json (height study, FreeCAD)", stand_r=STAND_R, range=[HEIGHTS[0]["hb"], HEIGHTS[-1]["hb"]],
              grid=HEIGHTS, dropped=DROPPED, named=NAMED_HEIGHTS, transition_ok=HREP["transition"]["ok"],
              transition=[HREP["transition"]["from_mm"], HREP["transition"]["to_mm"]])
model = dict(note="Exported by cad/viewer/export_viewer.py from the CAD script (cad/takt.py) with FreeCAD. mm, g, deg; body frame X right, Y front, Z up, floor z = 0.",
             C=C, instances=INST, physics=PHYS, presets=PRESETS, colors=COLORS, keycolor=KEYCOL, numbers=NUM, unverified=UNVERIFIED, height=HEIGHT)
json.dump(model, open(VIEW + "/model.json", "w"), separators=(",", ":"))
json.dump(dict(note="FreeCAD reference for selftest.mjs; placements are 4x4 row-major (FreeCAD Matrix.A)", poses=REF),
          open(VIEW + "/reference.json", "w"), separators=(",", ":"))
log("viewer export done")
