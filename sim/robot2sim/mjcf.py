# SPDX-License-Identifier: MIT
"""Write the MuJoCo model for one body file.

Joint coordinates are the CAD script's coxa γ, femur θ2 and knee β, in radians.
Positive femur motion is axis (0, 0, -1) so q = θ2 produces Rz(-θ2).
"""

from robot2sim.frames import Rx, Rz, apply, mat16, quat_of, translation
from robot2sim.units import deg_to_rad, g_to_kg, mm_to_m

# The body file's plain position actuators (step 1). servo.apply_motor replaces the gains; the episode range is
# randomise.RANGES (kp_scale × servo.KP = 56…104).
KP = 100.0  # UNVERIFIED
KV = 2.0  # UNVERIFIED

# TPU tip, design range 0.6..1.2 (randomise.RANGES foot_mu). Nominal is the midpoint. The statics check
# sets this to zero. Torsion and roll are MuJoCo's contact defaults.
MU = 0.9  # UNVERIFIED
MU_TORSION = 0.005  # UNVERIFIED
MU_ROLL = 0.0001  # UNVERIFIED

# Damping on the free base only (N·s/m and N·m·s/rad). With frictionless feet a
# striding tripod has nothing to kill a horizontal rock, and walk_01 is still
# moving at 2 s without it. The force is zero at rest, so the static torques
# are unchanged. Not a measured drag.
FREE_DAMPING = 1.0  # UNVERIFIED

# Leg joints carry the servo's reflected rotor inertia. With zero rotor inertia a
# frictionless tripod chatters on the stiff contact; the servo model (robot2sim/servo.py) uses this same constant.
ARMATURE = 0.005  # kg·m², reflected rotor inertia of the geared servo  # UNVERIFIED

FRICTION = f"{MU:.4f} {MU_TORSION:.4f} {MU_ROLL:.6f}"


def _stall_kgcm(body, servo):
    return float(body["servo_types"][servo]["stall_kgcm"])


def write_mjcf(body, path=None):
    raise RuntimeError(
        "The step 2–4 write_mjcf was replaced by this step-1 generator (robot2sim.mjcf.to_xml). "
        "The previous model is still at sim/assets/v0.xml."
    )


def _f(value, digits=8):
    return f"{float(value):.{digits}g}"


def _pos_mm(xyz):
    return " ".join(_f(mm_to_m(v)) for v in xyz)


def _quat(q):
    return " ".join(f"{float(v):.10f}" for v in q)


def _range_rad(lo, hi):
    return f"{deg_to_rad(lo):.10f} {deg_to_rad(hi):.10f}"


def _joint_spec(kind, hip, c):
    if kind == "coxa":
        d = hip["delta_deg"]
        return "0 0 1", _range_rad(d - c["COXA_RANGE"], d + c["COXA_RANGE"])
    if kind == "femur":
        return "0 0 -1", _range_rad(c["TH2_RANGE"][0], c["TH2_RANGE"][1])
    if kind == "knee":
        return "0 0 1", _range_rad(c["BETA_RANGE"][0], c["BETA_RANGE"][1])
    raise ValueError(kind)


def _emit_parts(lines, parts, spec, pad):
    radius = _f(mm_to_m(spec["point_mass_radius_mm"]))
    for part in parts:
        pos, quat = _pos_mm(part["pos_mm"]), _quat(part["quat"])
        if part["site"]:
            lines.append(f'{pad}<site name="{part["name"]}" pos="{pos}" quat="{quat}"/>')
        if part["mesh"]:
            mass = ""
            if part["mass_on_mesh"]:
                mass = f' mass="{_f(g_to_kg(part["mass_g"]), 10)}"'
            rgba = " ".join(_f(v, 4) for v in part["rgba"])
            lines.append(
                f'{pad}<geom name="{part["name"]}_vis" type="mesh" mesh="{part["mesh"]}" '
                f'pos="{pos}" quat="{quat}"{mass} rgba="{rgba}"/>'
            )
        if part["mass_g"] > 0 and not part["mass_on_mesh"]:
            lines.append(
                f'{pad}<geom name="{part["name"]}_mass" class="mass" type="sphere" '
                f'pos="{_pos_mm(part["com_mm"])}" size="{radius}" '
                f'mass="{_f(g_to_kg(part["mass_g"]), 10)}"/>'
            )


def _stand_qpos(spec):
    pose = spec["poses"]["stand"]
    B = mat16(pose["B"])
    q = [mm_to_m(v) for v in translation(B)]
    q.extend(float(v) for v in quat_of(B))
    for hip in spec["hips"]:
        q.extend(deg_to_rad(a) for a in pose["joints_deg"][hip["name"]])
    return " ".join(f"{v:.10f}" for v in q)


def to_xml(spec):
    c = spec["constants"]
    col = spec["collision"]
    cap_r = _f(mm_to_m(col["capsule_radius_mm"]))
    foot_r = _f(mm_to_m(col["foot_radius_mm"]))
    lines = [
        '<?xml version="1.0"?>',
        f'<mujoco model="robot2_{spec["name"]}">',
        '  <compiler angle="radian" meshdir="../assets/meshes" balanceinertia="true" autolimits="true"/>',
        f'  <option timestep="0.002" gravity="0 0 -{_f(9.80665, 6)}" integrator="implicitfast" cone="elliptic"/>',
        f'  <statistic center="0 0 {_f(mm_to_m(c["STAND_HB"]))}" extent="0.4"/>',
        "  <default>",
        '    <geom density="0" contype="0" conaffinity="0" group="2"/>',
        '    <default class="collision">',
        '      <geom group="3" contype="2" conaffinity="3" solref="0.004 1" density="0" rgba="0.3 0.6 0.35 0.25"/>',
        "    </default>",
        '    <default class="mass">',
        '      <geom group="4" contype="0" conaffinity="0" density="0"/>',
        "    </default>",
        "  </default>",
        "  <asset>",
    ]
    meshes = sorted({p["mesh"] for p in _all_parts(spec) if p["mesh"]})
    for name in meshes:
        lines.append(f'    <mesh name="{name}" file="{name}.obj"/>')
    lines.append('    <mesh name="body_hex" file="body_hex.obj"/>')
    lines += [
        "  </asset>",
        "  <worldbody>",
        '    <light pos="0.4 -0.3 0.8" dir="-0.4 0.3 -0.8" directional="true"/>',
        f'    <geom name="floor" type="plane" size="1 1 0.01" contype="1" conaffinity="1" '
        f'friction="{FRICTION}" solref="0.004 1" rgba="0.86 0.85 0.82 1"/>',
        '    <body name="body" pos="0 0 0">',
        f'      <joint name="root" type="free" damping="{_f(FREE_DAMPING)}"/>',
        f'      <camera name="front" pos="{_pos_mm(spec["camera"]["pos_mm"])}" xyaxes="1 0 0 0 0 1" mode="fixed"/>',
        '      <geom name="body_hex" class="collision" type="mesh" mesh="body_hex"/>',
    ]
    _emit_parts(lines, spec["body"]["parts"], spec, "      ")
    for hip in spec["hips"]:
        leg = hip["name"]
        mount = apply(Rz(hip["phi_deg"]), [c["R_HIP"], 0.0, 0.0])
        lines.append(f'      <body name="{leg}_coxa" pos="{_pos_mm(mount)}" quat="{_quat(quat_of(Rz(hip["phi_deg"])))}">')
        axis, rng = _joint_spec("coxa", hip, c)
        lines.append(f'        <joint name="{leg}_coxa" type="hinge" axis="{axis}" range="{rng}" damping="0" frictionloss="0" armature="{_f(ARMATURE)}"/>')
        _emit_parts(lines, spec["legs"][leg]["coxa"]["parts"], spec, "        ")
        _emit_capsule(lines, col, leg, "coxa", cap_r, "        ")
        femur_pos = [c["L_COXA"], 0.0, c["Z_F"]]
        lines.append(f'        <body name="{leg}_femur" pos="{_pos_mm(femur_pos)}" quat="{_quat(quat_of(Rx(-90.0)))}">')
        axis, rng = _joint_spec("femur", hip, c)
        lines.append(f'          <joint name="{leg}_femur" type="hinge" axis="{axis}" range="{rng}" damping="0" frictionloss="0" armature="{_f(ARMATURE)}"/>')
        _emit_parts(lines, spec["legs"][leg]["femur"]["parts"], spec, "          ")
        _emit_capsule(lines, col, leg, "femur", cap_r, "          ")
        lines.append(f'          <body name="{leg}_tibia" pos="{_pos_mm([c["L_FEMUR"], 0.0, 0.0])}">')
        axis, rng = _joint_spec("knee", hip, c)
        lines.append(f'            <joint name="{leg}_knee" type="hinge" axis="{axis}" range="{rng}" damping="0" frictionloss="0" armature="{_f(ARMATURE)}"/>')
        _emit_parts(lines, spec["legs"][leg]["tibia"]["parts"], spec, "            ")
        _emit_capsule(lines, col, leg, "tibia", cap_r, "            ")
        lines.append(
            f'            <geom name="{leg}_foot_tip" class="collision" type="sphere" '
            f'pos="{_pos_mm(spec["foot_contact_mm"])}" size="{foot_r}" friction="{FRICTION}"/>'
        )
        lines.append("          </body>")
        lines.append("        </body>")
        lines.append("      </body>")
    lines.append("    </body>")
    lines.append("  </worldbody>")
    lines.append("  <actuator>")
    for hip in spec["hips"]:
        for kind in ("coxa", "femur", "knee"):
            _axis, rng = _joint_spec(kind, hip, c)
            name = f"{hip['name']}_{kind}"
            lines.append(
                f'    <position name="{name}" joint="{name}" kp="{_f(KP)}" kv="{_f(KV)}" ctrlrange="{rng}"/>'
            )
    lines.append("  </actuator>")
    lines.append("  <keyframe>")
    lines.append(f'    <key name="stand" qpos="{_stand_qpos(spec)}"/>')
    lines.append("  </keyframe>")
    lines.append("</mujoco>")
    return "\n".join(lines) + "\n"


def _all_parts(spec):
    yield from spec["body"]["parts"]
    for leg in spec["legs"].values():
        for link in leg.values():
            yield from link["parts"]


def _emit_capsule(lines, col, leg, link, radius, pad):
    cap = col["capsules_mm"][link]
    a, b = cap["from"], cap["to"]
    lines.append(
        f'{pad}<geom name="{leg}_{link}_cap" class="collision" type="capsule" '
        f'fromto="{_pos_mm(a)} {_pos_mm(b)}" size="{radius}"/>'
    )


def write_xml(spec, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(to_xml(spec), encoding="utf-8")
