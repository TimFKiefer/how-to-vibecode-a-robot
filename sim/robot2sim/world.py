# SPDX-License-Identifier: MIT
"""One servo-driven robot on the floor: reset without penetration, one control step, a push, the true state.

The World owns the loaded MjModel. It removes the free-joint damping and the floor's sliding friction (the robot
geom alone decides the grip), and every reset writes that episode's mass, friction and servo gains into it.

`World(twin=True)` runs the headless twin: the same body without the CAD mesh geoms (`*_vis`, group 2, which never
collide), for runs nobody watches. Its physics equals the full model's (check_e1_run.py row 6).
"""

import math

import numpy as np

import mujoco

from robot2sim import contacts, randomise
from robot2sim.kinematics import stand_pose
from robot2sim.paths import body_xml
from robot2sim.scene import load_model, load_spec
from robot2sim.servo import DEFAULT_SERVO, KP, KV, SUBSTEPS, Bus, apply_motor
from robot2sim.units import deg_to_rad, m_to_mm

SIT_DEG = (0.0, 56.6, 110.6)  # every leg in the sit pose (pose_sit)
PLACE_DISTMAX_M = 1.0  # mj_geomDistance search range when the robot is put on the floor
PENETRATION_MAX_MM = 0.5
NOISE_KEYS = ("encoder_sigma_rad", "speed_sigma_rad_s", "load_sigma_pct", "voltage_sigma_v")
VIS_SUFFIX = "_vis"
# The mesh geoms carry mass: compiled without them the inertials change, so the twin takes these from the full model.
TWIN_ARRAYS = ("body_mass", "body_inertia", "body_ipos", "body_iquat")
# Compile-time statistics; the solver scales its stopping test by 1 / (meaninertia · nv).
TWIN_STATS = ("meaninertia", "meanmass", "meansize")


def twin_model(body: str, full) -> mujoco.MjModel:
    """The body without its `*_vis` geoms. The four inertial arrays and the three model statistics are copied from
    the compiled full model, the statistics so that the solver's stopping test uses the same number; measured: the
    statistics survive mj_setConst and World.reset (check_e1_run.py row 6 compares them after a randomised reset).

    `dof_length` differs: mj_setConst recomputes it from the remaining geoms, here and at every World.reset. Row 6's
    exact traces on randomised episodes prove the step path does not read it.
    """
    spec = mujoco.MjSpec.from_file(str(body_xml(body)))
    for geom in [g for g in spec.geoms if g.name.endswith(VIS_SUFFIX)]:
        spec.delete(geom)
    twin = spec.compile()
    for name in TWIN_ARRAYS:
        getattr(twin, name)[:] = getattr(full, name)
    mujoco.mj_setConst(twin, mujoco.MjData(twin))
    for name in TWIN_STATS:
        setattr(twin.stat, name, getattr(full.stat, name))
    return twin


def _unit(x):
    """Clip a rotation-matrix entry to −1…1 against round-off before asin / acos."""
    return max(-1.0, min(1.0, float(x)))


class World:
    def __init__(self, body: str = "v0", servo: str = DEFAULT_SERVO, twin: bool = False):
        self.spec = load_spec(body)
        self.model = twin_model(body, load_model(body)) if twin else load_model(body)
        self.twin = twin
        self.data = mujoco.MjData(self.model)
        self.servo = servo
        m = self.model
        self.joint_names = [f"{h['name']}_{k}" for h in self.spec["hips"] for k in self.spec["joint_order"]]
        actuators = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(m.nu)]
        assert actuators == self.joint_names, f"actuator order {actuators}"
        self.n_servos = m.nu
        joints = m.actuator_trnid[:, 0]
        self._qadr = m.jnt_qposadr[joints].astype(int)
        self.lower_rad, self.upper_rad = m.jnt_range[joints].T.copy()
        root = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "root")
        self._root_q = int(m.jnt_qposadr[root])
        self._root_body = int(m.jnt_bodyid[root])
        root_dof = int(m.jnt_dofadr[root])
        m.dof_damping[root_dof:root_dof + 6] = 0.0
        floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        m.geom_friction[floor, 0] = 0.0
        self._floor = floor
        self._robot = contacts.robot_geoms(m)
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, int(g)) for g in self._robot]
        self._feet = self._robot[[n.endswith("_foot_tip") for n in names]]
        self._blades = self._robot[[not n.endswith("_foot_tip") for n in names]]
        self._robot_set = frozenset(self._robot.tolist())  # for contacts.scan, once per World
        self._foot_set = frozenset(self._feet.tolist())
        self._body_hex = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "body_hex")
        self._mass = m.body_mass.copy()
        self._inertia = m.body_inertia.copy()
        self._friction = m.geom_friction.copy()
        self._push_steps = 0
        self.bus = self.params = self.jitter = self.home_rad = None

    def _home_deg(self, pose, belly_m):
        c = self.spec["constants"]
        if pose == "stand":
            _body, joints = stand_pose(self.spec["hips"], {**c, "STAND_HB": m_to_mm(belly_m)})
        elif pose == "sit":
            joints = {h["name"]: SIT_DEG for h in self.spec["hips"]}
        else:
            raise ValueError(pose)
        return [a for h in self.spec["hips"] for a in joints[h["name"]]]

    def reset(self, seed: int, nominal: bool, pose: str = "stand", belly_m: float = 0.062) -> None:
        m, d = self.model, self.data
        self.params = p = randomise.params(seed, nominal)
        self.jitter = j = randomise.jitter(seed)
        m.body_mass[:] = self._mass * p["mass_scale"]
        m.body_inertia[:] = self._inertia * p["mass_scale"]
        mujoco.mj_setConst(m, d)
        m.geom_friction[:] = self._friction
        m.geom_friction[self._feet, 0] = p["foot_mu"]
        m.geom_friction[self._blades, 0] = p["blade_mu"]
        apply_motor(m, self.servo, KP * p["kp_scale"], KV * p["kv_scale"])

        self.home_rad = deg_to_rad(np.array(self._home_deg(pose, belly_m)))
        mujoco.mj_resetData(m, d)
        self._push_steps = 0
        d.qpos[self._qadr] = self.home_rad + j["joint_offsets_rad"]
        r = self._root_q
        d.qpos[r:r + 3] = [j["dx_m"], j["dy_m"], belly_m]
        d.qpos[r + 3:r + 7] = [math.cos(j["yaw_rad"] / 2.0), 0.0, 0.0, math.sin(j["yaw_rad"] / 2.0)]
        mujoco.mj_kinematics(m, d)
        gap = min(mujoco.mj_geomDistance(m, d, int(g), self._floor, PLACE_DISTMAX_M, None) for g in self._robot)
        d.qpos[r + 2] -= gap
        mujoco.mj_forward(m, d)
        depth = contacts.penetration_mm(m, d)
        hits = contacts.self_hits(m, d)
        if depth > PENETRATION_MAX_MM or hits:
            raise RuntimeError(f"seed {seed} {pose}: {depth:.3f} mm in the floor, self hits {hits}")
        noise = {k: p[k] for k in NOISE_KEYS}
        self.bus = Bus(m, d, self.servo, p["latency_steps"], j["joint_offsets_rad"], noise, randomise.rng(seed, "bus"))

    def step(self, targets_rad: np.ndarray) -> None:
        self.bus.command(targets_rad)
        mujoco.mj_step(self.model, self.data, nstep=SUBSTEPS)
        self.bus.after_step()
        if self._push_steps > 0:
            self._push_steps -= 1
            if self._push_steps == 0:
                self.data.xfrc_applied[self._root_body, :3] = 0.0

    def push(self, fx_n: float, fy_n: float, seconds: float) -> None:
        """A world-frame force on the root body for round(seconds / control period) control steps, at least 1;
        step counts them down and removes the force, so the float sum of data.time never adds a step."""
        control_s = float(self.model.opt.timestep) * SUBSTEPS
        self.data.xfrc_applied[self._root_body, :3] = [fx_n, fy_n, 0.0]
        self._push_steps = max(1, round(seconds / control_s))

    def truth(self) -> dict:
        """The true state after the last step; one scan of data.contact (contacts.scan) for floor and self hits."""
        d = self.data
        r = self._root_q
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, d.qpos[r + 3:r + 7])
        R = R.reshape(3, 3)
        touching, depth, hit = contacts.scan(d, self._robot_set, self._floor)
        return {
            "time_s": float(d.time),
            "pos_m": d.qpos[r:r + 3].copy(),
            "com_m": d.subtree_com[self._root_body].copy(),
            "yaw_rad": math.atan2(R[1, 0], R[0, 0]),
            "roll_rad": math.asin(_unit(R[2, 0])),
            "pitch_rad": math.asin(_unit(R[2, 1])),
            "tilt_rad": math.acos(_unit(R[2, 2])),
            "belly_m": float(d.qpos[r + 2]),
            "body_on_floor": self._body_hex in touching,
            "feet_down": len(self._foot_set.intersection(touching)),
            "self_hit": hit,
            "penetration_mm": depth,
            "cut": self.bus.cut.copy(),
            "qpos": d.qpos.copy(),
        }
