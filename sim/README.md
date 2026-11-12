# Takt simulation

## What this is

A MuJoCo simulation of Takt, a six-legged walking robot: a body file built from the CAD, a servo model, the sensors a brain sees, four tasks, a scripted gait as the rival to beat, and a room view for replays. Beside it sits Puls, a small learning model, with its own checks.

How far it is: the simulation and its checks are complete. Puls learns a rhythm that is fed to it and plays it on blind for five beats. Puls is not connected to the simulated robot yet.

## Requirements

Python 3.14 and the packages in `requirements.txt` (`numpy`, `mujoco`, `pillow`). `--mp4` also needs `ffmpeg` on PATH. Commands are run from the repository root, except where the text says from `sim/` or the block starts with `cd sim`.

```bash
python3.14 -m venv sim/.venv
sim/.venv/bin/pip install -r sim/requirements.txt
```

## Run the checks

From the repository root:

```bash
sim/.venv/bin/python sim/checks/check_e1_all.py      # stage 1: the nine checks one after the other, about 2 minutes
sim/.venv/bin/python sim/checks/check_e1_live.py     # by hand: opens the window for about 3 s
sim/.venv/bin/python sim/checks/check_e2_all.py      # stage 2: core, model and rhythm checks, about 5 minutes
```

One check alone, for example:

```bash
sim/.venv/bin/python sim/checks/check_s1_body.py
```

`check_e1_all.py` runs `check_s1_body.py` and `check_e1_{body,servo,world,sensors,tasks,episode,scripted,run}.py`, each as its own subprocess with the same interpreter, passes their output through, and ends with one line per script (`PASS` or `FAIL`, seconds). Exit 0 only when all nine exit 0. It works from any directory. Every run writes one gate run of the scripted rival into `runs/` (`check_e1_scripted.py` row 5).

`check_e2_all.py` runs `check_e2_core.py` (about 5 s), `check_e2_model.py` (about 7 s) and `check_e2_rhythm.py` (280–340 s) the same way. Run it with `-W error` to turn warnings into failures.

Each check prints one line per row. `row_` rows pass or fail; `info_` rows only report a number.

## Build, view and run

### A new body file

Regenerate the body and the MJCF after the CAD script or the CAD's `viewer/model.json` (`cad/viewer/` in the repository) changes, then run `check_e1_all.py`, then `check_e1_live.py`:

```bash
cd sim && .venv/bin/python -m robot2sim.build
```

### Live view

On macOS the viewer has to run under `mjpython` (installed with the `mujoco` package). From `sim/`:

```bash
cd sim
.venv/bin/mjpython -m robot2sim.view --body v0 --pose stand
```

`python -m robot2sim.view` re-execs `mjpython` on macOS. `--headless` loads the pose and exits. Checks are headless and do not open a window.

### Run a brain on a task

One command runs a brain on a task, live in the window (default) or headless, and replays a run folder. From `sim/`:

```
.venv/bin/python -m robot2sim.run --brain {hold,random,scripted} --task {stand_up,walk_1m,turn_90,push_walk}
    [--episodes N] [--seed S] [--nominal] [--belly-mm B] [--servo C018_12V] [--body v0] [--headless] [--seconds T]
    [--out DIR] [--brain-seed N]
.venv/bin/python -m robot2sim.run --replay RUN_DIR [--episode K] [--png FILE [--at 0.5] | --mp4 FILE]
```

```bash
cd sim
.venv/bin/python -m robot2sim.run --brain scripted --task walk_1m --nominal           # window, seed 1001
.venv/bin/python -m robot2sim.run --replay runs/<run folder> --episode 3 --png walk.png
```

The plain interpreter re-executes itself under `mjpython` for the window. Closing the window or `--seconds T` ends the episode as `aborted`; the run folder is still written. `--headless` runs the 20 evaluation seeds. `--brain-seed N` fixes the root of the brain seeds (see Episode loop), so a random brain repeats; without it every run draws a fresh root. A refused argument is one line on stderr and exit code 2, before any run folder exists: an unknown brain, task, servo or body, `--belly-mm` outside the body's belly range (30 … 95 mm: the lowest and highest of the CAD script's `NAMED_HEIGHTS`, read through `scripted.heights_mm()`), and `--out` on a folder that exists and is not empty. Without `--png`, `--replay` plays the episode once in the same window. `--mp4 FILE` writes the whole episode in real time (25 frames per second, every second 50 Hz row) by piping `replay.frames` as raw RGB into `ffmpeg` from PATH; no `ffmpeg` on PATH is one line and exit 2.

## Body and physics

- Package `robot2sim`. Body file `bodies/v0.json`, MJCF `bodies/v0.xml`, visual OBJs in `assets/meshes/` taken from the CAD's `viewer/meshes.json` and scaled to metres.
- One link per moving frame: body (s1 included), coxa (s2 included), femur, tibia (s3, knee, blade, foot). Joint angle is γ, θ2, β in radians. Femur axis is `(0, 0, -1)` so a positive angle is θ2.
- Mass is each `model.json` part, plus `LUMPS_BODY` on the body and `LEG_HARDWARE/3` at the coxa, femur and tibia origins. `numbers.mass.total` is not used. A mesh carries the mass when its volume centroid is within 0.05 mm of the part CoM; otherwise a 1 mm sphere sits on the authored CoM (lid grille vs `lid_proxy`, electronics with no mesh, lumps, screws).
- Collision does not hit other robot geoms, only the floor: capsules of radius `T_BLADE/2` (8 mm) along the links, a hex prism (vertex on +X, apothem `APO`, from the belly plate to `LID_TOP`), and a 4 mm sphere at each foot tip.
- Plain position actuators (`kp` 100, `kv` 2) in the body file; the servo model replaces them (see Servos). Free-base damping 1 so a frictionless tripod stops rocking inside the 2 s hold; the damping force is zero at rest.
- Foot sliding friction 0.9, the midpoint of the design range 0.6–1.2. The statics check sets every geom friction to 0.
- Gravity is −9.80665 m/s² so that 1 kg·cm = 0.0980665 N·m. Physics step 2 ms.
- Camera on the front face at `(0, APO, CAM_Z)`, looking +Y.
- The stand pose is `leg_ik` of `spots(STAND_R)` at `STAND_HB`, checked against the CAD's `viewer/reference.json`.
- Contacts: robot collision geoms are `contype 2 / conaffinity 3`, the floor `contype 1 / conaffinity 1`, so robot-vs-floor and robot-vs-robot both collide; MuJoCo still skips geoms of one body and of parent and child. Contact softness `solref="0.004 1"` on the collision default and on the floor (0.004 s is the stiffest time constant the 2 ms step allows); `solimp` stays MuJoCo's default. Stand: 0.034 mm in the floor, belly 61.92 mm. `robot2sim/contacts.py` answers what touches what.
- The electronics and the hardware lumps are point masses and are not drawn: 30 parts without a mesh (`e_battery`, `e_bus_board`, `e_regulator`, `e_pi_zero`, `e_oled`, `e_speaker`, `e_amp`, `e_mic`, `e_switch`, `e_reg_5v`, `wiring`, `body_hardware`, 18 × `{leg}_hw_{coxa,femur,tibia}`). The other 74 parts are drawn as CAD meshes (`check_e1_body.py` row 1 counts them).
- Front and rear leg pairs can touch from about ±50° hip turn, and a folded leg (femur ≤ −10°, knee ≥ 145°) hits its own hip and the body: findings for the hardware plan, the simulation reports them as `self_hit`.
- Leg joint rotor inertia: every coxa, femur and knee joint in the body file carries `armature 0.005 kg·m²` (`mjcf.ARMATURE`, `# UNVERIFIED`), the geared servo's reflected rotor inertia; the servo model uses the same constant. With zero rotor inertia the frictionless tripods of `check_s1_body.py` row 4 chatter on the `solref 0.004` contact (force error up to 100 %); with it all three frames settle (force error 0.00 %, worst torque error 0.021 kg·cm; 0.001…0.01 all pass). No mass and no force at rest.

Mass: the parts of the CAD's `viewer/model.json` sum to about 1903.2 g, so parts + 70 g + 13 g × 6 is about 2051.2 g, which is the mass `reference.json` records. An earlier design figure was 1944.6 g for the parts and 2092.6 g for the whole robot; the blade and the foot are lighter now. The sim follows the files. The mass bar is `reference.json` ±0.5 %.

### The World

`robot2sim/world.py`: the World owns friction and removes the free-joint damping. At load it sets the six root dofs' damping to 0 and the floor's sliding friction to 0, so MuJoCo's max rule leaves the robot geom alone to decide the grip. Each reset restores the loaded mass, inertia and friction, scales and writes that seed's values, calls `mj_setConst` (MuJoCo's mass-derived constants feed the contact softness and are otherwise kept from compile), then `apply_motor` and a new `Bus` (stream `"bus"`). The stand uses `kinematics.stand_pose` at the requested belly height; the sit pose is `SIT_DEG` (0°, 56.6°, 110.6°) on every leg. `push` holds the force for `round(seconds / control period)` control steps (at least 1; 10 for `PUSH_S` 0.2 s), counted down by `step` and cleared by `reset`; the count is in whole steps, because comparing the float sum `data.time` with the end time pushed 11 steps on 11 of the 20 evaluation seeds (`check_e1_world.py` row 10 `push_steps` checks 10 on every one). The root is lowered with `mj_geomDistance` until the lowest robot geom just touches the floor; a reset with more than 0.5 mm in the floor or any self hit raises `RuntimeError`. `truth()` reads position and rotation from `qpos`, the rest from the last physics step.

Speed of the World alone (`check_e1_world.py` row 8, holding the stand, one process): 133–135× real time, 149–151 µs per control step.

### Episode ranges

`robot2sim/randomise.py`, drawn uniformly per seed from stream `"params"`, all `# UNVERIFIED`; `nominal` gives the middle of each range and latency 2:

| Key | Range | Applied as |
|---|---|---|
| `kp_scale` | 0.7 … 1.3 | `apply_motor` gain `KP × kp_scale` |
| `kv_scale` | 0.7 … 1.3 | `apply_motor` `KV × kv_scale` |
| `latency_steps` | 1 … 3 (whole control steps) | bus queue length |
| `encoder_sigma_rad` | 0.0005 … 0.002 | bus noise |
| `speed_sigma_rad_s` | 0.01 … 0.05 | bus noise |
| `load_sigma_pct` | 0.5 … 2.0 | bus noise |
| `voltage_sigma_v` | 0.01 … 0.05 | bus noise |
| `foot_mu` | 0.6 … 1.2 | sliding friction of the six `_foot_tip` geoms |
| `blade_mu` | 0.3 … 0.5 | sliding friction of every other robot collision geom (capsules, `body_hex`) |
| `mass_scale` | 0.95 … 1.05 | every body's mass and inertia, then `mj_setConst` |

Episode jitter, always on (stream `"jitter"`): yaw ±5°, x and y ±10 mm, 18 joint offsets ±1°.

## Servos

`robot2sim/servo.py`. Table, all `# UNVERIFIED` until a real servo is on the desk:

| Name | Stall | No load | Volts | Source |
|---|---|---|---|---|
| `C018_12V` (default) | 30 kg·cm = 2.941995 N·m | 45 rpm = 4.712389 rad/s | 12 | the planned servo; 45 rpm `# UNVERIFIED` |
| `C001_6V` | 16.5 kg·cm = 1.618097 N·m | 42 rpm = 4.398230 rad/s | 6 | stall: the CAD script's `STALL[6.0]`; 42 rpm `# UNVERIFIED` |

- Motor line: the position actuator is clipped to ± stall and the joint carries damping stall / no-load speed (0.624311 and 0.367897 N·m·s/rad), so at full drive the torque falls linearly to zero at the no-load speed. Gains `KP` 80 N·m/rad, `KV` 2 N·m·s/rad (`# UNVERIFIED`, chosen values; `check_e1_servo.py` rows 2 and 4 pass with them unchanged: free-spin peak 0.99999 × no load, half-stall load 1.0000 × half no-load speed). Rotor inertia is `mjcf.ARMATURE` (0.005 kg·m²), compiled into the body file; `apply_motor` checks it and never writes it, because MuJoCo derives the contact softness from it at compile.
- Servo bus: command latency 1–3 control steps (a queue that starts at the reset pose), mounting offsets (true joint = command + offset, encoder reads true − offset), encoder rounded to 2π/4096. `load` is the drive level, actuator force / stall in −1…1, what the STS3215 load register reports, not the motor current (`# UNVERIFIED`). Overload: above 0.8 × stall for 2 s without a break (counted per 50 Hz control step) → limp until the next `apply_motor` + `Bus`: gains 0, joint damping `LIMP_DAMPING` 0.05 N·m·s/rad (`# UNVERIFIED`, back-driven gears). `Bus` refuses to start while a servo is still limp. No voltage sag, no temperature.

## Sensors

`robot2sim/sensors.py`: the only view a brain gets of the robot, 10 channels, 107 numbers, each a fresh `float32` array per read. The eye and IMU values are `# UNVERIFIED`, to be measured on the real camera and the real BNO085:

| Channel | Size | Meaning |
|---|---|---|
| `servo_pos`, `servo_speed`, `servo_load`, `servo_voltage` | 18 each | `world.bus.read()`: encoder (true − mounting offset, 4096 steps), speed, drive level, volts, with the bus's own noise |
| `imu_gravity` | 3 | unit gravity vector in the body frame, (0, 0, −1) level; noise σ 0.02 per component, then normalised |
| `imu_gyro` | 3 | body-frame angular velocity rad/s (`qvel[3:6]` of the free joint); noise σ 0.01 rad/s |
| `eye` | 4 | outside camera: body origin x, y (m), yaw (rad, wrapped), height (m); 30 Hz, visible 0.10 s later; σ 3 mm, 3 mm, 0.02 rad |
| `goal` | 3 | the target seen from the robot, from the `eye` value, never from truth: dx right, dy front (m), dyaw counter-clockwise (rad, wrapped); zeros for no goal |
| `wish` | 4 | forward m/s, sideways m/s, turn rad/s, belly height m, passed through |
| `last_command` | 18 | the brain's last action, passed through |

Eye timing in whole physics ticks, `round(data.time / timestep)`, so the floating-point dust in `data.time` never moves a sample: the instant index at tick T is `T × 3 // 50` (30 Hz × 2 ms as an exact fraction); a read whose index is above the last sampled one takes a sample (one sample even if several instants passed), and a sample taken at tick T becomes visible at the first read at or after tick T + 50 (0.10 s). `reset` takes one noisy measurement of the start pose and shows it at once, so the start pose is seen without delay; the next sample is the 1/30 s one. Measured (`check_e1_sensors.py` row 2, reads every 20 ms): 90 changes in 3 s, the eye 0.100…0.120 s old. Noise: stream `"sensors"`, recreated at each `reset`, one draw of 10 standard normals per read (4 eye, 3 gravity, 3 gyro) whether or not an eye sample is taken, so the stream does not depend on timing (`NOISE_DRAW`). The IMU has no delay and no bias (`# UNVERIFIED`).

## Tasks

`robot2sim/tasks.py`, pure, no simulation; `check_e1_tasks.py`. Ledge tasks are not built.

| Task | Limit | Start pose | Goal | Wish (fwd m/s, side m/s, turn rad/s, belly m) | Success |
|---|---|---|---|---|---|
| `stand_up` | 30 s | sit | none | (0, 0, 0, 0.062) | belly ≥ 55 mm and \|roll\|, \|pitch\| < 10° for 2.0 s without a break |
| `walk_1m` | 60 s | stand | point: body origin + 1.0 m along the start heading | (0.05, 0, 0, belly) | CoM xy within 0.10 m of the goal |
| `turn_90` | 30 s | stand | yaw: start yaw + 90° (counter-clockwise), wrapped | (0, 0, 0.279, belly) | \|wrap(goal − yaw)\| < 10° and \|roll\|, \|pitch\| < 10° for 2.0 s without a break |
| `push_walk` | 60 s | stand | as `walk_1m` | (0.05, 0, 0, belly) | as `walk_1m` |

`judge`, first match wins: any servo cut out → `cutout`; tilt > 45° → `fall`; a walking task with the body on the floor → `fall` (`stand_up` starts sitting, so not there); success; `time_s` ≥ limit → `timeout`. Choices: the point goal starts from `truth0["pos_m"]` (the body origin), the judge measures `com_m`; the hold counts 0.02 s per call (`CONTROL_DT_S`) and succeeds at `held_s ≥ 2.0 − HOLD_TOL_S` with `HOLD_TOL_S` = 0.01 s, half a step, so the 100th call succeeds despite float sums; "within 0.10 m" is `≤`. `push_walk` push from stream `"task"`: `at_s` uniform in [2, 15] s, then the sign of the 3 N along world X from one fair integer draw; 0.2 s, no Y force. Belly default 62 mm (`STAND_BELLY_M`).

`turn_90` needs the 2.0 s upright hold, not a one-step yaw window: with the one-step window the random brain crossed the window while tumbling (1 of 20 eval seeds, 86 of 1,000 seeds); with the hold 0 of 1,220. The hold uses the `stand_up` mechanism (`HOLD_S`, `HOLD_TOL_S`, `CONTROL_DT_S`, `UPRIGHT_TILT_RAD`, `YAW_TOL_RAD`), no belly condition.

## Episode loop

`robot2sim/episode.py`, `robot2sim/brains.py`; `check_e1_episode.py`. The only code that sees truth; a brain gets the sensor dict and a `BodySpec` (number of servos, lower and upper limits). Before the loop: `tasks.start` → `World.reset` → `truth0` → `tasks.goal`, `tasks.wish` → `Sensors(world, seed).reset(goal, wish)` → first obs with `last_command = world.home_rad` → `brain.reset(body, task, brain_seed)`. One control step, in this order: (1) `act` — not 18 finite numbers → `bad_action`, else clipped to the joint limits; (2) the push of `push_walk` starts at the first step whose start time is ≥ `at_s`, once; (3) `world.step`; (4) any of MuJoCo's `BADQPOS`, `BADQVEL`, `BADQACC` warning counters above 0 or a non-finite `qpos` → `sim_error` (MuJoCo resets the data itself on a bad `qpos`, so the counters, not the finite check, catch it; `World.reset` → `mj_resetData` clears them, measured); (5) one `truth()`; (6) `judge`; (7) next obs, `brain.observe`; (8) `on_step(world)`, `False` → `aborted` (a step the judge already ended keeps the judge's reason). Brain exceptions are not caught.

- Reasons: `success`, `fall`, `cutout`, `timeout` (from `judge`), `bad_action`, `sim_error`, `aborted`.
- Metric keys (one `metrics.jsonl` line per episode, in this order): `episode` (0-based in the run), `seed`, `task`, `brain` (the brain class name, lower case), `body`, `servo`, `nominal`, `success`, `reason`, `steps` (control steps taken), `robot_minutes` (steps / 3,000), `time_s` (steps × 0.02 s), `wall_s` (the whole episode incl. reset), `brain_s` (inside `act` and `observe`), `belly_min_mm`, `belly_mean_mm`, `belly_max_mm` (over the start truth and every step), `cutouts` (servos cut at the end), `self_hit_steps`, `airborne_steps` (`feet_down == 0`), `max_penetration_mm` (start and every step), `ik_failures` (`brain.info()`, else 0).
- Run folder (`runs/<YYYYmmdd-HHMMSS>_<brain>_<body>_<task>/` or `out_dir`). A run folder is never overwritten: when the default name exists, the run takes `<name>-2`, `<name>-3`, …; an `out_dir` that exists and is not empty is refused (`ValueError` before anything is written; the command: exit 2); an existing empty folder is used. `config.json` (`arguments` incl. `brain_seed`, `cli` = the command's `--seconds` and `--headless` as given, `null` when `evaluate` is called from code, `model`, `ranges` = `randomise.RANGES`, `eye`, `imu`, `servo` = name + table row), `metrics.jsonl` (written and flushed per episode), `motion.npz` (`qpos` N × 25 float32, all episodes one after another, each with steps + 1 rows starting at the reset pose; `episode` N; `time_s` N, from 0 in each episode; `body`; `rate_hz` 50.0), `summary.json` (`brain, task, body, servo, nominal, episodes, successes, reasons` with every reason and its count, `robot_minutes`, `wall_s` = sum of the episodes' `wall_s`, `speed_x` = robot time / (wall − brain time), `run_dir`, `eval_seeds` = the seeds are exactly the 20 evaluation seeds in order). `evaluate` stops after an `aborted` episode.
- A `sim_error` step's trace row is the raw `qpos` after the failed step (MuJoCo's own reset pose after a bad `qpos`, or non-finite values); it is not dropped.
- MuJoCo's warning log: importing `episode.py` installs `mujoco.set_mju_user_warning`, which writes the text to stderr as `WARNING: MuJoCo: …` instead of MuJoCo's default (stdout line plus `MUJOCO_LOG.TXT` in the working directory). Verified: a NaN `qpos` run leaves no `MUJOCO_LOG.TXT` (row 6), the counter still counts.
- Brain seed: `seed` in `Brain.reset` is a brain seed that has no relation to the episode seed (with the episode seed a brain could recompute the hidden joint offsets, the parameter draw, the noise streams and the push). `run_episode(..., brain_seed=None)`: `None` → `secrets.randbits(32)`. `evaluate(..., brain_seed=None)`: the root is `secrets.randbits(32)` when `None`; episode `k` (0-based position in the seeds) gets `SeedSequence([root, k]).generate_state(1)[0]`; the root used is in `config.json` as `arguments.brain_seed`. The checks pass `brain_seed=0` wherever a random brain must repeat.
- `Random(hold_steps=10)`: a new uniform target in the limits every 10 steps from stream `"brain"` of its brain seed; `Hold` keeps its first `servo_pos`.

## Scripted gait

`robot2sim/scripted.py`, `brains.make("scripted")`; `check_e1_scripted.py`. Not a scored model: it uses IK and link lengths, and reads `GAIT_R`, `GAIT_LIFT`, `GAIT_OUT`, `TRI_A`, `TRI_B`, `NAMED_HEIGHTS` from the CAD script through `source`, so a changed gait there changes the rival (row 1 compares against `reference.json`). Open loop: at its first `act` it reads `wish` once (turn rate ≠ 0 → turn, forward > 0 → walk, else it holds its first `servo_pos`), then only its own step count. Walk and turn run **only at belly 30, 62 and 95 mm** (`NAMED_HEIGHTS`, ±0.5 mm); any other height → `ValueError`. Time line: stand pose, tripod A to its phase-0 gait spot in 0.6 s (cosine blend, lifted 15 mm × sin²), tripod B the same, then the gait (`PERIOD_S` 2.0 s, stride 50 mm, turn 16°) forever; it never stops, so `turn_90` (2 s hold in the window) is out of its reach. Choices: the body frame for IK is `T(0, 0, hb)` of the matched named height, not the wish's float32 value (so the first action equals `World.home_rad` exactly at the named heights); a negative turn rate is the mirror image x → −x of the counter-clockwise turn (RF↔LF, RM↔LM, RR↔LR), so tripod A's legs run tripod B's phase; the height check runs only for walk and turn (the hold needs no height); `CONTROL_DT_S` 0.02 s and `HEIGHT_TOL_MM` 0.5 are chosen values; an IK failure on a leg that never had a valid solution raises (cannot happen: the first command is the stand, which solves at every height).

- Gate (row 5, `walk_1m`, nominal, 62 mm): **20 of 20**, self hits 0, IK failures 0, `eval_seeds` true; time to goal mean 19.72 s, worst 19.90 s; sideways error at the goal mean 16.3 mm, worst 36.6 mm; airborne 0.00 % of steps; largest penetration 0.215 mm; stance-foot slip mean 0.67 mm over 1,448 stances (a stance = consecutive steps with the foot tip in `on_floor`, the World's `feet_down` decision, no force threshold); floor force 7.6…62.6 N. `speed_x` 52× with row 7's per-step collector, 64× without it; 75–81× in later gate runs, depending on the machine's load.
- Other runs (row 8, successes of 20): `walk_1m` randomised 20; `walk_1m` nominal at 30 mm 20, at 95 mm 20; `turn_90` nominal 0 (timeout 20: it turns through the 20° window without stopping); `push_walk` nominal 20 (with the 10-step push: time to goal mean 19.72 s, worst 19.90 s). Row 6: `turn_90` seed 1001, yaw +41.9° after 4 s.

## Command, window and replay

`robot2sim/run.py`, `robot2sim/replay.py`; `check_e1_run.py`, `check_e1_live.py`.

- Seeds: without `--seed` the seeds are the first N evaluation seeds (N = 1 with the window, 20 headless); with `--seed S` they are S … S + N − 1, N defaulting the same way. `--seconds T` applies per episode in both modes (the episode is `aborted` at T, so the run ends there). In both modes (window and `--headless`), `--belly-mm` for `scripted` must be within `scripted.HEIGHT_TOL_MM` (0.5 mm) of 30, 62 or 95 mm, for every task; for every brain it must be inside the belly range 30 … 95 mm.
- Real time is paced from the first control step: step k is shown no earlier than k × 0.02 s after step 1's wall time; the command prints `real time: robot … s, wall … s` per episode. The live run's window uses MuJoCo's tracking camera on the root body, distance 1.0 m, elevation −25°, azimuth 135° (`replay.CAMERA_*`, `# UNVERIFIED`, a look, not a measurement).
- The room: the replay (window, PNG, MP4) draws the robot in a room (`robot2sim/stage.py`): a display-only model compiled from the body file with an MjSpec (the physics model `World` and `scene.load_model` build is unchanged; `check_e1_run.py` rows 7–9): a tiled floor slab of 8 × 5 tiles of 1/3 m centred between (0, 0) and (0, 1), three panel walls, spot lights on the back wall, a board (top line, for every task: the whole seconds left in the attempt, ceil(the task's `limit_s` − elapsed), never below 0, two slots, so it starts at the limit, reads 00 when the time is up and stops where it is when the attempt ends early; bottom line: the attempts already had = the episode index, so the first attempt shows 00, four slots, at least two digits lit, the slots left of them dark; a number that does not fit raises `ValueError`, nothing is clipped), a green goal tile at the run's goal (`tasks.goal` of the recorded start pose) and an amber striped start tile; printed parts blue, servos near black, the physics floor plane hidden (group 5); one fixed camera from +x (`stage.CAMERA_*`).
- `png --at` is the share of the episode's finite rows, rounded to the nearest row; `--at` outside 0 … 1 is exit 2. `show` plays the episode once and closes. `replay.load` returns `qpos` as stored (float32); `frames`, `png` and `show` skip rows whose `qpos` is not finite (a `sim_error` episode can end on one).
- `view._ensure_mjpython` takes the module name and looks for `mjpython` next to `sys.executable` without resolving the symlink: the resolved path can be a Python installation that has no `mjpython`.

## Speed

- Per-step work out of the step path, byte-identical: `World.truth()` scans `data.contact` once (`contacts.scan`, plain Python over the handful of contacts, robot and foot geom ids as sets built once per World) instead of three scans with `numpy.isin`; `Sensors` works out the eye rate fraction and delay ticks once per `reset` instead of on every read. Measured before → after: `truth()` 63 → 5 µs, `Sensors.read` 20 → 18 µs, row 1 on the full model 65–68× → 82–89×. Byte-identical: reference runs before and after (scripted `walk_1m` 1001–1003, random `push_walk` 1001–1003, hold `stand_up` 1001–1002, randomised) have equal `motion.npz` arrays and equal `metrics.jsonl` lines except `wall_s`, `brain_s`; per step over 11,919 steps (195 with a self hit, 1,587 with the body on the floor) the new truth fields equal the old `contacts` functions in value and type.
- Headless twin: `World(twin=True)` loads the body without its 74 `_vis` geoms (`world.twin_model`). The mesh geoms carry mass, so `body_mass`, `body_inertia`, `body_ipos`, `body_iquat` are copied from the compiled full model, and so are `stat.meaninertia`, `meanmass`, `meansize`, so that the solver's stopping test (scaled by 1 / (meaninertia · nv)) uses the same number; measured: the statistics survive `mj_setConst` and `World.reset`, and row 6 compares them after a randomised reset. `dof_length` differs between twin and full model (`mj_setConst` recomputes it from the remaining geoms, at build and at every `World.reset`); row 6 `twin_equal` (identical trace and metrics on randomised scripted `walk_1m` and random `push_walk`, worst difference 0.0; also the float64 `data.qpos` and `data.qvel` after the last step, difference 0.0) proves this step path does not read it. `World()` stays the full model; `evaluate` and `run --headless` take the twin, a run with the window the full model; `config.json` records `"model"`; replay loads the full model.
- More byte-identical work: `Bus` keeps the overload threshold and encoder grid per bus, `after_step` returns early when no servo is overloaded long enough, the encoder uses `np.rint` (what `np.round` calls); `Sensors.read` uses a module `DOWN` vector and `np.linalg.norm`'s own `sqrt(x·x)`; `episode._physics_broken` reads the three warning counters as one array.
- `check_e1_run.py` row 1 (`speed_100x`, bar 100×, best of 3, scripted `walk_1m` seed 1001, robot time / (wall − brain time), on the twin): the row passes at about 107–118× real time on a quiet machine (physics only about 135–151×) and can miss under load (for example 93.5× while another job ran). Every measured run on the full model, without the twin, missed the bar.

## Constants

Other named constants of stage 1 not given above: `world.PLACE_DISTMAX_M` 1.0 m (the `mj_geomDistance` search range when the robot is put on the floor), `world.PENETRATION_MAX_MM` 0.5, `sensors.RATE_DENOMINATOR_MAX` 1000 (the eye rate × time step as an exact fraction, 3/50), `contacts.ROBOT_GROUP` 3, `servo.SUBSTEPS` 10, `servo.OVERLOAD_FRACTION` 0.8, `servo.OVERLOAD_S` 2.0, `servo.ENCODER_STEPS` 4096, `randomise.NOMINAL_LATENCY` 2, `randomise.JITTER` (yaw 5°, 10 mm, joints 1°), `randomise.STREAMS` (`params` 0, `jitter` 1, `bus` 2, `sensors` 3, `task` 4, `brain` 5), `tasks.FALL_TILT_RAD` 45°, `tasks.UPRIGHT_BELLY_M` 0.055, `tasks.UPRIGHT_TILT_RAD` 10°, `tasks.YAW_TOL_RAD` 10°, `tasks.POINT_TOL_M` 0.10, `tasks.PUSH_N` 3.0, `tasks.PUSH_S` 0.2, `tasks.PUSH_AT_S` (2, 15), `tasks.WALK_MPS` 0.05, `tasks.TURN_RAD_S` 0.279, `episode.ROBOT_MINUTE_STEPS` 3000, `episode.RATE_HZ` 50.0, `run.WINDOW_EPISODES` 1, `run.EXIT_REFUSED` 2, `replay.CAD_GROUP` 2.

Every `# UNVERIFIED` constant in `robot2sim/`: these are choices, not measurements.

| File | Constant | Value |
|---|---|---|
| `servo.py` | `SERVOS["C018_12V"]` | 30 kg·cm, 45 rpm, 12 V |
| `servo.py` | `SERVOS["C001_6V"]` | 16.5 kg·cm, 42 rpm, 6 V |
| `servo.py` | `KP`, `KV` | 80 N·m/rad, 2 N·m·s/rad |
| `servo.py` | `LIMP_DAMPING` | 0.05 N·m·s/rad |
| `mjcf.py` | `ARMATURE` | 0.005 kg·m² |
| `mjcf.py` | `KP`, `KV` | 100, 2 (the body file's step-1 actuators; `servo.apply_motor` replaces them; the episode range is `randomise.RANGES`, `kp_scale` × `servo.KP` = 56…104) |
| `mjcf.py` | `MU`, `MU_TORSION`, `MU_ROLL` | 0.9, 0.005, 0.0001 (the World sets the floor to 0 and the robot geoms per seed) |
| `mjcf.py` | `FREE_DAMPING` | 1.0 (the World sets it to 0) |
| `randomise.py` | `RANGES` | the ten ranges in the table above |
| `sensors.py` | `EYE` | 30 Hz, 0.10 s delay, σ 3 mm, 3 mm, 0.02 rad |
| `sensors.py` | `IMU` | σ gravity 0.02, σ gyro 0.01 rad/s |
| `replay.py` | `CAMERA_DISTANCE_M`, `CAMERA_ELEVATION_DEG`, `CAMERA_AZIMUTH_DEG` | 1.0 m, −25°, 135° (live run only) |
| `stage.py` | every look constant of the room (tile, room, walls, board, colours, lights, camera) | see the file |
| `spec.py` | `POINT_MASS_R_MM` | 1.0 |
| `collision.py` | `FLOOR_CLEARANCE_MM` | 2.0 |

## Puls

- `puls` is Puls: a small model with learned time in two speeds, a brake on the surprise and commands as channels, around a settle core. It imports only NumPy and the standard library, so the same files run in the simulation and on the Pi. float64 throughout.

### The settle core

- Energy per row (`puls/core.py`): `E(z) = ½‖z − p‖² + ½ Σ D_i (x_i − (C z)_i)² + λ‖w‖₁`, `w = z` for the brake `"state"`, `w = z − p` for `"surprise"`. `D` holds weights ≥ 0; a 0/1 mask is the special case. Step `1/L`, `L = 1 + max(D)·‖C‖²`.
- `minimum_gap(z, …)` is the 2-norm of the smallest subgradient of `E` at `z`. `E` is 1-strongly convex, so `‖z − z*‖₂ ≤ minimum_gap`: a gap below 1e-6 means `z` is within 1e-6 of the one minimum. That is bar 1: the settle finds the one minimum of its energy (rest below 1e-6) (`check_e2_core` rows 1 and 5); it does not reuse the settle's own stopping test.
- Refused with `ValueError` naming the argument: `lam`, `tol` that are NaN, inf or not a number; `max_iters` that is not a Python/NumPy integer ≥ 1 (a float such as 3.0 is refused, so a fractional count never slips through).
- Default bound `"spectral"` (the exact 2-norm of `C`, `np.linalg.norm(C, 2)`); `"frobenius"` (the Frobenius norm, up to rank(C) times looser) is an option.
- `check_e2_core` constants: `TIGHT` = tol 1e-10, 50,000 iterations (chosen); one seed per row, 1…9 (`SEED_*`); row 7 `BATCH_CASES` = 20 cases × both brakes (`# UNVERIFIED`, chosen). Measured: most iterations 1,256 (state) and 1,344 (surprise), worst gap 1.3e-8 and 3.2e-8.

### The stream model

- `puls/model.py`, `BodyModel`: per step `p = A z_prev` (zeros without time), `z = settle(z_init = p, p, x, d, C, λ, brake)`, readout `C z` for every channel; `C = C_slow + C_fast`, `A = A_slow + A_fast`. `d` is a weight ≥ 0 per channel; `d = 0` hides a channel, its `x` is ignored (NaN allowed) and the readout is the model's fill-in. Commands are channels like sensors.
- Local rules, one line each: `C`: `err = d·(x − C z)`, input `z`; `A`: `err = z − p`, input `z_prev`. Slow part `+= eta_slow·outer(err, input)`; fast part `= (1 − fast_decay)·fast + eta_fast·outer(err, input)`. `A` learns only with time on; `learn=False` changes and decays nothing.
- `Config` defaults: `n_latent` 8, `lam` 0.02, `brake` `"surprise"`, `time` on, `eta_slow` 0.01, `eta_fast` **0.01** (`# UNVERIFIED` on a robot; the first value, 0.05, missed bar 2 only: rows 1 and 2, one period 1.08 % off and blind 3 of 5; bar 3 passed; see below), `fast_decay` 0.01 (`# UNVERIFIED`), `init_scale` 0.1, `a_init` 0.5, `tol` 1e-7, `max_iters` 500. `PLAIN_CORE` = time off, brake `"state"`, `eta_fast` 0 (the plain core: no time, one speed, the brake on the state).
- `beat()`: eigenvalues of `A` with `|imag| > BEAT_MIN_IMAG` (1e-6) and modulus ≥ `BEAT_MIN_MODULUS` (0.9, `# UNVERIFIED`); the one with the smallest `|angle|` gives `(2π/|angle|, modulus)`, else `None`. It is never given the true period.
- Refused: bad `Config` at construction, each with a `ValueError` naming the field (sizes or `max_iters` below 1; `lam`, `eta_slow`, `eta_fast` not a finite number ≥ 0; `tol` not a finite number > 0; `init_scale`, `a_init` not finite numbers; `time` not a real `bool`; `fast_decay` outside [0, 1]; unknown brake; numbers by the core's own rule, so a string or a 0-d NumPy array is refused like the core refuses it; `check_e2_model` row 1, 24 cases) and bad `step` input (wrong length, `d` negative or not finite, non-finite `x` at a visible channel) with `ValueError`; a non-finite result with `FloatingPointError`. Both leave the model as before the call; NumPy warnings are silenced under `np.errstate` and the result tested with `np.isfinite`.
- Scaling the channels to about ±1 is the caller's job (the adapter of stage 3). Unscaled input (×1,000) stays finite in `check_e2_model` row 8, but there almost every step is unsettled (1,999 of 2,000): the model runs, its settle does not finish.

### The rhythm test

- The one setting that was tuned (five seeds 0–4, `check_e2_rhythm` constants): `eta_fast` 0.05 (first value): periods 39.57 40.13 40.17 40.11 40.19, R² last 0.873 0.918 0.970 0.948 0.847, fifth beat 0.896 0.931 0.974 0.955 0.869 → fails; 0.01: passes (table below). Seed 0 only: 0.005 → 39.94, 0.982/0.985; `fast_decay` 0.1 → 39.55, 0.847/0.873; `lam` 0.005 → 39.49, 0.833/0.861.
- Further `check_e2_rhythm` choices: row tags `TAG_*` 2–9 for each row's own noise; `HORIZONS` 10/40/200; `NEW_PERIOD` 30, looked at every 100 steps up to 20,000; `IRREGULAR_RANGE` 36–44 per cycle; lock-on from `z = 3·randn` over 400 steps from step 25,000, steady = mean of the last 100, held 20 steps, floor `LOCK_FLOOR` 1e-4 (chosen); step time over 2,000 learn steps; long blind `LONG_BLIND_STEPS` 1,500 from the first of `STARTS` with the blind test's own warm-up (tag `TAG_BLIND`).

## Measured results

### Stage 1

The success bars of stage 1, each with the rows that prove it and their values in a stage gate run:

| Bar | Proof | Measured |
|---|---|---|
| Body equals FreeCAD in 324 poses, mass ± 0.5 % | `check_s1_body` rows 1–5 (`actuators, ranges`, `FK vs FreeCAD`, `mass and CoM`, `statics`, `stand belly`); `check_e1_body` row 2 `cad_poses_clear` | 18 actuators; 324 poses, pos 0.0000 mm, rot 0.0000°; mass 2051.230 g = `reference.json` (0.0000 %), CoM 0.480 mm; 3 frictionless frames, force error 0.00 %, torque worst 0.021 kg·cm; belly 61.94 mm; 324 poses no `self_hit` |
| At least 100× real time on one core | `check_e1_run` row 1 `speed_100x` | 102.6×, physics only 140.1×, under load (see Speed) |
| Doing nothing and random: 0 of 20 on every task | `check_e1_episode` row 4 `baselines_zero` | hold: timeout 20 on all four tasks; random (brain seed root 0): `stand_up` fall 6 + timeout 14, the others fall 20; 0 successes |
| The fixed gait: 18 of 20 on `walk_1m` | `check_e1_scripted` row 5 `gate_walk_1m` | 20 of 20, self hits 0, IK failures 0 |
| Two legs driven against each other report `self_hit` | `check_e1_body` row 3 `legs_collide` (the pose); `check_e1_world` row 9 `legs_driven_together` (driven by the servos) | 4 RF/LF pairs both, none either alone; first hit 0.26 s, held to the end, deepest 0.69 mm |
| A scored model gets only channels of the sensor list | `check_e1_sensors` row 1 `sensor_list`; `check_e1_episode` row 1 `brain_sees_sensors_only` (every obs holds exactly the 10 channels; the brain's `reset` seed is never the episode seed, its jitter differs from the episode's on all 20 evaluation seeds, a rerun gets a new seed; `brain_seed` 0 twice repeats the random brain's metrics and is recorded in `config.json`) | 10 channels, 107 float32 numbers, copies; 50 steps, 10 keys every obs; 20 seeds, brain seed ≠ episode seed |
| Every visible body part carries its CAD mesh | `check_e1_body` row 1 `cad_meshes`; `check_e1_run` row 3 `png_cad_only` | 74 of 74 parts with a mesh; PNG geomgroup [0, 0, 1, 0, 0, 0], 2901 colours |
| One command from the terminal, live in the window | `check_e1_live` row 1 `plain_python_opens_window`, row 2 `real_time` | plain python exit 0, window open, groups [2]; robot 3.00 s in 3.00 s wall |
| The motion file, replay without re-computing | `check_e1_episode` row 3 `files`; `check_e1_run` row 2 `replay_no_physics` | 4 files, 3 × 22 metric keys, 9003 motion rows; 1501 rows equal the file, 751 frames with `mj_step` raising |

### Stage 2

The success bars of this stage, measured with `-W error`. The rows are deterministic: every gate run prints the same numbers.

| Bar | Proof | Measured |
|---|---|---|
| The settle core alone (no time, no learning), 1,000 random cases: the settle finds the one minimum of its energy (rest below 1e-6) | `check_e2_core` rows 1, 5 | worst gap 1.3e-8 (state), 3.2e-8 (surprise); the guard's `distance_bound` = L √d r in every case, largest 2.4e-8 / 5.8e-8 |
| With time: beat within 1 % and 5 beats blind with R² ≥ 0.95 in 5 of 5 seeds | `check_e2_rhythm` rows 1, 2 (row 4: time, the brake on the surprise and both learning speeds on, 0 unsettled) | periods 39.91 40.02 40.05 40.06 40.11; R² last step 0.979 0.974 0.980 0.969 0.964, whole 5th beat 0.983 0.978 0.983 0.974 0.969 (pooled over the 18 channels); reported, not gated: worst single channel over the whole 5th beat 0.978 0.972 0.982 0.971 0.962 |
| The core without time fails the same test (R² below 0.1) | `check_e2_rhythm` row 3 | R² −0.000 / +0.000 in all five seeds |

Further gate rows: `beat_none_without_rhythm` (constant input: `None` 5 of 5, largest complex modulus 0.50); `fill_in` (our own bar 0.9, not one of the stage's success bars): commands hidden 0.998–0.999, everything hidden 0.961–0.987. With everything hidden the model is already above the bar, so this row shows rhythm memory: the model plays back a remembered rhythm. It does not show a command filled in from what the sensors say.

Reported only (`info_` rows): hold last value over the 5th beat −1.00 (so the last step alone cannot tell a frozen state from a rhythm); R² after 10/40/200 blind steps 0.998–0.999 / 0.997–0.998 / 0.964–0.980; brake on the state (seeds 0, 1): first beat 0.35 0.39, 5th beat 0.000; tempo change 40 → 30 (seeds 0, 1): default 500 500 steps, one speed 400 400 — the beat is read every 100 steps, so that is one reading apart on two seeds; the fast part does not speed this up at these rates; irregular beat U(36, 44): period 40.72 39.42, first beat 0.923 0.953; lock-on 4–23 steps; step time: row `step_time`, 198 µs per learn step; one speed (row `one_speed`, seeds 0, 1): R² last step 0.984 0.978, 5th beat 0.986 0.980 against the default's 0.979 0.974 / 0.983 0.978; long blind (row `long_blind`, `LONG_BLIND_STEPS` 1,500 = 30 s at 50 Hz, five seeds, first start): R² over the last beat (blind steps 1,461–1,500) 0.641 0.629 0.861 0.754 0.434, RMS of the readout over the RMS of the signal there 1.065 0.424 0.925 0.659 0.363.

**The fast part.** The second learning speed is switched on in the gate (row `gate_ran_with_all_changes`), but at `eta_fast = eta_slow = 0.01`, `fast_decay = 0.01` it is nominal: after training `|C_fast|/|C_slow|` is 0.0035–0.0042 and `|A_fast|/|A_slow|` 0.0007–0.0009 (row 4, five seeds). The one-speed model blinds as well or slightly better (row `one_speed`) and finds a new tempo sooner (row `tempo_change`, 400 against 500 steps; the beat is read every 100 steps on two seeds, so that is one reading apart). The first value, `eta_fast` 0.05, made the fast part larger and missed bar 2. Its benefit (learning fast what is different today) is not shown in this stage. The second learning speed stays switched on at these rates; it is to be measured again on the simulated robot in stage 3, where something really changes (ground, load).

**What the rhythm test does not show.**
- "Blind" means that no input reaches the model, not data it has never seen: the blind stretch is the training signal carried on.
- The rhythm is fed in from outside. A robot finding its own rhythm in a closed loop is stage 3.
- The gate's horizon is 200 steps (4 s at 50 Hz). Row `long_blind` reports 1,500 steps (30 s): R² over the last beat 0.641 0.629 0.861 0.754 0.434, readout RMS / signal RMS 1.065 0.424 0.925 0.659 0.363. All five are below bar 2's 0.95: the 200-step result does not carry to 30 s, and the readout has shrunk in four of five seeds.
- The test signal is a steady sine plus one harmonic with noise, not the uneven signals of a walking robot.
- Bar 3 holds by construction: with time off and every channel hidden the readout is exactly 0, so R² is about 0 whatever was learned. The comparisons that carry information are the reported rows `hold_baseline` (hold last value −1.001 to −0.993) and `state_brake` (5th beat 0.000 on seeds 0, 1).

## Not built yet

- The adapter to the simulator and the `Brain` interface (stage 3).
- The goal pulling a hidden command in a closed loop (stage 3 or 4).
- Any use of the model on the robot.
- Ledge tasks.
