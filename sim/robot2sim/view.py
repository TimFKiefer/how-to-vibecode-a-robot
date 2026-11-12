# SPDX-License-Identifier: MIT
"""Open the MuJoCo viewer on a body.

From sim/, on macOS (mjpython ships with the mujoco package):

    .venv/bin/mjpython -m robot2sim.view --body v0 --pose stand

The same command with python re-execs mjpython on macOS. --headless loads the
pose and exits, which is what checks use. A live view is the default.
"""

import argparse
import os
import sys
from pathlib import Path

from robot2sim.scene import load_model, load_spec, set_pose
from robot2sim.units import m_to_mm


def _ensure_mjpython(argv, module: str = "robot2sim.view"):
    """Re-execute `python -m module argv` under mjpython on macOS, unless --headless or already under it."""
    if "--headless" in argv or sys.platform != "darwin":
        return
    if os.environ.get("MJPYTHON_BIN"):
        return
    mjpython = Path(sys.executable).parent / "mjpython"  # the venv's bin, not the resolved base interpreter
    if not mjpython.exists():
        raise SystemExit(
            "On macOS the live view needs mjpython from the mujoco package:\n"
            f"  {mjpython} -m {module} " + " ".join(argv)
        )
    os.execv(str(mjpython), [str(mjpython), "-m", module, *argv])


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    _ensure_mjpython(argv)
    parser = argparse.ArgumentParser(description="View Takt in MuJoCo")
    parser.add_argument("--body", required=True)
    parser.add_argument("--pose", default="stand")
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args(argv)
    spec = load_spec(args.body)
    if args.pose not in spec["poses"]:
        known = ", ".join(sorted(spec["poses"]))
        raise SystemExit(f"unknown pose {args.pose!r}; this body has {known}")
    import mujoco
    model = load_model(args.body)
    data = mujoco.MjData(model)
    pose = spec["poses"][args.pose]
    set_pose(model, data, spec, pose["B"], pose["joints_deg"])
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body")
    height = m_to_mm(float(data.xpos[bid][2]))
    print(f"{args.body} pose {args.pose}: belly {height:.2f} mm, {model.nu} actuators")
    if args.headless:
        return 0
    import mujoco.viewer
    mujoco.viewer.launch(model, data)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
