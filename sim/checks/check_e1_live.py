# SPDX-License-Identifier: MIT
"""Stage 1: the live window. Opens a real MuJoCo window for 3 s, so it is run on demand at a machine with a display
and is not part of the stage gate. The run writes into a temp folder, never runs/. Exit 0 only when every row passes.

    cd <repository root> && sim/.venv/bin/python sim/checks/check_e1_live.py
"""

import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _rows import run  # noqa: E402
from robot2sim.paths import SIM  # noqa: E402

SECONDS = 3
CHILD_TIMEOUT_S = 60
WALL_BAND_S = (2.7, 4.5)
CLOCK_LINE = re.compile(r"real time: robot ([0-9.]+) s, wall ([0-9.]+) s")
LIVE = {}  # row 1's run, read by row 2


def row_plain_python_opens_window():
    assert "mjpython" not in Path(sys.executable).name, sys.executable
    with tempfile.TemporaryDirectory() as tmp:
        child = subprocess.run(
            [sys.executable, "-m", "robot2sim.run", "--brain", "scripted", "--task", "walk_1m", "--nominal",
             "--seconds", str(SECONDS), "--out", tmp],
            cwd=SIM, capture_output=True, text=True, timeout=CHILD_TIMEOUT_S,
        )
        LIVE["stdout"] = child.stdout
        assert child.returncode == 0, f"exit {child.returncode}, stderr {child.stderr[-400:]!r}"
        lines = child.stdout.splitlines()
        assert "window: open" in lines, f"stdout {child.stdout!r}"
        assert "visible groups: [2]" in lines, f"stdout {child.stdout!r}"
        summary = json.loads((Path(tmp) / "summary.json").read_text())
    assert summary["episodes"] == 1 and summary["reasons"]["aborted"] == 1, summary
    return f"{Path(sys.executable).name}: exit 0, window: open, visible groups: [2], 1 episode aborted"


def row_real_time():
    match = CLOCK_LINE.search(LIVE["stdout"])
    assert match, f"no clock line in {LIVE['stdout']!r}"
    robot_s, wall_s = float(match.group(1)), float(match.group(2))
    assert abs(robot_s - SECONDS) < 1e-9, robot_s
    assert WALL_BAND_S[0] <= wall_s <= WALL_BAND_S[1], f"{robot_s} s robot time took {wall_s} s"
    return f"robot {robot_s:.2f} s in {wall_s:.2f} s wall (band {WALL_BAND_S[0]}…{WALL_BAND_S[1]} s)"


if __name__ == "__main__":
    sys.exit(run([row_plain_python_opens_window, row_real_time]))
