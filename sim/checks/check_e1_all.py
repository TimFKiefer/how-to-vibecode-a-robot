# SPDX-License-Identifier: MIT
"""Stage 1 gate: runs every check one after the other, each as its own subprocess, and prints one line per
script at the end. Exit 0 only when all nine exit 0. Works from any working directory.

    cd <repository root> && sim/.venv/bin/python sim/checks/check_e1_all.py

A new body file is checked by `.venv/bin/python -m robot2sim.build` (from sim/) followed by this script.
`check_e1_live.py` opens a window and is run by hand, not here.
"""

import subprocess
import sys
import time
from pathlib import Path

CHECKS = Path(__file__).resolve().parent
ROOT = CHECKS.parents[1]
SCRIPTS = (
    "check_s1_body.py",
    "check_e1_body.py",
    "check_e1_servo.py",
    "check_e1_world.py",
    "check_e1_sensors.py",
    "check_e1_tasks.py",
    "check_e1_episode.py",
    "check_e1_scripted.py",
    "check_e1_run.py",
)


def run_all(scripts) -> int:
    """Run each script with this interpreter from the repository root, output passed through; print the table;
    return the exit code."""
    results = []
    for script in scripts:
        print(f"==== {script.name}", flush=True)
        start = time.perf_counter()
        code = subprocess.run([sys.executable, str(script)], cwd=ROOT).returncode
        results.append((script.name, code, time.perf_counter() - start))
    print("==== stage gate", flush=True)
    for name, code, seconds in results:
        print(f"{'PASS' if code == 0 else 'FAIL'}  {name}  {seconds:.1f} s" + ("" if code == 0 else f"  exit {code}"))
    return 0 if all(code == 0 for _, code, _ in results) else 1


if __name__ == "__main__":
    sys.exit(run_all([CHECKS / s for s in SCRIPTS]))
