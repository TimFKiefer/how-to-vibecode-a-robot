# SPDX-License-Identifier: MIT
"""Stage 2 gate: runs every stage-2 check one after the other, each as its own subprocess (through
`check_e1_all.run_all`), and prints one line per script at the end. Exit 0 only when all exit 0. Works from any
working directory.

    cd <repository root> && sim/.venv/bin/python sim/checks/check_e2_all.py
"""

import sys

from check_e1_all import CHECKS, run_all

SCRIPTS = ("check_e2_core.py", "check_e2_model.py", "check_e2_rhythm.py")


if __name__ == "__main__":
    sys.exit(run_all([CHECKS / s for s in SCRIPTS]))
