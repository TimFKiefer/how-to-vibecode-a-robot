# SPDX-License-Identifier: MIT
"""Row runner for the checks. A row is a function; its name is the row's name."""


def run(rows) -> int:
    bad = 0
    for n, row in enumerate(rows, 1):
        info = row.__name__.startswith("info_")
        name = row.__name__.removeprefix("row_").removeprefix("info_")
        try:
            print(f"{'INFO' if info else 'PASS'}  {n} {name}  {row()}")
        except Exception as e:  # an assert or any crash is a FAIL
            bad += 1
            print(f"FAIL  {n} {name}  {type(e).__name__}: {e}")
    return 1 if bad else 0
