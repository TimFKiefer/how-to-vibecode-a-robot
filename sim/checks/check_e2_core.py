# SPDX-License-Identifier: MIT
"""Stage 2: the settle core of `puls`. Bar 1 (1,000 random cases: the settle finds the one minimum of
its energy, rest below 1e-6) and the core's own rows. Exit 0 only when every row passes.

    cd <repository root> && sim/.venv/bin/python sim/checks/check_e2_core.py
"""

import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _rows import run  # noqa: E402
from puls.core import BOUNDS, BRAKES, distance_bound, energy, lipschitz, minimum_gap, settle, soft  # noqa: E402

TIGHT = {"tol": 1e-10, "max_iters": 50_000}
GAP_BAR = 1e-6
MINIMUM_CASES = 1000
CLOSED_FORM_CASES = 200
MONOTONE_CASES = 50
MONOTONE_STEPS = 200
BATCH_CASES = 20  # UNVERIFIED: chosen
BATCH_ROWS = 7
SEED_MINIMUM = 1
SEED_LAM0 = 2
SEED_MASKED = 3
SEED_INNOVATION = 4
SEED_SURPRISE_MINIMUM = 5
SEED_MONOTONE = 6
SEED_BATCH = 7
SEED_REFUSED = 8
SEED_NON_CONVERGENCE = 9


def _case(rng, d=None, m=None, scale=None):
    """One random problem: (z_init, p, x, C, lam) with d in 1..12, m in 1..24, C = scale * randn(m, d)."""
    d = int(rng.integers(1, 13)) if d is None else d
    m = int(rng.integers(1, 25)) if m is None else m
    scale = rng.uniform(0.05, 1.0) if scale is None else scale
    C = scale * rng.standard_normal((m, d))
    return rng.standard_normal(d), rng.standard_normal(d), rng.standard_normal(m), C, rng.uniform(0.0, 0.5)


def _mask(rng, m):
    return rng.integers(0, 2, m).astype(float)


def _minimum_rows(seed, brake, d_kinds):
    rng = np.random.default_rng(seed)
    worst_gap, most_iters, far_points, worst_bound = 0.0, 0, 0, 0.0
    for n in range(MINIMUM_CASES):
        z0, p, x, C, lam = _case(rng)
        m = len(x)
        D = {"ones": np.ones(m), "zeros": np.zeros(m), "mask": _mask(rng, m),
             "weights": rng.uniform(0.0, 3.0, m)}[d_kinds[n % len(d_kinds)]]
        res = settle(z0, p, x, D, C, lam, brake=brake, **TIGHT)
        assert res["converged"], f"case {n}: not converged after {res['iters']} iterations"
        bound, want = distance_bound(res), res["L"] * np.sqrt(len(z0)) * res["final_residual"]
        assert abs(bound - want) <= 1e-12 * want, f"case {n}: distance_bound {bound:.6e} ≠ L √d r {want:.6e}"
        assert bound < GAP_BAR, f"case {n}: distance_bound {bound:.3e} (the guard below would be empty)"
        worst_bound = max(worst_bound, bound)
        gap = float(minimum_gap(res["z"], p, x, D, C, lam, brake=brake))
        assert gap < GAP_BAR, f"case {n}: gap {gap:.3e}"
        worst_gap, most_iters = max(worst_gap, gap), max(most_iters, res["iters"])
        w_zero = 0 * p if brake == "state" else p
        for label, q in (("z_init", z0), ("w = 0", w_zero)):
            floor = float(np.linalg.norm(q - res["z"])) - distance_bound(res) - 1e-12
            q_gap = float(minimum_gap(q, p, x, D, C, lam, brake=brake))
            assert q_gap >= floor, f"case {n} at {label}: gap {q_gap:.3e} < ‖q − z‖ − bound {floor:.3e}"
            far_points += 1
    return (f"{MINIMUM_CASES} cases converged, worst gap {worst_gap:.2e} < {GAP_BAR:.0e}, most iterations {most_iters}; "
            f"{far_points} points away from the minimum have gap ≥ their distance − distance_bound (= L √d r, "
            f"largest {worst_bound:.1e} < {GAP_BAR:.0e})")


def row_minimum_1000():
    return _minimum_rows(SEED_MINIMUM, "state", ("ones", "zeros", "mask"))


def row_lam0_closed_form():
    rng = np.random.default_rng(SEED_LAM0)
    worst = 0.0
    for n in range(CLOSED_FORM_CASES):
        z0, p, x, C, _ = _case(rng)
        D = rng.uniform(0.0, 3.0, len(x))
        H = np.eye(len(p)) + C.T @ (D[:, None] * C)
        z_star = np.linalg.solve(H, p + C.T @ (D * x))
        grad = float(np.linalg.norm(H @ (z0 - z_star)))
        gap0 = float(minimum_gap(z0, p, x, D, C, 0.0))
        assert abs(gap0 - grad) <= 1e-9 * max(1.0, grad), f"case {n}: gap at z_init {gap0:.6e} ≠ ‖H (z_init − z*)‖ {grad:.6e}"
        for bound in BOUNDS:
            res = settle(z0, p, x, D, C, 0.0, bound=bound, **TIGHT)
            assert res["converged"], f"case {n} {bound}: not converged after {res['iters']} iterations"
            dist = float(np.linalg.norm(res["z"] - z_star))
            assert dist <= distance_bound(res) + 1e-12, f"case {n} {bound}: {dist:.3e} > bound {distance_bound(res):.3e}"
            assert dist < GAP_BAR, f"case {n} {bound}: distance {dist:.3e}"
            worst = max(worst, dist)
    return (f"{CLOSED_FORM_CASES} cases × {len(BOUNDS)} bounds, worst ‖z − z*‖ {worst:.2e}, within the distance bound; "
            f"gap at z_init = ‖H (z_init − z*)‖ in {CLOSED_FORM_CASES} cases")


def row_masked_closed_form():
    rng = np.random.default_rng(SEED_MASKED)
    worst = 0.0
    for n in range(CLOSED_FORM_CASES):
        z0, p, x, C, lam = _case(rng)
        D = np.zeros(len(x))
        res = settle(z0, p, x, D, C, lam, brake="state", **TIGHT)
        assert res["converged"], f"case {n}: state not converged"
        err = float(np.max(np.abs(res["z"] - soft(p, lam))))
        assert err < 1e-9, f"case {n}: state off soft(p, λ) by {err:.3e}"
        worst = max(worst, err)
        res = settle(p, p, x, D, C, lam, brake="surprise", **TIGHT)
        assert np.array_equal(res["z"], p) and res["iters"] == 0, f"case {n}: surprise moved, iters {res['iters']}"
    return f"{CLOSED_FORM_CASES} cases: state = soft(p, λ) within {worst:.1e}; surprise from p returns p, 0 iterations"


def row_surprise_is_core_on_innovation():
    rng = np.random.default_rng(SEED_INNOVATION)
    worst = 0.0
    for n in range(CLOSED_FORM_CASES):
        z0, p, x, C, lam = _case(rng)
        D = _mask(rng, len(x))
        a = settle(z0, p, x, D, C, lam, brake="surprise", **TIGHT)
        b = settle(z0 - p, 0 * p, x - C @ p, D, C, lam, brake="state", **TIGHT)
        assert a["converged"] and b["converged"], f"case {n}: not converged"
        err = float(np.max(np.abs(a["z"] - (p + b["z"]))))
        assert err < 1e-8, f"case {n}: {err:.3e}"
        worst = max(worst, err)
    return f"{CLOSED_FORM_CASES} cases: surprise brake = p + state core on (z − p, x − C p) within {worst:.1e}"


def row_surprise_minimum_1000():
    return _minimum_rows(SEED_SURPRISE_MINIMUM, "surprise", ("ones", "zeros", "mask", "weights"))


def row_energy_monotone():
    rng = np.random.default_rng(SEED_MONOTONE)
    worst = -np.inf
    for n in range(MONOTONE_CASES):
        z0, p, x, C, lam = _case(rng)
        D = _mask(rng, len(x))
        for brake in BRAKES:
            for bound in BOUNDS:
                z = z0
                E = float(energy(z, p, x, D, C, lam, brake=brake))
                for k in range(MONOTONE_STEPS):
                    z = settle(z, p, x, D, C, lam, tol=TIGHT["tol"], max_iters=1, brake=brake, bound=bound)["z"]
                    E_new = float(energy(z, p, x, D, C, lam, brake=brake))
                    rise = (E_new - E) / (1 + abs(E))
                    assert rise <= 1e-12, f"case {n} {brake} {bound} step {k}: energy rose {E} → {E_new}"
                    worst, E = max(worst, rise), E_new
    return (f"{MONOTONE_CASES} cases × {len(BRAKES)} brakes × {len(BOUNDS)} bounds × {MONOTONE_STEPS} steps, "
            f"largest relative rise {worst:.1e} (negative = always fell)")


def row_batch_equals_rows():
    rng = np.random.default_rng(SEED_BATCH)
    worst_z, worst_same = 0.0, 0.0
    for n in range(BATCH_CASES):
        d, m = int(rng.integers(1, 13)), int(rng.integers(1, 25))
        C = rng.uniform(0.05, 1.0) * rng.standard_normal((m, d))
        lam = rng.uniform(0.0, 0.5)
        Z0, P = rng.standard_normal((BATCH_ROWS, d)), rng.standard_normal((BATCH_ROWS, d))
        X, D = rng.standard_normal((BATCH_ROWS, m)), rng.integers(0, 2, (BATCH_ROWS, m)).astype(float)
        for brake in BRAKES:
            batch = settle(Z0, P, X, D, C, lam, brake=brake, **TIGHT)
            assert batch["converged"] and batch["z"].shape == (BATCH_ROWS, d), f"case {n} {brake}: batch"
            Zb = batch["z"]
            Eb = energy(Zb, P, X, D, C, lam, brake=brake)
            Gb = minimum_gap(Zb, P, X, D, C, lam, brake=brake)
            assert Eb.shape == Gb.shape == (BATCH_ROWS,), f"case {n}: shapes {Eb.shape} {Gb.shape}"
            assert np.all(Gb < GAP_BAR), f"case {n} {brake}: batch gaps {Gb}"
            for i in range(BATCH_ROWS):
                one = settle(Z0[i], P[i], X[i], D[i], C, lam, brake=brake, **TIGHT)
                assert one["converged"] and one["z"].shape == (d,), f"case {n} {brake} row {i}: single"
                e1 = energy(one["z"], P[i], X[i], D[i], C, lam, brake=brake)
                g1 = minimum_gap(one["z"], P[i], X[i], D[i], C, lam, brake=brake)
                assert e1.shape == g1.shape == (), f"case {n}: 1-D shapes {e1.shape} {g1.shape}"
                assert g1 < GAP_BAR, f"case {n} {brake} row {i}: gap {g1:.3e}"
                dz = float(np.max(np.abs(Zb[i] - one["z"])))
                de = abs(float(Eb[i] - e1))
                assert dz < 1e-6 and de < 1e-6, f"case {n} {brake} row {i}: z {dz:.3e}, energy {de:.3e}"
                same_e = abs(float(Eb[i] - energy(Zb[i], P[i], X[i], D[i], C, lam, brake=brake)))
                same_g = abs(float(Gb[i] - minimum_gap(Zb[i], P[i], X[i], D[i], C, lam, brake=brake)))
                assert same_e <= 1e-12 and same_g <= 1e-12, f"case {n} {brake} row {i}: {same_e:.1e} {same_g:.1e}"
                worst_z, worst_same = max(worst_z, dz), max(worst_same, same_e, same_g)
    return (f"{BATCH_CASES} cases × {len(BRAKES)} brakes, ({BATCH_ROWS}, d) = {BATCH_ROWS} × (d,): z within "
            f"{worst_z:.1e}, same z energy/gap within {worst_same:.1e}")


def _refused(name, fn, *inputs):
    """fn() raises ValueError whose text names `name`, and leaves every input array bit-identical."""
    before = [a.tobytes() for a in inputs]
    try:
        fn()
    except ValueError as e:
        assert re.search(rf"\b{re.escape(name)}\b", str(e)), f"{name}: message {str(e)!r}"
    else:
        raise AssertionError(f"{name}: not refused")
    assert [a.tobytes() for a in inputs] == before, f"{name}: an input changed"


def row_bad_input_refused():
    rng = np.random.default_rng(SEED_REFUSED)
    z0, p, x, C, lam = _case(rng, d=4, m=6)
    D = np.ones(6)
    arrays = {"z_init": z0, "p": p, "x": x, "D": D, "C": C}

    def call(over, lam=lam, **kw):
        """(the settle call with `over` replacing inputs, the five input arrays it gets)."""
        a = {**arrays, **over}
        return lambda: settle(*a.values(), lam, **{**TIGHT, **kw}), *a.values()

    cases = [("z_init", call({"z_init": np.zeros(5)})), ("p", call({"p": np.zeros((2, 4))})),
             ("x", call({"x": np.zeros(5)})), ("D", call({"D": np.ones(7)})), ("C", call({"C": np.zeros((6, 5))})),
             ("C", call({"C": np.zeros(6)})), ("lam", call({}, lam=-0.1)), ("lam", call({}, lam=np.inf)),
             ("lam", call({}, lam=np.nan)), ("D", call({"D": np.array([1, 1, -0.5, 1, 1, 1.0])})),
             ("brake", call({}, brake="sometimes")), ("bound", call({}, bound="loose")), ("tol", call({}, tol=0.0)),
             ("tol", call({}, tol=-1e-9)), ("max_iters", call({}, max_iters=0)),
             ("tol", call({}, tol=np.nan)), ("tol", call({}, tol=np.inf)), ("tol", call({}, tol="tight")),
             ("max_iters", call({}, max_iters=np.nan)), ("max_iters", call({}, max_iters=np.inf)),
             ("max_iters", call({}, max_iters="many")), ("max_iters", call({}, max_iters=3.0)),
             ("max_iters", call({}, max_iters=2.5)), ("lam", call({}, lam="half")), ("lam", call({}, lam=None))]
    for name, arr in arrays.items():
        for bad in (np.nan, np.inf):
            broken = arr.copy()
            broken.flat[0] = bad
            cases.append((name, call({name: broken})))
    for name, (fn, *inputs) in cases:
        _refused(name, fn, *inputs)
    batch = (np.zeros((3, 4)), np.zeros((3, 4)), np.zeros((3, 6)), np.ones((3, 6)))
    _refused("D", lambda: settle(*batch[:3], np.ones((2, 6)), C, lam, **TIGHT), *batch, C)
    _refused("brake", lambda: energy(z0, p, x, D, C, lam, brake="never"), *arrays.values())
    _refused("brake", lambda: minimum_gap(z0, p, x, D, C, lam, brake="never"), *arrays.values())
    _refused("z", lambda: minimum_gap(np.full(4, np.nan), p, x, D, C, lam), *arrays.values())
    _refused("bound", lambda: lipschitz(C, bound="loose"), C)
    _refused("d_max", lambda: lipschitz(C, d_max=-1.0), C)
    return f"{len(cases) + 6} refusals, each a ValueError naming the argument, inputs bit-identical"


def row_non_convergence_reported():
    rng = np.random.default_rng(SEED_NON_CONVERGENCE)
    z0, p, x, C, lam = _case(rng, d=8, m=16, scale=1.0)
    res = settle(z0, p, x, np.ones(16), C, lam, tol=1e-12, max_iters=3)
    assert res["converged"] is False and res["iters"] == 3, (res["converged"], res["iters"])
    assert res["final_residual"] > 1e-12 and np.all(np.isfinite(res["z"])), res["final_residual"]
    return f"converged False after 3 iterations, final residual {res['final_residual']:.2e}, z finite"


if __name__ == "__main__":
    sys.exit(run((row_minimum_1000, row_lam0_closed_form, row_masked_closed_form, row_surprise_is_core_on_innovation,
                  row_surprise_minimum_1000, row_energy_monotone, row_batch_equals_rows, row_bad_input_refused,
                  row_non_convergence_reported)))
