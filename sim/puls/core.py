# SPDX-License-Identifier: AGPL-3.0-only
"""The settle core of Puls in NumPy, float64. Rows are independent problems. Per row, with w = z for the
brake "state" and w = z - p for the brake "surprise":

    E(z) = 1/2 ||z - p||^2 + 1/2 sum_i D_i (x_i - (C z)_i)^2 + lam ||w||_1

    C (m, d) shared by all rows; z, p (d,) or (N, d); x, D (m,) or (N, m); D holds weights >= 0.

The smooth part has Hessian I + C^T D C with eigenvalues in [1, L], L = 1 + max(D) ||C||^2, so E is 1-strongly convex
and has exactly one minimum z*. `settle` runs proximal gradient with step 1/L:

    g = (z - p) - (D * (x - z @ C.T)) @ C,   v = z - g / L
    u = soft(v, lam / L)  (state)   or   u = p + soft(v - p, lam / L)  (surprise),   r = max |z - u|

The step is a (1 - 1/L)-contraction, so ||z - z*||_2 <= L sqrt(d) r (`distance_bound`). The surprise brake is the
state core applied to the innovation (z - p, x - C p) with prior 0, which is why it keeps exactly one minimum.
`minimum_gap` measures the smallest subgradient of E itself, independent of the settle's stopping test.
"""

import numpy as np

BRAKES = ("state", "surprise")
BOUNDS = ("spectral", "frobenius")


def soft(v, t) -> np.ndarray:
    """Soft threshold sign(v) * max(|v| - t, 0)."""
    v = np.asarray(v, dtype=float)
    return np.sign(v) * np.maximum(np.abs(v) - t, 0.0)


def _matrix(C) -> np.ndarray:
    C = np.asarray(C, dtype=float)
    if C.ndim != 2:
        raise ValueError(f"C must be 2-D (m, d), got shape {C.shape}")
    if not np.all(np.isfinite(C)):
        raise ValueError("C has a non-finite entry")
    return C


def lipschitz(C, d_max=1.0, bound="spectral") -> float:
    """L = 1 + d_max ||C||^2 >= the largest eigenvalue of I + C^T D C for every D with entries in [0, d_max].

    bound "spectral" takes the exact 2-norm of C, "frobenius" the Frobenius norm (up to rank(C) times looser).
    """
    if bound not in BOUNDS:
        raise ValueError(f"bound must be one of {BOUNDS}, got {bound!r}")
    if not (np.isfinite(d_max) and d_max >= 0):
        raise ValueError(f"d_max must be finite and >= 0, got {d_max!r}")
    C = _matrix(C)
    norm = np.linalg.norm(C, 2 if bound == "spectral" else "fro")
    return float(1.0 + d_max * norm * norm)


def _number(name, v) -> float:
    """v as a finite float; ValueError naming `name` for anything else (NaN, inf, a string, None, an array)."""
    if isinstance(v, (bool, np.bool_)) or not isinstance(v, (int, float, np.integer, np.floating)):
        raise ValueError(f"{name} must be a finite number, got {v!r}")
    if not np.isfinite(v):
        raise ValueError(f"{name} must be finite, got {v!r}")
    return float(v)


def _inputs(z_name, z, p, x, D, C, lam, brake):
    """Checked float64 copies as 2-D rows: (z, p, x, D, C, lam, one_d). Raises ValueError naming the argument."""
    if brake not in BRAKES:
        raise ValueError(f"brake must be one of {BRAKES}, got {brake!r}")
    if _number("lam", lam) < 0:
        raise ValueError(f"lam must be >= 0, got {lam!r}")
    C = _matrix(C)
    m, d = C.shape
    z, p, x, D = (np.asarray(a, dtype=float) for a in (z, p, x, D))
    one_d = z.ndim == 1
    lead = () if one_d else z.shape[:1]
    if z.ndim not in (1, 2):
        raise ValueError(f"{z_name} must be (d,) or (N, d), got shape {z.shape}")
    if z.shape[-1] != d:
        raise ValueError(f"{z_name} has d = {z.shape[-1]} but C has shape {C.shape}: they do not fit")
    for name, a, width in (("p", p, d), ("x", x, m), ("D", D, m)):
        if a.shape != lead + (width,):
            raise ValueError(f"{name} must have shape {lead + (width,)}, got {a.shape}")
    for name, a in ((z_name, z), ("p", p), ("x", x), ("D", D)):
        if not np.all(np.isfinite(a)):
            raise ValueError(f"{name} has a non-finite entry")
    if np.any(D < 0):
        raise ValueError("D has a negative entry (weights must be >= 0)")
    rows = (a.reshape(1, -1) if one_d else a for a in (z, p, x, D))
    return (*rows, C, float(lam), one_d)


def _gradient(z, p, x, D, C) -> np.ndarray:
    return (z - p) - (D * (x - z @ C.T)) @ C


def _per_row(values, one_d):
    return values[0] if one_d else values


def energy(z, p, x, D, C, lam, brake="state") -> np.ndarray:
    """E(z) per row, shape (N,), or shape () for 1-D input."""
    z, p, x, D, C, lam, one_d = _inputs("z", z, p, x, D, C, lam, brake)
    w = z if brake == "state" else z - p
    E = (0.5 * np.sum((z - p) ** 2, axis=1) + 0.5 * np.sum(D * (x - z @ C.T) ** 2, axis=1)
         + lam * np.sum(np.abs(w), axis=1))
    return _per_row(E, one_d)


def minimum_gap(z, p, x, D, C, lam, brake="state") -> np.ndarray:
    """2-norm of the smallest subgradient of E at z, per row; shape () for 1-D input.

    E is 1-strongly convex, so ||z - z*||_2 <= minimum_gap(z).
    """
    z, p, x, D, C, lam, one_d = _inputs("z", z, p, x, D, C, lam, brake)
    g = _gradient(z, p, x, D, C)
    w = z if brake == "state" else z - p
    xi = np.where(w != 0, g + lam * np.sign(w), soft(g, lam))
    return _per_row(np.linalg.norm(xi, axis=1), one_d)


def settle(z_init, p, x, D, C, lam, *, tol=1e-7, max_iters=500, brake="state", bound="spectral") -> dict:
    """Minimise E by proximal gradient from z_init (module docstring); never raises on non-convergence.

    Returns {"z", "converged": bool, "iters": int, "final_residual": float, "L": float}; final_residual is r over all
    rows and coordinates at the returned z.
    """
    if bound not in BOUNDS:
        raise ValueError(f"bound must be one of {BOUNDS}, got {bound!r}")
    if _number("tol", tol) <= 0:
        raise ValueError(f"tol must be > 0, got {tol!r}")
    if isinstance(max_iters, (bool, np.bool_)) or not isinstance(max_iters, (int, np.integer)) or max_iters < 1:
        raise ValueError(f"max_iters must be an integer >= 1 (a float such as 3.0 is refused), got {max_iters!r}")
    z, p, x, D, C, lam, one_d = _inputs("z_init", z_init, p, x, D, C, lam, brake)
    z = z.copy()  # the result never shares memory with z_init
    L = lipschitz(C, float(np.max(D)) if D.size else 0.0, bound)
    CtDC = np.einsum("nm,md,me->nde", D, C, C)
    CtDx = (D * x) @ C
    t = lam / L
    surprise = brake == "surprise"

    def step(z):
        g = (z - p) + np.einsum("nde,ne->nd", CtDC, z) - CtDx
        v = z - g / L
        u = p + soft(v - p, t) if surprise else soft(v, t)
        return u, float(np.max(np.abs(z - u)))

    u, r = step(z)
    iters = 0
    while r > tol and iters < max_iters:
        z = u
        iters += 1
        u, r = step(z)
    return {"z": _per_row(z, one_d), "converged": bool(r <= tol), "iters": iters, "final_residual": r, "L": L}


def distance_bound(result) -> float:
    """Proved bound on ||z - z*||_2 for a settle() result: L sqrt(d) final_residual."""
    d = np.shape(result["z"])[-1]
    return float(result["L"] * np.sqrt(d) * result["final_residual"])
