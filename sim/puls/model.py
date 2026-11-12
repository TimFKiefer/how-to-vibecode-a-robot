# SPDX-License-Identifier: AGPL-3.0-only
"""The stream model of Puls around the settle core: per step

    p = A z_prev   (learned time; zeros when `time` is off)
    z = settle(z_init=p, p, x, d, C, lam, brake)      readout = C z

with C = C_slow + C_fast (latent -> channels) and A = A_slow + A_fast (last state -> expected state). Each weight
changes by (error at its output) x (activity at its input): err_c = d * (x - C z) with input z for C, err_a = z - p
with input z_prev for A. The slow part sums the rule at eta_slow; the fast part decays by fast_decay and adds the rule
at eta_fast. `d` is a weight >= 0 per channel; d = 0 hides the channel (its x is ignored, the readout fills it in).
Scaling the channels to about ±1 is the caller's job.
"""

from dataclasses import dataclass

import numpy as np

from puls.core import BRAKES, _number, settle

BEAT_MIN_MODULUS = 0.9  # UNVERIFIED: an eigenvalue pair of A below this is no held rhythm
BEAT_MIN_IMAG = 1e-6  # an eigenvalue with |imag| at or below this is real: no rotation, no beat
# the plain core: no time, one speed, the brake on the state
PLAIN_CORE = {"time": False, "brake": "state", "eta_fast": 0.0}


@dataclass(frozen=True)
class Config:
    n_channels: int
    n_latent: int = 8
    lam: float = 0.02
    brake: str = "surprise"
    time: bool = True
    eta_slow: float = 0.01
    eta_fast: float = 0.01     # UNVERIFIED on a robot; the one setting that was tuned: 0.05 missed bar 2 only (README, Stage 2 choices)
    fast_decay: float = 0.01   # UNVERIFIED
    init_scale: float = 0.1
    a_init: float = 0.5
    tol: float = 1e-7
    max_iters: int = 500

    def __post_init__(self):
        for name in ("n_channels", "n_latent", "max_iters"):
            v = getattr(self, name)
            if isinstance(v, bool) or not isinstance(v, (int, np.integer)) or v < 1:
                raise ValueError(f"{name} must be an integer >= 1, got {v!r}")
        for name in ("lam", "eta_slow", "eta_fast"):
            if _number(name, getattr(self, name)) < 0:  # the core's number rule: NaN, inf, str, 0-d array refused
                raise ValueError(f"{name} must be finite and >= 0, got {getattr(self, name)!r}")
        if _number("tol", self.tol) <= 0:
            raise ValueError(f"tol must be > 0, got {self.tol!r}")
        for name in ("init_scale", "a_init"):
            _number(name, getattr(self, name))
        if not isinstance(self.time, bool):
            raise ValueError(f"time must be True or False, got {self.time!r}")
        if not (0.0 <= self.fast_decay <= 1.0):
            raise ValueError(f"fast_decay must be in [0, 1], got {self.fast_decay!r}")
        if self.brake not in BRAKES:
            raise ValueError(f"brake must be one of {BRAKES}, got {self.brake!r}")


def _vector(name, v, n) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    if v.shape != (n,):
        raise ValueError(f"{name} must have shape ({n},), got {v.shape}")
    return v


class BodyModel:
    """Weights C_slow, C_fast (n_channels, n_latent), A_slow, A_fast (n_latent, n_latent); state z (n_latent,)."""

    def __init__(self, config: Config, seed: int = 0):
        self.config = config
        n, k = config.n_channels, config.n_latent
        self.C_slow = config.init_scale * np.random.default_rng(seed).standard_normal((n, k))
        self.C_fast = np.zeros((n, k))
        self.A_slow = config.a_init * np.eye(k) if config.time else np.zeros((k, k))
        self.A_fast = np.zeros((k, k))
        self.z = np.zeros(k)
        self.unsettled = 0

    @property
    def C(self) -> np.ndarray:
        return self.C_slow + self.C_fast

    @property
    def A(self) -> np.ndarray:
        return self.A_slow + self.A_fast

    def reset_state(self) -> None:
        """z = 0, weights kept: a new attempt."""
        self.z = np.zeros(self.config.n_latent)

    def _learned(self, slow, fast, err, pre):
        rule = np.outer(err, pre)
        return slow + self.config.eta_slow * rule, (1.0 - self.config.fast_decay) * fast + self.config.eta_fast * rule

    def step(self, x, d, learn: bool = True) -> np.ndarray:
        """One step on channels x with weights d (d = 0: hidden, x ignored there); returns C z for ALL channels.

        ValueError for bad input, FloatingPointError when a result would not be finite; either leaves the model as it
        was before the call.
        """
        cfg = self.config
        x = _vector("x", x, cfg.n_channels)
        d = _vector("d", d, cfg.n_channels)
        if not np.all(np.isfinite(d)) or np.any(d < 0):
            raise ValueError("d must be finite and >= 0")
        seen = d > 0
        if not np.all(np.isfinite(x[seen])):
            raise ValueError("x has a non-finite entry at a visible channel (d > 0)")
        x = np.where(seen, x, 0.0)
        C, A, z_prev = self.C, self.A, self.z
        with np.errstate(over="ignore", invalid="ignore", divide="ignore", under="ignore"):
            p = A @ z_prev if cfg.time else np.zeros(cfg.n_latent)
            if not np.all(np.isfinite(p)):
                raise FloatingPointError("the expectation A z is not finite")
            result = settle(p, p, x, d, C, cfg.lam, tol=cfg.tol, max_iters=cfg.max_iters, brake=cfg.brake)
            z = result["z"]
            readout = C @ z
            weights = (self.C_slow, self.C_fast, self.A_slow, self.A_fast)
            if learn:
                weights = self._learned(self.C_slow, self.C_fast, d * (x - readout), z) + (
                    self._learned(self.A_slow, self.A_fast, z - p, z_prev) if cfg.time else weights[2:])
            if not all(np.all(np.isfinite(a)) for a in (*weights, z, readout)):
                raise FloatingPointError("a weight, the state or the readout would not be finite")
        self.C_slow, self.C_fast, self.A_slow, self.A_fast = weights
        self.z = z
        self.unsettled += not result["converged"]
        return readout

    def beat(self) -> tuple[float, float] | None:
        """(period in steps, modulus) of the slowest held rhythm of A: among eigenvalues with |imag| > BEAT_MIN_IMAG and
        modulus >= BEAT_MIN_MODULUS the one with the smallest |angle|; None if there is none."""
        ev = np.linalg.eigvals(self.A)
        held = ev[(np.abs(ev.imag) > BEAT_MIN_IMAG) & (np.abs(ev) >= BEAT_MIN_MODULUS)]
        if held.size == 0:
            return None
        k = int(np.argmin(np.abs(np.angle(held))))
        return float(2 * np.pi / abs(np.angle(held[k]))), float(abs(held[k]))
