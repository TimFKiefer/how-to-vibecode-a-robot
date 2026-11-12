# SPDX-License-Identifier: MIT
"""Parameter ranges, the per-episode parameter draw and the episode jitter. One random stream per job."""

import numpy as np

from robot2sim.units import deg_to_rad, mm_to_m

RANGES = {
    "kp_scale": (0.7, 1.3),  # UNVERIFIED
    "kv_scale": (0.7, 1.3),  # UNVERIFIED
    "latency_steps": (1, 3),  # UNVERIFIED
    "encoder_sigma_rad": (0.0005, 0.002),  # UNVERIFIED
    "speed_sigma_rad_s": (0.01, 0.05),  # UNVERIFIED
    "load_sigma_pct": (0.5, 2.0),  # UNVERIFIED
    "voltage_sigma_v": (0.01, 0.05),  # UNVERIFIED
    "foot_mu": (0.6, 1.2),  # UNVERIFIED
    "blade_mu": (0.3, 0.5),  # UNVERIFIED
    "mass_scale": (0.95, 1.05),  # UNVERIFIED
}
NOMINAL_LATENCY = 2
JITTER = {"yaw_deg": 5.0, "pos_mm": 10.0, "joint_deg": 1.0}
STREAMS = {"params": 0, "jitter": 1, "bus": 2, "sensors": 3, "task": 4, "brain": 5}
N_JOINTS = 18


def rng(seed: int, stream: str) -> np.random.Generator:
    return np.random.default_rng([seed, STREAMS[stream]])


def params(seed: int, nominal: bool) -> dict:
    """One value per key of RANGES, uniform; latency is a whole number of control steps."""
    if nominal:
        return {k: NOMINAL_LATENCY if k == "latency_steps" else (lo + hi) / 2.0 for k, (lo, hi) in RANGES.items()}
    r = rng(seed, "params")
    out = {}
    for k, (lo, hi) in RANGES.items():
        out[k] = int(r.integers(lo, hi + 1)) if k == "latency_steps" else float(r.uniform(lo, hi))
    return out


def jitter(seed: int) -> dict:
    """Start yaw, xy shift and the 18 mounting offsets, uniform within ± JITTER."""
    r = rng(seed, "jitter")
    pos_m = mm_to_m(JITTER["pos_mm"])
    joint_rad = deg_to_rad(JITTER["joint_deg"])
    return {
        "yaw_rad": float(r.uniform(-1.0, 1.0) * deg_to_rad(JITTER["yaw_deg"])),
        "dx_m": float(r.uniform(-pos_m, pos_m)),
        "dy_m": float(r.uniform(-pos_m, pos_m)),
        "joint_offsets_rad": r.uniform(-joint_rad, joint_rad, N_JOINTS),
    }
