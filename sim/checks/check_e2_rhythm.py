# SPDX-License-Identifier: MIT
"""Stage 2: the rhythm test. Success bars 2 and 3 of this stage (with time: the beat within 1 % and five beats
blind with R² >= 0.95 in 5 of 5 seeds; the core without time fails the same test with R² below 0.1), the fill-in row
(commands as channels) and the reported-only rows. Exit 0 only when every gate row passes; `info_` rows never fail on a number.

    cd <repository root> && sim/.venv/bin/python sim/checks/check_e2_rhythm.py
"""

import copy
import functools
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _rows import run  # noqa: E402
from puls.model import PLAIN_CORE, BodyModel, Config  # noqa: E402

N_CH, N_LAT, PERIOD, NOISE, HARMONIC = 18, 8, 40.0, 0.05, 0.3
LEARN_STEPS, WARM_STEPS, BLIND_STEPS = 20_000, 200, 200          # 200 blind steps = 5 beats
SEEDS = (0, 1, 2, 3, 4)
INFO_SEEDS = SEEDS[:2]                                            # reported-only rows that need their own training
STARTS = range(LEARN_STEPS + 500, LEARN_STEPS + 2500, 97)         # 21 start points, 97 shares no factor with 40
R2_BAR, BEAT_TOL, NO_TIME_BAR = 0.95, 0.01, 0.1                  # the success bars of this stage
FILL_BAR = 0.9                                                    # chosen here, not a success bar of this stage
N_CMD, N_SENS, DELAY = 6, 12, 2                                   # the made-up body of the fill-in row

BEAT_STEPS = int(PERIOD)
HORIZONS = (10, 40, 200)
NEW_PERIOD = 30.0
TEMPO_LOOK_EVERY, TEMPO_MAX_STEPS = 100, 20_000
IRREGULAR_RANGE = (36.0, 44.0)
LOCK_START, LOCK_STEPS, LOCK_STEADY, LOCK_HOLD = LEARN_STEPS + 5000, 400, 100, 20
LOCK_Z_SCALE = 3.0
LOCK_FLOOR = 1e-4  # chosen: the floor under "twice the steady error"
TIMED_STEPS = 2000
LONG_BLIND_STEPS = 1500  # stage 3 plays back 30 s at 50 Hz
END = STARTS[-1] + BLIND_STEPS + 1  # one past the last step any row looks at

TAG_SIGNAL, TAG_TRAIN, TAG_BLIND, TAG_CONSTANT, TAG_FILL, TAG_FILL_BLIND = 0, 1, 2, 3, 4, 5
TAG_TEMPO, TAG_IRREGULAR, TAG_LOCK, TAG_TIMED = 6, 7, 8, 9

CONFIGS = {
    "default": Config(n_channels=N_CH, n_latent=N_LAT),
    "plain_core": Config(n_channels=N_CH, n_latent=N_LAT, **PLAIN_CORE),
    "state_brake": Config(n_channels=N_CH, n_latent=N_LAT, brake="state"),
    "one_speed": Config(n_channels=N_CH, n_latent=N_LAT, eta_fast=0.0),
}


def r2(pred, true) -> float:
    return float(1.0 - np.mean((pred - true) ** 2) / np.var(true))


def make_wave(seed, n=N_CH):
    """wave(φ) = amp sin(φ + ph) + HARMONIC amp sin(2φ + ph2), per channel, from default_rng([seed, 0])."""
    rng = np.random.default_rng([seed, TAG_SIGNAL])
    amp, ph, ph2 = rng.uniform(0.3, 1.0, n), rng.uniform(0, 2 * np.pi, n), rng.uniform(0, 2 * np.pi, n)

    def wave(phi):
        phi = np.asarray(phi, dtype=float)[..., None]
        return amp * np.sin(phi + ph) + HARMONIC * amp * np.sin(2 * phi + ph2)
    return wave


def regular(t, period=PERIOD) -> np.ndarray:
    return 2 * np.pi * np.asarray(t, dtype=float) / period


def _train(config, seed, true, noise=NOISE, steps=LEARN_STEPS) -> BodyModel:
    """`steps` learn steps, all channels visible, on true(t) + noise; noise from default_rng([seed, 1])."""
    model = BodyModel(config, seed)
    X = true(np.arange(steps)) + noise * np.random.default_rng([seed, TAG_TRAIN]).standard_normal((steps, N_CH))
    ones = np.ones(N_CH)
    for x in X:
        model.step(x, ones)
    return model


@functools.cache
def trained(name, seed) -> BodyModel:
    wave = make_wave(seed)
    return _train(CONFIGS[name], seed, lambda t: wave(regular(t)))


def _dark_run(model, true, noise, seed, tag, hidden, starts=STARTS, steps=BLIND_STEPS) -> tuple:
    """For each start: reset_state, WARM_STEPS visible (learn=False), then `steps` with `hidden` channels at d = 0
    (x NaN there). Returns (readouts, truths) of shape (starts, steps, channels) and the last visible truths."""
    model = copy.deepcopy(model)
    rng = np.random.default_rng([seed, tag])
    n = model.config.n_channels
    ones, dark = np.ones(n), np.where(hidden, 0.0, 1.0)
    preds, truths, held = [], [], []
    for start in starts:
        model.reset_state()
        warm = true(np.arange(start - WARM_STEPS, start))
        for x in warm + noise * rng.standard_normal(warm.shape):
            model.step(x, ones, learn=False)
        blind = true(np.arange(start, start + steps))
        seen = blind + noise * rng.standard_normal(blind.shape)
        preds.append([model.step(np.where(hidden, np.nan, x), dark, learn=False) for x in seen])
        truths.append(blind)
        held.append(warm[-1])
    return np.array(preds), np.array(truths), np.array(held)


def blind_test(model, true, seed, tag) -> dict:
    """The blind test on the signal true(t): r2_last, r2_fifth_beat, r2_first_beat, r2_hold, and r2 at HORIZONS."""
    preds, truths, held = _dark_run(model, true, NOISE, seed, tag, np.ones(N_CH, dtype=bool))
    fifth = slice(BLIND_STEPS - BEAT_STEPS, BLIND_STEPS)
    out = {"r2_last": r2(preds[:, -1], truths[:, -1]), "r2_fifth_beat": r2(preds[:, fifth], truths[:, fifth]),
           "r2_first_beat": r2(preds[:, :BEAT_STEPS], truths[:, :BEAT_STEPS]),
           "r2_hold": r2(np.broadcast_to(held[:, None], truths[:, fifth].shape), truths[:, fifth]),
           "r2_fifth_worst_channel": min(r2(preds[:, fifth, c], truths[:, fifth, c]) for c in range(N_CH))}
    out.update({h: r2(preds[:, h - 1], truths[:, h - 1]) for h in HORIZONS})
    return out


@functools.cache
def blind(name, seed) -> dict:
    wave = make_wave(seed)
    return blind_test(trained(name, seed), lambda t: wave(regular(t)), seed, TAG_BLIND)


def _fmt(values, spec="+.3f") -> str:
    return " ".join(format(v, spec) for v in values)


def row_beat_1pct():
    beats = [trained("default", s).beat() for s in SEEDS]
    assert all(b is not None for b in beats), f"beat() None: {beats}"
    periods, moduli = [b[0] for b in beats], [b[1] for b in beats]
    text = f"periods {_fmt(periods, '.2f')} | moduli {_fmt(moduli, '.4f')}"
    ok = sum(abs(p - PERIOD) / PERIOD <= BEAT_TOL for p in periods)
    assert ok == len(SEEDS), f"{ok} of {len(SEEDS)} within {BEAT_TOL:.0%}: {text}"
    return f"{ok} of {len(SEEDS)} within {BEAT_TOL:.0%} of {PERIOD:g}: {text}"


def row_blind_5_beats():
    res = [blind("default", s) for s in SEEDS]
    last, fifth = [r["r2_last"] for r in res], [r["r2_fifth_beat"] for r in res]
    text = f"R2 last step {_fmt(last)} | whole 5th beat {_fmt(fifth)}"
    ok = sum(a >= R2_BAR and b >= R2_BAR for a, b in zip(last, fifth))
    assert ok == len(SEEDS), f"{ok} of {len(SEEDS)} >= {R2_BAR}: {text}"
    worst = _fmt(r["r2_fifth_worst_channel"] for r in res)
    return (f"{ok} of {len(SEEDS)} >= {R2_BAR}: {text} (pooled over {N_CH} channels) | worst single channel, "
            f"whole 5th beat {worst} (reported, not gated)")


def row_no_time_fails():
    res = [blind("plain_core", s) for s in SEEDS]
    last, fifth = [r["r2_last"] for r in res], [r["r2_fifth_beat"] for r in res]
    text = f"R2 last step {_fmt(last)} | whole 5th beat {_fmt(fifth)}"
    ok = sum(a < NO_TIME_BAR and b < NO_TIME_BAR for a, b in zip(last, fifth))
    assert ok == len(SEEDS), f"{ok} of {len(SEEDS)} below {NO_TIME_BAR}: {text}"
    return f"plain core: {ok} of {len(SEEDS)} below {NO_TIME_BAR}: {text}"


def row_gate_ran_with_all_changes():
    c = CONFIGS["default"]
    assert c.time and c.brake == "surprise" and c.eta_fast > 0 and c.lam > 0, c
    models = [trained("default", s) for s in SEEDS]
    assert all(m.config is c for m in models), "a gate model was not trained with the default Config"
    assert all(m.C_fast.any() and m.A_fast.any() for m in models), "a fast part is zero after training"
    unsettled = [m.unsettled for m in models]
    assert not any(unsettled), f"unsettled after training: {unsettled}"
    c_ratio = [np.linalg.norm(m.C_fast) / np.linalg.norm(m.C_slow) for m in models]
    a_ratio = [np.linalg.norm(m.A_fast) / np.linalg.norm(m.A_slow) for m in models]
    return (f"time on, brake {c.brake}, eta_fast {c.eta_fast:g}, lam {c.lam:g}; unsettled "
            f"{' '.join(map(str, unsettled))} of {LEARN_STEPS}; |C_fast|/|C_slow| {_fmt(c_ratio, '.4f')}, "
            f"|A_fast|/|A_slow| {_fmt(a_ratio, '.4f')}")


def row_beat_none_without_rhythm():
    beats, eigs = [], []
    for s in SEEDS:
        level = np.random.default_rng([s, TAG_CONSTANT]).uniform(-1, 1, N_CH)
        model = _train(CONFIGS["default"], s, lambda t, level=level: np.broadcast_to(level, (len(t), N_CH)))
        beats.append(model.beat())
        eigs.append(np.linalg.eigvals(model.A))
    nones = sum(b is None for b in beats)
    assert nones == len(SEEDS), f"beat() {beats}; eigenvalues of A: {[np.round(e, 4).tolist() for e in eigs]}"
    worst = max(float(np.max(np.abs(e[np.abs(e.imag) > 1e-6]), initial=0.0)) for e in eigs)
    return f"constant input: beat() None in {nones} of {len(SEEDS)}; largest complex modulus {worst:.4f}"


def _fill_body(seed):
    """Commands wave_cmd(φ_t) (no noise) then sensors W u_{t-DELAY}; returns true(t) and the per-channel noise."""
    wave_cmd = make_wave(seed, N_CMD)
    W = np.random.default_rng([seed, TAG_FILL]).standard_normal((N_SENS, N_CMD)) / np.sqrt(N_CMD)

    def true(t):
        t = np.asarray(t)
        return np.concatenate([wave_cmd(regular(t)), wave_cmd(regular(t - DELAY)) @ W.T], axis=-1)
    noise = np.concatenate([np.zeros(N_CMD), np.full(N_SENS, NOISE)])
    return true, noise


def row_fill_in():
    commands = np.arange(N_CH) < N_CMD
    filled, all_hidden = [], []
    for s in SEEDS:
        true, noise = _fill_body(s)
        model = _train(CONFIGS["default"], s, true, noise)
        for hidden, out in ((commands, filled), (np.ones(N_CH, dtype=bool), all_hidden)):
            preds, truths, _ = _dark_run(model, true, noise, s, TAG_FILL_BLIND, hidden)
            out.append(r2(preds[..., :N_CMD], truths[..., :N_CMD]))
    text = f"commands hidden {_fmt(filled)} | everything hidden {_fmt(all_hidden)} (rhythm memory alone)"
    ok = sum(v >= FILL_BAR for v in filled)
    assert ok == len(SEEDS), f"{ok} of {len(SEEDS)} >= {FILL_BAR}: {text}"
    return (f"{ok} of {len(SEEDS)} >= {FILL_BAR} (our own bar): {text}; the model plays back a remembered rhythm, "
            f"this does not show a command filled in from what the sensors say")


def info_hold_baseline():
    return f"hold last value, R2 over the 5th beat: {_fmt(blind('default', s)['r2_hold'] for s in SEEDS)}"


def info_dark_horizons():
    return " | ".join(f"after {h}: {_fmt(blind('default', s)[h] for s in SEEDS)}" for h in HORIZONS)


def info_state_brake():
    res = [blind("state_brake", s) for s in INFO_SEEDS]
    return (f"seeds {INFO_SEEDS}: R2 first beat {_fmt(r['r2_first_beat'] for r in res)} | "
            f"5th beat {_fmt(r['r2_fifth_beat'] for r in res)}")


def _steps_to_new_tempo(model, seed) -> str:
    model = copy.deepcopy(model)
    wave = make_wave(seed)
    phi0 = regular(LEARN_STEPS)
    rng = np.random.default_rng([seed, TAG_TEMPO])
    ones = np.ones(N_CH)
    for k in range(1, TEMPO_MAX_STEPS + 1):
        t = LEARN_STEPS + k - 1
        model.step(wave(phi0 + regular(t - LEARN_STEPS, NEW_PERIOD)) + NOISE * rng.standard_normal(N_CH), ones)
        if k % TEMPO_LOOK_EVERY == 0:
            b = model.beat()
            if b is not None and abs(b[0] - NEW_PERIOD) / NEW_PERIOD <= BEAT_TOL:
                return str(k)
    return "never"


def info_tempo_change():
    parts = []
    for name in ("default", "one_speed"):
        steps = [_steps_to_new_tempo(trained(name, s), s) for s in INFO_SEEDS]
        parts.append(f"{name} {' '.join(steps)}")
    return f"steps until beat() within {BEAT_TOL:.0%} of {NEW_PERIOD:g}, seeds {INFO_SEEDS}: " + " | ".join(parts)


def info_irregular_beat():
    periods, first = [], []
    for s in INFO_SEEDS:
        cycles = np.random.default_rng([s, TAG_IRREGULAR]).uniform(*IRREGULAR_RANGE, int(END / IRREGULAR_RANGE[0]) + 1)
        ends = np.cumsum(cycles)  # cycle k runs from ends[k - 1] to ends[k] steps, phase 2π k to 2π (k + 1)
        steps = np.arange(END)
        k = np.searchsorted(ends, steps, side="right")
        phase = 2 * np.pi * (k + (steps - np.concatenate([[0.0], ends])[k]) / cycles[k])
        wave = make_wave(s)
        true = lambda t, phase=phase, wave=wave: wave(phase[np.asarray(t)])  # noqa: E731
        model = _train(CONFIGS["default"], s, true)
        b = model.beat()
        periods.append(float("nan") if b is None else b[0])
        first.append(blind_test(model, true, s, TAG_BLIND)["r2_first_beat"])
    return f"cycles U{IRREGULAR_RANGE}, seeds {INFO_SEEDS}: period {_fmt(periods, '.2f')} | R2 first beat {_fmt(first)}"


def info_lock_on():
    locks = []
    for s in SEEDS:
        model = copy.deepcopy(trained("default", s))
        wave = make_wave(s)
        rng = np.random.default_rng([s, TAG_LOCK])
        model.z = LOCK_Z_SCALE * rng.standard_normal(N_LAT)
        ones, err = np.ones(N_CH), []
        for t in range(LOCK_START, LOCK_START + LOCK_STEPS):
            true = wave(regular(t))
            err.append(float(np.mean((model.C @ (model.A @ model.z) - true) ** 2)))
            model.step(true + NOISE * rng.standard_normal(N_CH), ones, learn=False)
        bound = 2 * np.mean(err[-LOCK_STEADY:]) + LOCK_FLOOR
        locks.append(next(i for i in range(len(err)) if all(e < bound for e in err[i:i + LOCK_HOLD])))
    return f"steps from z = {LOCK_Z_SCALE:g} randn until locked: {' '.join(map(str, locks))}"


def info_step_time():
    model = copy.deepcopy(trained("default", SEEDS[0]))
    wave = make_wave(SEEDS[0])
    t = np.arange(LEARN_STEPS, LEARN_STEPS + TIMED_STEPS)
    X = wave(regular(t)) + NOISE * np.random.default_rng([SEEDS[0], TAG_TIMED]).standard_normal((TIMED_STEPS, N_CH))
    ones = np.ones(N_CH)
    start = time.perf_counter()
    for x in X:
        model.step(x, ones)
    us = (time.perf_counter() - start) / TIMED_STEPS * 1e6
    return f"{us:.0f} µs per learn step ({N_CH} channels, {N_LAT} latents), 1-minute load {os.getloadavg()[0]:.2f}"


def info_one_speed():
    one = [blind("one_speed", s) for s in INFO_SEEDS]
    two = [blind("default", s) for s in INFO_SEEDS]
    return (f"seeds {INFO_SEEDS}, R2 last step / whole 5th beat: one speed {_fmt(r['r2_last'] for r in one)} / "
            f"{_fmt(r['r2_fifth_beat'] for r in one)} | default {_fmt(r['r2_last'] for r in two)} / "
            f"{_fmt(r['r2_fifth_beat'] for r in two)}")


def info_long_blind():
    """The blind test's warm-up (TAG_BLIND noise) at the first start, then LONG_BLIND_STEPS blind; the last beat."""
    last_beat = slice(LONG_BLIND_STEPS - BEAT_STEPS, LONG_BLIND_STEPS)
    fits, ratios = [], []
    for s in SEEDS:
        wave = make_wave(s)
        preds, truths, _ = _dark_run(trained("default", s), lambda t: wave(regular(t)), NOISE, s, TAG_BLIND,
                                     np.ones(N_CH, dtype=bool), starts=STARTS[:1], steps=LONG_BLIND_STEPS)
        pred, true = preds[:, last_beat], truths[:, last_beat]
        fits.append(r2(pred, true))
        ratios.append(float(np.sqrt(np.mean(pred ** 2)) / np.sqrt(np.mean(true ** 2))))
    return (f"{LONG_BLIND_STEPS} blind steps from step {STARTS[0]}, last beat (steps {LONG_BLIND_STEPS - BEAT_STEPS + 1}"
            f"–{LONG_BLIND_STEPS}): R2 {_fmt(fits)} | RMS readout / RMS signal {_fmt(ratios, '.3f')}")


if __name__ == "__main__":
    sys.exit(run((
        row_beat_1pct, row_blind_5_beats, row_no_time_fails, row_gate_ran_with_all_changes,
        row_beat_none_without_rhythm, row_fill_in,
        info_hold_baseline, info_dark_horizons, info_state_brake, info_tempo_change, info_irregular_beat,
        info_lock_on, info_step_time, info_one_speed, info_long_blind,
    )))
