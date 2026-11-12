# SPDX-License-Identifier: MIT
"""Stage 2: the mechanics of the Puls stream model (`puls/model.py`) on small hand-made inputs:
shapes, the step against the core, the local rules, masks, refusals, divergence, a long blind stretch, the unsettled
count and the standalone imports. Exit 0 only when every row passes.

    cd <repository root> && sim/.venv/bin/python sim/checks/check_e2_model.py
"""

import ast
import copy
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _rows import run  # noqa: E402
from puls.core import BRAKES, settle  # noqa: E402
from puls.model import PLAIN_CORE, BodyModel, Config  # noqa: E402

N_CH, N_LAT = 5, 3
SMALL = Config(n_channels=N_CH, n_latent=N_LAT)
STEP_TOL = 1e-12
WARM_STEPS = 3
SINE_PERIOD = 20.0
DIVERGE_SCALE = 1000.0
DIVERGE_STEPS = 2000
FORCED_X = 1e200
BLIND_LEARN_STEPS = 200
BLIND_DARK_STEPS = 10_000
UNSETTLED_STEPS = 20
NO_LEARN_STEPS = 50
PACKAGE = Path(__file__).resolve().parents[1] / "puls"


def _sine(t, n=N_CH, scale=1.0) -> np.ndarray:
    """A hand-made rhythm: channel i is sin(2π t / SINE_PERIOD + i)."""
    return scale * np.sin(2 * np.pi * t / SINE_PERIOD + np.arange(n))


def _weights(model) -> tuple:
    return tuple(a.copy() for a in (model.C_slow, model.C_fast, model.A_slow, model.A_fast))


def _state(model) -> tuple:
    return (*_weights(model), model.z.copy(), model.unsettled)


def _same(a, b) -> bool:
    return all(np.array_equal(u, v) for u, v in zip(a, b))


def _warmed(config=SMALL, seed=0, steps=WARM_STEPS, learn=True) -> BodyModel:
    model = BodyModel(config, seed)
    for t in range(steps):
        model.step(_sine(t, config.n_channels), np.ones(config.n_channels), learn=learn)
    return model


def row_shapes_and_seed():
    m = BodyModel(SMALL, 0)
    shapes = [m.C_slow.shape, m.C_fast.shape, m.A_slow.shape, m.A_fast.shape, m.z.shape, m.C.shape, m.A.shape]
    want = [(N_CH, N_LAT)] * 2 + [(N_LAT, N_LAT)] * 2 + [(N_LAT,), (N_CH, N_LAT), (N_LAT, N_LAT)]
    assert shapes == want, shapes
    assert np.array_equal(m.C_slow, BodyModel(SMALL, 0).C_slow), "same seed, different C_slow"
    assert not np.array_equal(m.C_slow, BodyModel(SMALL, 1).C_slow), "another seed, same C_slow"
    assert np.array_equal(m.A_slow, 0.5 * np.eye(N_LAT)), m.A_slow
    assert not (m.C_fast.any() or m.A_fast.any() or m.z.any()) and m.unsettled == 0
    assert not BodyModel(Config(n_channels=N_CH, n_latent=N_LAT, **PLAIN_CORE)).A_slow.any(), "A_slow without time"
    refused = 0
    for over in ({"n_channels": 0}, {"brake": "x"}, {"n_latent": 0}, {"eta_slow": -0.1}, {"eta_fast": -0.1},
                 {"fast_decay": 1.5}, {"fast_decay": -0.1}, {"max_iters": 0},
                 {"tol": 0}, {"tol": -1}, {"tol": np.nan}, {"tol": np.inf}, {"tol": np.array(1e-7)},
                 {"lam": -0.1}, {"lam": np.nan}, {"lam": np.inf}, {"lam": "0.1"}, {"lam": np.array(0.1)},
                 {"init_scale": np.nan}, {"init_scale": np.inf}, {"a_init": np.nan}, {"a_init": "0.5"},
                 {"time": "no"}, {"time": 1}):
        (name,) = over
        try:
            Config(**{"n_channels": N_CH, **over})
        except ValueError as e:
            assert re.search(rf"\b{name}\b", str(e)), f"Config({over}): message {str(e)!r}"
            refused += 1
        else:
            raise AssertionError(f"Config({over}) accepted")
    return (f"shapes ok, seed reproducible, A_slow = 0.5 I (0 without time), {refused} bad configs refused, each a "
            f"ValueError naming the field")


def row_step_matches_core():
    x = _sine(7.0) + 0.3
    d = np.array([1.0, 0.0, 2.5, 1.0, 1.0])
    worst = 0.0
    for brake in BRAKES:
        for time in (True, False):
            config = Config(n_channels=N_CH, n_latent=N_LAT, brake=brake, time=time)
            model = _warmed(config, learn=False)
            assert model.z.any(), "z still zero after the warm-up"
            C, A = model.C.copy(), model.A.copy()
            p = A @ model.z if time else np.zeros(N_LAT)
            want = C @ settle(p, p, x, d, C, config.lam, tol=config.tol, max_iters=config.max_iters,
                              brake=brake)["z"]
            got = model.step(x, d, learn=False)
            worst = max(worst, float(np.max(np.abs(got - want))))
    assert worst <= STEP_TOL, worst
    return f"2 brakes x time on/off, d with a 0 and a 2.5: worst |step - C settle| = {worst:.1e}"


def row_learn_false_changes_nothing():
    model = _warmed(steps=10)
    assert model.C_fast.any() and model.A_fast.any(), "fast parts zero: a decay would not show"
    before, z_before = _weights(model), model.z.copy()
    for t in range(NO_LEARN_STEPS):
        model.step(_sine(t + 0.5), np.ones(N_CH), learn=False)
    assert _same(before, _weights(model)), "a weight changed or decayed with learn=False"
    assert not np.array_equal(z_before, model.z), "z did not change"
    return (f"learned model (|C_fast| {np.linalg.norm(model.C_fast):.3f}, |A_fast| {np.linalg.norm(model.A_fast):.3f}),"
            f" {NO_LEARN_STEPS} steps: 4 weight arrays bit-identical, z changed")


def _rule_gaps(config):
    model = _warmed(config, steps=10)
    x, d = _sine(10.5) - 0.2, np.array([1.0, 2.0, 1.0, 0.5, 1.0])
    C, A, z_prev = model.C.copy(), model.A.copy(), model.z.copy()
    old = _weights(model)
    readout = model.step(x, d)
    z = model.z
    readout_gap = float(np.max(np.abs(readout - C @ z)))  # the readout uses the weights that produced z
    err_c, err_a = d * (x - C @ z), z - A @ z_prev
    eta_s, eta_f, keep = config.eta_slow, config.eta_fast, 1 - config.fast_decay
    want = (old[0] + eta_s * np.outer(err_c, z), keep * old[1] + eta_f * np.outer(err_c, z),
            old[2] + eta_s * np.outer(err_a, z_prev), keep * old[3] + eta_f * np.outer(err_a, z_prev))
    if not config.time:
        want = want[:2] + old[2:]
    return model, max(readout_gap, *(float(np.max(np.abs(w - g))) for w, g in zip(want, _weights(model))))


def row_local_rules():
    model, gap = _rule_gaps(SMALL)
    assert model.C_fast.any() and model.A_fast.any(), "fast parts zero: the rule is not tested"
    assert gap <= STEP_TOL, gap
    as_is, gap_as_is = _rule_gaps(Config(n_channels=N_CH, n_latent=N_LAT, **PLAIN_CORE))
    assert gap_as_is <= STEP_TOL, gap_as_is
    assert not (as_is.A_slow.any() or as_is.A_fast.any() or as_is.C_fast.any()), "PLAIN_CORE grew time or fast parts"
    return (f"C_slow, C_fast, A_slow, A_fast = their local rule and readout = old C z within {max(gap, gap_as_is):.1e};"
            " PLAIN_CORE: A and fast 0")


def row_hidden_channel_never_teaches():
    hidden = np.array([1, 3])
    d = np.ones(N_CH)
    d[hidden] = 0.0
    x = _sine(4.2)
    a = _warmed(steps=10)
    b = copy.deepcopy(a)
    old = _weights(a)
    x0, xnan = x.copy(), x.copy()
    x0[hidden], xnan[hidden] = 0.0, np.nan
    out0, outnan = a.step(x0, d), b.step(xnan, d)
    assert np.array_equal(a.C_slow[hidden], old[0][hidden]), "a hidden row of C_slow changed"
    assert np.array_equal(a.C_fast[hidden], (1 - SMALL.fast_decay) * old[1][hidden]), "hidden C_fast row"
    assert out0.shape == (N_CH,) and np.all(np.isfinite(out0)), out0
    assert np.array_equal(out0, outnan) and _same(_state(a), _state(b)), "NaN at a hidden channel changed something"
    return f"2 hidden rows: C_slow bit-identical, C_fast decayed only; readout {N_CH} finite; NaN there = 0 there"


def row_reset_state_keeps_weights():
    model = _warmed(steps=10)
    before = _weights(model)
    model.reset_state()
    assert not model.z.any() and _same(before, _weights(model)), "reset_state"
    return "z = 0, weights bit-identical"


def row_visible_nan_refused():
    model = _warmed(steps=10)
    before = _state(model)
    nan_x, inf_x, neg_d, nan_d = _sine(1.0), _sine(1.0), np.ones(N_CH), np.ones(N_CH)
    nan_x[0], inf_x[2], neg_d[4], nan_d[1] = np.nan, np.inf, -0.5, np.nan
    cases = [(nan_x, np.ones(N_CH)), (inf_x, np.ones(N_CH)), (_sine(1.0, N_CH + 1), np.ones(N_CH)),
             (_sine(1.0), np.ones(N_CH - 1)), (_sine(1.0), neg_d), (_sine(1.0), nan_d),
             (_sine(1.0).reshape(1, -1), np.ones((1, N_CH)))]
    for x, d in cases:
        try:
            model.step(x, d)
        except ValueError:
            pass
        else:
            raise AssertionError(f"accepted x={x}, d={d}")
        assert _same(before, _state(model)), "a refused call changed the model"
    return f"{len(cases)} bad calls refused, weights, z and unsettled bit-identical"


def _forced_overflow(config=SMALL, converged=True) -> str:
    """A visible x = FORCED_X makes the learn step overflow: FloatingPointError, model bit-identical to before. The
    settle of that step converges or not, as `converged` says (checked against the core with the model's inputs)."""
    model = _warmed(config, steps=10)
    before = _state(model)
    x = _sine(10.0)
    x[0] = FORCED_X
    p = model.A @ model.z
    with np.errstate(over="ignore", invalid="ignore", divide="ignore", under="ignore"):  # as in BodyModel.step
        settled = settle(p, p, x, np.ones(N_CH), model.C, config.lam, tol=config.tol, max_iters=config.max_iters,
                         brake=config.brake)["converged"]
    assert settled is converged, f"x = {FORCED_X:g}: settle converged {settled}, wanted {converged}"
    try:
        model.step(x, np.ones(N_CH))
    except FloatingPointError:
        assert _same(before, _state(model)), "FloatingPointError left a changed model"
        return f"settle converged {settled}: FloatingPointError, weights, z and unsettled bit-identical"
    raise AssertionError(f"x = {FORCED_X:g} (settle converged {settled}) did not raise FloatingPointError")


def row_diverging_input_never_silent():
    forced = (_forced_overflow(), _forced_overflow(Config(n_channels=N_CH, n_latent=N_LAT, max_iters=1, tol=1e-12),
                                                   converged=False))
    forced = f"x = {FORCED_X:g}: default ({forced[0]}); max_iters=1, tol=1e-12 ({forced[1]})"
    model = BodyModel(SMALL, 0)
    for t in range(DIVERGE_STEPS):
        before = _state(model)
        try:
            model.step(_sine(t, scale=DIVERGE_SCALE), np.ones(N_CH))
        except FloatingPointError:
            assert _same(before, _state(model)), "FloatingPointError left a changed model"
            return (f"FloatingPointError at step {t}, model as before that call, unsettled {model.unsettled} of {t};"
                    f" {forced}")
        assert all(np.all(np.isfinite(a)) for a in _state(model)[:5]), f"non-finite at step {t}"
    return (f"finite to the end ({DIVERGE_STEPS} steps at scale {DIVERGE_SCALE:g}, |C| = {np.linalg.norm(model.C):.3g},"
            f" unsettled {model.unsettled} of {DIVERGE_STEPS}: scaling to about ±1 is the caller's job); {forced}")


def row_blind_learning_stays_finite():
    model = _warmed(steps=BLIND_LEARN_STEPS)
    C_slow, A_slow = model.C_slow.copy(), model.A_slow.copy()
    fast = np.linalg.norm(model.C_fast), np.linalg.norm(model.A_fast)
    dark_x, dark_d = np.full(N_CH, np.nan), np.zeros(N_CH)
    for _ in range(BLIND_DARK_STEPS):
        out = model.step(dark_x, dark_d)
    assert np.array_equal(C_slow, model.C_slow) and np.array_equal(A_slow, model.A_slow), "slow weights moved blind"
    after = np.linalg.norm(model.C_fast), np.linalg.norm(model.A_fast)
    assert after[0] < fast[0] and after[1] < fast[1], (fast, after)
    assert all(np.all(np.isfinite(a)) for a in (*_weights(model), model.z, out)), "non-finite after the blind stretch"
    return (f"{BLIND_DARK_STEPS} blind learn steps: slow bit-identical, |C_fast| {fast[0]:.2e} -> {after[0]:.2e}, "
            f"|A_fast| {fast[1]:.2e} -> {after[1]:.2e}, finite")


def row_unsettled_counted():
    tight = BodyModel(Config(n_channels=N_CH, n_latent=N_LAT, max_iters=1, tol=1e-12), 0)
    default = BodyModel(SMALL, 0)
    for t in range(UNSETTLED_STEPS):
        for model in (tight, default):
            model.step(_sine(t), np.ones(N_CH))
    assert tight.unsettled > 0 and default.unsettled == 0, (tight.unsettled, default.unsettled)
    return f"max_iters=1, tol=1e-12: {tight.unsettled} of {UNSETTLED_STEPS} unsettled; default: 0"


def row_standalone():
    allowed = set(sys.stdlib_module_names) | {"numpy", "puls"}
    files = sorted(PACKAGE.glob("*.py"))
    for f in files:
        for node in ast.walk(ast.parse(f.read_text())):
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or "puls"] if node.level == 0 else ["puls"]
            else:
                continue
            bad = [n for n in names if n.split(".")[0] not in allowed]
            assert not bad, f"{f.name} imports {bad}"
    return f"{len(files)} files import only numpy, the standard library and puls"


if __name__ == "__main__":
    sys.exit(run((
        row_shapes_and_seed, row_step_matches_core, row_learn_false_changes_nothing, row_local_rules,
        row_hidden_channel_never_teaches, row_reset_state_keeps_weights, row_visible_nan_refused,
        row_diverging_input_never_silent, row_blind_learning_stays_finite, row_unsettled_counted, row_standalone,
    )))
