"""The Monte Carlo's winner polished, and the same search under the constraints a builder and the nose lift
put on it. Each case: a Monte Carlo of 4000 samples inside its own box and constraints, then Nelder-Mead from
the best five, keeping the best. Score = weakest axis (rad/s^2); a constraint broken scores below zero.

  A  free             the full box, nothing but hover trim 5-85%            (the unconstrained optimum)
  B  keep pitch       as A, pitch authority >= 17 rad/s^2 (V4 has 20.3)    (the nose lift needs pitch)
  C  buildable        as B, every angle within +/-45 deg of vertical, and the vertical pair's cant within
                      +/-30 deg (two 80 mm ducts 104 mm apart)
"""
from __future__ import annotations

import json, sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mc_front_tilt as mc                                         # noqa: E402
from airframe_designer.analysis.geometric_optimiser import nelder_mead   # noqa: E402

CASES = {
    "A_free": dict(lo=mc.LO, hi=mc.HI, pitch_min=0.0),
    "B_keep_pitch": dict(lo=mc.LO, hi=mc.HI, pitch_min=17.0),
    "C_buildable": dict(lo=np.array([-45.0, -30.0, -30.0, -30.0]), hi=np.array([30.0, 30.0, 30.0, 30.0]), pitch_min=17.0),
}


def score(a, pitch_min):
    if not a["ok"]:
        return -1.0
    short = pitch_min - a["pitch"]["alpha"]
    return a["min_alpha"] if short <= 0 else -short


def run(name, lo, hi, pitch_min, n=4000, seed=3):
    rng = np.random.default_rng(seed)
    X = lo + (hi - lo) * rng.random((n, 4))
    res = sorted(((score(a, pitch_min), a) for a in (mc.evaluate(x) for x in X)), key=lambda t: -t[0])
    best = res[0]
    for s0, a0 in res[:5]:
        x, f = nelder_mead(lambda x: -score(mc.evaluate(x), pitch_min), np.array(a0["x"]), lo, hi,
                           np.full(4, 4.0), max_eval=400)
        if -f > best[0]:
            best = (-f, mc.evaluate(x))
    a = best[1]
    print(f"{name:13s} {mc.row(a)}  side force {a['yaw']['side_force_per_Nm']:.2f} N per N m of yaw", flush=True)
    return a


if __name__ == "__main__":
    out = {"as_drawn": mc.evaluate([-30.0, 0.0, 0.0, 0.0])}
    print(f"{'as drawn':13s} {mc.row(out['as_drawn'])}")
    for k, c in CASES.items():
        out[k] = run(k, **c)
        out[k]["robustness"] = mc.robustness(out[k]["x"], n=1000)
    json.dump(out, open(Path(__file__).resolve().parent / "mc_variants.json", "w"), indent=1, default=float)
    for k in CASES:
        r = out[k]["robustness"]
        print(f"{k:13s} as built (p5/p50): " + "  ".join(f"{ax} {r[ax]['p5']:.2f}/{r[ax]['p50']:.2f}" for ax in ("roll", "pitch", "yaw")))
