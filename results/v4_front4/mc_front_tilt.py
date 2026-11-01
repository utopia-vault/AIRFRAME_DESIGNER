"""Monte Carlo over the front motors' angles of the four-front-motor V4, towards the best per-axis authority.

Four angles, each pair kept mirrored about the centreline (the app's tilt / cant: tilt forward from vertical,
cant outward from the centreline, so V4's canted pair is at cant -30):
  side_cant, side_tilt   M7 / M8, the outer pair at x 0.255
  vert_cant, vert_tilt   M9 / M10, the side-by-side pair at x 0.355
For every sample the hover pitch is re-solved (the card follows the aircraft, as the export would), the trim
must leave every motor between 5% and 85%, and the score is the weakest axis's angular acceleration
(front4.authority). Rounds of uniform samples, each round narrowed around the best so far, then a
robustness pass on the winner: mounting errors, CG and thrust scattered and the authority taken again.
"""
from __future__ import annotations

import json, math, sys, time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from front4 import OUT, ROOT, authority, natural_hover_pitch       # noqa: E402
from airframe_designer.geometry.airframe import Airframe          # noqa: E402
from airframe_designer.geometry.frames import tilt_cant_to_axis   # noqa: E402

NAMES = ("side_cant", "side_tilt", "vert_cant", "vert_tilt")
LO = np.array([-60.0, -30.0, -60.0, -30.0])
HI = np.array([30.0, 30.0, 60.0, 30.0])
UTIL_MIN, UTIL_MAX = 0.05, 0.85
SIDE, VERT = ("M7", "M8"), ("M9", "M10")

base = Airframe.load(OUT)
base.resolve_mass()


def apply(af: Airframe, x) -> Airframe:
    for r in af.rotors:
        if r.name in SIDE:
            r.axis = tilt_cant_to_axis(x[1], x[0], r.side)
        elif r.name in VERT:
            r.axis = tilt_cant_to_axis(x[3], x[2], r.side)
    af.hover_pitch_deg = natural_hover_pitch(af)
    return af


def evaluate(x) -> dict:
    a = authority(apply(base, x))
    ok = UTIL_MIN <= a["min_util"] and a["max_util"] <= UTIL_MAX
    a["ok"] = ok
    a["score"] = a["min_alpha"] if ok else -1.0
    a["x"] = [float(v) for v in x]
    return a


def row(a) -> str:
    x = a["x"]
    return (f"side cant {x[0]:6.1f} tilt {x[1]:5.1f} | vert cant {x[2]:6.1f} tilt {x[3]:5.1f} | hover {a['hover_pitch_deg']:5.2f} | "
            f"roll {a['roll']['alpha']:5.2f} pitch {a['pitch']['alpha']:5.2f} yaw {a['yaw']['alpha']:5.2f} rad/s^2 | "
            f"util {a['min_util']:.2f}-{a['max_util']:.2f}")


def search(rounds=6, n=3000, seed=1):
    rng = np.random.default_rng(seed)
    lo, hi = LO.copy(), HI.copy()
    best, log = None, []
    for k in range(rounds):
        t0 = time.time()
        X = lo + (hi - lo) * rng.random((n, 4))
        res = [evaluate(x) for x in X]
        ok = [r for r in res if r["ok"]]
        top = max(res, key=lambda r: r["score"])
        if best is None or top["score"] > best["score"]:
            best = top
        log.append({"round": k, "box": [lo.tolist(), hi.tolist()], "n": n, "feasible": len(ok), "best": best})
        print(f"round {k}: {len(ok)}/{n} feasible, {time.time() - t0:.0f} s   best: {row(best)}", flush=True)
        # narrow: the box of the top 2% this round, never tighter than +/-2 deg around the best, inside the limits
        elite = np.array([r["x"] for r in sorted(ok, key=lambda r: -r["score"])[:max(5, n // 50)]])
        half = np.maximum((elite.max(0) - elite.min(0)) / 2 * 1.2, 2.0)
        c = np.array(best["x"])
        lo, hi = np.maximum(LO, c - half), np.minimum(HI, c + half)
        if k == 0:
            samples = res
    return best, log, samples


def robustness(x, n=2000, seed=7):
    """The winner as built rather than as drawn: each rotor +/-2 deg of tilt and cant error, the CG +/-10 mm in
    x and z, +/-3 mm in y, and every motor's thrust +/-5%. PX4 keeps the design's card; the authority is
    taken from the aircraft as built."""
    rng = np.random.default_rng(seed)
    nominal = apply(base.copy(), x)
    nominal.resolve_mass()
    card = nominal.effectiveness()
    out = []
    from airframe_designer.constants import G
    for _ in range(n):
        af = nominal.copy()
        af.resolve_mass()
        for r in af.rotors:
            r.tilt_error_deg, r.cant_error_deg = rng.normal(0, 1.0, 2).clip(-2, 2)
            r.max_thrust *= 1 + rng.normal(0, 0.025)
        af.mass.cg = (np.array(af.mass.cg) + rng.normal(0, [0.005, 0.0015, 0.005]).clip(-0.01, 0.01)).tolist()
        Et = af.effectiveness(built=True)
        A = np.linalg.pinv(card[[0, 1, 2, 5]])
        tmax = np.array([r.effective_max_thrust() for r in af.rotors])
        # trim on the real aircraft: what PX4 ends up asking for to hold zero torque and the weight
        M = Et[[0, 1, 2, 5]] @ A
        c = np.linalg.solve(M, np.array([0, 0, 0, -af.mass.mass * G]))
        t = A @ c
        if t.min() < 0 or (t / tmax).max() > 1:
            out.append({"ok": False})
            continue
        R = af.hover_rotation()
        Ii = np.linalg.inv(R @ af.mass.tensor() @ R.T)
        rec = {"ok": True, "max_util": float((t / tmax).max())}
        for k, ax in enumerate(("roll", "pitch", "yaw")):
            e = np.zeros(4); e[k] = 1.0
            dt = A @ e
            ways = []
            for s in (1, -1):
                d = s * dt
                with np.errstate(divide="ignore", invalid="ignore"):
                    room = np.where(d > 1e-12, (tmax - t) / d, np.where(d < -1e-12, -t / d, np.inf))
                ways.append(max(0.0, float(room.min())))
            rec[ax] = min(ways) * float((Ii @ Et[0:3] @ dt)[k])
        out.append(rec)
    good = [o for o in out if o["ok"]]
    summ = {"n": n, "hover_ok": len(good)}
    for ax in ("roll", "pitch", "yaw"):
        v = np.array([o[ax] for o in good])
        summ[ax] = {"p5": float(np.percentile(v, 5)), "p50": float(np.median(v)), "p95": float(np.percentile(v, 95))}
    v = np.array([o["max_util"] for o in good])
    summ["max_util"] = {"p50": float(np.median(v)), "p95": float(np.percentile(v, 95))}
    return summ


if __name__ == "__main__":
    as_cad = evaluate([-30.0, 0.0, 0.0, 0.0])
    print("as drawn in Fusion:", row(as_cad))
    best, log, samples = search()
    print("\nBEST:", row(best))
    rob = robustness(best["x"])
    rob_cad = robustness(as_cad["x"])
    print("robustness (5th / 50th / 95th percentile, rad/s^2):")
    for name, r in (("as drawn", rob_cad), ("best", rob)):
        print(f"  {name:9s} hover ok {r['hover_ok']}/{r['n']}  " + "  ".join(
            f"{ax} {r[ax]['p5']:.2f}/{r[ax]['p50']:.2f}/{r[ax]['p95']:.2f}" for ax in ("roll", "pitch", "yaw")))
    here = Path(__file__).resolve().parent
    json.dump({"as_drawn": as_cad, "best": best, "rounds": log, "robustness": {"as_drawn": rob_cad, "best": rob},
               "first_round_samples": [{"x": s["x"], "ok": s["ok"], "roll": s["roll"]["alpha"], "pitch": s["pitch"]["alpha"],
                                        "yaw": s["yaw"]["alpha"], "max_util": s["max_util"], "hover": s["hover_pitch_deg"]}
                                       for s in samples]},
              open(here / "mc_front_tilt.json", "w"), indent=1, default=float)
