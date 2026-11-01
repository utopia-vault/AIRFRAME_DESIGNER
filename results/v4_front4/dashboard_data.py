"""Data files for the authority dashboard: the 5-degree grid over the four front angles, the Monte Carlo cloud,
the options, the rotors, the mass and the convergence, all from the same model as mc_front_tilt.py."""
from __future__ import annotations

import csv, json, sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
OUT = HERE / "dashboard"


def _eval(x):
    import mc_front_tilt as mc
    a = mc.evaluate(x)
    return [int(v) for v in x] + [round(a["hover_pitch_deg"], 2), round(a["roll"]["alpha"], 2), round(a["pitch"]["alpha"], 2),
                                  round(a["yaw"]["alpha"], 2), round(a["min_util"], 3), round(a["max_util"], 3), int(a["ok"])]


def grid():
    import mc_front_tilt as mc
    axes = [np.arange(lo, hi + 0.1, 5.0) for lo, hi in zip(mc.LO, mc.HI)]
    pts = [(a, b, c, d) for a in axes[0] for b in axes[1] for c in axes[2] for d in axes[3]]
    with Pool(8) as p:
        rows = p.map(_eval, pts, chunksize=200)
    with open(OUT / "grid.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sc", "st", "vc", "vt", "hover", "roll", "pitch", "yaw", "umin", "umax", "ok"])
        w.writerows(rows)
    print("grid", len(rows))


def rest():
    import mc_front_tilt as mc
    from front4 import BASE, authority, natural_hover_pitch
    from airframe_designer.geometry.airframe import Airframe
    mcj = json.load(open(HERE / "mc_front_tilt.json"))
    var = json.load(open(HERE / "mc_variants.json"))

    with open(OUT / "mc_samples.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sample", "sc", "st", "vc", "vt", "hover", "roll", "pitch", "yaw", "umax", "ok"])
        for i, s in enumerate(mcj["first_round_samples"]):
            w.writerow([i] + [round(v, 1) for v in s["x"]] + [round(s["hover"], 2), round(s["roll"], 2), round(s["pitch"], 2),
                                                               round(s["yaw"], 2), round(s["max_util"], 3), int(s["ok"])])

    v4 = Airframe.load(BASE); v4.resolve_mass(); v4.hover_pitch_deg = natural_hover_pitch(v4)
    a4 = authority(v4)
    opts = [{"id": "v4_today", "label": "V4 today (3 front motors)", "sc": -30, "st": 0, "vc": None, "vt": None,
             "hover": round(a4["hover_pitch_deg"], 2), "roll": round(a4["roll"]["alpha"], 2), "pitch": round(a4["pitch"]["alpha"], 2),
             "yaw": round(a4["yaw"]["alpha"], 2), "umin": round(a4["min_util"], 3), "umax": round(a4["max_util"], 3),
             "roll_p5": None, "pitch_p5": None, "yaw_p5": None}]
    labels = {"as_drawn": "Four front, as drawn", "A_free": "A: best weakest axis", "B_keep_pitch": "B: pitch kept at 17+",
              "C_buildable": "C: buildable (recommended)"}
    rob = {"as_drawn": mcj["robustness"]["as_drawn"]}
    for k in labels:
        a = var[k]
        r = a.get("robustness") or rob.get(k)
        opts.append({"id": k, "label": labels[k], "sc": round(a["x"][0], 1), "st": round(a["x"][1], 1), "vc": round(a["x"][2], 1),
                     "vt": round(a["x"][3], 1), "hover": round(a["hover_pitch_deg"], 2), "roll": round(a["roll"]["alpha"], 2),
                     "pitch": round(a["pitch"]["alpha"], 2), "yaw": round(a["yaw"]["alpha"], 2), "umin": round(a["min_util"], 3),
                     "umax": round(a["max_util"], 3), "roll_p5": round(r["roll"]["p5"], 2), "pitch_p5": round(r["pitch"]["p5"], 2),
                     "yaw_p5": round(r["yaw"]["p5"], 2)})
    json.dump(opts, open(OUT / "options.json", "w"), indent=1)

    af = mc.base
    rot = [{"name": r.name, "group": ("side" if r.name in mc.SIDE else "vert" if r.name in mc.VERT else "rear"),
            "x": round(r.pos[0], 4), "y": round(r.pos[1], 4), "z": round(r.pos[2], 4),
            "tilt": round(r.tilt_deg, 1), "cant": round(r.cant_deg, 1), "max_thrust_n": r.max_thrust} for r in af.rotors]
    rot.append({"name": "CG", "group": "cg", "x": round(af.mass.cg[0], 4), "y": round(af.mass.cg[1], 4), "z": round(af.mass.cg[2], 4),
                "tilt": None, "cant": None, "max_thrust_n": None})
    json.dump(rot, open(OUT / "rotors.json", "w"), indent=1)

    mp = []
    for name, a in (("V4 today (3 front motors)", v4), ("V4 four front motors", af)):
        m = a.mass
        mp.append({"design": name, "mass_kg": round(m.mass, 3), "cg_x": round(m.cg[0], 4), "cg_y": round(m.cg[1], 4),
                   "cg_z": round(m.cg[2], 4), "ixx": round(m.inertia[0], 3), "iyy": round(m.inertia[1], 3), "izz": round(m.inertia[2], 3),
                   "thrust_n": round(a.total_max_thrust(), 1), "weight_n": round(m.mass * 9.80665, 1)})
    json.dump(mp, open(OUT / "mass_props.json", "w"), indent=1)
    changes = [
        {"change": "4th battery pack", "mass_kg": 0.700, "where": "beside the three, x -0.145 (assumed: not in the CAD)", "source": "BATTERY_3PACK__M700g label"},
        {"change": "2nd vertical front EDF (M10)", "mass_kg": 0.330, "where": "x 0.355, y +0.052", "source": "XFLY 80mm EDF__M330g label, Fusion position"},
        {"change": "Extra front ESC", "mass_kg": 0.200, "where": "x 0.355, y +0.136", "source": "ESC__M200g label, Fusion position"},
        {"change": "M9 moved to y -0.052, 1 cm higher", "mass_kg": 0.0, "where": "x 0.355", "source": "Fusion position"},
        {"change": "Centre front ESC moved", "mass_kg": 0.0, "where": "x 0.198 to x 0.355, y -0.136", "source": "Fusion position"},
    ]
    json.dump(changes, open(OUT / "mass_changes.json", "w"), indent=1)

    conv = [{"round": r["round"], "feasible": r["feasible"], "n": r["n"], "roll": round(r["best"]["roll"]["alpha"], 2),
             "pitch": round(r["best"]["pitch"]["alpha"], 2), "yaw": round(r["best"]["yaw"]["alpha"], 2),
             "weakest": round(r["best"]["min_alpha"], 2)} for r in mcj["rounds"]]
    json.dump(conv, open(OUT / "convergence.json", "w"), indent=1)
    print("rest done")


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    rest()
    grid()
