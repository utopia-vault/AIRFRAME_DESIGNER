"""Where the fourth 700 g battery should go on the four-front-motor V4, with the front motors at option C.

Each candidate keeps the three packs where the CAD has them (x -0.145, y -0.055 / 0 / +0.055, z -0.58) and
puts the fourth somewhere else, except "four-pack tray", which re-spaces all four at the same 55 mm pitch,
centred. A pack is a 160 x 50 x 50 mm box (CAD), its own inertia included. For each: mass, CG, natural hover
pitch, hover trim and authority, by front4.authority, with the front angles of option C and as drawn.
"""
from __future__ import annotations

import copy, json, sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from front4 import BASE, authority, build, natural_hover_pitch          # noqa: E402
from airframe_designer.geometry.airframe import Airframe              # noqa: E402
from airframe_designer.geometry.frames import tilt_cant_to_axis       # noqa: E402

PACK_M, PACK_DIM = 0.7, (0.160, 0.050, 0.050)
OWN = [PACK_M / 12 * (PACK_DIM[1] ** 2 + PACK_DIM[2] ** 2), PACK_M / 12 * (PACK_DIM[0] ** 2 + PACK_DIM[2] ** 2),
       PACK_M / 12 * (PACK_DIM[0] ** 2 + PACK_DIM[1] ** 2)]
ANGLES = {"option C": (22.6, -45.0, 11.1, 30.0), "as drawn": (0.0, -30.0, 0.0, 0.0)}

CANDIDATES = {
    "four-pack tray (55 mm pitch, centred)": None,
    "stacked on the lid, centred": (-0.145, 0.0, -0.644),
    "forward, on the centreline at x 0": (0.0, 0.0, -0.58),
    "on the PDB tray, x 0.15": (0.149, 0.0, -0.585),
    "aft of the tray, x -0.31": (-0.31, 0.0, -0.58),
}


def design(where) -> dict:
    d = build()
    items = d["mass"]["items"]
    packs = [it for it in items if it["name"].startswith("BATTERY_")]
    if where is None:
        return d
    for it, y in zip(packs[:3], (-0.055, 0.0, 0.055)):
        it["pos"] = [-0.145, y, -0.58]
    packs[3]["pos"] = list(where)
    for it in packs:
        it["inertia"], it["inertia_products"] = list(OWN), [0.0, 0.0, 0.0]
    return d


def evaluate(d: dict, ang) -> dict:
    af = Airframe.from_dict(copy.deepcopy(d))
    for r in af.rotors:
        if r.name in ("M7", "M8"):
            r.axis = tilt_cant_to_axis(ang[0], ang[1], r.side)
        elif r.name in ("M9", "M10"):
            r.axis = tilt_cant_to_axis(ang[2], ang[3], r.side)
    af.resolve_mass()
    af.hover_pitch_deg = natural_hover_pitch(af)
    a = authority(af)
    return {"mass": af.mass.mass, "cg": af.mass.cg, "hover": a["hover_pitch_deg"], "umin": a["min_util"], "umax": a["max_util"],
            "front_util": a["trim_util"][6:], "rear_util": a["trim_util"][:6],
            "roll": a["roll"]["alpha"], "pitch": a["pitch"]["alpha"], "yaw": a["yaw"]["alpha"], "weakest": a["min_alpha"]}


if __name__ == "__main__":
    out = {"candidates": [], "sweep": []}
    for name, where in CANDIDATES.items():
        d = design(where)
        row = {"name": name, "where": where}
        for k, ang in ANGLES.items():
            row[k] = evaluate(d, ang)
        out["candidates"].append(row)
        c = row["option C"]
        print(f"{name:40s} cg x {c['cg'][0]:+.4f} z {c['cg'][2]:+.4f} | hover {c['hover']:5.2f} | util {c['umin']:.2f}-{c['umax']:.2f} | "
              f"roll {c['roll']:5.2f} pitch {c['pitch']:5.2f} yaw {c['yaw']:5.2f}")
    for x in np.arange(-0.34, 0.2001, 0.02):
        r = evaluate(design((float(x), 0.0, -0.58)), ANGLES["option C"])
        r["x"] = float(x)
        out["sweep"].append(r)
    best = max(out["sweep"], key=lambda r: r["weakest"])
    print(f"sweep: weakest axis best at x {best['x']:+.2f} ({best['weakest']:.2f}); "
          + "  ".join(f"x{r['x']:+.2f}:{r['weakest']:.2f}/{r['umax']:.2f}" for r in out["sweep"]))
    json.dump(out, open(HERE / "battery_study.json", "w"), indent=1, default=float)
