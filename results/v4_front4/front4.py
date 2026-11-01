"""V4 with four front motors side by side (Fusion: "SMALL_SCALE_V4.2 - Four front motors - Side by side",
the "Side ESCs" variant), built from the weighed V4 (atlas_v4_fusion_v8_h18_p_0) and changed only where the CAD
changed, plus a fourth battery the CAD does not have yet.

Frame: Fusion world (cm) -> structural FRD (m) is x = -Z, y = X, z = -Y (checked on every labelled part shared
with the V4 CAD: the batteries, the PDBs, the rear EDFs and ESCs all land on the V4 items to the 0.1 mm).

What changed, from the CAD (fusion_four_front_side_by_side_bodies.json):
  * M7 / M8 (the canted pair, x 0.255, y -/+0.064, cant -30 deg): unchanged. Their lock bolts are where V4's are.
  * M9 (one vertical motor on the centreline, x 0.355) is now two vertical motors side by side, V_LEFT and V_RIGHT,
    at y -/+0.052, 1 cm higher than M9 sat (lock bolts at z -0.513 against V4's -0.523).
  * The centre front ESC (ESC__M200g (9)/(10), x 0.198, y 0) is hidden; two ESCs sit beside the vertical pair
    (ESC_FRONT_LEFT / _RIGHT, x 0.355, y -/+0.136). ESC_L / ESC_R are V4's two side ESCs, unchanged.
  * Not modelled: the new cradle against the old one, and the extra M4 bolts. The user's statement is that the
    weight changed by one battery and one motor; the cradle volumes say nothing about the print settings.
  * The fourth battery is not in the CAD. Assumed: four of the same 700 g packs side by side at the same pitch
    (55 mm), centred, in the place of the three (x -0.145, z -0.58).
"""
from __future__ import annotations

import copy, json, math, sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from airframe_designer.geometry.airframe import Airframe          # noqa: E402
from airframe_designer.geometry.frames import tilt_cant_to_axis   # noqa: E402
from airframe_designer.constants import G                         # noqa: E402

BASE = ROOT / "airframes" / "atlas_v4_fusion_v8_h18_p_0.json"
OUT = ROOT / "airframes" / "atlas_v4_front4_side_by_side.json"

EDF_BIG, EDF_SMALL = 0.322821, 0.007179            # V4's split of a 330 g XFLY EDF over its two bodies
ESC_BIG, ESC_SMALL = 0.132927, 0.067073            # and of a 200 g ESC
V_Y, V_X = 0.052, 0.355
V_BODY1_Z, V_BODY2_Z = -0.475, -0.5374             # CAD centroids of the vertical EDFs
ROTOR_ALONG_AXIS = 0.0226                          # V4 puts a rotor 22.6 mm along its thrust axis from the EDF centroid
V_ROTOR_Z = V_BODY1_Z - ROTOR_ALONG_AXIS            # thrust axis (0, 0, -1): -0.4976
M9_RAISE = 0.010                                   # the vertical motors sit 1 cm higher than M9 did


def S(r):
    r = np.asarray(r, float)
    return np.dot(r, r) * np.eye(3) - np.outer(r, r)


def own_tensor(it):
    ixy, ixz, iyz = it["inertia_products"]
    return np.diag(it["inertia"]) - np.array([[0, ixy, ixz], [ixy, 0, iyz], [ixz, iyz, 0]], float)


def set_own(it, I):
    it["inertia"] = [float(I[0, 0]), float(I[1, 1]), float(I[2, 2])]
    it["inertia_products"] = [float(-I[0, 1]), float(-I[0, 2]), float(-I[1, 2])]


def remove_point(it, m, p):
    """Take a point mass m at p out of a lumped item, keeping its tensor right about its new centroid."""
    M, c = it["mass"], np.array(it["pos"], float)
    M2 = M - m
    c2 = (M * c - m * np.asarray(p, float)) / M2
    I = own_tensor(it) + M * S(c - c2) - m * S(np.asarray(p, float) - c2)
    it["mass"], it["pos"] = M2, c2.tolist()
    set_own(it, I)


def item(name, m, pos):
    return {"name": name, "mass": m, "pos": [float(v) for v in pos], "inertia": [0.0, 0.0, 0.0],
            "inertia_products": [0.0, 0.0, 0.0]}


def build() -> dict:
    d = json.load(open(BASE))
    d = copy.deepcopy(d)
    d["name"] = "ATLAS_V4_FRONT4_SIDE_BY_SIDE"
    items = d["mass"]["items"]
    by = {it["name"]: it for it in items}

    # M9's EDF is in the unlabelled remainder (V4 labels only the rear EDFs): take it out where it was ...
    rem = next(it for it in items if it["name"].startswith("unlabelled remainder"))
    m9_z = (EDF_BIG * V_BODY1_Z + EDF_SMALL * V_BODY2_Z) / (EDF_BIG + EDF_SMALL) - M9_RAISE
    remove_point(rem, EDF_BIG + EDF_SMALL, [V_X, 0.0, m9_z])
    # ... and put two labelled ones in, where the CAD has them
    for side, tag in ((-1, "V_LEFT"), (1, "V_RIGHT")):
        items.append(item(f"XFLY 80mm EDF__M330g {tag}", EDF_BIG, [V_X, side * V_Y, V_BODY1_Z]))
        items.append(item(f"XFLY 80mm EDF__M330g {tag} (2)", EDF_SMALL, [V_X, side * V_Y, V_BODY2_Z]))

    # the centre front ESC goes to the left of the vertical pair, and a new one sits on the right
    by["ESC__M200g (9)"].update(name="ESC__M200g FRONT_LEFT", pos=[0.3546, -0.1360, -0.5100])
    by["ESC__M200g (10)"].update(name="ESC__M200g FRONT_LEFT (2)", pos=[0.3687, -0.1564, -0.5100])
    items.append(item("ESC__M200g FRONT_RIGHT", ESC_BIG, [0.3554, 0.1360, -0.5100]))
    items.append(item("ESC__M200g FRONT_RIGHT (2)", ESC_SMALL, [0.3413, 0.1564, -0.5100]))

    # four packs, 55 mm apart, centred where the three were (the CAD has no fourth one yet)
    pack = by["BATTERY_3PACK_01__M700g"]
    for k, y in enumerate((-0.0825, -0.0275, 0.0275)):
        by[f"BATTERY_3PACK_0{k + 1}__M700g"]["pos"] = [-0.145, y, -0.58]
    b4 = copy.deepcopy(pack)
    b4.update(name="BATTERY_4PACK_04__M700g (not in CAD: assumed)", pos=[-0.145, 0.0825, -0.58])
    items.append(b4)

    # rotors: M7 / M8 as they were, M9 -> the left vertical, M10 the right one, same EDF and thrust
    m9 = next(r for r in d["rotors"] if r["name"] == "M9")
    m9.update(pos=[V_X, -V_Y, V_ROTOR_Z], axis=[0.0, 0.0, -1.0])
    m10 = copy.deepcopy(m9)
    m10.update(name="M10", pos=[V_X, V_Y, V_ROTOR_Z])
    d["rotors"].append(m10)
    d["px4_overrides"]["PWM_AUX_FUNC2"] = 110

    d["notes"] = (d.get("notes", "") + "\n[2026-10-23] Four front motors side by side (Fusion 'SMALL_SCALE_V4.2 - Four front "
                  "motors - Side by side', Side ESCs variant, v1, read over the Fusion MCP): M9 split into two vertical "
                  "motors at y -/+0.052 (M9, M10), the centre front ESC moved beside them and a second added, a fourth "
                  "700 g battery assumed beside the three (not in the CAD). Built by results/v4_front4/front4.py.")
    af = Airframe.from_dict(d)
    af.resolve_mass()
    af.hover_pitch_deg = round(natural_hover_pitch(af), 2)
    return af.to_dict()


# ------------------------------------------------------------------ static model
def natural_hover_pitch(af: Airframe, lo: float = -10.0, hi: float = 45.0) -> float:
    """The hover pitch at which PX4's allocation for pure lift leaves no fore-aft force: the attitude the
    aircraft really hovers at, so the card and the aircraft agree."""
    def fx(th):
        af.hover_pitch_deg = th
        E6 = af.effectiveness()
        u = np.linalg.pinv(E6[[0, 1, 2, 5]]) @ np.array([0, 0, 0, -1.0])
        return float(E6[3] @ u)
    a, b = lo, hi
    fa = fx(a)
    for _ in range(60):
        m = 0.5 * (a + b)
        fm = fx(m)
        if (fm > 0) == (fa > 0):
            a, fa = m, fm
        else:
            b = m
    return 0.5 * (a + b)


def authority(af: Airframe, util_limit: float = 0.85) -> dict:
    """Per-axis authority at the hover trim, as PX4 allocates it (pseudo-inverse of the roll, pitch, yaw and
    lift rows of its effectiveness, thrust linear in the command). For each axis: the torque PX4 can ask for
    each way before any motor reaches 0 or full thrust, and the angular acceleration that torque gives,
    on that axis, in PX4's frame (the inertia products included)."""
    rotors = af.active_rotors()
    tmax = np.array([r.effective_max_thrust() for r in rotors])
    E6 = af.effectiveness()
    E = E6[[0, 1, 2, 5]]
    A = np.linalg.pinv(E)
    u = A @ np.array([0, 0, 0, -1.0])
    lift = float(-(E[3] @ u))
    t = u * (af.mass.mass * G / lift)
    util = t / tmax
    R = af.hover_rotation()
    I = R @ af.mass.tensor() @ R.T
    Ii = np.linalg.inv(I)
    out = {"hover_pitch_deg": af.hover_pitch_deg, "trim_util": util.tolist(), "max_util": float(util.max()),
           "min_util": float(util.min()), "feasible": bool(util.min() >= 0 and util.max() <= util_limit),
           "lean_deg": math.degrees(math.atan2(math.hypot(*(E6[3:5] @ u)), -(E6[5] @ u)))}
    for k, ax in enumerate(("roll", "pitch", "yaw")):
        e = np.zeros(4); e[k] = 1.0
        dt = A @ e                                   # N per N m asked
        ways = []
        for sgn in (1.0, -1.0):
            d = sgn * dt
            with np.errstate(divide="ignore", invalid="ignore"):
                room = np.where(d > 1e-12, (tmax - t) / d, np.where(d < -1e-12, -t / d, np.inf))
            ways.append(max(0.0, float(room.min())))
        alpha = Ii[k, k]                              # rad/s^2 per N m on that axis
        cpl = float(np.linalg.norm(np.delete(Ii[:, k], k)) / abs(Ii[k, k]))
        side = float(np.linalg.norm((E6[3:5] @ dt)))  # N of sideways force per N m: what PX4 does not see
        out[ax] = {"torque_plus": ways[0], "torque_minus": ways[1], "torque": min(ways),
                   "alpha": min(ways) * alpha, "coupling": cpl, "side_force_per_Nm": side}
    out["min_alpha"] = min(out[a]["alpha"] for a in ("roll", "pitch", "yaw"))
    return out


if __name__ == "__main__":
    d = build()
    json.dump(d, open(OUT, "w"), indent=1)
    af = Airframe.load(OUT)
    m = af.mass
    print(f"wrote {OUT.relative_to(ROOT)}")
    print(f"mass {m.mass:.3f} kg   cg {m.cg}   inertia {m.inertia}   products {m.inertia_products}")
    print(f"natural hover pitch {af.hover_pitch_deg:.2f} deg")
    print("validate:", af.validate())
    print("hover check:", af.hover_check()["problems"] or "ok")
    print(json.dumps(authority(af), indent=1, default=float))
