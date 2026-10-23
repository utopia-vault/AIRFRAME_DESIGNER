"""Physical coefficients for the ground sequence: a tripod with two rear feet, and two or three front fans."""
from itertools import combinations
import numpy as np

def ground_sequence_params(af):
    out = {"NLF_CFG_OK": 0}
    rotors = af.active_rotors()
    legs = af.active_legs()
    if not 3 <= len(rotors) <= 12 or len(legs) != 3:
        return out
    feet = sorted([np.asarray(l.foot(), float) for l in legs], key=lambda p: p[0])
    # Two rear feet must form a transverse hinge. General landing gear needs a different controller.
    if abs(feet[0][0]-feet[1][0]) > .01 or abs(feet[0][2]-feet[1][2]) > .01:
        return out
    pivot = (feet[0]+feet[1])/2
    cg = np.asarray(af.cg)
    if not (feet[0][1] < cg[1] < feet[1][1] or feet[1][1] < cg[1] < feet[0][1]):
        return out
    if cg[0] <= pivot[0] or feet[2][0] <= cg[0]:
        return out
    # The front fans are the ones ahead of the CG: they hold the nose while the rear feet carry the rest.
    front = [i for i, r in enumerate(rotors) if r.pos[0] > cg[0]]
    if len(front) not in (2, 3) or any(abs(rotors[i].thrust_exponent-2) > 1e-6 for i in front):
        return out
    thrust = {i: np.asarray(rotors[i].axis, float)/np.linalg.norm(rotors[i].axis)*rotors[i].effective_max_thrust() for i in front}
    pitch = {i: (np.cross(np.asarray(rotors[i].pos)-pivot, thrust[i]) - rotors[i].km*thrust[i])[1] for i in front}
    yaw = {i: np.cross(np.asarray(rotors[i].pos)-cg, thrust[i])[2] - rotors[i].km*thrust[i][2] for i in front}
    # A and B are the pair that makes opposite yaw (and steers the heading on the ground); a third fan takes a full share
    # and the pair's weights absorb its yaw. With two fans this is the ten-motor layout's motor 9 / motor 10 split.
    pairs = [(a, b) for a, b in combinations(front, 2) if yaw[a]*yaw[b] < 0]
    if not pairs or min(pitch.values()) <= 0:
        return out
    a, b = max(pairs, key=lambda p: abs(yaw[p[0]]-yaw[p[1]]))
    c = next((i for i in front if i not in (a, b)), None)
    wc = 1.0 if c is not None else 0.0
    w9 = -(2*yaw[b] + (wc*yaw[c] if c is not None else 0.0))/(yaw[a]-yaw[b])
    authority = w9*pitch[a] + (2-w9)*pitch[b] + (wc*pitch[c] if c is not None else 0.0)
    values = [af.mass.mass, *(cg-pivot)[[0,2]], authority, w9]
    if not np.isfinite(values).all() or af.mass.mass <= 0 or authority <= 0 or not 0 < w9 < 2:
        return out
    out.update(dict(zip(["NLF_MASS", "NLF_GX", "NLF_GZ", "NLF_MOM", "NLF_W9"], map(float,values))))
    out.update(NLF_FA=a+1, NLF_FB=b+1, NLF_FC=0 if c is None else c+1, NLF_WC=wc)
    # Zero-torque hover pattern (thrust fraction per motor): the pilot handover flies the ground phases by scaling the
    # rear motors along it with the throttle, and blends the front fans from the ground hold to their hover share.
    # It describes eight rear motors and a front pair on 9 and 10, so only that layout gets one.
    hover = af.hover_check()
    util = hover.get("hover_utilisation") or []
    if (len(rotors) == 10 and front == [8, 9] and len(util) == 10
            and all(u is not None and np.isfinite(u) and 0 < u < 1 for u in util) and not hover.get("negative")):
        for i in range(8):
            out[f"NLF_U{i+1}"] = round(float(util[i]), 4)
        out["NLF_UF"] = round(float((util[8]+util[9])/2), 4)
    out["NLF_CFG_OK"] = 1
    return out
