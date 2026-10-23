"""Summarise a fly_nose_lift.py run: the module's phases from its own messages, and what the aircraft did in each."""
import json, re, sys
import numpy as np

d = json.load(open(sys.argv[1]))
c = {k: i for i, k in enumerate(d["columns"])}
R = np.array([[np.nan if v is None else v for v in r] for r in d["rows"] if len(r) == len(d["columns"])], float)
t = R[:, c["t"]]
col = lambda k: R[:, c[k]]
print("events:", d["events"])
print("\nmodule messages:")
for l in d["log"]:
    if "atlas_nose_lift" in l or "nose" in l.lower() or "landing" in l.lower() or "hover" in l.lower() or "rmed" in l:
        print("  ", re.sub(r"^\S+\s+", "", l)[:150])

armed = col("armed") > 0.5
if not armed.any():
    sys.exit("never armed")
ta, tb = t[armed][0], t[armed][-1]
print(f"\narmed {ta:.1f} .. {tb:.1f} s ({tb-ta:.1f} s)")
h, pitch, roll, r = col("height"), col("pitch"), col("roll"), col("r")
ground = col("on_ground") > 0.5
n, e = col("n"), col("e")
off = np.where(armed & ~ground)[0]
if len(off):
    lift, land = t[off[0]], t[off[-1]]
    pre = armed & (t < lift)
    print(f"on the stand before liftoff: {lift-ta:.1f} s, pitch {pitch[pre][0]:.1f} -> {pitch[pre][-1]:.1f} deg, "
          f"slide {np.hypot(n[pre][-1]-n[pre][0], e[pre][-1]-e[pre][0])*100:.1f} cm, peak yaw rate {np.nanmax(abs(r[pre])):.1f} deg/s")
    fl = armed & ~ground
    print(f"airborne {lift:.1f} .. {land:.1f} s: max height {np.nanmax(h[fl]):.2f} m, pitch {np.nanmin(pitch[fl]):.1f} .. {np.nanmax(pitch[fl]):.1f} deg, "
          f"max |roll| {np.nanmax(abs(roll[fl])):.1f} deg, max |yaw rate| {np.nanmax(abs(r[fl])):.1f} deg/s, max speed {np.nanmax(col('speed')[fl]):.2f} m/s")
    k = fl & (t > lift + 5)
    if k.any():
        print(f"  hover (from 5 s after liftoff): height {np.nanmean(h[k]):.2f} +- {np.nanstd(h[k]):.2f}, pitch {np.nanmean(pitch[k]):.1f} +- {np.nanstd(pitch[k]):.1f}")
    post = armed & (t > land)
    if post.any():
        vd = col("vd")
        print(f"after rear touchdown: {tb-land:.1f} s to disarm, pitch {pitch[post][0]:.1f} -> {pitch[post][-1]:.1f} deg, "
              f"touchdown sink {np.nanmax(vd[(t > land-0.5) & (t <= land+0.2)]):.2f} m/s, "
              f"slide {np.hypot(n[post][-1]-n[post][0], e[post][-1]-e[post][0])*100:.1f} cm, peak yaw rate {np.nanmax(abs(r[post])):.1f} deg/s")
end = t > tb
if end.any():
    print(f"final: pitch {pitch[end][-1]:.1f} deg, height {h[end][-1]:.2f} m, armed {bool(armed[-1])}")
mot = [k for k in d["columns"] if k.endswith("_cmd")]
print("\nmotor commands (mean) by stage:")
for name, mask in [("stand", armed & (t < (t[off[0]] if len(off) else tb))), ("air", armed & ~ground),
                   ("lowering", armed & (t > (t[off[-1]] if len(off) else ta)))]:
    if mask.any():
        print(f"  {name:9s}", " ".join(f"{np.nanmean(col(k)[mask]):.2f}" for k in mot))
