"""The V4 nose-lift module, end to end in SITL: module takeoff from the park, hover, module landing back onto the park.

Usage: fly_nose_lift.py <port> <label>. Talks to the app only (PX4 shell + recording); writes <label>.json beside it.
"""
import json, sys, time, urllib.parse, urllib.request

PORT, LABEL = sys.argv[1], sys.argv[2]
U = f"http://127.0.0.1:{PORT}"
OUT = f"/Users/stefanosfragkoulis/AIRFRAME_REPRO/v4_weighed_card20/analysis/{LABEL}.json"

def get(p): return json.load(urllib.request.urlopen(U + p, timeout=60))
def post(p, d): return json.load(urllib.request.urlopen(urllib.request.Request(U + p, json.dumps(d).encode(), {"Content-Type": "application/json"}), timeout=60))
def shell(c): return urllib.request.urlopen(U + "/api/px4/shell?c=" + urllib.parse.quote(c), timeout=60).read().decode()
def log_lines():
    d = get("/api/log"); d = d if isinstance(d, list) else d.get("lines", d.get("log", []))
    return [l if isinstance(l, str) else " ".join(map(str, l)) for l in d]
def now_t():
    rows = get("/api/runs/current")["rows"]; return rows[-1][0] if rows else 0.0

def wait_for(text, timeout, since):
    t0 = time.time()
    while time.time() - t0 < timeout:
        hits = [l for l in log_lines()[since:] if text in l]
        if hits: return hits[0]
        fail = [l for l in log_lines()[since:] if "abort" in l.lower() or "ground model configuration invalid" in l]
        if fail: return "FAIL " + fail[0]
        time.sleep(0.5)
    return None

print(shell("atlas_nose_lift status").strip().splitlines()[-1:])
time.sleep(15)                                         # estimator settle
n0 = len(log_lines()); t_start = now_t()
print("takeoff:", post("/api/px4/command", {"command": "nose_lift_takeoff"}))   # what the Takeoff button sends
# PX4's own view during the lift: height estimate, range, and the land detector (the recording has the simulator's truth)
est = []
t_poll = time.time()
while time.time() - t_poll < 20:
    lp, ld = shell("listener vehicle_local_position -n 1"), shell("listener vehicle_land_detected -n 1")
    pick = lambda txt, key: next((ln.split(":", 1)[1].strip() for ln in txt.splitlines() if ln.strip().startswith(key + ":")), None)
    est.append({"t": round(time.time() - t_poll, 2), "z": pick(lp, "z"), "vz": pick(lp, "vz"), "dist_bottom": pick(lp, "dist_bottom"),
                "landed": pick(ld, "landed"), "maybe_landed": pick(ld, "maybe_landed"), "ground_contact": pick(ld, "ground_contact")})
    if any("abort" in l for l in log_lines()[n0:]): break
for e in est: print("  est", e)
events = {}
for key, text, tmo in [("hover", "hover achieved", 90)]:
    events[key] = wait_for(text, tmo, n0); print(key, "->", events[key])
time.sleep(10)
n1 = len(log_lines())
print("land:", post("/api/px4/command", {"command": "nose_lift_land"}))   # what the Land button sends
events["landed"] = wait_for("landing complete", 150, n1); print("landed ->", events["landed"])
time.sleep(3)
d = get("/api/runs/current")
lines = [l for l in log_lines()[n0:] if "[px4]" in l]
json.dump({"label": LABEL, "t_start": t_start, "events": events, "estimate": est, "columns": d["columns"],
           "rows": [r for r in d["rows"] if r[0] >= t_start - 1], "log": lines}, open(OUT, "w"))
print("saved", OUT, len(lines), "log lines")
