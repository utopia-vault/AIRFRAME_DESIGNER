"""Flight recorder: every simulation run is kept, with the design that produced it.

The point is to be able to come back to a flight. Not to a number someone wrote down about it, but to the
flight: the trace of every quantity the overlay shows, plus the attitude at each instant so the 3D view can
be scrubbed back and forth through it, plus the airframe and the exported parameters exactly as they were.
A run you cannot reproduce is an anecdote.

A run starts when the simulation is reset and ends when it is reset again, or when the app asks for it. It
is written to results/runs/<id>.json with a timestamp, and it can be renamed or deleted from the app. The
columns are fixed and documented here, so a saved run from today still reads in a year.
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

__all__ = ["Recorder", "RunStore", "COLUMNS", "output_fraction"]

#: One row per sample. Everything the overlay shows, plus the pose needed to replay the view.
COLUMNS = [
    "t",                                        # s since the run started
    "n", "e", "d",                              # position, world NED, m
    "vn", "ve", "vd",                           # velocity, world NED, m/s
    "qw", "qx", "qy", "qz",                     # attitude, body -> world
    "roll", "pitch", "yaw",                     # degrees, for reading and plotting
    "p", "q", "r",                              # body rates, deg/s
    "height",                                   # m above the ground
    "speed",                                    # ground speed, m/s
    "airspeed",                                 # m/s
    "thrust",                                   # total motor thrust, N
    "power",                                    # motor power, W
    "lift",                                     # wing lift, N
    "battery",                                  # charge left, %
    "wind",                                     # wind speed, m/s
    "armed",                                    # 1 or 0
    "on_ground",                                # 1 or 0
    # what the sensors read: the accelerometer is specific force, so it reads ~-9.81 on z standing still
    "ax", "ay", "az",                           # body accelerometer, m/s^2
    "gx", "gy", "gz",                           # body gyro, deg/s
    "alt_baro",                                 # pressure altitude above the start, m
    "flow_x", "flow_y",                         # H-flow line-of-sight rate, rad/s (0 without the sensor)
    "flow_range",                               # H-flow slant range to the ground, m (-1 = out of range)
    "flow_q",                                   # H-flow picture quality, 0-255
    "mode",                                     # PX4 flight mode: main mode, + sub mode / 100 when Auto
    # The pilot's own hands, as the receiver reports them: RC_CHANNELS 1-8 in microseconds (1000-2000,
    # 0 = no such channel). Recorded so a flight can be replayed with the stick that caused it visible
    # beside what the aircraft then did - the same columns whether the sticks came from the USB remote in
    # SITL or from the receiver on the aircraft. Eight because that is sticks plus the switches that
    # matter; a channel beyond those is configuration, not input.
    "rc1", "rc2", "rc3", "rc4", "rc5", "rc6", "rc7", "rc8",
]

#: How many receiver channels the recording keeps.
RC_CHANNELS = 8

#: Appended once per rotor, after the fixed columns above: the throttle PX4 asked for, what the motor has
#: actually spooled up to (both 0-1), and the thrust that makes, in newtons.
ROTOR_COLUMNS = ("cmd", "spool", "thrust")


def output_fraction(v: float) -> float:
    """One motor output as 0-1, however the aircraft reported it.

    Telemetry carries an output either as a fraction or as a PWM in microseconds, and both arrive on the
    same field. Anything above 1.5 can only be a pulse width."""
    v = float(v)
    if v > 1.5:
        v = (v - 1000.0) / 1000.0
    return max(0.0, min(1.0, v))


def _rc_values(status: dict) -> list[float]:
    """The first `RC_CHANNELS` receiver channels, in microseconds; 0 for a channel the receiver has not got."""
    ch = list((status or {}).get("rc") or [])
    return [float(ch[i]) if i < len(ch) and ch[i] is not None else 0.0 for i in range(RC_CHANNELS)]


def columns_for(rotors: int) -> list[str]:
    """The column list of a recording with this many rotors. A saved run carries its own copy."""
    return list(COLUMNS) + [f"m{i + 1}_{k}" for i in range(max(0, int(rotors))) for k in ROTOR_COLUMNS]


@dataclass
class Recorder:
    """Collects one run in memory at a fixed sample rate."""

    sample_hz: float = 20.0
    max_samples: int = 120_000                  # about 100 minutes at 20 Hz; a run longer than that is a mistake
    rows: list[list[float]] = field(default_factory=list)
    columns: list[str] = field(default_factory=lambda: list(COLUMNS))
    started: float = field(default_factory=time.time)
    t0: float | None = None
    saved_at: int = -1                          # rows already written to disk, so a run is never kept twice
    _next_t: float = 0.0

    def reset(self) -> None:
        self.rows = []
        self.columns = list(COLUMNS)
        self.started = time.time()
        self.t0 = None
        self.saved_at = -1
        self._next_t = 0.0

    def sample(self, t: float, body, status: dict, battery_percent: float, motors: list | None = None,
               extra: dict | None = None) -> None:
        """Take one sample if enough simulated time has passed. Called from the simulation loop.

        ``motors`` is used when there is no simulated rotor to read: a real flight reports its outputs over
        telemetry, and those go in the same per-rotor columns. ``extra`` carries what the sensors read
        (the H-flow), which the body itself knows nothing about.
        """
        if self.t0 is None:
            self.t0 = float(t)
        elapsed = float(t) - self.t0
        if self.rows and elapsed < self.rows[-1][0] - 0.5:
            # The simulation clock went back (a reset relaunches PX4 and restarts it at zero). The recording
            # is the session, not the clock, so it carries on from where it was instead of stopping dead.
            self.t0 = float(t) - self.rows[-1][0]
            elapsed = self.rows[-1][0]
        if elapsed + 1e-9 < self._next_t:
            return
        if len(self.rows) >= self.max_samples:
            # Full. Drop the oldest quarter and keep going: a graph that quietly stopped an hour ago is
            # worse than one that has forgotten the beginning.
            drop = self.max_samples // 4
            self.rows = self.rows[drop:]
            self.saved_at = max(-1, self.saved_at - drop)
        self._next_t = elapsed + 1.0 / max(1e-3, self.sample_hz)

        bd = body.breakdown or {}
        ex = extra or {}
        accel = getattr(body, "accel_body", None)
        cmds = getattr(body, "cmd", None)
        n_rotors = len(motors) if motors is not None else (0 if cmds is None else len(cmds))
        if not self.rows:
            self.columns = columns_for(n_rotors)
        # Attitude, rates and the accelerometer are recorded in PX4's body frame - the structural frame
        # pitched nose-up by hover_pitch_deg - and never in the structural one, because a column has to
        # mean the same thing in a simulated run and in a real flight, and in a real flight PX4's frame is
        # the only one there is. The quaternion columns stay structural: they are what the 3D view draws,
        # and the model is a physical object.
        R = body.af.hover_rotation() if getattr(body, "af", None) is not None else np.eye(3)
        euler = body.hover_frame_euler() if hasattr(body, "hover_frame_euler") else body.euler
        roll, pitch, yaw = (float(np.degrees(v)) for v in euler)
        rates = [float(np.degrees(v)) for v in R @ body.rates]
        if accel is not None:
            accel = R @ np.asarray(accel, float)
        wind = float(np.linalg.norm(body.wind_ned))
        self.rows.append([
            round(elapsed, 4),
            *[round(float(v), 4) for v in body.pos],
            *[round(float(v), 4) for v in body.vel],
            *[round(float(v), 6) for v in body.q],
            round(roll, 3), round(pitch, 3), round(yaw, 3),
            *[round(v, 3) for v in rates],
            round(float(-body.pos[2]), 4),
            round(float(np.linalg.norm(body.vel)), 4),
            round(float(bd.get("airspeed", 0.0)), 4),
            round(float(bd.get("thrust", 0.0)), 3),
            round(float(bd.get("power", 0.0)), 2),
            round(float(bd.get("lift", 0.0)), 3),
            round(float(battery_percent), 2),
            round(wind, 3),
            1.0 if status.get("armed") else 0.0,
            1.0 if body.on_ground else 0.0,
            *[round(float(v), 4) for v in (accel if accel is not None else np.zeros(3))],
            *[round(v, 3) for v in rates],
            round(float(ex.get("alt_baro", -body.pos[2])), 3),
            *[round(float(v), 5) for v in (ex.get("flow_rate") or (0.0, 0.0))],
            round(float(ex.get("flow_range", -1.0)), 4),
            round(float(ex.get("flow_quality", 0.0)), 1),
            round(float(status.get("mode", 0.0)), 2),
            *[round(float(v), 1) for v in _rc_values(status)],
            *self._rotor_values(body, motors, n_rotors),
        ])

    @staticmethod
    def _rotor_values(body, motors, n: int) -> list[float]:
        """Per rotor: commanded throttle, spooled throttle, thrust - simulated, or as a real flight reports."""
        out: list[float] = []
        for i in range(n):
            if motors is not None:
                v = output_fraction(motors[i])
                out += [round(v, 4), round(v, 4), 0.0]
            else:
                out += [round(float(body.cmd[i]), 4), round(float(body.omega[i]), 4), round(float(body.thrust[i]), 3)]
        return out

    @property
    def flew(self) -> bool:
        """True if the aircraft armed or left the ground. A reset after nothing happened is not a flight."""
        ia, ig = COLUMNS.index("armed"), COLUMNS.index("on_ground")
        return any(r[ia] > 0.5 or r[ig] < 0.5 for r in self.rows)

    @property
    def unsaved(self) -> bool:
        return len(self.rows) > self.saved_at

    @property
    def duration(self) -> float:
        return float(self.rows[-1][0]) if self.rows else 0.0

    def as_dict(self) -> dict:
        return {"columns": list(self.columns), "rows": self.rows, "duration": round(self.duration, 3),
                "samples": len(self.rows), "sample_hz": self.sample_hz}


def dump_run(path: Path, doc: dict, chunk: int = 500) -> None:
    """Write a run as JSON a slice of rows at a time, the same bytes json.dumps(doc) would give.

    One json.dumps of a long recording is a single C call that holds the interpreter until it is done - most of a
    second for forty minutes of flight - and the simulation loop cannot step meanwhile, which a HITL board reads as
    its sensors failing. Between slices the loop gets the interpreter back. Written to a temporary file and moved
    into place, so a run file is never half written."""
    rows = doc.get("rows") or []
    head = {k: v for k, v in doc.items() if k != "rows"}
    tmp = path.with_name(path.name + ".part")
    with open(tmp, "w") as f:
        h = json.dumps(head, separators=(",", ":"))
        f.write(h[:-1] + ("," if head else "") + '"rows":[')
        for i in range(0, len(rows), chunk):
            part = json.dumps(rows[i:i + chunk], separators=(",", ":"))[1:-1]
            if part:
                f.write(("," if i else "") + part)
            time.sleep(0)                      # hand the interpreter back to the simulation
        f.write("]}")
    tmp.replace(path)


class RunStore:
    """Saved runs on disk: results/runs/<id>.json, newest first."""

    def __init__(self, root: Path):
        self.root = Path(root)
        self._index: dict[str, list] = {}         # file name -> [mtime, size, entry]
        self._index_loaded = False

    # Listing runs used to parse every one of them in full - a hundred megabytes of JSON, each file a single C call
    # that holds the interpreter - whenever the graph opened, after every save and on every design reload. The
    # summary of a run is kept here instead, keyed by the file's size and time, in memory and in index.cache next to
    # the runs, so each run is read once ever.
    INDEX_FILE = "index.cache"

    def _load_index(self) -> None:
        if self._index_loaded:
            return
        self._index_loaded = True
        try:
            self._index = json.loads((self._dir() / self.INDEX_FILE).read_text())
        except Exception:
            self._index = {}

    def _store_index(self) -> None:
        try:
            tmp = self._dir() / (self.INDEX_FILE + ".part")
            tmp.write_text(json.dumps(self._index))
            tmp.replace(self._dir() / self.INDEX_FILE)
        except Exception:
            pass

    def _cached_entry(self, f: Path) -> dict | None:
        st = f.stat()
        hit = self._index.get(f.name)
        if hit and hit[0] == st.st_mtime and hit[1] == st.st_size:
            return hit[2]
        try:
            entry = self._entry(json.loads(f.read_text()))
        except Exception:
            return None
        time.sleep(0)                              # one file at a time: the simulation gets the interpreter back
        self._index[f.name] = [st.st_mtime, st.st_size, entry]
        self._dirty = True
        return entry

    def _dir(self) -> Path:
        self.root.mkdir(parents=True, exist_ok=True)
        return self.root

    def _path(self, run_id: str) -> Path | None:
        """Resolve an id to its file, refusing anything that is not a plain id."""
        if not run_id or "/" in run_id or "\\" in run_id or run_id.startswith("."):
            return None
        p = self._dir() / f"{run_id}.json"
        return p if p.parent == self._dir() else None

    def save(self, recorder: Recorder, *, airframe: dict, params: dict, name: str = "",
             scenario: str = "", note: str = "", log: list | None = None) -> dict:
        """Write a run with the design that produced it. Returns its index entry."""
        if not recorder.rows:
            return {"ok": False, "error": "nothing recorded"}
        run_id = time.strftime("%Y%m%d-%H%M%S", time.localtime(recorder.started))
        path = self._path(run_id)
        n = 1
        while path is not None and path.exists():
            n += 1
            run_id = time.strftime("%Y%m%d-%H%M%S", time.localtime(recorder.started)) + f"-{n}"
            path = self._path(run_id)
        if path is None:
            return {"ok": False, "error": "bad run id"}
        doc = {
            "id": run_id,
            "name": name or (scenario or "flight"),
            "started": recorder.started,
            "scenario": scenario,
            "note": note,
            "airframe": airframe,
            "params": params,
            "log": list(log or []),      # what the flight controller said while this run was flying
            **recorder.as_dict(),
        }
        dump_run(path, doc)
        recorder.saved_at = len(recorder.rows)
        return {"ok": True, **self._entry(doc)}

    def save_doc(self, doc: dict, run_id: str) -> dict:
        """Write a run that was not recorded here - a log read off the flight controller - under its own id."""
        path = self._path(run_id)
        n = 1
        while path is not None and path.exists():
            n += 1
            path = self._path(f"{run_id}-{n}")
        if path is None:
            return {"ok": False, "error": "bad run id"}
        doc = {**doc, "id": path.stem}
        dump_run(path, doc)
        return {"ok": True, **self._entry(doc)}

    @staticmethod
    def _entry(doc: dict) -> dict:
        return {"id": doc["id"], "name": doc.get("name", ""), "started": doc.get("started", 0),
                "duration": doc.get("duration", 0), "samples": doc.get("samples", 0),
                "scenario": doc.get("scenario", ""), "note": doc.get("note", ""),
                "airframe_name": (doc.get("airframe") or {}).get("name", "")}

    def list(self) -> list[dict]:
        self._load_index()
        self._dirty = False
        files = sorted(self._dir().glob("*.json"), reverse=True)
        out = [e for e in (self._cached_entry(f) for f in files) if e is not None]
        names = {f.name for f in files}
        stale = [k for k in self._index if k not in names]
        for k in stale:
            del self._index[k]
        if stale or self._dirty:
            self._store_index()
        return out

    def get(self, run_id: str) -> dict | None:
        p = self._path(run_id)
        if p is None or not p.is_file():
            return None
        try:
            return json.loads(p.read_text())
        except Exception:
            return None

    def rename(self, run_id: str, name: str) -> bool:
        doc = self.get(run_id)
        p = self._path(run_id)
        if doc is None or p is None:
            return False
        doc["name"] = str(name)[:80]
        dump_run(p, doc)
        return True

    def delete(self, run_id: str) -> bool:
        p = self._path(run_id)
        if p is None or not p.is_file():
            return False
        p.unlink()
        return True
