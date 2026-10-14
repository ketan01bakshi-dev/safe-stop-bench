"""L2 back-to-back: the Modelica FMU against the reference implementation ssb/plant.py, then a bench regression.

  b2b.direct[...]    the same 1 ms input sequences into Python (Actuators + Vehicle/DynamicVehicle) and into the FMU;
                     every output compared per millisecond within the ladder's fixed b2b tolerance
  bench.regression   all 56 bench scenarios on the reference controller: Python plant vs FMU plant must give the same
                     verdict status (PASS/FAIL/KNOWN) for every scenario; stop distances reported side by side

Known difference D1 (plant_spec.md section 5): the Python dead time is 79 ms, the model's 80 ms. The tolerance is not
widened for it; it is what the tolerance has to absorb, and the report says so.
"""
from __future__ import annotations

import copy
import math
import random
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from ssb import campaigns, config
from ssb.plant import Actuators, make_vehicle
from ssb.plant_fmu import FmuPlant

Inputs = Callable[[float], dict]


@dataclass
class Seq:
    id: str
    dynamic: bool
    v0: float
    duration: float
    inputs: Inputs
    grade: float = 0.0
    mu: float = 0.8
    payload: float = 0.0


def _const(**kw) -> Inputs:
    return lambda t: kw


def _random_drive(seed: int) -> Inputs:
    rng = random.Random(seed)
    knots, t = [], 0.0
    while t < 8.0:
        knots.append((t, rng.uniform(-6.0, 2.5), rng.uniform(-12.0, 12.0)))
        t += rng.uniform(0.2, 0.8)

    def f(t: float) -> dict:
        a = s = 0.0
        for k, ka, ks in knots:
            if k <= t:
                a, s = ka, ks
        return {"accel_cmd": a, "steer_cmd": s}
    return f


SEQS: list[Seq] = [
    Seq("step_lag", False, 10, 2.0, lambda t: {"accel_cmd": 2.0 if t >= 0.5 else 0.0}),
    Seq("friction_windup", False, 20, 2.0, lambda t: {"accel_cmd": -10.0 if 0.5 <= t < 1.0 else 0.0}),
    Seq("weak_brake", False, 20, 4.0, lambda t: {"accel_cmd": -4.0 if 0.5 <= t < 2.0 else (1.0 if t >= 2.0 else 0.0), "brake_factor": 0.5}),
    Seq("backup_brake", False, 10, 2.0, lambda t: {"backup_decel": 2.0 if t >= 0.5 else 0.0}),
    Seq("stop_distance", False, 30 / 3.6, 6.0, _const(accel_cmd=-3.0)),
    Seq("grade_coast", False, 5, 6.0, _const(), grade=10.0),
    Seq("steer_ramp", False, 10, 2.0, lambda t: {"steer_cmd": 10.0 if t >= 0.5 else 0.0}),
    Seq("steer_stuck", False, 10, 2.0, lambda t: {"steer_cmd": 10.0 if t >= 0.5 else 0.0, "steer_stuck": t >= 0.7}),
    Seq("random_101", False, 12, 8.0, _random_drive(101)),
    Seq("random_202", False, 8, 8.0, _random_drive(202), mu=0.4),
    Seq("random_303", False, 15, 8.0, _random_drive(303), grade=-5.0),
    Seq("dyn_payload", True, 15, 4.0, _const(accel_cmd=-3.0), payload=800),
    Seq("dyn_grade", True, 10, 6.0, _const(), grade=10.0),
    Seq("dyn_random", True, 12, 8.0, _random_drive(404), payload=400, grade=3.0),
]
COMPARE = ("a", "delta", "v", "x", "yaw_rate")


def run_python(seq: Seq, cfg: dict) -> dict[str, list[float]]:
    c = copy.deepcopy(cfg)
    c["plant"]["model"] = "dynamic" if seq.dynamic else "kinematic"
    act, veh = Actuators(c), make_vehicle(c, seq.v0, seq.payload if seq.dynamic else None)
    veh.mu = seq.mu
    return _drive(seq, act, veh)


def run_fmu(seq: Seq, cfg: dict, fmu_path: Path) -> dict[str, list[float]]:
    c = copy.deepcopy(cfg)
    c["plant"]["model"], c["plant"]["fmu"] = "fmu", str(fmu_path)
    p = FmuPlant(c, seq.v0, seq.payload if seq.dynamic else None, {"mu": seq.mu, "grade_pct": seq.grade})
    return _drive(seq, p.act, p.veh)


def _drive(seq: Seq, act, veh) -> dict[str, list[float]]:
    out: dict[str, list[float]] = {k: [] for k in COMPARE}
    for ms in range(round(seq.duration * 1000)):
        u = {"accel_cmd": 0.0, "steer_cmd": 0.0, "backup_decel": 0.0, "brake_factor": 1.0, "steer_stuck": False,
             **seq.inputs(ms / 1000)}
        act.brake_factor, act.steer_stuck, act.steer_rate_factor = u["brake_factor"], u["steer_stuck"], 1.0
        a = act.step(u["accel_cmd"], u["steer_cmd"], u["backup_decel"], seq.mu, 0.001)
        veh.step(a, act.delta, seq.grade, 0.001)
        for k, v in (("a", a), ("delta", act.delta), ("v", veh.v), ("x", veh.x), ("yaw_rate", veh.yaw_rate)):
            out[k].append(v)
    return out


def compare(py: dict, fm: dict, rel: float, floor: float) -> tuple[bool, dict]:
    rows, ok = {}, True
    for k in COMPARE:
        diff = max((abs(a - b) for a, b in zip(py[k], fm[k], strict=True)), default=0.0)
        rng = max((abs(v) for v in py[k]), default=0.0)
        lim = max(rel * rng, floor)
        rows[k] = {"max_abs_diff": diff, "limit": lim, "passed": diff <= lim}
        ok = ok and diff <= lim
    return ok, rows


def direct(fmu_kin: Path, fmu_dyn: Path, tol: dict) -> list[dict]:
    cfg = config.load("config/default.json")
    out = []
    for seq in SEQS:
        py = run_python(seq, cfg)
        fm = run_fmu(seq, cfg, fmu_dyn if seq.dynamic else fmu_kin)
        ok, rows = compare(py, fm, tol["b2b_rel"], tol["b2b_abs"])
        out.append({"id": seq.id, "dynamic": seq.dynamic, "passed": ok, "signals": rows,
                    "final_x_python": py["x"][-1], "final_x_fmu": fm["x"][-1]})
    return out


def bench_regression(fmu_kin: Path) -> dict:
    """All 56 scenarios on the reference controller with each plant; verdict status must match for every one."""
    base = config.load("config/default.json")
    fm_cfg = copy.deepcopy(base)
    fm_cfg["plant"]["model"], fm_cfg["plant"]["fmu"] = "fmu", str(fmu_kin)
    py_res = {r["key"]: r for r in campaigns.matrix(base, keep_trace=True)}
    fm_res = {r["key"]: r for r in campaigns.matrix(fm_cfg, keep_trace=True)}
    rows, same = [], True
    for k, r in py_res.items():
        f = fm_res[k]
        sp, sf = r["verdict"]["status"], f["verdict"]["status"]
        dp, df = r["verdict"].get("stop_dist_m"), f["verdict"].get("stop_dist_m")
        rows.append({"key": k, "python": sp, "fmu": sf, "stop_dist_python": dp, "stop_dist_fmu": df,
                     "stop_dist_diff": (abs(dp - df) if isinstance(dp, (int, float)) and isinstance(df, (int, float))
                                        and math.isfinite(dp) and math.isfinite(df) else None)})
        same = same and sp == sf
    counts = {s: sum(1 for r in rows if r["fmu"] == s) for s in ("PASS", "FAIL", "KNOWN")}
    return {"same_verdicts": same, "scenarios": len(rows), "fmu_counts": counts,
            "python_counts": {s: sum(1 for r in rows if r["python"] == s) for s in ("PASS", "FAIL", "KNOWN")},
            "max_stop_dist_diff_m": max((r["stop_dist_diff"] for r in rows if r["stop_dist_diff"] is not None), default=None),
            "rows": rows}
