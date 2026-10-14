"""L1 experiments: each drives one plant instance with time-based inputs; all instances live in ONE generated model
(`AllExperiments`) so a whole L1 pass is one compile + one simulation per solver tolerance.

Every check returns (ok, measured, expected, detail). Expected values come from plant/ci/handcalc.py, never from the solver.
"""
from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field

from plant.ci import handcalc as hc
from plant.ci.omc import SimResult

STOP_TIME = 6.0
DEFAULT_INPUTS = {"accel_cmd": "0", "steer_cmd": "0", "backup_decel": "0", "mu": "0.8", "grade_pct": "0",
                  "brake_factor": "1", "steer_stuck": "false", "steer_rate_factor": "1"}
OUTPUTS = ("a", "delta", "v", "a_veh", "yaw_rate", "x", "psi")

CheckFn = Callable[[SimResult, str, dict], tuple[bool, float, float, str]]


@dataclass
class Experiment:
    id: str
    cls: str                       # SafeStopPlant.Plant or SafeStopPlant.PlantDynamic
    params: dict[str, float]
    inputs: dict[str, str]         # Modelica expressions of `time`
    spec: list[str]                # spec requirements it exercises
    checks: dict[str, CheckFn] = field(default_factory=dict)
    key: str = "v"                 # output used for the convergence report

    def modelica(self) -> str:
        mods = ", ".join(f"{k} = {v}" for k, v in self.params.items())
        lines = [f"  {self.cls} {self.id}({mods});" if mods else f"  {self.cls} {self.id};"]
        return "\n".join(lines)

    def equations(self) -> list[str]:
        ins = {**DEFAULT_INPUTS, **self.inputs}
        return [f"  {self.id}.{k} = {v};" for k, v in ins.items()]


def _near(m: float, e: float, abs_tol: float) -> bool:
    return abs(m - e) <= abs_tol


def _rel(m: float, e: float, rel: float, floor: float = 1e-9) -> bool:
    return abs(m - e) <= max(rel * abs(e), floor)


def _first_time(sim: SimResult, name: str, pred: Callable[[float], bool], t_from: float = 0.0) -> float:
    for t, y in zip(sim.t, sim.series(name), strict=True):
        if t >= t_from and pred(y):
            return t
    return math.inf


# ---------------------------------------------------------------------------------------------------------------------------

def _exp(id_: str, cls: str, params: dict, inputs: dict, spec: list[str], key: str = "v") -> Experiment:
    return Experiment(id_, f"SafeStopPlant.{cls}", params, inputs, spec, key=key)


E: list[Experiment] = [
    _exp("step_lag", "Plant", {"v0": 10}, {"accel_cmd": "if time >= 0.5 then 2.0 else 0"}, ["R1", "R2"], key="a"),
    _exp("friction_windup", "Plant", {"v0": 20},
         {"accel_cmd": "if time >= 0.5 and time < 1.0 then -10 else 0"}, ["R3"], key="a"),
    _exp("accel_limit", "Plant", {"v0": 5}, {"accel_cmd": "if time >= 0.5 then 5 else 0"}, ["R3"], key="a"),
    _exp("weak_brake", "Plant", {"v0": 20},
         {"accel_cmd": "if time >= 0.5 and time < 2.0 then -4 elseif time >= 2.0 then 1 else 0", "brake_factor": "0.5"},
         ["R4"], key="a"),
    _exp("backup_brake", "Plant", {"v0": 10}, {"backup_decel": "if time >= 0.5 then 2 else 0"}, ["R5"], key="a"),
    _exp("stop_distance", "Plant", {"v0": 30 / 3.6}, {"accel_cmd": "-3"}, ["R1", "R2", "R7"], key="x"),
    _exp("grade_coast", "Plant", {"v0": 5}, {"grade_pct": "10"}, ["R7", "R8"], key="x"),
    _exp("standstill_uphill", "Plant", {"v0": 0}, {"grade_pct": "5", "accel_cmd": "-3"}, ["R7"], key="v"),
    _exp("steer_ramp", "Plant", {"v0": 10}, {"steer_cmd": "if time >= 0.5 then 10 else 0"}, ["R6"], key="delta"),
    _exp("steer_clamp", "Plant", {"v0": 10}, {"steer_cmd": "45"}, ["R6"], key="delta"),
    _exp("steer_stuck", "Plant", {"v0": 10}, {"steer_cmd": "if time >= 0.5 then 10 else 0", "steer_stuck": "time >= 0.7"},
         ["R6"], key="delta"),
    _exp("yaw_kinematics", "Plant", {"v0": 10}, {"steer_cmd": "5"}, ["R9"], key="yaw_rate"),
    _exp("dyn_coast", "PlantDynamic", {"v0": 10}, {}, ["R10"], key="v"),
    _exp("dyn_payload", "PlantDynamic", {"v0": 15, "payload": 800}, {"accel_cmd": "-3"}, ["R10"], key="v"),
    _exp("dyn_grade", "PlantDynamic", {"v0": 10}, {"grade_pct": "10"}, ["R8", "R10"], key="v"),
]
BY_ID = {e.id: e for e in E}


def check(exp_id: str, check_id: str) -> Callable[[CheckFn], CheckFn]:
    def deco(fn: CheckFn) -> CheckFn:
        BY_ID[exp_id].checks[check_id] = fn
        return fn
    return deco


def s(e: str, var: str) -> str:
    return f"{e}.{var}"


@check("step_lag", "handcalc.dead_time")
def _dead(sim, e, tol):
    before = sim.window(s(e, "a"), 0.5, 0.5 + hc.DEAD - tol["dead_time_s"])
    after = sim.at(s(e, "a"), 0.5 + hc.DEAD + tol["dead_time_s"])
    worst = max((abs(v) for v in before), default=0.0)
    return worst <= 1e-9 and after > 1e-3, after, 0.0, f"|a| <= 1e-9 until {0.5 + hc.DEAD - tol['dead_time_s']:.3f} s (max {worst:.2e}), moving at +2 ms"


@check("step_lag", "handcalc.lag63")
def _lag63(sim, e, tol):
    t = 0.5 + hc.DEAD + hc.TAU
    m, x = sim.at(s(e, "a"), t), hc.lag_step(2.0, hc.TAU)
    return _near(m, x, tol["lag63_abs"]), m, x, f"a at dead time + tau ({t:.3f} s) = 63.2 % of the 2 m/s2 step"


@check("friction_windup", "handcalc.friction_limit")
def _fric(sim, e, tol):
    lim = -0.8 * hc.G
    m = min(sim.window(s(e, "a"), 0, 1.0))
    return m >= lim - tol["limit_abs"] and _near(sim.at(s(e, "a"), 0.99), lim, 1e-3), m, lim, "a never below -mu*g; held at -mu*g"


@check("friction_windup", "handcalc.no_windup")
def _windup(sim, e, tol):
    t = 1.0 + hc.DEAD + hc.TAU
    m, x = sim.at(s(e, "a"), t), -0.8 * hc.G * math.exp(-1.0)
    return _near(m, x, tol["lag63_abs"]), m, x, "recovery from the friction limit starts AT the limit (no wound-up state)"


@check("accel_limit", "handcalc.accel_limit")
def _alim(sim, e, tol):
    m = max(sim.series(s(e, "a")))
    return m <= hc.MAX_ACCEL + tol["limit_abs"] and _near(sim.at(s(e, "a"), 3.0), hc.MAX_ACCEL, 1e-3), m, hc.MAX_ACCEL, "a <= max_accel"


@check("weak_brake", "handcalc.weak_brake")
def _weak(sim, e, tol):
    m = sim.at(s(e, "a"), 1.95)
    return _rel(m, -2.0, tol["steady_rel"]), m, -2.0, "brake_factor 0.5 halves a -4 m/s2 request"


@check("weak_brake", "handcalc.weak_brake_positive")
def _weak_pos(sim, e, tol):
    m = sim.at(s(e, "a"), 3.9)
    return _rel(m, 1.0, tol["steady_rel"]), m, 1.0, "a positive request is not scaled by brake_factor"


@check("backup_brake", "handcalc.backup_brake")
def _backup(sim, e, tol):
    t = 0.5 + hc.BACKUP_DEAD + hc.TAU
    m, x = sim.at(s(e, "a"), t), -hc.lag_step(2.0, hc.TAU)
    quiet = max(abs(v) for v in sim.window(s(e, "a"), 0.5, 0.5 + hc.BACKUP_DEAD - tol["dead_time_s"]))
    return _near(m, x, tol["lag63_abs"]) and quiet <= 1e-9, m, x, "backup brake: 50 ms dead time, then the same lag"


@check("stop_distance", "handcalc.stop_distance")
def _stopd(sim, e, tol):
    _, x = hc.stop_with_lag(30 / 3.6, 3.0)
    m = sim.at(s(e, "x"), 5.9)
    return _rel(m, x, tol["distance_rel"]), m, x, "distance from 30 km/h with -3 m/s2 through dead time + lag (closed form)"


@check("stop_distance", "handcalc.stop_time")
def _stopt(sim, e, tol):
    t_exp, _ = hc.stop_with_lag(30 / 3.6, 3.0)
    m = _first_time(sim, s(e, "v"), lambda v: v <= 1e-6)
    return _near(m, t_exp, tol["time_s"]), m, t_exp, "time to standstill"


@check("grade_coast", "handcalc.grade_distance")
def _grade(sim, e, tol):
    _, x = hc.coast_on_grade(5.0, 10.0)
    m = sim.at(s(e, "x"), 5.9)
    return _rel(m, x, tol["distance_rel"]), m, x, "uphill coast 10 %: v0^2 / (2 g grade)"


@check("grade_coast", "handcalc.never_negative")
def _neg(sim, e, tol):
    m = min(sim.series(s(e, "v")))
    return m >= -tol["standstill_abs"], m, 0.0, "v >= 0 (no rolling back after the stop)"


@check("standstill_uphill", "handcalc.standstill")
def _still(sim, e, tol):
    m = max(abs(v) for v in sim.series(s(e, "v")))
    return m <= tol["standstill_abs"] and abs(sim.at(s(e, "x"), 5.9)) <= tol["standstill_abs"], m, 0.0, "at rest on 5 % with brake request: stays at rest"


@check("steer_ramp", "handcalc.steer_rate")
def _sramp(sim, e, tol):
    t_exp = 0.5 + hc.steer_reach(9.5, 10.0)
    m = _first_time(sim, s(e, "delta"), lambda d: d >= 9.5)
    return _near(m, t_exp, tol["time_s"]), m, t_exp, "rate-limited ramp (20 deg/s) + 50 ms lag reaches 9.5 deg"


@check("steer_clamp", "handcalc.steer_clamp")
def _sclamp(sim, e, tol):
    m = sim.at(s(e, "delta"), 5.9)
    return _rel(m, 30.0, tol["steady_rel"]), m, 30.0, "a 45 deg request is clamped to max_steer 30 deg"


@check("steer_stuck", "handcalc.steer_stuck")
def _sstuck(sim, e, tol):
    d0, d1 = sim.at(s(e, "delta"), 0.71), sim.at(s(e, "delta"), 3.0)
    return abs(d1 - d0) <= 1e-6 and d0 > 0.5, d1 - d0, 0.0, "a stuck actuator freezes delta (moved before, frozen after)"


@check("yaw_kinematics", "handcalc.yaw_rate")
def _yaw(sim, e, tol):
    x = hc.yaw_rate(10.0, 5.0)
    m = sim.at(s(e, "yaw_rate"), 3.0)
    return _rel(m, x, tol["steady_rel"]), m, x, "yaw_rate = v / L * tan(delta)"


@check("dyn_coast", "handcalc.dyn_resistance")
def _dcoast(sim, e, tol):
    x = hc.dynamic_decel(10.0)
    m = sim.at(s(e, "a_veh"), 0.002)
    return _rel(m, x, 0.01), m, x, "coast-down at 10 m/s: (crr N + aero) / m_eff"


@check("dyn_payload", "handcalc.dyn_payload")
def _dpay(sim, e, tol):
    t = 1.5
    v = sim.at(s(e, "v"), t)
    x = hc.dynamic_decel(v, a_cmd=-3.0, payload=800)
    m = sim.at(s(e, "a_veh"), t)
    return _rel(m, x, tol["steady_rel"]), m, x, "force on the curb mass, divided by the loaded mass (payload 800 kg)"


@check("dyn_grade", "handcalc.dyn_grade")
def _dgrade(sim, e, tol):
    x = hc.dynamic_decel(10.0, grade_pct=10.0)
    m = sim.at(s(e, "a_veh"), 0.002)
    return _rel(m, x, 0.01), m, x, "10 % grade: m g sin(atan(0.1)) + resistances"


def model_source(experiments: list[Experiment] | None = None) -> str:
    """The generated `AllExperiments` model (one plant instance per experiment)."""
    exps = experiments or E
    body = "\n".join(x.modelica() for x in exps)
    eqs = "\n".join(line for x in exps for line in x.equations())
    return f"model AllExperiments \"Generated by plant/ci/experiments.py; do not edit\"\n{body}\nequation\n{eqs}\nend AllExperiments;\n"


def output_regex() -> str:
    return "(" + "|".join(x.id for x in E) + ")\\\\.(" + "|".join(OUTPUTS) + ")"
