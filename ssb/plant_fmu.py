"""The plant as an FMU (P6): the released SafeStopPlant FMU in place of ssb/plant.py's Actuators + Vehicle.

    python run.py --plant fmu:plant/releases/<ver>/SafeStopPlant_Plant.fmu

One FMI 2.0 co-simulation instance holds actuators AND vehicle. The runner keeps its two objects: `act.step()` sets the
inputs and advances the FMU by one 1 ms communication step (what Actuators.step + Vehicle.step do together in Python);
`veh.step()` is then a no-op and `veh.*` reads the FMU's outputs. The actuator ECU (E2E check, own fallback) stays in Python.
One instance per FMU file and process, fmi2Reset between scenarios (as FmuDUT does). AI-assisted, synthetic, illustrative.
"""
from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any

INPUTS = ("accel_cmd", "steer_cmd", "backup_decel", "mu", "grade_pct", "brake_factor", "steer_rate_factor")
OUTPUTS = ("a", "delta", "v", "a_veh", "yaw_rate", "x", "y", "psi")
_CACHE: dict[str, Any] = {}


class _Fmu:
    def __init__(self, path: str):
        try:
            from fmpy import extract, read_model_description
            from fmpy.fmi2 import FMU2Slave
        except ImportError as e:  # pragma: no cover
            raise RuntimeError("the FMU plant needs FMPy: pip install fmpy") from e
        self.path = path
        self.md = md = read_model_description(path, validate=True)
        if md.coSimulation is None or not md.fmiVersion.startswith("2."):
            raise ValueError(f"{Path(path).name}: needs an FMI 2.0 co-simulation FMU")
        self.vr = {v.name: v.valueReference for v in md.modelVariables}
        missing = [n for n in (*INPUTS, "steer_stuck", *OUTPUTS, "v0") if n not in self.vr]
        if missing:
            raise ValueError(f"{Path(path).name}: not a SafeStopPlant FMU, missing {missing}")
        self.dynamic = "payload" in self.vr
        self.fmu = FMU2Slave(guid=md.guid, unzipDirectory=extract(path), modelIdentifier=md.coSimulation.modelIdentifier,
                             instanceName="plant")
        self.live = False
        self.sha = hashlib.sha256(Path(path).read_bytes()).hexdigest()

    def start(self, v0: float, payload: float, mu: float, grade: float) -> None:
        if self.live:
            self.fmu.terminate()
            self.fmu.reset()
        else:
            self.fmu.instantiate()
            self.live = True
        self.fmu.setupExperiment(startTime=0.0)
        names, vals = ["v0", "mu", "grade_pct", "brake_factor", "steer_rate_factor"], [v0, mu, grade, 1.0, 1.0]
        if self.dynamic:
            names.append("payload")
            vals.append(payload)
        self.fmu.setReal([self.vr[n] for n in names], vals)
        self.fmu.setBoolean([self.vr["steer_stuck"]], [False])
        self.fmu.enterInitializationMode()
        self.fmu.exitInitializationMode()
        self.t = 0.0


class FmuPlant:
    """Build with make_plant(); gives the runner `act` and `veh` objects with the attributes it already uses."""

    def __init__(self, cfg: dict, v0: float, payload_kg: float | None, road: dict):
        path = str(Path(cfg["plant"]["fmu"]).resolve())
        if path not in _CACHE:
            _CACHE[path] = _Fmu(path)
        self.f: _Fmu = _CACHE[path]
        if payload_kg and not self.f.dynamic:
            raise ValueError("a payload needs the dynamic plant FMU (SafeStopPlant_PlantDynamic)")
        self.f.start(v0, payload_kg or 0.0, road["mu"], road["grade_pct"])
        self.grade = road["grade_pct"]
        self.act = _Act(self)
        self.veh = _Veh(self, v0)

    def advance(self, a_cmd: float, s_cmd: float, backup: float, mu: float, act: _Act) -> dict:
        f = self.f
        f.fmu.setReal([f.vr[n] for n in INPUTS], [a_cmd, s_cmd, backup, mu, self.grade, act.brake_factor, act.steer_rate_factor])
        f.fmu.setBoolean([f.vr["steer_stuck"]], [bool(act.steer_stuck)])
        f.fmu.doStep(currentCommunicationPoint=f.t, communicationStepSize=0.001)
        f.t += 0.001
        return dict(zip(OUTPUTS, f.fmu.getReal([f.vr[n] for n in OUTPUTS]), strict=True))


class _Act:
    def __init__(self, plant: FmuPlant):
        self.plant = plant
        self.delta = 0.0
        self.brake_factor = 1.0
        self.steer_stuck = False
        self.steer_rate_factor = 1.0

    def step(self, accel_cmd: float, steer_cmd: float, backup_decel: float, mu: float, dt: float) -> float:
        if abs(dt - 0.001) > 1e-12:
            raise ValueError("the FMU plant steps at 1 ms only")
        o = self.plant.advance(accel_cmd, steer_cmd, backup_decel, mu, self)
        self.delta = o["delta"]
        self.plant.veh._take(o)
        return o["a"]


class _Veh:
    """Read-only view of the FMU's vehicle outputs; step() is a no-op (the FMU advanced in act.step)."""

    def __init__(self, plant: FmuPlant, v0: float):
        self.plant = plant
        self.dynamic = plant.f.dynamic
        self.mu = 1.0
        self.v, self.a, self.yaw_rate, self.x, self.y, self.psi = v0, 0.0, 0.0, 0.0, 0.0, 0.0

    def _take(self, o: dict) -> None:
        self.v, self.a, self.yaw_rate = o["v"], o["a_veh"], o["yaw_rate"]
        self.x, self.y, self.psi = o["x"], o["y"], o["psi"]

    def step(self, a: float, delta_deg: float, grade_pct: float, dt: float) -> None:
        if grade_pct != self.plant.grade:
            raise ValueError("the FMU plant takes the road grade once per scenario")


def identity(cfg: dict) -> str:
    f = _CACHE.get(str(Path(cfg["plant"]["fmu"]).resolve()))
    return f"fmu:{Path(cfg['plant']['fmu']).name} sha256:{f.sha[:12]}" if f else f"fmu:{cfg['plant']['fmu']}"
