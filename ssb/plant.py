"""Plant: kinematic bicycle vehicle, actuator dynamics, actuator ECUs with their own fallback.

Illustrative parameters, not a calibrated vehicle.
"""
from __future__ import annotations

import math
import struct
from collections import deque

from . import e2e

G = 9.81


class Actuators:
    """Longitudinal path (dead time + first-order lag, friction limit) + steering (rate limit + lag) + backup brake."""

    def __init__(self, cfg: dict):
        p = cfg["plant"]
        self.p = p
        self.dead = deque([0.0] * max(1, p["long_dead_time_ms"]), maxlen=max(1, p["long_dead_time_ms"]))
        self.a_act = 0.0
        self.backup_dead = deque([0.0] * 50, maxlen=50)
        self.backup = 0.0
        self.delta = 0.0          # road-wheel angle, deg
        self.steer_target = 0.0
        self.brake_factor = 1.0   # < 1 = weak brake (fault)
        self.steer_stuck = False
        self.steer_rate_factor = 1.0

    def step(self, accel_cmd: float, steer_cmd: float, backup_decel: float, mu: float, dt: float) -> float:
        p = self.p
        self.dead.append(accel_cmd)
        a_in = self.dead[0]
        if a_in < 0:
            a_in *= self.brake_factor
        self.a_act += (a_in - self.a_act) * dt / p["long_tau_s"]
        self.a_act = max(-mu * G, min(p["max_accel"], self.a_act))
        self.backup_dead.append(backup_decel)
        self.backup += (self.backup_dead[0] - self.backup) * dt / p["long_tau_s"]
        if not self.steer_stuck:
            rate = p["steer_rate_dps"] * self.steer_rate_factor * dt
            target = max(-p["max_steer_deg"], min(p["max_steer_deg"], steer_cmd))
            self.steer_target += max(-rate, min(rate, target - self.steer_target))
            self.delta += (self.steer_target - self.delta) * dt / p["steer_tau_s"]
        return max(-mu * G, self.a_act - self.backup)


class Vehicle:
    mu = 1.0   # road friction, set by the runner (used by DynamicVehicle; the kinematic model leaves it to Actuators)

    def __init__(self, cfg: dict, v0: float):
        self.L = cfg["plant"]["wheelbase_m"]
        self.x = self.y = self.psi = 0.0
        self.v = v0
        self.a = 0.0
        self.yaw_rate = 0.0

    def step(self, a: float, delta_deg: float, grade_pct: float, dt: float) -> None:
        a_total = a - G * grade_pct / 100.0
        if self.v <= 0.0 and a_total <= 0.0:
            self.v, self.a = 0.0, 0.0
        else:
            self.v = max(0.0, self.v + a_total * dt)
            self.a = a_total
        self.yaw_rate = self.v / self.L * math.tan(math.radians(delta_deg))
        self.psi += self.yaw_rate * dt
        self.x += self.v * math.cos(self.psi) * dt
        self.y += self.v * math.sin(self.psi) * dt


class DynamicVehicle(Vehicle):
    """Force-based longitudinal model (v2.5): mass + payload, rolling resistance, aero drag, rotating inertia, true slope.

    The actuators' output is a demand calibrated on the NOMINAL (curb) mass: the brake/drive ECUs turn an acceleration
    request into a force as if the vehicle were empty (an open-loop pressure/torque map, the common case without a
    deceleration-closed-loop brake controller). So a payload gives proportionally less deceleration for the same
    request, and the tyre limit scales with the real normal force (mu * m * g * cos(theta)).
    Lateral motion stays the kinematic bicycle (low-speed ODD). Like the kinematic model, v >= 0: rolling backwards
    uphill is not modelled. Illustrative parameters, not a calibrated vehicle.
    """

    RHO = 1.2  # air density, kg/m3

    def __init__(self, cfg: dict, v0: float, payload_kg: float | None = None):
        super().__init__(cfg, v0)
        d = cfg["plant"]["dynamic"]
        self.m_nom = d["mass_kg"]
        self.payload = d.get("payload_kg", 0.0) if payload_kg is None else payload_kg
        self.m = self.m_nom + self.payload
        self.m_eff = self.m * (1.0 + d["rot_inertia_factor"])
        self.crr, self.cda = d["crr"], d["cd_a_m2"]
        self.mu = 1.0      # set by the runner each step (road friction)
        self.f_x = 0.0     # tyre force actually transmitted, N

    def step(self, a: float, delta_deg: float, grade_pct: float, dt: float) -> None:
        theta = math.atan(grade_pct / 100.0)
        n = self.m * G * math.cos(theta)
        f = max(-self.mu * n, min(self.mu * n, a * self.m_nom))   # demand on curb mass, capped by the loaded tyre limit
        self.f_x = f
        f_slope = self.m * G * math.sin(theta)
        moving = self.v > 0.0
        f_res = (self.crr * n + 0.5 * self.RHO * self.cda * self.v * self.v) if moving else 0.0
        a_total = (f - f_res - f_slope) / self.m_eff
        if not moving and a_total <= 0.0:
            self.v, self.a = 0.0, 0.0
        else:
            self.v = max(0.0, self.v + a_total * dt)
            self.a = a_total
        self.yaw_rate = self.v / self.L * math.tan(math.radians(delta_deg))
        self.psi += self.yaw_rate * dt
        self.x += self.v * math.cos(self.psi) * dt
        self.y += self.v * math.sin(self.psi) * dt


def grade_accel(veh: Vehicle, grade_pct: float) -> float:
    """What an IMU pitch estimate gives the safety controller: gravity along the slope (small angle in the kinematic model)."""
    if isinstance(veh, DynamicVehicle):
        return G * math.sin(math.atan(grade_pct / 100.0))
    return 9.81 * grade_pct / 100.0


def make_vehicle(cfg: dict, v0: float, payload_kg: float | None = None) -> Vehicle:
    """cfg["plant"]["model"]: "kinematic" (default, the v2.0 behaviour) or "dynamic" (DynamicVehicle)."""
    if cfg["plant"].get("model", "kinematic") == "dynamic":
        return DynamicVehicle(cfg, v0, payload_kg)
    if payload_kg:
        raise ValueError("a payload needs plant model 'dynamic' (the kinematic model has no mass)")
    return Vehicle(cfg, v0)


ACT_ID = 0x200


def encode_act(accel: float, steer: float, backup: bool) -> bytes:
    return struct.pack("<hhB", round(accel * 100), round(steer * 100), 1 if backup else 0)


def decode_act(payload: bytes) -> tuple[float, float, bool]:
    a, s, f = struct.unpack("<hhB", payload)
    return a / 100, s / 100, bool(f & 1)


class ActuatorECU:
    """Steering + brake/drive ECU behind the safety controller. Checks Profile 2 E2E on every command and has
    its own fallback: if commands stop or turn invalid for > timeout, it stops the vehicle itself (latched)."""

    def __init__(self, cfg: dict):
        c = cfg["ecu"]
        self.timeout = c["timeout_ms"]
        self.decel = c["fallback_decel"]
        self.rx = e2e.Profile2.Receiver(max_delta=2)
        self.sm = e2e.E2EStateMachine()
        self.sm.preset_valid()
        self.last_valid = 0
        self.cmd = (0.0, 0.0, False)
        self.fallback = False
        self.t_fallback: int | None = None
        self.rejected = 0
        self.decel_now = 0.0

    def step(self, t: int, frames: list) -> tuple[float, float, bool]:
        for f in frames:
            status, payload = self.rx.check(f.data)
            state = self.sm.update(status)
            if status in e2e.ERRORS:
                self.rejected += 1
            if status in e2e.VALID and state == "VALID" and payload is not None:
                self.cmd, self.last_valid = decode_act(payload), t
        if not self.fallback and (t - self.last_valid > self.timeout or self.sm.state == "INVALID"):
            self.fallback, self.t_fallback = True, t
        if self.fallback:
            self.decel_now = min(self.decel, self.decel_now + 6.0 * 0.001)  # jerk-limited ramp
            return -self.decel_now, self.cmd[1], False
        return self.cmd
