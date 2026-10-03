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
            if status in e2e.VALID and state == "VALID":
                self.cmd, self.last_valid = decode_act(payload), t
        if not self.fallback and (t - self.last_valid > self.timeout or self.sm.state == "INVALID"):
            self.fallback, self.t_fallback = True, t
        if self.fallback:
            self.decel_now = min(self.decel, self.decel_now + 6.0 * 0.001)  # jerk-limited ramp
            return -self.decel_now, self.cmd[1], False
        return self.cmd
