"""Hand calculations: closed-form or simple-arithmetic expectations, written WITHOUT the solver, for the L1 checks.

Each function states the physics it assumes, so a disagreement with the solver says which side is wrong.
Parameter values mirror plant/spec/plant_spec.md section 3 (placeholders, illustrative).
"""
from __future__ import annotations

import math

G = 9.81
DEAD = 0.080
TAU = 0.1
BACKUP_DEAD = 0.050
MAX_ACCEL = 2.5
STEER_RATE = 20.0
STEER_TAU = 0.05
WHEELBASE = 2.7
MASS, CRR, CDA, RHO, ROT = 1500.0, 0.015, 0.7, 1.2, 0.04


def lag_step(amplitude: float, t_since: float, tau: float = TAU) -> float:
    """First-order lag response to a step applied t_since seconds ago."""
    return amplitude * (1.0 - math.exp(-t_since / tau)) if t_since > 0 else 0.0


def stop_with_lag(v0: float, decel: float, dead: float = DEAD, tau: float = TAU) -> tuple[float, float]:
    """Stop time and distance for a constant deceleration request `decel` applied at t = 0 through dead time + lag.

    After the dead time, a(t') = -decel (1 - exp(-t'/tau)), so
      v(t') = v0 - decel (t' - tau (1 - exp(-t'/tau)))
      x(t') = v0 dead + v0 t' - decel (t'^2/2 - tau t' + tau^2 (1 - exp(-t'/tau)))
    The stop is the root of v(t') = 0 (bisection; no solver of the model involved).
    """
    def v(tp: float) -> float:
        return v0 - decel * (tp - tau * (1.0 - math.exp(-tp / tau)))

    lo, hi = 0.0, v0 / decel + 10 * tau
    for _ in range(200):
        mid = (lo + hi) / 2
        if v(mid) > 0:
            lo = mid
        else:
            hi = mid
    tp = (lo + hi) / 2
    x = v0 * dead + v0 * tp - decel * (tp * tp / 2 - tau * tp + tau * tau * (1.0 - math.exp(-tp / tau)))
    return dead + tp, x


def coast_on_grade(v0: float, grade_pct: float) -> tuple[float, float]:
    """Kinematic model, no actuator input: constant deceleration g*grade/100 (small-angle form, spec R8)."""
    d = G * grade_pct / 100.0
    return v0 / d, v0 * v0 / (2.0 * d)


def steer_reach(level_deg: float, step_deg: float, rate: float = STEER_RATE, tau: float = STEER_TAU) -> float:
    """Time after a steering step until delta reaches `level_deg`: a rate-limited ramp to `step_deg`, then a first-order lag.

    During the ramp (T = step/rate) delta follows a lagged ramp: rate (t - tau (1 - exp(-t/tau))).
    After it, delta approaches step_deg exponentially from delta(T).
    """
    T = step_deg / rate

    def ramp(t: float) -> float:
        return rate * (t - tau * (1.0 - math.exp(-t / tau)))

    if ramp(T) >= level_deg:
        lo, hi = 0.0, T
        for _ in range(200):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if ramp(mid) < level_deg else (lo, mid)
        return (lo + hi) / 2
    d_T = ramp(T)
    return T + tau * math.log((step_deg - d_T) / (step_deg - level_deg))


def yaw_rate(v: float, delta_deg: float, wheelbase: float = WHEELBASE) -> float:
    return v / wheelbase * math.tan(math.radians(delta_deg))


def dynamic_decel(v: float, grade_pct: float = 0.0, a_cmd: float = 0.0, payload: float = 0.0) -> float:
    """Force balance of the dynamic model (spec R10): (a_cmd*m_curb - crr*N - aero - m g sin(theta)) / m_eff."""
    m = MASS + payload
    m_eff = m * (1.0 + ROT)
    th = math.atan(grade_pct / 100.0)
    n = m * G * math.cos(th)
    f_x = max(-0.8 * n, min(0.8 * n, a_cmd * MASS))
    f_res = CRR * n + 0.5 * RHO * CDA * v * v if v > 0 else 0.0
    return (f_x - f_res - m * G * math.sin(th)) / m_eff
