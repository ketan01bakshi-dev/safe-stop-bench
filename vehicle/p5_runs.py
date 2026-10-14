"""P5 stage 0: run the P5 scenarios on the reference controller (the correct build and the two seeded bugs) and derive what the
timeline shows, all of it from the traces.

    from vehicle import p5_runs
    runs = p5_runs.run_variants()            # {variant: {scenario key: result with trace}}
    p5_runs.stats(runs["correct build"]["p5_planned_stop_link_lost"])

Nothing here touches a board. The 1:k mapping to the chassis is a PLACEHOLDER (K_PLACEHOLDER) until the floor calibration of stage 3.
"""
from __future__ import annotations

from ssb import campaigns, config
from ssb.dut import ReferenceDUT
from ssb.safety import RANK

SCENARIO_FILE = "vehicle/p5_scenarios.json"
VARIANTS: dict[str, tuple[str, ...]] = {
    "correct build": (),
    "no_latch (seeded bug)": ("no_latch",),
    "long_timeout (seeded bug)": ("long_timeout",),
}
# PLACEHOLDER, not a measurement: the chassis firmware (sketch_ADAS_OTA) estimates speed as PWM/255 x 150 rpm x 20.4 cm per
# revolution = 0.51 m/s = 1.836 km/h at full PWM. Mapped to a 30 km/h model speed that is k = 0.0612. Stage 3 replaces it.
CHASSIS_FULL_PWM_KMH = 1.836
K_PLACEHOLDER = CHASSIS_FULL_PWM_KMH / 30.0
TARGET_KMH = 20.0


def run_variants(cfg: dict | None = None) -> dict[str, dict[str, dict]]:
    cfg = cfg or config.load()
    out: dict[str, dict[str, dict]] = {}
    for name, defects in VARIANTS.items():
        rows = campaigns.matrix(cfg, dut_factory=lambda sc, d=defects: ReferenceDUT(cfg, frozenset(d), warm_start=sc.get("start_kmh", 30) > 0), scenario_file=SCENARIO_FILE)
        out[name] = {r["key"]: r for r in rows}
    return out


def scenarios(cfg: dict | None = None) -> dict[str, dict]:
    """The P5 scenarios by key, with the bench defaults filled in (inject_ms, faults, title)."""
    return {sc["key"]: sc for sc in campaigns.all_scenarios(cfg or config.load(), SCENARIO_FILE)}


def state_segments(r: dict) -> list[tuple[int, int, str]]:
    """(start ms, end ms, state) from the trace."""
    tr = r["trace"]
    segs: list[tuple[int, int, str]] = []
    for t, _v, _y, st, _x in tr:
        if segs and segs[-1][2] == st:
            segs[-1] = (segs[-1][0], t, st)
        else:
            segs.append((t, t, st))
    return segs


def stats(r: dict, k: float = K_PLACEHOLDER) -> dict:
    """What the correct build and the bugs do differently, read from the trace (t, v km/h, y, state, x m, every 20 ms)."""
    tr = r["trace"]
    vmax = max(v for _t, v, _y, _s, _x in tr)
    t_peak = next(t for t, v, _y, _s, _x in tr if v == vmax)
    t_reach = next((t for t, v, _y, _s, _x in tr if v >= 0.98 * TARGET_KMH), None)
    first_stop = next((t for t, _v, _y, s, _x in tr if RANK.get(s, 0) >= 2), None)
    after = [s for t, _v, _y, s, _x in tr if first_stop is not None and t > first_stop]
    resumed = first_stop is not None and any(RANK.get(s, 0) < 2 for s in after)
    stopped = next(((t, x) for t, v, _y, _s, x in tr if t >= (first_stop or 0) and first_stop is not None and v == 0.0), None)
    end_t, end_v, _y, end_s, end_x = tr[-1]
    return {"peak_kmh": vmax, "t_peak_ms": t_peak, "t_reach_target_ms": t_reach, "t_first_stop_state_ms": first_stop,
            "resumed_after_stop_state": resumed, "t_stopped_ms": stopped[0] if stopped else None, "stop_x_m": stopped[1] if stopped else None,
            "stop_floor_m": round(stopped[1] * k, 2) if stopped else None, "end_t_ms": end_t, "end_kmh": end_v, "end_state": end_s,
            "end_x_m": end_x, "end_floor_m": round(end_x * k, 2), "verdict_passed": r["verdict"]["passed"],
            "t_detect_ms": r["verdict"].get("t_detect_ms")}


def chassis_pwm(v_kmh: float, k: float = K_PLACEHOLDER) -> int:
    """PLACEHOLDER model speed -> chassis PWM (linear through the origin; deadband and top speed are measured in stage 3)."""
    return max(0, min(255, round(v_kmh * k / CHASSIS_FULL_PWM_KMH * 255)))
