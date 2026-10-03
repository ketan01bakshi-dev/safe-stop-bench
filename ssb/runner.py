"""Run one scenario: planner → planner bus → DUT → actuator bus → actuator ECU → actuators → vehicle.

The oracle has four parts: expected reaction from the scenario (traced to requirements), invariants checked
every millisecond, physics (the vehicle model), and optionally a reference DUT for back-to-back comparison.
"""
from __future__ import annotations

import random

from . import e2e
from .bus import VirtualBus
from .dut import DeviceUnderTest, ReferenceDUT
from .planner import CMD_ID, Planner, active
from .plant import ActuatorECU, Actuators, Vehicle
from .safety import LATCHED, RANK

DATA_ID = 0x1234


def _faults(sc, kind, t):
    return [f for f in sc.get("faults", []) if f["type"] == kind and active(f, t)]


def run(sc: dict, cfg: dict, dut: DeviceUnderTest | None = None, seed: int = 7, keep_trace: bool = True) -> dict:
    rng = random.Random(seed)
    road = dict(cfg["road"])
    if "friction" in sc:
        road["mu"] = cfg["frictions"][sc["friction"]]
    if "grade_pct" in sc:
        road["grade_pct"] = sc["grade_pct"]
    v0 = sc.get("start_kmh", 30) / 3.6
    dut = dut or ReferenceDUT(cfg, warm_start=v0 > 0)
    if hasattr(dut, "prepare"):
        dut.prepare(warm_start=v0 > 0, defects=dut.defects)
    realtime = getattr(dut, "realtime", False)
    import time as _time
    if realtime:
        from . import rt
        rt.boost()   # high priority + 1 ms timer on Windows (a stalled bench delays frames to a free-running DUT)
    t_wall0 = _time.perf_counter()
    max_lag_ms = 0.0
    planner = Planner(sc, cfg, rng, DATA_ID)
    bp = VirtualBus("planner", cfg["buses"]["planner"]["frames_per_ms"], cfg["buses"]["planner"]["latency_ms"], rng)
    ba = VirtualBus("actuator", cfg["buses"]["actuator"]["frames_per_ms"], cfg["buses"]["actuator"]["latency_ms"], rng)
    ecu, act, veh = ActuatorECU(cfg), Actuators(cfg), Vehicle(cfg, v0)
    releases = {e["t"] for e in sc.get("events", []) if e["type"] == "release"}
    lim = cfg["safety"]

    out = None
    frames_for_dut: list = []
    kicks_buf: list[int] = []
    inv: list[str] = []
    states_seen: list[tuple[int, str]] = []
    trace, max_lat, max_jerk, a_prev, x_max = [], 0.0, 0.0, 0.0, 0.0
    time_in = {}
    released_ok = False
    t_ecu = None
    last_rank, latched_since = 0, None
    prev_out_steer = 0.0

    for t in range(0, sc.get("duration_ms", 8000) + 1):
        if realtime:  # pace 1 simulated ms per wall-clock ms for a real-time DUT
            while _time.perf_counter() - t_wall0 < t / 1000:
                pass
            # a bench that falls behind real time sends 'old' commands to a DUT with its own clock: measure it
            max_lag_ms = max(max_lag_ms, (_time.perf_counter() - t_wall0) * 1000 - t)
        # fault hooks on buses and plant
        bp.jitter = max([f["ms"] for f in _faults(sc, "jitter", t)], default=0)
        bp.extra_latency = max([f["ms"] for f in _faults(sc, "latency", t)], default=0)
        if _faults(sc, "flood", t):
            for _ in range(bp.capacity + 1):
                if len(bp.queue) < 50:
                    bp.send(t, "babbler", 0x001, bytes(8))
        ba.bus_off = {"safety"} if _faults(sc, "act_bus_off", t) else set()
        act.steer_stuck = bool(_faults(sc, "steer_stuck", t))
        act.steer_rate_factor = min([f["factor"] for f in _faults(sc, "steer_slow", t)], default=1.0)
        act.brake_factor = min([f["factor"] for f in _faults(sc, "brake_weak", t)], default=1.0)
        power_ok = not _faults(sc, "safety_brownout", t)

        challenge = out.challenge if out else 0x5A
        mrm_req = out.mrm_request if out else None
        frames, kicks = planner.step(t, veh, challenge, mrm_req)
        for fr in frames:
            bp.send(t, "planner", CMD_ID, fr, fd=True)
        frames_for_dut = bp.step(t)
        # grade_accel: what an IMU-based pitch estimate gives the safety controller (gravity along the slope)
        fb = {"v": veh.v, "a": veh.a, "delta": act.delta, "yaw_rate": veh.yaw_rate, "grade_accel": 9.81 * road["grade_pct"] / 100.0}
        out = dut.step(t, frames_for_dut, kicks, fb, t in releases, power_ok, not ba.bus_off)
        if out.state == "INIT" and any(0 <= t - rt <= 100 for rt in releases):
            released_ok = True
        for can_id, data in out.act_frames:
            if _faults(sc, "act_frame_corrupt", t):
                data = e2e.flip_crc(data)
            ba.send(t, "safety", can_id, data)
        a_cmd, s_cmd, backup = ecu.step(t, ba.step(t))
        if ecu.fallback and t_ecu is None:
            t_ecu = t
        a = act.step(a_cmd, s_cmd, cfg["plant"]["backup_decel"] if backup else 0.0, road["mu"], 0.001)
        veh.step(a, act.delta, road["grade_pct"], 0.001)

        # invariants (every ms)
        st = out.state
        rank = RANK.get(st, 0)
        if not states_seen or states_seen[-1][1] != st:
            states_seen.append((t, st))
        time_in[st] = time_in.get(st, 0) + 1
        oa, os_, _ = out.out_cmd
        if st in LATCHED and oa > 0.0 and states_seen and t - states_seen[-1][0] > 10:  # one 10 ms cycle to react
            inv.append(f"{t} ms: positive accel {oa:.2f} commanded during {st}")
        if st == "INIT" and oa > 0.0:
            inv.append(f"{t} ms: positive accel commanded in INIT")
        if last_rank >= 2 and rank < 2 and st != "INIT" and st != "OFF":
            inv.append(f"{t} ms: left a latched stop without release ({st})")
        if st not in ("OFF",):
            last_rank = rank if st != "INIT" else 0
        if t % 20 == 0:
            # output steering slew over one command period; skipped across state changes and in
            # BRAKE_ONLY_STOP (steering not driven), INIT and OFF
            skip = st in ("BRAKE_ONLY_STOP", "INIT", "OFF") or t < 100 or (states_seen and t - states_seen[-1][0] < 40)
            slew = abs(os_ - prev_out_steer) / 0.02
            if not skip and slew > lim["steer_rate_max_dps"] + 1.0:
                inv.append(f"{t} ms: output steering slew {slew:.0f} deg/s above the envelope")
            prev_out_steer = os_
        if veh.v * 3.6 > lim["odd_max_kmh"] + 2.0 and sc.get("start_kmh", 30) <= lim["odd_max_kmh"]:
            inv.append(f"{t} ms: speed {veh.v*3.6:.1f} km/h above the ODD limit")
        max_lat = max(max_lat, abs(veh.y))
        x_max = max(x_max, veh.x)
        if t % 10 == 0:
            max_jerk = max(max_jerk, abs(veh.a - a_prev) / 0.01)
            a_prev = veh.a
        if keep_trace and t % 20 == 0:
            trace.append((t, round(veh.v * 3.6, 2), round(veh.y, 3), st, round(veh.x, 2)))

    sc_dut = getattr(dut, "sc", None)
    result = {
        "id": sc.get("id"), "key": sc["key"], "title": sc.get("title", ""), "req": sc.get("req", []),
        "expect": sc.get("expect", {}), "dut": dut.identity(), "seed": seed,
        "states": states_seen, "final_state": out.state, "cause": out.cause,
        "t_fault": getattr(sc_dut, "t_fault", None), "ecu_fallback": ecu.fallback, "t_ecu": t_ecu,
        "events": list(getattr(sc_dut, "events", [])), "dtcs": list(getattr(sc_dut, "dtcs", [])),
        "release_rejected": getattr(sc_dut, "release_rejected", 0), "released_ok": released_ok,
        "rejected_frames": getattr(sc_dut, "rejected", 0), "ecu_rejected": ecu.rejected,
        "v_end_kmh": round(veh.v * 3.6, 2), "x_max": round(x_max, 2), "y_end": round(veh.y, 2),
        "max_lateral_m": round(max_lat, 3), "max_jerk": round(max_jerk, 1),
        "bus_load_max": round(bp.max_load, 2), "time_in_states_ms": time_in,
        "invariant_violations": inv[:10], "invariant_count": len(inv), "trace": trace, "bench_lag_ms": round(max_lag_ms, 1) if realtime else None,
    }
    return result
