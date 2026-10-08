"""Run one scenario: planner → planner bus → DUT → actuator bus → actuator ECU → actuators → vehicle.

The oracle has four parts: expected reaction from the scenario (traced to requirements), invariants checked
every millisecond, physics (the vehicle model), and optionally a reference DUT for back-to-back comparison.
"""
from __future__ import annotations

import random

from . import e2e
from .bus import VirtualBus
from .dut import BenchFault, DeviceUnderTest, Outputs, ReferenceDUT
from .planner import CMD_ID, Planner, active
from .plant import ActuatorECU, Actuators, grade_accel, make_vehicle
from .safety import LATCHED, RANK

DATA_ID = 0x1234


def _faults(sc, kind, t):
    return [f for f in sc.get("faults", []) if f["type"] == kind and active(f, t)]


BENCH_RETRIES = 2
LAG_LIMIT_MS = 5.0   # a real-time run is trusted only if the bench never fell further behind than this


def run(sc: dict, cfg: dict, dut: DeviceUnderTest | None = None, seed: int = 7, keep_trace: bool = True) -> dict:
    """One scenario. A bench fault (observer lost, bus node reset) is not a DUT verdict: recover the bench and run the
    scenario again, up to BENCH_RETRIES times, and record the faults in the result (v2.9.3, found on hardware).
    v2.9.5: a host stall (bench lag above LAG_LIMIT_MS) is a bench fault too. A late bench sends old commands to a DUT
    with its own clock, so its verdict is untrustworthy either way: rerun it. If every attempt stalls, the last result
    is returned and the oracle's lag check fails it, as before: a verdict is never taken from a run that stalled."""
    faults: list[str] = []
    while True:
        try:
            r = _run_once_gc_paused(sc, cfg, dut, seed, keep_trace)
        except BenchFault as e:
            if len(faults) >= BENCH_RETRIES or not hasattr(dut, "recover"):
                raise
            faults.append(str(e)[:200])
            print(f"  bench fault in {sc['key']} ({e}); recovering the bench and rerunning", flush=True)
            dut.recover()   # type: ignore[union-attr]
            continue
        lag = r.get("bench_lag_ms")
        if lag is not None and lag > LAG_LIMIT_MS and len(faults) < BENCH_RETRIES:
            faults.append(f"host stall: bench lag {lag} ms > {LAG_LIMIT_MS:g} ms at t = {r.get('bench_lag_at_ms')} ms")
            print(f"  bench fault in {sc['key']} ({faults[-1]}); rerunning", flush=True)
            continue
        r["bench_faults"] = faults
        return r


def _run_once_gc_paused(sc: dict, cfg: dict, dut: DeviceUnderTest | None, seed: int, keep_trace: bool) -> dict:
    """Real-time runs: no cyclic garbage collection inside the 1 ms loop (a collection can pause the bench for
    milliseconds); collect between scenarios instead. Reference counting still frees memory meanwhile (v2.9.5)."""
    import gc
    if not getattr(dut, "realtime", False) or not gc.isenabled():
        return _run_once(sc, cfg, dut, seed, keep_trace)
    gc.collect()
    gc.disable()
    try:
        return _run_once(sc, cfg, dut, seed, keep_trace)
    finally:
        gc.enable()


def _run_once(sc: dict, cfg: dict, dut: DeviceUnderTest | None, seed: int, keep_trace: bool) -> dict:
    rng = random.Random(seed)
    road = dict(cfg["road"])
    if "friction" in sc:
        road["mu"] = cfg["frictions"][sc["friction"]]
    if "grade_pct" in sc:
        road["grade_pct"] = sc["grade_pct"]
    v0 = sc.get("start_kmh", 30) / 3.6
    dut = dut or ReferenceDUT(cfg, warm_start=v0 > 0)
    realtime = getattr(dut, "realtime", False)
    # v2.9.7: a real reset (EN pin) of a board that supports it; any other DUT gets a power loss as long as the board's
    # measured boot, the closest equivalent it can model
    hw_resets = {f["start"] for f in sc.get("faults", []) if f["type"] == "safety_hw_reset"}
    real_reset = bool(hw_resets) and getattr(dut, "can_hw_reset", False)
    boot_ms = cfg.get("dut_hw", {}).get("reset_boot_ms", 186)
    import time as _time
    max_lag_ms = 0.0
    lag_at_ms = None
    planner = Planner(sc, cfg, rng, DATA_ID)
    bp = VirtualBus("planner", cfg["buses"]["planner"]["frames_per_ms"], cfg["buses"]["planner"]["latency_ms"], rng)
    ba = VirtualBus("actuator", cfg["buses"]["actuator"]["frames_per_ms"], cfg["buses"]["actuator"]["latency_ms"], rng)
    ecu, act, veh = ActuatorECU(cfg), Actuators(cfg), make_vehicle(cfg, v0, sc.get("payload_kg"))
    veh.mu = road["mu"]
    releases = {e["t"] for e in sc.get("events", []) if e["type"] == "release"}
    lim = cfg["safety"]

    out: Outputs | None = None
    frames_for_dut: list = []
    inv: list[str] = []
    states_seen: list[tuple[int, str]] = []
    trace, max_lat, max_jerk, a_prev, x_max = [], 0.0, 0.0, 0.0, 0.0
    time_in: dict[str, int] = {}
    released_ok = False
    t_ecu = None
    last_rank = 0
    prev_out_steer = 0.0
    prev_cyc: tuple | None = None

    # v2.9.5: all set-up above, THEN reset the DUT, THEN start the clock. A board with its own clock starts its cycle
    # at the reset; set-up done after it (e.g. loading a replay log) was a head start the lag guard never saw, and it
    # showed up as the run's worst lag at t = 0-1 ms (up to 8.4 ms, measured on the boards).
    if realtime:
        from . import rt
        rt.boost()   # high priority + 1 ms timer on Windows (a stalled bench delays frames to a free-running DUT)
    if hasattr(dut, "prepare"):
        dut.prepare(warm_start=v0 > 0, defects=dut.defects)
    t_wall0 = _time.perf_counter()

    for t in range(0, sc.get("duration_ms", 8000) + 1):
        if realtime:  # pace 1 simulated ms per wall-clock ms for a real-time DUT
            while _time.perf_counter() - t_wall0 < t / 1000:
                pass
            # a bench that falls behind real time sends 'old' commands to a DUT with its own clock: measure it
            lag = (_time.perf_counter() - t_wall0) * 1000 - t
            if lag > max_lag_ms:
                max_lag_ms, lag_at_ms = lag, t
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
        power_ok = not _faults(sc, "safety_brownout", t) and (real_reset or not any(s <= t < s + boot_ms for s in hw_resets))
        if real_reset and t in hw_resets:
            dut.hw_reset(t)   # type: ignore[attr-defined]

        challenge = out.challenge if out else 0x5A
        mrm_req: str | None = out.mrm_request if out else None
        frames, kicks = planner.step(t, veh, challenge, mrm_req)
        for fr in frames:
            bp.send(t, "planner", CMD_ID, fr, fd=True)
        frames_for_dut = bp.step(t)
        # grade_accel: what an IMU-based pitch estimate gives the safety controller (gravity along the slope)
        fb = {"v": veh.v, "a": veh.a, "delta": act.delta, "yaw_rate": veh.yaw_rate, "grade_accel": grade_accel(veh, road["grade_pct"])}
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
        # output steering slew over one command period; skipped across state changes and in
        # BRAKE_ONLY_STOP (steering not driven), INIT and OFF
        skip = st in ("BRAKE_ONLY_STOP", "INIT", "OFF") or t < 100 or (states_seen and t - states_seen[-1][0] < 40)
        if out.cycle is not None:
            # HiL/PiL: on the DUT's own cycle clock, so link jitter cannot fake a fast slew
            c_ms, c_steer = out.cycle
            if prev_cyc is None or c_ms < prev_cyc[0]:
                prev_cyc = out.cycle
            elif c_ms - prev_cyc[0] >= 20:
                slew = abs(c_steer - prev_cyc[1]) / ((c_ms - prev_cyc[0]) / 1000)
                if not skip and slew > lim["steer_rate_max_dps"] + 1.0:
                    inv.append(f"{t} ms: output steering slew {slew:.0f} deg/s above the envelope (DUT cycle {c_ms} ms)")
                prev_cyc = out.cycle
        elif t % 20 == 0:
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

    assert out is not None   # the loop runs at least once (t = 0)
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
        "bench_lag_at_ms": lag_at_ms if realtime else None,
        "bus_replays": getattr(dut, "n_replay", None),
        "status_e2e_rejects": getattr(dut, "n_status_crc", 0) + getattr(dut, "n_status_seq", 0) if hasattr(dut, "n_status_crc") else None,
    }
    return result
