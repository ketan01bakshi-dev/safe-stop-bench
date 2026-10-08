"""A stand-alone virtual ECU: the reference safety controller in its own process, reachable ONLY over CAN.

This is what a supplied vECU looks like to the bench: a black box with a DBC. Run it with
    python -m ssb.vecu_process [--interface udp_multicast --channel 239.74.163.2]
The bench (CanDUT) drives time through VEH_SimTime (a shared time base), feeds sensors, commands and watchdog kicks,
and reads actuator commands and status back.

Lockstep (v2.11, BENCH_Lockstep = 1): the bench is not paced; it sends every input for ms T (commands, kicks, control)
and then VEH_Feedback(T), and waits at each 10 ms boundary for that cycle's status. So the vECU handles messages
strictly in order: cycle T runs as soon as VEH_Feedback(T) arrives (it has seen every input up to T), and a kick or
release that arrives after VEH_Feedback(T) belongs to ms T+1. Host stalls then slow the run down but cannot change it.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

from . import canio, config
from .safety import CYCLE_MS, MUTANTS, SafetyController


class Frame:  # the minimal shape SafetyController.cycle expects
    def __init__(self, can_id: int, data: bytes):
        self.can_id, self.data = can_id, data


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--interface", default="udp_multicast")
    ap.add_argument("--channel", default=canio.GROUP)
    ap.add_argument("--config", default="config/default.json")
    a = ap.parse_args()
    cfg = config.load(a.config)
    db = canio.load_dbc()
    bus = canio.DedupBus(a.interface, a.channel)
    mutant_names = list(MUTANTS)
    sc: SafetyController | None = None
    t_now, t_wrap, last_raw, next_cycle = 0, 0, 0, 0
    pending: list = []
    fb = {"v": 0.0, "a": 0.0, "delta": 0.0, "yaw_rate": 0.0, "grade_accel": 0.0}
    power_ok, tx_ok, last_release = True, True, 0
    status_ctr = 0   # SAF_Status alive counter (v2.10): +1 per frame, back to 0 on a controller reset
    lockstep = False

    def run_cycles() -> None:
        nonlocal next_cycle, pending, status_ctr
        sc.brownout(t_now, not power_ok)
        sc.tx_ok = tx_ok
        while next_cycle <= t_now:
            act = sc.cycle(next_cycle, pending, fb)
            pending = []
            for can_id, data in act:
                bus.send(can_id, data)
            a_out, s_out, _ = sc.out
            bus.send(canio.ID["status"], canio.encode_status(db, status_ctr, sc.state if sc.powered else "OFF", sc.cause,
                                                             sc.challenge, sc.mrm_request == "PULL_OVER", a_out, s_out))
            status_ctr += 1
            next_cycle += CYCLE_MS
    hello_t = time.time()
    DEBUG = bool(os.environ.get("SSB_DEBUG"))
    dbg_t, dbg_loops = time.time(), 0
    while True:
        for m in bus.recv_all():
            i, d = m.arbitration_id, bytes(m.data)
            if i == canio.ID["ctrl"]:
                c = db.decode_message(i, d, decode_choices=False)
                lockstep = bool(c.get("BENCH_Lockstep", 0))
                if c["BENCH_Reset"]:
                    defects = frozenset(n for k, n in enumerate(mutant_names) if int(c["BENCH_Defects"]) >> k & 1)
                    sc = SafetyController(cfg, defects, warm_start=c["BENCH_Reset"] == 1)
                    t_now = t_wrap = last_raw = next_cycle = 0
                    pending, last_release = [], 0
                    status_ctr = 0
                    if lockstep:   # acknowledge at once; cycle 0 runs when VEH_Feedback(0) brings the inputs for t = 0
                        bus.send(canio.ID["status"], canio.encode_status(db, 0, sc.state, sc.cause, sc.challenge,
                                                                         sc.mrm_request == "PULL_OVER", 0.0, 0.0))
                        status_ctr = 1
                if sc:
                    if c["BENCH_Release"] and not last_release:
                        sc.release(t_now + (1 if lockstep else 0), fb["v"])
                    last_release = c["BENCH_Release"]
                    power_ok, tx_ok = bool(c["BENCH_PowerOk"]), bool(c["BENCH_TxOk"])
            elif sc is None:
                continue
            elif i == canio.ID["fb"]:
                f = db.decode_message(i, d)
                raw = int(f["VEH_SimTime"])
                if raw < last_raw - 32768:      # a real wrap, not a late or duplicated message
                    t_wrap += 65536
                if raw > last_raw or raw < last_raw - 32768:
                    last_raw, t_now = raw, t_wrap + raw
                if os.environ.get("SSB_DEBUG") == "2" and (t_now < 150 and raw % 10 == 0):
                    print(f"[vecu] fb raw={raw} t_now={t_now}", file=sys.stderr, flush=True)
                fb = {"v": f["VEH_Speed"], "a": f["VEH_LongAccel"], "delta": f["VEH_RoadWheelAngle"],
                      "yaw_rate": f["VEH_YawRate"], "grade_accel": f["VEH_GradeAccel"]}
                if lockstep:
                    run_cycles()   # every input up to t_now has arrived: the bench sends VEH_Feedback last
            elif i == canio.ID["kick"]:
                sc.kick(t_now + (1 if lockstep else 0))
                if os.environ.get("SSB_DEBUG") == "2" and t_now < 150:
                    print(f"[vecu] kick at t_now={t_now} data={d.hex()}", file=sys.stderr, flush=True)
                n_kicks = locals().get("n_kicks", 0) + 1
            elif i == canio.ID["cmd"]:
                pending.append(Frame(i, d[:14]))   # strip CAN FD padding
        if sc is None:
            if time.time() - hello_t > 0.2:   # announce readiness until the bench resets us
                bus.send(canio.ID["status"], canio.encode_status(db, status_ctr, "OFF", None, 0, False, 0.0, 0.0))
                status_ctr += 1
                hello_t = time.time()
            time.sleep(0.0005)
            continue
        if DEBUG:
            dbg_loops += 1
            if time.time() - dbg_t > 1.0:
                print(f"[vecu] t={t_now} kicks={locals().get('n_kicks')} last_kick={sc.last_kick} loops/s={dbg_loops} state={sc.state} events={sc.events[-2:]} dups={bus.duplicates}", file=sys.stderr, flush=True)
                dbg_t, dbg_loops = time.time(), 0
        if not lockstep:
            run_cycles()
        # no sleep: Windows sleep granularity (~10 ms) would make a 10 ms controller run late


if __name__ == "__main__":
    main()
