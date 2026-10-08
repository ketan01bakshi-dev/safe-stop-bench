"""Safe-stop bench v2 command line.

    python run.py                         # matrix (51 scenarios incl. boundaries) → reports/report.html
    python run.py --full                  # + sweep, false-stop rate, fuzz, mutation score (≈ 2 min)
    python run.py --defect no_latch       # run the matrix against a seeded bug
    python run.py --b2b no_latch          # back-to-back: reference vs reference-with-bug (stand-in for a supplied vECU)
    python run.py --config config/offroad.json
    python run.py --mdf                   # + MDF4 traces (.mf4) for CANape / asammdf, next to the CSV ones
    python run.py --plant dynamic --payload 600   # force-based plant: mass, payload, drag, rolling resistance, true slope
    python run.py --plant dynamic --load-sweep    # + payload x grade sweep (planned stop, weak brake) in the report
    python run.py --scenario steering_stuck --scenario brake_weak
    python run.py --dut can                # same matrix against a SEPARATE-PROCESS vECU over CAN (python-can, real time, ~9 min)
    python run.py --dut can --repeat 5     # + worst-case timing over 5 real-time runs per key scenario
    python run.py --dut can --b2b-can      # back-to-back: in-process reference vs the vECU over CAN
    python3 run.py --dut ros2 --b2b-dut    # (WSL/Linux, after sourcing ROS 2) the safety controller as a ROS 2 node, real time
    python3 run.py --dut ros2 --ros2-lockstep --b2b-dut   # same, lockstep per 10 ms cycle: immune to OS stalls
    python run.py --dut fmu                # same matrix against an FMI 2.0 co-simulation FMU (default fmu/SafeStopVecu.fmu, FMPy)
    python run.py --dut fmu --b2b-dut      # back-to-back, EXACT: in-process reference vs the FMU, state timeline to the ms
    python run.py --dut fmu --file their_vecu.fmu --mapping their_mapping.json   # a supplied FMU
    python -m ssb.fmu_inspect their_vecu.fmu --write-mapping their_mapping.json --lifecycle   # intake checks first
    python run.py --dut native --b2b-dut   # the ESP32 firmware's C++ core built for the PC: EXACT back-to-back vs Python
    python run.py --dut loopback --b2b-dut # board B's whole node logic (core + link protocol) on the PC, lockstep
    python run.py --dut pil --port-b COM14                  # one ESP32-S3 board (processor in the loop), real time
    python run.py --dut hil --port-b COM14 --port-a COM13   # two boards: B's outputs read through the real CAN bus
    python -m ssb.hil --ports                                # which COM port is which board

Exit code 1 if any matrix scenario fails (CI gate).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from ssb import campaigns, config, report
from ssb.dut import ReferenceDUT
from ssb.safety import MUTANTS

CHART_KEYS = ["command_link_lost", "planner_hang", "steering_stuck", "brake_weak", "odd_exit", "perception_degraded",
              "release_procedure", "actuator_frame_corrupt"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="config/default.json")
    ap.add_argument("--defect", action="append", choices=sorted(MUTANTS), default=[])
    ap.add_argument("--scenario", action="append", default=[])
    ap.add_argument("--full", action="store_true")
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--b2b", choices=sorted(MUTANTS))
    ap.add_argument("--dut", choices=["reference", "fmu", "can", "ros2", "native", "loopback", "pil", "hil"], default="reference")
    ap.add_argument("--port-b", help="with --dut pil/hil: COM port of board B (safety controller)")
    ap.add_argument("--port-a", help="with --dut hil: COM port of board A (bus node)")
    ap.add_argument("--kick", choices=["usb", "gpio"], default="usb", help="with --dut hil: watchdog kicks over USB or the GPIO wire")
    ap.add_argument("--bus-monitor", action="store_true", help="with --dut pil: board A is on the bus, so B may treat CAN tx errors as a fault")
    ap.add_argument("--ros2-lockstep", action="store_true", help="with --dut ros2: wait for each 10 ms cycle instead of real time")
    ap.add_argument("--can-lockstep", action="store_true", help="with --dut can: wait for each 10 ms cycle instead of real time")
    ap.add_argument("--ros2-no-launch", action="store_true", help="with --dut ros2: don't start the reference node (a supplied node is running)")
    ap.add_argument("--file", default="fmu/SafeStopVecu.fmu", help="with --dut fmu: the FMU")
    ap.add_argument("--mapping", help="with --dut fmu: bench-name -> FMU-name JSON (default: identity)")
    ap.add_argument("--plant", choices=["kinematic", "dynamic"], help="vehicle model (default: the config's, kinematic)")
    ap.add_argument("--payload", type=float, help="with --plant dynamic: payload in kg on top of the curb mass")
    ap.add_argument("--load-sweep", action="store_true", help="with --plant dynamic: payload x grade sweep -> report")
    ap.add_argument("--mdf", action="store_true", help="also write each scenario's trace as MDF4 (.mf4, needs asammdf)")
    ap.add_argument("--out", default="reports")
    ap.add_argument("--repeat", type=int, default=1, help="real-time DUTs: run each timing scenario N times (min/typ/max)")
    ap.add_argument("--b2b-dut", "--b2b-can", dest="b2b_dut", action="store_true",
                    help="with --dut can/fmu: compare the in-process reference with that DUT (exact for fmu)")
    a = ap.parse_args()

    cfg = config.load(a.config)
    if a.plant:
        cfg["plant"]["model"] = a.plant
    if a.payload is not None:
        if cfg["plant"].get("model") != "dynamic":
            ap.error("--payload needs --plant dynamic")
        cfg["plant"]["dynamic"]["payload_kg"] = a.payload
    defects = frozenset(a.defect)
    if a.dut == "reference":
        factory = lambda sc: ReferenceDUT(cfg, defects, warm_start=sc.get("start_kmh", 30) > 0)
    elif a.dut == "fmu":
        from ssb.dut import FmuDUT
        mapping = json.loads(Path(a.mapping).read_text(encoding="utf-8")) if a.mapping else None
        shared = FmuDUT(a.file, mapping, config_name=Path(a.config).stem)   # one instance, fmi2Reset per scenario
        shared.defects = defects
        factory = lambda sc: shared
    elif a.dut == "ros2":
        from ssb.ros2_dut import Ros2DUT
        shared = Ros2DUT(launch=None if a.ros2_no_launch else "reference", config_path=a.config, lockstep=a.ros2_lockstep)
        shared.defects = defects
        factory = lambda sc: shared
    elif a.dut == "native":
        from ssb.native import NativeDUT
        shared = NativeDUT(cfg, defects)
        factory = lambda sc: shared
    elif a.dut in ("loopback", "pil", "hil"):
        from ssb import hil
        shared = hil.make(a.dut, cfg, a.port_b, a.port_a, kick=a.kick, bus_monitor=True if a.bus_monitor else None)
        shared.defects = defects
        factory = lambda sc: shared
    else:
        from ssb.dut import CanDUT
        shared = CanDUT(config_path=a.config, lockstep=a.can_lockstep)          # one vECU process for the whole campaign, reset per scenario
        shared.defects = defects
        factory = lambda sc: shared

    try:
        results = campaigns.matrix(cfg, factory, only=a.scenario or None)
        if a.repeat > 1:
            extras_timing = campaigns.timing_repeat(cfg, factory, a.scenario or campaigns.TIMING_KEYS, a.repeat)
        if a.b2b_dut:
            b2b_rows = campaigns.back_to_back(cfg, factory, only=a.scenario or None,
                                              exact=a.dut in ("fmu", "native", "loopback") or (a.dut == "can" and a.can_lockstep),
                                              quantum_ms=10 if a.dut in ("loopback", "pil", "hil") or a.can_lockstep else 1)
    finally:
        if a.dut not in ("reference",):
            shared.close()
    if a.dut in ("pil", "hil") and getattr(shared, "diag", None):
        print(f"board B diagnostics: {shared.diag}")
        if getattr(shared, "diag_a", None):
            print(f"board A diagnostics: {shared.diag_a}")
        replays = sum(r.get("bus_replays") or 0 for r in results)
        if replays:
            print(f"stale SAF_Status replays on the bus, filtered by the observer: {replays}")
        e2e_rej = sum(r.get("status_e2e_rejects") or 0 for r in results)
        print(f"SAF_Status E2E (CRC + alive counter) rejects: {e2e_rej}")
    if a.dut == "can":
        print(f"SAF_Status E2E (CRC + alive counter) rejects: {sum(r.get('status_e2e_rejects') or 0 for r in results)}")
    extras = {"requirements": campaigns.requirements(), "config_name": cfg["name"], "chart_keys": CHART_KEYS,
              "gaps": campaigns.gaps(cfg), "ftti": campaigns.ftti_sheet()}
    if a.full:
        print("sweep …", flush=True); extras["sweep"] = campaigns.sweep(cfg)
        print("false-stop rate …", flush=True); extras["false_stop"] = campaigns.false_stop_rate(cfg, seeds=a.seeds)
        print("fuzz …", flush=True); extras["fuzz"] = campaigns.fuzz(cfg)
        print("mutation …", flush=True); extras["mutation"] = campaigns.mutation(cfg)
    if a.load_sweep:
        print("load sweep …", flush=True); extras["load_sweep"] = campaigns.load_sweep(cfg)
    if a.repeat > 1:
        extras["timing"] = extras_timing
    if a.b2b_dut:
        other = {"can": "the vECU process over CAN" + (" (lockstep, exact on the 10 ms grid)" if a.can_lockstep else ""), "ros2": "the safety node over ROS 2 topics", "fmu": f"the FMU {Path(a.file).name} (exact)",
                 "native": "the firmware C++ core on the PC (exact)", "loopback": "board B's node logic on the PC (exact, 10 ms status grid)",
                 "pil": "board B, the ESP32-S3 (real time)", "hil": "board B through the real CAN bus (real time)"}[a.dut]
        extras["b2b"] = {"other": other, "rows": b2b_rows}
    if a.b2b:
        rows = campaigns.back_to_back(cfg, lambda sc: ReferenceDUT(cfg, frozenset([a.b2b]), warm_start=sc.get("start_kmh", 30) > 0))
        extras["b2b"] = {"other": f"reference[{a.b2b}]", "rows": rows}

    label = "report" + ("_" + "_".join(sorted(defects)) if defects else "") + ("_offroad" if "offroad" in a.config else "") + ("_" + a.dut if a.dut != "reference" else "") + ("_dynamic" if cfg["plant"].get("model") == "dynamic" else "") + (f"_{a.payload:g}kg" if a.payload else "")
    page = report.write_all(Path(__file__).resolve().parent / a.out, results, extras, label)
    if a.mdf:
        from ssb import mdf_export
        if not mdf_export.available():
            print("--mdf: asammdf not installed (pip install asammdf); CSV traces only")
        else:
            files = mdf_export.write_all(results, page.parent / "traces", report.bench_identity())
            print(f"MDF4: {len(files)} files in {page.parent / 'traces'}")
    for r in results:
        v = r["verdict"]
        print(f"  {v['status']:5} {r['key']:34} {v['peak_state']:17} cause={r['cause'] or '-':20} detect={v['t_detect_ms'] if v['t_detect_ms'] is not None else '-'}")
    ok = sum(r["verdict"]["passed"] for r in results)
    known = sum(r["verdict"]["status"] == "KNOWN" for r in results)
    print(f"\n{ok}/{len(results)} passed, {known} known finding(s) | {report.bench_identity()} | report: {page}")
    reruns = [(r["key"], f) for r in results for f in r.get("bench_faults") or []]
    if reruns:
        stalls = sum("host stall" in f for _, f in reruns)
        print(f"bench faults rerun: {len(reruns)} ({stalls} host stalls) in {len({k for k, _ in reruns})} scenario(s); "
              f"verdicts come from the clean attempt")
    if "mutation" in extras:
        print(f"mutation score {extras['mutation']['score']}%")
    for row in extras.get("load_sweep", []):
        print(f"  load {row['key']:34} {row['peak_state']:17} stop {row['stop_dist_m']} m in {row['t_stop_ms']} ms  {'PASS' if row['passed'] else 'FAIL ' + '; '.join(row['failed'])}")
    if "false_stop" in extras:
        f = extras["false_stop"]; print(f"false stops: {f['false_stops']} in {f['km']} km")
    return 0 if ok + known == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
