"""Smoke test: run a few scenarios through the separate-process vECU over CAN and compare with in-process."""
import sys
import time

sys.path.insert(0, ".")
from ssb import campaigns, config, oracle, runner
from ssb.dut import CanDUT

cfg = config.load(); reqs = campaigns.requirements()["requirements"]
keys = sys.argv[1:] or ["command_link_lost", "planner_hang", "single_bad_frame"]
dut = CanDUT()
try:
    for sc in campaigns.all_scenarios(cfg):
        if sc["key"] not in keys: continue
        t0 = time.time()
        rc = runner.run(sc, cfg, dut, keep_trace=True); vc = oracle.verdict(rc, sc, reqs, cfg)
        rr = runner.run(sc, cfg, keep_trace=True); vr = oracle.verdict(rr, sc, reqs, cfg)
        print(f"{sc['key']:28} CAN: {vc['status']} {vc['peak_state']} {rc['cause']} det={vc['t_detect_ms']} | in-process: {vr['status']} {rr['cause']} det={vr['t_detect_ms']} | {time.time()-t0:.1f}s dups={dut.bus.duplicates}"
              + ("" if vc['passed'] else f" failed={[c for c,ok in vc['checks'] if not ok]} inv={rc['invariant_violations'][:2]}"), flush=True)
finally:
    dut.close()
