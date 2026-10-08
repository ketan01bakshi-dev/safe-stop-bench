"""Probe for the HiL steering-slew finding (v2.9): when do SAF_Status frames reach the PC, and what does the
1 ms invariant loop see? Runs one scenario N times on the boards and logs per-ms frame arrivals."""
from __future__ import annotations

import json
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ssb import campaigns, config, hil

key = sys.argv[1] if len(sys.argv) > 1 else "bv_steer_rate_10kmh_49dps"
n = int(sys.argv[2]) if len(sys.argv) > 2 else 8
cfg = config.load("config/default.json")
dut = hil.make("hil", cfg, "COM13", "COM14", kick="gpio", bus_monitor=True)
log: list = []
orig = dut.step
def step(t, *a, **k):
    before = dut.n_status
    o = orig(t, *a, **k)
    log.append((t, dut.n_status - before, o.out_cmd[1], o.state))
    return o
dut.step = step
runs = []
try:
    for i in range(n):
        log.clear()
        r = campaigns.matrix(cfg, lambda sc: dut, only=[key])[0]
        per20 = Counter()
        for t, k, _s, _st in log:
            per20[t // 20] += k
        hist = Counter(per20.values())
        gaps, last = [], None
        for t, k, _s, _st in log:
            if k:
                if last is not None: gaps.append(t - last)
                last = t
        inv = r["invariant_violations"]
        runs.append({"run": i, "passed": r["verdict"]["passed"], "invariants": inv,
                     "status_per_20ms": dict(sorted(hist.items())), "max_gap_ms": max(gaps), "gap_hist": dict(sorted(Counter(gaps).items())),
                     "trace": [(t, k, s, st) for t, k, s, st in log if k]})
        print(i, runs[-1]["passed"], inv[:2], "frames/20ms:", runs[-1]["status_per_20ms"], "gaps:", runs[-1]["gap_hist"], flush=True)
finally:
    dut.close()
Path("reports/slew_probe.json").write_text(json.dumps(runs))
