"""Run one scenario N times on the boards and report the bench's worst lag and when it happened (v2.9.5)."""
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ssb import campaigns, config, hil, runner  # noqa: E402

key = sys.argv[1] if len(sys.argv) > 1 else "replay_sample_log"
n = int(sys.argv[2]) if len(sys.argv) > 2 else 5
cfg = config.load("config/default.json")
sc = next(s for s in campaigns.all_scenarios(cfg) if s["key"] == key)
dut = hil.make("hil", cfg, "COM13", "COM14", kick="gpio", bus_monitor=True)
lags = []
try:
    for _ in range(n):
        r = runner._run_once_gc_paused(sc, cfg, dut, 7, False)   # one attempt, no rerun: we want the raw lag
        lags.append(r["bench_lag_ms"])
        print(f"  lag {r['bench_lag_ms']:6.1f} ms at t = {r['bench_lag_at_ms']} ms", flush=True)
finally:
    dut.close()
print(f"{key}: worst lag median {statistics.median(lags):.1f} ms, max {max(lags):.1f} ms, over {runner.LAG_LIMIT_MS:g} ms in {sum(x > runner.LAG_LIMIT_MS for x in lags)}/{n}")
