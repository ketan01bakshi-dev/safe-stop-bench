"""Debug helper: the full emulated HiL matrix with per-scenario progress and a traceback dump every 5 min if stuck."""
import faulthandler
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from ssb import campaigns, config, hil, runner  # noqa: E402

faulthandler.dump_traceback_later(300, repeat=True)
cfg = config.load("config/default.json")
orig = runner.run


def run(sc, *a, **k):
    t0 = time.time()
    print(f"start {sc['key']}", flush=True)
    r = orig(sc, *a, **k)
    print(f"  done {sc['key']} {time.time() - t0:.1f}s lag={r.get('bench_lag_ms')}", flush=True)
    return r


runner.run = run
dut = hil.make("hil", cfg, port_b="EMU", port_a="EMU")
try:
    res = campaigns.matrix(cfg, lambda sc: dut)
finally:
    dut.close()
bad = [r["key"] for r in res if r["verdict"]["status"] == "FAIL"]
print(f"{sum(r['verdict']['passed'] for r in res)}/{len(res)} passed; fails: {bad}", flush=True)
