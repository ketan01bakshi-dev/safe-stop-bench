"""Where does the bench's real-time lag come from with Ros2DUT? Times every DUT step of one scenario (WSL, ROS 2 sourced)."""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from ssb import campaigns, config, rt, runner  # noqa: E402
from ssb.ros2_dut import Ros2DUT  # noqa: E402

print(rt.boost())
cfg = config.load()
dut = Ros2DUT()
durs = []
orig = dut.step
def timed(*a):
    t0 = time.perf_counter(); out = orig(*a); durs.append((time.perf_counter() - t0) * 1000); return out
dut.step = timed
sc = next(s for s in campaigns.all_scenarios(cfg) if s["key"] == sys.argv[1] if len(sys.argv) > 1) if len(sys.argv) > 1 else \
     next(s for s in campaigns.all_scenarios(cfg) if s["key"] == "planner_hang")
r = runner.run(sc, cfg, dut)
dut.close()
durs.sort()
n = len(durs)
print(f"{sc['key']}: {n} steps, mean {sum(durs)/n:.3f} ms, p50 {durs[n//2]:.3f}, p99 {durs[int(n*.99)]:.3f}, p99.9 {durs[int(n*.999)]:.3f}, max {durs[-1]:.1f} ms; "
      f"steps > 1 ms: {sum(d > 1 for d in durs)}, > 5 ms: {sum(d > 5 for d in durs)}; bench lag {r['bench_lag_ms']} ms")
