"""Exact back-to-back on the 10 ms task grid: reference in-process vs the ROS 2 node in lockstep (WSL, ROS 2 sourced).
Any difference = cross-topic ordering or transport changed the controller's behaviour by at least one cycle."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from ssb import campaigns, config  # noqa: E402
from ssb.ros2_dut import Ros2DUT  # noqa: E402

cfg = config.load()
d = Ros2DUT(lockstep=True)
try:
    rows = campaigns.back_to_back(cfg, lambda sc: d, exact=True, quantum_ms=10)
finally:
    d.close()
bad = [r for r in rows if not r["match"]]
print(f"exact on the 10 ms grid: {len(rows) - len(bad)}/{len(rows)} identical")
for r in bad:
    print(" ", r["key"], r["diffs"][:2])
