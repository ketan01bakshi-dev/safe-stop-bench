"""v2.9.8 diagnostics: does a burst of watchdog kicks on board A (busy-wait pulses) overflow its CAN receive buffer?
Runs one scenario with and without extra 'K' bursts to A and reports A's receive-overflow count for each."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ssb import campaigns, config, hil  # noqa: E402
from ssb.hil import frame_msg  # noqa: E402

key = sys.argv[1] if len(sys.argv) > 1 else "crc_corrupt"
burst = int(sys.argv[2]) if len(sys.argv) > 2 else 30
cfg = config.load("config/default.json")
dut = hil.make("hil", cfg, "COM13", "COM14", kick="gpio", bus_monitor=True)
orig = dut.step
inject = [False]


def step(t, *a, **k):
    if inject[0] and t % 497 == 0 and t > 0:   # 497 ms: the burst drifts through B's 10 ms cycle
        dut.a.write(frame_msg("K", bytes([burst])), t)   # burst x 100 us of pulses on A
    return orig(t, *a, **k)


dut.step = step
try:
    for label, on in (("baseline", False), (f"+{burst}-kick bursts every 497 ms", True), ("baseline again", False)):
        inject[0] = on
        before = dut.diag_a.get("rx_overflow", 0)
        r = campaigns.matrix(cfg, lambda sc: dut, only=[key])[0]
        print(f"{label:34} A receive overflows: {dut.diag_a.get('rx_overflow', 0) - before:3}  "
              f"(frames {dut.diag_a.get('rx', 0)})  verdict {'PASS' if r['verdict']['passed'] else 'FAIL'}")
    print("A diagnostics:", dut.diag_a)
finally:
    dut.close()
