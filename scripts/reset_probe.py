"""v2.9.7: measure a real reset of board B (EN via RTS) while it runs: time until its first SAF_Status on the bus
(read by board A) and what it reports. Run with both boards connected."""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ssb import config, hil  # noqa: E402
from ssb.canio import CAUSES  # noqa: E402
from ssb.native import STATES  # noqa: E402

n = int(sys.argv[1]) if len(sys.argv) > 1 else 5
cfg = config.load("config/default.json")
dut = hil.make("hil", cfg, "COM13", "COM14", kick="gpio", bus_monitor=True)
print("B:", dut.fw_b)
try:
    for i in range(n):
        dut.prepare(warm_start=True)            # B active, configuration stored in flash
        t_end = time.time() + 1.0
        while time.time() < t_end:
            dut._read()
        ser = dut.b.ser
        ser.dtr = False
        ser.rts = True
        t0 = time.perf_counter()
        time.sleep(0.002)
        ser.rts = False
        first, hello = None, None
        while time.perf_counter() - t0 < 3.0 and first is None:
            mb, ma = dut._read()
            for k, p in mb:
                if k == "H" and hello is None:
                    hello = ((time.perf_counter() - t0) * 1000, p.decode(errors="replace"))
            for k, p in ma:
                if k == "M" and (p[0] | p[1] << 8) == 0x201 and p[2] == 8:
                    d = bytes(p[3:11])
                    packed = d[2]
                    first = ((time.perf_counter() - t0) * 1000, STATES[packed & 7], CAUSES[packed >> 4], d[1])
        print(f"reset {i + 1}: first status after {first[0]:.0f} ms: {first[1]} / {first[2]} (counter {first[3]}); "
              f"hello after {hello[0]:.0f} ms: {hello[1]!r}" if first and hello else f"reset {i + 1}: first={first} hello={hello}")
finally:
    dut.close()
