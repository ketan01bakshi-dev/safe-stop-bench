"""v2.12: measure a real power cut of board B through the relay that board A drives (docs/HIL_POWER_CUT.md): time
from the cut's end until B's first SAF_Status on the bus (read by board A), what B reports, and whether B's COM port
really vanished (if it stays, USB is still powering the board and the cut is not real).

    python scripts/power_cut_probe.py --port-b COM13 --port-a COM14 [--cuts 5] [--ms 300]
    python scripts/power_cut_probe.py --port-b EMU --port-a EMU          # dry run on the emulated boards

Put the median "first status after power returns" into config/default.json dut_hw.power_on_boot_ms.
"""
import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ssb import config, hil  # noqa: E402
from ssb.canio import CAUSES  # noqa: E402
from ssb.native import STATES  # noqa: E402


def port_present(port: str) -> bool | None:
    if "://" in port:
        return None   # the emulator: no USB device to look for
    from serial.tools import list_ports
    return any(p.device.upper() == port.upper() for p in list_ports.comports())


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port-b", required=True)
    ap.add_argument("--port-a", required=True)
    ap.add_argument("--cuts", type=int, default=5)
    ap.add_argument("--ms", type=int, default=300, help="how long the relay keeps B's supply open")
    a = ap.parse_args()
    cfg = config.load("config/default.json")
    cfg["dut_hw"]["relay_fitted"] = True
    dut = hil.make("hil", cfg, a.port_b, a.port_a, kick="gpio" if a.port_b.upper() != "EMU" else "usb", bus_monitor=True)
    print("B:", dut.fw_b, "| A:", dut.fw_a)
    boots = []
    try:
        for i in range(a.cuts):
            dut.prepare(warm_start=True)            # B active, configuration stored in flash
            t_end = time.time() + 1.0
            while time.time() < t_end:
                dut._read()
            dut.power_cut(0, a.ms)
            t0 = time.perf_counter()
            gone = None
            first = None
            while time.perf_counter() - t0 < (a.ms + 5000) / 1000 and first is None:
                if gone is None and (time.perf_counter() - t0) * 1000 > a.ms / 2:
                    present = port_present(dut.b.port)
                    gone = None if present is None else not present
                for k, p in dut._read()[1]:
                    if k == "M" and (p[0] | p[1] << 8) == 0x201 and p[2] == 8 and (time.perf_counter() - t0) * 1000 > 20:
                        d = bytes(p[3:11])
                        first = ((time.perf_counter() - t0) * 1000 - a.ms, STATES[d[2] & 7], CAUSES[d[2] >> 4], d[1])
            if first:
                boots.append(first[0])
                print(f"cut {i + 1}: first status {first[0]:.0f} ms after power returned: {first[1]} / {first[2]} "
                      f"(counter {first[3]}); B's port vanished during the cut: {gone}")
            else:
                print(f"cut {i + 1}: no status from B within 5 s of power returning; port vanished: {gone}")
    finally:
        dut.close()
    if boots:
        print(f"power-on boot: median {statistics.median(boots):.0f} ms, min {min(boots):.0f}, max {max(boots):.0f} "
              f"(n = {len(boots)}) -> dut_hw.power_on_boot_ms")


if __name__ == "__main__":
    main()
