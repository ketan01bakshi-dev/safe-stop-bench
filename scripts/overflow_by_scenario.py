"""v2.29 diagnostics: which scenarios make board A's CAN receiver overflow?

    .venv\\Scripts\\python.exe scripts/overflow_by_scenario.py [scenario keys ...]     (default: a mixed set; --all for the whole matrix; --security for the attack scenarios)

Resets board A first (its counter is cumulative since it last booted), then runs each scenario on the real two-board bench and prints the
overflows A counted DURING that scenario (the difference of its diagnostics before and after), with the frames it received.
The counter is the MCP2515's own RX0OVR / RX1OVR error flag: a frame arrived while both receive buffers were still full.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ssb import campaigns, config, hil  # noqa: E402

DEFAULT = ["nominal_with_noise", "command_link_lost", "safety_hw_reset", "bus_flood", "crc_corrupt", "intermittent_link", "planner_power_dip"]


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    cfg = config.load("config/default.json")
    sec = "--security" in sys.argv   # the attack scenarios (scenarios/security.json): floods, fuzzing, replays
    keys = None if "--all" in sys.argv else (args or DEFAULT)
    dut = hil.make("hil", cfg, "COM13", "COM14", kick="gpio", bus_monitor=True)
    try:
        if "--no-reset" not in sys.argv:
            dut.a.reset_board()   # RTS pulse: A boots with a zero counter
            time.sleep(2.5)
        sf = "scenarios/security.json" if sec else None
        scenarios = [s["key"] for s in campaigns.all_scenarios(cfg, sf)] if (keys is None or sec) else keys
        print(f"{'scenario':28} {'A overflows':>12} {'frames':>8}  verdict")
        total = 0
        for key in scenarios:
            before = dict(dut.diag_a)
            try:
                r = campaigns.matrix(cfg, lambda sc: dut, only=[key], scenario_file=sf)[0]
            except Exception as e:   # noqa: BLE001
                print(f"{key:28} error: {e}")
                continue
            ovf = dut.diag_a.get("rx_overflow", 0) - before.get("rx_overflow", 0)
            frames = dut.diag_a.get("rx", 0) - before.get("rx", 0)
            total += ovf
            print(f"{key:28} {ovf:12d} {frames:8d}  {'PASS' if r['verdict']['passed'] else 'FAIL'}", flush=True)
        print(f"{'TOTAL':28} {total:12d}      A diagnostics: {dut.diag_a}")
    finally:
        dut.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
