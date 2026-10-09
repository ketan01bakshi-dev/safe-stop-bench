"""Run the two staged rollouts of the release and write their results (P4).

    .venv\\Scripts\\python.exe -m fleet.rollouts --real COM13     good build over a mixed fleet incl. the real board, bad build over 200 emulated
    .venv\\Scripts\\python.exe -m fleet.rollouts                  the same without the real board

good  2.16 (healthy): 40 emulated SafetyNodes (rev-b) + 2 of another hardware revision (rev-a, not targeted) + the real board at position 4.
      Expected: everyone targeted commits, the rev-a boards are skipped (not failures), nothing halts.
bad   an image that fails its health check, 200 emulated boards, a canary of 2 % (4 boards).
      Expected: all four canaries roll back on their own, the rollout halts after wave 1, 196 boards are never touched.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Any

from ssb.config import ROOT

from . import image
from .emulated import EmuBoard, synthetic_image
from .rollout import Device, rollout
from .telemetry import Telemetry

OUT = ROOT / "reports" / "fleet"


def emulated_device(i: int, version: str, hardware: str = "rev-b") -> Device:
    b = EmuBoard(f"vecu-{i:03d}", version, hardware=hardware)
    return Device(b.name, b, "emulated", hardware, b.advance)


def good_fleet(real_port: str | None, version: str) -> tuple[list[Device], Any]:
    devs = [emulated_device(i, version) for i in range(1, 41)]
    devs += [emulated_device(100 + i, version, "rev-a") for i in range(2)]
    link = None
    if real_port:
        from ssb.hil import SerialLink
        link = SerialLink(real_port)
        time.sleep(2.0)
        devs.insert(3, Device("real-board-B", link, "real", "rev-b", None, image.load(image.IMAGES / "SafetyNode_2.16.bin", "2.16")))
    return devs, link


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--real", metavar="COM", help="include the real board B on this port (it must run 2.15 and be idle)")
    ap.add_argument("--from-version", default="2.15")
    a = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    OUT.mkdir(parents=True, exist_ok=True)
    tel = Telemetry()

    devs, link = good_fleet(a.real, a.from_version)
    good = rollout(devs, synthetic_image("2.16"), {"rev-b"}, tel)
    if link is not None:
        link.close()
    (OUT / "rollout_good.json").write_text(json.dumps(good.summary() | {"devices": len(devs), "with_real_board": bool(a.real),
                                                                       "results": [vars(r) for r in good.results]}, indent=1), encoding="utf-8")

    bad_devs = [emulated_device(i, a.from_version) for i in range(1, 201)]
    bad = rollout(bad_devs, synthetic_image("2.16", healthy=False), {"rev-b"}, tel, canary=4)
    (OUT / "rollout_bad.json").write_text(json.dumps(bad.summary() | {"devices": len(bad_devs)}, indent=1), encoding="utf-8")
    (OUT / "fleet_events.jsonl").write_text("\n".join(json.dumps(e) for e in tel.events) + "\n", encoding="utf-8")

    for name, r, n in (("good build", good, len(devs)), ("bad build", bad, len(bad_devs))):
        s = r.summary()
        print(f"{name}, {n} devices: committed {s['committed']}, rolled back {s['rolled_back']}, rejected {s['rejected']}, skipped {s['skipped']}, "
              f"untouched {s['untouched']}, halted {s['halted']}" + (f" ({s['halt_reason']})" if s["halted"] else ""))
        print("   waves:", s["waves"])
    real_res = next((r for r in good.results if r.kind == "real"), None)
    if real_res:
        print(f"real board: {real_res.outcome} ({real_res.detail}) in {real_res.seconds:.0f} s")
    ok = not good.halted and good.count("committed") >= 40 and bad.halted and bad.count("rolled_back") == 4 and len(bad.untouched) == 196
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
