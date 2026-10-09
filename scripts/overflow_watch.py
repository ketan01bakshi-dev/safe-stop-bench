"""v2.29 diagnostics: watch board A's receive-overflow counter live while something else happens to the bench.

    .venv\\Scripts\\python.exe scripts/overflow_watch.py idle 60            just watch for 60 s
    .venv\\Scripts\\python.exe scripts/overflow_watch.py b-resets 10        pulse board B's EN pin 10 times, 4 s apart
    .venv\\Scripts\\python.exe scripts/overflow_watch.py b-update 2.12      push that image to B over USB, reboot it, wait for the commit

Board A reports its counters by itself every 200 ms, so this only listens (COM14, without touching DTR/RTS so A is not reset). The actions use
board B's port (COM13). Prints each time the counter changes, with what was happening.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fleet import image  # noqa: E402
from fleet.ota_client import OtaClient  # noqa: E402
from ssb.hil import Parser, SerialLink  # noqa: E402


class Watch:
    def __init__(self) -> None:
        self.link = SerialLink("COM14")
        self.parser = Parser()
        self.ovf: int | None = None
        self.rx: int | None = None
        self.eflg: int | None = None
        self.events: list[tuple[float, str, int]] = []
        self.what = "start"
        self.t0 = time.time()

    def pump(self) -> None:
        for kind, p in self.parser.feed(self.link.read()):
            if kind == "D" and len(p) >= 12:
                import struct
                rx, ovf = struct.unpack_from("<I", p)[0], struct.unpack_from("<I", p, 5)[0]
                if self.ovf is not None and ovf != self.ovf:
                    self.events.append((time.time() - self.t0, self.what, ovf - self.ovf))
                    print(f"  +{time.time() - self.t0:6.1f}s  A overflows {self.ovf} -> {ovf}  during: {self.what}", flush=True)
                self.ovf, self.rx, self.eflg = ovf, rx, p[4]

    def run_for(self, seconds: float, what: str) -> None:
        self.what = what
        end = time.time() + seconds
        while time.time() < end:
            self.pump()
            time.sleep(0.02)


def main() -> int:
    w = Watch()
    w.run_for(1.5, "settle")
    print(f"A counter at start: overflows {w.ovf}, frames {w.rx}", flush=True)
    mode = sys.argv[1] if len(sys.argv) > 1 else "idle"
    if mode == "idle":
        w.run_for(float(sys.argv[2]) if len(sys.argv) > 2 else 60.0, "idle (nothing touching the bench)")
    elif mode == "b-resets":
        b = SerialLink("COM13")
        for i in range(int(sys.argv[2]) if len(sys.argv) > 2 else 10):
            w.what = f"B reset #{i + 1}"
            b.reset_board()
            w.run_for(4.0, f"after B reset #{i + 1}")
        b.close()
    elif mode == "b-update":
        ver = sys.argv[2] if len(sys.argv) > 2 else "2.12"
        b = SerialLink("COM13")
        time.sleep(2.0)
        c = OtaClient(b)
        img = image.load(image.IMAGES / f"SafetyNode_{ver}.bin", ver)
        w.what = "OTA push"
        import threading
        out: dict = {}
        t = threading.Thread(target=lambda: out.update(r=c.push(img)))
        t.start()
        while t.is_alive():
            w.pump()
            time.sleep(0.02)
        w.run_for(1.0, "after push")
        c.forget_hello()
        c.reboot()
        w.run_for(8.0, "B rebooting into the new image")
        w.run_for(5.0, "trial commit")
        print("push:", out["r"].outcome, out["r"].status)
        b.close()
    print(f"A counter at end: overflows {w.ovf}, frames {w.rx}  ({len(w.events)} change(s))")
    w.link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
