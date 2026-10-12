"""v2.32/v2.33 diagnostics: when does board B's 10 ms cycle run late, and what was it doing? Passive log of B's own counters.

    .venv\\Scripts\\python.exe scripts/b_stall_watch.py SECONDS [LOGFILE] [--reset] [--poke OP[,OP..] [--every S] [--count N]]

Board B reports, every 200 ms, how late its worst cycle has run since it booted (max_late_ms) and how many CAN transmits failed (all three
MCP2515 transmit buffers busy). This prints a line each time either changes, with the time. A board built with -DB_DIAG=1 (hil/firmware/SafetyNode/
ssc_diag.h, scripts/hil_flash.py --cflags=-DB_DIAG=1) also sends 'S' (which section of loop() stalled, and for how long) and 'E' (Wi-Fi / MQTT /
radio events with B's clock): each new stall is printed with the events around it.

    --reset            pulse B's EN pin first (zeroes the counters)
    --poke 1,2,3,4,5   test hooks of a B_DIAG build, one every --every seconds (default 20) for --count rounds (default 1):
                       1 MQTT reconnect, 2 Wi-Fi reconnect, 3 radio off then on, 4 NVS write, 5 100 NVS opens (what health_ok() does every pass)
Found in v2.32: B had max_late_ms 187 and 55 failed transmits, and board A saw ~190 ms of silence followed by a flood of out-of-order frames:
node.poll() catches up with every missed 10 ms cycle at once, ~38 CAN sends into three transmit buffers.
"""
from __future__ import annotations

import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ssb.hil import Parser, SerialLink, frame_msg  # noqa: E402

SECTIONS = ["serial", "net.poll", "net.rx", "persist", "ota.confirm+health_ok", "wd", "can flags", "node.poll"]
EVENTS = {1: "radio ON", 2: "radio OFF", 3: "MQTT client start", 4: "MQTT UP", 5: "MQTT DOWN", 6: "MQTT ERROR", 7: "TEST op", 8: "NVS write begin", 9: "NVS write end", 10: "radio task stack free", 11: "esp-mqtt task priority", 12: "esp-mqtt task core (-1 = any)", 13: "loopTask priority",
          110: "wifi STA start", 111: "wifi STA stop", 112: "wifi CONNECTED", 113: "wifi DISCONNECTED", 115: "wifi GOT IP", 117: "wifi LOST IP"}
POKES = {1: "MQTT reconnect", 2: "Wi-Fi reconnect", 3: "radio off then on", 4: "NVS write", 5: "100 NVS opens"}


def longest(sec_max: list[int]) -> str:
    return "; longest section (us): " + ", ".join(f"{n} {v}" for n, v in zip(SECTIONS, sec_max, strict=False)) if sec_max else ""


def main() -> int:
    argv = sys.argv[1:]

    def opt(name: str, default: str | None = None) -> str | None:
        return argv[argv.index(name) + 1] if name in argv else default

    skip = {opt("--poke"), opt("--every"), opt("--count")} - {None}
    args = [a for a in argv if not a.startswith("--") and a not in skip]
    seconds = float(args[0]) if args else 600.0
    log = open(args[1], "w", encoding="utf-8") if len(args) > 1 else None
    pokes = [int(x) for x in (opt("--poke") or "").split(",") if x]
    every, rounds = float(opt("--every", "20") or 20), int(opt("--count", "1") or 1)

    def say(msg: str) -> None:
        line = f"{time.strftime('%H:%M:%S')} {msg}"
        print(line, flush=True)
        if log:
            log.write(line + "\n")
            log.flush()

    link = SerialLink("COM13")
    if "--reset" in argv:
        link.reset_board()
        say("B reset")
    parser = Parser()
    t0 = time.time()
    prev: tuple[int, int] | None = None
    last_beat = t0
    last = (0, 0, 0)
    seen_stalls = 0
    seen_events = 0
    all_events: list[tuple[int, int, int]] = []
    sec_max: list[int] = []
    queue = [(op, t0 + 5 + i * every) for i in range(rounds) for op in pokes]
    queue = [(op, t0 + 5 + n * every) for n, (op, _) in enumerate(queue)]
    try:
        while time.time() - t0 < seconds:
            now = time.time()
            if queue and now >= queue[0][1]:
                op, _ = queue.pop(0)
                say(f"  >>> poke {op}: {POKES.get(op, '?')}")
                link.write(frame_msg("N", bytes([7, op])), 0)
            for kind, p in parser.feed(link.read()):
                if kind == "D" and len(p) >= 23:
                    cycles, _ex, late, txf, _ovf = struct.unpack("<5I", p[:20])
                    last = (cycles, late, txf)
                    if prev is None or (late, txf) != prev:
                        say(f"+{time.time() - t0:8.1f}s  max_late_ms {late}  can_tx_fail {txf}  (cycles {cycles})")
                        prev = (late, txf)
                elif kind == "E" and len(p) >= 8:
                    up, n_ev = struct.unpack_from("<II", p)
                    cnt = (len(p) - 8) // 7
                    ents = [struct.unpack_from("<IBh", p, 8 + 7 * j) for j in range(cnt)]
                    new = min(n_ev - seen_events, cnt) if n_ev > seen_events else 0
                    for t_ms, code, arg in ents[cnt - new:] if new else []:
                        all_events.append((t_ms, code, arg))
                        extra = f" (reason {arg})" if code == 113 else f" (error type {arg})" if code == 6 else f" ({POKES.get(arg, arg)})" if code == 7 else f" ({arg} bytes)" if code == 10 else f" = {arg}" if code in (11, 12, 13) else ""
                        say(f"      event  B+{t_ms / 1000:9.3f}s  {EVENTS.get(code, code)}{extra}")
                    seen_events = n_ev
                elif kind == "T":
                    say(f"      who ran (last stall): {bytes(p).decode('ascii', 'replace')}")
                elif kind == "S" and len(p) >= 12 + 4 * len(SECTIONS):
                    up, gap_us, n_st = struct.unpack_from("<III", p)
                    sec_max = list(struct.unpack_from(f"<{len(SECTIONS)}I", p, 12))
                    off = 12 + 4 * len(SECTIONS)
                    cnt = (len(p) - off) // 9
                    ents = [struct.unpack_from("<IIB", p, off + 9 * j) for j in range(cnt)]
                    new = min(n_st - seen_stalls, cnt) if n_st > seen_stalls else 0
                    for t_ms, dur_us, sec in ents[cnt - new:] if new else []:
                        what = "the whole loop pass" if sec == 255 else f"in {SECTIONS[sec]}" if sec < len(SECTIONS) else f"section {sec}"
                        near = [f"{EVENTS.get(c, c)}@{(t - t_ms) / 1000:+.2f}s" for t, c, _a in all_events if abs(t - t_ms) <= 400]
                        say(f"  ** STALL B+{t_ms / 1000:9.3f}s  {dur_us / 1000:7.1f} ms  {what}   events within 0.4 s: {', '.join(near) or 'none'}")
                    seen_stalls = n_st
            if time.time() - last_beat >= 600:
                last_beat = time.time()
                say(f"  heartbeat +{time.time() - t0:.0f}s: cycles {last[0]}, max_late_ms {last[1]}, can_tx_fail {last[2]}{longest(sec_max)}")
            time.sleep(0.005)
    except KeyboardInterrupt:
        say("interrupted")
    finally:
        link.close()
    say(f"end: cycles {last[0]}, max_late_ms {last[1]}, can_tx_fail {last[2]}{longest(sec_max)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
