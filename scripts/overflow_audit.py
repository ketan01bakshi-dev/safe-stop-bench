"""v2.32 diagnostics: audit board A's receive path frame by frame, passively.

    .venv\\Scripts\\python.exe scripts/overflow_audit.py SECONDS [LOGFILE]

Board B sends SAF_ActuatorCmd 0x200 (E2E Profile 2: CRC-8, 4-bit counter) and SAF_Status 0x201 (CRC-8, 8-bit counter) once per 10 ms
cycle, also when idle, so nothing has to run: this only listens to board A (COM14, without touching DTR/RTS) and checks every frame it
forwards: the CRC, and the counter (it must advance by one per cycle). A counter that skips is a frame A lost or never forwarded; a
repeat is a duplicate; a bad CRC is a corrupted read. Each time A's overflow counter rises, the event is printed with A's own
classification (BusNode 2.13: confirmed by a second read? first-read value? see BusNode.ino v2.32) and the frame events around it.
The question it answers: is every counted overflow a lost frame, and which frame is lost?
"""
from __future__ import annotations

import struct
import sys
import time
from collections import Counter, deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ssb.e2e import crc8_h2f  # noqa: E402
from ssb.hil import Parser, SerialLink  # noqa: E402

WINDOW = 0.6   # seconds: a counter gap and an overflow increment this close are the same event (A reports every 200 ms, the PC's clock jitters by ms)


def crc_ok(cid: int, d: bytes) -> bool:
    if cid == 0x200 and len(d) == 7:
        c = d[1] & 0x0F
        return crc8_h2f(bytes([c]) + d[2:] + bytes([0x10 + c])) == d[0]
    if cid == 0x201 and len(d) == 8:
        return crc8_h2f(d[1:8] + (0x201).to_bytes(2, "little")) == d[0]
    return True   # other ids (UDS answers): not checked


def main() -> int:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0
    log = open(sys.argv[2], "w", encoding="utf-8") if len(sys.argv) > 2 else None

    def say(msg: str) -> None:
        line = f"{time.strftime('%H:%M:%S')} {msg}"
        print(line, flush=True)
        if log:
            log.write(line + "\n")
            log.flush()

    link = SerialLink("COM14")
    parser = Parser()
    t0 = time.time()
    last = {0x200: None, 0x201: None}
    mod = {0x200: 16, 0x201: 256}
    n_frames: Counter = Counter()
    crc_bad: Counter = Counter()
    dups: Counter = Counter()
    lost: Counter = Counter()
    jumps: Counter = Counter()
    gaps: list[tuple[float, int, int, int]] = []      # (time, id, counter before, frames lost)
    crcs: list[tuple[float, int]] = []
    overflows: list[tuple[float, int, dict]] = []     # (time, increase, A's classification at that report)
    prev = None
    d0 = None
    recent: deque = deque(maxlen=14)   # the last frames, dumped when something looks wrong (with the frames that follow)
    post = 0

    def anomaly(msg: str) -> None:
        nonlocal post
        say(msg)
        for t_, c_, k_, ok_ in recent:
            say(f"        before: +{t_:9.3f}s {c_:#x} counter {k_}{'' if ok_ else '  CRC FAIL'}")
        post = 10
    say(f"audit for {seconds:.0f} s (frames: 0x200 and 0x201 once per 10 ms)")
    try:
        while time.time() - t0 < seconds:
            for kind, p in parser.feed(link.read()):
                now = time.time() - t0
                if kind == "M" and len(p) >= 3:
                    cid, n = p[0] | p[1] << 8, p[2]
                    data = bytes(p[3:3 + n])
                    n_frames[cid] += 1
                    if post > 0 and cid in last:
                        post -= 1
                        say(f"        after:  +{now:9.3f}s {cid:#x} counter {(data[1] & 0x0F) if cid == 0x200 else data[1]}")
                    if cid in last:
                        recent.append((now, cid, (data[1] & 0x0F) if cid == 0x200 else data[1], crc_ok(cid, data)))
                        if not crc_ok(cid, data):
                            crc_bad[cid] += 1
                            crcs.append((now, cid))
                            anomaly(f"  +{now:8.1f}s  CRC FAIL {cid:#x} {data.hex()}")
                            continue
                        c = (data[1] & 0x0F) if cid == 0x200 else data[1]
                        if last[cid] is not None:
                            delta = (c - last[cid]) % mod[cid]
                            if delta == 0:
                                dups[cid] += 1
                                anomaly(f"  +{now:8.1f}s  DUPLICATE {cid:#x} counter {c}")
                            elif 2 <= delta <= 8:
                                lost[cid] += delta - 1
                                gaps.append((now, cid, last[cid], delta - 1))
                                anomaly(f"  +{now:8.1f}s  LOST {delta - 1} x {cid:#x} (counter {last[cid]} -> {c})")
                            elif delta > 8:
                                jumps[cid] += 1
                                anomaly(f"  +{now:8.1f}s  JUMP {cid:#x} counter {last[cid]} -> {c} (a reset of B? out of order?)")
                        last[cid] = c
                elif kind == "D" and len(p) >= 31:
                    rx, ov = struct.unpack_from("<I", p)[0], struct.unpack_from("<I", p, 5)[0]
                    info = {"eflg": p[4], "glitch": p[12] | p[13] << 8, "mismatch": p[16] | p[17] << 8, "qdrop": struct.unpack_from("<I", p, 27)[0]}
                    if len(p) >= 151:
                        v2 = struct.unpack_from("<8I", p, 111)
                        info.update(confirmed=v2[0], cls=list(v2[1:5]), intf_ff=v2[5], intf_wake=v2[6], bad_dlc=v2[7], raw=bytes(p[143:151]).hex())
                    if d0 is None:
                        d0 = (rx, ov, info)
                    if prev is not None and ov > prev[1]:
                        overflows.append((now, ov - prev[1], info))
                        anomaly(f"  +{now:8.1f}s  OVERFLOW +{ov - prev[1]} (A total {ov}) {info}")
                    elif prev is not None:
                        changed = {k: (prev[2].get(k), v) for k, v in info.items() if k in ("glitch", "mismatch", "intf_ff", "intf_wake", "bad_dlc", "qdrop")
                                   and v != prev[2].get(k)}
                        if changed:
                            anomaly(f"  +{now:8.1f}s  A COUNTER CHANGED {changed}")
                    prev = (rx, ov, info)
            time.sleep(0.003)
    except KeyboardInterrupt:
        say("interrupted")
    finally:
        link.close()
    dur = time.time() - t0
    say("---- summary")
    say(f"duration {dur:.0f} s; frames seen on the PC: " + ", ".join(f"{k:#x}: {v}" for k, v in sorted(n_frames.items())))
    if prev and d0:
        say(f"A's own count over the same time: frames {prev[0] - d0[0]}, overflows {prev[1] - d0[1]}; the PC saw {sum(n_frames.values())} frames "
            f"(a difference is frames A dropped before USB or the PC missed: A queue drops now {prev[2].get('qdrop')})")
    say(f"counter gaps (lost frames): {dict(lost)}   duplicates: {dict(dups)}   CRC failures: {dict(crc_bad)}   other jumps: {dict(jumps)}")
    n_ovf = sum(o for _, o, _ in overflows)
    matched = 0
    for t, inc, info in overflows:
        near = [g for g in gaps if -WINDOW <= g[0] - t <= WINDOW]
        near_crc = [c for c in crcs if -WINDOW <= c[0] - t <= WINDOW]
        matched += 1 if (near or near_crc) else 0
        what = ", ".join(f"{g[3]} x {g[1]:#x}" for g in near) or ("CRC fail" if near_crc else "NO frame lost")
        say(f"overflow at +{t:.0f}s (+{inc}): {what}; A says {info.get('cls')} confirmed={info.get('confirmed')} raw={info.get('raw')}")
    say(f"overflow events {len(overflows)} (total increase {n_ovf}); with a lost frame or a bad CRC beside them: {matched}")
    unmatched_gaps = [g for g in gaps if not any(-WINDOW <= g[0] - t <= WINDOW for t, _, _ in overflows)]
    say(f"lost frames with NO overflow beside them: {len(unmatched_gaps)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
