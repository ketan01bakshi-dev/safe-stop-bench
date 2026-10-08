"""v2.9.4 diagnostics: A runs BusNode (diagnostic receive), B runs SpiStress in normal mode (~1500 frames/s).
Counts how often READ STATUS claims RXB1 holds a frame that CANINTF does not confirm, and stale frames forwarded."""
import sys
import time
from collections import Counter
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ssb.hil import Parser  # noqa: E402

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60
b = serial.Serial("COM13", 921600, timeout=0.1)
a = serial.Serial("COM14", 921600, timeout=0)
time.sleep(2.5)
a.read(1000000)
b.dtr = False
b.rts = True
time.sleep(0.1)
b.rts = False
time.sleep(2.5)
b.read(100000)
b.write(b"N")
p, n100, stale, d, stats, sent, bbuf = Parser(), 0, 0, None, Counter(), 0, b""
seen, dup, old, top = set(), 0, 0, 0
t0 = time.time()
while time.time() - t0 < secs:
    for k, q in p.feed(a.read(1000000)):
        if k == "M":
            if (q[0] | q[1] << 8) == 0x100 and bytes(q[5:11]) == bytes([0xAA, 0x55, 0x0F, 0xF0, 0x33, 0xCC]):
                n100 += 1
                sq = q[3] | q[4] << 8
                if sq in seen:
                    dup += 1
                    if sq < top - 2:
                        old += 1
                seen.add(sq)
                top = max(top, sq)
            else:
                stale += 1
        elif k == "D" and len(q) >= 20:
            if d is None or (q[16] | q[17] << 8) != (d[16] | d[17] << 8):
                stats[(q[18], q[19])] += 1
            d = q
    bbuf += b.read(100000)
    *lines, bbuf = bbuf.split(b"\n")
    for ln in lines:
        if ln.startswith(b"S ") and b" sent=" in ln:
            sent += int(ln.split(b" sent=")[1].split()[0])
b.write(b"L")   # stop B transmitting, so the next run starts clean
print(f"{secs:.0f} s: sequence-stamped frames forwarded by A {n100}, unique {len(seen)}; DUPLICATES {dup} "
      f"(of which stale, older than the last 2: {old}); other frames {stale}; never arrived {top - len(seen)}; "
      f"receive overflows {d[5] | d[6] << 8 if d else '?'}")
if d is None or len(d) < 20:
    sys.exit(0)
s1, mm = d[14] | d[15] << 8, d[16] | d[17] << 8
print(f"{secs:.0f} s: frames {n100}, stale/other frames forwarded {stale}; READ STATUS said RXB1 {s1}x, "
      f"CANINTF did not confirm {mm}x; SPI glitches {d[12] | d[13] << 8}")
if len(d) >= 24:
    print(f"A: frames identical to the previous one from the same buffer: RXB0 {d[20] | d[21] << 8}, RXB1 {d[22] | d[23] << 8}")
if len(d) >= 25:
    print(f"A: RXB0CTRL {d[24]:#04x} (BUKT = {(d[24] >> 2) & 1}); flag misreads caught and re-read: {mm} "
          f"(last value {d[18]:#04x})")
print("sampled (status read, CANINTF) at mismatches:", {f"{k[0]:#04x}/{k[1]:#04x}": v for k, v in stats.most_common(8)})
b.close()
a.close()
