"""Capture every frame board A forwards during one scenario and print the status frames around state blips."""
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ssb import campaigns, config, hil  # noqa: E402

key = sys.argv[1] if len(sys.argv) > 1 else "nominal_with_noise"
cfg = config.load("config/default.json")
dut = hil.make("hil", cfg, "COM13", "COM14", kick="gpio", bus_monitor=True)
log, clock, mirror = [], [0], []
orig_read, orig_step = dut._read, dut.step
def rd():
    mb, ma = orig_read()
    for k, p in mb:
        if k == "M" and (p[0] | p[1] << 8) == 0x201:
            mirror.append((clock[0], bytes(p)))
    for k, p in ma:
        if k == "M":
            log.append((clock[0], bytes(p)))
    return mb, ma
def st(t, *a, **k):
    clock[0] = t
    return orig_step(t, *a, **k)
dut._read, dut.step = rd, st
try:
    r = campaigns.matrix(cfg, lambda sc: dut, only=[key])[0]
finally:
    dut.close()
print("states", r["states"][:8], "diag_a", dut.diag_a)
st_frames = [(t, p) for t, p in log if (p[0] | p[1] << 8) == 0x201]
prev = None
for i, (_t, p) in enumerate(st_frames):
    if prev is not None and p[3] != prev[3]:
        for tt, pp in st_frames[max(0, i - 3): i + 3]:
            print(tt, pp[3:].hex(" "))
        print("--")
    prev = p
ids = {}
for _t, p in log:
    ids[(p[0] | p[1] << 8, p[2])] = ids.get((p[0] | p[1] << 8, p[2]), 0) + 1
print("ids", {f"{k[0]:#x}/{k[1]}": v for k, v in ids.items()})

bus = [(t, p) for t, p in log if (p[0] | p[1] << 8) == 0x201]
print("mirror 0x201", len(mirror), "bus 0x201", len(bus))
seen = {}
for t, p in mirror:
    seen.setdefault(p, []).append(t)
lags, unmatched = [], []
for t, p in bus:
    ts = [m for m in seen.get(p, []) if abs(m - t) <= 50]
    if ts:
        lags.append(t - min(ts, key=lambda m: abs(m - t)))
    else:
        unmatched.append((t, p[3:].hex(" ")))
print("bus minus mirror (ms):", sorted(Counter(lags).items()))
print("bus frames with no mirror match within 50 ms:", unmatched[:20])
