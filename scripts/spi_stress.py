"""Find out whether one of the SPI / power wires between an ESP32-S3 and its MCP2515 module is faulty, and which (v2.9.4). Needs hil/firmware/SpiStress on the board.

    python scripts/spi_stress.py --port COM13 --seconds 120      # hands-off: error signature
    python scripts/spi_stress.py --port COM13 --wiggle           # guided: wiggle one wire at a time when told
    add --normal --port-a COM14: B transmits on the real bus (A running BusNode ACKs and forwards); A's side is
    watched for frames B never requested (TXB1 0x200, the TXB2 canary 0x7FF): the stale-frame replay mechanism

Every 500 ms the sketch reports that window's counts; this script adds them up per phase and names the likely line.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
import time
from pathlib import Path

import serial

ROOT = Path(__file__).resolve().parent.parent
# module pin -> ESP32 pin (HIL_ESP32.md section 3); INT is not used by the SafetyNode firmware
WIRES = [("VCC", "5V"), ("GND", "GND"), ("CS", "GPIO10"), ("SO", "GPIO13 (MISO)"), ("SI", "GPIO11 (MOSI)"),
         ("SCK", "GPIO12"), ("INT", "GPIO4")]
KEYS = ["it", "rf", "rs", "wf", "un", "tail", "iso", "rst", "cfg", "mode"]
LINE = re.compile(r"^S (\d+) (.*)$")


def parse(line: str) -> dict | None:
    m = LINE.match(line)
    if not m:
        return None
    d: dict = {}
    for kv in m.group(2).split():
        k, v = kv.split("=")
        d[k] = [int(x) for x in v.split(",")] if "," in v else int(v)
    return d


def add(acc: dict, d: dict) -> None:
    for k, v in d.items():
        if isinstance(v, list):
            acc[k] = [a + b for a, b in zip(acc.get(k, [0] * len(v)), v, strict=True)]
        else:
            acc[k] = acc.get(k, 0) + v


def verdict(a: dict) -> list[str]:
    it = max(a.get("it", 0), 1)
    rf, rs, wf = a.get("rf", 0), a.get("rs", 0), a.get("wf", 0)
    tx, rst, cfg, mode = a.get("tx", [0, 0, 0]), a.get("rst", 0), a.get("cfg", 0), a.get("mode", 0)
    cls = a.get("cls", [0] * 5)
    out = [f"{it} test cycles: read errors {rf} at 10 MHz, {rs} at 1 MHz; write errors {wf} at 10 MHz; "
           f"unstable read-backs {a.get('un', 0)}; burst tails {a.get('tail', 0)}, isolated {a.get('iso', 0)}; "
           f"byte classes ff/00/shift/1bit/other {cls}; spurious transmits TXB0/1/2 {tx}; chip resets {rst}; "
           f"config overwritten {cfg}; mode changed {mode}"]
    if rst:
        out.append("-> CHIP RESETS: the module lost power for a moment: VCC or GND jumper (or the 5 V supply).")
    if sum(tx) or cfg or mode:
        out.append("-> SPURIOUS COMMANDS (transmits / config / mode changes nobody asked for): a write-path line, CS or SI. "
                   "If write errors are also high, SI; if bursts end in tails, CS.")
    if (rf or rs) and not wf:
        out.append("-> READ-ONLY ERRORS: the SO (MISO, GPIO13) jumper.")
    if wf and not (rf or rs):
        out.append("-> WRITE-ONLY ERRORS: the SI (MOSI, GPIO11) jumper.")
    if wf and (rf or rs):
        shift = cls[2] / max(sum(cls), 1)
        out.append(f"-> READ AND WRITE ERRORS ({shift:.0%} one-bit shifts): "
                   + ("the SCK (GPIO12) jumper." if shift > 0.3 or a.get("tail", 0) > a.get("iso", 0) else "CS or GND; see the wiggle test."))
    if rf and not rs:
        out.append("-> errors only at 10 MHz: signal integrity (long or loose wires, a poor GND return) rather than an open contact.")
    if "a_0x100" in a or "a_other" in a:
        out.append(f"   bus as seen by A: {a.get('a_0x100', 0)} requested frames (B sent {a.get('sent', 0)}); never-requested "
                   f"TXB1 {a.get('a_TXB1', 0)}, TXB2 canary {a.get('a_TXB2', 0)}, other IDs {a.get('a_other', 0)}")
        if a.get("a_TXB1", 0) or a.get("a_TXB2", 0):
            out.append("-> STALE-BUFFER REPLAY REPRODUCED: a buffer nobody requested went out on the bus.")
    if len(out) == 1:
        out.append(f"-> no SPI errors in {it} cycles (about {it * 40} transactions).")
    return out


def run(port: str, phases: list[tuple[str, float]], log_path: Path, normal: bool = False, port_a: str | None = None) -> dict:
    s = serial.Serial(port, 921600, timeout=0.1)
    sa, pa = None, None
    if port_a:
        sys.path.insert(0, str(ROOT))
        from ssb.hil import Parser
        sa, pa = serial.Serial(port_a, 921600, timeout=0), Parser()
    s.dtr = False
    s.rts = True
    time.sleep(0.1)
    s.rts = False                      # reset the board so the line test and the setup report are fresh
    buf, results, t0 = b"", {}, time.time()
    with log_path.open("w", encoding="utf-8") as log:
        boot_end = time.time() + 2.5
        while time.time() < boot_end:
            buf += s.read(4096)
        for raw in buf.split(b"\n"):
            line = raw.decode(errors="replace").strip()
            if line.startswith(("L ", "H ")):
                print(" ", line)
                log.write(line + "\n")
        buf = b""
        if normal:
            s.write(b"N")
            time.sleep(0.3)
            print(" ", s.read(4096).decode(errors="replace").strip().splitlines()[-1:])
        if sa:
            sa.reset_input_buffer()
        for label, secs in phases:
            if label != "baseline" and not label.startswith("rest"):
                print(f"\n>>> NOW: gently wiggle the {label} jumper at BOTH ends for {secs:.0f} s (keep it plugged in)")
            elif label.startswith("rest"):
                print(f"\n    hands off ({secs:.0f} s)")
            else:
                print(f"\n    baseline: hands off ({secs:.0f} s)")
            acc: dict = {}
            t_end = time.time() + secs
            last_shown = 0.0
            while time.time() < t_end:
                buf += s.read(4096)
                if sa and pa:
                    for k, p in pa.feed(sa.read(65536)):
                        if k == "M":
                            cid = p[0] | p[1] << 8
                            key = {0x100: "a_0x100", 0x200: "a_TXB1", 0x7FF: "a_TXB2"}.get(cid, "a_other")
                            acc[key] = acc.get(key, 0) + 1
                *lines, buf = buf.split(b"\n")
                for raw in lines:
                    line = raw.decode(errors="replace").strip()
                    d = parse(line)
                    if d:
                        add(acc, d)
                        log.write(f"{time.time() - t0:.1f} {label} {line}\n")
                if time.time() - last_shown >= 1.0:
                    last_shown = time.time()
                    errs = acc.get("a_TXB1", 0) + acc.get("a_TXB2", 0) + acc.get("rf", 0) + acc.get("rs", 0) + acc.get("wf", 0) + sum(acc.get("tx", [0])) + acc.get("rst", 0)
                    print(f"    {t_end - time.time():4.0f} s left   errors so far: {errs}", end="\r", flush=True)
            print()
            results.setdefault(label.split(" #")[0], {})
            add(results[label.split(" #")[0]], acc)
    s.close()
    if sa:
        sa.close()
    return results


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM13")
    ap.add_argument("--seconds", type=float, default=60)
    ap.add_argument("--wiggle", action="store_true")
    ap.add_argument("--normal", action="store_true", help="transmit on the real bus (needs board A on --port-a)")
    ap.add_argument("--port-a", default=None)
    a = ap.parse_args()
    if a.wiggle:
        phases = [("baseline", 15.0)]
        for name, pin in WIRES:
            phases += [(f"{name} ({pin})", 15.0), (f"rest #{name}", 5.0)]
    else:
        phases = [("baseline", a.seconds)]
    stamp = time.strftime("%Y%m%d_%H%M%S")
    log = ROOT / "reports" / f"spi_stress_{stamp}.log"
    res = run(a.port, phases, log, normal=a.normal, port_a=a.port_a)
    print("\n================ RESULT ================")
    base = res.get("baseline", {})
    for label, acc in res.items():
        if label.startswith("rest"):
            continue
        print(f"\n[{label}]")
        for line in verdict(acc):
            print("  " + line)
    if a.wiggle:
        print("\nWire ranking (errors per 1000 cycles while wiggled, baseline first):")
        def rate(acc):
            e = (acc.get("rf", 0) + acc.get("rs", 0) + acc.get("wf", 0) + sum(acc.get("tx", [0, 0, 0])) + 50 * acc.get("rst", 0)
                 + 50 * (acc.get("a_TXB1", 0) + acc.get("a_TXB2", 0)))
            return 1000 * e / max(acc.get("it", 0), 1)
        rows = sorted(((rate(v), k) for k, v in res.items() if not k.startswith("rest")), reverse=True)
        for r, k in rows:
            print(f"  {r:9.2f}  {k}{'   <- baseline' if k == 'baseline' else ''}")
        print(f"  baseline rate {rate(base):.2f}")
    out = log.with_suffix(".json")
    out.write_text(json.dumps(res, indent=1), encoding="utf-8")
    print(f"\nlog: {log}\nsummary: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
