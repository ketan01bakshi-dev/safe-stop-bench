"""Preflight for a HiL run (P2): the bench checks itself before it is allowed to judge the product.

    python -m health.preflight            quick: ports, USB hello vs inventory, UDS version read of both boards (about 8 s)
    python -m health.preflight --full     + firmware source vs board, compile both sketches, partition audit, esptool upload probe
    python run.py --dut hil ...           runs the quick preflight first and refuses to start when it fails (--skip-preflight)

Every check is ok / warn / fail / skip with a plain-words detail; a failure carries fixes from health/error_catalogue.json
(seeded from the real bring-up traps in docs/HIL_FIRST_RUN.md). The result is written to reports/health/preflight_latest.json
for the dashboard. Only a 'fail' blocks the run; a 'warn' is shown. The checks never change a board, except that the full
level's esptool probe and the silent-node recovery reset it (a reset is harmless: B boots into its restored state).
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Callable

from ssb.config import ROOT

from .check import load_inventory
from .uds import uds_report

CATALOGUE = Path(__file__).with_name("error_catalogue.json")
HEALTH_DIR = ROOT / "reports" / "health"
FQBN = "esp32:esp32:esp32s3"
SKETCHES = {"board_b": ROOT / "hil" / "firmware" / "SafetyNode", "board_a": ROOT / "hil" / "firmware" / "BusNode"}
HELLO_RE = re.compile(r"(SafetyNode|BusNode) (\d+(?:\.\d+)+)")


@dataclass
class Check:
    name: str
    status: str                      # ok | warn | fail | skip
    detail: str
    fixes: list[str] = field(default_factory=list)


def load_catalogue(path: Path = CATALOGUE) -> list[dict]:
    return json.loads(path.read_text(encoding="utf-8"))


def explain(text: str, catalogue: list[dict] | None = None) -> list[dict]:
    """Catalogue entries whose pattern matches a failure text (regex, case-insensitive): the cause and the fixes."""
    return [e for e in (catalogue or load_catalogue()) if re.search(e["pattern"], text, re.I)]


def _fail(name: str, detail: str, status: str = "fail") -> Check:
    fixes: list[str] = []
    for e in explain(detail):
        fixes += [f"{e['cause']}"] + e["fix"]
    return Check(name, status, detail, list(dict.fromkeys(fixes)))


# ---- the real world (replaceable in tests) ------------------------------------------------------------------------

def list_ports() -> list[dict]:
    from serial.tools import list_ports as lp
    return [{"port": p.device, "description": p.description, "vid": p.vid, "pid": p.pid} for p in lp.comports()]


def open_link(port: str):
    from ssb.hil import SerialLink
    return SerialLink(port)


def read_hello(link, timeout_s: float = 3.0) -> str | None:
    from ssb.hil import Parser
    par, end = Parser(), time.time() + timeout_s
    while time.time() < end:
        for kind, p in par.feed(link.read()):
            if kind == "H":
                return p.decode(errors="replace")
        time.sleep(0.01)
    return None


def source_version(role: str) -> str | None:
    """The version the firmware source on disk would report (FW_VERSION macro, else the literal in the hello string)."""
    text = (SKETCHES[role] / f"{SKETCHES[role].name}.ino").read_text(encoding="utf-8")
    m = re.search(r'#define\s+FW_VERSION\s+"([\d.]+)"', text) or re.search(r'(?:SafetyNode|BusNode) ([\d.]+) \(', text)
    return m.group(1) if m else None


def arduino_cli() -> str | None:
    for c in (os.environ.get("ARDUINO_CLI"), shutil.which("arduino-cli")):
        if c and Path(c).exists():
            return c
    return None


def compile_sketch(role: str) -> dict:
    cli = arduino_cli()
    if not cli:
        return {"ok": False, "output": "arduino-cli not found: set ARDUINO_CLI"}
    build = ROOT / "hil" / "build" / f"preflight_{role}"
    p = subprocess.run([cli, "compile", "--fqbn", FQBN, "--library", str(ROOT / "hil" / "SafeStopCore"), "--build-path", str(build),
                        str(SKETCHES[role])], capture_output=True, text=True, encoding="utf-8", errors="replace")
    out = (p.stdout or "") + (p.stderr or "")
    m = re.search(r"Sketch uses (\d+) bytes \((\d+)%\).*?Maximum is (\d+) bytes", out, re.S)
    part = build / "partitions.csv"
    return {"ok": p.returncode == 0, "output": out[-1500:], "used": m and int(m[1]), "percent": m and int(m[2]), "max": m and int(m[3]),
            "partitions": part.read_text(encoding="utf-8") if part.exists() else None}


def esptool_path() -> str | None:
    found = sorted((Path.home() / "AppData/Local/Arduino15/packages/esp32/tools/esptool_py").glob("*/esptool.exe"))
    return str(found[-1]) if found else shutil.which("esptool")


def upload_probe(port: str) -> dict:
    """esptool flash-id: reaches the ROM bootloader, reads chip and flash size, writes nothing. Resets the board afterwards."""
    exe = esptool_path()
    if not exe:
        return {"ok": False, "output": "esptool not found"}
    p = subprocess.run([exe, "--chip", "esp32s3", "--port", port, "--baud", "115200", "flash-id"], capture_output=True, text=True,
                       encoding="utf-8", errors="replace", timeout=40)
    out = (p.stdout or "") + (p.stderr or "")
    size = re.search(r"Detected flash size:\s*(\d+)\s*MB", out)
    return {"ok": p.returncode == 0 and "ESP32-S3" in out, "output": out[-600:], "flash_mb": size and int(size[1])}


@dataclass
class World:
    list_ports: Callable[..., Any] = list_ports
    open_link: Callable[..., Any] = open_link
    read_hello: Callable[..., Any] = read_hello
    source_version: Callable[..., Any] = source_version
    compile_sketch: Callable[..., Any] = compile_sketch
    upload_probe: Callable[..., Any] = upload_probe
    uds_report: Callable[..., Any] = uds_report


# ---- the checks ---------------------------------------------------------------------------------------------------

def partition_end(csv: str | None) -> int:
    """End offset of the last partition in a partitions.csv (the flash size the layout needs)."""
    end = 0
    for line in (csv or "").splitlines():
        cols = [c.strip() for c in line.split(",")]
        if len(cols) >= 5 and not line.lstrip().startswith("#"):
            try:
                end = max(end, int(cols[3], 0) + int(cols[4], 0))
            except ValueError:
                pass
    return end


def run_preflight(level: str = "quick", inventory: dict | None = None, world: World | None = None, write: bool = True,
                  ports_override: dict[str, str] | None = None) -> dict:
    inv, w = inventory or load_inventory(), world or World()
    for b, port in (ports_override or {}).items():   # run.py --port-b / --port-a beat the inventory's hint
        if port and b in inv:
            inv = {**inv, b: {**inv[b], "port_hint": port}}
    boards = [k for k in inv if not k.startswith("_")]
    checks: list[Check] = []
    hellos: dict[str, str | None] = {}
    suspect: list[str] = []

    # 1. ports: the expected USB-UART bridges are there
    seen = {p["port"]: p for p in w.list_ports()}
    ports: dict[str, str] = {}
    for b in boards:
        port = inv[b].get("port_hint", "")
        ports[b] = port
        if port in seen:
            vid = seen[port].get("vid")
            checks.append(Check(f"port {b}", "ok" if vid in (0x1A86, 0x10C4, 0x0403, 0x303A) else "warn",
                                f"{port}: {seen[port]['description']}" + ("" if vid in (0x1A86, 0x10C4, 0x0403, 0x303A) else " (not a known USB-UART bridge)")))
        else:
            checks.append(_fail(f"port {b}", f"no COM port {port} for {b}; present: {', '.join(sorted(seen)) or 'none'}"))
    live = [b for b in boards if ports[b] in seen]

    # 2. full level: the ROM bootloader answers, and the flash is as big as the partition table assumes
    flash_mb: dict[str, int | None] = {}
    if level == "full":
        for b in live:
            r = w.upload_probe(ports[b])
            flash_mb[b] = r.get("flash_mb")
            checks.append(Check(f"upload probe {b}", "ok", f"esptool reached the ESP32-S3 on {ports[b]}, flash {r.get('flash_mb') or '?'} MB")
                           if r["ok"] else _fail(f"upload probe {b}", r.get("output", "")))
        time.sleep(2.0)   # the probe resets the board: let it boot and send its hello

    # 3. USB hello vs inventory (and silent-node recovery), UDS through A
    links: dict[str, Any] = {}
    uds: dict = {}
    try:
        for b in live:
            try:
                links[b] = w.open_link(ports[b])
            except Exception as e:   # noqa: BLE001  port busy etc.
                checks.append(_fail(f"open {b}", f"{ports[b]}: {e}"))
        for b, link in links.items():
            h = w.read_hello(link)
            if h is None and hasattr(link, "reset_board"):
                link.reset_board()   # a stale 'active' node sends no hello until reset (HIL_FIRST_RUN finding 3)
                h = w.read_hello(link)
            hellos[b] = h
            want = inv[b]["firmware"]
            m = HELLO_RE.search(h or "")
            if h is None:
                checks.append(_fail(f"hello {b}", f"{b}: no firmware hello on {ports[b]} (inventory expects {want})"))
            elif m and f"{m[1]} {m[2]}" != want:
                msg = f"{b}: {m[1]} {m[2]} is not the inventory's {want}"
                checks.append(_fail(f"hello {b}", msg))
                suspect.append(msg)
            elif "FAILED" in h:
                checks.append(_fail(f"hello {b}", f"{b}: {h}"))
            else:
                checks.append(Check(f"hello {b}", "ok", h))
        if "board_a" in links and len(hellos) == len(boards):
            uds = w.uds_report(links["board_a"], inv, hellos)
            if uds["problems"]:
                for pr in uds["problems"]:
                    checks.append(_fail("uds", pr))
            else:
                ids = "; ".join(f"{n}: v{i['version']} serial {i['serial']}" for n, i in uds["boards"].items())
                checks.append(Check("uds", "ok", f"TesterPresent, version and serial read from both boards ({ids})"))
        else:
            checks.append(Check("uds", "skip", "needs both boards and the bus node's link"))
    finally:
        for link in links.values():
            try:
                link.close()
            except Exception:   # noqa: BLE001
                pass

    # 4. full level: firmware source vs board, compile, partition audit
    if level == "full":
        for b in boards:
            src, got = w.source_version(b), HELLO_RE.search(hellos.get(b) or "")
            if src and got and src != got[2]:
                msg = f"{b}: the board runs {got[2]} but the source on disk is {src} (board is older than the source: reflash)"
                checks.append(Check(f"source {b}", "warn", msg, ["python scripts/hil_flash.py --b COM13 --a COM14"]))
            elif src:
                checks.append(Check(f"source {b}", "ok", f"source and board agree on {src}"))
            c = w.compile_sketch(b)
            if not c["ok"]:
                checks.append(_fail(f"compile {b}", f"{SKETCHES[b].name} does not compile: {c['output'][-300:]}"))
                continue
            checks.append(Check(f"compile {b}", "ok", f"{SKETCHES[b].name} compiles"))
            pct, mx = c.get("percent"), c.get("max")
            need = partition_end(c.get("partitions"))
            have = (flash_mb.get(b) or 0) * 1024 * 1024
            if pct is None:
                checks.append(Check(f"partition {b}", "warn", "compile output has no size line"))
            elif have and need > have:
                checks.append(Check(f"partition {b}", "fail", f"the partition table ends at 0x{need:X} but the chip has {flash_mb[b]} MB",
                                    ["Pick a partition scheme that fits the chip's flash."]))
            else:
                st = "fail" if pct >= 100 else "warn" if pct >= 85 else "ok"
                checks.append(Check(f"partition {b}", st, f"app uses {pct}% of its {mx // 1024} KiB partition"
                                    + (f"; layout ends at 0x{need:X}" if need else "") + (f" of {flash_mb[b]} MB flash" if have else ""),
                                    ["Build with -Os and drop unused code before adding features."] if st != "ok" else []))

    result = {"time": time.strftime("%Y-%m-%dT%H:%M:%S"), "level": level, "ok": not any(c.status == "fail" for c in checks),
              "checks": [asdict(c) for c in checks], "firmware": hellos, "suspect_bench": suspect,
              "uds": uds.get("boards") or {}}
    if write:
        HEALTH_DIR.mkdir(parents=True, exist_ok=True)
        text = json.dumps(result, indent=1, default=str)
        (HEALTH_DIR / "preflight_latest.json").write_text(text, encoding="utf-8")
        (HEALTH_DIR / f"preflight_{time.strftime('%Y%m%d_%H%M%S')}.json").write_text(text, encoding="utf-8")
    return result


def render(result: dict) -> str:
    icon = {"ok": "PASS", "warn": "WARN", "fail": "FAIL", "skip": "skip"}
    lines = [f"Preflight ({result['level']}): {'READY' if result['ok'] else 'NOT READY'}"]
    for c in result["checks"]:
        lines.append(f"  [{icon[c['status']]}] {c['name']}: {c['detail']}")
        lines += [f"         fix: {f}" for f in c["fixes"]]
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description="HiL preflight")
    ap.add_argument("--full", action="store_true", help="also compile, audit partitions, compare source and board, esptool probe")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    r = run_preflight("full" if a.full else "quick")
    print(json.dumps(r, indent=1, default=str) if a.json else render(r))
    return 0 if r["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
