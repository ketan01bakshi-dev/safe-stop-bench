"""Compile (and optionally upload) the HiL firmware for the EdgeHex ESP32-S3 boards.

    python scripts/hil_flash.py                         # compile both sketches, print flash/RAM use
    python scripts/hil_flash.py --b COM14 --a COM13     # compile and upload: B = safety controller, A = bus node

Uses arduino-cli (ARDUINO_CLI or PATH), the esp32:esp32 core and the autowp-mcp2515 library.
"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FQBN = "esp32:esp32:esp32s3"
SKETCHES = {"b": ROOT / "hil" / "firmware" / "SafetyNode", "a": ROOT / "hil" / "firmware" / "BusNode"}


def cli() -> str:
    for c in (os.environ.get("ARDUINO_CLI"), shutil.which("arduino-cli")):
        if c and Path(c).exists():
            return c
    sys.exit("arduino-cli not found: set ARDUINO_CLI")


def compile_sketch(role: str) -> dict:
    sk = SKETCHES[role]
    build = ROOT / "hil" / "build" / f"fw_{role}"
    p = subprocess.run([cli(), "compile", "--fqbn", FQBN, "--library", str(ROOT / "hil" / "SafeStopCore"),
                        "--build-path", str(build), "--warnings", "default", str(sk)], capture_output=True, text=True)
    if p.returncode != 0:
        print(p.stdout[-3000:], p.stderr[-3000:])
        sys.exit(f"compile failed: {sk.name}")
    flash = re.search(r"Sketch uses (\d+) bytes \((\d+)%\)", p.stdout)
    ram = re.search(r"Global variables use (\d+) bytes \((\d+)%\)", p.stdout)
    warn = [ln for ln in (p.stdout + p.stderr).splitlines() if "warning:" in ln and "SafeStop" in ln]
    return {"sketch": sk.name, "build": build, "flash": flash and f"{int(flash[1]) // 1024} KiB ({flash[2]}%)",
            "ram": ram and f"{int(ram[1]) // 1024} KiB ({ram[2]}%)", "warnings": warn}


def upload(role: str, port: str) -> None:
    sk, build = SKETCHES[role], ROOT / "hil" / "build" / f"fw_{role}"
    subprocess.run([cli(), "upload", "-p", port, "--fqbn", FQBN, "--build-path", str(build), str(sk)], check=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--b", help="COM port of board B (safety controller): upload SafetyNode")
    ap.add_argument("--a", help="COM port of board A (bus node): upload BusNode")
    a = ap.parse_args()
    for role in ("b", "a"):
        r = compile_sketch(role)
        print(f"{r['sketch']:11} flash {r['flash']}, RAM {r['ram']}, {len(r['warnings'])} warning(s) in our code")
        for w in r["warnings"]:
            print("   ", w)
    for role, port in (("b", a.b), ("a", a.a)):
        if port:
            upload(role, port)
            print(f"uploaded {SKETCHES[role].name} to {port}")
