"""Build hil/build/ssc_native.dll: the ESP32 safety-controller core (hil/SafeStopCore) compiled for the PC.

    .venv\\Scripts\\python.exe scripts/build_native.py

Uses the zig C++ compiler from PyPI (pip install ziglang): no Visual Studio, no MinGW. The same source files are
compiled for the ESP32-S3 by scripts/hil_flash.py; only the -DSSC_HOST_API C API is added here.
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "hil" / "SafeStopCore" / "src"
OUT = ROOT / "hil" / "build" / ("ssc_native.dll" if sys.platform == "win32" else "libssc_native.so")


def build() -> Path:
    OUT.parent.mkdir(parents=True, exist_ok=True)
    cmd = [sys.executable, "-m", "ziglang", "c++", "-std=c++17", "-O2", "-shared", "-w", "-Wall", "-DSSC_HOST_API",
           "-fno-fast-math", "-ffp-contract=off",   # no fused multiply-add: results must match Python bit for bit
           "-o", str(OUT), str(SRC / "ssc_core.cpp"), str(SRC / "ssc_host_api.cpp")]
    subprocess.run(cmd, check=True)
    return OUT


if __name__ == "__main__":
    print(f"built {build()}")
