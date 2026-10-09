"""Firmware images for the OTA path (P4): build variants of SafetyNode and sign them.

A signed image = the app binary + SHA-256 + HMAC-SHA256(key, sha256 | size:u32 LE | version). The version is part of what is signed,
so an old image cannot be re-labelled as a new one. The key is a DEMO key shared with the firmware (hil/firmware/SafetyNode/ssc_ota.h);
a product signs on a build server with a key that never leaves it.
"""
from __future__ import annotations

import hashlib
import hmac
import struct
from dataclasses import dataclass
from pathlib import Path

from ssb.config import ROOT

DEMO_KEY = b"demo-only-ssb-ota-key"
IMAGES = ROOT / "fleet" / "images"
SKETCH = ROOT / "hil" / "firmware" / "SafetyNode"
ESP_IMAGE_MAGIC = 0xE9


@dataclass(frozen=True)
class SignedImage:
    version: str
    data: bytes
    sha256: bytes
    hmac: bytes

    @property
    def size(self) -> int:
        return len(self.data)


def sign(data: bytes, version: str, key: bytes = DEMO_KEY) -> SignedImage:
    sha = hashlib.sha256(data).digest()
    mac = hmac.new(key, sha + struct.pack("<I", len(data)) + version.encode(), hashlib.sha256).digest()
    return SignedImage(version, data, sha, mac)


def verify(img: SignedImage, key: bytes = DEMO_KEY) -> bool:
    """The check the board runs, on the PC (tests, and the emulated fleet)."""
    if hashlib.sha256(img.data).digest() != img.sha256:
        return False
    want = hmac.new(key, img.sha256 + struct.pack("<I", img.size) + img.version.encode(), hashlib.sha256).digest()
    return hmac.compare_digest(want, img.hmac)


def tag_of(minor: int, unhealthy: bool = False, cut_at: int | None = None) -> str:
    return f"2.{minor}" + ("_unhealthy" if unhealthy else "") + (f"_cut{cut_at}" if cut_at else "")


def build(minor: int, unhealthy: bool = False, cut_at: int | None = None) -> Path:
    """Compile SafetyNode as version 2.<minor> and return the app .bin. `unhealthy`: its health check fails (the rollback test).
    `cut_at=n`: a test build that resets itself once at the nth persistent write of the next update's END (a power cut at that boundary)."""
    import subprocess

    from health.preflight import arduino_cli
    cli = arduino_cli()
    if not cli:
        raise RuntimeError("arduino-cli not found: set ARDUINO_CLI")
    tag = tag_of(minor, unhealthy, cut_at)
    out = ROOT / "hil" / "build" / f"fleet_{tag}"
    flags = f"-DFW_MINOR={minor}" + (" -DOTA_UNHEALTHY=1" if unhealthy else "") + (f" -DOTA_CUT_AT={cut_at}" if cut_at else "")
    p = subprocess.run([cli, "compile", "--fqbn", "esp32:esp32:esp32s3", "--library", str(ROOT / "hil" / "SafeStopCore"), "--build-path", str(out),
                        "--build-property", f"compiler.cpp.extra_flags={flags}", str(SKETCH)], capture_output=True, text=True, encoding="utf-8", errors="replace")
    if p.returncode != 0:
        raise RuntimeError(f"compile of SafetyNode {tag} failed:\n{(p.stdout + p.stderr)[-1500:]}")
    IMAGES.mkdir(parents=True, exist_ok=True)
    dst = IMAGES / f"SafetyNode_{tag}.bin"
    dst.write_bytes((out / "SafetyNode.ino.bin").read_bytes())
    return dst


def load(path: Path, version: str) -> SignedImage:
    data = Path(path).read_bytes()
    if data[0] != ESP_IMAGE_MAGIC:
        raise ValueError(f"{path} is not an ESP32 application image")
    return sign(data, version)
