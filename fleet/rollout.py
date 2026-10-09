"""Staged rollout across a mixed fleet (P4): emulated SafetyNodes plus the one real board, one halt rule.

    wave 1  a canary (at least one device)     wave 2  a quarter of the fleet     wave 3  the rest
After each wave, if more than `halt_pct` percent of the devices touched SO FAR failed (rolled back, rejected, or never confirmed), the rollout
stops and the remaining devices are not touched. A device the image is not meant for (another hardware revision) or that already runs
this version or newer is skipped and is not a failure. With a canary of one, a single bad outcome is 100 %: a bad build is stopped after
the first device.

Every device is updated through the same `OtaClient` calls (BEGIN, CHUNK..., END, REBOOT, STATUS), whether it is an emulated board or the
real one on COM13, so one rollout proves both.
"""
from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass, field

from .image import SignedImage
from .ota_client import Faults, OtaClient
from .telemetry import Telemetry

COMMIT_WAIT_S = 3.5   # the trial image commits itself after 3 s of healthy operation


@dataclass
class Device:
    name: str
    link: object
    kind: str = "emulated"                 # emulated | real
    hardware: str = "rev-b"
    advance: Callable[[float], None] | None = None   # emulated boards: move time on; real boards: wait in real time
    image: SignedImage | None = None                 # this device's own build of the release (the real board takes the real binary)


@dataclass
class DeviceResult:
    name: str
    kind: str
    outcome: str                           # committed | rolled_back | rejected | skipped | failed
    detail: str = ""
    seconds: float = 0.0

    @property
    def failure(self) -> bool:
        return self.outcome in ("rolled_back", "rejected", "failed")


@dataclass
class Wave:
    index: int
    devices: list[str]
    results: list[DeviceResult] = field(default_factory=list)


@dataclass
class RolloutResult:
    version: str
    waves: list[Wave]
    halted: bool
    halt_reason: str
    untouched: list[str]

    @property
    def results(self) -> list[DeviceResult]:
        return [r for w in self.waves for r in w.results]

    def count(self, outcome: str) -> int:
        return sum(r.outcome == outcome for r in self.results)

    def summary(self) -> dict:
        return {"version": self.version, "halted": self.halted, "halt_reason": self.halt_reason, "touched": len(self.results),
                "committed": self.count("committed"), "rolled_back": self.count("rolled_back"), "rejected": self.count("rejected"),
                "skipped": self.count("skipped"), "failed": self.count("failed"), "untouched": len(self.untouched),
                "waves": [{"wave": w.index, "devices": len(w.devices), "failures": sum(r.failure for r in w.results)} for w in self.waves]}


def update_device(dev: Device, img: SignedImage, targets: set[str], tel: Telemetry, faults: Faults | None = None) -> DeviceResult:
    t0 = time.time()
    if dev.hardware not in targets:
        tel.emit(dev.name, "skipped", reason="hardware revision not targeted", hardware=dev.hardware)
        return DeviceResult(dev.name, dev.kind, "skipped", f"hardware {dev.hardware} is not targeted")
    client = OtaClient(dev.link)
    try:
        img = dev.image or img
        r = client.push(img, faults)
        if r.status == "DOWNGRADE":
            tel.emit(dev.name, "skipped", reason="already on this version or newer")
            return DeviceResult(dev.name, dev.kind, "skipped", "already on this version or newer")
        if r.outcome != "staged":
            tel.emit(dev.name, "update_rejected", status=r.status)
            return DeviceResult(dev.name, dev.kind, "rejected", r.status, time.time() - t0)
        tel.emit(dev.name, "staged", version=img.version, seconds=round(r.seconds, 1))
        client.reboot()
        client.wait_for_hello(10.0) if dev.kind == "real" else None
        (dev.advance or time.sleep)(COMMIT_WAIT_S)
        st = client.status()
    except Exception as e:   # noqa: BLE001  a device that does not answer is a failed update, not a crashed rollout
        tel.emit(dev.name, "update_failed", error=str(e))
        return DeviceResult(dev.name, dev.kind, "failed", str(e), time.time() - t0)
    if st["state"] == "NONE" and st["last"].startswith("committed"):
        tel.emit(dev.name, "commit", version=img.version)
        return DeviceResult(dev.name, dev.kind, "committed", st["last"], time.time() - t0)
    tel.emit(dev.name, "rollback", detail=st["last"])
    return DeviceResult(dev.name, dev.kind, "rolled_back", st["last"], time.time() - t0)


def plan_waves(devices: list[Device], canary: int = 1, fractions: tuple[float, ...] = (0.25, 1.0)) -> list[list[Device]]:
    waves, done = [devices[:canary]], canary
    for f in fractions:
        upto = max(done, round(len(devices) * f))
        if upto > done:
            waves.append(devices[done:upto])
            done = upto
    return [w for w in waves if w]


def rollout(devices: list[Device], img: SignedImage, targets: set[str] | None = None, tel: Telemetry | None = None,
            halt_pct: float = 5.0, canary: int = 1, fractions: tuple[float, ...] = (0.25, 1.0),
            faults: dict[str, Faults] | None = None) -> RolloutResult:
    tel, targets, faults = tel or Telemetry(), targets or {"rev-b"}, faults or {}
    waves, halted, why, touched, failed = [], False, "", 0, 0
    plan = plan_waves(devices, canary, fractions)
    for i, group in enumerate(plan, 1):
        wave = Wave(i, [d.name for d in group])
        tel.emit("fleet", "wave_start", wave=i, devices=len(group), version=img.version)
        for d in group:
            res = update_device(d, img, targets, tel, faults.get(d.name))
            wave.results.append(res)
            touched += res.outcome != "skipped"
            failed += res.failure
        waves.append(wave)
        rate = 100.0 * failed / touched if touched else 0.0
        tel.emit("fleet", "wave_done", wave=i, failed=failed, touched=touched, failure_pct=round(rate, 1))
        if rate > halt_pct:
            halted, why = True, f"{failed} of {touched} updated devices failed ({rate:.0f} % > {halt_pct:g} %) after wave {i}"
            tel.emit("fleet", "halt", reason=why)
            break
    done = {n for w in waves for n in w.devices}
    return RolloutResult(img.version, waves, halted, why, [d.name for d in devices if d.name not in done])
