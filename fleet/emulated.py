"""An emulated SafetyNode for the fleet (P4): the same OTA protocol and state machine as hil/firmware/SafetyNode/ssc_ota.h, in Python.

`EmuBoard` behaves as a serial link (`write()` takes the PC's bytes, `read()` returns the board's), so the very same `OtaClient` that
updates the real board updates an emulated one. That is what makes a mixed fleet possible: N emulated vECUs plus the one real board
under one rollout. The emulation is held to the firmware by `tests/test_fleet.py` (same checks, same statuses, same rollback rules);
where the two differ, the real board wins.

Images for the emulation are synthetic: bytes that start with b"EMU|<version>|OK|" (healthy) or b"EMU|<version>|BAD|" (fails its health
check), signed with the same HMAC scheme as the real ones. Power can be cut at any point: `reset()` drops the receive state and boots.
"""
from __future__ import annotations

import hashlib
import hmac
import random
import struct

from ssb.hil import Parser, frame_msg

from .image import DEMO_KEY, SignedImage, sign

OK, BUSY, BAD_STATE, BAD_SIZE, NO_SPACE, BAD_OFFSET, WRITE_FAIL, HASH_MISMATCH, BAD_SIGNATURE, END_FAIL, NOT_STAGED, BAD_MESSAGE, DOWNGRADE = range(13)
NONE, TRIAL, ROLLED_BACK = 0, 1, 2
SLOT_ADDR = (0x10000, 0x150000)
# The persistent writes that end an update, in the order the firmware makes them. A power cut can fall between any two.
#   fixed  (v2.26+): the probation record first, the boot-slot switch last, so a cut never leaves a new image without its record
#   legacy (v2.26 as first flashed): the switch first, then the record, with the state flag written before the slot it points at
END_WRITES = ("prev", "tries", "last", "st", "switch")
END_WRITES_LEGACY = ("switch", "st", "tries", "prev", "last")


def synthetic_image(version: str, healthy: bool = True, size: int = 60_000, seed: int | None = None) -> SignedImage:
    head = f"EMU|{version}|{'OK' if healthy else 'BAD'}|".encode()
    rng = random.Random(seed if seed is not None else version)
    return sign(head + rng.randbytes(size - len(head)), version)


def minor_of(v: str) -> int:
    return int(v.split(".")[1]) if "." in v else -1


class EmuBoard:
    realtime = True

    def __init__(self, name: str, version: str = "2.10", key: bytes = DEMO_KEY, hardware: str = "rev-b"):
        self.name, self.key, self.hardware = name, key, hardware
        self.slots: list[tuple[str, bool]] = [(version, True), ("", False)]   # (version, image present & healthy)
        self.running = self.boot_slot = 0   # boot_slot = what the bootloader will start next (the firmware switches it at END)
        self.st, self.tries, self.last = NONE, 0, ""
        self.prev: int | None = None   # the slot to roll back to; None = not recorded (address 0 in the firmware: no such slot)
        self.legacy_ota = False        # reproduce the first END order (see END_WRITES_LEGACY) to show what the cut-window test catches
        self.cut_end_after: int | None = None   # lose power right after the Nth persistent write of the next END
        self.parser, self.out = Parser(), bytearray()
        self.pc_session = False
        self.t_boot_s = 0.0          # seconds since boot (the test advances it)
        self.events: list[dict] = []
        self._rx_reset()
        self.boot()

    # ---- the link ----------------------------------------------------------------------------------------------------

    def write(self, data: bytes, now_ms: int = 0) -> None:
        for kind, p in self.parser.feed(data):
            if kind == "R":
                self.pc_session = True
            elif kind == "U":
                self._handle(bytes(p))

    def read(self) -> bytes:
        out, self.out = bytes(self.out), bytearray()
        return out

    def close(self) -> None:
        pass

    def reset_board(self) -> bool:
        self.reset("EN pulse")
        return True

    # ---- the board ---------------------------------------------------------------------------------------------------

    @property
    def version(self) -> str:
        return self.slots[self.running][0]

    def _rx_reset(self) -> None:
        self.receiving = self.staged = False
        self.buf = bytearray()
        self.want: dict = {}

    def _event(self, kind: str, **kw) -> None:
        self.events.append({"device": self.name, "event": kind, "version": self.version, **kw})

    def _healthy(self) -> bool:
        return self.slots[self.running][1]

    def boot(self) -> None:
        """Power-on: trial bookkeeping first (as boot_check() in the firmware), then the hello."""
        self.t_boot_s = 0.0
        self.pc_session = False
        self._rx_reset()
        self.running = self.boot_slot
        if self.st == TRIAL and not self.legacy_ota and self.prev == self.running:
            # the probation was recorded but the slot never switched: the old image is still the one running, nothing to try
            self.st, self.tries, self.prev, self.last = NONE, 0, None, "update not completed (power cut before the slot switch)"
            self._event("update_abandoned")
        elif self.st == TRIAL:
            self.tries += 1
            if self.tries >= 2:
                return self._rollback("reset before the new image confirmed itself")
            if not self._healthy():
                return self._rollback("health check failed")
            self.last = f"trial boot {self.tries}"
        self.out += frame_msg("H", f"SafetyNode {self.version} (B) CAN 8MHz restored".encode())
        self._event("boot", state=self.st, tries=self.tries)

    def _rollback(self, why: str) -> None:
        if self.prev is not None:   # the firmware's `if (p) esp_ota_set_boot_partition(p)`: with no recorded slot nothing is switched back
            self.boot_slot = self.prev
        self.st, self.tries, self.prev, self.last = ROLLED_BACK, 0, None, "rolled back: " + why
        self._event("rollback", reason=why)
        self.boot()

    def reset(self, why: str = "power loss") -> None:
        self._event("reset", reason=why)
        self.boot()

    def advance(self, seconds: float) -> None:
        """Time passes while the board runs: a trial image that stays healthy for 3 s commits itself."""
        before = self.t_boot_s
        self.t_boot_s += seconds
        if self.st == TRIAL and before < 3.0 <= self.t_boot_s:
            if not self._healthy():
                return self._rollback("health check failed")
            self.st, self.tries, self.prev, self.last = NONE, 0, None, f"committed {self.version}"
            self._event("commit")

    # ---- the OTA protocol (mirrors ssc_ota.h) -----------------------------------------------------------------------------

    def _persist(self, what: str, spare: int) -> None:
        if what == "prev":
            self.prev = self.running
        elif what == "tries":
            self.tries = 0
        elif what == "last":
            self.last = f"staged {self.want['ver']}"
        elif what == "st":
            self.st = TRIAL
        else:
            self.boot_slot = spare

    def _reply(self, op: int, status: int, extra: bytes = b"") -> None:
        self.out += frame_msg("u", bytes([op, status]) + extra)

    def _handle(self, p: bytes) -> None:
        if not p:
            return self._reply(0, BAD_MESSAGE)
        op = p[0]
        if op == 1:
            if len(p) < 70:
                return self._reply(1, BAD_MESSAGE)
            if self.pc_session:
                return self._reply(1, BUSY)
            self._rx_reset()
            size, sha, mac, vl = struct.unpack("<I", p[1:5])[0], p[5:37], p[37:69], p[69]
            ver = p[70:70 + vl].decode()
            if vl > 16 or len(p) < 70 + vl:
                return self._reply(1, BAD_MESSAGE)
            if size < 1024 or size > 0x140000:
                return self._reply(1, BAD_SIZE)
            if minor_of(ver) <= minor_of(self.version):
                return self._reply(1, DOWNGRADE)
            self.receiving, self.want = True, {"size": size, "sha": sha, "mac": mac, "ver": ver}
            return self._reply(1, OK, struct.pack("<I", 0))
        if op == 2:
            if not self.receiving or len(p) < 6:
                return self._reply(2, BAD_STATE)
            off, data, w = struct.unpack("<I", p[1:5])[0], p[5:], len(self.buf)
            if off < w:
                return self._reply(2, OK, struct.pack("<I", w))
            if off > w:
                return self._reply(2, BAD_OFFSET, struct.pack("<I", w))
            if w + len(data) > self.want["size"]:
                return self._reply(2, BAD_SIZE, struct.pack("<I", w))
            self.buf += data
            return self._reply(2, OK, struct.pack("<I", len(self.buf)))
        if op == 3:
            if not self.receiving:
                return self._reply(3, BAD_STATE)
            if len(self.buf) != self.want["size"]:
                return self._reply(3, BAD_SIZE)
            sha = hashlib.sha256(bytes(self.buf)).digest()
            if not hmac.compare_digest(sha, self.want["sha"]):
                self._rx_reset()
                return self._reply(3, HASH_MISMATCH)
            mac = hmac.new(self.key, self.want["sha"] + struct.pack("<I", self.want["size"]) + self.want["ver"].encode(), hashlib.sha256).digest()
            if not hmac.compare_digest(mac, self.want["mac"]):
                self._rx_reset()
                return self._reply(3, BAD_SIGNATURE)
            if not bytes(self.buf).startswith(b"EMU|"):
                self._rx_reset()
                return self._reply(3, END_FAIL)
            spare = 1 - self.running
            healthy = bytes(self.buf).split(b"|", 3)[2] == b"OK"
            self.slots[spare] = (self.want["ver"], healthy)
            order = END_WRITES_LEGACY if self.legacy_ota else END_WRITES
            for n, what in enumerate(order, 1):
                self._persist(what, spare)
                if self.cut_end_after == n:
                    self.cut_end_after = None
                    self._event("power_cut", after_write=n, write=what)
                    self.boot()
                    return None   # the PC never gets an answer
            self.receiving, self.staged = False, True
            self._event("staged", staged=self.want["ver"])
            return self._reply(3, OK)
        if op == 4:
            if not self.staged:
                return self._reply(4, NOT_STAGED)
            self._reply(4, OK)
            self.staged = False
            return self.boot()
        if op == 5:
            return self._reply(5, OK, bytes([self.st, self.tries]) + struct.pack("<I", SLOT_ADDR[self.running]) + self.last.encode()[:40])
        return self._reply(op, BAD_MESSAGE)
