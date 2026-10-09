"""The PC side of the OTA update (P4): push a signed image to the board over the bench's USB link, with injectable faults.

Stop-and-wait: one 'U' CHUNK, one 'u' ack carrying the bytes the board has written; a lost or repeated chunk is harmless (the board
answers with its own offset and the client resumes there). Faults the tests inject: a corrupted byte (the image hash must fail), a
dropped chunk (resume), a tampered image or a wrong signing key (the signature must fail), and a board reset in the middle of the
download (the old image must keep running, and a retry must succeed).
"""
from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field

from ssb.hil import Parser, frame_msg

from .image import SignedImage

BEGIN, CHUNK, END, REBOOT, STATUS = 1, 2, 3, 4, 5
STATUS_NAMES = {0: "OK", 1: "BUSY", 2: "BAD_STATE", 3: "BAD_SIZE", 4: "NO_SPACE", 5: "BAD_OFFSET", 6: "WRITE_FAIL", 7: "HASH_MISMATCH",
                8: "BAD_SIGNATURE", 9: "END_FAIL", 10: "NOT_STAGED", 11: "BAD_MESSAGE", 12: "DOWNGRADE"}
STATE_NAMES = {0: "NONE", 1: "TRIAL", 2: "ROLLED_BACK"}
CHUNK_BYTES = 200


class OtaError(RuntimeError):
    pass


@dataclass
class Faults:
    corrupt_at: int | None = None        # flip a byte of the data sent at this offset (the board's SHA-256 must catch it)
    drop_chunks: list[int] = field(default_factory=list)   # offsets whose first transmission is "lost"
    reset_at_pct: float | None = None    # reset the board (EN pulse) when this share of the image has been sent
    resend_every: int = 0                # send every Nth chunk twice (idempotency)


@dataclass
class PushResult:
    outcome: str                         # staged | rejected | reset | lost (no answer to END: the board went down in its final writes)
    status: str = "OK"
    bytes_acked: int = 0
    chunks: int = 0
    resumes: int = 0
    seconds: float = 0.0


class OtaClient:
    def __init__(self, link, timeout_s: float = 2.0, end_timeout_s: float = 10.0):
        self.link, self.timeout_s, self.end_timeout_s, self.parser = link, timeout_s, end_timeout_s, Parser()
        self.hello: str | None = None

    def _send(self, op: int, payload: bytes = b"") -> None:
        self.link.write(frame_msg("U", bytes([op]) + payload), 0)

    def _recv(self, op: int, timeout_s: float | None = None) -> tuple[int, bytes]:
        end = time.time() + (timeout_s or self.timeout_s)
        while time.time() < end:
            found = None
            for kind, p in self.parser.feed(self.link.read()):   # take every message of this read: a hello can follow the answer
                if kind == "H":
                    self.hello = p.decode(errors="replace")
                elif kind == "u" and len(p) >= 2 and p[0] == op and found is None:
                    found = (p[1], bytes(p[2:]))
            if found:
                return found
            time.sleep(0.002)
        raise OtaError(f"no answer from the board to OTA op {op} within {timeout_s or self.timeout_s} s")

    def _call(self, op: int, payload: bytes = b"", timeout_s: float | None = None) -> tuple[str, bytes]:
        self._send(op, payload)
        status, extra = self._recv(op, timeout_s)
        return STATUS_NAMES.get(status, str(status)), extra

    # ---- the update ---------------------------------------------------------------------------------------------------

    def push(self, img: SignedImage, faults: Faults | None = None) -> PushResult:
        f, t0 = faults or Faults(), time.time()
        ver = img.version.encode()
        status, extra = self._call(BEGIN, struct.pack("<I", img.size) + img.sha256 + img.hmac + bytes([len(ver)]) + ver)
        if status != "OK":
            return PushResult("rejected", status, seconds=time.time() - t0)
        written, chunks, resumes = struct.unpack("<I", extra[:4])[0], 0, 0
        dropped = set(f.drop_chunks)
        reset_at = None if f.reset_at_pct is None else int(img.size * f.reset_at_pct / 100)
        while written < img.size:
            if reset_at is not None and written >= reset_at:
                if hasattr(self.link, "reset_board"):
                    self.link.reset_board()
                return PushResult("reset", "RESET", written, chunks, resumes, time.time() - t0)
            data = bytearray(img.data[written:written + CHUNK_BYTES])
            if f.corrupt_at is not None and written <= f.corrupt_at < written + len(data):
                data[f.corrupt_at - written] ^= 0xFF
            msg = struct.pack("<I", written) + bytes(data)
            if written in dropped:   # the chunk is lost on the way: nothing is sent, the next chunk arrives out of order
                dropped.discard(written)
                msg = struct.pack("<I", written + len(data)) + bytes(img.data[written + len(data):written + 2 * len(data)])
            status, extra = self._call(CHUNK, msg)
            chunks += 1
            if status == "BAD_OFFSET":
                resumes += 1
                written = struct.unpack("<I", extra[:4])[0]   # resume where the board is
                continue
            if status != "OK":
                return PushResult("rejected", status, written, chunks, resumes, time.time() - t0)
            written = struct.unpack("<I", extra[:4])[0]
            if f.resend_every and chunks % f.resend_every == 0:
                self._call(CHUNK, msg)   # the same chunk again: must not corrupt the image
        try:
            status, _ = self._call(END, timeout_s=self.end_timeout_s)   # hash + HMAC + the image header check + the slot switch
        except OtaError:   # the board stopped answering during its final writes: a power cut or a crash; the caller looks at what it boots
            return PushResult("lost", "NO_ANSWER", written, chunks, resumes, time.time() - t0)
        return PushResult("staged" if status == "OK" else "rejected", status, written, chunks, resumes, time.time() - t0)

    def reboot(self) -> str:
        self.hello = None
        status, _ = self._call(REBOOT)
        return status

    def forget_hello(self) -> None:
        self.hello = None
        self.parser.feed(self.link.read())

    def status(self) -> dict:
        status, extra = self._call(STATUS)
        if status != "OK" or len(extra) < 6:
            raise OtaError(f"STATUS failed: {status}")
        return {"state": STATE_NAMES.get(extra[0], extra[0]), "tries": extra[1], "running_slot_addr": hex(struct.unpack("<I", extra[2:6])[0]),
                "last": extra[6:].decode(errors="replace")}

    def wait_for_hello(self, timeout_s: float = 8.0) -> str | None:
        """The board's next hello. A hello already seen since reboot() / forget_hello() counts (it can arrive with the reboot's own answer)."""
        end = time.time() + timeout_s
        while time.time() < end and not self.hello:
            for kind, p in self.parser.feed(self.link.read()):
                if kind == "H":
                    self.hello = p.decode(errors="replace")
            time.sleep(0.01)
        return self.hello
