"""End-to-end protection in the style of AUTOSAR E2E Profiles 2 and 5, plus a windowed state machine.

Sketch, not a certified library. Byte layouts are fixed here; in a real system they come
from the signal configuration (ARXML). Profile 2 is used on the classic-CAN actuator bus,
Profile 5 on the CAN FD planner bus, mirroring how the two profiles are typically split.
"""
from __future__ import annotations

from collections import deque

OK, OK_SOME_LOST, NO_NEW_DATA = "OK", "OK_SOME_LOST", "NO_NEW_DATA"
WRONG_CRC, REPEATED, WRONG_SEQUENCE = "WRONG_CRC", "REPEATED", "WRONG_SEQUENCE"
VALID = {OK, OK_SOME_LOST}
ERRORS = {WRONG_CRC, REPEATED, WRONG_SEQUENCE}


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF): the Profile 5 CRC."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def crc8_h2f(data: bytes, crc: int = 0xFF) -> int:
    """CRC-8 with polynomial 0x2F (AUTOSAR 'CRC8H2F'), init 0xFF, final XOR 0xFF: the Profile 2 CRC."""
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x2F) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc ^ 0xFF


class _Receiver:
    counter_mod = 256

    def __init__(self, max_delta: int, explain_gaps: bool = False):
        self.max_delta = max_delta
        self.last: int | None = None
        # v2.6 option: a counter jump fully explained by the CRC failures just before it is not a second error
        # (two corrupted frames + the next good one would otherwise count as three errors for one noise burst)
        self.explain_gaps = explain_gaps
        self.crc_fails = 0

    def _sequence(self, counter: int) -> str:
        explained, self.crc_fails = self.crc_fails, 0
        if self.last is None:
            self.last = counter
            return OK
        delta = (counter - self.last) % self.counter_mod
        if delta == 0:
            return REPEATED
        self.last = counter
        if delta == 1:
            return OK
        limit = self.max_delta + (explained if self.explain_gaps else 0)
        return OK_SOME_LOST if delta <= limit else WRONG_SEQUENCE


class Profile5:
    """CRC-16 + 8-bit counter + 16-bit data ID (inside the CRC, never sent). Frame: CRC(2, LE) | counter | payload."""
    HEADER = 3

    @staticmethod
    def protect(payload: bytes, counter: int, data_id: int) -> bytes:
        crc = crc16_ccitt(bytes([counter & 0xFF]) + payload + data_id.to_bytes(2, "little"))
        return crc.to_bytes(2, "little") + bytes([counter & 0xFF]) + payload

    class Receiver(_Receiver):
        counter_mod = 256

        def __init__(self, data_id: int, max_delta: int = 2, explain_gaps: bool = False):
            super().__init__(max_delta, explain_gaps)
            self.data_id = data_id

        def check(self, frame: bytes | None) -> tuple[str, bytes | None]:
            if frame is None:
                return NO_NEW_DATA, None
            if len(frame) < 3:
                self.crc_fails += 1
                return WRONG_CRC, None
            counter, payload = frame[2], frame[3:]
            if crc16_ccitt(bytes([counter]) + payload + self.data_id.to_bytes(2, "little")) != int.from_bytes(frame[:2], "little"):
                self.crc_fails += 1
                return WRONG_CRC, None
            status = self._sequence(counter)
            return status, (payload if status in VALID else None)


class Profile2:
    """CRC-8 (0x2F) + 4-bit counter; the data ID used in the CRC is picked from a list of 16 by the counter.
    Frame: CRC | counter (low nibble) | payload."""
    HEADER = 2
    DEFAULT_ID_LIST = bytes(range(0x10, 0x20))

    @staticmethod
    def protect(payload: bytes, counter: int, id_list: bytes = DEFAULT_ID_LIST) -> bytes:
        c = counter & 0x0F
        crc = crc8_h2f(bytes([c]) + payload + bytes([id_list[c]]))
        return bytes([crc, c]) + payload

    class Receiver(_Receiver):
        counter_mod = 16

        def __init__(self, id_list: bytes | None = None, max_delta: int = 2):
            super().__init__(max_delta)
            self.id_list = id_list or Profile2.DEFAULT_ID_LIST

        def check(self, frame: bytes | None) -> tuple[str, bytes | None]:
            if frame is None:
                return NO_NEW_DATA, None
            if len(frame) < 2:
                return WRONG_CRC, None
            c, payload = frame[1] & 0x0F, frame[2:]
            if crc8_h2f(bytes([c]) + payload + bytes([self.id_list[c]])) != frame[0]:
                return WRONG_CRC, None
            status = self._sequence(c)
            return status, (payload if status in VALID else None)


def flip_crc(frame: bytes) -> bytes:
    """Fault injection: flip one CRC bit."""
    return bytes([frame[0] ^ 0x01]) + frame[1:]


class E2EStateMachine:
    """Windowed state machine in the spirit of AUTOSAR E2E_SM: NODATA → INIT → VALID ⇄ INVALID.

    Judges the last `window` received frames, so scattered errors add up (a run-length count
    such as '3 bad in a row' never sees errors separated by good frames).
    """

    def __init__(self, window: int = 6, min_ok_init: int = 2, max_err_init: int = 1,
                 min_ok_valid: int = 3, max_err_valid: int = 2, min_ok_invalid: int = 4, max_err_invalid: int = 0):
        self.window: deque[str] = deque(maxlen=window)
        self.min_ok_init, self.max_err_init = min_ok_init, max_err_init
        self.min_ok_valid, self.max_err_valid = min_ok_valid, max_err_valid
        self.min_ok_invalid, self.max_err_invalid = min_ok_invalid, max_err_invalid
        self.state = "NODATA"

    def preset_valid(self) -> None:
        self.window.extend([OK] * (self.window.maxlen or 0))
        self.state = "VALID"

    def update(self, status: str) -> str:
        if status == NO_NEW_DATA:
            return self.state  # missing data is the timeout's job
        self.window.append(status)
        ok = sum(s in VALID for s in self.window)
        err = sum(s in ERRORS for s in self.window)
        if self.state == "NODATA":
            self.state = "INIT"
        if self.state == "INIT":
            if ok >= self.min_ok_init and err <= self.max_err_init:
                self.state = "VALID"
            elif err > self.max_err_init:
                self.state = "INVALID"
        elif self.state == "VALID":
            if err > self.max_err_valid:
                self.state = "INVALID"
        elif self.state == "INVALID":
            recent = list(self.window)[-self.min_ok_invalid:]
            if len(recent) == self.min_ok_invalid and all(s in VALID for s in recent):
                self.state = "VALID"
        return self.state


# ---- SAF_Status (0x201) ------------------------------------------------------------------------------------------------
# One layout at every level since v2.10 (it was the C++ core's alone in v2.9.6; the CAN-process vECU's DBC frame had no
# protection): CRC-8 | alive counter | state:3 mrm:1 cause:4 | challenge | accel out (int16, 0.01) | steer out (int16, 0.01).
# The CRC (CRC-8 0x2F, as Profile 2) covers bytes 1-7 plus the data ID 0x201 (LE). Found on hardware: without a counter
# a stale copy of an old status could not be rejected (v2.9.3-v2.9.6).
STATUS_ID = 0x201


def _i16(x: float) -> int:
    return max(-32768, min(32767, round(x * 100)))   # round(): half to even, as nearbyint() in the C++ core


def status_protect(counter: int, state: int, mrm: bool, cause: int, challenge: int, accel: float, steer: float) -> bytes:
    if not (0 <= state < 8 and 0 <= cause < 16):
        raise ValueError(f"SAF_Status packs the state in 3 bits and the cause in 4: state={state}, cause={cause}")
    body = (bytes([counter & 0xFF, state | (0x08 if mrm else 0) | cause << 4, challenge & 0xFF])
            + _i16(max(-327.0, min(327.0, accel))).to_bytes(2, "little", signed=True)
            + _i16(max(-327.0, min(327.0, steer))).to_bytes(2, "little", signed=True))
    return bytes([crc8_h2f(body + STATUS_ID.to_bytes(2, "little"))]) + body


def status_unpack(data: bytes) -> dict:
    """The fields of an 8-byte SAF_Status, unchecked (run StatusReceiver.check first)."""
    p = data[2]
    return {"counter": data[1], "state": p & 0x07, "mrm": bool(p & 0x08), "cause": p >> 4, "challenge": data[3],
            "accel": int.from_bytes(data[4:6], "little", signed=True) / 100,
            "steer": int.from_bytes(data[6:8], "little", signed=True) / 100}


class StatusReceiver:
    """SAF_Status E2E check (HiL v2.9.6; shared with the CAN-process bench in v2.10). CRC first, then an 8-bit alive
    counter that must advance by 1..MAX_DELTA (one lost frame tolerated). A repeat or a jump is rejected; after a jump
    (e.g. the DUT reset and restarted its counter) two consecutive frames in sequence resynchronise, or at once if the
    last accepted frame said OFF (the DUT announced it was going down, so a restart is expected). A stale copy of an
    old status passes only if its counter happens to fit: about 2 in 256, against always without the counter."""
    MAX_DELTA = 2
    OFF = 7

    def __init__(self):
        self.n_crc = self.n_seq = 0
        self.reset()

    def reset(self) -> None:
        """The sender's counter restarts (the bench reset it): accept the next valid frame as the first."""
        self.last: int | None = None
        self.cand: int | None = None
        self.last_off = False

    def check(self, data: bytes) -> bool:
        if len(data) != 8 or crc8_h2f(bytes(data[1:8]) + STATUS_ID.to_bytes(2, "little")) != data[0]:
            self.n_crc += 1
            return False
        c = data[1]
        ok = (self.last is None                                         # first frame
              or 1 <= (c - self.last) % 256 <= self.MAX_DELTA            # in sequence
              or self.last_off                                          # the DUT said OFF: a restart is expected
              or (self.cand is not None and (c - self.cand) % 256 == 1))   # two in a row: resync
        if not ok:
            self.cand = c
            self.n_seq += 1
            return False
        self.last, self.cand, self.last_off = c, None, (data[2] & 0x07) == self.OFF
        return True
