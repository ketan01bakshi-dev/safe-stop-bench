"""P5 stage 1: the wire format of the chassis link (one UDP datagram = one message).

    frame = 'S5' | version(1) | type(1) | body | crc16(2)          all integers big-endian, crc = CRC-16/CCITT-FALSE over everything before it

    HELLO     chassis -> bridge   boot_id u32, build 8s, state u8, reason u8, reject u8, last_run_id u32
    ARM       bridge  -> chassis  boot_id u32, run_id u32, nonce u32, pwm_cap u8
    SETPOINT  bridge  -> chassis  run_id u32, nonce u32, seq u32, t_ms u32, pwm_l i16, pwm_r i16
    STOP      bridge  -> chassis  run_id u32, nonce u32, reason u8
    ECHO      chassis -> bridge   boot_id u32, run_id u32, nonce u32, state u8, reason u8, reject u8, seq u32, pwm_l i16, pwm_r i16,
                                  uptime_ms u32, rx_ok u16, rx_bad u16, enc_t_ms u32, enc_mask u8, enc_fl u32, enc_fr u32, enc_rl u32, enc_rr u32

Version 2 (v2.38) added the wheel-encoder fields to ECHO: cumulative rising edges of the four slot sensors (20 per wheel turn, wrapping at 2**32),
the chassis uptime when they were sampled, and a mask of the sensors fitted (bit 0 FL, 1 FR, 2 RL, 3 RR; a wheel outside the mask reads 0).
The counts are a witness for calibration, never an input to a verdict. A version-1 peer rejects every version-2 frame (and the other way round).

The run id and nonce are a guard against STALE packets and a confused bridge, not authentication: nothing here is secret or signed, so the
chassis network must stay private. The contract (states, timings, failure table) is docs/P5_LINK_CONTRACT.md; vehicle/link_vectors.json holds
golden frames that a firmware port must reproduce byte for byte (tests/test_p5_link.py keeps the file and this module in step).
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

MAGIC = b"S5"
VERSION = 2
HEADER = struct.Struct(">2sBB")
CRC_LEN = 2


class LinkError(ValueError):
    """A datagram that is not a valid frame (length, magic, version, crc, type or body size)."""


class State(IntEnum):
    DISARMED = 0
    ARMED = 1
    LATCHED_STOP = 2


class Reason(IntEnum):
    """Why the chassis is in LATCHED_STOP (0 = not latched)."""
    NONE = 0
    DEADMAN = 1          # no valid SETPOINT for more than the dead-man time
    BRIDGE_ABORT = 2     # the bridge sent STOP(ABORT)
    BRIDGE_WATCHDOG = 3  # the bridge's own watchdog fired and it sent STOP(WATCHDOG)
    BAD_NONCE = 4        # a frame from the armed peer carried the wrong nonce
    BAD_RUN = 5          # a frame from the armed peer carried the wrong run id (an old run)
    CMD_RANGE = 6        # a SETPOINT outside 0..pwm_cap
    LINK_LOST = 7        # the Wi-Fi association dropped


class StopReason(IntEnum):
    END_OF_RUN = 0       # planned end: back to DISARMED, no latch
    ABORT = 1            # the bridge gives up: latch
    WATCHDOG = 2         # the bridge's watchdog: latch


class Reject(IntEnum):
    """Why the chassis refused the last ARM (0 = none). Carried in HELLO and ECHO for diagnosis."""
    NONE = 0
    BOOT_ID = 1          # the ARM names another boot of the chassis (a replayed ARM)
    RUN_OLD = 2          # run id not newer than the last accepted one
    CAP = 3              # pwm_cap is 0 or above the chassis hard cap
    LATCHED = 4          # the chassis is latched: only the physical re-arm releases it
    OTHER_PEER = 5       # a second bridge tried to arm an armed chassis


ENC_WHEELS = ("fl", "fr", "rl", "rr")
ENC_ALL = 0b1111

T_HELLO, T_ARM, T_SETPOINT, T_STOP, T_ECHO = 1, 2, 3, 4, 5

_BODY = {
    T_HELLO: struct.Struct(">I8sBBBI"),
    T_ARM: struct.Struct(">IIIB"),
    T_SETPOINT: struct.Struct(">IIIIhh"),
    T_STOP: struct.Struct(">IIB"),
    T_ECHO: struct.Struct(">IIIBBBIhhIHHIBIIII"),
}


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final xor); check value of b'123456789' is 0x29B1."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _build_bytes(build: str) -> bytes:
    raw = build.encode("ascii")
    if not 0 < len(raw) <= 8:
        raise LinkError(f"build id must be 1..8 ASCII characters, got {build!r}")
    return raw.ljust(8, b"\0")


@dataclass(frozen=True)
class Hello:
    boot_id: int
    build: str
    state: int
    reason: int = 0
    reject: int = 0
    last_run_id: int = 0
    TYPE = T_HELLO


@dataclass(frozen=True)
class Arm:
    boot_id: int
    run_id: int
    nonce: int
    pwm_cap: int
    TYPE = T_ARM


@dataclass(frozen=True)
class Setpoint:
    run_id: int
    nonce: int
    seq: int
    t_ms: int
    pwm_l: int
    pwm_r: int
    TYPE = T_SETPOINT


@dataclass(frozen=True)
class Stop:
    run_id: int
    nonce: int
    reason: int
    TYPE = T_STOP


@dataclass(frozen=True)
class Echo:
    boot_id: int
    run_id: int
    nonce: int
    state: int
    reason: int
    reject: int
    seq: int
    pwm_l: int
    pwm_r: int
    uptime_ms: int
    rx_ok: int
    rx_bad: int
    enc_t_ms: int = 0        # chassis uptime when the counts below were sampled
    enc_mask: int = 0        # sensors fitted: bit 0 FL, 1 FR, 2 RL, 3 RR (0 = no encoders)
    enc_fl: int = 0          # cumulative rising edges, 20 per wheel turn, wrapping at 2**32
    enc_fr: int = 0
    enc_rl: int = 0
    enc_rr: int = 0
    TYPE = T_ECHO


Message = Hello | Arm | Setpoint | Stop | Echo
_CLASS = {T_HELLO: Hello, T_ARM: Arm, T_SETPOINT: Setpoint, T_STOP: Stop, T_ECHO: Echo}


def encode(msg: Message) -> bytes:
    try:
        if isinstance(msg, Hello):
            body = _BODY[T_HELLO].pack(msg.boot_id, _build_bytes(msg.build), msg.state, msg.reason, msg.reject, msg.last_run_id)
        else:
            fields = [getattr(msg, f) for f in msg.__dataclass_fields__]
            body = _BODY[msg.TYPE].pack(*fields)
    except struct.error as e:
        raise LinkError(f"{type(msg).__name__}: field out of range ({e})") from e
    frame = HEADER.pack(MAGIC, VERSION, msg.TYPE) + body
    return frame + struct.pack(">H", crc16(frame))


def decode(data: bytes) -> Message:
    if len(data) < HEADER.size + CRC_LEN:
        raise LinkError("frame too short")
    magic, version, mtype = HEADER.unpack_from(data)
    if magic != MAGIC:
        raise LinkError("bad magic")
    if version != VERSION:
        raise LinkError(f"unsupported version {version}")
    if struct.unpack(">H", data[-CRC_LEN:])[0] != crc16(data[:-CRC_LEN]):
        raise LinkError("bad crc")
    if mtype not in _BODY:
        raise LinkError(f"unknown type {mtype}")
    body = data[HEADER.size:-CRC_LEN]
    if len(body) != _BODY[mtype].size:
        raise LinkError(f"type {mtype}: body is {len(body)} bytes, expected {_BODY[mtype].size}")
    vals = list(_BODY[mtype].unpack(body))
    if mtype == T_HELLO:
        vals[1] = vals[1].rstrip(b"\0").decode("ascii", "replace")
    return _CLASS[mtype](*vals)


def describe(msg: Message) -> dict:
    """The message as a flat dict for logs: {'type': 'SETPOINT', ...fields}."""
    d = {f: getattr(msg, f) for f in msg.__dataclass_fields__}
    d["type"] = type(msg).__name__.upper()
    return d


# --- golden frames: what a firmware port must reproduce byte for byte -------------------------------------------------------------

def golden_vectors() -> list[dict]:
    msgs: list[tuple[str, Message]] = [
        ("hello disarmed", Hello(boot_id=0x0A0B0C0D, build="p5-s1", state=State.DISARMED, last_run_id=41)),
        ("hello latched by dead-man, last arm refused", Hello(0x0A0B0C0D, "p5-s1", State.LATCHED_STOP, Reason.DEADMAN, Reject.LATCHED, 42)),
        ("arm", Arm(boot_id=0x0A0B0C0D, run_id=42, nonce=0xDEADBEEF, pwm_cap=210)),
        ("setpoint at 20 km/h plan value", Setpoint(run_id=42, nonce=0xDEADBEEF, seq=1, t_ms=0, pwm_l=170, pwm_r=170)),
        ("setpoint at the largest field values", Setpoint(0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 255, 255)),
        ("stop end of run", Stop(run_id=42, nonce=0xDEADBEEF, reason=StopReason.END_OF_RUN)),
        ("stop abort", Stop(run_id=42, nonce=0xDEADBEEF, reason=StopReason.ABORT)),
        ("echo armed, no encoders fitted", Echo(0x0A0B0C0D, 42, 0xDEADBEEF, State.ARMED, Reason.NONE, Reject.NONE, 17, 170, 170, 123456, 20, 1)),
        ("echo armed, four encoders", Echo(0x0A0B0C0D, 42, 0xDEADBEEF, State.ARMED, Reason.NONE, Reject.NONE, 17, 170, 170, 123456, 20, 1,
                                           123450, ENC_ALL, 401, 398, 405, 0xFFFFFFFF)),
        ("echo latched, two encoders (front pair)", Echo(0x0A0B0C0D, 42, 0xDEADBEEF, State.LATCHED_STOP, Reason.DEADMAN, Reject.NONE, 90, 0, 0,
                                                         200000, 90, 0, 199990, 0b0011, 1500, 1497, 0, 0)),
    ]
    return [{"name": n, "message": describe(m), "hex": encode(m).hex()} for n, m in msgs]


VECTORS = "link_vectors.json"


if __name__ == "__main__":                                   # python -m vehicle.link  rewrites vehicle/link_vectors.json
    import json
    from pathlib import Path
    path = Path(__file__).with_name(VECTORS)
    path.write_text(json.dumps({"about": "Golden frames of the P5 link, from vehicle/link.py; a firmware port must produce these bytes.",
                                "vectors": golden_vectors()}, indent=1) + "\n", encoding="utf-8")
    print("wrote", path)
