"""Attacks on the planner bus (P3): an extra node that sniffs the bus and injects frames, at every bench level.

A scenario lists an attack as a fault: {"type": "attack", "kind": ..., "start": ms, "end": ms, ...parameters}. The runner calls
`Attacker.observe()` with what the legitimate planner sent (the attacker sniffs the bus: it sees every frame and the safety
controller's watchdog challenge), then `Attacker.frames()` for what to inject, and `Attacker.suppress()` to know whether the legitimate
frames are removed from the bus (a man-in-the-middle / compromised gateway: `takeover`).

Kinds (all aimed at the IDs in config/can_ids.json):
  flood         high- or low-priority junk at a high rate (id, per_ms)          bus starvation
  fuzzy         random IDs / lengths / bytes at a fixed rate (period_ms)        protocol robustness
  spoof_invalid forged PLN_Command with a wrong CRC, sent right after the real one
  spoof_valid   forged PLN_Command with a CORRECT CRC, counter, data ID, fresh timestamp and watchdog answer: nothing in E2E can tell it
                from the planner (the CRC is not a secret). inject = race with the real planner; takeover = real frames removed
  replay        old real frames sent again (age_ms back). age_ms 5120 = exactly 256 planner periods: the counter matches again
  period_glitch an extra copy of the last real frame at an odd moment (offset_ms)
  signal_ramp   takeover with valid frames whose steering drifts slowly (dps) inside every limit
"""
from __future__ import annotations

import random

from ssb import e2e
from ssb.planner import CMD_ID, PERIOD_MS, decode_cmd, encode_cmd, qa_answer

KINDS = ("flood", "fuzzy", "spoof_invalid", "spoof_valid", "replay", "period_glitch", "signal_ramp")
KNOWN_IDS = (0x100, 0x101, 0x200, 0x201, 0x300, 0x301)
Frame = tuple[int, bytes, bool]   # (can id, data, CAN FD)


def attacks_of(sc: dict) -> list[dict]:
    return [f for f in sc.get("faults", []) if f["type"] == "attack"]


class Attacker:
    def __init__(self, sc: dict, data_id: int, seed: int = 0):
        self.specs = attacks_of(sc)
        for s in self.specs:
            if s["kind"] not in KINDS:
                raise ValueError(f"unknown attack kind {s['kind']!r}; choose one of {KINDS}")
        self.data_id = data_id
        self.rng = random.Random(seed * 7919 + 13)
        self.history: dict[int, bytes] = {}   # t -> the real planner frame sent then (the attacker sniffs the bus)
        self.last: bytes | None = None          # the last real frame that reached the bus
        self.latest: bytes | None = None        # the real planner's newest output (a man-in-the-middle sees it even when it removes it)
        self.last_t = 0
        self.challenge = 0x5A
        self.counter = 0                       # the counter the attacker will use next (continues the real stream)
        self.n_sent = 0

    @property
    def first_start(self) -> int | None:
        return min((s["start"] for s in self.specs), default=None)

    def _active(self, s: dict, t: int) -> bool:
        return s["start"] <= t < s.get("end", 10**9)

    def suppress(self, t: int) -> bool:
        """True while the real planner's frames are removed from the bus (takeover modes)."""
        return any(self._active(s, t) and (s["kind"] == "signal_ramp" or s.get("mode") == "takeover" and s["kind"] in ("spoof_valid", "replay"))
                   for s in self.specs)

    def observe(self, t: int, legit: list[bytes], challenge: int) -> None:
        self.challenge = challenge
        for fr in legit:
            self.history[t] = fr
            self.latest = fr
            if not self.suppress(t):   # the real counter is only learnt from the real stream
                self.last, self.last_t = fr, t
                self.counter = (fr[2] + 1) % 256

    # ---- frame builders -------------------------------------------------------------------------------------

    def _forged(self, t: int, base: bytes | None, accel: float | None, steer: float | None, speed: float | None) -> bytes | None:
        if base is None:
            return None
        d = decode_cmd(base[e2e.Profile5.HEADER:])
        payload = encode_cmd(t, d["accel"] if accel is None else accel, d["steer"] if steer is None else steer,
                             d["speed_req"] if speed is None else speed, qa_answer(self.challenge), d["perception"],
                             1 if d["odd_exit"] else 0)
        frame = e2e.Profile5.protect(payload, self.counter, self.data_id)
        self.counter = (self.counter + 1) % 256
        return frame

    def frames(self, t: int) -> list[Frame]:
        out: list[Frame] = []
        for s in self.specs:
            if not self._active(s, t):
                continue
            k, rel = s["kind"], t - s["start"]
            on_period = t % PERIOD_MS == 0
            if k == "flood":
                out += [(int(str(s.get("id", "0x001")), 16) if isinstance(s.get("id"), str) else s.get("id", 0x001), bytes(8), False)
                        for _ in range(s.get("per_ms", 2))]
            elif k == "fuzzy":
                if rel % s.get("period_ms", 5) == 0:
                    cid = self.rng.choice(KNOWN_IDS) if self.rng.random() < 0.7 else self.rng.randrange(0x800)
                    n = self.rng.choice((0, 1, 4, 8, 16 if cid == CMD_ID else 8))
                    out.append((cid, bytes(self.rng.randrange(256) for _ in range(n)), n > 8))
            elif k == "spoof_invalid":
                if t % PERIOD_MS == 1 and self.last:
                    f = self._forged(t, self.last, s.get("accel"), s.get("steer"), s.get("speed"))
                    if f:
                        out.append((CMD_ID, e2e.flip_crc(f), True))
            elif k == "spoof_valid":
                if (t % PERIOD_MS == (0 if s.get("mode") == "takeover" else 1)) and self.last:
                    f = self._forged(t, self.latest or self.last, s.get("accel", 1.5), s.get("steer", 25.0), s.get("speed", 20.0))
                    if f:
                        out.append((CMD_ID, f, True))
            elif k == "replay":
                if on_period:
                    old = self.history.get(t - s.get("age_ms", 1000))
                    if old is not None:
                        out.append((CMD_ID, old, True))
            elif k == "period_glitch":
                if t % PERIOD_MS == s.get("offset_ms", 7) and self.last:
                    out.append((CMD_ID, self.last, True))
            elif k == "signal_ramp":
                if on_period and self.last and self.latest:
                    real = decode_cmd(self.latest[e2e.Profile5.HEADER:])["steer"]
                    f = self._forged(t, self.latest, None, max(-45.0, min(45.0, real + s.get("dps", 2.0) * rel / 1000)), None)
                    if f:
                        out.append((CMD_ID, f, True))
        self.n_sent += len(out)
        return out
