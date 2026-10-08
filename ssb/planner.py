"""The doer: a stand-in for the RL planner and its comms, driven by a scenario's fault list.

Sends a Profile 5-protected CAN FD command every 20 ms and kicks the hardware watchdog line.
Each command carries: timestamp, accel, road-wheel angle, speed request, the answer to the safety
controller's watchdog challenge, perception health and an ODD-exit flag.
"""
from __future__ import annotations

import csv
import math
import struct

from . import e2e

CMD_ID = 0x100
PERIOD_MS = 20
STRUCT = "<HhhHBBB"   # t_stamp, accel cm/s², steer 0.01 deg, speed_req cm/s, qa_resp, perception, flags


def qa_answer(challenge: int) -> int:
    return (challenge * 31 + 7) & 0xFF


def encode_cmd(t_stamp: int, accel: float, steer: float, speed_req: float, qa: int, perc: int, flags: int) -> bytes:
    return struct.pack(STRUCT, t_stamp & 0xFFFF, round(accel * 100), round(steer * 100),
                       max(0, min(65535, round(speed_req * 100))), qa & 0xFF, perc, flags)


def decode_cmd(payload: bytes) -> dict:
    ts, a, s, v, qa, perc, flags = struct.unpack(STRUCT, payload)
    return {"t_stamp": ts, "accel": a / 100, "steer": s / 100, "speed_req": v / 100, "qa": qa,
            "perception": perc, "odd_exit": bool(flags & 1)}


def active(f: dict, t: int) -> bool:
    end = f.get("end")
    if t < f.get("start", 0) or (end is not None and t >= end):
        return False
    if "off_ms" in f:  # intermittent: off for off_ms, on for on_ms, repeating
        return (t - f["start"]) % (f["off_ms"] + f["on_ms"]) < f["off_ms"]
    return True


class Planner:
    def __init__(self, sc: dict, cfg: dict, rng, data_id: int):
        self.sc, self.cfg, self.rng, self.data_id = sc, cfg, rng, data_id
        self.faults = sc.get("faults", [])
        self.counter = 0
        self.last_frame: bytes | None = None
        self.held: bytes | None = None
        self.v_target = sc.get("target_kmh", sc.get("start_kmh", 30)) / 3.6
        self.steer_ramp = 0.0
        self.frozen_qa: int | None = None
        self.done_once: set[int] = set()
        self.log = None
        if sc.get("replay_log"):
            with open(sc["replay_log"], newline="") as fh:
                self.log = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(fh)]

    def f(self, kind: str, t: int):
        for f in self.faults:
            if f["type"] == kind and active(f, t):
                return f
        return None

    def once(self, kind: str, t: int):
        for i, f in enumerate(self.faults):
            if f["type"] == kind and t >= f["start"] and i not in self.done_once:
                self.done_once.add(i)
                return f
        return None

    def _nominal(self, t: int, veh, mrm_request: str | None) -> tuple[float, float, float, int, int]:
        if self.log:
            row = self.log[min(len(self.log) - 1, t // PERIOD_MS)]
            return row["accel"], row["steer_deg"], row["speed_req_kmh"] / 3.6, int(row["perception"]), 0
        if mrm_request == "PULL_OVER":
            steer = math.degrees(-(0.06 * (veh.y - 3.0) + 0.9 * veh.psi))
            return -1.0 if veh.v > 0 else 0.0, max(-6, min(6, steer)), 0.0, 2, 0
        steer = math.degrees(-(0.05 * veh.y + 0.8 * veh.psi)) + 1.5 * math.sin(2 * math.pi * t / 4000)
        err = self.v_target - veh.v
        self.i_err = max(-3.0, min(3.0, getattr(self, "i_err", 0.0) + err * PERIOD_MS / 1000))
        accel = max(-2.0, min(1.5, 0.8 * err + 0.6 * self.i_err))   # PI: holds speed on a grade
        return accel, steer, self.v_target, 2, 0

    def step(self, t: int, veh, challenge: int, mrm_request: str | None) -> tuple[list[bytes], list[int]]:
        kicks = []
        reset = self.f("planner_reset", t)
        hang = self.f("hang_comms_alive", t)
        if self.f("kick_fast", t):
            if t % 2 == 0:
                kicks.append(t)
        elif t % PERIOD_MS == 0 and not reset and not hang:
            kicks.append(t)
        if t % PERIOD_MS != 0:
            return [], kicks
        if reset:
            self.counter = 0
            return [], kicks
        if self.f("counter_frozen", t) and self.last_frame is not None:
            return [self.last_frame], kicks

        accel, steer, speed_req, perc, flags = self._nominal(t, veh, mrm_request)
        if (m := self.f("lane_change", t)):
            steer = m.get("steer_deg", 5.0)
        if (m := self.f("brake_event", t)):
            accel = m.get("accel", -2.0)
        if (m := self.f("steer_rate", t)):
            self.steer_ramp = max(-45.0, min(45.0, self.steer_ramp + m["dps"] * PERIOD_MS / 1000))
            steer = self.steer_ramp
        elif self.log:
            self.steer_ramp = steer   # replay: send exactly what was recorded
        else:
            # a well-behaved planner slews its steering request (12.5 deg/s), inside every envelope
            step = 12.5 * PERIOD_MS / 1000
            steer = self.steer_ramp + max(-step, min(step, steer - self.steer_ramp))
            self.steer_ramp = steer
        if (m := self.f("speed_req", t)):
            self.v_target = m["kmh"] / 3.6
            speed_req = self.v_target
            accel = max(-2.0, min(1.5, 0.8 * (self.v_target - veh.v)))
        if (m := self.f("accel_req", t)):
            accel = m["accel"]
        if (m := self.f("perception", t)):
            perc = m["health"]
        if self.f("odd_exit", t):
            flags |= 1

        qa = qa_answer(challenge)
        if hang:
            if self.frozen_qa is None:
                self.frozen_qa = qa
            qa = self.frozen_qa
        if self.f("qa_wrong", t):
            qa ^= 0x55
        t_stamp = t
        if (m := self.f("stale_timestamp", t)):
            t_stamp = t - m["age_ms"]
        if (m := self.f("clock_drift", t)):
            t_stamp = t - int((t - m["start"]) * m["pct"] / 100)

        if (m := self.once("counter_jump", t)):
            self.counter = (self.counter + m["n"] - 1) % 256
        data_id = 0x4321 if self.f("wrong_data_id", t) else self.data_id
        frame = e2e.Profile5.protect(encode_cmd(t_stamp, accel, steer, speed_req, qa, perc, flags), self.counter, data_id)
        self.counter = (self.counter + 1) % 256
        if self.f("crc_corrupt", t) or (self.once("single_crc", t)):
            frame = e2e.flip_crc(frame)
        if (m := self.f("noise", t)) and self.rng.random() < m["rate"]:
            frame = e2e.flip_crc(frame)
        self.last_frame = frame
        if self.f("link_lost", t):
            return [], kicks   # frame built (counter advanced) but lost in transit
        if self.once("reorder", t):
            self.held = frame
            return [], kicks
        if self.held is not None:
            out, self.held = [frame, self.held], None
            return out, kicks
        return [frame], kicks
