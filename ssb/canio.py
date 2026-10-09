"""CAN transport for separate-process DUTs: python-can + cantools (DBC).

Default transport: python-can's `udp_multicast` interface, which connects processes on one PC with no hardware.
Swap `interface`/`channel` for real hardware later (e.g. a USB-CAN adapter, SocketCAN on Linux).

Finding (2 Oct 2026): on a PC with several network adapters, UDP multicast delivers each frame twice, and python-can
stamps the RECEIVE time, so copies can't be matched by timestamp; the second copy can also arrive 60–90 ms late.
A real CAN bus never duplicates frames, and a duplicate would look like a REPEATED E2E frame. So every sender stamps the
python-can `channel` field (which the multicast transport carries end to end) with "<pid>:<sequence>", and receivers
drop any (pid, sequence) already seen. This is transport metadata only; it is not part of the CAN frame.
"""
from __future__ import annotations

import os
from collections import deque

from .config import ROOT

GROUP = "239.74.163.2"
STATES = ["INIT", "NORMAL", "DEGRADED", "PULL_OVER", "STOP_IN_LANE", "BRAKE_ONLY_STOP", "BACKUP_BRAKE_STOP", "OFF"]
CAUSES = [None, "TIMEOUT", "E2E_INVALID", "STALE_DATA", "WATCHDOG_LATE", "WATCHDOG_EARLY", "WATCHDOG_QA", "ENVELOPE",
          "STEER_ACTUATOR", "BRAKE_ACTUATOR", "ACT_BUS_OFF", "PERCEPTION_DEGRADED", "ODD_EXIT", "PERCEPTION_LOST", "SAFETY_RESET"]
ID = {"cmd": 0x100, "kick": 0x101, "act": 0x200, "status": 0x201, "fb": 0x300, "ctrl": 0x301}


def load_dbc():
    import cantools
    return cantools.database.load_file(str(ROOT / "dbc" / "safe_stop.dbc"))


def encode_status(db, counter: int, state: str, cause, challenge: int, mrm: bool, accel: float, steer: float) -> bytes:
    """SAF_Status through the DBC, as a supplied vECU would build it, then the CRC over bytes 1-7 + data ID (v2.10).
    Raw values (scaling off), rounded half to even like the C++ core, so the bytes match e2e.status_protect exactly."""
    from .e2e import STATUS_ID, crc8_h2f

    def i16(x: float) -> int:
        return max(-32768, min(32767, round(max(-327.0, min(327.0, x)) * 100)))
    d = bytearray(db.encode_message("SAF_Status", {
        "SAF_Status_CRC": 0, "SAF_Status_Counter": counter & 0xFF, "SAF_State": STATES.index(state),
        "SAF_MrmRequest": int(bool(mrm)), "SAF_Cause": CAUSES.index(cause) if cause in CAUSES else 0,
        "SAF_WdChallenge": challenge & 0xFF, "SAF_AccelOut": i16(accel), "SAF_SteerOut": i16(steer)}, scaling=False, strict=False))
    d[0] = crc8_h2f(bytes(d[1:8]) + STATUS_ID.to_bytes(2, "little"))
    return bytes(d)


class DedupBus:
    def __init__(self, interface: str = "udp_multicast", channel: str = GROUP):
        import can
        self.can = can
        self.bus = can.Bus(interface=interface, channel=channel, fd=True, receive_own_messages=False)
        self.pid: int | str = os.getpid()   # only ever formatted into the channel tag; a second sender appends a suffix
        self.seq = 0
        self.seen: set = set()
        self.order: deque = deque()
        self.duplicates = 0
        self.send_errors = 0

    def send(self, can_id: int, data: bytes, fd: bool = False) -> None:
        if fd and len(data) not in (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64):
            data = data + bytes(16 - len(data)) if len(data) < 16 else data   # pad to a valid CAN FD length
        self.seq += 1
        msg = self.can.Message(arbitration_id=can_id, data=data, is_fd=fd, is_extended_id=False, channel=f"{self.pid}:{self.seq}")
        for _attempt in range(20):   # Windows can briefly drop the multicast route (WinError 10065) when adapters change
            try:
                self.bus.send(msg)
                return
            except self.can.CanOperationError:
                self.send_errors += 1
                import time
                time.sleep(0.05)
        raise RuntimeError("multicast CAN transport unavailable (check network adapters / firewall)")

    def recv_all(self, timeout: float = 0) -> list:
        """Everything waiting now; with a timeout, wait up to that long for the first frame."""
        out = []
        while (m := self.bus.recv(timeout=timeout)) is not None:
            timeout = 0
            key = m.channel
            if key in self.seen:
                self.duplicates += 1
                continue
            self.seen.add(key)
            self.order.append(key)
            if len(self.order) > 100000:
                self.seen.discard(self.order.popleft())
            out.append(m)
        return out

    def shutdown(self) -> None:
        self.bus.shutdown()
