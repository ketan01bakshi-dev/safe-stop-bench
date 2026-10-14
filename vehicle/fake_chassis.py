"""P5 stage 1: the chassis side of the link as a pure state machine, plus a UDP wrapper (the "fake chassis").

ChassisCore has no sockets and no clock of its own: the host passes the time in milliseconds into rx() and step(), and gets back the
datagrams to send as (address, bytes). The same core is driven by a virtual clock (vehicle/simnet.py) and by a real UDP thread (UdpChassis),
and the chassis firmware of stage 2 is a port of it (golden cases in vehicle/link_cases.json).

    DISARMED --ARM (right boot id, newer run id, sane cap)--> ARMED --STOP(END_OF_RUN)--> DISARMED
    ARMED --dead-man / STOP(ABORT, WATCHDOG) / bad nonce / bad run / out-of-range pwm / link lost--> LATCHED_STOP
    LATCHED_STOP --physical re-arm only--> DISARMED

Wheel encoders (link version 2): the host passes the four cumulative pulse counts into set_encoders() whenever it samples them (the firmware's
PCNT read); the next ECHO carries the latest sample and its time. They never change a state or a pin.

The motors follow `target` through ONE writer that runs every `writer_period_ms` (as the firmware's single motor-pin task will): a latch zeroes
the target at once, the pins follow at the next writer tick. outputs / out_log are what the pins would carry.
"""
from __future__ import annotations

import socket
import threading
import time
from collections.abc import Callable
from typing import Any

from vehicle import link
from vehicle.link import Reason, Reject, State, StopReason

Address = Any
Outgoing = list[tuple[Address, bytes]]

DEADMAN_MS = 200            # more than this since the last valid SETPOINT (or the ARM) latches the stop
WRITER_PERIOD_MS = 10
ECHO_PERIOD_MS = 50         # 20 Hz
HELLO_PERIOD_MS = 500
HARD_CAP = 255

_STOP_LATCH = {StopReason.ABORT: Reason.BRIDGE_ABORT, StopReason.WATCHDOG: Reason.BRIDGE_WATCHDOG}


class ChassisCore:
    def __init__(self, boot_id: int, *, build: str = "p5-sim", hello_peer: Address = "bridge", deadman_ms: int = DEADMAN_MS,
                 writer_period_ms: int = WRITER_PERIOD_MS, echo_period_ms: int = ECHO_PERIOD_MS, hello_period_ms: int = HELLO_PERIOD_MS,
                 hard_cap: int = HARD_CAP, now_ms: int = 0):
        self.boot_id, self.build, self.hello_peer = boot_id, build, hello_peer
        self.deadman_ms, self.writer_period_ms = deadman_ms, writer_period_ms
        self.echo_period_ms, self.hello_period_ms, self.hard_cap = echo_period_ms, hello_period_ms, hard_cap
        self.t0 = now_ms
        self.state, self.reason, self.reject = State.DISARMED, Reason.NONE, Reject.NONE
        self.last_run_id = 0
        self.run_id = self.nonce = self.cap = 0
        self.peer: Address = None
        self.last_seq = 0
        self.last_valid = now_ms
        self.target = (0, 0)
        self.outputs = (0, 0)
        self._last_write = now_ms - writer_period_ms
        self._last_echo = self._last_hello = now_ms - 10**9
        self.rx_ok = self.rx_bad = self.rx_ignored = self.rx_foreign = self.rx_stale = self.rx_rejected = 0
        self.enc_t, self.enc_mask, self.enc = now_ms, 0, (0, 0, 0, 0)
        self.out_log: list[tuple[int, int, int]] = [(now_ms, 0, 0)]                   # (t, pwm_l, pwm_r) at every change of the pins
        self.state_log: list[tuple[int, str, str]] = [(now_ms, State.DISARMED.name, Reason.NONE.name)]
        self.events: list[tuple[int, str]] = []

    # --- helpers --------------------------------------------------------------------------------------------------------------------

    def _note(self, now: int, text: str) -> None:
        self.events.append((now, text))

    def _set_state(self, now: int, state: State, reason: Reason = Reason.NONE) -> None:
        self.state, self.reason = state, reason
        self.state_log.append((now, state.name, reason.name))

    def _latch(self, now: int, reason: Reason) -> None:
        if self.state is State.LATCHED_STOP:
            return
        self.target = (0, 0)
        self._set_state(now, State.LATCHED_STOP, reason)
        self._note(now, f"latched: {reason.name}")

    def _echo(self, now: int) -> bytes:
        self._last_echo = now
        return link.encode(link.Echo(self.boot_id, self.run_id, self.nonce, self.state, self.reason, self.reject, self.last_seq,
                                     self.outputs[0], self.outputs[1], (now - self.t0) & 0xFFFFFFFF, self.rx_ok & 0xFFFF, self.rx_bad & 0xFFFF,
                                     (self.enc_t - self.t0) & 0xFFFFFFFF, self.enc_mask, *self.enc))

    def _hello(self, now: int) -> bytes:
        self._last_hello = now
        return link.encode(link.Hello(self.boot_id, self.build, self.state, self.reason, self.reject, self.last_run_id))

    def _refuse(self, now: int, why: Reject, src: Address) -> Outgoing:
        self.rx_rejected += 1
        self.reject = why
        self._note(now, f"ARM refused: {why.name}")
        return [(src, self._hello(now))]

    # --- inputs ---------------------------------------------------------------------------------------------------------------------

    def link_down(self, now: int) -> None:
        """The Wi-Fi association dropped (the firmware's WiFi event handler)."""
        if self.state is State.ARMED:
            self._latch(now, Reason.LINK_LOST)

    def set_encoders(self, now: int, counts: tuple[int, int, int, int], mask: int = link.ENC_ALL) -> None:
        """A sample of the four cumulative slot counts (FL, FR, RL, RR). Kept as u32 (they wrap); a wheel outside `mask` reads 0."""
        mask &= link.ENC_ALL
        self.enc_t, self.enc_mask = now, mask
        self.enc = tuple((c & 0xFFFFFFFF) if mask >> i & 1 else 0 for i, c in enumerate(counts))  # type: ignore[assignment]

    def physical_rearm(self, now: int) -> None:
        """The hardware re-arm control (a button, not a message): the only way out of LATCHED_STOP."""
        if self.state is State.LATCHED_STOP:
            self._set_state(now, State.DISARMED, Reason.NONE)
            self._note(now, "physical re-arm")
            self.reject = Reject.NONE

    def rx(self, data: bytes, src: Address, now: int) -> Outgoing:
        try:
            msg = link.decode(data)
        except link.LinkError:
            self.rx_bad += 1
            return []
        if isinstance(msg, link.Arm):
            return self._on_arm(msg, src, now)
        if isinstance(msg, (link.Hello, link.Echo)):                                   # chassis-to-bridge types are not for us
            self.rx_ignored += 1
            return []
        if self.state is State.DISARMED:
            self.rx_ignored += 1                                                       # a late frame after the run ended
            return []
        if src != self.peer:
            self.rx_foreign += 1
            return []
        if self.state is State.LATCHED_STOP:
            self.rx_ignored += 1
            return []
        # ARMED, from the armed peer
        if msg.run_id != self.run_id:
            self._latch(now, Reason.BAD_RUN)
            return [(self.peer, self._echo(now))]
        if msg.nonce != self.nonce:
            self._latch(now, Reason.BAD_NONCE)
            return [(self.peer, self._echo(now))]
        if isinstance(msg, link.Stop):
            self.rx_ok += 1
            if msg.reason == StopReason.END_OF_RUN:
                self.target = (0, 0)
                self._set_state(now, State.DISARMED, Reason.NONE)
                self._note(now, "end of run: disarmed")
                return [(src, self._hello(now))]
            self._latch(now, _STOP_LATCH.get(msg.reason, Reason.BRIDGE_ABORT))            # an unknown stop reason is treated as an abort
            return [(self.peer, self._echo(now))]
        assert isinstance(msg, link.Setpoint)
        if not (0 <= msg.pwm_l <= self.cap and 0 <= msg.pwm_r <= self.cap):
            self._latch(now, Reason.CMD_RANGE)
            return [(self.peer, self._echo(now))]
        if msg.seq <= self.last_seq:
            self.rx_stale += 1                                                         # duplicate or reordered: harmless, but not a sign of life
            return []
        self.rx_ok += 1
        self.last_seq = msg.seq
        self.last_valid = now
        self.target = (msg.pwm_l, msg.pwm_r)
        return []

    def _on_arm(self, msg: link.Arm, src: Address, now: int) -> Outgoing:
        if self.state is State.LATCHED_STOP:
            return self._refuse(now, Reject.LATCHED, src)
        if self.state is State.ARMED:
            if src != self.peer:
                self.rx_foreign += 1
                self.reject = Reject.OTHER_PEER
                return []
            if (msg.boot_id, msg.run_id, msg.nonce) == (self.boot_id, self.run_id, self.nonce):
                return [(self.peer, self._echo(now))]                                  # a retransmitted ARM: say so again, change nothing
            self._latch(now, Reason.BAD_RUN)                                           # the armed bridge itself contradicts its ARM
            return [(self.peer, self._echo(now))]
        if msg.boot_id != self.boot_id:
            return self._refuse(now, Reject.BOOT_ID, src)
        if msg.run_id <= self.last_run_id:
            return self._refuse(now, Reject.RUN_OLD, src)
        if not 1 <= msg.pwm_cap <= self.hard_cap:
            return self._refuse(now, Reject.CAP, src)
        self.run_id, self.nonce, self.cap, self.peer = msg.run_id, msg.nonce, msg.pwm_cap, src
        self.last_run_id, self.last_seq, self.last_valid, self.target = msg.run_id, 0, now, (0, 0)
        self.reject = Reject.NONE
        self.rx_ok += 1
        self._set_state(now, State.ARMED, Reason.NONE)
        self._note(now, f"armed: run {msg.run_id}, cap {msg.pwm_cap}")
        return [(src, self._echo(now))]

    # --- the periodic part ----------------------------------------------------------------------------------------------------------

    def step(self, now: int) -> Outgoing:
        out: Outgoing = []
        if self.state is State.ARMED and now - self.last_valid > self.deadman_ms:
            self._latch(now, Reason.DEADMAN)
            out.append((self.peer, self._echo(now)))
        if now - self._last_write >= self.writer_period_ms:                          # the single motor-pin writer
            self._last_write = now
            want = self.target if self.state is State.ARMED else (0, 0)
            if want != self.outputs:
                self.outputs = want
                self.out_log.append((now, want[0], want[1]))
        if self.state is State.ARMED and self.peer is not None and now - self._last_echo >= self.echo_period_ms:
            out.append((self.peer, self._echo(now)))                                  # 20 Hz while armed
        elif self.state is not State.ARMED and self.hello_peer is not None and now - self._last_hello >= self.hello_period_ms:
            out.append((self.hello_peer, self._hello(now)))                           # 2 Hz while disarmed or latched: anyone listening sees why
        return out

    def summary(self) -> dict:
        return {"state": self.state.name, "reason": self.reason.name, "outputs": list(self.outputs), "last_run_id": self.last_run_id,
                "rx_ok": self.rx_ok, "rx_bad": self.rx_bad, "rx_ignored": self.rx_ignored, "rx_foreign": self.rx_foreign,
                "rx_stale": self.rx_stale, "rx_rejected": self.rx_rejected, "writer_changes": len(self.out_log) - 1,
                "enc_mask": self.enc_mask, "enc": list(self.enc)}


class UdpChassis:
    """ChassisCore on a real UDP socket in a thread (the 'fake chassis' of a rehearsal). Its clock is its own perf_counter, as a board's would be."""

    def __init__(self, core: ChassisCore, hello_peer: tuple[str, int], bind: tuple[str, int] = ("127.0.0.1", 0),
                 drop_rx: Callable[[int, bytes], bool] | None = None):
        self.core = core
        core.hello_peer = hello_peer
        self.drop_rx = drop_rx                                                         # test hook: (now_ms, data) -> True swallows the datagram
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(bind)
        self.sock.settimeout(0.001)
        self.address = self.sock.getsockname()
        self._t0 = time.perf_counter()
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._loop, name="fake-chassis", daemon=True)

    def now(self) -> int:
        return int((time.perf_counter() - self._t0) * 1000)

    def start(self) -> UdpChassis:
        self._thread.start()
        return self

    def _send(self, out: Outgoing) -> None:
        for addr, data in out:
            try:
                self.sock.sendto(data, addr)
            except OSError:
                pass

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, src = self.sock.recvfrom(256)
            except TimeoutError:
                data, src = None, None
            except ConnectionResetError:                                               # Windows: an ICMP 'port unreachable' for an earlier sendto
                data, src = None, None
            except OSError:
                break
            now = self.now()
            with self._lock:
                if data is not None and not (self.drop_rx and self.drop_rx(now, data)):
                    self._send(self.core.rx(data, src, now))
                self._send(self.core.step(now))

    def call(self, fn: Callable[[ChassisCore, int], Any]) -> Any:
        """Run fn(core, now) under the chassis lock (e.g. core.physical_rearm, core.link_down)."""
        with self._lock:
            return fn(self.core, self.now())

    def snapshot(self) -> dict:
        with self._lock:
            return self.core.summary()

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.sock.close()

    def __enter__(self) -> UdpChassis:
        return self.start()

    def __exit__(self, *exc) -> None:
        self.close()
