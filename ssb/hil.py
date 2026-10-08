"""HiL on two ESP32-S3 boards (or PiL on one), and the link protocol the bench uses to talk to them.

    board B  = the safety controller: hil/firmware/SafetyNode (SafeStopCore C++ + Arduino shim + MCP2515 CAN)
    board A  = the actuator-bus node: hil/firmware/BusNode (forwards every CAN frame it receives to the PC)
    PC       = planner, vehicle model, fault injection, oracle (this bench)

    PC --USB--> B : 'R' reset + calibration, 'P' planner frame (14 bytes, E2E Profile 5 checked ON the MCU),
                    'F' sensors, 'K' watchdog kicks, 'C' power / tx ok / operator release
    B ==CAN 500 kbps==> A : SAF_ActuatorCmd 0x200 (Profile 2) and SAF_Status 0x201, every 10 ms, on a real bus
    A --USB--> PC : 'M' every CAN frame A received.   B --USB--> PC : 'M' mirror of what B sent, 'D' diagnostics

Modes (run.py --dut ...):  loopback = board B's node logic built for the PC, lockstep, no hardware;
pil = one board, outputs read from B's USB mirror;  hil = two boards, outputs read from A, i.e. through the real bus.

Honest limits: the MCP2515 is classic CAN only, so the planner link (CAN FD in the design) runs over USB carrying
the same bytes; the brown-out is emulated in firmware (the board stays powered); kicks go over USB unless the GPIO
watchdog wire is fitted (--kick gpio).
"""
from __future__ import annotations

import struct
import time

from .dut import BenchFault, BlackBoxObserver, DeviceUnderTest, Outputs
from .e2e import StatusReceiver, crc8_h2f
from .native import STATES, cfg_array, core_identity, defects_mask

SYNC = 0xA5


def frame_msg(kind: str, payload: bytes = b"") -> bytes:
    body = bytes([ord(kind), len(payload)]) + payload
    return bytes([SYNC]) + body + bytes([crc8_h2f(body)])


class Parser:
    """Byte stream -> (kind, payload) messages; same rules as ssc::Parser in the firmware."""

    def __init__(self):
        self.buf, self.errors = bytearray(), 0

    def feed(self, data: bytes) -> list[tuple[str, bytes]]:
        self.buf += data
        out: list[tuple[str, bytes]] = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                self.buf.clear()
                return out
            del self.buf[:i]
            if len(self.buf) < 3 or len(self.buf) < 4 + self.buf[2]:
                return out
            n = self.buf[2]
            body, crc = bytes(self.buf[1:3 + n]), self.buf[3 + n]
            if crc8_h2f(body) == crc:
                out.append((chr(body[0]), body[2:]))
                del self.buf[:4 + n]
            else:
                self.errors += 1
                del self.buf[:1]


class SerialLink:
    """A board on a COM port (pyserial). Opening a port can reset an ESP32 through DTR/RTS; we hold both low."""
    realtime = True

    def __init__(self, port: str, baud: int = 921600):
        import serial
        self.port, self.baud = port, baud   # kept to reopen the port (v2.12: B's port vanishes during a power cut)
        self.url = "://" in port
        if "://" in port:   # e.g. socket://127.0.0.1:7777 (the board emulator)
            self.ser = serial.serial_for_url(port, timeout=0, write_timeout=1.0)
        else:
            self.ser = serial.Serial()
            self.ser.port, self.ser.baudrate, self.ser.timeout, self.ser.write_timeout = port, baud, 0, 1.0
            self.ser.dtr = False
            self.ser.rts = False
            self.ser.open()
        self.name = f"serial {port}" + ("" if "://" in port else f"@{baud}")

    def write(self, data: bytes, now_ms: int) -> None:
        self.ser.write(data)

    def reset_board(self) -> bool:
        """Pulse EN through RTS (DTR held low = normal boot, not download mode). Found on hardware 2026-10-06: a run that
        ends abruptly leaves the node 'active', and an active node sends no hello until it is reset."""
        if self.url:
            return False
        import time
        self.ser.dtr = False
        self.ser.rts = True
        time.sleep(0.1)
        self.ser.rts = False
        time.sleep(1.5)            # boot + MCP2515 init
        self.ser.reset_input_buffer()
        return True

    def poll(self, now_ms: int) -> None:
        pass

    def read(self) -> bytes:
        if self.url:   # pyserial's socket:// reports in_waiting as 0 or 1 only; a non-blocking read returns all there is
            return self.ser.read(65536)
        n = self.ser.in_waiting
        return self.ser.read(n) if n else b""

    def close(self) -> None:
        self.ser.close()


class Emulator:
    """Starts ssb.hil_emu (board B, and optionally A, emulated in a separate process) on free local TCP ports."""

    def __init__(self, with_a: bool):
        import socket
        import subprocess
        import sys
        from pathlib import Path

        def free_port() -> int:
            with socket.socket() as s:
                s.bind(("127.0.0.1", 0))
                return s.getsockname()[1]
        self.port_b, self.port_a = free_port(), (free_port() if with_a else None)
        cmd = [sys.executable, "-m", "ssb.hil_emu", "--port-b", str(self.port_b)] + (["--port-a", str(self.port_a)] if with_a else [])
        root = Path(__file__).resolve().parent.parent
        (root / "reports").mkdir(exist_ok=True)
        self.log = root / "reports" / "emulator.log"
        self.proc = subprocess.Popen(cmd, cwd=str(root), stdout=subprocess.PIPE, stderr=open(self.log, "w"), text=True)
        assert self.proc.stdout is not None
        line = self.proc.stdout.readline()
        if "ready" not in line:
            raise RuntimeError(f"board emulator did not start: {line!r}")

    def alive(self) -> bool:
        return self.proc.poll() is None

    def close(self) -> None:
        self.proc.terminate()
        self.proc.wait(timeout=5)


class LinkDUT(BlackBoxObserver, DeviceUnderTest):
    """Drives a safety node over the link protocol. `link_b` = board B (or the loopback); `link_a` = board A (hil)."""

    emulator: "Emulator | None" = None   # set by make() for --port-b EMU

    def __init__(self, cfg: dict, link_b, link_a=None, kick: str = "usb", fb_period_ms: int | None = None,
                 hello_timeout_s: float = 5.0, bus_monitor: bool | None = None):
        self.cfg, self.b, self.a = cfg, link_b, link_a
        self.realtime = getattr(link_b, "realtime", False)
        self.kick_src = {"usb": 0, "gpio": 1}[kick]
        # B treats CAN transmit error-passive / bus-off as an actuator-bus fault only when another node can ACK
        self.bus_monitor = (link_a is not None) if bus_monitor is None else bus_monitor
        self.fb_period = fb_period_ms or (2 if self.realtime else 1)
        self.relay_fitted = bool(cfg.get("dut_hw", {}).get("relay_fitted"))   # v2.12: A switches B's supply
        self.b_down = False   # B unpowered by a power cut: its USB port is gone until the next prepare()
        self.pb, self.pa = Parser(), Parser()
        self.defects: frozenset = frozenset()
        self.diag: dict = {}
        self.diag_a: dict = {}
        self.fw_b = self.fw_a = "?"
        self.mode = "hil" if link_a else ("pil" if self.realtime else "loopback")
        self.name = f"{self.mode}: B={link_b.name}" + (f", A={link_a.name}" if link_a else "")
        self._wait_hello(hello_timeout_s)
        self.timing: list[dict] = []

    # ---- link helpers ----------------------------------------------------------------------------------------------
    def _read(self) -> tuple[list, list]:
        mb = [] if self.b_down else self.pb.feed(self.b.read())
        ma = self.pa.feed(self.a.read()) if self.a else []
        return mb, ma

    def _wait_hello(self, timeout_s: float) -> None:
        if not self.realtime:
            self.b.poll(0)
            for k, p in self._read()[0]:
                if k == "H":
                    self.fw_b = p.decode(errors="replace")
            return
        t_end, got_b, got_a = time.time() + timeout_s, False, self.a is None
        reset_tried = False
        while not (got_b and got_a):
            if time.time() >= t_end:
                if reset_tried:
                    break
                reset_tried = True       # a stale 'active' node is silent: reset the silent board(s) once, then wait again
                for got, link in ((got_b, self.b), (got_a, self.a)):
                    if not got and link is not None and hasattr(link, "reset_board"):
                        link.reset_board()
                t_end = time.time() + timeout_s
            mb, ma = self._read()
            for k, p in mb:
                if k == "H":
                    got_b, self.fw_b = True, p.decode(errors="replace")
            for k, p in ma:
                if k == "H":
                    got_a, self.fw_a = True, p.decode(errors="replace")
            time.sleep(0.01)
        if not got_b:
            raise RuntimeError(f"board B ({self.b.name}) sent no hello: wrong port, wrong firmware, or it is still booting")
        if not got_a:
            raise RuntimeError(f"board A ({self.a.name}) sent no hello: wrong port or wrong firmware")

    def prepare(self, warm_start: bool, defects=frozenset()) -> None:
        if self.b_down:
            self._reopen_b()
        self.defects = frozenset(defects)
        options = self.kick_src | (2 if self.bus_monitor else 0)
        payload = struct.pack("<BIBB", 1 if warm_start else 2, defects_mask(self.defects), options, len(cfg_array(self.cfg)))
        payload += struct.pack(f"<{len(cfg_array(self.cfg))}d", *cfg_array(self.cfg))
        self._read()   # drop anything left over from the previous scenario
        self.b.write(frame_msg("R", payload), 0)
        t_end, acked = time.time() + 3.0, False
        while not acked:   # the node acknowledges while parsing 'R'; no poll here, or cycle 0 would run early
            for k, _p in self._read()[0]:
                acked = acked or k == "A"
            if acked or not self.realtime:
                break
            if time.time() > t_end:
                raise RuntimeError("board B did not acknowledge the reset")
            time.sleep(0.001)
        if not acked:
            raise RuntimeError("node did not acknowledge the reset")
        self.ctrl: tuple[bool, bool, bool] | None = None
        self.rts_release_t: int | None = None
        self.resetting_since: int | None = None
        self.n_inflight = 0
        self.kick_n = 0
        self.last = Outputs(state="NORMAL" if warm_start else "INIT")
        self._observe_reset()
        self.n_status = self.n_act = 0
        self.last_status_t = 0
        self.a_reinits0 = self.diag_a.get("reinits", 0)
        self.mirror: list[list] = []    # B's own USB copy of each SAF_Status it sent: [t, bytes, used]
        self.n_replay = 0
        self.status_rx = StatusReceiver()   # SAF_Status E2E receiver (v2.9.6; shared with the CAN-process bench, v2.10)
        self.pend: list[tuple[int, bytes]] = []
        self.act_c: int | None = None   # last Profile 2 counter seen on SAF_ActuatorCmd
        self.cyc_ms = 0                 # B's cycle time, unwrapped from that counter (10 ms per count)
        self.cyc: tuple | None = None

    # Found on hardware 2026-10-06: when the CAN link to board A dropped mid-campaign, every later scenario "failed"
    # with the DUT apparently stuck in NORMAL (48 false FAILs). The bench must call its own fault, not blame the DUT.
    OBSERVER_TIMEOUT_MS = 500

    # ---- one millisecond -------------------------------------------------------------------------------------------
    def step(self, t, frames, kicks, fb, release, power_ok, tx_ok) -> Outputs:
        if self.rts_release_t is not None and t >= self.rts_release_t:
            self.b.ser.rts = False   # release EN: the board boots
            self.rts_release_t = None
        out = b""
        if t % self.fb_period == 0 or release:
            out += frame_msg("F", struct.pack("<5d", fb["v"], fb["a"], fb["delta"], fb["yaw_rate"], fb.get("grade_accel", 0.0)))
        ctrl = (bool(power_ok), bool(tx_ok), bool(release))
        if ctrl != self.ctrl or t % 10 == 0:
            out += frame_msg("C", bytes([ctrl[0] | ctrl[1] << 1 | ctrl[2] << 2]))
            self.ctrl = ctrl
        if kicks and self.kick_src == 1 and self.a:   # board A pulses the hardware watchdog line into board B
            self.a.write(frame_msg("K", bytes([len(kicks)])), t)
        elif kicks:
            out += frame_msg("K", bytes([len(kicks)]))
        for f in frames:
            if f.can_id == 0x100:
                out += frame_msg("P", bytes(f.data)[:16])
        if out and not self.b_down:
            try:
                self.b.write(out, t)
            except Exception as e:
                emu = getattr(self, "emulator", None)
                if emu and not emu.alive():
                    raise RuntimeError(f"the board emulator died; see {emu.log}") from e
                raise
        self.b.poll(t)

        mb, ma = self._read()
        frames_in = []
        for k, p in mb:
            if k == "M" and not self.a:
                frames_in.append(p)
            elif k == "M" and (p[0] | p[1] << 8) == 0x201:
                self.mirror.append([t, bytes(p), False])
            elif k == "D" and len(p) >= 23:
                c, ex, late, txf, ovf = struct.unpack("<5I", p[:20])
                self.diag = {"cycles": c, "max_exec_us": ex, "max_late_ms": late, "can_tx_fail": txf, "rx_overflow": ovf,
                             "link_errors": p[20], "bus_fault": bool(p[21]), "kick_src": "gpio" if p[22] & 1 else "usb",
                             "bus_monitor": bool(p[22] & 2)}
        self.mirror = [m for m in self.mirror if t - m[0] <= 50]
        for k, p in ma:
            if k == "M":
                if (p[0] | p[1] << 8) == 0x201:
                    self.pend.append((t, bytes(p)))
                    continue
                frames_in.append(p)
            elif k == "D" and len(p) >= 12:
                self.diag_a = {"rx": struct.unpack_from("<I", p)[0], "eflg": p[4], "rx_overflow": struct.unpack_from("<I", p, 5)[0],
                               "bad_mode": p[9], "reinits": p[10] | p[11] << 8,
                               "spi_glitches": p[12] | p[13] << 8 if len(p) >= 14 else None}
                if len(p) >= 31:   # BusNode 2.6: the RAM queue between the CAN task and USB
                    self.diag_a["queue_high"] = p[25] | p[26] << 8
                    self.diag_a["queue_drops"] = struct.unpack_from("<I", p, 27)[0]
        # SAF_Status in arrival order: used once B's USB copy vouches for it; one with no copy within 20 ms of its arrival
        # is a replay. Usually the copy is already here (0-2 ms ahead); if B's port is read late, the frame waits.
        while self.pend:
            t_bus, q = self.pend[0]
            if self._sent_by_b(t_bus, q):
                frames_in.append(q)
            elif t - t_bus > 20:
                self.n_replay += 1
            else:
                break
            self.pend.pop(0)
        if self.diag_a.get("reinits", 0) != self.a_reinits0:
            self.a_reinits0 = self.diag_a["reinits"]
            raise BenchFault(f"observer fault at {t} ms: board A's MCP2515 left normal mode (CANSTAT mode "
                             f"{self.diag_a['bad_mode']}) and was re-initialised; B's frames went unacknowledged meanwhile")
        act, st, cause, ch, mrm, cmd = [], self.last.state, self.last.cause, self.last.challenge, self.last.mrm_request, self.last.out_cmd
        from .canio import CAUSES
        for p in frames_in:
            cid, n = p[0] | p[1] << 8, p[2]
            data = bytes(p[3:3 + n])
            if cid == 0x200:
                act.append((cid, data))
                self.n_act += 1
                if n == 7:
                    # B's own time base: the Profile 2 counter advances once per 10 ms cycle (mod 16, so up to
                    # 15 lost frames unwrap correctly). Found on hardware: sampling B's steering on the PC's clock
                    # turned 1 ms of USB arrival jitter into a false 72 deg/s slew (v2.9 finding 5).
                    c = data[1] & 0x0F
                    d = 1 if self.act_c is None else (c - self.act_c) % 16
                    if d:   # d == 0: a repeated frame, not a new cycle
                        self.cyc_ms += 10 * d
                        self.act_c = c
                        self.cyc = (self.cyc_ms, struct.unpack_from("<h", data, 4)[0] / 100)
            elif cid == 0x201 and n == 8:
                if self.resetting_since is not None and t - self.resetting_since < self.MIN_BOOT_MS:
                    self.n_inflight += 1   # sent before the reset, still in flight: the board can't be back yet
                    continue
                if not self._status_e2e_ok(data):
                    continue
                _crc, _ctr, packed, ch, a_raw, s_raw = struct.unpack("<BBBBhh", data)
                s_i, m_i, c_i = packed & 0x07, (packed >> 3) & 1, packed >> 4
                st = STATES[s_i] if s_i < len(STATES) else f"UNKNOWN_{s_i}"
                cause = CAUSES[c_i] if c_i < len(CAUSES) else f"CAUSE_{c_i}"
                mrm = "PULL_OVER" if m_i else None
                cmd = (a_raw / 100, s_raw / 100, False)
                self.n_status += 1
                self.last_status_t = t
                if self.resetting_since is not None:
                    self.events_hw = getattr(self, "events_hw", [])
                    self.events_hw.append((self.resetting_since, t))
                    self.resetting_since = None   # back from the reset: its first status
        if self.resetting_since is not None:
            st = "OFF"   # booting after the bench's reset: nothing on the bus yet
        if power_ok and self.resetting_since is None and t - self.last_status_t > self.OBSERVER_TIMEOUT_MS:
            src = "board A (CAN bus)" if self.a else "board B (USB mirror)"
            raise BenchFault(f"observer lost at {t} ms: no SAF_Status from {src} for {t - self.last_status_t} ms. "
                               f"Bench fault, not a DUT verdict: check the CAN link (J2 H/L/GND, termination) and A's "
                               f"MCP2515. Board B diagnostics: {self.diag}; board A: {self.diag_a}")
        self._observe(t, self.last.state, st, cause, release)
        self.last = Outputs(act, st, cause, ch, mrm, cmd, self.cyc)
        return self.last

    # Found on hardware (v2.9.3, cause found in v2.9.4): now and then an old SAF_Status, byte-identical to one from
    # seconds or hours earlier, reached the bench. SAF_Status has no alive counter, so a 1 ms "STOP -> NORMAL" blip
    # looked like the DUT leaving a latched stop. The source was board A: its MCP2515 misread a flag as "RXB1 full"
    # and the library re-read a stale RXB1 (fixed in BusNode 2.5). Kept as defence in depth: a bus frame counts only if
    # B's own USB copy shows it sent exactly those bytes in the last 20 ms (the copy usually arrives 0-2 ms before the bus one),
    # each copy vouching for one bus frame. Anything else is counted as a replay and not used. v2.9.4: the copy may
    # also arrive after the bus frame (B's port read late for a while: 40 genuine frames were rejected in one run), so
    # an unvouched frame waits up to 20 ms, in order, before it is called a replay.
    def _sent_by_b(self, t: int, p: bytes) -> bool:
        if not self.mirror:
            return True   # no USB copies (older firmware or a stalled link): cannot tell, so do not filter
        for m in self.mirror:
            if not m[2] and m[1] == p and abs(t - m[0]) <= 20:
                m[2] = True
                return True
        return False

    # SAF_Status E2E (v2.9.6): CRC-8 over bytes 1-7 + data ID 0x201, then an 8-bit alive counter (see e2e.StatusReceiver,
    # the same receiver the CAN-process bench uses since v2.10).
    def _status_e2e_ok(self, data: bytes) -> bool:
        return self.status_rx.check(data)

    @property
    def n_status_crc(self) -> int:
        return self.status_rx.n_crc

    @property
    def n_status_seq(self) -> int:
        return self.status_rx.n_seq

    # ---- real reset (v2.9.7) -------------------------------------------------------------------------------------
    # EN through RTS on the CH340 (DTR held low, or the ESP32 would start its bootloader). Non-blocking: RTS goes high
    # now and low 2 ms later inside step(). While the board boots it sends nothing at all, so the observer reports
    # OFF (the bench applied the reset, it knows) and the observer-loss guard waits for it, up to BOOT_TIMEOUT_MS.
    # A board that does not come back stays OFF: that is a DUT verdict (no latched stop), not a bench fault.
    BOOT_TIMEOUT_MS = 3000
    MIN_BOOT_MS = 50   # measured boot 185-187 ms; a status within 50 ms of the reset pulse was sent before it

    @property
    def can_hw_reset(self) -> bool:
        return isinstance(self.b, SerialLink) and not self.b.url

    def hw_reset(self, t: int) -> None:
        ser = self.b.ser
        ser.dtr = False
        ser.rts = True
        self.rts_release_t = t + 2
        self.resetting_since = t
        self.status_rx.reset()   # the board's alive counter restarts
        self.n_hw_resets = getattr(self, "n_hw_resets", 0) + 1

    # ---- power cut (v2.12) ------------------------------------------------------------------------------------------
    # Board A drives a relay in board B's 5 V supply ('X' <ms>, BusNode 2.7; docs/HIL_POWER_CUT.md). Unlike the reset,
    # the whole board goes: CPU, CAN transceiver and USB-serial chip (B's USB lead has its 5 V line blocked, or USB would
    # back-power the board). So the bench stops using B's port, reads B only through A on the bus (it always does), and
    # reopens the port before the next scenario. As with the reset: OFF while down and booting; a board that doesn't
    # come back stays OFF, which is a DUT verdict.
    @property
    def can_power_cut(self) -> bool:
        return self.a is not None and (self.emulator is not None or self.relay_fitted)

    def power_cut(self, t: int, ms: int) -> None:
        self.a.write(frame_msg("X", struct.pack("<H", max(1, min(int(ms), 60000)))), t)
        self.b_down = True
        self.resetting_since = t
        self.status_rx.reset()   # the board's alive counter restarts
        self.n_power_cuts = getattr(self, "n_power_cuts", 0) + 1

    def _reopen_b(self, timeout_s: float = 15.0) -> None:
        """After a power cut: wait for B's port to come back (USB re-enumeration on the board), reopen it, wait for B."""
        port, baud = self.b.port, self.b.baud
        try:
            self.b.close()
        except Exception:  # noqa: BLE001 (the port vanished under us)
            pass
        t_end = time.time() + timeout_s
        while True:
            try:
                self.b = SerialLink(port, baud)
                break
            except Exception as e:  # noqa: BLE001
                if time.time() > t_end:
                    raise BenchFault(f"board B's port {port} did not come back within {timeout_s:g} s after the power cut: {e}") from e
                time.sleep(0.2)
        self.b_down, self.pb = False, Parser()
        self._wait_hello(5.0)

    def recover(self) -> None:
        """After a bench fault: reset both boards (EN via RTS) and wait for their hellos."""
        if self.b_down:
            self._reopen_b()
        for link in (self.b, self.a):
            if link is not None and hasattr(link, "reset_board"):
                link.reset_board()
        self.pb, self.pa = Parser(), Parser()
        self.diag_a = {}
        self._wait_hello(5.0)

    def identity(self) -> str:
        fw = f"fw B '{self.fw_b}'" + (f", fw A '{self.fw_a}'" if self.a else "")
        return f"{self.name}; {fw}; {core_identity()}"

    def close(self) -> None:
        for link in (self.b, self.a):
            if link:
                try:
                    link.close()
                except Exception:  # noqa: BLE001 (B's port may have vanished in a power cut)
                    pass
        if self.emulator:
            self.emulator.close()


def make(mode: str, cfg: dict, port_b: str | None = None, port_a: str | None = None, kick: str = "usb",
         baud: int = 921600, bus_monitor: bool | None = None) -> LinkDUT:
    if mode == "loopback":
        from .native import LoopbackLink
        return LinkDUT(cfg, LoopbackLink(), kick=kick, bus_monitor=bus_monitor)
    if not port_b:
        raise ValueError(f"--dut {mode} needs --port-b COMx (board B, the safety controller)")
    if port_b.upper() == "EMU":   # dry run without hardware: emulated boards in a separate process
        emu = Emulator(with_a=mode == "hil")
        b = SerialLink(f"socket://127.0.0.1:{emu.port_b}")
        a = SerialLink(f"socket://127.0.0.1:{emu.port_a}") if mode == "hil" else None
        dut = LinkDUT(cfg, b, a, kick=kick, bus_monitor=bus_monitor)
        dut.emulator = emu
        dut.name += " [EMULATED boards]"
        return dut
    b = SerialLink(port_b, baud)
    a = None
    if mode == "hil":
        if not port_a:
            b.close()
            raise ValueError("--dut hil needs --port-a COMy (board A, the bus node) as well")
        a = SerialLink(port_a, baud)
    return LinkDUT(cfg, b, a, kick=kick, bus_monitor=bus_monitor)


def list_ports() -> list[str]:
    from serial.tools import list_ports as lp
    rows = []
    for p in lp.comports():
        tag = ""
        if p.vid == 0x303A:
            tag = "  <- Espressif native USB (ESP32-S3)"
        elif p.vid in (0x1A86, 0x10C4, 0x0403):
            tag = "  <- USB-UART bridge (CH34x / CP210x / FTDI): typical ESP32 dev board"
        rows.append(f"{p.device:8} {p.description} [{p.vid and hex(p.vid)}:{p.pid and hex(p.pid)}]{tag}")
    return rows


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="HiL bring-up helpers")
    ap.add_argument("--ports", action="store_true", help="list COM ports and flag likely ESP32 boards")
    ap.add_argument("--hello", nargs="+", metavar="COM", help="open each port and print the firmware hello")
    ap.add_argument("--identify", metavar="LABEL", help="wait for ONE newly plugged-in board and print its COM port")
    a = ap.parse_args()
    if a.identify:
        from serial.tools import list_ports as lp
        before = {p.device for p in lp.comports()}
        print(f"Plug in board {a.identify} now (USB data cable). Waiting up to 60 s ...", flush=True)
        t_end = time.time() + 60
        while time.time() < t_end:
            new = [p for p in lp.comports() if p.device not in before]
            if new:
                for p in new:
                    print(f"board {a.identify} = {p.device}  ({p.description}, VID:PID {p.vid and hex(p.vid)}:{p.pid and hex(p.pid)})")
                    print(f"Write it on a label: '{a.identify} = {p.device}'. Windows keeps the same COM number for the same board on the same USB socket.")
                break
            time.sleep(0.5)
        else:
            print("No new COM port appeared. Check: a DATA cable (many USB-C cables are charge-only), the board's UART/USB socket, "
                  "and the USB-serial driver (CH343 / CP210x) in Device Manager under 'Ports (COM & LPT)'.")
        raise SystemExit(0)
    if a.ports or not a.hello:
        print("\n".join(list_ports()) or "no COM ports")
    for port in a.hello or []:
        link, par = SerialLink(port), Parser()
        t_end, seen = time.time() + 3, None
        while time.time() < t_end and not seen:
            for k, p in par.feed(link.read()):
                if k == "H":
                    seen = p.decode(errors="replace")
            time.sleep(0.01)
        link.close()
        print(f"{port}: {seen or 'no hello in 3 s'}")
