"""Board emulator for HiL dry runs: board B's node logic (host build of the firmware core) in its OWN process with its
own free-running clock, reachable like a COM port over a local TCP socket (pyserial opens "socket://127.0.0.1:PORT").
Optionally also emulates board A, which forwards every CAN frame B sends.

    python -m ssb.hil_emu --port-b 7777 [--port-a 7778]
    run.py --dut pil --port-b EMU          # starts this automatically
    run.py --dut hil --port-b EMU --port-a EMU

Why a separate process: a thread inside the bench fights the bench's 1 ms busy-wait for Python's GIL and runs late,
which looks exactly like a slow MCU. A process has its own interpreter, like a real board has its own CPU.

v2.12, power cut: 'X' <ms:u16> sent to emulated board A opens its relay, as BusNode 2.7 does on the board. Board B then
loses power: it sends nothing, its link drops (on the board its USB-serial chip loses power and the COM port vanishes),
and after the cut plus BOOT_MS a fresh node boots from the configuration it had stored, as the firmware does from NVS.
"""
from __future__ import annotations

import argparse
import socket
import struct
import time

from .hil import Parser, frame_msg
from .native import LoopbackLink

BOOT_MS = 186   # board B's measured boot after a reset (config dut_hw.reset_boot_ms); assumed the same after power-on


def serve(port_b: int, port_a: int | None) -> None:
    from .rt import boost
    boost()
    listeners = {}
    for role, port in (("b", port_b), ("a", port_a)):
        if port:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("127.0.0.1", port))
            s.listen(1)
            s.setblocking(False)
            listeners[role] = s
    conns: dict[str, socket.socket] = {}
    node: LoopbackLink | None = LoopbackLink()
    par = Parser()
    par_a = Parser()                  # commands the bench sends to board A ('K' kicks, 'X' power cut)
    boot_at = None                    # board B unpowered (then booting) until this time
    stored = b""                      # B's configuration "in NVS"
    t0 = time.perf_counter()
    last_hello_a = -10000
    pending = {"a": bytearray(), "b": bytearray()}
    dropped = {"a": 0, "b": 0}
    print(f"emulator ready: B on {port_b}" + (f", A on {port_a}" if port_a else ""), flush=True)
    while True:
        now = int((time.perf_counter() - t0) * 1000)   # the board's own millis()
        for role, s in listeners.items():
            if role not in conns:
                try:
                    c, _ = s.accept()
                    c.setblocking(False)
                    c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    conns[role] = c
                except BlockingIOError:
                    pass
        for role in list(conns):
            if role not in conns:   # B's link was just closed by a power cut handled for A in this same pass
                continue
            try:
                data = conns[role].recv(65536)
                if not data:
                    raise ConnectionError
                if role == "b" and node is not None:
                    node.write(data, now)
                elif role == "a":
                    for k, p in par_a.feed(data):
                        if k == "X" and len(p) >= 2 and node is not None:   # relay opens: B loses power
                            stored = node.config() or stored
                            node.close()
                            node, par = None, Parser()
                            boot_at = now + struct.unpack_from("<H", p)[0] + BOOT_MS
                            if "b" in conns:
                                conns.pop("b").close()   # B's USB-serial chip is unpowered: the port vanishes
            except BlockingIOError:
                pass
            except (ConnectionError, OSError):
                conns.pop(role).close()
                if role == "b" and node is not None:
                    node.close()
                    node, par = LoopbackLink(), Parser()   # a new connection = a freshly booted board
        if node is None and boot_at is not None and now >= boot_at:
            node, par = LoopbackLink(), Parser()             # power is back and B has booted: from its stored config
            if stored:
                node.restore(stored, now)
            boot_at = None
        out = b""
        if node is not None:   # unpowered: B sends nothing (board A carries on)
            node.poll(now)
            out = node.read()
        if out:
            pending["b"] += out
            fwd = b"".join(frame_msg("M", p) for k, p in par.feed(out) if k == "M")
            if fwd:
                pending["a"] += fwd
        if now - last_hello_a >= 1000:
            last_hello_a = now
            pending["a"] += frame_msg("H", b"BusNode emulated (A)")
        for role in ("b", "a"):
            if role not in conns:
                pending[role].clear()
                continue
            if len(pending[role]) > 256 * 1024:   # the bench isn't reading (between scenarios): drop, as a UART would
                dropped[role] += len(pending[role])
                pending[role].clear()
                continue
            if pending[role]:
                try:
                    n_sent = conns[role].send(pending[role])
                    del pending[role][:n_sent]
                except BlockingIOError:
                    pass
                except OSError:
                    conns.pop(role).close()
                    pending[role].clear()
        time.sleep(0)   # yield the core without a timed sleep (Windows sleep granularity ~15 ms would make a 10 ms task late)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port-b", type=int, required=True)
    ap.add_argument("--port-a", type=int)
    a = ap.parse_args()
    serve(a.port_b, a.port_a)
