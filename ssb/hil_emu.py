"""Board emulator for HiL dry runs: board B's node logic (host build of the firmware core) in its OWN process with its
own free-running clock, reachable like a COM port over a local TCP socket (pyserial opens "socket://127.0.0.1:PORT").
Optionally also emulates board A, which forwards every CAN frame B sends.

    python -m ssb.hil_emu --port-b 7777 [--port-a 7778]
    run.py --dut pil --port-b EMU          # starts this automatically
    run.py --dut hil --port-b EMU --port-a EMU

Why a separate process: a thread inside the bench fights the bench's 1 ms busy-wait for Python's GIL and runs late,
which looks exactly like a slow MCU. A process has its own interpreter, like a real board has its own CPU.
"""
from __future__ import annotations

import argparse
import socket
import time

from .hil import Parser, frame_msg
from .native import LoopbackLink


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
    node, par = LoopbackLink(), Parser()
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
            try:
                data = conns[role].recv(65536)
                if not data:
                    raise ConnectionError
                if role == "b":
                    node.write(data, now)
            except BlockingIOError:
                pass
            except (ConnectionError, OSError):
                conns.pop(role).close()
                if role == "b":
                    node.close()
                    node, par = LoopbackLink(), Parser()   # a new connection = a freshly booted board
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
                    k = conns[role].send(pending[role])
                    del pending[role][:k]
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
