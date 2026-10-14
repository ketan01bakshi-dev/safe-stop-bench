"""P5 stage 1: the bridge, i.e. the laptop side of the chassis link.

Bridge is sans-I/O like ChassisCore: step(now_ms, received_datagrams) returns the datagrams to send. run_udp() puts it on a real socket in a
separate process-style loop with a STOP in a `finally`; vehicle/simnet.py drives it on a virtual clock.

    WAIT_HELLO -> ARMING -> RUNNING -> ENDING -> DONE            outcome 'COMPLETE' or 'ABORTED:<why>'

The bridge refuses to arm a chassis that is not DISARMED or runs an unexpected build, streams SETPOINTs at the plan's rate, and keeps its own
watchdog: no ECHO for `watchdog_ms` (100 ms) aborts the run and sends STOP(WATCHDOG). Any abort ends the take as INCONCLUSIVE, never as a result.

    python -m vehicle.bridge --sim --scenario p5_planned_stop_link_lost_300ms --variant no_latch     # virtual clock, instant, simulated chassis
    python -m vehicle.bridge --fake --scenario p5_planned_stop_link_lost                              # real time, UDP loopback, fake chassis
"""
from __future__ import annotations

import argparse
import json
import secrets
import socket
import sys
import time
from collections.abc import Callable, Sequence
from pathlib import Path

from vehicle import link, p5_runs, replay
from vehicle.link import Reject, State, StopReason

SETPOINT_PERIOD_MS = replay.SETPOINT_PERIOD_MS
WATCHDOG_MS = 100
ARM_RETRY_MS = 200
STOP_REPEATS = 3                       # UDP: every STOP is sent this many times (end of run: one setpoint period apart; abort: back to back)


def _enum(cls, value):
    """cls(value), or the member with value 0 when the chassis sent a code this version does not know."""
    try:
        return cls(value)
    except ValueError:
        return cls(0)


class Bridge:
    def __init__(self, plan: replay.Plan, *, run_id: int, nonce: int, pwm_cap: int, expect_builds: Sequence[str] | None = None,
                 watchdog_ms: int = WATCHDOG_MS, hello_timeout_ms: int = 5000, arm_timeout_ms: int = 1500, end_timeout_ms: int = 1500):
        if pwm_cap < plan.pwm_max:
            raise ValueError(f"pwm_cap {pwm_cap} is below the plan's peak {plan.pwm_max}: the chassis would latch the stop")
        self.plan, self.run_id, self.nonce, self.pwm_cap = plan, run_id, nonce, pwm_cap
        self.expect_builds = tuple(expect_builds) if expect_builds is not None else None
        self.watchdog_ms, self.hello_timeout_ms, self.arm_timeout_ms, self.end_timeout_ms = watchdog_ms, hello_timeout_ms, arm_timeout_ms, end_timeout_ms
        self.state = "WAIT_HELLO"
        self.outcome: str | None = None
        self.boot_id: int | None = None
        self.chassis_build: str | None = None
        self.log: list[dict] = []                                  # every datagram and decision, with the bridge's own clock
        self.seq = 0
        self.sent_setpoints: dict[int, tuple[int, int]] = {}      # seq -> (pwm_l, pwm_r) for the tracking check
        self.stats = {"setpoints": 0, "echoes": 0, "late_ticks": 0, "max_echo_gap_ms": 0, "max_seq_lag": 0, "max_pwm_err": 0, "arm_sent": 0, "enc_mask": 0}
        self.enc_last = [0, 0, 0, 0]                               # the last wheel counts echoed (FL, FR, RL, RR)
        self._t_begin: int | None = None
        self._t_state = 0
        self._t0 = 0                                               # now_ms at which the chassis confirmed ARMED (plan time 0)
        self._next_send = 0
        self._next_arm = 0
        self._last_echo = 0
        self._stops_left = 0
        self._next_stop = 0
        self._stop_reason = StopReason.END_OF_RUN
        self._last_reject = Reject.NONE

    # --- bookkeeping ----------------------------------------------------------------------------------------------------------------

    def _note(self, now: int, what: str, **kw) -> None:
        self.log.append({"t_ms": now, "note": what, **kw})

    def _tx(self, now: int, msg: link.Message) -> bytes:
        self.log.append({"t_ms": now, "dir": "tx", **link.describe(msg)})
        return link.encode(msg)

    def _finish(self, now: int, outcome: str) -> None:
        self.state, self.outcome = "DONE", outcome
        self._note(now, "finished", outcome=outcome)

    def stop_frames(self, now: int) -> list[bytes]:
        """STOP(ABORT) or STOP(WATCHDOG), STOP_REPEATS times back to back (UDP). Idempotent on the chassis, so it is safe to send again."""
        reason = StopReason.WATCHDOG if (self.outcome or "").endswith("WATCHDOG") else StopReason.ABORT
        return [self._tx(now, link.Stop(self.run_id, self.nonce, reason)) for _ in range(STOP_REPEATS)]

    def abort(self, why: str, now: int) -> list[bytes]:
        """End the run now and return the STOPs to send. A finished run (COMPLETE or already aborted) sends nothing more."""
        if self.outcome is not None:
            return []
        self._finish(now, f"ABORTED:{why}")
        return self.stop_frames(now)

    # --- the step -------------------------------------------------------------------------------------------------------------------

    def step(self, now: int, rx: Sequence[bytes] = ()) -> list[bytes]:
        if self.state == "DONE":
            return []
        if self._t_begin is None:
            self._t_begin = self._t_state = now
        out: list[bytes] = []
        for data in rx:
            try:
                msg = link.decode(data)
            except link.LinkError as e:
                self._note(now, "bad frame from the chassis", error=str(e))
                continue
            self.log.append({"t_ms": now, "dir": "rx", **link.describe(msg)})
            out += self._on_message(msg, now)
            if self.state == "DONE":
                return out
        if self.state == "WAIT_HELLO":
            if now - self._t_state > self.hello_timeout_ms:
                return out + self.abort("NO_CHASSIS", now)
        elif self.state == "ARMING":
            if now - self._t_state > self.arm_timeout_ms:
                return out + self.abort(f"ARM_TIMEOUT(last reject {self._last_reject.name})", now)
            if now >= self._next_arm:
                out.append(self._tx(now, link.Arm(self.boot_id or 0, self.run_id, self.nonce, self.pwm_cap)))
                self.stats["arm_sent"] += 1
                self._next_arm = now + ARM_RETRY_MS
        elif self.state == "RUNNING":
            out += self._running(now)
        elif self.state == "ENDING":
            out += self._ending(now)
        return out

    def _on_message(self, msg: link.Message, now: int) -> list[bytes]:
        if isinstance(msg, link.Hello):
            if self.boot_id is not None and msg.boot_id != self.boot_id:
                return self.abort("CHASSIS_REBOOTED", now)
            self._last_reject = _enum(Reject, msg.reject)
            if self.state == "WAIT_HELLO":
                if self.expect_builds is not None and msg.build not in self.expect_builds:
                    return self.abort(f"UNEXPECTED_BUILD({msg.build})", now)
                if msg.state != State.DISARMED:
                    return self.abort(f"CHASSIS_{_enum(State, msg.state).name}({_enum(link.Reason, msg.reason).name})", now)
                if msg.last_run_id >= self.run_id:
                    return self.abort(f"RUN_ID_NOT_NEWER({msg.last_run_id})", now)
                self.boot_id, self.chassis_build = msg.boot_id, msg.build
                self.state, self._t_state, self._next_arm = "ARMING", now, now
            elif self.state == "ENDING" and msg.state == State.DISARMED:
                self._finish(now, "COMPLETE")
            elif self.state == "RUNNING" and msg.state != State.ARMED:
                return self.abort(f"CHASSIS_{_enum(State, msg.state).name}", now)
            return []
        if not isinstance(msg, link.Echo):
            return []
        if (msg.boot_id, msg.run_id, msg.nonce) != (self.boot_id, self.run_id, self.nonce):
            return []                                              # an old run's echo: not ours, not a sign of life for this run
        self._last_echo = now
        self.stats["echoes"] += 1
        if self.state == "ARMING" and msg.state == State.ARMED:
            self.state, self._t_state, self._t0, self._next_send, self._last_echo = "RUNNING", now, now, now, now
            self._note(now, "chassis armed; plan time 0")
            return []
        if self.state in ("RUNNING", "ENDING"):
            if msg.state != State.ARMED:
                if self.state == "ENDING" and msg.state == State.DISARMED:
                    self._finish(now, "COMPLETE")
                    return []
                return self.abort(f"CHASSIS_{_enum(State, msg.state).name}({_enum(link.Reason, msg.reason).name})", now)
            self._track(msg)
        return []

    def _track(self, echo: link.Echo) -> None:
        """How far behind the chassis is, and whether what it APPLIED is what was sent for the setpoint it last accepted. A witness, not a verdict."""
        self.stats["enc_mask"] = echo.enc_mask                     # the encoders are a witness: recorded, never judged
        self.enc_last = [echo.enc_fl, echo.enc_fr, echo.enc_rl, echo.enc_rr]
        self.stats["max_seq_lag"] = max(self.stats["max_seq_lag"], self.seq - echo.seq)
        sent = self.sent_setpoints.get(echo.seq)
        if sent is not None and echo.seq > 0:
            # the pins follow the target only at the next writer tick, so a fresh setpoint may not be applied yet: compare with the one before too
            prev = self.sent_setpoints.get(echo.seq - 1, sent)
            err = min(max(abs(echo.pwm_l - sent[0]), abs(echo.pwm_r - sent[1])), max(abs(echo.pwm_l - prev[0]), abs(echo.pwm_r - prev[1])))
            self.stats["max_pwm_err"] = max(self.stats["max_pwm_err"], err)

    def _running(self, now: int) -> list[bytes]:
        gap = now - self._last_echo
        self.stats["max_echo_gap_ms"] = max(self.stats["max_echo_gap_ms"], gap)
        if gap > self.watchdog_ms:
            return self.abort("WATCHDOG", now)
        t_run = now - self._t0
        if t_run >= self.plan.duration_ms:
            self.state, self._t_state = "ENDING", now
            self._stops_left, self._next_stop, self._stop_reason = STOP_REPEATS, now, StopReason.END_OF_RUN
            self._note(now, "plan finished: ending the run")
            return self._ending(now)
        out: list[bytes] = []
        if now >= self._next_send:
            self.seq += 1
            pwm_l, pwm_r = self.plan.setpoint(t_run)
            self.sent_setpoints[self.seq] = (pwm_l, pwm_r)
            out.append(self._tx(now, link.Setpoint(self.run_id, self.nonce, self.seq, t_run, pwm_l, pwm_r)))
            self.stats["setpoints"] += 1
            self._next_send += SETPOINT_PERIOD_MS
            while self._next_send <= now:                          # the loop ran late: skip ahead, never burst to catch up
                self._next_send += SETPOINT_PERIOD_MS
                self.stats["late_ticks"] += 1
        return out

    def _ending(self, now: int) -> list[bytes]:
        if now - self._t_state > self.end_timeout_ms + STOP_REPEATS * SETPOINT_PERIOD_MS:
            return self.abort("END_NOT_CONFIRMED", now)
        out: list[bytes] = []
        if self._stops_left > 0 and now >= self._next_stop:
            out.append(self._tx(now, link.Stop(self.run_id, self.nonce, self._stop_reason)))
            self._stops_left -= 1
            self._next_stop = now + SETPOINT_PERIOD_MS
        return out

    def summary(self) -> dict:
        return {"outcome": self.outcome, "state": self.state, "chassis_build": self.chassis_build, "boot_id": self.boot_id,
                "run_id": self.run_id, "pwm_cap": self.pwm_cap, "watchdog_ms": self.watchdog_ms, **self.stats,
                "enc_last": list(self.enc_last)}


# --- the real-time runner ---------------------------------------------------------------------------------------------------------------

def run_udp(bridge: Bridge, sock: socket.socket, *, chassis: tuple[str, int] | None = None, tick_s: float = 0.001,
            now_fn: Callable[[], int] | None = None, max_s: float = 120.0) -> Bridge:
    """Drive `bridge` on `sock` in real time. The chassis address is learned from the first HELLO (and must equal `chassis` if given).

    Whatever happens inside the loop (an exception, Ctrl-C, a timeout), the `finally` sends STOP(ABORT) unless the run completed.
    """
    t0 = time.perf_counter()
    clock = now_fn or (lambda: int((time.perf_counter() - t0) * 1000))
    peer: tuple[str, int] | None = None
    sock.settimeout(tick_s)

    def send(frames: list[bytes]) -> None:
        if peer is None:
            return
        for f in frames:
            try:
                sock.sendto(f, peer)
            except OSError:
                pass

    try:
        while bridge.state != "DONE":
            now = clock()
            if now > max_s * 1000:
                send(bridge.abort("RUN_TIMEOUT", now))
                break
            rx: list[bytes] = []
            try:
                data, src = sock.recvfrom(256)
                if peer is None and (chassis is None or src == chassis):
                    try:
                        if isinstance(link.decode(data), link.Hello):
                            peer = src
                    except link.LinkError:
                        pass
                if src == peer:
                    rx.append(data)
            except (TimeoutError, ConnectionResetError):
                pass
            send(bridge.step(clock(), rx))
    finally:
        if bridge.outcome != "COMPLETE":
            if bridge.outcome is None:
                bridge.abort("INTERRUPTED", clock())
            send(bridge.stop_frames(clock()))                      # always, even after an earlier abort: a STOP is idempotent
    return bridge


# --- command line -----------------------------------------------------------------------------------------------------------------------

def _plan_from_args(args: argparse.Namespace) -> replay.Plan:
    runs = p5_runs.run_variants()
    scen = p5_runs.scenarios()
    if args.scenario not in scen:
        raise SystemExit(f"unknown scenario {args.scenario!r}; choose from {sorted(scen)}")
    names = [v for v in runs if v.startswith(args.variant)] if args.variant else ["correct build"]
    if len(names) != 1:
        raise SystemExit(f"--variant must match exactly one of {list(runs)}")
    return replay.build_plan(runs[names[0]][args.scenario], scen[args.scenario], variant=names[0], k=args.k)


def _is_loopback(host: str) -> bool:
    return host in ("127.0.0.1", "::1", "localhost")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m vehicle.bridge", description=__doc__.split("\n\n")[0])
    ap.add_argument("--scenario", default="p5_planned_stop_link_lost")
    ap.add_argument("--variant", default="", help="prefix of: correct build | no_latch | long_timeout (default: correct build)")
    ap.add_argument("--k", type=float, default=p5_runs.K_PLACEHOLDER, help="1:k scale (PLACEHOLDER until the stage 3 calibration)")
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sim", action="store_true", help="virtual clock, simulated chassis, instant")
    mode.add_argument("--fake", action="store_true", help="real time over UDP loopback against the fake chassis (about the plan's length)")
    mode.add_argument("--listen", metavar="HOST:PORT", help="real chassis: listen here for its HELLO (stage 2 and later; needs --allow-real off loopback)")
    ap.add_argument("--chassis", metavar="HOST:PORT", help="accept HELLO only from this address")
    ap.add_argument("--allow-real", action="store_true", help="I confirm a real chassis, WHEELS UP, e-stop in reach (stage 2); required off loopback")
    ap.add_argument("--cap", type=int, default=0, help="PWM cap sent in ARM (default: just above the plan's peak)")
    ap.add_argument("--watchdog-ms", type=int, default=WATCHDOG_MS)
    ap.add_argument("--out", default="", help="folder for bridge_log.jsonl and manifest.json")
    args = ap.parse_args(argv)

    if args.listen:
        host = args.listen.rpartition(":")[0]
        peer_host = args.chassis.rpartition(":")[0] if args.chassis else host
        if not (_is_loopback(host) and _is_loopback(peer_host)) and not args.allow_real:
            raise SystemExit("refusing: a non-loopback chassis needs --allow-real (wheels up, e-stop in reach; stage 2 of docs/P5_WITNESS_PLAN.md)")

    plan = _plan_from_args(args)
    cap = args.cap or replay.suggest_cap(plan)
    run_id = int(time.time()) & 0x7FFFFFFF                           # newer than any earlier run of this laptop
    nonce = secrets.randbits(32)
    bridge = Bridge(plan, run_id=run_id, nonce=nonce, pwm_cap=cap, watchdog_ms=args.watchdog_ms)
    kind = "virtual" if args.sim else "fake-udp" if args.fake else "real"
    chassis_summary: dict | None = None

    if args.sim:
        from vehicle import simnet
        res = simnet.simulate(plan, bridge=bridge)
        chassis_summary = res.chassis.summary()
    else:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if args.fake:
            from vehicle.fake_chassis import ChassisCore, UdpChassis
            sock.bind(("127.0.0.1", 0))
            core = ChassisCore(secrets.randbits(32), build="p5-fake", hello_peer=sock.getsockname())
            with UdpChassis(core, sock.getsockname()) as fake:
                run_udp(bridge, sock)
                time.sleep(0.05)                                         # the chassis pins follow at its next writer tick: report the settled state
                chassis_summary = fake.snapshot()
        else:
            host, _, port = args.listen.rpartition(":")
            peer = None
            if args.chassis:
                ph, _, pp = args.chassis.rpartition(":")
                peer = (ph, int(pp))
            sock.bind((host, int(port)))
            run_udp(bridge, sock, chassis=peer)
        sock.close()

    summary = {"kind": kind, **bridge.summary(), "chassis": chassis_summary}
    print(f"{kind} chassis, plan {plan.key} / {plan.variant or 'correct build'}: {bridge.outcome}")
    print(json.dumps({k: v for k, v in summary.items() if k != "chassis"}, indent=1))
    if chassis_summary:
        print("chassis:", json.dumps(chassis_summary))
    if args.out:
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        (out / "bridge_log.jsonl").write_text("\n".join(json.dumps(e) for e in bridge.log) + "\n", encoding="utf-8")
        manifest = {"label": f"{kind} chassis: not a physical result" if kind != "real" else "real chassis, open-loop, calibrated by video",
                    "plan": plan.header(), "summary": summary}
        (out / "manifest.json").write_text(json.dumps(manifest, indent=1), encoding="utf-8")
    return 0 if bridge.outcome == "COMPLETE" else 1


if __name__ == "__main__":
    sys.exit(main())
