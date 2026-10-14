"""P5 stage 1: the bridge and the chassis core on a virtual clock (1 ms steps, no sockets, no sleeping).

A 14 s replay runs in a fraction of a second and every timing is exact, so the link contract (dead-man, watchdog, latch, re-arm) is tested here
and the real-time UDP run is only a smoke test of the same code. Faults are scripted: a window in which datagrams are dropped, a bridge that dies
without sending STOP, a Wi-Fi drop on the chassis, a physical re-arm, a stale or foreign datagram injected on the wire.

    res = simnet.simulate(plan, drop=[("bridge->chassis", 6000, 6300)])
    res.bridge.outcome, res.chassis.state_log, res.chassis.out_log
"""
from __future__ import annotations

import heapq
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field

from vehicle import replay
from vehicle.bridge import Bridge
from vehicle.fake_chassis import ChassisCore
from vehicle.link import State

BRIDGE, CHASSIS = "bridge", "chassis"


@dataclass
class SimResult:
    bridge: Bridge
    chassis: ChassisCore
    t_end: int
    wire: list[tuple[int, str, bytes]] = field(default_factory=list)     # (t, 'bridge->chassis' | 'chassis->bridge', frame) as sent


def simulate(plan: replay.Plan, *, bridge: Bridge | None = None, chassis: ChassisCore | None = None, latency_ms: int = 2,
             drop: Sequence[tuple[str, int, int]] = (), drop_if: Callable[[int, str, bytes], bool] | None = None,
             bridge_dies_at: int | None = None, link_down_at: int | None = None, rearm_at: int | None = None,
             inject: Sequence[tuple[int, str, bytes, str]] = (), until_ms: int | None = None, from_ms: int = 0,
             start_ms: int | None = None) -> SimResult:
    """Run to the end of the bridge's job (or `until_ms`).

    drop         (direction, t0, t1): datagrams SENT in [t0, t1) on that direction are lost.
    bridge_dies_at   the bridge process vanishes at that time: no more steps, no STOP.
    link_down_at / rearm_at   the chassis's Wi-Fi drops / its physical re-arm control is pressed.
    inject       (t, direction, frame, src) a datagram delivered at t (e.g. a stale frame from an earlier run, or one from another host).
    from_ms / start_ms   the virtual clock starts at from_ms (continue a chassis that already ran: its clock must not go back); the bridge
                 starts at start_ms (default from_ms + 100). until_ms is absolute.
    """
    start_ms = from_ms + 100 if start_ms is None else start_ms
    if chassis is None:
        chassis = ChassisCore(0x5EED0001, build="p5-sim", hello_peer=BRIDGE)
    if bridge is None:
        bridge = Bridge(plan, run_id=1000, nonce=0xA5A5A5A5, pwm_cap=replay.suggest_cap(plan))
    res = SimResult(bridge, chassis, 0)
    queue: list[tuple[int, int, str, str, bytes]] = []                      # (due, order, to, src, frame)
    order = 0
    for t, direction, frame, src in inject:
        order += 1
        heapq.heappush(queue, (t, order, CHASSIS if direction.endswith(CHASSIS) else BRIDGE, src, frame))
    limit = until_ms if until_ms is not None else start_ms + plan.duration_ms + 30000

    def send(now: int, direction: str, frames: Sequence[bytes], src: str) -> None:
        nonlocal order
        for f in frames:
            res.wire.append((now, direction, f))
            if any(d == direction and a <= now < b for d, a, b in drop) or (drop_if and drop_if(now, direction, f)):
                continue
            order += 1
            heapq.heappush(queue, (now + latency_ms, order, CHASSIS if direction.endswith(CHASSIS) else BRIDGE, src, f))

    bridge_alive = True
    for now in range(from_ms, limit):
        if link_down_at is not None and now == link_down_at:
            chassis.link_down(now)
        if rearm_at is not None and now == rearm_at:
            chassis.physical_rearm(now)
        if bridge_dies_at is not None and now == bridge_dies_at:
            bridge_alive = False
        to_chassis: list[tuple[str, bytes]] = []
        to_bridge: list[bytes] = []
        while queue and queue[0][0] <= now:
            _due, _o, to, src, frame = heapq.heappop(queue)
            (to_chassis.append((src, frame)) if to == CHASSIS else to_bridge.append(frame))
        for src, frame in to_chassis:
            for _addr, out in chassis.rx(frame, src, now):
                send(now, f"{CHASSIS}->{BRIDGE}", [out], CHASSIS)
        for _addr, out in chassis.step(now):
            send(now, f"{CHASSIS}->{BRIDGE}", [out], CHASSIS)
        if now >= start_ms and bridge_alive:
            frames = bridge.step(now - start_ms, to_bridge)
            send(now, f"{BRIDGE}->{CHASSIS}", frames, BRIDGE)
            if bridge.state == "DONE" and not queue:
                return _finish(res, now)
        elif not bridge_alive and chassis.state is not State.ARMED and now > (bridge_dies_at or 0) + 500 and not queue:
            return _finish(res, now)
    res.t_end = limit
    return res


def _finish(res: SimResult, now: int) -> SimResult:
    """Run the chassis alone for two more writer periods, so the pins have followed the last state change."""
    for t in range(now + 1, now + 2 * res.chassis.writer_period_ms + 1):
        res.chassis.step(t)
    res.t_end = now
    return res


def cut_time(chassis: ChassisCore, after_ms: int = 0) -> int | None:
    """The first time at or after `after_ms` at which the pins went to (0, 0) after having been non-zero, or None."""
    nonzero = False
    for t, pwm_l, pwm_r in chassis.out_log:
        if pwm_l or pwm_r:
            nonzero = True
        elif nonzero and t >= after_ms:
            return t
    return None
