"""Virtual CAN / CAN FD bus: priority arbitration, capacity per ms, latency, jitter, reordering, flooding, bus-off.

A behavioural model, not a bit-level one: each millisecond the bus moves up to `frames_per_ms`
ready frames, lowest CAN ID first, then delivers them after latency (+ jitter).
"""
from __future__ import annotations

import random
from dataclasses import dataclass, field


@dataclass(order=True)
class Frame:
    ready_ms: int
    can_id: int
    seq: int
    data: bytes = field(compare=False)
    fd: bool = field(compare=False, default=False)
    arrive_ms: int = field(compare=False, default=0)

    def bits(self) -> int:
        """Rough on-wire size: ~47 bits overhead + 8 per byte (+ stuffing ~15%)."""
        return int((47 + 8 * len(self.data)) * 1.15)


class VirtualBus:
    def __init__(self, name: str, frames_per_ms: int, latency_ms: int = 1, rng: random.Random | None = None):
        self.name = name
        self.capacity = frames_per_ms
        self.latency = latency_ms
        self.jitter = 0
        self.extra_latency = 0
        self.rng = rng or random.Random(0)
        self.queue: list[Frame] = []
        self.in_flight: list[Frame] = []
        self.seq = 0
        self.bus_off: set[str] = set()   # nodes currently bus-off
        self.reorder_next = False
        self.sent_bits = 0
        self.max_load = 0.0
        self.capacity_bits = frames_per_ms * Frame(0, 0, 0, bytes(8)).bits()

    def send(self, t: int, node: str, can_id: int, data: bytes, fd: bool = False) -> bool:
        if node in self.bus_off:
            return False  # a bus-off node can't transmit
        self.seq += 1
        self.queue.append(Frame(t, can_id, self.seq, data, fd))
        return True

    def step(self, t: int) -> list[Frame]:
        ready = sorted(f for f in self.queue if f.ready_ms <= t)
        ready.sort(key=lambda f: (f.can_id, f.seq))   # arbitration: lowest ID wins
        moved = ready[: self.capacity]
        for f in moved:
            self.queue.remove(f)
            f.arrive_ms = t + self.latency + self.extra_latency + (self.rng.randint(0, self.jitter) if self.jitter else 0)
            self.in_flight.append(f)
        bits = sum(f.bits() for f in moved)
        self.sent_bits += bits
        self.max_load = max(self.max_load, bits / self.capacity_bits)
        if self.reorder_next and len(self.in_flight) >= 2:
            a, b = self.in_flight[-2], self.in_flight[-1]
            a.arrive_ms, b.arrive_ms = b.arrive_ms + 1, a.arrive_ms
            self.reorder_next = False
        out = sorted((f for f in self.in_flight if f.arrive_ms <= t), key=lambda f: (f.arrive_ms, f.seq))
        for f in out:
            self.in_flight.remove(f)
        return out
