"""Real-time hygiene for a PC bench on Windows: high process priority and a 1 ms timer resolution.

Measured 2026-10-03: without it, the bench stalled for up to ~94 ms in real-time runs after the laptop woke from
sleep, and the lag guard (oracle: "bench kept real time") failed the runs. A stall like that delays planner frames,
so the DUT could react to the BENCH's fault.

Linux / WSL (v2.8, the ROS 2 adapter): pin the bench to its own cores (the DUT process gets the others, see
DUT_CPUS) and stop the cyclic garbage collector during real-time runs (reference counting still frees memory;
only reference cycles wait until gc is re-enabled).
"""
from __future__ import annotations

import sys

_done = False


def _cpus() -> list[int]:
    import os
    return sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else []


def bench_cpus() -> set[int]:
    c = _cpus()
    return set(c[: max(1, len(c) // 2)]) if len(c) >= 2 else set(c)


def dut_cpus() -> set[int]:
    c = _cpus()
    return set(c[len(c) // 2:]) if len(c) >= 2 else set(c)


def boost() -> str:
    global _done
    if _done:
        return "already set"
    if sys.platform != "win32":
        import gc
        import os
        msg = "gc off"
        gc.disable()
        if hasattr(os, "sched_setaffinity") and len(_cpus()) >= 2:
            os.sched_setaffinity(0, bench_cpus())
            msg += f", bench on CPUs {sorted(bench_cpus())}"
        _done = True
        return msg
    import ctypes
    k32, winmm = ctypes.windll.kernel32, ctypes.windll.winmm
    HIGH_PRIORITY_CLASS = 0x00000080
    ok_prio = bool(k32.SetPriorityClass(k32.GetCurrentProcess(), HIGH_PRIORITY_CLASS))
    ok_timer = winmm.timeBeginPeriod(1) == 0
    _done = True
    return f"priority high: {ok_prio}, 1 ms timer: {ok_timer}"
