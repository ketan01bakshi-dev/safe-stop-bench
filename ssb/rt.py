"""Real-time hygiene for a PC bench on Windows: high process priority and a 1 ms timer resolution.

Measured 2026-10-03: without it, the bench stalled for up to ~94 ms in real-time runs after the laptop woke from
sleep, and the lag guard (oracle: "bench kept real time") failed the runs. A stall like that delays planner frames,
so the DUT could react to the BENCH's fault. No-op on other platforms.
"""
from __future__ import annotations

import sys

_done = False


def boost() -> str:
    global _done
    if _done or sys.platform != "win32":
        return "not needed" if sys.platform != "win32" else "already set"
    import ctypes
    k32, winmm = ctypes.windll.kernel32, ctypes.windll.winmm
    HIGH_PRIORITY_CLASS = 0x00000080
    ok_prio = bool(k32.SetPriorityClass(k32.GetCurrentProcess(), HIGH_PRIORITY_CLASS))
    ok_timer = winmm.timeBeginPeriod(1) == 0
    _done = True
    return f"priority high: {ok_prio}, 1 ms timer: {ok_timer}"
