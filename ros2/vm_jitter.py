"""Baseline: how long can this machine/VM stall a busy-looping process? No ROS, no bench. 10 s, gaps between loop passes."""
import os
import time

if hasattr(os, "sched_setaffinity"):
    os.sched_setaffinity(0, {0})
gaps, t_prev, t_end = [], time.perf_counter(), time.perf_counter() + 10
while (t := time.perf_counter()) < t_end:
    gaps.append(t - t_prev)
    t_prev = t
gaps = sorted(g * 1000 for g in gaps)
n = len(gaps)
print(f"{n} passes; p99.99 {gaps[int(n * .9999)]:.3f} ms, max {gaps[-1]:.2f} ms; gaps > 1 ms: {sum(g > 1 for g in gaps)}, > 5 ms: {sum(g > 5 for g in gaps)}, > 20 ms: {sum(g > 20 for g in gaps)}")
