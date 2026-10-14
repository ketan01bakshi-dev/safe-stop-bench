"""P5 stage 1: the replay plan, i.e. what the bridge streams to the chassis, built from a recorded run.

    from vehicle import p5_runs, replay
    runs = p5_runs.run_variants()
    plan = replay.build_plan(runs["no_latch (seeded bug)"]["p5_planned_stop_link_lost_300ms"], p5_runs.scenarios()[...], variant="no_latch (seeded bug)")

The plan is the trace (t, speed, lateral, state, x every 20 ms) sampled at the setpoint rate (sample and hold) and mapped to PWM with the
labelled PLACEHOLDER scale of vehicle/p5_runs.py. It is JOINED with what the trace alone does not carry: the scenario's inject time, the result's
state changes and events, and the expected outcome, so a clip or a manifest can say what happened and why. No chassis measurement enters it.
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field

from vehicle import p5_runs

SETPOINT_PERIOD_MS = 50          # 20 Hz


@dataclass(frozen=True)
class Plan:
    key: str
    variant: str
    dut: str
    seed: int
    k: float
    period_ms: int
    steps: tuple[tuple[int, int, int], ...]           # (t_ms, pwm_l, pwm_r), one per period; the last step is held until the end
    duration_ms: int
    source_sha256: str                                 # of the recorded trace, so a manifest ties the replay to the run it came from
    markers: dict = field(default_factory=dict)       # inject / fault times, state changes, events, expected outcome (documentation, never a command)

    @property
    def pwm_max(self) -> int:
        return max(max(s[1], s[2]) for s in self.steps)

    def setpoint(self, t_ms: int) -> tuple[int, int]:
        i = min(max(t_ms, 0) // self.period_ms, len(self.steps) - 1)
        return self.steps[i][1], self.steps[i][2]

    def header(self) -> dict:
        """What a manifest records about the plan (no steps)."""
        return {"key": self.key, "variant": self.variant, "dut": self.dut, "seed": self.seed, "k_placeholder": self.k,
                "period_ms": self.period_ms, "steps": len(self.steps), "duration_ms": self.duration_ms, "pwm_max": self.pwm_max,
                "source_sha256": self.source_sha256, "markers": self.markers}


def trace_sha256(trace: list) -> str:
    return hashlib.sha256(json.dumps([list(row) for row in trace], separators=(",", ":")).encode()).hexdigest()


def build_plan(result: dict, scenario: dict, *, variant: str = "", k: float = p5_runs.K_PLACEHOLDER,
               period_ms: int = SETPOINT_PERIOD_MS) -> Plan:
    trace = result["trace"]
    end_t = trace[-1][0]
    steps: list[tuple[int, int, int]] = []
    j = 0
    for t in range(0, end_t + 1, period_ms):
        while j + 1 < len(trace) and trace[j + 1][0] <= t:                             # sample and hold: the last trace row at or before t
            j += 1
        pwm = p5_runs.chassis_pwm(trace[j][1], k)
        steps.append((t, pwm, pwm))
    return Plan(key=result["key"], variant=variant, dut=result["dut"], seed=result["seed"], k=k, period_ms=period_ms, steps=tuple(steps),
                duration_ms=end_t, source_sha256=trace_sha256(trace),
                markers={"inject_ms": scenario.get("inject_ms"), "faults": scenario.get("faults"), "t_fault_ms": result.get("t_fault"),
                         "states": [list(s) for s in result["states"]], "events": [list(e) for e in result["events"]],
                         "expect": scenario.get("expect"), "verdict_passed": result["verdict"]["passed"]})


def suggest_cap(plan: Plan, margin: int = 5) -> int:
    """A cap just above the plan's peak (the next multiple of `margin`), never above 255. The chassis latches the stop on anything over it."""
    return min(255, (plan.pwm_max // margin + 1) * margin)
