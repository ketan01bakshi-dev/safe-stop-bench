"""The checker: the reference safety controller (v2).

Checks on the planner command stream: Profile 5 E2E + windowed state machine, command timeout,
freshness (timestamp age), hardware window watchdog, question-and-answer watchdog, speed-dependent
envelope with jerk limit. Checks on the actuators: commanded vs measured steering and braking.
Reaction ladder: NORMAL → DEGRADED (speed cap) → PULL_OVER (planner-executed) → STOP_IN_LANE →
BRAKE_ONLY_STOP / BACKUP_BRAKE_STOP. Stops are latched; release needs an operator and conditions.
Outputs: actuator command frames (Profile 2), state, cause, DTCs, watchdog challenge, MRM request.

`defects` seeds deliberate bugs (mutants) to prove the test suite can catch them.
"""
from __future__ import annotations

import math

from . import e2e
from .planner import CMD_ID, decode_cmd, qa_answer
from .plant import ACT_ID, encode_act

RANK = {"INIT": 0, "NORMAL": 0, "DEGRADED": 1, "PULL_OVER": 2, "STOP_IN_LANE": 3,
        "BRAKE_ONLY_STOP": 4, "BACKUP_BRAKE_STOP": 4}
LATCHED = {"PULL_OVER", "STOP_IN_LANE", "BRAKE_ONLY_STOP", "BACKUP_BRAKE_STOP"}
CYCLE_MS = 10

MUTANTS = {
    "no_latch": "Stop releases itself once the fault clears",
    "no_watchdog": "Hardware window watchdog not implemented",
    "long_timeout": "Command timeout 1000 ms instead of 100 ms",
    "envelope_off_by_one": "Steering-rate limit compared with a 5 deg/s margin (threshold bug)",
    "e2e_lenient": "E2E window tolerates 6 errors instead of 2",
    "e2e_run_length": "E2E judged by '3 bad in a row' (v1 logic) instead of a window",
    "no_freshness": "Timestamp age not checked",
    "no_qa": "Question-and-answer watchdog not checked",
    "no_actuator_check": "Commanded vs measured actuator checks missing",
    "release_unconditional": "Release accepted while moving or with the fault still present",
    "startup_moves": "Leaves INIT without waiting for valid frames",
    "no_speed_cap": "DEGRADED mode doesn't cap speed",
    "no_heading_hold": "Stop in lane centres the steering instead of holding heading from the yaw-rate sensor",
    "no_grade_compensation": "Brake check uses raw measured deceleration (an uphill grade hides a weak brake)",
}


class SafetyController:
    def __init__(self, cfg: dict, defects: frozenset = frozenset(), warm_start: bool = True, data_id: int = 0x1234):
        self.cfg, self.p, self.defects = cfg, dict(cfg["safety"]), set(defects)
        if "long_timeout" in self.defects:
            self.p["cmd_timeout_ms"] = 1000
        # E2E tuning (v2.6): optional keys, defaults = the v2.0 behaviour and the C++ core's constants
        p = self.p
        self.rx = e2e.Profile5.Receiver(data_id, max_delta=p.get("e2e_max_delta", 2), explain_gaps=p.get("e2e_explain_gaps", False))
        sm_kw = {"window": p.get("e2e_window", 6), "max_err_valid": p.get("e2e_max_err_valid", 2)}
        if "e2e_lenient" in self.defects:
            sm_kw["max_err_valid"] = 6
        self.sm = e2e.E2EStateMachine(**sm_kw)
        self.bad_run = 0
        self.state = "NORMAL" if warm_start else "INIT"
        if warm_start:
            self.sm.preset_valid()
        self.cause: str | None = None
        self.t_fault: int | None = None
        self.events: list[tuple[int, str]] = []
        self.dtcs: list[tuple[int, str]] = []
        self.last_valid_ms = 0
        self.valid_since_init = 0
        self.cmd = {"accel": 0.0, "steer": 0.0, "speed_req": 0.0, "perception": 2, "odd_exit": False}
        self.last_steer: float | None = None
        self.last_cmd_t: int | None = None
        self.last_accel = 0.0
        self.env_since: int | None = None
        self.stale_run = 0
        self.last_kick: int | None = 0
        self.early_run = 0
        self.challenges: list[int] = []
        self.challenge = 0x5A
        self.qa_fail_run = 0
        self.steer_mis_since: int | None = None
        self.brake_mis_since: int | None = None
        self.decel_now = 0.0
        self.out = (0.0, 0.0, False)
        self.out_prev_steer = 0.0
        self.act_counter = 0
        self.rejected = 0
        self.fault_active_t = -10**9
        self.healthy_perc_since: int | None = None
        self.release_rejected = 0
        self.powered = True
        self.mrm_request: str | None = None
        self.tx_ok = True
        self.psi_mrm = 0.0   # heading change since the stop began, integrated from the yaw-rate sensor

    # ── helpers ───────────────────────────────────────────────────────────────
    def _log(self, t: int, text: str) -> None:
        self.events.append((t, text))

    def _escalate(self, t: int, state: str, cause: str) -> None:
        self.fault_active_t = t
        if RANK[state] > RANK[self.state] or (self.state == "INIT" and state != "NORMAL"):
            if self.t_fault is None:
                self.t_fault, self.cause = t, cause
            self.dtcs.append((t, cause))
            self._log(t, f"{self.state} → {state} ({cause})")
            self.state = state
            self.mrm_request = "PULL_OVER" if state == "PULL_OVER" else None

    def steer_rate_limit(self, v: float) -> float:
        p = self.p
        lim = max(p["steer_rate_min_dps"], p["steer_rate_max_dps"] - p["steer_rate_slope"] * v * 3.6)
        return lim + (5.0 if "envelope_off_by_one" in self.defects else 0.0)

    # ── inputs ────────────────────────────────────────────────────────────────
    def kick(self, t: int) -> None:
        if "no_watchdog" in self.defects or not self.powered:
            return
        if self.last_kick is not None and t - self.last_kick < self.p["wd_window_min_ms"]:
            self.early_run += 1
            if self.early_run >= self.p["wd_early_kicks"]:
                self._escalate(t, "STOP_IN_LANE", "WATCHDOG_EARLY")
        else:
            self.early_run = 0
        self.last_kick = t

    def release(self, t: int, v: float) -> bool:
        ok_conditions = v == 0.0 and t - self.fault_active_t >= self.p["release_clear_ms"]
        if self.state in LATCHED and ("release_unconditional" in self.defects or ok_conditions):
            self._log(t, "release accepted → INIT")
            self.state, self.valid_since_init, self.mrm_request, self.decel_now = "INIT", 0, None, 0.0
            return True
        self.release_rejected += 1
        self._log(t, "release rejected")
        return False

    def brownout(self, t: int, on: bool) -> None:
        if on and self.powered:
            self.powered = False
            self._log(t, "safety controller power lost (reset)")
        elif not on and not self.powered:
            self.powered = True
            keep = (self.events, self.dtcs, self.release_rejected)
            self.__init__(self.cfg, frozenset(self.defects), warm_start=False)  # type: ignore[misc]  # reset in place, as a power cycle
            self.events, self.dtcs, self.release_rejected = keep
            self.last_kick, self.last_valid_ms = t, t
            self._escalate(t, "STOP_IN_LANE", "SAFETY_RESET")

    # ── one 10 ms cycle ──────────────────────────────────────────────────────
    def cycle(self, t: int, frames: list, fb: dict) -> list[tuple[int, bytes]]:
        if not self.powered:
            return []
        p, v = self.p, fb["v"]
        if t % 20 == 10:
            self.challenge = (self.challenge * 73 + 41) & 0xFF
            self.challenges = (self.challenges + [self.challenge])[-3:]
        for f in frames:
            if f.can_id != CMD_ID:
                continue
            status, payload = self.rx.check(f.data)
            sm_state = self.sm.update(status)
            if status in e2e.ERRORS:
                self.rejected += 1
                self.bad_run += 1
            elif status in e2e.VALID:
                self.bad_run = 0
            e2e_bad = (self.bad_run >= 3) if "e2e_run_length" in self.defects else (sm_state == "INVALID")
            if e2e_bad:
                self._escalate(t, "STOP_IN_LANE", "E2E_INVALID")
            if status not in e2e.VALID or sm_state not in ("VALID", "INIT"):
                continue
            assert payload is not None   # VALID status always carries the payload
            c = decode_cmd(payload)
            age = (t - c["t_stamp"]) % 65536
            if "no_freshness" not in self.defects and age > p["max_age_ms"]:
                self.stale_run += 1
                if self.stale_run >= 3:
                    self._escalate(t, "STOP_IN_LANE", "STALE_DATA")
                continue
            self.stale_run = 0
            if "no_qa" not in self.defects:
                if c["qa"] in [qa_answer(ch) for ch in self.challenges] or not self.challenges:
                    self.qa_fail_run = 0
                else:
                    self.qa_fail_run += 1
                    if self.qa_fail_run >= 3:
                        self._escalate(t, "STOP_IN_LANE", "WATCHDOG_QA")
            self._envelope(t, c, v)
            self.cmd, self.last_valid_ms = c, t
            self.valid_since_init += 1

        if t - self.last_valid_ms > p["cmd_timeout_ms"]:
            self._escalate(t, "STOP_IN_LANE", "TIMEOUT")
        if "no_watchdog" not in self.defects and self.last_kick is not None and t - self.last_kick > p["wd_window_max_ms"]:
            self._escalate(t, "STOP_IN_LANE", "WATCHDOG_LATE")
        if not self.tx_ok:
            self._escalate(t, "STOP_IN_LANE", "ACT_BUS_OFF")
        self._perception(t)
        if "no_actuator_check" not in self.defects:
            self._actuator_checks(t, fb)
        if self.state == "INIT":
            if self.valid_since_init >= 2 or "startup_moves" in self.defects:
                if t - self.last_valid_ms <= p["cmd_timeout_ms"] or "startup_moves" in self.defects:
                    self._log(t, "INIT → NORMAL")
                    self.state = "NORMAL"
        if self.state in LATCHED and "no_latch" in self.defects:
            healthy = t - self.last_valid_ms <= 40 and self.sm.state == "VALID" and (self.last_kick is not None and t - self.last_kick <= p["wd_window_max_ms"])
            if healthy and self.state == "STOP_IN_LANE":
                self._log(t, "STOP_IN_LANE → NORMAL (no latch!)")
                self.state = "NORMAL"
        if self.state == "PULL_OVER" and v == 0.0:
            pass  # stays latched, stopped on the shoulder
        self.out = self._output(fb)
        a, s, backup = self.out
        self.out_prev_steer = s
        frame = e2e.Profile2.protect(encode_act(a, s, backup), self.act_counter)
        self.act_counter = (self.act_counter + 1) % 16
        return [(ACT_ID, frame)]

    def _envelope(self, t: int, c: dict, v: float) -> None:
        p, bad = self.p, False
        if self.last_steer is not None and self.last_cmd_t is not None and t > self.last_cmd_t:
            dt = max(0.02, (t - self.last_cmd_t) / 1000)
            rate = (c["steer"] - self.last_steer) / dt
            lim = self.steer_rate_limit(v)
            if abs(rate) > lim:
                c["steer"] = self.last_steer + math.copysign(lim * dt, rate)
                bad = True
        self.last_steer, self.last_cmd_t = c["steer"], t
        if c["speed_req"] > p["odd_max_kmh"] / 3.6:
            c["speed_req"], bad = p["odd_max_kmh"] / 3.6, True
        if not p["min_accel"] <= c["accel"] <= p["max_accel"]:
            c["accel"], bad = max(p["min_accel"], min(p["max_accel"], c["accel"])), True
        jerk = (c["accel"] - self.last_accel) / 0.02
        if abs(jerk) > p["max_jerk"]:
            c["accel"] = self.last_accel + math.copysign(p["max_jerk"] * 0.02, jerk)
        self.last_accel = c["accel"]
        if v > p["odd_max_kmh"] / 3.6 + 0.3 and c["accel"] > -0.5:
            c["accel"] = -0.5
        if bad:
            self.env_since = t if self.env_since is None else self.env_since
            if t - self.env_since >= p["env_debounce_ms"]:
                self._escalate(t, "STOP_IN_LANE", "ENVELOPE")
        else:
            self.env_since = None

    def _perception(self, t: int) -> None:
        c = self.cmd
        if t - self.last_valid_ms > self.p["cmd_timeout_ms"]:
            return
        if c["perception"] == 0:
            self._escalate(t, "STOP_IN_LANE", "PERCEPTION_LOST")
        elif c["odd_exit"]:
            self._escalate(t, "PULL_OVER", "ODD_EXIT")
        elif c["perception"] == 1:
            self._escalate(t, "DEGRADED", "PERCEPTION_DEGRADED")
            self.healthy_perc_since = None
        elif self.state == "DEGRADED":
            self.healthy_perc_since = t if self.healthy_perc_since is None else self.healthy_perc_since
            if t - self.healthy_perc_since >= self.p["degraded_recover_ms"]:
                self._log(t, "DEGRADED → NORMAL (perception recovered)")
                self.state = "NORMAL"

    def _actuator_checks(self, t: int, fb: dict) -> None:
        p = self.p
        steer_cmd = self.out[1]
        if abs(steer_cmd - fb["delta"]) > p["steer_mismatch_deg"]:
            self.steer_mis_since = t if self.steer_mis_since is None else self.steer_mis_since
            if t - self.steer_mis_since >= p["steer_mismatch_ms"]:
                self._escalate(t, "BRAKE_ONLY_STOP", "STEER_ACTUATOR")
        else:
            self.steer_mis_since = None
        a_cmd = self.out[0]
        # remove the slope's share of the measured deceleration, or an uphill grade hides a weak brake
        a_brake = fb["a"] if "no_grade_compensation" in self.defects else fb["a"] + fb.get("grade_accel", 0.0)
        if a_cmd < -1.0 and fb["v"] > 0.5 and a_brake > a_cmd * p["brake_min_ratio"]:
            self.brake_mis_since = t if self.brake_mis_since is None else self.brake_mis_since
            if t - self.brake_mis_since >= p["brake_mismatch_ms"]:
                self._escalate(t, "BACKUP_BRAKE_STOP", "BRAKE_ACTUATOR")
        else:
            self.brake_mis_since = None

    def _ramp(self, target: float, jerk: float) -> float:
        self.decel_now = min(target, self.decel_now + jerk * CYCLE_MS / 1000)
        return -self.decel_now if self.decel_now > 0 else 0.0

    def _output(self, fb: dict) -> tuple[float, float, bool]:
        p, v, st = self.p, fb["v"], self.state
        steer_lim = self.steer_rate_limit(v) * CYCLE_MS / 1000

        def toward(target):
            return self.out_prev_steer + max(-steer_lim, min(steer_lim, target - self.out_prev_steer))

        if st == "INIT":
            return (-1.0, 0.0, False)
        if st == "NORMAL":
            self.decel_now = 0.0
            return (self.cmd["accel"], self.cmd["steer"], False)
        if st == "DEGRADED":
            cap = p["degraded_cap_kmh"] / 3.6
            a = self.cmd["accel"] if "no_speed_cap" in self.defects else min(self.cmd["accel"], 0.8 * (cap - v))
            return (max(-2.0, a), self.cmd["steer"], False)
        if st == "PULL_OVER":
            return (min(0.0, self.cmd["accel"]), self.cmd["steer"], False)
        if v == 0.0:
            return (-1.0, toward(0.0) if st != "BRAKE_ONLY_STOP" else fb["delta"], st == "BACKUP_BRAKE_STOP")
        if "no_heading_hold" not in self.defects:
            self.psi_mrm += fb["yaw_rate"] * CYCLE_MS / 1000
        hold = 0.0 if "no_heading_hold" in self.defects else max(-6.0, min(6.0, -math.degrees(self.psi_mrm) * 2.0))
        if st == "STOP_IN_LANE":
            return (self._ramp(p["mrm_decel"], p["mrm_jerk"]), toward(hold), False)
        if st == "BRAKE_ONLY_STOP":
            return (self._ramp(p["brake_only_decel"], p["emergency_jerk"]), fb["delta"], False)
        return (self._ramp(p["mrm_decel"], p["mrm_jerk"]), toward(hold), True)  # BACKUP_BRAKE_STOP
