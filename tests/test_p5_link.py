"""python -m unittest tests.test_p5_link -v   (v2.37: P5 stage 1, the replay driver, the chassis link contract and a fake chassis; v2.38: link version 2, wheel encoders in ECHO)

Almost everything runs on the virtual clock of vehicle/simnet.py (exact timing, no sleeping). Three tests use real UDP on the loopback
and take about a second each; their margins are wide so that a busy machine cannot fail them.
"""
import binascii
import json
import socket
import struct
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from vehicle import bridge as bridge_mod  # noqa: E402
from vehicle import conformance, link, p5_runs, replay, simnet  # noqa: E402
from vehicle.fake_chassis import ChassisCore, UdpChassis  # noqa: E402
from vehicle.link import Reason, Reject, State  # noqa: E402

PERMANENT = "p5_planned_stop_link_lost"
TRANSIENT = "p5_planned_stop_link_lost_300ms"
NO_LATCH = "no_latch (seeded bug)"
LONG_TIMEOUT = "long_timeout (seeded bug)"


# --- the codec -------------------------------------------------------------------------------------------------------------------------

class CodecTests(unittest.TestCase):
    def test_crc_known_answer_and_independent_implementation(self):
        self.assertEqual(link.crc16(b"123456789"), 0x29B1)                    # the published check value of CRC-16/CCITT-FALSE
        for data in (b"", b"\x00", b"S5\x01\x03" + bytes(range(40))):
            self.assertEqual(link.crc16(data), binascii.crc_hqx(data, 0xFFFF))  # the standard library's CCITT with the same init

    def test_a_setpoint_frame_assembled_by_hand(self):
        body = struct.pack(">IIIIhh", 42, 0xDEADBEEF, 1, 0, 170, 170)
        head = b"S5" + bytes([link.VERSION, 3])
        want = head + body + struct.pack(">H", binascii.crc_hqx(head + body, 0xFFFF))
        self.assertEqual(link.encode(link.Setpoint(42, 0xDEADBEEF, 1, 0, 170, 170)), want)
        self.assertEqual(len(want), 4 + 20 + 2)

    def test_round_trip_of_every_message_type(self):
        msgs = [link.Hello(1, "p5", State.LATCHED_STOP, Reason.DEADMAN, Reject.LATCHED, 9),
                link.Arm(0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 255),
                link.Setpoint(0xFFFFFFFF, 0, 0xFFFFFFFF, 0xFFFFFFFF, 255, 0),
                link.Stop(5, 6, link.StopReason.WATCHDOG),
                link.Echo(1, 2, 3, State.ARMED, Reason.NONE, Reject.NONE, 4, 5, 6, 7, 65535, 65535),
                link.Echo(1, 2, 3, State.ARMED, Reason.NONE, Reject.NONE, 4, 5, 6, 7, 8, 9, 10, link.ENC_ALL, 0, 1, 0xFFFFFFFE, 0xFFFFFFFF)]
        for m in msgs:
            self.assertEqual(link.decode(link.encode(m)), m)

    def test_echo_v2_layout_assembled_by_hand(self):
        body = struct.pack(">IIIBBBIhhIHH", 1, 2, 3, 1, 0, 0, 4, 5, 6, 7, 8, 9) + struct.pack(">IBIIII", 10, 0b1111, 11, 12, 13, 14)
        head = b"S5" + bytes([2, 5])
        want = head + body + struct.pack(">H", binascii.crc_hqx(head + body, 0xFFFF))
        self.assertEqual(link.encode(link.Echo(1, 2, 3, State.ARMED, Reason.NONE, Reject.NONE, 4, 5, 6, 7, 8, 9, 10, 0b1111, 11, 12, 13, 14)), want)
        self.assertEqual(len(want), 4 + 52 + 2)

    def test_a_version_1_echo_is_refused(self):
        """A chassis on the stage 1 firmware (31-byte ECHO, version 1) must not be mistaken for one without encoders."""
        head = b"S5" + bytes([1, 5])
        body = struct.pack(">IIIBBBIhhIHH", 1, 2, 3, 1, 0, 0, 4, 5, 6, 7, 8, 9)
        with self.assertRaises(link.LinkError):
            link.decode(head + body + struct.pack(">H", link.crc16(head + body)))

    def test_golden_vectors_file_is_in_step_with_the_code(self):
        doc = json.loads((ROOT / "vehicle" / link.VECTORS).read_text(encoding="utf-8"))
        self.assertEqual(doc["vectors"], link.golden_vectors(), "run: python -m vehicle.link")
        for v in doc["vectors"]:
            self.assertEqual(link.describe(link.decode(bytes.fromhex(v["hex"]))), v["message"])

    def test_every_single_bit_error_is_rejected(self):
        for v in link.golden_vectors():
            frame = bytes.fromhex(v["hex"])
            for i in range(len(frame) * 8):
                bad = bytearray(frame)
                bad[i // 8] ^= 1 << (i % 8)
                with self.assertRaises(link.LinkError, msg=f"{v['name']} bit {i}"):
                    link.decode(bytes(bad))

    def test_malformed_frames(self):
        good = link.encode(link.Stop(1, 2, 0))
        for name, data in {"empty": b"", "short": good[:5], "truncated": good[:-1], "extended": good + b"\0"}.items():
            with self.assertRaises(link.LinkError, msg=name):
                link.decode(data)
        # a frame with a valid CRC but the wrong size of body, an unknown type, and another version
        v = bytes([link.VERSION])
        for head, body in ((b"S5" + v + b"\x04", b"\0" * 8), (b"S5" + v + b"\x09", b"\0" * 9), (b"S5\x01\x04", b"\0" * 9), (b"S5\x03\x04", b"\0" * 9),
                           (b"X5" + v + b"\x04", b"\0" * 9)):
            frame = head + body
            with self.assertRaises(link.LinkError):
                link.decode(frame + struct.pack(">H", link.crc16(frame)))

    def test_fields_out_of_range_are_refused_when_encoding(self):
        for bad in (link.Setpoint(1, 2, 3, 4, 40000, 0), link.Arm(1, 2, 3, 256), link.Stop(-1, 2, 0), link.Hello(1, "", 0), link.Hello(1, "123456789", 0),
                    link.Echo(1, 2, 3, 1, 0, 0, 4, 5, 6, 7, 8, 9, enc_fl=2**32), link.Echo(1, 2, 3, 1, 0, 0, 4, 5, 6, 7, 8, 9, enc_mask=256)):
            with self.assertRaises(link.LinkError):
                link.encode(bad)


# --- the chassis state machine against the hand-written cases ---------------------------------------------------------------------------

class ConformanceTests(unittest.TestCase):
    def test_all_cases_pass(self):
        n, failures = conformance.run_all()
        self.assertGreaterEqual(n, 20)
        self.assertEqual(failures, [])

    def test_the_runner_notices_a_wrong_core(self):
        """The cases are a real check: a dead-man of 300 ms, a core that does not latch on a bad nonce, and one that never zeroes the pins each fail some case."""
        class NoNonceLatch(ChassisCore):
            def _latch(self, now, reason):
                if reason is not Reason.BAD_NONCE:
                    super()._latch(now, reason)

        class NoDeadman(ChassisCore):
            def step(self, now):
                if self.state is State.ARMED:
                    self.last_valid = now                                          # a core whose dead-man never fires
                return super().step(now)

        class NoMask(ChassisCore):
            def set_encoders(self, now, counts, mask=link.ENC_ALL):
                super().set_encoders(now, counts, link.ENC_ALL)                   # reports wheels that are not fitted

        def slow(params):
            return ChassisCore(params["boot_id"], hello_peer="b", **{**{k: v for k, v in params.items() if k != "boot_id"}, "deadman_ms": 300})

        def with_class(cls):
            return lambda params: cls(params["boot_id"], hello_peer="b", **{k: v for k, v in params.items() if k != "boot_id"})

        for name, factory in (("dead-man 300 ms", slow), ("no latch on bad nonce", with_class(NoNonceLatch)), ("no dead-man", with_class(NoDeadman)),
                              ("encoder mask ignored", with_class(NoMask))):
            _n, failures = conformance.run_all(factory=factory)
            self.assertTrue(failures, f"the cases did not notice: {name}")


# --- the replay plan -------------------------------------------------------------------------------------------------------------------

class PlanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runs = p5_runs.run_variants()
        cls.scen = p5_runs.scenarios()
        cls.plans = {(v, k): replay.build_plan(r, cls.scen[k], variant=v) for v, per in cls.runs.items() for k, r in per.items()}

    def test_plan_shape_and_peak(self):
        plan = self.plans[("correct build", PERMANENT)]
        self.assertEqual(plan.period_ms, 50)
        self.assertEqual(plan.duration_ms, 14000)
        self.assertEqual(len(plan.steps), 281)
        self.assertEqual([s[0] for s in plan.steps[:3]], [0, 50, 100])
        self.assertEqual(plan.steps[0][1:], (0, 0))
        peak_kmh = p5_runs.stats(self.runs["correct build"][PERMANENT])["peak_kmh"]
        self.assertEqual(plan.pwm_max, p5_runs.chassis_pwm(peak_kmh))          # sized on the planner's overshoot, not the 20 km/h target
        self.assertGreater(plan.pwm_max, p5_runs.chassis_pwm(p5_runs.TARGET_KMH))
        self.assertEqual((plan.pwm_max, replay.suggest_cap(plan)), (203, 205))
        # a seeded bug that stops late lets the planner accelerate further: its replay needs a higher cap, and the bridge refuses a lower one
        self.assertEqual(self.plans[(LONG_TIMEOUT, PERMANENT)].pwm_max, 232)

    def test_plan_is_joined_with_the_scenario_and_the_result(self):
        m = self.plans[(NO_LATCH, TRANSIENT)].markers
        self.assertEqual(m["inject_ms"], 4500)
        self.assertEqual(m["t_fault_ms"], 4600)
        self.assertEqual(m["faults"], [{"type": "link_lost", "start": 4500, "end": 4800}])
        self.assertEqual([s[1] for s in m["states"]], ["INIT", "NORMAL", "STOP_IN_LANE", "NORMAL"])    # it stopped, then drove on
        self.assertFalse(m["verdict_passed"])
        self.assertEqual(m["expect"]["latched"], True)

    def test_plan_reproduces_the_stage_0_contrast_in_pwm(self):
        ok, nl, lt = (self.plans[(v, TRANSIENT)] for v in ("correct build", NO_LATCH, LONG_TIMEOUT))
        self.assertEqual(ok.setpoint(13900), (0, 0))                          # stopped and latched
        self.assertGreater(nl.setpoint(13900)[0], 160)                        # no_latch: back at about 20 km/h
        self.assertGreater(lt.setpoint(13900)[0], 160)                        # long_timeout: never reacted to a 300 ms loss
        long_perm = self.plans[(LONG_TIMEOUT, PERMANENT)]
        ok_perm = self.plans[("correct build", PERMANENT)]
        t_zero = lambda p: next(t for t, pl, _r in p.steps if t > 5000 and pl == 0)   # noqa: E731
        self.assertGreater(t_zero(long_perm), t_zero(ok_perm))                # the slower timeout stops later

    def test_plan_is_deterministic_and_tied_to_its_source(self):
        a = replay.build_plan(self.runs["correct build"][PERMANENT], self.scen[PERMANENT])
        b = replay.build_plan(self.runs["correct build"][PERMANENT], self.scen[PERMANENT])
        self.assertEqual(a, b)
        self.assertEqual(len(a.source_sha256), 64)
        self.assertNotEqual(a.source_sha256, self.plans[(NO_LATCH, TRANSIENT)].source_sha256)

    def test_scale_changes_the_pwm_not_the_shape(self):
        full = self.plans[("correct build", PERMANENT)]
        half = replay.build_plan(self.runs["correct build"][PERMANENT], self.scen[PERMANENT], k=p5_runs.K_PLACEHOLDER / 2)
        self.assertAlmostEqual(half.pwm_max / full.pwm_max, 0.5, delta=0.02)
        t_zero = lambda p: next(t for t, pl, _r in p.steps if t > 5000 and pl == 0)   # noqa: E731
        self.assertAlmostEqual(t_zero(half), t_zero(full), delta=300)         # the stop instant does not move with the scale


# --- the bridge and the chassis core on the virtual clock ----------------------------------------------------------------------------------

class VirtualLinkTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        runs = p5_runs.run_variants()
        scen = p5_runs.scenarios()
        cls.plans = {(v, k): replay.build_plan(r, scen[k], variant=v) for v, per in runs.items() for k, r in per.items()}
        cls.plan = cls.plans[("correct build", PERMANENT)]
        cls.clean = simnet.simulate(cls.plan)
        cls.setpoint_sends = [t for t, d, f in cls.clean.wire if d == "bridge->chassis" and isinstance(link.decode(f), link.Setpoint)]

    def sim(self, **kw):
        return simnet.simulate(self.plan, **kw)

    def drop_setpoints(self, first_index, count):
        """A window that loses `count` consecutive setpoints, starting with the one at index `first_index` of the clean run."""
        t0 = self.setpoint_sends[first_index]
        return [("bridge->chassis", t0, self.setpoint_sends[first_index + count - 1] + 1)]

    def last_setpoint_rx(self, res):
        """When the chassis received the last setpoint that was sent (send time + the 2 ms latency of the virtual link)."""
        return max(ts + 2 for ts, d, f in res.wire if d == "bridge->chassis" and isinstance(link.decode(f), link.Setpoint))

    def test_clean_run_completes_and_the_pins_follow_the_plan(self):
        res = self.clean
        self.assertEqual(res.bridge.outcome, "COMPLETE")
        s = res.bridge.summary()
        self.assertEqual((s["setpoints"], s["late_ticks"], s["arm_sent"]), (280, 0, 1))
        self.assertLessEqual(s["max_echo_gap_ms"], 55)                        # 20 Hz echoes
        self.assertEqual(res.chassis.state, State.DISARMED)                    # a planned end is not a latch
        self.assertEqual(res.chassis.outputs, (0, 0))
        self.assertEqual({st for _t, st, _r in res.chassis.state_log}, {"DISARMED", "ARMED"})
        allowed = {v for _t, v, _w in self.plan.steps} | {0}
        self.assertTrue({pl for _t, pl, _r in res.chassis.out_log} <= allowed)   # the pins only ever carry plan values
        self.assertEqual(max(pl for _t, pl, _r in res.chassis.out_log), self.plan.pwm_max)
        self.assertLessEqual(s["max_pwm_err"], 8)                              # one plan step of the ramp (the echo shows the pins, which lag by a writer tick)

    def test_pins_zero_within_latency_plus_writer_tick_of_the_plan(self):
        res = self.clean
        t0 = next(e["t_ms"] for e in res.bridge.log if e.get("note") == "chassis armed; plan time 0") + 100      # bridge clock -> chassis clock
        plan_zero = next(t for t, pl, _r in self.plan.steps if t > 5000 and pl == 0)
        pins_zero = next(t for t, pl, _r in res.chassis.out_log if t > t0 + 5000 and pl == 0)
        self.assertTrue(0 <= pins_zero - (t0 + plan_zero) <= 2 + 10 + 1, pins_zero - (t0 + plan_zero))        # latency 2 ms + one writer period

    def test_every_variant_and_scenario_replays_on_the_chassis(self):
        for (variant, key), plan in self.plans.items():
            res = simnet.simulate(plan)
            self.assertEqual(res.bridge.outcome, "COMPLETE", (variant, key))
            self.assertEqual(res.chassis.state, State.DISARMED, (variant, key))
            self.assertEqual(max(pl for _t, pl, _r in res.chassis.out_log), plan.pwm_max, (variant, key))
            self.assertLessEqual(res.chassis.out_log[-1][1], 0)

    def test_bridge_dies_without_a_stop_the_deadman_cuts_the_motors(self):
        res = self.sim(bridge_dies_at=6200, until_ms=9000)
        c = res.chassis
        self.assertEqual((c.state, c.reason), (State.LATCHED_STOP, Reason.DEADMAN))
        t_rx = self.last_setpoint_rx(res)
        t_latch = next(t for t, st, r in c.state_log if r == "DEADMAN")
        t_pins = simnet.cut_time(c, 6000)
        self.assertEqual(t_latch - t_rx, c.deadman_ms + 1)                     # the first ms with MORE than 200 ms since the last valid setpoint
        self.assertTrue(0 <= t_pins - t_latch <= c.writer_period_ms, (t_pins, t_latch))   # then the single writer follows within one period
        self.assertLessEqual(t_pins - t_rx, c.deadman_ms + c.writer_period_ms + 1)
        self.assertEqual(c.outputs, (0, 0))

    def test_a_loss_of_up_to_200_ms_between_setpoints_is_tolerated_more_latches(self):
        # one setpoint is sent every 50 ms: losing n in a row leaves a gap of 50 (n + 1) ms
        for lost, ok in ((2, True), (3, True), (4, False)):                    # 150 ms, 200 ms (exactly: tolerated), 250 ms
            res = self.sim(drop=self.drop_setpoints(60, lost))
            self.assertEqual(res.bridge.outcome == "COMPLETE", ok, (lost, res.bridge.outcome))
            self.assertEqual(res.chassis.reason == Reason.DEADMAN, not ok, lost)

    def test_after_a_latch_the_chassis_stays_stopped_when_the_link_returns(self):
        res = self.sim(drop=self.drop_setpoints(80, 6))                        # a 300 ms hole in the middle of the run
        c = res.chassis
        self.assertEqual((c.state, c.reason), (State.LATCHED_STOP, Reason.DEADMAN))
        self.assertEqual(res.bridge.outcome, "ABORTED:CHASSIS_LATCHED_STOP(DEADMAN)")     # the take is INCONCLUSIVE, not a result
        t_cut = simnet.cut_time(c)
        self.assertIsNotNone(t_cut)
        self.assertTrue(all(pl == 0 and pr == 0 for t, pl, pr in c.out_log if t >= t_cut))   # no restart when the setpoints come back

    def test_bridge_watchdog_aborts_and_stops_the_chassis_when_the_echoes_stop(self):
        res = self.sim(drop=[("chassis->bridge", 7000, 99999)])
        b, c = res.bridge, res.chassis
        self.assertEqual(b.outcome, "ABORTED:WATCHDOG")
        self.assertEqual((c.state, c.reason), (State.LATCHED_STOP, Reason.BRIDGE_WATCHDOG))
        t_abort = next(e["t_ms"] for e in b.log if e.get("note") == "finished")
        last_echo = max(e["t_ms"] for e in b.log if e.get("dir") == "rx" and e["type"] == "ECHO")
        self.assertEqual(t_abort - last_echo, b.watchdog_ms + 1)               # 100 ms of silence is tolerated, 101 ms aborts

    def test_a_stale_or_confused_frame_on_the_wire_latches_the_chassis(self):
        bad_run = link.encode(link.Setpoint(run_id=999, nonce=0xA5A5A5A5, seq=500, t_ms=0, pwm_l=10, pwm_r=10))
        bad_nonce = link.encode(link.Setpoint(run_id=1000, nonce=1, seq=500, t_ms=0, pwm_l=10, pwm_r=10))
        for frame, reason in ((bad_run, Reason.BAD_RUN), (bad_nonce, Reason.BAD_NONCE)):
            res = self.sim(inject=[(5000, "bridge->chassis", frame, "bridge")], until_ms=6000)
            self.assertEqual(res.chassis.reason, reason)
            self.assertEqual(res.chassis.outputs, (0, 0))
            self.assertTrue(res.bridge.outcome.startswith("ABORTED:CHASSIS_LATCHED_STOP"), res.bridge.outcome)

    def test_a_frame_from_another_host_changes_nothing(self):
        frame = link.encode(link.Stop(1000, 0xA5A5A5A5, link.StopReason.ABORT))     # a perfect STOP, but not from the armed peer
        res = self.sim(inject=[(5000, "bridge->chassis", frame, "someone-else")])
        self.assertEqual(res.bridge.outcome, "COMPLETE")
        self.assertEqual(res.chassis.rx_foreign, 1)

    def test_wifi_drop_on_the_chassis_latches(self):
        res = self.sim(link_down_at=6000, drop=[("chassis->bridge", 6000, 99999)], until_ms=7000)      # a Wi-Fi drop: nothing passes either way
        self.assertEqual((res.chassis.state, res.chassis.reason), (State.LATCHED_STOP, Reason.LINK_LOST))
        self.assertEqual(res.bridge.outcome, "ABORTED:WATCHDOG")               # nothing can be echoed over a dead link: the bridge's own watchdog ends the take
        self.assertEqual(res.chassis.reason, Reason.LINK_LOST)                 # and the first reason stays: the later STOP(WATCHDOG) does not overwrite it

    def test_no_chassis_no_arm(self):
        res = self.sim(drop=[("chassis->bridge", 0, 99999)], bridge=bridge_mod.Bridge(self.plan, run_id=1, nonce=1, pwm_cap=210, hello_timeout_ms=1200))
        self.assertEqual(res.bridge.outcome, "ABORTED:NO_CHASSIS")
        self.assertEqual(res.chassis.state, State.DISARMED)
        self.assertFalse([1 for _t, d, f in res.wire if d == "bridge->chassis" and isinstance(link.decode(f), link.Arm)])   # nothing was armed

    def test_encoder_counts_reach_the_bridge_and_change_nothing_else(self):
        class Wheels(ChassisCore):
            """A placeholder wheel: counts grow with the applied PWM (no physics; only to move the counters)."""
            def step(self, now):
                self._acc = getattr(self, "_acc", 0.0) + self.outputs[0] / 255 * 0.05
                self.set_encoders(now, (int(self._acc),) * 4, mask=0b0011)
                return super().step(now)

        res = self.sim(chassis=Wheels(0x5EED0004, build="p5-sim", hello_peer="bridge"))
        s = res.bridge.summary()
        self.assertEqual(res.bridge.outcome, "COMPLETE")
        self.assertEqual(s["enc_mask"], 0b0011)
        self.assertGreater(s["enc_last"][0], 0)
        self.assertEqual(s["enc_last"][0], s["enc_last"][1])
        self.assertEqual(s["enc_last"][2:], [0, 0])                            # outside the mask
        self.assertEqual(res.chassis.out_log, self.clean.chassis.out_log)      # the pins are exactly those of the run without encoders
        self.assertEqual(self.clean.bridge.summary()["enc_mask"], 0)

    def test_unexpected_firmware_is_not_armed(self):
        b = bridge_mod.Bridge(self.plan, run_id=1000, nonce=1, pwm_cap=210, expect_builds=("p5-real",))
        res = self.sim(bridge=b)
        self.assertEqual(res.bridge.outcome, "ABORTED:UNEXPECTED_BUILD(p5-sim)")
        self.assertEqual(res.chassis.state, State.DISARMED)

    def test_a_latched_chassis_is_not_armed_and_needs_the_physical_rearm(self):
        chassis = ChassisCore(0x5EED0002, build="p5-sim", hello_peer="bridge")
        first = simnet.simulate(self.plan, chassis=chassis, bridge_dies_at=6200, until_ms=9000)
        self.assertEqual(first.chassis.state, State.LATCHED_STOP)
        # a new run: the bridge hears the latched HELLO and does not arm; no ARM frame is ever sent
        b2 = bridge_mod.Bridge(self.plan, run_id=1001, nonce=7, pwm_cap=210)
        second = simnet.simulate(self.plan, chassis=chassis, bridge=b2, from_ms=9100, until_ms=12000)
        self.assertEqual(b2.outcome, "ABORTED:CHASSIS_LATCHED_STOP(DEADMAN)")
        self.assertFalse([1 for _t, d, f in second.wire if d == "bridge->chassis" and isinstance(link.decode(f), link.Arm)])
        # a plain ARM from the wire is refused too
        arm = link.encode(link.Arm(0x5EED0002, 1002, 8, 210))
        chassis.rx(arm, "bridge", 12000)
        self.assertEqual((chassis.state, chassis.reject), (State.LATCHED_STOP, Reject.LATCHED))
        # the human presses the button, and only then a newer run id is armed and completes
        chassis.physical_rearm(12100)
        b3 = bridge_mod.Bridge(self.plan, run_id=1002, nonce=9, pwm_cap=210)
        third = simnet.simulate(self.plan, chassis=chassis, bridge=b3, from_ms=12200)
        self.assertEqual(b3.outcome, "COMPLETE")
        self.assertEqual(third.chassis.state, State.DISARMED)

    def test_the_same_or_an_older_run_id_is_not_armed_again(self):
        chassis = ChassisCore(0x5EED0003, build="p5-sim", hello_peer="bridge")
        simnet.simulate(self.plan, chassis=chassis)                                   # run 1000 completes
        again = bridge_mod.Bridge(self.plan, run_id=1000, nonce=3, pwm_cap=210)
        res = simnet.simulate(self.plan, chassis=chassis, bridge=again, from_ms=20000, until_ms=21500)
        self.assertEqual(again.outcome, "ABORTED:RUN_ID_NOT_NEWER(1000)")
        self.assertEqual(res.chassis.state, State.DISARMED)

    def test_bridge_refuses_a_cap_below_the_plan_and_notices_a_rebooted_chassis(self):
        with self.assertRaises(ValueError):
            bridge_mod.Bridge(self.plan, run_id=1, nonce=1, pwm_cap=self.plan.pwm_max - 1)
        res = self.sim(until_ms=3000)                                                 # running, 2.5 s into the plan
        b = res.bridge
        self.assertEqual(b.state, "RUNNING")
        b.step(3001, [link.encode(link.Hello(0x1234, "p5-sim", State.DISARMED))])     # another boot id announces itself
        self.assertEqual(b.outcome, "ABORTED:CHASSIS_REBOOTED")


# --- the same code on real UDP (loopback), short ----------------------------------------------------------------------------------------

def _short_plan(seconds=1.2):
    steps = tuple((t, min(80, t // 20), min(80, t // 20)) for t in range(0, int(seconds * 1000) + 1, 50))
    return replay.Plan(key="smoke", variant="", dut="none", seed=0, k=0.0, period_ms=50, steps=steps, duration_ms=int(seconds * 1000), source_sha256="0" * 64)


class UdpSmokeTests(unittest.TestCase):
    def _pair(self, **chassis_kw):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 0))
        core = ChassisCore(0xB007, build="p5-fake", hello_peer=sock.getsockname(), hello_period_ms=100, **chassis_kw)
        fake = UdpChassis(core, sock.getsockname())
        return sock, fake

    def test_a_short_replay_over_udp_completes(self):
        sock, fake = self._pair()
        self.addCleanup(sock.close)
        with fake:
            b = bridge_mod.Bridge(_short_plan(), run_id=int(time.time()), nonce=5, pwm_cap=100, watchdog_ms=400)
            bridge_mod.run_udp(b, sock, max_s=15)
            snap = fake.snapshot()
        self.assertEqual(b.outcome, "COMPLETE", b.log[-3:])
        self.assertEqual((snap["state"], snap["rx_bad"], snap["rx_foreign"]), ("DISARMED", 0, 0))
        self.assertGreaterEqual(b.stats["setpoints"], 20)

    def test_an_exception_in_the_bridge_still_sends_stop(self):
        sock, fake = self._pair()
        self.addCleanup(sock.close)

        class Crashy(bridge_mod.Bridge):
            def step(self, now, rx=()):
                if now > 900:
                    raise RuntimeError("the bridge crashed")                       # after about 0.9 s: the run is armed and streaming
                return super().step(now, rx)

        with fake:
            b = Crashy(_short_plan(3.0), run_id=int(time.time()), nonce=6, pwm_cap=100, watchdog_ms=400)
            with self.assertRaises(RuntimeError):
                bridge_mod.run_udp(b, sock, max_s=15)
            time.sleep(0.15)
            snap = fake.snapshot()
        self.assertEqual((snap["state"], snap["reason"]), ("LATCHED_STOP", "BRIDGE_ABORT"))   # by the STOP of the `finally`, not by the dead-man

    def test_a_bridge_that_dies_silently_is_stopped_by_the_deadman(self):
        sock, fake = self._pair()
        self.addCleanup(sock.close)
        sock.settimeout(0.2)
        with fake:
            # arm by hand, send three setpoints, then fall silent (a killed process sends no STOP)
            deadline = time.time() + 3
            while time.time() < deadline:
                try:
                    data, src = sock.recvfrom(256)
                except socket.timeout:
                    continue
                if isinstance(link.decode(data), link.Hello):
                    break
            else:
                self.fail("no HELLO from the fake chassis")
            hello = link.decode(data)
            sock.sendto(link.encode(link.Arm(hello.boot_id, 4242, 77, 100)), src)
            for seq in (1, 2, 3):
                sock.sendto(link.encode(link.Setpoint(4242, 77, seq, 50 * seq, 60, 60)), src)
                time.sleep(0.05)
            self.assertEqual(fake.snapshot()["state"], "ARMED")
            time.sleep(0.7)                                                          # 200 ms dead-man, plus slack for a busy machine
            snap = fake.snapshot()
        self.assertEqual((snap["state"], snap["reason"], snap["outputs"]), ("LATCHED_STOP", "DEADMAN", [0, 0]))


# --- the command line ------------------------------------------------------------------------------------------------------------------

class CommandLineTests(unittest.TestCase):
    def test_sim_run_writes_a_labelled_manifest_and_log(self):
        with tempfile.TemporaryDirectory() as d:
            rc = bridge_mod.main(["--sim", "--scenario", TRANSIENT, "--variant", "no_latch", "--out", d])
            self.assertEqual(rc, 0)
            man = json.loads((Path(d) / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(man["label"], "virtual chassis: not a physical result")
            self.assertEqual(man["plan"]["variant"], NO_LATCH)
            self.assertEqual(man["plan"]["source_sha256"], replay.trace_sha256(p5_runs.run_variants()["no_latch (seeded bug)"][TRANSIENT]["trace"]))
            self.assertEqual(man["summary"]["outcome"], "COMPLETE")
            lines = (Path(d) / "bridge_log.jsonl").read_text(encoding="utf-8").splitlines()
            self.assertGreater(len(lines), 500)
            self.assertTrue(all(json.loads(x) for x in lines))

    def test_a_real_chassis_address_needs_the_explicit_flag(self):
        with self.assertRaises(SystemExit) as cm:
            bridge_mod.main(["--listen", "192.0.2.10:5005"])
        self.assertIn("--allow-real", str(cm.exception))

    def test_unknown_scenario_and_ambiguous_variant_are_errors(self):
        with self.assertRaises(SystemExit):
            bridge_mod.main(["--sim", "--scenario", "nope"])
        with self.assertRaises(SystemExit):
            bridge_mod.main(["--sim", "--variant", "xyz"])


if __name__ == "__main__":
    unittest.main()
