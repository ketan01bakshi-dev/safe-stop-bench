"""P4 fleet: signed images, the OTA client against the emulated board, the OTA scenarios, staged rollout, telemetry, the release decision."""
from __future__ import annotations

import dataclasses
import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from fleet import image, ota_client, ota_matrix, release, telemetry  # noqa: E402
from fleet.emulated import EmuBoard, synthetic_image  # noqa: E402
from fleet.ota_client import Faults, OtaClient  # noqa: E402
from fleet.rollout import Device, plan_waves, rollout  # noqa: E402
from ssb import e2e  # noqa: E402


def board(version: str = "2.10") -> tuple[EmuBoard, OtaClient]:
    b = EmuBoard("t", version)
    return b, OtaClient(b)


class ImageTests(unittest.TestCase):
    def test_round_trip_and_tamper(self):
        img = image.sign(b"\xe9" + bytes(5000), "2.12")
        self.assertTrue(image.verify(img))
        self.assertFalse(image.verify(dataclasses.replace(img, data=img.data[:-1] + b"\x01")))

    def test_version_is_part_of_the_signature(self):
        img = image.sign(b"\xe9" + bytes(5000), "2.12")
        self.assertFalse(image.verify(dataclasses.replace(img, version="2.99")), "an old image cannot be re-labelled as a new one")

    def test_wrong_key_fails(self):
        self.assertFalse(image.verify(image.sign(b"\xe9" + bytes(5000), "2.12", key=b"other")))

    def test_client_status_codes_match_the_firmware_header(self):
        """ssc_ota.h is the truth: the client's table must list its enum, name for name."""
        text = (ROOT / "hil" / "firmware" / "SafetyNode" / "ssc_ota.h").read_text(encoding="utf-8")
        enum = re.search(r"enum Status : uint8_t \{(.*?)\};", text, re.S).group(1)
        names = [n.strip().split("=")[0].strip() for n in enum.replace("\n", " ").split(",") if n.strip()]
        self.assertEqual([ota_client.STATUS_NAMES[i] for i in range(len(names))], names)
        ops = re.search(r"enum Op : uint8_t \{(.*?)\};", text, re.S).group(1)
        self.assertIn("BEGIN = 1", ops)
        self.assertIn("STATUS = 5", ops)


class EmulatedBoardTests(unittest.TestCase):
    def test_happy_path_trial_then_commit(self):
        b, c = board()
        r = c.push(synthetic_image("2.12"))
        self.assertEqual((r.outcome, b.version), ("staged", "2.10"), "the old image runs until the reboot")
        c.reboot()
        self.assertEqual(b.version, "2.12")
        self.assertEqual(c.status()["state"], "TRIAL")
        b.advance(3.1)
        self.assertEqual(c.status()["last"], "committed 2.12")

    def test_busy_while_a_run_is_in_progress(self):
        b, c = board()
        b.write(ota_client.frame_msg("R", bytes(8)), 0)   # the PC started a run
        self.assertEqual(c.push(synthetic_image("2.12")).status, "BUSY")
        b.reset()
        self.assertEqual(c.push(synthetic_image("2.12")).outcome, "staged", "a reset ends the session")

    def test_downgrade_and_same_version_are_refused(self):
        _, c = board("2.12")
        self.assertEqual(c.push(synthetic_image("2.12")).status, "DOWNGRADE")
        self.assertEqual(c.push(synthetic_image("2.11")).status, "DOWNGRADE")

    def test_chunks_out_of_order_are_resumed_and_repeats_are_harmless(self):
        b, c = board()
        r = c.push(synthetic_image("2.12"), Faults(drop_chunks=[200, 4000], resend_every=3))
        self.assertEqual(r.outcome, "staged")
        self.assertEqual(r.resumes, 2)

    def test_corrupt_byte_and_wrong_signature_leave_no_trace(self):
        b, c = board()
        self.assertEqual(c.push(synthetic_image("2.12"), Faults(corrupt_at=1234)).status, "HASH_MISMATCH")
        forged = image.sign(synthetic_image("2.12").data, "2.12", key=b"x")
        self.assertEqual(c.push(forged).status, "BAD_SIGNATURE")
        self.assertEqual((c.status()["state"], b.version), ("NONE", "2.10"))

    def test_reboot_without_a_staged_image_is_refused(self):
        self.assertEqual(board()[1].reboot(), "NOT_STAGED")

    def test_unhealthy_image_rolls_back_before_it_says_hello(self):
        b, c = board()
        c.push(synthetic_image("2.12", healthy=False))
        c.reboot()
        self.assertEqual(c.hello.split()[1], "2.10")
        self.assertEqual(c.status()["state"], "ROLLED_BACK")

    def test_reset_in_trial_rolls_back_and_a_new_attempt_works(self):
        b, c = board()
        c.push(synthetic_image("2.12"))
        c.reboot()
        b.advance(1.0)
        b.reset("power loss")
        self.assertEqual(b.version, "2.10")
        self.assertEqual(c.push(synthetic_image("2.13")).outcome, "staged")

    def test_the_whole_scenario_sequence_passes_on_the_emulation(self):
        rows = ota_matrix.run_all(ota_matrix.emulated(), ota_matrix.images_emulated())
        self.assertEqual([r["id"] for r in rows if not r["ok"]], [], [r["evidence"] for r in rows if not r["ok"]])


class RolloutTests(unittest.TestCase):
    def fleet(self, n, version="2.15", rev_a=0):
        out = []
        for i in range(n):
            b = EmuBoard(f"d{i}", version, hardware="rev-a" if i >= n - rev_a else "rev-b")
            out.append(Device(b.name, b, "emulated", b.hardware, b.advance))
        return out

    def test_waves(self):
        sizes = [len(w) for w in plan_waves(self.fleet(100), canary=2)]
        self.assertEqual(sizes, [2, 23, 75])

    def test_a_bad_build_is_halted_after_the_canary(self):
        r = rollout(self.fleet(200), synthetic_image("2.16", healthy=False), canary=4)
        s = r.summary()
        self.assertTrue(s["halted"])
        self.assertEqual((s["touched"], s["rolled_back"], s["untouched"]), (4, 4, 196))

    def test_a_good_build_reaches_every_targeted_device_and_skips_the_others(self):
        r = rollout(self.fleet(30, rev_a=3), synthetic_image("2.16"))
        s = r.summary()
        self.assertFalse(s["halted"])
        self.assertEqual((s["committed"], s["skipped"], s["rolled_back"]), (27, 3, 0))

    def test_devices_already_on_the_version_are_skipped_not_failed(self):
        r = rollout(self.fleet(5, version="2.16"), synthetic_image("2.16"))
        self.assertEqual(r.summary()["skipped"], 5)
        self.assertFalse(r.halted)

    def test_a_dead_device_is_a_failure_not_a_crash(self):
        class Dead:
            def write(self, d, t=0): pass
            def read(self): return b""
        devs = self.fleet(3)
        devs[0] = Device("dead", Dead(), "emulated", "rev-b", lambda s: None)
        r = rollout(devs, synthetic_image("2.16"), canary=1)
        self.assertTrue(r.halted)
        self.assertEqual(r.results[0].outcome, "failed")

    def test_halt_threshold_is_a_percentage_of_devices_touched(self):
        devs = self.fleet(100)
        for d in (devs[20], devs[60]):   # two devices (outside the canary) that cannot take the image: 2 of 100 = 2 %, under the 5 % rule
            d.link.key = b"different-key"
        r = rollout(devs, synthetic_image("2.16"), canary=10, fractions=(1.0,))
        self.assertFalse(r.halted)
        self.assertEqual(r.summary()["rejected"], 2)


class TelemetryTests(unittest.TestCase):
    def test_events_are_kept_without_a_broker(self):
        t = telemetry.Telemetry(broker=None)
        t.emit("d1", "commit", version="2.12")
        self.assertEqual(t.events[0]["event"], "commit")
        b = EmuBoard("e", "2.10")
        c = OtaClient(b)
        c.push(synthetic_image("2.12"))
        c.reboot()
        t.absorb(b.events)
        self.assertIn("staged", [e["event"] for e in t.events])

    def test_state_changes_become_events_and_bad_frames_are_counted(self):
        frames = [(100 + 10 * i, 0x201, e2e.status_protect(i, 1, False, 0, 0, 0.0, 0.0)) for i in range(5)]
        frames += [(200 + 10 * i, 0x201, e2e.status_protect(5 + i, 4, False, 1, 0, 0.0, 0.0)) for i in range(3)]
        bad = bytearray(e2e.status_protect(9, 4, False, 1, 0, 0.0, 0.0))
        bad[0] ^= 0xFF
        ev = telemetry.status_events("B", frames + [(300, 0x201, bytes(bad))])
        changes = [(e["state"], e["cause"]) for e in ev if e["event"] == "saf_state"]
        self.assertEqual(changes, [("NORMAL", None), ("STOP_IN_LANE", "TIMEOUT")])
        self.assertEqual(next(e for e in ev if e["event"] == "status_e2e_rejects")["count"], 1)


try:
    import amqtt  # noqa: F401
    import paho.mqtt.client  # noqa: F401
    HAVE_MQTT = True
except ImportError:
    HAVE_MQTT = False


@unittest.skipUnless(HAVE_MQTT, "pip install -e .[fleet]")
class MqttTransportTests(unittest.TestCase):
    """The same OtaClient, with a real (amqtt) broker and a gateway between it and the board."""

    @classmethod
    def setUpClass(cls):
        from fleet.mqtt_link import LocalBroker
        cls.broker = LocalBroker().start()

    @classmethod
    def tearDownClass(cls):
        cls.broker.stop()

    def setUp(self):
        from fleet.mqtt_link import Gateway, MqttLink
        self.emu = EmuBoard("mq-1", "2.10")
        self.gw = Gateway("mq-1", self.emu, self.broker.port)
        self.link = MqttLink("mq-1", self.broker.port)
        self.addCleanup(self.link.close)
        self.addCleanup(self.gw.close)

    def test_an_update_commits_through_the_broker(self):
        c = OtaClient(self.link)
        r = c.push(synthetic_image("2.12"))
        self.assertEqual((r.outcome, r.status), ("staged", "OK"), r)
        c.reboot()
        self.assertIsNotNone(c.wait_for_hello(10.0))
        self.link.advance(3.6)
        self.assertEqual(self.emu.version, "2.12")
        self.assertTrue(c.status()["last"].startswith("committed 2.12"))
        self.assertGreater(self.gw.forwarded, 100, "the image really went through the gateway")

    def test_a_tampered_image_is_rejected_through_the_broker(self):
        r = OtaClient(self.link).push(synthetic_image("2.12"), Faults(corrupt_at=1000))
        self.assertEqual(r.status, "HASH_MISMATCH")
        self.assertEqual(self.emu.version, "2.10")

    def test_lost_chunks_are_resumed_through_the_broker(self):
        r = OtaClient(self.link).push(synthetic_image("2.12"), Faults(drop_chunks=[400, 2000], resend_every=7))
        self.assertEqual((r.outcome, r.status), ("staged", "OK"), r)
        self.assertGreaterEqual(r.resumes, 2)

    def test_the_control_channel_resets_and_advances_the_board(self):
        before = len(self.emu.events)
        self.assertTrue(self.link.reset_board())
        self.assertGreater(len(self.emu.events), before)
        self.link.advance(1.0)

    def test_a_reset_mid_download_leaves_the_old_image_through_the_broker(self):
        r = OtaClient(self.link).push(synthetic_image("2.12"), Faults(reset_at_pct=50))
        self.assertEqual(r.outcome, "reset")
        self.assertEqual(self.emu.version, "2.10")


class PowerCutWindowTests(unittest.TestCase):
    """REL-11 / REL-12: power lost at each of the five persistent writes that end an update."""

    def run_windows(self, legacy: bool):
        dev = ota_matrix.emulated(version="2.16")
        dev.link.legacy_ota = legacy
        rows = {r["id"]: r for r in ota_matrix.run_cut_windows(dev, ota_matrix.images_cut_emulated())}
        return rows, dev.link

    def test_the_fixed_order_survives_a_cut_at_every_write(self):
        rows, board = self.run_windows(legacy=False)
        self.assertTrue(rows["REL-11"]["ok"], rows["REL-11"]["evidence"])
        self.assertTrue(rows["REL-12"]["ok"], rows["REL-12"]["evidence"])
        self.assertEqual(board.version, "2.50", "the sequence ends on a plain release image")
        self.assertEqual(sum(1 for e in board.events if e["event"] == "power_cut"), 5)
        self.assertEqual(sum(1 for e in board.events if e["event"] == "update_abandoned"), 1, "only the cut after the record, before the switch")

    def test_the_first_order_fails_it(self):
        """Test the test: the order the first firmware used (switch, then record; state flag before the slot it points at) must not pass."""
        rows, _ = self.run_windows(legacy=True)
        self.assertFalse(rows["REL-11"]["ok"], "REL-11 would pass on the defective order: it proves nothing")
        ev = rows["REL-11"]["evidence"]
        self.assertIn("cut 1: lost, runs 2.40", ev, "cut after the switch, before the record: the unhealthy image runs with no probation")

    def test_first_order_second_defect_the_rollback_switches_nothing(self):
        """A cut between the state flag and the slot it points at (first order): TRIAL with no recorded slot, so the rollback is a no-op."""
        emu = EmuBoard("legacy", "2.10")
        emu.legacy_ota = True
        emu.cut_end_after = 2   # switch, st = TRIAL ... then power is lost before `prev` is written
        c = OtaClient(emu, end_timeout_s=0.5)
        r = c.push(synthetic_image("2.40", healthy=False))
        self.assertEqual(r.outcome, "lost")
        self.assertEqual((emu.version, emu.st), ("2.40", 2), "the unhealthy image keeps running; the state even says ROLLED_BACK")
        fixed = EmuBoard("fixed", "2.10")
        fixed.cut_end_after = 2   # the fixed order: only `prev` and `tries` are written by now, the state flag is not
        OtaClient(fixed, end_timeout_s=0.5).push(synthetic_image("2.40", healthy=False))
        self.assertEqual((fixed.version, fixed.st), ("2.10", 0))

    @unittest.skipUnless(HAVE_MQTT, "pip install -e .[fleet]")
    def test_the_control_channel_arms_a_cut_through_the_broker(self):
        from fleet.mqtt_link import Gateway, LocalBroker, MqttLink
        with LocalBroker() as broker:
            emu = EmuBoard("mq-cut", "2.10")
            gw, link = Gateway("mq-cut", emu, broker.port), MqttLink("mq-cut", broker.port)
            try:
                self.assertTrue(link.control("cut 4"))
                self.assertEqual(emu.cut_end_after, 4)
            finally:
                link.close()
                gw.close()


class TraceabilityTests(unittest.TestCase):
    def test_every_ota_scenario_names_a_requirement_in_hazards_json(self):
        """REL-01..10 are the requirement ids themselves, so the matrix is traceable only if hazards.json has them."""
        from ssb import campaigns
        reqs = campaigns.requirements()["requirements"]
        ids = re.findall(r'(?:row\(|"id": )"(REL-\d+)"', (ROOT / "fleet" / "ota_matrix.py").read_text(encoding="utf-8"))
        self.assertEqual(len(ids), len(set(ids)))
        for rid in ids:
            self.assertIn(rid, reqs, f"{rid} is run by fleet.ota_matrix but is not in safety/hazards.json")
            self.assertEqual(reqs[rid]["goal"], "SG9")
        self.assertEqual(sorted(q for q in reqs if q.startswith("REL-")), sorted(ids),
                         "hazards.json has REL requirements the matrix does not run, or the other way round")


def evidence(**over):
    ok = [{"id": "SC-1", "key": "k", "ok": True, "failed_checks": [], "ids_problems": [], "safety_status": "PASS"}]
    ota = [{"id": "REL-01", "title": "t", "ok": True}]
    ev = {"preflight": {"ok": True, "level": "quick", "checks": [], "firmware": {"board_b": "SafetyNode 2.16"}}, "matrix_reference": ok, "matrix_hil": ok,
          "ota_emulated": ota, "ota_real": ota,
          "rollout_good": {"halted": False, "rolled_back": 0, "failed": 0, "rejected": 0, "committed": 41, "untouched": 0, "halt_reason": ""},
          "rollout_bad": {"halted": True, "rolled_back": 4, "failed": 0, "rejected": 0, "committed": 0, "untouched": 196, "halt_reason": "x"},
          "dashboard": {"explained": [], "coverage": {}}, "_files": {}}
    ev.update(over)
    return ev


class ReleaseTests(unittest.TestCase):
    def test_clean_evidence_is_a_go(self):
        self.assertEqual(release.decide(evidence()).verdict, "GO")

    def test_an_open_finding_makes_it_go_with_risks(self):
        row = {"id": "SC-70", "key": "ramp", "ok": True, "open_finding": True, "ids_problems": ["IDS late"], "failed_checks": [], "safety_status": "KNOWN"}
        d = release.decide(evidence(matrix_hil=[row]))
        self.assertEqual(d.verdict, "GO WITH RISKS")
        self.assertTrue(any("SC-70" in r for r in d.risks))

    def test_the_same_finding_in_both_matrices_is_listed_once(self):
        row = {"id": "SC-70", "key": "ramp", "ok": True, "open_finding": True, "ids_problems": ["IDS late"], "failed_checks": [], "safety_status": "KNOWN"}
        d = release.decide(evidence(matrix_reference=[row], matrix_hil=[row]))
        self.assertEqual(len([r for r in d.risks if "SC-70" in r]), 1, d.risks)

    def test_a_failing_mqtt_ota_scenario_is_a_no_go_and_is_named(self):
        d = release.decide(evidence(ota_mqtt_real=[{"id": "REL-05", "title": "resume", "ok": False}]))
        self.assertEqual(d.verdict, "NO GO")
        self.assertTrue(any("mqtt real" in b for b in d.blocking), d.blocking)

    def test_a_failing_cut_window_is_a_no_go(self):
        d = release.decide(evidence(ota_cut_real=[{"id": "REL-11", "title": "cut", "ok": False}]))
        self.assertEqual(d.verdict, "NO GO")
        self.assertTrue(any("cut real" in b for b in d.blocking), d.blocking)

    def test_the_transport_limit_follows_the_evidence(self):
        usb = release.limits(evidence())[0][0]
        self.assertIn("not exercised", usb)
        mq = release.limits(evidence(ota_mqtt_real=[{"id": "REL-01", "title": "t", "ok": True}]))[0][0]
        self.assertIn("no Wi-Fi radio", mq)

    def test_a_failing_ota_scenario_is_a_no_go(self):
        d = release.decide(evidence(ota_real=[{"id": "REL-06", "title": "rollback", "ok": False}]))
        self.assertEqual(d.verdict, "NO GO")

    def test_a_bad_build_that_was_not_halted_is_a_no_go(self):
        bad = evidence()["rollout_bad"] | {"halted": False}
        self.assertEqual(release.decide(evidence(rollout_bad=bad)).verdict, "NO GO")

    def test_a_product_failure_is_a_no_go_but_a_seeded_defect_is_not(self):
        fail = {"box": "PRODUCT", "case": "SR-01-FI-01", "level": "hil", "campaign": "c", "detail": "late", "verdict": "FAIL", "meaning": ""}
        d = release.decide(evidence(dashboard={"explained": [fail], "coverage": {}}))
        self.assertEqual(d.verdict, "NO GO")
        seeded = fail | {"detail": "seeded defect (long_timeout): the bench was asked to catch it. late"}
        self.assertEqual(release.decide(evidence(dashboard={"explained": [seeded], "coverage": {}})).verdict, "GO")

    def test_missing_evidence_or_an_unfit_bench_blocks(self):
        self.assertEqual(release.decide(evidence(ota_real=None)).verdict, "BLOCKED")
        pf = {"ok": False, "level": "quick", "checks": [{"status": "fail", "detail": "no hello"}], "firmware": {}}
        self.assertEqual(release.decide(evidence(preflight=pf)).verdict, "BLOCKED")

    def test_report_renders_with_the_standing_limits(self):
        ev = evidence()
        text = release.render(release.decide(ev), ev)
        self.assertIn("Standing limits", text)
        self.assertIn("demo key", text)


if __name__ == "__main__":
    unittest.main()
