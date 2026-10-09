"""P3 security layer: the attacks, the intrusion detector, the attack x detection matrix, and the controller fix they found."""
from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from security import ids as idsmod  # noqa: E402
from security import matrix  # noqa: E402
from security.attacks import Attacker, attacks_of  # noqa: E402
from ssb import campaigns, config, e2e  # noqa: E402
from ssb.planner import encode_cmd  # noqa: E402
from ssb.runner import run  # noqa: E402

CFG = config.load("config/default.json")
DATA_ID = 0x1234
SCEN = json.loads((ROOT / "scenarios" / "security.json").read_text(encoding="utf-8"))["scenarios"]
BY_KEY = {s["key"]: s for s in SCEN}


def legit(counter: int, t: int, steer: float = 0.0) -> bytes:
    return e2e.Profile5.protect(encode_cmd(t, 0.0, steer, 8.3, 0, 2, 0), counter, DATA_ID)


def attack_sc(**fault) -> dict:
    return {"key": "x", "faults": [{"type": "attack", "start": 100, **fault}]}


class AttackTests(unittest.TestCase):
    def test_forged_valid_frame_passes_e2e(self):
        """The CRC is not a secret: a forged command with the next counter is accepted by the same receiver as the real stream."""
        a = Attacker(attack_sc(kind="spoof_valid", mode="inject", steer=25.0), DATA_ID)
        a.observe(100, [legit(7, 100)], 0x5A)
        forged = a.frames(101)
        self.assertEqual(len(forged), 1)
        rx = e2e.Profile5.Receiver(DATA_ID)
        self.assertEqual(rx.check(legit(7, 100))[0], e2e.OK)
        status, payload = rx.check(forged[0][1])
        self.assertIn(status, e2e.VALID)
        self.assertIsNotNone(payload)

    def test_forged_invalid_frame_is_rejected_by_e2e(self):
        a = Attacker(attack_sc(kind="spoof_invalid"), DATA_ID)
        a.observe(100, [legit(7, 100)], 0x5A)
        rx = e2e.Profile5.Receiver(DATA_ID)
        rx.check(legit(7, 100))
        self.assertEqual(rx.check(a.frames(101)[0][1])[0], e2e.WRONG_CRC)

    def test_replay_of_256_periods_has_the_right_counter(self):
        a = Attacker(attack_sc(kind="replay", age_ms=5120, mode="takeover", start=6000), DATA_ID)
        for i in range(0, 6001, 20):
            a.observe(i, [legit((i // 20) % 256, i)], 0x5A)
        self.assertTrue(a.suppress(6000))
        (cid, frame, _), = a.frames(6000)
        self.assertEqual(frame[2], (6000 // 20) % 256, "the replayed frame carries the counter a fresh frame would have")
        self.assertEqual(int.from_bytes(frame[3:5], "little"), 880, "...but its timestamp is 5.12 s old")

    def test_takeover_removes_the_real_frames_only_while_active(self):
        a = Attacker(attack_sc(kind="spoof_valid", mode="takeover", end=200), DATA_ID)
        self.assertFalse(a.suppress(99))
        self.assertTrue(a.suppress(150))
        self.assertFalse(a.suppress(200))

    def test_unknown_attack_kind_is_refused(self):
        with self.assertRaises(ValueError):
            Attacker(attack_sc(kind="teleport"), DATA_ID)

    def test_scenarios_are_well_formed_and_traceable(self):
        reqs = campaigns.requirements()["requirements"]
        for sc in SCEN:
            for q in sc["req"]:
                self.assertIn(q, reqs, f"{sc['key']}: {q} is not in safety/hazards.json")
            if attacks_of(sc):
                Attacker(sc, DATA_ID)   # kind and parameters are known
        self.assertEqual(len({s["id"] for s in SCEN}), len(SCEN))


class ControllerFixTests(unittest.TestCase):
    def test_second_valid_command_in_the_same_cycle_is_rate_limited(self):
        """Found by SC-68: a forged frame right behind the real one used to skip the steering rate limit (25 deg in one period)."""
        r = run({**BY_KEY["sec_spoof_valid_inject"], "ids": None}, CFG, keep_trace=False)
        self.assertEqual(r["invariant_violations"], [])
        self.assertIn(r["states"][-1][1], ("STOP_IN_LANE", "BRAKE_ONLY_STOP"))


class IdsTests(unittest.TestCase):
    def test_no_alert_on_clean_traffic(self):
        r = run(BY_KEY["sec_clean_control"], CFG, keep_trace=False)
        self.assertFalse(r["security"]["ids"]["alerted"], r["security"]["ids"])

    def test_flood_is_seen_within_a_millisecond_by_the_id_rule(self):
        r = run(BY_KEY["sec_flood_low_priority"], CFG, keep_trace=False)
        ids = r["security"]["ids"]
        self.assertEqual(ids["first_by"], "rule:unknown_id")
        self.assertLessEqual(ids["first_ms"] - 3000, 2)

    def test_the_mimic_is_invisible_to_both_layers(self):
        """Valid CRC, counter, fresh timestamp, the real values: nothing in the stream differs. A documented residual risk."""
        r = run(BY_KEY["sec_mimic"], CFG, keep_trace=False)
        self.assertFalse(r["security"]["ids"]["alerted"])
        self.assertEqual(r["rejected_frames"], 0)
        self.assertEqual(r["final_state"], "NORMAL")

    def test_rules_name_their_reason(self):
        prof = json.loads(idsmod.PROFILE.read_text(encoding="utf-8"))
        rules = idsmod.Rules(prof)
        rules.frame(0, 0x555, bytes(8))
        rules.frame(10, 0x100, bytes(3))
        self.assertIn("unknown_id", rules.alerts)
        self.assertIn("dlc", rules.alerts)

    def test_window_features_have_the_documented_length(self):
        prof = json.loads(idsmod.PROFILE.read_text(encoding="utf-8"))
        self.assertEqual(len(idsmod.window_features([], 99, prof)), len(idsmod.FEATURES))

    def test_mlp_learns_a_separable_toy_problem(self):
        import numpy as np
        rng = np.random.default_rng(1)
        x = np.vstack([rng.normal(0, 1, (200, 3)), rng.normal(4, 1, (60, 3))])
        y = np.array([0.0] * 200 + [1.0] * 60)
        m = idsmod.train_mlp(x, y, epochs=300)
        p = m.predict(x)
        self.assertGreater((p[y == 1] > 0.5).mean(), 0.95)
        self.assertLess((p[y == 0] > 0.5).mean(), 0.05)


class MatrixTests(unittest.TestCase):
    def test_reference_matrix_meets_every_expectation(self):
        rows = []
        for sc in SCEN:
            rows.append(matrix.row(sc, run(sc, CFG, keep_trace=False) | {"verdict": _verdict(sc)}))
        # the safety verdict comes from the oracle in run.py; here we only check the matrix logic on real results
        self.assertEqual(rows[0]["first"], "-")
        flood = next(r for r in rows if r["key"] == "sec_flood_high_priority")
        self.assertEqual(flood["first"], "IDS")
        self.assertTrue(flood["stopped"])
        mimic = next(r for r in rows if r["key"] == "sec_mimic")
        self.assertEqual(mimic["first"], "nobody")
        ramp = next(r for r in rows if r["key"] == "sec_signal_ramp")
        self.assertEqual(ramp["first"], "nobody", "the slow drift is seen by no layer (open finding)")

    def test_a_false_alarm_on_clean_traffic_fails_the_row(self):
        sc = BY_KEY["sec_clean_control"]
        res = {"states": [[0, "NORMAL"]], "rejected_frames": 0, "t_fault": None, "cause": None, "invariant_count": 0,
               "security": {"ids": {"alerted": True, "first_ms": 500, "first_by": "rule:period"}}}
        r = matrix.row(sc, res | {"verdict": {"status": "PASS", "peak_state": "NORMAL", "checks": []}})
        self.assertFalse(r["ok"])
        self.assertIn("false alarm", r["ids_problems"][0])

    def test_late_ids_alert_is_reported_and_a_known_finding_does_not_fail_ci(self):
        sc = BY_KEY["sec_signal_ramp"]
        res = {"states": [[0, "NORMAL"]], "rejected_frames": 0, "t_fault": None, "cause": None, "invariant_count": 0,
               "security": {"ids": {"alerted": False}}}
        open_ = matrix.row(sc, res | {"verdict": {"status": "KNOWN", "known": "x", "peak_state": "NORMAL", "checks": [("c", False)]}})
        self.assertTrue(open_["ok"] and open_["open_finding"])
        strict = matrix.row(sc, res | {"verdict": {"status": "PASS", "peak_state": "NORMAL", "checks": []}})
        self.assertFalse(strict["ok"])


def _verdict(sc: dict) -> dict:
    return {"status": "PASS", "peak_state": "NORMAL", "checks": []}


if __name__ == "__main__":
    unittest.main()
