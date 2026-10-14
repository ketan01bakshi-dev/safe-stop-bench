"""python -m unittest tests.test_p5 -v   (v2.36: P5 stage 0, the offline contrast of the correct build and two seeded bugs)

The numbers are read from the reference model's own traces; no board is involved.
"""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from vehicle import p5_runs  # noqa: E402

PERMANENT = "p5_planned_stop_link_lost"
TRANSIENT = "p5_planned_stop_link_lost_300ms"


class P5ContrastTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runs = p5_runs.run_variants()
        cls.s = {name: {k: p5_runs.stats(r) for k, r in per.items()} for name, per in cls.runs.items()}

    def test_scenarios_carry_their_own_timing(self):
        sc = p5_runs.scenarios()
        self.assertEqual({k: (v["inject_ms"], v["duration_ms"], v["start_kmh"]) for k, v in sc.items()},
                         {PERMANENT: (4500, 14000, 0), TRANSIENT: (4500, 14000, 0)})   # not the bench defaults (2000 / 8000 / 30)

    def test_correct_build_stops_and_latches_in_both(self):
        for key in (PERMANENT, TRANSIENT):
            s = self.s["correct build"][key]
            self.assertTrue(s["verdict_passed"], key)
            self.assertFalse(s["resumed_after_stop_state"])
            self.assertAlmostEqual(s["t_stopped_ms"] / 1000, 7.38, delta=0.05)
            self.assertAlmostEqual(s["stop_x_m"], 25.0, delta=0.2)
        # a 300 ms loss and a permanent one give the same stop: the latch, not the fault, decides
        self.assertEqual(self.s["correct build"][PERMANENT]["t_stopped_ms"], self.s["correct build"][TRANSIENT]["t_stopped_ms"])

    def test_no_latch_drives_on_after_a_transient_loss_only(self):
        s = self.s["no_latch (seeded bug)"]
        self.assertTrue(s[TRANSIENT]["resumed_after_stop_state"])
        self.assertFalse(s[TRANSIENT]["verdict_passed"])
        self.assertGreater(s[TRANSIENT]["end_x_m"], 60.0)                   # still driving at the end of the run
        self.assertTrue(s[PERMANENT]["verdict_passed"])                      # with no end to the fault the bug cannot show: the control needs 300 ms

    def test_long_timeout_stops_late_and_misses_the_transient_loss(self):
        s = self.s["long_timeout (seeded bug)"]
        ok = self.s["correct build"][PERMANENT]
        self.assertFalse(s[PERMANENT]["verdict_passed"])
        self.assertGreater(s[PERMANENT]["stop_x_m"] - ok["stop_x_m"], 8.0)   # about 9 m of model travel further
        self.assertIsNone(s[TRANSIENT]["t_first_stop_state_ms"])             # a 300 ms loss never trips a 1 s timeout
        self.assertFalse(s[TRANSIENT]["verdict_passed"])

    def test_planner_overshoot_is_in_the_trace(self):
        # the cap and runway of the chassis must be sized on the peak, not the 20 km/h target
        self.assertGreater(self.s["correct build"][PERMANENT]["peak_kmh"], p5_runs.TARGET_KMH + 3.0)

    def test_chassis_mapping_is_a_labelled_placeholder(self):
        self.assertEqual(p5_runs.chassis_pwm(20.0), 170)
        self.assertEqual(p5_runs.chassis_pwm(1000.0), 255)
        self.assertAlmostEqual(p5_runs.K_PLACEHOLDER, 0.0612, places=4)


if __name__ == "__main__":
    unittest.main()
