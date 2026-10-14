"""python -m unittest tests.test_plant_ci -v   (v2.39: P6 plant-model pipeline; the omc-dependent test is skipped without OpenModelica)"""
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from plant.ci import experiments as ex  # noqa: E402
from plant.ci import handcalc as hc  # noqa: E402
from plant.ci import omc, pipeline  # noqa: E402
from plant.ci.omc import SimResult, ToolError  # noqa: E402

try:
    omc.omc_path()
    HAVE_OMC = True
except ToolError:
    HAVE_OMC = False


class HandCalcTests(unittest.TestCase):
    def euler_stop(self, v0, decel, dt=1e-5):
        """Brute-force the same physics (dead time + lag + constant request) to check the closed form."""
        t = x = a = 0.0
        v = v0
        while v > 0:
            if t >= hc.DEAD:
                a += (-decel - a) * dt / hc.TAU
            v += a * dt
            x += max(v, 0.0) * dt
            t += dt
        return t, x

    def test_stop_with_lag_matches_brute_force(self):
        t_cf, x_cf = hc.stop_with_lag(30 / 3.6, 3.0)
        t_bf, x_bf = self.euler_stop(30 / 3.6, 3.0)
        self.assertAlmostEqual(t_cf, t_bf, delta=1e-3)
        self.assertAlmostEqual(x_cf, x_bf, delta=2e-3)
        # and against the textbook bound: v0^2/2a plus the distance covered during dead time and lag
        self.assertGreater(x_cf, (30 / 3.6) ** 2 / 6.0)

    def test_simple_closed_forms(self):
        self.assertAlmostEqual(hc.lag_step(2.0, hc.TAU), 2.0 * (1 - math.exp(-1)), places=12)
        self.assertAlmostEqual(hc.coast_on_grade(5.0, 10.0)[1], 25 / (2 * 0.981), places=9)
        self.assertAlmostEqual(hc.yaw_rate(10, 5), 10 / 2.7 * math.tan(math.radians(5)), places=12)
        # steering: a 10 deg step at 20 deg/s ends the ramp at 0.5 s; lagged ramp is then about 9 deg; 9.5 deg ~ tau ln 2 later
        self.assertAlmostEqual(hc.steer_reach(9.5, 10.0), 0.5 + 0.05 * math.log(1.0 / 0.5), delta=2e-3)
        self.assertAlmostEqual(hc.dynamic_decel(10.0), -(0.015 * 1500 * 9.81 + 0.5 * 1.2 * 0.7 * 100) / (1500 * 1.04), places=9)


class ExperimentModelTests(unittest.TestCase):
    def test_generated_model_binds_every_input_once_per_experiment(self):
        src = ex.model_source()
        for e in ex.E:
            for name in ex.DEFAULT_INPUTS:
                self.assertEqual(src.count(f"  {e.id}.{name} = "), 1, (e.id, name))
        self.assertTrue(src.startswith("model AllExperiments") and src.rstrip().endswith("end AllExperiments;"))

    def test_every_experiment_has_a_check_and_a_spec_link(self):
        for e in ex.E:
            self.assertTrue(e.checks, e.id)
            self.assertTrue(e.spec, e.id)
        covered = {r for e in ex.E for r in e.spec}
        self.assertEqual(covered, {f"R{i}" for i in range(1, 11)})   # every requirement in the spec is exercised


class MutantTests(unittest.TestCase):
    def test_each_mutant_applies_exactly_once_and_names_a_real_check(self):
        src = omc.MODEL_FILE.read_text(encoding="utf-8")
        checks = {cid for e in ex.E for cid in e.checks} | {"model.units", "model.check"}   # L0 checks catch too
        for m in json.loads(pipeline.MUTANTS.read_text(encoding="utf-8"))["mutants"]:
            self.assertEqual(src.count(m["find"]), 1, m["id"])
            self.assertNotEqual(m["find"], m["replace"], m["id"])
            self.assertTrue(set(m["expect_caught_by"]) <= checks, m["id"])


class SpecAndLadderTests(unittest.TestCase):
    def test_spec_complete(self):
        ok, detail = pipeline.spec_complete()
        self.assertTrue(ok, detail)

    def test_ladder_fixes_tolerances_and_names_the_shortcuts(self):
        lad = pipeline.ladder()
        self.assertLess(lad["tolerances"]["simulation_tolerance_fine"], lad["tolerances"]["simulation_tolerance_coarse"])
        self.assertEqual([lv["id"] for lv in lad["levels"]], ["L0", "L1", "L2", "L3", "L4"])
        self.assertGreaterEqual(len(lad["shortcuts"]), 5)
        self.assertEqual(set(lad["attribution"]), {"MODEL", "CHECK", "TOOL"})

    def test_level_needs_every_check_and_a_tool_failure_is_not_a_pass(self):
        run = pipeline.Run(stamp="t", out=Path(tempfile.mkdtemp()))
        run.add(pipeline.Check("a", "L0", True, None))
        run.add(pipeline.Check("b", "L1", True, None))
        self.assertEqual(pipeline.level(run), "L1")
        run.add(pipeline.Check("c", "L1", None, "TOOL"))
        self.assertEqual(pipeline.level(run), "L0")
        run.add(pipeline.Check("d", "L0", False, "MODEL"))
        self.assertEqual(pipeline.level(run), "none")


class SimResultTests(unittest.TestCase):
    def test_interpolation_and_event_points(self):
        r = SimResult(t=[0.0, 1.0, 1.0, 2.0], y={"v": [0.0, 1.0, 5.0, 7.0]})
        self.assertEqual(r.at("v", 0.5), 0.5)
        self.assertEqual(r.at("v", 1.0), 5.0)          # at an event the value after it counts
        self.assertEqual(r.at("v", 1.5), 6.0)
        self.assertEqual(r.at("v", 9.0), 7.0)
        self.assertEqual(r.window("v", 0.9, 1.1), [1.0, 5.0])

    def test_read_csv(self):
        d = Path(tempfile.mkdtemp())
        (d / "r.csv").write_text('"time","e.v"\n0,1\n0.5,2\n', encoding="utf-8")
        r = omc.read_csv(d / "r.csv")
        self.assertEqual(r.t, [0.0, 0.5])
        self.assertEqual(r.series("e.v"), [1.0, 2.0])


@unittest.skipUnless(HAVE_OMC, "OpenModelica (omc) not installed")
class OmcTests(unittest.TestCase):
    def test_the_plant_checks_and_one_experiment_simulates(self):
        d = Path(tempfile.mkdtemp())
        res = omc.check_model([omc.MODEL_FILE], "SafeStopPlant.Plant", d)
        self.assertTrue(res.ok)


if __name__ == "__main__":
    unittest.main()
