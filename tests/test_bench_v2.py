"""python -m unittest discover -s tests -v"""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from ssb import campaigns, config, e2e, oracle, runner  # noqa: E402
from ssb.bus import VirtualBus  # noqa: E402
from ssb.dut import ReferenceDUT  # noqa: E402
from ssb.plant import Actuators, Vehicle  # noqa: E402

CFG = config.load()
REQS = campaigns.requirements()["requirements"]


def by_key(key, cfg=CFG):
    return next(s for s in campaigns.all_scenarios(cfg) if s["key"] == key)


class E2ETests(unittest.TestCase):
    def test_crc_check_values(self):
        self.assertEqual(e2e.crc16_ccitt(b"123456789"), 0x29B1)   # CRC-16/CCITT-FALSE
        self.assertEqual(e2e.crc8_h2f(b"123456789"), 0xDF)        # CRC-8 0x2F (AUTOSAR CRC8H2F)

    def test_profile5_statuses(self):
        rx, p = e2e.Profile5.Receiver(0x1234), b"\x01\x02"
        self.assertEqual(rx.check(e2e.Profile5.protect(p, 5, 0x1234))[0], e2e.OK)
        self.assertEqual(rx.check(e2e.Profile5.protect(p, 5, 0x1234))[0], e2e.REPEATED)
        self.assertEqual(rx.check(e2e.Profile5.protect(p, 7, 0x1234))[0], e2e.OK_SOME_LOST)
        self.assertEqual(rx.check(e2e.Profile5.protect(p, 20, 0x1234))[0], e2e.WRONG_SEQUENCE)
        self.assertEqual(rx.check(e2e.Profile5.protect(p, 21, 0x9999))[0], e2e.WRONG_CRC)  # wrong data ID

    def test_profile2_counter_wraps_at_16(self):
        rx = e2e.Profile2.Receiver()
        statuses = [rx.check(e2e.Profile2.protect(b"\x00", c))[0] for c in range(0, 40)]
        self.assertTrue(all(s == e2e.OK for s in statuses))

    def test_explained_gaps_do_not_double_count_a_crc_burst(self):
        def run(explain):
            rx, p = e2e.Profile5.Receiver(0x1234, max_delta=2, explain_gaps=explain), b"\x01"
            out = [rx.check(e2e.Profile5.protect(p, 1, 0x1234))[0]]
            out += [rx.check(e2e.flip_crc(e2e.Profile5.protect(p, c, 0x1234)))[0] for c in (2, 3)]
            out.append(rx.check(e2e.Profile5.protect(p, 4, 0x1234))[0])   # jump of 3 after two CRC failures
            out.append(rx.check(e2e.Profile5.protect(p, 7, 0x1234))[0])   # unexplained jump of 3: still an error
            return out
        self.assertEqual(run(False)[3], e2e.WRONG_SEQUENCE)                  # v2.0: the burst counts three times
        self.assertEqual(run(True)[3], e2e.OK_SOME_LOST)
        self.assertEqual(run(True)[4], e2e.WRONG_SEQUENCE)

    def test_tuning_monte_carlo_shows_the_gain(self):
        from ssb import e2e_tuning
        old, new = e2e_tuning.CANDIDATES[0], e2e_tuning.CANDIDATES[4]
        self.assertEqual(new["name"], "explained gaps")
        a = e2e_tuning.false_stops(old, 0.01, 100_000)["false_stops"]
        b = e2e_tuning.false_stops(new, 0.01, 100_000)["false_stops"]
        self.assertGreater(a, 3 * max(b, 1))
        self.assertEqual(e2e_tuning.detection(new, 1.0, runs=50)["p99_frames"], 3)   # a dead stream: still 3 frames

    def test_window_catches_scattered_errors(self):
        sm = e2e.E2EStateMachine()
        sm.preset_valid()
        for s in [e2e.WRONG_CRC, e2e.OK, e2e.WRONG_CRC, e2e.OK, e2e.WRONG_CRC]:
            sm.update(s)
        self.assertEqual(sm.state, "INVALID")


class BusAndPlantTests(unittest.TestCase):
    def test_arbitration_lowest_id_first(self):
        bus = VirtualBus("b", frames_per_ms=1, latency_ms=0)
        bus.send(0, "a", 0x300, b"\x01")
        bus.send(0, "b", 0x100, b"\x02")
        self.assertEqual(bus.step(0)[0].can_id, 0x100)

    def test_stopping_distance_physics(self):
        cfg = config.load()
        cfg["plant"]["long_dead_time_ms"], cfg["plant"]["long_tau_s"] = 1, 0.001
        act, veh = Actuators(cfg), Vehicle(cfg, 10.0)
        for _ in range(10000):
            veh.step(act.step(-4.0, 0.0, 0.0, 0.8, 0.001), 0.0, 0.0, 0.001)
        self.assertAlmostEqual(veh.x, 10.0 ** 2 / (2 * 4.0), delta=0.2)   # v²/2a


class DynamicPlantTests(unittest.TestCase):
    """v2.5: force-based plant. Demands are mapped to force on the curb mass, so payload scales deceleration."""

    def _cfg(self, payload=0.0, crr=0.0, cda=0.0):
        cfg = config.load()
        cfg["plant"]["model"] = "dynamic"
        d = cfg["plant"]["dynamic"]
        d["payload_kg"], d["crr"], d["cd_a_m2"], d["rot_inertia_factor"] = payload, crr, cda, 0.0
        return cfg

    def _stop(self, cfg, a_cmd=-4.0, grade=0.0, mu=0.8):
        from ssb.plant import make_vehicle
        veh = make_vehicle(cfg, 10.0)
        veh.mu = mu
        for _ in range(30000):
            veh.step(a_cmd, 0.0, grade, 0.001)
            if veh.v == 0.0:
                break
        return veh

    def test_no_payload_no_losses_matches_v2_over_2a(self):
        self.assertAlmostEqual(self._stop(self._cfg()).x, 10.0 ** 2 / (2 * 4.0), delta=0.05)

    def test_payload_scales_deceleration_by_the_mass_ratio(self):
        cfg = self._cfg(payload=750.0)        # 1500 / 2250 = 2/3 of the demand
        self.assertAlmostEqual(self._stop(cfg).x, 10.0 ** 2 / (2 * 4.0 * 1500 / 2250), delta=0.05)

    def test_tyre_limit_uses_the_loaded_normal_force(self):
        veh = self._stop(self._cfg(), a_cmd=-20.0, mu=0.3)   # demand far above the tyre limit
        self.assertAlmostEqual(veh.x, 10.0 ** 2 / (2 * 0.3 * 9.81), delta=0.05)

    def test_downhill_and_resistances(self):
        flat, down = self._stop(self._cfg()), self._stop(self._cfg(), grade=-6.0)
        self.assertGreater(down.x, flat.x * 1.1)
        self.assertLess(self._stop(self._cfg(crr=0.015, cda=0.7)).x, flat.x)   # rolling resistance + drag help

    def test_kinematic_rejects_a_payload(self):
        from ssb.plant import make_vehicle
        with self.assertRaises(ValueError):
            make_vehicle(config.load(), 10.0, payload_kg=500)

    def test_matrix_unchanged_with_the_dynamic_plant_and_no_payload(self):
        cfg = self._cfg(crr=0.015, cda=0.7)
        rs = campaigns.matrix(cfg, keep_trace=False)
        self.assertEqual([r["key"] for r in rs if r["verdict"]["status"] == "FAIL"], [])

    def test_load_sweep_finding_planned_stop_grows_silently(self):
        cfg = self._cfg(crr=0.015, cda=0.7)
        rows = {r["key"]: r for r in campaigns.load_sweep(cfg, payloads=(0, 800), grades=(0.0,))}
        empty, loaded = rows["command_link_lost@0kg/+0%"], rows["command_link_lost@800kg/+0%"]
        self.assertEqual(loaded["peak_state"], "STOP_IN_LANE")          # no brake diagnosis at +53% mass ...
        self.assertGreater(loaded["stop_dist_m"], empty["stop_dist_m"] * 1.25)   # ... although the stop is >25% longer


class MdfExportTests(unittest.TestCase):
    """v2.7: MDF4 traces (run.py --mdf). Skipped without asammdf."""

    def test_round_trip(self):
        import tempfile

        from ssb import mdf_export
        if not mdf_export.available():
            self.skipTest("asammdf not installed")
        from asammdf import MDF
        r = runner.run(by_key("command_link_lost"), CFG)
        with tempfile.TemporaryDirectory() as tmp:
            p = mdf_export.write(r, Path(tmp) / "x.mf4")
            m = MDF(p)
            try:
                self.assertTrue(m.version.startswith("4."))
                speed, state = m.get("VehSpeed"), m.get("SAF_State")
                self.assertEqual(speed.unit, "km/h")
                self.assertEqual(len(speed.samples), len(r["trace"]))
                self.assertEqual(speed.samples[0], r["trace"][0][1])
                self.assertEqual(state.samples[-1], b"STOP_IN_LANE")       # value-to-text conversion
                self.assertEqual(m.get("SAF_State", raw=True).samples[-1], 4)
            finally:
                m.close()


class MatrixTests(unittest.TestCase):
    def test_default_matrix(self):
        rs = campaigns.matrix(CFG, keep_trace=True)
        bad = [r["key"] for r in rs if r["verdict"]["status"] == "FAIL"]
        self.assertEqual(bad, [])

    def test_known_finding_is_flagged_not_hidden(self):
        sc = by_key("bv_steer_rate_40kmh_16dps")
        v = oracle.verdict(runner.run(sc, CFG), sc, REQS, CFG)
        self.assertEqual(v["status"], "KNOWN")


class MutantTests(unittest.TestCase):
    """Each seeded bug must be caught by the scenario designed for it."""
    CASES = {"e2e_run_length": "intermittent_crc", "no_freshness": "stale_data", "no_qa": "qa_wrong_answer",
             "no_actuator_check": "steering_stuck", "release_unconditional": "release_procedure", "no_watchdog": "planner_looping",
             "long_timeout": "command_link_lost", "no_speed_cap": "perception_degraded", "envelope_off_by_one": "bv_steer_rate_10kmh_49dps"}

    def test_mutants_caught(self):
        for mutant, key in self.CASES.items():
            with self.subTest(mutant):
                sc = by_key(key)
                dut = ReferenceDUT(CFG, frozenset([mutant]), warm_start=sc.get("start_kmh", 30) > 0)
                self.assertFalse(oracle.verdict(runner.run(sc, CFG, dut), sc, REQS, CFG)["passed"])

    def test_grade_compensation_needed_offroad(self):
        cfg = config.load("config/offroad.json")
        sc = by_key("brake_weak", cfg)
        dut = ReferenceDUT(cfg, frozenset(["no_grade_compensation"]))
        r = runner.run(sc, cfg, dut)
        self.assertNotEqual(r["cause"], "BRAKE_ACTUATOR")   # without compensation the grade hides the weak brake


class DutTests(unittest.TestCase):
    def test_optional_adapters_fail_clearly_without_dependencies(self):
        from ssb import dut
        try:
            import fmpy  # noqa: F401
        except ImportError:
            with self.assertRaises(RuntimeError):
                dut.FmuDUT("missing.fmu", {})



class DbcTests(unittest.TestCase):
    """The DBC must describe exactly the bytes the bench packs (checked with cantools, if installed)."""

    def setUp(self):
        try:
            from ssb import canio
            self.db = canio.load_dbc()
        except ImportError:
            self.skipTest("cantools not installed")

    def test_planner_command_layout(self):
        from ssb import planner
        payload = planner.encode_cmd(1234, -1.5, 3.25, 8.33, 0xA7, 2, 1)
        frame = e2e.Profile5.protect(payload, 42, 0x1234) + bytes(2)   # + CAN FD padding to 16
        d = self.db.decode_message(0x100, frame)
        self.assertEqual(d["PLN_E2E_Counter"], 42)
        self.assertEqual(d["PLN_TimeStamp"], 1234)
        self.assertAlmostEqual(d["PLN_AccelReq"], -1.5)
        self.assertAlmostEqual(d["PLN_SteerReq"], 3.25)
        self.assertAlmostEqual(d["PLN_SpeedReq"], 8.33)
        self.assertEqual(d["PLN_WdAnswer"], 0xA7)

    def test_actuator_command_layout(self):
        from ssb import plant
        frame = e2e.Profile2.protect(plant.encode_act(-3.0, -1.2, True), 9)
        d = self.db.decode_message(0x200, frame)
        self.assertEqual(d["SAF_E2E_Counter"], 9)
        self.assertAlmostEqual(d["SAF_AccelCmd"], -3.0)
        self.assertAlmostEqual(d["SAF_SteerCmd"], -1.2)
        self.assertEqual(d["SAF_BackupBrake"], 1)

    def test_status_layout_matches_the_shared_encoder(self):
        """v2.10: SAF_Status built through the DBC (as the vECU process does) is byte for byte the shared E2E layout,
        decodes to the same fields, and passes the receiver."""
        import random
        from ssb import canio
        rng, rx = random.Random(7), e2e.StatusReceiver()
        for k in range(300):
            st, cause, mrm, ch = rng.choice(canio.STATES), rng.choice(canio.CAUSES), rng.random() < 0.5, rng.randrange(256)
            a, s = (rng.uniform(-400, 400), rng.uniform(-400, 400)) if k % 10 == 0 else (rng.uniform(-9, 9), rng.uniform(-30, 30))
            f = canio.encode_status(self.db, k, st, cause, ch, mrm, a, s)
            self.assertEqual(f, e2e.status_protect(k, canio.STATES.index(st), mrm, canio.CAUSES.index(cause), ch, a, s))
            self.assertTrue(rx.check(f), k)
            d, u = self.db.decode_message(0x201, f, decode_choices=False), e2e.status_unpack(f)
            self.assertEqual((d["SAF_Status_Counter"], d["SAF_State"], d["SAF_Cause"], d["SAF_MrmRequest"], d["SAF_WdChallenge"]),
                             (k & 0xFF, canio.STATES.index(st), canio.CAUSES.index(cause), int(mrm), ch))
            self.assertAlmostEqual(d["SAF_AccelOut"], u["accel"])
            self.assertAlmostEqual(d["SAF_SteerOut"], u["steer"])
        self.assertEqual(rx.n_crc + rx.n_seq, 0)


class PythonFmuIsolated(unittest.TestCase):
    """Runs FmuTests in a CHILD process. The PythonFMU DLLs embed a Python interpreter; after they are freed, the
    parent sometimes segfaulted later in an unrelated test (about 1 run in 3, v2.4-v2.6). Isolating them keeps a
    crash inside the embedded interpreter from taking the whole suite down, and still reports it as a failure."""

    def test_python_fmu_suite_in_a_child_process(self):
        import os
        import subprocess
        r = subprocess.run([sys.executable, "-m", "unittest", "test_bench_v2.FmuTests"], cwd=str(Path(__file__).parent),
                           env={**os.environ, "SSB_FMU_CHILD": "1"}, capture_output=True, text=True, timeout=1200)
        tail = (r.stdout + r.stderr)[-1500:]
        self.assertEqual(r.returncode, 0, tail)
        self.assertIn("OK", tail)


class FmuTests(unittest.TestCase):
    """The FMU adapter, end to end: FMPy loads an FMI 2.0 co-simulation FMU (built by scripts/build_fmu.py with
    PythonFMU) and the bench drives it as a black box. Skipped if FMPy/PythonFMU are not installed.
    Runs only in the child process started by PythonFmuIsolated."""

    @classmethod
    def setUpClass(cls):
        import os
        if os.environ.get("SSB_FMU_CHILD") != "1":
            raise unittest.SkipTest("runs in a child process (PythonFmuIsolated)")
        try:
            import fmpy  # noqa: F401
            import pythonfmu  # noqa: F401
        except ImportError:
            raise unittest.SkipTest("fmpy / pythonfmu not installed") from None
        sys.path.insert(0, str(ROOT / "scripts"))
        import build_fmu
        fmu = ROOT / "fmu" / "SafeStopVecu.fmu"
        newest_src = max(f.stat().st_mtime for f in [*(ROOT / "ssb").glob("*.py"), *(ROOT / "fmu").glob("*.py")])
        if not fmu.exists() or fmu.stat().st_mtime < newest_src:
            build_fmu.build()
        from ssb.dut import FmuDUT
        cls.fmu, cls.supplier = fmu, ROOT / "fmu" / "SupplierStyleVecu.fmu"
        cls.cfg = config.load("config/default.json")
        cls.dut = FmuDUT(str(fmu))
        cls.keys = ["command_link_lost", "planner_hang", "crc_corrupt", "steering_stuck", "release_procedure", "safety_brownout",
                    "reorder_once", "jitter_above_period", "startup_no_frames", "startup_normal", "actuator_bus_off", "counter_wrap"]

    @classmethod
    def tearDownClass(cls):
        cls.dut.close()

    def test_exactly_matches_the_reference(self):
        rows = campaigns.back_to_back(self.cfg, lambda sc: self.dut, only=self.keys, exact=True)
        self.assertTrue(rows)
        self.assertEqual([r for r in rows if not r["match"]], [])

    def test_a_seeded_bug_inside_the_fmu_is_caught(self):
        self.dut.defects = frozenset(["no_latch"])
        try:
            res = campaigns.matrix(self.cfg, lambda sc: self.dut, only=["command_link_lost", "release_procedure"])
        finally:
            self.dut.defects = frozenset()
        self.assertFalse(all(r["verdict"]["passed"] for r in res))

    def test_mapping_errors_are_listed_not_crashed(self):
        from ssb import fmu_contract
        from ssb.dut import FmuDUT
        m = fmu_contract.identity_mapping()
        m["inputs"]["VEH_Speed"] = "NoSuchVariable"
        del m["outputs"]["SAF_State"]
        with self.assertRaises(ValueError) as cm:
            FmuDUT(str(self.fmu), m)
        self.assertIn("NoSuchVariable", str(cm.exception))
        self.assertIn("SAF_State: not mapped", str(cm.exception))

    def test_supplier_style_fmu_through_a_mapping_file(self):
        import json

        from ssb.dut import FmuDUT
        mapping = json.loads((ROOT / "fmu" / "mapping_supplier_style.json").read_text(encoding="utf-8"))
        d = FmuDUT(str(self.supplier), mapping)
        try:
            rows = campaigns.back_to_back(self.cfg, lambda sc: d, only=["command_link_lost", "steering_stuck", "brake_weak"], exact=True)
        finally:
            d.close()
        self.assertEqual([r for r in rows if not r["match"]], [])

    def test_inspect_proposes_most_of_the_supplier_mapping(self):
        from fmpy import read_model_description

        from ssb import fmu_inspect
        mapping, notes = fmu_inspect.propose_mapping(read_model_description(str(self.supplier)).modelVariables)
        self.assertFalse([n for n in notes if n.startswith("MISSING")], notes)
        self.assertEqual(mapping["inputs"]["PLN_E2E_CRC"], "PlnCmd_Crc")


class CFmuTests(unittest.TestCase):
    """The compiled FMU (scripts/build_c_fmu.py): the C++ core behind the FMI 2.0 C API, no Python inside. Same
    variables as the PythonFMU reference, so the identity mapping drives it. Skipped without FMPy or ziglang."""

    @classmethod
    def setUpClass(cls):
        try:
            import fmpy  # noqa: F401
            import ziglang  # noqa: F401
        except ImportError:
            raise unittest.SkipTest("fmpy / ziglang not installed") from None
        sys.path.insert(0, str(ROOT / "scripts"))
        import build_c_fmu
        srcs = [*(ROOT / "hil" / "SafeStopCore" / "src").glob("ssc_core.*"), ROOT / "fmu" / "c_src" / "SafeStopVecuC.cpp",
                ROOT / "scripts" / "build_c_fmu.py", ROOT / "ssb" / "fmu_contract.py", *(ROOT / "config").glob("*.json")]
        if not build_c_fmu.OUT.exists() or build_c_fmu.OUT.stat().st_mtime < max(f.stat().st_mtime for f in srcs):
            build_c_fmu.build()
        from ssb.dut import FmuDUT
        cls.fmu = build_c_fmu.OUT
        cls.cfg = config.load("config/default.json")
        cls.dut = FmuDUT(str(cls.fmu))
        cls.keys = [
            "command_link_lost", "planner_hang", "crc_corrupt", "steering_stuck", "release_procedure", "safety_brownout",
            "reorder_once", "jitter_above_period", "startup_no_frames", "startup_normal", "actuator_bus_off", "counter_wrap"]

    @classmethod
    def tearDownClass(cls):
        cls.dut.close()

    def test_no_python_inside(self):
        import zipfile
        names = zipfile.ZipFile(self.fmu).namelist()
        self.assertIn("binaries/win64/SafeStopVecuC.dll", names)
        self.assertFalse([n for n in names if n.endswith(".py") or "python" in n.lower()], names)

    def test_exactly_matches_the_reference(self):
        rows = campaigns.back_to_back(self.cfg, lambda sc: self.dut, only=self.keys, exact=True)
        self.assertTrue(rows)
        self.assertEqual([r for r in rows if not r["match"]], [])

    def test_offroad_calibration_is_selected_by_bench_config(self):
        from ssb.dut import FmuDUT
        cfg = config.load("config/offroad.json")
        d = FmuDUT(str(self.fmu), config_name="offroad")
        try:
            rows = campaigns.back_to_back(cfg, lambda sc: d, only=["command_link_lost", "steering_stuck"], exact=True)
        finally:
            d.close()
        self.assertEqual([r for r in rows if not r["match"]], [])

    def test_a_seeded_bug_inside_the_fmu_is_caught(self):
        self.dut.defects = frozenset(["no_latch"])
        try:
            res = campaigns.matrix(self.cfg, lambda sc: self.dut, only=["command_link_lost", "release_procedure"])
        finally:
            self.dut.defects = frozenset()
        self.assertFalse(all(r["verdict"]["passed"] for r in res))

    def test_lifecycle_intake_all_ok(self):
        from ssb import fmu_inspect
        for title, (ops, _) in fmu_inspect.LIFECYCLE.items():
            import subprocess
            r = subprocess.run([sys.executable, "-c", fmu_inspect._CHILD, str(self.fmu), ops], capture_output=True, text=True,
                               timeout=120)
            self.assertIn("OK", r.stdout, f"{title}: {r.stderr[-500:]}")

    def test_unknown_config_name_fails_initialisation(self):
        from fmpy.fmi1 import FMICallException

        from ssb.dut import FmuDUT
        with self.assertRaises(FMICallException):        # fmi2ExitInitializationMode returns fmi2Error
            FmuDUT(str(self.fmu), config_name="no_such_config")


class Ros2Tests(unittest.TestCase):
    """v2.8: the safety controller as a ROS 2 node in its own process, driven over topics in real time.
    Runs where rclpy is importable (WSL/Linux after sourcing ROS 2); skipped on the Windows venv."""

    @classmethod
    def setUpClass(cls):
        try:
            import rclpy  # noqa: F401
        except ImportError:
            raise unittest.SkipTest("rclpy not available (source /opt/ros/<distro>/setup.bash in WSL/Linux)") from None
        from ssb.ros2_dut import Ros2DUT
        cls.cfg = config.load("config/default.json")
        cls.dut = Ros2DUT()

    @classmethod
    def tearDownClass(cls):
        cls.dut.close()

    def test_reactions_match_the_reference(self):
        keys = ["command_link_lost", "crc_corrupt", "planner_hang", "steering_stuck", "single_bad_frame", "crc_burst_two"]
        rows = campaigns.back_to_back(self.cfg, lambda sc: self.dut, only=keys)
        self.assertEqual([r["key"] for r in rows if not r["match"]], [])

    def test_a_seeded_bug_inside_the_node_is_caught(self):
        self.dut.defects = frozenset(["no_latch"])
        try:
            res = campaigns.matrix(self.cfg, lambda sc: self.dut, only=["release_procedure"])
        finally:
            self.dut.defects = frozenset()
        self.assertFalse(all(r["verdict"]["passed"] for r in res))


class SlewTimeBaseTests(unittest.TestCase):
    """v2.9.1: the HiL steering-slew finding. A reference DUT whose outputs reach the bench like B's do over a link:
    one 10 ms frame every 40 ms arrives 1 ms late. On the PC's 20 ms grid that fakes a 72 deg/s slew; on the DUT's
    own cycle clock it does not, and a really fast slew is still caught."""

    class LinkLike(ReferenceDUT):
        def __init__(self, cfg, gain=1.0, stamp=True):
            self.gain, self.stamp = gain, stamp
            super().__init__(cfg)

        def reset(self):
            super().reset()
            self.q, self.cmd, self.cyc = [], (0.0, 0.0, False), None

        def step(self, t, *a):
            from dataclasses import replace
            o = super().step(t, *a)
            if t % 10 == 0:
                late = 1 if (t // 10) % 4 == 0 else 0
                cmd = (o.out_cmd[0], o.out_cmd[1] * self.gain, o.out_cmd[2])
                self.q.append((t + late, cmd, (t, cmd[1])))
            while self.q and self.q[0][0] <= t:
                _, self.cmd, self.cyc = self.q.pop(0)
            return replace(o, out_cmd=self.cmd, cycle=self.cyc if self.stamp else None)

    def run_with(self, **kw):
        sc = by_key("bv_steer_rate_10kmh_49dps")
        return runner.run(sc, CFG, self.LinkLike(CFG, **kw), keep_trace=False)["invariant_violations"]

    def test_pc_grid_turns_1ms_jitter_into_a_false_slew(self):
        self.assertTrue(any("slew 72" in v for v in self.run_with(stamp=False)))

    def test_dut_cycle_clock_is_immune_to_link_jitter(self):
        self.assertEqual(self.run_with(), [])

    def test_dut_cycle_clock_still_catches_a_fast_slew(self):
        self.assertTrue(any("slew" in v for v in self.run_with(gain=3.0)))


class ReplayFilterTests(unittest.TestCase):
    """v2.9.3: B's MCP2515 replays old SAF_Status frames from a transmit buffer. The observer accepts a bus frame only
    if B's own USB copy shows it sent those bytes within 20 ms, and each copy vouches for one bus frame."""

    def setUp(self):
        from ssb import hil
        self.d = object.__new__(hil.LinkDUT)
        self.d.mirror = []

    def test_fresh_frame_accepted_once(self):
        f = bytes([0x01, 0x02, 8, 1, 0, 0x5A, 0, 0, 0, 0, 0])
        self.d.mirror.append([100, f, False])
        self.assertTrue(self.d._sent_by_b(101, f))
        self.assertFalse(self.d._sent_by_b(102, f))      # a second copy of the same frame is a replay

    def test_old_frame_rejected(self):
        old = bytes([0x01, 0x02, 8, 4, 7, 0x71, 0, 0x9C, 0xFF, 0, 0])
        self.d.mirror.append([100, bytes([0x01, 0x02, 8, 1, 0, 0x5A, 0, 0, 0, 0, 0]), False])
        self.assertFalse(self.d._sent_by_b(101, old))
        self.d.mirror.append([100, old, False])
        self.assertFalse(self.d._sent_by_b(130, old))    # sent, but more than 20 ms ago

    def test_copy_arriving_after_the_bus_frame_still_vouches(self):
        f = bytes([0x01, 0x02, 8, 1, 0, 0x5A, 0, 0, 0, 0, 0])
        self.d.mirror.append([105, f, False])           # B's port read late: its copy is 4 ms behind the bus copy
        self.assertTrue(self.d._sent_by_b(101, f))

    def test_no_mirror_means_no_filtering(self):
        self.assertTrue(self.d._sent_by_b(5, bytes(11)))


class HostStallRerunTests(unittest.TestCase):
    """v2.9.5: a host stall (bench lag over the limit) is a bench fault: the scenario is rerun, and a verdict is never
    taken from a run that stalled."""

    def run_with_lags(self, lags):
        calls = iter(lags)
        orig = runner._run_once_gc_paused
        runner._run_once_gc_paused = lambda *a, **k: {"key": "x", "bench_lag_ms": next(calls), "bench_lag_at_ms": 1234}
        try:
            return runner.run({"key": "x"}, CFG, ReferenceDUT(CFG))
        finally:
            runner._run_once_gc_paused = orig

    def test_stall_then_clean_run_keeps_the_clean_verdict(self):
        r = self.run_with_lags([12.5, 1.1])
        self.assertEqual(r["bench_lag_ms"], 1.1)
        self.assertEqual(len(r["bench_faults"]), 1)
        self.assertIn("host stall", r["bench_faults"][0])
        self.assertIn("1234", r["bench_faults"][0])

    def test_every_attempt_stalls_so_the_lag_check_still_fails_it(self):
        r = self.run_with_lags([9.0, 8.0, 7.0, 1.0])
        self.assertEqual(r["bench_lag_ms"], 7.0)              # retries used up: last attempt returned, not the 4th
        self.assertEqual(len(r["bench_faults"]), runner.BENCH_RETRIES)

    def test_no_stall_no_rerun(self):
        r = self.run_with_lags([4.9])
        self.assertEqual(r["bench_faults"], [])

    def test_offline_dut_never_reruns(self):
        r = self.run_with_lags([None])
        self.assertEqual(r["bench_faults"], [])


class StatusE2ETests(unittest.TestCase):
    """v2.9.6: SAF_Status carries CRC-8 + an 8-bit alive counter; the observer rejects corrupted, repeated and stale
    frames itself, and resynchronises after the DUT restarts its counter."""

    @staticmethod
    def frame(ctr, state=1, cause=0, mrm=0, ch=0x5A, a=0, s=0):
        import struct
        body = struct.pack("<BBBhh", ctr & 0xFF, (state & 7) | (mrm << 3) | (cause << 4), ch, a, s)
        return bytes([e2e.crc8_h2f(body + b"")]) + body

    def setUp(self):
        from ssb import hil
        self.d = object.__new__(hil.LinkDUT)
        self.d.status_rx = e2e.StatusReceiver()

    def test_in_sequence_and_one_lost_frame_accepted(self):
        self.assertTrue(all(self.d._status_e2e_ok(self.frame(c)) for c in (250, 251, 252, 254, 255, 0, 1)))

    def test_repeat_rejected(self):
        self.d._status_e2e_ok(self.frame(7))
        self.assertFalse(self.d._status_e2e_ok(self.frame(7)))

    def test_stale_copy_rejected(self):
        for c in range(10, 20):
            self.d._status_e2e_ok(self.frame(c))
        self.assertFalse(self.d._status_e2e_ok(self.frame(140, state=4, cause=7)))   # an old STOP status
        self.assertTrue(self.d._status_e2e_ok(self.frame(20)))                      # the stream carries on

    def test_corrupted_crc_rejected(self):
        f = bytearray(self.frame(3))
        f[2] ^= 0x04
        self.assertFalse(self.d._status_e2e_ok(bytes(f)))
        self.assertEqual(self.d.n_status_crc, 1)

    def test_resync_after_dut_restart(self):
        for c in range(100, 105):
            self.d._status_e2e_ok(self.frame(c))
        self.assertFalse(self.d._status_e2e_ok(self.frame(0)))   # counter restarted: one frame rejected
        self.assertTrue(self.d._status_e2e_ok(self.frame(1)))    # two in a row: resynchronised
        self.assertTrue(self.d._status_e2e_ok(self.frame(2)))


    def test_restart_after_announced_off_accepted_at_once(self):
        for c in range(30, 33):
            self.d._status_e2e_ok(self.frame(c))
        self.assertTrue(self.d._status_e2e_ok(self.frame(33, state=7)))   # DUT reports OFF (power loss)
        self.assertTrue(self.d._status_e2e_ok(self.frame(0, state=4)))    # restarted counter: accepted, no lost cycle


class StatusLayoutAcrossLevelsTests(unittest.TestCase):
    """v2.10: one SAF_Status layout at every level. The C++ core's real frames (the code that runs on board B, here in
    the host DLL over the link protocol) re-encode byte for byte through the DBC; and on the CAN-process level a stale
    status replayed on the bus is rejected by the alive counter (with the check off, the same replay breaks the latch)."""

    def test_cpp_core_frames_match_the_dbc(self):
        from ssb import canio, hil
        try:
            d, db = hil.make("loopback", CFG), canio.load_dbc()
        except Exception as exc:   # no compiler / DLL, or no cantools
            self.skipTest(f"loopback DUT unavailable: {exc}")
        frames, check = [], d._status_e2e_ok
        d._status_e2e_ok = lambda data: (frames.append(bytes(data)), check(data))[1]
        try:
            r = runner.run(by_key("command_link_lost"), CFG, d, keep_trace=False)
        finally:
            d.close()
        self.assertGreater(len(frames), 500)
        seen = set()
        for f in frames:
            u = e2e.status_unpack(f)
            seen.add(canio.STATES[u["state"]])
            self.assertEqual(canio.encode_status(db, u["counter"], canio.STATES[u["state"]], canio.CAUSES[u["cause"]],
                                                 u["challenge"], u["mrm"], u["accel"], u["steer"]), f)
        self.assertTrue({"NORMAL", "STOP_IN_LANE"} <= seen, seen)
        self.assertEqual(r["status_e2e_rejects"], 0)

    def test_can_process_rejects_a_replayed_status(self):
        from ssb.dut import CanDUT
        sc, replay = by_key("command_link_lost"), {"capture_ms": 1500, "inject_ms": [5005, 6005, 7005]}
        out = {}
        for on in (True, False):
            try:
                d = CanDUT(status_replay=replay, status_e2e=on)
            except Exception as exc:   # no cantools / python-can, or no multicast route
                self.skipTest(f"CAN-process DUT unavailable: {exc}")
            try:
                r = runner.run(sc, CFG, d, keep_trace=False)   # host stalls rerun; neither check below depends on timing
                out[on] = (r, d.n_replays_sent)
            finally:
                d.close()
        r, sent = out[True]
        self.assertEqual(sent, 3)
        self.assertGreaterEqual(r["status_e2e_rejects"], 3)   # the stale NORMAL copies (counter from t = 1500 ms)
        self.assertEqual(r["invariant_count"], 0)
        r, sent = out[False]
        self.assertEqual(sent, 3)
        self.assertTrue(any("left a latched stop" in v for v in r["invariant_violations"]), r["invariant_violations"])


class CanLockstepTests(unittest.TestCase):
    """v2.11: the CAN-process bench in lockstep. The bench waits at each 10 ms boundary for that cycle's status (the
    alive counter says which cycle), so a host stall slows the run but can't change it: same timeline as the in-process
    reference on the 10 ms grid, same vehicle end state, no bench fault, and the stale-status replay still rejected."""

    def make(self, **kw):
        from ssb.dut import CanDUT
        try:
            return CanDUT(lockstep=True, **kw)
        except Exception as exc:   # no cantools / python-can, or no multicast route
            self.skipTest(f"CAN-process DUT unavailable: {exc}")

    def test_a_host_stall_does_not_change_the_result(self):
        import time as _t
        sc = by_key("planner_hang")
        d = self.make()
        step = d.step

        def stalled(t, *a):
            if t in (1500, 2005, 2040):
                _t.sleep(0.5)   # the laptop freezes for half a second, around the fault and the reaction
            return step(t, *a)
        d.step = stalled
        try:
            rc = runner.run(sc, CFG, d, keep_trace=False)
        finally:
            d.close()
        rr = runner.run(sc, CFG, keep_trace=False)
        self.assertEqual([tuple(x) for x in rc["states"]], [(-(-t // 10) * 10, st) for t, st in rr["states"]])
        self.assertEqual((rc["v_end_kmh"], rc["max_lateral_m"]), (rr["v_end_kmh"], rr["max_lateral_m"]))
        self.assertEqual(rc["bench_faults"], [])
        self.assertIsNone(rc["bench_lag_ms"])   # no real-time guard in lockstep
        self.assertEqual(oracle.verdict(rc, sc, campaigns.requirements()["requirements"], CFG)["status"], "PASS")

    def test_replayed_status_still_rejected(self):
        d = self.make(status_replay={"capture_ms": 1500, "inject_ms": [5005, 6005, 7005]})
        try:
            r = runner.run(by_key("command_link_lost"), CFG, d, keep_trace=False)
            sent = d.n_replays_sent
        finally:
            d.close()
        self.assertEqual(sent, 3)
        self.assertGreaterEqual(r["status_e2e_rejects"], 3)
        self.assertEqual(r["invariant_count"], 0)
        self.assertEqual(r["bench_faults"], [])


class HwResetTests(unittest.TestCase):
    """v2.9.7: a real reset of the safety controller. On the ESP32 the bench pulls EN; any other DUT gets a power
    loss as long as the board's measured boot (dut_hw.reset_boot_ms), then must come back latched (SAFETY_RESET)."""

    def test_emulated_reset_comes_back_latched(self):
        sc = by_key("safety_hw_reset")
        r = runner.run(sc, CFG, keep_trace=False)
        boot = CFG["dut_hw"]["reset_boot_ms"]
        self.assertEqual(r["states"], [(0, "NORMAL"), (2000, "OFF"), (2000 + boot, "STOP_IN_LANE")])
        self.assertEqual(r["cause"], "SAFETY_RESET")
        self.assertTrue(r["ecu_fallback"])                 # the actuator ECU brakes on its own while it boots
        self.assertTrue(oracle.verdict(r, sc, REQS, CFG)["passed"])

    def test_a_dut_that_forgets_the_latch_fails(self):
        sc = by_key("safety_hw_reset")
        r = runner.run(sc, CFG, ReferenceDUT(CFG, frozenset({"no_latch"})), keep_trace=False)
        self.assertFalse(oracle.verdict(r, sc, REQS, CFG)["passed"])


class NativeHilTests(unittest.TestCase):
    """The ESP32 firmware's C++ (hil/SafeStopCore) built for the PC: the controller alone, and board B's whole node
    behind the link protocol. Skipped if the DLL can't be built (needs: pip install ziglang)."""

    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(ROOT / "scripts"))
        from ssb import native
        src = ROOT / "hil" / "SafeStopCore" / "src"
        newest = max(f.stat().st_mtime for f in src.glob("*.*"))
        if not native.DLL.exists() or native.DLL.stat().st_mtime < newest:
            try:
                import build_native
                build_native.build()
            except Exception as e:  # noqa: BLE001
                raise unittest.SkipTest(f"native core not buildable here: {e}") from e
        cls.cfg = config.load("config/default.json")
        cls.keys = ["command_link_lost", "planner_hang", "planner_looping", "crc_corrupt", "steering_stuck", "brake_weak",
                    "release_procedure", "safety_brownout", "reorder_once", "startup_no_frames", "odd_exit", "counter_wrap"]

    def test_cpp_core_exactly_matches_python(self):
        from ssb.native import NativeDUT
        d = NativeDUT(self.cfg)
        rows = campaigns.back_to_back(self.cfg, lambda sc: d, only=self.keys, exact=True)
        d.close()
        self.assertEqual([r for r in rows if not r["match"]], [])

    def test_node_over_the_link_protocol_matches_on_the_status_grid(self):
        from ssb import hil
        d = hil.make("loopback", self.cfg)
        rows = campaigns.back_to_back(self.cfg, lambda sc: d, only=self.keys, exact=True, quantum_ms=10)
        d.close()
        self.assertEqual([r for r in rows if not r["match"]], [])

    def test_seeded_bugs_behave_the_same_in_cpp(self):
        from ssb.dut import ReferenceDUT
        from ssb.native import NativeDUT
        for m in ("no_latch", "e2e_run_length", "no_grade_compensation"):
            ref = campaigns.matrix(self.cfg, lambda sc, m=m: ReferenceDUT(self.cfg, frozenset([m]), sc.get("start_kmh", 30) > 0), only=self.keys)
            nd = NativeDUT(self.cfg, frozenset([m]))
            nat = campaigns.matrix(self.cfg, lambda sc, d=nd: d, only=self.keys)
            nd.close()
            self.assertEqual([r["verdict"]["status"] for r in ref], [r["verdict"]["status"] for r in nat], m)

    def test_link_framing_and_resync(self):
        from ssb import hil
        from ssb.native import LoopbackLink
        link, par = LoopbackLink(), hil.Parser()
        link.poll(0)
        self.assertIn("H", [k for k, _ in par.feed(link.read())])         # hello while not reset
        good = hil.frame_msg("K", bytes([1]))
        bad = bytearray(good); bad[-1] ^= 0xFF                               # wrong CRC
        par2 = hil.Parser()
        self.assertEqual(par2.feed(bytes(bad) + bytes([0, 0x13]) + good), [("K", bytes([1]))])
        self.assertEqual(par2.errors, 1)
        link.close()

    def test_hardware_modes_fail_clearly_without_ports(self):
        from ssb import hil
        with self.assertRaises(ValueError):
            hil.make("pil", self.cfg)
        with self.assertRaises(ValueError):
            hil.make("hil", self.cfg)

    def test_realtime_path_with_emulated_boards(self):
        """The hardware path minus the hardware: B (+ A) emulated in a separate process on its own clock, reached
        through the serial link (socket://), paced in real time. Catches link, handshake and timing bugs."""
        try:
            import serial  # noqa: F401
        except ImportError:
            self.skipTest("pyserial not installed")
        from ssb import hil
        d = hil.make("hil", self.cfg, port_b="EMU", port_a="EMU")
        try:
            res = campaigns.matrix(self.cfg, lambda sc: d, only=["command_link_lost"])
        finally:
            d.close()
        r = res[0]
        # the controller checks must pass; the bench-lag check depends on the PC (battery, load), so it's reported only
        failed = [c for c, ok in r["verdict"]["checks"] if not ok and not c.startswith("bench kept real time")]
        self.assertEqual(failed, [])
        self.assertIsNotNone(r["bench_lag_ms"])
        print(f"\n  emulated HiL run: bench lag {r['bench_lag_ms']} ms (<= 5 ms needed for a trusted verdict)")


if __name__ == "__main__":
    unittest.main()

