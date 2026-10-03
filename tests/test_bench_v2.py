"""python -m unittest discover -s tests -v"""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from ssb import campaigns, config, e2e, oracle, runner, scenarios  # noqa: E402
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


class FmuTests(unittest.TestCase):
    """The FMU adapter, end to end: FMPy loads an FMI 2.0 co-simulation FMU (built by scripts/build_fmu.py with
    PythonFMU) and the bench drives it as a black box. Skipped if FMPy/PythonFMU are not installed."""

    @classmethod
    def setUpClass(cls):
        try:
            import fmpy  # noqa: F401
            import pythonfmu  # noqa: F401
        except ImportError:
            raise unittest.SkipTest("fmpy / pythonfmu not installed")
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
                raise unittest.SkipTest(f"native core not buildable here: {e}")
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
            ref = campaigns.matrix(self.cfg, lambda sc: ReferenceDUT(self.cfg, frozenset([m]), sc.get("start_kmh", 30) > 0), only=self.keys)
            nd = NativeDUT(self.cfg, frozenset([m]))
            nat = campaigns.matrix(self.cfg, lambda sc: nd, only=self.keys)
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

