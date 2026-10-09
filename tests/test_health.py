"""P2 bench health: UDS responder (the firmware's own C++ header, through the native DLL), the PC tester, preflight, dashboard."""
from __future__ import annotations

import ctypes
import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from health import dashboard, preflight  # noqa: E402
from health.uds import Tester, read_identity, uds_report  # noqa: E402
from ssb.hil import Parser, frame_msg  # noqa: E402

INV = {"board_b": {"firmware": "SafetyNode 2.7", "port_hint": "COM13"}, "board_a": {"firmware": "BusNode 2.8", "port_hint": "COM14"}}


def native():
    from ssb import native as n
    if not n.DLL.exists() or n.DLL.stat().st_mtime < (ROOT / "hil" / "SafeStopCore" / "src" / "ssc_uds.h").stat().st_mtime:
        try:
            import build_native
            build_native.build()
        except Exception as e:   # noqa: BLE001
            raise unittest.SkipTest(f"native core not buildable here: {e}") from e
    d = ctypes.CDLL(str(n.DLL))
    d.ssc_uds_respond.restype = ctypes.c_int
    d.ssc_uds_respond.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_uint32, ctypes.c_uint16, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p]
    return d


def respond(dll, which, rx_id, data, version=b"2.7", serial=0xAABBCCDD):
    out = ctypes.create_string_buffer(8)
    n = dll.ssc_uds_respond(which, version, serial, rx_id, bytes(data), len(data), out)
    return out.raw[:n]


class UdsResponderTests(unittest.TestCase):
    """The same ssc_uds.h that is flashed to the boards."""

    def setUp(self):
        self.dll = native()

    def test_tester_present_version_serial(self):
        self.assertEqual(respond(self.dll, 0, 0x7E2, [2, 0x3E, 0]), bytes([2, 0x7E, 0]))
        self.assertEqual(respond(self.dll, 0, 0x7E2, [3, 0x22, 0xF1, 0x95]), bytes([6, 0x62, 0xF1, 0x95]) + b"2.7")
        self.assertEqual(respond(self.dll, 0, 0x7E2, [3, 0x22, 0xF1, 0x8C]), bytes([7, 0x62, 0xF1, 0x8C, 0xAA, 0xBB, 0xCC, 0xDD]))

    def test_suppress_positive_response_and_other_node(self):
        self.assertEqual(respond(self.dll, 0, 0x7E2, [2, 0x3E, 0x80]), b"")
        self.assertEqual(respond(self.dll, 1, 0x7E2, [2, 0x3E, 0]), b"", "A must not answer B's address")
        self.assertEqual(respond(self.dll, 1, 0x7E3, [2, 0x3E, 0]), bytes([2, 0x7E, 0]))

    def test_negative_responses(self):
        self.assertEqual(respond(self.dll, 0, 0x7E2, [3, 0x22, 0x12, 0x34]), bytes([3, 0x7F, 0x22, 0x31]))
        self.assertEqual(respond(self.dll, 0, 0x7E2, [2, 0x10, 1]), bytes([3, 0x7F, 0x10, 0x11]))
        self.assertEqual(respond(self.dll, 0, 0x7E2, [2, 0x22, 0xF1]), bytes([3, 0x7F, 0x22, 0x13]))
        self.assertEqual(respond(self.dll, 0, 0x7E2, [2, 0x3E, 0x05]), bytes([3, 0x7F, 0x3E, 0x12]))

    def test_functional_requests_stay_silent_for_what_the_node_does_not_offer(self):
        self.assertEqual(respond(self.dll, 0, 0x7DF, [3, 0x22, 0x12, 0x34]), b"")
        self.assertEqual(respond(self.dll, 0, 0x7DF, [2, 0x10, 1]), b"")
        self.assertEqual(respond(self.dll, 0, 0x7DF, [2, 0x3E, 0]), bytes([2, 0x7E, 0]))

    def test_not_a_single_frame_is_ignored(self):
        self.assertEqual(respond(self.dll, 0, 0x7E2, [0x10, 0x08, 0x22, 0xF1, 0x95]), b"")


class FakeBus:
    """Board A's USB link with both boards behind it: a 'T' frame is answered by the native UDS responder, as 'M' frames."""

    def __init__(self, dll, b_version=b"2.7", a_version=b"2.8", silent_b=False):
        self.dll, self.parser, self.rx = dll, Parser(), bytearray()
        self.versions, self.silent_b = {0: b_version, 1: a_version}, silent_b

    def write(self, data, now_ms):
        for kind, p in self.parser.feed(data):
            if kind != "T":
                continue
            cid, dlc = struct.unpack("<HB", p[:3])
            for which, resp in ((0, 0x7EA), (1, 0x7EB)):
                if which == 0 and self.silent_b:
                    continue
                out = respond(self.dll, which, cid, p[3:3 + dlc], self.versions[which], 0x91F61B44 + which)
                if out:
                    self.rx += frame_msg("M", struct.pack("<HB", resp, len(out)) + out)

    def read(self):
        out, self.rx = bytes(self.rx), bytearray()
        return out


class UdsTesterTests(unittest.TestCase):
    def setUp(self):
        self.dll = native()

    def test_reads_both_boards(self):
        r = uds_report(FakeBus(self.dll), INV, {"board_b": "SafetyNode 2.7 (B)", "board_a": "BusNode 2.8 (A)"})
        self.assertEqual(r["problems"], [])
        self.assertEqual(r["boards"]["board_b"]["version"], "2.7")
        self.assertEqual(r["boards"]["board_a"]["serial"], "91F61B45")

    def test_old_firmware_is_named(self):
        r = uds_report(FakeBus(self.dll, b_version=b"2.6"), INV)
        self.assertTrue(any("UDS says 2.6, the inventory expects 2.7" in p for p in r["problems"]), r["problems"])

    def test_uds_and_hello_that_disagree_are_not_trusted(self):
        r = uds_report(FakeBus(self.dll), INV, {"board_b": "SafetyNode 2.6 (B)", "board_a": "BusNode 2.8 (A)"})
        self.assertTrue(any("the two disagree" in p for p in r["problems"]), r["problems"])

    def test_silent_board_has_no_uds_response(self):
        ident = read_identity(Tester(FakeBus(self.dll, silent_b=True), timeout_s=0.05), "board_b")
        self.assertFalse(ident["alive"])
        self.assertIn("no UDS response", ident["problems"][0])


class FakeLink:
    def close(self):
        pass


def world(ports=("COM13", "COM14"), hellos=None, uds=None, **over):
    hellos = hellos or {"COM13": "SafetyNode 2.7 (B) CAN 8MHz restored", "COM14": "BusNode 2.8 (A) CAN 8MHz"}
    w = preflight.World()
    w.list_ports = lambda: [{"port": p, "description": f"USB-SERIAL CH340 ({p})", "vid": 0x1A86, "pid": 0x7523} for p in ports]
    w.open_link = lambda port: type("L", (FakeLink,), {"port": port})()
    w.read_hello = lambda link, timeout_s=3.0: hellos.get(link.port)
    w.uds_report = lambda link, inv, h: uds or {"boards": {"board_b": {"version": "2.7", "serial": "1"}, "board_a": {"version": "2.8", "serial": "2"}}, "problems": []}
    w.source_version = lambda role: "2.7" if role == "board_b" else "2.8"
    w.compile_sketch = lambda role: {"ok": True, "output": "", "percent": 26, "max": 1310720, "used": 340000,
                                     "partitions": "nvs,data,nvs,0x9000,0x5000,\napp0,app,ota_0,0x10000,0x140000,\nspiffs,data,spiffs,0x290000,0x160000,\n"}
    w.upload_probe = lambda port: {"ok": True, "output": "", "flash_mb": 16}
    for k, v in over.items():
        setattr(w, k, v)
    return w


class PreflightTests(unittest.TestCase):
    def run_pf(self, level="quick", **kw):
        return preflight.run_preflight(level, INV, world(**kw), write=False)

    def test_healthy_bench_is_ready(self):
        r = self.run_pf()
        self.assertTrue(r["ok"], r["checks"])
        self.assertEqual([c["name"] for c in r["checks"]][-1], "uds")

    def test_missing_port_blocks_with_fixes_from_the_catalogue(self):
        r = self.run_pf(ports=("COM13",))
        bad = next(c for c in r["checks"] if c["status"] == "fail")
        self.assertIn("COM14", bad["detail"])
        self.assertTrue(any("charge-only" in f for f in bad["fixes"]))
        self.assertFalse(r["ok"])

    def test_stale_firmware_blocks_and_is_a_suspect_bench_reason(self):
        r = self.run_pf(hellos={"COM13": "SafetyNode 2.5 (B) CAN 8MHz", "COM14": "BusNode 2.8 (A) CAN 8MHz"})
        self.assertFalse(r["ok"])
        self.assertIn("SafetyNode 2.5 is not the inventory's SafetyNode 2.7", r["suspect_bench"][0])

    def test_silent_board_is_recovered_with_one_reset_then_reported(self):
        resets = []

        class Link(FakeLink):
            def __init__(self, port):
                self.port = port

            def reset_board(self):
                resets.append(self.port)

        r = self.run_pf(open_link=Link, read_hello=lambda link, timeout_s=3.0: None if link.port == "COM13" else "BusNode 2.8 (A) CAN 8MHz")
        self.assertEqual(resets, ["COM13"])
        self.assertTrue(any(c["name"] == "hello board_b" and c["status"] == "fail" and any("RESET" in f or "RTS" in f for f in c["fixes"]) for c in r["checks"]))

    def test_can_failed_hello_blocks(self):
        r = self.run_pf(hellos={"COM13": "SafetyNode 2.7 (B) CAN FAILED - PiL only", "COM14": "BusNode 2.8 (A) CAN 8MHz"})
        self.assertFalse(r["ok"])

    def test_uds_problem_blocks(self):
        r = self.run_pf(uds={"boards": {}, "problems": ["board_a: no UDS response to TesterPresent"]})
        self.assertFalse(r["ok"])

    def test_full_level_audits_compile_and_partition(self):
        r = self.run_pf("full")
        names = {c["name"]: c for c in r["checks"]}
        self.assertTrue(r["ok"], r["checks"])
        self.assertIn("26% of its 1280 KiB partition", names["partition board_b"]["detail"])
        self.assertIn("16 MB", names["upload probe board_a"]["detail"])

    def test_full_level_catches_partition_larger_than_the_chip_and_a_nearly_full_app(self):
        r = self.run_pf("full", upload_probe=lambda port: {"ok": True, "output": "", "flash_mb": 2})
        self.assertTrue(any(c["name"] == "partition board_b" and c["status"] == "fail" for c in r["checks"]))
        r = self.run_pf("full", compile_sketch=lambda role: {"ok": True, "output": "", "percent": 92, "max": 1310720, "used": 1, "partitions": None})
        self.assertTrue(any(c["name"] == "partition board_a" and c["status"] == "warn" for c in r["checks"]))

    def test_board_older_than_its_source_warns(self):
        r = self.run_pf("full", source_version=lambda role: "2.9")
        self.assertTrue(any(c["name"].startswith("source") and c["status"] == "warn" for c in r["checks"]))
        self.assertTrue(r["ok"])

    def test_compile_failure_blocks(self):
        r = self.run_pf("full", compile_sketch=lambda role: {"ok": False, "output": "error: x was not declared"})
        self.assertFalse(r["ok"])

    def test_port_override_beats_the_inventory(self):
        r = preflight.run_preflight("quick", INV, world(ports=("COM21", "COM14"), hellos={"COM21": "SafetyNode 2.7 (B) CAN 8MHz", "COM14": "BusNode 2.8 (A) CAN 8MHz"}),
                                    write=False, ports_override={"board_b": "COM21"})
        self.assertTrue(r["ok"], r["checks"])

    def test_catalogue_patterns_all_compile_and_have_fixes(self):
        for e in preflight.load_catalogue():
            self.assertTrue(e["fix"] and e["cause"], e["id"])
        ids = {e["id"] for e in preflight.explain("COM14 sent no hello and CAN FAILED")}
        self.assertEqual(ids, {"no-hello", "can-failed"})


class DashboardTests(unittest.TestCase):
    def campaign(self, root: Path, name: str, level: str, verdicts: dict, defects=(), preflight_ok=True):
        d = root / name
        (d / "02_design").mkdir(parents=True)
        (d / "04_triage").mkdir()
        (d / "03_execute").mkdir()
        cases = [{"id": cid, "req": "SR-01", "coverage_item": f"item {cid}"} for cid in verdicts]
        (d / "02_design" / "cases.json").write_text(json.dumps(cases))
        (d / "04_triage" / "case_results.json").write_text(json.dumps(verdicts))
        (d / "03_execute" / "run_manifest.json").write_text(json.dumps(
            {"firmware": {"board_b": "SafetyNode 2.7"}, "run": {"level": level, "defects": list(defects), "preflight": {"ok": preflight_ok}}}))

    def test_every_non_pass_lands_in_exactly_one_box(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.campaign(root, "c1", "hil", {
                "A": {"verdict": "PASS"}, "B": {"verdict": "FAIL", "failed": ["detect within FTTI"], "cause": "TIMEOUT"},
                "C": {"verdict": "SUSPECT", "why": "also fails on the known-good reference controller"},
                "D": {"verdict": "SUSPECT BENCH", "why": "the bench may be at fault, not the product: board_b: SafetyNode 2.5 is not the inventory's SafetyNode 2.7"},
                "E": {"verdict": "BLOCKED"}, "F": {"verdict": "INFO"}})
            d = dashboard.build(root, root / "health")
        boxes = {x["case"]: x["box"] for x in d["explained"]}
        self.assertEqual(boxes, {"B": "PRODUCT", "C": "TEST", "D": "BENCH", "E": "GAP"})
        self.assertEqual(d["coverage"]["SR-01"]["hil"]["counts"]["PASS"], 1)
        self.assertEqual(d["clusters"][0]["cases"], ["B"])

    def test_seeded_defect_is_labelled_and_latest_campaign_wins(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.campaign(root, "old", "reference", {"A": {"verdict": "FAIL", "failed": ["x"]}})
            self.campaign(root, "seeded", "hil", {"A": {"verdict": "FAIL", "failed": ["x"]}}, defects=["long_timeout"])
            d = dashboard.build(root, root / "health")
            self.assertIn("seeded defect (long_timeout)", next(x for x in d["explained"] if x["level"] == "hil")["detail"])
            page = dashboard.render_html(d)
            self.assertIn("Every non-PASS, in one box", page)
            self.assertIn("PRODUCT", page)

    def test_page_escapes_html_and_works_without_preflight(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.campaign(root, "c", "hil", {"A": {"verdict": "FAIL", "failed": ["<script>x</script>"]}})
            out = dashboard.write(root, root / "health")
            text = out.read_text(encoding="utf-8")
        self.assertNotIn("<script>x", text)
        self.assertIn("No preflight on record", text)


if __name__ == "__main__":
    unittest.main()
