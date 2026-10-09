"""v2.28: the board's own Wi-Fi radio. Provisioning over USB, the broker login, the firmware's contract with the PC, and secrets hygiene.

The radio itself needs hardware and your network; what can be held down in software is held down here: the message format both ends
agree on, that no password ever comes back from the board, that a wrong login is refused, and that no secret can end up in git.
"""
from __future__ import annotations

import re
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from fleet import broker, ota_matrix, provision  # noqa: E402
from fleet.emulated import EmuBoard  # noqa: E402
from fleet.provision import Provisioner, ProvisionError  # noqa: E402
from ssb.hil import Parser  # noqa: E402

try:
    import amqtt  # noqa: F401
    import paho.mqtt.client  # noqa: F401
    HAVE_MQTT = True
except ImportError:
    HAVE_MQTT = False

FW = ROOT / "hil" / "firmware" / "SafetyNode"
GOOD = dict(ssid="HomeNet-2G", wifi_password="correct-horse-battery", host="192.168.1.20", port=1884, user="ssb", mqtt_password="broker-pass-123", device="board-b")


class SetPayloadTests(unittest.TestCase):
    def test_a_valid_set_is_one_message(self):
        body = provision.set_payload(**GOOD)
        self.assertEqual(body[0], provision.SET)
        self.assertLessEqual(len(body), 255)

    def test_what_the_board_would_refuse_is_refused_first_with_a_reason(self):
        cases = [
            (dict(ssid="x" * 33), "ssid"), (dict(wifi_password="p" * 64), "wifi_password"), (dict(device="d" * 17), "device"),
            (dict(ssid=""), "required"), (dict(host=""), "required"), (dict(wifi_password="short"), "8 to 63"),
            (dict(user="a\0b"), "NUL"), (dict(port=0), "port"), (dict(port=70000), "port"),
        ]
        for over, needle in cases:
            with self.subTest(over=over), self.assertRaisesRegex(ProvisionError, needle):
                provision.set_payload(**{**GOOD, **over})

    def test_an_open_network_needs_no_password(self):
        provision.set_payload(**{**GOOD, "wifi_password": ""})

    def test_no_broker_login_is_allowed_for_an_open_local_broker(self):
        provision.set_payload(**{**GOOD, "user": "", "mqtt_password": ""})


class FirmwareContractTests(unittest.TestCase):
    """ssc_net.h is the truth: the PC's limits and op numbers must be its own, name for name."""

    @classmethod
    def setUpClass(cls):
        cls.h = (FW / "ssc_net.h").read_text(encoding="utf-8")
        cls.ino = (FW / "SafetyNode.ino").read_text(encoding="utf-8")

    def test_buffer_sizes_match_the_limits(self):
        m = re.search(r"static char ssid\[(\d+)\], wpass\[(\d+)\], host\[(\d+)\], user\[(\d+)\], mpass\[(\d+)\], dev\[(\d+)\];", self.h)
        self.assertIsNotNone(m, "the credential buffers in ssc_net.h changed shape")
        sizes = [int(x) - 1 for x in m.groups()]   # a buffer holds one character less than its size (the NUL)
        self.assertEqual(sizes, list(provision.LIMITS.values()))

    def test_ops_and_statuses_match(self):
        self.assertIn("enum Op : uint8_t { SET = 1, CLEAR = 2, INFO = 3 };", self.h)
        self.assertEqual((provision.SET, provision.CLEAR, provision.INFO), (1, 2, 3))
        self.assertIn("enum Status : uint8_t { OK = 0, BAD_MESSAGE = 1, TOO_LONG = 2, NO_STORE = 3 };", self.h)
        self.assertEqual(provision.STATUS, {0: "OK", 1: "BAD_MESSAGE", 2: "TOO_LONG", 3: "NO_STORE"})

    def test_provisioning_is_accepted_over_usb_only(self):
        """Credentials are never accepted over the network they configure: 'N' is handled in the serial branch, the MQTT branch takes 'U' only."""
        self.assertIn("else if (ota_parser.type == 'N') net::handle(", self.ino)
        self.assertIn("net_parser.type != 'U'", self.ino)
        self.assertEqual(self.ino.count("net::handle("), 1)

    def test_the_radio_is_off_while_a_scenario_runs(self):
        self.assertIn("net::poll(!(node.active() && pc_session), fw);", self.ino)

    def test_nothing_is_provisioned_means_the_radio_never_starts(self):
        self.assertRegex(self.h, r"static void poll\(bool allowed, const char \*hello\) \{\s*if \(!provisioned\) return;")

    def test_no_password_leaves_the_board(self):
        info = re.search(r"case INFO: \{(.*?)\n    \}", self.h, re.S)
        self.assertIsNotNone(info)
        self.assertNotRegex(info.group(1), r"wpass|mpass")

    def test_no_credential_is_compiled_into_the_firmware(self):
        for f in FW.glob("*"):
            if f.suffix in (".h", ".ino", ".cpp"):
                text = f.read_text(encoding="utf-8", errors="ignore")
                self.assertNotRegex(text, r'WiFi\.begin\(\s*"', f"{f.name}: a literal network name or password")


class ProvisionerAgainstTheEmulatedBoardTests(unittest.TestCase):
    def test_set_then_info_then_clear(self):
        emu = EmuBoard("e", "2.10")
        p = Provisioner(emu, timeout_s=1.0)
        self.assertFalse(p.info().provisioned)
        p.set(**GOOD)
        info = p.info()
        self.assertTrue(info.provisioned)
        self.assertEqual((info.ssid, info.host, info.device), ("HomeNet-2G", "192.168.1.20", "board-b"))
        self.assertIn("HomeNet-2G", provision.describe(info))
        p.clear()
        self.assertFalse(p.info().provisioned)

    def test_no_password_appears_in_anything_the_board_sends(self):
        emu = EmuBoard("e", "2.10")
        p = Provisioner(emu, timeout_s=1.0)
        seen = bytearray()
        orig = emu.read

        def spy() -> bytes:
            b = orig()
            seen.extend(b)
            return b
        emu.read = spy   # type: ignore[method-assign]
        p.set(**GOOD)
        p.info()
        self.assertNotIn(GOOD["wifi_password"].encode(), bytes(seen))
        self.assertNotIn(GOOD["mqtt_password"].encode(), bytes(seen))
        self.assertNotIn(GOOD["wifi_password"], provision.describe(p.info()))

    def test_the_board_refuses_a_malformed_set(self):
        emu = EmuBoard("e", "2.10")
        emu.write(__import__("ssb.hil", fromlist=["frame_msg"]).frame_msg("N", bytes([provision.SET, 0x5B, 0x07]) + b"only-one-string\0"), 0)
        msgs = list(Parser().feed(emu.read()))
        self.assertTrue(any(k == "n" and p[1] != 0 for k, p in msgs), msgs)
        self.assertEqual(emu.net, {})

    def test_a_silent_board_gives_a_helpful_error(self):
        class Silent:
            def write(self, d, n=0): pass
            def read(self): return b""
        with self.assertRaisesRegex(ProvisionError, "SafetyNode 2.28"):
            Provisioner(Silent(), timeout_s=0.2).info()


class BrokerLoginTests(unittest.TestCase):
    def test_only_a_hash_is_stored(self):
        with tempfile.TemporaryDirectory() as d:
            f = Path(d) / "passwd"
            broker.write_login(f, "ssb", "a-long-broker-password")
            text = f.read_text(encoding="utf-8")
            self.assertTrue(text.startswith("ssb:$argon2"), text[:20])
            self.assertNotIn("a-long-broker-password", text)

    def test_weak_or_malformed_logins_are_refused(self):
        with tempfile.TemporaryDirectory() as d:
            for user, pw in (("", "long-enough-pw"), ("a b", "long-enough-pw"), ("a:b", "long-enough-pw"), ("ssb", "short")):
                with self.subTest(user=user), self.assertRaises(ValueError):
                    broker.write_login(Path(d) / "p", user, pw)

    @unittest.skipUnless(HAVE_MQTT, "pip install -e .[fleet]")
    def test_the_broker_refuses_a_wrong_or_missing_login(self):
        from fleet.mqtt_link import BrokerRefused, LocalBroker, MqttLink
        with tempfile.TemporaryDirectory() as d:
            f = Path(d) / "passwd"
            broker.write_login(f, "ssb", "the-right-password")
            with LocalBroker(password_file=str(f)) as b:
                MqttLink("t", b.port, user="ssb", password="the-right-password").close()
                with self.assertRaises(BrokerRefused):
                    MqttLink("t", b.port, user="ssb", password="the-wrong-password")
                with self.assertRaises(BrokerRefused):
                    MqttLink("t", b.port)

    @unittest.skipUnless(HAVE_MQTT, "pip install -e .[fleet]")
    def test_the_update_runs_against_a_board_on_the_network_behind_a_login(self):
        """The --wifi code path with a stand-in board: login-protected broker, hello on connect, the EN-pin reset on a side channel."""
        from fleet.mqtt_link import LocalBroker
        with tempfile.TemporaryDirectory() as d:
            f = Path(d) / "passwd"
            broker.write_login(f, "ssb", "the-right-password")
            with LocalBroker(password_file=str(f)) as b:
                dev, cleanup = ota_matrix.over_mqtt(ota_matrix.emulated(), b.port, wifi=True, user="ssb", password="the-right-password", device="board-b")
                try:
                    self.assertEqual(dev.kind, "emulated+wifi")
                    rows = ota_matrix.run_all(dev, ota_matrix.images_emulated())
                finally:
                    cleanup()
            bad = [r["id"] for r in rows if not r["ok"]]
            self.assertEqual(bad, [], rows)
            self.assertTrue(time.time() > 0)


class SecretsHygieneTests(unittest.TestCase):
    def test_the_secret_paths_are_ignored(self):
        ignore = (ROOT / ".gitignore").read_text(encoding="utf-8").split()
        for needle in ("secrets/", "broker_passwd", "*.passwd", "wifi_secrets*.h", ".env.wifi"):
            self.assertIn(needle, ignore)

    def test_no_tracked_text_file_assigns_a_literal_wifi_or_broker_password(self):
        tracked = subprocess.run(["git", "ls-files"], cwd=ROOT, capture_output=True, text=True, check=False).stdout.splitlines()
        if not tracked:
            self.skipTest("not a git checkout")
        pat = re.compile(r"""SSB_(?:WIFI|MQTT)_PASS["']?\s*[:=]\s*["']?[A-Za-z0-9]{6,}""")
        hits = []
        for rel in tracked:
            p = ROOT / rel
            if p.suffix in (".md", ".py", ".h", ".ino", ".json", ".yml", ".toml", ".txt") and p.is_file() and rel != "tests/test_wifi.py":
                if pat.search(p.read_text(encoding="utf-8", errors="ignore")):
                    hits.append(rel)
        self.assertEqual(hits, [], "a literal password assigned to SSB_*_PASS")

    def test_the_script_never_prints_or_stores_what_it_is_given(self):
        text = (ROOT / "scripts" / "provision_wifi.py").read_text(encoding="utf-8")
        self.assertNotRegex(text, r"print\([^)]*(wpass|mpass)")
        self.assertNotIn("open(", text)
        self.assertNotIn("write_text", text)


if __name__ == "__main__":
    unittest.main()
