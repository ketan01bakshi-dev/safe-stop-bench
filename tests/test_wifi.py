"""v2.28: the board's own Wi-Fi radio. Provisioning over USB, the broker login, the firmware's contract with the PC, and secrets hygiene.

The radio itself needs hardware and your network; what can be held down in software is held down here: the message format both ends
agree on, that no password ever comes back from the board, that a wrong login is refused, and that no secret can end up in git.
"""
from __future__ import annotations

import re
import struct
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
from ssb import firmware_build  # noqa: E402
from ssb.hil import Parser, frame_msg  # noqa: E402

try:
    import amqtt  # noqa: F401
    import paho.mqtt.client  # noqa: F401
    HAVE_MQTT = True
except ImportError:
    HAVE_MQTT = False

FW = ROOT / "hil" / "firmware" / "SafetyNode"
GOOD = dict(ssid="HomeNet-2G", wifi_password="correct-horse-battery", host="192.168.1.20", port=1884, user="ssb", mqtt_password="broker-pass-123", device="board-b")


class InfoFlagsTests(unittest.TestCase):
    """INFO's flags byte as the PC decodes it (bit 5, v2.34: the firmware refused to run MQTT because its build lacks the core-0 pin)."""

    class Fixed:
        def __init__(self, flags: int):
            self.flags, self.out = flags, b""

        def write(self, data: bytes, now: int) -> None:
            extra = bytes([self.flags, 0]) + struct.pack("<IIH", 200_000, 150_000, 0) + b"net\0\0broker\0dev"
            self.out = frame_msg("n", bytes([provision.INFO, 0]) + extra)

        def read(self) -> bytes:
            out, self.out = self.out, b""
            return out

    def test_the_refusal_bit_is_decoded_and_described(self):
        ok = Provisioner(self.Fixed(0b00000111)).info()
        self.assertFalse(ok.mqtt_refused)
        self.assertNotIn("REFUSED", provision.describe(ok))
        bad = Provisioner(self.Fixed(0b00100001)).info()
        self.assertTrue(bad.mqtt_refused)
        self.assertIn("MQTT REFUSED", provision.describe(bad))


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
        self.assertIn("enum Op : uint8_t { SET = 1, CLEAR = 2, INFO = 3, CA_BEGIN = 4, CA_CHUNK = 5, CA_END = 6 };", self.h)
        self.assertEqual((provision.SET, provision.CLEAR, provision.INFO, provision.CA_BEGIN, provision.CA_CHUNK, provision.CA_END), (1, 2, 3, 4, 5, 6))
        self.assertIn("enum Status : uint8_t { OK = 0, BAD_MESSAGE = 1, TOO_LONG = 2, NO_STORE = 3, BAD_CA = 4 };", self.h)
        self.assertEqual(provision.STATUS, {0: "OK", 1: "BAD_MESSAGE", 2: "TOO_LONG", 3: "NO_STORE", 4: "BAD_CA"})
        self.assertIn(f"static const size_t CA_MAX = {provision.CA_MAX};", self.h)

    def test_tls_is_pinned_to_the_uploaded_ca_and_never_falls_back_to_plain(self):
        self.assertIn("MQTT_TRANSPORT_OVER_SSL", self.h)
        self.assertIn("cfg.broker.verification.certificate = ca_pem;", self.h)
        self.assertIn("bool usable = provisioned && !(tls && ca_len == 0);", self.h)
        self.assertNotIn("skip_cert_common_name_check", self.h, "the address check stays on")
        self.assertNotIn("use_global_ca_store", self.h, "no public CA store: only the uploaded CA is trusted")

    def test_clear_forgets_the_tls_state_in_memory_too(self):
        clear = self.h[self.h.index("case CLEAR:"):self.h.index("case CA_BEGIN:")]
        for needle in ("tls = false;", "ca_len = ca_want = ca_have = 0;", "memset(ca_pem"):
            self.assertIn(needle, clear)

    def test_provisioning_is_accepted_over_usb_only(self):
        """Credentials are never accepted over the network they configure: 'N' is handled in the serial branch, the MQTT branch takes 'U' only."""
        self.assertIn("else if (ota_parser.type == 'N') net::handle(", self.ino)
        self.assertIn("net_parser.type != 'U'", self.ino)
        self.assertEqual(self.ino.count("net::handle("), 1)

    def test_the_mqtt_task_does_not_outrank_the_control_loop(self):
        """v2.33: at the esp-mqtt default priority (5) every TLS handshake stalled the 10 ms loop (priority 1) for ~190 ms. Keep it at the loop's priority."""
        m = re.search(r"#define NET_MQTT_PRIO (\d+)", self.h)
        self.assertIsNotNone(m)
        self.assertEqual(int(m.group(1)), 1)
        self.assertIn("cfg.task.priority = NET_MQTT_PRIO;", self.h)

    def test_the_diagnostics_are_compiled_out_of_a_normal_build(self):
        """ssc_diag.h is empty unless -DB_DIAG=1; the test hooks of the 'N' protocol exist only in that build."""
        diag = (FW / "ssc_diag.h").read_text(encoding="utf-8")
        self.assertIn("#ifdef B_DIAG", diag)
        self.assertRegex(diag, r"#else\s+#define BD_SEC\(i\)\s+#define BD_LOOP_START\(\)\s+#define BD_REPORT\(\)\s+#define BD_EVENT\(code, arg\)\s+#endif")
        self.assertRegex(self.h, r"#ifdef B_DIAG\s+case 7:")
        self.assertNotIn("case 7:", self.h.replace(self.h[self.h.index("#ifdef B_DIAG\n    case 7:"):self.h.index("#endif\n    default:")], ""))

    def test_the_ota_reply_is_sent_at_once_and_at_qos_0(self):
        """Measured on the real board (v2.31): enqueue (sent when the MQTT task next wakes) cost ~1 s per chunk, and a QoS 1 reply made the
        broker's PUBACK hold the next chunk back for the board's delayed ACK (250 ms). Replies go out at once, at QoS 0."""
        reply = self.h[self.h.index("static void publish("):self.h.index("static bool take_str(")]
        self.assertIn("esp_mqtt_client_publish(cli, t_from, (const char *)d, (int)n, 0, 0)", reply)
        self.assertNotIn("esp_mqtt_client_enqueue", reply)

    def test_the_wifi_link_to_the_real_board_uses_qos_0(self):
        text = (ROOT / "fleet" / "ota_matrix.py").read_text(encoding="utf-8")
        self.assertIn("link.to_board_qos = 0", text)

    def test_the_radio_is_off_while_a_scenario_runs(self):
        self.assertIn("net::poll(!(node.active() && pc_session), fw);", self.ino)

    def test_nothing_is_provisioned_means_the_radio_never_starts(self):
        """Nothing provisioned (or TLS with no CA) or a scenario running: poll() wants no radio. The radio task is made only when one is wanted."""
        poll = self.h[self.h.index("static void poll("):self.h.index("static bool pop(")]
        self.assertIn("bool usable = provisioned && !(tls && ca_len == 0);", poll)
        self.assertRegex(poll, r"if \(!usable \|\| !allowed\) \{\s*allowed_since = 0;\s*set_want\(false\);\s*return;\s*\}")
        self.assertEqual(self.h.count("xTaskCreatePinnedToCore("), 2, "the radio task, and the throw-away probe of mqtt_pin_state()")
        self.assertIn("if (on && !radio_task) xTaskCreatePinnedToCore(", self.h)

    def test_the_slow_radio_calls_are_made_by_a_core_0_task_never_by_the_control_loop(self):
        """v2.34: WiFi.begin took ~50 ms and stop_radio 80-130 ms inside loop() (measured, v2.33). poll() only sets a flag; a task on core 0,
        below the Wi-Fi and MQTT tasks, makes the calls. Anything that stops the radio does it with the lock the task also takes."""
        poll = self.h[self.h.index("static void poll("):self.h.index("static bool pop(")]
        for slow in ("WiFi.", "stop_radio", "start_mqtt", "esp_mqtt"):
            self.assertNotIn(slow, poll, f"poll() runs in the 10 ms loop and must not call {slow}")
        self.assertIn('xTaskCreatePinnedToCore(radio_main, "radio", 8192, nullptr, 1, &radio_task, 0)', self.h)
        self.assertEqual(self.h.count("WiFi.begin(ssid, wpass)"), 1)
        step = self.h[self.h.index("static void radio_step()"):self.h.index("static void radio_main(")]
        for needle in ("WiFi.begin(ssid, wpass)", "start_mqtt()", "stop_radio()"):
            self.assertIn(needle, step)
        main = self.h[self.h.index("static void radio_main("):self.h.index("static void set_want(")]
        self.assertRegex(main, r"Lock lk;\s*radio_step\(\);")
        handle = self.h[self.h.index("static void handle("):]
        self.assertNotIn("stop_radio()", handle, "handle() halts the radio through halt_locked(), under the lock")
        for case, nxt in (("case SET:", "case CLEAR:"), ("case CLEAR:", "case CA_BEGIN:"), ("case CA_END:", "case INFO:")):
            block = handle[handle.index(case):handle.index(nxt)]
            self.assertRegex(block, r"Lock lk;\s*halt_locked\(\);", f"{case} changes what the radio uses: it must hold radio_mx")

    def test_the_mqtt_task_is_pinned_to_core_0_so_a_handshake_cannot_starve_the_control_loop(self):
        """v2.34, measured on the board: unpinned, the esp-mqtt task ran on core 1 during a TLS handshake, boosted to priority 23 by mutex inheritance, and the
        loop (ready, priority 1) did not run for 130-190 ms. ssc_esp_pin.cpp routes the task's creation to core 0 via the linker's --wrap."""
        lib = ROOT / "hil" / "SafeStopCore"
        pin = (lib / "src" / "ssc_esp_pin.cpp").read_text(encoding="utf-8")
        self.assertIn("#ifdef ARDUINO_ARCH_ESP32", pin)
        self.assertIn("__wrap_xTaskCreatePinnedToCore", pin)
        self.assertIn('strcmp(name, "mqtt_task") == 0', pin)
        self.assertIn("mqtt ? 0 : core", pin)
        self.assertEqual(firmware_build.LINK_FLAGS, "-Wl,--wrap=xTaskCreatePinnedToCore")
        self.assertNotIn("ssc_esp_pin", (ROOT / "scripts" / "build_native.py").read_text(encoding="utf-8"), "the PC build lists its sources; this one is ESP32 only")

    def test_every_way_of_building_the_firmware_passes_the_link_flag(self):
        for rel in ("scripts/hil_flash.py", "fleet/image.py", "health/preflight.py"):
            text = (ROOT / rel).read_text(encoding="utf-8")
            self.assertIn("build_properties(", text, f"{rel} must compile with ssb.firmware_build.build_properties")
        self.assertEqual(firmware_build.build_properties(), ["--build-property", f"compiler.c.elf.extra_flags={firmware_build.LINK_FLAGS}"])
        self.assertEqual(firmware_build.build_properties("-DX=1")[2:], ["--build-property", "compiler.cpp.extra_flags=-DX=1"])

    def test_a_build_without_the_link_flag_refuses_to_run_the_mqtt_client_and_says_so(self):
        probe = self.h[self.h.index("static int mqtt_pin_state()"):self.h.index("// One pass of the radio task")]
        self.assertIn('"mqtt_task", 2048, nullptr, 1, &t, tskNO_AFFINITY', probe)
        self.assertIn("xTaskGetAffinity(t) == 0", probe)
        step = self.h[self.h.index("static void radio_step()"):self.h.index("static void radio_main(")]
        self.assertIn("if (!want_radio || mqtt_refused) {", step)
        self.assertRegex(step, r"int pin = mqtt_pin_state\(\);\s*if \(pin == 0\) mqtt_refused = true;\s*if \(pin != 1\) return;\s*if \(!radio_on\) \{")
        # the refusal comes BEFORE the radio is switched on, and tears no client down: stopping a client whose task has not run yet crashes esp-mqtt (found on the board)
        self.assertLess(step.index("mqtt_pin_state()"), step.index("WiFi.begin("))
        start = self.h[self.h.index("static void start_mqtt()"):self.h.index("// v2.34: the control loop is safe")]
        self.assertNotIn("esp_mqtt_client_stop", start)
        self.assertNotIn("esp_mqtt_client_destroy", start)
        stop = self.h[self.h.index("static void stop_radio()"):self.h.index("static void on_event(")]
        self.assertRegex(stop, r"uint32_t age = millis\(\) - cli_started_ms;\s*if \(age < 50\) vTaskDelay\(pdMS_TO_TICKS\(50 - age\)\);\s*esp_mqtt_client_stop\(cli\);")
        info = re.search(r"case INFO: \{(.*?)\n    \}", self.h, re.S).group(1)
        self.assertIn("(mqtt_refused ? 32 : 0)", info)

    def test_a_reply_is_not_sent_while_the_radio_is_not_wanted_or_being_switched(self):
        reply = self.h[self.h.index("static void publish("):self.h.index("static bool take_str(")]
        self.assertIn("!want_radio", reply)
        self.assertIn("xSemaphoreTake(radio_mx, pdMS_TO_TICKS(5))", reply)
        self.assertIn("xSemaphoreGive(radio_mx);", reply)

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

    def test_the_ca_certificate_is_uploaded_in_chunks_and_reported_as_stored(self):
        from fleet import certs
        ca_pem = certs.pem(certs.make_ca()[0])
        emu = EmuBoard("e", "2.10")
        p = Provisioner(emu, timeout_s=1.0)
        p.set_ca(ca_pem)
        p.set(**GOOD, tls=True)
        info = p.info()
        self.assertTrue(info.tls and info.ca_stored)
        self.assertEqual(info.ca_len, len(ca_pem))
        self.assertEqual(emu.net["ca"], ca_pem, "stored byte for byte")
        self.assertIn("TLS, CA pinned", provision.describe(info))
        self.assertGreaterEqual(len(ca_pem) // provision.CA_CHUNK_BYTES, 5, "and it really took several chunks")

    def test_tls_without_a_ca_is_reported_loudly(self):
        emu = EmuBoard("e", "2.10")
        p = Provisioner(emu, timeout_s=1.0)
        p.set(**GOOD, tls=True)
        self.assertIn("NO CA STORED", provision.describe(p.info()))

    def test_a_private_key_or_a_non_certificate_is_never_sent(self):
        from fleet import certs
        key = certs.key_pem(certs.make_ca()[1])
        emu = EmuBoard("e", "2.10")
        tiny = b"-----BEGIN CERTIFICATE-----\nx\n-----END CERTIFICATE-----\n"
        for bad, needle in ((key, "PRIVATE KEY"), (b"hello", "not a PEM"), (tiny, "bytes")):
            with self.subTest(needle=needle), self.assertRaisesRegex(ProvisionError, needle):
                Provisioner(emu, timeout_s=1.0).set_ca(bad)
        self.assertEqual(emu.net, {}, "nothing reached the board")

    def test_an_interrupted_upload_does_not_replace_a_stored_ca(self):
        from fleet import certs
        from ssb.hil import frame_msg
        emu = EmuBoard("e", "2.10")
        p = Provisioner(emu, timeout_s=1.0)
        good = certs.pem(certs.make_ca()[0])
        p.set_ca(good)
        emu.write(frame_msg("N", bytes([provision.CA_BEGIN]) + (len(good)).to_bytes(2, "little")), 0)   # a second upload starts and never finishes
        emu.write(frame_msg("N", bytes([provision.CA_END])), 0)
        self.assertEqual(emu.net["ca"], good)

    def test_clear_forgets_the_ca_too(self):
        from fleet import certs
        emu = EmuBoard("e", "2.10")
        p = Provisioner(emu, timeout_s=1.0)
        p.set_ca(certs.pem(certs.make_ca()[0]))
        p.set(**GOOD, tls=True)
        p.clear()
        self.assertFalse(p.info().ca_stored)

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


    @unittest.skipUnless(HAVE_MQTT, "pip install -e .[fleet]")
    def test_the_update_runs_through_an_existing_broker_given_by_host_and_port(self):
        """--broker HOST:PORT: no broker is started by the matrix; an anonymous broker on some host:port is just used."""
        from fleet.mqtt_link import LocalBroker
        with LocalBroker() as existing:   # stands in for the Mosquitto service
            dev, cleanup = ota_matrix.over_mqtt(ota_matrix.emulated(), existing.port, wifi=True, device="board-b", host="127.0.0.1")
            try:
                rows = ota_matrix.run_all(dev, ota_matrix.images_emulated())
            finally:
                cleanup()
        self.assertEqual([r["id"] for r in rows if not r["ok"]], [])

    def test_the_matrix_cli_has_the_broker_options(self):
        text = (ROOT / "fleet" / "ota_matrix.py").read_text(encoding="utf-8")
        for needle in ("--broker", "--anonymous", "rpartition(\":\")"):
            self.assertIn(needle, text)


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
