"""v2.30: TLS for the boards. The certificates, the Mosquitto configuration, and a PRIVATE Mosquitto instance that proves the configuration works.

The Mosquitto tests start the real mosquitto.exe (when installed) on free ports in a temporary folder with exactly the configuration the setup
script writes. They never touch the Mosquitto service or its files.
"""
from __future__ import annotations

import datetime as dt
import ipaddress
import shutil
import socket
import ssl
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

try:
    from cryptography import x509  # noqa: E402

    from fleet import certs, mosquitto_conf  # noqa: E402
except ImportError as e:   # pip install -e .[fleet]
    raise unittest.SkipTest(f"TLS tests need the fleet extra: {e}") from e

try:
    import amqtt  # noqa: F401
    import paho.mqtt.client as mqtt  # noqa: F401
    HAVE_MQTT = True
except ImportError:
    HAVE_MQTT = False

MOSQUITTO = next((p for p in (Path(r"C:\Program Files\Mosquitto\mosquitto.exe"), shutil.which("mosquitto")) if p and Path(p).exists()), None)
MOSQUITTO_PASSWD = next((p for p in (Path(r"C:\Program Files\Mosquitto\mosquitto_passwd.exe"), shutil.which("mosquitto_passwd")) if p and Path(p).exists()), None)


def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


class CertificateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ca, cls.ca_key = certs.make_ca()
        cls.srv, cls.srv_key = certs.make_server(cls.ca, cls.ca_key, ["192.168.1.6"])

    def test_the_certificates_are_valid_from_1970_because_the_board_has_no_clock(self):
        for c in (self.ca, self.srv):
            self.assertEqual(c.not_valid_before_utc.year, 1970)
            self.assertEqual(c.not_valid_after_utc.year, 2049)
            self.assertGreater(c.not_valid_after_utc, dt.datetime.now(dt.timezone.utc), "and still valid today")

    def test_the_address_is_named_as_an_ip_entry_and_as_a_name_entry(self):
        san = self.srv.extensions.get_extension_for_class(x509.SubjectAlternativeName).value
        self.assertIn(ipaddress.ip_address("192.168.1.6"), san.get_values_for_type(x509.IPAddress))
        self.assertIn("192.168.1.6", san.get_values_for_type(x509.DNSName), "mbedTLS compares the text it was given with name entries")
        self.assertIn("localhost", san.get_values_for_type(x509.DNSName))

    def test_the_server_certificate_is_signed_by_the_ca_and_cannot_sign_others(self):
        self.srv.verify_directly_issued_by(self.ca)
        self.assertFalse(self.srv.extensions.get_extension_for_class(x509.BasicConstraints).value.ca)
        self.assertTrue(self.ca.extensions.get_extension_for_class(x509.BasicConstraints).value.ca)

    def test_the_certificate_is_for_servers_only(self):
        eku = self.srv.extensions.get_extension_for_class(x509.ExtendedKeyUsage).value
        self.assertEqual([o.dotted_string for o in eku], ["1.3.6.1.5.5.7.3.1"])

    def test_the_ca_pem_fits_the_boards_upload_buffer(self):
        self.assertLess(len(certs.pem(self.ca)), 2000)

    def test_fingerprint_is_stable_and_colon_separated(self):
        fp = certs.fingerprint(certs.pem(self.ca))
        self.assertEqual(len(fp.split(":")), 32)
        self.assertEqual(fp, certs.fingerprint(certs.pem(self.ca)))

    def test_existing_certificates_are_not_replaced_without_force(self):
        with tempfile.TemporaryDirectory() as d:
            certs.write_all(Path(d), ["10.0.0.5"])
            with self.assertRaises(FileExistsError):
                certs.write_all(Path(d), ["10.0.0.5"])
            certs.write_all(Path(d), ["10.0.0.5"], force=True)

    def test_at_least_one_host_is_required(self):
        with self.assertRaises(ValueError):
            certs.make_server(self.ca, self.ca_key, [])


class MosquittoConfigTests(unittest.TestCase):
    EXISTING = "# my broker\nlisten_dummy_comment\nlistener 1883\nallow_anonymous true\n".replace("listen_dummy_comment\n", "")

    def test_the_existing_lines_are_kept_and_per_listener_settings_comes_first(self):
        out = mosquitto_conf.merge(self.EXISTING, r"C:\Program Files\Mosquitto\ssb")
        self.assertTrue(out.startswith("per_listener_settings true"))
        self.assertIn("listener 1883\nallow_anonymous true\n", out)
        self.assertIn("listener 8883", out)
        self.assertLess(out.index("per_listener_settings"), out.index("listener 1883"))

    def test_running_it_twice_does_not_stack_blocks(self):
        once = mosquitto_conf.merge(self.EXISTING, "D:/x")
        twice = mosquitto_conf.merge(once, "D:/x")
        self.assertEqual(twice.count(mosquitto_conf.BEGIN), 1)
        self.assertEqual(twice.count("per_listener_settings"), 1)
        self.assertEqual(once, twice)

    def test_the_boards_listener_is_tls_1_2_with_a_login_and_an_acl(self):
        b = mosquitto_conf.block(r"C:\Program Files\Mosquitto\ssb")
        for line in ("listener_allow_anonymous false", "tls_version tlsv1.2", "password_file", "acl_file", "cafile", "certfile", "keyfile"):
            self.assertIn(line, b)
        self.assertIn(r"C:\Program Files\Mosquitto\ssb\server.key", b)

    def test_a_config_that_says_per_listener_settings_false_is_not_changed_silently(self):
        with self.assertRaises(ValueError):
            mosquitto_conf.merge("per_listener_settings false\nlistener 1883\n", "D:/x")

    def test_the_acl_gives_the_login_only_the_bench_topics(self):
        self.assertEqual(mosquitto_conf.acl("ssb"), "user ssb\ntopic readwrite ssb/#\n")


@unittest.skipUnless(MOSQUITTO and MOSQUITTO_PASSWD and HAVE_MQTT, "needs mosquitto.exe and pip install -e .[fleet]")
class PrivateMosquittoTests(unittest.TestCase):
    """A real Mosquitto on free ports, configured by fleet.mosquitto_conf, in a temp folder."""

    PASSWORD = "unit-test-password-1"

    @classmethod
    def setUpClass(cls):
        cls.tmp = Path(tempfile.mkdtemp(prefix="ssb_mosq_"))
        cls.tls_port, cls.plain_port = free_port(), free_port()
        d = cls.tmp / "ssb"
        d.mkdir()
        cls.paths = certs.write_all(d, ["127.0.0.1"])
        subprocess.run([str(MOSQUITTO_PASSWD), "-b", "-c", str(d / "passwd"), "ssb", cls.PASSWORD], check=True, capture_output=True)
        (d / "acl").write_text(mosquitto_conf.acl("ssb"), encoding="utf-8")
        existing = f"listener {cls.plain_port} 127.0.0.1\nallow_anonymous true\n"   # stands in for the user's own anonymous listener
        conf = mosquitto_conf.merge(existing, str(d), cls.tls_port)
        (cls.tmp / "mosquitto.conf").write_text(conf, encoding="utf-8")
        cls.proc = subprocess.Popen([str(MOSQUITTO), "-c", str(cls.tmp / "mosquitto.conf")], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        deadline = time.time() + 10
        while time.time() < deadline:
            try:
                socket.create_connection(("127.0.0.1", cls.tls_port), 0.3).close()
                break
            except OSError:
                time.sleep(0.2)
        else:
            cls.proc.kill()
            raise RuntimeError("the private Mosquitto did not start:\n" + (cls.proc.stdout.read().decode(errors="replace") if cls.proc.stdout else ""))

    @classmethod
    def tearDownClass(cls):
        cls.proc.kill()
        cls.proc.wait(5)
        if cls.proc.stdout:
            cls.proc.stdout.close()
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def client(self, port, user="ssb", password=None, ca="default", tls=True):
        from fleet.mqtt_link import _client
        return _client(port, "127.0.0.1", user, self.PASSWORD if password is None else password,
                       str(self.paths["ca.pem"]) if (ca == "default" and tls) else (ca if tls else None))

    def test_the_login_works_over_tls(self):
        c = self.client(self.tls_port)
        got = threading.Event()
        c.on_message = lambda *a: got.set()
        c.subscribe("ssb/ota/x/echo", qos=1)
        time.sleep(0.3)
        c.publish("ssb/ota/x/echo", b"hello", qos=1)
        self.assertTrue(got.wait(5), "a message published under ssb/ must come back")
        c.loop_stop()

    def test_a_wrong_password_is_refused(self):
        from fleet.mqtt_link import BrokerRefused
        with self.assertRaises(BrokerRefused):
            self.client(self.tls_port, password="not-the-password")

    def test_no_login_at_all_is_refused_on_the_tls_listener(self):
        from fleet.mqtt_link import BrokerRefused
        with self.assertRaises(BrokerRefused):
            self.client(self.tls_port, user=None)   # type: ignore[arg-type]

    def test_a_broker_certificate_from_another_ca_is_not_trusted(self):
        from fleet.mqtt_link import BrokerRefused
        other = self.tmp / "other"
        other.mkdir(exist_ok=True)
        paths = certs.write_all(other, ["127.0.0.1"], force=True)
        with self.assertRaises(BrokerRefused) as cm:
            self.client(self.tls_port, ca=str(paths["ca.pem"]))
        self.assertIn("certificate", str(cm.exception).lower())

    def test_plain_mqtt_to_the_tls_port_fails(self):
        from fleet.mqtt_link import BrokerRefused
        with self.assertRaises(BrokerRefused):
            self.client(self.tls_port, tls=False)

    def test_the_login_cannot_read_or_write_outside_the_bench_topics(self):
        """Mosquitto may accept a SUBSCRIBE it will never deliver on, so test what matters: what actually arrives, in both directions."""
        c = self.client(self.tls_port)
        anon = self.client(self.plain_port, user=None, tls=False)   # type: ignore[arg-type]   # the user's own anonymous listener
        heard_by_login: list[str] = []
        heard_by_anon: list[str] = []
        c.on_message = lambda _c, _u, m: heard_by_login.append(m.topic)
        anon.on_message = lambda _c, _u, m: heard_by_anon.append(m.topic)
        c.subscribe([("other/projects/#", 1), ("ssb/#", 1)])
        anon.subscribe([("other/#", 1), ("ssb/#", 1)])
        time.sleep(0.5)
        anon.publish("other/projects/x", b"1", qos=1)   # an unrelated project's message: the bench login must not see it
        c.publish("other/projects/y", b"2", qos=1)      # the bench login writing outside ssb/#: must be dropped
        anon.publish("ssb/ota/probe", b"3", qos=1)      # inside the bench topics: both directions work
        c.publish("ssb/ota/probe2", b"4", qos=1)
        time.sleep(1.5)
        self.assertEqual(sorted(heard_by_login), ["ssb/ota/probe", "ssb/ota/probe2"], heard_by_login)
        self.assertNotIn("other/projects/y", heard_by_anon, "the bench login's write outside ssb/# must be dropped")
        self.assertEqual(sorted(t for t in heard_by_anon if t.startswith("ssb/")), ["ssb/ota/probe", "ssb/ota/probe2"], heard_by_anon)
        c.loop_stop()
        anon.loop_stop()

    def test_the_existing_anonymous_listener_is_unchanged(self):
        c = self.client(self.plain_port, user=None, tls=False)   # type: ignore[arg-type]
        c.loop_stop()

    def test_the_whole_update_sequence_runs_over_tls_with_the_login_and_the_acl(self):
        """REL-01..10 with the stand-in board: TLS, the pinned CA, the login and the ssb/# restriction, all at once, on a real Mosquitto."""
        from fleet import ota_matrix
        dev, cleanup = ota_matrix.over_mqtt(ota_matrix.emulated(), self.tls_port, wifi=True, user="ssb", password=self.PASSWORD, device="board-b",
                                            host="127.0.0.1", ca=str(self.paths["ca.pem"]))
        try:
            rows = ota_matrix.run_all(dev, ota_matrix.images_emulated())
        finally:
            cleanup()
        self.assertEqual([r["id"] for r in rows if not r["ok"]], [], rows)

    def test_the_server_certificate_checks_out_with_the_standard_library_too(self):
        ctx = ssl.create_default_context(cafile=str(self.paths["ca.pem"]))
        with socket.create_connection(("127.0.0.1", self.tls_port), 5) as s, ctx.wrap_socket(s, server_hostname="127.0.0.1") as t:
            self.assertIn(t.version(), ("TLSv1.2", "TLSv1.3"), "tls_version is a minimum: 1.2 or newer")
        legacy = ssl.create_default_context(cafile=str(self.paths["ca.pem"]))
        legacy.maximum_version = ssl.TLSVersion.TLSv1_2
        with socket.create_connection(("127.0.0.1", self.tls_port), 5) as s, legacy.wrap_socket(s, server_hostname="127.0.0.1") as t:
            self.assertEqual(t.version(), "TLSv1.2", "a client that only speaks 1.2 (like the board's library may) still connects")


if __name__ == "__main__":
    unittest.main()
