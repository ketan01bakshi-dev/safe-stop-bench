"""Put the board's Wi-Fi and broker settings into its flash over the USB cable (v2.28). The PC side of ssc_net.h's 'N' messages.

    SET    port:u16 LE + six NUL-terminated strings (ssid, wifi password, broker host, broker user, broker password, device id) + a flags
           byte (bit0 = TLS)
    CLEAR  forget everything
    INFO   flags (provisioned / wifi up / mqtt up / TLS on / CA stored / MQTT refused), rssi, free and lowest-free heap, CA length, then ssid, ip,
           host, device id. The board never sends a password back.
    CA_BEGIN / CA_CHUNK / CA_END   upload the CA certificate (public) that the board will pin for TLS

The command line is scripts/provision_wifi.py; it reads the secrets from environment variables or a hidden prompt. Nothing here writes
a secret to disk, to a log or to a report, and `describe()` is the only place a result is turned into text: it has no password to show.
"""
from __future__ import annotations

import struct
import time
from dataclasses import dataclass

from ssb.hil import Parser, frame_msg

SET, CLEAR, INFO, CA_BEGIN, CA_CHUNK, CA_END = 1, 2, 3, 4, 5, 6
STATUS = {0: "OK", 1: "BAD_MESSAGE", 2: "TOO_LONG", 3: "NO_STORE", 4: "BAD_CA"}
CA_MAX = 2000
CA_CHUNK_BYTES = 200
# max characters per field = the firmware's buffer minus its terminating NUL
LIMITS = {"ssid": 32, "wifi_password": 63, "host": 64, "user": 32, "mqtt_password": 64, "device": 16}


class ProvisionError(RuntimeError):
    pass


@dataclass
class NetInfo:
    provisioned: bool
    wifi_up: bool
    mqtt_up: bool
    rssi: int
    ssid: str
    ip: str
    host: str
    device: str
    tls: bool = False
    ca_stored: bool = False
    ca_len: int = 0
    heap_free: int = 0
    heap_min: int = 0
    mqtt_refused: bool = False   # v2.34: the firmware was built without the link flag that pins the esp-mqtt task to core 0, so it will not run the client


def set_payload(ssid: str, wifi_password: str, host: str, port: int, user: str, mqtt_password: str, device: str, tls: bool = False) -> bytes:
    """The body of a SET message. Refuses what the board would refuse, with a reason, before anything is sent."""
    fields = {"ssid": ssid, "wifi_password": wifi_password, "host": host, "user": user, "mqtt_password": mqtt_password, "device": device}
    for name, value in fields.items():
        raw = value.encode("utf-8")
        if b"\0" in raw:
            raise ProvisionError(f"{name} contains a NUL character")
        if len(raw) > LIMITS[name]:
            raise ProvisionError(f"{name} is {len(raw)} bytes; the board stores at most {LIMITS[name]}")
    for name in ("ssid", "host", "device"):
        if not fields[name]:
            raise ProvisionError(f"{name} is required")
    if not 1 <= port <= 65535:
        raise ProvisionError("the broker port must be 1..65535")
    if wifi_password and len(wifi_password) < 8:
        raise ProvisionError("a WPA2 password is 8 to 63 characters (leave it empty only for an open network)")
    body = bytes([SET]) + struct.pack("<H", port) + b"".join(v.encode("utf-8") + b"\0" for v in fields.values()) + bytes([1 if tls else 0])
    if len(body) > 255:
        raise ProvisionError(f"the message is {len(body)} bytes; one message holds at most 255")
    return body


def check_ca(pem: bytes) -> bytes:
    """The CA certificate as the board will store it (LF line ends, one trailing newline), or the reason it is the wrong file.
    Anything carrying a PRIVATE KEY is refused first: the CA key must never leave this PC."""
    if b"PRIVATE KEY" in pem:
        raise ProvisionError("this file contains a PRIVATE KEY: send only the public ca.pem, never ca.key")
    if not pem.startswith(b"-----BEGIN CERTIFICATE-----") or b"-----END CERTIFICATE-----" not in pem:
        raise ProvisionError("that is not a PEM certificate (expected -----BEGIN CERTIFICATE-----)")
    pem = pem.replace(b"\r\n", b"\n").strip() + b"\n"
    if not 100 <= len(pem) <= CA_MAX:
        raise ProvisionError(f"the certificate is {len(pem)} bytes; the board stores 100 to {CA_MAX}")
    return pem


class Provisioner:
    """`link` is anything with write(bytes, now_ms) and read() -> bytes: a SerialLink, or an EmuBoard in the tests."""

    def __init__(self, link, timeout_s: float = 3.0):
        self.link, self.timeout_s, self.parser = link, timeout_s, Parser()

    def _call(self, payload: bytes) -> tuple[str, bytes]:
        self.parser.feed(self.link.read())   # drop anything older (a hello, a stale answer)
        self.link.write(frame_msg("N", payload), 0)
        end = time.time() + self.timeout_s
        while time.time() < end:
            for kind, p in self.parser.feed(self.link.read()):
                if kind == "n" and len(p) >= 2 and p[0] == payload[0]:
                    return STATUS.get(p[1], str(p[1])), bytes(p[2:])
            time.sleep(0.02)
        raise ProvisionError("no answer to the network set-up message: is board B running a firmware with Wi-Fi support (SafetyNode 2.28+)?")

    def set(self, ssid: str, wifi_password: str, host: str, port: int, user: str, mqtt_password: str, device: str, tls: bool = False) -> None:
        status, _ = self._call(set_payload(ssid, wifi_password, host, port, user, mqtt_password, device, tls))
        if status != "OK":
            raise ProvisionError(f"the board refused the settings: {status}")

    def set_ca(self, pem: bytes) -> None:
        """Upload the CA certificate (public). The board checks it looks like a certificate and keeps it in flash."""
        pem = check_ca(pem)
        status, _ = self._call(bytes([CA_BEGIN]) + struct.pack("<H", len(pem)))
        if status != "OK":
            raise ProvisionError(f"the board refused the certificate upload: {status}")
        for off in range(0, len(pem), CA_CHUNK_BYTES):
            status, _ = self._call(bytes([CA_CHUNK]) + struct.pack("<H", off) + pem[off:off + CA_CHUNK_BYTES])
            if status != "OK":
                raise ProvisionError(f"the board refused part of the certificate at byte {off}: {status}")
        status, _ = self._call(bytes([CA_END]))
        if status != "OK":
            raise ProvisionError(f"the board did not accept the certificate: {status}")

    def clear(self) -> None:
        status, _ = self._call(bytes([CLEAR]))
        if status != "OK":
            raise ProvisionError(f"the board refused to clear: {status}")

    def info(self) -> NetInfo:
        status, extra = self._call(bytes([INFO]))
        if status != "OK" or len(extra) < 12:
            raise ProvisionError(f"INFO failed: {status}")
        flags, rssi = extra[0], struct.unpack("b", extra[1:2])[0]
        heap_free, heap_min, ca_len = struct.unpack_from("<IIH", extra, 2)
        parts = [x.decode("utf-8", errors="replace") for x in extra[12:].split(b"\0")] + ["", "", "", ""]
        return NetInfo(bool(flags & 1), bool(flags & 2), bool(flags & 4), rssi, parts[0], parts[1], parts[2], parts[3],
                       tls=bool(flags & 8), ca_stored=bool(flags & 16), ca_len=ca_len, heap_free=heap_free, heap_min=heap_min,
                       mqtt_refused=bool(flags & 32))

    def wait_online(self, timeout_s: float = 45.0, on_progress=None) -> NetInfo:
        """Poll INFO until Wi-Fi and the broker are both up, or the time is out. Returns the last INFO either way."""
        end, last = time.time() + timeout_s, None
        while time.time() < end:
            last = self.info()
            if on_progress:
                on_progress(last)
            if last.wifi_up and last.mqtt_up:
                return last
            time.sleep(1.0)
        return last or self.info()


def describe(i: NetInfo) -> str:
    if not i.provisioned:
        return "nothing provisioned: the radio stays off"
    state = "Wi-Fi up" + (f" ({i.ip}, {i.rssi} dBm)" if i.wifi_up else " NOT yet") + ", broker " + ("connected" if i.mqtt_up else "NOT connected yet")
    link = ("TLS, CA pinned" if i.ca_stored else "TLS ON BUT NO CA STORED: the radio stays off") if i.tls else "plain MQTT (no TLS)"
    if i.mqtt_refused:
        link += "; MQTT REFUSED: this firmware was built without ssb.firmware_build.LINK_FLAGS (the esp-mqtt task is not pinned to core 0), rebuild with scripts/hil_flash.py"
    return f"network '{i.ssid}', broker {i.host}, device id '{i.device}', {link}: {state}; free heap {i.heap_free // 1024} KiB (lowest {i.heap_min // 1024})"
