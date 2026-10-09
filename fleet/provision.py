"""Put the board's Wi-Fi and broker settings into its flash over the USB cable (v2.28). The PC side of ssc_net.h's 'N' messages.

    SET    port:u16 LE + six NUL-terminated strings (ssid, wifi password, broker host, broker user, broker password, device id)
    CLEAR  forget everything
    INFO   flags (provisioned / wifi up / mqtt up), rssi, ssid, ip, host, device id. The board never sends a password back.

The command line is scripts/provision_wifi.py; it reads the secrets from environment variables or a hidden prompt. Nothing here writes
a secret to disk, to a log or to a report, and `describe()` is the only place a result is turned into text: it has no password to show.
"""
from __future__ import annotations

import struct
import time
from dataclasses import dataclass

from ssb.hil import Parser, frame_msg

SET, CLEAR, INFO = 1, 2, 3
STATUS = {0: "OK", 1: "BAD_MESSAGE", 2: "TOO_LONG", 3: "NO_STORE"}
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


def set_payload(ssid: str, wifi_password: str, host: str, port: int, user: str, mqtt_password: str, device: str) -> bytes:
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
    body = bytes([SET]) + struct.pack("<H", port) + b"".join(v.encode("utf-8") + b"\0" for v in fields.values())
    if len(body) > 255:
        raise ProvisionError(f"the message is {len(body)} bytes; one message holds at most 255")
    return body


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

    def set(self, ssid: str, wifi_password: str, host: str, port: int, user: str, mqtt_password: str, device: str) -> None:
        status, _ = self._call(set_payload(ssid, wifi_password, host, port, user, mqtt_password, device))
        if status != "OK":
            raise ProvisionError(f"the board refused the settings: {status}")

    def clear(self) -> None:
        status, _ = self._call(bytes([CLEAR]))
        if status != "OK":
            raise ProvisionError(f"the board refused to clear: {status}")

    def info(self) -> NetInfo:
        status, extra = self._call(bytes([INFO]))
        if status != "OK" or len(extra) < 2:
            raise ProvisionError(f"INFO failed: {status}")
        flags, rssi = extra[0], struct.unpack("b", extra[1:2])[0]
        parts = [x.decode("utf-8", errors="replace") for x in extra[2:].split(b"\0")] + ["", "", "", ""]
        return NetInfo(bool(flags & 1), bool(flags & 2), bool(flags & 4), rssi, parts[0], parts[1], parts[2], parts[3])

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
    return f"network '{i.ssid}', broker {i.host}, device id '{i.device}': {state}"
