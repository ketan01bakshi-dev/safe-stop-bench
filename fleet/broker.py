"""The MQTT broker for boards on your LAN (v2.28): a login you choose, and a server that listens beyond this PC.

    python -m fleet.broker login --user ssb        asks for a password twice (hidden), stores only its argon2 hash in secrets/broker_passwd
    python -m fleet.broker serve                   listens on 0.0.0.0:1884 with that login; prints the addresses a board can use
    python -m fleet.broker addresses               this PC's LAN addresses (what goes into SSB_MQTT_HOST)

The password file holds a hash, never the password. It is git-ignored (secrets/), as is everything else in that folder.
`serve` is plain MQTT without TLS: fine on a home network you trust for a bench, not for anything else.
"""
from __future__ import annotations

import argparse
import getpass
import os
import socket
import sys
import time
from pathlib import Path

from ssb.config import ROOT

DEFAULT_FILE = ROOT / "secrets" / "broker_passwd"
DEFAULT_PORT = 1884   # not 1883: a Mosquitto or another MQTT project often owns the standard port on the same PC


def lan_addresses() -> list[str]:
    """IPv4 addresses of this PC that are not loopback. The first one is usually the Wi-Fi/Ethernet address the router gave it."""
    found: list[str] = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))   # no packet is sent: this only asks the OS which interface it would use
        found.append(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = str(info[4][0])
            if not ip.startswith("127.") and ip not in found:
                found.append(ip)
    except OSError:
        pass
    return [ip for ip in found if not ip.startswith("127.")]


def hash_password(password: str) -> str:
    from pwdlib import PasswordHash
    from pwdlib.hashers.argon2 import Argon2Hasher
    return PasswordHash((Argon2Hasher(),)).hash(password)


def write_login(path: Path, user: str, password: str) -> None:
    if not user or ":" in user or any(c.isspace() for c in user):
        raise ValueError("the user name must be non-empty, with no spaces or colon")
    if len(password) < 8:
        raise ValueError("the broker password must be at least 8 characters")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(f"{user}:{hash_password(password)}\n", encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    lg = sub.add_parser("login", help="create or replace the broker login")
    lg.add_argument("--user", default="ssb")
    lg.add_argument("--file", type=Path, default=DEFAULT_FILE)
    sv = sub.add_parser("serve", help="run the broker for boards on the LAN")
    sv.add_argument("--port", type=int, default=DEFAULT_PORT)
    sv.add_argument("--file", type=Path, default=DEFAULT_FILE)
    sv.add_argument("--local-only", action="store_true", help="listen on 127.0.0.1 only")
    sub.add_parser("addresses", help="print this PC's LAN addresses")
    a = ap.parse_args()

    if a.cmd == "addresses":
        print("\n".join(lan_addresses()) or "no LAN address found: is this PC on the network?")
        return 0
    if a.cmd == "login":
        pw = os.environ.get("SSB_MQTT_PASS") or getpass.getpass("broker password (hidden, 8+ characters): ")
        if "SSB_MQTT_PASS" not in os.environ and getpass.getpass("again: ") != pw:
            print("the two entries differ; nothing written")
            return 1
        write_login(a.file, a.user, pw)
        print(f"login '{a.user}' stored (hash only) in {a.file}\nuse the SAME user and password when you provision the board (SSB_MQTT_USER / SSB_MQTT_PASS)")
        return 0

    from .mqtt_link import LocalBroker
    if not a.file.exists():
        print(f"no login file at {a.file}: run `python -m fleet.broker login` first (an open broker on the LAN is refused)")
        return 1
    bind = "127.0.0.1" if a.local_only else "0.0.0.0"
    with LocalBroker(port=a.port, bind=bind, password_file=str(a.file)) as b:
        addrs = lan_addresses()
        print(f"broker up on {bind}:{b.port}, login required")
        if not a.local_only:
            print("a board reaches it at: " + ", ".join(f"{ip}:{b.port}" for ip in addrs) if addrs else "no LAN address found")
            print("if the board cannot connect, allow inbound TCP %d for the Private network profile in Windows Firewall" % b.port)
        print("Ctrl+C to stop")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
