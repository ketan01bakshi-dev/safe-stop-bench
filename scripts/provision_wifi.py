"""Give board B its Wi-Fi and broker settings over the USB cable (v2.28). Run it in YOUR terminal: you type the secrets, not a chat.

    .venv\\Scripts\\python.exe scripts/provision_wifi.py --port COM13            set (asks for anything not in the environment)
    .venv\\Scripts\\python.exe scripts/provision_wifi.py --port COM13 --info     what the board has: network, address, connected? (no passwords)
    .venv\\Scripts\\python.exe scripts/provision_wifi.py --port COM13 --clear    forget everything; the radio stays off from then on
    .venv\\Scripts\\python.exe scripts/provision_wifi.py --dry-run               check your entries against the board's limits, send nothing

Environment variables it reads (anything missing is asked for; passwords are asked for hidden and never echoed, stored or printed):
    SSB_WIFI_SSID   SSB_WIFI_PASS   your 2.4 GHz network and its password
    SSB_MQTT_HOST   this PC's LAN address (python -m fleet.broker addresses)      SSB_MQTT_PORT  default 1884
    SSB_MQTT_USER   SSB_MQTT_PASS   the login you made with `python -m fleet.broker login`
    SSB_DEVICE      the board's id in the topics, default board-b
    SSB_TLS         1 (default when secrets/tls/ca.pem exists) or 0: encrypt the link and pin the CA certificate on the board
    SSB_TLS_CA      path of the CA certificate (public ca.pem), default secrets/tls/ca.pem; never the .key

The settings are stored in the board's flash (not in the firmware), so the signed images stay free of secrets. See docs/WIFI_SETUP.md.
"""
from __future__ import annotations

import argparse
import getpass
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fleet import broker, certs, provision  # noqa: E402


def ask(env: str, prompt: str, secret: bool = False, default: str | None = None) -> str:
    v = os.environ.get(env)
    if v is not None:
        return v
    shown = f"{prompt}" + (f" [{default}]" if default else "") + ": "
    if not sys.stdin.isatty():   # no terminal to ask on (a script, a pipe): a default is fine, a missing secret is not
        if default is not None:
            return default
        raise SystemExit(f"{env} is not set and there is no terminal to ask on")
    try:
        got = getpass.getpass(shown) if secret else input(shown)
    except EOFError:   # no terminal to ask on (a script, a pipe): a default is fine, a missing secret is not
        if default is not None:
            return default
        raise SystemExit(f"{env} is not set and there is no terminal to ask on") from None
    return got or (default or "")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="board B's serial port, e.g. COM13")
    ap.add_argument("--info", action="store_true")
    ap.add_argument("--clear", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--no-wait", action="store_true", help="do not wait for the board to join the network and reach the broker")
    a = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    if not a.dry_run and not a.port:
        ap.error("--port is required (board B's serial port)")
    link = None
    if a.port and not a.dry_run and (a.info or a.clear):
        from ssb.hil import SerialLink
        link = SerialLink(a.port)
    try:
        if a.info or a.clear:
            p = provision.Provisioner(link)
            if a.clear:
                p.clear()
                print("cleared: the board forgot its network and broker settings")
            print(provision.describe(p.info()))
            return 0

        ssid = ask("SSB_WIFI_SSID", "Wi-Fi network name (2.4 GHz)")
        wpass = ask("SSB_WIFI_PASS", "Wi-Fi password", secret=True)
        guess = (broker.lan_addresses() or [""])[0]
        host = ask("SSB_MQTT_HOST", "this PC's LAN address (the broker)", default=guess)
        ca_path = Path(os.environ.get("SSB_TLS_CA") or (certs.DEFAULT_DIR / "ca.pem"))
        use_tls = ask("SSB_TLS", "encrypt with TLS and pin the CA certificate? (1 = yes, 0 = no)", default="1" if ca_path.exists() else "0") == "1"
        if use_tls and not ca_path.exists():
            raise SystemExit(f"TLS needs the CA certificate: {ca_path} not found (python -m fleet.certs make --host <this PC's address>)")
        port = int(ask("SSB_MQTT_PORT", "broker port", default=str(8883 if use_tls else broker.DEFAULT_PORT)))
        user = ask("SSB_MQTT_USER", "broker user", default="ssb")
        mpass = ask("SSB_MQTT_PASS", "broker password", secret=True)
        device = ask("SSB_DEVICE", "board id", default="board-b")
        body = provision.set_payload(ssid, wpass, host, port, user, mpass, device, use_tls)   # refuses what the board would refuse
        ca_pem = provision.check_ca(ca_path.read_bytes()) if use_tls else b""   # refuses a key file or a non-certificate before anything is sent
        print(f"settings are valid ({len(body)} bytes): network '{ssid}', broker {host}:{port} as '{user}', board id '{device}', "
              + (f"TLS with CA {certs.fingerprint(ca_pem)[:23]}..." if use_tls else "NO TLS (plain MQTT)"))
        if a.dry_run:
            print("dry run: nothing sent")
            return 0
        # open the port only now: kept open through the prompts, the board did not answer the first message (found on the real board)
        from ssb.hil import SerialLink
        link = SerialLink(a.port)
        p = provision.Provisioner(link)
        if use_tls:
            p.set_ca(ca_pem)   # first: with TLS on and no CA the radio stays off, so the order never leaves a window of plain MQTT
        p.set(ssid, wpass, host, port, user, mpass, device, use_tls)
        print("stored in the board's flash" + (f" (CA certificate {len(ca_pem)} bytes, pinned)" if use_tls else "")
              + ". The radio comes up within a few seconds (it stays off while a scenario runs).")
        if a.no_wait:
            return 0
        print(f"waiting for the network and for the broker at {host}:{port} (python -m fleet.broker serve must be running) ...")
        info = p.wait_online(45.0, on_progress=lambda i: print("  " + provision.describe(i)))
        if info.wifi_up and info.mqtt_up:
            print("OK: the board is on the network and connected to the broker.")
            return 0
        if not info.wifi_up:
            print("The board did not join the network. Check: 2.4 GHz network (not 5 GHz), the exact name and password, not a guest network.")
        else:
            print("The board is on the network but not on the broker. Check: the broker is running (python -m fleet.broker serve),")
            print("the address above is this PC's, the user and password match `fleet.broker login`, and Windows Firewall allows TCP 1884 (Private).")
        return 2
    except provision.ProvisionError as e:
        print(f"error: {e}")
        return 1
    finally:
        if link is not None:
            link.close()


if __name__ == "__main__":
    sys.exit(main())
