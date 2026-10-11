"""The Mosquitto configuration for the boards (v2.30): one extra TLS listener with a login, next to whatever the broker already does.

    .venv\\Scripts\\python.exe -m fleet.mosquitto_conf --existing "C:\\Program Files\\Mosquitto\\mosquitto.conf" --dir "C:\\Program Files\\Mosquitto\\ssb"

prints the whole new configuration (nothing is written; scripts/setup_mosquitto_tls.ps1 does the writing, with a backup, as Administrator).

What it adds, as ONE block between two marker lines, so running it again replaces the block instead of stacking another one:

    listener 8883               TLS 1.2 or newer (tls_version is a MINIMUM in Mosquitto 2.x), the certificate from `python -m fleet.certs`
    listener_allow_anonymous false   a login is required (password file made with mosquitto_passwd)
    acl_file ...                that login may read and write only ssb/# and nothing else

and, because password_file and acl_file must belong to this listener alone, one line at the top, `per_listener_settings true`. With it every
listener carries its own rules, so an existing `listener 1883` with `allow_anonymous true` keeps behaving exactly as it did.

Mosquitto 2.1 calls `per_listener_settings` deprecated (it goes away in 3.0, where the password and ACL *plugins* take over) and logs a
warning about it at start-up; it still works, and it is the only way in 2.1 to give one listener its own password and ACL files. Moving to the
plugins is a change to make when Mosquitto 3 is installed.
"""
from __future__ import annotations

import argparse
import re
import sys

BEGIN = "# --- safe-stop bench: begin (managed by fleet.mosquitto_conf; edit outside this block) ---"
END = "# --- safe-stop bench: end ---"


def block(install_dir: str, port: int = 8883) -> str:
    d = install_dir.rstrip("\\/")
    sep = "\\" if "\\" in d else "/"
    return "\n".join([
        BEGIN,
        f"listener {port}",
        "protocol mqtt",
        f"cafile {d}{sep}ca.pem",
        f"certfile {d}{sep}server.pem",
        f"keyfile {d}{sep}server.key",
        "tls_version tlsv1.2",
        "listener_allow_anonymous false",
        f"password_file {d}{sep}passwd",
        f"acl_file {d}{sep}acl",
        END,
        "",
    ])


def acl(user: str = "ssb") -> str:
    return f"user {user}\ntopic readwrite ssb/#\n"


def merge(existing: str, install_dir: str, port: int = 8883) -> str:
    """The existing configuration, with our block (replacing an older one) at the end and per_listener_settings true in front."""
    text = existing.replace("\r\n", "\n")
    text = re.sub(re.escape(BEGIN) + r".*?" + re.escape(END) + r"\n?", "", text, flags=re.S).rstrip("\n") + "\n"
    if not re.search(r"^\s*per_listener_settings\s+true\b", text, flags=re.M):
        if re.search(r"^\s*per_listener_settings\s+false\b", text, flags=re.M):
            raise ValueError("the existing configuration sets per_listener_settings false: change that line to true by hand, then run this again "
                             "(it changes how every listener's settings are read, so it is not done silently)")
        text = "per_listener_settings true\n\n" + text
    return text + "\n" + block(install_dir, port)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--existing", help="path of the current mosquitto.conf (omit for a new one)")
    ap.add_argument("--dir", required=True, help="the folder that will hold ca.pem, server.pem, server.key, passwd and acl")
    ap.add_argument("--port", type=int, default=8883)
    ap.add_argument("--print-acl", action="store_true")
    a = ap.parse_args()
    if a.print_acl:
        sys.stdout.write(acl())
        return 0
    existing = ""
    if a.existing:
        with open(a.existing, encoding="utf-8") as f:
            existing = f.read()
    try:
        sys.stdout.write(merge(existing, a.dir, a.port))
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
