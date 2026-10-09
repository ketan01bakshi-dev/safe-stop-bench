"""The OTA release scenarios (REL-01..10), run on an emulated board or on the real one.

    .venv\\Scripts\\python.exe -m fleet.ota_matrix --emulated
    .venv\\Scripts\\python.exe -m fleet.ota_matrix --real COM13        board B must run SafetyNode 2.10, idle, with fleet/images built
    .venv\\Scripts\\python.exe -m fleet.ota_matrix --build-images       compile those four images (needs arduino-cli)
    .venv\\Scripts\\python.exe -m fleet.ota_matrix --emulated --mqtt   the same, through a local broker (add --mqtt to --real too)

One sequence, in order, because each step leaves the board in a known state for the next. After every reboot or reset the board's hello
must say "restored": it came back in its latched safe state (SR-17), whichever image it booted. Writes reports/fleet/ota_matrix_<where>.md/json.
Exit code 1 if any scenario fails.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import os
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from ssb.config import ROOT

from . import image
from .emulated import EmuBoard, synthetic_image
from .image import SignedImage
from .ota_client import Faults, OtaClient

OUT = ROOT / "reports" / "fleet"


@dataclass
class Dev:
    """What the scenarios need from a board, whatever it is."""
    name: str
    link: Any
    wait: Callable[[float], None]
    hard_reset: Callable[[], object]
    kind: str
    arm_cut: Callable[[int], object] = lambda n: None
    hello_s: float = 10.0   # how long to wait for a board's hello after a boot: over Wi-Fi the board must join the network and the broker first   # emulated boards cut at the nth END write; a real board has the cut compiled into its image

    def client(self, **kw: float) -> OtaClient:
        return OtaClient(self.link, **kw)


def emulated(name: str = "emu-1", version: str = "2.10") -> Dev:
    b = EmuBoard(name, version)
    return Dev(name, b, b.advance, lambda: b.reset("power loss"), "emulated", lambda n: setattr(b, "cut_end_after", n))


def real(port: str) -> Dev:
    from ssb.hil import SerialLink
    link = SerialLink(port)
    time.sleep(2.0)
    def hard_reset() -> None:
        link.reset_board()
    return Dev(f"B@{port}", link, time.sleep, hard_reset, "real")


def over_mqtt(dev: Dev, port: int, *, wifi: bool = False, user: str | None = None, password: str | None = None,
              device: str | None = None) -> tuple[Dev, Callable[[], None]]:
    """The same board, reached through the broker. Returns (device, cleanup).

    Default: a Gateway on this PC in front of the board, an MqttLink for the client (the board has no radio of its own).
    `wifi=True`, emulated: the Gateway says hello once subscribed, like the real board does, with a login on the broker. It stands in for
        a board that is on the network itself, so the --wifi code path runs in CI.
    `wifi=True`, real: NO gateway. The board is on the network, the client talks to it through the broker; only the EN-pin reset (test
        infrastructure, not part of the update) stays on the USB cable."""
    from .mqtt_link import Gateway, MqttLink
    device = device or dev.name
    emu = dev.kind == "emulated"
    if wifi and not emu:
        link = MqttLink(device, port, user=user, password=password, reset_hook=dev.hard_reset)
        wrapped = Dev(device, link, time.sleep, dev.hard_reset, "real+wifi", lambda n: None, hello_s=45.0)

        def cleanup_wifi() -> None:
            link.close()
            close = getattr(dev.link, "close", None)
            if close:
                close()
        return wrapped, cleanup_wifi
    gw = Gateway(device, dev.link, port, user=user, password=password, hello_on_connect=wifi)
    link = MqttLink(device, port, user=user, password=password)
    wait = link.advance if emu else time.sleep
    arm = (lambda n: link.control(f"cut {n}")) if emu else (lambda n: None)
    wrapped = Dev(device, link, wait, lambda: link.reset_board(), f"{dev.kind}+{'wifi' if wifi else 'mqtt'}", arm)

    def cleanup() -> None:
        link.close()
        gw.close()
        close = getattr(dev.link, "close", None)
        if close:
            close()
    return wrapped, cleanup


def images_emulated() -> dict[str, SignedImage]:
    return {"2.12": synthetic_image("2.12"), "2.13u": synthetic_image("2.13", healthy=False), "2.14": synthetic_image("2.14"), "2.15": synthetic_image("2.15")}


REAL_IMAGES = {"2.12": ("SafetyNode_2.12.bin", 12, False), "2.13u": ("SafetyNode_2.13_unhealthy.bin", 13, True),
               "2.14": ("SafetyNode_2.14.bin", 14, False), "2.15": ("SafetyNode_2.15.bin", 15, False)}


def build_real_images() -> list[Path]:
    """Compile the four images the real-board sequence needs (needs arduino-cli). They are build artifacts, not in git."""
    return [image.build(minor, unhealthy) for _, minor, unhealthy in REAL_IMAGES.values()]


def images_real() -> dict[str, SignedImage]:
    missing = [n for n, _, _ in REAL_IMAGES.values() if not (image.IMAGES / n).exists()]
    if missing:
        raise SystemExit(f"missing images in {image.IMAGES}: {', '.join(missing)}; "
                         "build them first: python -m fleet.ota_matrix --build-images")
    return {k: image.load(image.IMAGES / n, k.rstrip("u")) for k, (n, _, _) in REAL_IMAGES.items()}


CUT_POINTS = (1, 2, 3, 4, 5)   # the persistent writes that end an update: prev, tries, last, state flag, boot-slot switch
# real-board images of the cut-window sequence: key -> (minor, unhealthy, cut_at). cutN resets itself once at the Nth END write.
REAL_CUT_IMAGES = {**{f"cut{n}": (30 + n, False, n) for n in CUT_POINTS}, "bad": (40, True, None), "final": (50, False, None)}


def build_cut_images() -> list[Path]:
    return [image.build(minor, unhealthy, cut) for minor, unhealthy, cut in REAL_CUT_IMAGES.values()]


def images_cut_real() -> dict[str, SignedImage]:
    names = {k: f"SafetyNode_{image.tag_of(*v)}.bin" for k, v in REAL_CUT_IMAGES.items()}
    missing = [n for n in names.values() if not (image.IMAGES / n).exists()]
    if missing:
        raise SystemExit(f"missing images in {image.IMAGES}: {', '.join(missing)}; build them first: python -m fleet.ota_matrix --build-cut-images")
    return {k: image.load(image.IMAGES / names[k], f"2.{REAL_CUT_IMAGES[k][0]}") for k in REAL_CUT_IMAGES}


def images_cut_emulated() -> dict[str, SignedImage]:
    return {**{f"cut{n}": synthetic_image(f"2.{30 + n}") for n in CUT_POINTS}, "bad": synthetic_image("2.40", healthy=False), "final": synthetic_image("2.50")}


def _version_of(hello: str) -> str:
    import re
    m = re.search(r"SafetyNode (\d+[.]\d+)", hello)
    return m.group(1) if m else "?"


def _boot_back(dev: Dev, c: OtaClient) -> str:
    """The hello that follows the last reboot / reset (callers clear the old one first with `again`)."""
    return c.wait_for_hello(dev.hello_s) or ""


def again(dev: Dev, c: OtaClient, how: str) -> None:
    """Reboot (the command) or reset (the pin) and forget every hello from before it."""
    if how == "reboot":
        c.reboot()
    else:
        c.forget_hello()
        dev.hard_reset()
    if dev.kind.startswith("real"):
        time.sleep(0.4)   # the old image may still say hello for a moment
        c.forget_hello()


def run_all(dev: Dev, imgs: dict[str, SignedImage]) -> list[dict]:
    rows: list[dict] = []
    c = dev.client()
    again(dev, c, "reset")   # a known start: the board boots, restores its safe state, sends its hello
    start_hello = _boot_back(dev, c)
    start_version = _version_of(start_hello)

    def row(rid: str, title: str, ok: bool, evidence: str) -> None:
        rows.append({"id": rid, "title": title, "ok": bool(ok), "evidence": evidence})

    def safe(hello: str) -> bool:
        return "restored" in hello

    good = imgs["2.12"]

    # REL-01 tampered image (hash recomputed would need the key: here the bytes change after signing)
    bad = dataclasses.replace(good, data=good.data[:5000] + bytes([good.data[5000] ^ 1]) + good.data[5001:])
    r = c.push(bad)
    st = c.status()
    row("REL-01", "A tampered image is rejected before the boot slot changes", r.status == "HASH_MISMATCH" and st["state"] == "NONE", f"{r.status}; state {st['state']}")

    # REL-02 signed with the wrong key
    forged = image.sign(good.data, good.version, key=b"not-the-key")
    r = c.push(forged)
    row("REL-02", "An image signed with another key is rejected", r.status == "BAD_SIGNATURE" and c.status()["state"] == "NONE", r.status)

    # REL-03 one corrupted byte in transit
    r = c.push(good, Faults(corrupt_at=len(good.data) // 2))
    row("REL-03", "A byte corrupted in transit is caught by the hash", r.status == "HASH_MISMATCH" and c.status()["state"] == "NONE", r.status)

    # REL-04 reset in the middle of the download (1 %, 50 %, 99 %): the old image keeps running and a retry works
    parts = []
    ok = True
    for pct in (1, 50, 99):
        c.forget_hello()
        r = c.push(good, Faults(reset_at_pct=pct))   # the push resets the board itself at that point
        hello = _boot_back(dev, c)
        s = c.status()
        good_here = r.outcome == "reset" and _version_of(hello) == start_version and s["state"] == "NONE" and safe(hello)
        ok &= good_here
        parts.append(f"{pct}%: {r.outcome}, runs {_version_of(hello)}, {s['state']}, safe state {'restored' if safe(hello) else 'NOT restored'}")
    row("REL-04", "A reset at 1 / 50 / 99 % of the download leaves the old image running in its safe state", ok, "; ".join(parts))

    # REL-05 lost and repeated chunks, then the update completes and commits
    drops = [200 * k for k in (5, 40, 90)]
    r = c.push(good, Faults(drop_chunks=drops, resend_every=7))
    staged = r.outcome == "staged"
    again(dev, c, "reboot")
    hello = _boot_back(dev, c)
    dev.wait(3.6)
    s = c.status()
    row("REL-05", "Lost and repeated chunks are resumed; the update commits", staged and r.resumes >= 3 and s["last"].startswith("committed 2.12") and _version_of(hello) == "2.12" and safe(hello),
        f"{r.chunks} chunks, {r.resumes} resumes; trial boot hello '{hello[:34]}'; {s['last']}")

    # REL-06 an image whose health check fails goes back on its own (the first hello is already the old image's)
    c.push(imgs["2.13u"])
    again(dev, c, "reboot")
    hello6 = _boot_back(dev, c)
    dev.wait(3.6)
    s = c.status()
    row("REL-06", "An image that fails its health check rolls back automatically", s["state"] == "ROLLED_BACK" and "health check" in s["last"] and _version_of(hello6) == "2.12" and safe(hello6),
        f"the board says '{hello6[:34]}'; {s['state']}: {s['last']}")

    # REL-07 a reset (power loss) during the trial boot, before the image confirmed itself
    c.push(imgs["2.14"])
    if dev.kind.endswith("+wifi"):
        # The trial hello comes over the network, after the radio is up: later than the 3 s probation. So the reset is timed from the
        # reboot command, on the cable, and the proof is the same: the second boot is the OLD image and the state says ROLLED_BACK.
        c.forget_hello()
        c.reboot()
        time.sleep(1.2) if dev.kind.startswith("real") else dev.wait(1.2)
        again(dev, c, "reset")
        h1, h2 = "(not observable over Wi-Fi inside the 3 s probation)", _boot_back(dev, c)
        dev.wait(3.6)
        s = c.status()
        row("REL-07", "A reset during the trial boot rolls back to the old image, in its safe state", _version_of(h2) == "2.12" and safe(h2) and s["state"] == "ROLLED_BACK" and "reset before" in s["last"],
            f"trial {h1} -> after reset {h2[:30]}; {s['last']}")
    else:
        again(dev, c, "reboot")
        h1 = _boot_back(dev, c)
        dev.wait(1.0)
        again(dev, c, "reset")
        h2 = _boot_back(dev, c)
        dev.wait(3.6)
        s = c.status()
        row("REL-07", "A reset during the trial boot rolls back to the old image, in its safe state", _version_of(h1) == "2.14" and _version_of(h2) == "2.12" and safe(h2) and s["state"] == "ROLLED_BACK",
            f"trial {h1[:30]} -> after reset {h2[:30]}; {s['last']}")

    # REL-08 a reset after the image was staged but before the reboot command: the new image boots in trial and commits
    c.push(imgs["2.15"])
    again(dev, c, "reset")
    h = _boot_back(dev, c)
    dev.wait(3.6)
    s = c.status()
    row("REL-08", "A reset between 'staged' and 'reboot' boots the new image, which commits", _version_of(h) == "2.15" and safe(h) and s["last"].startswith("committed 2.15"), f"{h[:30]}; {s['last']}")

    # REL-09 no downgrade
    r = c.push(good)
    row("REL-09", "An older version is refused (anti-rollback)", r.status == "DOWNGRADE", f"{r.status}; still running {_version_of(h)}")

    # REL-10 a trial image never loses the safe state: every hello in this run said "restored"
    row("REL-10", "The board came back in its latched safe state after every boot", all("restored" in x for x in (start_hello, hello, hello6, h1, h2, h) if not x.startswith("(")), "all hellos say restored")
    return rows


def render(rows: list[dict], where: str) -> str:
    lines = [f"# OTA release scenarios ({where})", "", "| ID | Scenario | Result | Evidence |", "|---|---|---|---|"]
    for r in rows:
        lines.append(f"| {r['id']} | {r['title']} | {'PASS' if r['ok'] else '**FAIL**'} | {r['evidence']} |")
    lines += ["", f"{sum(r['ok'] for r in rows)}/{len(rows)} pass."]
    return "\n".join(lines) + "\n"


def _settle(dev: Dev, c: OtaClient, seconds: float) -> str:
    """Read what the board says for a while and return its LAST hello: after a cut it may boot twice (new image, then the rollback)."""
    end = time.time() + seconds
    while time.time() < end:
        for kind, p in c.parser.feed(dev.link.read()):
            if kind == "H":
                c.hello = p.decode(errors="replace")
        time.sleep(0.05)
    return c.hello or ""


def run_cut_windows(dev: Dev, imgs: dict[str, SignedImage]) -> list[dict]:
    """REL-11 / REL-12: power lost at each of the five persistent writes that end an update.

    For each cut point n: install the build that cuts at n (cutN), then push an UNHEALTHY image to it. The cut falls inside that update.
    Whatever the cut point, the unhealthy image must not be the one running afterwards, the board must say it restored its safe
    state, and the next update must go through (REL-12). Ends on a plain release image. Needs a board that already runs the fixed END order.
    """
    end_timeout = 1.0 if dev.kind.startswith("emulated") else 8.0
    c = dev.client(end_timeout_s=end_timeout)
    again(dev, c, "reset")
    _boot_back(dev, c)

    def install(key: str) -> tuple[bool, str]:
        r = c.push(imgs[key])
        if r.outcome != "staged":
            return False, f"push {key}: {r.outcome} {r.status}"
        c.forget_hello()
        c.reboot()
        hello = c.wait_for_hello(dev.hello_s) or ""
        dev.wait(3.6)
        st = c.status()
        ok = _version_of(hello) == imgs[key].version and "restored" in hello and st["state"] == "NONE"
        return ok, f"{key} installed: runs {_version_of(hello)}, {st['state']}"

    per_cut: list[tuple[bool, str]] = []
    per_retry: list[tuple[bool, str]] = []
    ok_in, why_in = install("cut1")
    for n in CUT_POINTS:
        dev.arm_cut(n)
        c.forget_hello()
        r = c.push(imgs["bad"])
        hello = _settle(dev, c, 7.0 if dev.kind.startswith("real") else 0.3)
        st = c.status()
        runs = _version_of(hello)
        ok = (ok_in or n > 1) and r.outcome == "lost" and runs == f"2.{30 + n}" and "restored" in hello and st["state"] != "TRIAL"
        per_cut.append((ok, f"cut {n}: {r.outcome}, runs {runs}, {st['state']} ({st['last'][:38]})"))
        ok_in = True
        ok2, why2 = install(f"cut{n + 1}" if n < CUT_POINTS[-1] else "final")
        per_retry.append((ok2, why2))
    rows = [{"id": "REL-11", "title": "A power loss at any of the five final writes of an update never leaves an unhealthy image running, in the safe state",
             "ok": all(o for o, _ in per_cut) and ok_in, "evidence": "; ".join(w for _, w in per_cut)},
            {"id": "REL-12", "title": "After a cut at any of those writes, the next update goes through",
             "ok": all(o for o, _ in per_retry), "evidence": "; ".join(w for _, w in per_retry)}]
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--emulated", action="store_true")
    g.add_argument("--real", metavar="COM")
    g.add_argument("--build-images", action="store_true", help="compile the four images the --real sequence needs, then stop")
    g.add_argument("--build-cut-images", action="store_true", help="compile the seven images of the --cut-windows sequence, then stop")
    ap.add_argument("--cut-windows", action="store_true", help="run REL-11 / REL-12 (power lost at each final write of an update) instead of REL-01..10")
    ap.add_argument("--wifi", action="store_true", help="the board is on the network itself (provision it first: scripts/provision_wifi.py): "
                    "this PC runs a login-protected broker on the LAN and the update traffic goes over Wi-Fi. With --emulated, a stand-in board.")
    ap.add_argument("--mqtt", action="store_true", help="reach the board through a local MQTT broker and a gateway instead of the direct link")
    a = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    if a.build_images:
        for q in build_real_images():
            print(f"built {q.name} ({q.stat().st_size} bytes)")
        return 0
    if a.build_cut_images:
        for q in build_cut_images():
            print(f"built {q.name} ({q.stat().st_size} bytes)")
        return 0
    if a.cut_windows:
        dev, imgs, where = (emulated(version="2.16"), images_cut_emulated(), "emulated") if a.emulated else (real(a.real), images_cut_real(), f"real board {a.real}")
    else:
        dev, imgs, where = (emulated(), images_emulated(), "emulated") if a.emulated else (real(a.real), images_real(), f"real board {a.real}")
    cleanup: Callable[[], None] = lambda: None   # noqa: E731
    broker = None
    if a.wifi:
        import tempfile

        from . import broker as brk
        from .mqtt_link import LocalBroker
        if a.emulated:   # a throw-away login: the real path (password file, refused logins) is exercised, no secret exists
            tmp = Path(tempfile.mkdtemp()) / "passwd"
            user, password, device = "ssb-test", "test-password-" + str(time.time_ns()), "board-b"
            brk.write_login(tmp, user, password)
            broker = LocalBroker(password_file=str(tmp)).start()
            where += " over Wi-Fi stand-in (login-protected broker)"
        else:
            import getpass
            user = os.environ.get("SSB_MQTT_USER", "ssb")
            password = os.environ.get("SSB_MQTT_PASS", "")
            if not brk.DEFAULT_FILE.exists():
                raise SystemExit("--wifi needs the broker login: run `python -m fleet.broker login` first")
            if not password:
                if not sys.stdin.isatty():
                    raise SystemExit("SSB_MQTT_PASS is not set and there is no terminal to ask on")
                password = getpass.getpass(f"broker password for '{user}' (hidden; the one you gave `fleet.broker login`): ")
            from .provision import Provisioner, describe
            info = Provisioner(dev.link).info()
            print(describe(info))
            if not info.provisioned:
                raise SystemExit("board B has no network settings: run scripts/provision_wifi.py --port " + a.real)
            device = info.device
            broker = LocalBroker(port=int(os.environ.get("SSB_MQTT_PORT", str(brk.DEFAULT_PORT))), bind="0.0.0.0", password_file=str(brk.DEFAULT_FILE)).start()
            print(f"broker up for the LAN on port {broker.port}; the board must reach {', '.join(brk.lan_addresses())}")
            where += " over its own Wi-Fi radio (login-protected broker on this PC)"
        dev, cleanup = over_mqtt(dev, broker.port, wifi=True, user=user, password=password, device=device)
    elif a.mqtt:
        from .mqtt_link import LocalBroker
        broker = LocalBroker().start()
        dev, cleanup = over_mqtt(dev, broker.port)
        where += " over MQTT (local broker, PC gateway)"
    try:
        rows = run_cut_windows(dev, imgs) if a.cut_windows else run_all(dev, imgs)
    finally:
        cleanup()
        if broker:
            broker.stop()
    OUT.mkdir(parents=True, exist_ok=True)
    tag = ("emulated" if a.emulated else "real") + ("_wifi" if a.wifi else "_mqtt" if a.mqtt else "")
    stem = "ota_cut" if a.cut_windows else "ota_matrix"
    (OUT / f"{stem}_{tag}.json").write_text(json.dumps(rows, indent=1), encoding="utf-8")
    text = render(rows, where)
    (OUT / f"{stem}_{tag}.md").write_text(text, encoding="utf-8")
    print(text)
    return 0 if all(r["ok"] for r in rows) else 1


if __name__ == "__main__":
    sys.exit(main())
