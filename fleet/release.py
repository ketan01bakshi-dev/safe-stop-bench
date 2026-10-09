"""The release decision (P4): bench result + attack matrix + OTA result + rollout in one verdict, with the evidence and the rules.

    .venv\\Scripts\\python.exe -m fleet.release        -> reports/fleet/release_report.md / .json

Rules (stated, so the verdict is reproducible and arguable):
  BLOCKED        evidence missing or unusable (no preflight, no OTA result, bench not fit): decide nothing, fix the evidence first
  NO GO          a product failure (a FAIL the known-good reference passes, not a seeded defect) in the latest campaigns; an attack scenario that
                 fails its safety expectation; an OTA scenario that fails; a rollout that did not halt a bad build
  GO WITH RISKS  nothing blocking, but open findings, suspect cases or documented residual risks remain (they are listed, with their owners' next step)
  GO             nothing blocking and no open finding

The verdict is a recommendation. The owner of the release owns the risk; this report owns the truth of the evidence.
"""
from __future__ import annotations

import hashlib
import json
import time
from dataclasses import dataclass, field
from pathlib import Path

from health import dashboard
from ssb.config import ROOT

REPORTS = ROOT / "reports"
OUT = REPORTS / "fleet"

LIMITS = [  # what this evidence does NOT cover; always on the risk register
    ("The signing key is a demo key compiled into the image", "High for a product", "Sign on a build server; keep the verification key in eFuse or a secure element"),
    ("No supply is ever cut: a reset at the exact write boundaries of an update stands in for it (REL-11); a torn single flash write and the supply ramp are not run",
     "Medium", "Fit the relay and 5 V supply (docs/HIL_POWER_CUT.md) and cut the supply inside REL-04 / REL-07 / REL-11"),
    ("The health check is minimal (CAN controller, settings store, heap)", "Medium", "Run the controller's self-test suite in the trial boot"),
    ("Everything runs on one bench: 1 real board and emulated vECUs, synthetic traffic", "Medium", "Repeat on a second board and on a vehicle log"),
]


@dataclass
class Decision:
    verdict: str
    blocking: list[str] = field(default_factory=list)
    risks: list[str] = field(default_factory=list)
    evidence: dict = field(default_factory=dict)


LIMIT_USB = ("The OTA transport is the bench's USB link; MQTT delivery of the image is not exercised", "Medium",
             "python -m fleet.ota_matrix --emulated --mqtt (and --real COM13 --mqtt), then rerun the release")
LIMIT_MQTT = ("The update reaches the real board through a broker and a PC gateway on its USB link; the board has no Wi-Fi radio or MQTT client of its own", "Medium",
              "Add Wi-Fi + an MQTT client to SafetyNode (the network credentials stay out of the repo) and point the gateway's topics at it")


LIMIT_WIFI = ("The board's own Wi-Fi link is plain MQTT (no TLS) with one login shared by every board, on a LAN you trust: the signature protects the "
              "image, not the link, so an attacker on that network can stop an update, not forge one", "Medium",
              "Add TLS with the broker's certificate pinned in the board, and one login per board")


def limits(ev: dict) -> list[tuple[str, str, str]]:
    first = LIMIT_WIFI if ev.get("ota_wifi_real") else LIMIT_MQTT if ev.get("ota_mqtt_real") else LIMIT_USB
    return [first] + LIMITS


def _load(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def gather(reports: Path = REPORTS) -> dict:
    files = {"preflight": reports / "health" / "preflight_latest.json", "matrix_reference": reports / "security" / "attack_matrix_reference.json",
             "matrix_hil": reports / "security" / "attack_matrix_hil.json", "ota_emulated": reports / "fleet" / "ota_matrix_emulated.json",
             "ota_real": reports / "fleet" / "ota_matrix_real.json", "rollout_good": reports / "fleet" / "rollout_good.json",
             "rollout_bad": reports / "fleet" / "rollout_bad.json",
             "ota_mqtt_emulated": reports / "fleet" / "ota_matrix_emulated_mqtt.json", "ota_mqtt_real": reports / "fleet" / "ota_matrix_real_mqtt.json",
             "ota_cut_emulated": reports / "fleet" / "ota_cut_emulated.json", "ota_cut_real": reports / "fleet" / "ota_cut_real.json",
             "ota_wifi_emulated": reports / "fleet" / "ota_matrix_emulated_wifi.json", "ota_wifi_real": reports / "fleet" / "ota_matrix_real_wifi.json"}
    ev = {k: _load(p) for k, p in files.items()}
    ev["_files"] = {k: p for k, p in files.items() if p.exists()}
    ev["dashboard"] = dashboard.build(reports / "design", reports / "health")
    return ev


def decide(ev: dict) -> Decision:
    d = Decision("GO", evidence={"files": {k: hashlib.sha256(p.read_bytes()).hexdigest()[:12] for k, p in (ev.get("_files") or {}).items()}})
    blocked, no_go, risks = [], d.blocking, d.risks

    pf = ev.get("preflight")
    if not pf:
        blocked.append("no preflight on record (python -m health.preflight)")
    elif not pf["ok"]:
        blocked.append("the bench failed its last preflight: " + "; ".join(c["detail"] for c in pf["checks"] if c["status"] == "fail"))
    for k in ("ota_emulated", "ota_real", "matrix_reference", "matrix_hil", "rollout_good", "rollout_bad"):
        if not ev.get(k):
            blocked.append(f"missing evidence: {k}")

    prod = [x for x in ev["dashboard"]["explained"] if x["box"] == "PRODUCT" and not x["detail"].startswith("seeded defect")]
    no_go += [f"product failure {x['case']} ({x['level']}, campaign {x['campaign']}): {x['detail']}" for x in prod]
    for key in ("matrix_reference", "matrix_hil"):
        for r in ev.get(key) or []:
            if not r["ok"]:
                no_go.append(f"attack scenario {r['id']} {r['key']} ({key.split('_')[1]}) fails: {'; '.join(r['failed_checks'] + r['ids_problems'])}")
            elif r.get("open_finding"):
                risks.append(f"open finding {r['id']} {r['key']}: {'; '.join(r['ids_problems']) or 'tracked in known_findings'}")
            elif r.get("residual_risk"):
                risks.append(f"residual risk {r['id']} {r['key']}: valid, fresh, in-envelope forged commands are invisible to E2E and the intrusion detector")
            elif r["safety_status"] == "KNOWN":
                risks.append(f"known finding {r['id']} {r['key']}: stop later than its FTTI")
    for key in ("ota_emulated", "ota_real", "ota_mqtt_emulated", "ota_mqtt_real", "ota_cut_emulated", "ota_cut_real",
                "ota_wifi_emulated", "ota_wifi_real"):
        for r in ev.get(key) or []:
            if not r["ok"]:
                no_go.append(f"OTA scenario {r['id']} ({key[4:].replace('_', ' ')}) fails: {r['title']}")
    good, bad = ev.get("rollout_good"), ev.get("rollout_bad")
    if good and (good["halted"] or good["rolled_back"] or good["failed"] or good["rejected"]):
        no_go.append(f"the good build did not roll out cleanly: {good['committed']} committed, {good['rolled_back']} rolled back, halted={good['halted']}")
    if bad and not bad["halted"]:
        no_go.append("a bad build was NOT halted by the rollout's halt rule")
    for x in ev["dashboard"]["explained"]:
        if x["box"] in ("TEST", "BENCH", "GAP"):
            risks.append(f"{x['box']}: {x['case']} ({x['level']}) {x['verdict']}: {x['detail'] or x['meaning']}")

    # The same open finding is seen by both attack matrices (reference and the two boards); it is one risk, so list it once.
    risks[:] = list(dict.fromkeys(risks))
    d.verdict = "BLOCKED" if blocked else "NO GO" if no_go else "GO WITH RISKS" if risks else "GO"
    d.blocking = blocked + no_go
    return d


def render(d: Decision, ev: dict) -> str:
    fw = (ev.get("preflight") or {}).get("firmware") or {}
    lines = [f"# Release decision: {d.verdict}", "", f"Generated {time.strftime('%Y-%m-%d %H:%M')} from the bench's own evidence. Firmware on the bench: "
             + ", ".join(f"{k} {v}" for k, v in fw.items() if v) + ".", ""]
    if d.blocking:
        lines += ["## Why not a GO", ""] + [f"- {b}" for b in d.blocking] + [""]
    lines += ["## Evidence", "", "| Area | Result |", "|---|---|"]
    pf = ev.get("preflight")
    lines.append(f"| Bench health (preflight {pf['level'] if pf else '-'}) | {'READY' if pf and pf['ok'] else 'NOT READY'} |")
    for k, label in (("matrix_reference", "Attack matrix, reference"), ("matrix_hil", "Attack matrix, two boards")):
        rows = ev.get(k) or []
        lines.append(f"| {label} | {sum(r['ok'] for r in rows)}/{len(rows)} meet their expectation |")
    for k, label in (("ota_emulated", "OTA scenarios, emulated board"), ("ota_real", "OTA scenarios, real board"),
                     ("ota_mqtt_emulated", "OTA scenarios, emulated board over MQTT"), ("ota_mqtt_real", "OTA scenarios, real board over MQTT"),
                     ("ota_cut_emulated", "Power-cut windows (REL-11/12), emulated board"), ("ota_cut_real", "Power-cut windows (REL-11/12), real board"),
                     ("ota_wifi_emulated", "OTA scenarios, stand-in board on the network"), ("ota_wifi_real", "OTA scenarios, real board over its own Wi-Fi")):
        rows = ev.get(k)
        if k not in ("ota_emulated", "ota_real") and not rows:
            continue
        rows = rows or []
        lines.append(f"| {label} | {sum(r['ok'] for r in rows)}/{len(rows)} pass |")
    for k, label in (("rollout_good", "Rollout of the good build"), ("rollout_bad", "Rollout of a bad build")):
        r = ev.get(k)
        if r:
            lines.append(f"| {label} | {r['committed']} committed, {r['rolled_back']} rolled back, {r['untouched']} untouched; "
                         f"{'HALTED: ' + r['halt_reason'] if r['halted'] else 'ran to the end'} |")
    cov = ev["dashboard"]["coverage"]
    lines.append(f"| Design campaigns | {sum(v['total'] for lv in cov.values() for v in lv.values())} cases over {len(cov)} requirements (latest per level) |")
    lines += ["", "## Risks you accept with a GO WITH RISKS", ""] + ([f"- {r}" for r in d.risks] or ["- none open"])
    lines += ["", "## Standing limits of this evidence (true for every release from this bench)", ""] + [f"- {a} [{sev}]. Next step: {nxt}" for a, sev, nxt in limits(ev)]
    lines += ["", "## Input files (sha256, first 12)", ""] + [f"- {k}: `{v}`" for k, v in d.evidence["files"].items()]
    return "\n".join(lines) + "\n"


def main() -> int:
    import sys
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ev = gather()
    d = decide(ev)
    OUT.mkdir(parents=True, exist_ok=True)
    text = render(d, ev)
    (OUT / "release_report.md").write_text(text, encoding="utf-8")
    (OUT / "release_report.json").write_text(json.dumps({"verdict": d.verdict, "blocking": d.blocking, "risks": d.risks, "evidence": d.evidence}, indent=1), encoding="utf-8")
    print(text)
    return 0 if d.verdict in ("GO", "GO WITH RISKS") else 1


if __name__ == "__main__":
    raise SystemExit(main())
