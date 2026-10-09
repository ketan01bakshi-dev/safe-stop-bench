"""The attack x detection matrix (P3): for every attack, did E2E reject it, did the intrusion detector alert (and when), did the
safety controller reach a safe stop (and when), and which of the layers reacted first?

    .venv\\Scripts\\python.exe -m security.matrix                       reference controller (fast, CI)
    .venv\\Scripts\\python.exe -m security.matrix --dut hil --port-b COM13 --port-a COM14 --kick gpio     the two boards

It runs scenarios/security.json through run.py (so it works at every level), reads the results and writes
reports/security/attack_matrix.md / .json. The 'sec' block of each scenario says what the intrusion detector is expected to do.
Exit code 1 if any scenario fails its safety expectation or its intrusion-detector expectation.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SCEN = ROOT / "scenarios" / "security.json"
OUT = ROOT / "reports" / "security"


def run_scenarios(dut: str, extra: list[str], out: Path, label: str) -> list[dict]:
    cmd = [sys.executable, str(ROOT / "run.py"), "--scenario-file", str(SCEN), "--out", str(out), "--label", label, "--dut", dut, *extra]
    proc = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, encoding="utf-8", errors="replace")
    f = out / f"{label}_results.json"
    if not f.exists():
        raise SystemExit(f"run.py did not finish (exit {proc.returncode}):\n{(proc.stdout + proc.stderr)[-1500:]}")
    return json.loads(f.read_text(encoding="utf-8"))["results"]


def attack_of(sc: dict) -> dict | None:
    return next((x for x in sc.get("faults", []) if x["type"] == "attack"), None)


def row(sc: dict, r: dict, e2e_visible: bool = True) -> dict:
    a, v, sec = attack_of(sc), r["verdict"], sc.get("sec", {})
    start = a["start"] if a else None
    ids = (r.get("security") or {}).get("ids") or {}
    ids_ms = ids.get("first_ms")
    stop_ms = r.get("t_fault")
    stopped = bool(r["states"]) and any(s not in ("NORMAL", "INIT") for _, s in r["states"])
    out = {"id": sc["id"], "key": sc["key"], "title": sc["title"], "attack": a["kind"] if a else "none (control)", "start_ms": start,
           "e2e_rejected": r.get("rejected_frames", 0) if e2e_visible else None,   # a black-box DUT does not report its E2E rejects
            "ids_alerted": bool(ids.get("alerted")), "ids_by": ids.get("first_by"),
           "ids_after_ms": None if not ids.get("alerted") or start is None else ids_ms - start,
           "stopped": stopped, "reaction": v["peak_state"], "cause": r.get("cause"), "stop_after_ms": None if not stopped or start is None or stop_ms is None else stop_ms - start,
           "invariants": r.get("invariant_count", 0), "safety_status": v["status"], "failed_checks": [n for n, ok in v["checks"] if not ok]}
    if start is None:
        out["first"] = "-"
        out["false_alarm"] = bool(ids.get("alerted")) or stopped
    elif out["ids_alerted"] and (not stopped or out["ids_after_ms"] <= out["stop_after_ms"]):
        out["first"] = "IDS"
    elif stopped:
        out["first"] = "safety" if not out["ids_alerted"] else "safety (IDS later)"
    else:
        out["first"] = "nobody"
    # the intrusion detector's own expectation
    want = sec.get("ids_within_ms", "unset")
    problems = []
    if start is None:
        if sec.get("ids_alert") is False and out["false_alarm"]:
            problems.append("false alarm / false stop on clean traffic")
    elif want is None:
        out["residual_risk"] = bool(sec.get("residual_risk"))
        if out["ids_alerted"] and sec.get("residual_risk"):
            problems.append("expected to be undetectable but the IDS alerted: update the residual-risk note")
    elif want != "unset" and (not out["ids_alerted"] or out["ids_after_ms"] > want):
        problems.append(f"IDS expected within {want} ms, got {'no alert' if not out['ids_alerted'] else str(out['ids_after_ms']) + ' ms'}")
    out["ids_problems"] = problems
    out["open_finding"] = bool(v.get("known")) and bool(problems or v["status"] == "KNOWN")   # a tracked gap does not fail CI
    out["ok"] = v["status"] in ("PASS", "KNOWN") and (not problems or bool(v.get("known")))
    return out


def render(rows: list[dict], dut: str) -> str:
    def c(x, unit=" ms"):
        return "-" if x is None else f"{x}{unit}"
    lines = [f"# Attack x detection matrix ({dut})", "",
             "E2E rejected = frames the safety controller's E2E check discarded. IDS = the intrusion detector's first alert after the attack began.",
             "Safe stop = the controller left NORMAL. First = which layer reacted first.", "",
             "| Scenario | Attack | E2E rejected | IDS alert | Safe stop | First | Reaction / cause | Safety | IDS vs expectation |", "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        ids = "-" if not r["ids_alerted"] else f"{c(r['ids_after_ms'])} ({r['ids_by']})"
        stop = "no" if not r["stopped"] else c(r["stop_after_ms"])
        verdict = "; ".join(r["ids_problems"]) or ("residual risk (documented)" if r.get("residual_risk") else "as expected")
        if r.get("open_finding") and r["ids_problems"]:
            verdict += " (OPEN FINDING)"
        lines.append(f"| {r['id']} {r['key']} | {r['attack']} | {'n/a (black box)' if r['e2e_rejected'] is None else r['e2e_rejected']} | {ids} | {stop} | {r['first']} | {r['reaction']} / {r['cause'] or '-'} | "
                     f"{r['safety_status']}{'' if not r['failed_checks'] else ' (' + '; '.join(r['failed_checks']) + ')'} | {verdict} |")
    lines += ["", f"{sum(r['ok'] for r in rows)}/{len(rows)} scenarios meet both their safety and their intrusion-detection expectation."]
    return "\n".join(lines) + "\n"


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")   # Windows consoles default to cp1252
    ap = argparse.ArgumentParser(description="attack x detection matrix")
    ap.add_argument("--dut", default="reference")
    ap.add_argument("--label", default=None)
    ap.add_argument("extra", nargs="*", help="extra run.py arguments (e.g. --port-b COM13 --port-a COM14 --kick gpio)")
    a = ap.parse_args()
    label = a.label or f"matrix_{a.dut}"
    OUT.mkdir(parents=True, exist_ok=True)
    scenarios = {s["key"]: s for s in json.loads(SCEN.read_text(encoding="utf-8"))["scenarios"]}
    results = run_scenarios(a.dut, a.extra, OUT, label)
    rows = [row(scenarios[r["key"]], r, e2e_visible=a.dut == "reference") for r in results]
    (OUT / f"attack_matrix_{a.dut}.json").write_text(json.dumps(rows, indent=1), encoding="utf-8")
    text = render(rows, a.dut)
    (OUT / f"attack_matrix_{a.dut}.md").write_text(text, encoding="utf-8")
    print(text)
    return 0 if all(r["ok"] for r in rows) else 1


if __name__ == "__main__":
    sys.exit(main())
