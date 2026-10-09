"""Bench health dashboard (P2): one static page over the report files (nothing is re-run, nothing is sent anywhere).

    python -m health.dashboard            -> reports/health/dashboard.html + dashboard.json

It answers three questions:
  1. Is the bench fit to judge right now?  (latest preflight: ports, hello, UDS versions, compile / partition audit)
  2. What is covered?  (per requirement: cases by verdict, at which levels, in which campaign)
  3. Why is every non-PASS not-PASS?  Each one is put in exactly one box:
       PRODUCT  a FAIL the known-good reference passes, on a healthy bench: a defect candidate
       TEST     SUSPECT: the case also fails on the known-good controller, so the case or its basis is wrong
       BENCH    SUSPECT BENCH: the firmware is not the inventory's, or the scenario needed bench-fault reruns
       GAP      BLOCKED / NOT RUN: the bench cannot inject it, or it was not executed
  plus failure clusters: the same requirement failing the same checks again and again.
"""
from __future__ import annotations

import html
import json
from collections import defaultdict
from pathlib import Path

from ssb.config import ROOT

REPORTS = ROOT / "reports" / "design"
HEALTH = ROOT / "reports" / "health"
BOX = {"FAIL": "PRODUCT", "SUSPECT": "TEST", "SUSPECT BENCH": "BENCH", "BLOCKED": "GAP", "NOT RUN": "GAP"}
WHY = {"PRODUCT": "fails on this bench while the known-good reference passes: a defect candidate",
       "TEST": "also fails on the known-good reference controller: the case or its basis is wrong, not the product",
       "BENCH": "the bench is in doubt (firmware not the inventory's, or bench-fault reruns), so the verdict is not read as a product failure",
       "GAP": "the bench cannot inject this fault, or the case was not executed"}


def _json(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def load_campaigns(reports: Path = REPORTS) -> list[dict]:
    out = []
    for d in sorted(p for p in reports.iterdir() if p.is_dir()) if reports.exists() else []:
        cases, res = _json(d / "02_design" / "cases.json"), _json(d / "04_triage" / "case_results.json")
        if not cases or not res:
            continue
        man = _json(d / "03_execute" / "run_manifest.json") or {}
        run = man.get("run") or {}
        out.append({"name": d.name, "mtime": (d / "04_triage" / "case_results.json").stat().st_mtime, "level": run.get("level", "?"),
                    "defects": run.get("defects") or [], "firmware": man.get("firmware"), "preflight": run.get("preflight"), "cases": cases, "results": res})
    return out


def build(reports: Path = REPORTS, health: Path = HEALTH) -> dict:
    campaigns = load_campaigns(reports)
    latest: dict[tuple[str, str], dict] = {}   # (requirement, level) -> the most recent campaign that covered it
    for c in sorted(campaigns, key=lambda c: c["mtime"]):
        for case in c["cases"]:
            latest[(case["req"], c["level"])] = c
    coverage: dict[str, dict] = defaultdict(dict)
    explained, clusters = [], defaultdict(list)
    for (req, level), c in sorted(latest.items()):
        counts: dict[str, int] = defaultdict(int)
        for case in c["cases"]:
            if case["req"] != req:
                continue
            r = c["results"].get(case["id"], {"verdict": "NOT RUN"})
            counts[r["verdict"]] += 1
            if r["verdict"] in BOX:
                box = BOX[r["verdict"]]
                why = r.get("why") or ("; ".join(r.get("failed") or []) if r["verdict"] == "FAIL" else "")
                if c["defects"] and r["verdict"] == "FAIL":
                    why = f"seeded defect ({', '.join(c['defects'])}): the bench was asked to catch it. " + why
                explained.append({"req": req, "level": level, "campaign": c["name"], "case": case["id"], "coverage_item": case["coverage_item"],
                                  "verdict": r["verdict"], "box": box, "meaning": WHY[box], "detail": why})
                if r["verdict"] == "FAIL":
                    clusters[(req, level, tuple(r.get("failed") or []), r.get("cause"))].append(case["id"])
        coverage[req][level] = {"campaign": c["name"], "counts": dict(counts), "total": sum(counts.values()),
                                "firmware": c["firmware"], "preflight": c["preflight"]}
    return {"preflight": _json(health / "preflight_latest.json"), "coverage": coverage, "explained": explained,
            "clusters": [{"req": k[0], "level": k[1], "checks": list(k[2]), "cause": k[3], "cases": v} for k, v in clusters.items()],
            "campaigns": [{"name": c["name"], "level": c["level"], "preflight": c["preflight"], "firmware": c["firmware"],
                           "cases": len(c["cases"])} for c in campaigns]}


CSS = """:root{--bg:#fff;--fg:#1d2330;--mut:#667;--line:#dde1e8;--ok:#1b7f3b;--warn:#a86a00;--bad:#b3261e;--card:#f6f8fb}
@media(prefers-color-scheme:dark){:root{--bg:#14171d;--fg:#e6e9ef;--mut:#99a;--line:#2b303b;--ok:#5fd07f;--warn:#e3a53b;--bad:#ff7b72;--card:#1c2029}}
body{font:14px/1.5 system-ui,Segoe UI,sans-serif;background:var(--bg);color:var(--fg);margin:0;padding:24px 16px}main{max-width:1000px;margin:auto}
h1{font-size:22px;margin:0 0 4px}h2{font-size:16px;margin:28px 0 8px}p.s{color:var(--mut);margin:0 0 16px}
table{border-collapse:collapse;width:100%;background:var(--card)}th,td{border:1px solid var(--line);padding:5px 8px;text-align:left;vertical-align:top}th{font-weight:600}
.ok{color:var(--ok)}.warn{color:var(--warn)}.fail{color:var(--bad)}.skip{color:var(--mut)}.box{font-weight:600}
.scroll{overflow-x:auto}"""


def _e(x) -> str:
    return html.escape(str(x))


def render_html(d: dict) -> str:
    p = d["preflight"]
    h = ["<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>",
         f"<title>Bench health</title><style>{CSS}</style><main><h1>Bench health</h1>",
         "<p class=s>Is the bench fit to judge, what is covered, and why every non-PASS is not a PASS.</p>"]
    h.append("<h2>1. Is the bench fit to judge?</h2>")
    if p:
        h.append(f"<p><b class={'ok' if p['ok'] else 'fail'}>{'READY' if p['ok'] else 'NOT READY'}</b> · preflight {p['level']} · {_e(p['time'])}</p><div class=scroll><table><tr><th>Check<th>Result<th>Detail")
        for c in p["checks"]:
            h.append(f"<tr><td>{_e(c['name'])}<td class={c['status']}>{c['status'].upper()}<td>{_e(c['detail'])}" + "".join(f"<br><i>fix: {_e(f)}</i>" for f in c["fixes"]))
        h.append("</table></div>")
    else:
        h.append("<p>No preflight on record. Run <code>python -m health.preflight</code>.</p>")
    h.append("<h2>2. Coverage per requirement</h2><div class=scroll><table><tr><th>Requirement<th>Level<th>Cases<th>Verdicts<th>Campaign")
    for req, levels in sorted(d["coverage"].items()):
        for level, v in sorted(levels.items()):
            h.append(f"<tr><td>{_e(req)}<td>{_e(level)}<td>{v['total']}<td>" + ", ".join(f"{_e(k)} {n}" for k, n in sorted(v["counts"].items())) + f"<td>{_e(v['campaign'])}")
    h.append("</table></div><h2>3. Every non-PASS, in one box</h2>")
    if d["explained"]:
        h.append("<div class=scroll><table><tr><th>Box<th>Case<th>Verdict<th>Why<th>Detail")
        for x in d["explained"]:
            h.append(f"<tr><td class=box>{x['box']}<td>{_e(x['case'])}<br>{_e(x['coverage_item'])}<td>{_e(x['verdict'])} ({_e(x['level'])})<td>{_e(x['meaning'])}<td>{_e(x['detail'])}")
        h.append("</table></div>")
    else:
        h.append("<p class=ok>Nothing non-PASS in the latest campaigns.</p>")
    h.append("<h2>4. Failure clusters</h2>")
    if d["clusters"]:
        h.append("<table><tr><th>Requirement<th>Level<th>Failed checks<th>Cause<th>Cases")
        for c in d["clusters"]:
            h.append(f"<tr><td>{_e(c['req'])}<td>{_e(c['level'])}<td>{_e('; '.join(c['checks']))}<td>{_e(c['cause'])}<td>{_e(', '.join(c['cases']))}")
        h.append("</table>")
    else:
        h.append("<p class=ok>No product failures.</p>")
    h.append("<h2>Campaigns</h2><div class=scroll><table><tr><th>Campaign<th>Level<th>Cases<th>Preflight<th>Firmware")
    for c in d["campaigns"]:
        pf = c["preflight"]
        h.append(f"<tr><td>{_e(c['name'])}<td>{_e(c['level'])}<td>{c['cases']}<td>{'skipped' if pf and pf.get('skipped') else ('ok' if pf and pf.get('ok') else '–')}<td>{_e(c['firmware'] or '–')}")
    h.append("</table></div></main>")
    return "".join(h)


def write(reports: Path = REPORTS, health: Path = HEALTH) -> Path:
    d = build(reports, health)
    health.mkdir(parents=True, exist_ok=True)
    (health / "dashboard.json").write_text(json.dumps(d, indent=1, default=str), encoding="utf-8")
    out = health / "dashboard.html"
    out.write_text(render_html(d), encoding="utf-8")
    return out


if __name__ == "__main__":
    print(write())
