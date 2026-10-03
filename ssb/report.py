"""Reports: HTML (one page), JSON, JUnit XML, CSV traces, diff against the previous run."""
from __future__ import annotations

import csv
import hashlib
import html
import json
from datetime import datetime
from pathlib import Path
from xml.sax.saxutils import escape

from .config import ROOT

VERSION = "2.0"


def bench_identity() -> str:
    h = hashlib.sha256()
    for f in sorted((ROOT / "ssb").glob("*.py")) + sorted((ROOT / "scenarios").glob("*.json")) + [ROOT / "safety" / "hazards.json"]:
        h.update(f.read_bytes())
    return f"safe-stop bench v{VERSION} sha256:{h.hexdigest()[:12]}"


def _f(v, unit=""):
    return "–" if v is None else f"{v}{unit}"


def _svg(trace, idx, vmax, label, w=330, h=120, fault=None):
    if not trace:
        return ""
    tmax = trace[-1][0] or 1
    X = lambda t: 30 + (w - 40) * t / tmax
    Y = lambda v: h - 18 - (h - 30) * max(-vmax, min(vmax, v)) / vmax if vmax else 0
    pts = " ".join(f"{X(r[0]):.1f},{Y(r[idx]):.1f}" for r in trace)
    m = f'<line x1="{X(fault):.1f}" y1="8" x2="{X(fault):.1f}" y2="{h-18}" class="inj"/>' if fault else ""
    return (f'<svg viewBox="0 0 {w} {h}" class="chart"><line x1="30" y1="{h-18}" x2="{w-8}" y2="{h-18}" class="ax"/>'
            f'<line x1="30" y1="8" x2="30" y2="{h-18}" class="ax"/>{m}<polyline points="{pts}" class="spd"/>'
            f'<text x="34" y="{h-4}" class="lbl">{label} · {tmax/1000:.0f} s</text></svg>')


CSS = """:root{--bg:#fff;--fg:#1d1d1f;--mut:#666;--line:#ddd;--ok:#137a3a;--bad:#b3261e;--acc:#1f5fbf;--card:#f6f7f9;--warn:#8a5a00}
@media (prefers-color-scheme:dark){:root{--bg:#16181c;--fg:#e8e8ea;--mut:#a0a3a8;--line:#33363c;--ok:#5fd28a;--bad:#ff8a80;--acc:#7fb0ff;--card:#1f2228;--warn:#ffcf70}}
body{background:var(--bg);color:var(--fg);font:14px/1.5 system-ui,Segoe UI,sans-serif;margin:auto;padding:24px 16px;max-width:1250px}
h1{margin:0 0 4px;font-size:24px}h2{margin-top:32px}.mut{color:var(--mut)}table{border-collapse:collapse;width:100%;margin:10px 0 18px}
th,td{border-bottom:1px solid var(--line);padding:6px 7px;text-align:left;vertical-align:top}th{font-size:12px;color:var(--mut)}
.pass{color:var(--ok);font-weight:700}.fail{color:var(--bad);font-weight:700}.warn{color:var(--warn);font-weight:700}
.banner{background:var(--card);border-radius:10px;padding:12px 16px;margin:14px 0}.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(300px,1fr));gap:12px}
figure{margin:0;background:var(--card);border-radius:8px;padding:8px}figcaption{font-size:12px;font-weight:600}.chart{width:100%;height:auto}
.ax{stroke:var(--mut);stroke-width:1}.spd{fill:none;stroke:var(--acc);stroke-width:2}.inj{stroke:var(--bad);stroke-dasharray:4 3}.lbl{fill:var(--mut);font-size:10px}
.wrap{overflow-x:auto}pre{background:var(--card);padding:12px;border-radius:8px;overflow:auto;font-size:12px}details{margin:4px 0}small{color:var(--mut)}"""


def write_all(out_dir: Path, results: list, extras: dict, label: str = "report") -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    prev_path = out_dir / f"{label}_results.json"
    prev = json.loads(prev_path.read_text(encoding="utf-8")) if prev_path.exists() else None
    slim = [{k: v for k, v in r.items() if k != "trace"} for r in results]
    meta = {"bench": bench_identity(), "generated": datetime.now().isoformat(timespec="seconds"), "dut": results[0]["dut"] if results else "-"}
    prev_path.write_text(json.dumps({"meta": meta, "results": slim, "extras": extras}, indent=1, default=str), encoding="utf-8")
    _junit(out_dir / f"{label}_junit.xml", results)
    traces = out_dir / "traces"
    traces.mkdir(exist_ok=True)
    for r in results:
        with (traces / f"{r['key'].replace('/', '_').replace('@', '_')}.csv").open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t_ms", "speed_kmh", "lateral_m", "state", "x_m"])
            w.writerows(r["trace"])
    page = out_dir / f"{label}.html"
    page.write_text(_html(results, extras, meta, prev), encoding="utf-8")
    return page


def _junit(path: Path, results: list) -> None:
    fails = sum(not r["verdict"]["passed"] for r in results)
    cases = []
    for r in results:
        body = "" if r["verdict"]["passed"] else f'<failure message="{escape(", ".join(c for c, ok in r["verdict"]["checks"] if not ok))}"/>'
        cases.append(f'<testcase classname="safe_stop_bench" name="{escape(r["key"])}">{body}</testcase>')
    path.write_text(f'<?xml version="1.0" encoding="UTF-8"?>\n<testsuite name="safe-stop-bench" tests="{len(results)}" failures="{fails}">'
                    + "".join(cases) + "</testsuite>\n", encoding="utf-8")


def _html(results, ex, meta, prev) -> str:
    n, ok = len(results), sum(r["verdict"]["passed"] for r in results)
    known = sum(r["verdict"].get("status") == "KNOWN" for r in results)
    rows = []
    for r in results:
        v = r["verdict"]
        failed = [c for c, good in v["checks"] if not good]
        verdict = {"PASS": '<span class="pass">PASS</span>', "KNOWN": '<span class="warn">KNOWN FINDING</span>'}.get(v.get("status"), '<span class="fail">FAIL</span>')
        note = ("<br><small>failed: " + html.escape(", ".join(failed)) + "</small>") if failed else ""
        if v.get("known"):
            note += f'<br><small class="warn">{html.escape(v["known"])}</small>' 
        finding = f'<br><span class="warn">FINDING</span> <small>{html.escape(r["finding"])}</small>' if r.get("finding") else ""
        tl = "".join(f"<li>{t} ms: {html.escape(txt)}</li>" for t, txt in r["events"][:12])
        rows.append(
            f"<tr><td><b>{html.escape(str(r['id']))}</b> {html.escape(r['key'])}<br><small>{html.escape(r['title'])}</small>{finding}"
            f"<details><summary><small>timeline</small></summary><ul>{tl or '<li>no events</li>'}</ul></details></td>"
            f"<td>{', '.join(r['req'])}</td><td>{html.escape(str(r['expect'].get('reaction', '–')))}</td>"
            f"<td>{html.escape(v['peak_state'])}<br><small>{html.escape(_f(r['cause']))}</small></td>"
            f"<td>{_f(v['t_detect_ms'], ' ms')}<br><small>FTTI {_f(v['ftti_ms'])}, margin {_f(v['margin_ms'])}</small></td>"
            f"<td>{_f(None if v['t_stop_ms'] is None else round(v['t_stop_ms']/1000, 2), ' s')}<br><small>{_f(v['stop_dist_m'], ' m')}</small></td>"
            f"<td>{r['max_lateral_m']} m<br><small>jerk {r['max_jerk']}</small></td><td>{verdict}{note}</td></tr>")

    # traceability
    by_req = {}
    for r in results:
        for q in r["req"]:
            by_req.setdefault(q, []).append(r)
    reqs = ex["requirements"]["requirements"]
    trace_rows = "".join(
        f"<tr><td><b>{q}</b> <small>({reqs[q]['goal']})</small></td><td>{html.escape(reqs[q]['text'])}</td><td>{_f(reqs[q].get('ftti_ms'), ' ms')}</td>"
        f"<td>{len(by_req.get(q, []))}</td><td>{'<span class=pass>all pass</span>' if by_req.get(q) and all(x['verdict']['passed'] for x in by_req[q]) else ('<span class=fail>gap</span>' if not by_req.get(q) else '<span class=fail>fail</span>')}</td></tr>"
        for q in reqs)
    goals = ex["requirements"]["goals"]
    cov_rows = "".join(f"<tr><td>{g}</td><td>{html.escape(t)}</td><td class=pass>SiL ✓</td><td class=mut>HiL –</td><td class=mut>vehicle –</td></tr>" for g, t in goals.items())

    charts = "".join(
        f'<figure><figcaption>{html.escape(r["key"])} · {"PASS" if r["verdict"]["passed"] else "FAIL"}</figcaption>'
        f'{_svg(r["trace"], 1, 45, "km/h")}{_svg(r["trace"], 2, 3.0, "lateral m (±3)")}</figure>'
        for r in results if r["key"] in ex.get("chart_keys", []))

    sw = ex.get("sweep", [])
    sweep_rows = "".join(f"<tr><td>{s['base']}</td><td>{s['kmh']}</td><td>{s['friction']}</td><td>{_f(s['t_detect_ms'], ' ms')}</td>"
                         f"<td>{_f(s['stop_dist_m'], ' m')}</td><td>{s['max_lateral_m']} m</td><td>{'<span class=pass>PASS</span>' if s['passed'] else '<span class=fail>FAIL</span><br><small>' + html.escape(', '.join(s['failed'])) + '</small>'}</td></tr>" for s in sw)
    fs = ex.get("false_stop")
    fz = ex.get("fuzz")
    mu = ex.get("mutation")
    mu_rows = "".join(f"<tr><td>{m['mutant']}</td><td>{html.escape(m['description'])}</td><td>{'<span class=pass>killed</span>' if m['killed'] else '<span class=fail>survived</span>'}</td><td>{_f(m['killed_by'])}</td></tr>" for m in (mu or {}).get("mutants", []))
    gp = ex.get("gaps", {})
    ftti_rows = "".join(f"<tr><td>{d['req']}</td><td>{d['speed_kmh']}</td><td>{html.escape(d['hazard'])}</td><td>{html.escape(d['assumption'])}</td><td>{html.escape(d['budget'])}</td></tr>" for d in ex.get("ftti", []))
    b2b = ex.get("b2b")
    b2b_html = ""
    if b2b:
        mism = [b for b in b2b["rows"] if not b["match"]]
        b2b_html = (f"<h2>Back-to-back: reference vs {html.escape(b2b['other'])}</h2><p>{len(b2b['rows']) - len(mism)}/{len(b2b['rows'])} scenarios match.</p>"
                    "<table><tr><th>Scenario</th><th>Differences</th></tr>" + "".join(f"<tr><td>{m['key']}</td><td>{html.escape('; '.join(m['diffs']))}</td></tr>" for m in mism) + "</table>")
    diff_html = ""
    if prev:
        before = {r["key"]: r["verdict"]["passed"] for r in prev["results"]}
        changes = [(r["key"], before.get(r["key"]), r["verdict"]["passed"]) for r in results if before.get(r["key"]) != r["verdict"]["passed"]]
        diff_html = (f"<p class=mut>Compared with the previous run ({html.escape(prev['meta']['generated'])}, {html.escape(prev['meta']['bench'])}): "
                     + (", ".join(f"{k}: {'new' if a is None else ('PASS' if a else 'FAIL')} → {'PASS' if b else 'FAIL'}" for k, a, b in changes) or "no verdict changes") + ".</p>")

    case = []
    for g, text in goals.items():
        qs = [q for q in reqs if reqs[q]["goal"] == g]
        ev = [r for q in qs for r in by_req.get(q, [])]
        good = ev and all(r["verdict"]["passed"] for r in ev)
        case.append(f"<li><b>Claim {g}:</b> {html.escape(text)}. <b>Argument:</b> requirements {', '.join(qs)} define the reaction and its time budget. "
                    f"<b>Evidence:</b> {len(ev)} SiL scenarios, {'all passing' if good else 'NOT all passing'}; mutation score {mu['score'] if mu else '–'}%. "
                    f"<b>Open:</b> no HiL or vehicle evidence yet; limits are illustrative.</li>")

    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Safe-stop bench v2 report</title><style>{CSS}</style></head><body>
<h1>Safe-stop bench v2: fault-injection report</h1>
<div class="mut">{html.escape(meta['bench'])} · DUT {html.escape(meta['dut'])} · {html.escape(meta['generated'])} · config: {html.escape(ex['config_name'])} · synthetic data, illustrative limits</div>
<div class="banner"><span class="{'pass' if ok + known == n else 'fail'}">{ok}/{n} scenarios passed</span>{f" · <span class=warn>{known} known finding(s)</span>" if known else ""}
{f" · mutation score <b>{mu['score']}%</b> ({mu['killed']}/{mu['total']} seeded bugs caught)" if mu else ""}
{f" · false stops <b>{fs['false_stops']}</b> in {fs['km']} km ({fs['per_100km']}/100 km at {fs['noise']*100:.1f}% CRC errors + {fs['jitter_ms']} ms jitter)" if fs else ""}
{f" · fuzz: <b>{fz['violating_runs']}</b>/{fz['runs']} runs with invariant violations" if fz else ""}</div>
{diff_html}
<pre>planner ──CAN FD, Profile 5 E2E, 20 ms──► SAFETY CONTROLLER (DUT) ──classic CAN, Profile 2 E2E, 10 ms──► actuator ECU (own fallback) ──► steering / brake / backup brake ──► bicycle-model vehicle
checks: E2E window state machine · timeout · freshness · window watchdog · Q&amp;A watchdog · speed-dependent envelope + jerk · commanded vs measured actuators
reactions: DEGRADED (speed cap) → PULL_OVER (planner-executed) → STOP_IN_LANE → BRAKE_ONLY_STOP / BACKUP_BRAKE_STOP · latched · release needs stop + clear 500 ms + operator</pre>
<h2>Results</h2><div class="wrap"><table><tr><th>Scenario</th><th>Req</th><th>Expected</th><th>Peak state / cause</th><th>Detect</th><th>Fault → stop</th><th>Lateral</th><th>Verdict</th></tr>{''.join(rows)}</table></div>
<h2>Traceability: requirement → scenarios</h2><table><tr><th>Req</th><th>Text</th><th>FTTI</th><th>Scenarios</th><th>Status</th></tr>{trace_rows}</table>
<h2>Coverage by layer</h2><table><tr><th>Goal</th><th>Text</th><th>SiL</th><th>HiL</th><th>Vehicle</th></tr>{cov_rows}</table>
<p class="mut">HiL and vehicle columns stay empty until the same scenarios run on hardware (see docs/CHANGES_V2.md, items 6.4–6.5).</p>
<h2>Speed and lateral traces</h2><div class="grid">{charts}</div>
{f"<h2>Sweep: speed × friction</h2><table><tr><th>Scenario</th><th>km/h</th><th>Friction</th><th>Detect</th><th>Stopping distance</th><th>Lateral</th><th>Verdict</th></tr>{sweep_rows}</table>" if sw else ""}
{f"<h2>False-stop rate</h2><p>{fs['false_stops']} false stops in {fs['seeds']} × 60 s runs ({fs['km']} km). Rate {fs['per_100km']} per 100 km" + (f", 95% upper bound {fs['upper95_per_100km']} per 100 km (rule of three)" if fs['upper95_per_100km'] is not None else "") + ". <span class=warn>FINDING</span> with 0.5% CRC errors (far above real CAN error rates), two corrupted frames in a row make the next frame's counter jump count as a third error, which trips the E2E window. Tune the window or the max delta against the real bus error rate.</p>" if fs else ""}
{f"<h2>Fuzzing</h2><p>{fz['runs']} random fault combinations; {fz['violating_runs']} with invariant violations or crashes.</p><pre>{html.escape(json.dumps(fz['examples'], indent=1)) if fz['examples'] else 'none'}</pre>" if fz else ""}
{f"<h2>Mutation testing (testing the tests)</h2><table><tr><th>Mutant</th><th>Seeded bug</th><th>Result</th><th>First scenario that caught it</th></tr>{mu_rows}</table>" if mu else ""}
{b2b_html}
{("<h2>Real-time timing (repeated runs)</h2><table><tr><th>Scenario</th><th>Runs passed</th><th>Detect min / typ / max</th><th>FTTI</th><th>Worst-case margin</th></tr>" + "".join(f"<tr><td>{x['key']}</td><td>{x['passed']}/{x['runs']}</td><td>{_f(x['min_ms'])} / {_f(x['typ_ms'])} / {_f(x['max_ms'])} ms</td><td>{_f(x['ftti_ms'], ' ms')}</td><td>{_f(x['worst_margin_ms'], ' ms')}</td></tr>" for x in ex['timing']) + "</table>") if ex.get("timing") else ""}
<h2>Gaps</h2><p>Requirements with no scenario: {html.escape(', '.join(gp.get('requirements_without_scenario', [])) or 'none')}.</p>
<table><tr><th>SOTIF item</th><th>Condition</th><th>Coverage</th><th>How / why not</th></tr>{''.join(f"<tr><td>{s['id']}</td><td>{html.escape(s['condition'])}</td><td>{s['bench_coverage']}</td><td>{html.escape(s['how'])}</td></tr>" for s in gp.get('sotif_items_out_of_scope', []))}</table>
<h2>FTTI derivation (illustrative)</h2><table><tr><th>Req</th><th>km/h</th><th>Hazard</th><th>Assumption</th><th>Budget</th></tr>{ftti_rows}</table>
<h2>Safety-case fragment (claim → argument → evidence)</h2><ul>{''.join(case)}</ul>
<p class="mut">Not a certified safety case. It shows the structure a driverless-vehicle safety argument needs (UL 4600 style), filled from this bench's evidence.</p>
</body></html>"""
