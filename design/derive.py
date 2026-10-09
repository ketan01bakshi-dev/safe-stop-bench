"""ISTQB test cases from an approved test basis (code, not a model): the HIL ML Ops fault-injection rules on safe-stop faults.

For a timed-loss requirement ("<trigger> for > N ms → <reaction>", FTTI F, frame period P):

| Case      | Technique                 | What it checks                                                                    |
|-----------|---------------------------|-----------------------------------------------------------------------------------|
| FI-01..03 | fault injection + EP      | permanent loss at a low, mid and near-ODD-limit start speed: reaction, cause,     |
|           |                           | stop, latch, detection ≤ F and NOT before N − P (one period early)                |
| FI-04     | fault injection           | the same, injected mid-period (phase), the worst case for the FTTI margin         |
| BVA-01    | 3-value BVA, first value   | a loss just past the threshold that then ends: must still react and latch         |
|           | that triggers             |                                                                                   |
| BVA-02/03 | 3-value BVA, values that  | blocked GAP: no requirement says what may happen there. Run as probes:            |
|           | do not trigger            | the observation goes to the open question, never into a verdict                   |
"""
from __future__ import annotations

import re


def speeds(cfg: dict) -> list[tuple[str, float]]:
    odd = cfg["safety"]["odd_max_kmh"]
    pts = [("low", 10.0), ("mid", round(odd * 0.75)), ("near the ODD limit", odd - 1)]
    out: list[tuple[str, float]] = []
    for name, v in pts:
        if v > 0 and all(v != x for _, x in out):
            out.append((name, float(v)))
    return out


def _sc(case_id: str, key: str, title: str, req: list[str], faults: list[dict], expect: dict, **kw) -> dict:
    return {"id": case_id, "key": key, "title": title, "req": req, "faults": faults, "expect": expect, **kw}


def derive(b: dict, cfg: dict) -> list[dict]:
    if b.get("kind") == "event":
        return derive_event(b, cfg)
    if b.get("kind") != "timed_loss":
        return []
    rid, n, op, p = b["req"], b["threshold"]["value_ms"], b["threshold"]["op"], b["period_ms"]
    tag = rid.lower().replace("-", "")
    req = [rid] + ([b["latched_by"]] if b.get("latched_by") else [])
    expect = {"reaction": b["reaction"], "cause": b["cause"], "min_detect_ms": n - p}
    if b.get("stop"):
        expect["stop"] = True
    if b.get("latched_by"):
        expect["latched"] = True
    cases = []

    def add(cid, technique, item, sc=None, blocked=None, probe=False):
        cases.append({"id": f"{rid}-{cid}", "req": rid, "technique": technique, "coverage_item": item,
                      "blocked": blocked, "probe": probe, "scenario": sc})

    for i, (name, v) in enumerate(speeds(cfg), 1):
        add(f"FI-{i:02d}", "FI + EP (start speed)", f"permanent {b['message']} loss at {v:g} km/h ({name})",
            _sc(f"{rid}-FI-{i:02d}", f"{tag}_fi{i:02d}_loss_{v:g}kmh", f"{rid}: {b['message']} lost for good at {v:g} km/h",
                req, [{"type": b["fault"], "start": 2000}], dict(expect), start_kmh=v))
    mid = speeds(cfg)[min(1, len(speeds(cfg)) - 1)][1]
    k = len(speeds(cfg)) + 1
    add(f"FI-{k:02d}", "FI (injection phase)", f"loss starting mid-period (+{p // 2} ms), the worst case for the FTTI margin",
        _sc(f"{rid}-FI-{k:02d}", f"{tag}_fi{k:02d}_loss_midperiod", f"{rid}: {b['message']} lost from mid-period at {mid:g} km/h",
            req, [{"type": b["fault"], "start": 2000 + p // 2}], dict(expect), start_kmh=mid, inject_ms=2000 + p // 2))
    first = n + 1 if op == ">" else n                       # first loss duration the requirement makes react
    after = dict(expect, cause=b["cause_after_resume"])
    add("BVA-01", "BVA3 (loss duration)", f"loss of {first} ms, then the link returns: first value that triggers",
        _sc(f"{rid}-BVA-01", f"{tag}_bva01_loss_{first}ms", f"{rid}: {b['message']} lost for {first} ms, then back",
            req, [{"type": b["fault"], "start": 2000, b["duration_param"]: 2000 + first}], after))
    for j, d in enumerate((first - 1, first - 2), 2):
        add(f"BVA-{j:02d}", "BVA3 (loss duration)", f"loss of {d} ms ({rid} does not cover it)",
            _sc(f"{rid}-BVA-{j:02d}", f"{tag}_bva{j:02d}_loss_{d}ms_probe", f"{rid} probe: {b['message']} lost for {d} ms, then back",
                [rid], [{"type": b["fault"], "start": 2000, b["duration_param"]: 2000 + d}], {}, probe=True),
            blocked=f"GAP: no requirement says what may happen after a {d} ms loss ({rid} only covers {op} {n} ms)", probe=True)
    return cases


def derive_event(b: dict, cfg: dict) -> list[dict]:
    """Event requirement ("<alternatives> → <stop>", drafted by the LLM, validated by code):

    | FI-nn   | one case per alternative (trigger) at the mid start speed: reaction, cause, stop, latch, detection ≤ FTTI |
    | EP-nn   | the first trigger again at the low and near-ODD-limit speeds                                             |
    | BVA-nn  | a trigger with a limit on a fault parameter: first value that triggers (executable) + the two below      |
    |         | (blocked GAP, run as probes)                                                                             |
    | GAP-nn  | an alternative the bench cannot inject: blocked, no scenario                                             |
    """
    rid = b["req"]
    tag = rid.lower().replace("-", "")
    req = [rid] + ([b["latched_by"]] if b.get("latched_by") else [])
    sp = speeds(cfg)
    mid = sp[min(1, len(sp) - 1)][1]
    cases: list[dict] = []

    def add(cid, technique, item, sc=None, blocked=None, probe=False):
        cases.append({"id": f"{rid}-{cid}", "req": rid, "technique": technique, "coverage_item": item,
                      "blocked": blocked, "probe": probe, "scenario": sc})

    def expect(t):
        return {"reaction": b["reaction"], "cause": t["cause"], "stop": True, **({"latched": True} if b.get("latched_by") else {})}

    def fault(t, **over):
        return {"type": t["fault"], "start": t.get("bench_timing", {}).get("start", 2000), **(t.get("params") or {}), **over}

    def timing(t):   # inherited from the bench scenario the trigger equals (see basis.finish_event)
        return {k: v for k, v in t.get("bench_timing", {}).items() if k in ("no_ftti", "inject_ms", "duration_ms")}

    for i, t in enumerate(b["triggers"], 1):
        slug = re.sub(r"[^a-z0-9]+", "_", t["covers"].lower()).strip("_")[:24]
        add(f"FI-{i:02d}", "FI (alternative)", f"\"{t['covers']}\" via {t['fault']} at {mid:g} km/h",
            _sc(f"{rid}-FI-{i:02d}", f"{tag}_fi{i:02d}_{slug}", f"{rid}: {t['covers']} ({t['fault']}) at {mid:g} km/h",
                req, [fault(t)], expect(t), start_kmh=mid, **timing(t)))
    # EP over the fault PATTERN: other bench scenarios of the same fault that also expect this stop (SC-05: every other frame
    # corrupted instead of all). A permanent fault alone never tests a "window": the e2e_run_length bug (3 bad IN A ROW)
    # passed every permanent-fault case of the first drafted SR-02 design
    from .drafter import fault_catalogue
    fcat = fault_catalogue()
    pv = 0
    for t in b["triggers"]:
        for ex in fcat.get(t["fault"], {}).get("examples", []):
            params = {k: x for k, x in ex["fault"].items() if k not in ("type", "end")}
            if params == (t.get("params") or {}) or ex["reaction"] != b["reaction"]:
                continue
            pv += 1
            tv = {**t, "params": params, "bench_timing": {**ex["timing"], "from": ex["id"]}}
            add(f"PV-{pv:02d}", "EP (fault pattern)", f"\"{t['covers']}\" as in bench scenario {ex['id']}: {ex['title']}",
                _sc(f"{rid}-PV-{pv:02d}", f"{tag}_pv{pv:02d}_{t['fault']}", f"{rid}: {t['covers']}, pattern of {ex['id']}",
                    req, [fault(tv)], expect(tv), start_kmh=mid, **timing(tv)))
    first = b["triggers"][0]
    for j, (name, v) in enumerate([x for x in sp if x[1] != mid], 1):
        add(f"EP-{j:02d}", "EP (start speed)", f"\"{first['covers']}\" at {v:g} km/h ({name})",
            _sc(f"{rid}-EP-{j:02d}", f"{tag}_ep{j:02d}_{v:g}kmh", f"{rid}: {first['covers']} at {v:g} km/h",
                req, [fault(first)], expect(first), start_kmh=v, **timing(first)))
    n = 0
    for t in b["triggers"]:
        th = t.get("threshold")
        if not th:
            continue
        lo = th["value"] + 1 if th["op"] == ">" else th["value"]
        for k, val in enumerate((lo, lo - 1, lo - 2)):
            n += 1
            probe = k > 0
            add(f"BVA-{n:02d}", "BVA3 (fault parameter)", f"{th['param']} = {val:g} ({'first value that triggers' if not probe else rid + ' does not cover it'})",
                _sc(f"{rid}-BVA-{n:02d}", f"{tag}_bva{n:02d}_{th['param']}_{val:g}" + ("_probe" if probe else ""),
                    f"{rid}{' probe' if probe else ''}: {t['fault']} with {th['param']} = {val:g}", req if not probe else [rid],
                    [fault(t, **{th["param"]: val})], {} if probe else expect(t), **timing(t)),
                blocked=f"GAP: no requirement says what may happen at {th['param']} = {val:g}" if probe else None, probe=probe)
    for g, x in enumerate(b.get("not_injectable") or [], 1):
        add(f"GAP-{g:02d}", "FI (alternative)", f"\"{x['covers']}\"", None, blocked=f"GAP: not injectable on this bench ({x['reason']})")
    return cases


def coverage(cases: list[dict]) -> dict:
    out: dict[str, dict] = {}
    for c in cases:
        t = c["technique"].split(" ")[0]
        row = out.setdefault(t, {"items": 0, "executable": 0, "blocked": 0})
        row["items"] += 1
        row["blocked" if c["blocked"] else "executable"] += 1
    return out


def scenario_file(cases: list[dict]) -> dict:
    return {"_note": "Generated by python -m design from an approved test basis. Do not edit: change the basis and re-derive.",
            "scenarios": [c["scenario"] for c in cases if c["scenario"]]}


def to_md(cases: list[dict]) -> str:
    lines = ["## Test cases", "", "| Case | Technique | Coverage item | Expected | Status |", "|---|---|---|---|---|"]
    for c in cases:
        e = (c["scenario"] or {}).get("expect", {})
        exp = ("–" if c["probe"] or not c["scenario"] else
               f"{e.get('reaction')} ({'/'.join(e.get('cause', []))}), detect "
               + (f"{e['min_detect_ms']} ms … FTTI" if "min_detect_ms" in e else "within FTTI")
               + (", stop" if e.get("stop") else "") + (", latched" if e.get("latched") else ""))
        lines.append(f"| {c['id']} | {c['technique']} | {c['coverage_item']} | {exp} | "
                     f"{('blocked, runs as probe: ' if c['probe'] else 'blocked: ') + c['blocked'] if c['blocked'] else 'executable'} |")
    lines += ["", "| Technique | Items | Executable | Blocked (GAP) |", "|---|---|---|---|"]
    lines += [f"| {t} | {r['items']} | {r['executable']} | {r['blocked']} |" for t, r in coverage(cases).items()]
    return "\n".join(lines) + "\n"
