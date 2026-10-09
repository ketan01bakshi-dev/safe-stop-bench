"""Test basis per requirement: what triggers it, at what threshold, and what reaction the oracle must see.

The HIL ML Ops rule, kept here: the basis is the oracle, so it is drafted, validated by code and approved by a human
(gate `basis_review`) before any case is derived. Drafted by rules when the wording is regular
("<trigger> for > N ms → <reaction>"), or hand-written in design/basis/<req>.json; both go through `validate`.
"""
from __future__ import annotations

import json
import re
from pathlib import Path

from ssb.config import ROOT

HERE = Path(__file__).resolve().parent
TIMED = re.compile(r"^(?P<trigger>.+?)\s+for\s+(?P<op>>=|≥|>|more than|at least)\s*(?P<n>\d+)\s*ms\s*→\s*(?P<reaction>.+?)\s*$", re.I)
OPS = {">": ">", "more than": ">", ">=": ">=", "≥": ">=", "at least": ">="}


def catalogue() -> dict:
    return json.loads((HERE / "catalogue.json").read_text(encoding="utf-8"))


def known_faults() -> set[str]:
    """Fault types the bench can inject: those the scenarios use plus those the planner / runner code handles."""
    sc = json.loads((ROOT / "scenarios" / "scenarios.json").read_text(encoding="utf-8"))
    out = {f["type"] for s in sc["scenarios"] for f in s.get("faults", [])}
    for p in (ROOT / "ssb").glob("*.py"):
        t = p.read_text(encoding="utf-8")
        out |= set(re.findall(r'\.f\("([a-z_]+)"', t)) | set(re.findall(r'\["type"\] == "([a-z_]+)"', t))
    return out


def draft(req_id: str, reqs: dict, cfg: dict, cat: dict | None = None, llm=None, feedback: str = "") -> dict:
    """Rules first; if the wording is irregular and an LLM client is given, the drafter (design/drafter.py) tries."""
    cat = cat or catalogue()
    if req_id not in reqs:
        return {"req": req_id, "kind": "invalid", "errors": [f"{req_id} is not in safety/hazards.json"]}
    r = reqs[req_id]
    hand = HERE / "basis" / f"{req_id}.json"
    if hand.exists():
        b = json.loads(hand.read_text(encoding="utf-8"))
        b.update(req=req_id, text=r["text"], goal=r["goal"], ftti_ms=r.get("ftti_ms"), drafted_by=f"hand-written ({hand.name})")
        return validate(b, reqs, cfg, cat)
    m = TIMED.match(r["text"])
    if not m and llm is not None:
        from . import drafter
        return finish_event(drafter.draft(req_id, r, llm, feedback=feedback), reqs, cat)
    if not m:
        return {"req": req_id, "text": r["text"], "kind": "unparsed", "drafted_by": "rules",
                "errors": ["wording is not '<trigger> for > N ms → <reaction>': needs design/basis/<req>.json or an LLM draft"]}
    trig = [t for t in cat["triggers"] if any(w in m["trigger"].lower() for w in t["match"])]
    b = {"req": req_id, "text": r["text"], "goal": r["goal"], "ftti_ms": r.get("ftti_ms"), "kind": "timed_loss", "drafted_by": "rules",
         "trigger_phrase": m["trigger"], "trigger": trig[0]["id"] if len(trig) == 1 else None,
         "threshold": {"op": OPS[m["op"].lower()], "value_ms": int(m["n"])}, "reaction_phrase": m["reaction"].lower()}
    if len(trig) != 1:
        b["errors_pre"] = [f"trigger '{m['trigger']}' matches {len(trig)} catalogue entries (needs exactly 1)"]
    return validate(b, reqs, cfg, cat)


def validate(b: dict, reqs: dict, cfg: dict, cat: dict) -> dict:
    """Code checks on a drafted basis. Errors make it 'invalid' (no cases derived); open questions are listed for the reviewer."""
    errors, checks, questions = list(b.pop("errors_pre", [])), [], list(b.get("open_questions", []))
    trig = next((t for t in cat["triggers"] if t["id"] == b.get("trigger")), None)
    react = cat["reactions"].get(b.get("reaction_phrase", ""))
    if trig is None:
        errors.append(f"trigger '{b.get('trigger')}' is not in design/catalogue.json")
    if react is None:
        errors.append(f"reaction '{b.get('reaction_phrase')}' is not in design/catalogue.json")
    if errors:
        return {**b, "kind": "invalid", "errors": errors}
    assert trig is not None and react is not None
    n, op, period, ftti = b["threshold"]["value_ms"], b["threshold"]["op"], trig["period_ms"], b.get("ftti_ms")
    faults = known_faults()
    checks.append({"rule": f"bench can inject fault '{trig['fault']}'", "ok": trig["fault"] in faults})
    if trig.get("config_threshold"):
        have = cfg["safety"].get(trig["config_threshold"])
        checks.append({"rule": f"threshold {n} ms equals config safety.{trig['config_threshold']} ({have})", "ok": have == n})
    checks.append({"rule": f"FTTI given ({ftti} ms)", "ok": bool(ftti)})
    if ftti:
        checks.append({"rule": f"FTTI {ftti} ms ≥ threshold {n} ms + one {trig['message']} period ({period} ms)", "ok": ftti >= n + period})
    latch = cat.get("latch_requirement") if react.get("stop") else None
    if latch:
        checks.append({"rule": f"stop reactions are latched by {latch} (exists)", "ok": latch in reqs})
    errors += [c["rule"] for c in checks if not c["ok"]]
    below = trig.get("below_threshold_req")
    edge = n if op == ">" else n - 1
    if not below:
        questions.append(f"{b['req']} says what happens when the {trig['message']} loss lasts {op} {n} ms, but no requirement says what may "
                         f"happen at {edge} ms or less: may the controller stop (and latch) already, or must the vehicle keep driving? "
                         f"Cases at and below {edge} ms are blocked until a requirement answers this; they run as probes for information.")
    out = {**b, "kind": "timed_loss" if not errors else "invalid", "fault": trig["fault"], "duration_param": trig["duration_param"],
           "message": trig["message"], "period_ms": period, "cause": trig["cause"], "cause_after_resume": trig["cause_after_resume"],
           "reaction": react["reaction"], "stop": bool(react.get("stop")), "latched_by": latch,
           "checks": checks, "open_questions": questions}
    if errors:
        out["errors"] = errors
    return out


def finish_event(b: dict, reqs: dict, cat: dict) -> dict:
    """Code-owned fields of an LLM-drafted event basis: latch, FTTI check, gaps for alternatives the bench can't inject."""
    if b.get("kind") != "event":
        return b
    latch = cat.get("latch_requirement")
    checks = [{"rule": "drafter checks passed (faults, params, causes, alternatives, numbers)", "ok": True},
              {"rule": f"FTTI given ({b.get('ftti_ms')} ms)", "ok": bool(b.get("ftti_ms"))},
              {"rule": f"stop reactions are latched by {latch} (exists)", "ok": latch in reqs}]
    qs = list(b.get("open_questions") or [])
    qs += [f"'{x['covers']}' cannot be injected on this bench ({x['reason']}): blocked GAP case" for x in b.get("not_injectable") or []]
    for t in b["triggers"]:
        th = t.get("threshold")
        if th:
            edge = th["value"] if th["op"] == ">" else th["value"] - 1
            qs.append(f"{b['req']} sets the limit {th['param']} {th['op']} {th['value']:g}, but no requirement says what may happen at "
                      f"{edge:g} or less: those cases are blocked and run as probes for information.")
    from .drafter import fault_catalogue
    fcat = fault_catalogue()
    triggers = []
    for t in b["triggers"]:
        # a trigger equal to a bench scenario inherits how the bench times that fault (SC-37b: jitter reorders frames at a
        # random moment, so no FTTI from a fixed injection time). The model never sees or sets timing.
        ex = next((x for x in fcat.get(t["fault"], {}).get("examples", [])
                   if {k: v for k, v in x["fault"].items() if k not in ("type", "end")} == (t.get("params") or {})), None)
        if ex and set(ex["timing"]) - {"start"}:
            t = {**t, "bench_timing": {**ex["timing"], "from": ex["id"]}}
            note = f"'{t['covers']}' reuses the timing of bench scenario {ex['id']}" + (": no FTTI check (no fixed injection moment)"
                                                                                      if ex["timing"].get("no_ftti") else "")
            if ex.get("finding"):
                note += f". That scenario is a documented finding: {ex['finding']}"
            qs.append(note)
        triggers.append(t)
    out = {**b, "triggers": triggers, "stop": True, "latched_by": latch, "checks": checks, "open_questions": qs}
    if not all(c["ok"] for c in checks):
        out.update(kind="invalid", errors=[c["rule"] for c in checks if not c["ok"]])
    return out


def to_md(bases: list[dict]) -> str:
    lines = ["## Test basis (the oracle) per requirement", ""]
    for b in bases:
        lines += [f"### {b['req']} ({b.get('kind')}, drafted by {b.get('drafted_by', '?')})", "", f"> {b.get('text', '')}", ""]
        if b.get("errors"):
            lines += ["**Not usable:**"] + [f"- {e}" for e in b["errors"]] + [""]
        if b.get("kind") == "timed_loss":
            th = b["threshold"]
            lines += [f"- Trigger: {b['trigger_phrase']} → bench fault `{b['fault']}` on {b['message']} (period {b['period_ms']} ms)",
                      f"- Threshold: loss {th['op']} {th['value_ms']} ms · FTTI {b['ftti_ms']} ms",
                      f"- Reaction: {b['reaction']}, cause {' / '.join(b['cause'])} (after the link returns: {' / '.join(b['cause_after_resume'])})"
                      + (f", stop latched ({b['latched_by']})" if b.get("latched_by") else "")]
        if b.get("kind") == "event":
            lines += [f"- Reaction: {b['reaction']}" + (f", stop latched ({b['latched_by']})" if b.get("latched_by") else "") + f" · FTTI {b['ftti_ms']} ms"]
            for t in b["triggers"]:
                th = t.get("threshold")
                lines.append(f"- Trigger \"{t['covers']}\" → fault `{t['fault']}` {json.dumps(t.get('params') or {})}, cause {' / '.join(t['cause'])}"
                             + (f", limit {th['param']} {th['op']} {th['value']:g}" if th else "") + f". {t.get('why', '')}")
            for x in b.get("not_injectable") or []:
                lines.append(f"- Not injectable: \"{x['covers']}\": {x['reason']}")
            for a in b.get("assumptions") or []:
                lines.append(f"- Assumption: {a}")
        for c in b.get("checks", []):
            lines.append(f"- {'✅' if c['ok'] else '❌'} {c['rule']}")
        if b.get("open_questions"):
            lines += ["", "Open questions:"] + [f"- {q}" for q in b["open_questions"]]
        lines.append("")
    return "\n".join(lines)
