"""LLM drafter for requirements the rules can't parse (v2.14). The model drafts, code validates, a human approves.

The model gets the requirement and a fault catalogue built from this bench's own scenarios (fault type, parameters seen,
example title and expected reaction / cause), so it can only map wording onto faults the bench can really inject.
Code then checks the draft (the HIL ML Ops grounding rules, on this bench's vocabulary):
- every alternative the requirement lists in parentheses is covered by a trigger or declared not injectable
- every fault exists and uses only parameters seen for it; reaction and cause are known to the oracle
- every number in the requirement appears in a trigger threshold or an assumption (no silent dropping, no invented values)
- a trigger threshold names a parameter of its fault, with a value from the requirement text
Errors go back to the model (up to `repairs` rounds); a draft that still fails is 'invalid' and never reaches a gate.
"""
from __future__ import annotations

import json
import re

from ssb.config import ROOT

from .llm import Client

STOP_REACTIONS = {"STOP_IN_LANE": "controlled stop in the lane", "BRAKE_ONLY_STOP": "stop with brakes only (steering unusable)",
                  "BACKUP_BRAKE_STOP": "stop with the backup brake"}


def fault_catalogue() -> dict:
    sc = json.loads((ROOT / "scenarios" / "scenarios.json").read_text(encoding="utf-8"))["scenarios"]
    out: dict[str, dict] = {}
    for s in sc:
        for f in s.get("faults", []):
            e = out.setdefault(f["type"], {"params": set(), "examples": []})
            e["params"] |= {k for k in f if k not in ("type", "start")}
            if len(e["examples"]) < 2:
                e["examples"].append({"title": s["title"], "fault": {k: v for k, v in f.items() if k not in ("start",)},
                                      "reaction": s.get("expect", {}).get("reaction"), "cause": s.get("expect", {}).get("cause"),
                                      # how the bench times this fault (v2.14): a drafted trigger equal to the example inherits it
                                      "timing": {k: s[k] for k in ("no_ftti", "inject_ms", "duration_ms") if k in s} | {"start": f["start"]},
                                      "id": s["id"], "finding": s.get("finding")})
    return out


def fault_evidence(fault: str) -> list[str]:
    """Every bench scenario that injects `fault`: its parameters and what the bench expects (no reaction / a stop). Facts for a
    redraft, taken from the scenario file, never from a model."""
    sc = json.loads((ROOT / "scenarios" / "scenarios.json").read_text(encoding="utf-8"))["scenarios"]
    out = []
    for s in sc:
        for f in s.get("faults", []):
            if f["type"] == fault:
                params = {k: v for k, v in f.items() if k not in ("type", "start", "end")}
                exp = s.get("expect", {}).get("reaction")
                out.append(f"{s['id']} {params or '(no parameters)'} -> " + ("NO reaction, tolerated" if exp == "NORMAL" else f"{exp}"))
    return list(dict.fromkeys(out))   # a scenario that injects the same fault several times is listed once


def known_causes() -> set[str]:
    sc = json.loads((ROOT / "scenarios" / "scenarios.json").read_text(encoding="utf-8"))["scenarios"]
    return {c for s in sc for c in s.get("expect", {}).get("cause", [])}


def alternatives(text: str) -> list[str]:
    """The alternatives a requirement lists, found by code: in parentheses ('INVALID (bad CRC, repeated, wrong data ID)')
    or as a list before the arrow ('Watchdog: no kick > 60 ms, 3 early kicks, or 3 wrong challenge answers → ...')."""
    m = re.search(r"\(([^)]*,[^)]*)\)", text)
    if m:
        return [a.strip() for a in m.group(1).split(",") if a.strip()]
    if not re.search(r"→|->", text):   # a list only counts as alternatives in "<conditions> → <reaction>" wording
        return []
    cond = re.split(r"→|->", text)[0]
    cond = cond.split(":", 1)[1] if ":" in cond else cond
    parts = [a.strip() for a in re.split(r",\s*(?:or\s+)?|\s+or\s+", cond) if a.strip()]
    return parts if len(parts) > 1 else []


def numbers(text: str) -> set[float]:
    return {float(x) for x in re.findall(r"(?<![\w.-])(\d+(?:\.\d+)?)", text)}


SCHEMA = """{
  "kind": "event",
  "triggers": [
    {"covers": "<the alternative, word for word from the requirement>", "fault": "<fault type from the catalogue>",
     "params": {"<param seen for that fault>": <value>}, "cause": ["<cause the controller should report>"],
     "threshold": null | {"param": "<param of this fault>", "op": ">" | ">=", "value": <number from the requirement>},
     "why": "<one sentence: why this fault produces this alternative>"}
  ],
  "not_injectable": [{"covers": "<alternative>", "reason": "<what the bench lacks>"}],
  "reaction": "<one of the reactions>",
  "assumptions": ["<anything you had to assume; name every number of the requirement you did not use as a threshold>"],
  "open_questions": ["<what the requirement leaves unclear>"]
}"""


def prompt(req_id: str, text: str, cat: dict, causes: set[str], feedback: str = "") -> list[dict]:
    alts = alternatives(text)
    lines = [f"- {f}: params {sorted(e['params']) or '[]'}; e.g. " + "; ".join(
        f"\"{x['title']}\" {json.dumps(x['fault'])} -> {x['reaction']} {x['cause'] or ''}" for x in e["examples"]) for f, e in sorted(cat.items())]
    sys_msg = ("You draft the TEST BASIS for one safety requirement of an autonomous-vehicle safety controller bench. "
               "The basis is the oracle a human will review, so be exact and never invent. Answer with one JSON object only.")
    user = f"""Requirement {req_id}: "{text}"

Bench fault catalogue (the ONLY faults you may use; each with the parameters it accepts and real examples):
{chr(10).join(lines)}

{"Alternatives found in the requirement (each needs exactly one trigger, or a not_injectable entry; set covers to this exact text): " + json.dumps(alts) if alts else "The requirement has a single condition: one trigger."}

Reactions (stop reactions only): {json.dumps(STOP_REACTIONS)}
Causes the controller can report: {sorted(causes)}

Rules:
1. One trigger per alternative the requirement lists (in parentheses or joined by "or"); "covers" quotes it word for word.
2. Use only fault types and parameters from the catalogue. Do not set "start" or "end": the test design sets timing.
3. If no catalogue fault can produce an alternative, list it in "not_injectable" with the reason. Never force a wrong fault.
4. "cause" per trigger: the cause the examples show for that fault, if it fits the alternative.
5. A number in the requirement that is a limit on a fault parameter goes in that trigger's "threshold"
   (e.g. "older than 60 ms" -> {{"param": "age_ms", "op": ">", "value": 60}}). Every other number goes in an assumption, named.
6. Do not add behaviour the requirement does not state. Unclear points go in "open_questions".

Schema:
{SCHEMA}"""
    if feedback:   # rejection reasons from basis_review: the reviewer's word beats the model's guess
        user += f"\n\nA reviewer REJECTED an earlier draft (a person, or the pipeline's own check against the known-good controller). Every point below must be followed:\n{feedback}"
    return [{"role": "system", "content": sys_msg}, {"role": "user", "content": user}]


def validate_event(d: dict, text: str, cat: dict, causes: set[str]) -> list[str]:
    errs = []
    if d.get("kind") != "event":
        errs.append('"kind" must be "event"')
    if d.get("reaction") not in STOP_REACTIONS:
        errs.append(f"reaction {d.get('reaction')!r} is not one of {sorted(STOP_REACTIONS)}")
    trig, ni = d.get("triggers") or [], d.get("not_injectable") or []
    if not trig:
        errs.append("no triggers")
    for i, t in enumerate(trig):
        f = t.get("fault")
        if f not in cat:
            errs.append(f"trigger {i + 1}: fault {f!r} is not in the catalogue")
            continue
        bad = set(t.get("params") or {}) - cat[f]["params"]
        if bad:
            errs.append(f"trigger {i + 1}: params {sorted(bad)} are not accepted by {f} (accepted: {sorted(cat[f]['params'])})")
        if {"start", "end"} & set(t.get("params") or {}):
            errs.append(f"trigger {i + 1}: do not set start / end")
        if not t.get("cause") or not set(t["cause"]) <= causes:
            errs.append(f"trigger {i + 1}: cause {t.get('cause')} must be a non-empty subset of the known causes")
        th = t.get("threshold")
        if th:
            if th.get("param") not in cat[f]["params"]:
                errs.append(f"trigger {i + 1}: threshold param {th.get('param')!r} is not a parameter of {f}")
            if th.get("op") not in (">", ">="):
                errs.append(f"trigger {i + 1}: threshold op must be > or >=")
            if float(th.get("value", -1)) not in numbers(text):
                errs.append(f"trigger {i + 1}: threshold value {th.get('value')} is not a number in the requirement")
    alts = alternatives(text)
    hits: dict[str, int] = {a: 0 for a in alts}
    for i, x in enumerate(trig + ni):
        cov = str(x.get("covers", "")).lower()
        found = [a for a in alts if a.lower() in cov]
        if alts and len(found) != 1:
            errs.append(f"entry {i + 1} covers {x.get('covers')!r}: it must name exactly one of {alts}")
        for a in found:
            hits[a] += 1
    for a, count in hits.items():
        if count == 0:
            errs.append(f"alternative '{a}' is neither covered by a trigger nor declared not_injectable")
    seen: dict[str, str] = {}
    for t in trig:   # one fault, same parameters, cannot produce two different alternatives: one of them is mapped wrong
        sig = json.dumps([t.get("fault"), t.get("params") or {}], sort_keys=True)
        if sig in seen and seen[sig] != t.get("covers"):
            errs.append(f"'{seen[sig]}' and '{t.get('covers')}' use the same fault and parameters ({t.get('fault')} {t.get('params') or {}}): "
                        f"two different conditions need different faults; recheck which fault produces each")
        seen.setdefault(sig, t.get("covers"))
        f = t.get("fault")
        for ex in cat.get(f, {}).get("examples", []):
            same = {k: v for k, v in ex["fault"].items() if k not in ("type", "end")} == (t.get("params") or {})
            if same and ex["reaction"] == "NORMAL":
                errs.append(f"trigger '{t.get('covers')}': {f} {t.get('params') or {}} is the bench example \"{ex['title']}\", which the "
                            f"bench must TOLERATE (no reaction): it cannot be a stop trigger")
    used = {float(t["threshold"]["value"]) for t in trig if t.get("threshold") and isinstance(t["threshold"].get("value"), (int, float))}
    said = " ".join(d.get("assumptions") or [])
    for num in numbers(text) - used:
        if not re.search(rf"(?<![\d.]){num:g}(?![\d.])", said):
            errs.append(f"the number {num:g} in the requirement is neither a trigger threshold nor named in an assumption")
    return errs


def _json_text(raw: str) -> str:
    """The JSON object in a model answer: Claude may wrap it in a code fence or add a sentence around it."""
    start, end = raw.find("{"), raw.rfind("}")
    return raw[start:end + 1] if 0 <= start < end else raw


def draft(req_id: str, req: dict, client: Client, repairs: int = 2, feedback: str = "") -> dict:
    cat, causes = fault_catalogue(), known_causes()
    messages = prompt(req_id, req["text"], cat, causes, feedback)
    errs: list[str] = []
    for attempt in range(1, repairs + 2):
        raw = client.chat(messages)
        try:
            d = json.loads(_json_text(raw))
        except json.JSONDecodeError as e:
            d, errs = {}, [f"not valid JSON: {e}"]
        else:
            errs = validate_event(d, req["text"], cat, causes)
        if not errs:
            last = client.calls[-1]
            return {**d, "req": req_id, "text": req["text"], "goal": req["goal"], "ftti_ms": req.get("ftti_ms"),
                    "drafted_by": f"llm {last['provider']}:{last['model']}" + (f" (after {attempt - 1} repair round(s))" if attempt > 1 else ""),
                    "repairs": attempt - 1}
        messages = messages + [{"role": "assistant", "content": raw},
                               {"role": "user", "content": "The checks found these errors. Return the corrected JSON object only:\n- "
                                + "\n- ".join(errs)}]
    return {"req": req_id, "text": req["text"], "kind": "invalid", "drafted_by": "llm", "errors": [f"after {repairs} repairs: {e}" for e in errs]}
