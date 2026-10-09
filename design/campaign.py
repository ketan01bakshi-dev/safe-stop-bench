"""One campaign: requirement → test basis → [basis_review] → cases → [case_review] → bench run → triage → [publish_approval] → tracker.

Resumable: every step writes its files under reports/design/<campaign>/ and a gate stops the run (exit code 3) until a
human approves the exact content (sha256). Re-running picks up where it stopped. Verdicts come from the bench oracle
(ssb/oracle.py) through run.py; nothing here decides PASS / FAIL.
"""
from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
import sys
from pathlib import Path

from health.check import bench_health
from ssb import campaigns, config
from ssb.config import ROOT

from . import basis, derive
from .gates import GatePending, HumanGate, artifact_hash
from .llm import Client, LLMUnavailable
from .tracker import GitHubError, GitHubTracker, LocalTracker, read_env_file

DESIGN_DIR = ROOT / "reports" / "design"


def _write(path: Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(data if isinstance(data, str) else json.dumps(data, indent=1, ensure_ascii=False), encoding="utf-8")


def _open_new(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    return path.open("a", encoding="utf-8")


def tracker(kind: str = "local", github_repo: str | None = None):
    if kind == "github":
        return GitHubTracker(github_repo or "")
    return LocalTracker(DESIGN_DIR / "tracker.json")


def run(name: str, reqs: list[str], level: str = "reference", defects: list[str] | None = None, config_path: str = "config/default.json",
        bench_args: list[str] | None = None, env_file: str | None = None, auto_approve: bool = False,
        rerun: bool = False, llm_plan: str = "subscription", log=print, llm_client=None, auto_actor: str = "demo-auto",
        tracker_kind: str = "local", github_repo: str | None = None, redraft: int = 0, _attempt: int = 0) -> int:
    """redraft: how many times the pipeline may reject its own LLM-drafted basis when cases turn out SUSPECT (they fail on the
    known-good reference controller). The rejection carries only facts the code knows; the model then drafts again."""
    cdir = DESIGN_DIR / name
    cfg = config.load(config_path)
    all_reqs = campaigns.requirements()["requirements"]
    gates = HumanGate(cdir, auto_approve=auto_approve, auto_actor=auto_actor)
    defects = sorted(defects or [])
    try:
        trk = tracker(tracker_kind, github_repo)   # built first: a malformed repo stops the run before any bench time
    except GitHubError as e:
        log(f"tracker: {e}")
        return 2
    try:
        # 1. test basis: rules, a hand-written file, or the LLM drafter (design/drafter.py); validated by code either way
        llm = llm_client
        if llm is None and llm_plan != "off":
            env = dict(os.environ, **(read_env_file(Path(env_file)) if env_file else {}))
            llm = Client(llm_plan, env, DESIGN_DIR / ".llm_cache")
        try:
            fb = gates.feedback(HumanGate.GATE_BASIS)   # rejection reasons go into the next LLM draft
            bases = [basis.draft(r, all_reqs, cfg, llm=llm, feedback=fb) for r in reqs]
        except LLMUnavailable as e:
            log(f"LLM drafter unavailable ({e}); use --llm off for rule-drafted requirements only, or --llm api with --env-file for the billed API")
            return 2
        if llm is not None and llm.calls:
            with _open_new(cdir / "01_basis" / "llm_calls.jsonl") as fh:   # provider, model, tokens, cached: the usage log
                for c in llm.calls:
                    fh.write(json.dumps(c) + "\n")
            llm.calls.clear()   # written once; a redraft round appends only its own calls
        basis_md = basis.to_md(bases)
        _write(cdir / "01_basis" / "basis.json", bases)
        _write(cdir / "01_basis" / "basis.md", f"# Test basis: {name}\n\n{basis_md}")
        bad = [b for b in bases if b.get("kind") not in ("timed_loss", "event")]
        if bad:
            log("not usable: " + "; ".join(f"{b['req']}: {', '.join(b.get('errors', []))}" for b in bad))
            return 2
        gates.require(HumanGate.GATE_BASIS, bases, basis_md)

        # 2. ISTQB cases (code), scenarios for the bench
        cases = [c for b in bases for c in derive.derive(b, cfg)]
        spec = derive.to_md(cases)
        _write(cdir / "02_design" / "cases.json", cases)
        _write(cdir / "02_design" / "spec.md", f"# Test specification: {name}\n\n{basis_md}\n{spec}")
        scen_path = cdir / "02_design" / "scenarios.json"
        _write(scen_path, derive.scenario_file(cases))
        gates.require(HumanGate.GATE_CASES, cases, spec)

        # 3. bench run through run.py (any level: reference, fmu, can, native, pil, hil ...)
        exe = cdir / "03_execute"
        key = artifact_hash({"cases": artifact_hash(cases), "level": level, "defects": defects, "config": config_path, "bench": bench_args or []})
        key_file = exe / "exec_key.txt"
        same_run = (exe / "run_manifest.json").exists() and key_file.exists() and key_file.read_text() == key
        if rerun or not same_run:   # the same approved cases on the same level are not run twice unless asked (--rerun)
            cmd = [sys.executable, str(ROOT / "run.py"), "--config", config_path, "--scenario-file", str(scen_path),
                   "--out", str(exe), "--label", "run", "--dut", level, *sum((["--defect", d] for d in defects), []), *(bench_args or [])]
            log(f"bench: {' '.join(cmd[1:])}")
            proc = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, encoding="utf-8", errors="replace")
            _write(exe / "run_stdout.txt", proc.stdout + proc.stderr)
            if not (exe / "run_manifest.json").exists():
                log(proc.stdout[-2000:] + proc.stderr[-2000:])
                raise SystemExit(f"run.py did not finish (exit {proc.returncode}); see {exe / 'run_stdout.txt'}")
            _write(exe / "exec_key.txt", key)
        manifest = json.loads((exe / "run_manifest.json").read_text(encoding="utf-8"))
        results = json.loads((exe / "run_results.json").read_text(encoding="utf-8"))["results"]

        # 3b. test the test: a case that FAILS on the known-good reference controller is a wrong case, not a defect.
        # If the run itself was the clean reference, its results are the golden ones; otherwise run it once more.
        if level == "reference" and not defects:
            golden = results
        else:
            gdir = exe / "golden"
            gkey = artifact_hash({"cases": artifact_hash(cases), "config": config_path})
            gkey_file = gdir / "exec_key.txt"
            golden_done = (gdir / "golden_results.json").exists() and gkey_file.exists() and gkey_file.read_text() == gkey
            if rerun or not golden_done:
                gcmd = [sys.executable, str(ROOT / "run.py"), "--config", config_path, "--scenario-file", str(scen_path),
                        "--out", str(gdir), "--label", "golden", "--dut", "reference"]
                gproc = subprocess.run(gcmd, cwd=ROOT, capture_output=True, text=True, encoding="utf-8", errors="replace")
                if not (gdir / "golden_results.json").exists():
                    raise SystemExit(f"reference (golden) run did not finish (exit {gproc.returncode}): {(gproc.stdout + gproc.stderr)[-500:]}")
                _write(gdir / "exec_key.txt", gkey)
            golden = json.loads((gdir / "golden_results.json").read_text(encoding="utf-8"))["results"]
        suspects = {r["key"] for r in golden if r["verdict"]["status"] == "FAIL"}

        # 4. triage: per-case verdicts, probes as information, failures grouped into defect drafts
        case_results = triage(cases, results, suspects, bench_health(manifest, results))

        # 4b. the agent as reviewer: suspect cases from an LLM-drafted basis are rejected with the facts and drafted again
        if suspects and _attempt < redraft and llm is not None:
            reason = redraft_reason(bases, cases, case_results)
            if reason:
                gates.reject(HumanGate.GATE_BASIS, auto_actor, reason)
                with _open_new(cdir / "01_basis" / "redraft_log.jsonl") as fh:
                    fh.write(json.dumps({"round": _attempt + 1, "rejected_by": auto_actor, "reason": reason,
                                         "suspect_cases": [k for k, v in case_results.items() if v["verdict"] == "SUSPECT"]}) + "\n")
                log(f"redraft {_attempt + 1}/{redraft}: the basis is rejected (suspect cases) and drafted again")
                return run(name, reqs, level, defects, config_path, bench_args, env_file, auto_approve, rerun, llm_plan, log,
                           llm_client=llm, auto_actor=auto_actor, tracker_kind=tracker_kind, github_repo=github_repo,
                           redraft=redraft, _attempt=_attempt + 1)
        drafts = ticket_drafts(bases, cases, case_results, manifest, trk)
        _write(cdir / "04_triage" / "case_results.json", case_results)
        _write(cdir / "04_triage" / "ticket_drafts.json", drafts)
        man_sha = hashlib.sha256((exe / "run_manifest.json").read_bytes()).hexdigest()
        packet = {"run_manifest_sha256": man_sha, "run_id": manifest["run_id"], "tracker": trk.name,
                  "case_results": case_results, "tickets": drafts}
        gates.require(HumanGate.GATE_PUBLISH, packet, results_md(case_results, drafts, manifest, trk.name))

        # 5. publish (once per approved packet)
        summary_path = cdir / "05_publish" / "publish_summary.json"
        ph = artifact_hash(packet)
        done = json.loads(summary_path.read_text(encoding="utf-8")) if summary_path.exists() else {}
        if done.get("packet_sha256") != ph:
            approver = gates.load().get(HumanGate.GATE_PUBLISH, {}).get("approver", "?")
            created = []
            for t in drafts:
                text = t["description"] + f"\nApproved at publish_approval by {approver} (packet sha256 {ph[:16]})."
                if t["type"] == "update":
                    created.append({"req": t["req"], "action": "comment", **trk.add_comment(t["key"], text)})
                else:
                    created.append({"req": t["req"], "action": "create", **trk.create_issue(t["title"], text, t["labels"])})
            done = {"packet_sha256": ph, "tracker": trk.name, "created": created, "run_id": manifest["run_id"]}
            _write(summary_path, done)
        _write(cdir / "05_publish" / "verification_report.md", verification_report(name, bases, cases, case_results, manifest, gates, done))
        log(f"published: {', '.join(c['key'] + ' (' + c['action'] + ')' for c in done['created']) or 'no defects'}; "
            f"report {cdir / '05_publish' / 'verification_report.md'}")
        return 0
    except GatePending as g:
        log(f"waiting for a human at gate '{g.gate}': review {g.review_path}")
        log(f"  approve: python -m design approve --campaign {name} --gate {g.gate} --by \"<name>\"")
        log(f"  reject:  python -m design reject --campaign {name} --gate {g.gate} --by \"<name>\" --reason \"<what to fix>\"")
        return 3


def redraft_reason(bases: list[dict], cases: list[dict], case_results: dict) -> str:
    """The rejection text: for each suspect case of an LLM-drafted requirement, what the case expected, what the known-good
    controller did, and what the bench's own scenarios say about that fault. Facts only; the fix is the model's job."""
    from .drafter import fault_evidence
    llm_reqs = {b["req"]: b for b in bases if str(b.get("drafted_by", "")).startswith("llm")}
    lines = []
    for c in cases:
        cr = case_results[c["id"]]
        b = llm_reqs.get(c["req"])
        if cr["verdict"] != "SUSPECT" or not b or not c["scenario"]:
            continue
        fault = c["scenario"]["faults"][0]
        params = {k: v for k, v in fault.items() if k not in ("type", "start", "end")}
        trig = next((t for t in b.get("triggers", []) if t["fault"] == fault["type"] and (t.get("params") or {}) == params), None)
        exp = c["scenario"]["expect"]
        lines.append(
            f"- {c['req']}, alternative '{trig['covers'] if trig else c['coverage_item']}' (fault {fault['type']} {params or '(no parameters)'}): "
            f"the case expects {exp.get('reaction')} with cause {'/'.join(exp.get('cause', []))}, but the KNOWN-GOOD reference controller "
            f"shows {cr.get('peak_state')}" + (f" (cause {cr['cause']})" if cr.get("cause") else "")
            + ". So this fault does not produce that alternative on this bench. "
            f"Bench scenarios that inject {fault['type']}: " + "; ".join(fault_evidence(fault["type"])[:6]) + ".")
    if not lines:
        return ""
    return ("These cases of your last draft FAIL on the healthy reference controller, so the fault chosen for them is wrong "
            "(not the product):\n" + "\n".join(lines) + "\nFor each alternative above choose a different fault or different "
            "parameters that the bench expects to cause a stop (see the examples), or declare it not_injectable with the reason. "
            "Keep every other trigger unchanged.")


def triage(cases: list[dict], results: list[dict], suspects: set[str] | None = None, bench: dict | None = None) -> dict:
    """suspects: scenario keys that FAIL on the known-good reference controller. Such a case is SUSPECT (the case or its
    basis is wrong), never a defect: it is shown for review and kept out of the defect drafts.
    bench: health/check.bench_health(): a FAIL on a run whose firmware is not the inventory's, or on a scenario that needed
    bench-fault reruns, is SUSPECT BENCH (the bench may be at fault), never a defect either."""
    suspects = suspects or set()
    bench = bench or {"fleet": [], "by_key": {}}
    by_key = {r["key"]: r for r in results}
    out = {}
    for c in cases:
        if not c["scenario"]:
            out[c["id"]] = {"verdict": "BLOCKED", "probe": False}
            continue
        r = by_key.get(c["scenario"]["key"])
        if r is None:
            out[c["id"]] = {"verdict": "NOT RUN", "probe": c["probe"]}
            continue
        v = r["verdict"]
        obs = {"peak_state": v["peak_state"], "cause": r.get("cause"), "t_detect_ms": v.get("t_detect_ms"), "ftti_ms": v.get("ftti_ms")}
        if c["probe"]:
            out[c["id"]] = {"verdict": "INFO", "probe": True, "blocked": c["blocked"], **obs}
        else:
            verdict = "SUSPECT" if v["status"] == "FAIL" and c["scenario"]["key"] in suspects else v["status"]
            why_bench = bench["fleet"] + bench["by_key"].get(c["scenario"]["key"], [])
            if verdict == "FAIL" and why_bench:
                verdict = "SUSPECT BENCH"
            out[c["id"]] = {"verdict": verdict, "probe": False, "failed": [n for n, ok in v["checks"] if not ok], **obs}
            if verdict == "SUSPECT BENCH":
                out[c["id"]]["why"] = "the bench may be at fault, not the product: " + "; ".join(why_bench)
            if verdict == "SUSPECT":
                out[c["id"]]["why"] = "also fails on the known-good reference controller: the case or its test basis is wrong, not the product"
    return out


NEXT_CHECKS = {
    "detected within FTTI": "Compare the controller's timeout / debounce parameters in the build under test with the requirement threshold; "
                            "detection far past the threshold points at the parameter, not the bus.",
    "no reaction before": "The controller reacted before the threshold: check the timeout start (last valid frame vs last frame) and the "
                          "first-frame-after-gap rule.",
    "reaction ": "The controller did not reach the required state: check that this fault reaches the monitor at all (frame filter, "
                 "receive path) and the state machine's transition into it.",
    "still latched": "The stop released itself: check the latch and the release conditions (SR-05).",
    "vehicle stopped": "The state was reached but the vehicle did not stop: check the actuator commands in that state.",
}


def _input(b: dict, fault: str) -> str:
    """What failed, in words: the message lost (timed requirement) or the alternative the fault produces (event requirement)."""
    if b["kind"] == "timed_loss":
        return f"{b['message']} loss"
    return next((f"\"{x['covers']}\"" for x in b["triggers"] if x["fault"] == fault), fault)


def _root(parent: dict, f: str) -> str:
    """Union-find root: faults joined by a shared failure mode end up under one root."""
    while parent.setdefault(f, f) != f:
        f = parent[f]
    return f


def _mode(cr: dict) -> str:
    """How a case failed, without its numbers: "cause in ['E2E_INVALID']" -> "cause in", "detected within FTTI 250 ms" -> ..."""
    return re.split(r"[\d\[']", cr["failed"][0], maxsplit=1)[0].strip()


def _title(b: dict, what: str, crs: list[dict]) -> str:
    first = crs[0]["failed"][0]
    if first.startswith("detected within FTTI"):
        worst = max((c["t_detect_ms"] for c in crs if c.get("t_detect_ms") is not None), default=None)
        return f"{b['req']}: {what} detected after {worst} ms, FTTI {b['ftti_ms']} ms"
    if first.startswith("no reaction before") and b["kind"] == "timed_loss":
        return f"{b['req']}: reacted {crs[0]['t_detect_ms']} ms into a {what}, before the {b['threshold']['value_ms']} ms threshold"
    if first.startswith("reaction "):
        return f"{b['req']}: no {b['reaction']} after {what} (saw {crs[0]['peak_state']})"
    if first.startswith("still latched"):
        return f"{b['req']}: stop not latched after {what}"
    if first.startswith("cause in"):
        return f"{b['req']}: {what} stopped with cause {crs[0]['cause']}, expected {first[len('cause in '):]}"
    return f"{b['req']}: '{first}' failed on {what}"


def ticket_drafts(bases: list[dict], cases: list[dict], case_results: dict, manifest: dict, trk) -> list[dict]:
    """Defect drafts per requirement. Failing cases are grouped by failure mode (first failed check), and groups that share
    an injected fault (the deciding input) are merged: one bug with several symptoms on overlapping inputs is ONE ticket
    (e2e_lenient: CRC / counter / data-ID cases report TIMEOUT, the intermittent-CRC and jitter cases don't stop at all),
    while independent failures on separate inputs stay separate. Across runs, an open defect with one of the same inputs
    AND the same failure modes gets a comment instead of a duplicate."""
    out = []
    run_info = manifest.get("run", {})
    seeded = run_info.get("defects") or []
    g = manifest.get("git") or {}
    for b in bases:
        failing = [(c, case_results[c["id"]]) for c in cases if c["req"] == b["req"] and case_results[c["id"]]["verdict"] == "FAIL"]
        parent: dict[str, str] = {}
        by_mode: dict[str, set] = {}
        for c, cr in failing:
            by_mode.setdefault(_mode(cr), set()).add(c["scenario"]["faults"][0]["type"])
        for fs in by_mode.values():        # union the faults that share a failure mode
            first, *rest = sorted(fs)
            for f in rest:
                parent[_root(parent, f)] = _root(parent, first)
        comps: dict[str, list] = {}
        for c, cr in failing:
            comps.setdefault(_root(parent, c["scenario"]["faults"][0]["type"]), []).append((c, cr))
        n_exec = sum(1 for c in cases if c["req"] == b["req"] and not c["blocked"])
        for grp in comps.values():
            faults = list(dict.fromkeys(c["scenario"]["faults"][0]["type"] for c, _ in grp))
            sigs = [f"sig-{f}" for f in faults]
            what = " / ".join(_input(b, f) for f in faults)
            crs = [cr for _, cr in grp]
            lines = [f"Requirement {b['req']}: {b['text']} (FTTI {b['ftti_ms']} ms).",
                     f"Deciding input{'s' if len(faults) > 1 else ''}: {what} (bench fault{'s' if len(faults) > 1 else ''} "
                     f"{', '.join(faults)}). Failing cases: {len(grp)} of {n_exec} executable."]
            for c, cr in grp:
                lines.append(f"- {c['id']} ({c['coverage_item']}): failed {'; '.join(cr['failed'])}. Observed {cr['peak_state']}, "
                             f"cause {cr['cause']}, detection {cr['t_detect_ms']} ms.")
            lines += [f"Run {manifest['run_id']}: level {run_info.get('level')}, config {run_info.get('config_name')}, bench {manifest.get('bench')}, "
                      f"commit {str(g.get('commit', '?'))[:10]}{' (uncommitted changes)' if g.get('dirty') else ''}"
                      + (f", firmware {manifest['firmware']}" if manifest.get("firmware") else "") + "."]
            lines.append(f"Test basis drafted by {b['drafted_by']}, approved at basis_review.")
            if seeded:
                lines.append(f"Seeded defect(s) in this run: {', '.join(seeded)}. A deliberate bug, to prove the chain catches it; not a product defect.")
            # hints from each case's FIRST failed check: the later ones (not stopped, not latched) follow from it
            nxt = sorted({v for cr in crs for k, v in NEXT_CHECKS.items() if cr["failed"][0].startswith(k)})
            if nxt:
                lines += ["Next checks:"] + [f"- {x}" for x in nxt]
            lines.append("Verdicts come from the bench oracle (code), not from a model.")
            # same defect again = an open ticket with one of these inputs AND the same failure mode: a stop that no longer
            # latches is not a recurrence of "wrong sequence not detected", even when the jitter case is in both
            modes = sorted({"mode-" + re.sub(r"[^a-z0-9]+", "-", _mode(cr).lower()).strip("-") for cr in crs})
            labels = ["ssb", "defect", b["req"], *sigs, *modes] + (["seeded"] if seeded else [])
            same = [d for d in trk.open_defects(b["req"])
                    if set(sigs) & set(d["labels"]) and sorted(x for x in d["labels"] if x.startswith("mode-")) == modes]
            draft = {"req": b["req"], "signature": " ".join(sigs), "labels": labels, "title": _title(b, what, crs), "description": "\n".join(lines),
                     "cases": [c["id"] for c, _ in grp]}
            if same:
                draft.update(type="update", key=same[0]["key"], title=same[0]["title"],
                             description=f"Recurred in run {manifest['run_id']}.\n" + draft["description"])
            else:
                draft["type"] = "defect"
            out.append(draft)
    return out


def results_md(case_results: dict, drafts: list[dict], manifest: dict, tracker_name: str) -> str:
    lines = [f"## Results of run {manifest['run_id']} ({manifest['run'].get('level')})", "",
             "| Case | Verdict | Observed | Failed checks |", "|---|---|---|---|"]
    for k, r in case_results.items():
        obs = f"{r.get('peak_state', '–')}, cause {r.get('cause') or '–'}, detect {r.get('t_detect_ms', '–')} ms" if "peak_state" in r else "–"
        lines.append(f"| {k} | {r['verdict']} | {obs} | {'; '.join(r.get('failed', [])) or '–'} |")
    lines += ["", f"On approval, in {tracker_name}: "
              + (", ".join(f"{d['type']} {d.get('key', '(new)')}: {d['title']}" for d in drafts) or "nothing (no failures)") + "."]
    for d in drafts:
        lines += ["", f"### {'Comment on ' + d['key'] if d['type'] == 'update' else 'New defect'}: {d['title']}", "", "Labels: " + ", ".join(d["labels"]),
                  "", d["description"]]
    return "\n".join(lines)


def verification_report(name: str, bases, cases, case_results, manifest, gates: HumanGate, done: dict) -> str:
    g = gates.load()
    created: dict[str, list] = {}
    for c in done.get("created", []):
        created.setdefault(c["req"], []).append(c)
    lines = [f"# Verification report: {name}", "",
             f"Run {manifest['run_id']} · level {manifest['run'].get('level')} · {manifest.get('bench')} · "
             f"defects seeded: {', '.join(manifest['run'].get('defects') or []) or 'none'}", "",
             "> Test basis drafted by rules (or by hand), validated by code, approved by a human. Cases derived by code.",
             "> PASS / FAIL from the bench oracle. Defects go to the tracker only after publish_approval.", "",
             "## Traceability", "", "| Requirement | Basis | Cases (exec / blocked) | PASS | FAIL | Ticket |", "|---|---|---|---|---|---|"]
    for b in bases:
        cs = [c for c in cases if c["req"] == b["req"]]
        v = [case_results[c["id"]]["verdict"] for c in cs]
        ts = created.get(b["req"], [])
        lines.append(f"| {b['req']} | {b['drafted_by']} | {sum(1 for c in cs if not c['blocked'])} / {sum(1 for c in cs if c['blocked'])} | "
                     f"{v.count('PASS')} | {v.count('FAIL')}{' (+' + str(v.count('SUSPECT')) + ' suspect)' if v.count('SUSPECT') else ''} | {', '.join(x['key'] + ' (' + x['action'] + ')' for x in ts) or '–'} |")
    lines += ["", "## Cases", "", "| Case | Technique | Coverage item | Verdict | Observed |", "|---|---|---|---|---|"]
    for c in cases:
        r = case_results[c["id"]]
        lines.append(f"| {c['id']} | {c['technique']} | {c['coverage_item']} | {r['verdict']} | "
                     f"{r.get('peak_state', '–')}, detect {r.get('t_detect_ms', '–')} ms |")
    probes = [(c, case_results[c["id"]]) for c in cases if c["probe"]]
    qs = [q for b in bases for q in b.get("open_questions", [])]
    suspects = [(c, case_results[c["id"]]) for c in cases if case_results[c["id"]]["verdict"] == "SUSPECT"]
    if suspects:
        lines += ["", "## Suspect cases (not filed as defects)", "",
                  "These cases also FAIL on the known-good reference controller, so the case or its test basis is wrong. Fix the basis "
                  "(a rejection reason at basis_review goes into the next draft) and re-run.", ""]
        for c, r in suspects:
            lines.append(f"- {c['id']} ({c['coverage_item']}): failed {'; '.join(r['failed'])}. Observed {r.get('peak_state')}.")
        for b in bases:
            for a in b.get("assumptions") or []:
                if "verif" in a.lower() or "assum" in a.lower():
                    lines.append(f"- The drafter's own note on {b['req']}: {a}")
    sb = [(c, case_results[c["id"]]) for c in cases if case_results[c["id"]]["verdict"] == "SUSPECT BENCH"]
    if sb:
        lines += ["", "## Suspect bench (not filed as defects)", "",
                  "These cases FAILED, but the bench itself is in doubt. Fix the bench (reflash, reseat, rerun) before reading them as product failures.", ""]
        for c, r in sb:
            lines.append(f"- {c['id']} ({c['coverage_item']}): {r['why']}.")
    if qs:
        lines += ["", "## Open questions in the requirements", ""] + [f"- {q}" for q in qs]
        if probes:
            lines += ["", "Probe observations (information for the question, not verdicts):", ""]
            lines += [f"- {c['coverage_item']}: {r.get('peak_state')} (cause {r.get('cause') or '–'}, after {r.get('t_detect_ms', '–')} ms)"
                      for c, r in probes]
    lines += ["", "## Human approvals", "", "| Gate | Decision | By | When | sha256 |", "|---|---|---|---|---|"]
    lines += [f"| {k} | {g.get(k, {}).get('decision', '–')} | {g.get(k, {}).get('approver', '–')} | {str(g.get(k, {}).get('at', '–'))[:19]} | "
              f"`{str(g.get(k, {}).get('artifact_hash', ''))[:16]}` |" for k in HumanGate.ALL]
    lines += ["", f"Run record: `03_execute/run_manifest.json` (git {str((manifest.get('git') or {}).get('commit', '?'))[:10]}, "
                  f"{len(manifest.get('files', {}))} files hashed)."]
    return "\n".join(lines) + "\n"
