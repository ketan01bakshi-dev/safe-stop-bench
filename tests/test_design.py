"""python -m unittest tests.test_design -v   (v2.14: the test-design front end, integration plan P1)"""
import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from design import __main__ as cli  # noqa: E402
from design import basis, campaign, derive  # noqa: E402
from design.gates import GatePending, HumanGate  # noqa: E402
from ssb import campaigns, config, oracle, runner, scenarios  # noqa: E402
from ssb.dut import ReferenceDUT  # noqa: E402

CFG = config.load()
REQS = campaigns.requirements()["requirements"]


class BasisTests(unittest.TestCase):
    def test_sr01_drafted_by_rules_and_valid(self):
        b = basis.draft("SR-01", REQS, CFG)
        self.assertEqual(b["kind"], "timed_loss")
        self.assertEqual((b["trigger"], b["fault"], b["threshold"]), ("command_loss", "link_lost", {"op": ">", "value_ms": 100}))
        self.assertEqual((b["reaction"], b["latched_by"], b["ftti_ms"]), ("STOP_IN_LANE", "SR-05", 250))
        self.assertTrue(all(c["ok"] for c in b["checks"]), b["checks"])
        self.assertEqual(len(b["open_questions"]), 1)   # nothing says what may happen at 100 ms or less

    def test_irregular_wording_is_not_guessed(self):
        b = basis.draft("SR-02", REQS, CFG)
        self.assertEqual(b["kind"], "unparsed")

    def test_threshold_must_match_the_config(self):
        cfg = copy.deepcopy(CFG)
        cfg["safety"]["cmd_timeout_ms"] = 120
        b = basis.draft("SR-01", REQS, cfg)
        self.assertEqual(b["kind"], "invalid")
        self.assertTrue(any("cmd_timeout_ms" in e for e in b["errors"]))

    def test_ftti_shorter_than_threshold_plus_period_is_rejected(self):
        reqs = copy.deepcopy(REQS)
        reqs["SR-01"]["ftti_ms"] = 110
        self.assertEqual(basis.draft("SR-01", reqs, CFG)["kind"], "invalid")

    def test_unknown_requirement(self):
        self.assertEqual(basis.draft("SR-99", REQS, CFG)["kind"], "invalid")


class DeriveTests(unittest.TestCase):
    def setUp(self):
        self.cases = derive.derive(basis.draft("SR-01", REQS, CFG), CFG)
        self.by = {c["id"]: c for c in self.cases}

    def test_case_set(self):
        self.assertEqual(list(self.by), ["SR-01-FI-01", "SR-01-FI-02", "SR-01-FI-03", "SR-01-FI-04",
                                         "SR-01-BVA-01", "SR-01-BVA-02", "SR-01-BVA-03"])
        self.assertEqual([c["scenario"]["start_kmh"] for c in self.cases[:3]], [10.0, 30.0, 39.0])   # ODD 40 km/h

    def test_timing_and_boundaries(self):
        fi = self.by["SR-01-FI-01"]["scenario"]
        self.assertEqual(fi["expect"]["min_detect_ms"], 80)            # threshold 100 - one 20 ms frame period
        self.assertEqual(fi["req"], ["SR-01", "SR-05"])
        self.assertEqual(self.by["SR-01-FI-04"]["scenario"]["inject_ms"], 2010)
        self.assertEqual(self.by["SR-01-BVA-01"]["scenario"]["faults"][0]["end"], 2101)
        self.assertEqual(self.by["SR-01-BVA-01"]["scenario"]["expect"]["cause"], ["TIMEOUT", "E2E_INVALID"])

    def test_unspecified_side_is_blocked_and_probed(self):
        for cid in ("SR-01-BVA-02", "SR-01-BVA-03"):
            c = self.by[cid]
            self.assertTrue(c["blocked"].startswith("GAP"))
            self.assertTrue(c["probe"])
            self.assertEqual(c["scenario"]["expect"], {})
        self.assertEqual(derive.coverage(self.cases)["BVA3"], {"items": 3, "executable": 1, "blocked": 2})


class OracleMinDetectTests(unittest.TestCase):
    def test_reaction_before_the_threshold_fails(self):
        sc = scenarios._defaults({"id": "x", "key": "x", "req": ["SR-01"], "faults": [{"type": "link_lost", "start": 2000}],
                                  "expect": {"reaction": "STOP_IN_LANE", "min_detect_ms": 150}})
        r = runner.run(sc, CFG, ReferenceDUT(CFG), keep_trace=True)
        v = oracle.verdict(r, sc, REQS, CFG)
        self.assertIn("no reaction before 150 ms", [c for c, ok in v["checks"] if not ok])

    def test_no_reaction_at_all_is_not_blamed_on_it(self):
        sc = scenarios._defaults({"id": "x", "key": "x", "req": ["SR-01"], "faults": [],
                                  "expect": {"reaction": "STOP_IN_LANE", "min_detect_ms": 80}})
        v = oracle.verdict(runner.run(sc, CFG, ReferenceDUT(CFG), keep_trace=True), sc, REQS, CFG)
        failed = [c for c, ok in v["checks"] if not ok]
        self.assertIn("reaction STOP_IN_LANE", failed)
        self.assertNotIn("no reaction before 80 ms", failed)


class GateTests(unittest.TestCase):
    def test_changed_artifact_reopens_an_approved_gate(self):
        with tempfile.TemporaryDirectory() as tmp:
            g = HumanGate(Path(tmp))
            with self.assertRaises(GatePending):
                g.require("basis_review", {"a": 1})
            g.approve("basis_review", "tester")
            g.require("basis_review", {"a": 1})                       # same content: approval reused
            with self.assertRaises(GatePending):
                g.require("basis_review", {"a": 2})                   # changed: open again


class CampaignTests(unittest.TestCase):
    """The whole chain on the reference controller, in a temporary reports/design."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.saved = campaign.DESIGN_DIR
        campaign.DESIGN_DIR = Path(self.tmp.name)

    def tearDown(self):
        campaign.DESIGN_DIR = self.saved
        self.tmp.cleanup()

    def run_c(self, name, **kw):
        return campaign.run(name, ["SR-01"], log=lambda *a: None, **kw)

    def test_human_gates_in_order_then_clean_publish(self):
        for gate in ("basis_review", "case_review", "publish_approval"):
            self.assertEqual(self.run_c("h"), 3)
            self.assertEqual(cli.main(["approve", "--campaign", "h", "--gate", gate, "--by", "tester"]), 0)
        self.assertEqual(self.run_c("h"), 0)
        summary = json.loads((campaign.DESIGN_DIR / "h" / "05_publish" / "publish_summary.json").read_text(encoding="utf-8"))
        self.assertEqual(summary["created"], [])                     # the reference passes: nothing to the tracker
        res = json.loads((campaign.DESIGN_DIR / "h" / "04_triage" / "case_results.json").read_text(encoding="utf-8"))
        self.assertEqual({k: v["verdict"] for k, v in res.items()}["SR-01-FI-01"], "PASS")
        self.assertEqual(res["SR-01-BVA-03"]["verdict"], "INFO")
        self.assertEqual(res["SR-01-BVA-03"]["peak_state"], "STOP_IN_LANE")   # the probe: a 99 ms loss already stops
        report = (campaign.DESIGN_DIR / "h" / "05_publish" / "verification_report.md").read_text(encoding="utf-8")
        self.assertIn("| basis_review | approved | tester |", report)

    def test_approve_needs_a_pending_gate(self):
        self.assertEqual(self.run_c("p"), 3)
        self.assertEqual(cli.main(["approve", "--campaign", "p", "--gate", "publish_approval", "--by", "tester"]), 2)

    def test_seeded_bug_is_caught_then_deduplicated(self):
        self.assertEqual(self.run_c("m1", defects=["long_timeout"], auto_approve=True), 0)
        self.assertEqual(self.run_c("m2", defects=["long_timeout"], auto_approve=True), 0)
        t = json.loads((campaign.DESIGN_DIR / "tracker.json").read_text(encoding="utf-8"))["issues"]
        self.assertEqual(list(t), ["SSB-1"])                         # one defect, the second run commented on it
        self.assertEqual(len(t["SSB-1"]["comments"]), 1)
        self.assertEqual(t["SSB-1"]["labels"], ["ssb", "defect", "SR-01", "sig-link_lost", "mode-detected-within-ftti",
                                                "mode-reaction-stop-in-lane", "seeded"])
        self.assertIn("FTTI 250 ms", t["SSB-1"]["title"])
        self.assertIn("Seeded defect(s) in this run: long_timeout", t["SSB-1"]["description"])


# ---- v2.14: the LLM drafter, with a fake model (no network, no cost) ----------------------------------------------------

def sr02_draft(**over):
    """What gpt-4o-mini drafted for SR-02 on 2026-10-08 (after the alternatives list was added to the prompt)."""
    d = {"kind": "event", "reaction": "STOP_IN_LANE", "not_injectable": [], "assumptions": [], "open_questions": [],
         "triggers": [
             {"covers": "bad CRC", "fault": "crc_corrupt", "params": {}, "cause": ["E2E_INVALID"], "threshold": None, "why": "x"},
             {"covers": "repeated", "fault": "counter_frozen", "params": {}, "cause": ["E2E_INVALID"], "threshold": None, "why": "x"},
             {"covers": "wrong sequence", "fault": "jitter", "params": {"ms": 30}, "cause": ["E2E_INVALID"], "threshold": None, "why": "x"},
             {"covers": "wrong data ID", "fault": "wrong_data_id", "params": {}, "cause": ["E2E_INVALID"], "threshold": None, "why": "x"}]}
    d.update(over)
    return d


class FakeLLM:
    def __init__(self, *answers):
        self.answers, self.calls, self.prompts = [json.dumps(a) if isinstance(a, dict) else a for a in answers], [], []

    def chat(self, messages):
        self.prompts.append(messages)
        self.calls.append({"provider": "fake", "model": "m", "cached": False, "tokens": {"in": 1, "out": 1}})
        return self.answers.pop(0)


class DrafterTests(unittest.TestCase):
    def setUp(self):
        from design import drafter
        self.d, self.cat, self.causes = drafter, drafter.fault_catalogue(), drafter.known_causes()
        self.text = REQS["SR-02"]["text"]

    def errs(self, draft, text=None):
        return self.d.validate_event(draft, text or self.text, self.cat, self.causes)

    def test_alternatives_found_by_code(self):
        self.assertEqual(self.d.alternatives(self.text), ["bad CRC", "repeated", "wrong sequence", "wrong data ID"])
        self.assertEqual(self.d.alternatives(REQS["SR-04"]["text"]), ["no kick > 60 ms", "3 early kicks", "3 wrong challenge answers"])
        self.assertEqual(self.d.alternatives(REQS["SR-05"]["text"]), [])          # no "→": a comma is not a list of triggers

    def test_good_draft_passes(self):
        self.assertEqual(self.errs(sr02_draft()), [])

    def test_unknown_fault_param_and_cause_rejected(self):
        bad = sr02_draft()
        bad["triggers"][0] = {**bad["triggers"][0], "fault": "laser_attack"}
        bad["triggers"][1] = {**bad["triggers"][1], "params": {"n": 3}, "cause": ["MADE_UP"]}
        e = " | ".join(self.errs(bad))
        self.assertIn("'laser_attack' is not in the catalogue", e)
        self.assertIn("params ['n'] are not accepted by counter_frozen", e)
        self.assertIn("cause ['MADE_UP']", e)

    def test_every_alternative_must_be_covered_once(self):
        bad = sr02_draft()
        bad["triggers"] = bad["triggers"][:3]
        self.assertIn("alternative 'wrong data ID' is neither covered", " | ".join(self.errs(bad)))
        bad = sr02_draft()
        bad["triggers"][0] = {**bad["triggers"][0], "covers": "E2E window INVALID (bad CRC, repeated)"}
        self.assertIn("must name exactly one", " | ".join(self.errs(bad)))

    def test_same_fault_for_two_alternatives_rejected(self):
        # gpt-4o-mini's first SR-04 draft: "no kick > 60 ms" and "3 early kicks" both as kick_fast
        bad = {"kind": "event", "reaction": "STOP_IN_LANE", "assumptions": ["3 early kicks and 3 wrong answers are counted by the controller"],
               "triggers": [{"covers": "no kick > 60 ms", "fault": "kick_fast", "params": {}, "cause": ["WATCHDOG_EARLY"]},
                            {"covers": "3 early kicks", "fault": "kick_fast", "params": {}, "cause": ["WATCHDOG_EARLY"]},
                            {"covers": "3 wrong challenge answers", "fault": "qa_wrong", "params": {}, "cause": ["WATCHDOG_QA"]}]}
        e = " | ".join(self.errs(bad, REQS["SR-04"]["text"]))
        self.assertIn("use the same fault and parameters", e)
        self.assertIn("the number 60", e)                                   # neither a threshold nor an assumption

    def test_a_tolerated_bench_example_cannot_be_a_stop_trigger(self):
        bad = sr02_draft()
        bad["triggers"][2] = {**bad["triggers"][2], "params": {"ms": 15}}  # SC-37: 15 ms jitter must NOT stop the vehicle
        self.assertIn("TOLERATE", " | ".join(self.errs(bad)))

    def test_repair_round_then_basis(self):
        bad = sr02_draft()
        bad["triggers"] = bad["triggers"][:3]
        llm = FakeLLM(bad, sr02_draft())
        b = basis.draft("SR-02", REQS, CFG, llm=llm)
        self.assertEqual(b["kind"], "event")
        self.assertEqual(b["drafted_by"], "llm fake:m (after 1 repair round(s))")
        self.assertIn("wrong data ID", llm.prompts[1][-1]["content"])     # the errors went back to the model

    def test_draft_that_never_validates_is_invalid(self):
        bad = sr02_draft(reaction="EXPLODE")
        b = basis.draft("SR-02", REQS, CFG, llm=FakeLLM(bad, bad, bad))
        self.assertEqual(b["kind"], "invalid")

    def test_reviewer_feedback_reaches_the_prompt(self):
        llm = FakeLLM(sr02_draft())
        basis.draft("SR-02", REQS, CFG, llm=llm, feedback="- 'wrong sequence' must use reorder, not jitter")
        self.assertIn("must use reorder, not jitter", llm.prompts[0][-1]["content"])

    def test_bench_timing_inherited_and_pattern_variant_added(self):
        b = basis.draft("SR-02", REQS, CFG, llm=FakeLLM(sr02_draft()))
        jit = next(t for t in b["triggers"] if t["fault"] == "jitter")
        self.assertEqual(jit["bench_timing"]["from"], "SC-37b")
        cases = {c["id"]: c for c in derive.derive(b, CFG)}
        self.assertTrue(cases["SR-02-FI-03"]["scenario"]["no_ftti"])       # reorder happens at a random moment
        self.assertEqual(cases["SR-02-FI-03"]["scenario"]["inject_ms"], 0)
        pv = cases["SR-02-PV-01"]["scenario"]                              # SC-05: every other frame corrupted
        self.assertEqual((pv["faults"][0]["type"], pv["faults"][0]["off_ms"], pv["faults"][0]["on_ms"]), ("crc_corrupt", 20, 20))

    def test_cached_answer_needs_no_key(self):
        import hashlib

        from design.llm import Client
        with tempfile.TemporaryDirectory() as tmp:
            msgs = [{"role": "user", "content": "hi"}]
            key = hashlib.sha256(json.dumps(["claude-sonnet-5-5", "medium", msgs], sort_keys=True).encode()).hexdigest()
            Path(tmp, f"{key}.json").write_text(json.dumps({"text": "{}", "tokens": None}), encoding="utf-8")
            c = Client("subscription", {}, Path(tmp))
            self.assertEqual(c.chat(msgs), "{}")
            self.assertTrue(c.calls[0]["cached"])


class FakeSdk:
    """Stands in for anthropic.Anthropic: records the request, returns a canned Sonnet-style answer."""

    def __init__(self, text='```json\n{"kind": "event"}\n```', stop_reason="end_turn"):
        from types import SimpleNamespace
        self.requests = []
        self.reply = SimpleNamespace(stop_reason=stop_reason, stop_details=SimpleNamespace(category="cyber"),
                                     content=[SimpleNamespace(type="thinking", thinking=""), SimpleNamespace(type="text", text=text)],
                                     usage=SimpleNamespace(input_tokens=100, output_tokens=20))
        self.messages = SimpleNamespace(create=self._create)

    def _create(self, **kw):
        self.requests.append(kw)
        return self.reply


class SonnetClientTests(unittest.TestCase):
    """Claude Sonnet 5.5 at medium effort is the only model (2026-10-09)."""

    def test_request_shape(self):
        from design.llm import Client
        sdk = FakeSdk()
        with tempfile.TemporaryDirectory() as tmp:
            c = Client("api", {}, Path(tmp), sdk=sdk)
            out = c.chat([{"role": "system", "content": "be exact"}, {"role": "user", "content": "q"},
                          {"role": "assistant", "content": "a"}, {"role": "user", "content": "fix it"}])
            self.assertEqual(out, '```json\n{"kind": "event"}\n```')
            r = sdk.requests[0]
            self.assertEqual((r["model"], r["output_config"], r["system"]), ("claude-sonnet-5-5", {"effort": "medium"}, "be exact"))
            self.assertNotIn("temperature", r)                      # a non-default value is a 400 on Sonnet 5.5
            self.assertEqual([m["role"] for m in r["messages"]], ["user", "assistant", "user"])   # ends on a user turn: no prefill
            c.chat([{"role": "system", "content": "be exact"}, {"role": "user", "content": "q"},
                    {"role": "assistant", "content": "a"}, {"role": "user", "content": "fix it"}])
            self.assertEqual(len(sdk.requests), 1)                  # the second identical call came from the cache
            self.assertEqual(c.calls[0]["effort"], "medium")

    def test_no_key_and_no_silent_fallback(self):
        from design.llm import Client, LLMUnavailable
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(LLMUnavailable, "ANTHROPIC_API_KEY"):
                Client("api", {}, Path(tmp)).chat([{"role": "user", "content": "x"}])
            with self.assertRaises(ValueError):
                Client("paid", {}, Path(tmp))                       # the old Groq / OpenAI / Ollama plans are gone

    def test_refusal_and_truncation_fail_loudly(self):
        from design.llm import Client, LLMUnavailable
        for reason in ("refusal", "max_tokens"):
            with tempfile.TemporaryDirectory() as tmp, self.assertRaises(LLMUnavailable):
                Client("api", {}, Path(tmp), sdk=FakeSdk(stop_reason=reason)).chat([{"role": "user", "content": "x"}])

    def test_json_in_a_code_fence_is_accepted_by_the_drafter(self):
        from design import drafter
        fenced = "Here is the basis:\n```json\n" + json.dumps(sr02_draft()) + "\n```\nDone."
        b = basis.draft("SR-02", REQS, CFG, llm=FakeLLM(fenced))
        self.assertEqual(b["kind"], "event")
        self.assertTrue(drafter._json_text("no json here") == "no json here")

    def test_agent_approval_is_labelled_and_not_reused_by_a_human_gated_run(self):
        from design.gates import AGENT_AUTO, AUTONOMOUS_APPROVER
        with tempfile.TemporaryDirectory() as tmp:
            g = HumanGate(Path(tmp), auto_approve=True, auto_actor=AGENT_AUTO + AUTONOMOUS_APPROVER)
            rec = g.require("basis_review", {"a": 1})
            self.assertTrue(rec["approver"].startswith("agent-auto:claude on Ketan's behalf"))
            self.assertIn("no human review", rec["comment"])
            with self.assertRaises(GatePending):                    # human gates back on: the agent's approval does not count
                HumanGate(Path(tmp)).require("basis_review", {"a": 1})


class DrafterCampaignTests(CampaignTests):
    """SR-02 through the whole chain with the fake model."""

    def test_human_gates_in_order_then_clean_publish(self):   # covered for SR-01 above
        pass

    def test_approve_needs_a_pending_gate(self):
        pass

    def test_seeded_bug_is_caught_then_deduplicated(self):
        pass

    def run02(self, name, defects=()):
        return campaign.run(name, ["SR-02"], defects=list(defects), auto_approve=True, llm_client=FakeLLM(sr02_draft()), log=lambda *a: None)

    def test_reference_passes_and_bugs_get_one_ticket_per_failure_mode(self):
        self.assertEqual(self.run02("ref"), 0)
        self.assertFalse((campaign.DESIGN_DIR / "tracker.json").exists())          # reference: nothing published
        self.assertEqual(self.run02("len", ["e2e_lenient"]), 0)
        self.assertEqual(self.run02("nolatch", ["no_latch"]), 0)
        t = json.loads((campaign.DESIGN_DIR / "tracker.json").read_text(encoding="utf-8"))["issues"]
        modes = {k: [x for x in v["labels"] if x.startswith("mode-")] for k, v in t.items()}
        # e2e_lenient: CRC / counter / data ID report TIMEOUT, intermittent CRC and jitter don't stop: one bug, ONE ticket.
        # no_latch fails differently (the stop releases): its own ticket, NOT a comment on the lenient one
        self.assertEqual(len(t), 2, modes)
        self.assertTrue(all(not v["comments"] for v in t.values()))
        lenient = next(v for v in t.values() if "mode-cause-in" in v["labels"])
        for s in ("sig-crc_corrupt", "sig-counter_frozen", "sig-wrong_data_id", "sig-jitter"):
            self.assertIn(s, lenient["labels"])
        self.assertEqual(self.run02("len2", ["e2e_lenient"]), 0)                   # recurrence: a comment, no new ticket
        t = json.loads((campaign.DESIGN_DIR / "tracker.json").read_text(encoding="utf-8"))["issues"]
        self.assertEqual(len(t), 2)
        self.assertEqual(sum(len(v["comments"]) for v in t.values()), 1)


if __name__ == "__main__":
    unittest.main()


class FakeGitHubApi:
    """Behaves like the bits of the GitHub REST API the tracker uses (pull requests show up in the issues list too)."""

    def __init__(self):
        self.issues, self.comments = [], []

    def __call__(self, method, path, body=None):
        from urllib.parse import parse_qs, unquote, urlparse
        u = urlparse(path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        if method == "POST" and u.path.endswith("/issues"):
            self.issues.append({"number": len(self.issues) + 1, "title": body["title"], "body": body["body"], "state": "open",
                                "labels": [{"name": x} for x in body["labels"]], "html_url": f"https://github.com/o/n/issues/{len(self.issues) + 1}"})
            return self.issues[-1]
        if method == "POST" and u.path.endswith("/comments"):
            self.comments.append((int(u.path.split("/")[-2]), body["body"]))
            return {"id": 1}
        if method == "GET" and u.path.endswith("/issues"):
            want = set(unquote(q["labels"]).split(","))
            rows = [i for i in self.issues if i["state"] == "open" and want <= {x["name"] for x in i["labels"]}]
            return rows + [{"number": 99, "title": "a PR", "pull_request": {}, "labels": [{"name": x} for x in want]}]
        raise AssertionError((method, path))


class GitHubTrackerTests(unittest.TestCase):
    def test_files_finds_and_comments(self):
        from design.tracker import GitHubError, GitHubTracker
        gh = FakeGitHubApi()
        t = GitHubTracker("o/n", api=gh)
        made = t.create_issue("SR-01: late", "details", ["ssb", "defect", "SR-01", "sig-link_lost", "mode-x" * 20, "a,b"])
        self.assertEqual((made["key"], made["url"]), ("GH-1", "https://github.com/o/n/issues/1"))
        self.assertTrue(all(len(x["name"]) <= 50 and "," not in x["name"] for x in gh.issues[0]["labels"]))
        self.assertEqual([d["key"] for d in t.open_defects("SR-01")], ["GH-1"])          # the pull request is ignored
        self.assertEqual(t.open_defects("SR-02"), [])
        self.assertEqual(t.add_comment("GH-1", "again")["key"], "GH-1")
        self.assertEqual(gh.comments, [(1, "again")])
        for bad in ("nope", "a/b/c", ""):
            with self.assertRaises(GitHubError):
                GitHubTracker(bad)
        with self.assertRaises(GitHubError):
            t.add_comment("KAN-8", "x")

    def test_campaign_files_the_seeded_bug_to_github_then_comments(self):
        import tempfile as tf
        from unittest import mock
        gh = FakeGitHubApi()
        with tf.TemporaryDirectory() as tmp, mock.patch("design.tracker.gh_api", gh):
            saved, campaign.DESIGN_DIR = campaign.DESIGN_DIR, Path(tmp)
            try:
                kw = dict(defects=["long_timeout"], auto_approve=True, tracker_kind="github", github_repo="o/n", log=lambda *a: None)
                self.assertEqual(campaign.run("g1", ["SR-01"], **kw), 0)
                self.assertEqual(campaign.run("g2", ["SR-01"], **kw), 0)
                self.assertFalse((Path(tmp) / "tracker.json").exists())                   # nothing went to the local file
            finally:
                campaign.DESIGN_DIR = saved
        self.assertEqual(len(gh.issues), 1)                                               # one defect, the rerun commented
        self.assertEqual([c[0] for c in gh.comments], [1])
        labels = {x["name"] for x in gh.issues[0]["labels"]}
        self.assertTrue({"ssb", "defect", "SR-01", "sig-link_lost", "seeded"} <= labels)

    def test_bad_repo_stops_before_any_bench_time(self):
        with tempfile.TemporaryDirectory() as tmp:
            saved, campaign.DESIGN_DIR = campaign.DESIGN_DIR, Path(tmp)
            try:
                self.assertEqual(campaign.run("bad", ["SR-01"], auto_approve=True, tracker_kind="github", github_repo="nope",
                                              log=lambda *a: None), 2)
                self.assertFalse((Path(tmp) / "bad" / "03_execute").exists())
            finally:
                campaign.DESIGN_DIR = saved


def sonnet_sr02_draft():
    """What Claude Sonnet 5.5 drafted for SR-02 on 2026-10-09: it maps 'wrong sequence' to ONE counter jump of 4, which this
    bench's window tolerates as a single error (SC-34), so the healthy controller keeps driving."""
    d = sr02_draft()
    d["triggers"][2] = {"covers": "wrong sequence", "fault": "counter_jump", "params": {"n": 4}, "cause": ["E2E_INVALID"],
                        "threshold": None, "why": "x"}
    d["assumptions"] = ["The value n=4 is assumed to exceed the window's maximum accepted delta; it must be verified on the bench."]
    return d


class SuspectCaseTests(unittest.TestCase):
    """Test the test: a case that fails on the known-good reference controller is a wrong case, not a defect."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.saved, campaign.DESIGN_DIR = campaign.DESIGN_DIR, Path(self.tmp.name)

    def tearDown(self):
        campaign.DESIGN_DIR = self.saved
        self.tmp.cleanup()

    def run02(self, name, defects=()):
        return campaign.run(name, ["SR-02"], defects=list(defects), auto_approve=True, llm_client=FakeLLM(sonnet_sr02_draft()),
                            log=lambda *a: None)

    def results(self, name):
        return json.loads((campaign.DESIGN_DIR / name / "04_triage" / "case_results.json").read_text(encoding="utf-8"))

    def test_wrong_case_on_the_healthy_controller_files_nothing(self):
        self.assertEqual(self.run02("ref"), 0)
        r = self.results("ref")
        self.assertEqual(r["SR-02-FI-03"]["verdict"], "SUSPECT")
        self.assertIn("known-good reference controller", r["SR-02-FI-03"]["why"])
        self.assertEqual(sum(v["verdict"] == "PASS" for v in r.values()), 6)
        self.assertFalse((campaign.DESIGN_DIR / "tracker.json").exists())              # no false defect
        report = (campaign.DESIGN_DIR / "ref" / "05_publish" / "verification_report.md").read_text(encoding="utf-8")
        self.assertIn("## Suspect cases (not filed as defects)", report)
        self.assertIn("must be verified on the bench", report)                         # the drafter's own doubt is surfaced

    def test_a_real_bug_is_still_filed_and_the_suspect_case_stays_out_of_it(self):
        self.assertEqual(self.run02("bug", ["e2e_lenient"]), 0)
        r = self.results("bug")
        self.assertEqual(r["SR-02-FI-03"]["verdict"], "SUSPECT")                       # fails on the reference too
        self.assertTrue(any(v["verdict"] == "FAIL" for v in r.values()))               # the seeded bug is caught by other cases
        issues = json.loads((campaign.DESIGN_DIR / "tracker.json").read_text(encoding="utf-8"))["issues"]
        self.assertEqual(len(issues), 1)
        self.assertNotIn("SR-02-FI-03", next(iter(issues.values()))["description"])
        self.assertTrue((campaign.DESIGN_DIR / "bug" / "03_execute" / "golden" / "golden_results.json").exists())

    def test_triage_marks_only_failures_that_the_reference_shares(self):
        sc = {"key": "k1"}
        case = {"id": "R-1", "scenario": sc, "probe": False, "blocked": None}
        res = {"key": "k1", "cause": None, "verdict": {"status": "FAIL", "peak_state": "NORMAL", "checks": [("c", False)]}}
        self.assertEqual(campaign.triage([case], [res], {"k1"})["R-1"]["verdict"], "SUSPECT")
        self.assertEqual(campaign.triage([case], [res], set())["R-1"]["verdict"], "FAIL")
        res["verdict"]["status"] = "PASS"
        self.assertEqual(campaign.triage([case], [res], {"k1"})["R-1"]["verdict"], "PASS")


class SubscriptionRouteTests(unittest.TestCase):
    """Sonnet 5.5 through the logged-in Claude plan (claude -p): no API key may ever reach it, and nothing falls back to the API."""

    @staticmethod
    def ok(text='{"kind": "event"}'):
        return json.dumps({"type": "result", "is_error": False, "result": text, "total_cost_usd": 0.03,
                           "usage": {"input_tokens": 700, "output_tokens": 250, "cache_read_input_tokens": 50}})

    def client(self, tmp, stdout, rc=0, stderr="", seen=None):
        from types import SimpleNamespace

        from design.llm import Client

        def run(cmd, **kw):
            if seen is not None:
                seen.append((cmd, kw))
            return SimpleNamespace(returncode=rc, stdout=stdout, stderr=stderr)
        return Client("subscription", {"ANTHROPIC_API_KEY": "sk-ant-api03-must-not-be-used"}, Path(tmp), runner=run)

    def test_request_shape_and_clean_environment(self):
        from unittest import mock
        seen = []
        env = {"ANTHROPIC_API_KEY": "sk-ant-api03-x", "CLAUDE_CODE_SESSION_ID": "enclosing", "PATH": "p", "CLAUDE_CODE_BIN": __file__}
        with tempfile.TemporaryDirectory() as tmp, mock.patch.dict("os.environ", env, clear=False):
            c = self.client(tmp, self.ok(), seen=seen)
            out = c.chat([{"role": "system", "content": "be exact"}, {"role": "user", "content": "q"}])
            cmd, kw = seen[0]
            self.assertEqual(out, '{"kind": "event"}')
            self.assertEqual(cmd[1:9], ["-p", "--model", "claude-sonnet-5-5", "--effort", "medium", "--tools", "", "--no-session-persistence"])
            self.assertEqual(cmd[cmd.index("--system-prompt") + 1], "be exact")
            self.assertNotIn("--bare", cmd)                                     # --bare would never read the plan login
            self.assertEqual(kw["input"], "q")
            self.assertFalse(any(k.startswith(("ANTHROPIC_", "CLAUDE_CODE_")) for k in kw["env"]))
            self.assertIn("ssb_claude_", kw["cwd"])
            self.assertEqual(c.calls[0]["billed_to"], "claude plan (subscription usage)")
            self.assertEqual(c.calls[0]["tokens"], {"in": 750, "out": 250})
            self.assertEqual(c.calls[0]["api_equivalent_usd"], 0.03)

    def test_a_repair_round_is_sent_as_a_transcript(self):
        from unittest import mock
        seen = []
        with tempfile.TemporaryDirectory() as tmp, mock.patch.dict("os.environ", {"CLAUDE_CODE_BIN": __file__}):
            self.client(tmp, self.ok(), seen=seen).chat([{"role": "system", "content": "s"}, {"role": "user", "content": "ask"},
                                                         {"role": "assistant", "content": "bad draft"}, {"role": "user", "content": "fix it"}])
        prompt = seen[0][1]["input"]
        self.assertIn("[USER]\nask", prompt)
        self.assertIn("[ASSISTANT]\nbad draft", prompt)
        self.assertTrue(prompt.rstrip().endswith("[ASSISTANT]"))

    def test_failures_stop_the_run_and_never_fall_back_to_the_api(self):
        from unittest import mock

        from design.llm import LLMUnavailable
        cases = [('{"is_error": true, "result": "Not logged in · Please run /login"}', 1, "", "claude auth login"),
                 ('{"is_error": true, "result": "Claude AI usage limit reached"}', 1, "", "usage limit"),
                 ("", 1, "authentication_error 401", "claude auth login"),
                 ('{"is_error": true, "result": "boom"}', 1, "", "boom")]
        for stdout, rc, stderr, needle in cases:
            with tempfile.TemporaryDirectory() as tmp, mock.patch.dict("os.environ", {"CLAUDE_CODE_BIN": __file__}):
                with self.assertRaisesRegex(LLMUnavailable, needle):
                    self.client(tmp, stdout, rc, stderr).chat([{"role": "user", "content": "x"}])

    def test_both_routes_share_one_cache(self):
        from unittest import mock
        with tempfile.TemporaryDirectory() as tmp, mock.patch.dict("os.environ", {"CLAUDE_CODE_BIN": __file__}):
            msgs = [{"role": "user", "content": "same prompt"}]
            self.client(tmp, self.ok("answer")).chat(msgs)
            from design.llm import Client
            api = Client("api", {}, Path(tmp))                                   # no key, no sdk: only the cache can answer
            self.assertEqual(api.chat(msgs), "answer")
            self.assertTrue(api.calls[0]["cached"])

    def test_binary_discovery_prefers_the_newest_desktop_copy(self):
        from unittest import mock

        from design.llm import claude_code_binary
        with tempfile.TemporaryDirectory() as tmp:
            for v in ("2.1.289", "2.1.293", "2.1.30"):
                d = Path(tmp) / "Claude" / "claude-code" / v / "abc"
                d.mkdir(parents=True)
                (d / "claude.exe").write_text("x")
            with mock.patch.dict("os.environ", {"APPDATA": tmp, "CLAUDE_CODE_BIN": ""}), mock.patch("shutil.which", lambda n: None), mock.patch(
                    "pathlib.Path.exists", lambda self: False):   # not the shared D:/Agents/tools copy
                self.assertIn("2.1.293", claude_code_binary() or "")


class AutoRedraftTests(unittest.TestCase):
    """The agent as reviewer: SUSPECT cases from an LLM-drafted basis are rejected with the facts, and the model drafts again."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.saved, campaign.DESIGN_DIR = campaign.DESIGN_DIR, Path(self.tmp.name)
        self.actor = "agent-auto:claude on Ketan's behalf (test)"

    def tearDown(self):
        campaign.DESIGN_DIR = self.saved
        self.tmp.cleanup()

    def run02(self, name, llm, redraft):
        return campaign.run(name, ["SR-02"], auto_approve=True, auto_actor=self.actor, llm_client=llm, redraft=redraft, log=lambda *a: None)

    def case_results(self, name):
        return json.loads((campaign.DESIGN_DIR / name / "04_triage" / "case_results.json").read_text(encoding="utf-8"))

    def test_a_wrong_fault_is_corrected_by_one_redraft(self):
        llm = FakeLLM(sonnet_sr02_draft(), sr02_draft())                  # the real wrong draft, then a right one (jitter 30)
        self.assertEqual(self.run02("fix", llm, redraft=1), 0)
        self.assertEqual(len(llm.prompts), 2)
        ask = llm.prompts[1][-1]["content"]
        self.assertIn("A reviewer REJECTED an earlier draft", ask)
        self.assertIn("KNOWN-GOOD reference controller", ask)
        self.assertIn("alternative 'wrong sequence' (fault counter_jump {'n': 4})", ask)
        self.assertIn("SC-33 {'n': 2} -> NO reaction, tolerated", ask)    # the bench's own evidence, not a model's opinion
        self.assertTrue(all(v["verdict"] != "SUSPECT" for v in self.case_results("fix").values()))
        log = [json.loads(line) for line in (campaign.DESIGN_DIR / "fix" / "01_basis" / "redraft_log.jsonl").read_text().splitlines()]
        self.assertEqual((len(log), log[0]["rejected_by"], log[0]["suspect_cases"]), (1, self.actor, ["SR-02-FI-03"]))
        gate = json.loads((campaign.DESIGN_DIR / "fix" / "gates" / "gates.json").read_text(encoding="utf-8"))["basis_review"]
        self.assertIn("counter_jump", gate["feedback"])                    # the rejection is on the record, like a person's
        self.assertFalse((campaign.DESIGN_DIR / "tracker.json").exists())

    def test_one_round_only_then_the_suspect_case_is_reported_not_filed(self):
        llm = FakeLLM(sonnet_sr02_draft(), sonnet_sr02_draft())          # the model repeats its mistake; a 3rd call would raise
        self.assertEqual(self.run02("stuck", llm, redraft=1), 0)
        self.assertEqual(len(llm.prompts), 2)
        self.assertEqual(self.case_results("stuck")["SR-02-FI-03"]["verdict"], "SUSPECT")
        self.assertFalse((campaign.DESIGN_DIR / "tracker.json").exists())
        report = (campaign.DESIGN_DIR / "stuck" / "05_publish" / "verification_report.md").read_text(encoding="utf-8")
        self.assertIn("## Suspect cases (not filed as defects)", report)

    def test_without_redraft_nothing_changes(self):
        llm = FakeLLM(sonnet_sr02_draft())
        self.assertEqual(self.run02("none", llm, redraft=0), 0)
        self.assertEqual(len(llm.prompts), 1)
        self.assertFalse((campaign.DESIGN_DIR / "none" / "01_basis" / "redraft_log.jsonl").exists())

    def test_reason_names_only_llm_drafted_requirements(self):
        cases = [{"id": "SR-01-FI-01", "req": "SR-01", "scenario": {"faults": [{"type": "link_lost", "start": 2000}],
                                                                    "expect": {"reaction": "STOP_IN_LANE", "cause": ["TIMEOUT"]}},
                  "coverage_item": "x"}]
        res = {"SR-01-FI-01": {"verdict": "SUSPECT", "peak_state": "NORMAL"}}
        self.assertEqual(campaign.redraft_reason([{"req": "SR-01", "drafted_by": "rules"}], cases, res), "")   # rules cannot redraft


class BenchHealthTests(unittest.TestCase):
    """P2: a failure on a doubtful bench is SUSPECT BENCH, never a defect."""

    INV = {"board_b": {"firmware": "SafetyNode 2.6"}, "board_a": {"firmware": "BusNode 2.6"}}

    def test_stale_safetynode_is_named(self):
        from health.check import firmware_problems
        fw = {"board_b": "SafetyNode 2.5 (B) CAN 8MHz", "board_a": "BusNode 2.6 (A) CAN 8MHz"}
        out = firmware_problems(fw, "hil", self.INV)
        self.assertEqual(len(out), 1)
        self.assertIn("SafetyNode 2.5 is not the inventory's SafetyNode 2.6", out[0])

    def test_matching_firmware_and_non_hil_levels_are_healthy(self):
        from health.check import firmware_problems
        fw = {"board_b": "SafetyNode 2.6 (B) CAN 8MHz", "board_a": "BusNode 2.6 (A) CAN 8MHz"}
        self.assertEqual(firmware_problems(fw, "hil", self.INV), [])
        self.assertEqual(firmware_problems(None, "reference", self.INV), [])

    def test_silent_board_is_suspect(self):
        from health.check import firmware_problems
        out = firmware_problems({"board_b": "SafetyNode 2.6", "board_a": "?"}, "hil", self.INV)
        self.assertTrue(any("board_a: no firmware hello" in x for x in out))

    def test_triage_marks_failures_suspect_bench_but_keeps_passes(self):
        from design.campaign import triage
        cases = [{"id": f"C{i}", "scenario": {"key": f"K{i}"}, "probe": False, "blocked": False} for i in (1, 2, 3)]
        mk = lambda k, st: {"key": k, "verdict": {"status": st, "peak_state": "NORMAL", "checks": [("c", st == "PASS")]}}
        results = [mk("K1", "FAIL"), mk("K2", "PASS"), mk("K3", "FAIL")]
        fleet = triage(cases, results, set(), {"fleet": ["board_b: SafetyNode 2.5 is not the inventory's SafetyNode 2.6"], "by_key": {}})
        self.assertEqual([fleet[f"C{i}"]["verdict"] for i in (1, 2, 3)], ["SUSPECT BENCH", "PASS", "SUSPECT BENCH"])
        self.assertIn("SafetyNode 2.5", fleet["C1"]["why"])
        one = triage(cases, results, set(), {"fleet": [], "by_key": {"K3": ["host stall: bench lag 40 ms"]}})
        self.assertEqual([one[f"C{i}"]["verdict"] for i in (1, 2, 3)], ["FAIL", "PASS", "SUSPECT BENCH"])
