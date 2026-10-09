"""python -m design: requirement → test basis → cases → bench → defects, stopping at each human gate.

    python -m design run --req SR-01                          # reference controller; stops at basis_review (exit 3)
    python -m design approve --campaign <c> --gate basis_review --by "<name>"
    python -m design run --campaign <c> --req SR-01           # resumes: case_review, then the bench, then publish_approval
    python -m design run --campaign <c> --req SR-01 --level hil -- --port-b COM13 --port-a COM14 --kick gpio
    python -m design run --campaign <c> --req SR-01 --defect long_timeout     # seeded bug: the chain must catch it
    python -m design run ... --env-file <path to .env>                        # ANTHROPIC_API_KEY (+ ANTHROPIC_WORKSPACE_ID) from a file
    python -m design status --campaign <c>

Exit codes: 0 done, 2 requirement not usable, 3 waiting for a human at a gate.
"""
from __future__ import annotations

import argparse
import sys
from datetime import date

from ssb.safety import MUTANTS

from . import campaign
from .gates import AGENT_AUTO, AUTONOMOUS_APPROVER, HumanGate


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    bench_args: list[str] = []
    if "--" in argv:
        i = argv.index("--")
        argv, bench_args = argv[:i], argv[i + 1:]
    ap = argparse.ArgumentParser(prog="python -m design", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--req", action="append", required=True)
    r.add_argument("--campaign")
    r.add_argument("--level", default="reference", help="run.py --dut: reference, fmu, can, native, loopback, pil, hil, ros2")
    r.add_argument("--defect", action="append", default=[], choices=sorted(MUTANTS))
    r.add_argument("--config", default="config/default.json")
    r.add_argument("--env-file", help="file with ANTHROPIC_API_KEY (and ANTHROPIC_WORKSPACE_ID): read, never copied")
    r.add_argument("--llm", choices=["subscription", "api", "off"], default="subscription",
                   help="drafter for wording the rules cannot parse: Claude Sonnet 5.5 at medium effort (the only model) through your Claude "
                        "plan (subscription, default; needs `claude auth login`) or the billed API (api; needs ANTHROPIC_API_KEY), or off = rules only")
    r.add_argument("--tracker", choices=["local", "github"], default="local",
                   help="where defects go after publish_approval: a local JSON file (default) or GitHub Issues (needs --github-repo and `gh auth login`)")
    r.add_argument("--github-repo", help="owner/name of the repository that holds the defects")
    r.add_argument("--redraft", type=int, default=None,
                   help="how many times the pipeline rejects its own LLM-drafted basis when cases turn out SUSPECT (they fail on the "
                        "known-good controller) and has the model draft again; default 1 with --autonomous, else 0")
    r.add_argument("--rerun", action="store_true", help="run the bench again even if these approved cases already ran on this level")
    r.add_argument("--auto-approve", action="store_true", help="tests only: gates approve themselves as 'demo-auto'")
    r.add_argument("--autonomous", action="store_true",
                   help="NO waiting at gates: the agent approves every gate on the owner's behalf (autonomy test; recorded as "
                        "agent-auto, never reused by a human-gated run)")
    for name in ("approve", "reject"):
        g = sub.add_parser(name)
        g.add_argument("--campaign", required=True)
        g.add_argument("--gate", required=True, choices=HumanGate.ALL)
        g.add_argument("--by", required=True)
        g.add_argument("--comment", default="")
        if name == "reject":
            g.add_argument("--reason", required=True)
    s = sub.add_parser("status")
    s.add_argument("--campaign", required=True)
    a = ap.parse_args(argv)

    if a.cmd == "run":
        name = a.campaign or f"{date.today():%Y-%m-%d}_{'_'.join(x.lower().replace('-', '') for x in a.req)}_{a.level}"
        print(f"campaign {name}")
        return campaign.run(name, a.req, a.level, a.defect, a.config, bench_args, a.env_file, a.auto_approve or a.autonomous,
                            a.rerun, a.llm, auto_actor=AGENT_AUTO + AUTONOMOUS_APPROVER if a.autonomous else "demo-auto",
                            tracker_kind=a.tracker, github_repo=a.github_repo,
                            redraft=(1 if a.autonomous else 0) if a.redraft is None else a.redraft)
    gates = HumanGate(campaign.DESIGN_DIR / a.campaign, approver=getattr(a, "by", "cli"))
    if not gates.store_path.exists():
        print(f"no campaign {a.campaign} in {campaign.DESIGN_DIR}")
        return 2
    if a.cmd == "approve":
        rec = gates.load().get(a.gate, {})
        if rec.get("decision") != "pending":
            print(f"gate {a.gate} is not pending (it is {rec.get('decision', 'not reached')}); run the campaign first")
            return 2
        gates.approve(a.gate, a.by, a.comment)
        print(f"approved {a.gate} (sha256 {rec['artifact_hash'][:16]}); re-run the campaign to continue")
    elif a.cmd == "reject":
        gates.reject(a.gate, a.by, a.reason)
        print(f"rejected {a.gate}: {a.reason}")
    else:
        for k in HumanGate.ALL:
            rec = gates.load().get(k, {})
            print(f"  {k:17} {rec.get('decision', '-'):9} {rec.get('approver', ''):12} {str(rec.get('at', ''))[:19]}  {rec.get('review_packet', '')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
