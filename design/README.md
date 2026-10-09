# design/: requirement → test basis → cases → bench → defects (v2.16)

The test-design front end. It takes a requirement from `safety/hazards.json`, works out what has to be tested, runs the
cases on any bench level through `run.py`, and turns failures into defect tickets. A human approves three times on the way.
Ported from the HIL ML Ops pipeline (its gates module is copied as is; its fault-injection and boundary rules are re-applied
to this bench's timed faults).

**Rule: the model drafts, code checks and decides, a human approves.** The bench oracle (`ssb/oracle.py`) gives every
PASS / FAIL; no model is in the verdict path. The test basis is drafted by rules where the wording is regular (SR-01), and
by an LLM otherwise (SR-02, SR-04); both go through the same code checks and the same gate.

## The LLM drafter (`drafter.py`, `llm.py`)

The model gets the requirement, the alternatives code found in it, and a fault catalogue built from the bench's own
scenarios (fault types, the parameters each accepts, real examples with their expected reaction and cause). Code then
rejects a draft that:

- leaves an alternative uncovered, or covers two in one trigger (it may declare one not injectable: a blocked GAP case)
- uses a fault or parameter the bench doesn't have, or a cause or reaction the oracle doesn't know
- maps two different alternatives to the same fault and parameters (gpt-4o-mini's first SR-04 draft: "no kick" and
  "3 early kicks" both as `kick_fast`)
- makes a stop trigger out of a scenario the bench must tolerate (15 ms jitter, SC-37)
- drops a number of the requirement, or invents a threshold that is not in it

Errors go back to the model for up to 2 repair rounds; a draft that still fails never reaches a gate. A rejection at
`basis_review` goes into the next draft as reviewer feedback. Code, not the model, owns timing: a trigger equal to a bench
scenario inherits its timing (SC-37b, jitter: no FTTI from a fixed injection time), and derivation adds every other
pattern of the same fault that the bench expects to stop (SC-05: every other frame corrupted).

**Model: Claude Sonnet 5.5 at medium effort is the only model** (`llm.py`), reached two ways. `--llm subscription` (default) uses
the Claude Code CLI signed in to your Claude plan (`claude auth login`, once): the calls count against the plan's usage limits,
no per-token charge. `--llm api` uses the Anthropic API (`ANTHROPIC_API_KEY`, plus `ANTHROPIC_WORKSPACE_ID` for a user-scoped
key; `pip install -e .[llm]`) and is billed per token to the **API account**, a separate wallet from the subscription.
`--llm off` uses rules only. There is **no fallback between the routes** (a silent one would spend API money): not logged in,
usage limit, no key, 401, refusal or truncation stop the run with the reason. The subscription route runs the CLI without any
`ANTHROPIC_*` variable (an API key in its environment would be billed), in a neutral directory, without tools or a saved session,
and never with `--bare` (which ignores the plan login). Both routes send no `temperature` (a non-default value is a 400 on
Sonnet 5.5). Every answer is cached in `reports/design/.llm_cache/` (one cache for both routes, keyed by model + effort +
prompt), so a re-run reproduces the approved draft at no cost; calls are logged per campaign in `01_basis/llm_calls.jsonl`.
Older drafts came from gpt-4o-mini, Groq and Ollama; those routes are gone.

**What code cannot check is why the gate exists.** A small model's validated SR-04 draft (gpt-4o-mini, 2026-10-08) mapped
"no kick > 60 ms" to frame `latency` (that tests SR-03's stale-data rule, so the cases would pass while testing the wrong
thing); "no kick" is a hung planner (`hang_comms_alive`, cause WATCHDOG_LATE). And before timing was inherited, a validated
SR-02 draft produced a false defect on the healthy controller: the jitter case was timed from a fixed injection moment.
Whether Sonnet 5.5 makes the same SR-04 mistake is not measured yet (no key on this machine when this was written).

**Measured with the cached gpt-4o-mini drafts, before the switch** (all 14 seeded bugs, reference controller): the SR-01
cases (rules) catch `long_timeout` and `no_latch`; the SR-02 cases catch `e2e_lenient`, `e2e_run_length` (only with the SC-05
pattern case) and `no_latch`: every bug in each requirement's own mechanism, 0 failures on the healthy controller. Re-measure
with Sonnet drafts.

**Test the test (v2.18).** After the bench run, the same cases run once on the known-good reference controller; a case that
FAILS there is **SUSPECT** (the case or its basis is wrong), is listed in the report with the drafter's own doubts, and never
becomes a defect. Found by the first Sonnet autonomy run: for "wrong sequence" Sonnet chose one `counter_jump` of 4, which
this bench's window tolerates as a single error, so the healthy controller kept driving and a false defect was filed. Sonnet
had written "must be verified on the bench" in its own assumptions; only a gate or this check can act on that. Sonnet's SR-04
draft, by contrast, mapped "no kick" correctly to `hang_comms_alive` (the mistake gpt-4o-mini made).

**The agent as reviewer (v2.19, `--redraft N`, default 1 with `--autonomous`).** When cases come back SUSPECT and their
requirement was drafted by the model, the pipeline rejects the basis at `basis_review` (recorded like a person's rejection) with
a reason made only of facts the code holds: the alternative, the fault and parameters, what the case expected, what the
known-good controller did, and every bench scenario that injects that fault with the reaction the bench expects. The model drafts
again, the gates run again, the bench runs again. It stops after N rounds (a model that repeats its mistake ends with the case
still SUSPECT and reported, never filed) and never redrafts a rules-drafted requirement.

**Autonomy test (2026-10-09).** `--autonomous` runs with no waiting at the gates: the agent approves each one on the owner's
behalf, recorded as `agent-auto:claude on Ketan's behalf (test 2026-10-09)` with the comment "no human review", and such an
approval is never reused by a run with human gates on. Used for SR-01 on the reference controller and on both HiL boards.

## Run it

```
python -m design run --req SR-01                       # stops at basis_review (exit 3) with a review packet
python -m design approve --campaign <c> --gate basis_review --by "<name>"
python -m design run --campaign <c> --req SR-01        # → case_review, then the bench, then publish_approval
python -m design run --campaign <c> --req SR-01 --level hil -- --port-b COM13 --port-a COM14 --kick gpio
python -m design run --campaign <c> --req SR-01 --defect long_timeout    # a seeded bug the chain must catch
python -m design status --campaign <c>
```

Everything goes under `reports/design/<campaign>/`: `01_basis/`, `02_design/` (spec + generated scenarios),
`03_execute/` (run.py output + run manifest), `04_triage/`, `05_publish/verification_report.md`, `gates/` + `audit.jsonl`.
Defects go to a free tracker: a local file (`reports/design/tracker.json`, default) or **GitHub Issues** (`--tracker github
--github-repo owner/name`, via the `gh` login; keys `GH-n`; the same duplicate rule on labels). Jira Cloud was dropped on 2026-10-09
(cost); other free options are in `docs/INTEGRATION_PLAN.md` section 9.

## The three gates

| Gate | What the reviewer approves | Re-opens when |
|---|---|---|
| `basis_review` | the test basis: trigger → bench fault, threshold, reaction, FTTI, code checks, open questions | the requirement, catalogue or config changes |
| `case_review` | the derived cases and their expected results, before bench time | the basis or the derivation changes |
| `publish_approval` | per-case verdicts + the defect drafts (new ticket or comment on an open one) | a new bench run (its manifest hash) |

Approvals are bound to the sha256 of what was shown, logged to `audit.jsonl`. `--auto-approve` is for tests and demos only;
its approvals are recorded as `demo-auto` and never reused by a real run.

## Modules

| File | Does |
|---|---|
| `catalogue.json` | requirement wording → bench fault (`link_lost` on PLN_Command, period, causes) and reaction (`STOP_IN_LANE`) |
| `basis.py` | drafts the basis from "<trigger> for > N ms → <reaction>" (or reads `design/basis/<req>.json`), then checks it: fault known to the bench, threshold = config value, FTTI ≥ threshold + one period, latch requirement exists; lists open questions |
| `derive.py` | ISTQB cases: permanent loss at low / mid / near-ODD speed (FI + EP), mid-period injection, 3-value BVA on the loss duration. Expected: reaction, cause, stop, latch, detection ≤ FTTI and **not before threshold − one period**. Durations no requirement covers are blocked GAP cases, run as probes for information only. Event requirements (LLM-drafted): one case per alternative, the first again at low / near-ODD speed, other bench patterns of the same fault, BVA on a fault-parameter limit, GAP cases for what can't be injected |
| `campaign.py` | the pipeline, triage, defect drafts (failing cases grouped by failure mode, groups sharing an injected fault merged: one bug = one ticket; labels `sig-<fault>` + `mode-<check>`; an open defect with the same input and modes gets a comment instead), verification report |
| `drafter.py` | LLM drafter for event requirements and its code checks |
| `llm.py` | Claude Sonnet 5.5 (medium effort) through the Anthropic SDK, cached |
| `gates.py` | human gates (from HIL ML Ops) |
| `tracker.py` | the free local JSON tracker (one file for all campaigns) and the GitHub Issues tracker |

## What SR-01 shows

Generated: 5 executable cases + 2 blocked probes. The reference controller passes all 5. The probes answer the open question
with evidence: a 99 ms loss already stops and latches the vehicle (the first frame after a gap fails the sequence check),
although SR-01 only demands a reaction above 100 ms. With the seeded `long_timeout` bug, all 5 cases fail (detection at
1000 ms against a 250 ms FTTI; a 101 ms loss is not detected at all) and one defect is drafted; a second run adds a comment
to it instead of a duplicate. The FMU passes the same 5 cases; the firmware's C++ core built for the PC, with the seeded
bug, fails the same 5.
