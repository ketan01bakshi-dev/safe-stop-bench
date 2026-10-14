"""P5 stage 1: run vehicle/link_cases.json against a chassis core.

    python -m vehicle.conformance            # the Python ChassisCore; prints every failed expectation, exit 1 if any

The cases are the executable form of docs/P5_LINK_CONTRACT.md. The stage 2 firmware port is checked with the same file through an adapter that has
the same calls (rx, step, link_down, physical_rearm, set_encoders) and read-only state / reason / reject / outputs / counters.
"""
from __future__ import annotations

import json
import sys
from collections.abc import Callable
from pathlib import Path

from vehicle import link
from vehicle.fake_chassis import ChassisCore

CASES = Path(__file__).with_name("link_cases.json")
COUNTERS = ("rx_ok", "rx_bad", "rx_ignored", "rx_foreign", "rx_stale", "rx_rejected")


def load_cases(path: Path = CASES) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def default_core(params: dict) -> ChassisCore:
    p = dict(params)
    return ChassisCore(p.pop("boot_id"), hello_peer="b", **p)


def _frame(spec: dict) -> tuple[str, bytes]:
    spec = dict(spec)
    src = spec.pop("src")
    corrupt = spec.pop("corrupt", False)
    cls = getattr(link, spec.pop("type"))
    data = link.encode(cls(**spec))
    if corrupt:
        data = data[:-1] + bytes([data[-1] ^ 0x01])
    return src, data


def run_case(case: dict, base_params: dict, factory: Callable[[dict], ChassisCore] = default_core) -> list[str]:
    """The failed expectations of one case (empty = pass)."""
    core = factory({**base_params, **case.get("params", {})})
    failures: list[str] = []
    now = -1
    for ev in case["steps"]:
        t = ev["t"]
        for tt in range(now + 1, t):
            core.step(tt)                                        # the ticks in between: their output is not checked
        sent: list[bytes] = []
        if "rx" in ev:
            src, data = _frame(ev["rx"])
            sent += [d for _a, d in core.rx(data, src, t)]
        if ev.get("link_down"):
            core.link_down(t)
        if ev.get("physical_rearm"):
            core.physical_rearm(t)
        if "encoders" in ev:
            core.set_encoders(t, tuple(ev["encoders"]["counts"]), ev["encoders"]["mask"])
        sent += [d for _a, d in core.step(t)]
        now = t
        failures += _check(case["name"], t, core, sent, ev.get("expect", {}))
    return failures


def _check(name: str, t: int, core: ChassisCore, sent: list[bytes], expect: dict) -> list[str]:
    bad: list[str] = []

    def want(field: str, got, exp) -> None:
        if got != exp:
            bad.append(f"{name} @ {t} ms: {field} is {got!r}, expected {exp!r}")

    if "state" in expect:
        want("state", core.state.name, expect["state"])
    if "reason" in expect:
        want("reason", core.reason.name, expect["reason"])
    if "reject" in expect:
        want("reject", core.reject.name, expect["reject"])
    if "outputs" in expect:
        want("outputs", list(core.outputs), expect["outputs"])
    if "sent" in expect:
        want("sent", [type(link.decode(d)).__name__.upper() for d in sent], expect["sent"])
    if "echo" in expect:                                         # fields of the last ECHO sent at this event
        echoes = [link.decode(d) for d in sent if isinstance(link.decode(d), link.Echo)]
        if not echoes:
            bad.append(f"{name} @ {t} ms: no ECHO sent to check {sorted(expect['echo'])}")
        else:
            for k, v in expect["echo"].items():
                want(f"echo.{k}", getattr(echoes[-1], k), v)
    for k, v in expect.get("counters", {}).items():
        want(k, getattr(core, k), v)
    return bad


def run_all(path: Path = CASES, factory: Callable[[dict], ChassisCore] = default_core) -> tuple[int, list[str]]:
    doc = load_cases(path)
    failures: list[str] = []
    for case in doc["cases"]:
        failures += run_case(case, doc["params"], factory)
    return len(doc["cases"]), failures


def main() -> int:
    n, failures = run_all()
    for f in failures:
        print("FAIL", f)
    print(f"{n} cases, {len(failures)} failed expectations")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
