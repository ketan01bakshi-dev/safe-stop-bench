"""Load scenarios from JSON and generate boundary-value scenarios from the configured limits."""
from __future__ import annotations

import copy
import json

from .config import ROOT

DEFAULTS: dict = {"start_kmh": 30, "duration_ms": 8000, "inject_ms": 2000}


def _defaults(sc: dict) -> dict:
    out = copy.deepcopy(DEFAULTS)
    out.update(sc)
    if out.get("replay_log") and not str(out["replay_log"]).startswith(str(ROOT)):
        out["replay_log"] = str(ROOT / out["replay_log"])
    return out


def load(path: str = "scenarios/scenarios.json") -> list[dict]:
    data = json.loads((ROOT / path).read_text(encoding="utf-8"))
    return [_defaults(s) for s in data["scenarios"]]


def boundary(cfg: dict) -> list[dict]:
    """Just-inside / just-outside cases for each threshold (ISTQB boundary value analysis)."""
    s = cfg["safety"]
    out = []
    odd = s["odd_max_kmh"]
    for kmh in sorted({10, int(min(30, odd - 5)), int(odd)}):
        lim = max(s["steer_rate_min_dps"], s["steer_rate_max_dps"] - s["steer_rate_slope"] * kmh)
        for d, inside in ((-1.0, True), (+1.0, False)):
            out.append({"id": f"BV-steer-{kmh}-{'in' if inside else 'out'}", "key": f"bv_steer_rate_{kmh}kmh_{lim + d:.0f}dps",
                        "title": f"Steering rate {lim + d:.0f} deg/s at {kmh} km/h (limit {lim:.0f})", "req": ["SR-07"] if not inside else ["SR-18"],
                        # injected at 5 s so the speed has settled (the limit depends on speed; on a grade the planner needs time)
                        "start_kmh": kmh, "inject_ms": 5000, "duration_ms": 11000,
                        "faults": [{"type": "steer_rate", "start": 5000, "end": 5120, "dps": lim + d}],
                        "expect": {"reaction": "NORMAL"} if inside else {"reaction": "STOP_IN_LANE", "cause": ["ENVELOPE"], "stop": True, "latched": True}})
    # the first frame after a gap > max delta is rejected (WRONG_SEQUENCE), so the tolerated outage is
    # timeout − one frame period: 60 ms passes, 80 ms trips (a finding the boundary tests surfaced)
    for ms, inside in ((60, True), (80, False)):
        out.append({"id": f"BV-timeout-{ms}", "key": f"bv_link_outage_{ms}ms", "title": f"Link outage of {ms} ms (timeout {s['cmd_timeout_ms']} ms; effective tolerance 60 ms)",
                    "req": ["SR-18"] if inside else ["SR-01"], "faults": [{"type": "link_lost", "start": 2000, "end": 2000 + ms}],
                    "expect": {"reaction": "NORMAL"} if inside else {"reaction": "STOP_IN_LANE", "cause": ["TIMEOUT", "E2E_INVALID"], "stop": True, "latched": True}})
    for kmh, inside in ((s["odd_max_kmh"] - 1, True), (s["odd_max_kmh"] + 1, False)):
        out.append({"id": f"BV-odd-{kmh:.0f}", "key": f"bv_speed_req_{kmh:.0f}kmh", "title": f"Speed request {kmh:.0f} km/h (ODD limit {s['odd_max_kmh']:.0f})",
                    "req": ["SR-18"] if inside else ["SR-08"], "duration_ms": 10000, "faults": [{"type": "speed_req", "start": 2000, "kmh": kmh}],
                    "expect": {"reaction": "NORMAL"} if inside else {"reaction": "STOP_IN_LANE", "cause": ["ENVELOPE"], "stop": True, "latched": True}})
    return [_defaults(x) for x in out]


def sweep(base: dict, speeds=(10, 20, 30, 40), frictions=("dry", "wet", "gravel")) -> list[dict]:
    out = []
    for v in speeds:
        for mu in frictions:
            sc = copy.deepcopy(base)
            sc["key"] = f"{base['key']}@{v}kmh/{mu}"
            sc["start_kmh"], sc["friction"] = v, mu
            out.append(sc)
    return out
