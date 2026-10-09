"""Verdicts: expected reaction (from the scenario, traced to requirements) + invariants + KPIs."""
from __future__ import annotations

from .safety import RANK


def kpis(r: dict, inject_ms: int | None) -> dict:
    out = {"t_detect_ms": None, "t_stop_ms": None, "stop_dist_m": None, "margin_ms": None}
    t_det = r["t_fault"] if r["t_fault"] is not None else r["t_ecu"]
    if r["ecu_fallback"] and (r["t_fault"] is None or (r["t_ecu"] or 0) < r["t_fault"]):
        t_det = r["t_ecu"]
    if inject_ms is not None and t_det is not None:
        out["t_detect_ms"] = t_det - inject_ms
        x0 = next((x for t, v, y, st, x in r["trace"] if t >= t_det), None)
        stop = next(((t, x) for t, v, y, st, x in r["trace"] if t >= t_det and v == 0.0), None)
        if stop and x0 is not None:
            out["t_stop_ms"] = stop[0] - inject_ms
            out["stop_dist_m"] = round(stop[1] - x0, 2)
    return out


def verdict(r: dict, sc: dict, reqs: dict, cfg: dict) -> dict:
    e = sc.get("expect", {})
    inject = sc.get("inject_ms")
    k = kpis(r, inject)
    ftti = min([reqs[q]["ftti_ms"] for q in sc.get("req", []) if q in reqs and reqs[q].get("ftti_ms")], default=None)
    peak = max((RANK.get(s, 0) for _, s in r["states"]), default=0)
    peak_state = next((s for _, s in r["states"] if RANK.get(s, 0) == peak and peak > 0), "NORMAL")
    checks = []
    want = e.get("reaction")
    if want == "NORMAL":
        checks.append(("no reaction (no false stop)", peak == 0 and not r["ecu_fallback"]))
    elif want == "ECU_FALLBACK":
        checks.append(("actuator ECU fallback stopped the vehicle", r["ecu_fallback"]))
    elif isinstance(want, list):   # any of these stop reactions is acceptable (v2.24)
        checks.append((f"reaction in {want}", peak_state in want))
    elif want:
        checks.append((f"reaction {want}", peak_state == want))
    if e.get("cause"):
        checks.append((f"cause in {e['cause']}", r["cause"] in e["cause"]))
    if e.get("drives"):
        checks.append(("vehicle drives off normally", r["v_end_kmh"] > 3.0))
    if sc.get("start_kmh", 30) == 0 and r["states"]:
        # v2.2: a supplier-style FMU without a start-mode hook began a cold start in NORMAL; only back-to-back saw it
        checks.append(("cold start begins in INIT", r["states"][0][1] == "INIT"))
    if sc.get("no_ftti"):
        ftti = None
    if ftti and want not in (None, "NORMAL"):
        checks.append((f"detected within FTTI {ftti} ms", k["t_detect_ms"] is not None and k["t_detect_ms"] <= ftti))
        if k["t_detect_ms"] is not None:
            k["margin_ms"] = ftti - k["t_detect_ms"]
    if "min_detect_ms" in e:
        # v2.14 (generated fault-injection cases): no reaction before the requirement's threshold minus one frame period;
        # a controller that stops at once on any gap passes every "reaction" check but fails this one. No reaction at all is
        # the reaction check's failure, not this one's
        checks.append((f"no reaction before {e['min_detect_ms']} ms", k["t_detect_ms"] is None or k["t_detect_ms"] >= e["min_detect_ms"]))
    if e.get("stop"):
        checks.append(("vehicle stopped", r["v_end_kmh"] == 0.0))
    if e.get("latched"):
        checks.append(("still latched at end", RANK.get(r["final_state"], 0) >= 2 or r["ecu_fallback"]))
    if e.get("no_motion"):
        checks.append(("vehicle never moved", r["x_max"] < 0.05))
    if "speed_cap_kmh" in e:
        checks.append((f"speed settled ≤ {e['speed_cap_kmh']} km/h", r["v_end_kmh"] <= e["speed_cap_kmh"] + 1.0))
    if e.get("resume"):
        checks.append(("release accepted and vehicle drives again", r["released_ok"] and r["v_end_kmh"] > 3.0))
    if e.get("release_rejected"):
        first = next((txt for t, txt in r["events"] if txt.startswith("release")), "")
        checks.append(("first (unsafe) release attempt rejected", first == "release rejected"))
    if "min_y_m" in e:
        checks.append((f"moved to the shoulder (y ≥ {e['min_y_m']} m)", r["y_end"] >= e["min_y_m"]))
    max_lat = e.get("max_lateral_m", cfg["road"]["lane_half_width_m"] - 0.85)
    if "min_y_m" not in e:
        checks.append((f"stayed in lane (|y| ≤ {max_lat:.2f} m)", r["max_lateral_m"] <= max_lat))
    checks.append(("no invariant violations", r["invariant_count"] == 0))
    if r.get("bench_lag_ms") is not None:   # real-time DUTs only: the bench itself must keep up
        from .runner import LAG_LIMIT_MS
        checks.append((f"bench kept real time (max lag {r['bench_lag_ms']} ms <= {LAG_LIMIT_MS:g} ms)", r["bench_lag_ms"] <= LAG_LIMIT_MS))
    passed = all(ok for _, ok in checks)
    known = cfg.get("known_findings", {}).get(sc["key"])
    status = "PASS" if passed else ("KNOWN" if known else "FAIL")
    return {**k, "ftti_ms": ftti, "peak_state": peak_state, "checks": checks, "passed": passed, "status": status, "known": known}
