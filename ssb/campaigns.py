"""Campaigns on top of single runs: matrix, sweeps, false-stop rate, fuzzing, mutation score, back-to-back, gaps."""
from __future__ import annotations

import copy
import json
import random

from . import oracle, runner, scenarios
from .config import ROOT
from .dut import ReferenceDUT
from .safety import MUTANTS


def requirements() -> dict:
    return json.loads((ROOT / "safety" / "hazards.json").read_text(encoding="utf-8"))


def all_scenarios(cfg: dict) -> list[dict]:
    """Scenarios fitted to the configured ODD: start speeds above the ODD limit are lowered to limit − 5 km/h."""
    odd = cfg["safety"]["odd_max_kmh"]
    out = []
    for sc in scenarios.load() + scenarios.boundary(cfg):
        if sc.get("start_kmh", 30) > odd:
            sc = dict(sc, start_kmh=odd - 5, target_kmh=odd - 5)
        if sc.get("target_kmh", 0) > odd:
            sc = dict(sc, target_kmh=odd - 5)
        out.append(sc)
    return out


def matrix(cfg: dict, dut_factory=None, keep_trace=True, only=None) -> list[dict]:
    reqs = requirements()["requirements"]
    out = []
    for sc in all_scenarios(cfg):
        if only and sc["key"] not in only:
            continue
        dut = dut_factory(sc) if dut_factory else None
        r = runner.run(sc, cfg, dut, keep_trace=keep_trace)
        r["verdict"] = oracle.verdict(r, sc, reqs, cfg)
        r["finding"] = sc.get("finding")
        out.append(r)
    return out


def sweep(cfg: dict) -> list[dict]:
    reqs = requirements()["requirements"]
    by_key = {s["key"]: s for s in scenarios.load()}
    rows = []
    for key in ("command_link_lost", "steer_out_of_envelope", "steering_stuck", "brake_weak"):
        for sc in scenarios.sweep(by_key[key]):
            r = runner.run(sc, cfg, keep_trace=True)
            v = oracle.verdict(r, sc, reqs, cfg)
            rows.append({"key": sc["key"], "base": key, "kmh": sc["start_kmh"], "friction": sc["friction"],
                         "t_detect_ms": v["t_detect_ms"], "t_stop_ms": v["t_stop_ms"], "stop_dist_m": v["stop_dist_m"],
                         "max_lateral_m": r["max_lateral_m"], "passed": v["passed"],
                         "failed": [c for c, ok in v["checks"] if not ok]})
    return rows


LOAD_KEYS = ("command_link_lost", "brake_weak")


def load_sweep(cfg: dict, payloads=(0, 300, 600, 700, 800, 900, 1500), grades=(0.0, -6.0), duration_ms: int = 15000) -> list[dict]:
    """v2.5, dynamic plant only: payload x grade for a planned stop (MRM, link lost) and a weak brake.
    Shows what the kinematic model can't: the same brake request gives less deceleration with load, and the
    brake-plausibility check (achieved < 50% of demand for 400 ms) reacts to load + actuator lag together.
    15 s runs: a loaded stop downhill takes longer than the matrix's 8 s window."""
    if cfg["plant"].get("model") != "dynamic":
        raise ValueError("load_sweep needs the dynamic plant (run.py --plant dynamic)")
    reqs = requirements()["requirements"]
    by_key = {s["key"]: s for s in scenarios.load()}
    rows = []
    for key in LOAD_KEYS:
        for g in grades:
            for kg in payloads:
                sc = copy.deepcopy(by_key[key])
                sc["key"], sc["payload_kg"], sc["grade_pct"] = f"{key}@{kg}kg/{g:+.0f}%", kg, g
                sc["duration_ms"] = max(sc.get("duration_ms", 8000), duration_ms)
                r = runner.run(sc, cfg, keep_trace=True)
                v = oracle.verdict(r, sc, reqs, cfg)
                rows.append({"key": sc["key"], "base": key, "payload_kg": kg, "grade_pct": g,
                             "mass_ratio": round(cfg["plant"]["dynamic"]["mass_kg"] / (cfg["plant"]["dynamic"]["mass_kg"] + kg), 2),
                             "peak_state": v["peak_state"], "cause": r["cause"], "t_detect_ms": v["t_detect_ms"],
                             "t_stop_ms": v["t_stop_ms"], "stop_dist_m": v["stop_dist_m"], "v_end_kmh": r["v_end_kmh"],
                             "passed": v["passed"], "failed": [c for c, ok in v["checks"] if not ok]})
    return rows


def false_stop_rate(cfg: dict, seeds: int = 10, seconds: int = 60, noise: float = 0.005, jitter_ms: int = 10) -> dict:
    stops, km, details = 0, 0.0, []
    for seed in range(seeds):
        sc = scenarios._defaults({"key": f"nominal_seed{seed}", "inject_ms": None, "duration_ms": seconds * 1000,
                                  "faults": [{"type": "noise", "start": 0, "rate": noise}, {"type": "jitter", "start": 0, "ms": jitter_ms}]})
        r = runner.run(sc, cfg, seed=seed, keep_trace=False)
        stopped = r["t_fault"] is not None or r["ecu_fallback"]
        stops += stopped
        km += r["x_max"] / 1000
        details.append({"seed": seed, "stopped": stopped, "cause": r["cause"], "rejected": r["rejected_frames"]})
    per100 = stops / km * 100 if km else 0.0
    upper = (3.0 / km * 100) if stops == 0 else None   # rule of three: 95% upper bound with zero events
    return {"seeds": seeds, "km": round(km, 2), "false_stops": stops, "per_100km": round(per100, 2),
            "upper95_per_100km": None if upper is None else round(upper, 1), "noise": noise, "jitter_ms": jitter_ms, "details": details}


FUZZ_TYPES = ["link_lost", "crc_corrupt", "counter_frozen", "kick_fast", "qa_wrong", "steer_rate", "speed_req",
              "accel_req", "stale_timestamp", "steer_stuck", "brake_weak", "act_frame_corrupt", "perception", "odd_exit"]


def fuzz(cfg: dict, runs: int = 30, seed: int = 1) -> dict:
    """Random fault combinations; no expected reaction, only the invariants must hold and no crash."""
    rng = random.Random(seed)
    bad = []
    for i in range(runs):
        faults = []
        for _ in range(rng.randint(1, 3)):
            ty = rng.choice(FUZZ_TYPES)
            start = rng.randint(500, 5000)
            f: dict[str, object] = {"type": ty, "start": start, "end": start + rng.randint(20, 3000)}
            params: dict[str, dict[str, object]] = {"steer_rate": {"dps": rng.uniform(5, 120)}, "speed_req": {"kmh": rng.uniform(20, 70)},
                      "accel_req": {"accel": rng.uniform(-8, 5)}, "stale_timestamp": {"age_ms": rng.randint(20, 300)},
                      "brake_weak": {"factor": rng.uniform(0.1, 0.9)}, "perception": {"health": rng.choice([0, 1])}}
            f.update(params.get(ty, {}))
            faults.append(f)
        sc = scenarios._defaults({"key": f"fuzz_{i}", "inject_ms": None, "start_kmh": rng.choice([10, 20, 30, 40]),
                                  "duration_ms": 8000, "faults": faults})
        try:
            r = runner.run(sc, cfg, seed=i, keep_trace=False)
            if r["invariant_count"]:
                bad.append({"run": i, "faults": faults, "violations": r["invariant_violations"][:3]})
        except Exception as e:  # a crash is a finding too
            bad.append({"run": i, "faults": faults, "violations": [f"CRASH: {e!r}"]})
    return {"runs": runs, "violating_runs": len(bad), "examples": bad[:5]}


def mutation(cfg: dict) -> dict:
    """Run the matrix against each seeded mutant; a mutant is 'killed' if at least one scenario fails."""
    reqs = requirements()["requirements"]
    scs = all_scenarios(cfg)
    rows = []
    for name, desc in MUTANTS.items():
        killer = None
        for sc in scs:
            dut = ReferenceDUT(cfg, frozenset([name]), warm_start=sc.get("start_kmh", 30) > 0)
            r = runner.run(sc, cfg, dut, keep_trace=False)
            if not oracle.verdict(r, sc, reqs, cfg)["passed"]:
                killer = sc["key"]
                break
        rows.append({"mutant": name, "description": desc, "killed": killer is not None, "killed_by": killer})
    killed = sum(bool(r["killed"]) for r in rows)
    return {"mutants": rows, "killed": killed, "total": len(rows), "score": round(100 * killed / len(rows), 1)}


TIMING_KEYS = ["command_link_lost", "crc_corrupt", "planner_hang", "qa_wrong_answer", "steer_out_of_envelope", "steering_stuck"]


def timing_repeat(cfg: dict, factory, keys, n: int) -> list[dict]:
    """Real-time DUTs jitter: run each scenario n times; judge the WORST detection time against the FTTI."""
    reqs = requirements()["requirements"]
    rows = []
    for sc in [s for s in all_scenarios(cfg) if s["key"] in keys]:
        dets, passes = [], 0
        for i in range(n):
            r = runner.run(sc, cfg, factory(sc), seed=7 + i, keep_trace=True)
            v = oracle.verdict(r, sc, reqs, cfg)
            passes += v["passed"]
            if v["t_detect_ms"] is not None:
                dets.append(v["t_detect_ms"])
        ftti = v["ftti_ms"]
        rows.append({"key": sc["key"], "runs": n, "passed": passes, "min_ms": min(dets, default=None),
                     "typ_ms": sorted(dets)[len(dets) // 2] if dets else None, "max_ms": max(dets, default=None), "ftti_ms": ftti,
                     "worst_margin_ms": None if not dets or ftti is None else ftti - max(dets)})
    return rows


def back_to_back(cfg: dict, other_factory, tol_ms: int = 20, only=None, exact: bool = False, quantum_ms: int = 1) -> list[dict]:
    """Same stimulus into the reference and another DUT; compare peak state, cause and detection time.
    exact=True (deterministic DUTs such as an FMU): also the whole state timeline, to the millisecond.
    quantum_ms: how often the other DUT REPORTS its state (10 for a node that only says it in a 10 ms status frame);
    reference change times are rounded up to that grid, because a change between two status frames can't be seen."""
    reqs = requirements()["requirements"]
    rows = []
    for sc in all_scenarios(cfg):
        if only and sc["key"] not in only:
            continue
        ra = runner.run(sc, cfg, keep_trace=False)
        rb = runner.run(sc, cfg, other_factory(sc), keep_trace=False)
        va, vb = oracle.verdict(ra, sc, reqs, cfg), oracle.verdict(rb, sc, reqs, cfg)
        diffs = []
        if va["peak_state"] != vb["peak_state"]:
            diffs.append(f"state {va['peak_state']} vs {vb['peak_state']}")
        if ra["cause"] != rb["cause"]:
            diffs.append(f"cause {ra['cause']} vs {rb['cause']}")
        if va["t_detect_ms"] is not None and vb["t_detect_ms"] is not None and abs(va["t_detect_ms"] - vb["t_detect_ms"]) > tol_ms:
            diffs.append(f"detect {va['t_detect_ms']} vs {vb['t_detect_ms']} ms")
        qa = [(-(-t // quantum_ms) * quantum_ms, s) for t, s in ra["states"]]
        if exact and qa != [tuple(x) for x in rb["states"]]:
            first = next((f"{x} vs {tuple(y)}" for x, y in zip(qa, rb["states"], strict=False) if x != tuple(y)),
                         f"{len(qa)} vs {len(rb['states'])} state changes")
            diffs.append(f"timeline differs: {first}")
        if exact and (ra["v_end_kmh"], ra["max_lateral_m"]) != (rb["v_end_kmh"], rb["max_lateral_m"]):
            diffs.append(f"plant end state {ra['v_end_kmh']}/{ra['max_lateral_m']} vs {rb['v_end_kmh']}/{rb['max_lateral_m']}")
        rows.append({"key": sc["key"], "match": not diffs, "diffs": diffs})
    return rows


def gaps(cfg: dict) -> dict:
    """Requirements with no scenario, and SOTIF catalogue items this bench can't cover."""
    h = requirements()
    covered = {q for sc in all_scenarios(cfg) for q in sc.get("req", [])}
    missing = [q for q in h["requirements"] if q not in covered]
    sotif = json.loads((ROOT / "safety" / "sotif_catalogue.json").read_text(encoding="utf-8"))
    return {"requirements_without_scenario": missing,
            "sotif_items_out_of_scope": [s for s in sotif["items"] if s["bench_coverage"] != "covered"]}


def ftti_sheet() -> list[dict]:
    return requirements().get("ftti_derivation", [])
