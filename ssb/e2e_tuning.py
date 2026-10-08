"""E2E false-stop tuning (v2.6): sweep the receiver/state-machine parameters, weigh false stops against detection.

    .venv\\Scripts\\python.exe -m ssb.e2e_tuning            # → reports/e2e_tuning.md + .json (~5 min)

The v2.0 finding: at 0.5% random CRC errors, two corrupted frames in a row make the next good frame's counter jump
by 3, which counts as a THIRD error (WRONG_SEQUENCE) and trips the window (> 2 errors in 6) -> a false stop.
One noise burst is counted three times.

Two halves, one verdict per candidate:
- False stops: a fast Monte Carlo through the REAL receiver and state machine (ssb.e2e, the same classes the
  controller uses), 1M planner frames per error rate (20 ms period = 5.6 h, ~170 km at 30 km/h). Random
  (independent) frame corruption; each trip to INVALID is one false stop, then the receiver restarts warm.
- Detection: (a) the full scenario matrix with the candidate's settings (every E2E scenario must still pass);
  (b) a fast check of how many frames a persistently degraded stream (k % corrupted) survives before INVALID,
  against the 250 ms FTTI of SR-02 (12 frames of 20 ms after the first bad one).
"""
from __future__ import annotations

import json
import random
import time
from pathlib import Path

from . import campaigns, config, e2e

ROOT = Path(__file__).resolve().parent.parent
PERIOD_MS = 20
DATA_ID = 0x1234

CANDIDATES = [
    {"name": "v2.0 default", "e2e_max_delta": 2, "e2e_max_err_valid": 2, "e2e_explain_gaps": False},
    {"name": "max delta 3", "e2e_max_delta": 3, "e2e_max_err_valid": 2, "e2e_explain_gaps": False},
    {"name": "max delta 4", "e2e_max_delta": 4, "e2e_max_err_valid": 2, "e2e_explain_gaps": False},
    {"name": "3 errors in 6", "e2e_max_delta": 2, "e2e_max_err_valid": 3, "e2e_explain_gaps": False},
    {"name": "explained gaps", "e2e_max_delta": 2, "e2e_max_err_valid": 2, "e2e_explain_gaps": True},
    {"name": "explained gaps + 3 in 6", "e2e_max_delta": 2, "e2e_max_err_valid": 3, "e2e_explain_gaps": True},
]
ERROR_RATES = [1e-4, 1e-3, 5e-3, 1e-2]
DEGRADED_RATES = [0.2, 0.35, 0.5, 1.0]
FTTI_FRAMES = 250 // PERIOD_MS


def _frames():
    payload = bytes(11)
    good = [e2e.Profile5.protect(payload, c, DATA_ID) for c in range(256)]
    return good, [e2e.flip_crc(f) for f in good]


def _rx_sm(cand: dict):
    rx = e2e.Profile5.Receiver(DATA_ID, max_delta=cand["e2e_max_delta"], explain_gaps=cand["e2e_explain_gaps"])
    sm = e2e.E2EStateMachine(max_err_valid=cand["e2e_max_err_valid"])
    sm.preset_valid()
    return rx, sm


def false_stops(cand: dict, rate: float, n_frames: int = 1_000_000, seed: int = 1, kmh: float = 30.0) -> dict:
    good, bad = _frames()
    rng = random.Random(seed)
    rx, sm = _rx_sm(cand)
    stops = 0
    for i in range(n_frames):
        c = i & 0xFF
        st, _ = rx.check(bad[c] if rng.random() < rate else good[c])
        if sm.update(st) == "INVALID":
            stops += 1
            rx, sm = _rx_sm(cand)          # operator release, warm restart
    hours = n_frames * PERIOD_MS / 3.6e6
    km = hours * kmh
    return {"rate": rate, "frames": n_frames, "km": round(km, 1), "false_stops": stops,
            "per_1000km": round(stops / km * 1000, 2),
            "upper95_per_1000km": round(3.0 / km * 1000, 2) if stops == 0 else None}


def detection(cand: dict, rate: float, runs: int = 2000, seed: int = 2, horizon: int = 200) -> dict:
    """A stream that turns bad (each frame corrupted with probability `rate`): frames until INVALID, from the start."""
    good, bad = _frames()
    rng = random.Random(seed)
    lat, missed = [], 0
    for _ in range(runs):
        rx, sm = _rx_sm(cand)
        start = rng.randrange(256)
        for k in range(horizon):
            c = (start + k) & 0xFF
            if sm.update(rx.check(bad[c] if rng.random() < rate else good[c])[0]) == "INVALID":
                lat.append(k + 1)
                break
        else:
            missed += 1
    lat.sort()
    within = sum(x <= FTTI_FRAMES for x in lat) / runs
    return {"rate": rate, "p50_frames": lat[len(lat) // 2] if lat else None,
            "p99_frames": lat[min(len(lat) - 1, int(len(lat) * 0.99))] if lat else None,
            "within_ftti": round(within, 4), "never": missed}


def matrix_check(cand: dict) -> dict:
    cfg = config.load()
    cfg["safety"].update({k: v for k, v in cand.items() if k.startswith("e2e_")})
    rs = campaigns.matrix(cfg, keep_trace=True)
    fails = [r["key"] for r in rs if r["verdict"]["status"] == "FAIL"]
    det = {r["key"]: r["verdict"]["t_detect_ms"] for r in rs if r["key"] in ("crc_corrupt", "intermittent_crc", "counter_frozen", "counter_gap_repeated")}
    return {"passed": len(rs) - len(fails), "total": len(rs), "fails": fails, "detect_ms": det}


def run(n_frames: int = 1_000_000) -> dict:
    out = []
    for cand in CANDIDATES:
        t0 = time.perf_counter()
        row: dict = {"candidate": cand,
               "false_stops": [false_stops(cand, r, n_frames) for r in ERROR_RATES],
               "detection": [detection(cand, r) for r in DEGRADED_RATES],
               "matrix": matrix_check(cand)}
        row["seconds"] = round(time.perf_counter() - t0, 1)
        print(f"  {cand['name']:26} done in {row['seconds']} s", flush=True)
        out.append(row)
    return {"error_rates": ERROR_RATES, "degraded_rates": DEGRADED_RATES, "ftti_frames": FTTI_FRAMES, "rows": out}


def to_markdown(res: dict) -> str:
    L = ["# E2E false-stop tuning (generated by `python -m ssb.e2e_tuning`)", "",
         "Random, independent frame corruption; planner frames every 20 ms at 30 km/h. Synthetic, illustrative.", "",
         "## False stops per 1000 km", "",
         "| Candidate | " + " | ".join(f"{r:.2%} errors" for r in res["error_rates"]) + " |",
         "|---|" + "---|" * len(res["error_rates"])]
    for row in res["rows"]:
        cells = []
        for f in row["false_stops"]:
            cells.append(f"{f['per_1000km']}" if f["false_stops"] else f"0 (< {f['upper95_per_1000km']})")
        L.append(f"| {row['candidate']['name']} | " + " | ".join(cells) + " |")
    L += ["", f"## Detection of a degraded stream (frames to INVALID; FTTI = {res['ftti_frames']} frames)", "",
          "| Candidate | " + " | ".join(f"{r:.0%} corrupted: p50 / p99 / within FTTI" for r in res["degraded_rates"]) + " |",
          "|---|" + "---|" * len(res["degraded_rates"])]
    for row in res["rows"]:
        L.append(f"| {row['candidate']['name']} | " + " | ".join(
            f"{d['p50_frames']} / {d['p99_frames']} / {d['within_ftti']:.1%}" + (f" ({d['never']} never)" if d["never"] else "")
            for d in row["detection"]) + " |")
    L += ["", "## Scenario matrix with each candidate", "", "| Candidate | Matrix | Failing scenarios | Detection (ms) |", "|---|---|---|---|"]
    for row in res["rows"]:
        m = row["matrix"]
        L.append(f"| {row['candidate']['name']} | {m['passed']}/{m['total']} | {', '.join(m['fails']) or '–'} | "
                 + ", ".join(f"{k} {v}" for k, v in m["detect_ms"].items()) + " |")
    return "\n".join(L) + "\n"


if __name__ == "__main__":
    import sys
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 1_000_000
    res = run(n)
    (ROOT / "reports").mkdir(exist_ok=True)
    (ROOT / "reports" / "e2e_tuning.json").write_text(json.dumps(res, indent=1), encoding="utf-8")
    (ROOT / "reports" / "e2e_tuning.md").write_text(to_markdown(res), encoding="utf-8")
    print(to_markdown(res))
