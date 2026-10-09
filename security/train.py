"""Train the intrusion detector on this bench's own traffic (P3).

    .venv\\Scripts\\python.exe -m security.train          -> security/profile.json, security/model.json, security/TRAINING.md

Clean traffic = the reference planner doing benign things (several speeds, lane change, brake event, speed request, noise, jitter,
latency). The rule profile is learned from clean traffic ONLY. The MLP sees clean windows and windows from attack runs with TRAINING
parameters; it is judged on attack runs with DIFFERENT parameters (other IDs, rates, offsets, ages, steering values) and on clean runs
with other seeds, so the numbers are not a memory test.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from ssb import config
from ssb.planner import CMD_ID, PERIOD_MS
from ssb.runner import run

from .ids import FEATURES, MODEL, PROFILE, SIGNALS, WINDOW_MS, Ids, parse, train_mlp, window_features

HERE = Path(__file__).parent


class Recorder:
    """An IDS stand-in: keeps every frame the DUT received."""

    def __init__(self):
        self.frames: list[tuple[int, int, bytes]] = []

    def observe(self, t: int, frames: list) -> None:
        self.frames += [(t, f.can_id, f.data) for f in frames]

    def summary(self) -> dict:
        return {}


def base(key: str, kmh: float, faults: list[dict] | None = None, duration: int = 8000) -> dict:
    return {"key": key, "id": key, "title": key, "start_kmh": kmh, "duration_ms": duration, "faults": faults or [], "req": [], "expect": {}}


CLEAN = [base(f"clean_{v}kmh", v) for v in (10, 20, 30, 40, 50)] + [
    base("clean_lane", 30, [{"type": "lane_change", "start": 3000, "end": 5000, "steer_deg": 5.0}]),
    base("clean_brake", 40, [{"type": "brake_event", "start": 3000, "end": 5000, "accel": -2.0}]),
    base("clean_speed", 20, [{"type": "speed_req", "start": 2000, "kmh": 35}]),
    base("clean_noise", 30, [{"type": "noise", "start": 1000, "rate": 0.03}]),
    base("clean_jitter", 30, [{"type": "jitter", "start": 1000, "end": 6000, "ms": 4}]),
    base("clean_latency", 30, [{"type": "latency", "start": 1000, "end": 6000, "ms": 6}]),
]


def attack(kind: str, start: int = 3000, end: int = 6000, duration: int = 8000, **kw) -> dict:
    name = kind + "_" + "_".join(f"{k}{v}" for k, v in kw.items())
    return base(f"atk_{name}", 30, [{"type": "attack", "kind": kind, "start": start, "end": end, **kw}], duration)


TRAIN_ATTACKS = [attack("flood", id=0x001, per_ms=2), attack("flood", id=0x7FF, per_ms=3), attack("fuzzy", period_ms=5),
                 attack("spoof_invalid"), attack("spoof_valid", mode="inject", steer=25.0), attack("spoof_valid", mode="takeover", steer=25.0),
                 attack("replay", age_ms=1000, mode="takeover"), attack("replay", age_ms=300), attack("period_glitch", offset_ms=7),
                 attack("signal_ramp", dps=2.0)]
TEST_ATTACKS = [attack("flood", id=0x002, per_ms=4), attack("flood", id=0x7F0, per_ms=1), attack("fuzzy", period_ms=11),
                attack("spoof_invalid", steer=-20.0), attack("spoof_valid", mode="inject", steer=-20.0, accel=-1.0),
                attack("spoof_valid", mode="takeover", steer=-18.0, accel=1.0), attack("replay", age_ms=400, mode="takeover"),
                attack("replay", age_ms=5120, mode="takeover", start=6000, end=9000, duration=10000),   # needs 5.12 s of history first
                attack("spoof_valid", mode="takeover", accel=None, steer=None, speed=None),   # mimic: valid, fresh, same values as the real planner
                attack("period_glitch", offset_ms=13), attack("period_glitch", offset_ms=3),
                attack("signal_ramp", dps=4.0), attack("signal_ramp", dps=1.0)]


def capture(sc: dict, seed: int, cfg: dict) -> list[tuple[int, int, bytes]]:
    rec = Recorder()
    run({**sc, "_ids": rec}, cfg, seed=seed, keep_trace=False)
    return rec.frames


def learn_profile(clean: list[list[tuple[int, int, bytes]]]) -> dict:
    frames = [f for run_ in clean for f in run_]
    cmds = [(t, p) for t, i, d in frames if i == CMD_ID and (p := parse(d)) is not None]
    sig = {s: [p[s] for _, p in cmds] for s in SIGNALS}
    slew: dict[str, float] = {s: 0.0 for s in SIGNALS}
    lags = []
    for run_ in clean:
        prev = None
        for t, i, d in run_:
            p = parse(d) if i == CMD_ID else None
            if p:
                lags.append(((t & 0xFFFF) - p["t_stamp"]) & 0xFFFF)
                if prev:
                    for s in SIGNALS:
                        slew[s] = max(slew[s], abs(p[s] - prev[s]))
                prev = p
    per_win = []
    for run_ in clean:
        buckets: dict[int, int] = {}
        for t, _, _ in run_:
            buckets[t // WINDOW_MS] = buckets.get(t // WINDOW_MS, 0) + 1
        per_win.append(max(buckets.values(), default=0))
    rng = {}
    for s, v in sig.items():
        lo, hi = min(v), max(v)
        m = 0.25 * (hi - lo) + (0.5 if s in ("accel", "steer", "speed_req") else 0)
        rng[s] = [round(lo - m, 3), round(hi + m, 3)]
    return {"ids": sorted({i for _, i, _ in frames}), "dlc": max(len(d) for _, i, d in frames if i == CMD_ID), "period_ms": PERIOD_MS,
            "max_per_window": max(per_win) + 2, "range": rng,
            "slew": {s: round(slew[s] * 1.5 + (0.05 if s != "perception" else 0), 3) for s in SIGNALS},
            "max_lag_ms": round(max(lags) * 1.5 + 5, 1), "learned_from": f"{len(clean)} clean runs, {len(cmds)} commands"}


def windows(frames: list[tuple[int, int, bytes]], profile: dict, attack_span: tuple[int, int] | None, end_ms: int = 8000):
    xs, ys = [], []
    for w0 in range(0, end_ms - WINDOW_MS + 1, WINDOW_MS):
        inside = [f for f in frames if w0 <= f[0] < w0 + WINDOW_MS]
        xs.append(window_features(inside, w0 + WINDOW_MS - 1, profile))
        ys.append(1.0 if attack_span and w0 + WINDOW_MS > attack_span[0] and w0 < attack_span[1] else 0.0)
    return xs, ys


def span(sc: dict) -> tuple[int, int] | None:
    a = [f for f in sc["faults"] if f["type"] == "attack"]
    return (a[0]["start"], a[0]["end"]) if a else None


def main() -> None:
    cfg = config.load("config/default.json")
    clean_tr = [capture(sc, seed, cfg) for sc in CLEAN for seed in (1, 2)]
    profile = learn_profile(clean_tr)
    atk_tr = [(sc, capture(sc, 3, cfg)) for sc in TRAIN_ATTACKS]
    x, y = [], []
    for fr in clean_tr:
        a, b = windows(fr, profile, None)
        x += a
        y += b
    for sc, fr in atk_tr:
        a, b = windows(fr, profile, span(sc), sc["duration_ms"])
        x += a
        y += b
    X, Y = np.array(x), np.array(y)
    model = train_mlp(X, Y)

    # held-out judgement
    clean_te = [(sc["key"], capture(sc, seed, cfg)) for sc in CLEAN for seed in (5, 6)]
    atk_te = [(sc, capture(sc, 7, cfg)) for sc in TEST_ATTACKS]
    ids_rules_fp = ids_mlp_fp = 0
    for _, fr in clean_te:
        r, m = Ids(profile, model, "rules"), Ids(profile, model, "mlp")
        ids_rules_fp += _replay(r, fr)["alerted"]
        ids_mlp_fp += _replay(m, fr)["alerted"]
    rows = []
    for sc, fr in atk_te:
        res = {m: _replay(Ids(profile, model, m), fr, sc["duration_ms"]) for m in ("rules", "mlp", "both")}
        start = (span(sc) or (0, 0))[0]
        rows.append((sc["key"], {m: (None if not r["alerted"] else r["first_ms"] - start) for m, r in res.items()},
                     res["both"]["first_by"], res["rules"]["alerted"] and res["rules"]["first_ms"] < start))
    PROFILE.write_text(json.dumps(profile, indent=1), encoding="utf-8")
    MODEL.write_text(json.dumps(model.dump()), encoding="utf-8")
    write_report(profile, len(X), int(Y.sum()), len(clean_te), ids_rules_fp, ids_mlp_fp, rows)


def _replay(ids: Ids, frames: list[tuple[int, int, bytes]], duration: int = 8000) -> dict:
    class F:
        def __init__(self, i, d):
            self.can_id, self.data = i, d
    by_t: dict[int, list] = {}
    for t, i, d in frames:
        by_t.setdefault(t, []).append(F(i, d))
    for t in range(duration):
        ids.observe(t, by_t.get(t, []))
    return ids.summary()


def write_report(profile: dict, n_win: int, n_atk: int, n_clean: int, fp_rules: int, fp_mlp: int, rows: list) -> None:
    def cell(v):
        return "missed" if v is None else f"{v} ms"
    lines = ["# Intrusion detector: training and held-out result", "",
             f"Rule profile learned from clean traffic only ({profile['learned_from']}). MLP: {len(FEATURES)} window features, 12 hidden units, "
             f"trained on {n_win} windows ({n_atk} with an attack).", "",
             f"False alarms on {n_clean} held-out clean runs (other seeds): rules {fp_rules}, MLP {fp_mlp}.", "",
             "Held-out attacks (parameters the MLP never saw). Detection latency from the attack's start:", "",
             "| Attack | Rules | MLP | Both | First by |", "|---|---|---|---|---|"]
    for key, lat, by, _early in rows:
        lines.append(f"| {key} | {cell(lat['rules'])} | {cell(lat['mlp'])} | {cell(lat['both'])} | {by or '-'} |")
    lines += ["", "Rule thresholds are in `profile.json`; the learned ranges are the evidence for what counts as normal on this bench."]
    (HERE / "TRAINING.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
