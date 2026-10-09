"""Intrusion detector for the planner bus (P3): learned rules + a small MLP, both trained on this bench's own traffic.

The detector only SEES frames (what the safety controller receives); it does not check the E2E CRC, so it is a second line of
defence that does not duplicate the first. What it knows comes from `security/profile.json` (the rule thresholds learned from clean
runs: IDs, length, period, counter step, signal ranges and slew) and `security/model.json` (MLP weights over 100 ms windows).

Rules (a rule fires once per frame; the alert list keeps the first time per rule and the count):
  unknown_id  an ID the profile never saw          dlc        a length the profile never saw
  period      two PLN_Commands closer than 0.5 periods (a second sender, a glitch)
  gap         no PLN_Command for more than 2.5 periods (an outage: not an attack by itself, reported separately)
  counter     the alive counter is not +1 (repeat, jump, replay)
  range       a signal outside the learned min / max (with margin)         slew   a signal change per frame above the learned maximum
  stamp       the timestamp is far older than the arrival time (stale data, replay)
  load        far more frames in 100 ms than the profile has (flood)
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from ssb.planner import CMD_ID, decode_cmd

HERE = Path(__file__).parent
PROFILE = HERE / "profile.json"
MODEL = HERE / "model.json"
WINDOW_MS = 100
FEATURES = ("rate", "other_ids", "bad_len", "dt_dev", "dt_min", "counter_bad", "distinct_bytes", "d_steer", "steer", "accel", "stamp_lag")
SIGNALS = ("accel", "steer", "speed_req", "perception")


def parse(data: bytes) -> dict | None:
    if len(data) < 3 + 11:
        return None
    d = decode_cmd(data[3:3 + 11])
    d["counter"] = data[2]
    return d


# ---- features over one window ---------------------------------------------------------------------------------

def window_features(frames: list[tuple[int, int, bytes]], t_end: int, profile: dict) -> list[float]:
    """frames = [(arrival ms, can id, data)] inside one tumbling window. A fixed-length vector (see FEATURES)."""
    n_per = WINDOW_MS / profile["period_ms"]
    cmd = [(t, d) for t, i, d in frames if i == CMD_ID]
    other = sum(1 for _, i, _ in frames if i != CMD_ID)
    bad_len = sum(1 for _, i, d in frames if i == CMD_ID and len(d) != profile["dlc"])
    parsed = [(t, p) for t, d in cmd if (p := parse(d)) is not None]
    dts = [b[0] - a[0] for a, b in zip(parsed, parsed[1:], strict=False)]
    dt_dev = sum(abs(x - profile["period_ms"]) for x in dts) / len(dts) / profile["period_ms"] if dts else 0.0
    dt_min = min(dts) / profile["period_ms"] if dts else 1.0
    steps = [(b[1]["counter"] - a[1]["counter"]) % 256 for a, b in zip(parsed, parsed[1:], strict=False)]
    counter_bad = sum(1 for s in steps if s != 1) / len(steps) if steps else 0.0
    distinct = sum(len(set(d)) / max(1, len(d)) for _, d in cmd) / len(cmd) if cmd else 0.0
    d_steer = max((abs(b[1]["steer"] - a[1]["steer"]) for a, b in zip(parsed, parsed[1:], strict=False)), default=0.0)
    steer = max((abs(p["steer"]) for _, p in parsed), default=0.0)
    accel = max((abs(p["accel"]) for _, p in parsed), default=0.0)
    lag = max(float(((t & 0xFFFF) - p["t_stamp"]) & 0xFFFF) for t, p in parsed) / profile["period_ms"] if parsed else 0.0
    return [len(frames) / n_per, float(other), float(bad_len), dt_dev, dt_min, counter_bad, distinct, d_steer, steer, accel, min(lag, 1000.0)]


# ---- rules -----------------------------------------------------------------------------------------------------

class Rules:
    def __init__(self, profile: dict):
        self.p = profile
        self.last_t: int | None = None
        self.last_counter: int | None = None
        self.last_sig: dict | None = None
        self.recent: list[int] = []
        self.alerts: dict[str, dict] = {}

    def _fire(self, rule: str, t: int, why: str) -> None:
        a = self.alerts.setdefault(rule, {"first_ms": t, "count": 0, "why": why})
        a["count"] += 1

    def frame(self, t: int, can_id: int, data: bytes) -> None:
        p = self.p
        self.recent = [x for x in self.recent if x > t - WINDOW_MS] + [t]
        if len(self.recent) > p["max_per_window"]:
            self._fire("load", t, f"{len(self.recent)} frames in {WINDOW_MS} ms (profile max {p['max_per_window']})")
        if can_id not in p["ids"]:
            self._fire("unknown_id", t, f"id 0x{can_id:X} not in the profile")
            return
        if can_id != CMD_ID:
            return
        if len(data) != p["dlc"]:
            self._fire("dlc", t, f"length {len(data)} (profile {p['dlc']})")
            return
        s = parse(data)
        if s is None:
            return
        if self.last_t is not None:
            dt = t - self.last_t
            if dt < 0.5 * p["period_ms"]:
                self._fire("period", t, f"{dt} ms after the previous command (period {p['period_ms']} ms)")
            elif dt > 2.5 * p["period_ms"]:
                self._fire("gap", t, f"{dt} ms without a command")
        if self.last_counter is not None and (s["counter"] - self.last_counter) % 256 != 1:
            self._fire("counter", t, f"counter {self.last_counter} -> {s['counter']}")
        for sig in SIGNALS:
            lo, hi = p["range"][sig]
            if not (lo <= s[sig] <= hi):
                self._fire("range", t, f"{sig} = {s[sig]:g} outside the learned [{lo:g}, {hi:g}]")
            if self.last_sig is not None and abs(s[sig] - self.last_sig[sig]) > p["slew"][sig]:
                self._fire("slew", t, f"{sig} changed by {abs(s[sig] - self.last_sig[sig]):g} in one frame (learned max {p['slew'][sig]:g})")
        lag = ((t & 0xFFFF) - s["t_stamp"]) & 0xFFFF
        if lag > p["max_lag_ms"]:
            self._fire("stamp", t, f"timestamp {lag} ms older than its arrival (learned max {p['max_lag_ms']:g})")
        self.last_t, self.last_counter, self.last_sig = t, s["counter"], s


# ---- the MLP ----------------------------------------------------------------------------------------------------

class Mlp:
    """One hidden layer (tanh), sigmoid output: P(window contains an attack). Numpy only; weights in security/model.json."""

    def __init__(self, w: dict):
        self.w = {k: np.array(v, dtype=float) for k, v in w.items()}

    def predict(self, x: list[float] | np.ndarray) -> np.ndarray:
        z = (np.atleast_2d(np.array(x, dtype=float)) - self.w["mu"]) / self.w["sd"]
        h = np.tanh(z @ self.w["W1"] + self.w["b1"])
        return 1.0 / (1.0 + np.exp(-(h @ self.w["W2"] + self.w["b2"]).ravel()))

    def dump(self) -> dict:
        return {k: v.tolist() for k, v in self.w.items()}


def train_mlp(x: np.ndarray, y: np.ndarray, hidden: int = 12, epochs: int = 600, lr: float = 0.05, seed: int = 0) -> Mlp:
    rng = np.random.default_rng(seed)
    mu, sd = x.mean(0), x.std(0) + 1e-6
    z = (x - mu) / sd
    w = {"mu": mu, "sd": sd, "W1": rng.normal(0, 0.5, (x.shape[1], hidden)), "b1": np.zeros(hidden),
         "W2": rng.normal(0, 0.5, (hidden, 1)), "b2": np.zeros(1)}
    pos_w = float(len(y) - y.sum()) / max(1.0, float(y.sum()))   # class balance: clean windows far outnumber attack windows
    m = {k: np.zeros_like(v) for k, v in w.items() if k not in ("mu", "sd")}
    v2 = {k: np.zeros_like(v) for k, v in m.items()}
    for step in range(1, epochs + 1):
        h = np.tanh(z @ w["W1"] + w["b1"])
        p = 1.0 / (1.0 + np.exp(-(h @ w["W2"] + w["b2"]).ravel()))
        wt = np.where(y > 0.5, min(pos_w, 20.0), 1.0)
        g = ((p - y) * wt / len(y))[:, None]
        grads = {"W2": h.T @ g, "b2": g.sum(0)}
        gh = (g @ w["W2"].T) * (1 - h ** 2)
        grads |= {"W1": z.T @ gh, "b1": gh.sum(0)}
        for k, gk in grads.items():   # Adam
            m[k] = 0.9 * m[k] + 0.1 * gk
            v2[k] = 0.999 * v2[k] + 0.001 * gk ** 2
            w[k] -= lr * (m[k] / (1 - 0.9 ** step)) / (np.sqrt(v2[k] / (1 - 0.999 ** step)) + 1e-8)
    return Mlp(w)


# ---- the detector ------------------------------------------------------------------------------------------------

class Ids:
    """Taps the frames the DUT receives. mode: 'rules', 'mlp' or 'both'. `summary()` -> what it saw."""

    THRESHOLD = 0.9

    def __init__(self, profile: dict, model: Mlp | None, mode: str = "both"):
        self.profile, self.mlp, self.mode = profile, model, mode
        self.rules = Rules(profile)
        self.win: list[tuple[int, int, bytes]] = []
        self.next_close = WINDOW_MS - 1
        self.streak, self.mlp_hits = 0, 0
        self.mlp_first: int | None = None
        self.first_alert: dict | None = None
        self.windows = 0

    @classmethod
    def default(cls, spec=True) -> Ids:
        mode = spec if isinstance(spec, str) else "both"
        model = Mlp(json.loads(MODEL.read_text(encoding="utf-8"))) if mode != "rules" and MODEL.exists() else None
        return cls(json.loads(PROFILE.read_text(encoding="utf-8")), model, mode)

    def observe(self, t: int, frames: list) -> None:
        for f in frames:
            self.win.append((t, f.can_id, f.data))
            if self.mode != "mlp":
                self.rules.frame(t, f.can_id, f.data)
        if t >= self.next_close:
            if self.mlp is not None and self.mode != "rules":
                p = float(self.mlp.predict(window_features(self.win, t, self.profile))[0])
                self.windows += 1
                self.streak = self.streak + 1 if p > self.THRESHOLD else 0
                self.mlp_hits += p > self.THRESHOLD
                if self.streak >= 2 and self.mlp_first is None:   # two windows in a row
                    self.mlp_first = t
            self.win, self.next_close = [], t + WINDOW_MS

    def summary(self) -> dict:
        rule_alerts = {k: v for k, v in self.rules.alerts.items() if k != "gap"}   # an outage alone is not an intrusion
        firsts = [("rule:" + k, v["first_ms"]) for k, v in rule_alerts.items()]
        if self.mlp_first is not None:
            firsts.append(("mlp", self.mlp_first))
        first = min(firsts, key=lambda x: x[1]) if firsts else None
        return {"mode": self.mode, "alerted": first is not None, "first_ms": first[1] if first else None, "first_by": first[0] if first else None,
                "rules": rule_alerts, "gap": self.rules.alerts.get("gap"), "mlp_windows": self.mlp_hits}
