# E2E false-stop tuning (v2.6, 5 Oct 2026)

Closes the v2.0 finding "false stops at high bus error rates" (`docs/CHANGES_V2.md`, finding 3). Synthetic, illustrative rates.

## The problem

At 0.5% random CRC errors, the bench saw about 1 false stop in 4.8 km. The mechanism is a **double count**:

```
frame n     OK
frame n+1   WRONG_CRC         error 1
frame n+2   WRONG_CRC         error 2
frame n+3   counter jumps by 3 > max delta 2  → WRONG_SEQUENCE   error 3   → window "> 2 errors in 6" → INVALID → stop
```

One noise burst of two frames is counted as three errors. The counter jump is a *consequence* of the two rejected frames, not new evidence.

## Method

`python -m ssb.e2e_tuning` (about 10 min) → `reports/e2e_tuning.md` / `.json`. For each candidate:

1. **False stops:** a Monte Carlo through the **real** `ssb.e2e` receiver and state machine (the classes the controller uses). 1M planner frames per error rate (20 ms period ≈ 167 km at 30 km/h), random independent corruption, warm restart after each trip.
2. **Detection of a degraded stream:** frames to INVALID when each frame is corrupted with probability 20 / 35 / 50 / 100%, against the SR-02 FTTI (250 ms = 12 frames).
3. **The whole scenario matrix** with the candidate's settings.

## Results

| Candidate | False stops /1000 km at 0.5% | at 1% | Dead stream (100%) | 50% corrupted, within FTTI | Matrix |
|---|---|---|---|---|---|
| v2.0 default (max delta 2, > 2 errors in 6) | 216 | 624 | 3 frames | 94.5% | 52/52 |
| max delta 3 | 6 | 54 | 3 frames | 93.0% | 50/52: **misses repeated counter jumps of 3** |
| max delta 4 | 6 | 54 | 3 frames | 93.0% | 49/52 |
| > 3 errors in 6 | 6 | 24 | 4 frames | 82.5% | 49/52: **misses intermittent CRC**, slower (70 ms) |
| **explained gaps** (chosen) | **6** | **54** | **3 frames** | **93.0%** | **52/52** |
| explained gaps + > 3 in 6 | 0 (< 18) | 6 | 4 frames | 68.5% | 49/52 |

**Confirmation with 5M frames (833 km per point):** v2.0 default 147.6 / 561.6 per 1000 km at 0.5% / 1% errors. Explained gaps: **12.0 / 66.0**. That is **about 12× fewer false stops at 0.5%** and 8.5× fewer at 1%. The 1M-frame figure of 6 was one event: too few to quote.

## The change: "explained gaps"

`ssb/e2e.py` `_Receiver(explain_gaps=True)`, and the same in the C++ core (`p5_check`): the receiver counts the CRC failures since the last frame that passed the CRC. A counter jump up to `max_delta + that count` is `OK_SOME_LOST`, not `WRONG_SEQUENCE`. The CRC failures themselves still count as errors. A jump that the rejected frames don't explain is still an error.

- It is now the default (`config/default.json`: `e2e_explain_gaps: true`, `e2e_max_delta: 2`, `e2e_max_err_valid: 2`). Python can switch it off for comparison; the C++ core has it built in.
- **Why not simply raise max delta or the error budget:** both also accept *real* sequence faults or slow detection. Max delta 3 no longer detects repeated counter jumps of 3 (`counter_gap_repeated`); a budget of 3 errors misses `intermittent_crc` and detects 20 ms later. Explained gaps removes only the double count.
- **New regression scenarios:** `crc_burst_two` (two corrupted frames in a row → NORMAL; it FAILs with the v2.0 rule) and `crc_burst_three` (three in a row → still a stop in 50 ms).
- **Parity:** C++ core exact back-to-back 52/52 on default and off-road; all 14 seeded mutants identical in Python and C++; C FMU and native core exact on the burst and noise scenarios.

## Findings

1. **One fault counted twice is the commonest false-stop source in windowed E2E checks.** Fix the counting before you tune thresholds; threshold tuning trades detection away.
2. **Degraded-stream detection was never guaranteed by the FTTI, before or after.** With 20% of frames corrupted, only ~27–33% of cases go INVALID within 250 ms (median 19–24 frames). A heavily degraded link is caught (100% corrupted: 3 frames, every time), and 80% of commands still arrive valid at 20% corruption, so the hazard is limited. But SR-02 should say what it covers: *a stream with more than X% bad frames is detected within Y*. That is a requirement gap.
3. **0.5% CRC errors is far above a healthy CAN bus**: CAN retransmits frames that fail its own bus CRC, so an E2E CRC failure points to corruption outside the bus (gateway, memory, software). The tuning is about robustness margin, not the expected field rate. Measure the field rate (DTC counters for E2E errors) before calibrating.

## Honest limits

- Independent random corruption only; real error bursts (EMC events) are correlated. A burst model is the next step.
- The ESP32 firmware was rebuilt only for the PC (native/FMU); it is not reflashed (no boards connected).
