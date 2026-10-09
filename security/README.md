# security/: attacks and intrusion detection (phase P3)

The question: which attacks does E2E catch, which only the intrusion detector (IDS) catches, which reach a safe stop first, and which does nobody see?

| File | What it does |
|---|---|
| `attacks.py` | An attacker node on the planner bus (every bench level, including HiL where the frames reach board B over USB): `flood`, `fuzzy`, `spoof_invalid`, `spoof_valid` (inject or takeover), `replay`, `period_glitch`, `signal_ramp`. It sniffs the real stream (counter, watchdog challenge), so a forged command carries a correct CRC, counter, data ID, fresh timestamp and watchdog answer. A scenario lists one as `{"type": "attack", "kind": ..., "start": ms, ...}`. |
| `ids.py` | The detector: learned rules (ID, length, period, counter step, signal range and slew, timestamp age, load) plus a small numpy MLP over 100 ms windows. It never checks the CRC, so it does not duplicate E2E. |
| `train.py` | `python -m security.train` learns `profile.json` from clean traffic only, trains `model.json`, and judges both on attacks with parameters the MLP never saw. Result: `TRAINING.md`. |
| `matrix.py` | `python -m security.matrix [--dut hil ...]` runs `scenarios/security.json` (SC-60..71, requirements SEC-01..05) and writes `reports/security/attack_matrix_<dut>.md`: E2E rejected? IDS alert (when, by which rule)? safe stop (when)? who was first? |

## What the matrix shows (reference controller)

* Everything that is structurally wrong (junk IDs, wrong length, extra or duplicated frames, a counter that does not step, a replay) is seen by a **rule within 1-14 ms**, well ahead of the safety layer (50-410 ms). The MLP adds **nothing the rules miss** on this bench; it alerts later (about 200 ms, two windows in a row).
* **The CRC is not a secret.** A forged command with a correct CRC and counter is accepted by E2E. What stops it is the envelope (SR-07..09) and, for an injected one, the repeated counter.
* **Replay with the counter aligned** (exactly 256 planner periods, 5.12 s, old): E2E rejects nothing, because the counter matches again. The **freshness check** (STALE_DATA) stops it. The rules see it too (the content jumps).
* **Fuzzing** is stopped only after ~410 ms (the E2E window tolerates sparse garbage by design, SR-18), later than SR-02's 250 ms: known finding `sec_fuzzy`. The IDS alerts at once.
* **A defect in the controller, found by SC-68 and fixed (v2.24):** a second valid command in the same 10 ms cycle skipped the steering rate limit, so a forged frame right behind the real one moved the output steering 25 deg in one period. Fixed in `ssb/safety.py` and `ssc_core.cpp`; back-to-back exactness re-proved.
* **Open finding, SEC-05 (`sec_signal_ramp`):** a man-in-the-middle that adds a slow steering drift (2 deg/s) to the real command is seen by **nobody**: E2E accepts every frame, the IDS finds every frame normal, the envelope never trips. In 12 s the vehicle ends 4.8 m off its path. The mimic that copies the real values exactly (`sec_mimic`) is invisible by construction. Both need a check against vehicle feedback (yaw rate, lane position), not a bus rule.

## On the two boards (v2.25)

`python -m security.matrix --dut hil -- --port-b COM13 --port-a COM14 --kick gpio` (SafetyNode 2.8, BusNode 2.8): **12/12 as on the reference controller**, with stop times within 1-10 ms of it (for example SC-68 70 ms, SC-61 100 ms, SC-63 411 ms vs 410). The attacker's frames are sent to board B over USB like the real planner's, so the real C++ core on the real chip took the forged commands; the IDS ran on the PC from the same stream. Board B reports no E2E counter, so that column reads "n/a (black box)" at this level. Results: `reports/security/attack_matrix_reference.md` and `attack_matrix_hil.md`.

Flashing the controller fix to B (SafetyNode 2.7 -> 2.8) came first; full flash images of the previous firmware are in `hil/build/rollback_2026-10-09/`.

## Limits

Trained and judged on this bench's synthetic traffic: ~22 clean runs, 12 attack parameter sets held out. Zero false alarms on 22 held-out clean runs says little about a real vehicle's variety. The attacker is given sniffing access to the bus and, for takeover, the ability to remove the real frames (a compromised gateway). The physical-bus attacks on board B's outputs (0x200 / 0x201) are a separate experiment on the boards.
