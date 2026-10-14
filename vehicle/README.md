# vehicle/: the chassis as a physical witness (phase P5)

Design note: [`docs/P5_WITNESS_PLAN.md`](../docs/P5_WITNESS_PLAN.md); link contract: [`docs/P5_LINK_CONTRACT.md`](../docs/P5_LINK_CONTRACT.md). The chassis has no speed
sensor and no forward obstacle sensing yet, so P5 starts as a witness of recorded runs and gets more real as sensors are fitted. AEB is the goal; the word is gated
(section 5 of the note).

| File | What |
|---|---|
| `p5_scenarios.json` | the two P5 scenarios (cold start to 20 km/h, command stream lost at 4.5 s: for good, and for 300 ms); run with `python run.py --scenario-file vehicle/p5_scenarios.json [--defect no_latch]` |
| `p5_runs.py` | stage 0: runs them on the reference controller (correct build, `no_latch`, `long_timeout`) and derives the numbers from the traces; the 1:k chassis mapping is a labelled placeholder |
| `tools/make_timeline.py` | the contrast timeline as PNG (and `--clip` for an MP4); `pip install matplotlib` (the `p5` extra), ffmpeg for the clip |
| `link.py` | stage 1: the wire format (frames, CRC-16, five message types) and the golden frames (`python -m vehicle.link` rewrites `link_vectors.json`) |
| `fake_chassis.py` | the chassis side as a pure state machine (`ChassisCore`: DISARMED, ARMED, LATCHED_STOP, dead-man, single writer) and `UdpChassis`, the same core on a real UDP socket |
| `replay.py` | the recorded trace sampled at 20 Hz and mapped to PWM (placeholder scale), joined with the inject time, state changes and expected outcome; one cap per plan |
| `bridge.py` | the laptop side: refuses to arm a wrong build or a latched chassis, streams the plan, its own 100 ms watchdog, STOP in a `finally`; command line below |
| `simnet.py` | bridge and chassis on a virtual clock (1 ms steps) with scripted faults: lost datagrams, a dead bridge, a Wi-Fi drop, stale or foreign frames |
| `conformance.py`, `link_cases.json` | 27 hand-written cases for the chassis side (3 for the wheel encoders of link version 2); the stage 2 firmware port must pass the same file |
| `out/` | the generated PNGs (the MP4 is git-ignored) |

```
python -m vehicle.bridge --sim  --scenario p5_planned_stop_link_lost_300ms --variant no_latch   # virtual clock, instant
python -m vehicle.bridge --fake --scenario p5_planned_stop_link_lost                             # real time, UDP loopback, fake chassis (14 s)
python -m vehicle.conformance                                                                    # the chassis cases against the Python core
```

`--out DIR` writes `bridge_log.jsonl` (every datagram, the bridge's own clock) and `manifest.json` (plan header with the source-trace hash, outcome, statistics, and a
label saying the chassis was `virtual` or `fake-udp`: not a physical result). A non-loopback chassis needs `--allow-real` and belongs to stage 2.

Stages 0 and 1 need no hardware. Tests: `python -m unittest tests.test_p5 tests.test_p5_link`.
