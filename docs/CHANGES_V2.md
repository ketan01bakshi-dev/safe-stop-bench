# Safe-stop bench v2: what changed, item by item

Mapped to `IMPROVEMENTS.md` (v1). **Status:** ✅ done and tested · 🟡 written but untested, or partial · ❌ not done (reason given).
Built 2 Oct 2026. v1 is frozen at `..\safe_stop_bench_v1_backup_2026-10-02\` (also the zip and the git tag `v1.0`).

## Headline results (default config)

| | v1 | v2 |
|---|---|---|
| Scenarios | 11 | **52** (41 hand-written + 11 boundary) |
| Pass | 11/11 | **51/52 + 1 known finding** |
| Seeded bugs (mutants) | 3 | **14, mutation score 100%** |
| Stopping distance from 30 km/h | 11.6 m (no brake delay) | **~16 m** (80 ms dead time + lag + jerk-limited ramp) |
| Vehicle model | 1-D | **Bicycle model** (lateral position, heading) |
| Buses | byte strings | **Virtual CAN FD + classic CAN** with arbitration, capacity, latency, jitter, bus-off |
| E2E | "Profile 5-style", 3-in-a-row | **Profiles 5 and 2**, windowed state machine |

## Findings the bench surfaced (the point of building it)

1. **Jitter above the frame period breaks E2E.** 30 ms jitter on a 20 ms frame period reorders frames; the sequence check rejects them and the vehicle stops (SC-37b). Either keep jitter below the period or add a reorder buffer.
2. **The effective outage tolerance is shorter than the timeout.** After a gap larger than max delta, the first frame is rejected (WRONG_SEQUENCE), so a 100 ms timeout tolerates only a 60 ms outage (boundary cases BV-timeout-60/80).
3. **False stops at high bus error rates.** At 0.5% CRC errors, two corrupted frames in a row make the next frame's counter jump count as a third error: 1 false stop in 4.8 km. Tune the E2E window or max delta against the real error rate.
4. **An uphill grade hides a weak brake.** Measured deceleration includes gravity; without grade compensation the 30%-brake fault goes undetected on an 8% slope (mutant `no_grade_compensation`).
5. **At 40 km/h, a stop in lane drifts ~1 m without lane information**, even with heading hold (known finding). Steering-stuck cases at 40 km/h drift the same way.
6. **Off-road, the weak-brake check misses its 600 ms budget by 20 ms** (known finding in `config/offroad.json`).
7. **Defence in depth works:** with the hardware watchdog removed, a hung planner is still caught by the Q&A watchdog.

## Item by item

### 0. v1 limitations
| # | Status | How |
|---|---|---|
| L1 1-D model | ✅ | Kinematic bicycle model (`plant.Vehicle`); lateral deviation is a pass criterion |
| L2 no actuator dynamics | ✅ | Dead time, first-order lag, friction limit, steering rate limit (`plant.Actuators`) |
| L3 one reaction | ✅ | DEGRADED → PULL_OVER → STOP_IN_LANE → BRAKE_ONLY_STOP / BACKUP_BRAKE_STOP |
| L4 run-length E2E | ✅ | Windowed state machine (`e2e.E2EStateMachine`); v1 logic kept as mutant `e2e_run_length` |
| L5 cycle-rounded timing | 🟡 | Still 10 ms cycle granularity (realistic for a 10 ms controller); detection is measured from the first state change |
| L6 one-seed false stops | ✅ | `false_stop_rate`: N seeds, km driven, rate per 100 km, 95% upper bound |
| L7 safety-controller supply | ✅ | `safety_brownout` scenario: reset → latched stop |
| L8 byte strings | ✅ | `bus.VirtualBus` (arbitration, capacity, latency, jitter, flooding, bus-off) |
| L9 illustrative limits | 🟡 | Still illustrative, but now in `config/*.json` with an FTTI derivation sheet and a hazard chain (`safety/hazards.json`) |
| L10 no DUT interface | ✅ / 🟡 | Interface + reference adapter ✅; **CAN adapter ✅ tested** (v2.1); **FMU adapter ✅ tested** (v2.2) |

### 1. Architecture
| # | Status | Notes |
|---|---|---|
| 1.1 DUT interface + adapters | ✅ reference · **✅ CAN (v2.1)** · **✅ FMU (v2.2, FMPy)** · ❌ ROS 2, shared library | ROS 2 needs a ROS install; a ctypes adapter needs a real header to be meaningful |
| 1.2 Scenarios as data | ✅ | `scenarios/scenarios.json` |
| 1.3 Config files | ✅ | `config/default.json`, `config/offroad.json` (inherits and overrides) |
| 1.4 Separate oracle | ✅ | `ssb/oracle.py` |
| 1.5 Event log | ✅ | Per-scenario timeline in the report |
| 1.6 Pluggable plant | 🟡 | Plant isolated in `plant.py`; no second plant implementation yet |

### 2. Safety controller
| # | Status | Notes |
|---|---|---|
| 2.1 Reaction ladder | ✅ | |
| 2.2 Manoeuvre by what still works | ✅ | Pull-over (planner-executed), stop in lane, brake-only, backup brake |
| 2.3 Commanded vs measured | ✅ | Steering and braking, grade-compensated |
| 2.4 E2E state machine | ✅ | |
| 2.5 Profile 2 | ✅ | Used on the actuator bus |
| 2.6 E2E on outputs + ECU check | ✅ | Actuator ECU checks Profile 2 and has its own fallback stop |
| 2.7 Q&A watchdog | ✅ | Challenge every 20 ms; the answer travels in the command frame |
| 2.8 Speed-dependent envelope + jerk | ✅ | |
| 2.9 Jerk-limited stop | ✅ | |
| 2.10 Release procedure | ✅ | Stopped + fault clear 500 ms + operator |
| 2.11 State / cause / DTC outputs | ✅ | Exposed via `Outputs` |
| 2.12 Startup safe state | ✅ | INIT holds until 2 valid frames |
| new | ✅ | Heading hold from the yaw-rate sensor during the stop |

### 3. Plant
| # | Status | Notes |
|---|---|---|
| 3.1 Bicycle model | ✅ | |
| 3.2 Actuator dynamics | ✅ | |
| 3.3 Friction and grade | ✅ | Per config and per scenario |
| 3.4 Several start speeds | ✅ | Sweep 10–40 km/h |
| 3.5 Mass / load | ❌ | Low value with a kinematic model; needs a dynamic model first |
| 3.6 CARLA coupling | ❌ | Large external dependency; out of scope for a laptop sketch |

### 4. Faults (all ✅ unless noted)
Wrong data ID · counter wrap · gap = max delta · one gap over · repeated gaps · jitter (15 ms OK, 30 ms finding) · late frames · reorder · intermittent link · scattered CRC errors · two faults at once · fault during the stop · startup with no frames · safety-controller brownout · steering stuck / slow · weak brake · bus flooding · actuator bus-off · stale timestamps · **clock drift** · Q&A wrong answers.

### 5. Test design
| # | Status | Notes |
|---|---|---|
| 5.1 Boundary values | ✅ | Steering rate at 3 speeds, outage length, ODD speed |
| 5.2 Sweeps | ✅ | 4 faults × 4 speeds × 3 frictions = 48 runs |
| 5.3 False-stop rate | ✅ | |
| 5.4 Fuzzing | ✅ | Random fault combinations, invariants only (stdlib, not Hypothesis) |
| 5.5 Requirement IDs + traceability | ✅ | |
| 5.6 Coverage matrix | ✅ | By goal and layer (SiL filled; HiL / vehicle empty by design) |
| 5.7 Mutants + score | ✅ | 14 mutants |
| 5.8 Back-to-back | ✅ | `--b2b <mutant>` as a stand-in for a supplied vECU |

### 6. Realism / hardware
| # | Status | Notes |
|---|---|---|
| 6.1 Real CAN frames + DBC | ✅ (v2.1) | python-can frames between processes; every message encoded/decoded with cantools from `dbc/safe_stop.dbc`; a unit test checks the DBC matches the packed bytes |
| 6.2 CAN FD + bus load | ✅ | Bus load reported |
| 6.3 Wall-clock mode | ✅ (v2.1) | `--dut can` paces 1 simulated ms per wall-clock ms; `--repeat N` reports min/typ/max detection and the worst-case margin |
| 6.4 ESP32-S3 HiL | 🟡 (v2.3) | Firmware, C++ core, link protocol, PiL/HiL adapters built; core proven exact on the PC; **not yet run on the boards** (none connected) |
| 6.5 PSU + relay board | ❌ | Hardware |
| 6.6 Field-log replay | ✅ / 🟡 | Replay works; the log is **synthetic** (`logs/sample_field_log.csv`) |

### 7. Metrics and reporting
| # | Status |
|---|---|
| 7.1 Invariants every ms | ✅ |
| 7.2 Per-requirement FTTI | ✅ |
| 7.3 More KPIs (jerk, lateral, time in state, margin) | ✅ |
| 7.4 JUnit XML | ✅ |
| 7.5 Timeline per scenario | ✅ |
| 7.6 Run-to-run diff | ✅ (against the previous `*_results.json`) |
| 7.7 CSV traces | ✅ (MDF ❌: needs asammdf) |

### 8. Engineering
| # | Status | Notes |
|---|---|---|
| 8.1 Git repo + CI | ✅ GitHub, CI on every push | |
| 8.2 CLI options | ✅ | |
| 8.3 pyproject / lint | ✅ pyproject · 🟡 ruff/mypy configured, not run (not installed) | |
| 8.4 Hashes in reports | ✅ | Bench + DUT sha256 |
| 8.5 Dockerfile | 🟡 | Written, not built (no Docker here) |

### 9. Standards structure
| # | Status |
|---|---|
| 9.1 Hazard → goal → requirement → scenario | ✅ |
| 9.2 FTTI derivation sheet | ✅ (3 worked examples, illustrative) |
| 9.3 SOTIF catalogue | ✅ (6 triggering conditions with coverage status) |
| 9.4 Bench confidence evidence | ✅ (unit tests + mutation score) |
| 9.5 Safety-case fragment | ✅ (generated per goal) |

### 10. AI assistance
| # | Status | Notes |
|---|---|---|
| 10.1 LLM drafts scenarios | ❌ | Deliberately not wired: cost-first, and the gated pipeline already exists in the HIL ML Ops POC |
| 10.2 LLM drafts RCA | ❌ | Same reason |
| 10.3 Gap finder | ✅ | Rule-based (requirements without scenarios + SOTIF items out of scope) |

## v2.1 (2 Oct 2026): CAN adapter made real and tested

**What:** the reference safety controller now also runs as a **separate process** (`python -m ssb.vecu_process`) that the bench reaches **only over CAN**: python-can frames, every message defined in `dbc/safe_stop.dbc` and encoded/decoded with cantools. This is exactly how a supplied vECU would be tested: a black box plus a DBC.

- **Transport:** python-can `udp_multicast` (processes on one PC, no hardware). For a USB-CAN adapter or Linux SocketCAN, change `interface`/`channel`; nothing else changes.
- **Black-box observation:** detection time, timeline and release outcome are derived **only from status frames seen on CAN**, not from the controller's internals.
- **Time base:** the bench drives a shared time (`VEH_SimTime`) and paces itself in real time, so measured timing includes real transport and scheduling.
- **New messages in the DBC:** watchdog kick (a GPIO line on a real vehicle, carried on CAN here), safety status (state, cause, challenge, MRM request), vehicle feedback, and a test-harness control frame.
- **Environment:** `.venv` with python-can 4.6.1, cantools 44.1.0, msgpack (python-can's dependency for multicast). The in-process bench still needs only the standard library.

**Findings while building it (each fixed and documented in `ssb/canio.py` / `ssb/vecu_process.py`):**
1. **UDP multicast delivered every frame twice** (two network paths), sometimes 60–90 ms late. A duplicate looks like a REPEATED E2E frame, so it caused false stops. python-can stamps the receive time, so copies couldn't be matched by timestamp. Fix: each sender stamps the preserved `channel` field with `pid:sequence`; receivers drop repeats. Exact, with no time window.
2. **Windows sleep granularity (~10 ms) made the vECU run at ~100 Hz.** A 10 ms controller then batches cycles and misjudges timeouts. Fix: no sleep in the vECU loop.
3. **A late duplicate of an old time message looked like a 16-bit clock wrap**, jumping the vECU clock by 65 s. Fix: only treat a drop of more than half the range as a wrap; ignore older messages.
4. **The multicast route can briefly disappear** (WinError 10065) when network adapters change. Fix: bounded send retry with a counter.
5. **Zombie processes from killed test runs share the group** and inject stale frames. Lesson: always close the DUT (`CanDUT.close()` in a `finally`), and check for stray processes before a campaign.

Lessons 1–4 matter for any PC-based vECU test setup, not only this one.

## v2.2 (2 Oct 2026): FMU adapter made real and tested

**What:** the bench now drives a supplied vECU delivered as an **FMI 2.0 co-simulation FMU**, the most likely delivery format. Full write-up: [`docs/FMU_ADAPTER.md`](FMU_ADAPTER.md).

- **`ssb/fmu_contract.py`:** the bench's FMU contract, by bench name: raw DBC signals (so E2E stays testable bit for bit), Rx/Tx counters as frame events, and required vs optional signals.
- **`FmuDUT` (`ssb/dut.py`):** loads the FMU with FMPy, maps names, units and state enumerations through a JSON mapping file, reports every mapping problem at once, observes the FMU as a black box (`BlackBoxObserver`, now shared with `CanDUT`), uses `fmi2Reset` between scenarios, and runs deterministically, not in real time.
- **`ssb/fmu_inspect.py`:** intake before any test: full FMI validation, CS/ME, platform binaries, the variable list, a proposed mapping (with an automotive abbreviation dictionary, every guess marked CHECK, and units flagged), and a lifecycle check repeated in child processes.
- **Two test FMUs (`scripts/build_fmu.py`, PythonFMU):** `fmu/SafeStopVecu.fmu` (the reference controller, bench names, mutant hook) and `fmu/SupplierStyleVecu.fmu` (the same controller with supplier names, km/h, OFF = 0 state codes and no test hooks; usable only through `fmu/mapping_supplier_style.json`).
- **`run.py --dut fmu [--file --mapping] [--b2b-dut] [--defect …]`**; `--b2b-dut` is **exact** for an FMU (state timeline to the millisecond plus the plant's end state).
- **Oracle:** new check "cold start begins in INIT" (SR-06). **Tests:** 5 new FMU tests (18 in total), skipped cleanly without FMPy.

**Results:** see `docs/FMU_ADAPTER.md` §5.

**Findings while building it** (details in `docs/FMU_ADAPTER.md` §6): (1) the export tool's defaults wrote an invalid model description, twice; (2) free → re-instantiate is an access violation, so the adapter uses `fmi2Reset`; (3) an intermittent two-instance crash, 8 runs in 10, so lifecycle checks repeat; (4) plain name matching swapped acceleration and speed, fixed with an abbreviation dictionary and best-first assignment; (5) **an oracle blind spot that only back-to-back exposed**: the supplier-style FMU cold-started in NORMAL and the matrix said PASS; (6) two frames can land in the same millisecond; (7) raw vs physical signals decide whether E2E is testable.

## v2.3 (2 Oct 2026): ESP32-S3 HiL, built and proven up to the USB cable

**What:** the safety controller ported to portable C++ (`hil/SafeStopCore`), two firmwares for the existing EdgeHex ESP32-S3 + MCP2515 lab (`hil/firmware/SafetyNode` = board B, the device under test; `hil/firmware/BusNode` = board A, the actuator-bus node), a framed USB link protocol, and four new `--dut` levels: `native`, `loopback`, `pil`, `hil`. Full write-up: [`docs/HIL_ESP32.md`](HIL_ESP32.md).

- **One source of truth:** everything except the Arduino shim is in the core. It is compiled for the PC (`scripts/build_native.py`, zig C++ from PyPI) and proven against the Python reference before it goes on the chip.
- **Proven:** C++ core exact back-to-back **52/52** (default and off-road); board B's node logic over the link protocol exact **52/52** on the 10 ms status grid; all 14 seeded mutants fail the same scenarios in C++ as in Python (`scripts/native_mutation_parity.py`); firmware builds for the ESP32-S3 with 0 warnings; 5 new tests.
- **Measured on hardware once connected:** worst execution time per 10 ms cycle (µs), task lateness, CAN transmit failures, detection timing with real transport.
- **Not done yet:** PiL/HiL runs on the boards (none on a COM port today); real power cut; CAN FD planner link (the MCP2515 is classic CAN only); GPIO watchdog wire fitted.

**Findings while building it** (details in `docs/HIL_ESP32.md` §8): (1) the lab's CAN controller can't carry the planner's CAN FD frame, so that link runs over USB with the same bytes; (2) a black box shows a state change only on its next status frame, so exact comparisons need the DUT's reporting grid; (3) four porting rules gave bit-exactness on the first full run (half-to-even rounding, non-negative modulo, Python's tie behaviour of max/min, no fused multiply-add); (4) the S3's FPU is single-precision, so `double` runs in software (execution time to be measured); (5) opening a COM port can reset an ESP32 (DTR/RTS); (6) **one board alone can never get a CAN ACK**, so treating transmit error-passive as a bus fault would stop every single-board run; bus monitoring is now a reset option, on for two boards.

