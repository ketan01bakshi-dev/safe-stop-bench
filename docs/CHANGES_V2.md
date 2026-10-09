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
3. **False stops at high bus error rates** (fixed in v2.6, see `docs/E2E_TUNING.md`). At 0.5% CRC errors, two corrupted frames in a row make the next frame's counter jump count as a third error: 1 false stop in 4.8 km. Tune the E2E window or max delta against the real error rate.
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
| 1.1 DUT interface + adapters | ✅ reference · **✅ CAN (v2.1)** · **✅ FMU (v2.2, FMPy)** · **✅ shared library (v2.3, `NativeDUT`, ctypes on the C++ core's C API)** · **✅ compiled C FMU (v2.4)** · **✅ ROS 2 (v2.8, rclpy, WSL)** | see `docs/ROS2_ADAPTER.md` |
| 1.2 Scenarios as data | ✅ | `scenarios/scenarios.json` |
| 1.3 Config files | ✅ | `config/default.json`, `config/offroad.json` (inherits and overrides) |
| 1.4 Separate oracle | ✅ | `ssb/oracle.py` |
| 1.5 Event log | ✅ | Per-scenario timeline in the report |
| 1.6 Pluggable plant | ✅ (v2.5) | `make_vehicle`: kinematic (default) or `DynamicVehicle` (`--plant dynamic`) |

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
| 3.5 Mass / load | ✅ (v2.5) | Payload, rolling resistance, drag, true slope; `--load-sweep`; see `docs/DYNAMIC_PLANT.md` |
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
| 6.4 ESP32-S3 HiL | ✅ (v2.9) | **Run on the boards 2026-10-06:** PiL 53/54 + 1 known, HiL 52/54 + 1 known, back-to-back 54/54 both; see `docs/HIL_FIRST_RUN.md` |
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
| 7.7 CSV traces | ✅ · **MDF4 ✅ (v2.7, `run.py --mdf`, asammdf)** |

### 8. Engineering
| # | Status | Notes |
|---|---|---|
| 8.1 Git repo + CI | ✅ GitHub, CI on every push | |
| 8.2 CLI options | ✅ | |
| 8.3 pyproject / lint | ✅ pyproject · **✅ ruff + mypy clean (v2.7)** | `python -m ruff check .` · `python -m mypy ssb` |
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

**Findings while building it** (details in `docs/FMU_ADAPTER.md` §6): (1) the export tool's defaults wrote an invalid model description, twice; (2) free → re-instantiate is an access violation, so the adapter uses `fmi2Reset` (**corrected in v2.4:** a bug in the bench's own lifecycle test, not in PythonFMU); (3) an intermittent two-instance crash, 8 runs in 10, so lifecycle checks repeat; (4) plain name matching swapped acceleration and speed, fixed with an abbreviation dictionary and best-first assignment; (5) **an oracle blind spot that only back-to-back exposed**: the supplier-style FMU cold-started in NORMAL and the matrix said PASS; (6) two frames can land in the same millisecond; (7) raw vs physical signals decide whether E2E is testable.

## v2.3 (2 Oct 2026): ESP32-S3 HiL, built and proven up to the USB cable

**What:** the safety controller ported to portable C++ (`hil/SafeStopCore`), two firmwares for the existing EdgeHex ESP32-S3 + MCP2515 lab (`hil/firmware/SafetyNode` = board B, the device under test; `hil/firmware/BusNode` = board A, the actuator-bus node), a framed USB link protocol, and four new `--dut` levels: `native`, `loopback`, `pil`, `hil`. Full write-up: [`docs/HIL_ESP32.md`](HIL_ESP32.md).

- **One source of truth:** everything except the Arduino shim is in the core. It is compiled for the PC (`scripts/build_native.py`, zig C++ from PyPI) and proven against the Python reference before it goes on the chip.
- **Proven:** C++ core exact back-to-back **52/52** (default and off-road); board B's node logic over the link protocol exact **52/52** on the 10 ms status grid; all 14 seeded mutants fail the same scenarios in C++ as in Python (`scripts/native_mutation_parity.py`); firmware builds for the ESP32-S3 with 0 warnings; 5 new tests.
- **Measured on hardware once connected:** worst execution time per 10 ms cycle (µs), task lateness, CAN transmit failures, detection timing with real transport.
- **Not done yet:** PiL/HiL runs on the boards (none on a COM port today); real power cut; CAN FD planner link (the MCP2515 is classic CAN only); GPIO watchdog wire fitted.

**Findings while building it** (details in `docs/HIL_ESP32.md` §8): (1) the lab's CAN controller can't carry the planner's CAN FD frame, so that link runs over USB with the same bytes; (2) a black box shows a state change only on its next status frame, so exact comparisons need the DUT's reporting grid; (3) four porting rules gave bit-exactness on the first full run (half-to-even rounding, non-negative modulo, Python's tie behaviour of max/min, no fused multiply-add); (4) the S3's FPU is single-precision, so `double` runs in software (execution time to be measured); (5) opening a COM port can reset an ESP32 (DTR/RTS); (6) **one board alone can never get a CAN ACK**, so treating transmit error-passive as a bus fault would stop every single-board run; bus monitoring is now a reset option, on for two boards.

## v2.4 (5 Oct 2026): compiled C FMU from the C++ core

**What:** `fmu/SafeStopVecuC.fmu`, the C++ safety core (`hil/SafeStopCore`, unchanged) behind a hand-written FMI 2.0 co-simulation C API (`fmu/c_src/SafeStopVecuC.cpp`). This is what a supplier actually ships: `modelDescription.xml` + `binaries/win64/SafeStopVecuC.dll`, **no Python inside** (the v2.2 PythonFMU embeds the host's Python). Same variables and names as the PythonFMU reference, so the identity mapping drives it with no bench changes. Full write-up: `docs/FMU_ADAPTER.md` §8.

- **`scripts/build_c_fmu.py`:** ONE table (from `ssb/fmu_contract.py`) generates both `fmu/c_src/fmu_gen.h` (value references, start values, both calibrations) and `modelDescription.xml`, so the XML and the C code can't drift. The GUID is a content hash of interface + calibrations + source. zig c++ from PyPI, the same no-FMA flags as the native build. 165 KiB.
- **Proven:** full matrix 51/52 + 1 known (as every DUT); **exact back-to-back vs the in-process Python reference 52/52**; off-road calibration selected through `Bench_Config` exact; seeded bug (`no_latch`) caught inside the FMU; FMI validation clean; intake lifecycle **all four sequences OK**.
- **Speed through FMPy:** 3.7 µs per 1 ms step vs 12.9 µs for the PythonFMU (≈3.5×; most of what remains is FMPy's ctypes call overhead).
- **Tests:** 6 new (`CFmuTests`), 30 in total, all pass.

**Findings while building it:**
1. **A v2.2 finding was wrong: the bench's own test caused it.** "Free → re-instantiate is an access violation" came from the lifecycle test, not PythonFMU: FMPy's `freeInstance()` also unloads the DLL, and the test then called `instantiate()` on the same, now-unloaded wrapper. Fixed in `ssb/fmu_inspect.py` (a fresh wrapper after a free). Re-run: the compiled FMU and the PythonFMU reference pass all four sequences. The supplier-style PythonFMU (a two-module FMU) still fails re-instantiate and is still flaky with two instances (3 in 5); that is inside PythonFMU's embedded Python and is not investigated further. *Lesson: when a tool reports a crash in someone else's binary, prove the harness first. The compiled FMU, which I could reason about line by line, is what made the harness the suspect.*
2. **A hand-written FMI wrapper is small but has rules a generator hides:** parameters (`fixed`) are settable only before initialisation ends; inputs need start values; outputs `initial="exact"`; `fmi2Reset` must restore every start value; and the GUID must match between the XML and the binary (the binary refuses a wrong GUID).
3. **Step size:** the FMU takes any communication step and splits it into 1 ms sub-steps (`canHandleVariableCommunicationStepSize="true"`), so an importer that steps at 10 ms still gets the controller's exact 1 ms behaviour.

## v2.5 (5 Oct 2026): dynamic plant with mass, payload and grade

**What:** `DynamicVehicle` in `ssb/plant.py` (`run.py --plant dynamic [--payload KG] [--load-sweep]`): force-based longitudinal dynamics (curb mass + payload, rolling resistance, aero drag, rotating inertia, true slope, loaded tyre limit). Brake/drive demands are mapped to force on the curb mass (open loop), so a payload scales deceleration. The kinematic model stays the default; nothing earlier changes. Full write-up: [`docs/DYNAMIC_PLANT.md`](DYNAMIC_PLANT.md).

- **Proven:** matrix on the dynamic plant 51/52 + 1 known (the same verdicts); 7 new tests (v²/2a without losses, mass-ratio scaling, loaded tyre limit, downhill + resistances, payload rejected on the kinematic model, the matrix unchanged, the load finding); 37 in total.
- **Load sweep (payload 0–1500 kg × grade 0 / −6%):** planned stop 15.4 → 20.6 m flat (+800 kg), 20.2 → 33.6 m at −6% (+900 kg); weak brake + 1500 kg at −6%: 45.8 m.

**Findings:** (1) load lengthens a planned stop by up to 67% with no diagnosis, and no requirement bounds stopping distance (gap: a stopping-distance budget per ODD); (2) the brake-plausibility check misdiagnoses load as a brake fault at +900 kg on flat ground, but not on −6%: a knife-edge threshold eroded by actuator lag (fix: compare against a lag-filtered demand or a mass estimate); (3) the backup brake is mass-blind too: 3.6× the distance and still a "pass"; (4) the 8 s scenario window hid loaded downhill stops, so the sweep runs 15 s.

**Known test flake (found while verifying v2.5, pre-existing):** the full unit-test run segfaults about 1 run in 3, always after `FmuTests` (the PythonFMU tests) have freed their instances, sometimes during the next test class. The v2.4 tag shows the same rate (1 of 3), so it is not the dynamic plant. Likely the supplier-style PythonFMU's embedded interpreter (see v2.4 finding 1). All tests pass in every run that completes. Fix candidate: run the PythonFMU tests in a child process.

## v2.6 (5 Oct 2026): E2E false-stop tuning

**What:** a tuning campaign (`python -m ssb.e2e_tuning`, a Monte Carlo through the real receiver + the scenario matrix per candidate) and a fix: **"explained gaps"**. A counter jump explained by the CRC failures just before it is frames lost, not a new error. It is the default now, in Python and in the C++ core (bit-exact). Full write-up: [`docs/E2E_TUNING.md`](E2E_TUNING.md).

- **Result:** false stops at 0.5% random CRC errors 148 → 12 per 1000 km (5M frames), at 1% 562 → 66; matrix 52/52 unchanged; a dead stream still detected in 3 frames.
- **Rejected:** max delta 3 (misses repeated counter jumps), an error budget of 3 (misses intermittent CRC, 20 ms slower).
- **New:** scenarios `crc_burst_two` / `crc_burst_three` (54 in total); 2 tests; E2E parameters configurable (`e2e_max_delta`, `e2e_max_err_valid`, `e2e_explain_gaps`).
- **Findings:** fix the double count before tuning thresholds; SR-02 doesn't define detection for a *partly* degraded stream (20% bad frames → only ~30% detected within the FTTI); 0.5% is far above a healthy CAN bus, so measure the field rate before calibrating.
- **Test flake fixed:** the PythonFMU tests now run in a child process (`PythonFmuIsolated`), so a crash in PythonFMU's embedded interpreter can't kill the suite. The full suite was 3 of 3 clean afterwards (it failed 2 of 2 just before). The child still reports a crash as a failure.

## v2.7 (5 Oct 2026): ruff + mypy clean, MDF4 traces

**Lint and types:**
- `pyproject.toml` now sets ruff's rules explicitly: pyflakes, pycodestyle errors, import order and bugbear. The deliberate compact style is ignored and documented: one-line DUT-factory lambdas and `print(...); x = ...` progress lines.
- ruff: 52 findings → 0. mypy (`check_untyped_defs`): 31 → 0 across 23 modules.
- **No behaviour bugs found**, but the tidy-up was worth doing:
  - three dead variables (`runner.py` ×2, `hil_emu.py`);
  - a wrong annotation (`CanDUT(launch=...)` accepts `"reference"`, a command list or `None`);
  - closures now bind their loop variables explicitly;
  - `zip(..., strict=True)` where two lists must match;
  - `raise ... from` in skips;
  - a test that asserted a blind `Exception` now asserts `FMICallException`;
  - asserts that document invariants (a VALID frame carries a payload; the runner's loop runs at least once).
- **Finding: the tidy-up itself introduced a bug, and only the tests caught it.** Renaming an unused loop variable by text match hit the wrong loop in `ssb/hil.py` (`_wait_hello`). That made `p` unbound on the loopback path. Neither ruff nor mypy flagged it, because `p` is assigned later in the same function. `NativeHilTests` failed at once. *Lesson: a lint-only change still gets the full test run before commit.*

**MDF4 traces (`run.py --mdf`, `ssb/mdf_export.py`):**
- One `.mf4` per scenario next to the CSV traces: speed (km/h), lateral offset, distance, and `SAF_State` with a value-to-text table, so CANape or the asammdf GUI shows "STOP_IN_LANE" rather than 4. MDF 4.10, read back in a round-trip test.
- **Python 3.14 install finding:** asammdf hard-depends on `zstd`, which has no wheel and doesn't build for 3.14. Install it with `pip install --no-deps asammdf` plus its other dependencies. `ssb/mdf_export.py` maps `zstd` to the standard library's `compression.zstd` (same `compress`/`decompress`) only if `zstd` is missing.

**Verification:** `run.py --full` 53/54 + 1 known, mutation score 100%, 0 false stops in 5 km; 36 tests pass (+1 MDF), 3 runs.

## v2.8 (5 Oct 2026): ROS 2 adapter

**What:** the safety controller as a ROS 2 node in its own process (`ssb/ros2_vecu_node.py`), driven by `Ros2DUT` (`ssb/ros2_dut.py`) over six `std_msgs` topics (`ssb/ros2_io.py`). The raw E2E frame travels on the planner topic, so E2E stays testable. `run.py --dut ros2 [--ros2-lockstep] [--b2b-dut] [--ros2-no-launch]`. ROS 2 Lyrical installed in WSL Ubuntu 26.04 (`ros2/install_ros2_wsl.sh`). Full write-up: [`docs/ROS2_ADAPTER.md`](ROS2_ADAPTER.md).

- **Proven:** lockstep matrix 53/54 + 1 known; back-to-back 54/54; **exact on the 10 ms grid 54/54**; real time 6/6 on key scenarios when the machine kept time; 2 ROS 2 tests (in WSL; skipped on Windows).
- **Findings:** (1) topics aren't ordered against each other: an untimestamped kick counted 1 ms early and moved a watchdog stop one cycle (52/54 exact), fixed with a time in the kick and sent-counts in the feedback; (2) one topic = one CAN ID, so the adapter must filter: babbler frames were read as corrupt commands, and their backlog leaked past a reset; (3) this laptop stalls up to 69 ms (Windows) / 92 ms (WSL) with no ROS at all, so verdicts come from lockstep and latency needs a quieter machine; `ssb/rt.py` now pins CPUs and pauses the GC on Linux.

## v2.9 (6 Oct 2026): first hardware run, PiL and HiL on two ESP32-S3 boards

**What:** the C++ safety core on board B (SafetyNode), its outputs read by board A (BusNode) through the real 500 kbit/s CAN bus. Full write-up: [`docs/HIL_FIRST_RUN.md`](HIL_FIRST_RUN.md).

- **Results:** PiL 53/54 + 1 known (the FAIL = a 21 ms laptop stall, PASS on rerun); **HiL 52/54 + 1 known**, 0 CAN transmit failures; back-to-back **54/54** in both; worst 10 ms cycle **42–79 µs**, lateness 0–1 ms; worst-case detection over 5 real-time runs within 3 ms of typical, tightest FTTI margin 69 ms (steering stuck).
- **New tools:** `hil/firmware/SpiProbe` (raw MCP2515 register test + driven-vs-floating line test); `SerialLink.reset_board` (a silent, stale node is reset through RTS); an **observer-loss guard** in HiL.
- **Findings:** worn jumpers caused every bring-up fault; a charge-only cable / loose plug looks like a dead board; a stale node is silent; **a dead CAN link once produced 48 false FAILs until the bench learned to call its own fault**; open: an intermittent steering-slew invariant in HiL (1 run in 4), likely a bus-sampling artefact.

## v2.9.1: hardware watchdog line (6 Oct 2026)

- Fitted the GPIO watchdog wire, A GPIO5 → B GPIO5. Full HiL run with `--kick gpio`: **53/54 + 1 known, back-to-back 54/54**, `kick_src: gpio`. WATCHDOG_LATE at 50 ms, WATCHDOG_EARLY at 10 ms, worst cycle 69 µs. Details in `HIL_FIRST_RUN.md`.

## v2.9.2: steering-slew finding fixed (6 Oct 2026)

- Root cause of HIL_FIRST_RUN finding 5: the bench checked B's steering slew on the PC's arrival clock (20 ms samples). One frame arriving 1 ms late made 30 ms of legal 48 °/s re-centring look like 72 °/s in 20 ms. B was correct in every run.
- Fix: `Outputs.cycle` carries (DUT cycle time, commanded steer). In HiL/PiL it comes from SAF_ActuatorCmd's Profile 2 counter (10 ms per count, unwrapped mod 16) and the steering in the same frame. The runner's slew invariant uses it when present. In-process DUTs are unchanged.
- Proof: `SlewTimeBaseTests` (old check reproduces the exact 72 °/s under simulated jitter; new check immune; a 3× slew still caught); `scripts/slew_probe.py` on the boards: 2/8 false fails before, 0/12 after.

## v2.9.3: observer robustness on real hardware (7 Oct 2026)

- **Stale-frame replay filter:** B's MCP2515 replays old SAF_Status frames from a transmit buffer (≈20 a minute after a rewire). These faked 1 ms "STOP → NORMAL" exits from a latched stop (26/54 in one run). Now a bus status frame counts only if B's USB copy shows it was sent within 20 ms, one bus frame per copy. Replays are counted (`bus_replays` per scenario) and reported. `ReplayFilterTests`.
- **Bench faults are not verdicts:** new `BenchFault` (observer lost, A's controller re-initialised). The runner recovers the boards (RTS reset + hello) and reruns the scenario, up to 2 times, and records `bench_faults`.
- **Firmware 2.4:** a 5 ms MCP2515 normal-mode check on both nodes, debounced over three reads (the undebounced first version caused outages). A re-initialises and reports `reinits` / `spi_glitches` in its diagnostics. B treats a confirmed loss as an actuator-bus fault.
- Full HiL: **52/54 + 1 known** (the FAIL was the bench's lag guard; PASS on rerun), b2b **54/54**, 0 invariants, 174 replays filtered.

## v2.9.4: the "replay" root-caused, not a jumper (7 Oct 2026)

- **Stress sketch** `hil/firmware/SpiStress` + `scripts/spi_stress.py`. It separates SO / SI / SCK / CS / VCC-GND faults by their error signature, in loopback or on the real bus (`--normal --port-a`), keeps a never-requested canary in TXB2, and has a guided `--wiggle` mode. B: 0 errors in ~7.6M (loopback) and ~7.2M (bus) transactions. A: 0 in ~7.5M.
- **Root cause (board A):** at the instant a frame lands, the MCP2515 sometimes returns its flag register shifted by one bit (0x02 "RXB1 full" for 0x01), in READ STATUS and in CANINTF alike, at 10/4/1 MHz. The library then read RXB1, which held a frame from hours earlier. `scripts/rx_status_check.py` measures it with sequence-stamped frames.
- **Fix:** `mcp_init` turns rollover off (BUKT = 0), and BusNode 2.5 receives from RXB0 only, re-reading on an RXB1 flag. Stale duplicates per 30 s: **37–50 → 0**; lost frames 0; overflows 0.
- **Mirror filter:** an unvouched SAF_Status now waits up to 20 ms for B's late copy (40 genuine frames were rejected per scenario when B's port was read late). `ReplayFilterTests` covers it.
- Full HiL: **51/54 + 1 known** (both FAILs are the lag guard; rerun PASS), b2b **54/54**, 0 invariants, **0 replays**. Finding 7 in `HIL_FIRST_RUN.md` is corrected (the source was A, not B), and finding 8 is added.

## v2.9.5: host stalls rerun, start-up lag removed (7 Oct 2026)

- **Start order:** the runner builds the planner, buses and vehicle first, then resets the DUT, then starts the clock. Set-up after the reset was a head start for a board with its own clock and showed up as the worst lag at t = 0–1 ms (up to 8.4 ms).
- **GC paused** inside real-time scenarios on Windows too (collected between scenarios).
- **Host stall = bench fault:** bench lag above `LAG_LIMIT_MS` (5 ms) reruns the scenario (up to `BENCH_RETRIES` = 2), recording the stall and the time it happened (`bench_lag_at_ms`). If every attempt stalls, the lag check still fails the run. The report shows "bench rerun ×n", and the console prints a total. `HostStallRerunTests`.
- `scripts/lag_probe.py`: one scenario N times, worst lag and when. `replay_sample_log`: median 2.8 → 0.8 ms, max 6.7 → 1.6 ms.
- Two full HiL runs: **53/54 + 1 known, 0 FAIL** both, b2b 54/54, 0 invariants, 0 replays; one 20.4 ms stall rerun automatically.

## v2.9.6: SAF_Status alive counter + CRC (7 Oct 2026)

- **New layout (8 bytes):** CRC-8 (0x2F, over bytes 1–7 + data ID 0x201 LE) | 8-bit alive counter | state:3 · mrm:1 · cause:4 | challenge | accel int16 | steer int16. `static_assert`s guard the packing. The counter resets with the controller (a real reset restarts it). The CAN-process vECU's DBC SAF_Status is unchanged.
- **Receiver** (`LinkDUT._status_e2e_ok`): CRC, then counter step 1–2; a repeat or a jump is rejected (counted); resync after two frames in sequence, or at once if the last accepted status said OFF (an announced restart: back-to-back caught a one-cycle loss in `safety_brownout` without it). `status_e2e_rejects` per scenario, and a total on the console. `StatusE2ETests` (6).
- **Proof on the boards:** old receive firmware on A + the mirror filter off → the stale frames rejected by the counter alone (6 + 1), 3/3 PASS, 0 invariants.
- Full HiL: **53/54 + 1 known, 0 FAIL**, b2b **54/54**, 0 invariants, 0 replays, 0 E2E rejects. SafetyNode 2.5.

## v2.9.7: a real reset of board B mid-scenario (8 Oct 2026)

- **SC-41 `safety_hw_reset`** (SR-17, SG6): at 2000 ms the bench pulls B's EN through RTS (non-blocking; DTR held low). Expect STOP_IN_LANE, SAFETY_RESET, stopped, latched.
- **Core:** `Node::start` (factored out of 'R'), `Node::restore` (boot from a stored configuration: cold init, then the brownout path → latched STOP_IN_LANE / SAFETY_RESET), `config_blob()` / `config_seq()`; hello also while active (every 1 s).
- **SafetyNode 2.6:** stores the last configuration in NVS (Preferences) when it changes, and restores it at every boot; hello says "restored". `-DSSC_NO_RESTORE` builds the negative-test variant.
- **Bench:** `LinkDUT.can_hw_reset` / `hw_reset()`: OFF while booting, observer guard paused (`BOOT_TIMEOUT_MS` 3 s), alive-counter receiver reset, in-flight status within `MIN_BOOT_MS` (50) dropped. Other DUTs: a power loss of `dut_hw.reset_boot_ms` (186, measured). `scripts/reset_probe.py` measures the boot. `HwResetTests`.
- Measured: first status 185–187 ms after EN (5/5). Full HiL **54/55 + 1 known, 0 FAIL**, b2b **55/55**.

## v2.9.8: RAM queue on board A (8 Oct 2026)

- **Cause of A's receive overflows:** its own busy-wait watchdog pulses (100 µs per kick; bursts after a host stall) with one receive buffer (rollover off). Not the PC: no flow control on the CH340. `scripts/overflow_probe.py`: 40 overflows in 60 s with drifting kick bursts, 0 without.
- **BusNode 2.6:** a CAN task on core 0 (woken by INT, 1 ms fallback) fills a 256-frame RAM queue; `loop()` on core 1 drains it to USB and drives the watchdog line; all SPI in the CAN task. ERRIF/MERRF cleared whenever set (INT is level-active); at most 4 passes with INT low before the 1 ms wait (no idle starvation); a persisting RX1IF is cleared (RXB1 can only be stale); overflows in the boot window not counted. Diagnostics 'D' add the queue high-water mark and drops (bench: `queue_high`, `queue_drops`).
- Probe: 0 overflows in 3 × 60 s incl. bursts, queue high 2, 0 drops. Full HiL **54/55 + 1 known, 0 FAIL**, b2b **55/55**, A 0 overflows.
- Correction: the v2.9.7 write-up blamed A's 2 overflows on the PC; finding 12 corrects it.

## v2.10: one protected SAF_Status at every level (8 Oct 2026)

- **DBC:** `SAF_Status` (0x201) redefined to the C++ core's v2.9.6 layout: `SAF_Status_CRC` (byte 0, CRC-8 0x2F over bytes 1-7 + data ID 0x201 LE), `SAF_Status_Counter` (byte 1, 8-bit alive counter), `SAF_State`:3 · `SAF_MrmRequest`:1 · `SAF_Cause`:4, `SAF_WdChallenge`, `SAF_AccelOut`, `SAF_SteerOut`. Until now the CAN-process vECU sent the old unprotected layout, so one message ID had two layouts depending on the test level.
- **Shared code (`ssb/e2e.py`):** `status_protect`, `status_unpack` and `StatusReceiver` (step 1-2 accepted, repeats and jumps rejected, resync after two in sequence or at once after an announced OFF). The HiL bench (`LinkDUT`) and the CAN-process bench (`CanDUT`) use the same receiver.
- **vECU process:** builds SAF_Status through the DBC (`canio.encode_status`, as a supplied vECU would), adds the CRC, counter +1 per frame, 0 on a controller reset.
- **CanDUT:** every status frame passes the receiver before the observer uses it; rejects reported as `status_e2e_rejects`. `status_replay={"capture_ms", "inject_ms"}` puts a byte-identical stale copy on the bus from a second sender; `status_e2e=False` is the negative test.
- **Tests (58):** DBC encoder vs shared encoder byte for byte (300 random frames); the C++ core's real status frames (host DLL, link protocol) re-encode byte for byte through the DBC; a NORMAL status recorded at 1.5 s replayed 3× after the latched stop: check on → 3 rejected, 0 invariant violations; check off → "left a latched stop without release".
- **Full CAN-process run:** back-to-back **55/55**, 0 status E2E rejects, 0 invariant violations. Matrix **53/55 + 1 known** after reruns: the remaining 2 (`bus_flood`, `nominal_with_noise`) were thrown out by the real-time guard on all three attempts (host stalls of 0.1-4 s while a screen recorder and browser were running), with every other check passing. To repeat on a quiet host.

## v2.11: lockstep for the CAN-process bench (8 Oct 2026)

- **Why:** v2.10's real-time CAN run lost scenarios to laptop stalls of 0.1-4 s (71 reruns, 2 scenarios never clean) although the controller was never wrong. As with ROS 2 (v2.8), a lockstep mode makes stalls slow a run down instead of changing it.
- **Protocol:** the bench sends every input for ms t (commands, kicks, control) and then `VEH_Feedback(t)`, and at each 10 ms boundary waits until that cycle's SAF_Status has arrived. The v2.10 alive counter (+1 per cycle since the reset) says which cycle a status belongs to; only frames that pass the E2E check count, so a replayed stale status can't release the wait. Timeout 2 s → bench fault (rerun).
- **vECU (`BENCH_Lockstep`, new bit 5 of BENCH_Control):** messages handled strictly in order; cycle T runs as soon as `VEH_Feedback(T)` arrives; a kick or release received after `VEH_Feedback(T)` belongs to ms T+1; a reset is acknowledged at once (counter 0) so cycle 0 sees the inputs for t = 0. First full run without that acknowledgement: `startup_normal` ended at 30.94 vs 30.96 km/h. Real-time behaviour unchanged.
- **CLI:** `run.py --dut can --can-lockstep [--b2b-dut]` (back-to-back exact on the 10 ms grid, plant end state included).
- **Result (screen recorder and browser still running):** matrix **54/55 + 1 known, 0 FAIL, 0 bench reruns**; back-to-back **55/55 exact**; 0 invariant violations, 0 E2E rejects; 8 min for matrix + b2b (real time: ~30 min). Real time stays the default for transport timing.
- **Tests (60):** three 0.5 s host stalls around the fault and the reaction → same timeline and plant end state as the reference, no bench fault; the stale-status replay is still rejected in lockstep.

## v2.12: power cut of the safety controller, prepared (8 Oct 2026)

- **Why:** a reset (SC-41) keeps the board powered; a supply loss also takes the CAN transceiver and the USB-serial chip, and may boot differently. Everything but the relay is built, so the hardware day is wiring + running. Plan, wiring, procedure and expected results (written before measuring): `docs/HIL_POWER_CUT.md`.
- **SC-42 `safety_power_cut`** (SR-17): B loses its supply for 300 ms at 2 s; expect STOP_IN_LANE, SAFETY_RESET, stopped, latched. On a board with a relay (`--relay` / `dut_hw.relay_fitted`, or the emulated boards) the relay opens; every other DUT gets a power loss of the cut plus `dut_hw.power_on_boot_ms` (186, **assumed** equal to the reset boot until measured). Reference: OFF 2000 → STOP_IN_LANE 2486 ms, ECU fallback braking, stopped; `no_latch` FAILs. Exact back-to-back for the C++ core, the FMU and CAN lockstep.
- **BusNode 2.7:** `'X' <ms>` holds GPIO6 high (relay on) for that long, non-blocking, max 60 s; low at boot, so with the relay's normally-closed contact in B's 5 V supply an A that resets or hangs leaves B powered. Compiled, not flashed.
- **LinkDUT:** `power_cut()`; while B is down its USB port is not used (B is observed through A on the bus), OFF until its first status; before the next scenario (or a recovery) the bench waits up to 15 s for B's port to reappear and reopens it.
- **Emulator + host build:** `ssc_node_config` / `ssc_node_restore` exported; emulated A handles `'X'`: B goes silent, its link drops, and it boots from its stored configuration after the cut plus the boot. Found while testing: a dict-iteration bug (A's command closed B's link mid-loop) crashed the emulator on the second cut.
- **`scripts/power_cut_probe.py`:** cuts B N times, measures the boot after power returns and checks that B's COM port really vanished (if not, USB is still powering B).
- **Fix:** the v2.11 JUnit change used a nested same-quote f-string, valid only from Python 3.12 (the project supports 3.10+); found by ruff. All files now parse with the 3.10 grammar. (Already public in v2.11's JUnit fix; to be republished.)
- **Tests (64):** emulated power cut latched; no-latch fails; host build boots latched from a stored configuration; the relay path on the emulated boards incl. reopening B's port. Reference matrix 55/56 + 1 known.

## v2.13: foundations for merging related projects (8 Oct 2026)

- **Why:** the bench is becoming the base for a requirement → test → hardware → release chain (test design, bench health, CAN security, OTA). Before any of that touches the bus or the reports, the bench needs one ID list, one record per run and full requirement tracing.
- **`config/can_ids.json`:** the one list of CAN IDs on the bench bus: the six DBC messages (active), UDS IDs reserved for the planned version check (0x7DF, 0x7E2/0x7EA, 0x7E3/0x7EB; 0x7E0 avoided because a related intrusion-detection demo spoofs it), and IDs used by related projects on their own buses. Found while writing it: a related ACC demo uses 0x100 and 0x200, the same IDs as `PLN_Command` and `SAF_ActuatorCmd`, so it must stay on its own bus. Tests: the DBC equals the active list, the C++ core and the Python bench use only registered IDs, reserved IDs are not in the DBC, clashes are marked off this bus.
- **Run manifest:** every `run.py` run writes `reports/<label>_manifest.json`: run ID, bench version and identity, git commit (+ dirty flag), level, config, plant, defects, command line, scenario count + hash of the keys, pass / known / fail, bench-fault reruns and host stalls, board firmware versions (PiL / HiL), host, start / end, and the SHA-256 of every file the run wrote (results, JUnit, HTML, CSV traces, MDF4). Later approval gates bind to this file's hash.
- **SR-19 (SAF_Status E2E):** the CRC + alive counter built in v2.9.6 had no requirement. Added to `safety/hazards.json` with `verified_by` (the status E2E unit tests, the cross-level layout tests, the replay test under CAN lockstep) and the v2.9.6 hardware evidence. The report lists it as "unit tests only", apart from both the scenario-covered requirements and the real gaps, and the safety case no longer counts it as scenario evidence.
- **Phase folders:** `design/`, `health/`, `security/`, `fleet/`, `vehicle/` with a README each saying what goes there; no code yet.
- **Fix:** the C-FMU test rebuilt the FMU whenever any `config/*.json` changed; it now watches only the two configs bundled into the FMU.
- **Tests (73):** 6 registry, 2 traceability, 1 manifest. Reference matrix unchanged: 55/56 + 1 known.

## v2.14: test design front end, first slice (8 Oct 2026)

- **Why:** the matrix was written by hand. Now a requirement can go to a defect ticket through code-derived cases and three human approvals, with no hand edits in between (integration plan P1). Ported from the HIL ML Ops pipeline: its gates module as is, its fault-injection and boundary-value rules re-applied to this bench's timed faults. Details: `design/README.md`.
- **`python -m design`:** test basis (drafted by rules from "<trigger> for > N ms → <reaction>", validated by code) → `basis_review` → ISTQB cases → `case_review` → `run.py` at any level → triage → `publish_approval` → local tracker or Jira Cloud (stdlib REST client). One defect per requirement + failure signature; a recurrence becomes a comment.
- **SR-01:** 5 executable cases (permanent loss at 10 / 30 / 39 km/h, mid-period injection, 101 ms loss then recovery) and 2 blocked probes (100 ms, 99 ms: no requirement covers them). Reference: 5/5 PASS; the probes show a 99 ms loss already stops and latches (finding 2, now reached from the requirement side as an open question with evidence). Seeded `long_timeout`: 5/5 FAIL, one defect; second run: a comment, no duplicate. FMU (clean): 5/5 PASS; native C++ core with the seeded bug: 5/5 FAIL.
- **Bench:** `run.py --scenario-file` (run a generated file instead of the matrix, also for back-to-back) and `--label`; oracle check `min_detect_ms` ("no reaction before threshold − one frame period"; no reaction at all stays the reaction check's failure).
- **Fix:** mypy clean again (15 errors since v2.8–v2.12 in `dut.py`, `hil_emu.py`, `vecu_process.py`: annotations only, no logic change).
- **Tests (87):** basis parsing and validation (irregular wording not guessed, threshold vs config, FTTI vs threshold + period), case set and timing, probes blocked, oracle check both ways, gate re-opens on changed content, the full human flow through the CLI, seeded bug caught then de-duplicated. Reference matrix unchanged: 55/56 + 1 known.

## v2.15: generated cases on hardware, and an LLM drafter (8 Oct 2026)

- **HiL:** the derived SR-01 cases on both boards (`--kick gpio`): 7/7 PASS, back-to-back 7/7, detection 98–110 ms, worst cycle 82 µs; seeded `long_timeout` through B's reset command: the same 5 cases FAIL as on the PC levels. Detection at 98 ms (before the 100 ms threshold) confirms the derived lower bound of threshold − one frame period. Before the run, board B reported "CAN FAILED" on three boots; SpiProbe found the SPI and wiring fine, a reflash cleared it, cause unknown (`docs/HIL_FIRST_RUN.md`). The gated HiL campaign waits at `basis_review`.
- **LLM drafter** (`design/drafter.py`, `design/llm.py`): for requirements the rules can't parse. The model maps the requirement's alternatives (found by code) onto a fault catalogue built from the bench's own scenarios; code rejects uncovered or merged alternatives, unknown faults / parameters / causes, one fault for two alternatives, a tolerated bench scenario as a stop trigger, and dropped or invented numbers; up to 2 repair rounds; `basis_review` rejections feed the next draft. Plans `free` (Groq, then Ollama), `local`, `paid` (gpt-4o-mini), `off`; every answer cached.
- **Code owns timing and patterns:** a trigger equal to a bench scenario inherits its timing (SC-37b jitter: no FTTI from a fixed moment; without it, a validated SR-02 draft raised a false defect on the healthy controller), and derivation adds the other patterns the bench expects to stop (SC-05 intermittent CRC; without it, `e2e_run_length` escaped every SR-02 case).
- **Defect drafts:** failing cases grouped by failure mode, groups sharing an injected fault merged (one bug = one ticket: `e2e_lenient` gave 4 tickets before, 1 now); labels `sig-<fault>` + `mode-<check>`; a recurrence needs the same input AND the same modes (before, `no_latch` was posted as a comment on the jitter ticket).
- **Measured:** with gpt-4o-mini (OpenAI, 13 calls, about 35k tokens in, under one US cent; the Groq key in the HIL ML Ops `.env` is empty): SR-02, SR-03, SR-04 drafts pass the checks after the hardening; SR-04's maps "no kick" to frame latency (a meaning error only a reviewer can catch; its campaign waits at `basis_review` for that). Against all 14 seeded bugs, the SR-01 cases catch `long_timeout` + `no_latch`, the SR-02 cases `e2e_lenient` + `e2e_run_length` + `no_latch`: every bug in each requirement's own mechanism, 0 failures on the healthy controller.
- **Tests (102):** drafter checks one by one (with the real drafts as fixtures), repair round, invalid after repairs, reviewer feedback in the prompt, timing inheritance + pattern variant, cache without a key, SR-02 through the whole chain with a fake model incl. one-ticket-per-bug and de-duplication.

## v2.16: Claude Sonnet 5.5 is the only model; the pipeline can run without a human at the gates (9 Oct 2026)

- **One model:** the `design/` drafter now calls Claude Sonnet 5.5 (`claude-sonnet-5-5`) at medium effort through the official Anthropic SDK. The Groq, Ollama and OpenAI plans are removed, so a failure (no key, 401, rate limit, refusal, truncation) stops the run loudly and no other model answers. No `temperature` is sent (400 on this model); the drafter accepts JSON inside a code fence. The same change was made in the HIL ML Ops pipeline (its own repo): see its `config/llm_providers.yaml`.
- **Not run yet:** there is no `ANTHROPIC_API_KEY` on this machine (the HIL ML Ops `.env` now has an empty placeholder), so no real Sonnet call has been made. The request shape is tested against a fake SDK client, not against the API. The two campaigns waiting on gpt-4o-mini drafts (SR-02, SR-04) were deleted; the old OpenAI cache too.
- **`--autonomous`:** no waiting at the gates; the agent approves each one on Ketan's behalf for this test, recorded as `agent-auto:claude on Ketan's behalf (test 2026-10-09)` with the comment "no human review", and never reused by a run with human gates on. SR-01 ran this way on the reference controller (5/5 PASS, nothing to the tracker) and on both HiL boards: clean 5/5 PASS; with the seeded `long_timeout` bug, caught and filed as one defect (local tracker; Jira Cloud was not used).
- **Tests (107):** request shape (model, effort, no temperature, system split, ends on a user turn), response cache, no key = no fallback, refusal and truncation fail loudly, fenced JSON accepted, agent approval labelled and not reused by a human-gated run.

## v2.17: Jira dropped, workspace header for the Anthropic key (9 Oct 2026)

- **Jira Cloud removed** from `design/` (cost): `design/jira.py` is replaced by `design/tracker.py` (the free local JSON tracker only); `--jira` is gone. The Jira client is in git history (v2.15/v2.16). Suggested free hosted option: GitHub Issues (`docs/INTEGRATION_PLAN.md` section 9).
- **Anthropic key scope:** a personal key (`sk-ant-usr-...`) is not tied to a workspace, and the API answers 400 until the request carries `anthropic-workspace-id`. `design/llm.py` now sends it when `ANTHROPIC_WORKSPACE_ID` is set (environment or `--env-file`); a workspace-scoped API key needs nothing. No Sonnet campaign has run yet for this reason.

## v2.18: GitHub Issues tracker, and "test the test" (9 Oct 2026)

- **GitHub Issues tracker** (`design/tracker.py`, `--tracker github --github-repo owner/name`): through the `gh` CLI login (no token in the code); same three methods as the local tracker, so the duplicate rule is unchanged (labels `ssb`, `defect`, requirement, `sig-<fault>`, `mode-<check>`; keys `GH-n`; label limits handled; pull requests in the issues list ignored; a malformed repo stops the run before any bench time). Live against the private repo `ketan01bakshi-dev/hil-mlops-defects`: the seeded `long_timeout` bug became issue #1; a second run added a comment instead of a second issue.
- **First Sonnet 5.5 runs** (key + workspace ID in place): SR-02 and SR-04 drafted in one call each (about 6 s, 4.2k in / 0.85k out). SR-04: correct fault mapping. SR-02: Sonnet mapped "wrong sequence" to one `counter_jump` of 4, which this bench's window tolerates, so the healthy controller failed that case and, with nobody at the gate, a **false defect** was filed (SSB-2, now closed as invalid).
- **Test the test:** after the bench run the same cases run on the clean reference controller; a case that fails there is `SUSPECT`, listed under "Suspect cases (not filed as defects)" with the drafter's own assumptions, and kept out of the defect drafts. Re-run of SR-02 with the same Sonnet draft: nothing filed. A real seeded bug (`e2e_lenient`) is still filed, without the suspect case in it.
- **Tests (113):** the GitHub tracker against a fake API, a campaign filing to it and commenting on the rerun, the bad-repo guard, the suspect-case logic on the exact failing draft.

## v2.29: board A's receive overflows, investigated (9 Oct 2026)

**Question:** board A's overflow counter read 28 to 29 after a short HiL run, against "0" in the v2.9.8 write-up. **Answer, in short: the rate is a steady ~5 per million frames; it was not caused by the Wi-Fi code; the old "0" was a sample too small to see it; the receive path had thin margin and now has more; about half the remaining events are still unexplained.**

| Measured | Result |
|---|---|
| Fresh board A, 14 scenarios incl. floods, fuzzing, replays (~50 000 frames) | 0 overflows |
| Idle, 10 resets of B, an OTA update of B, A's port opened and closed 9 times | 0 caused by any of these |
| Previous firmware on B (control run) vs the Wi-Fi firmware | the same readings, so not the radio |
| 5 samples of 324 000 to 601 000 frames of `nominal_with_noise` | 15 overflows in 2.3 million frames, **6.6 per million**, never more than 1 to 2 per 12 000-frame run |
| The v2.9.8 claim ("0 in 3 x 60 s") | about 36 000 to 260 000 frames: at this rate the expectation is 0.2 to 1.7, so "0" was luck, not proof |

**Why the counter looked bad:** it is cumulative since A booted (hours of traffic, 4 million frames), and the bench printed it as if it belonged to the run. The bench now also reports this run's own `rx_run` / `rx_overflow_run`.

**What the instrumentation (BusNode 2.9 to 2.12) found:**
1. B sends frames back to back, ~222 us each at 500 kbit/s, and the MCP2515 has one receive buffer (rollover is off since v2.9.4), so the buffer must be free within ~220 us of the interrupt.
2. Wake latency is mostly under 50 us, but 4 percent of wakes are 100 to 230 us (the task still busy with the previous frame) and about 1 per 500 000 takes **370 to 520 us**. Those coincide with some overflows.
3. **Two real margin fixes, kept:** (a) the CAN interrupt was installed from `setup()`, which runs on core 1, so every frame's interrupt was serviced on the busy core and had to wake a task on core 0; it is now installed from the CAN task itself. (b) The receive buffer was read in several separate SPI transactions; it is now one `READ RX BUFFER` (0x90) which also clears the flag. **Interrupt to buffer-free: never above 93 us over 601 000 frames** (587 000 of them between 50 and 100 us); the drain alone used to reach 190 to 314 us.
4. **The overflow rate did not measurably change** (3 in 601 000 frames afterwards, vs 6.6 per million before), and in the final run **two of the three had nothing wrong in their 2 ms window**: no slow wake, no long hold of the buffer, no misread, no SPI glitch. So the dominant remaining cause is **not task timing**, and it is **not identified**. Candidates not yet tested: a retransmitted frame from B (an error on the last EOF bit makes the sender repeat a frame the receivers already accepted), SPI signal quality on the jumper wires, and the second frame of a pair arriving while RX0IF is being cleared.

**Effect on the bench:** A is the observer and relay; an overflow drops one observed frame. Every scenario still passes (no run has failed because of it) and the E2E layer tolerates a lone lost frame. It is a measurement-quality item to watch, not a safety finding on B.

Changes: BusNode 2.12 (instrumented, ISR on core 0, one-transaction read), `scripts/overflow_by_scenario.py`, `scripts/overflow_watch.py`, per-run counters in `ssb/hil.py`, inventory BusNode 2.12.

## v2.28: the board's own Wi-Fi radio (9 Oct 2026)

- **Firmware (`ssc_net.h`, SafetyNode):** Wi-Fi station + the ESP-IDF MQTT client (shipped with the Arduino core, no library added). The network name, passwords, broker address and board id are **provisioned over USB into the board's flash**, never compiled in, so the signed images carry no secret. Nothing provisioned: the radio never starts. The radio is **off while a scenario runs** and back 2 s after; credentials are accepted over USB only; INFO never returns a password. The OTA module answers over MQTT for messages that arrived over MQTT (`ota::sink`).
- **PC side:** `fleet/provision.py` + `scripts/provision_wifi.py` (reads secrets from the environment or a hidden prompt, never prints, stores or logs them; refuses what the board would refuse before sending), `fleet/broker.py` (`login` stores an argon2 hash only, `serve` listens on the LAN with that login, `addresses`), `LocalBroker` with login, `MqttLink`/`Gateway` with login, a clear error for a refused login (it used to be silence), hello-on-connect and a reset hook.
- **`python -m fleet.ota_matrix --real COM13 --wifi`:** REL-01..10 with the update traffic on Wi-Fi; the EN-pin reset stays on the cable. REL-07 is timed from the reboot command (the trial hello arrives after the 3 s probation over Wi-Fi). `--emulated --wifi` runs the path against a stand-in board with a login-protected broker: 10/10, and it runs in CI.
- **On the real board, without a network (fake test settings):** provisioning and INFO round trip; the radio retrying does not disturb the board; a USB update still **commits** with the radio active (the health check needs free heap); 3 of 3 HiL scenarios pass with the network provisioned (detection 99 / 102 ms as before). **Not run: the real Wi-Fi network**, which needs your credentials (`docs/WIFI_SETUP.md`).
- **Re-run on the real board with the network firmware:** REL-01..10 **10/10** over USB and **10/10** through the PC gateway, both rollouts as before, REL-11/12 **2/2** (about 5150 chunks per push). Board B was left on the 2.10 baseline with nothing provisioned.
- **A question asked and answered:** board A's receive-overflow counter reads 28 to 29 on a short HiL run (it is cumulative since A last booted: `rx` keeps climbing across runs). A control run with the previous firmware on B gave the same 28 / 29, so the radio code did not cause it; whether it was already there since v2.23 was not recorded earlier. Worth a look on its own: the v2.9.8 write-up says 0.
- **Cost, stated:** the firmware grew from about 350 KB to about 1 MB (78 % of the 1.25 MB slot): a push is about 5150 chunks, and there is little room left. Plain MQTT, no TLS, one shared login: the release decision's standing limit says so once a Wi-Fi result exists.
- **Secrets hygiene:** `secrets/`, `*.passwd`, `broker_passwd`, `wifi_secrets*.h`, `.env.wifi` are git-ignored; tests fail if a tracked file assigns a literal password to `SSB_*_PASS`, if firmware contains `WiFi.begin("...")`, or if the provisioning script prints or stores what it is given.
- Docs: **`docs/WIFI_SETUP.md`** (step by step), `docs/FLEET_OTA.md` §9. **Tests (236 after v2.29):** provisioning against the emulated board, the firmware contract (buffer sizes, ops, USB-only credentials, radio off during a run), broker login, the --wifi path behind a login, secrets hygiene.

## v2.27: P4 gaps: OTA through an MQTT broker, and a power loss inside an update (9 Oct 2026)

- **A defect in the v2.26 update path, found by cutting power inside it.** `END` made the boot-slot switch first and the probation record second, and wrote the state flag before the slot it points at. A cut between the switch and the record boots the new image with **no probation** (an unhealthy image is never rolled back); a cut between the state flag and `prev` leaves TRIAL with no slot, so the rollback **switches nothing**. Fixed in `ssc_ota.h`: record first with the state flag last, switch after, and `boot_check()` discards a record whose slot never switched. Hardware flashed with the fix; every image rebuilt from the fixed source.
- **REL-11 / REL-12** (`--cut-windows`; hazard H9): power lost at each of the five persistent writes that end an update, with an **unhealthy** image being installed. The unhealthy image never runs afterwards, the board reports its safe state restored, and the next update goes through. **Pass on the emulated board and on the real board** (cut builds `-DOTA_CUT_AT=n` reset the CPU once at the nth write). The emulation of the first order fails REL-11 (`tests`), so the test is known to catch the defect.
- **OTA through a broker** (`fleet/mqtt_link.py`): the unchanged `OtaClient`, an amqtt broker, and a gateway in front of the board. **REL-01..10: 10/10 emulated and 10/10 on the real board** through it. The board still has no Wi-Fi radio of its own (that needs network credentials, which stay out of the repo).
- The release decision reads the MQTT and power-cut results as optional evidence (a failing row is a NO GO) and states its two remaining limits plainly: no radio on the board, and no supply ever cut.
- Fixes: `OtaClient.push` returns a `lost` outcome when END is never answered (a power loss), instead of raising; `telemetry.py` typing under paho 2; the emulator models `prev` as unset after a commit or rollback, as the firmware does.
- Packaging: `pip install -e .[fleet]` (paho-mqtt, amqtt); the MQTT tests skip without it; CI's full job runs the emulated matrix, the MQTT matrix and the cut windows.
- Board B ends on the OTA-delivered release image **SafetyNode 2.50**; inventory updated. Docs: `docs/FLEET_OTA.md` §7-8. **Tests (208):** MQTT transport against a real broker, the cut windows on the fixed and on the first order, the control channel, release evidence.

## v2.26: P4 release layer: OTA of the safety controller, staged rollout, one release verdict (9 Oct 2026)

- **OTA in the firmware** (`hil/firmware/SafetyNode/ssc_ota.h`): the spare app slot is written while the running image is untouched; the boot slot switches only after SHA-256 **and** HMAC-SHA256 over hash + size + **version** check out (so an old image cannot be re-labelled); trial boot commits itself after 3 s of healthy operation, a failed health check or any reset before the commit rolls back, and the old image comes up in its latched safe state (SR-17). BEGIN is refused while a scenario runs (only an idle ECU updates) and on a downgrade.
- **The PC side** (`fleet/image.py`, `fleet/ota_client.py`): build and sign images; stop-and-wait BEGIN / CHUNK / END / REBOOT / STATUS over the bench's own USB link, with injectable faults (corrupt byte, dropped chunk, wrong key, reset mid-download). The ack carries the board's own offset, so a lost or repeated chunk only costs a resume.
- **One emulation, two targets** (`fleet/emulated.py`): the same protocol and state machine in Python behind a serial-like interface, so the *same* client updates an emulated vECU or the real board. The tests hold the emulation to the firmware; where they differ, the board wins.
- **REL-01..10** (`python -m fleet.ota_matrix`, requirements in `safety/hazards.json` as hazard H9 / goal SG9): tampered image, wrong key, corruption in transit, reset at 1 / 50 / 99 % of the download, lost and repeated chunks, failed health check, reset in the trial boot, reset between staged and reboot, anti-rollback, and the safe state restored after every boot. **10/10 emulated and 10/10 on the real board B** (1798 chunks, 3 resumes); board B now runs the OTA-delivered SafetyNode 2.16.
- **Staged rollout** (`fleet/rollout.py`, `fleet/rollouts.py`): canary, then a quarter, then the rest, with one halt rule on the devices touched so far; another hardware revision or an equal/newer version is skipped, not failed. Good build 2.16 over 40 emulated + 2 rev-a + the real board: 41 committed, 2 skipped, ran to the end. A build that fails its health check over 200 emulated boards: all 4 canaries rolled back on their own and the rollout **halted after wave 1**, 196 boards never touched.
- **Telemetry** (`fleet/telemetry.py`): boot (version, safe state restored), staged / commit / rollback, resets and SAF_Status changes on `ssb/fleet/<device>/<event>`; best effort, never blocks a test, in memory without a broker.
- **One release verdict** (`python -m fleet.release`): BLOCKED / NO GO / GO WITH RISKS / GO from preflight + both attack matrices + both OTA matrices + both rollouts + the design campaigns, with the rules stated, the standing limits of the evidence listed, and every input file hashed into the report. Today: **GO WITH RISKS** on the three carried-over security findings. It reads report files only; it cannot change a PASS or a FAIL and never calls a model.
- **Fixed:** the risk list repeated every open finding, once per attack matrix (reference and the two boards). It is one risk, so it is listed once.
- `python -m fleet.ota_matrix --build-images` compiles the four images the real-board sequence needs; they are build artifacts (`fleet/images/`, now git-ignored) and `--real` refuses with that hint when one is missing. The version the board ends on reads back over UDS (preflight: board_b v2.16), so the release report's firmware line is diagnostic evidence, not the image's own claim.
- Docs: `docs/FLEET_OTA.md`, `fleet/README.md`. **Tests (196):** signed images and the signature's version binding, the OTA client against the emulated board under each injected fault, the REL sequence, wave planning and the halt rule, telemetry events, the release rules (clean evidence, open finding, failing OTA scenario, un-halted bad build, product failure vs seeded defect, the dedupe), and REL traceability against `hazards.json`.

## v2.25: the attack matrix on the two boards (9 Oct 2026)

- Controller fix flashed to board B: **SafetyNode 2.8** (rollback image of 2.7 saved); inventory updated; UDS reads v2.8 / v2.8.
- `python -m security.matrix --dut hil -- --port-b COM13 --port-a COM14 --kick gpio`: 12/12 as on the reference, stop times within 1-10 ms of it. SC-68 (forged valid command beside the real one) now ends in STOP_IN_LANE on the real chip too. The open findings reproduce: `sec_fuzzy` stops at 411 ms, `sec_signal_ramp` is seen by nobody.
- The matrix marks the E2E-rejected column "n/a (black box)" for levels that do not report it, instead of a misleading 0.

## v2.24: P3 security layer: attacks, intrusion detector, attack x detection matrix (9 Oct 2026)

- `security/attacks.py`: seven attack kinds on the planner bus (flood, fuzzy, spoof_invalid, spoof_valid inject / takeover, replay, period_glitch, signal_ramp); the runner got three small hooks (attacker node, IDS tap, `security` block in the results), all inert without an `attack` fault or `ids` flag.
- `security/ids.py` + `train.py`: learned rules + a numpy MLP, trained on this bench's own clean traffic; judged on attacks with held-out parameters (TRAINING.md). 0 false alarms on 22 held-out clean runs. The MLP adds nothing the rules do not already catch.
- `scenarios/security.json` (SC-60..71) and requirements SEC-01..05 (hazard H8 / goal SG8 in `safety/hazards.json`); `python -m security.matrix` writes the attack x detection matrix. Reference: 12/12 meet their expectation, with two tracked findings.
- **Defect found and fixed:** a second valid command in the same cycle skipped the steering rate limit (SC-68: output steering moved 25 deg in 20 ms). Fixed in the Python reference and the C++ core; the 150 existing tests (incl. native back-to-back) still pass.
- **Open findings:** `sec_fuzzy` (stop at ~410 ms vs the 250 ms FTTI) and `sec_signal_ramp` (a slow in-envelope drift: no layer reacts, 4.8 m off path in 12 s); the exact-copy mimic is invisible by construction.
- `ssb/oracle.py`: `expect.reaction` may be a list of acceptable reactions. 16 new tests (`tests/test_security.py`).

## v2.23: P2 complete: preflight, UDS version read, dashboard (9 Oct 2026)

- **UDS on both boards** (`hil/SafeStopCore/src/ssc_uds.h`, SafetyNode 2.7, BusNode 2.8): TesterPresent, ReadDataByIdentifier F195 (version) and F18C (serial) over ISO-TP single frames; B answers on the real bus, A answers for itself. BusNode gets a `'T'` message to put a frame on the bus. Flashed to both boards (rollback images of the 2.6 firmware saved first). Live: B v2.7 serial 91F61B44, A v2.8 serial 95F61B44; a full HiL SR-01 run on the new firmware: 7 cases, no defects.
- **Preflight** (`python -m health.preflight [--full]`): ports, USB hello vs inventory (with the one-reset recovery of a silent stale node), UDS read, and in full mode source-vs-board version, compile of both sketches, partition audit (app size, layout vs the chip's real flash) and an esptool upload probe. Every failure prints fixes from `health/error_catalogue.json` (10 entries seeded from the bring-up findings).
- **Gate:** `run.py --dut hil` runs the quick preflight first and does not start when it fails (`--skip-preflight` is recorded in the manifest). Shown: a wrong port refuses with the fixes; a normal run starts and the manifest carries the preflight result.
- **Dashboard** (`python -m health.dashboard`): bench fitness, coverage per requirement and level, every non-PASS in one box (PRODUCT / TEST / BENCH / GAP, seeded defects labelled), failure clusters.
- **Audit finding:** both chips have 16 MB of flash but the default partition layout uses 4 MB (app 1.25 MB, 26% used): room for a larger layout or OTA slots later (P4).
- Inventory: SafetyNode 2.7, BusNode 2.8. 24 new tests (`tests/test_health.py`): the responder through the native DLL, the tester against a simulated bus, preflight with injected faults, the dashboard.

## v2.22: bench health checked on the real boards (9 Oct 2026)

- Reading the real hellos corrected the inventory I had written from memory: board B (COM13) is **SafetyNode 2.6**, board A (COM14) is **BusNode 2.6** (not 2.5), and the ports are the other way round from my first guess.
- Live HiL run SR-01 on the two ESP32-S3 boards (`--port-b COM13 --port-a COM14 --kick gpio`, autonomous gates on the plan): 7 cases, no defects, no bench suspicion; the manifest records both hellos.
- The stale-firmware case was shown on that real manifest against an inventory that expects SafetyNode 2.5: "board_b: SafetyNode 2.6 is not the inventory's SafetyNode 2.5". A board was not reflashed to old firmware on purpose.

## v2.21: suspect bench instead of false FAIL (P2, first slice) (9 Oct 2026)

- `health/inventory.json` and `health/check.py`: firmware per board vs the run manifest, and scenarios that needed bench-fault reruns.
- `design` triage: a FAIL on a HiL run with a wrong or silent board, or on a scenario that needed reruns, becomes SUSPECT BENCH and gets its own report section; it is never drafted as a defect. Reference / PiL levels have no boards and are unaffected.
- 4 tests (stale SafetyNode 2.5 named, healthy passes, silent board, per-scenario reruns). Live proof (flash the old SafetyNode on purpose) waits for the boards.

## v2.20: plan route finds the CLI on short paths; redraft evidence deduplicated (9 Oct 2026)

- `design/llm.py`: the Claude Code binary is also looked for in `D:/Agents/tools/claude-code/claude.exe` (a copy of the desktop app's, short path, survives app updates) before the app's own versioned folder.
- `design/drafter.py`: `fault_evidence` lists a scenario once even if it injects the same fault several times, so the facts sent back to the model in an auto-redraft are not repeated.
- Live proof (SR-02, plan route, `--autonomous`): the first draft (single counter jump) was rejected with the bench's own evidence, Sonnet redrafted as 30 ms out-of-order delivery, 7/7 pass, nothing filed.
- Test fix: binary discovery test no longer sees the shared copy.

## v2.19: your Claude plan instead of the API wallet, and the agent as reviewer (9 Oct 2026)

- **Subscription route** (`design/llm.py`, `--llm subscription`, default): Sonnet 5.5 at medium effort through the Claude Code CLI signed in to the Claude plan; the API route (`--llm api`) stays selectable and is billed to the API account. No fallback either way. The first autonomous HIL ML Ops campaign (about $0.74 at list prices) had been billed to the API wallet, which the subscription does not cover. Safeguards: CLI started without any `ANTHROPIC_*` or enclosing-session variable, neutral working directory, no tools, no saved session, never `--bare`.
- **Auto-redraft** (`--redraft N`, default 1 with `--autonomous`): suspect cases of an LLM-drafted basis are rejected with facts only (`design/campaign.py: redraft_reason`, evidence from `scenarios.json` via `drafter.fault_evidence`), the model drafts again, bounded, logged in `01_basis/redraft_log.jsonl` and in the gate record. HIL ML Ops has the same loop (`run_campaign(redraft=N)`, `stages/execute.py: suspect_reason`).
- **Tests (122):** subscription request shape and clean environment, repair rounds as a transcript, failure messages, shared cache, binary discovery; the redraft loop (a wrong fault corrected in one round, bounded at N, off by default, rules-drafted never redrafted).
