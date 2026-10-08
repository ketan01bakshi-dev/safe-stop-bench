# The FMU adapter (v2.2): testing a supplied vECU delivered as an FMU

## 1. Why an FMU is the most likely delivery

When a team hands over a "virtual ECU" without handing over its source, the most common package is an **FMU (Functional Mock-up Unit)**, defined by the **FMI (Functional Mock-up Interface)** standard (Modelica Association; FMI 2.0 from 2014 is still the most widely used, FMI 3.0 from 2022 is growing).

- Simulink/Embedded Coder, TargetLink, Synopsys Silver, dSPACE VEOS, Vector vVIRTUALtarget, Modelica tools and many in-house build systems all export FMUs.
- It hides the source code (IP protection) but gives a **standard C API** that any test tool can drive, which is why OEMs and suppliers exchange controllers this way.
- The same FMU can run on a PC (SiL), in CI, and on many HiL real-time systems (dSPACE, ETAS, NI, Speedgoat all import FMUs). That is the step from SiL to HiL without rewriting the model.

### What is inside an FMU

An `.fmu` is a **zip** file:

| Part | What it is | What the bench does with it |
|---|---|---|
| `modelDescription.xml` | Every variable: name, value reference (an integer handle), causality (input / output / parameter / local), type (Real / Integer / Boolean / String / Enumeration), variability, unit, start value; plus the FMI version, the GUID and the capabilities | Validated (FMPy schema + rules), listed, and matched to the bench's names |
| `binaries/<platform>/` | A compiled shared library per platform (`win64/*.dll`, `linux64/*.so`) that implements the FMI C functions | Loaded in-process by FMPy |
| `resources/` | Whatever the model needs at run time (tables, configs, calibration data) | Nothing; the FMU reads it itself |
| `sources/` (optional) | C sources, if the supplier allows | Could be compiled for another platform |

### Co-simulation vs model exchange

- **Co-simulation (CS):** the FMU contains its own solver/scheduler. The bench calls `doStep(t, h)` and the FMU advances by `h`. This is what a controller (a vECU) normally is: discrete tasks, no continuous states. **The adapter drives CS.**
- **Model exchange (ME):** the FMU only gives derivatives; the importer must integrate. Typical for plant models, not for ECUs. The intake tool flags an ME-only FMU as blocked (it needs a solver wrapper).

### The FMI 2.0 call sequence the bench uses

```text
fmi2Instantiate → fmi2SetupExperiment(t0 = 0) → set parameters (WarmStart, Config, Defects)
→ fmi2EnterInitializationMode → fmi2ExitInitializationMode
→ every 1 ms:  fmi2SetInteger / SetReal / SetBoolean (inputs) → fmi2DoStep(t, 0.001) → fmi2GetInteger / GetReal (outputs)
→ next scenario:  fmi2Terminate → fmi2Reset → (set parameters, initialise again)
→ end of campaign:  fmi2Terminate → fmi2FreeInstance
```

## 2. The design decisions, and why

### 2.1 Signal level, not frame level
FMI 2.0 has **no binary type** and **no events in co-simulation**. Supplier vECU FMUs normally expose **decoded signals** (the COM / RTE level), not CAN frames. So the bench's FMU contract (`ssb/fmu_contract.py`) is a list of signals named after the DBC.

**The E2E problem:** if the FMU only got *physical* signals (m/s², degrees), the CRC and counter would be meaningless and E2E could not be tested. The solution here: pass the DBC signals as **raw integers** (no scaling). The 14-byte planner frame is then rebuilt **bit for bit** inside the FMU, so the FMU's own E2E check runs on exactly the bytes the planner sent, including corrupted CRCs, frozen counters and short frames (`PLN_Command_DLC`).

### 2.2 Frames as events: Rx/Tx counters
A co-simulation step has no "a frame arrived" event. The standard workaround is a **counter per message**:
- `PLN_Command_RxCounter` increments for every received planner frame; the FMU sees a new frame when it changes.
- `PLN_WdKickCounter` increments per watchdog kick (mod 256), so several kicks in one step are not lost.
- `SAF_ActuatorCmd_TxCounter` increments for every frame the FMU sends; the bench rebuilds the frame (cantools, raw) when it changes.

Two planner frames can reach the controller in the **same millisecond** (measured: scenarios `reorder_once` and `jitter_above_period`). The adapter keeps a **one-deep receive queue** and delivers the second frame 1 ms later. The controller runs a 10 ms task, so this changes nothing unless a task boundary falls exactly between the two; the exact back-to-back run below shows it didn't.

*(FMI 3.0 adds Binary variables and clocks, and the FMI-LS-BUS layered standard defines frame-level CAN between FMUs. That is the clean long-term route. FMI 2.0 FMUs are what gets delivered today, so the adapter starts there.)*

### 2.3 A mapping file instead of code changes
A supplier will not use our names, units or enumerations. `FmuDUT` reads a JSON mapping:

```json
{"inputs":  {"VEH_Speed": {"name": "VehSpd_kph", "factor": 3.6}, "PLN_E2E_CRC": "PlnCmd_Crc", "...": "..."},
 "outputs": {"SAF_State": "SfmState", "...": "..."},
 "parameters": {},
 "state_values": {"0": "OFF", "1": "INIT", "2": "NORMAL", "...": "..."}}
```
- `factor` / `offset` convert units (FMU value = bench value × factor + offset).
- `state_values` maps their state enumeration to the bench's states, which is what the oracle judges.
- Required vs optional is defined in the contract. **Every** problem (unmapped required signal, a name that doesn't exist, wrong causality, wrong type) is collected and reported **at once**, before any test runs.

### 2.4 Black-box observation
The oracle needs the first reaction time and cause, the state timeline and whether a release was accepted. `FmuDUT` derives all of it **from the FMU's outputs only** (`BlackBoxObserver`, shared with the CAN adapter). It never looks inside the FMU.

### 2.5 Deterministic, not real time
The FMU runs as fast as the CPU allows (about 1 s per 8 s scenario), and the same seed gives the same result. That makes an **exact** back-to-back comparison possible: the full state timeline to the millisecond, plus the plant's end state. Real-time effects (jitter, scheduling) are the CAN adapter's job (v2.1).

## 3. How the reference FMU was made (and its honest limits)

- `fmu/SafeStopVecu.py` wraps the reference safety controller as an FMI 2.0 CS slave using **PythonFMU 0.7.0**. `scripts/build_fmu.py` stages the controller code as the package `vecu_core` inside the FMU's `resources/` folder, so the FMU **never imports the bench's own `ssb` package**. To the bench it is a zip with an XML file and a DLL.
- `fmu/SupplierStyleVecu.py` is the **same controller exported the way a supplier would**: their own names (`PlnCmd_Crc`, `SfmState`, `Dio_WdgTrigCnt`…), vehicle speed in **km/h**, a **different state enumeration** (OFF = 0), and **no test hooks**. The bench can only drive it through `fmu/mapping_supplier_style.json`. This is what proves the mapping path.
- **Limit:** a PythonFMU DLL embeds the host's Python interpreter; a supplier FMU is compiled C. The bench side (FMPy → FMI 2.0 C API → DLL) is identical, which is what this step tests. Performance, memory and threading behaviour of a compiled FMU will differ.

## 4. Intake: what to run the day an FMU arrives

```bash
.venv\Scripts\python.exe -m ssb.fmu_inspect their_vecu.fmu --write-mapping their_mapping.json --lifecycle
```

| Check | Why it matters | Blocks the run if |
|---|---|---|
| FMPy validation of `modelDescription.xml` | A malformed description is the most common FMU defect; importers react differently to it | Any error |
| FMI version, CS / ME | The adapter drives FMI 2.0 CS | ME only, or FMI 3.0 (needs an adapter extension) |
| Platforms vs this PC | No `win64` binary = cannot load on Windows | Missing binary for this platform |
| Variable list with types, variability, units, start values | The basis of the mapping, and a quick read of how the supplier thinks about the interface | — |
| Proposed mapping (exact → normalised → fuzzy, marked CHECK) | Saves hand work; every guess is shown for a human to confirm | A required bench signal has no candidate |
| Lifecycle in **child processes**: run, reset, two instances, free-then-reinstantiate | A crashing FMU must not kill the bench; lifecycle bugs are common and only show up in campaigns | A required sequence crashes or hangs |

Then the runs:

```bash
.venv\Scripts\python.exe run.py --dut fmu --file their_vecu.fmu --mapping their_mapping.json                   # the 52-scenario matrix
.venv\Scripts\python.exe run.py --dut fmu --file their_vecu.fmu --mapping their_mapping.json --b2b-dut         # vs our reference
```

The back-to-back run against our reference is the fastest way to find **where their behaviour differs**. Each difference is either their bug, our bug, or a requirement that was read two ways. All three are worth knowing before HiL.

## 5. Results (2 Oct 2026)

| Run | Result |
|---|---|
| Reference FMU `SafeStopVecu.fmu`, full matrix (52 scenarios) | **51/52 + 1 known finding**, identical to in-process; ~1 s per scenario |
| Reference FMU, **exact** back-to-back vs in-process reference | **52/52 identical**: every state change to the millisecond, plus the vehicle's end speed and lateral position |
| Bug seeded **inside** the FMU (`--defect no_latch`) | 18 scenarios fail, the **same 18** as the same bug in-process |
| Supplier-style FMU (other names, km/h, other state codes, no hooks), driven only through `mapping_supplier_style.json` | Intake: valid; 31 of 31 proposed matches correct; 1 optional signal mapped by hand. Matrix: **49/52 + 1 known**, 2 FAIL = the cold-start gap (finding 5 below); back-to-back 50/52, the same 2 |
| Intake lifecycle (5 repeats each, child processes) | Run ✅, reset ✅; reference two-instance ✅, supplier-style two-instance ❌ 3–8 in 10 (finding 3); free→re-instantiate ❌ always (finding 2; **corrected in v2.4**, see §8) |
| Unit tests | 18 pass (5 new FMU tests: exact b2b on 12 scenarios incl. both same-millisecond cases, seeded bug caught, mapping errors listed, supplier FMU via mapping, mapping proposal); FMU tests skip cleanly without FMPy |
| Regression | In-process default and off-road: 51/52 + 1 known each; CAN adapter (refactored onto the shared observer) re-checked on 2 scenarios ✅ |

Reports: `reports/report_fmu.html`, `reports/supplier_style/report_fmu.html`, `reports/fmu_mutant/`.

## 6. Findings while building it

Each one is a problem a supplier FMU can have too. Every one is now caught by the intake tool, the adapter, or the oracle.

1. **The export tool's defaults broke the FMI rules, twice.** PythonFMU 0.7.0 gave Integer and Boolean variables `variability="continuous"` (forbidden), and left `ModelStructure/InitialUnknowns` empty although the outputs are "calculated" (also forbidden). The adapter's schema check let the second one through; only FMPy's full rule check (`validate_fmu`, run by the intake tool) caught it. Fix: discrete variability, outputs `initial="exact"` with a start value. *Lesson: run the full validator, not just schema parsing. Some importers accept these silently and some refuse them, so the same FMU "works" in one tool and not another.*
2. ~~**Free → instantiate again in one process is an access violation**~~ **Corrected in v2.4: this was the bench's own test.** FMPy's `freeInstance()` also unloads the DLL, and the test re-used the unloaded wrapper. With a fresh wrapper, the reference FMU re-instantiates fine (§8). The adapter still uses one instance per campaign with `fmi2Reset` between scenarios (cheaper anyway). *Lesson kept: test the lifecycle on intake, in child processes. New lesson: prove the harness before blaming the binary.*
3. **An intermittent crash: two live instances of the supplier-style FMU crashed 8 runs in 10** (Python thread-state fault in the embedded interpreter); the reference FMU, 0 in 10. My first single-shot lifecycle check missed it. Fix: the lifecycle check repeats every sequence (default 5×) and reports the crash rate. Only the sequences the bench actually needs (run, reset) block a campaign; the others are warnings with the reason they matter (parallel runs, no-reset fallback).
4. **Name matching on its own is dangerous.** The first proposer (plain string similarity) **swapped `PLN_AccelReq` and `PLN_SpeedReq`** (`PlnCmd_VReq` vs `PlnCmd_AxReq`) and found only 16 of 31 signals. A swapped acceleration/speed pair would have produced nonsense results that look like controller bugs. Fix: an automotive abbreviation dictionary (Ctr/Cnt/Alive → counter, Ax → accel, Wdg → watchdog, Qly → health…), owner prefixes ignored, best-score-first assignment. Now 31 of 31 proposals are correct, every one is still marked CHECK, a km/h speed is flagged with factor 3.6, and the one signal no name can reveal (`Bench_TxOk` → `Can_ActBusOk`) is listed as UNMAPPED for a human.
5. **The oracle had a blind spot that only back-to-back exposed.** The supplier-style FMU has no start-mode hook, so it began cold-start scenarios in NORMAL instead of INIT. The matrix still said PASS: the vehicle behaved safely, and nothing checked the start state. The exact back-to-back flagged both scenarios. Fix: a new oracle check, "cold start begins in INIT" (SR-06). The supplier-style run now fails them, exit 1. *Lesson: pass/fail checks only what someone thought of; back-to-back against a reference finds what nobody wrote a check for.*
6. **Two planner frames can arrive in the same millisecond.** A plain "new data" flag would silently drop one, and the E2E counter check would then report a lost frame: a false finding. Fix: Rx counters and a one-deep receive queue in the adapter.
7. **Raw vs physical signals decide whether E2E is testable at all.** With physical-only signals the CRC is lost. Ask the supplier for raw COM-level signals (or FMI 3.0 binary) if E2E is in their scope.

## 8. v2.4: a compiled C FMU (what a supplier actually ships)

`fmu/SafeStopVecuC.fmu`, built by `scripts/build_c_fmu.py`: the C++ core from the HiL work (`hil/SafeStopCore`, unchanged) behind a hand-written FMI 2.0 co-simulation API (`fmu/c_src/SafeStopVecuC.cpp`, ~250 lines). `binaries/win64` DLL + `modelDescription.xml`, no Python inside.

```
.venv\Scripts\python.exe scripts/build_c_fmu.py
.venv\Scripts\python.exe -m ssb.fmu_inspect fmu/SafeStopVecuC.fmu --lifecycle
.venv\Scripts\python.exe run.py --dut fmu --file fmu/SafeStopVecuC.fmu --b2b-dut      # exact vs the Python reference
```

| Check | Result (5 Oct 2026) |
|---|---|
| FMI 2.0 validation (FMPy, full rules) | clean |
| Intake lifecycle, 5 repeats, child processes | run ✅ · reset ✅ · two instances ✅ · free → re-instantiate ✅ |
| Full matrix, 52 scenarios | 51/52 + 1 known finding (same as every DUT) |
| **Exact back-to-back vs in-process Python reference** | **52/52 identical** |
| Off-road calibration via `Bench_Config` | exact on the tested scenarios |
| Seeded bug `no_latch` via `Bench_Defects` | caught |
| Speed through FMPy (20 000 × 1 ms steps) | 3.7 µs/step vs 12.9 µs for the PythonFMU |

**Design choices:** one generator table → header + XML (they can't drift); GUID = content hash (a changed core gives a new GUID, so a stale XML/binary pair is refused); no allocation in `fmi2DoStep`; any step size, split into 1 ms sub-steps; unsupported optional functions return `fmi2Error` and the capability flags say so.

**Correction to finding 2 (§6):** see `docs/CHANGES_V2.md` v2.4, finding 1.

**Honest limits:** win64 binary only (no Linux `.so` in the zip yet); FMI 2.0, not 3.0; the "supplier" is still my own core, so back-to-back proves the packaging and the port, not an independent implementation.
