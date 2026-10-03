# Safe-stop bench: how it works, and how to test a virtual ECU we're given but can't see inside

**Written:** 2 Oct 2026, as the plan for testing a supplied virtual ECU.
**Confidence tags:** ✅ = how this bench actually works (checked by running it) · 📚 = general industry knowledge · 🟡 = an assumption to confirm with the supplier.

---

## 1. The short version

Today the bench tests **our own** safety controller: a reference model written in Python. In real use, the controller under test would be **the supplier's own virtual ECU (vECU)**, delivered as files in a format we don't know yet, with no access to its source.

That doesn't change the method. It changes **what the bench is for**:

| Today | With the supplier's vECU |
|---|---|
| The bench contains the controller | The bench **surrounds** the controller |
| Our `SafetyController` is the device under test | Their vECU is the device under test (DUT); ours becomes a **reference** to compare against |
| We know every internal state | We see only its **inputs and outputs** (black-box testing) |

So the bench becomes three parts we own: **stimulus** (planner + fault injection), **plant** (vehicle model) and **oracle** (pass/fail rules). A thin **adapter** connects them to their vECU in whatever format it arrives.

```text
          WE OWN THIS (the test harness)                             THEY OWN THIS
┌──────────────────────────────────────────────────────┐
│  scenarios + fault injection  ──►  stimulus frames ──┼──►┌──────────┐   ┌────────────────┐
│                                                      │   │ ADAPTER  │──►│ Supplied vECU  │
│  vehicle model  ◄── actuator commands  ◄─────────────┼───┤ (per     │◄──│ (black box)    │
│                                                      │   │ format)  │   └────────────────┘
│  oracle: requirements + invariants + reference model │   └──────────┘
│  report: PASS / FAIL, times, distances               │
└──────────────────────────────────────────────────────┘
```

---

## 2. How the bench works today ✅

### 2.1 The parts

| File | What it is | Role in a test |
|---|---|---|
| `e2e.py` | E2E protection in the style of AUTOSAR Profile 5: CRC-16/CCITT-FALSE over data ID + counter + payload, 8-bit alive counter, receiver statuses (OK, OK_SOME_LOST, REPEATED, WRONG_CRC, WRONG_SEQUENCE, NO_NEW_DATA) | Protects and checks every command frame |
| `bench.py` → `Planner` | Stand-in for the RL planner and its comms. Sends a command every 20 ms and kicks the watchdog | **Stimulus**, with fault hooks |
| `bench.py` → `SafetyController` | Our reference safety controller (see 2.3) | **Device under test today**; the **reference model** later |
| `bench.py` → vehicle model (in `run()`) | Speed and distance from the commanded acceleration, 1 ms step | **Plant**: turns commands into stopping time and distance |
| `bench.py` → `SCENARIOS` | 11 scenarios: fault, injection time, expected reaction | **Test cases** |
| `bench.py` → checks in `run()` | Pass rules per scenario | **Oracle** |
| `run.py` | Runs the matrix, writes `reports/report*.html` and `results*.json`; exit code 1 on any failure | **Test runner + report**; CI gate |
| `test_bench.py` | Unit tests: CRC check value, receiver statuses, wrong data ID, matrix passes, seeded defects are caught | **Tests of the bench itself** |

### 2.2 Timing

| Item | Period | Why |
|---|---|---|
| Simulation step | 1 ms | Fine enough to measure detection times to the millisecond |
| Planner command + watchdog kick | 20 ms | A typical command rate for a planner-to-controller interface (illustrative) |
| Safety-controller cycle | 10 ms | Runs twice per command, so a missing frame is noticed within one cycle |

### 2.3 What the safety controller checks (the rules under test)

| Check | Rule (illustrative values) | Fault cause reported |
|---|---|---|
| E2E | 3 invalid frames in a row (bad CRC, repeated, wrong sequence) | `E2E_INVALID` |
| Timeout | No valid command for > 100 ms | `TIMEOUT` |
| Window watchdog, late | No kick for > 60 ms | `WATCHDOG_LATE` |
| Window watchdog, early | 3 kicks in a row less than 10 ms apart | `WATCHDOG_EARLY` |
| Envelope | Steering rate > 30 °/s, speed request > 40 km/h (the ODD), acceleration outside −6 to +2.5 m/s²; the command is clamped at once, and a fault is raised if it persists ≥ 50 ms | `ENVELOPE` |
| Reaction | Minimal risk manoeuvre: decelerate at 3 m/s², steering held, **latched** until an operator releases it | state `MRM` |

### 2.4 One scenario, step by step: `planner_hang`

1. From 0 to 2.0 s, the planner sends valid frames and kicks the watchdog every 20 ms. The vehicle cruises at 30 km/h.
2. At 2.0 s the fault is injected: the planner logic hangs, **but its comms thread keeps sending the last command with a fresh counter and a valid CRC**. E2E sees nothing wrong.
3. The watchdog stops being kicked. At 2.05 s (last kick 1.98 s + 60 ms + one cycle), the safety controller raises `WATCHDOG_LATE` and starts the stop.
4. The vehicle decelerates at 3 m/s² and stops 2.83 s after the fault, 11.56 m later. That matches the physics: v²/2a = 8.33² / 6 ≈ 11.57 m.
5. The oracle checks five things: a stop was triggered, the cause was the expected one, it was detected within 200 ms, the vehicle stopped, and it was still latched at the end. All five pass.

**Why this scenario matters:** it's the case where a message-level check alone would pass a dangerous situation. It shows why a safety layer needs **several independent checks**.

### 2.5 How pass/fail is decided

- **Fault scenarios:** a stop is triggered · the expected cause · detected within the 200 ms budget (an illustrative share of the FTTI) · the vehicle stops · still latched at the end.
- **No-fault scenarios:** no stop at all (the false-stop check), including 60 s with 0.5% of frames corrupted at random.

### 2.6 Testing the tester: seeded defects ✅

A test that can't fail proves nothing. So the bench ships with three deliberate bugs, and each must turn specific tests red:

| Defect | What it breaks | Result |
|---|---|---|
| `no_latch` | The stop releases itself when the fault clears | 6/11: the vehicle stops and starts again |
| `no_watchdog` | No watchdog | 9/11: hung and looping planner never caught |
| `long_timeout` | Timeout 1000 ms instead of 100 ms | 9/11: misses the detection budget |

📚 This is **mutation testing**: bugs are put in on purpose to measure whether the test suite notices them.

---

## 3. Testing the supplier's vECU without seeing inside it

### 3.1 What we need from them, and why

A black-box test can only judge what is **specified** and what is **observable**. So the first deliverable isn't code; it's the **interface contract**.

| Ask | Why we need it | If they can't give it |
|---|---|---|
| **The format and how to run it** (see 3.2) | Decides which adapter we build | We identify it ourselves (3.3) |
| **Input and output signal list**: names, units, ranges, scaling, byte layout (a DBC, ARXML, ROS message definitions or a C header) | Without it we can't build stimulus frames or read outputs | We read it from the file where the format allows (an FMU lists its variables) |
| **E2E configuration**: profile, data ID, counter offset, max delta | Without the right data ID, every frame we send fails its CRC check | Ask: this has no workaround |
| **Timing**: cycle times, timeouts, watchdog window | Detection times can only be judged against their numbers | We measure, then ask them to confirm |
| **Requirements for fault reactions**: what it must do, and how fast (their FTTI) | These are the **expected results** of our tests | We use invariants and the reference model (3.6), and flag the gaps |
| **A state or diagnostic output** (normal / degraded / MRM, a fault code) | Lets us see *why* it reacted, not only *that* it did | We infer from actuator outputs, which is weaker (3.7) |
| **How to start, reset and stop it**; any config files | Every test must start from a known state | Trial and error; slower |
| **Version and a file hash** | Results must be traceable to the exact build tested | We hash it ourselves on receipt |
| **Known limitations** | Avoids raising defects they already know about | — |

### 3.2 The formats a vECU may arrive in, and how each connects

📚 "Virtual ECU" isn't one format. These are the common ones, roughly from most to least convenient for a test bench:

| Format | How you recognise it | How the bench drives it | Time control | Notes |
|---|---|---|---|---|
| **FMU** (Functional Mock-up Unit, FMI 2.0 / 3.0, co-simulation) | A `.fmu` file; it's a zip containing `modelDescription.xml` and a binary | Load it with **FMPy** (open-source Python). Each step: set inputs, call `doStep(1 ms)`, read outputs | **Exact**: the bench owns time | The industry standard for exchanging models and vECUs. The XML lists every input and output, so the interface documents itself. **Best case** |
| **Shared library** (`.so` / `.dll`) + a C header | A library file and a `.h` with functions like `init()`, `step()` | Call it from Python with `ctypes` | Exact | Easy if the header is clear; crashes take the bench down, so run it in a separate process |
| **Simulink model** (`.slx`) or **generated C code** | `.slx`, or C files from Embedded Coder | Run in MATLAB (needs a licence), or compile the code into a library, or export an FMU | Exact | Ask them to **export an FMU**; it removes the licence problem |
| **A Linux program or Docker container that speaks CAN** | An executable or an image, plus a DBC | Connect both to a virtual CAN interface (`vcan0`, SocketCAN); the bench uses `python-can` | **Wall-clock**: real time, with jitter | Realistic, but timing varies run to run, so use tolerances and repeat runs (3.8). Needs Linux or WSL2 |
| **A ROS 2 node / package** | A package with `package.xml`, message definitions | The bench publishes and subscribes with `rclpy`; run with **simulated time** (`/clock`) | Exact if they honour sim time | 🟡 Plausible for the supplier, since the role mentions an inter-process communication environment |
| **A commercial vECU** (Synopsys Silver, dSPACE VEOS, Vector tools) | Tool-specific project files | Needs that tool and its licence | Exact | Ask for an **FMU export**, or a **Vector SIL Kit** connection (📚 SIL Kit is open source) |
| **Firmware for the real microcontroller** (`.elf` / `.hex`) | A binary for a specific MCU | Not a vECU. Run it on the real board (**minimal HiL**) or in an MCU emulator such as **Renode** (📚 open source) | Depends | The most realistic, but the most work |

**What to ask for:** *"An FMU if possible; otherwise a container that speaks CAN on a virtual interface, plus the DBC and the E2E configuration."* Those two cover most cases with the least effort.

### 3.3 Step 0: identifying an unknown file

| Check | What it tells you |
|---|---|
| File extension | `.fmu`, `.so`, `.dll`, `.slx`, `.elf`, `.hex`, `.tar` (Docker image), a folder with `package.xml` (ROS 2) |
| Unzip it (an FMU is a zip) | `modelDescription.xml` → FMI version, co-simulation or model exchange, every variable with its type and causality (input / output / parameter) |
| `file <name>` on Linux | ELF executable, shared object, architecture (x86-64 vs ARM) |
| List exported symbols of a library (`nm -D`, or `dumpbin /exports` on Windows) | The function names you'll need to call |
| `docker image inspect` | Entry point, exposed ports, environment variables |
| Anything else | Ask them. Guessing an interface wastes days |

### 3.4 How to show that their ECU works properly: eight stages

Each stage only makes sense once the one before it passes. Each has a reason to exist.

| Stage | What we test | Example checks | Why this stage exists |
|---|---|---|---|
| **A. Integration smoke** | Does it load, start and talk? | Loads without error; produces outputs at the expected cycle; its own output frames have valid E2E | Separates "**it won't connect**" from "**it's wrong**". Without it, every later failure is ambiguous |
| **B. Interface conformance** | Does it match the contract? | Every signal in the spec exists, with the right units and ranges; cycle-time jitter within tolerance; counter increments and wraps 255 → 0; correct startup state | 📚 Most integration defects are interface mismatches. Catching them first is cheap |
| **C. Nominal behaviour** | Does it do its job when nothing is wrong? | Commands within limits pass through unchanged; **no false stop** over a long run with bus noise | A safety layer that stops for no reason will be switched off by the team. False stops count as defects too |
| **D. Fault-injection matrix** | Does it react correctly to each fault? | The 11 scenarios, mapped to **their** requirements; detection time against **their** FTTI; stop latched | This is the core of the safety claim: every fault has a proven reaction |
| **E. Boundaries** | Behaviour exactly at the limits | Steering rate 29.9 / 30.0 / 30.1 °/s; timeout at 99 / 101 ms; counter jump of exactly max delta and max delta + 1 | 📚 Defects cluster at boundaries (ISTQB boundary value analysis). Off-by-one errors in thresholds are common |
| **F. Back-to-back with the reference model** | Same stimulus into our reference controller and their vECU; compare outputs | Same cause, similar detection time, same final state | Gives an **independent oracle** where their spec is thin. 📚 ISO 26262 lists back-to-back comparison as a verification method. A mismatch is a defect, a reference error or a spec ambiguity, and all three are worth finding |
| **G. Regression and repeatability** | Same results every run and every version | Fixed random seed; identical results on rerun; whole matrix in CI on every new vECU version | A pass/fail that changes on rerun isn't evidence. Regression stops fixed bugs coming back |
| **H. Bench self-check** | Is the bench itself trustworthy? | Seeded defects still caught (2.6); unit tests pass; CRC check value correct | 📚 If a tool decides safety pass/fail, you need confidence in the tool (ISO 26262 calls this tool confidence) |

### 3.5 Mapping the existing scenarios to a supplied vECU

| Scenario | What we inject at the vECU's inputs | What we expect to see at its outputs |
|---|---|---|
| command_link_lost | Stop sending command frames; keep kicking | Deceleration command within their timeout + FTTI |
| crc_corrupt | Frames with a flipped CRC bit | Frames rejected; stop after their debounce count |
| counter_frozen | The same frame repeated | Treated as stale; stop |
| planner_hang | Valid fresh frames, no watchdog kicks | Stop: proves the watchdog is independent of the data path |
| planner_looping | Kicks every 2 ms | Stop, if they use a window watchdog. If not, **that's a finding to discuss**, not automatically a defect |
| steer / speed out of envelope | Commands beyond their limits | Output clamped at once; stop if it persists |
| power_dip | Planner side silent 300 ms, counter restarts at 0 | Stop; no reset of the vECU itself |
| latch_after_resume | Link lost 200 ms, then back | **Stays stopped** until the release input |
| single_bad_frame / nominal_with_noise | Occasional corrupted frames | **No stop** |

The scenarios don't change. Only the **expected values** come from their requirements instead of ours.

### 3.6 Where "expected" comes from: four kinds of oracle

| Oracle | Example | Strength | Weakness |
|---|---|---|---|
| **Their requirements** | "On command loss > 100 ms, enter MRM within 50 ms" | The real definition of correct | Often incomplete for faults |
| **Invariants**: rules that must always hold | "Never accelerate while in MRM" · "output steering rate never above the limit" · "once in MRM, stay there until released" | Work even with a thin spec; catch surprises | Don't say what *should* happen, only what must never happen |
| **Physics** | Stopping distance ≈ v² / 2a | Independent of any code | Only for the plant side |
| **Our reference model** (back-to-back) | Our controller and theirs on the same stimulus | Covers everything the stimulus exercises | Our model might be the one that's wrong, so investigate each mismatch |

**Justification:** no single oracle is enough for a black box. Requirements define correct, invariants catch the unexpected, physics checks the plant, and the reference model fills gaps. Where they disagree, you've found a defect or a spec ambiguity. This is the STAR-04 lesson: a "defect" that bounced between teams turned out to be an ambiguous requirement.

### 3.7 Seeing inside a black box (observability)

- **Best:** they expose a state output (normal / degraded / MRM) and a fault code. Then we check both the **reaction** and the **reason**.
- **Without it:** we infer from the actuator outputs: a deceleration command appears, steering is held. We can still measure **time to react**, but we can't tell a timeout from an E2E fault. Raise it as a testability finding: *"Add a state and cause output; it costs one signal and makes every field stop diagnosable."*
- **Timing is measured at the output**: from the moment we inject the fault to the first output that shows the reaction.

### 3.8 Time and determinism

| vECU type | Time | How to judge timing |
|---|---|---|
| FMU, library, simulated-time ROS 2 | The bench controls time; every run is identical | Exact limits, one run per case |
| Process or container on virtual CAN | Real (wall-clock) time; the operating system adds jitter | Run each case **N times** (e.g. 20); report min / typical / max; pass if the **max** is within the limit, with a stated margin |

**Justification:** a timing pass that depends on luck isn't evidence. With a stepped vECU, timing is exact. With a real-time one, you need the worst case over repeated runs.

### 3.9 What a virtual bench can't prove (be honest about this)

| Not covered | Why | Covered by |
|---|---|---|
| Timing on the **real microcontroller** | A PC runs the code at a different speed, with a different scheduler | Minimal HiL with the real safety MCU |
| **Electrical** faults: supply dips, shorts, open wires, EMC | No electrons in a simulation | Programmable supply + relay board on the HiL |
| **Compiler and target** differences | The vECU may be built for x86, the real ECU for ARM | Run the same matrix on hardware (back-to-back: vECU vs real ECU) |
| The **planner's** decisions | This bench tests the layer around it | Scenario-based testing and field-log replay (SOTIF) |

The same scenario files carry over to the HiL unchanged; only the adapter changes. Tests written now aren't wasted later.

---

## 4. What changes in the code

The scenarios, oracle and report stay the same. One interface is added, and one adapter per format:

```python
class DeviceUnderTest:            # the only thing the scenarios talk to
    def reset(self) -> None: ...
    def step(self, t_ms: int, frames_in: list[bytes], kicks: list[int]) -> "Outputs": ...
    # Outputs: actuator command (+ state and fault code if exposed)

class ReferenceDUT(DeviceUnderTest):   # wraps today's SafetyController
class FmuDUT(DeviceUnderTest):         # FMPy: set inputs, doStep(1 ms), get outputs
class CanDUT(DeviceUnderTest):         # python-can on vcan0; wall-clock timing
class Ros2DUT(DeviceUnderTest):        # rclpy publish/subscribe with /clock
```

**Built and tested in v2.2** (see `docs/FMU_ADAPTER.md`): `python -m ssb.fmu_inspect their_vecu.fmu --write-mapping m.json --lifecycle` does the intake, `run.py --dut fmu --file their_vecu.fmu --mapping m.json` runs the full matrix, and `--b2b-dut` runs back-to-back against the reference (stage F), exactly to the millisecond.

---

## 5. Glossary

| Term | Meaning |
|---|---|
| **vECU** | Virtual ECU: the controller's software running on a PC instead of its real hardware |
| **DUT** | Device under test |
| **Black-box testing** | Testing through inputs and outputs only, without the internals |
| **FMU / FMI** | Functional Mock-up Unit / Interface: the open standard for packaging models and vECUs for simulation |
| **Co-simulation** | The FMU contains its own solver; the bench just tells it to step forward |
| **SocketCAN / vcan** | Linux's CAN interface; `vcan` is a virtual one with no hardware |
| **Oracle** | Whatever decides the expected result of a test |
| **Back-to-back test** | The same inputs into two implementations; compare outputs |
| **Invariant** | A rule that must always hold, whatever the inputs |
| **Mutation testing** | Seeding deliberate bugs to check the tests catch them |
| **FTTI** | Fault tolerant time interval: time from a fault to possible harm; detection + reaction must fit inside it |
| **MRM / MRC** | Minimal risk manoeuvre / condition: the safe stop, and the stopped state it ends in |
