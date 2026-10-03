# safe-stop-bench

**A test bench for the safety layer between an autonomous vehicle's planner and its actuators, with the same 52 scenarios running at five levels: Python SiL, a separate process over CAN, an FMI 2.0 FMU, the firmware's C++ on the PC, and an ESP32-S3 on a real CAN bus.**

```text
planner (the "doer") ──CAN FD, E2E Profile 5, 20 ms──► SAFETY CONTROLLER (the "checker", device under test) ──classic CAN, E2E Profile 2──► actuator ECU ──► vehicle model
   + hardware watchdog line                              timeout · E2E window · freshness · window + Q&A watchdog
                                                         envelope + jerk limit · commanded vs measured
                                                         reaction ladder: degraded → pull over → stop in lane → brake-only → backup brake (latched)
```

> **Synthetic data and illustrative limits throughout.** This is a portfolio project and a method demonstrator, not a safety product. It tests the safety layer *around* a planner, not the planner itself.
> **AI-assisted:** designed and written with Claude Code to the author's specification; every result here was produced by running it.

## Why it exists

A learned planner can't be fully verified, but the simple controller that checks every command, and owns the safe stop, can. This bench shows how I'd prove that controller works, cheapest layer first, and how I'd test one that a supplier hands over as a black box.

## The five levels

| `--dut` | What runs the safety controller | What it proves | Status |
|---|---|---|---|
| `reference` | Python reference model | The requirements and the test suite | ✅ 51/52 + 1 known finding |
| `can` | A **separate process**, reachable only over CAN (python-can + cantools DBC), in real time | The black-box method, with real transport | ✅ same result |
| `fmu` | An **FMI 2.0 co-simulation FMU** via FMPy, wired through a name/unit/enum **mapping file** | Testing a supplied vECU; exact back-to-back | ✅ exact 52/52 |
| `native` / `loopback` | The **firmware's C++ core** on the PC; then the board's whole node logic over the link protocol | The port is bit-exact before it goes on a chip | ✅ exact 52/52; 14/14 mutants identical |
| `pil` / `hil` | **ESP32-S3** (MCP2515 + TJA1050); outputs read back over USB, then **through a real CAN bus** by a second board | Real MCU timing, a real bus | ⏳ built, emulator-tested; [bring-up guide](docs/HIL_BRINGUP.md) |

## Quick start

```bash
python run.py                                   # 52 scenarios → reports/report.html (standard library only)
python run.py --full                            # + speed × friction sweep, false-stop rate, fuzzing, mutation score
python run.py --defect no_latch                 # run against one of 14 seeded bugs: the suite must catch it
python -m unittest discover -s tests -v

pip install -e .[can,fmu,hil]                   # optional adapters
python scripts/build_fmu.py && python run.py --dut fmu --b2b-dut
python -m ssb.fmu_inspect their_vecu.fmu --write-mapping map.json --lifecycle      # intake for a supplied FMU
python scripts/build_native.py && python run.py --dut native --b2b-dut             # firmware C++, exact vs Python
python run.py --dut pil --port-b EMU                                               # dry run with an emulated board
python scripts/hil_flash.py --b COM14 --a COM13 && python run.py --dut hil --port-b COM14 --port-a COM13 --b2b-dut
```

## What the bench found

1. **A grade hides a weak brake** unless the brake check removes the slope's share of the measured deceleration.
2. **Pass/fail missed a defect that back-to-back caught.** A supplier-style FMU cold-started in NORMAL instead of INIT, and the matrix still said PASS. The missing oracle check has since been added.
3. **30 ms of jitter on 20 ms frames reorders them**, so E2E rejects them and the vehicle stops. Also, a 100 ms timeout effectively tolerates only a 60 ms outage.
4. **Defence in depth works:** with the hardware watchdog removed, a hung planner is still caught by the question-and-answer watchdog.
5. **FMU intake catches real supplier-FMU problems:**
   - the export tool's defaults wrote an invalid model description;
   - free → re-instantiate crashed the process;
   - two live instances crashed intermittently, 8 runs in 10;
   - name matching swapped acceleration and speed.
6. **Bit-exact porting needs four rules:** half-to-even rounding, Python's non-negative modulo, Python's tie behaviour of max/min, and no fused multiply-add.
7. **On a two-node CAN bus, a pulled wire gives error passive, not bus-off** (the ACK-error exception), and one node alone never gets an ACK. So bus monitoring is a reset option.

Every item, with its status and reason: [docs/CHANGES_V2.md](docs/CHANGES_V2.md).

## Docs

| Doc | What |
|---|---|
| [HOW_IT_WORKS_AND_TESTING_A_SUPPLIED_VECU.md](docs/HOW_IT_WORKS_AND_TESTING_A_SUPPLIED_VECU.md) | How the bench works; testing a vECU you can't see inside: formats, eight stages, four oracles |
| [FMU_ADAPTER.md](docs/FMU_ADAPTER.md) | FMUs explained, the adapter's design decisions, intake, results, findings |
| [HIL_ESP32.md](docs/HIL_ESP32.md) | The SiL → PiL → HiL ladder, architecture, honest deviations, findings |
| [HIL_BRINGUP.md](docs/HIL_BRINGUP.md) | Step-by-step hardware bring-up, with a pass criterion for each step |
| [CHANGES_V2.md](docs/CHANGES_V2.md) | What changed from v1 to v2.3, what's done, partial or not done, and why |

## Layout

| Path | What |
|---|---|
| `ssb/safety.py` | Reference safety controller + 14 seeded mutants |
| `ssb/e2e.py` | E2E Profiles 5 and 2, CRCs, windowed state machine |
| `ssb/planner.py`, `ssb/plant.py`, `ssb/bus.py` | Planner with fault hooks, actuator ECU + bicycle-model vehicle, virtual CAN / CAN FD bus |
| `ssb/runner.py`, `ssb/oracle.py`, `ssb/campaigns.py`, `ssb/report.py` | Scenario runner with invariants every ms; verdicts; matrix, sweep, false-stop, fuzz, mutation, back-to-back; HTML / JSON / JUnit |
| `ssb/dut.py` | Device-under-test interface; reference, CAN and FMU adapters |
| `ssb/fmu_contract.py`, `ssb/fmu_inspect.py`, `fmu/` | FMU contract, intake tool, reference + supplier-style FMU sources and mappings |
| `ssb/native.py`, `ssb/hil.py`, `ssb/hil_emu.py` | C++ core via ctypes; link protocol, PiL/HiL adapters, board emulator |
| `hil/SafeStopCore/`, `hil/firmware/` | Portable C++ core (Arduino library); SafetyNode (board B) and BusNode (board A) sketches |
| `scenarios/`, `safety/`, `config/`, `dbc/` | 41 scenarios traced to requirements; hazards → goals → requirements (with FTTI); SOTIF catalogue; configs; DBC |

## License

MIT, see [LICENSE](LICENSE).
