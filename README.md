# safe-stop-bench

**A test bench for the safety layer between an autonomous vehicle's planner and its actuators. The same 56 scenarios run at seven levels: Python SiL, a separate process over CAN, a ROS 2 node, an FMI 2.0 FMU, the firmware's C++ on the PC, the board's node logic over its link protocol, and two ESP32-S3 boards on a real CAN bus.**

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

## The levels

| `--dut` | What runs the safety controller | What it proves | Result (v2.12 unless noted) |
|---|---|---|---|
| `reference` | Python reference model | The requirements and the test suite | ✅ 55/56 + 1 known finding |
| `can` | A **separate process**, reachable only over CAN (python-can + cantools DBC) | The black-box method, with real transport | ✅ lockstep: 55/56 + 1 known, back-to-back **exact 56/56**, unaffected by host stalls; real time needs a quiet PC |
| `ros2` | A **ROS 2 node** in its own process, reachable only through topics (WSL) | The same method over ROS 2 | ✅ 53/54 + 1 known, back-to-back 54/54 (v2.8, 54 scenarios then) |
| `fmu` | An **FMI 2.0 co-simulation FMU** via FMPy, wired through a name/unit/enum **mapping file** | Testing a supplied vECU | ✅ back-to-back **exact 56/56** |
| `native` / `loopback` | The **firmware's C++ core** on the PC; then the board's whole node logic over the link protocol | The port is bit-exact before it goes on a chip | ✅ back-to-back **exact 56/56** each |
| `pil` / `hil` | **ESP32-S3** + MCP2515/TJA1050: board B is the controller, board A reads its outputs **from a real 500 kbit/s CAN bus**; GPIO watchdog wire; real reset mid-scenario | Real MCU timing, a real bus | ✅ 54/55 + 1 known, 0 FAIL, back-to-back 55/55 (v2.9.8, 55 scenarios then; [first-run report](docs/HIL_FIRST_RUN.md)). Power cut via a relay: prepared, not yet run ([plan](docs/HIL_POWER_CUT.md)) |

## Quick start

```bash
python run.py                                   # 56 scenarios → reports/report.html (standard library only)
python run.py --full                            # + speed × friction sweep, false-stop rate, fuzzing, mutation score
python run.py --defect no_latch                 # run against one of 14 seeded bugs: the suite must catch it
python -m unittest discover -s tests -v

pip install -e .[can,fmu,hil]                   # optional adapters
python run.py --dut can --can-lockstep --b2b-dut                                   # vECU process over CAN, exact vs reference
python scripts/build_fmu.py && python run.py --dut fmu --b2b-dut
python -m ssb.fmu_inspect their_vecu.fmu --write-mapping map.json --lifecycle      # intake for a supplied FMU
python scripts/build_native.py && python run.py --dut native --b2b-dut             # firmware C++, exact vs Python
python run.py --dut pil --port-b EMU                                               # dry run with an emulated board
python scripts/hil_flash.py --b COM14 --a COM13 && python run.py --dut hil --port-b COM14 --port-a COM13 --kick gpio --b2b-dut
```

## What the bench found

On the PC:

1. **A grade hides a weak brake** unless the brake check removes the slope's share of the measured deceleration.
2. **Pass/fail missed a defect that back-to-back caught.** A supplier-style FMU cold-started in NORMAL instead of INIT, and the matrix still said PASS. The missing oracle check has since been added.
3. **30 ms of jitter on 20 ms frames reorders them**, so E2E rejects them and the vehicle stops. Also, a 100 ms timeout effectively tolerates only a 60 ms outage.
4. **Defence in depth works:** with the hardware watchdog removed, a hung planner is still caught by the question-and-answer watchdog.
5. **FMU intake catches real supplier-FMU problems:** an invalid model description from the export tool's defaults, intermittent crashes with two live instances, name matching that swapped acceleration and speed, and one "FMU crash" that was the test harness itself (FMPy unloads the DLL on free).
6. **Bit-exact porting needs four rules:** half-to-even rounding, Python's non-negative modulo, Python's tie behaviour of max/min, and no fused multiply-add.

On the two boards ([details](docs/HIL_FIRST_RUN.md)):

7. **Sample the DUT on its own clock.** A steering-slew violation was USB arrival jitter on the PC's clock; the actuator frame's E2E counter is the DUT's time base.
8. **An unprotected status message is not evidence.** Old status frames reached the bench now and then, making a latched stop look released for 1 ms. The source was the bus node: on these boards its MCP2515 flag register was sometimes read as "buffer 1 full" just as buffer 0 filled, and an old buffer was re-read. Fixed in the receiver; and the status message now carries a CRC-8 and an 8-bit alive counter at every level (DBC, vECU, firmware), so any receiver can reject a stale copy.
9. **Most host "stalls" were the bench's own set-up after the clock started.** Set-up first, then reset, then clock; a stall that still happens reruns the scenario as a bench fault, never a verdict.
10. **Test a reset for real.** The bench pulls board B's reset pin mid-scenario; it boots from its stored configuration straight into the latched stop (first status 185–187 ms after reset), and a build without that restore fails the test.
11. **Lost CAN frames came from the receiver's own busy-waits**, not the PC (the USB-serial chip has no flow control). Reception moved to its own task on the other core with a RAM queue: 40 overflows a minute → 0.
12. **When real time isn't the question, use lockstep.** Over CAN on a busy laptop, real time needed 71 stall reruns; lockstep (the bench waits for each 10 ms cycle, identified by the alive counter) gave 0 reruns and an exact match with the reference.

Every item, with its status and reason: [docs/CHANGES_V2.md](docs/CHANGES_V2.md).

## Docs

| Doc | What |
|---|---|
| [HOW_IT_WORKS_AND_TESTING_A_SUPPLIED_VECU.md](docs/HOW_IT_WORKS_AND_TESTING_A_SUPPLIED_VECU.md) | How the bench works; testing a vECU you can't see inside: formats, eight stages, four oracles |
| [FMU_ADAPTER.md](docs/FMU_ADAPTER.md) | FMUs explained, the adapter's design decisions, intake, results, findings |
| [ROS2_ADAPTER.md](docs/ROS2_ADAPTER.md) | The safety controller as a ROS 2 node: setup in WSL, real time vs lockstep, results |
| [HIL_ESP32.md](docs/HIL_ESP32.md) | The SiL → PiL → HiL ladder, architecture, honest deviations |
| [HIL_BRINGUP.md](docs/HIL_BRINGUP.md) | Step-by-step hardware bring-up, with a pass criterion for each step |
| [HIL_FIRST_RUN.md](docs/HIL_FIRST_RUN.md) | What happened on the two boards: findings, probes, fixes, result tables |
| [HIL_POWER_CUT.md](docs/HIL_POWER_CUT.md) | The power-cut test, prepared before the relay arrives: parts, wiring, procedure, expected results |
| [DYNAMIC_PLANT.md](docs/DYNAMIC_PLANT.md), [E2E_TUNING.md](docs/E2E_TUNING.md) | Mass / payload / grade plant; E2E false-stop tuning |
| [CHANGES_V2.md](docs/CHANGES_V2.md) | Every version from v2.0 to v2.12: what changed, what's done, partial or not done, and why |

## Layout

| Path | What |
|---|---|
| `ssb/safety.py` | Reference safety controller + 14 seeded mutants |
| `ssb/e2e.py` | E2E Profiles 5 and 2, the protected status frame, CRCs, windowed state machine |
| `ssb/planner.py`, `ssb/plant.py`, `ssb/bus.py` | Planner with fault hooks, actuator ECU + kinematic or dynamic vehicle, virtual CAN / CAN FD bus |
| `ssb/runner.py`, `ssb/oracle.py`, `ssb/campaigns.py`, `ssb/report.py` | Scenario runner with invariants every ms and bench-fault reruns; verdicts; matrix, sweep, false-stop, fuzz, mutation, back-to-back; HTML / JSON / JUnit / MDF4 |
| `ssb/dut.py`, `ssb/vecu_process.py`, `ssb/canio.py` | Device-under-test interface; reference, CAN (real time or lockstep) and FMU adapters; the vECU process |
| `ssb/ros2_*.py`, `ros2/` | ROS 2 adapter, node and WSL helpers |
| `ssb/fmu_contract.py`, `ssb/fmu_inspect.py`, `fmu/` | FMU contract, intake tool, reference, supplier-style and C FMU sources and mappings |
| `ssb/native.py`, `ssb/hil.py`, `ssb/hil_emu.py` | C++ core via ctypes; link protocol, PiL/HiL adapters, board emulator |
| `hil/SafeStopCore/`, `hil/firmware/` | Portable C++ core (Arduino library); SafetyNode (board B), BusNode (board A) and SPI diagnostic sketches |
| `scripts/*_probe.py`, `scripts/spi_stress.py` | The hardware probes behind the findings above, and the power-cut probe |
| `scenarios/`, `safety/`, `config/`, `dbc/` | Scenarios traced to requirements; hazards → goals → requirements (with FTTI); SOTIF catalogue; configs; DBC |

## License

MIT, see [LICENSE](LICENSE).
