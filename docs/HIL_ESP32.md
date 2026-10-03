# ESP32-S3 HiL (v2.3): the safety controller on a real microcontroller and a real CAN bus

## 1. What this step adds, and the ladder it completes

The same 52 scenarios now run at five levels. Each level removes one "it only works because it's a simulation" excuse:

| Level | `--dut` | What runs the safety controller | What it proves | Status |
|---|---|---|---|---|
| SiL, reference | `reference` | Python (`ssb/safety.py`) | The requirements and the test suite | ✅ |
| **Native SiL** | `native` | **The firmware's C++** (`hil/SafeStopCore`), compiled for the PC | The C++ port behaves exactly like the reference, before it touches hardware | ✅ exact, 52/52 |
| **Loopback** | `loopback` | Board B's **whole node logic** (C++ core + link protocol + 10 ms scheduler) on the PC, fed bytes in lockstep | The firmware's message handling, framing, resync and scheduling, without hardware | ✅ exact, 52/52 |
| **PiL** (processor in the loop) | `pil` | **Board B, the ESP32-S3**, outputs read from its USB copy | Real MCU, real compiler (Xtensa GCC), real timing, real execution time | ⏳ needs the board plugged in |
| **HiL** | `hil` | Board B; its outputs read by **board A from the real CAN bus** (MCP2515 + TJA1050, 500 kbps) | Real bus: arbitration, ACK, error counters, transceivers, wiring | ⏳ needs both boards |

**The key idea is one source of truth for the firmware logic.** Everything except about 60 lines of Arduino shim is in `hil/SafeStopCore`. That code is proven against the Python reference on the PC, so when a PiL or HiL run differs from the reference, the cause is hardware or timing, not a porting bug. That's the triage rule that makes a HiL result trustworthy.

## 2. Architecture

```text
            USB (921600 baud, framed + CRC-8)                      CAN 500 kbps (real bus, 120 Ω both ends)
  PC  ─────────────────────────────────────►  board B  ═══════════════════════════════════════►  board A  ──USB──► PC
  planner, vehicle model,  'R' reset + calibration  SafetyNode: SafeStopCore         SAF_ActuatorCmd 0x200 (E2E P2)   BusNode: forwards every
  fault injection, oracle  'P' planner frame 14 B   (E2E P5 checked ON the MCU)      SAF_Status      0x201            received frame ('M')
                           'F' sensors  'K' kicks   10 ms task, millis() clock       every 10 ms
                           'C' power / tx / release
                                                    ◄── 'M' copy of each frame sent, 'D' diagnostics (exec time, lateness, tx failures)
  optional wire: board A GPIO5 ──► board B GPIO5 (+ GND) = a real hardware watchdog line (--kick gpio)
```

- **Board B (`hil/firmware/SafetyNode`):** the device under test. It keeps its **own clock** (`millis()` from the reset message) and runs its **own 10 ms task**. It is not slaved to the bench, so timing on hardware is real.
- **Board A (`hil/firmware/BusNode`):** the actuator-bus node. It ACKs and receives what B transmits, and forwards it to the PC. In `hil` mode the bench sees B's outputs **only through the bus**.
- **PC:** planner, vehicle model, the actuator ECU's own E2E check and fallback, fault injection, oracle. These are unchanged; only the adapter (`ssb/hil.py`) is new.
- **Calibration download:** the PC sends all 24 limits from the config at every reset (like an XCP calibration download), so firmware and bench can never disagree about a threshold.

### Honest deviations from the vehicle design

| Design | On this HiL | Why | How to close it |
|---|---|---|---|
| Planner command on **CAN FD** (16-byte frame) | Same 14 bytes over **USB**; E2E Profile 5 still checked on the MCU | The MCP2515 is classic CAN only (8 bytes max) | MCP2518FD module (CAN FD) on board A as the planner node |
| Supply brown-out | **Emulated** in firmware: the controller logic resets, the board stays powered | No switchable supply yet | Relay on B's 5 V (IMPROVEMENTS 6.5); persist "was running" in RTC memory so a real reboot latches SAFETY_RESET |
| Hardware watchdog line | Kicks over USB by default | Needs one jumper wire | Fit A GPIO5 → B GPIO5 and use `--kick gpio` |
| Actuator ECU | On the PC, fed from board A | Keeps the plant and its fallback in one place | A third board, or the actuator ECU logic on board A |

## 3. Wiring

Two identical nodes, each an ESP32-S3 dev board plus an MCP2515 + TJA1050 module:

| Item | Board A and board B (identical) |
|---|---|
| MCP2515 J4 (SPI) | INT → GPIO4, SCK → GPIO12, SI → GPIO11, SO → GPIO13, CS → GPIO10, GND, VCC → **5 V** |
| CAN (J2) | H ↔ H, L ↔ L, **GND ↔ GND** between the modules |
| Termination (J1) | **ON on both** modules (two 120 Ω ends → ~60 Ω across H–L, measure it with the power off) |
| Optional watchdog wire | A GPIO5 → B GPIO5 (B has a pull-down) |

## 4. Bring-up, step by step

**The full checklist, with a pass criterion and the usual failures for every step, is [HIL_BRINGUP.md](HIL_BRINGUP.md).** It starts with a dry run against emulated boards (`--port-b EMU`), so the PC side is proven before any wiring. The short version:

```bash
.venv\Scripts\python.exe -m ssb.hil --ports                               # which COM port is which (Espressif / CH34x / CP210x flagged)
.venv\Scripts\python.exe scripts/hil_flash.py --b COM14 --a COM13          # compile + upload both (ports: yours)
.venv\Scripts\python.exe -m ssb.hil --hello COM14 COM13                    # expect "SafetyNode 2.3 (B) CAN 8MHz" and "BusNode 2.3 (A) CAN 8MHz"
.venv\Scripts\python.exe run.py --dut pil --port-b COM14 --scenario command_link_lost        # one scenario, one board (~9 s)
.venv\Scripts\python.exe run.py --dut hil --port-b COM14 --port-a COM13 --scenario command_link_lost
.venv\Scripts\python.exe run.py --dut hil --port-b COM14 --port-a COM13 --b2b-dut            # all 52 in real time (~9 min) + back-to-back
```

| Step | Pass criterion | If it fails |
|---|---|---|
| hello | Both firmware strings, and "CAN 8MHz" (or 16MHz) | "CAN FAILED": SPI wiring or 5 V; run the lab's `LOOPBACK` self-test |
| PiL, 1 scenario | PASS, detection ~100 ms (one board: bus monitoring is off, as nobody ACKs) | No ack: wrong port, or the port open reset the board (wait 1 s, retry) |
| HiL, 1 scenario | Same result as PiL | PiL passes and HiL doesn't: CAN H/L/GND, termination, crystal |
| Full HiL + back-to-back | 51/52 + 1 known; differences only in timing, within 20 ms | See section 6 |

## 5. What the hardware run measures (that the PC can't)

- **Worst-case execution time** of one 10 ms safety cycle on the ESP32-S3, in µs (`D` diagnostics: `max_exec_us`). The S3's FPU is **single-precision**, so the core's `double` maths runs in software; this number is the honest cost of that choice.
- **Worst task lateness** (`max_late_ms`): how late the 10 ms task started.
- **CAN transmit failures** (`can_tx_fail`): all three MCP2515 transmit buffers busy, e.g. no ACK on the bus.
- **Detection timing with real transport**: `--repeat 5` gives min/typical/max per key scenario, and the back-to-back flags anything more than 20 ms away from the reference.

## 6. Manual physical fault tests (a hand-operated FIU)

Run a long nominal scenario in **PiL mode with both boards on the bus** (`--dut pil --bus-monitor --scenario nominal_with_noise`, so B's state is still visible over its USB while the bus is broken), then:

| Action during the run | Expected (to verify on the bench) | Why (CAN detail worth knowing) |
|---|---|---|
| Pull CAN_H | B: STOP_IN_LANE, cause ACT_BUS_OFF, within a few tens of ms (16 failed attempts to reach error passive, then the next 5 ms flag poll and 10 ms cycle) | With only two nodes, the missing ACK makes B's transmit error counter climb to **error passive** (128). By the ISO 11898-1 ACK-error exception it does **not** go bus-off. So the firmware treats transmit-error-passive as an actuator-bus fault, not only bus-off |
| Pull one termination jumper | Usually still works at 500 kbps over 20 cm | Short lab wires hide reflections; a vehicle harness would not |
| Swap H and L | Bus dead → same as pulling CAN_H | Differential polarity |
| Remove the common GND | Unreliable, sometimes works | Common-mode range of the TJA1050; this is why GND is wired |
| Reset board A | B detects the ACK loss; A's frames stop | Same mechanism as pulling CAN_H |

Each row is a candidate requirement for the bus-fault part of the safety concept. Today only the first is automated.

## 7. Results so far (2 Oct 2026, no boards connected)

| Check | Result |
|---|---|
| C++ core vs Python reference, exact back-to-back (state timeline to the ms + plant end state), default config | **52/52 identical** |
| Same, off-road config | **52/52 identical** |
| Board B's node logic over the link protocol (loopback), exact on the 10 ms status grid | **52/52 identical** |
| Matrix through native and loopback | 51/52 + 1 known finding, the same as the reference |
| 14 seeded mutants: same failing scenarios in C++ as in Python (default matrix) | **14/14 identical** (failing scenarios per mutant: 18, 1, 6, 2, 7, 2, 2, 1, 3, 1, 3, 1, 0, 0). The last two, `no_heading_hold` and `no_grade_compensation`, are caught only by the full mutation campaign (sweep / off-road), so for them this shows "same", not "caught". Logs: `reports/native_mutation_parity*` |
| Firmware build for ESP32-S3 (Arduino core 3.3.10, autowp-mcp2515 1.3.1) | SafetyNode 324 KiB flash (25%), 23 KiB RAM (7%); BusNode 315 KiB / 21 KiB; **0 warnings** in our code |
| Core C++ under `-Wall -Wextra -Wconversion` | 0 warnings |
| Unit tests | 23 pass (5 new: exact C++ parity, node parity, mutant parity, framing + resync, clear errors without ports) |
| **PiL / HiL on the boards** | **Not run yet: no ESP32 on a COM port today.** Everything up to the USB cable is proven |

## 8. Findings while building it

1. **The MCP2515 can't carry the planner's CAN FD frame.** The design and the lab disagreed; the HiL now says so openly, and the fix (an MCP2518FD) is named. Caught at design time by checking the lab's CAN controller against the DBC, not after wiring.
2. **A black box can only show a state change on its next status frame.** In `planner_looping` the watchdog reaction happens inside the kick handler at 2006 ms, between two cycles. The reference reports it at 2006 and the node at 2010, the next 10 ms status frame. It's not a bug (the actuators also see it at 2010), but an exact comparison has to know each DUT's reporting grid. Hence `back_to_back(quantum_ms=10)`.
3. **Porting for bit-exactness needs rules, not care.** Python's `round()` rounds halves to even (`nearbyint`, not `lround`); Python's `%` is never negative; `max(a, b)` returns `a` on ties; and the compiler must not fuse multiply-adds (`-ffp-contract=off`). With those four rules the C++ matched on the first full run.
4. **The ESP32-S3 has a single-precision FPU.** `double` keeps exact parity with the reference but runs in software. A production safety MCU would use `float` or fixed point; the PiL execution-time measurement shows the cost, and the back-to-back shows whether `float` would change any verdict.
5. **Opening a COM port can reset an ESP32** (DTR/RTS drive EN and BOOT through the auto-reset circuit). The link holds both low and waits for the firmware's hello before the reset message.
6. **One board alone can never get a CAN ACK.** My first firmware treated transmit-error-passive as an actuator-bus fault, which would have stopped the vehicle in every single-board PiL run. I found it on review, before any hardware run. Bus monitoring is now a reset option: on for `hil` (board A ACKs), off for `pil`, and on for `pil --bus-monitor` when A is on the bus for the wire-pull tests.

7. **(v2.3.1) A dry run against emulated boards found two real-time bugs before any hardware run.** (a) pyserial's `socket://` port reports `in_waiting` as 0 or 1 only, so the bench read one byte per millisecond and fell behind; real COM ports report the true count, but the link now reads everything available. (b) More important: the board judges a command's age against **its own clock**, so a PC bench that falls behind real time makes fresh commands look stale, and STALE_DATA stops appear that are really bench faults. The runner now measures its own lag on every real-time run, and the oracle adds a check, **"bench kept real time (max lag ≤ 5 ms)"**, so a slow PC can never be mistaken for a controller verdict. *Lesson: on a HiL, prove the bench keeps time before you trust a timing verdict about the DUT.*
8. **An emulator thread inside the bench ran late for the same reason a slow MCU would** (Python's GIL against the bench's 1 ms busy-wait). The emulator now runs in its own process (`ssb/hil_emu.py`), as a real board has its own CPU.

## 9. Next steps (in order)

1. Plug in both boards and run section 4 (≈ 30 min).
2. `--repeat 5` timing statistics on hardware; record the WCET.
3. Fit the watchdog wire (`--kick gpio`), so the hardware watchdog line is real.
4. Relay on B's 5 V plus RTC-memory latch, for a real brown-out.
5. MCP2518FD for the CAN FD planner link.
6. Build a **compiled C FMU** from the same core: it removes the PythonFMU limit of v2.2 and gives the supplier-style FMU test a real binary.

