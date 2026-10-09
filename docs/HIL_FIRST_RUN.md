# First hardware run: PiL and HiL on two ESP32-S3 boards (v2.9, 6 Oct 2026)

The first time the C++ safety core ran on real hardware. Until now (v2.3 – v2.8) it was proven on the PC only. Synthetic scenarios, illustrative limits.

## Setup

| | Board B (device under test) | Board A (bus node) |
|---|---|---|
| Board | EdgeHex ESP32-S3 Pro (N16R8), CH340 USB-serial | same |
| Firmware | `SafetyNode 2.3 (B)`, 324 KiB, includes the v2.6 E2E fix | `BusNode 2.3 (A)`, 315 KiB |
| CAN | MCP2515 + TJA1050, 8 MHz crystal, 500 kbit/s | same |
| PC link | USB 921600 baud, framed + CRC-8 (COM13) | USB (COM14) |

Wiring as in `HIL_ESP32.md` §3: J4 SPI to GPIO 10/11/12/13 (CS/SI/SCK/SO), INT to GPIO4; J2 CAN_H/CAN_L/GND between the modules; termination on both (60 Ω measured with power off). Laptop on mains power.

## Results

| Run | Result |
|---|---|
| **PiL**, board B alone, 54 scenarios, real time | **53/54 + 1 known** (the one FAIL was the bench's lag guard, a 21 ms laptop stall; rerun alone: PASS, lag 4.4 ms) |
| PiL back-to-back vs the Python reference (B's 10 ms grid) | **54/54 match** |
| **HiL**, B's outputs read by A **through the real CAN bus**, 54 scenarios | **52/54 + 1 known**, 0 CAN transmit failures, bench lag ≤ 1.9 ms |
| HiL back-to-back | **54/54 match** |
| Worst-case execution time of one 10 ms safety cycle (B's counter, last scenario of a run) | **42 – 79 µs** (≈ 0.4 – 0.8% of the cycle), `double` maths in software on the S3's single-precision FPU |
| Cycle lateness | **0 – 1 ms** |
| Detection on hardware | lost link 96 – 99 ms, CRC corruption 50 ms, hung planner 49 ms, envelope 66 – 71 ms |

**Worst-case detection over the real bus** (`--repeat 5`, HiL, 30 runs, all PASS):

| Scenario | min / typ / max | FTTI | Worst margin |
|---|---|---|---|
| command_link_lost (timeout) | 100 / 101 / 102 ms | 250 ms | 148 ms |
| crc_corrupt (E2E) | 50 / 50 / 51 ms | 250 ms | 199 ms |
| planner_hang (watchdog) | 50 / 50 / 52 ms | 250 ms | 198 ms |
| qa_wrong_answer (Q&A watchdog) | 49 / 51 / 51 ms | 250 ms | 199 ms |
| steer_out_of_envelope | 69 / 70 / 72 ms | 200 ms | 128 ms |
| steering_stuck (actuator check) | 330 / 330 / 331 ms | 400 ms | **69 ms** (the tightest) |

Spread ≤ 3 ms across five real-time runs; B's worst cycle in this campaign 61 µs, lateness 0 ms, 0 CAN transmit failures.

## Findings

1. **Worn jumpers caused every hardware fault in bring-up.** Both MCP2515 modules first failed SPI. Board B: no response at all (INT and MISO low); a reseat fixed it. Board A: MISO held low, then mostly 0xFF; only **fresh** Dupont wires fixed it. A new probe sketch (`hil/firmware/SpiProbe`) separated the cases: raw register reads (CANSTAT 0x80, CANCTRL 0x87, write/read-back 5A/A5) at 0.1 / 1 / 10 MHz, plus a pull-up / pull-down test that tells a driven line from a floating one.
2. **A charge-only cable and a loose USB-C plug look the same as a dead board.** LEDs on, no USB device at all in Windows. Swapping cables at the laptop end, then at the board end, located it in four steps.
3. **A stale node is silent.** A run that ends abruptly leaves the SafetyNode "active", and it sends no hello until reset. The bench now pulses EN through RTS when no hello arrives (`SerialLink.reset_board`); tested by killing a run on purpose.
4. **The bench must call its own faults.** In the first full HiL run the CAN link between the modules dropped after ~1 minute. B logged 2000 transmit failures and a bus fault, but the bench reads B's state through A, saw nothing, and reported **48 false FAILs** ("still NORMAL"). Now: no status frame from the observer for 500 ms while B is powered → the run stops with **"observer lost"**. *Lesson: an observer that can die silently turns a bench fault into DUT failures.*
5. **Fixed (v2.9.2): the intermittent steering-slew violation was the bench's clock, not B.** `bv_steer_rate_10kmh_49dps` failed in about 1 HiL run in 4 with "output steering slew 72 °/s", and never on the PC. A probe (`scripts/slew_probe.py`) logged every status frame's arrival time. B's output was identical in every run: after the stop it re-centres the steering at 0.48° per 10 ms cycle (48 °/s, legal). The cause was the bench. It checked the slew on *its own* 1 ms clock, sampling every 20 ms. When B's frame for 5100 ms arrived at 5101 ms, the 5100 sample still held the value from 5090, and the next sample at 5120 saw three cycles of change in "20 ms": 1.44° / 0.02 s = **72 °/s**. That is one millisecond of USB/CAN arrival jitter turning into a false invariant violation. **Fix:** in HiL/PiL the slew is computed on B's own time base, the Profile 2 counter in each SAF_ActuatorCmd (one count per 10 ms cycle, unwrapped mod 16), from the steering value in that same frame, the one the actuator actually receives. Proof: unit tests reproduce the exact 72 °/s with simulated 1 ms jitter on the old check, show the new check is immune, and show it still catches a 3× slew. On the boards it went from 2 fails in 8 runs before to **0 in 12** after, with the same jitter pattern. *Lesson: when the observer sees the DUT through a link, judge timing on the DUT's clock, not on the arrival time.*
6. **The PC's timing is still the weakest link:** one 21 ms laptop stall in the PiL run (on mains). The lag guard caught it.

7. **A stale-frame "replay" (v2.9.3; source corrected in finding 8).** After a rewire, the CAN link first dropped out intermittently, then the first rerun of the full matrix gave 26/54. Every new failure had the same signature: B's state went from a latched stop back to NORMAL for 1 ms. A raw capture (`scripts/blip_probe.py`) showed the cause. Board B's MCP2515 occasionally puts an **old** SAF_Status back on the bus, byte-identical to a frame sent seconds or even a run earlier. That is about 20 extra frames a minute, and 174 in one full run. B's own USB copy of what it sent had exactly one status per actuator frame, and the extras matched none of them. A had just been reset when the replays continued, which pointed to B's transmit buffer. Finding 8 shows that was wrong: it was A re-reading a stale receive buffer. Switching the new firmware checks off on B, then on A, did not change it, so it is not code from this session. SAF_Status has no alive counter, so a black-box observer cannot tell a replay from a fresh frame. The E2E-protected actuator frame showed **0** replays. **Fix (bench):** a bus status frame counts only if B's USB copy shows it sent exactly those bytes in the last 20 ms. That copy always arrives 0–2 ms before the bus copy, so there is no added latency. Each copy vouches for one bus frame, and replays are counted and reported. **Also added (firmware 2.4):** both nodes check every 5 ms that the MCP2515 is still in normal mode, debounced over three reads, and re-initialise it if not. A reports its re-inits and single-read SPI glitches, and the bench treats an observer fault as a **bench fault**: it recovers the boards and reruns the scenario instead of aborting the campaign. *Lessons: an unprotected status message is not evidence. And the first, undebounced version of the mode check caused the outage it was meant to catch, because one glitched read triggered a blocking re-init.*

8. **The "replay" was not B, and not a jumper: an MCP2515 flag-read race on board A (v2.9.4).** The first question was which jumper on board B to replace. A stress sketch (`hil/firmware/SpiStress`, `scripts/spi_stress.py`) was built to tell the SPI lines apart: read-only errors point at SO, write-only at SI, one-bit shifts on both at SCK, spurious commands at CS, register resets at VCC/GND, and errors at 10 MHz only point at signal integrity. It also kept a never-requested "canary" frame in TXB2.
   - **B: 0 errors in ~7.6M SPI transactions** in loopback, and 0 in ~7.2M with B transmitting on the real bus. No canary ever appeared, and B's transmit error counter stayed at 0.
   - **The stale 0x201 kept arriving while B ran the stress sketch, which never loads a 0x201 at all.** So B could not be the source. The earlier inference ("A was reset, so A's buffers were clean") was wrong: an ESP32 reset leaves the MCP2515's receive-buffer contents in place.
   - **A: 0 errors in ~7.5M SPI transactions** too. So it is not A's wiring either.
   - **Cause:** at the instant a frame lands, A's MCP2515 sometimes returns its flag register **shifted by one bit**: `0x02` ("RXB1 full") for `0x01` ("RXB0 full"). It happens in READ STATUS (94 of 94 RXB1 claims unconfirmed) and in a plain CANINTF read alike (21 of 21 contradicted by an immediate re-read). The rate is about 1 per 1000–2000 frames, much the same at 10, 4 and 1 MHz. The library then read RXB1, which still held a frame from hours earlier, and A forwarded it. A wiring fault would corrupt static reads too and get worse with clock speed; this does neither.
   - **Fix, BusNode 2.5 + `mcp_init`:** rollover is off (RXB0CTRL.BUKT = 0), so with accept-all filters every frame goes to RXB0 and RXB1 is never used. A receives from RXB0 only, and an RXB1 flag is counted as a misread and re-read. Measured with sequence-stamped frames at ~1450 frames/s: **0 stale duplicates, 0 lost frames, 0 receive overflows** in 2 × 30 s, against 37–50 stale duplicates per 30 s with the library's `readMessage()`. 118–136 misreads were caught and handled.
   - *Lessons: rule out the hypothesis you like with a test that could prove it wrong. The "B transmit buffer" story fitted every observation until a test removed B as a possible source. And a status flag read at the moment it changes is not the same as a static register: stress tests must exercise the race, not only the wires.*

9. **The laptop's "stalls" were mostly the bench's own start-up, and the rest are now rerun (v2.9.5).** Every FAIL in three full runs was the lag guard. A probe (`scripts/lag_probe.py`) showed that in `replay_sample_log` the worst lag was **always at t = 0–1 ms**: the planner (loading the replay log), buses and vehicle model were built *after* the clock started. Worse, board B starts its own cycle when the bench resets it, and that reset came *before* the set-up, so the set-up time was also a head start for B that the lag guard never measured. **Fixes:** (1) all set-up first, then reset B, then start the clock; (2) no cyclic garbage collection inside the 1 ms loop on Windows either (collect between scenarios); (3) a run that still stalls is a **bench fault**: the scenario is rerun (up to 2 times) and the stall is recorded with its time, and a verdict is never taken from a stalled run. `replay_sample_log`, worst lag per run: median **2.8 → 0.8 ms**, max **6.7 → 1.6 ms** (8 and 12 runs). *Lesson: before blaming the host, look at where in the run the lag happens.*

10. **The status message now protects itself (v2.9.6).** Findings 7–8 were fixed at the receiver (A's firmware) and in the bench (B's USB copy vouches for each frame). The real gap was that SAF_Status had no alive counter, so *no* receiver could reject a stale copy. Now it carries **CRC-8 + an 8-bit alive counter** in the same 8 bytes (state, pull-over and cause packed into one byte). The receiver accepts a counter step of 1–2 (one lost frame), rejects repeats and jumps, and resynchronises after two frames in sequence, or at once if B's last status said OFF. A naive receiver lost one cycle after B's emulated power loss, and back-to-back caught it (2050 vs 2060 ms). **Proof on the boards:** the old, faulty receive firmware back on A and the bench's mirror filter switched off, so the E2E check was the only defence: the stale frames were rejected on their counter (6 in `nominal_with_noise`, 1 in `intermittent_crc`), all 3 scenarios passed, and there were 0 invariant violations. *A stale copy now passes only if its counter happens to fit: about 2 in 256.* *Since v2.10 the DBC and the CAN-process vECU use the same layout and the same receiver, so one message ID means one layout at every level.*

11. **A real reset of the safety controller, mid-scenario (v2.9.7).** Until now SG6 ("come back from a reset in a safe, latched state") was only *told* to B (`safety_brownout`: a control message, B's CPU kept running). Now `safety_hw_reset` (SC-41) pulls B's EN pin through RTS at t = 2000 ms, and B really reboots.
   - **What B needed:** a real ECU boots with its calibration, so B now keeps its last configuration in flash (written only when it changes) and boots straight into the latched STOP_IN_LANE with cause SAFETY_RESET, on its own clock, with its alive counter restarted at 0. Its hello says "restored".
   - **Measured (5 resets):** first status on the bus **185–187 ms** after EN; always STOP_IN_LANE / SAFETY_RESET. Every other DUT (reference, native, FMU, CAN process, ROS 2) emulates this as a power loss of `dut_hw.reset_boot_ms` = 186 ms.
   - **What the bench needed:** while B boots it sends nothing, so the observer reports OFF (the bench applied the reset) and the observer-loss guard waits (up to 3 s; a board that doesn't come back stays OFF, a DUT verdict). A status frame sent just *before* the reset arrived ~1 ms after it, and the observer took it as "back", costing one cycle (2 of 3 runs). Fix: no board boots in < 50 ms, so a status within 50 ms of the pulse is in-flight and dropped. After that, **5/5 identical**: NORMAL → OFF 2000 → STOP_IN_LANE 2187–2188 ms (reference 2186); the actuator ECU's own fallback brakes at ~2102 ms (detect 102 ms, FTTI 300).
   - **The test can fail:** B built without the restore (`-DSSC_NO_RESTORE`) reboots idle and stays OFF: **FAIL** on reaction and cause, with no bench fault raised; the vehicle still stops on the ECU fallback (defence in depth). The no-latch mutant fails the emulated version too.

12. **A's lost frames came from its own busy-waits, not the laptop (v2.9.8).** The v2.9.7 run counted 2 receive overflows on board A, written up at the time as "the stalled PC stopped draining A's USB port". That can't be right: A reaches the PC through a CH340 with no flow control, so a stalled PC never makes A wait. The real suspect was A's own loop: every watchdog kick is a 100 µs busy-wait pulse, and after a laptop stall the bench catches up with a burst of kicks, while rollover off (v2.9.4) leaves A one receive buffer for B's two back-to-back frames (~230 µs apart). **Measured** (`scripts/overflow_probe.py`, 60 s, bursts of 30 kicks every 497 ms so they drift through B's cycle): **40 overflows with bursts, 0 without** (a burst at a fixed 500 ms phase missed B's frame pair every time and showed 0, a trap in its own right). **Fix, BusNode 2.6:** CAN reception moved to its own FreeRTOS task on core 0, woken by the MCP2515's INT line (1 ms fallback), into a 256-frame RAM queue; `loop()` on core 1 only drains the queue to USB and pulses the watchdog line; all SPI is in the CAN task. Two bugs found on the way: ERRIF is set on *any* error-flag change, including back to "no errors", so clearing it only while the flags were non-zero could leave INT stuck low and the task polling every 1 ms (1 overflow per minute); and one overflow at A's boot, before the task ran, while B was already transmitting (now not counted: no scenario runs then). **Result: 0 overflows in 3 × 60 s including bursts; queue high-water mark 2, 0 drops.**

## Hardware watchdog line (`--kick gpio`, v2.9.1)

Before this run the planner's watchdog kicks reached B as messages over USB. Now one jumper, **A GPIO5 → B GPIO5** (plus the shared GND), carries them as real pulses. A gets a `K` frame from the PC each cycle and pulses the line. B's ISR timestamps the edge, and its windowed watchdog checks that the kick is neither late nor early. B reports the source as `kick_src: gpio`.

| Run (HiL, real CAN bus, GPIO kicks) | Result |
|---|---|
| 54 scenarios | **53/54 + 1 known**, 0 invariant failures, 0 observer losses |
| Back-to-back vs the Python reference | **54/54 match** |
| planner_hang / planner_power_dip (kicks stop) | WATCHDOG_LATE at **50 ms** |
| planner_looping (kicks too fast) | WATCHDOG_EARLY at **10 ms** |
| qa_wrong_answer | WATCHDOG_QA at 50 ms |
| B diagnostics | worst cycle 69 µs, lateness 0 ms, 0 CAN transmit failures, 0 link errors, no bus fault |

Watchdog detection is the same as with USB kicks (49 – 52 ms). The detection time is set by the window logic, not by the transport. The steering-slew invariant (finding 5) did not fire in this run, which is consistent with "intermittent". It was fixed in v2.9.2.

## After the fixes (v2.9.3, 7 Oct 2026)

| Run (HiL, real CAN bus, GPIO kicks, firmware 2.4) | Result |
|---|---|
| 54 scenarios | **52/54 + 1 known**, **0 invariant violations**, 0 bench faults |
| The one FAIL | the bench's own lag guard (laptop stall 8.7 ms > 5 ms) in `replay_sample_log`; rerun alone: PASS, lag 4.5 ms |
| Back-to-back vs the Python reference | **54/54 match** |
| Stale status replays filtered | 174 |
| Board A | 0 re-inits, 2 single-read SPI glitches, 0 receive overflows |
| Board B | worst cycle 61 µs, lateness 0 ms, 0 CAN transmit failures |

## After the receive fix (v2.9.4, 7 Oct 2026)

| Run (HiL, real CAN bus, GPIO kicks, SafetyNode 2.4 + BusNode 2.5) | Result |
|---|---|
| 54 scenarios | **51/54 + 1 known**, **0 invariant violations**, 0 bench faults, **0 replays filtered** (149 in the previous run) |
| The two FAILs | both the bench's lag guard: `nominal_with_noise` had a 96.9 ms laptop stall, longer than `max_age_ms` (60), so B rightly stopped for STALE_DATA; `replay_sample_log` 12.5 ms. Rerun alone: PASS (lag 1.1 ms; 4.5 and 4.1 ms) |
| Back-to-back vs the Python reference | **54/54 match** |
| Board A | 0 re-inits, 0 receive overflows, 2 single-read SPI glitches; flag misreads are caught and re-read |
| Board B | worst cycle 69 µs, lateness 0 ms, 0 CAN transmit failures |

The mirror filter needed one more fix. In one run it rejected 40 genuine frames in each of three scenarios, because B's USB port was read late for a while and B's copy arrived *after* the bus frame. An unvouched frame now waits up to 20 ms, in order, before it is called a replay.

## Stall fixes (v2.9.5, 7 Oct 2026)

| Two full HiL runs back to back (SafetyNode 2.4, BusNode 2.5) | Run 1 | Run 2 |
|---|---|---|
| 54 scenarios | **53/54 + 1 known, 0 FAIL** | **53/54 + 1 known, 0 FAIL** |
| Back-to-back | 54/54 | 54/54 |
| Invariant violations / replays filtered | 0 / 0 | 0 / 0 |
| Worst bench lag (max / median of scenarios) | 4.4 / 0.3 ms | 2.2 / 0.3 ms (after the rerun) |
| Bench faults rerun | none | 1: `clock_drift`, a 20.4 ms host stall at t = 9371 ms, PASS on rerun |

The first full matrix with no FAIL at all, twice in a row, without hand-picking reruns.

## Status E2E (v2.9.6, 7 Oct 2026)

| Full HiL (SafetyNode 2.5, BusNode 2.5) | Result |
|---|---|
| 54 scenarios | **53/54 + 1 known, 0 FAIL**, no reruns needed |
| Back-to-back | **54/54** |
| Invariant violations / replays filtered / status E2E rejects | 0 / 0 / 0 |
| Worst bench lag | 4.7 ms |
| Board B | worst cycle 63 µs, lateness 0 ms, 0 CAN transmit failures; firmware 325 KiB |

## Real reset (v2.9.7, 8 Oct 2026)

| Full HiL (SafetyNode 2.6, BusNode 2.5), 55 scenarios | Result |
|---|---|
| Matrix | **54/55 + 1 known, 0 FAIL** |
| Back-to-back | **55/55** |
| `safety_hw_reset` on the board | NORMAL → OFF 2000 → STOP_IN_LANE 2187 ms, SAFETY_RESET; ECU fallback 2102 ms |
| Invariant violations / replays / status E2E rejects | 0 / 0 / 0 |
| Host stalls rerun | 3 (113.5, 92.8 and 5.9 ms); verdicts from the clean attempts |
| Board A | 2 receive overflows; 0 re-inits. *Corrected in v2.9.8 (finding 12): not the PC, A's own busy-wait kick pulses; fixed with a RAM queue.* |

## RAM queue on board A (v2.9.8, 8 Oct 2026)

| Full HiL (SafetyNode 2.6, BusNode 2.6), 55 scenarios | Result |
|---|---|
| Matrix | **54/55 + 1 known, 0 FAIL** |
| Back-to-back | **55/55** (one 6.1 ms host stall rerun automatically) |
| Invariant violations / replays / status E2E rejects | 0 / 0 / 0 |
| Board A | **0 receive overflows**, queue high-water mark 2, 0 drops, 0 re-inits, 0 SPI glitches |
| Board B | worst cycle 80 µs, lateness 0 ms, 0 CAN transmit failures |

## Generated SR-01 cases on hardware (v2.15, 8 Oct 2026)

The SR-01 cases derived by `python -m design` (5 executable + 2 probes), run with `run.py --scenario-file` on both boards
(`--kick gpio`), as an engineering check before the gated campaign (`reports/design/2026-10-08_sr01_hil`, waiting at
`basis_review`).

| Run | Result |
|---|---|
| Reference firmware | **7/7 PASS** (probes included); detection 98 / 100 / 98 / 110 / 101 ms; worst cycle 82 µs; 0 status E2E rejects |
| Back-to-back vs the PC reference | **7/7 match** |
| Seeded `long_timeout` (sent in B's reset command, no separate build) | **5/5 executable cases FAIL**: detection 1000 / 1000 / 1000 / 1010 ms against FTTI 250; the 101 ms loss not detected. Same as the PC levels |

Hardware detects at 98 ms twice, i.e. before the 100 ms threshold: the timeout counts from the last good frame, which can be
up to one 20 ms period before the injection. That is why the derived lower bound is threshold − one period (80 ms), not 100.

**Board B "CAN FAILED - PiL only" (open, cause unknown).** Before the run, B booted with its MCP2515 init failing, three
times in a row (RTS resets). SpiProbe then read every register right at 0.1 / 1 / 10 MHz and showed all lines driven: not
the wiring. After reflashing SafetyNode 2.6, CAN came up at once and stayed up for the runs above. SafetyNode's init
(`ssc_board.h`) already retries SPI RESET + bitrate + normal mode 3 times per crystal setting, and all 6 attempts failed on
each of the three boots, while the probe sketch flashed right after read the chip correctly. Cause unknown. Next time, before
reflashing: log which step fails (reset, bitrate, or normal mode) and CANSTAT, and note whether a USB unplug alone clears it.

## Not yet done (needs parts)

A real power cut through a relay (prepared in v2.12: SC-42, BusNode 2.7, wiring and procedure in `HIL_POWER_CUT.md`; needs the relay, a 5 V supply and a data-only USB lead), CAN FD (MCP2518FD), an independent USB-CAN sniffer. See `IMPROVEMENTS.md` 6.5 and the hardware plan in the v2.8 notes.
