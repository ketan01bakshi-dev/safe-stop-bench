# P5 chassis link: the contract (stage 1, v2.37; link version 2 with wheel encoders, v2.38)

**Status:** designed and tested without hardware (Python reference chassis, virtual clock and real UDP on the loopback). The chassis firmware of stage 2 is a
**port** of this contract and must pass the same files (`vehicle/link_cases.json`, `vehicle/link_vectors.json`). AI-assisted, synthetic data, home bench.

**What the link is for:** the laptop (the *bridge*) streams a recorded speed profile to the chassis as PWM setpoints, and the chassis stops safely whenever the
stream cannot be trusted. It carries commands, never a verdict, and nothing measured by the chassis feeds back into the bench's safety logic. It is **not
authenticated and not encrypted**: the run id and the nonce only guard against stale packets and a confused bridge. The chassis network must stay private
(a dedicated access point or an isolated segment), and no motion command ever travels over the bench's MQTT broker or over TCP.

## 1. Frame

`'S5'` | version (1) | type (1) | body | CRC-16/CCITT-FALSE (2), all big-endian, one datagram per message, CRC over everything before it (check value of `123456789` is `0x29B1`).

| Type | Direction | Body (bytes) | Sent |
|---|---|---|---|
| 1 HELLO | chassis to bridge | boot_id u32, build 8 ASCII, state u8, reason u8, reject u8, last_run_id u32 (19) | every 500 ms while DISARMED or LATCHED_STOP, at once after a refused ARM and after an end of run |
| 2 ARM | bridge to chassis | boot_id u32, run_id u32, nonce u32, pwm_cap u8 (13) | at once after a HELLO that passes the bridge's checks, repeated every 200 ms until the ECHO says ARMED |
| 3 SETPOINT | bridge to chassis | run_id u32, nonce u32, seq u32, t_ms u32, pwm_l i16, pwm_r i16 (20) | every 50 ms (20 Hz) |
| 4 STOP | bridge to chassis | run_id u32, nonce u32, reason u8 (9): 0 end of run, 1 abort, 2 watchdog | three times: one setpoint period apart at the end of a run, back to back on an abort |
| 5 ECHO | chassis to bridge | boot_id, run_id, nonce u32 each, state u8, reason u8, reject u8, seq u32, applied pwm_l i16, pwm_r i16, uptime_ms u32, rx_ok u16, rx_bad u16, then (version 2) enc_t_ms u32, enc_mask u8, enc_fl, enc_fr, enc_rl, enc_rr u32 each (52) | every 50 ms while ARMED, and at once after a state change |

**Version 2 (v2.38): wheel encoders in ECHO.** `enc_fl..enc_rr` are the cumulative rising edges of the four slot sensors (20 per wheel turn, about 10.2 mm of tyre),
u32 that wrap (the bridge takes differences modulo 2^32; the firmware extends the S3's 16-bit PCNT counters in software). `enc_t_ms` is the chassis uptime when they
were sampled, so speed comes from count differences over the chassis's own sample times, not over packet arrival. `enc_mask` says which sensors are fitted
(bit 0 FL, 1 FR, 2 RL, 3 RR; 15 = four, 3 = the front pair fallback); a wheel outside the mask reads 0, and mask 0 with zeros means no encoders or no sample yet.
The counts never change a state or a pin, and the bridge records them as a witness (`enc_mask`, `enc_last` in its summary; every ECHO in `bridge_log.jsonl`),
never as a verdict. The version byte moved from 1 to 2 for every type, so a stage 1 firmware and a version 2 bridge refuse each other's frames instead of
misreading a 31-byte ECHO.

A frame with the wrong length, magic, version, CRC or type is dropped and counted (`rx_bad`); every single-bit error is caught (tested over all golden frames).
`boot_id` is random at each chassis boot: an ARM that names another boot (a recorded ARM played back later) is refused. `build` lets the bridge refuse to command
firmware it does not know (the old ADAS sketch does not speak this protocol and is never commanded).

## 2. Chassis states

| State | Pins | Leaves it by |
|---|---|---|
| **DISARMED** (power-up) | zero | an ARM with the right `boot_id`, a run id **newer** than the last accepted one, and `1 <= pwm_cap <= hard cap` |
| **ARMED** | follow the last valid SETPOINT | STOP(end of run) to DISARMED; everything in the next table to LATCHED_STOP |
| **LATCHED_STOP** | zero | **the physical re-arm control only** (a button on the chassis, not a message), to DISARMED. ARM and SETPOINT are refused or ignored, STOP is idempotent |

What latches an ARMED chassis (`reason` in HELLO and ECHO), each covered by a conformance case and a virtual-clock test:

| Event | Reason | Note |
|---|---|---|
| more than **200 ms** since the last valid SETPOINT or the ARM | DEADMAN | a gap of exactly 200 ms is tolerated; the timer starts at the ARM, so a bridge that arms and never streams is stopped too |
| STOP(abort) / STOP(watchdog), or a STOP with an unknown reason | BRIDGE_ABORT / BRIDGE_WATCHDOG | |
| a frame from the armed peer with another run id / another nonce | BAD_RUN / BAD_NONCE | the run id is checked first |
| a SETPOINT outside `0..pwm_cap` (stage 1 drives forward only) | CMD_RANGE | latched, not clamped: clamping would hide a bridge bug |
| the Wi-Fi association drops | LINK_LOST | the firmware's WiFi event handler calls it |
| the armed bridge sends an ARM that contradicts its own | BAD_RUN | an identical repeated ARM is simply answered with an ECHO |

Ignored and counted, never a sign of life: a duplicate or reordered SETPOINT (`seq` not larger than the last, `rx_stale`), anything from another address while
ARMED or LATCHED (`rx_foreign`, even a perfect STOP), a second bridge's ARM (`reject` OTHER_PEER), any SETPOINT or STOP while DISARMED (`rx_ignored`).
Refused ARMs are answered with a HELLO whose `reject` says why: BOOT_ID, RUN_OLD, CAP, LATCHED.

## 3. Timing (the budget stage 2 measures)

| Parameter | Value | |
|---|---|---|
| setpoint and echo period | 50 ms (20 Hz) | |
| chassis dead-man | **> 200 ms** | four setpoint periods: three lost in a row (a 200 ms gap) is tolerated, four lost latches |
| single motor-pin writer | every 10 ms | a latch zeroes the target at once; the pins follow at the next writer tick |
| bridge watchdog | **> 100 ms** without an ECHO | aborts the take and sends STOP(watchdog) |
| ARM retry / timeouts | 200 ms; HELLO 5 s, ARM 1.5 s, end of run 1.5 s | any timeout aborts |

**Cut time**: from the arrival of the last valid SETPOINT, the chassis latches at 201 ms and the pins are zero within 10 ms more, so at most **211 ms plus the link delay**
(virtual-clock test `test_bridge_dies_without_a_stop_the_deadman_cuts_the_motors`: latch at exactly +201, pins within the writer period). That is the property of the
chassis firmware that stage 2 measures on an **independent clock** (logic analyser on the PWM pins, the stimulus edge on a second channel): the pass criterion is
dead-man + writer period + the stated link delay with margin, not "within exactly 210 ms". The coast distance after the pins are zero is added to the runway in stage 3.

## 4. The bridge

`vehicle/bridge.py`: waits for a HELLO; **refuses** to arm when the build is not on its list, the chassis is not DISARMED (a latched chassis is reported with its reason and
is not armed), the chassis has seen a run id that is not older than this run's, or the PWM cap is below the plan's peak. After the ECHO says ARMED it streams the plan
from plan time 0, never bursts to catch up after a late tick (`late_ticks` counts them), and ends the run with STOP(end of run), confirmed by a HELLO showing DISARMED.
Any abort (no HELLO, no ARM, the chassis latches or reboots mid-run, 100 ms without an ECHO, a timeout, an exception, Ctrl-C) sends STOP three times from a
`finally`, and the take is `ABORTED:<why>`, which is **INCONCLUSIVE, never a result**. It records every datagram with its own clock (`bridge_log.jsonl`) and a manifest
that labels the chassis (`virtual`, `fake-udp` or `real`). Witness statistics (echo gap, setpoint lag, applied-versus-sent PWM) are reported, not judged.
A non-loopback chassis needs `--allow-real` (wheels up, e-stop in reach, stage 2).

## 5. Decisions made beyond the P5 plan note

- `boot_id` in HELLO and ARM (a recorded ARM cannot arm a rebooted chassis), `last_run_id` in HELLO (the bridge refuses a run id that would be refused), `reject` in HELLO and ECHO.
- A planned end is **not** a latch: STOP(end of run) returns to DISARMED, otherwise every good take would need the button. Faults, aborts and watchdogs do latch.
- A latched chassis sends HELLO (2 Hz) instead of ECHO (20 Hz), so a new bridge sees why it is stopped; a running bridge learns from the immediate ECHO at the latch, or from its watchdog.
- PWM is validated against the cap given in ARM. The cap is chosen **per plan** (next multiple of 5 above its peak) because the seeded bugs change the peak: 203 for the
  correct build, 209 for `no_latch` after a 300 ms loss, **232 for `long_timeout`** after a permanent loss (it stops late and the planner keeps accelerating).

## 6. Real-time rehearsal on this laptop (one run, Windows, Python 3.14)

`python -m vehicle.bridge --fake --scenario p5_planned_stop_link_lost_300ms --variant no_latch`: COMPLETE; 280 setpoints, no late tick, no bad frame, the chassis ended
DISARMED. **Finding:** the fake chassis sent 225 echoes in 14 s (one per 62 ms, not 50) and the longest gap seen by the bridge was 64 ms, which fits Windows rounding a 1 ms
socket timeout up to its 15.6 ms timer tick (not isolated further). The 100 ms watchdog therefore has only about 36 ms of margin on this machine; a bridge on Windows needs a finer timer
(`timeBeginPeriod(1)`) or a busy-wait, which stage 2 decides after measuring on the real chassis (an ESP32 task has a 1 ms tick). The virtual-clock results do not depend on it.

## 7. What stage 2 must do with this

Port `ChassisCore` to the new sketch (one motor-pin writer, pull-downs on the six motor pins, hardware arm switch, e-stop, Wi-Fi modem sleep off), run `link_cases.json` and
`link_vectors.json` against it (an adapter with `rx`, `step`, `link_down`, `physical_rearm`, `set_encoders` (the PCNT sample) and read-only state, reason, reject, outputs and counters; the Python runner is
`vehicle/conformance.py`), then measure the cut time above. Still open: the off state (free-run or brake), reverse, the Wi-Fi topology, where the laptop's run id comes from after
a clock change, and the arm-switch GPIO. None of it changes a frame.
