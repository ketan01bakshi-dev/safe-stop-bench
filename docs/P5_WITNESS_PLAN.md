# P5: the chassis as a physical witness, then sensors and AEB (design note, v2.37)

**Status:** stages 0 and 1 are done and tested (no hardware). Stages 2 to 4 are planned; wheel encoders are part of stages 2 and 3 (owner's decision, 13 Oct 2026). AI-assisted, synthetic data, illustrative limits, home
bench. The plan comes from a read-only review of the chassis project and the bench interfaces (12 Oct 2026), three independently designed approaches
that were judged on hardware reality, test validity and value, and a critic pass; the critic's findings are folded into this note.

## 1. What the chassis is today (from its firmware, `sketch_ADAS_OTA.ino`)

| Fact | Consequence for P5 |
|---|---|
| EdgeHex ESP32-S3 drives an L298N with open-loop PWM (5 kHz, 8 bit) on ENA=13/ENB=12 and IN1..IN4=14/11/10/9 | the pin range collides with the bench boards' CAN, watchdog and relay pins: the chassis needs its **own** board |
| No encoder, IMU, radar or CAN. Telemetry `spd` / `rpm` are computed from PWM (`PWM/255 x 150 rpm x 20.4 cm`, 0.51 m/s at full PWM) | no measured speed: a chassis result can only be a **witness**, never a measurement |
| Front ultrasonic and IR avoid sensors are compiled out; forward path distance is fixed at 400 cm | the chassis has **no forward obstacle sensing** today: no AEB is possible yet |
| It starts in AUTO forward cruise (PWM 210) at power-up; no heartbeat, command timeout or last will | the old firmware must **never** be powered with the motors live; P5 uses a new minimal sketch |
| Plain MQTT on 1883, credentials in the source | none of that goes into P5 or any public file |

### 1a. The chassis as built (owner's photos and answers, 13 Oct 2026)

Two-deck acrylic 4WD kit: four TT gear motors (about 65 mm wheels, left pair and right pair on the two L298N channels, skid steering), one L298N with heatsink,
a panel voltmeter, a rear rocker switch, a breadboard and Dupont jumpers on the top deck, no forward sensor mounted.

| Fact | Consequence |
|---|---|
| Pack: **3 x 18650** (about 12.6 V full, 11.1 V nominal, about 9 to 10 V empty); **motors rated 12 V** (owner's answer, 13 Oct 2026) | the plan's "12 V source" holds. After the L298N's drop of about 2 V the motors see at most about 10.6 V at PWM 255 on a full pack and about 7 to 8 V near empty: always **under** their rating, so full PWM is allowed and the stage 1 plan peaks (PWM 203 to 232) do not over-volt anything. The cap now comes from the runway and k, not from the motor. Consequences: (1) the motors never reach rated speed, and top speed falls with charge, so every take records the pack voltage at start and end and k is stated for a voltage band; (2) a 12 V motor at low duty has little torque, so breakaway PWM will be high and must be measured, not assumed; (3) the firmware's 150 rpm / 0.51 m/s full-PWM figure is unverified until the encoders measure it |
| Power path: pack, rocker, L298N; the **ESP32-S3 is fed from the L298N's on-board 5 V regulator**, and every sensor from the ESP32 | (1) the regulator (a 78M05 class part, 500 mA) sits at the edge of its module rating at 12.6 V and burns about 3 W at Wi-Fi peaks; motor starts sag the pack. A brownout reset is likely under load; the link contract handles it (a new boot id aborts the take, the chassis comes up DISARMED), but during a reset the motor pins float, hence the pull-downs. (2) **No separate motor cut is possible today**: anything in the L298N's supply also kills the board and its log |
| Rocker switch cuts everything; it is small and at the rear | it is the main switch, not an e-stop |
| All four wheels carry **slotted discs, 20 slots** (owner's answer), no sensors fitted | **Decided (13 Oct 2026): encoders move from stage 6 to stage 3**, so the floor calibration measures wheel speed instead of guessing it. See 1b |
| Jumpers on the motor pins, a loose resistor on the breadboard | a jumper that drops out leaves an L298N input floating: the six motor/enable wires go to a soldered or screw-terminal board before any floor run |

**Stage 2 hardware, revised:** an **inline fuse** right at the pack (an 18650 pack can drive a short on a breadboard far beyond the wiring); a **buck converter** pack-to-5 V
(at least 1 A) feeding the ESP32 and the sensors, with the L298N's 5 V jumper removed and its logic fed from the buck; a **latching e-stop in the L298N motor supply only**,
reachable from the operator's position, so the board stays up and logs the stop; the rocker stays as the main switch; pull-downs on the six motor pins; a block that lifts all
four wheels; four slot sensors (1b). Pack care: the holder probably has no protection board, so stop runs at about 3.3 V per cell (about 10 V
on the voltmeter). Owner's answers (13 Oct 2026): encoders move to stage 3, motors are rated 12 V, discs have 20 slots.

### 1b. Wheel encoders (stages 2 and 3)

| Item | Decision / number |
|---|---|
| Sensors | slot-type optical modules (LM393 class), **four** (one per wheel): the S3 has exactly four PCNT units, and four allow a front/rear slip check on a skid-steer chassis. Two (one per side) is the fallback if a unit is needed elsewhere |
| Resolution | 20 slots, about 65 mm wheel, about 204 mm per turn: **about 10.2 mm per slot** (about 5.1 mm counting both edges, but LM393 duty is uneven, so speed uses rising edges only) |
| Speed estimate | count over 100 ms windows (one count = about 0.10 m/s, coarse at chassis speeds of 0.2 to 0.5 m/s), plus edge-to-edge period for a finer reading above about 0.1 m/s; both are logged, the window count is the primary |
| Direction | none (a slot sensor cannot tell direction); acceptable while stage 1 drives forward only. Reverse needs the commanded sign as context, stated in the log |
| Wiring | power the modules from **3.3 V**, or level-shift: the S3 inputs are not 5 V tolerant and a 5 V-powered module pulls its output to 5 V. Short shielded or twisted leads away from the motor wires; PCNT glitch filter on |
| Pins | from the chassis board's free pins only: not IO9 to IO14 (motors), not IO19/20 (USB), not IO26 to IO37 (flash and octal PSRAM on the N16R8), not strapping pins 0, 3, 45, 46; checked against the EdgeHex pin table before wiring |
| Stage 2 check (wheels up) | one hand turn of each wheel = 20 counts; then a PWM step, wheels up: counts appear on all four, none while the pins are zero; the e-stop and dead-man cuts are also seen as the moment counts stop (coast in counts) |
| What it measures | **wheel** speed, not ground speed: skid steering and slip mean the floor distance still comes from video and tape; the encoder sets k, breakaway PWM, top speed per voltage band and coast-down |
| Claim level | a chassis calibration measurement. It sets the replay scale; it **never** enters a safety-layer verdict (section 4) |
| Link contract | **done (v2.38, link version 2):** ECHO carries the four cumulative counts (u32, wrapping), their sample time and the fitted-sensor mask; golden vectors, 3 new conformance cases (27 in all) and the fake chassis moved together. See `docs/P5_LINK_CONTRACT.md` section 1 |

## 2. Stage 0 (done): what the reference model shows, from its own traces

`vehicle/p5_scenarios.json` (run with `run.py --scenario-file`, so the matrix and CI baseline are untouched): cold start to 20 km/h, command stream lost
at 4.5 s, either for good (**P5-01**) or for 300 ms (**P5-01L**, a new scenario; SC-15 is 200 ms). `python vehicle/tools/make_timeline.py [--clip]`
draws the contrast (needs `pip install matplotlib`; the MP4 needs ffmpeg). `vehicle/p5_runs.py` derives every number below from the traces.

| Run | P5-01 (lost for good) | P5-01L (lost 300 ms) |
|---|---|---|
| correct build | stops at 7.38 s, **25.0 m** of model travel, latched, PASS | the same stop and latch, PASS |
| `no_latch` (seeded bug) | the same stop, PASS (nothing to release) | stops, **returns to NORMAL at 4.84 s and drives on**: 20 km/h and 69 m at 14 s, FAIL |
| `long_timeout` (seeded bug) | stops at 8.60 s, **34.1 m** (+9.1 m), FAIL | **never reacts** (a 300 ms loss does not trip a 1 s timeout), FAIL |

The planner overshoots its target (peak **23.8 km/h** at a 20 km/h target), so a chassis speed cap and runway must be sized on the **peak**, not
the target. At the placeholder scale k = 0.0612 (1.836 km/h of chassis speed at full PWM per 30 km/h of model speed) 25.0 m of model travel is
1.53 m of floor. **k is a placeholder** until stage 3 measures breakaway PWM, top speed and tether drag. The no_latch bug needs a fault that
ENDS to show, which is why P5-01L exists.

Reference matrix at this commit: 55 of 56 pass plus 1 known finding (46 hand-written + 10 generated scenarios).

## 3. Stages 1 to 4 (summary; every step keeps the findings of the review; stage 1 is done, see its result below)

1. **Replay driver, fake chassis, link contract (no hardware).** UDP over the chassis Wi-Fi, no motion over MQTT or TCP. Messages: HELLO (repeats,
   carries the build id), ARM(run id, nonce, PWM cap), SETPOINT(seq, t, pwmL, pwmR) at 20 Hz, STOP, ECHO at 20 Hz. Chassis state machine: DISARMED, ARMED,
   LATCHED_STOP; any dead-man trip (200 ms), Wi-Fi loss, bad nonce or old run id latches the stop, and leaving it needs a **physical** re-arm. The bridge is
   a separate process with its own 100 ms watchdog and sends STOP in a `finally`. The replay input is the trace (t, speed, lateral, state, x every 20 ms)
   **joined** with the result's state changes, events and the scenario's inject time: the trace alone has no cause or inject marks.
   **Done (v2.37), contract in `docs/P5_LINK_CONTRACT.md`:** `vehicle/link.py` (frames, CRC, golden vectors), `vehicle/fake_chassis.py` (the chassis state machine as a pure
   core plus a UDP wrapper), `vehicle/replay.py` (trace to 20 Hz plan, joined with the markers), `vehicle/bridge.py` (watchdog, STOP in a `finally`, manifest),
   `vehicle/simnet.py` (virtual clock), `vehicle/conformance.py` with 24 hand-written cases for the firmware port. 35 new tests. On the virtual clock all six
   (variant, scenario) replays complete; losing three setpoints in a row (200 ms) is tolerated and four latch; the pins are zero within 211 ms of the last valid
   setpoint (dead-man 201 ms + one 10 ms writer tick) plus link delay; a dead bridge, a stale run id, a wrong nonce, a Wi-Fi drop and a silent chassis each end the take as
   INCONCLUSIVE with the chassis latched. A real-time run over UDP loopback completed (one run; Windows timer granularity leaves the 100 ms watchdog about 36 ms of margin).
   The seeded bugs change the peak PWM the plan needs (203 correct build, 209 `no_latch` after a 300 ms loss, 232 `long_timeout` after a permanent loss), so each plan
   carries its own cap. Nothing here has touched a chassis.2. **Chassis bring-up, wheels up (hardware).** A **new** sketch with one motor-pin writer (a timer task; `loop()` only posts targets), pull-downs on the
   six motor pins, a hardware arm switch, a latching e-stop in the motor supply sized from the measured stall current, Wi-Fi modem sleep **off**,
   credentials in a git-ignored `secrets.h` (a `.gitignore` must exist in the chassis repo first). Motor cuts are timed on an **independent clock**
   (logic analyser on ENA/ENB with the stimulus edge on a second channel); the pass criterion is dead-man + writer period + stated link delay, with
   margin, not "within exactly 210 ms". Flash by USB with the motor supply disconnected; an identity read must not reset a board that runs the old build
   with 12 V present. Off state (free-run or brake) is chosen and the coast distance is added to the runway.
3. **Floor calibration with encoders, first floor replay.** A cheap go/no-go first, with the e-stop fitted: does the chassis move at the PWMs this
   profile needs? The sweep runs to PWM 255 (allowed: 12 V motors, section 1a) in steps, wheels on the floor, and the encoders log breakaway PWM, lowest
   moving PWM, wheel speed per step for each side, left/right mismatch and coast-down in counts, each at a recorded pack voltage. Video and tape give ground
   distance as the cross-check for slip. k, cap and runway are chosen from the data (a 15 km/h target saves only about 13 % of the model distance, so it is not a fix for a short floor).
4. **Filmed evidence set.** Pre-registered: 5 valid takes per main scenario, 2 per control, every attempt recorded. A take that the e-stop, the dead-man
   or the operator ended is INCONCLUSIVE inside the denominator. **Control takes** (`no_latch`, `long_timeout`) end by design at the runway limit; that is
   their expected outcome ("did not stop within runway") and a separate valid class, not INCONCLUSIVE. Counts are reported as counts (N valid of M attempted),
   never as a rate or reliability. Clips carry the standard labels; video, logs and manifest are scrubbed before anything is shared.

## 4. What may and may not be claimed

**Allowed:** after stage 0 the contrast above, with its level (reference model) stated; after stage 2 the chassis firmware's own cut timing on its own
dead-man and link statistics (a property of the chassis, **not** of the safety layer, measured on an independent clock); after stage 4 "recorded runs of the
safety logic were replayed on a small chassis at about 1:k scale, N valid of M attempted; motion is open-loop, calibrated by wheel encoders and video; no chassis measurement
enters any verdict". The wording names the level: "the reference model" for reference recordings, "the SafetyNode firmware, version and hash" only for HiL ones.

**Not allowed:** that the chassis "drove", "controlled" or was "commanded by" the SafetyNode, or that the plant was replaced (B's feedback stays on the
simulated model); any measured chassis speed, stopping distance or time as a result; any FTTI or timing claim for the actuation path of the safety layer;
chassis evidence for SR-06, SR-10, SR-11, lane, pull-over, grade or payload; results transferring to 30 to 40 km/h vehicles; planner validation.

## 5. AEB: the goal, and the gate on the word

AEB with real sensors (ultrasonic first, then camera, radar, lidar) is the stated goal of P5. The word **AEB** may be claimed only for a stop that the
safety layer decided from an obstacle input. Today there is no obstacle hazard or requirement, no obstacle message into the SafetyNode, no scenario or
oracle for it, and no validated forward sensor on the chassis, so the gate needs, in this order: (1) a hazard and requirement for obstacle braking in
`safety/hazards.json` (with FTTI and the stopping-distance logic); (2) an obstacle input to SafetyNode as a new CAN message, registered in
`config/can_ids.json` and the DBC; (3) a scenario and an oracle for it (and the design pipeline's drafter for the test cases); (4) a forward sensor on
the chassis, validated against a tape measure before it feeds anything; (5) only then the chassis run. The chassis's own 15 cm stop is not the
SafetyNode and must not be shown as AEB. Each added sensor is a separate increment with its own claim row.

## 6. Board B on the chassis

The chassis EdgeHex is a separate board (owner's answer, 12 Oct 2026); bench boards A and B stay on the bench. Using **B** on the chassis was asked about:

- **B alone cannot drive the chassis.** B is the safety controller: CAN and USB, no motor outputs. The L298N still needs a board that writes PWM and
  direction pins, and the chassis motor pins (IO9 to IO14) collide with B's own CAN, watchdog and relay pins (from the review; check against the pin
  tables before any wiring), so the same board cannot be both.
- **B physically on the chassis is possible but changes the validated bench.** It would need a CAN cable to board A (500 kbit/s, terminations), the
  watchdog-kick GPIO wire from A to B (otherwise USB kicks only), the USB tether to the laptop, and a supply that keeps the relay power-cut design valid.
  That is the CAN-native design, which scored lowest in the review: it also needs a third MCP2515 + TJA1050 module for the chassis controller (both
  owned modules are in use). The slot encoders of 1b give wheel speed, not direction or ground speed.
- **Recommendation:** keep B and A on the bench for stages 0 to 4 (the replay needs no B on the vehicle). Treat "B on the chassis" as the later,
  live-coupling variant (stage 5 and beyond, and the natural home of the AEB track): at that point buy the third CAN module, or a third EdgeHex so the
  bench stays intact, and re-run the bench acceptance before and after the move.

## 7. Open before stage 2

The revised stage 2 hardware of section 1a (fuse, buck, motor-only e-stop, wiring) and the four slot sensors of 1b; Wi-Fi topology (shared home router or a dedicated access point; the P5 link does not use the bench's TLS broker, but the laptop's address must not move
under the bench's pinned certificates); the motor stall current at full charge; the arm-switch GPIO (not an octal-PSRAM pin on the N16R8 board);
how the ADAS firmware, dashboard and traffic-light demo are restored on the chassis board (dual OTA slot or a dedicated P5 board); a USB-port plan
(A, B, chassis, logic analyser: the laptop has two ports); a bill of materials (now including four LM393 slot modules) with price and delivery for every part; counts of tests and scenarios
reconciled from a fresh run once the P5 tests exist (this note quotes only the run above).
