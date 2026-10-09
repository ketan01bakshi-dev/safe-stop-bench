# OTA update, staged rollout and the release decision (v2.26-v2.27, integration plan P4)

**Status:** built and run. REL-01..10 pass on an emulated board and on the real board B (COM13), both over the direct USB
link and through an MQTT broker; REL-11/12 (power lost inside an update) pass on both; the two staged rollouts ran (one
good build over a mixed fleet including the real board, one bad build over 200 emulated boards); the release report lands
on GO WITH RISKS. Still not run: a physical supply cut (the relay is not fitted) and the board's own Wi-Fi radio. See
"What is not proven" at the end.

The point of this phase: a firmware change is not released because someone looked at a test report. It is released
because **the bench, the attack matrix, the OTA path and a staged rollout all left evidence, and a stated rule turned that
evidence into one verdict.**

## 1. The chain

```
bench campaigns      attack matrix        OTA matrix (REL-01..10)      staged rollout
(design/, run.py)    (security/)          emulated + real board        canary -> 25 % -> rest
        \                  \                      |                        /
         `------------------+----------------------+-----------------------'
                                   fleet/release.py
                     BLOCKED / NO GO / GO WITH RISKS / GO
```

`release.py` reads only report files (`reports/**/*.json`) and hashes every one of them into the report. It cannot change a
PASS or a FAIL, and it never calls a model. Same rule as everywhere else in this bench: **rules decide, LLMs draft, humans
approve.**

## 2. The image

A signed image is the app binary plus:

| Field | Over what |
|---|---|
| SHA-256 (32 bytes) | the app bytes |
| HMAC-SHA256 (32 bytes) | the hash, the size (u32 LE) and the version |

The **version is inside the signature**, so an old image cannot be re-labelled as a new one; the board also refuses a
version that is not newer than the one running (DOWNGRADE, REL-09). The key is a DEMO key compiled into both
`fleet/image.py` and `hil/firmware/SafetyNode/ssc_ota.h`; a product signs on a build server and keeps the verification key
in eFuse or a secure element.

## 3. The protocol (`fleet/ota_client.py` against `ssc_ota.h`)

Stop-and-wait over the existing USB link: the PC sends an op byte, the board answers with the same op and a status.

| Op | PC sends | The board |
|---|---|---|
| 0x01 BEGIN | size, sha256, hmac, version | refuses while a scenario is running (BUSY: only an idle ECU updates) and on a DOWNGRADE; erases the spare slot |
| 0x02 CHUNK | offset, up to 200 bytes | writes to the **spare** slot; the ack carries the bytes written so far, so a lost or repeated chunk is harmless — the client resumes at the board's own offset |
| 0x03 END | — | checks SHA-256 and the HMAC; only then switches the boot slot and records TRIAL in NVS |
| 0x04 REBOOT | — | only accepted after a successful END |
| 0x05 STATUS | — | state, tries, running slot, last result |

The running image is never touched (A/B slots: `ota_0` / `ota_1`, 1.25 MB each; the firmware uses about 26 % — headroom
found by the P2 partition audit). The checks do not depend on the transport, so a Wi-Fi / MQTT push reuses them unchanged.

**Trial boot.** The first boot of a new image is on probation: it commits itself after 3 s of healthy operation. A failed
health check rolls back at once; any reset before the commit counts as a second try and rolls back. After a rollback the old
image comes up in its **latched safe state**, like after any other reset (SR-17, NVS restore).

## 4. One emulation, two targets

`fleet/emulated.py` implements the same protocol and state machine in Python behind a serial-like interface
(`write()` takes the PC's bytes, `read()` returns the board's). The *same* `OtaClient` therefore drives an emulated board or
the real one, which is what makes a mixed fleet possible: N emulated vECUs plus the one real board under one rollout.
`tests/test_fleet.py` holds the emulation to the firmware (same checks, same statuses, same rollback rules);
**where the two differ, the real board wins.**

## 5. REL-01..10 (`python -m fleet.ota_matrix`)

The requirements live in `safety/hazards.json` (hazard H9, goal SG9) and are checked against the matrix by
`tests/test_fleet.py::TraceabilityTests`. One ordered sequence, because each step leaves the board in a known state for the
next; after every reboot or reset the hello must say "restored".

| ID | What it proves |
|---|---|
| REL-01 | bytes changed after signing: rejected on the hash, boot slot unchanged |
| REL-02 | signed with another key: rejected on the signature |
| REL-03 | one byte corrupted in transit: caught by the whole-image hash |
| REL-04 | reset at 1 / 50 / 99 % of the download: the old image still runs, in its safe state |
| REL-05 | lost and repeated chunks: resumed (about 1800 chunks, 3 resumes on the real board) and committed |
| REL-06 | an image that fails its health check: rolls back on its own |
| REL-07 | reset during the trial boot: back to the old image, in its safe state |
| REL-08 | reset between "staged" and "reboot": the new image still boots and commits |
| REL-09 | an older version: refused (anti-rollback) |
| REL-10 | every boot in the sequence came up with the safe state restored |

Results: **10/10 emulated, 10/10 on the real board** (`reports/fleet/ota_matrix_emulated.md`, `ota_matrix_real.md`).
Board B ends the REL-11 sequence on an OTA-delivered release image (2.50) and is then cable-flashed back to the 2.10 baseline; the preflight reads that version back **over UDS**
(`board_b: v2.50 serial 91F61B44`), not only from the firmware hello — so the release report's firmware line is diagnostic
evidence, not the image's own claim.

The real-board sequence needs four compiled images (2.12, an unhealthy 2.13, 2.14, 2.15), and `--cut-windows` seven more.
They are build artifacts, out of git: `--build-images` and `--build-cut-images` compile them with arduino-cli, and `--real`
refuses with that hint if any is missing. Rebuild them after any change to `ssc_ota.h`: an image carries the update code of
its own source, and the one that runs the next END is the one already on the board.

## 6. Staged rollout (`python -m fleet.rollouts`)

Waves: a canary (at least one device), then a quarter of the fleet, then the rest. After each wave, if more than `halt_pct`
of the devices touched **so far** failed (rolled back, rejected, or never confirmed), the rollout stops and the rest are
never touched. A device of another hardware revision, or one already on this version or newer, is **skipped and is not a
failure**.

| Run | Fleet | Outcome |
|---|---|---|
| good build 2.16 | 40 emulated rev-b + 2 rev-a (not targeted) + the real board at position 4 | 41 committed, 2 skipped, 0 rolled back, ran to the end |
| bad build (fails its health check) | 200 emulated, canary 2 % | all 4 canaries rolled back on their own; **halted after wave 1**, 196 boards never touched |

The bad-build run is the one that matters: the fleet protected itself from a build that fails its health check, without a
service visit and without a human in the loop.

## 7. Over MQTT (`fleet/mqtt_link.py`, v2.27)

`python -m fleet.ota_matrix --emulated --mqtt` and `--real COM13 --mqtt` run the same REL-01..10 with a local amqtt broker
between the client and the board:

```
OtaClient -> MqttLink --ssb/ota/<dev>/to_board--> broker --> Gateway -> board (emulated, or the real one on its USB port)
          <-          <--ssb/ota/<dev>/from_board--        <--         <-
```

`MqttLink` has the four calls the client uses on a serial link (write, read, reset_board, close), so nothing in the
protocol, the image checks or the scenarios changed. The `Gateway` stands where a Wi-Fi radio would stand. A second topic,
`ctl`, carries `reset`, `advance <s>` (emulated time) and `cut <n>`. Result: **10/10 on the emulated board and 10/10 on the
real board**, about 1800 chunks and 3 resumes through the broker, same as over the cable.

What it proves: the update survives a real broker, real MQTT framing and a real topic layout, with the real board behind
the gateway. What it does not: the board has no Wi-Fi radio or MQTT client, and adding them needs network credentials that
do not belong in this repo. The gateway is the PC doing that job.

## 8. Power lost inside an update: REL-11 / REL-12 (`--cut-windows`, v2.27)

A reset test (REL-04, REL-07) cannot see this: the end of an update is five separate persistent writes, and a power cut can
fall between any two of them.

| # | Write | What a cut right after it must leave |
|---|---|---|
| 1 | `prev` (slot to roll back to) | the old image runs; nothing on record |
| 2 | `tries` | same |
| 3 | `last` (text) | same |
| 4 | `st` = TRIAL (written **last** of the record) | the old image runs; boot sees TRIAL with the old slot still booted and cleans it up ("update not completed") |
| 5 | boot-slot switch | the new image boots on probation; an unhealthy one rolls back |

**The defect this found.** The first firmware did the switch first and the record second, and wrote the state flag before
the slot it points at:

- a cut after the switch, before any record: the new image boots with **no probation**, so an unhealthy image is never rolled back;
- a cut between the state flag and `prev`: TRIAL with `prev = 0`, which is no slot, so the rollback **switches nothing** and the
  unhealthy image keeps running, with the state even reading ROLLED_BACK.

Both are shown by the emulation of the first order (`tests/test_fleet.py::PowerCutWindowTests`: REL-11 must fail on it, or it
proves nothing). The fix, in `ssc_ota.h`: write the record first with the state flag last, switch the boot slot after it,
and let `boot_check()` discard a record whose slot never switched.

**How it is tested on the real board.** The relay is not fitted, so no supply is cut. Seven images are built from the
fixed source (`--build-cut-images`): `cut1`..`cut5` (`-DOTA_CUT_AT=n`, each resets the CPU once at the nth write of the
next update, then behaves normally) and an unhealthy image. For each n, `cutN` is installed, an **unhealthy** image is
pushed to it, and the cut falls inside that update. A reset at a write boundary leaves flash exactly as a cut there would;
it does not show a *torn* single write or the supply ramp.

| | REL-11: afterwards | REL-12: next update |
|---|---|---|
| Fixed firmware, real board and emulation | the unhealthy 2.40 never runs; the board reports its safe state restored; cut 4 reads "update not completed"; cut 5 reads ROLLED_BACK | goes through at every cut point, ends on a plain release image (2.50) |
| First order (emulation only) | cut 1 leaves the unhealthy image running | not reached |

## 9. The board's own Wi-Fi radio (`ssc_net.h`, v2.28)

Board B can now take the same OTA messages over your Wi-Fi: the board is the MQTT client, this PC runs a login-protected broker
on the LAN, and the PC-side client is unchanged (`ssb/ota/<id>/to_board` in, `from_board` out, the same topics the gateway used).
Set-up, credentials and troubleshooting: **`docs/WIFI_SETUP.md`**.

- **Credentials live in the board's flash, not in the firmware.** `scripts/provision_wifi.py` sends them over USB (`'N'` messages
  SET / CLEAR / INFO); no secret is compiled into the signed images that are copied to every board. INFO never returns a password.
  Provisioning is accepted over USB only, never over the network it configures.
- **The radio is a guest on a safety controller.** Nothing provisioned means it never starts. It is off while a scenario runs
  and returns 2 s after, it runs in its own tasks (Wi-Fi, esp-mqtt), and the 10 ms loop only pops a queue.
- **The broker** (`python -m fleet.broker login | serve`) takes a user and password you choose; the file holds an argon2 hash.
- **`python -m fleet.ota_matrix --real COM13 --wifi`** runs REL-01..10 with the update traffic on Wi-Fi. The EN-pin reset stays on
  the cable (test equipment). REL-07 is timed from the reboot command, because over Wi-Fi the trial hello arrives after the 3 s
  probation; its proof is unchanged: the second boot is the old image and the state says ROLLED_BACK.
  `--emulated --wifi` runs the same code path against a stand-in board with a login-protected broker (CI).
- **Cost:** the firmware grew from about 350 KB to about 1 MB (78 % of the 1.25 MB slot), so a push is about 5150 chunks.

Verified on the real board without a network: provisioning, INFO, a USB update that still **commits** with the radio retrying (the
health check needs free heap, and the network stack uses a lot), and three HiL scenarios unchanged. The run over a real Wi-Fi network
needs your credentials and is recorded as `ota_matrix_real_wifi` once you have run it.

## 10. Telemetry (`fleet/telemetry.py`)

Safety events as JSON on `ssb/fleet/<device>/<event>`: boot (version, whether the safe state was restored), staged /
commit / rollback, resets, and SAF_Status changes (state and cause) read from the frames board A mirrors — the status
frames are E2E-protected, so a stale copy is counted as a reject instead of being reported as a state change (SR-19).
Publishing is best effort: with no broker, or no `paho-mqtt`, events are kept in memory only and never block or fail a
test. `reports/fleet/fleet_events.jsonl` holds the events of the release runs.

## 11. The release decision (`python -m fleet.release`)

| Verdict | When |
|---|---|
| **BLOCKED** | evidence missing or unusable (no preflight, no OTA result, bench not fit): decide nothing, fix the evidence |
| **NO GO** | a product failure (a FAIL the known-good reference passes, not a seeded defect); an attack scenario that fails its safety expectation; a failing OTA scenario; a rollout that did not halt a bad build |
| **GO WITH RISKS** | nothing blocking, but open findings, suspect cases or documented residual risks remain — each listed with its next step |
| **GO** | nothing blocking and no open finding |

Both MQTT matrices and both power-cut-window results are read as optional evidence: when present, a failing row is a NO GO,
and the standing limit about the transport changes from "not exercised" to "no radio on the board".

Today: **GO WITH RISKS**, on three carried-over security findings (SC-63 `sec_fuzzy` stops later than its FTTI, SC-70
`sec_signal_ramp` is seen by no layer, SC-71 `sec_mimic` is a documented residual risk). The same open finding seen by both
attack matrices is listed once. The verdict is a recommendation: the owner of the release owns the risk, this report owns
the truth of the evidence.

## What is not proven

Listed in the report itself, under the standing limits of the evidence:

- until you run `--real --wifi` on your network, the update reaches the real board through a broker and a **PC gateway on its USB link**.
  Once you have, the link is the board's own radio, as **plain MQTT without TLS and one login shared by all boards**, on a LAN you trust
- the signing key is a **demo key** compiled into the image
- no supply is ever cut: a reset at the exact write boundaries of an update stands in for it (REL-11). A **torn single flash write**
  and the **supply ramp / brown-out** need the relay and 5 V supply (`docs/HIL_POWER_CUT.md`), which are not fitted
- the **health check** in the trial boot is minimal (CAN controller, settings store, heap)
- one bench: one real board plus emulated vECUs, synthetic traffic
