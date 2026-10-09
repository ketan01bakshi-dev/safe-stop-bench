# fleet/: release decision, OTA update and telemetry (phase P4)

The last step of the chain: a firmware change goes **bench → signed image → OTA → health check → rollback test → release
verdict**. Everything here reads the bench's own report files; nothing here can change a PASS or a FAIL.

| File | What it is |
|---|---|
| `image.py` | Builds and signs SafetyNode images: app bytes + SHA-256 + HMAC-SHA256 over hash, size and **version** (so an old image cannot be re-labelled as new). DEMO key, shared with `hil/firmware/SafetyNode/ssc_ota.h` |
| `ota_client.py` | The PC side: stop-and-wait BEGIN / CHUNK / END / REBOOT / STATUS over the bench's USB link, with injectable faults (corrupt byte, dropped chunk, wrong key, reset mid-download) |
| `emulated.py` | An emulated SafetyNode with the same protocol and state machine as the firmware, behind a serial-like interface, so the *same* client updates an emulated board or the real one. Held to the firmware by `tests/test_fleet.py`; where they differ, the board wins |
| `ota_matrix.py` | The release scenarios **REL-01..10** (`safety/hazards.json`, hazard H9 / goal SG9), as one ordered sequence. `--emulated` or `--real COM13` |
| `rollout.py` / `rollouts.py` | Staged rollout over a mixed fleet (canary → a quarter → the rest) with one halt rule; the two runs of the release: a good build and a build that fails its health check |
| `mqtt_link.py` | The same update through a local MQTT broker: `LocalBroker` (amqtt), `MqttLink` (what the client sees as a serial link) and `Gateway` (puts an emulated or the real board behind the broker) |
| `provision.py` / `broker.py` | v2.28: the PC side of the board's Wi-Fi set-up (`'N'` messages over USB) and the login-protected LAN broker. Scripts: `scripts/provision_wifi.py`, `python -m fleet.broker login \| serve \| addresses` |
| `telemetry.py` | Safety events as JSON (boot, staged / commit / rollback, resets, SAF_Status changes read off the bus) to `ssb/fleet/<device>/<event>`; best effort, never blocks a test |
| `release.py` | One verdict — **BLOCKED / NO GO / GO WITH RISKS / GO** — from preflight + attack matrices + OTA matrices + both rollouts + the design campaigns, with the rules stated and every input file hashed |

## Run it

```
python -m fleet.ota_matrix --emulated          no hardware needed
python -m fleet.ota_matrix --build-images      compile the four images --real needs (arduino-cli); they are not in git
python -m fleet.ota_matrix --real COM13        board B on SafetyNode 2.10, idle, images built
python -m fleet.ota_matrix --emulated --mqtt   the same through a local broker (add --mqtt to --real too); needs pip install -e ".[fleet]"
python -m fleet.ota_matrix --emulated --cut-windows           REL-11/12: power lost at each final write of an update
python -m fleet.ota_matrix --build-cut-images                 the seven images --real --cut-windows needs
python -m fleet.ota_matrix --real COM13 --wifi  REL-01..10 over the board's own Wi-Fi (docs/WIFI_SETUP.md: provision it first)
python -m fleet.rollouts --real COM13          good build over a mixed fleet; bad build over 200 emulated boards
python -m fleet.release                        -> reports/fleet/release_report.md / .json
```

## What it is not

The board has no Wi-Fi radio or MQTT client: the broker path ends in a PC gateway on its USB link. The signing key is a demo key
compiled into the firmware. No supply is ever cut: a reset at the exact write boundaries of an update stands in for it (REL-11);
a torn single write and the supply ramp need the relay in `docs/HIL_POWER_CUT.md`. The health check in the trial boot is minimal. All of this is listed in the release report itself, under the standing limits of the evidence.
