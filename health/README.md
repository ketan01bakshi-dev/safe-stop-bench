# health/: is a failure about the product or about the bench? (phase P2, complete as of v2.23)

| File | What it does |
|---|---|
| `inventory.json` | The firmware and COM port each HiL board must have (SafetyNode 2.7 on B = COM13, BusNode 2.8 on A = COM14). Update it when you flash on purpose. |
| `check.py` | `bench_health()`: firmware vs inventory and bench-fault reruns -> a FAIL becomes **SUSPECT BENCH** in `python -m design` triage. |
| `preflight.py` | `python -m health.preflight` (quick: ports, USB hello vs inventory, UDS read, ~8 s) or `--full` (+ source vs board, compile both sketches, partition audit, esptool upload probe). `run.py --dut hil` runs the quick one first and **refuses to start** when it fails (`--skip-preflight` overrides; the manifest records it). |
| `uds.py` | The PC tester: sends UDS requests through board A's tester port and reads the answers back. |
| `error_catalogue.json` | Failure pattern -> cause -> fixes, seeded with the real bring-up traps (charge-only cable, worn jumpers, stale silent node, observer lost, CH340 overflow, download mode...). A failed check prints its fixes. |
| `dashboard.py` | `python -m health.dashboard` -> `reports/health/dashboard.html`: bench fitness, coverage per requirement and level, every non-PASS in one box (PRODUCT / TEST / BENCH / GAP), failure clusters. |

## UDS on the boards (firmware SafetyNode 2.7, BusNode 2.8)

`hil/SafeStopCore/src/ssc_uds.h`, ISO-TP single frames only, touches no safety state. Physical: B `0x7E2 -> 0x7EA`, A `0x7E3 -> 0x7EB`; functional `0x7DF`.
Services: `0x3E` TesterPresent (`80` = suppress response), `0x22 F195` software version (ASCII), `0x22 F18C` serial (low 32 bits of the chip MAC).
Negative responses `0x11 / 0x12 / 0x13 / 0x31`; a functional request that would get 0x11, 0x12 or 0x31 is not answered.
The PC reaches the bus through board A: a new `'T'` message makes A transmit one standard CAN frame (in A's CAN task, like all SPI traffic), and A forwards B's answer
as an `'M'` frame. A's own answer is mirrored to the PC too, because a node does not receive its own frames. The same header is compiled into `ssc_native.dll`, so the tests run the firmware's code.

Rollback: full flash images of both boards before the change are in `hil/build/rollback_2026-10-09/` (`esptool --chip esp32s3 --port COMx write-flash 0 <file>.bin`).
