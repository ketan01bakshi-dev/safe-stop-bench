# ESP32-S3 HiL bring-up: step by step

From "two boards in a drawer" to "52 scenarios through a real CAN bus", in about **90 minutes**. Each step has a pass criterion. Don't start a step until the one before it passes: then you always know which step a problem belongs to.

All commands run from the bench folder:

```bash
cd safe-stop-bench
```

## What you need

| Item | Notes |
|---|---|
| 2 × ESP32-S3 dev boards (EdgeHex ESP32-S3 Pro, N16R8) | One is **board B** (safety controller, `SafetyNode`), the other **board A** (bus node, `BusNode`). Label them with tape |
| 2 × MCP2515 + TJA1050 CAN modules (8 MHz crystal) | One per board, wired as in the table in step 1 |
| Dupont jumpers | 7 per module (SPI + power), plus 3 between the modules (CAN H, CAN L, GND) |
| 2 × **USB data** cables | Many USB-C cables only charge. If no COM port appears, try another cable first |
| Multimeter | For the termination check (step 1) |
| Optional: 1 jumper wire | Board A GPIO5 → board B GPIO5: makes the hardware watchdog line real (step 9) |

Software: arduino-cli with the esp32 core (3.3.x) and the autowp-mcp2515 library; Python packages from `pip install -e .[hil,can,fmu]` (pyserial, ziglang, python-can, cantools, fmpy, pythonfmu).

## Step −1: prepare the PC (2 min)

A HiL run is only as trustworthy as the bench's timing: it sends a frame every millisecond to a board that keeps its **own** clock. If the PC stalls, the board sees late commands and reacts to the **bench's** fault. The oracle checks this on every real-time run ("bench kept real time, max lag ≤ 5 ms").

- **Plug in the charger.** On battery, Windows holds the CPU at its base clock. Measured on this laptop (2026-10-03): stalls of 20–70 ms on battery, ≤ 2.6 ms the evening before on mains.
- **Power mode: Best performance** (Settings → System → Power).
- **Close Chrome, IDEs and other heavy apps** for the 20 minutes of the run.
- The bench raises its own process priority and timer resolution automatically (`ssb/rt.py`).

## Step 0: dry run with emulated boards (5 min, no hardware)

Proves the PC side is healthy before you touch a wire.

```bash
.venv\Scripts\python.exe run.py --dut pil --port-b EMU --scenario command_link_lost --scenario planner_hang --out reports/emu
```

**Pass:** 2/2 PASS, detection ≈ 100 ms and ≈ 50 ms. The emulator runs board B's exact firmware logic in a separate process, on its own clock.

## Step 1: wiring, power OFF (15 min)

Per module (MCP2515 header J4 → ESP32-S3):

| MCP2515 pin | ESP32-S3 |
|---|---|
| INT | GPIO 4 |
| SCK | GPIO 12 |
| SI (MOSI) | GPIO 11 |
| SO (MISO) | GPIO 13 |
| CS | GPIO 10 |
| GND | GND |
| VCC | **5 V** (not 3.3 V) |

Between the two modules (terminal J2): **H ↔ H, L ↔ L, GND ↔ GND**. Termination jumper **J1 ON on both** modules.

**Pass:** with both boards unpowered, the multimeter reads **≈ 60 Ω between CAN H and CAN L** (two 120 Ω ends in parallel). About 120 Ω means one terminator is off; open circuit means a broken wire.

## Step 2: find each board's COM port (5 min)

Plug in **one board at a time**:

```bash
.venv\Scripts\python.exe -m ssb.hil --identify B
```
```bash
.venv\Scripts\python.exe -m ssb.hil --identify A
```

**Pass:** each prints `board B = COMx` and `board A = COMy`. Write them on the labels. (The number follows the USB socket.)

**If nothing appears:**
- Try another cable (it may be charge-only).
- Try the board's other USB socket (use the **UART / COM** one, not the native USB one).
- Look in Device Manager → Ports (COM & LPT). A yellow mark means a missing CH343 / CP210x driver.

## Step 3: compile and flash (10 min)

Close anything holding the ports first: Arduino IDE serial monitor.

```bash
.venv\Scripts\python.exe scripts/hil_flash.py --b COMx --a COMy
```

**Pass:** "uploaded SafetyNode to COMx" and "uploaded BusNode to COMy".

**If the upload fails with "Failed to connect":** hold **BOOT**, tap **RESET (EN)**, release BOOT, then run the command again.

## Step 4: hello check (2 min)

```bash
.venv\Scripts\python.exe -m ssb.hil --hello COMx COMy
```

**Pass:**
- COMx prints `SafetyNode 2.3 (B) CAN 8MHz`.
- COMy prints `BusNode 2.3 (A) CAN 8MHz`. 16MHz is fine too: it's whichever crystal your module has.

**If it fails:**
- **"CAN FAILED"**: SPI wiring or 5 V. Check the SPI wiring and the module's crystal.
- **"no hello in 3 s"**: the wrong port, the board still booting (wait 2 s and retry), or another program holding the port.

## Step 5: PiL, one board in the loop (5 min)

Board B alone, its outputs read back over its own USB:

```bash
.venv\Scripts\python.exe run.py --dut pil --port-b COMx --scenario command_link_lost --out reports/pil
```

**Pass:**
- PASS, with a detection time of about 100 ms.
- The printed "board B diagnostics" show `max_late_ms` ≤ 1 and `rx_overflow` = 0.
- Note `max_exec_us`: it's the first real measurement of one 10 ms safety cycle on the ESP32-S3.

**If the bench reports a lag:** the oracle adds a check, "bench kept real time (max lag ≤ 5 ms)". If that fails, the **PC** fell behind, not the board: close other programs and rerun.

## Step 6: HiL, outputs through the real CAN bus (5 min)

```bash
.venv\Scripts\python.exe run.py --dut hil --port-b COMx --port-a COMy --scenario command_link_lost --scenario planner_hang --out reports/hil
```

**Pass:** the same result as PiL. Now the bench sees board B **only through board A, over the bus**.

**If PiL passes but HiL doesn't:**
- the CAN H, L or GND wires;
- J1 termination (back to step 1);
- the crystal reported in step 4.

## Step 7: the full matrix and back-to-back (≈ 20 min, real time)

```bash
.venv\Scripts\python.exe run.py --dut hil --port-b COMx --port-a COMy --b2b-dut --out reports/hil
```

**Pass:**
- **51/52 plus 1 known finding**, the same as the reference on the PC.
- Back-to-back differences only in timing, within 20 ms.

Anything else is a finding. Triage it in this order: bench lag → transport → board timing. The C++ logic itself is already proven identical on the PC.

## Step 8: timing statistics (10 min)

```bash
.venv\Scripts\python.exe run.py --dut hil --port-b COMx --port-a COMy --repeat 5 --scenario command_link_lost --scenario planner_hang --scenario crc_corrupt --out reports/hil
```

**Pass:** you get min / typical / max detection times and the worst-case margin against each FTTI. These are the hardware numbers worth quoting, once measured.

## Step 9 (optional): the real hardware watchdog line (5 min)

Fit one jumper wire: **board A GPIO5 → board B GPIO5** (the boards already share GND through the CAN wiring). Then:

```bash
.venv\Scripts\python.exe run.py --dut hil --port-b COMx --port-a COMy --kick gpio --scenario planner_hang --scenario planner_looping --out reports/hil
```

**Pass:** both PASS. Watchdog kicks now arrive as real edges on a pin, so the planner-hang and looping cases test a real watchdog line.

## Step 10 (optional): manual fault injection with your hands (10 min)

Start a long nominal run in **PiL mode with bus monitoring on**, so you still see board B over USB while the bus is broken:

```bash
.venv\Scripts\python.exe run.py --dut pil --port-b COMx --bus-monitor --scenario nominal_with_noise --out reports/fiu
```

During the run, pull the CAN H jumper. **Expected (to verify):** board B goes to STOP_IN_LANE with cause ACT_BUS_OFF within a few tens of ms. With only two nodes the transmitter reaches error passive, not bus-off (the ISO 11898-1 ACK-error exception), and the firmware counts that as a bus fault. The other manual tests, with their expected outcomes, are in `HIL_ESP32.md` §6.

## After the run

Keep:
- the summary line of steps 5–8;
- the `board B diagnostics` line (`max_exec_us`, `max_late_ms`);
- `reports/hil/report_hil.html`.

Record the measured hardware numbers in `docs/HIL_ESP32.md` §7. Until then, the HiL is "proven on the PC only".
