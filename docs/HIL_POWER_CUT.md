# Power cut of the safety controller (v2.12: prepared, relay not fitted yet)

**Status:** everything except the relay is built and tested: scenario SC-42 `safety_power_cut`, board A's relay command
(BusNode 2.7, compiled, not flashed), the bench's handling of B's vanishing USB port, the emulated version on every other
level, and a probe that measures the real thing. What is still **assumed** until measured is marked ⚠️ below.

## Why a power cut is not a reset

The v2.9.7 reset test (SC-41) pulls board B's EN pin: the CPU restarts, but the board stays powered. A supply loss
takes more with it:

| | Reset (EN pin, SC-41) | Power cut (relay, SC-42) |
|---|---|---|
| CPU | restarts | off, then cold boot |
| CAN transceiver (TJA1050) and MCP2515 | stay powered | off: B is gone from the bus |
| USB-serial chip (CH340) | stays powered | off: B's COM port vanishes (with VBUS blocked, below) |
| Flash (NVS) write in progress | completes | can be interrupted (the configuration is written only when it changes, so rarely) |
| Supply ramp, brown-out detector | no | yes: boot time may differ ⚠️ |

The requirement is the same (SR-17, SG6): come back in the latched safe state, STOP_IN_LANE with cause SAFETY_RESET,
while the actuator ECU brakes on its own fallback in the meantime.

## Parts

| Part | Notes |
|---|---|
| 1-channel 5 V relay module with optocoupler and a **high/low trigger jumper** | Set to **high-level trigger**. ⚠️ Check it switches reliably from a 3.3 V GPIO (most optocoupler modules do; some need 5 V on IN). |
| 5 V supply for board B, ≥ 1 A | A USB charger with a cut-open cable or a bench supply. B draws a few hundred mA ⚠️ (ESP32-S3 without Wi-Fi + MCP2515 module). |
| USB "data-only" lead or VBUS blocker for board B | **Essential.** On these boards USB VBUS feeds the 5 V rail, so with a normal lead the PC keeps B powered and the relay cuts nothing. |
| 3 jumper wires | A GPIO6 → relay IN, A 5 V → relay VCC, A GND → relay GND |

## Wiring

```text
             board A (BusNode 2.7, USB to the PC as now)
             GPIO6 ───────────► relay IN   (high = relay on)
             5V    ───────────► relay VCC
             GND   ───────────► relay GND

5 V supply + ───► relay COM          relay NC ───► board B 5V pin
5 V supply − ─────────────────────────────────────► board B GND  (A and B already share GND through the CAN wiring)

board B USB ──[VBUS blocked: data only]──► PC   (B's COM port: data only, no power)
CAN H / L / GND between the modules: unchanged.  Watchdog wire A GPIO5 → B GPIO5: unchanged.
```

**Why normally closed (NC) and high-level trigger:** GPIO6 is low while board A boots, resets or hangs, so the relay is
off and B stays powered. B loses power only while A actively holds the pin high, and A releases it on its own after
the requested time (at most 60 s). A crash on A can't leave B unpowered. GPIO6 is not an ESP32-S3 strapping pin.

## Procedure on the day the relay arrives

1. **Flash board A only** (B keeps SafetyNode 2.6, which already restores its configuration from NVS):
   `python scripts/hil_flash.py --a COM14`. A's hello must say `BusNode 2.7`.
2. **Wire as above, B on the external supply, USB data-only.** Check B's hello still arrives over USB.
3. **Probe 5 cuts:** `python scripts/power_cut_probe.py --port-b COM13 --port-a COM14 --cuts 5 --ms 300`.
   Pass criteria for the wiring itself:
   - B's port **vanished during each cut** (`True`). If it says `False`, USB is still powering B: the cut is not real.
   - B's first status after power returns is STOP_IN_LANE / SAFETY_RESET, counter 0.
   - Board A's diagnostics show no error flags or re-inits during the cut (an unpowered TJA1050 should leave the bus
     passive ⚠️ to confirm here).
4. **Record the median** "first status after power returns" in `config/default.json` → `dut_hw.power_on_boot_ms`
   (currently **186, assumed** equal to the reset boot).
5. **Run the scenario on the boards:** `python run.py --dut hil --port-b COM13 --port-a COM14 --kick gpio --relay
   --scenario safety_power_cut --b2b-dut`, then the whole matrix with `--relay`.
6. **Negative test:** flash B with `-DSSC_NO_RESTORE` (the v2.9.7 variant). SC-42 must **FAIL** (B comes back idle, OFF),
   with no bench fault; reflash the normal SafetyNode afterwards.

## Expected results (written before measuring)

| Item | Expected | Basis |
|---|---|---|
| Observer state | NORMAL → OFF at 2000 ms → STOP_IN_LANE at about 2000 + 300 + boot ms | the reference model: 2486 ms with boot 186 |
| Cause | SAFETY_RESET, latched until an operator release | SR-17 |
| Actuator ECU | brakes on its own fallback about 100 ms after B's frames stop | the ECU's timeout (as in SC-41) |
| Boot after power-on | ≈ the reset boot (185–187 ms) | ⚠️ assumed; supply ramp and brown-out may add time |
| B's COM port | gone during the cut, back before the next scenario (the bench waits up to 15 s) | CH340 powered from B's rail |
| Bus during the cut | A sees no errors; B simply absent | ⚠️ TJA1050 unpowered behaviour, to confirm |
| Back-to-back vs reference | same timeline on the 10 ms status grid, within the measured boot | as SC-41 on the boards |

## What the bench does (already built and tested)

- **Runner:** `safety_power_cut` faults open the relay on a board that has one (`dut.can_power_cut`: emulated boards, or
  `--relay` / `dut_hw.relay_fitted`); every other DUT gets a power loss of the cut plus `power_on_boot_ms`.
- **LinkDUT:** `power_cut(t, ms)` sends `'X' <ms>` to board A; B is reported OFF while down and booting; B's USB port is
  not used (B is observed only through A on the bus, as always); the alive-counter receiver restarts; before the next
  scenario the bench waits for B's port to reappear and reopens it (`BenchFault` if it doesn't within 15 s).
- **BusNode 2.7:** `'X'` drives GPIO6 high for the given time, non-blocking; low at boot.
- **Emulator:** emulated A handles `'X'`: B goes silent, its link drops, and after the cut plus the boot a fresh node
  boots from the configuration B had stored (the host build's new `ssc_node_config` / `ssc_node_restore`).
- **Tests:** the emulated cut comes back latched on the reference (2486 ms); a no-latch DUT fails; the host build boots
  latched from a stored configuration; the whole relay path on the emulated boards, including reopening B's port for
  the next scenario. SC-42 matches the reference exactly for the firmware C++, the FMU and CAN lockstep.

## Risks to watch on the bench

- **Contact bounce** at release can give B several short supply pulses: it may boot twice, which shows up as a longer
  boot in the probe.
- **Inrush** when the supply returns: a weak supply can brown out again; use ≥ 1 A.
- **Never put board A's supply through the relay:** A drives the relay and is the observer.
- **A partly powered B** (USB still feeding VBUS) passes every check except "port vanished": the probe reports it.
