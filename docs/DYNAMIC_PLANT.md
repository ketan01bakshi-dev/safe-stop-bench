# Dynamic plant: mass, payload, grade (v2.5, 5 Oct 2026)

Closes items **1.6 (pluggable plant)** and **3.5 (mass / load)** of `docs/CHANGES_V2.md`. Synthetic, illustrative parameters, not a calibrated vehicle.

## What changed

- **`ssb/plant.py` → `DynamicVehicle`**, chosen with `plant.model = "dynamic"` (`run.py --plant dynamic`). The kinematic model stays the default, so every earlier result is unchanged.
  - Longitudinal: `m_eff · a = F_x − C_rr·N − ½ρ·C_dA·v² − m·g·sin θ`, where `N = m·g·cos θ` and `m = curb + payload`.
  - **The brake and drive ECUs turn an acceleration request into a force calibrated on the curb mass.** This is an open-loop pressure/torque map, the common case without a deceleration-closed-loop brake controller. So a payload gives proportionally less deceleration for the same request.
  - Tyre limit `|F_x| ≤ μ·N` uses the loaded normal force. The true slope angle replaces the small-angle approximation, and the IMU's `grade_accel` is computed the same way.
  - Lateral motion stays the kinematic bicycle (low-speed ODD). Actuator lag and dead time are unchanged (`Actuators`).
- **Config:** `plant.dynamic` in `config/default.json`: curb mass 1500 kg, payload, rolling resistance 0.015, C_dA 0.7 m², rotating inertia +4%.
- **Scenarios:** an optional `payload_kg` per scenario. **CLI:** `--plant dynamic`, `--payload KG`, `--load-sweep` (a payload × grade table in the report).

```
.venv\Scripts\python.exe run.py --plant dynamic                  # matrix on the dynamic plant: 51/52 + 1 known, as kinematic
.venv\Scripts\python.exe run.py --plant dynamic --payload 600
.venv\Scripts\python.exe run.py --plant dynamic --load-sweep     # → reports/report_dynamic.html, "Load sweep" section
```

## Results

The matrix on the dynamic plant with no payload gives **51/52 + 1 known**, the same verdicts as the kinematic plant.

The load sweep below uses 15 s runs, 30 km/h, μ 0.8. "Planned stop" = `command_link_lost` (MRM 3 m/s² request); "weak brake" = `brake_weak` (30% brake).

| Payload | Mass ratio | Planned stop, flat | Planned stop, −6% | Weak brake, flat | Weak brake, −6% |
|---|---|---|---|---|---|
| 0 kg | 1.00 | 15.4 m | 20.2 m | 12.6 m | 19.2 m |
| 300 kg | 0.83 | 17.4 m | 24.3 m | 14.7 m | 23.8 m |
| 600 kg | 0.71 | 19.4 m | 28.8 m | 16.7 m | 28.6 m |
| 800 kg | 0.65 | 20.6 m | 32.0 m | 18.1 m | 32.1 m |
| 900 kg | 0.62 | 15.6 m ⚠ backup brake | 33.6 m | 18.7 m | 33.9 m |
| 1500 kg | 0.50 | 18.0 m ⚠ backup brake | 26.4 m ⚠ backup brake | 22.5 m | 45.8 m |

## Findings

1. **Load lengthens a planned stop, and nothing in the safety layer notices.** +800 kg makes the flat stop 34% longer and the −6% stop 58% longer, with no diagnosis at all. The brake-plausibility check (SR-11: achieved < 50% of demand for 400 ms) is a fault detector, not a performance monitor, and there is **no stopping-distance requirement** for the planned stop. *Gap: add a stopping-distance budget per ODD (speed, grade, max payload) to the hazard analysis, and a load-aware check or mass estimate if payload varies.*
2. **The same check misdiagnoses load as a brake fault, and only near its threshold.** At +900 kg (mass ratio 0.62, nominally above 50%), the actuator lag during the jerk-limited ramp pushes the instantaneous ratio below 0.5 for 400 ms. The controller escalates to BACKUP_BRAKE_STOP and the oracle fails the "expected reaction". At the same load on −6% it doesn't trip. A check sitting on a knife edge, whose outcome depends on grade, gives intermittent field DTCs that blame the brake for payload. *Fix options: compare against a lag-filtered demand (a model of the actuator) instead of the raw command; widen the window; or feed a mass estimate into the expectation. Then re-run this sweep.*
3. **The fallback is mass-blind too.** The backup brake is also a force calibrated on curb mass. Weak brake + 1500 kg on −6% stops in **45.8 m, 3.6× the unloaded flat case**, and still "passes", because no check bounds distance.
4. **The oracle had a time-window blind spot.** The first sweep used the matrix's 8 s window, so loaded downhill stops "failed to stop" when they were still slowing down. The sweep now runs 15 s. *Lesson: a scenario's duration is part of its expected result; size it from the worst case it covers.*

## Honest limits

- No weight transfer, no load-dependent brake fade, no ABS, no tyre model beyond μ·N. The lateral model is still kinematic.
- Open-loop force mapping is an assumption. With a decel-closed-loop brake controller (common in by-wire stacks), finding 1 mostly disappears, and the question becomes that controller's saturation and its own diagnostics.
- The DUT (the safety controller) is unchanged: the plant lives on the bench side, so every DUT level (FMU, native, HiL) sees the same plant.
