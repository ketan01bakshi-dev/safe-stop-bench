# SafeStopPlant: plant specification (v1)

**Status:** the specification the Modelica plant (`plant/modelica/SafeStopPlant`) is built and graded against. AI-assisted, synthetic,
illustrative parameters: **not a calibrated vehicle**. The reference implementation for back-to-back checks is `ssb/plant.py`
(`Actuators`, `Vehicle`, `DynamicVehicle`), which steps an explicit 1 ms Euler update; the Modelica model is the continuous-time
statement of the same physics, integrated by a variable-step solver.

## 1. Scope

The plant is everything between the actuator ECU's command and the vehicle state the safety controller sees:
longitudinal actuator (dead time + first-order lag + limits), backup brake, steering actuator (rate limit + lag), and the vehicle
(kinematic bicycle, or force-based longitudinal model). It does not contain the actuator ECU (E2E check, own fallback), which stays
in the bench (`ssb/plant.py ActuatorECU`).

## 2. Interface of `SafeStopPlant.Plant` (the FMU)

| Signal | Dir | Unit | Range | Meaning |
|---|---|---|---|---|
| `accel_cmd` | in | m/s2 | -10 .. 3 | longitudinal acceleration request from the actuator ECU |
| `steer_cmd` | in | deg | -30 .. 30 | road-wheel angle request |
| `backup_decel` | in | m/s2 | 0 .. 3 | backup-brake request (0 = off) |
| `mu` | in | 1 | 0.1 .. 1.0 | road friction (constant within a scenario) |
| `grade_pct` | in | % | -15 .. 15 | road grade, positive = uphill |
| `brake_factor` | in | 1 | 0 .. 1 | fault: braking effectiveness (1 = healthy) |
| `steer_stuck` | in | bool | | fault: steering actuator frozen |
| `steer_rate_factor` | in | 1 | 0 .. 1 | fault: slow steering actuator (1 = healthy) |
| `a` | out | m/s2 | | acceleration delivered by the actuators (after limits), the bench's `act.step()` return value |
| `delta` | out | deg | | road-wheel angle |
| `v` | out | m/s | >= 0 | vehicle speed |
| `a_veh` | out | m/s2 | | vehicle acceleration along the path (incl. grade, resistances) |
| `yaw_rate` | out | rad/s | | |
| `x`, `y` | out | m | | position |
| `psi` | out | rad | | heading |

## 3. Parameters (from `config/default.json` → `plant`; every value is a placeholder, source = illustrative)

| Parameter | Value | Unit | Source |
|---|---|---|---|
| `dead_time` | 0.080 | s | config `long_dead_time_ms` (placeholder) |
| `tau` | 0.1 | s | config `long_tau_s` (placeholder) |
| `max_accel` | 2.5 | m/s2 | config (placeholder) |
| `backup_dead_time` | 0.050 | s | `ssb/plant.py` (50-sample buffer; placeholder) |
| `steer_rate` | 20.0 | deg/s | config `steer_rate_dps` (placeholder) |
| `steer_tau` | 0.05 | s | config (placeholder) |
| `max_steer` | 30.0 | deg | config (placeholder) |
| `wheelbase` | 2.7 | m | config (placeholder) |
| `g` | 9.81 | m/s2 | physical constant |
| dynamic: `mass`, `payload`, `rot_inertia_factor`, `crr`, `cd_a`, `rho` | 1500, 0, 0.04, 0.015, 0.7, 1.2 | kg, kg, 1, 1, m2, kg/m3 | config `plant.dynamic` (placeholder) |

## 4. Required behaviour (each is a check in `plant/maturity/ladder.json`)

- **R1 dead time.** A step of `accel_cmd` has no effect on `a` before `dead_time`.
- **R2 lag.** After the dead time, `a` follows a first-order lag: it reaches 63.2 % of a step at `dead_time + tau` (within limits).
- **R3 limits.** `a` never exceeds `max_accel`, never goes below `-mu*g`; the lag state does not wind up past a limit.
- **R4 weak brake.** A negative request is scaled by `brake_factor` after the dead time; positive requests are not.
- **R5 backup brake.** `backup_decel` acts after `backup_dead_time` through the same lag, subtracted from the main path, still limited to `-mu*g`.
- **R6 steering.** The steering target moves toward `clamp(steer_cmd, ±max_steer)` at no more than `steer_rate*steer_rate_factor`;
  `delta` follows it with lag `steer_tau`; with `steer_stuck` both stop moving.
- **R7 standstill.** `v` never goes negative: a vehicle at rest stays at rest while the net acceleration is not positive (no rolling back).
- **R8 grade.** Kinematic model: grade adds `-g*grade_pct/100` to the acceleration. Dynamic model: `-g*sin(atan(grade_pct/100))`
  through the mass, and the tyre limit scales with `mu*m*g*cos(theta)`.
- **R9 kinematics.** `yaw_rate = v/wheelbase*tan(delta)`; `x`, `y`, `psi` integrate the bicycle model.
- **R10 dynamic model** (v2.5 semantics): the actuator request is a force on the curb mass; payload reduces deceleration
  proportionally; rolling resistance and drag act only while moving.

## 5. Known differences from the reference implementation (recorded, not hidden)

| # | Reference (`ssb/plant.py`) | This model | Effect |
|---|---|---|---|
| D1 (fixed in the reference, v2.39) | dead time was an 80-sample buffer read **after** appending: **79 ms** (backup brake 49 ms) | `delay(u, 0.080)` exactly | found by the L2 back-to-back (the whole remaining gap; with 79 ms all 14 drives passed). The owner chose to fix the reference: `ssb/plant.py` now reads before appending, exactly 80 / 50 ms |
| D2 | explicit Euler at 1 ms | variable-step solver | lag/position differences of order dt/tau (about 1 %) |
| D3 | limits clamp the lag state after each step | limited integrator (derivative zero at a limit while pushing outward) | equivalent in the limit dt → 0 |
| D4 (fixed) | the dead-time buffer starts full of zeros | first version: `delay(u, T)` alone, which returns **u(0)** for t < T | found by the L1 hand calculation on 14 Oct 2026: a brake request present at t = 0 acted with **no** dead time (stop 0.667 m = v0 × 80 ms short). Now `if time < T then 0 else delay(u, T)` for both delays |

## 6. Out of scope

Tyre slip, lateral dynamics beyond the kinematic bicycle, suspension, rolling backwards, actuator temperature, any real-vehicle
calibration. The vehicle plant has no measured data, so it cannot be **validated** (maturity L3): it tops out at **L2**.
