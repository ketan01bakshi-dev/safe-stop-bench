# P6: the plant model in OpenModelica, with CI, CT and CD

**Status (v2.39):** the vehicle and actuator plant of the bench is a Modelica model (`plant/modelica/SafeStopPlant.mo`), built,
tested and graded to a maturity level by `python -m plant.ci`. AI-assisted, synthetic, illustrative parameters: not a calibrated
vehicle. Results of the latest run: `reports/plant/certificate_latest.json` (section 5 below quotes one run).

## 1. Why OpenModelica, not Simulink

Simulink is the industry default for plant models, but there is no free route to a *private* CI pipeline with it: the desktop
product is free only as a 30-day trial; the free MATLAB Online basic tier includes Simulink Online but is browser-only (20 hours a
month, 15-minute compute sessions); on GitHub Actions MathWorks licenses products automatically only for public repositories
(private ones need a batch-licensing token from a pilot programme). OpenModelica is open source, scriptable (`omc` + `.mos`),
exports FMI 2.0 FMUs, and is on the tool list of simulation-evaluation work. A released FMU can still be opened in Simulink later
(FMU import) as a parity note, never as a gate.

## 2. What is modelled

`plant/spec/plant_spec.md` is the falsifiable specification (interface with units and ranges, every parameter with its source,
requirements R1 to R10, known differences from the reference, scope). Two FMU variants:

| Class | Content | Bench equivalent |
|---|---|---|
| `SafeStopPlant.Plant` | actuators (dead time 80 ms, lag 0.1 s, limits, backup brake, steering rate limit + lag, fault inputs) + kinematic bicycle | `ssb/plant.py` `Actuators` + `Vehicle` |
| `SafeStopPlant.PlantDynamic` | the same actuators + force-based vehicle (mass, payload, rolling, drag, inertia, true slope) | `Actuators` + `DynamicVehicle` |

The bench runs either one with `python run.py --plant fmu:<path>` (`ssb/plant_fmu.py`): one FMU instance per file, stepped at 1 ms,
the actuator ECU (E2E check, own fallback) stays in Python.

## 3. The maturity ladder (`plant/maturity/ladder.json`)

The method is the one of a simulation-evaluation review: a falsifiable spec, hand calculations against the solver,
a two-point convergence check with both values reported, the shortcuts named and blocked, and every failure attributed.

| Level | Name | Evidence |
|---|---|---|
| L0 | Specified | spec complete (every parameter sourced or flagged placeholder), `checkModel` clean, **unit check** clean |
| L1 | Verified | 21 hand-calculation checks over 15 experiments (closed forms in `plant/ci/handcalc.py`, never the solver); convergence at two solver tolerances with both values reported; **every seeded model bug caught** |
| L2 | Back-to-back | the FMU against `ssb/plant.py` on 14 input sequences (1 ms, every output); all 56 bench scenarios give the same verdicts with the FMU plant as with the Python plant |
| L3 | Validated | against measured data. The vehicle plant has none: it **tops out at L2** by design. The P5 chassis motor model can reach L3 with stage-3 encoder data |
| L4 | Released | `python -m plant.ci release`: a hash-bound human gate (`design/gates.HumanGate`, gate `plant_release`), then the FMUs + certificate go to `plant/releases/<version>/` |

**Attribution.** MODEL: the model violates the spec. CHECK: the check could not evaluate (fix the check). TOOL: omc or the
compiler failed; the run is BLOCKED and the level is neither raised nor lowered.

**Shortcuts blocked** (from the review's "shortcuts and misleading results"): tolerances fixed in the ladder file whose
hash is in the certificate (S1); both convergence values reported (S2); L2 runs on the hash frozen at L1 (S3); seeded model
mutants must all be caught, so the pipeline can fail (S4); a tool crash is never a pass or a model failure (S5).

## 4. CI, CT, CD

| Stage | Command | Runs | Gate |
|---|---|---|---|
| CI | `python -m plant.ci l0` | spec, `checkModel`, unit check | red stops the pipeline |
| CT | `python -m plant.ci all` | L0 + L1 (+ mutants) + L2 | the level in the certificate; a level regression is red |
| CD | `python -m plant.ci release` | gate `plant_release`, then the FMU release | needs L2 on the current model hash |

**Release evidence.** `python -m fleet.release` reads `plant/releases/latest.json` and adds a plant-model row to the release decision
(level, check counts, bench regression, approver). An approved certificate or FMU that no longer matches its hash BLOCKS the decision;
a model source changed since the release is a listed risk; the plant's missing L3 is a standing limit. FMUs are not in git, so an FMU
absent on a machine is a note, not a failure. Released 14 Oct 2026: SafeStopPlant 1.0.0 at L2, approved by the owner.

## 5. Results (14 Oct 2026, OpenModelica 1.26.9, Windows)

**Level L2.** L0 5/5 (spec complete, both classes check, unit check clean); L1 36/36 (21 hand-calculation checks over 15
experiments, 15 convergence checks; e.g. the closed-form stop from 30 km/h with -3 m/s2 through dead time and lag is 13.059 m and the
model gives 13.059 m); all **9 seeded model bugs caught**, each by a check that is green on the clean model; L2 15/15 (14
back-to-back drives, worst signal at 75 % of its fixed limit; all 56 scenarios give the same verdicts with the FMU plant as with
the Python plant, 55 PASS + 1 KNOWN, largest stop-distance difference 0.03 m).

**What the pipeline found on its first day** (each fixed, each now guarded by a check):

1. **Modelica `delay()` has no zero history.** Before t = T it returns the input's value at t = 0, so a brake request present at
   start acted with **no dead time**. The L1 hand calculation caught it: the stop was 0.667 m (v0 × 80 ms) short.
2. **The bench's own reference plant was 1 ms short.** `ssb/plant.py` read its 80-sample buffer after appending: 79 ms (backup
   brake 49). The L2 back-to-back exposed it: with 79 ms in the model all drives passed. The owner chose to fix the reference
   (v2.39); the reference matrix stays 55/56 + 1 known and the full suite passes.
3. **A fixed-step FMU parks a hair below zero speed.** The bench and the controller test standstill as `v == 0.0`, so seven
   scenarios never "stopped" with the FMU plant. Fixed in the model with a reset to exactly zero (spec R7).

**Tool findings (TOOL, never counted against the model):** the first runs were BLOCKED while the installer was still unpacking
(`Compile.bat not found`); after a fresh compile omc sometimes reports "Simulation execution failed" although a re-run of the same
executable succeeds (seen with and without spaces in the path; cause not isolated), so `plant.ci` re-launches the executable up to
3 times and logs each retry; omc checks units only while translating and only through parameters (a package constant is folded
first), which is why gravity is a parameter, and a seeded unit error (`units_friction`) proves the check can fail.

## 6. Limits

Illustrative parameters, no measured data: the vehicle plant cannot be validated (L3) and tops out at L2. The convergence check
uses two DASSL tolerances, not a step-size study of the FMU's fixed-step solver. Back-to-back compares against another model of
the same equations (ssb/plant.py), not against physics. AI-assisted.
