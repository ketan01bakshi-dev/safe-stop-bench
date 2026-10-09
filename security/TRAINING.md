# Intrusion detector: training and held-out result

Rule profile learned from clean traffic only (22 clean runs, 8800 commands). MLP: 11 window features, 12 hidden units, trained on 2560 windows (300 with an attack).

False alarms on 22 held-out clean runs (other seeds): rules 0, MLP 0.

Held-out attacks (parameters the MLP never saw). Detection latency from the attack's start:

| Attack | Rules | MLP | Both | First by |
|---|---|---|---|---|
| atk_flood_id2_per_ms4 | 1 ms | 199 ms | 1 ms | rule:unknown_id |
| atk_flood_id2032_per_ms1 | 1 ms | 199 ms | 1 ms | rule:unknown_id |
| atk_fuzzy_period_ms11 | 1 ms | 199 ms | 1 ms | rule:unknown_id |
| atk_spoof_invalid_steer-20.0 | 2 ms | 199 ms | 2 ms | rule:period |
| atk_spoof_valid_modeinject_steer-20.0_accel-1.0 | 2 ms | 199 ms | 2 ms | rule:period |
| atk_spoof_valid_modetakeover_steer-18.0_accel1.0 | 1 ms | missed | 1 ms | rule:slew |
| atk_replay_age_ms400_modetakeover | 1 ms | missed | 1 ms | rule:counter |
| atk_replay_age_ms5120_modetakeover | 1 ms | 199 ms | 1 ms | rule:slew |
| atk_spoof_valid_modetakeover_accelNone_steerNone_speedNone | missed | missed | missed | - |
| atk_period_glitch_offset_ms13 | 14 ms | 199 ms | 14 ms | rule:counter |
| atk_period_glitch_offset_ms3 | 4 ms | 199 ms | 4 ms | rule:period |
| atk_signal_ramp_dps4.0 | 3001 ms | 4199 ms | 3001 ms | rule:slew |
| atk_signal_ramp_dps1.0 | 3001 ms | missed | 3001 ms | rule:slew |

Rule thresholds are in `profile.json`; the learned ranges are the evidence for what counts as normal on this bench.
