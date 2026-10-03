"""Generate logs/sample_field_log.csv: a SYNTHETIC stand-in for a recorded field log (20 ms rows).
Normal driving at 30 km/h with a gentle weave, then a sharp 60 deg/s swerve at 6.0 s for 0.3 s."""
import csv
import math
from pathlib import Path

out = Path(__file__).resolve().parent.parent / "logs" / "sample_field_log.csv"
with out.open("w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t_ms", "accel", "steer_deg", "speed_req_kmh", "perception"])
    steer = 0.0
    for i in range(0, 501):
        t = i * 20
        if 6000 <= t < 6300:
            steer += 60 * 0.02
        else:
            steer = 1.0 * math.cos(2 * math.pi * t / 4000) if t < 6000 else steer
        w.writerow([t, 0.0, round(steer, 2), 30, 2])
print(out)
