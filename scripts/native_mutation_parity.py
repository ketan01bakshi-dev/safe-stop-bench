"""Every seeded mutant must fail exactly the same scenarios in the C++ core as in the Python reference.

    .venv\\Scripts\\python.exe scripts/native_mutation_parity.py      (~40 min for all 14; pass mutant names to split the run)

If the sets match for all 14 mutants, the mutation score measured on the PC carries over to the firmware: the bugs
the suite can catch on the reference, it can catch on the C++ that runs on the ESP32.
"""
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from ssb import campaigns, config  # noqa: E402
from ssb.dut import ReferenceDUT  # noqa: E402
from ssb.native import NativeDUT  # noqa: E402
from ssb.safety import MUTANTS  # noqa: E402

cfg = config.load("config/default.json")
only = sys.argv[1:] or list(MUTANTS)          # optional subset, to split the run over several processes
rows, t0 = [], time.time()
for m in only:
    ref = campaigns.matrix(cfg, lambda sc: ReferenceDUT(cfg, frozenset([m]), warm_start=sc.get("start_kmh", 30) > 0), keep_trace=False)
    nat_dut = NativeDUT(cfg, frozenset([m]))
    nat = campaigns.matrix(cfg, lambda sc: nat_dut, keep_trace=False)
    nat_dut.close()
    fr = sorted(r["key"] for r in ref if r["verdict"]["status"] == "FAIL")
    fn = sorted(r["key"] for r in nat if r["verdict"]["status"] == "FAIL")
    rows.append({"mutant": m, "python_fails": len(fr), "cpp_fails": len(fn), "identical": fr == fn,
                 "only_python": sorted(set(fr) - set(fn)), "only_cpp": sorted(set(fn) - set(fr))})
    print(f"{m:24} python {len(fr):2}  C++ {len(fn):2}  {'identical' if fr == fn else 'DIFFERENT'}", flush=True)
ok = all(r["identical"] for r in rows)
tag = "" if only == list(MUTANTS) else "_" + "_".join(only)
(ROOT / "reports" / f"native_mutation_parity{tag}.json").write_text(json.dumps({"all_identical": ok, "rows": rows}, indent=1))
print(f"\n{'ALL ' + str(len(rows)) + ' IDENTICAL' if ok else 'DIFFERENCES'} in {time.time() - t0:.0f} s")
sys.exit(0 if ok else 1)
