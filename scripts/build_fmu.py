"""Build fmu/SafeStopVecu.fmu: the reference safety controller as an FMI 2.0 co-simulation FMU.

    .venv\\Scripts\\python.exe scripts/build_fmu.py

Stages the controller as the package `vecu_core` (so the FMU never imports the bench's `ssb`), bundles the resolved
configs, runs PythonFMU, and writes fmu/mapping_reference.json (identity mapping). Also builds
fmu/SupplierStyleVecu.fmu: the same controller with a supplier's names, units and state codes, to test mapping.
Needs: pip install pythonfmu
"""
from __future__ import annotations

import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from ssb import config, fmu_contract  # noqa: E402

CORE = ["safety.py", "e2e.py", "planner.py", "plant.py"]   # what ssb/safety.py needs, nothing else


def build(out_dir: Path = ROOT / "fmu") -> Path:
    out_dir.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        stage = Path(tmp)
        core = stage / "vecu_core"
        core.mkdir()
        (core / "__init__.py").write_text('"""Safety-controller core, staged into the FMU by scripts/build_fmu.py."""\n')
        for f in CORE:
            shutil.copy(ROOT / "ssb" / f, core / f)
        cfgs = stage / "configs"
        cfgs.mkdir()
        for name in ("default", "offroad"):
            (cfgs / f"{name}.json").write_text(json.dumps(config.load(f"config/{name}.json"), indent=1))
        for script, extra in (("SafeStopVecu.py", []), ("SupplierStyleVecu.py", [str(ROOT / "fmu" / "SafeStopVecu.py")])):
            subprocess.run([sys.executable, "-m", "pythonfmu", "build", "-f", str(ROOT / "fmu" / script),
                            "-d", str(out_dir), str(core), str(cfgs), *extra], check=True)
    (out_dir / "mapping_reference.json").write_text(json.dumps(fmu_contract.identity_mapping(), indent=2))
    return out_dir / "SafeStopVecu.fmu"


if __name__ == "__main__":
    p = build()
    print(f"built {p} ({p.stat().st_size // 1024} KiB), {p.parent / 'SupplierStyleVecu.fmu'} and {p.parent / 'mapping_reference.json'}")
