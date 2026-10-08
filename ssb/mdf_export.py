"""MDF4 (ASAM MDF 4.x, .mf4) export of the scenario traces, for CANape / asammdf GUI / MDA (v2.7).

    python run.py --mdf          # reports/traces/<scenario>.mf4 next to the CSV traces

One file per scenario: speed (km/h), lateral offset (m), distance (m) and the safety state as an integer channel with a
value-to-text table, so tools show "STOP_IN_LANE" rather than 4. Time base: the bench's 20 ms trace grid, in seconds.
Needs: pip install asammdf. On Python 3.14 its `zstd` dependency has no wheel; `_zstd_shim` maps it to the standard
library's `compression.zstd` (same compress/decompress), only if `zstd` is missing.
"""
from __future__ import annotations

import sys
import types
from pathlib import Path

STATE_CODES = ["INIT", "NORMAL", "DEGRADED", "PULL_OVER", "STOP_IN_LANE", "BRAKE_ONLY_STOP", "BACKUP_BRAKE_STOP", "OFF"]


def _zstd_shim() -> None:
    try:
        import zstd  # noqa: F401
    except ImportError:
        from compression import zstd as std  # Python 3.14+
        shim = types.ModuleType("zstd")
        shim.compress, shim.decompress = std.compress, std.decompress   # type: ignore[attr-defined]
        sys.modules["zstd"] = shim


def available() -> bool:
    try:
        _zstd_shim()
        import asammdf  # noqa: F401
    except ImportError:
        return False
    return True


def write(result: dict, path: Path, bench: str = "") -> Path:
    """One scenario's trace (runner result with keep_trace=True) -> an MDF4 file."""
    _zstd_shim()
    import numpy as np
    from asammdf import MDF, Signal

    tr = result["trace"]
    t = np.array([row[0] / 1000.0 for row in tr], dtype=np.float64)
    conv: dict[str, object] = {f"val_{i}": i for i in range(len(STATE_CODES))}
    conv.update({f"text_{i}": s for i, s in enumerate(STATE_CODES)})
    sigs = [
        Signal(np.array([row[1] for row in tr]), t, name="VehSpeed", unit="km/h", comment="vehicle speed (plant)"),
        Signal(np.array([row[2] for row in tr]), t, name="LateralOffset", unit="m", comment="lateral offset from the lane centre"),
        Signal(np.array([row[4] for row in tr]), t, name="Distance", unit="m", comment="distance travelled"),
        Signal(np.array([STATE_CODES.index(row[3]) if row[3] in STATE_CODES else 255 for row in tr], dtype=np.uint8), t,
               name="SAF_State", conversion=conv, comment="safety controller state"),
    ]
    mdf = MDF(version="4.10")
    mdf.append(sigs, comment=f"{result['key']}: {result.get('title', '')} | DUT {result.get('dut', '')} | {bench}")
    path.parent.mkdir(parents=True, exist_ok=True)
    mdf.save(path, overwrite=True)
    mdf.close()
    return path


def write_all(results: list, out_dir: Path, bench: str = "") -> list[Path]:
    return [write(r, out_dir / f"{r['key'].replace('/', '_').replace('@', '_')}.mf4", bench) for r in results if r.get("trace")]
