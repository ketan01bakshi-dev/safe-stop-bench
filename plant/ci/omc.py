"""Thin wrapper around the OpenModelica compiler, run as a subprocess with .mos scripts (no OMPython, no ZeroMQ).

Every failure here is attributed: a model that does not translate is a MODEL failure (the model is wrong); omc missing, a
crash, a timeout or a simulation that produces no result after a clean check is a TOOL failure (the run is BLOCKED).
"""
from __future__ import annotations

import csv
import json
import os
import shutil
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PLANT = ROOT / "plant"
MODEL_FILE = PLANT / "modelica" / "SafeStopPlant.mo"
OUT = ROOT / "reports" / "plant"
TOOLS = PLANT / "tools.json"
# Measured 2026-10-14 (OpenModelica 1.26.9, Windows): right after a fresh compile, omc sometimes reports "Simulation
# execution failed" with an empty result, and running the same executable again a moment later succeeds. It happened with
# and without spaces in the path; cause not isolated (a scan of the brand-new .exe is a guess). simulate() re-launches the
# built executable up to EXEC_RETRIES times and records every retry as a TOOL event in omc_stdout.txt.
EXEC_RETRIES = 3


class ToolError(RuntimeError):
    """The tool chain failed (attribution TOOL): the run is BLOCKED, never a verdict on the model."""


class ModelError(RuntimeError):
    """The model does not check or translate (attribution MODEL)."""


def omc_path() -> Path:
    """omc from $SSB_OMC, $OPENMODELICAHOME, plant/tools.json, the default D:\\OpenModelica, or PATH."""
    cands: list[str | None] = [os.environ.get("SSB_OMC")]
    home = os.environ.get("OPENMODELICAHOME")
    if home:
        cands.append(str(Path(home) / "bin" / "omc.exe"))
        cands.append(str(Path(home) / "bin" / "omc"))
    if TOOLS.exists():
        cands.append(json.loads(TOOLS.read_text(encoding="utf-8")).get("omc"))
    cands += [r"D:\OpenModelica\bin\omc.exe", shutil.which("omc")]
    for c in cands:
        if c and Path(c).is_file():
            return Path(c)
    raise ToolError("omc not found: install OpenModelica or set SSB_OMC / OPENMODELICAHOME")


def run_script(lines: list[str], cwd: Path, timeout: int = 900) -> str:
    """Run a .mos script in `cwd` and return omc's stdout (also written to cwd/omc_stdout.txt)."""
    cwd.mkdir(parents=True, exist_ok=True)
    mos = cwd / "script.mos"
    mos.write_text("\n".join(lines) + "\n", encoding="utf-8")
    try:
        p = subprocess.run([str(omc_path()), mos.name], cwd=cwd, capture_output=True, text=True, timeout=timeout,
                           encoding="utf-8", errors="replace")
    except subprocess.TimeoutExpired as e:
        raise ToolError(f"omc timed out after {timeout} s") from e
    except OSError as e:
        raise ToolError(f"omc could not start: {e}") from e
    (cwd / "omc_stdout.txt").write_text(p.stdout + "\n--- stderr ---\n" + p.stderr, encoding="utf-8")
    if p.returncode != 0:
        raise ToolError(f"omc exited with {p.returncode}: {p.stderr.strip()[:300]}")
    return p.stdout


def mo_str(path: Path) -> str:
    return '"' + str(path).replace("\\", "/") + '"'


def version() -> str:
    out = run_script(["getVersion()"], OUT / "_version")
    return out.strip().strip('"')


@dataclass
class CheckResult:
    ok: bool
    messages: str
    unit_messages: str = ""


UNIT_MARK = "### UNIT CHECK ###"


def check_model(files: list[Path], cls: str, cwd: Path) -> CheckResult:
    """checkModel, then unit checking. A check or translation error is a MODEL failure (raises ModelError).

    omc checks units only while TRANSLATING (checkModel ignores --unitChecking; measured on 1.26.9), and only through
    variables and parameters: a package constant is folded to a bare number first, which is why g is a parameter.
    `unit_messages` holds the INCONSISTENT warnings, empty when the units agree.
    """
    lines = [f"loadFile({mo_str(f)});" for f in files]
    lines += ["getErrorString();", f"checkModel({cls});", "getErrorString();", f'print("{UNIT_MARK}\\n");',
              'setCommandLineOptions("--unitChecking");', f"translateModel({cls});", "getErrorString();"]
    out = run_script(lines, cwd)
    head, _, unit_part = out.partition(UNIT_MARK)
    if "completed successfully" not in head:
        raise ModelError(f"checkModel({cls}) failed:\n{head[-1500:]}")
    bad = [ln.strip() for ln in unit_part.splitlines() if "INCONSISTENT" in ln or "has unit" in ln]
    return CheckResult(ok=True, messages=head, unit_messages="\n".join(bad))


@dataclass
class SimResult:
    t: list[float]
    y: dict[str, list[float]] = field(default_factory=dict)

    def series(self, name: str) -> list[float]:
        if name not in self.y:
            raise KeyError(f"{name} not in the result ({len(self.y)} variables)")
        return self.y[name]

    def at(self, name: str, t: float) -> float:
        """Linear interpolation; at an event time (repeated time stamps) the LAST value counts."""
        ts, ys = self.t, self.series(name)
        if t <= ts[0]:
            return ys[0]
        if t >= ts[-1]:
            return ys[-1]
        lo, hi = 0, len(ts) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ts[mid] <= t:
                lo = mid
            else:
                hi = mid
        if ts[hi] == ts[lo]:
            return ys[hi]
        w = (t - ts[lo]) / (ts[hi] - ts[lo])
        return ys[lo] + w * (ys[hi] - ys[lo])

    def window(self, name: str, t0: float, t1: float) -> list[float]:
        return [y for t, y in zip(self.t, self.series(name), strict=True) if t0 <= t <= t1]


def read_csv(path: Path) -> SimResult:
    with path.open(newline="", encoding="utf-8") as fh:
        rows = list(csv.reader(fh))
    head = [h.strip().strip('"') for h in rows[0]]
    cols: list[list[float]] = [[] for _ in head]
    for r in rows[1:]:
        if not r:
            continue
        for i, v in enumerate(r):
            cols[i].append(float(v))
    return SimResult(t=cols[0], y={h: c for h, c in zip(head[1:], cols[1:], strict=True)})


def simulate(files: list[Path], cls: str, stop: float, tol: float, cwd: Path, prefix: str,
             interval: float = 0.001, var_regex: str | None = None) -> SimResult:
    """Translate + simulate `cls` (DASSL) and read the CSV result. A missing result after a clean check is a TOOL failure."""
    n = max(1, round(stop / interval))
    filt = f', variableFilter="{var_regex}"' if var_regex else ""
    lines = [f"loadFile({mo_str(f)});" for f in files]
    lines += ["getErrorString();",
              f'simulate({cls}, stopTime={stop}, numberOfIntervals={n}, tolerance={tol}, method="dassl", '
              f'outputFormat="csv", fileNamePrefix="{prefix}"{filt});',
              "getErrorString();"]
    out = run_script(lines, cwd)
    res = cwd / f"{prefix}_res.csv"
    exe = cwd / f"{prefix}.exe"
    retries: list[str] = []
    while "Simulation execution failed" in out and exe.exists() and len(retries) < EXEC_RETRIES:
        time.sleep(2.0)
        env = dict(os.environ)
        env["PATH"] = str(omc_path().parent) + os.pathsep + env.get("PATH", "")
        r = subprocess.run([str(exe)], cwd=cwd, env=env, capture_output=True, text=True, timeout=900,
                           encoding="utf-8", errors="replace")
        ok = r.returncode == 0 and res.exists() and res.stat().st_size > 0
        retries.append(f"re-launch {len(retries) + 1}: exit {r.returncode}, result {'written' if ok else 'missing'}")
        if ok:
            break
    if retries:
        with (cwd / "omc_stdout.txt").open("a", encoding="utf-8") as fh:
            fh.write("\n--- TOOL retries (simulation execution failed after a fresh compile) ---\n" + "\n".join(retries) + "\n")
    if not res.exists() or res.stat().st_size == 0:
        if "Error" in out and ("Translation" in out or "translat" in out):
            raise ModelError(f"simulate({cls}) did not translate:\n{out[-1500:]}")
        raise ToolError(f"simulate({cls}) produced no result:\n{out[-1500:]}")
    return read_csv(res)
