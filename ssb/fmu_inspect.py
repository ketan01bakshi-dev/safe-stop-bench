"""Intake checks for a supplied FMU, before any test runs on it.

    python -m ssb.fmu_inspect their_vecu.fmu                              # what is it? does it fit the bench?
    python -m ssb.fmu_inspect their_vecu.fmu --write-mapping m.json       # proposed bench-name -> FMU-name mapping
    python -m ssb.fmu_inspect their_vecu.fmu --lifecycle                  # instantiate/step/reset/free, each in a subprocess

Answers, in order: (1) is the zip a valid FMI 2.0/3.0 model description (FMPy validation)? (2) co-simulation or
model exchange? (3) does it carry a binary for THIS platform? (4) which inputs/outputs/parameters does it expose, with
types and units? (5) which bench names can be mapped, and which required ones can't? (6) does its lifecycle survive
(crashes run in a child process, so they can't take the bench down)? Exit code 1 if anything blocks a test run.
"""
from __future__ import annotations

import argparse
import re
import json
import subprocess
import sys
from pathlib import Path

from . import fmu_contract


def _norm(s: str) -> str:
    return "".join(ch for ch in s.lower() if ch.isalnum())


# Automotive abbreviations seen in AUTOSAR / supplier signal names -> one canonical word.
SYN = {"ctr": "counter", "cnt": "counter", "count": "counter", "alive": "counter", "ax": "accel", "acc": "accel",
       "spd": "speed", "v": "speed", "vel": "speed", "velocity": "speed", "wdg": "wd", "watchdog": "wd",
       "resp": "answer", "response": "answer", "perc": "perception", "qly": "health", "quality": "health",
       "flg": "flags", "flag": "flags", "tstamp": "timestamp", "time": "timestamp", "stamp": "timestamp",
       "request": "req", "command": "cmd", "ang": "angle", "trig": "kick", "trigger": "kick", "bkp": "backup",
       "brk": "brake", "whl": "wheel", "pwr": "power", "supply": "power", "fault": "cause", "reason": "cause",
       "chlg": "challenge", "rel": "release"}
# Prefixes and owner tags that say where a signal lives, not what it is.
WEAK = {"pln", "planner", "saf", "safety", "sfm", "veh", "vehicle", "bench", "e", "2", "com", "dio", "can", "hmi", "act",
        "actuator", "cmd", "dbg", "long", "operator", "kph", "kmh", "mps"}


def _tokens(name: str) -> set:
    out = set()
    for part in re.split(r"[_.\[\]\s]+", name):
        for tok in re.findall(r"[A-Z]+(?=[A-Z][a-z])|[A-Z]?[a-z]+|[A-Z]+|\d+", part):
            t = SYN.get(tok.lower(), tok.lower())
            if t not in WEAK:
                out.add(t)
    return out


def _score(bench: str, fmu_name: str) -> float:
    if fmu_name == bench:
        return 3.0
    if _norm(fmu_name) == _norm(bench):
        return 2.0
    a, b = _tokens(bench), _tokens(fmu_name)
    return len(a & b) / len(a | b) if a and b else 0.0


def propose_mapping(variables: list) -> tuple[dict, list[str]]:
    """Match bench names to FMU variables of the same causality and type. Score = exact name (3) > same name ignoring
    case/punctuation (2) > overlap of meaning tokens after expanding abbreviations (0..1). Pairs are assigned best score
    first, so one good match can't be stolen by a weaker one. Anything below exact is marked CHECK for a human."""
    mapping = {"inputs": {}, "outputs": {}, "parameters": {}, "state_values": {str(k): v for k, v in fmu_contract.STATE_VALUES.items()}}
    notes = []
    for kind, rows, section in (("input", fmu_contract.INPUTS, "inputs"), ("output", fmu_contract.OUTPUTS, "outputs"),
                                ("parameter", fmu_contract.PARAMETERS, "parameters")):
        pool = [v for v in variables if v.causality == kind]
        pairs = sorted(((_score(bench, v.name), bench, v) for bench, typ, _, _ in rows for v in pool
                        if ("Integer" if v.type == "Enumeration" else v.type) == typ), key=lambda p: -p[0])
        taken_b, taken_v, chosen = set(), set(), {}
        for s, bench, v in pairs:
            if s >= 0.5 and bench not in taken_b and v.name not in taken_v:
                chosen[bench] = (v, s)
                taken_b.add(bench)
                taken_v.add(v.name)
        for bench, typ, required, doc in rows:
            if bench not in chosen:
                notes.append(f"{'MISSING ' if required else 'UNMAPPED'} {kind:9} {bench:28} ({typ}, "
                             f"{'required' if required else 'optional'}) - {doc}")
                continue
            v, s = chosen[bench]
            entry = v.name
            unit = (getattr(v, "unit", None) or "").lower()
            if bench == "VEH_Speed" and ({"kph", "kmh"} & {t.lower() for t in re.findall(r"[A-Za-z]+", v.name)} or unit in ("km/h", "kph")):
                entry = {"name": v.name, "factor": 3.6}
                notes.append(f"UNIT     {kind:9} {bench:28} -> {v.name}: looks like km/h, factor 3.6 set (confirm)")
            mapping[section][bench] = entry
            if s < 3.0:
                notes.append(f"CHECK    {kind:9} {bench:28} -> {v.name} (score {s:.2f})")
            if bench == "SAF_State" and s < 3.0:
                notes.append("CHECK    state_values: assumed to be the DBC enumeration; take the real one from the supplier's docs")
    return mapping, notes


# title -> (operations, what the bench needs it for; None = a bench run can't work without it)
LIFECYCLE = {
    "instantiate, init, 100 x 1 ms steps, terminate, free": ("inst init step term free", None),
    "fmi2Reset, then init and step again": ("inst init step term reset init step term free", None),
    "two instances at once": ("inst inst2 init step term free", "parallel campaigns, or several FMUs in one process"),
    "free, then instantiate again in the same process": ("inst init step term free inst init step term free",
                                                          "a fallback when fmi2Reset is missing"),
}

_CHILD = r'''
import sys
from fmpy import extract, read_model_description
from fmpy.fmi2 import FMU2Slave
path, ops = sys.argv[1], sys.argv[2].split()
md = read_model_description(path, validate=False)
u = extract(path)
mk = lambda name: FMU2Slave(guid=md.guid, unzipDirectory=u, modelIdentifier=md.coSimulation.modelIdentifier, instanceName=name)
f = mk("a"); t = 0.0; keep = []
for op in ops:
    if op == "inst": f.instantiate()
    elif op == "inst2": g = mk("b"); g.instantiate(); keep.append(g)   # keep it alive
    elif op == "init": f.setupExperiment(startTime=0.0); f.enterInitializationMode(); f.exitInitializationMode(); t = 0.0
    elif op == "step":
        for _ in range(100): f.doStep(currentCommunicationPoint=t, communicationStepSize=0.001); t += 0.001
    elif op == "term": f.terminate()
    elif op == "reset": f.reset()
    elif op == "free": f.freeInstance()
print("OK")
'''


def lifecycle(path: str, repeats: int = 5) -> list[tuple[str, bool, str]]:
    """Each sequence `repeats` times in fresh child processes. Repeats matter: a supplier-style test FMU crashed on a
    second instance 8 runs in 10 (v2.2 finding), so one passing run proves little."""
    rows = []
    for title, (ops, _) in LIFECYCLE.items():
        fails, why = 0, ""
        for _ in range(repeats):
            try:
                p = subprocess.run([sys.executable, "-X", "faulthandler", "-c", _CHILD, path, ops], capture_output=True,
                                   text=True, timeout=60)
                if p.returncode == 0 and "OK" in p.stdout:
                    continue
                lines = p.stderr.strip().splitlines()
                key = [ln for ln in lines if ln.startswith(("Fatal Python error", "Windows fatal exception", "OSError"))]
                why = why or ((key or lines or [""])[0] + f" (exit code {p.returncode})")[:140]
            except subprocess.TimeoutExpired:
                why = why or "hung (> 60 s)"
            fails += 1
        rows.append((title, fails == 0, f"{fails}/{repeats} runs failed: {why}" if fails else ""))
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fmu")
    ap.add_argument("--write-mapping")
    ap.add_argument("--lifecycle", action="store_true")
    a = ap.parse_args(argv)
    from fmpy import platform, read_model_description, supported_platforms
    from fmpy.validation import validate_fmu

    blocking, warnings = [], []
    errors = validate_fmu(a.fmu)
    print(f"FMU          {a.fmu}")
    if errors:
        blocking.append("model description invalid")
        print("VALIDATION   FAILED")
        for e in errors[:20]:
            print(f"  - {e}")
    else:
        print("VALIDATION   ok (FMPy schema + rules)")
    md = read_model_description(a.fmu, validate=False)
    kinds = [k for k, x in (("co-simulation", md.coSimulation), ("model exchange", md.modelExchange)) if x is not None]
    print(f"FMI          {md.fmiVersion}   kind: {', '.join(kinds)}   tool: {md.generationTool}   model: {md.modelName}")
    plats = supported_platforms(a.fmu)
    print(f"PLATFORMS    {', '.join(plats)}   (this PC: {platform})")
    if md.coSimulation is None:
        blocking.append("no co-simulation interface (model exchange needs a solver wrapper, e.g. FMPy's)")
    if platform not in plats:
        blocking.append(f"no binary for {platform}: ask the supplier for one, or run it on that OS (Docker/WSL)")
    if not md.fmiVersion.startswith("2."):
        blocking.append(f"FMI {md.fmiVersion}: the bench adapter drives FMI 2.0 (FMPy reads 3.0; adapter extension needed)")
    de = md.defaultExperiment
    if de is not None and de.stepSize:
        print(f"STEP         default experiment step {de.stepSize} s (bench steps 0.001 s)")
    if md.coSimulation is not None and not md.coSimulation.canHandleVariableCommunicationStepSize:
        print("NOTE         fixed communication step: confirm it equals or divides 1 ms")
    print(f"\nVARIABLES    ({len(md.modelVariables)})")
    for v in md.modelVariables:
        if v.causality in ("input", "output", "parameter"):
            unit = getattr(v, "unit", None) or (getattr(v.declaredType, "unit", None) if v.declaredType else None) or ""
            print(f"  {v.causality:9} {v.type:11} {v.variability or '':10} {v.name:30} {unit:8} start={v.start}")

    mapping, notes = propose_mapping(md.modelVariables)
    print("\nMAPPING      bench name -> FMU variable")
    for n in notes:
        print(f"  {n}")
    missing = [n for n in notes if n.startswith("MISSING")]
    print(f"  {sum(len(mapping[s]) for s in ('inputs', 'outputs', 'parameters'))} mapped, {len(missing)} required missing, "
          f"{sum(n.startswith('CHECK') for n in notes)} to confirm by a human")
    if missing:
        blocking.append(f"{len(missing)} required bench signal(s) have no FMU variable (ask for them, or adapt the contract)")
    if a.write_mapping:
        Path(a.write_mapping).write_text(json.dumps(mapping, indent=2), encoding="utf-8")
        print(f"  written {a.write_mapping} - review every CHECK line and the state_values before trusting it")

    if a.lifecycle:
        print("\nLIFECYCLE    (each in a child process)")
        for title, ok, why in lifecycle(a.fmu):
            print(f"  {'ok  ' if ok else 'FAIL'} {title}" + (f"  -> {why}" if why else ""))
            needed_for = LIFECYCLE[title][1]
            if not ok and needed_for is None:
                blocking.append(f"lifecycle: {title}")
            elif not ok:
                warnings.append(f"lifecycle '{title}' fails: matters for {needed_for}")
    for w in warnings:
        print(f"\nWARNING      {w}")
    print("\nVERDICT      " + ("ready for the bench" if not blocking else "BLOCKED:\n  - " + "\n  - ".join(blocking)))
    return 1 if blocking else 0


if __name__ == "__main__":
    sys.exit(main())
