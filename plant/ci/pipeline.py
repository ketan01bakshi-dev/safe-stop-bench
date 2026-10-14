"""P6 plant-model pipeline: L0 (specified) and L1 (verified) checks, seeded model mutants, maturity certificate.

    python -m plant.ci l0 | l1 | mutants | all        (writes reports/plant/<stamp>/ and reports/plant/certificate_latest.json)

Attribution of every red result: MODEL (the model violates the spec), CHECK (the check could not evaluate), TOOL (omc failed:
the run is BLOCKED). A TOOL failure never lowers the level and never passes a check.
"""
from __future__ import annotations

import datetime as _dt
import hashlib
import json
import shutil
import traceback
from dataclasses import asdict, dataclass, field
from pathlib import Path

from plant.ci import experiments as ex
from plant.ci import omc
from plant.ci.omc import ModelError, SimResult, ToolError

LADDER = omc.PLANT / "maturity" / "ladder.json"
SPEC = omc.PLANT / "spec" / "plant_spec.md"
MUTANTS = omc.PLANT / "mutants" / "mutants.json"
LEVELS = ("L0", "L1", "L2")
LABEL = "AI-assisted; synthetic; illustrative parameters (not a calibrated vehicle)"


def sha(path_or_text: Path | str) -> str:
    data = path_or_text.read_bytes() if isinstance(path_or_text, Path) else path_or_text.encode("utf-8")
    return hashlib.sha256(data).hexdigest()


def ladder() -> dict:
    return json.loads(LADDER.read_text(encoding="utf-8"))


@dataclass
class Check:
    id: str
    level: str
    passed: bool | None            # None = not evaluated (BLOCKED)
    attribution: str | None        # MODEL / CHECK / TOOL when not passed
    measured: float | str | None = None
    expected: float | str | None = None
    detail: str = ""


@dataclass
class Run:
    stamp: str
    out: Path
    checks: list[Check] = field(default_factory=list)
    convergence: list[dict] = field(default_factory=list)
    mutants: list[dict] = field(default_factory=list)
    blocked: list[str] = field(default_factory=list)
    b2b: list[dict] = field(default_factory=list)
    regression: dict = field(default_factory=dict)
    fmus: dict = field(default_factory=dict)

    def add(self, c: Check) -> None:
        self.checks.append(c)


# --- L0 -----------------------------------------------------------------------------------------------------------------------

def spec_complete() -> tuple[bool, str]:
    text = SPEC.read_text(encoding="utf-8")
    missing = [h for h in ladder()["spec_required_sections"] if h not in text]
    params = text.split("## 3. Parameters")[1].split("## 4.")[0]
    rows = [r for r in params.splitlines() if r.startswith("| `")]
    unsourced = [r.split("|")[1].strip() for r in rows if "placeholder" not in r and "physical constant" not in r]
    ok = not missing and not unsourced and rows
    return bool(ok), f"missing sections {missing}; parameters without a source {unsourced}; {len(rows)} parameter rows"


def run_l0(run: Run, model_file: Path = omc.MODEL_FILE) -> None:
    ok, detail = spec_complete()
    run.add(Check("spec.complete", "L0", ok, None if ok else "MODEL", detail=detail))
    for cls in ("SafeStopPlant.Plant", "SafeStopPlant.PlantDynamic"):
        try:
            res = omc.check_model([model_file], cls, run.out / "l0")
            warn = [ln for ln in res.unit_messages.splitlines() if "INCONSISTENT" in ln]
            run.add(Check("model.check", "L0", True, None, detail=f"{cls}: checkModel completed"))
            run.add(Check("model.units", "L0", not warn, None if not warn else "MODEL", measured=len(warn), expected=0,
                          detail=f"{cls}: unit check {'clean' if not warn else '; '.join(warn[:3])}"))
        except ModelError as e:
            run.add(Check("model.check", "L0", False, "MODEL", detail=str(e)[:400]))
        except ToolError as e:
            run.blocked.append(f"L0 {cls}: {e}")
            run.add(Check("model.check", "L0", None, "TOOL", detail=str(e)[:400]))


# --- L1 -----------------------------------------------------------------------------------------------------------------------

def simulate_all(model_file: Path, tol: float, cwd: Path, prefix: str) -> SimResult:
    exp_file = cwd / "AllExperiments.mo"
    cwd.mkdir(parents=True, exist_ok=True)
    exp_file.write_text(ex.model_source(), encoding="utf-8")
    return omc.simulate([model_file, exp_file], "AllExperiments", ex.STOP_TIME, tol, cwd, prefix, var_regex=ex.output_regex())


def evaluate(sim: SimResult, tol: dict) -> list[Check]:
    out = []
    for e in ex.E:
        for cid, fn in e.checks.items():
            try:
                ok, m, x, detail = fn(sim, e.id, tol)
                out.append(Check(f"{cid}[{e.id}]", "L1", bool(ok), None if ok else "MODEL", round(m, 6), round(x, 6), detail))
            except Exception as err:   # a check that cannot evaluate is the check's fault, not the model's
                out.append(Check(f"{cid}[{e.id}]", "L1", False, "CHECK", detail=f"{type(err).__name__}: {err}"))
    return out


def convergence(coarse: SimResult, fine: SimResult, tol: dict) -> list[dict]:
    rows = []
    grid = [i * 0.01 for i in range(int(ex.STOP_TIME / 0.01) + 1)]
    for e in ex.E:
        name = f"{e.id}.{e.key}"
        yc = [coarse.at(name, t) for t in grid]
        yf = [fine.at(name, t) for t in grid]
        diff = max(abs(a - b) for a, b in zip(yc, yf, strict=True))
        scale = max(max(abs(v) for v in yf), 1e-12)
        limit = max(tol["convergence_rel"] * scale, tol["convergence_abs_floor"])
        rows.append({"experiment": e.id, "output": e.key, "tol_coarse": tol["simulation_tolerance_coarse"],
                     "tol_fine": tol["simulation_tolerance_fine"], "final_coarse": round(yc[-1], 6), "final_fine": round(yf[-1], 6),
                     "max_abs_diff": diff, "limit": limit, "passed": diff <= limit})
    return rows


def run_l1(run: Run, model_file: Path = omc.MODEL_FILE) -> None:
    tol = ladder()["tolerances"]
    try:
        coarse = simulate_all(model_file, tol["simulation_tolerance_coarse"], run.out / "l1_coarse", "coarse")
        fine = simulate_all(model_file, tol["simulation_tolerance_fine"], run.out / "l1_fine", "fine")
    except ModelError as e:
        run.add(Check("handcalc.simulate", "L1", False, "MODEL", detail=str(e)[:400]))
        return
    except ToolError as e:
        run.blocked.append(f"L1: {e}")
        run.add(Check("handcalc.simulate", "L1", None, "TOOL", detail=str(e)[:400]))
        return
    for c in evaluate(fine, tol):
        run.add(c)
    run.convergence = convergence(coarse, fine, tol)
    for row in run.convergence:
        run.add(Check(f"convergence[{row['experiment']}]", "L1", row["passed"], None if row["passed"] else "MODEL",
                      row["max_abs_diff"], row["limit"],
                      f"{row['output']}: final {row['final_coarse']} (tol {row['tol_coarse']}) vs {row['final_fine']} (tol {row['tol_fine']})"))


def run_mutants(run: Run, model_file: Path = omc.MODEL_FILE) -> None:
    tol = ladder()["tolerances"]
    src = model_file.read_text(encoding="utf-8")
    # a mutant is caught only by a check that is green on the clean model (a check already red proves nothing about it)
    baseline_red = {c.id for c in run.checks if c.passed is False}
    for m in json.loads(MUTANTS.read_text(encoding="utf-8"))["mutants"]:
        row = {"id": m["id"], "bug": m["bug"], "expect_caught_by": m["expect_caught_by"]}
        n = src.count(m["find"])
        if n != 1:
            row.update(caught=None, attribution="CHECK", detail=f"substitution applies {n} times (must be exactly 1)")
            run.mutants.append(row)
            continue
        mdir = run.out / "mutants" / m["id"]
        mdir.mkdir(parents=True, exist_ok=True)
        mfile = mdir / "SafeStopPlant.mo"
        mfile.write_text(src.replace(m["find"], m["replace"]), encoding="utf-8")
        try:
            red = []
            for cls in ("SafeStopPlant.Plant", "SafeStopPlant.PlantDynamic"):
                if any("INCONSISTENT" in ln for ln in omc.check_model([mfile], cls, mdir / "l0").unit_messages.splitlines()):
                    red.append(f"model.units[{cls.split('.')[-1]}]")
            sim = simulate_all(mfile, tol["simulation_tolerance_coarse"], mdir, "mutant")
            red += [c.id for c in evaluate(sim, tol) if c.passed is False and c.attribution == "MODEL" and c.id not in baseline_red]
            row.update(caught=bool(red), red_checks=red, attribution=None,
                       expected_catcher_red=any(any(r.startswith(x) for r in red) for x in m["expect_caught_by"]))
        except ModelError as e:
            row.update(caught=True, red_checks=["model.translate"], attribution=None, detail=str(e)[:200])
        except ToolError as e:
            row.update(caught=None, attribution="TOOL", detail=str(e)[:300])
            run.blocked.append(f"mutant {m['id']}: {e}")
        run.mutants.append(row)
    caught = [r for r in run.mutants if r.get("caught") is True]
    ok = len(caught) == len(run.mutants) and run.mutants
    run.add(Check("mutants.all_caught", "L1", bool(ok) if not any(r.get("caught") is None for r in run.mutants) else None,
                  None if ok else ("TOOL" if any(r.get("attribution") == "TOOL" for r in run.mutants) else "MODEL"),
                  len(caught), len(run.mutants), "every seeded model bug turns at least one L1 check red"))


# --- L2: back-to-back with ssb/plant.py + bench regression ------------------------------------------------------------------

FMUS = {"kinematic": ("SafeStopPlant.Plant", "SafeStopPlant_Plant"), "dynamic": ("SafeStopPlant.PlantDynamic", "SafeStopPlant_PlantDynamic")}


def build_fmus(cwd: Path, model_file: Path = omc.MODEL_FILE) -> dict[str, Path]:
    """FMI 2.0 co-simulation FMUs of both plant variants (OpenModelica's built-in fixed-step solver)."""
    lines = [f"loadFile({omc.mo_str(model_file)});", "getErrorString();"]
    for cls, prefix in FMUS.values():
        lines += [f'buildModelFMU({cls}, version="2.0", fmuType="cs", fileNamePrefix="{prefix}");', "getErrorString();"]
    out = omc.run_script(lines, cwd)
    paths = {k: cwd / f"{prefix}.fmu" for k, (_, prefix) in FMUS.items()}
    missing = [str(v) for v in paths.values() if not v.exists()]
    if missing:
        raise ToolError(f"buildModelFMU produced no FMU {missing}: {out[-1200:]}")
    return paths


def run_l2(run: Run, model_file: Path = omc.MODEL_FILE) -> None:
    from plant.ci import l2
    tol = ladder()["tolerances"]
    try:
        fmus = build_fmus(run.out / "fmu", model_file)
    except ToolError as e:
        run.blocked.append(f"L2 FMU build: {e}")
        run.add(Check("b2b.build", "L2", None, "TOOL", detail=str(e)[:400]))
        return
    run.fmus = {k: {"path": str(v), "sha256": sha(v)} for k, v in fmus.items()}
    for row in l2.direct(fmus["kinematic"], fmus["dynamic"], tol):
        worst = max(row["signals"].items(), key=lambda kv: kv[1]["max_abs_diff"] / max(kv[1]["limit"], 1e-12))
        run.b2b.append(row)
        run.add(Check(f"b2b.direct[{row['id']}]", "L2", row["passed"], None if row["passed"] else "MODEL",
                      round(worst[1]["max_abs_diff"], 6), round(worst[1]["limit"], 6),
                      f"worst signal {worst[0]}; final x python {row['final_x_python']:.3f} m vs fmu {row['final_x_fmu']:.3f} m"))
    reg = l2.bench_regression(fmus["kinematic"])
    run.regression = reg
    diff = [r["key"] for r in reg["rows"] if r["python"] != r["fmu"]]
    run.add(Check("bench.regression", "L2", reg["same_verdicts"], None if reg["same_verdicts"] else "MODEL",
                  str(reg["fmu_counts"]), str(reg["python_counts"]),
                  f"{reg['scenarios']} scenarios; verdicts differ: {diff or 'none'}; max stop-distance difference {reg['max_stop_dist_diff_m']} m"))


# --- level + certificate --------------------------------------------------------------------------------------------------------

def level(run: Run) -> str:
    by = {lv: [c for c in run.checks if c.level == lv] for lv in LEVELS}
    achieved = "none"
    for lv in LEVELS:
        cs = by[lv]
        if not cs or any(c.passed is not True for c in cs):
            break
        achieved = lv
    return achieved


def certificate(run: Run, model_file: Path = omc.MODEL_FILE) -> dict:
    try:
        ver = omc.version()
    except ToolError as e:
        ver = f"unavailable ({e})"
    lvl = level(run)
    cert = {
        "schema": "ssb.plant.certificate/1", "model": "SafeStopPlant", "stamp": run.stamp, "label": LABEL,
        "level": lvl, "blocked": run.blocked,
        "status": "BLOCKED" if run.blocked else lvl,
        "hashes": {"model": sha(model_file), "ladder": sha(LADDER), "spec": sha(SPEC), "mutants": sha(MUTANTS),
                   "experiments": sha(Path(ex.__file__)), "handcalc": sha(omc.PLANT / "ci" / "handcalc.py")},
        "tool": {"omc": ver},
        "checks": [asdict(c) for c in run.checks],
        "convergence": run.convergence, "mutants": run.mutants, "b2b": run.b2b, "fmus": run.fmus,
        "regression": {k: v for k, v in run.regression.items() if k != "rows"},
        "regression_rows": run.regression.get("rows", []),
        "summary": {lv: f"{sum(1 for c in run.checks if c.level == lv and c.passed)} / {sum(1 for c in run.checks if c.level == lv)}"
                    for lv in LEVELS},
        "note": "L3 needs measured data the vehicle plant does not have (it tops out at L2); L4 is the gated release (python -m plant.ci release).",
    }
    (run.out / "certificate.json").write_text(json.dumps(cert, indent=1), encoding="utf-8")
    shutil.copyfile(run.out / "certificate.json", omc.OUT / "certificate_latest.json")
    return cert


def new_run() -> Run:
    stamp = _dt.datetime.now().strftime("%Y-%m-%d_%H%M%S")
    out = omc.OUT / stamp
    out.mkdir(parents=True, exist_ok=True)
    return Run(stamp=stamp, out=out)


def main(stage: str = "all") -> int:
    run = new_run()
    try:
        if stage in ("l0", "l1", "all"):
            run_l0(run)
        if stage in ("l1", "all"):
            run_l1(run)
        if stage in ("mutants", "all"):
            run_mutants(run)
        if stage in ("l2", "all"):
            run_l2(run)
    except Exception:
        run.blocked.append("pipeline crashed:\n" + traceback.format_exc())
    cert = certificate(run)
    red = [c for c in run.checks if c.passed is not True]
    print(f"SafeStopPlant: level {cert['level']}  status {cert['status']}  {cert['summary']}")
    for c in red:
        print(f"  {'BLOCKED' if c.passed is None else 'FAIL'} {c.id} [{c.attribution}] measured {c.measured} expected {c.expected}: {c.detail[:160]}")
    for r in run.mutants:
        print(f"  mutant {r['id']}: {'caught' if r.get('caught') else 'NOT CAUGHT' if r.get('caught') is False else 'BLOCKED'}"
              f" by {r.get('red_checks', [])[:3]}")
    print(f"certificate: {run.out / 'certificate.json'}")
    if run.blocked:
        return 5
    return 0 if not red else 1


# --- L4 / CD: gated release ------------------------------------------------------------------------------------------------------

RELEASES = omc.PLANT / "releases"
VERSION_FILE = omc.PLANT / "VERSION"


def release(approve: bool = False, by: str = "") -> int:
    """Release the FMUs of the latest certificate: needs L2 (or higher) on the CURRENT model hash, then the human gate."""
    from design.gates import GatePending, HumanGate
    cert_path = omc.OUT / "certificate_latest.json"
    if not cert_path.exists():
        print("no certificate: run python -m plant.ci all first")
        return 2
    cert = json.loads(cert_path.read_text(encoding="utf-8"))
    if cert.get("status") == "BLOCKED" or cert.get("level") not in ("L2", "L3"):
        print(f"not releasable: certificate {cert.get('stamp')} is {cert.get('status')} at level {cert.get('level')} (needs L2)")
        return 1
    if cert["hashes"]["model"] != sha(omc.MODEL_FILE):
        print("not releasable: the model changed since the certificate (shortcut S3); run python -m plant.ci all again")
        return 1
    version = VERSION_FILE.read_text(encoding="utf-8").strip()
    artifacts = {"model": "SafeStopPlant", "version": version, "level": cert["level"], "certificate": cert["stamp"],
                 "certificate_sha256": sha(cert_path), "model_sha256": cert["hashes"]["model"],
                 "fmus": {k: v["sha256"] for k, v in cert["fmus"].items()}}
    gate = HumanGate(omc.OUT / "release_gate", approvals={"plant_release": approve}, approver=by or "cli",
                     approve_hint='python -m plant.ci release --approve --by "<name>"',
                     reject_hint="(reject: do not run the approve command; fix and re-run python -m plant.ci all)")
    if approve and not by:
        print("--approve needs --by <name>")
        return 2
    summary = (f"Release SafeStopPlant {version} at maturity {cert['level']}: {cert['summary']}; "
               f"bench regression {cert['regression'].get('fmu_counts')} (same verdicts: {cert['regression'].get('same_verdicts')}). "
               "AI-assisted, synthetic, illustrative parameters.")
    try:
        rec = gate.require("plant_release", artifacts, summary)
    except GatePending as e:
        print(f"waiting for the human gate plant_release: review {e.args[0] if e.args else ''}")
        print('approve with: python -m plant.ci release --approve --by "<name>"')
        return 3
    dest = RELEASES / version
    dest.mkdir(parents=True, exist_ok=True)
    for v in cert["fmus"].values():
        src = Path(v["path"])
        if sha(src) != v["sha256"]:
            print(f"FMU {src.name} changed since the certificate")
            return 1
        shutil.copyfile(src, dest / src.name)
    shutil.copyfile(cert_path, dest / "maturity_certificate.json")
    rel = {"schema": "ssb.plant.release/1", "model": "SafeStopPlant", "version": version, "level": cert["level"],
           "approved_by": rec.get("approver"), "approved_at": rec.get("at"), "artifacts": artifacts,
           "fmus": {Path(v["path"]).name: v["sha256"] for v in cert["fmus"].values()}, "label": LABEL,
           "use": f"python run.py --plant fmu:plant/releases/{version}/SafeStopPlant_Plant.fmu"}
    (dest / "release.json").write_text(json.dumps(rel, indent=1), encoding="utf-8")
    (RELEASES / "latest.json").write_text(json.dumps(rel, indent=1), encoding="utf-8")
    print(f"released SafeStopPlant {version} ({cert['level']}) to {dest}")
    return 0
