"""P5 stage 0: the contrast timeline (PNG, optional MP4) of the correct build against the two seeded bugs, from the reference traces.

    python vehicle/tools/make_timeline.py                                  # P5-01L (300 ms loss) -> vehicle/out/p5_timeline_300ms.png
    python vehicle/tools/make_timeline.py --scenario p5_planned_stop_link_lost   # the permanent loss
    python vehicle/tools/make_timeline.py --clip                           # also an MP4 that draws the three runs in real time (needs ffmpeg)

Needs matplotlib (`pip install matplotlib`, the `p5` extra). Nothing here touches a board. The floor distance on the right axis uses the
PLACEHOLDER scale k of vehicle/p5_runs.py until the stage-3 calibration; it is labelled as such on the figure.
"""
from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FFMpegWriter, FuncAnimation  # noqa: E402

from vehicle import p5_runs  # noqa: E402

COLORS = {"INIT": "#e6e6e6", "NORMAL": "#dcf2e0", "STOP_IN_LANE": "#f7d4d4"}
LABEL = ("AI-assisted, synthetic data, illustrative limits. Home bench, safety layer only. Scaled kinematic replay of the reference model "
         f"(floor scale k = {p5_runs.K_PLACEHOLDER:.4f}, PLACEHOLDER until calibrated). Not a planner validation.")


def outcome_line(name: str, s: dict) -> str:
    if s["t_stopped_ms"] is not None and not s["resumed_after_stop_state"]:
        return (f"stopped at {s['t_stopped_ms'] / 1000:.2f} s after {s['stop_x_m']:.1f} m of model travel "
                f"({s['stop_floor_m']:.2f} m of floor), latched")
    if s["resumed_after_stop_state"]:
        return f"STOPPED, THEN DROVE ON: back in NORMAL, {s['end_kmh']:.0f} km/h and {s['end_x_m']:.0f} m ({s['end_floor_m']:.1f} m of floor) at {s['end_t_ms'] / 1000:.0f} s"
    return f"NEVER REACTED: {s['end_kmh']:.0f} km/h and {s['end_x_m']:.0f} m ({s['end_floor_m']:.1f} m of floor) at {s['end_t_ms'] / 1000:.0f} s"


def build(runs: dict, key: str, upto_ms: int | None = None) -> tuple:
    names = list(runs)
    fig, axes = plt.subplots(len(names), 1, figsize=(12.5, 9.2), sharex=True)
    scen = p5_runs.scenarios()[key]
    inj = scen.get("faults") or []
    cursors = []
    for ax, name in zip(axes, names, strict=True):
        r = runs[name][key]
        tr = r["trace"]
        s = p5_runs.stats(r)
        t_s = [t / 1000 for t, *_ in tr]
        for a, b, st in p5_runs.state_segments(r):
            ax.axvspan(a / 1000, (b + 20) / 1000, color=COLORS.get(st, "#ffffff"), lw=0)
        line, = ax.plot(t_s, [v for _t, v, *_ in tr], color="#1f4e9c", lw=2.0, label="model speed (km/h)")
        ax2 = ax.twinx()
        ax2.plot(t_s, [x * p5_runs.K_PLACEHOLDER for *_, x in tr], color="#8a6d3b", lw=1.2, ls="--", label="floor distance, placeholder scale (m)")
        ax2.set_ylim(0, 4.5)
        ax2.set_ylabel("floor distance (m)\nplaceholder scale", fontsize=8, color="#8a6d3b")
        if upto_ms is not None:   # animation: draw only the past
            line.set_data(t_s, [v if t <= upto_ms else float("nan") for t, v, *_ in tr])
        for f in inj:
            if f["type"] == "link_lost":
                end = f.get("end", tr[-1][0])
                ax.axvspan(f["start"] / 1000, min(end, tr[-1][0]) / 1000, ymin=0.0, ymax=0.06, color="#444444")
        ax.axvline(scen["inject_ms"] / 1000, color="#444444", lw=1.0, ls=":")
        ax.set_ylim(0, 32)
        ax.set_xlim(0, tr[-1][0] / 1000)
        ax.set_ylabel("km/h")
        ax.set_title(f"{name}: {outcome_line(name, s)}", loc="left", fontsize=10.5, color="#8a1c1c" if (s["resumed_after_stop_state"] or s["t_stopped_ms"] is None) else "#1c5a2a")
        cur = ax.axvline(0, color="#000000", lw=1.2)
        cursors.append(cur)
        if name == names[0]:
            ax.text(scen["inject_ms"] / 1000 + 0.05, 29.0, "command stream lost", fontsize=8.5, color="#444444")
    axes[-1].set_xlabel("time (s)   [green: NORMAL, red: STOP_IN_LANE, grey bar: link lost]")
    fig.suptitle(f"P5 stage 0 - {scen['title'].removeprefix('P5: ')}", fontsize=12, x=0.01, ha="left")
    fig.text(0.01, 0.005, LABEL, fontsize=7.5, color="#555555", wrap=True)
    fig.tight_layout(rect=(0, 0.025, 1, 0.97))
    return fig, cursors, list(axes)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="p5_planned_stop_link_lost_300ms", help="scenario key from vehicle/p5_scenarios.json")
    ap.add_argument("--out", default="vehicle/out")
    ap.add_argument("--clip", action="store_true", help="also write an MP4 (needs ffmpeg on PATH)")
    a = ap.parse_args()
    out = ROOT / a.out
    out.mkdir(parents=True, exist_ok=True)
    runs = p5_runs.run_variants()
    stem = "p5_timeline_300ms" if a.scenario.endswith("300ms") else "p5_timeline_permanent"
    fig, _, _ = build(runs, a.scenario)
    png = out / f"{stem}.png"
    fig.savefig(png, dpi=130)
    plt.close(fig)
    print(f"wrote {png}")
    for name, per in runs.items():
        s = p5_runs.stats(per[a.scenario])
        print(f"  {name:28} {outcome_line(name, s)} | verdict {'PASS' if s['verdict_passed'] else 'FAIL'}")
    if a.clip:
        ff = shutil.which("ffmpeg")
        if not ff:
            print("ffmpeg not found: no clip")
            return 0
        matplotlib.rcParams["animation.ffmpeg_path"] = ff
        end = next(iter(runs.values()))[a.scenario]["trace"][-1][0]
        fig, cursors, axes = build(runs, a.scenario, upto_ms=0)
        frames = list(range(0, end + 1, 50))

        def update(i: int):
            t = frames[i]
            for ax, name, cur in zip(axes, runs, cursors, strict=True):
                cur.set_xdata([t / 1000, t / 1000])
                tr = runs[name][a.scenario]["trace"]
                ax.lines[0].set_data([x / 1000 for x, *_ in tr], [v if x <= t else float("nan") for x, v, *_ in tr])
            return cursors

        anim = FuncAnimation(fig, update, frames=len(frames), blit=False)
        mp4 = out / f"{stem}.mp4"
        anim.save(mp4, writer=FFMpegWriter(fps=20, bitrate=2400), dpi=90)
        print(f"wrote {mp4}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
