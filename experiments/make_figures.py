"""Paper figures and tables from the reproduced runs (results/<experiment>/).

    python3 experiments/make_figures.py        # -> results/figures/*.png, results/REPORT.md
"""
import json
import sys
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.transforms as mtrans  # noqa: E402
from matplotlib.patches import Patch, Rectangle  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ecsa_core import Params  # noqa: E402
from ecsa_core.perception import roi_bounds  # noqa: E402

RES = ROOT / "results"
FIG = RES / "figures"
P = Params()
ON, OFF, BG = np.array([234, 56, 56]), np.array([51, 122, 242]), np.array([11, 11, 16])
NAVY = "#1f4e79"


def load(name):
    d = RES / name
    return (json.loads((d / "summary.json").read_text()), dict(np.load(d / "log.npz")),
            json.loads((d / "snapshots.json").read_text()), d)


def snap(d, key):
    return dict(np.load(d / f"snap_{key}.npz"))


def ev_rgb(pol, mute_outside=False):
    out = np.zeros(pol.shape + (3,), np.uint8) + BG.astype(np.uint8)
    out[pol > 0], out[pol < 0] = ON, OFF
    if mute_outside:
        u0, u1, v0, v1 = roi_bounds(P.camera, P.perception.roi_radius)
        m = np.ones(pol.shape, bool); m[v0:v1 + 1, u0:u1 + 1] = False
        out[m] = (BG + 0.25 * (out[m] - BG)).astype(np.uint8)
    return out


def crop_box(ax, **kw):
    u0, u1, v0, v1 = roi_bounds(P.camera, P.perception.roi_radius)
    ax.add_patch(Rectangle((u0 - 0.5, v0 - 0.5), u1 - u0 + 1, v1 - v0 + 1, fill=False, ec="#f2c94c", lw=1.4, **kw))


def legend(ax):
    ax.legend(handles=[Patch(fc=ON / 255, ec="k", label="ON (+)"), Patch(fc=OFF / 255, ec="k", label="OFF (−)")],
              loc="lower right", fontsize=7, framealpha=0.85)


def frames_figure(name, keys, titles, out, suptitle):
    s, L, S, d = load(name)
    fig, axs = plt.subplots(2, len(keys), figsize=(4.6 * len(keys) / 1.5, 4.8), dpi=200)
    for j, (k, ttl) in enumerate(zip(keys, titles)):
        sn = snap(d, k)
        axs[0, j].imshow(sn["image"], cmap="gray", vmin=0, vmax=255, interpolation="nearest")
        axs[1, j].imshow(ev_rgb(sn["polarity"]), interpolation="nearest")
        axs[0, j].set_title(ttl, color=NAVY, fontsize=10)
        for a in axs[:, j]:
            a.set_xticks([]); a.set_yticks([])
            for sp in a.spines.values():
                sp.set_color(NAVY)
    axs[0, 0].set_ylabel("Image", fontsize=11); axs[1, 0].set_ylabel("Events", fontsize=11)
    legend(axs[1, -1])
    fig.suptitle(suptitle, x=0.02, ha="left", color=NAVY, fontsize=12)
    fig.tight_layout(); fig.savefig(out); plt.close(fig)


def isolation_figure(name, out):
    s, L, S, d = load(name)
    keys = ["3.60", "3.90"]
    fig, axs = plt.subplots(2, 3, figsize=(7.3, 4.7), dpi=200)
    for i, k in enumerate(keys):
        sn, meta = snap(d, k), S[k]
        axs[i, 0].imshow(sn["image"], cmap="gray", vmin=0, vmax=255, interpolation="nearest")
        axs[i, 1].imshow(ev_rgb(sn["polarity"]), interpolation="nearest")
        axs[i, 2].imshow(ev_rgb(sn["polarity"], mute_outside=True), interpolation="nearest")
        for a in axs[i]:
            crop_box(a); a.set_xticks([]); a.set_yticks([])
        if meta["selected_bbox"]:                         # only a component actually passed to detection
            u0, v0, u1, v1 = meta["selected_bbox"]
            axs[i, 2].add_patch(Rectangle((u0 - 1, v0 - 1), u1 - u0 + 2, v1 - v0 + 2, fill=False, ec="#6fcf97", lw=1.2))
        axs[i, 0].set_ylabel(f"{'Detection' if i == 0 else 'After sidestep'}\n(t = {meta['t']:.2f} s)", color=NAVY)
    for j, ttl in enumerate(["Image", "Full events (scene + ball)", "Isolated (centre crop)"]):
        axs[0, j].set_title(ttl, color=NAVY, fontsize=10)
    legend(axs[1, 1])
    fig.tight_layout(); fig.savefig(out); plt.close(fig)


def trajectory_figure(name, out):
    """Same two-panel style as the corrected manuscript Fig. 7, from the logged run."""
    s, L, S, d = load(name)
    t, x, y, bx, by = L["t"], L["x"], L["y"], L["bx"], L["by"]
    tl, tc = s["t_launch"], s["commit_t"]
    fl = t >= tl
    sep = np.hypot(np.hypot(bx - x, by - y), L["bz"] - L["z"])
    plt.rcParams.update({"font.family": "DejaVu Serif", "font.size": 8})
    UAV, BALL, SEP, GOAL, GREY = "#1b7f3a", "#c0392b", "#7f8c8d", "#2e86de", "#808080"
    fig, (a, b) = plt.subplots(1, 2, figsize=(7.17, 2.57), dpi=300)
    for yw in (2.0, -2.0):
        w, = a.plot([0, 26], [yw, yw], color=GREY, lw=1.2)
    w.set_label("corridor wall")
    fly = L["z"] > 1.45
    a.plot(x[fly], y[fly], color=UAV, lw=1.6, label="UAV", zorder=2)
    inflight = fl & (bx > P.corridor.x_min + 0.2)
    a.plot(np.r_[P.encounter.x_launch, bx[inflight]], np.r_[0, by[inflight]], color=BALL, ls="--", lw=1.0,
           marker="o", ms=3, markevery=10, label="ball", zorder=3)
    if tc is not None:
        i = int(np.argmin(abs(t - tc)))
        a.plot(x[i], y[i], "o", color=UAV, mec="k", mew=0.6, ms=5, zorder=4)
        a.annotate("dodge", (x[i], y[i]), xytext=(1.6, 1.2), color=UAV, fontsize=7.5,
                   arrowprops=dict(arrowstyle="-", color=UAV, lw=0.7))
    a.plot(P.control.goal_x, 0, marker="*", ms=10, color=GOAL, ls="none", label="goal", zorder=4)
    a.set_xlim(0, 26); a.set_ylim(-2.4, 2.4); a.set_aspect("equal"); a.grid(alpha=0.3)
    a.set_xlabel("x (m)"); a.set_ylabel("y (m)")
    a.legend(loc="lower center", bbox_to_anchor=(0.5, 1.02), ncol=4, frameon=False, fontsize=7)
    a.text(0, 1.0, "(a)", transform=a.transAxes, va="bottom", fontsize=8.5)
    m = inflight
    b.plot(t[m], by[m], color=BALL, ls="--", lw=1.0, marker="o", ms=3, markevery=12, label="ball y", zorder=3)
    b.plot(t, y, color=UAV, lw=2.0, label="UAV y", zorder=2)
    b.set_ylim(-0.2, 3.2); b.set_xlim(0, 16); b.grid(alpha=0.3)
    b.set_xlabel("t (s)"); b.set_ylabel("y (m)")
    b2 = b.twinx()
    kk = np.ceil(sep[m].max() / 3.2 * 1.05)
    b2.plot(t[m], sep[m], color=SEP, lw=1.2, label="separation")
    b2.set_ylim(-0.2 * kk, 3.2 * kk); b2.set_ylabel("separation (m)")
    b.axvline(tl, color=GREY, ls=":", lw=1.0, label=f"throw (t = {tl:.2f} s)")
    top = mtrans.blended_transform_factory(b.transData, b.transAxes)
    b.text(tl - 0.15, 0.97, "throw", transform=top, ha="right", va="top", color="#595959", fontsize=7.5)
    if tc is not None:
        b.axvline(tc, color=UAV, ls=":", lw=1.0, label=f"dodge commanded (t = {tc:.2f} s)")
        b.text(tc + 0.15, 0.97, "dodge", transform=top, ha="left", va="top", color=UAV, fontsize=7.5)
    h1, l1 = b.get_legend_handles_labels(); h2, l2 = b2.get_legend_handles_labels()
    b.legend(h1[:2] + h2 + h1[2:], l1[:2] + l2 + l1[2:], loc="center right", fontsize=6.5, framealpha=0.95)
    b.text(0, 1.0, "(b)", transform=b.transAxes, va="bottom", fontsize=8.5)
    fig.tight_layout(); fig.savefig(out, dpi=300); plt.close(fig)
    plt.rcParams.update(plt.rcParamsDefault)


# ------------------------------------------------------------------------------------ report
PAPER_T1 = {  # ecsa_paper Table 1: dodge (s), d (px), tau_hat/tau (s), miss (m), goal (s)
    "closed_loop_ttc_check": ("3.75", "8", "0.33/0.38", "0.77", "15.75"),
    "closed_loop_noball_check": ("none", "—", "—", "no dodge", "—"),
    "closed_loop_nododge": ("—", "—", "—", "0.10 (hit)", "—"),
    "gz_closed_015": ("3.72", "7", "0.29/0.40", "0.90", "15.75"),
    "gz_closed_015_noball": ("none", "—", "—", "no dodge", "—"),
    "gz_closed_nododge": ("—", "—", "—", "0.10 (hit)", "—"),
}
PAPER_T2 = {"uav1.0_ball10": ("5.50", "9", "0.19/0.30", "0.57"), "uav1.5_ball10": ("3.75", "8", "0.33/0.38", "0.77"),
            "uav2.0_ball10": ("3.00", "9", "0.32/0.30", "0.55"), "uav2.5_ball10": ("2.45", "8", "0.33/0.35", "0.74"),
            "uav3.0_ball10": ("2.15", "8", "0.22/0.32", "0.75"), "uav1.5_ball12": ("3.82", "8", "0.33/0.31", "0.70")}


def ours(s):
    if not s["dodged"]:
        miss = f"{s['sep_min_tick']:.2f} (hit)" if s.get("collision") else "no dodge"
        return ("none" if not s["ball_present"] else "—", "—", "—", miss, f"{s['goal_t']:.2f}" if s["goal_t"] else "—")
    return (f"{s['commit_t']:.3f}", f"{s['commit_d']:.0f}", f"{s['tau_hat']:.2f}/{s['tau_true']:.2f}",
            f"{s['sep_min']:.2f}", f"{s['goal_t']:.2f}" if s["goal_t"] else "—")


def report():
    lines = ["# Reproduction report", "",
             "Paper values: `PHD/ecsa_paper.docx` (Tables 1-3). Reproduced: this repository, one run per row "
             "(both simulators are deterministic). Miss = closest centre-to-centre distance; for hits the per-tick "
             "sampled minimum is shown, as in the paper.", "",
             "## Table 1: nominal encounter (UAV 1.5 m/s, ball 10 m/s) and controls", "",
             "Each cell is **paper → reproduced**.", "", "| Experiment | Dodge (s) | d (px) | τ̂ / τ (s) | Miss (m) | Goal (s) |", "|---|---|---|---|---|---|"]
    for n, pv in PAPER_T1.items():
        s = json.loads((RES / n / "summary.json").read_text()); o = ours(s)
        lines.append(f"| `{n}` | {pv[0]} → {o[0]} | {pv[1]} → {o[1]} | {pv[2]} → {o[2]} | {pv[3]} → {o[3]} | {pv[4]} → {o[4]} |")
    lines += ["", "## Table 2: speed sweep (ray-traced corridor)", "",
              "| Run | Confirm (s) | d (px) | τ̂ / τ (s) | Miss (m) |", "|---|---|---|---|---|"]
    for n, pv in PAPER_T2.items():
        s = json.loads((RES / "speed_sweep" / n / "summary.json").read_text()); o = ours(s)
        lines.append(f"| `{n}` | {pv[0]} → {o[0]} | {pv[1]} → {o[1]} | {pv[2]} → {o[2]} | {pv[3]} → {o[3]} |")
    L = dict(np.load(RES / "gz_closed_015" / "log.npz"))
    lines += ["", "## Table 3: command-path time on gz_closed_015", "",
              f"Mean command-path time (detect + track + command) per tick: **{L['t_cmd_ms'].mean():.2f} ms** "
              f"(of which detection {L['t_perc_ms'].mean():.2f} ms), 25 ms budget. Paper: 1.27 ms on the original laptop.", "",
              "## Figures", "", "`results/figures/`: fig4_gazebo_frames, fig5_raytrace_frames, fig6_isolation, fig7_trajectory.", ""]
    (RES / "REPORT.md").write_text("\n".join(lines))


def main():
    FIG.mkdir(parents=True, exist_ok=True)
    frames_figure("gz_closed_015", ["1.35", "3.60", "3.90"], ["Cruise", "Ball on axis (t = 3.60 s)", "After sidestep (t = 3.90 s)"],
                  FIG / "fig4_gazebo_frames.png", "Gazebo lockstep: camera image (top) and 50 ms events (bottom)")
    frames_figure("closed_loop_ttc_check", ["1.35", "3.75", "3.90"], ["Cruise", "Confirm (ball on axis)", "After sidestep"],
                  FIG / "fig5_raytrace_frames.png", "Ray-traced corridor: image (top) and events (bottom)")
    isolation_figure("gz_closed_015", FIG / "fig6_isolation.png")
    trajectory_figure("gz_closed_015", FIG / "fig7_trajectory.png")
    report()
    print((RES / "REPORT.md").read_text())


if __name__ == "__main__":
    main()
