"""REPORT_v3 figures -> results/v3/figures/*.png  (run inside the lio-py env).

Fixed entity -> colour mapping (validated categorical palette, light surface); every line is
also direct-labelled at its end (relief for the three low-contrast slots) and the legend is kept.
"""
from __future__ import annotations

import json
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from liobench import audit, v3  # noqa: E402
from liobench.correct import heading as H  # noqa: E402
from liobench.dataset import _gt_heading  # noqa: E402
from liobench.geometry import cumulative_distance, interpolate, level  # noqa: E402
from liobench.io import RESULTS_ROOT, load_config, load_ground_truth, load_run, results_dir  # noqa: E402
from liobench.metrics import evaluate  # noqa: E402

SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
C = {"faster_lio": "#2a78d6", "fast_lio": "#eb6834", "dlio": "#1baf7a", "genz_lio": "#eda100",
     "kiss_icp": "#e87ba4", "pipe1": "#008300", "pipe2": "#6250d6"}
LABEL = {"faster_lio": "Faster-LIO", "fast_lio": "FAST-LIO", "dlio": "DLIO", "genz_lio": "GenZ-LIO", "kiss_icp": "KISS-ICP"}
STYLE = {"faster_lio": "-", "fast_lio": "--", "dlio": "-.", "genz_lio": (0, (5, 1.5, 1, 1.5)), "kiss_icp": ":"}
OUT = results_dir("v3", "figures")

plt.rcParams.update({"figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
                     "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
                     "text.color": INK, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
                     "axes.spines.top": False, "axes.spines.right": False, "font.size": 9,
                     "legend.frameon": False, "lines.linewidth": 1.6})


_PENDING: dict = {}


def end_label(ax, x, y, text, color):
    """Queue a direct label at the series' last point; placed without overlaps by place_labels()."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    k = np.flatnonzero(np.isfinite(x) & np.isfinite(y))[-1]
    ax.plot([x[k]], [y[k]], "o", ms=4, color=color, mec=SURFACE, mew=1.0)
    _PENDING.setdefault(ax, []).append((x[k], y[k], text))


def place_labels(ax, min_gap_frac=0.07):
    items = sorted(_PENDING.pop(ax, []), key=lambda it: it[1])
    if not items:
        return
    lo, hi = ax.get_ylim()
    gap = (hi - lo) * min_gap_frac
    ys = [it[1] for it in items]
    for i in range(1, len(ys)):                       # push labels up until they are gap apart
        ys[i] = max(ys[i], ys[i - 1] + gap)
    over = ys[-1] - (hi - gap / 2)
    if over > 0:                                      # shift the stack down if it ran off the top
        ys = [y - over for y in ys]
        for i in range(len(ys) - 2, -1, -1):
            ys[i] = min(ys[i], ys[i + 1] - gap)
    xr = ax.get_xlim()
    dx = (xr[1] - xr[0]) * 0.012
    for (x, y, text), yl in zip(items, ys):
        ax.annotate(text, (x, y), xytext=(x + dx, yl), textcoords="data", va="center", fontsize=8, color=INK2,
                    arrowprops=dict(arrowstyle="-", color=GRID, lw=0.6) if abs(yl - y) > gap * 0.2 else None)


def save(fig, name):
    for ax in fig.axes:
        place_labels(ax)
    fig.tight_layout()
    fig.savefig(OUT / name, dpi=180)
    plt.close(fig)
    print("wrote", OUT / name)


def _slow(gt, v_min=3.0, pad_s=3.0):
    """True where the vehicle is slower than v_min (GT course undefined), padded by pad_s."""
    sp = np.r_[0, np.linalg.norm(np.diff(gt.p[:, :2], axis=0), axis=1) / np.diff(gt.t)]
    slow = sp < v_min
    k = int(pad_s * 10)
    return np.convolve(slow.astype(float), np.ones(2 * k + 1), "same") > 0


def fig_heights(cfg):
    gt = load_ground_truth(cfg)
    dg = cumulative_distance(gt.p) / 1000
    fig, ax = plt.subplots(figsize=(7, 3.4))
    ax.plot(dg, gt.p[:, 2] - gt.p[0, 2], color=INK, lw=1.8, label="GNSS altitude")
    end_label(ax, dg, gt.p[:, 2] - gt.p[0, 2], "GNSS", INK)
    for a, tag in (("faster_lio", "full"), ("fast_lio", "full"), ("dlio", "full"), ("genz_lio", "full_ros"), ("kiss_icp", "full")):
        tr = level(load_run(a, tag, cfg))
        tr = tr.shifted(audit.train_offset(load_run(a, tag, cfg), cfg))
        p, _, ok = interpolate(tr, gt.t, 0.15)
        z = p[:, 2] - p[ok][0, 2]
        ax.plot(dg[ok], z[ok], color=C[a], ls=STYLE[a], label=LABEL[a])
        end_label(ax, dg[ok], z[ok], LABEL[a], C[a])
    ax.set_xlabel("distance driven [km]")
    ax.set_ylabel("height change [m]")
    ax.set_title("Estimated height vs GNSS: every estimator 'climbs' kilometres (pitch drift)", loc="left", fontsize=10)
    ax.legend(loc="upper left", ncol=3, fontsize=8)
    ax.set_xlim(0, dg[-1] * 1.12)
    save(fig, "f1_height_profile.png")


def fig_heading(cfg, S):
    gt = load_ground_truth(cfg)
    g = _gt_heading(gt)
    t0 = gt.t[0]
    slow = _slow(gt)
    fig, ax = plt.subplots(figsize=(7, 3.4))
    for k in ("faster_lio", "fast_lio", "dlio", "genz_lio", "kiss_icp"):
        s = S[k]
        ok = (gt.t >= s.tu[0]) & (gt.t <= s.tu[-1])
        e = np.degrees(np.interp(gt.t[ok], s.tu, np.cumsum(s.u[:, 2])) - g[ok])
        e -= e[0]
        e[slow[ok]] = np.nan
        ax.plot(gt.t[ok] - t0, e, color=C[k], ls=STYLE[k], label=LABEL[k])
        end_label(ax, gt.t[ok] - t0, e, LABEL[k], C[k])
    for lo, hi in v3.blocks(cfg)[1:]:
        ax.axvline(lo - t0, color=GRID, lw=0.8, zorder=0)
    ax.set_xlabel("time [s]  (thin lines: the 5 evaluation blocks)")
    ax.set_ylabel("heading error [deg]")
    ax.set_title("Heading error of each estimator (tilt removed): all drift counter-clockwise", loc="left", fontsize=10)
    ax.legend(loc="upper left", ncol=3, fontsize=8)
    ax.set_xlim(0, (gt.t[-1] - t0) * 1.13)
    save(fig, "f2_heading_error.png")


def fig_probes(cfg):
    names = [("kiss_icp", "default deskew"), ("kiss_icp_shift5ms", "times +5 %"), ("kiss_icp_vox05", "0.5 m voxels"),
             ("kiss_icp_nodeskew", "deskew off"), ("kiss_icp_revtime", "deskew reversed")]
    cols = ["#e87ba4", "#2a78d6", "#1baf7a", "#eb6834", "#6250d6"]
    styles = ["-", "--", "-.", (0, (5, 1.5, 1, 1.5)), ":"]
    S = v3.load_sources(cfg, {n: (n, "full") for n, _ in names[1:]})
    gt = load_ground_truth(cfg)
    g = _gt_heading(gt)
    t0 = gt.t[0]
    slow = _slow(gt)
    fig, ax = plt.subplots(figsize=(7, 3.4))
    for (n, lab), col, st in zip(names, cols, styles):
        if n not in S:
            continue
        s = S[n]
        ok = (gt.t >= s.tu[0]) & (gt.t <= s.tu[-1])
        e = np.degrees(np.interp(gt.t[ok], s.tu, np.cumsum(s.u[:, 2])) - g[ok])
        e -= e[0]
        e[slow[ok]] = np.nan
        ax.plot(gt.t[ok] - t0, e, color=col, ls=st, label=lab)
        end_label(ax, gt.t[ok] - t0, e, lab, col)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("heading error [deg]")
    ax.set_title("KISS-ICP: motion distortion drives a counter-clockwise heading bias", loc="left", fontsize=10)
    ax.legend(loc="upper left", ncol=2, fontsize=8)
    ax.set_xlim(0, (gt.t[-1] - t0) * 1.2)
    save(fig, "f3_kiss_distortion_probes.png")


def fig_duty(cfg):
    rep = json.loads((RESULTS_ROOT / "v3" / "combos.json").read_text())
    P = rep["paired"]
    def meta_ms(name):
        m = json.loads((RESULTS_ROOT / "runs" / name / "full" / "meta.json").read_text())
        return m.get("ms_per_scan")
    pts = [("G: Faster-LIO + gravity filter (0 ms)", 0.0, "no ICP")]
    for n in (20, 10, 5, 2):
        pts.append((f"VGICP-grav every {n}", meta_ms(f"faster_lio+vgicp_grav_every{n}"), f"every {n}"))
    pts.append(("VGICP-grav every 1", meta_ms("faster_lio+vgicp_grav"), "every scan"))
    fig, ax = plt.subplots(figsize=(6.2, 3.4))
    x = [p[1] for p in pts]
    y = [P[p[0]]["ate_m"]["mean"] for p in pts]
    se = [P[p[0]]["ate_m"]["se"] if p[0] != pts[0][0] else 0 for p in pts]
    ax.errorbar(x, y, yerr=se, color=C["faster_lio"], marker="o", ms=5, capsize=2, lw=1.6, label="VGICP-grav correction, own heading")
    offs = [(4, 6), (-6, -14), (2, 10), (6, -14), (4, 6), (-10, 8)]   # staggered: the low-duty points are close
    for (name, xi, lab), yi, (dx, dy) in zip(pts, y, offs):
        ax.annotate(lab, (xi, yi), xytext=(dx, dy), textcoords="offset points", fontsize=8, color=INK2,
                    ha="right" if dx < 0 else "left")
    yk = P["G + KISS yaw"]["ate_m"]["mean"]
    ax.axhline(yk, color=C["kiss_icp"], ls="--", lw=1.4, label="no ICP, heading from KISS-ICP")
    ax.annotate("heading from KISS-ICP", (max(x), yk), xytext=(-4, 5), textcoords="offset points", ha="right", fontsize=8, color=INK2)
    ax.set_xlabel("added correction cost [ms per scan, laptop, Python]")
    ax.set_ylabel("mean ATE per 2 km block [m]")
    ax.set_ylim(0, max(y) * 1.25)
    ax.set_title("Correction duty vs accuracy (5 held-out blocks, ±SE of paired Δ)", loc="left", fontsize=10)
    ax.legend(loc="lower left", fontsize=8)
    save(fig, "f4_duty_cycle.png")


def fig_bias(cfg):
    d = json.loads((RESULTS_ROOT / "v3" / "bias_per_block.json").read_text())["bias_deg_s"]
    order = [k for k in ("faster_lio", "fast_lio", "genz_lio", "dlio", "vgicp_grav", "kiss_icp") if k in d]
    lab = dict(LABEL, vgicp_grav="VGICP-grav\n(LIO-seeded)")
    fig, ax = plt.subplots(figsize=(6.2, 3.2))
    for i, k in enumerate(order):
        v = np.asarray(d[k])
        col = C.get(k, C["pipe1"])
        ax.plot(np.full(len(v), i) + np.linspace(-0.12, 0.12, len(v)), v, "o", ms=5, color=col, mec=SURFACE, mew=1.0)
        ax.plot([i - 0.25, i + 0.25], [v.mean()] * 2, color=INK, lw=1.8)
    ax.set_xticks(range(len(order)), [lab[k] for k in order], fontsize=8)
    ax.set_ylabel("yaw-rate bias [deg/s]")
    ax.set_ylim(bottom=0)
    ax.set_title("Yaw-rate bias per 2 km block (dots) and mean (bar)", loc="left", fontsize=10)
    save(fig, "f5_bias_per_block.png")


def fig_map(cfg, S):
    gt = load_ground_truth(cfg)
    ecfg = dict(cfg["evaluation"], estimate_time_offset=False, head_align_m=200.0)
    base = S["faster_lio"]
    tracks = [("Faster-LIO (raw)", load_run("faster_lio", "full", cfg).shifted(base.dt), C["faster_lio"], "-"),
              ("+ gravity filter", H.to_traj("g", base.t, base.G.T, cfg).shifted(base.dt), C["pipe1"], "--"),
              ("+ heading from KISS-ICP", H.to_traj("h", base.t, H.integrate(base.t, base.u, base.x0, base.yaw0,
               dyaw_override=H.resample_dyaw(S["kiss_icp"].t, S["kiss_icp"].u[:, 2], base.t)), cfg).shifted(base.dt),
               C["pipe2"], "-.")]
    fig, ax = plt.subplots(figsize=(6.4, 5.6))
    x0, y0 = gt.p[0, 0], gt.p[0, 1]
    ax.plot(gt.p[:, 0] - x0, gt.p[:, 1] - y0, color=INK, lw=2.0, label="ground truth (GNSS)")
    end_label(ax, gt.p[:, 0] - x0, gt.p[:, 1] - y0, "GT", INK)
    for name, tr, col, st in tracks:
        ev = evaluate(tr, gt, "se2_head", ecfg)
        xy = ev.est_xy[ev.valid]
        ax.plot(xy[:, 0] - x0, xy[:, 1] - y0, color=col, ls=st, label=f"{name}: final error {ev.summary['final_error_m']:.0f} m")
        end_label(ax, xy[:, 0] - x0, xy[:, 1] - y0, name.replace("+ ", ""), col)
    ax.set_aspect("equal")
    ax.set_xlabel("east [m]")
    ax.set_ylabel("north [m]")
    ax.set_title("Whole drive, aligned on the first 200 m only", loc="left", fontsize=10)
    ax.legend(loc="best", fontsize=8)
    save(fig, "f0_map.png")


if __name__ == "__main__":
    cfg = load_config()
    which = sys.argv[1:] or ["map", "height", "heading", "probes", "duty", "bias"]
    S = v3.load_sources(cfg) if {"map", "heading"} & set(which) else None
    for w in which:
        {"map": lambda: fig_map(cfg, S), "height": lambda: fig_heights(cfg), "heading": lambda: fig_heading(cfg, S),
         "probes": lambda: fig_probes(cfg), "duty": lambda: fig_duty(cfg), "bias": lambda: fig_bias(cfg)}[w]()
