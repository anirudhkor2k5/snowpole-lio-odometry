"""Figures for the snow-pole pipeline results (REPORT_v3 §12). Reads only files already written by
run_all_variants.py, rate_sensitivity.py and validate_against_original.py; writes
results/pipeline/figures/p*.png.

  python lio_bench/pipeline/make_pipeline_figures.py
"""
from __future__ import annotations

import json
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results" / "pipeline"
FIG = RES / "figures"
LOG = ROOT / "results" / "logs" / "original_pipeline.log"

INPUTS = [  # label, legend, colour
    ("fastlio_raw", "FAST-LIO2 raw (baseline)", "#eb6834"),
    ("fasterlio_raw", "Faster-LIO raw", "#2a78d6"),
    ("fastlio_g", "FAST-LIO2 + gravity", "#e3a21a"),
    ("fasterlio_g", "Faster-LIO + gravity", "#5aa9e6"),
    ("fastlio_gk", "FAST-LIO2 + gravity + KISS", "#0f7f86"),
    ("fasterlio_gk", "Faster-LIO + gravity + KISS (offline)", "#1baf7a"),
    ("final_online", "final online node", "#6250d6"),
    ("legacy_csv", "legacy CSV (reference only)", "#9a9a94"),
]
NAME = {k: v for k, v, _ in INPUTS}
COL = {k: c for k, _, c in INPUTS}
MODES = [("none", "original pipeline"), ("reset", "+ heading reset"), ("reset_rate", "+ heading reset + drift rate")]
KEY = ["fastlio_raw", "fasterlio_raw", "fastlio_gk", "final_online"]


def frames(lab, mode):
    return pd.read_csv(RES / f"{lab}__{mode}_frames.csv")


def summary(lab, mode):
    return json.loads((RES / f"{lab}__{mode}_summary.json").read_text())


def style():
    plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                         "axes.grid": True, "grid.color": "#e4e3df", "grid.linewidth": 0.6,
                         "legend.frameon": False, "savefig.dpi": 170})


def span():
    p = summary("fastlio_raw", "none")["pole_span"]
    return p["from_km"], p["to_km"]


def p1_validation():
    txt = LOG.read_text(errors="ignore")
    orig = np.array([float(x) for x in re.findall(
        r"Average Error between orignal and predicted gnss: (\d+\.\d+(?:[eE][+-]?\d+)?)", txt)])
    d = frames("legacy_csv", "none")
    e = d.err_m[d.in_box].values
    ours = np.cumsum(e) / np.arange(1, len(e) + 1)
    n = min(len(orig), len(ours))
    fig, ax = plt.subplots(1, 2, figsize=(9, 3.2))
    ax[0].plot(orig[:n], color="#222", lw=2.4, label="original script (unmodified)")
    ax[0].plot(ours[:n], color="#eb6834", lw=1.0, ls="--", label="re-implementation")
    ax[0].set(xlabel="frame inside pole bounding box", ylabel="running average error [m]",
              title=f"Native metric: {orig[n - 1]:.5f} vs {ours[n - 1]:.5f} m ({n} frames)")
    ax[0].legend()
    diff = np.abs(orig[:n] - ours[:n])
    ax[1].semilogy(np.maximum(diff, 1e-12), color="#6250d6", lw=0.8)
    ax[1].set(xlabel="frame inside pole bounding box", ylabel="|difference| [m]",
              title=f"max difference {diff.max():.1e} m")
    for a in ax:
        a.title.set_fontsize(9)
    fig.suptitle("P1. Re-implementation vs the original snowpole_based_vehicle_localization.py (legacy CSV input)",
                 x=0.01, ha="left", fontsize=10)
    fig.tight_layout()
    fig.savefig(FIG / "p1_validation.png")
    plt.close(fig)
    return n, float(diff.max())


def p2_error_vs_distance():
    a, b = span()
    fig, axs = plt.subplots(3, 1, figsize=(8, 8), sharex=True, sharey=True)
    for ax, (mode, mdesc) in zip(axs, MODES):
        ax.axvspan(a, b, color="#eeeeea", zorder=0)
        for lab in KEY:
            d = frames(lab, mode)
            ax.plot(d.s_km, np.maximum(d.err_m, 0.05), color=COL[lab], lw=1.1, label=NAME[lab])
        ax.set_yscale("log")
        ax.set_ylim(0.1, 5000)
        ax.set_ylabel("position error [m]")
        ax.set_title(mdesc, loc="left", fontsize=9)
    h, l = axs[0].get_legend_handles_labels()
    fig.legend(h, l, loc="lower center", ncol=4, fontsize=8)
    axs[-1].set_xlabel("distance along the drive [km]")
    fig.suptitle("P2. Pipeline position error along the 10.3 km drive (shaded: first → last pole)", x=0.01, ha="left", fontsize=10)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(FIG / "p2_error_vs_distance.png")
    plt.close(fig)


def p3_pole_span():
    a, b = span()
    fig, axs = plt.subplots(2, 1, figsize=(8, 5.2), sharex=True, sharey=True)
    for ax, mode in zip(axs, ["none", "reset"]):
        for lab in ["fastlio_raw", "final_online"]:
            d = frames(lab, mode)
            m = (d.s_km >= a) & (d.s_km <= b)
            st = summary(lab, mode)["pole_span"]
            ax.plot(d.s_km[m], d.err_m[m], color=COL[lab], lw=0.9,
                    label=f"{NAME[lab]}: mean {st['mean']:.1f} m, median {st['median']:.1f} m")
        ax.set_ylabel("position error [m]")
        ax.set_title(dict(MODES)[mode], loc="left", fontsize=9)
        ax.legend(loc="upper right", fontsize=8)
    axs[-1].set_xlabel("distance along the drive [km]")
    fig.suptitle(f"P3. First pole → last pole (km {a:.2f}–{b:.2f}), linear scale", x=0.01, ha="left", fontsize=10)
    fig.tight_layout()
    fig.savefig(FIG / "p3_pole_span.png")
    plt.close(fig)


def p4_summary_bars():
    labs = [k for k, _, _ in INPUTS]
    panels = [("whole_drive", "mean", "whole drive: mean [m]", True),
              ("whole_drive", "final_m", "whole drive: final [m]", True),
              ("pole_span", "mean", "first → last pole: mean [m]", False),
              ("pole_span", "max", "first → last pole: max [m]", False)]
    shades = {"none": 1.0, "reset": 0.6, "reset_rate": 0.3}
    fig, axs = plt.subplots(2, 2, figsize=(10, 6.4))
    y = np.arange(len(labs))
    h = 0.27
    for ax, (blk, key, title, log) in zip(axs.flat, panels):
        for j, (mode, mdesc) in enumerate(MODES):
            v = [summary(l, mode)[blk][key] for l in labs]
            bars = ax.barh(y + (j - 1) * h, v, h, color=[COL[l] for l in labs], alpha=shades[mode],
                           edgecolor="#333", lw=0.3, label=mdesc)
            for bb, val in zip(bars, v):
                ax.text(bb.get_width() * (1.08 if log else 1.0) + (0 if log else 0.2), bb.get_y() + h / 2,
                        f"{val:.0f}" if val >= 10 else f"{val:.1f}", va="center", fontsize=6)
        if log:
            ax.set_xscale("log")
        ax.set_yticks(y, [NAME[l] for l in labs], fontsize=7)
        ax.invert_yaxis()
        ax.set_title(title, loc="left", fontsize=9)
    handles = [plt.Rectangle((0, 0), 1, 1, color="#555", alpha=shades[m]) for m, _ in MODES]
    fig.legend(handles, [d for _, d in MODES], loc="lower center", ncol=3, fontsize=8)
    fig.suptitle("P4. All 24 runs (bar shade = last stage: dark original, mid + heading reset, light + drift rate)",
                 x=0.01, ha="left", fontsize=10)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(FIG / "p4_summary_bars.png")
    plt.close(fig)


def p5_rate_sensitivity():
    r = json.loads((RES / "rate_sensitivity.json").read_text())
    fig, ax = plt.subplots(figsize=(9, 3.8))
    for lab in ["fastlio_raw", "fasterlio_raw", "fastlio_gk", "fasterlio_gk", "final_online"]:
        pts = sorted((float(k.split("|")[2]), v[1]) for k, v in r.items()
                     if k.startswith(lab + "|None|"))
        o, f = zip(*pts)
        ax.plot(o, f, "o-", color=COL[lab], ms=3.5, lw=1.2, label=NAME[lab])
    ax.axvspan(-0.0021, 0.0021, color="#6250d6", alpha=0.08, lw=0, label="±1 SE of the online node's learned rate")
    ax.axvline(0, color="#999", lw=0.8)
    ax.set_yscale("log")
    ax.set(xlabel="offset added to the learned drift rate [°/s]", ylabel="final error at km 10.3 [m]")
    ax.set_title("P5. After the last pole: sensitivity to the drift-rate estimate", loc="left", fontsize=10)
    ax.legend(fontsize=7, loc="upper left", bbox_to_anchor=(1.01, 1))
    fig.tight_layout()
    fig.savefig(FIG / "p5_rate_sensitivity.png")
    plt.close(fig)


def p6_map():
    a, b = span()
    ref = frames("final_online", "none")
    e0, n0 = ref.ref_e.iloc[0], ref.ref_n.iloc[0]
    fig, axs = plt.subplots(1, 2, figsize=(10, 5.4), sharex=True, sharey=True)
    for ax, mode in zip(axs, ["none", "reset_rate"]):
        ax.plot((ref.ref_e - e0) / 1000, (ref.ref_n - n0) / 1000, color="#222", lw=2.6, label="GNSS reference")
        m = (ref.s_km >= a) & (ref.s_km <= b)
        ax.plot((ref.ref_e[m] - e0) / 1000, (ref.ref_n[m] - n0) / 1000, color="#c9c9c2", lw=6, zorder=0,
                label="pole stretch")
        for lab in ["fastlio_raw", "final_online"]:
            d = frames(lab, mode)
            ax.plot((d.pred_e - e0) / 1000, (d.pred_n - n0) / 1000, color=COL[lab], lw=1.1,
                    label=f"{NAME[lab]} (final {d.err_m.iloc[-1]:.0f} m)")
        ax.set_aspect("equal", adjustable="box")
        ax.set(xlabel="east [km]", ylabel="north [km]")
        ax.set_title(dict(MODES)[mode], loc="left", fontsize=9)
        ax.legend(fontsize=7, loc="best")
    fig.suptitle("P6. Pipeline track vs GNSS reference (UTM 33N, origin at the start)", x=0.01, ha="left", fontsize=10)
    fig.tight_layout()
    fig.savefig(FIG / "p6_map.png")
    plt.close(fig)


def p7_cross_track():
    """Cross-track error in the pole stretch vs the opposite-side pole offset (wrong-side association)."""
    from scipy.spatial import cKDTree
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import snowpole_stage3 as S
    gu = S._import_geoloc_utils()
    r = gu.load_gnss_data_and_calculate_utm_range(str(S.POLES))
    P = np.c_[r[1], r[0]]
    a, b = span()

    def cross(d):
        tx, ty = np.gradient(d.ref_e.values), np.gradient(d.ref_n.values)
        L = np.hypot(tx, ty) + 1e-9
        return (-(d.pred_e - d.ref_e) * ty + (d.pred_n - d.ref_n) * tx) / L

    ref = frames("fastlio_raw", "none")
    R = np.c_[ref.ref_e, ref.ref_n]
    _, i = cKDTree(R).query(P)
    i = np.clip(i, 1, len(R) - 2)
    t = R[i + 1] - R[i - 1]
    t /= np.linalg.norm(t, axis=1)[:, None]
    side = t[:, 0] * (P - R[i])[:, 1] - t[:, 1] * (P - R[i])[:, 0]
    width = np.median(side[side > 0]) - np.median(side[side < 0])
    fig, axs = plt.subplots(3, 1, figsize=(8, 6.4), sharex=True, sharey=True)
    for ax, (lab, mode) in zip(axs, [("fastlio_raw", "none"), ("final_online", "none"), ("final_online", "reset")]):
        d = frames(lab, mode)
        m = (d.s_km >= a) & (d.s_km <= b)
        c = cross(d)
        ax.axhspan(width - 2, width + 2, color="#f3d9c9", lw=0)
        ax.axhline(0, color="#999", lw=0.8)
        ax.plot(d.s_km[m], c[m], color=COL[lab], lw=0.9)
        locked = (m & (c.abs() > 5)).sum() / m.sum()
        ax.set_title(f"{NAME[lab]}, {dict(MODES)[mode]}: |cross-track| > 5 m in {100 * locked:.0f} % of frames",
                     loc="left", fontsize=9)
        ax.set_ylabel("cross-track [m]\n(+ = left)")
    axs[0].text(a + 0.02, width + 2.3, f"pole-pair spacing across the road ≈ {width:.1f} m", fontsize=7, color="#b5651d")
    axs[-1].set_xlabel("distance along the drive [km]")
    fig.suptitle("P7. Pole stretch: the ~10 m floor of the original pipeline is cross-track (wrong-side pole)",
                 x=0.01, ha="left", fontsize=10)
    fig.tight_layout()
    fig.savefig(FIG / "p7_cross_track.png")
    plt.close(fig)
    return float(width)


def main():
    style()
    FIG.mkdir(parents=True, exist_ok=True)
    n, dmax = p1_validation()
    print(f"validation: {n} frames, max |diff| {dmax:.2e} m")
    p2_error_vs_distance()
    p3_pole_span()
    p4_summary_bars()
    p5_rate_sensitivity()
    p6_map()
    print(f"pole-pair spacing {p7_cross_track():.1f} m")
    print("wrote", sorted(p.name for p in FIG.glob("p*.png")))


if __name__ == "__main__":
    main()
