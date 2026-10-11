"""Step 3: where and why do the baselines go wrong?

Errors are measured on body-frame increments (frame-free, no alignment), over
short windows so that one early mistake does not colour the rest of the drive:
  scale_err  sum(forward_m) / sum(forward_gt) - 1      (along-track stretch/shrink)
  lat_err    sum(lateral_m - lateral_gt) / length      (cross-track slip)
  yaw_err    sum(dyaw_m) - sum(dyaw_gt)  [deg]         (heading drift)
  trans_err  mean |u_m - u_gt| per step [m]            (overall)
Then: Spearman correlation of each context feature with each baseline's window
error, error by condition regime, best method per window, and maps.
"""
from __future__ import annotations

import json

import numpy as np
import pandas as pd
from scipy.stats import spearmanr

from . import plots
from .dataset import StepTable, build
from .io import load_config, results_dir

FEATURE_PREFIXES = ("scan_", "can_", "genz_")


def window_errors(st: StepTable, window_s: float = 10.0) -> pd.DataFrame:
    w = np.floor((st.t - st.t[0]) / window_s).astype(int)
    rows = []
    for k in np.unique(w):
        m = (w == k) & st.valid
        if m.sum() < 0.5 * window_s * 10:
            continue
        g = st.gt[m]
        L = float(np.abs(g[:, 0]).sum())
        row = {"window": int(k), "t0": float(st.t[m][0]), "length_m": L,
               "x": float(st.gt_xy[m][0, 0]), "y": float(st.gt_xy[m][0, 1]),
               "gt_speed": L / window_s, "gt_turn_deg": float(np.degrees(np.abs(g[:, 2].sum())))}
        for name, u in st.inc.items():
            mm = np.isfinite(u[m]).all(1)
            if mm.mean() < 0.8 or L < 5:
                continue
            um, gm = u[m][mm], g[mm]
            row[f"{name}__scale_err"] = float(um[:, 0].sum() / max(gm[:, 0].sum(), 1e-6) - 1)
            row[f"{name}__lat_err"] = float((um[:, 1] - gm[:, 1]).sum() / L)
            row[f"{name}__yaw_err_deg"] = float(np.degrees(um[:, 2].sum() - gm[:, 2].sum()))
            row[f"{name}__trans_err"] = float(np.linalg.norm(um[:, :2] - gm[:, :2], axis=1).mean())
        for c in st.ctx.columns:
            if c.startswith(FEATURE_PREFIXES):
                row[c] = float(np.nanmean(st.ctx[c].to_numpy()[m])) if np.isfinite(st.ctx[c].to_numpy()[m]).any() else np.nan
        rows.append(row)
    return pd.DataFrame(rows)


def run(cfg: dict | None = None, window_s: float = 10.0) -> dict:
    cfg = cfg or load_config()
    out = results_dir("analysis")
    st = build(cfg)
    if not st.inc:
        raise RuntimeError("no runs found; run the baselines first (see README)")
    W = window_errors(st, window_s)
    W.to_csv(out / "window_errors.csv", index=False)
    methods = st.methods
    feats = [c for c in W.columns if c.startswith(FEATURE_PREFIXES)]
    report = {"methods": methods, "mount_yaw_deg": {k: float(np.degrees(v)) for k, v in st.mount_yaw.items()},
              "time_offsets_s": st.meta["time_offsets_s"], "windows": len(W), "per_method": {}, "correlations": {},
              "regimes": {}, "best_method_share": {}}

    # Overall per-method error summaries.
    for m in methods:
        cols = [f"{m}__{k}" for k in ("scale_err", "lat_err", "yaw_err_deg", "trans_err")]
        if cols[0] not in W:
            continue
        report["per_method"][m] = {k.split("__")[1]: {"median": float(W[k].median()), "p90_abs": float(W[k].abs().quantile(0.9))}
                                   for k in cols}

    # Which conditions go with which errors (Spearman, |rho| ranked).
    for m in methods:
        for err in ("trans_err", "scale_err", "yaw_err_deg"):
            col = f"{m}__{err}"
            if col not in W:
                continue
            target = W[col].abs() if err != "trans_err" else W[col]
            res = []
            for f in feats + ["gt_speed", "gt_turn_deg"]:
                ok = W[f].notna() & target.notna()
                if ok.sum() < 10 or W.loc[ok, f].nunique() < 3:
                    continue
                rho, p = spearmanr(W.loc[ok, f], target[ok])
                res.append({"feature": f, "rho": float(rho), "p": float(p)})
            report["correlations"][col] = sorted(res, key=lambda r: -abs(r["rho"]))[:12]

    # Error by regime (tertiles of key conditions).
    for f in ["gt_speed", "gt_turn_deg", "scan_trans_cond", "scan_return_ratio", "scan_range_p50", "scan_planar_frac"]:
        if f not in W or W[f].nunique() < 3:
            continue
        bins = pd.qcut(W[f], 3, labels=["low", "mid", "high"], duplicates="drop")
        tab = {}
        for m in methods:
            col = f"{m}__trans_err"
            if col in W:
                tab[m] = W.groupby(bins, observed=True)[col].median().round(4).to_dict()
        report["regimes"][f] = {"edges": W.groupby(bins, observed=True)[f].agg(["min", "max"]).round(3).to_dict("index"), "median_trans_err": tab}

    # Best method per window.
    errc = [f"{m}__trans_err" for m in methods if f"{m}__trans_err" in W]
    if errc:
        has = W[errc].notna().any(axis=1)          # windows where no method has an error (gaps) get no winner
        best = W.loc[has, errc].idxmin(axis=1).str.replace("__trans_err", "")
        W["best_method"] = best.reindex(W.index)
        report["best_method_share"] = best.value_counts(normalize=True).round(3).to_dict()

    (out / "analysis_summary.json").write_text(json.dumps(report, indent=2, default=float))
    _plots(st, W, methods, out)
    _markdown(report, out)
    return report


def _plots(st, W, methods, out):
    xy = W[["x", "y"]].to_numpy()
    for m in methods:
        col = f"{m}__trans_err"
        if col in W:
            plots.colored_map(xy, W[col], out / f"map_{m}_step_error.png", "mean step error [m]", f"{m}: where it goes wrong")
    if "best_method" in W:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(9, 7))
        ax.plot(st.gt_xy[:, 0], st.gt_xy[:, 1], color="0.8", lw=4, zorder=0)
        for m in methods:
            s = W["best_method"] == m
            ax.scatter(W.loc[s, "x"], W.loc[s, "y"], s=18, color=plots.color_for(m), label=m)
        ax.set_aspect("equal"); ax.legend(); ax.set_title("best method per 10 s window")
        plots.save(fig, out / "map_best_method.png")
    ys = {m: W[f"{m}__scale_err"] * 100 for m in methods if f"{m}__scale_err" in W}
    if ys:
        plots.series(W["t0"] - W["t0"].iloc[0], ys, out / "scale_error_vs_time.png", "time [s]",
                     "along-track scale error [%]", "forward distance over/under-estimation per window")
    ys = {m: W[f"{m}__yaw_err_deg"] for m in methods if f"{m}__yaw_err_deg" in W}
    if ys:
        plots.series(W["t0"] - W["t0"].iloc[0], ys, out / "yaw_error_vs_time.png", "time [s]",
                     "heading drift per window [deg]", "yaw drift per window")


def _markdown(rep, out):
    L = ["# Step 3 - weakness analysis\n",
         f"{rep['windows']} windows. Mounting yaw removed (deg): {json.dumps({k: round(v, 2) for k, v in rep['mount_yaw_deg'].items()})}\n",
         "## Errors per method (10 s windows)\n",
         "| method | scale err median | scale |p90| | lateral median | yaw drift median deg | step err median m |",
         "|---|---:|---:|---:|---:|---:|"]
    for m, d in rep["per_method"].items():
        L.append(f"| {m} | {d['scale_err']['median']*100:.2f}% | {d['scale_err']['p90_abs']*100:.2f}% | "
                 f"{d['lat_err']['median']*100:.2f}% | {d['yaw_err_deg']['median']:.2f} | {d['trans_err']['median']:.3f} |")
    L.append("\n## What the errors go with (Spearman rho, top 6)\n")
    for k, lst in rep["correlations"].items():
        L.append(f"- **{k}**: " + ", ".join(f"{r['feature']} ({r['rho']:+.2f})" for r in lst[:6]))
    L.append("\n## Median step error by regime (tertiles)\n")
    for f, d in rep["regimes"].items():
        L.append(f"- **{f}** edges {d['edges']}: {d['median_trans_err']}")
    L.append(f"\n## Best method share of windows\n\n{rep['best_method_share']}\n")
    (out / "analysis_summary.md").write_text("\n".join(L) + "\n")
