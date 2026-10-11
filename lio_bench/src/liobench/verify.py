"""Step 1: are fastlio.csv / fasterlio.csv accurate?

Three independent checks:
  A. Each legacy export (both position columns) against groundtruth_extracted.csv,
     as stored ('none') and under the same alignment protocols our runs get.
  B. Each legacy export against our own re-run of that algorithm on the bag:
     after a rigid planar fit, do the two describe the same motion?
     If yes, the export is faithful and any error is the algorithm's.
     If no, the export (or the run that produced it) is not what the algorithm does.
  C. The ground truth itself against the dual-GNSS midpoint.
"""
from __future__ import annotations

import json

import numpy as np
import pandas as pd

from . import plots
from .geometry import cumulative_distance, fit_alignment, interpolate, level
from .io import (Trajectory, available_runs, latlon_to_utm, load_config, load_ground_truth, load_legacy_export,
                 load_run, repo_path, results_dir)
from .metrics import estimate_time_offset, evaluate, stats

LEGACY_TO_RUN = {"fastlio": "fast_lio", "fasterlio": "faster_lio"}
# extra re-runs reported alongside the "full" ones: FAST-LIO with the legacy point_filter_num 4
EXTRA_RERUNS = [("fast_lio", "full_pf4_legacy")]


def gnss_midpoint(cfg: dict) -> Trajectory:
    l = pd.read_csv(repo_path(cfg["gnss_left"]))
    r = pd.read_csv(repo_path(cfg["gnss_right"]))
    tl = l["header.stamp.secs"].to_numpy(float) + l["header.stamp.nsecs"].to_numpy(float) * 1e-9
    tr = r["header.stamp.secs"].to_numpy(float) + r["header.stamp.nsecs"].to_numpy(float) * 1e-9
    el, nl = latlon_to_utm(l["latitude"].to_numpy(float), l["longitude"].to_numpy(float), cfg["utm_epsg"])
    er, nr = latlon_to_utm(r["latitude"].to_numpy(float), r["longitude"].to_numpy(float), cfg["utm_epsg"])
    e = 0.5 * (el + np.interp(tl, tr, er))
    n = 0.5 * (nl + np.interp(tl, tr, nr))
    return Trajectory("gnss_midpoint", tl, np.c_[e, n, l["altitude"].to_numpy(float)])


def compare_legacy_to_rerun(legacy: Trajectory, rerun: Trajectory, ecfg: dict) -> dict:
    """Same motion? Rigid planar fit of the re-run onto the export, then residuals."""
    rerun = level(rerun)
    off = estimate_time_offset(rerun, legacy)
    rr = rerun.shifted(off["dt_s"])
    p, _, valid = interpolate(rr, legacy.t, ecfg.get("max_assoc_dt", 0.15))
    d = cumulative_distance(legacy.p)
    out = {"time_offset": off, "coverage": float(valid.mean())}
    if valid.sum() < 10:
        return out
    R, t, s, info = fit_alignment(p, legacy.p, "se2_full", d)
    pa = p @ R.T + t
    res = np.linalg.norm(pa[:, :2] - legacy.p[:, :2], axis=1)[valid]
    # Motion agreement independent of any alignment: per-step distance travelled.
    step_l = np.linalg.norm(np.diff(legacy.p[:, :2], axis=0), axis=1)
    step_r = np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1)
    ok = valid[1:] & valid[:-1]
    out.update(alignment=info, residual_m=stats(res),
               step_length_diff_m=stats(np.abs(step_l - step_r)[ok]),
               path_length_ratio_legacy_over_rerun=float(step_l[ok].sum() / max(step_r[ok].sum(), 1e-9)))
    return out


def _fmt(x, nd=1):
    return "—" if x is None or (isinstance(x, float) and not np.isfinite(x)) else f"{x:.{nd}f}"


def run(cfg: dict | None = None) -> dict:
    cfg = cfg or load_config()
    ecfg = cfg["evaluation"]
    out = results_dir("verify")
    gt = load_ground_truth(cfg)
    protocols = ecfg["protocols"]

    trajs: list[tuple[Trajectory, list[str]]] = []
    for name in cfg["legacy_exports"]:
        for col in ("latlon", "stored_en"):
            trajs.append((load_legacy_export(cfg, name, col), ["none"] + protocols))
    for algo in available_runs("full"):
        tr = load_run(algo, "full", cfg)
        if tr is not None:
            trajs.append((tr, protocols))
    for algo, tag in EXTRA_RERUNS:
        tr = load_run(algo, tag, cfg)
        if tr is not None:
            trajs.append((tr, protocols))

    report = {"ground_truth": {}, "evaluations": {}, "legacy_vs_rerun": {}}

    # C. ground-truth sanity
    mid = gnss_midpoint(cfg)
    ev = evaluate(mid, gt, "none", dict(ecfg, estimate_time_offset=True))
    report["ground_truth"] = {"vs_gnss_midpoint": {k: ev.summary[k] for k in ("ate_xy", "time_offset", "coverage")},
                              "length_m": float(cumulative_distance(gt.p)[-1]), "samples": len(gt),
                              "duration_s": float(gt.t[-1] - gt.t[0])}

    # A. everything against GT
    evals = {}
    for tr, prots in trajs:
        for prot in prots:
            try:
                e = evaluate(tr, gt, prot, ecfg)
            except Exception as exc:  # noqa: BLE001 - report and continue
                report["evaluations"][f"{tr.name}|{prot}"] = {"error": str(exc)}
                continue
            evals[(tr.name, prot)] = e
            report["evaluations"][f"{tr.name}|{prot}"] = e.summary

    # B. legacy vs our re-runs
    for legacy_name, algo in LEGACY_TO_RUN.items():
        for tag in ["full"] + [t for a, t in EXTRA_RERUNS if a == algo]:
            rerun = load_run(algo, tag, cfg)
            key = legacy_name if tag == "full" else f"{legacy_name}_vs_{tag}"
            if rerun is None:
                report["legacy_vs_rerun"][key] = f"no re-run {algo}/{tag} yet (see scripts/run_all.sh)"
                continue
            for col in ("latlon", "stored_en"):
                report["legacy_vs_rerun"][f"{key}_{col}"] = compare_legacy_to_rerun(
                    load_legacy_export(cfg, legacy_name, col), rerun, ecfg)

    (out / "verify_summary.json").write_text(json.dumps(report, indent=2, default=float))
    _write_markdown(report, evals, out)
    _plots(gt, evals, out, protocols)
    return report


def _write_markdown(report, evals, out):
    L = ["# Step 1 - verification of the legacy exports\n"]
    g = report["ground_truth"]
    gv = g["vs_gnss_midpoint"]
    L.append(f"Ground truth: {g['samples']} samples, {g['duration_s']:.0f} s, {g['length_m']:.0f} m. "
             f"Distance to the dual-GNSS midpoint: median {_fmt(gv['ate_xy'].get('median'), 2)} m, "
             f"p95 {_fmt(gv['ate_xy'].get('p95'), 2)} m (clock offset {gv['time_offset']['dt_s']*1e3:.0f} ms).\n")
    L.append("## A. Against ground truth (horizontal)\n")
    L.append("| trajectory | protocol | ATE RMSE m | ATE median m | final m | drift % (KITTI mean) | RPE 1 s median m | length ratio | clock dt ms |")
    L.append("|---|---|---:|---:|---:|---:|---:|---:|---:|")
    for (name, prot), e in evals.items():
        s = e.summary
        L.append(f"| {name} | {prot} | {_fmt(s['ate_xy'].get('rmse'))} | {_fmt(s['ate_xy'].get('median'))} | "
                 f"{_fmt(s['final_error_m'])} | {_fmt(s['drift_pct_mean_kitti'], 2)} | "
                 f"{_fmt(s['rpe_time']['1s'].get('median'), 2)} | {_fmt(s['path_length_m']['ratio'], 3)} | "
                 f"{_fmt(s.get('time_offset', {}).get('dt_s', np.nan) * 1e3, 0)} |")
    L.append("\n`none` = as stored in UTM 33N. `se2_head` = rigid fit on the first 200 m only, then pure prediction. "
             "`se2_full`/`se3_full` = rigid fit on the whole drive (standard benchmark ATE). "
             "Drift % is frame-free (each segment anchored at its own start).\n")
    L.append("## B. Legacy export vs our re-run of the same algorithm\n")
    for k, v in report["legacy_vs_rerun"].items():
        if isinstance(v, str):
            L.append(f"- **{k}**: {v}")
        elif "residual_m" in v:
            L.append(f"- **{k}**: residual after rigid planar fit median {_fmt(v['residual_m']['median'])} m, "
                     f"max {_fmt(v['residual_m']['max'])} m; per-step length difference median "
                     f"{_fmt(v['step_length_diff_m']['median'], 3)} m; length ratio legacy/rerun "
                     f"{_fmt(v['path_length_ratio_legacy_over_rerun'], 3)}; clock offset {v['time_offset']['dt_s']:.3f} s.")
        else:
            L.append(f"- **{k}**: not enough overlap ({v})")
    L.append("\nSmall residual and length ratio ≈ 1: the CSV is a faithful record of what the algorithm does. "
             "Large residual: the CSV does not describe the algorithm's motion (export, clock or config problem).")
    (out / "verify_summary.md").write_text("\n".join(L) + "\n")


def _plots(gt, evals, out, protocols):
    for prot in ["none", "se2_head", "se2_full"]:
        tracks = {name: e.est_xy for (name, p), e in evals.items() if p == prot}
        if tracks:
            plots.map_overlay(gt.p, tracks, out / f"map_{prot}.png", f"trajectories ({prot})")
    for prot in ["se2_head", "se2_full"]:
        ys = {name: e.err for (name, p), e in evals.items() if p == prot}
        if ys:
            any_e = next(e for (n, p), e in evals.items() if p == prot)
            plots.series(any_e.dist / 1000, ys, out / f"error_vs_distance_{prot}.png", "distance [km]",
                         "horizontal error [m]", f"error vs distance ({prot})", logy=True)
