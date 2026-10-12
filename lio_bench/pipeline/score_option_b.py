"""Option B for every method: pure odometry, GNSS used for ONE heading at frame 1 only.

  python lio_bench/pipeline/score_option_b.py          (after scripts/run_option_b_all.sh)

For each LIO (Faster-LIO, FAST-LIO2, DLIO, GenZ-LIO) four variants, plus KISS-ICP alone:
  raw        the estimator's own poses
  gravity    + online gravity filter (IMU only, tau 5 s), tilt-free steps
  gk         + gravity + heading change from KISS-ICP (LiDAR only, 2.0 m voxels, runs/kiss_icp_vox20/optb)
  node       the online node run next to the LIO in real time (same rule, causal; 2.0 m voxels)
No yaw-bias calibration, no clock offset, no fit to GNSS. Placement: the whole track is rotated once
so its heading at the first pose equals the dual-antenna heading in row 0 of gps_left/right_position.csv,
and shifted so its first pose sits on the GNSS position at that time.

Metrics (horizontal, against groundtruth_extracted.csv at its own stamps):
  - option B ATE RMSE / mean / final error / end-to-end drift % (final / GT path length) /
    heading error at the end: the trajectory as placed above, nothing fitted afterwards;
  - KITTI relative error over 100-800 m segments: translation % and rotation deg/100 m. As in KITTI,
    each segment is compared after matching its start to the reference (start point + heading over
    the first 10 m); it says how well the odometry tracks a few hundred metres, not where it is;
  - RPE over 1 s (translation, same per-start matching), median m;
  - paper-style ATE: RMSE after the best rigid 2D fit over the WHOLE drive (what LIO papers print);
    evaluation only, uses GNSS everywhere; never applied to any CSV.
Pipeline CSVs (same format as export_option_b.py): results/export_optionB_all/.
Report + tables: results/optionB_all/SCORES.md, scores.json, figures/.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from pyproj import Transformer

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "lio_bench" / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import export_option_b as E  # noqa: E402
from liobench import export as X  # noqa: E402
from liobench.geometry import cumulative_distance, fit_alignment  # noqa: E402
from liobench.io import load_config, load_ground_truth, repo_path  # noqa: E402

OUT = REPO / "lio_bench" / "results" / "optionB_all"
CSV_OUT = REPO / "lio_bench" / "results" / "export_optionB_all"
KISS = ("kiss_icp_vox20", "optb")
LIOS = [("faster_lio", "Faster-LIO", "rerun_b", ("kiss_fusion", "rerun_b_fusion")),
        ("fast_lio", "FAST-LIO2", "optb", ("kiss_fusion_fast_lio", "optb")),
        ("dlio", "DLIO", "optb", ("kiss_fusion_dlio", "optb")),
        ("genz_lio", "GenZ-LIO", "optb", ("kiss_fusion_genz_lio", "optb"))]
VARIANTS = [("raw", "raw"), ("gravity", "+ gravity"), ("gk", "+ gravity + KISS heading (offline)"),
            ("node", "+ gravity + KISS heading (online node)")]
SEG = (100, 200, 300, 400, 500, 600, 700, 800)
POLES_KM = (0.72, 5.19)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def place(tr, yaw0, gt):
    """Option B: one rotation (antenna heading at the first pose) + one shift (GNSS position there)."""
    rot = yaw0 - E.yaw_of(tr.q[0])
    c, s = np.cos(rot), np.sin(rot)
    xy = (tr.p[:, :2] - tr.p[0, :2]) @ np.array([[c, -s], [s, c]]).T
    p0 = np.array([np.interp(tr.t[0], gt.t, gt.p[:, 0]), np.interp(tr.t[0], gt.t, gt.p[:, 1])])
    return xy, p0, float(np.degrees(rot))


def course(xy, dist, chord=10.0):
    """Direction of travel over the next `chord` metres of reference distance (NaN near the end)."""
    j = np.searchsorted(dist, dist + chord)
    ok = j < len(dist)
    d = np.full((len(dist), 2), np.nan)
    d[ok] = xy[j[ok]] - xy[ok]
    return np.arctan2(d[:, 1], d[:, 0])


def kitti(est, ref, dist, t):
    ce, cr = course(est, dist), course(ref, dist)
    trans, rot = [], []
    starts = np.arange(0, len(dist), 10)
    for L in SEG:
        j = np.searchsorted(dist, dist[starts] + L)
        ok = j < len(dist)
        i, j = starts[ok], j[ok]
        a = cr[i] - ce[i]
        de = est[j] - est[i]
        de = np.c_[np.cos(a) * de[:, 0] - np.sin(a) * de[:, 1], np.sin(a) * de[:, 0] + np.cos(a) * de[:, 1]]
        e = np.linalg.norm(de - (ref[j] - ref[i]), axis=1) / L * 100
        r = np.abs(wrap((ce[j] - ce[i]) - (cr[j] - cr[i]))) / L * 100
        g = np.isfinite(e) & np.isfinite(r)
        trans.append(e[g])
        rot.append(np.degrees(r[g]))
    # RPE over 1 s with the same per-start matching
    j = np.clip(np.searchsorted(t, t + 1.0), 0, len(t) - 1)
    ok = (np.abs(t[j] - t - 1.0) < 0.15) & np.isfinite(ce) & np.isfinite(cr) & (dist[j] - dist > 1.0)
    i, j = np.nonzero(ok)[0], j[ok]
    a = cr[i] - ce[i]
    de = est[j] - est[i]
    de = np.c_[np.cos(a) * de[:, 0] - np.sin(a) * de[:, 1], np.sin(a) * de[:, 0] + np.cos(a) * de[:, 1]]
    rpe = np.linalg.norm(de - (ref[j] - ref[i]), axis=1)
    return (float(np.mean(np.concatenate(trans))), float(np.mean(np.concatenate(rot))),
            float(np.median(rpe)), float(np.sqrt(np.mean(rpe ** 2))))


def score(tr, yaw0, gt, dist):
    xy, p0, rot = place(tr, yaw0, gt)
    inside = (gt.t >= tr.t[0]) & (gt.t <= tr.t[-1])
    t = gt.t[inside]
    ref = gt.p[inside, :2]
    d = dist[inside] - dist[inside][0]
    est = np.c_[np.interp(t, tr.t, xy[:, 0]), np.interp(t, tr.t, xy[:, 1])] + p0
    err = np.linalg.norm(est - ref, axis=1)
    ce, cr = course(est, d, 20.0), course(ref, d, 20.0)
    k = np.nonzero(np.isfinite(ce) & np.isfinite(cr))[0][-1]
    kt, kr, rpe_med, rpe_rmse = kitti(est, ref, d, t)
    R, t0, _, _ = fit_alignment(np.c_[est, np.zeros(len(est))], np.c_[ref, np.zeros(len(ref))], "se2_full", d, 200.0)
    paper = np.linalg.norm((np.c_[est, np.zeros(len(est))] @ R.T + t0)[:, :2] - ref, axis=1)
    at = {f"err_at_km{km}": float(np.interp(km * 1000, dist[inside], err)) for km in POLES_KM}
    return {"rotation_applied_deg": rot, "poses": len(tr.t),
            "ate_rmse_m": float(np.sqrt(np.mean(err ** 2))), "ate_mean_m": float(err.mean()),
            "ate_max_m": float(err.max()), "final_m": float(err[-1]),
            "end_to_end_drift_pct": float(100 * err[-1] / d[-1]), "path_km": float(d[-1] / 1000),
            "length_ratio": float(cumulative_distance(np.c_[est, np.zeros(len(est))])[-1] / d[-1]),
            "heading_err_end_deg": float(np.degrees(wrap(ce[k] - cr[k]))),
            "kitti_trans_pct": kt, "kitti_rot_deg_per_100m": kr, "rpe1s_median_m": rpe_med, "rpe1s_rmse_m": rpe_rmse,
            "paper_ate_rmse_m": float(np.sqrt(np.mean(paper ** 2))), **at}, (t, est, err, d)


def write_csv(tr, yaw0, name, cfg, meta):
    """Same rows / columns as export_option_b.export (anchored at the template's row 0)."""
    tmpl = pd.read_csv(repo_path(X.TEMPLATE))
    ts = tmpl["header.stamp.secs"].to_numpy(float) + tmpl["header.stamp.nsecs"].to_numpy(float) * 1e-9
    rot = yaw0 - E.yaw_of(tr.q[0])
    c, s = np.cos(rot), np.sin(rot)
    p = (tr.p[:, :2] - tr.p[0, :2]) @ np.array([[c, -s], [s, c]]).T
    e = tmpl["easting"].iloc[0] + np.interp(ts, tr.t, p[:, 0])
    n = tmpl["northing"].iloc[0] + np.interp(ts, tr.t, p[:, 1])
    z = np.interp(ts, tr.t, tr.p[:, 2])
    df = tmpl.copy()
    for col in X.CLEAR:
        if col in df:
            df[col] = np.nan
    lon, lat = Transformer.from_crs(f"EPSG:{cfg['utm_epsg']}", "EPSG:4326", always_xy=True).transform(e, n)
    df["easting"], df["northing"], df["latitude"], df["longitude"] = e, n, lat, lon
    df["altitude"] = tmpl["altitude"].iloc[0] + (z - z[0])
    out = CSV_OUT / f"incremental_navigation_{name}.csv"
    df.to_csv(out, index=False)
    out.with_suffix(".json").write_text(json.dumps(dict(meta, rows=len(df), clock_offset_s=0.0,
                                                        rotation_applied_deg=float(np.degrees(rot)),
                                                        start_heading="one dual-antenna GNSS reading at the first frame (option B)",
                                                        template=X.TEMPLATE), indent=2))
    return out


def trajectories(cfg):
    X.v3.SOURCES["kiss_icp"] = KISS
    for algo, label, tag, (node, ntag) in LIOS:
        for key, desc in VARIANTS:
            if key == "node":
                src, stag, pipe = node, ntag, "run"
            else:
                src, stag, pipe = algo, tag, {"raw": "raw", "gravity": "gravity", "gk": "gravity+kiss_heading"}[key]
            try:
                tr, _ = X.trajectory(cfg, src, stag, pipe)          # clock offset returned, not applied
            except Exception as ex:                                  # a run that is missing or failed
                print(f"  !! {label} {desc}: {ex}", flush=True)
                continue
            yield f"{algo}_{key}", label, desc, tr, {"source": src, "tag": stag, "pipeline": pipe,
                                                    "kiss_icp": f"{KISS[0]}/{KISS[1]}" if key in ("gk",) else None}
    tr, _ = X.trajectory(cfg, KISS[0], KISS[1], "raw")
    yield "kiss_icp_alone", "KISS-ICP", "alone (LiDAR only, 2.0 m)", tr, {"source": KISS[0], "tag": KISS[1], "pipeline": "raw"}


def figures(rows, curves, gt):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    col = {"raw": "#2a78d6", "gravity": "#eb6834", "gk": "#1baf7a", "node": "#6250d6"}
    plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                         "axes.grid": True, "grid.color": "#e4e3df", "text.color": "#0b0b0b",
                         "axes.labelcolor": "#52514e", "xtick.color": "#52514e", "ytick.color": "#52514e"})
    fd = OUT / "figures"
    fd.mkdir(exist_ok=True)
    # 1. error vs distance, one panel per LIO
    fig, axs = plt.subplots(2, 2, figsize=(10, 6.4), sharex=True, sharey=True)
    for ax, (algo, label, _, _) in zip(axs.flat, LIOS):
        for key, desc in VARIANTS:
            c = curves.get(f"{algo}_{key}")
            if c is not None:
                ax.plot(c[3] / 1000, c[2], color=col[key], lw=1.6, label=desc)
        c = curves.get("kiss_icp_alone")
        if c is not None:
            ax.plot(c[3] / 1000, c[2], color="#8a8984", lw=1.2, ls="--", label="KISS-ICP alone (reference)")
        for km in POLES_KM:
            ax.axvline(km, color="#b5b4ae", lw=0.8)
        ax.set_yscale("log")
        ax.set_ylim(0.5, 1e4)
        ax.set_title(label, loc="left", fontsize=10)
    for ax in axs[1]:
        ax.set_xlabel("distance driven [km]")
    for ax in axs[:, 0]:
        ax.set_ylabel("position error vs GNSS [m], log")
    h, lab = axs[0, 0].get_legend_handles_labels()
    fig.legend(h, lab, loc="lower center", ncol=3, frameon=False, fontsize=8)
    fig.suptitle("Option B (GNSS heading at frame 1 only): position error along the drive. Grey lines: first and last pole.",
                 x=0.01, ha="left", fontsize=10)
    fig.tight_layout(rect=(0, 0.07, 1, 0.96))
    fig.savefig(fd / "error_vs_distance.png", dpi=160)
    plt.close(fig)
    # 2. maps, one panel per LIO
    fig, axs = plt.subplots(2, 2, figsize=(10, 8))
    e0, n0 = gt.p[0, 0], gt.p[0, 1]
    for ax, (algo, label, _, _) in zip(axs.flat, LIOS):
        ax.plot((gt.p[:, 0] - e0) / 1000, (gt.p[:, 1] - n0) / 1000, color="#0b0b0b", lw=2.2, label="GNSS (reference)")
        for key, desc in VARIANTS:
            c = curves.get(f"{algo}_{key}")
            if c is not None:
                ax.plot((c[1][:, 0] - e0) / 1000, (c[1][:, 1] - n0) / 1000, color=col[key], lw=1.4, label=desc)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_title(label, loc="left", fontsize=10)
        ax.set_xlabel("east [km]")
        ax.set_ylabel("north [km]")
    h, lab = axs[0, 0].get_legend_handles_labels()
    fig.legend(h, lab, loc="lower center", ncol=3, frameon=False, fontsize=8)
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(fd / "maps.png", dpi=160)
    plt.close(fig)
    # 3./4. bars: KITTI translation % and end-to-end drift %, grouped by LIO (two charts, one axis each)
    for metric, ylab, fname in (("kitti_trans_pct", "KITTI translation error [%] (100-800 m)", "kitti_translation.png"),
                                ("end_to_end_drift_pct", "end-to-end drift [%] (final error / distance)", "end_to_end_drift.png")):
        fig, ax = plt.subplots(figsize=(8.5, 3.6))
        w = 0.2
        for k, (key, desc) in enumerate(VARIANTS):
            xs, vs = [], []
            for g, (algo, *_r) in enumerate(LIOS):
                r = rows.get(f"{algo}_{key}")
                if r:
                    xs.append(g + (k - 1.5) * w)
                    vs.append(r[metric])
            ax.bar(xs, vs, width=w - 0.03, color=col[key], label=desc)
        ka = rows.get("kiss_icp_alone")
        if ka:
            ax.axhline(ka[metric], color="#8a8984", ls="--", lw=1.2, label=f"KISS-ICP alone: {ka[metric]:.2f} %")
        ax.set_xticks(range(len(LIOS)), [l[1] for l in LIOS])
        ax.set_ylabel(ylab)
        ax.grid(axis="x", visible=False)
        ax.legend(fontsize=7.5, frameon=False, ncol=2)
        fig.tight_layout()
        fig.savefig(fd / fname, dpi=160)
        plt.close(fig)


def report(rows):
    L = ["# Option B for every method: pure odometry, GNSS heading at frame 1 only\n",
         "*`lio_bench/pipeline/score_option_b.py`. Every trajectory: one rotation so its heading at the first pose "
         "equals the dual-antenna heading (row 0 of `gps_left/right_position.csv`), one shift to the GNSS position "
         "there. No 200 m fit, no clock offset, no yaw-bias calibration, nothing fitted afterwards. KISS-ICP at 2.0 m "
         "voxels everywhere. Reference: `groundtruth_extracted.csv` (dual-antenna midpoint). Pipeline CSVs: "
         "`results/export_optionB_all/`.*\n",
         "## 1. Whole drive, as placed (nothing fitted after frame 1)\n",
         "| method | variant | ATE RMSE m | mean m | final m | end-to-end drift % | heading err at end ° | length ratio | first pole m | last pole m |",
         "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for name, r in rows.items():
        L.append(f"| {r['label']} | {r['variant']} | {r['ate_rmse_m']:.0f} | {r['ate_mean_m']:.0f} | {r['final_m']:.0f} | "
                 f"{r['end_to_end_drift_pct']:.1f} | {r['heading_err_end_deg']:+.1f} | {r['length_ratio']:.3f} | "
                 f"{r['err_at_km0.72']:.1f} | {r['err_at_km5.19']:.0f} |")
    L += ["\n## 2. Relative accuracy (KITTI-style, needs no placement) and paper-style ATE\n",
          "| method | variant | KITTI translation % | KITTI rotation °/100 m | RPE 1 s median m | RPE 1 s RMSE m | paper-style ATE m (whole-drive best fit, evaluation only) |",
          "|---|---|---:|---:|---:|---:|---:|"]
    for name, r in rows.items():
        L.append(f"| {r['label']} | {r['variant']} | {r['kitti_trans_pct']:.2f} | {r['kitti_rot_deg_per_100m']:.3f} | "
                 f"{r['rpe1s_median_m']:.2f} | {r['rpe1s_rmse_m']:.2f} | {r['paper_ate_rmse_m']:.0f} |")
    L += ["\n## 3. Start rotation applied and pipeline CSV\n", "| method | variant | rotation ° | poses | CSV |", "|---|---|---:|---:|---|"]
    for name, r in rows.items():
        L.append(f"| {r['label']} | {r['variant']} | {r['rotation_applied_deg']:+.2f} | {r['poses']} | `{r['csv']}` |")
    L += ["\n## Figures\n", "- `figures/error_vs_distance.png`: position error along the drive, one panel per LIO",
          "- `figures/maps.png`: tracks vs GNSS, one panel per LIO",
          "- `figures/kitti_translation.png`: KITTI translation error per method",
          "- `figures/end_to_end_drift.png`: end-to-end drift per method"]
    (OUT / "SCORES.md").write_text("\n".join(L) + "\n")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    CSV_OUT.mkdir(parents=True, exist_ok=True)
    cfg = load_config()
    gt = load_ground_truth(cfg)
    dist = cumulative_distance(gt.p)
    yaw0 = E.antenna_heading_enu()
    print(f"antenna heading at frame 1: ENU {np.degrees(yaw0):.2f} deg", flush=True)
    rows, curves = {}, {}
    for name, label, desc, tr, meta in trajectories(cfg):
        r, c = score(tr, yaw0, gt, dist)
        r.update(label=label, variant=desc, csv=str(write_csv(tr, yaw0, name, cfg, meta).relative_to(REPO)), **meta)
        rows[name], curves[name] = r, c
        print(f"{label:10s} {desc:40s} rot {r['rotation_applied_deg']:+6.2f}  ATE {r['ate_rmse_m']:7.0f}  final {r['final_m']:6.0f}  "
              f"e2e {r['end_to_end_drift_pct']:5.1f} %  KITTI {r['kitti_trans_pct']:5.2f} % {r['kitti_rot_deg_per_100m']:.3f} deg/100m  "
              f"paper ATE {r['paper_ate_rmse_m']:6.0f}  poles {r['err_at_km0.72']:.1f}/{r['err_at_km5.19']:.0f}", flush=True)
    (OUT / "scores.json").write_text(json.dumps(rows, indent=2))
    report(rows)
    figures(rows, curves, gt)


if __name__ == "__main__":
    main()
