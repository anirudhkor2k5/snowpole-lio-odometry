"""Which export step makes the raw Faster-LIO CSV score ~10 m in the pipeline?

Rebuilds the pipeline CSV from the raw Faster-LIO trajectory (runs/faster_lio/full, untouched
estimator poses) with plain numpy, changing one step at a time. Each CSV goes through the stage-3
loop in its original mode ("none" = the professor's pipeline, validated frame by frame against the
original script). Prints the pipeline's own in-box metric (mean / median), which is what the
original script reports.

  python lio_bench/verify/export_ablation.py        (from the repo root)
"""
import contextlib
import io
import sys
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd
from pyproj import Transformer

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "lio_bench" / "pipeline"))
import snowpole_stage3 as S  # noqa: E402

TUM = REPO / "lio_bench" / "results" / "runs" / "faster_lio" / "full" / "trajectory_tum.txt"
OURS = REPO / "lio_bench" / "results" / "export" / "incremental_navigation_faster_lio_full_raw.csv"
TEMPLATE = REPO / "incremental_navigation_results.csv"
TO_WGS = Transformer.from_crs("EPSG:32633", "EPSG:4326", always_xy=True)


def se2_fit(src, dst):
    ms, md = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((src - ms).T @ (dst - md))
    R = (U @ Vt).T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = (U @ Vt).T
    return R, md - R @ ms


def rot(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


def build(t, xy, gt_t, gt_xy, *, clock=0.06, head_m=200.0, yaw_err=0.0, rows="template", jitter=0.0, hold_every=0, scale=1.0):
    """Raw poses -> pipeline CSV. clock: s added to estimator stamps; head_m: length of the start fit;
    yaw_err: extra rotation (deg) after the fit; rows: 'template' (one per GNSS row) or 'scans'."""
    t = t + clock
    m = (t >= gt_t[0]) & (t <= gt_t[-1])
    g = np.c_[np.interp(t[m], gt_t, gt_xy[:, 0]), np.interp(t[m], gt_t, gt_xy[:, 1])]
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(g, axis=0), axis=1))]
    h = s <= head_m
    R, T = se2_fit(xy[m][h], g[h])
    p = xy @ R.T + T
    p = scale * (p - p[0]) @ rot(yaw_err).T + p[0]
    tmpl = pd.read_csv(TEMPLATE)
    if rows == "template":
        ts = tmpl["header.stamp.secs"].to_numpy(float) + tmpl["header.stamp.nsecs"].to_numpy(float) * 1e-9
        e, n = np.interp(ts, t, p[:, 0]), np.interp(ts, t, p[:, 1])
        df = tmpl.copy()
    else:                                   # one row per scan at the estimator's own (shifted) stamps
        k = len(tmpl) - len(t)              # fewer scans than frames: hold the last pose
        e, n = np.r_[p[:, 0], np.repeat(p[-1, 0], k)], np.r_[p[:, 1], np.repeat(p[-1, 1], k)]
        df = tmpl.copy()
        df["Time"] = np.r_[t, np.repeat(t[-1], k)]
    if jitter:                              # white position noise per row (seed fixed)
        rng = np.random.default_rng(0)
        e, n = e + rng.normal(0, jitter, len(e)), n + rng.normal(0, jitter, len(n))
    if hold_every:                          # every k-th row repeats the previous one (a zero step)
        idx = np.arange(len(e))
        idx[hold_every::hold_every] -= 1
        e, n = e[idx], n[idx]
    lon, lat = TO_WGS.transform(e, n)
    df["easting"], df["northing"], df["latitude"], df["longitude"] = e, n, lat, lon
    return df


def main():
    a = np.loadtxt(TUM, comments="#")
    t, xy = a[:, 0], a[:, 1:3]
    g = pd.read_csv(REPO / "groundtruth_extracted.csv", usecols=["Time", "easting", "northing"]).dropna()
    gt_t, gt_xy = g["Time"].to_numpy(), g[["easting", "northing"]].to_numpy()

    variants = {"our export file (as shared)": None,
                "rebuilt here, same steps": {},
                "no clock correction (0 s)": dict(clock=0.0),
                "clock 0.5 s late": dict(clock=-0.5),
                "start fit on 50 m": dict(head_m=50.0),
                "start fit on 1000 m": dict(head_m=1000.0),
                "one row per scan (not per GNSS row)": dict(rows="scans")}
    variants.update({f"yaw {d:+.0f} deg after fit": dict(yaw_err=d) for d in (-4, -2, -1, 1, 2, 4)})
    variants.update({f"row jitter {j:.2f} m": dict(jitter=j) for j in (0.02, 0.05, 0.1, 0.2)})
    variants.update({f"every {k}th row repeated (zero step)": dict(hold_every=k) for k in (50, 10)})
    variants.update({f"distance scaled x{k}": dict(scale=k) for k in (0.95, 0.9, 0.75, 0.5)})
    variants["fasterlio.csv (old export, as is)"] = REPO / "fasterlio.csv"
    print(f"{'variant':40s} {'in-box mean':>12s} {'median':>8s} {'pole-span max':>14s} {'final':>8s}")
    with tempfile.TemporaryDirectory() as tmp:
        S.OUT = Path(tmp)
        for i, (name, kw) in enumerate(variants.items()):
            csv = OURS if kw is None else Path(tmp) / f"v{i}.csv"
            if isinstance(kw, Path):                               # 5 415 rows: hold the last row to 5 422
                d = pd.read_csv(kw)
                pd.concat([d, d.iloc[[-1] * (5422 - len(d))]]).to_csv(csv, index=False)
            elif kw is not None:
                build(t, xy, gt_t, gt_xy, **kw).to_csv(csv, index=False)
            with contextlib.redirect_stdout(io.StringIO()):       # the original loop prints per detection
                r = S.run(str(csv), "none", f"v{i}")
            nb, ps = r["native_box_metric"], r["pole_span"]
            print(f"{name:40s} {nb['mean']:12.1f} {nb['median']:8.1f} {ps['max']:14.1f} "
                  f"{r['whole_drive']['final_m']:8.0f}", flush=True)


if __name__ == "__main__":
    main()
