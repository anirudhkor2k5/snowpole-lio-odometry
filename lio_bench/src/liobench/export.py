"""Write a corrected trajectory as the snow-pole pipeline's `incremental_navigation_results.csv`.

The pipeline (snowpole_based_vehicle_localization*.py) reads only `easting` / `northing`
(UTM 33N), derives heading and step length from consecutive rows, and pairs rows with the
image stream **by row index**. So the export keeps the template's rows and timestamps exactly
(one row per left-GNSS message) and replaces the positions:

  - positions are the chosen pipeline's trajectory sampled at each row's GNSS header stamp;
  - the estimator clock is corrected by its training-block residual offset (calibration);
  - the trajectory is placed in UTM 33N by ONE SE(2) fit on the first 200 m of the drive
    (the initial GNSS fix the pipeline itself starts from), nothing after that;
  - latitude/longitude are recomputed from easting/northing; template columns that described
    FastReg's own processing (translation_*, rotation_angle_degrees, transformed_*, error,
    covariances) are left empty so nothing stale is carried over.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd

from . import audit, v3
from .correct import heading as H
from .geometry import cumulative_distance, fit_alignment, interpolate
from .io import REPO_ROOT, Trajectory, load_config, load_ground_truth, load_run, repo_path, results_dir

TEMPLATE = "incremental_navigation_results.csv"
CLEAR = ["position_covariance_%d" % i for i in range(9)] + ["position_covariance_type", "translation_easting",
         "translation_northing", "rotation_angle_degrees", "transformed_easting", "transformed_northing", "error"]

PIPELINES = {
    "raw": {"raw": True},
    "gravity": {},
    "gravity+kiss_heading": {"yaw": "kiss_icp"},
}


def trajectory(cfg: dict, source: str, tag: str, pipeline: str) -> tuple[Trajectory, float]:
    """(trajectory on Unix time, clock offset) for a pipeline built on `source` (no GT used after the start)."""
    if pipeline == "run":                       # an already-corrected run, e.g. the online kiss_fusion node
        tr = load_run(source, tag, cfg)
        return tr, audit.train_offset(tr, cfg)
    opts = dict(PIPELINES[pipeline])
    yaw_src = opts.pop("yaw", None)
    extra = {source: (source, tag)}
    if yaw_src:
        extra[yaw_src] = v3.SOURCES[yaw_src]
    S = v3.load_sources(cfg, extra)
    base = S[source]
    t, T, dt = v3.combo(base, S[yaw_src] if yaw_src else base, cfg, [], **opts)
    return H.to_traj(f"{source}:{pipeline}", t, T, cfg), dt


def run(cfg: dict | None = None, source: str = "faster_lio", tag: str = "full", pipeline: str = "gravity+kiss_heading",
        out: str | None = None) -> Path:
    from pyproj import Transformer
    cfg = cfg or load_config()
    tr, dt = trajectory(cfg, source, tag, pipeline)
    tr = tr.shifted(dt)
    gt = load_ground_truth(cfg)
    # one SE(2) alignment on the first 200 m (initial fix), as in the canonical protocol
    p_gt, _, ok = interpolate(tr, gt.t, cfg["evaluation"].get("max_assoc_dt", 0.15))
    from .geometry import level
    R, t0, _, info = fit_alignment(p_gt, gt.p, "se2_head", cumulative_distance(gt.p), 200.0)
    xy = tr.p @ R.T + t0
    tmpl = pd.read_csv(repo_path(TEMPLATE))
    ts = tmpl["header.stamp.secs"].to_numpy(float) + tmpl["header.stamp.nsecs"].to_numpy(float) * 1e-9
    # sample on the rows' GNSS stamps; before the first / after the last pose hold the end values
    e = np.interp(ts, tr.t, xy[:, 0])
    n = np.interp(ts, tr.t, xy[:, 1])
    z = np.interp(ts, tr.t, xy[:, 2])
    held = (ts < tr.t[0]) | (ts > tr.t[-1])
    df = tmpl.copy()
    for c in CLEAR:
        if c in df:
            df[c] = np.nan
    to_wgs = Transformer.from_crs(f"EPSG:{cfg['utm_epsg']}", "EPSG:4326", always_xy=True)
    lon, lat = to_wgs.transform(e, n)
    df["easting"], df["northing"], df["latitude"], df["longitude"] = e, n, lat, lon
    df["altitude"] = tmpl["altitude"].iloc[0] + (z - z[0])
    name = f"incremental_navigation_{source}_{tag}_{pipeline}.csv".replace("+", "_")
    out_path = Path(out) if out else results_dir("export") / name
    df.to_csv(out_path, index=False)
    # self-check against GT on the rows (diagnostic; the pipeline's own metrics are what matter)
    g_e, g_n = np.interp(ts, gt.t, gt.p[:, 0]), np.interp(ts, gt.t, gt.p[:, 1])
    inside = (ts >= gt.t[0]) & (ts <= gt.t[-1])
    err = np.hypot(e - g_e, n - g_n)[inside]
    meta = {"source": source, "tag": tag, "pipeline": pipeline, "rows": len(df), "rows_held_at_ends": int(held.sum()),
            "clock_offset_s": dt, "alignment": info, "template": TEMPLATE,
            "row_error_vs_gt_m": {"median": float(np.median(err)), "p95": float(np.percentile(err, 95)),
                                  "max": float(err.max()), "final": float(err[-1])}}
    out_path.with_suffix(".json").write_text(json.dumps(meta, indent=2, default=float))
    return out_path
