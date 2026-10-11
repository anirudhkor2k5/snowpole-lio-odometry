"""Option B exports: start heading from ONE dual-antenna GNSS reading at the first frame, no GNSS after that.

Same as liobench.export.run (same trajectories, rows, columns), except: no clock offset (the 0.06 s
offset was measured against GNSS on the training block, so it is left out), and the placement in
UTM 33N. Instead of a rotation fitted to GNSS over the first 200 m, the whole trajectory is rotated
once so that the odometry's own heading at its first pose equals the heading of the left->right GNSS
antenna baseline (+90 deg) in the first row of gps_left/right_position.csv. The car is parked there,
and the antennas agree with the GNSS course to ~1 deg once it moves. The /vehicle_enu_heading INS
topic is not used: its heading_init flag is never set in this bag, and it reads ~5 deg low while
driving. Position: the pipeline starts from GNSS frame 0 itself and uses only row-to-row steps.

Writes results/export_optionB/<same names as results/export>.csv (+ .json). results/export is untouched.

  python lio_bench/pipeline/export_option_b.py
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
from liobench import export as X  # noqa: E402
from liobench.io import load_config, load_ground_truth, repo_path  # noqa: E402

OUT = REPO / "lio_bench" / "results" / "export_optionB"
# (output name, source, tag key, pipeline); tags come from the command line (defaults = the original runs)
JOBS = [("faster_lio_full_raw", "faster_lio", "faster", "raw"),                                     # 1. raw Faster-LIO
        ("faster_lio_full_gravity", "faster_lio", "faster", "gravity"),                             # 2. + gravity
        ("faster_lio_full_gravity_kiss_heading", "faster_lio", "faster", "gravity+kiss_heading"),   # 3. + KISS-ICP (offline)
        ("kiss_fusion_rt_fusion_v20_run", "kiss_fusion", "fusion", "run")]                          # 4. online fusion node


def antenna_heading_enu() -> float:
    """ENU yaw (rad, from east, CCW) of the vehicle from the first left/right antenna row."""
    to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32633", always_xy=True)
    L = pd.read_csv(REPO / "gps_left_position.csv").iloc[0]
    R = pd.read_csv(REPO / "gps_right_position.csv").iloc[0]
    le, ln = to_utm.transform(L["longitude"], L["latitude"])
    re, rn = to_utm.transform(R["longitude"], R["latitude"])
    return float(np.arctan2(rn - ln, re - le) + np.pi / 2)   # baseline left->right, turned 90 deg CCW = forward


def yaw_of(q) -> float:
    x, y, z, w = q
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def export(cfg, source, tag, pipeline, yaw0, name, out_dir) -> Path:
    tr, dt = X.trajectory(cfg, source, tag, pipeline)   # dt: clock offset measured against GNSS; NOT applied
    rot = yaw0 - yaw_of(tr.q[0])
    c, s = np.cos(rot), np.sin(rot)
    tmpl = pd.read_csv(repo_path(X.TEMPLATE))
    ts = tmpl["header.stamp.secs"].to_numpy(float) + tmpl["header.stamp.nsecs"].to_numpy(float) * 1e-9
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
    out = out_dir / f"incremental_navigation_{name}.csv"
    df.to_csv(out, index=False)
    gt = load_ground_truth(cfg)
    inside = (ts >= gt.t[0]) & (ts <= gt.t[-1])
    err = np.hypot(e - np.interp(ts, gt.t, gt.p[:, 0]), n - np.interp(ts, gt.t, gt.p[:, 1]))[inside]
    meta = {"source": source, "tag": tag, "kiss_icp_tag": X.v3.SOURCES["kiss_icp"][1] if "kiss" in pipeline else None, "pipeline": pipeline, "rows": len(df), "clock_offset_s": 0.0,
            "clock_offset_measured_not_applied_s": dt,
            "start_heading": "one dual-antenna GNSS reading at the first frame (option B)",
            "antenna_heading_enu_deg": float(np.degrees(yaw0)), "odometry_yaw0_deg": float(np.degrees(yaw_of(tr.q[0]))),
            "rotation_applied_deg": float(np.degrees(rot)), "template": X.TEMPLATE,
            "row_error_vs_gt_m": {"median": float(np.median(err)), "p95": float(np.percentile(err, 95)),
                                  "max": float(err.max()), "final": float(err[-1])}}
    out.with_suffix(".json").write_text(json.dumps(meta, indent=2, default=float))
    return out


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--faster-tag", default="full", help="results/runs/faster_lio/<tag> for files 1-3")
    ap.add_argument("--kiss-tag", default="full", help="results/runs/kiss_icp/<tag> for file 3")
    ap.add_argument("--fusion-tag", default="rt_fusion_v20", help="results/runs/kiss_fusion/<tag> for file 4")
    ap.add_argument("--out", default=str(OUT))
    a = ap.parse_args()
    X.v3.SOURCES["kiss_icp"] = ("kiss_icp", a.kiss_tag)
    tags = {"faster": a.faster_tag, "fusion": a.fusion_tag}
    out_dir = Path(a.out)
    cfg = load_config()
    out_dir.mkdir(parents=True, exist_ok=True)
    yaw0 = antenna_heading_enu()
    print(f"antenna heading at the first frame: ENU {np.degrees(yaw0):.2f} deg (compass {(90 - np.degrees(yaw0)) % 360:.2f})")
    for name, source, key, pipeline in JOBS:
        p = export(cfg, source, tags[key], pipeline, yaw0, name, out_dir)
        print(f"  {source}/{tags[key]} ({pipeline})" + (f" + kiss_icp/{a.kiss_tag}" if "kiss" in pipeline else ""))
        m = json.loads(p.with_suffix(".json").read_text())
        old = json.loads((X.results_dir("export") / p.name).with_suffix(".json").read_text())
        print(f"{p.name:70s} rotation {m['rotation_applied_deg']:+6.2f} deg (200 m fit had "
              f"{old['alignment']['yaw_deg']:+6.2f}); final row error {m['row_error_vs_gt_m']['final']:6.0f} m "
              f"(200 m fit {old['row_error_vs_gt_m']['final']:6.0f} m)", flush=True)


if __name__ == "__main__":
    main()
