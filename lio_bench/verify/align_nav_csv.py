"""Fix the start heading of a pipeline input CSV using GNSS from the first 200 m only.

  python lio_bench/verify/align_nav_csv.py in.csv out.csv [--head-m 200]      (from the repo root)

The pipeline only uses the row-to-row steps of `easting` / `northing`, so a CSV whose track points a
few degrees off at the start drifts sideways before the first pole (0.72 km) and locks onto the wrong
poles. This finds the one rotation + shift that best fits the CSV's first `head_m` metres onto
groundtruth_extracted.csv (a ±1 s clock offset is searched too) and applies it to every row. Nothing
after the first `head_m` metres is used. Step lengths are unchanged. latitude / longitude are
recomputed. All other columns are copied as they are.
"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd
from pyproj import Transformer

REPO = Path(__file__).resolve().parents[2]


def se2_fit(src, dst):
    ms, md = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((src - ms).T @ (dst - md))
    R = (U @ Vt).T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = (U @ Vt).T
    return R, md - R @ ms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("inp")
    ap.add_argument("out")
    ap.add_argument("--head-m", type=float, default=200.0)
    a = ap.parse_args()

    d = pd.read_csv(a.inp)
    g = pd.read_csv(REPO / "groundtruth_extracted.csv", usecols=["Time", "easting", "northing"]).dropna()
    gt_t, gt = g["Time"].to_numpy(), g[["easting", "northing"]].to_numpy()
    t, xy = d["Time"].to_numpy(float), d[["easting", "northing"]].to_numpy(float)

    best = None
    for off in np.arange(-1.0, 1.0001, 0.02):
        tt = t + off
        m = (tt >= gt_t[0]) & (tt <= gt_t[-1])
        ref = np.c_[np.interp(tt[m], gt_t, gt[:, 0]), np.interp(tt[m], gt_t, gt[:, 1])]
        s = np.r_[0, np.cumsum(np.hypot(*np.diff(ref, axis=0).T))]
        h = s <= a.head_m
        R, T = se2_fit(xy[m][h], ref[h])
        rms = np.sqrt(np.mean(np.sum((xy[m][h] @ R.T + T - ref[h]) ** 2, axis=1)))
        if best is None or rms < best[0]:
            best = (rms, off, R, T)
    rms, off, R, T = best
    new = xy @ R.T + T
    d["easting"], d["northing"] = new[:, 0], new[:, 1]
    lon, lat = Transformer.from_crs("EPSG:32633", "EPSG:4326", always_xy=True).transform(new[:, 0], new[:, 1])
    d["latitude"], d["longitude"] = lat, lon
    d.to_csv(a.out, index=False)
    print(f"rotated {np.degrees(np.arctan2(R[1, 0], R[0, 0])):+.2f} deg (counter-clockwise), clock offset "
          f"{off:+.2f} s, fit rms {rms:.2f} m over the first {a.head_m:.0f} m -> {a.out}")


if __name__ == "__main__":
    main()
