"""Independent check of the Faster-LIO baseline. Uses only numpy/pandas, none of the liobench code.

Compares three trajectories against groundtruth_extracted.csv:
  A. fasterlio.csv                     the old export that was already in the repo (not made by us)
  B. runs/faster_lio/full              our fresh run of gaoxiang12/faster-lio @ 8038fba
  C. runs/fast_lio/full                our fresh run of FAST-LIO2 (hku-mars @ 7cc4175), for scale
and A against B directly.

Each trajectory is fitted to ground truth with a 2D rotation + translation on its first 200 m only
(a clock offset is searched too), then left alone for the rest of the 10.3 km.

  python lio_bench/verify/verify_faster_lio.py          (run from the repo root)
"""
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[2]
RUNS = REPO / "lio_bench" / "results" / "runs"


def load_csv(path):
    d = pd.read_csv(path, usecols=["Time", "easting", "northing"]).dropna()
    return d["Time"].to_numpy(), d[["easting", "northing"]].to_numpy()


def load_tum(path):
    a = np.loadtxt(path, comments="#")
    return a[:, 0], a[:, 1:3]


def interp(t_src, xy_src, t):
    return np.c_[np.interp(t, t_src, xy_src[:, 0]), np.interp(t, t_src, xy_src[:, 1])]


def se2_fit(src, dst):
    ms, md = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((src - ms).T @ (dst - md))
    R = (U @ Vt).T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = (U @ Vt).T
    return R, md - R @ ms


def path_len(xy):
    return float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())


def evaluate(name, t, xy, gt_t, gt_xy, head_m=200.0):
    best = None
    for off in np.arange(-1.0, 1.0001, 0.02):                 # clock offset search
        tt = t + off
        m = (tt >= gt_t[0]) & (tt <= gt_t[-1])
        g = interp(gt_t, gt_xy, tt[m])
        s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(g, axis=0), axis=1))]
        h = s <= head_m
        R, T = se2_fit(xy[m][h], g[h])
        rms = np.sqrt((np.linalg.norm(xy[m][h] @ R.T + T - g[h], axis=1) ** 2).mean())
        if best is None or rms < best[0]:
            best = (rms, off, R, T, m, g, s)
    rms, off, R, T, m, g, s = best
    p = xy[m] @ R.T + T
    e = np.linalg.norm(p - g, axis=1)
    at = lambda km: e[np.searchsorted(s, km * 1000) - 1] if s[-1] >= km * 1000 else np.nan
    print(f"\n{name}")
    print(f"  poses {len(t)}, clock offset {off:+.2f} s, head-fit rms {rms:.2f} m, "
          f"yaw {np.degrees(np.arctan2(R[1, 0], R[0, 0])):+.2f} deg")
    print(f"  horizontal length {path_len(p) / 1000:.3f} km vs ground truth {s[-1] / 1000:.3f} km "
          f"(ratio {path_len(p) / s[-1]:.3f})")
    print("  error at km 1/2/5/8/10: " + " / ".join(f"{at(k):.0f}" for k in (1, 2, 5, 8, 10))
          + f" m; final {e[-1]:.0f} m; final/length {100 * e[-1] / s[-1]:.1f} %")
    return (t + off)[m], p


def main():
    gt_t, gt_xy = load_csv(REPO / "groundtruth_extracted.csv")
    print(f"ground truth: {len(gt_t)} rows, {gt_t[-1] - gt_t[0]:.0f} s, {path_len(gt_xy) / 1000:.2f} km")
    tA, A = evaluate("A. fasterlio.csv (old export already in the repo)", *load_csv(REPO / "fasterlio.csv"), gt_t, gt_xy)
    tB, B = evaluate("B. our Faster-LIO run (runs/faster_lio/full)",
                     *load_tum(RUNS / "faster_lio" / "full" / "trajectory_tum.txt"), gt_t, gt_xy)
    evaluate("C. our FAST-LIO2 run (runs/fast_lio/full)",
             *load_tum(RUNS / "fast_lio" / "full" / "trajectory_tum.txt"), gt_t, gt_xy)
    # fastlio.csv's clock starts at 0; assume it starts with fasterlio.csv's first row (the ±1 s search absorbs the rest)
    tD, D = load_csv(REPO / "fastlio.csv")
    evaluate("D. fastlio.csv (old FAST-LIO export already in the repo)", tD + load_csv(REPO / "fasterlio.csv")[0][0], D,
             gt_t, gt_xy)

    # A vs B directly: same algorithm, independent runs. Fit A onto B over the whole drive.
    t = tB[(tB >= tA[0]) & (tB <= tA[-1])]
    a, b = interp(tA, A, t), interp(tB, B, t)
    R, T = se2_fit(a, b)
    d = np.linalg.norm(a @ R.T + T - b, axis=1)
    print(f"\nA vs B (old export vs our run, best rigid fit over the whole drive): "
          f"median {np.median(d):.1f} m, p95 {np.percentile(d, 95):.1f} m, max {d.max():.1f} m, "
          f"length ratio A/B {path_len(a) / path_len(b):.3f}")


if __name__ == "__main__":
    main()
