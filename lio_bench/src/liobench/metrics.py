"""Trajectory error metrics against the ground-truth CSV (horizontal unless stated)."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .geometry import (course_from_positions, cumulative_distance, fit_alignment, interpolate,
                       level, transform)
from .io import Trajectory


def stats(x: np.ndarray) -> dict:
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if len(x) == 0:
        return {"n": 0}
    return {"n": int(len(x)), "rmse": float(np.sqrt(np.mean(x ** 2))), "mean": float(x.mean()),
            "median": float(np.median(x)), "p95": float(np.percentile(x, 95)), "max": float(x.max())}


# --------------------------------------------------------------------------- clock

def estimate_time_offset(est: Trajectory, ref: Trajectory, search_s: float = 1.0, step_s: float = 0.005) -> dict:
    """Shift dt such that est(t + dt) best matches ref(t) in speed.

    Speed is invariant to the estimator's frame, so this works before any
    spatial alignment. Returned dt should be *added* to the estimate's stamps.
    """
    _, v_ref = course_from_positions(ref.t, ref.p, 0.3, min_speed=0.0)
    grid = ref.t[(ref.t > max(ref.t[0], est.t[0]) + search_s + 1) & (ref.t < min(ref.t[-1], est.t[-1]) - search_s - 1)]
    vr = np.interp(grid, ref.t, v_ref)
    _, v_est = course_from_positions(est.t, est.p, 0.3, min_speed=0.0)
    shifts = np.arange(-search_s, search_s + step_s / 2, step_s)
    cost = np.array([np.nanmean(np.abs(np.interp(grid - s, est.t, v_est) - vr)) for s in shifts])
    k = int(np.nanargmin(cost))
    dt = shifts[k]
    if 0 < k < len(shifts) - 1:  # parabolic refinement
        a, b, c = cost[k - 1], cost[k], cost[k + 1]
        den = a - 2 * b + c
        if den > 0:
            dt += 0.5 * step_s * (a - c) / den
    return {"dt_s": float(dt), "speed_mae_mps": float(cost[k]), "speed_mae_at_zero_mps": float(cost[np.argmin(np.abs(shifts))])}


# --------------------------------------------------------------------------- relative errors

def rpe_time(est_xy, ref_xy, t, horizon_s, tol=0.15):
    """Displacement error over a fixed time horizon, in the aligned frame. NaN where undefined."""
    j = np.searchsorted(t, t + horizon_s)
    j = np.clip(j, 0, len(t) - 1)
    ok = np.abs(t[j] - (t + horizon_s)) <= tol
    de = est_xy[j] - est_xy
    dr = ref_xy[j] - ref_xy
    e = np.linalg.norm(de - dr, axis=1)
    e[~ok] = np.nan
    return e


def segment_drift(est_xy, ref_xy, dist, length_m, chord_m=10.0, stride=10):
    """KITTI-style translational drift (% of length) that needs no ground-truth orientation.

    For each start i: anchor both paths at i, rotate the estimate so its heading
    over the first `chord_m` matches the reference's, and measure the end-point
    error after `length_m` of reference travel. Frame-independent, so it is also
    fair to estimates that were never aligned.
    """
    starts = np.arange(0, len(dist), stride)
    c = np.searchsorted(dist, dist[starts] + chord_m)
    j = np.searchsorted(dist, dist[starts] + length_m)
    ok = j < len(dist)
    starts, c, j = starts[ok], c[ok], j[ok]
    he = est_xy[c] - est_xy[starts]
    hr = ref_xy[c] - ref_xy[starts]
    rot = np.arctan2(hr[:, 1], hr[:, 0]) - np.arctan2(he[:, 1], he[:, 0])
    de = est_xy[j] - est_xy[starts]
    cr, sr = np.cos(rot), np.sin(rot)
    de_r = np.c_[cr * de[:, 0] - sr * de[:, 1], sr * de[:, 0] + cr * de[:, 1]]
    dr = ref_xy[j] - ref_xy[starts]
    err = np.linalg.norm(de_r - dr, axis=1)
    good = np.isfinite(err)
    return starts[good], 100.0 * err[good] / length_m


# --------------------------------------------------------------------------- full evaluation

@dataclass
class Evaluation:
    name: str
    protocol: str
    t: np.ndarray
    ref_xy: np.ndarray
    est_xy: np.ndarray
    err: np.ndarray
    dist: np.ndarray
    valid: np.ndarray
    summary: dict = field(default_factory=dict)
    aligned: Trajectory | None = None


def evaluate(est: Trajectory, ref: Trajectory, protocol: str, ecfg: dict) -> Evaluation:
    """Associate on the reference clock, align by `protocol`, compute every metric."""
    summary: dict = {"name": est.name, "protocol": protocol, "n_est": len(est)}

    est = level(est)
    if ecfg.get("estimate_time_offset", True):
        off = estimate_time_offset(est, ref)
        summary["time_offset"] = off
        if abs(off["dt_s"]) > ecfg.get("min_time_offset_s", 0.02):
            est = est.shifted(off["dt_s"])

    p, _, valid = interpolate(est, ref.t, ecfg.get("max_assoc_dt", 0.15))
    dist = cumulative_distance(ref.p)
    R, tt, s, ainfo = fit_alignment(p, ref.p, protocol, dist, ecfg.get("head_align_m", 200.0))
    summary["alignment"] = ainfo
    aligned = transform(est, R, tt, s, name=est.name)
    pa = s * p @ R.T + tt

    exy, rxy = pa[:, :2], ref.p[:, :2]
    err = np.linalg.norm(exy - rxy, axis=1)
    err[~valid] = np.nan
    summary["coverage"] = float(valid.mean())
    summary["ate_xy"] = stats(err)
    if protocol.startswith("se3"):
        summary["ate_3d"] = stats(np.linalg.norm(pa - ref.p, axis=1)[valid])
    vi = np.flatnonzero(valid)
    summary["final_error_m"] = float(err[vi[-1]]) if len(vi) else float("nan")
    est_len = float(np.nansum(np.linalg.norm(np.diff(exy[valid], axis=0), axis=1)))
    ref_len = float(np.sum(np.linalg.norm(np.diff(rxy[valid], axis=0), axis=1)))
    summary["path_length_m"] = {"est": est_len, "ref": ref_len, "ratio": est_len / ref_len if ref_len else float("nan")}

    summary["rpe_time"] = {f"{h:g}s": stats(rpe_time(exy, rxy, ref.t, h)) for h in ecfg.get("rpe_time_s", [1, 10])}
    drift = {}
    for L in ecfg.get("drift_lengths_m", [100, 200, 400, 800]):
        _, d = segment_drift(np.where(valid[:, None], exy, np.nan), rxy, dist, L)
        drift[f"{L:g}m"] = stats(d)
    summary["drift_pct"] = drift
    allp = np.concatenate([np.atleast_1d(segment_drift(np.where(valid[:, None], exy, np.nan), rxy, dist, L)[1])
                           for L in ecfg.get("drift_lengths_m", [100, 200, 400, 800])])
    summary["drift_pct_mean_kitti"] = float(np.mean(allp)) if len(allp) else float("nan")

    seg = ecfg.get("segment_m", 500.0)
    bins = (dist // seg).astype(int)
    summary["segment_rmse"] = [
        {"start_m": float(b * seg), "rmse_m": float(np.sqrt(np.nanmean(err[bins == b] ** 2)))}
        for b in np.unique(bins) if np.isfinite(err[bins == b]).any()
    ]
    return Evaluation(est.name, protocol, ref.t, rxy, exy, err, dist, valid, summary, aligned)
