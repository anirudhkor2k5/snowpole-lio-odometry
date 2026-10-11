"""Relative-motion labels against the dual-GNSS reference.

The reference is not survey ground truth. Labels are GNSS-referenced motion
error. Samples with low reference weight are left unlabeled.
"""

from __future__ import annotations

import numpy as np

from odom_refinement.evaluation.trajectory_metrics import _index_at_horizon, relative_displacement_error


def label_relative_motion(
    est_xy: np.ndarray,
    ref_xy: np.ndarray,
    time_s: np.ndarray,
    valid: np.ndarray,
    quality_weight: np.ndarray,
    *,
    horizon_s: float,
    tolerance_s: float,
    threshold_m: float,
    min_quality: float = 0.5,
) -> dict:
    motion = relative_displacement_error(est_xy, ref_xy, time_s, valid, horizon_s, tolerance_s)
    end = _index_at_horizon(time_s, horizon_s, tolerance_s)
    translation = motion["translation_series_m"]
    usable = np.isfinite(translation)
    for i, j in enumerate(end):
        if j < 0:
            usable[i] = False
            continue
        if min(quality_weight[i], quality_weight[j]) < min_quality:
            usable[i] = False
    labels = np.full(len(time_s), np.nan)
    labels[usable] = (translation[usable] > threshold_m).astype(np.float64)
    return {
        "horizon_s": horizon_s,
        "threshold_m": threshold_m,
        "translation_error_m": translation,
        "forward_error_m": motion["forward_series_m"],
        "lateral_error_m": motion["lateral_series_m"],
        "excessive": labels,
        "usable": usable,
        "positive_rate": float(np.nanmean(labels[usable])) if usable.any() else None,
        "n_usable": int(usable.sum()),
    }
