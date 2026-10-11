"""Reliability scores from motion features.

The rules are fixed before looking at the test block. Scores are ranking
functions. They are not claimed to be calibrated covariances.
"""

from __future__ import annotations

import numpy as np

from odom_refinement.features.odometry import FEATURE_NAMES


def _index(name: str) -> int:
    return FEATURE_NAMES.index(name)


def analytical_scores(features: np.ndarray, valid: np.ndarray) -> dict:
    """Physics-based ranks.

    ``without_vertical`` uses speed jumps, turn rate, curvature and the
    constant-velocity residual. ``with_vertical`` also flags an exported
    vertical rate above 5 m/s. That vertical term describes the exported z
    channel. It is not a measurement of road grade.
    """
    features = np.asarray(features, dtype=np.float64)
    valid = np.asarray(valid, dtype=bool)
    speed = features[:, _index("speed_m_s")]
    accel = np.abs(features[:, _index("accel_m_s2")])
    course_rate = np.abs(features[:, _index("course_rate_deg_s")])
    curvature = np.abs(features[:, _index("curvature_1_m")])
    residual = features[:, _index("cv_residual_m")]
    z_rate = np.abs(features[:, _index("exported_z_rate_m_s")])

    def robust_z(values: np.ndarray) -> np.ndarray:
        base = values[valid & np.isfinite(values)]
        if len(base) < 10:
            return np.zeros(len(values))
        center = np.median(base)
        scale = np.median(np.abs(base - center))
        scale = 1.4826 * scale
        if not np.isfinite(scale) or scale < 1e-6:
            scale = float(np.std(base)) or 1.0
        z = (values - center) / scale
        z[~np.isfinite(z)] = 0.0
        return z

    parts = np.column_stack(
        [
            robust_z(accel),
            robust_z(course_rate),
            robust_z(curvature),
            robust_z(residual),
        ]
    )
    # Robust z is estimated on all valid frames of this trajectory, including
    # frames that later fall in the test block. That uses the test distribution
    # of the features. For the deployed comparison we therefore also expose a
    # version whose scale is supplied by the caller.
    rank = np.max(parts, axis=1)
    rank[~valid] = np.nan
    probability = 1.0 / (1.0 + np.exp(-np.clip(rank - 3.0, -30, 30)))
    probability[~valid] = np.nan
    vertical_flag = valid & np.isfinite(z_rate) & (z_rate > 5.0)
    with_vertical = np.maximum(probability, vertical_flag.astype(np.float64))
    with_vertical[~valid] = np.nan
    hard = valid & (rank >= 3.0)
    return {
        "motion_rank": rank,
        "motion_probability": probability,
        "motion_or_vertical_probability": with_vertical,
        "motion_flag": hard,
        "vertical_flag": vertical_flag,
        "scale_note": "robust z uses the evaluated trajectory's own valid frames",
    }


def analytical_scores_with_train_scale(
    features: np.ndarray,
    valid: np.ndarray,
    train_mask: np.ndarray,
) -> dict:
    """Same rules, with robust scales fit on the training block only."""
    features = np.asarray(features, dtype=np.float64)
    names = ("accel_m_s2", "course_rate_deg_s", "curvature_1_m", "cv_residual_m")
    columns = []
    scales = {}
    for name in names:
        values = np.abs(features[:, _index(name)])
        base = values[train_mask & valid & np.isfinite(values)]
        center = float(np.median(base)) if len(base) else 0.0
        mad = float(np.median(np.abs(base - center))) if len(base) else 1.0
        scale = 1.4826 * mad
        if scale < 1e-6:
            scale = float(np.std(base)) if len(base) else 1.0
            scale = scale or 1.0
        z = (values - center) / scale
        z[~np.isfinite(z)] = 0.0
        columns.append(z)
        scales[name] = {"center": center, "scale": scale}
    rank = np.max(np.column_stack(columns), axis=1)
    rank[~valid] = np.nan
    probability = 1.0 / (1.0 + np.exp(-np.clip(rank - 3.0, -30, 30)))
    probability[~valid] = np.nan
    z_rate = np.abs(features[:, _index("exported_z_rate_m_s")])
    vertical_flag = valid & np.isfinite(z_rate) & (z_rate > 5.0)
    with_vertical = np.maximum(probability, vertical_flag.astype(np.float64))
    with_vertical[~valid] = np.nan
    return {
        "motion_probability": probability,
        "motion_or_vertical_probability": with_vertical,
        "motion_flag": valid & (rank >= 3.0),
        "scales": scales,
    }
