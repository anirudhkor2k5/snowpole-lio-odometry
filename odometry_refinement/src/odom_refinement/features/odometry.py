"""Causal odometry features.

At sample k the features use only samples at or before k. Absolute position,
trip identity, and sample index are not features.
"""

from __future__ import annotations

import numpy as np

from odom_refinement.data.transforms import course_deg, wrap_deg

FEATURE_NAMES = (
    "dt_s",
    "speed_m_s",
    "speed_delta_m_s",
    "accel_m_s2",
    "jerk_m_s3",
    "course_rate_deg_s",
    "curvature_1_m",
    "smoothness_speed_std_m_s",
    "cv_residual_m",
    "step_m",
    "exported_z_rate_m_s",
    "exported_z_accel_m_s2",
)

FEATURE_GROUPS = {
    "timing": ("dt_s",),
    "kinematics": ("speed_m_s", "speed_delta_m_s", "accel_m_s2", "jerk_m_s3", "step_m"),
    "turning": ("course_rate_deg_s", "curvature_1_m"),
    "consistency": ("smoothness_speed_std_m_s", "cv_residual_m"),
    "exported_vertical": ("exported_z_rate_m_s", "exported_z_accel_m_s2"),
}

BLOCKED_FEATURE_TOKENS = (
    "easting",
    "northing",
    "latitude",
    "longitude",
    "altitude",
    "index",
    "seq",
    "label",
    "error",
    "ate",
    "rpe",
    "gt",
    "ground",
)


def extract_odometry_features(
    time_s: np.ndarray,
    xy: np.ndarray,
    z_m: np.ndarray | None = None,
    *,
    smoothness_window_s: float = 1.0,
    max_dt_s: float = 0.5,
    min_speed_for_curvature_m_s: float = 0.5,
) -> dict:
    """Batch features that match :meth:`OnlineOdometryFeatures.push`."""
    time_s = np.asarray(time_s, dtype=np.float64)
    xy = np.asarray(xy, dtype=np.float64)
    n = len(time_s)
    z = np.full(n, np.nan) if z_m is None else np.asarray(z_m, dtype=np.float64)
    dt = np.diff(time_s, prepend=np.nan)
    de = np.diff(xy[:, 0], prepend=np.nan)
    dn = np.diff(xy[:, 1], prepend=np.nan)
    dz = np.diff(z, prepend=np.nan)
    step = np.hypot(de, dn)
    speed = step / dt
    course = course_deg(xy[:, 0], xy[:, 1])
    course_rate = np.full(n, np.nan)
    course_rate[1:] = wrap_deg(np.diff(course)) / dt[1:]
    speed_delta = np.full(n, np.nan)
    speed_delta[1:] = np.diff(speed)
    accel = np.full(n, np.nan)
    accel[1:] = np.diff(speed) / dt[1:]
    jerk = np.full(n, np.nan)
    jerk[2:] = np.diff(accel[1:]) / dt[2:]
    z_rate = dz / dt
    z_accel = np.full(n, np.nan)
    z_accel[1:] = np.diff(z_rate) / dt[1:]
    curvature = np.radians(course_rate) / speed
    curvature[speed < min_speed_for_curvature_m_s] = np.nan
    smoothness = np.full(n, np.nan)
    cv_residual = np.full(n, np.nan)
    for k in range(n):
        if k == 0 or not np.isfinite(time_s[k]):
            continue
        start = k
        while start > 0 and time_s[k] - time_s[start - 1] <= smoothness_window_s:
            start -= 1
        window = speed[start : k + 1]
        window = window[np.isfinite(window)]
        if len(window) >= 2:
            smoothness[k] = float(window.std())
        if k >= 1 and np.isfinite(course[k - 1]) and np.isfinite(speed[k - 1]) and np.isfinite(dt[k]):
            yaw = np.radians(course[k - 1])
            pred_e = xy[k - 1, 0] + np.sin(yaw) * speed[k - 1] * dt[k]
            pred_n = xy[k - 1, 1] + np.cos(yaw) * speed[k - 1] * dt[k]
            cv_residual[k] = float(np.hypot(xy[k, 0] - pred_e, xy[k, 1] - pred_n))
    columns = {
        "dt_s": dt,
        "speed_m_s": speed,
        "speed_delta_m_s": speed_delta,
        "accel_m_s2": accel,
        "jerk_m_s3": jerk,
        "course_rate_deg_s": course_rate,
        "curvature_1_m": curvature,
        "smoothness_speed_std_m_s": smoothness,
        "cv_residual_m": cv_residual,
        "step_m": step,
        "exported_z_rate_m_s": z_rate,
        "exported_z_accel_m_s2": z_accel,
    }
    features = np.column_stack([columns[name] for name in FEATURE_NAMES])
    valid = np.isfinite(dt) & (dt > 0.0) & (dt <= max_dt_s) & np.isfinite(step)
    valid[0] = False
    return {
        "names": FEATURE_NAMES,
        "features": features,
        "valid": valid,
        "course_deg": course,
    }


class OnlineOdometryFeatures:
    """Frame-by-frame extractor. The latest feature equals the batch feature."""

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self._t: list[float] = []
        self._xy: list[list[float]] = []
        self._z: list[float] = []

    def push(self, time_s: float, easting_m: float, northing_m: float, z_m: float | None = None) -> dict:
        self._t.append(float(time_s))
        self._xy.append([float(easting_m), float(northing_m)])
        self._z.append(np.nan if z_m is None else float(z_m))
        batch = extract_odometry_features(
            np.asarray(self._t),
            np.asarray(self._xy),
            np.asarray(self._z),
            **self.kwargs,
        )
        k = len(self._t) - 1
        return {
            "features": batch["features"][k],
            "valid": bool(batch["valid"][k]),
            "names": batch["names"],
        }


def assert_feature_names_are_causal(names: tuple[str, ...] | list[str]) -> None:
    blocked = set(BLOCKED_FEATURE_TOKENS)
    for name in names:
        parts = set(name.lower().split("_"))
        if parts & blocked:
            raise ValueError(f"feature {name} is not an allowed causal runtime input")
