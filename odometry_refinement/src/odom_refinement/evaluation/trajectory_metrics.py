"""Planar trajectory metrics.

Orientation columns in the exports are not used as yaw. Course over ground
is computed from positions and is named as course, not body yaw. Vertical
odometry channels are reported separately and are excluded from the primary
horizontal error because the exported altitude is not a verified height.
"""

from __future__ import annotations

import numpy as np

from odom_refinement.data.transforms import course_deg, path_length_m, wrap_deg


def _finite_stats(values: np.ndarray) -> dict:
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    if len(values) == 0:
        return {"n": 0}
    return {
        "n": int(len(values)),
        "rmse": float(np.sqrt(np.mean(values**2))),
        "mean": float(values.mean()),
        "median": float(np.median(values)),
        "std": float(values.std()),
        "p95": float(np.percentile(values, 95)),
        "max": float(values.max()),
    }


def horizontal_errors(est_xy: np.ndarray, ref_xy: np.ndarray, valid: np.ndarray) -> np.ndarray:
    err = np.linalg.norm(est_xy - ref_xy, axis=1)
    err = err.astype(np.float64)
    err[~valid] = np.nan
    return err


def ate_summary(errors_m: np.ndarray) -> dict:
    return _finite_stats(errors_m)


def _index_at_horizon(time_s: np.ndarray, horizon_s: float, tolerance_s: float) -> np.ndarray:
    """For each i, index j whose timestamp is closest to t_i + horizon, or -1."""
    time_s = np.asarray(time_s, dtype=np.float64)
    target = time_s + horizon_s
    pos = np.searchsorted(time_s, target, side="left")
    choice = np.full(len(time_s), -1, dtype=int)
    for i, raw in enumerate(pos):
        candidates = []
        if 0 <= raw - 1 < len(time_s) and raw - 1 > i:
            candidates.append(raw - 1)
        if 0 <= raw < len(time_s) and raw > i:
            candidates.append(raw)
        if not candidates:
            continue
        best = min(candidates, key=lambda j: abs(time_s[j] - target[i]))
        if abs(time_s[best] - target[i]) <= tolerance_s:
            choice[i] = best
    return choice


def relative_displacement_error(
    est_xy: np.ndarray,
    ref_xy: np.ndarray,
    time_s: np.ndarray,
    valid: np.ndarray,
    horizon_s: float,
    tolerance_s: float,
) -> dict:
    """World-frame difference of displacements over a fixed time horizon.

    This is the translational magnitude of the relative-motion residual when
    both poses are positions in the same east/north frame. It does not require
    a yaw measurement.
    """
    end = _index_at_horizon(time_s, horizon_s, tolerance_s)
    n = len(time_s)
    err = np.full(n, np.nan)
    forward = np.full(n, np.nan)
    lateral = np.full(n, np.nan)
    course_change = np.full(n, np.nan)
    est_course = course_deg(est_xy[:, 0], est_xy[:, 1])
    ref_course = course_deg(ref_xy[:, 0], ref_xy[:, 1])
    for i, j in enumerate(end):
        if j < 0 or not valid[i] or not valid[j]:
            continue
        delta_est = est_xy[j] - est_xy[i]
        delta_ref = ref_xy[j] - ref_xy[i]
        residual = delta_est - delta_ref
        err[i] = np.linalg.norm(residual)
        # Body-frame components use the estimate course at i as a travel-direction
        # proxy. They are undefined when that course is missing.
        if np.isfinite(est_course[i]):
            yaw = np.radians(est_course[i])
            forward[i] = residual[0] * np.sin(yaw) + residual[1] * np.cos(yaw)
            lateral[i] = residual[0] * np.cos(yaw) - residual[1] * np.sin(yaw)
        if np.isfinite(est_course[i]) and np.isfinite(est_course[j]) and np.isfinite(ref_course[i]) and np.isfinite(ref_course[j]):
            est_turn = wrap_deg(est_course[j] - est_course[i])
            ref_turn = wrap_deg(ref_course[j] - ref_course[i])
            course_change[i] = float(wrap_deg(est_turn - ref_turn))
    return {
        "horizon_s": horizon_s,
        "translation_m": _finite_stats(err),
        "forward_m": _finite_stats(forward),
        "lateral_m": _finite_stats(lateral),
        "course_change_deg": _finite_stats(course_change),
        "translation_series_m": err,
        "forward_series_m": forward,
        "lateral_series_m": lateral,
    }


def relative_error_at_distance(
    est_xy: np.ndarray,
    ref_xy: np.ndarray,
    ref_distance_m: np.ndarray,
    valid: np.ndarray,
    distance_m: float,
    tolerance_m: float,
) -> dict:
    """Displacement error after the reference has travelled ``distance_m``."""
    err = []
    end_distance = ref_distance_m + distance_m
    pos = np.searchsorted(ref_distance_m, end_distance, side="left")
    for i, raw in enumerate(pos):
        if not valid[i]:
            continue
        candidates = [j for j in (raw - 1, raw) if 0 <= j < len(ref_distance_m) and j > i]
        if not candidates:
            continue
        j = min(candidates, key=lambda k: abs(ref_distance_m[k] - end_distance[i]))
        if abs(ref_distance_m[j] - end_distance[i]) > tolerance_m or not valid[j]:
            continue
        err.append(np.linalg.norm((est_xy[j] - est_xy[i]) - (ref_xy[j] - ref_xy[i])))
    return {"distance_m": distance_m, "translation_m": _finite_stats(np.asarray(err, dtype=np.float64))}


def cumulative_distance(xy: np.ndarray, valid: np.ndarray) -> np.ndarray:
    dist = np.zeros(len(xy), dtype=np.float64)
    for i in range(1, len(xy)):
        step = np.linalg.norm(xy[i] - xy[i - 1]) if valid[i] and valid[i - 1] else np.nan
        dist[i] = dist[i - 1] + (0.0 if not np.isfinite(step) else step)
    return dist


def error_versus_distance(errors_m: np.ndarray, distance_m: np.ndarray, bin_m: float) -> list[dict]:
    finite = np.isfinite(errors_m) & np.isfinite(distance_m)
    if not finite.any():
        return []
    edges = np.arange(0.0, distance_m[finite].max() + bin_m, bin_m)
    rows = []
    for start, stop in zip(edges[:-1], edges[1:]):
        mask = finite & (distance_m >= start) & (distance_m < stop)
        if mask.sum() < 5:
            continue
        stats = _finite_stats(errors_m[mask])
        rows.append({"start_m": float(start), "stop_m": float(stop), **stats})
    return rows


def failure_events(errors_m: np.ndarray, time_s: np.ndarray, valid: np.ndarray, threshold_m: float) -> dict:
    above = valid & np.isfinite(errors_m) & (errors_m > threshold_m)
    events = 0
    durations = []
    run_start = None
    for i, flag in enumerate(above):
        if flag and run_start is None:
            run_start = i
        if run_start is not None and (not flag or i == len(above) - 1):
            end = i if flag and i == len(above) - 1 else i - 1
            if end >= run_start:
                events += 1
                durations.append(float(time_s[end] - time_s[run_start]))
            run_start = None
    duration = float(time_s[valid][-1] - time_s[valid][0]) if valid.any() else 0.0
    return {
        "threshold_m": threshold_m,
        "n_frames": int(above.sum()),
        "frame_fraction": float(above.sum() / valid.sum()) if valid.any() else None,
        "n_events": events,
        "total_duration_s": float(sum(durations)),
        "longest_event_s": float(max(durations)) if durations else 0.0,
        "evaluated_duration_s": duration,
    }


def drift_summary(errors_m: np.ndarray, distance_m: np.ndarray) -> dict:
    finite = np.isfinite(errors_m) & np.isfinite(distance_m)
    if finite.sum() < 2:
        return {"n": int(finite.sum())}
    # Slope of horizontal error against reference distance. A positive slope is
    # growing absolute error. This is descriptive, not a fitted time warp.
    coef = np.polyfit(distance_m[finite], errors_m[finite], 1)
    return {
        "n": int(finite.sum()),
        "error_growth_m_per_m": float(coef[0]),
        "error_growth_m_per_km": float(coef[0] * 1000.0),
        "path_length_m": float(distance_m[finite].max()),
        "final_error_m": float(errors_m[finite][-1]),
        "initial_error_m": float(errors_m[finite][0]),
    }


def endpoint_distance_m(xy: np.ndarray) -> float:
    return float(np.linalg.norm(xy[-1] - xy[0]))


def trajectory_length_m(xy: np.ndarray, valid: np.ndarray | None = None) -> float:
    if valid is None:
        return path_length_m(xy)
    kept = xy[valid]
    return path_length_m(kept)
