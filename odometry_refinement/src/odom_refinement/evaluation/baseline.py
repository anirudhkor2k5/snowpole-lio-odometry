"""Baseline evaluation against the dual-GNSS midpoint.

No ground-truth file is invented. Absolute numbers are GNSS-referenced.
"""

from __future__ import annotations

import numpy as np

from odom_refinement.data.csv_loader import Trajectory, column_disagreement_m
from odom_refinement.data.synchronization import (
    ReferenceTrajectory,
    odometry_unix_time,
    resample_onto_reference,
)
from odom_refinement.data.transforms import (
    course_deg,
    diagnostic_global_se2,
    se2_from_chord,
    wrap_deg,
)
from odom_refinement.evaluation.trajectory_metrics import (
    ate_summary,
    cumulative_distance,
    drift_summary,
    error_versus_distance,
    failure_events,
    horizontal_errors,
    relative_displacement_error,
    relative_error_at_distance,
)


def _rotation_diagnostic(rotation_deg: np.ndarray | None, xy: np.ndarray, ref_heading: np.ndarray, valid: np.ndarray) -> dict | None:
    if rotation_deg is None or not np.isfinite(rotation_deg).any():
        return None
    # rotation is on the original samples; here xy is already on the reference clock,
    # so this diagnostic is computed by the caller on the native series. Kept for API
    # symmetry when a resampled rotation is supplied.
    course = course_deg(xy[:, 0], xy[:, 1])
    both = valid & np.isfinite(rotation_deg) & np.isfinite(course)
    heading = valid & np.isfinite(rotation_deg) & np.isfinite(ref_heading)
    def _med_abs(mask, other):
        if mask.sum() < 10:
            return None
        residual = np.abs(wrap_deg(rotation_deg[mask] - other[mask]))
        return float(np.median(residual))
    corr = None
    if both.sum() > 10:
        corr = float(np.corrcoef(rotation_deg[both], course[both])[0, 1])
    return {
        "median_abs_rotation_minus_course_deg": _med_abs(both, course),
        "correlation_with_course": corr,
        "median_abs_rotation_minus_baseline_heading_deg": _med_abs(heading, ref_heading),
        "interpretation": "not used as yaw; compared only as a diagnostic",
    }


def evaluate_synchronized(
    name: str,
    position_source: str,
    est_xy: np.ndarray,
    est_alt: np.ndarray | None,
    reference: ReferenceTrajectory,
    valid: np.ndarray,
    *,
    time_info: dict,
    chord_m: float,
    rpe_horizons_s: tuple[float, ...],
    rpe_tolerance_s: float,
    distance_horizons_m: tuple[float, ...],
    segment_m: float,
    failure_thresholds_m: tuple[float, ...],
    rotation_deg: np.ndarray | None = None,
) -> dict:
    ref_xy = reference.xy()
    errors = horizontal_errors(est_xy, ref_xy, valid)
    distance = cumulative_distance(ref_xy, np.ones(len(ref_xy), dtype=bool))
    origin = est_xy.copy()
    origin_valid = valid & np.isfinite(est_xy).all(axis=1) & np.isfinite(ref_xy).all(axis=1)
    if origin_valid.any():
        first = int(np.argmax(origin_valid))
        shifted = est_xy - est_xy[first] + ref_xy[first]
    else:
        first = None
        shifted = est_xy.copy()
    origin_errors = horizontal_errors(shifted, ref_xy, valid)
    finite_idx = np.where(valid)[0]
    chord_report: dict
    if len(finite_idx) < 10:
        chord_report = {"valid": False, "reason": "too few synchronized samples"}
        chord_errors = np.full(len(valid), np.nan)
    else:
        chord_result = se2_from_chord(est_xy[finite_idx], ref_xy[finite_idx], chord_m)
        chord_errors = np.full(len(valid), np.nan)
        if chord_result["valid"]:
            aligned = np.full_like(est_xy, np.nan)
            aligned[finite_idx] = chord_result["aligned_xy"]
            chord_errors = horizontal_errors(aligned, ref_xy, valid)
            chord_report = {key: value for key, value in chord_result.items() if key != "aligned_xy"}
            chord_report["ate"] = ate_summary(chord_errors)
        else:
            chord_report = {key: value for key, value in chord_result.items() if key != "aligned_xy"}
    global_fit = None
    if valid.sum() > 10:
        fit = diagnostic_global_se2(est_xy[valid], ref_xy[valid])
        global_fit = {key: value for key, value in fit.items() if key != "aligned_xy"}
        global_fit["note"] = "whole-trajectory rigid fit; diagnostic only, not the reported alignment"
    rpe = [
        _strip_series(relative_displacement_error(est_xy, ref_xy, reference.time_s, valid, horizon, rpe_tolerance_s))
        for horizon in rpe_horizons_s
    ]
    distance_rpe = [
        relative_error_at_distance(est_xy, ref_xy, distance, valid, horizon, tolerance_m=max(5.0, 0.1 * horizon))
        for horizon in distance_horizons_m
    ]
    alt_stats = None
    if est_alt is not None:
        alt_delta = est_alt - reference.altitude_m
        alt_delta[~valid] = np.nan
        finite = alt_delta[np.isfinite(alt_delta)]
        alt_stats = {
            "n": int(len(finite)),
            "median_m": float(np.median(finite)) if len(finite) else None,
            "mean_m": float(finite.mean()) if len(finite) else None,
            "min_m": float(finite.min()) if len(finite) else None,
            "max_m": float(finite.max()) if len(finite) else None,
            "note": "exported altitude minus GNSS altitude; not part of horizontal ATE",
        }
    course = course_deg(est_xy[:, 0], est_xy[:, 1])
    heading_mask = valid & np.isfinite(course) & np.isfinite(reference.heading_from_baseline_deg) & (reference.speed_m_s > 2.0)
    heading_residual = wrap_deg(course - reference.heading_from_baseline_deg)
    heading_abs = np.abs(heading_residual[heading_mask]) if heading_mask.any() else np.array([])
    return {
        "name": name,
        "position_source": position_source,
        "time": time_info,
        "n_valid": int(valid.sum()),
        "valid_fraction": float(valid.mean()),
        "estimate_path_length_m": _consecutive_path_length(est_xy, valid),
        "reference_path_length_on_valid_m": _consecutive_path_length(ref_xy, valid),
        "start_offset_m": None
        if first is None
        else float(np.linalg.norm(est_xy[first] - ref_xy[first])),
        "identity_ate": ate_summary(errors),
        "origin_only_ate": ate_summary(origin_errors),
        "initial_chord_alignment": chord_report,
        "diagnostic_global_se2": global_fit,
        "rpe_time": rpe,
        "rpe_distance": distance_rpe,
        "drift": drift_summary(errors, distance),
        "segments_500m": error_versus_distance(errors, distance, segment_m),
        "failure_events": [failure_events(errors, reference.time_s, valid, threshold) for threshold in failure_thresholds_m],
        "altitude_vs_gnss": alt_stats,
        "course_vs_baseline_heading": {
            "n": int(heading_mask.sum()),
            "median_abs_deg": float(np.median(heading_abs)) if len(heading_abs) else None,
            "note": "estimate course over ground versus heading derived from the antenna baseline",
        },
        "rotation_column": _rotation_diagnostic(rotation_deg, est_xy, reference.heading_from_baseline_deg, valid),
        "_series": {
            "error_m": errors,
            "distance_m": distance,
            "xy": est_xy,
            "valid": valid,
        },
    }


def _strip_series(result: dict) -> dict:
    return {key: value for key, value in result.items() if not key.endswith("_series_m")}


def prepare_case(
    trajectory: Trajectory,
    reference: ReferenceTrajectory,
    position_source: str,
    origin_unix_s: float | None,
    max_gap_s: float,
) -> dict:
    unix, info = odometry_unix_time(trajectory, origin_unix_s)
    resampled = resample_onto_reference(unix, trajectory.xy(position_source), trajectory.altitude_m, reference.time_s, max_gap_s)
    rotation = None
    if trajectory.rotation_angle_deg is not None:
        pos = np.searchsorted(unix, reference.time_s, side="left")
        left = np.clip(pos - 1, 0, len(unix) - 1)
        right = np.clip(pos, 0, len(unix) - 1)
        use_right = np.abs(unix[right] - reference.time_s) < np.abs(unix[left] - reference.time_s)
        choice = np.where(use_right, right, left)
        rotation = trajectory.rotation_angle_deg[choice]
        rotation[~resampled["valid"]] = np.nan
    return {
        "xy": resampled["xy"],
        "altitude_m": resampled["altitude_m"],
        "valid": resampled["valid"],
        "time_info": info,
        "rotation_deg": rotation,
        "unix_time_s": unix,
    }


def representation_motion_disagreement(
    xy_a: np.ndarray,
    xy_b: np.ndarray,
    time_s: np.ndarray,
    valid: np.ndarray,
) -> dict:
    """Relative-motion difference between two columns of the same export."""
    both = valid & np.isfinite(xy_a).all(axis=1) & np.isfinite(xy_b).all(axis=1)
    absolute = np.linalg.norm(xy_a - xy_b, axis=1)
    absolute[~both] = np.nan
    relative = relative_displacement_error(xy_a, xy_b, time_s, both, 1.0, 0.15)
    return {
        "absolute_horizontal_m": ate_summary(absolute),
        "relative_1s": _strip_series(relative),
    }


def nearest_pole_distances(track_xy: np.ndarray, poles_xy: np.ndarray) -> dict:
    # Subsample the track; 10 Hz over 10 km does not need every sample for a proximity check.
    step = max(1, len(track_xy) // 1000)
    track = track_xy[::step]
    distances = []
    for pole in poles_xy:
        distances.append(float(np.min(np.linalg.norm(track - pole, axis=1))))
    distances_arr = np.asarray(distances)
    return {
        "n_poles": int(len(poles_xy)),
        "median_nearest_track_distance_m": float(np.median(distances_arr)),
        "p10_m": float(np.percentile(distances_arr, 10)),
        "min_m": float(distances_arr.min()),
        "fraction_within_30m": float(np.mean(distances_arr <= 30.0)),
        "fraction_within_15m": float(np.mean(distances_arr <= 15.0)),
    }


def public_summary(result: dict) -> dict:
    return {key: value for key, value in result.items() if not key.startswith("_")}


def _consecutive_path_length(xy: np.ndarray, valid: np.ndarray) -> float:
    total = 0.0
    for index in range(1, len(xy)):
        if valid[index] and valid[index - 1]:
            total += float(np.linalg.norm(xy[index] - xy[index - 1]))
    return total


def disagreement_record(trajectory: Trajectory) -> dict | None:
    return column_disagreement_m(trajectory)
