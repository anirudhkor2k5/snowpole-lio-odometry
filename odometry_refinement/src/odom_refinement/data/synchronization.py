"""Build the dual-GNSS reference and put odometry on that clock.

Ground truth is not inferred. The reference is the midpoint of the two
exported GNSS solutions. A single declared time origin is applied to
relative clocks. The origin is not chosen by minimizing trajectory error.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from odom_refinement.data.csv_loader import Trajectory
from odom_refinement.data.transforms import course_deg, wrap_360, wrap_deg


@dataclass
class ReferenceTrajectory:
    time_s: np.ndarray
    easting_m: np.ndarray
    northing_m: np.ndarray
    altitude_m: np.ndarray
    speed_m_s: np.ndarray
    course_deg: np.ndarray
    baseline_length_m: np.ndarray
    baseline_azimuth_deg: np.ndarray
    heading_from_baseline_deg: np.ndarray
    quality_weight: np.ndarray
    status_left: np.ndarray
    covariance_east_m2: np.ndarray
    left_minus_right_time_s: float

    def xy(self) -> np.ndarray:
        return np.column_stack([self.easting_m, self.northing_m])


def _interp(src_t: np.ndarray, src_y: np.ndarray, dst_t: np.ndarray) -> np.ndarray:
    return np.interp(dst_t, src_t, src_y)


def build_dual_gnss_reference(
    left: Trajectory,
    right: Trajectory,
    *,
    separation_tolerance_m: float = 0.05,
    low_covariance_flag_m2: float = 400.0,
) -> ReferenceTrajectory:
    """Midpoint of the right antenna interpolated onto the left timestamps.

    Linear interpolation is in projected metres. The median left-right stamp
    offset in this recording is tens of microseconds, so this is a
    synchronization choice, not a spatial calibration.

    ``quality_weight`` is a supervision weight, not a calibrated variance.
    NavSat covariance type in the export is UNKNOWN and the up-variance is
    empty, so the covariance values are used only as a relative flag.
    """
    if left.time_basis != "unix" or right.time_basis != "unix":
        raise ValueError("GNSS CSV timestamps are not unix time")
    if np.any(np.diff(left.time_s) <= 0) or np.any(np.diff(right.time_s) <= 0):
        raise ValueError("GNSS timestamps are not strictly increasing")
    right_e = _interp(right.time_s, right.easting_wgs84_m, left.time_s)
    right_n = _interp(right.time_s, right.northing_wgs84_m, left.time_s)
    right_alt = _interp(right.time_s, right.altitude_m, left.time_s)
    east = 0.5 * (left.easting_wgs84_m + right_e)
    north = 0.5 * (left.northing_wgs84_m + right_n)
    alt = 0.5 * (left.altitude_m + right_alt)
    baseline_e = left.easting_wgs84_m - right_e
    baseline_n = left.northing_wgs84_m - right_n
    baseline = np.hypot(baseline_e, baseline_n)
    baseline_azimuth = wrap_360(np.degrees(np.arctan2(baseline_e, baseline_n)))
    dt = np.diff(left.time_s, prepend=np.nan)
    de = np.diff(east, prepend=np.nan)
    dn = np.diff(north, prepend=np.nan)
    speed = np.hypot(de, dn) / dt
    course = course_deg(east, north)
    # The two antennas are lateral: baseline azimuth leads or trails course by
    # about 90 degrees. The sign is chosen from the moving samples so that the
    # derived heading agrees with course over ground. This is vehicle heading
    # only while the car is moving and the baseline length is stable.
    moving = np.isfinite(course) & np.isfinite(speed) & (speed > 2.0) & np.isfinite(baseline_azimuth)
    candidates = []
    for sign in (90.0, -90.0):
        heading = wrap_360(baseline_azimuth + sign)
        residual = np.abs(wrap_deg(heading[moving] - course[moving]))
        candidates.append((float(np.median(residual)), sign, heading))
    _, sign, heading = min(candidates, key=lambda item: item[0])
    heading = heading.copy()
    heading[~np.isfinite(baseline)] = np.nan
    median_baseline = float(np.median(baseline[np.isfinite(baseline)]))
    status = left.status if left.status is not None else np.full(len(left.time_s), np.nan)
    cov0 = (
        left.covariance_diag_m2[:, 0]
        if left.covariance_diag_m2 is not None
        else np.full(len(left.time_s), np.nan)
    )
    weight = np.ones(len(left.time_s), dtype=np.float64)
    weight[status != 2] = 0.2
    weight[np.abs(baseline - median_baseline) > separation_tolerance_m] *= 0.5
    weight[np.isfinite(cov0) & (cov0 > low_covariance_flag_m2)] *= 0.5
    weight[~np.isfinite(east) | ~np.isfinite(north)] = 0.0
    return ReferenceTrajectory(
        time_s=left.time_s.copy(),
        easting_m=east,
        northing_m=north,
        altitude_m=alt,
        speed_m_s=speed,
        course_deg=course,
        baseline_length_m=baseline,
        baseline_azimuth_deg=baseline_azimuth,
        heading_from_baseline_deg=heading,
        quality_weight=weight,
        status_left=status,
        covariance_east_m2=cov0,
        left_minus_right_time_s=float(np.median(left.time_s[: min(len(left.time_s), len(right.time_s))] - right.time_s[: min(len(left.time_s), len(right.time_s))])),
    )


def odometry_unix_time(trajectory: Trajectory, origin_unix_s: float | None) -> tuple[np.ndarray, dict]:
    """Map a trajectory clock onto unix seconds.

    Absolute exports keep their stamps. Relative exports are shifted by a
    caller-supplied origin. The shift is not identified from the position error.
    """
    if trajectory.time_basis == "unix":
        return trajectory.time_s.copy(), {
            "basis": "unix",
            "origin_unix_s": None,
            "applied_offset_s": 0.0,
        }
    if origin_unix_s is None:
        raise ValueError(f"{trajectory.name} has a relative clock and no declared origin")
    offset = float(origin_unix_s - trajectory.time_s[0])
    return trajectory.time_s + offset, {
        "basis": "relative",
        "origin_unix_s": float(origin_unix_s),
        "applied_offset_s": offset,
    }


def resample_onto_reference(
    est_time_s: np.ndarray,
    est_xy: np.ndarray,
    est_alt: np.ndarray | None,
    ref_time_s: np.ndarray,
    max_gap_s: float,
) -> dict:
    """Linearly resample a trajectory onto the reference clock.

    A reference stamp is valid when it lies inside the estimate span and the
    bracketing estimate samples are at most ``max_gap_s`` apart. Samples are
    not extrapolated.
    """
    est_time_s = np.asarray(est_time_s, dtype=np.float64)
    est_xy = np.asarray(est_xy, dtype=np.float64)
    ref_time_s = np.asarray(ref_time_s, dtype=np.float64)
    if np.any(np.diff(est_time_s) <= 0):
        raise ValueError("estimate timestamps are not strictly increasing")
    order = np.searchsorted(est_time_s, ref_time_s, side="right")
    right = np.clip(order, 1, len(est_time_s) - 1)
    left = right - 1
    span = est_time_s[right] - est_time_s[left]
    inside = (ref_time_s >= est_time_s[0]) & (ref_time_s <= est_time_s[-1])
    valid = inside & (span <= max_gap_s) & np.isfinite(span)
    xy = np.full((len(ref_time_s), 2), np.nan, dtype=np.float64)
    xy[:, 0] = np.interp(ref_time_s, est_time_s, est_xy[:, 0])
    xy[:, 1] = np.interp(ref_time_s, est_time_s, est_xy[:, 1])
    xy[~valid] = np.nan
    alt = None
    if est_alt is not None:
        alt = np.interp(ref_time_s, est_time_s, np.asarray(est_alt, dtype=np.float64))
        alt = np.where(valid, alt, np.nan)
    return {"valid": valid, "xy": xy, "altitude_m": alt, "max_bracket_s": float(np.nanmax(span)) if len(span) else None}
