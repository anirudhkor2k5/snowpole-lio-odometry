"""Coordinate transforms.

The snow-pole pipeline treats planar positions as EPSG:32633
(WGS84 / UTM zone 33N), including where the site longitude is inside
zone 32. This module follows that declared frame. It does not estimate
a time-varying alignment.
"""

from __future__ import annotations

import math

import numpy as np

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
UTM33_LON0_DEG = 15.0
UTM_K0 = 0.9996
UTM_FALSE_EASTING = 500000.0
UTM_FALSE_NORTHING = 0.0

_PYPROJ = None


def _pyproj_transformer():
    global _PYPROJ
    if _PYPROJ is False:
        return None
    if _PYPROJ is not None:
        return _PYPROJ
    try:
        from pyproj import Transformer

        _PYPROJ = Transformer.from_crs("EPSG:4326", "EPSG:32633", always_xy=True)
    except Exception:
        _PYPROJ = False
        return None
    return _PYPROJ


def _tm_forward(lat_deg: np.ndarray, lon_deg: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """WGS84 geographic to UTM 33N. Fallback used only when pyproj is absent."""
    a = WGS84_A
    f = WGS84_F
    e2 = f * (2.0 - f)
    ep2 = e2 / (1.0 - e2)
    lat = np.radians(np.asarray(lat_deg, dtype=np.float64))
    lon = np.radians(np.asarray(lon_deg, dtype=np.float64))
    lon0 = math.radians(UTM33_LON0_DEG)
    n = a / np.sqrt(1.0 - e2 * np.sin(lat) ** 2)
    t = np.tan(lat) ** 2
    c = ep2 * np.cos(lat) ** 2
    aa = (lon - lon0) * np.cos(lat)
    m = a * (
        (1 - e2 / 4 - 3 * e2**2 / 64 - 5 * e2**3 / 256) * lat
        - (3 * e2 / 8 + 3 * e2**2 / 32 + 45 * e2**3 / 1024) * np.sin(2 * lat)
        + (15 * e2**2 / 256 + 45 * e2**3 / 1024) * np.sin(4 * lat)
        - (35 * e2**3 / 3072) * np.sin(6 * lat)
    )
    easting = UTM_FALSE_EASTING + UTM_K0 * n * (
        aa
        + (1 - t + c) * aa**3 / 6
        + (5 - 18 * t + t**2 + 72 * c - 58 * ep2) * aa**5 / 120
    )
    northing = UTM_FALSE_NORTHING + UTM_K0 * (
        m
        + n
        * np.tan(lat)
        * (
            aa**2 / 2
            + (5 - t + 9 * c + 4 * c**2) * aa**4 / 24
            + (61 - 58 * t + t**2 + 600 * c - 330 * ep2) * aa**6 / 720
        )
    )
    return easting, northing


def wgs84_to_utm33(lat_deg: np.ndarray, lon_deg: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Convert WGS84 latitude/longitude in degrees to UTM 33N easting/northing in metres.

    Longitude is the east/west geographic coordinate. pyproj is preferred.
    """
    lat = np.asarray(lat_deg, dtype=np.float64)
    lon = np.asarray(lon_deg, dtype=np.float64)
    transformer = _pyproj_transformer()
    if transformer is not None:
        easting, northing = transformer.transform(lon, lat)
        return np.asarray(easting, dtype=np.float64), np.asarray(northing, dtype=np.float64)
    return _tm_forward(lat, lon)


def wrap_deg(angle_deg: np.ndarray | float) -> np.ndarray:
    """Wrap degrees to (-180, 180]."""
    return (np.asarray(angle_deg, dtype=np.float64) + 180.0) % 360.0 - 180.0


def wrap_360(angle_deg: np.ndarray | float) -> np.ndarray:
    """Wrap degrees to [0, 360)."""
    return np.asarray(angle_deg, dtype=np.float64) % 360.0


def course_deg(easting_m: np.ndarray, northing_m: np.ndarray) -> np.ndarray:
    """Travel direction at the destination sample, degrees clockwise from north.

    This is course over ground from consecutive positions. It is not body yaw.
    Sample 0 is NaN because no previous position exists.
    """
    easting = np.asarray(easting_m, dtype=np.float64)
    northing = np.asarray(northing_m, dtype=np.float64)
    de = np.diff(easting, prepend=np.nan)
    dn = np.diff(northing, prepend=np.nan)
    course = np.degrees(np.arctan2(de, dn))
    course = wrap_360(course)
    course[~np.isfinite(de) | ~np.isfinite(dn)] = np.nan
    course[0] = np.nan
    return course


def rotate_clockwise_about(xy: np.ndarray, origin_xy: np.ndarray, clockwise_deg: float) -> np.ndarray:
    """Rotate east/north points clockwise about origin. Angle is in degrees."""
    points = np.asarray(xy, dtype=np.float64)
    origin = np.asarray(origin_xy, dtype=np.float64)
    delta = points - origin
    angle = math.radians(clockwise_deg)
    c = math.cos(angle)
    s = math.sin(angle)
    east = delta[:, 0] * c + delta[:, 1] * s
    north = -delta[:, 0] * s + delta[:, 1] * c
    return np.column_stack([east, north]) + origin


def se2_from_chord(
    source_xy: np.ndarray,
    target_xy: np.ndarray,
    chord_m: float,
) -> dict:
    """Yaw and translation that match the initial chord, then the first point.

    Yaw is the clockwise angle from the source chord to the target chord.
    The chord ends at the first sample where both polylines have travelled
    at least ``chord_m`` from their own starts. No later samples are used.
    """
    source = np.asarray(source_xy, dtype=np.float64)
    target = np.asarray(target_xy, dtype=np.float64)
    if len(source) != len(target):
        raise ValueError("chord alignment requires synchronized sequences")
    src_dist = _cumdist(source)
    tgt_dist = _cumdist(target)
    ready = np.where((src_dist >= chord_m) & (tgt_dist >= chord_m))[0]
    if len(ready) == 0:
        return {
            "valid": False,
            "reason": "neither polyline reaches the requested initial chord",
            "clockwise_deg": 0.0,
            "end_index": None,
        }
    end = int(ready[0])
    src_chord = source[end] - source[0]
    tgt_chord = target[end] - target[0]
    src_yaw = math.degrees(math.atan2(src_chord[0], src_chord[1]))
    tgt_yaw = math.degrees(math.atan2(tgt_chord[0], tgt_chord[1]))
    yaw = float(wrap_deg(tgt_yaw - src_yaw))
    rotated = rotate_clockwise_about(source, source[0], yaw)
    translation = target[0] - rotated[0]
    aligned = rotated + translation
    return {
        "valid": True,
        "reason": "initial chord only",
        "clockwise_deg": yaw,
        "end_index": end,
        "source_chord_m": float(np.linalg.norm(src_chord)),
        "target_chord_m": float(np.linalg.norm(tgt_chord)),
        "translation_m": translation.astype(float).tolist(),
        "aligned_xy": aligned,
    }


def diagnostic_global_se2(source_xy: np.ndarray, target_xy: np.ndarray) -> dict:
    """Single rigid SE(2), no scale, fit on the whole sequence.

    This is a shape diagnostic. It is not the evaluation alignment: fitting
    the whole test trajectory removes a constant frame offset that the
    experiment may need to expose.
    """
    source = np.asarray(source_xy, dtype=np.float64)
    target = np.asarray(target_xy, dtype=np.float64)
    src_mean = source.mean(axis=0)
    tgt_mean = target.mean(axis=0)
    p = source - src_mean
    q = target - tgt_mean
    h = p.T @ q
    u, _, vt = np.linalg.svd(h)
    rot = vt.T @ u.T
    if np.linalg.det(rot) < 0:
        vt = vt.copy()
        vt[-1, :] *= -1
        rot = vt.T @ u.T
    aligned = (rot @ (source - src_mean).T).T + tgt_mean
    # Clockwise yaw of the math rotation that maps source to target.
    # rot is applied as [[e'],[n']] = R @ [[e],[n]].
    clockwise = math.degrees(math.atan2(rot[0, 1], rot[1, 1]))
    residual = np.linalg.norm(aligned - target, axis=1)
    return {
        "clockwise_deg": float(wrap_deg(clockwise)),
        "rmse_m": float(np.sqrt(np.mean(residual**2))),
        "median_m": float(np.median(residual)),
        "aligned_xy": aligned,
    }


def _cumdist(xy: np.ndarray) -> np.ndarray:
    step = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    out = np.zeros(len(xy), dtype=np.float64)
    if len(step):
        out[1:] = np.cumsum(step)
    return out


def path_length_m(xy: np.ndarray) -> float:
    if len(xy) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
