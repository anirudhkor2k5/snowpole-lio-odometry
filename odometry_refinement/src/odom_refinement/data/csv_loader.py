"""Read-only loaders for the existing odometry and GNSS CSV exports.

The original files are never opened for writing.
"""

from __future__ import annotations

import csv
import hashlib
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from odom_refinement.data.transforms import wgs84_to_utm33

ODOMETRY_EXTRA = (
    "easting",
    "northing",
    "translation_easting",
    "translation_northing",
    "rotation_angle_degrees",
    "transformed_easting",
    "transformed_northing",
    "error",
)


@dataclass
class Trajectory:
    """One exported trajectory.

    ``easting_wgs84_m`` / ``northing_wgs84_m`` are recomputed from latitude and
    longitude in EPSG:32633. ``easting_stored_m`` is the CSV column when present.
    Those two representations are not assumed to be the same trajectory.
    """

    name: str
    source_path: str
    sha256: str
    time_s: np.ndarray
    time_basis: str
    latitude_deg: np.ndarray
    longitude_deg: np.ndarray
    altitude_m: np.ndarray
    header_seq: np.ndarray
    easting_wgs84_m: np.ndarray
    northing_wgs84_m: np.ndarray
    easting_stored_m: np.ndarray | None = None
    northing_stored_m: np.ndarray | None = None
    rotation_angle_deg: np.ndarray | None = None
    translation_easting_m: np.ndarray | None = None
    translation_northing_m: np.ndarray | None = None
    transformed_easting_m: np.ndarray | None = None
    transformed_northing_m: np.ndarray | None = None
    exported_error_m: np.ndarray | None = None
    status: np.ndarray | None = None
    service: np.ndarray | None = None
    covariance_type: np.ndarray | None = None
    covariance_diag_m2: np.ndarray | None = None
    empty_counts: dict[str, int] = field(default_factory=dict)
    columns: tuple[str, ...] = ()

    @property
    def n_rows(self) -> int:
        return int(len(self.time_s))

    def xy(self, source: str) -> np.ndarray:
        if source == "wgs84_utm33":
            return np.column_stack([self.easting_wgs84_m, self.northing_wgs84_m])
        if source == "stored_en":
            if self.easting_stored_m is None or self.northing_stored_m is None:
                raise ValueError(f"{self.name} has no stored easting/northing")
            return np.column_stack([self.easting_stored_m, self.northing_stored_m])
        raise ValueError(f"unknown position source {source}")

    def available_sources(self) -> tuple[str, ...]:
        sources = ["wgs84_utm33"]
        if self.easting_stored_m is not None and np.isfinite(self.easting_stored_m).any():
            sources.append("stored_en")
        return tuple(sources)


def file_sha256(path: Path, chunk: int = 1 << 20) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while True:
            block = handle.read(chunk)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def _column(rows: list[dict[str, str]], key: str) -> np.ndarray:
    out = np.empty(len(rows), dtype=np.float64)
    for i, row in enumerate(rows):
        raw = row.get(key, "")
        if raw is None or raw == "":
            out[i] = np.nan
        else:
            out[i] = float(raw)
    return out


def load_navsat_csv(path: Path, name: str) -> Trajectory:
    """Load a bagpy-style NavSatFix CSV, with optional odometry columns."""
    path = Path(path)
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path} has no header")
        columns = tuple(reader.fieldnames)
        rows = list(reader)
    empty = {
        column: sum(1 for row in rows if row.get(column) in (None, ""))
        for column in columns
    }
    time_s = _column(rows, "Time")
    latitude = _column(rows, "latitude")
    longitude = _column(rows, "longitude")
    easting, northing = wgs84_to_utm33(latitude, longitude)
    basis = "unix" if np.nanmedian(time_s) > 1.0e9 else "relative"
    stored_e = _column(rows, "easting") if "easting" in columns else None
    stored_n = _column(rows, "northing") if "northing" in columns else None
    cov = None
    if "position_covariance_0" in columns:
        cov = np.column_stack(
            [
                _column(rows, "position_covariance_0"),
                _column(rows, "position_covariance_4"),
                _column(rows, "position_covariance_8"),
            ]
        )
    return Trajectory(
        name=name,
        source_path=str(path.resolve()),
        sha256=file_sha256(path),
        time_s=time_s,
        time_basis=basis,
        latitude_deg=latitude,
        longitude_deg=longitude,
        altitude_m=_column(rows, "altitude"),
        header_seq=_column(rows, "header.seq"),
        easting_wgs84_m=easting,
        northing_wgs84_m=northing,
        easting_stored_m=stored_e,
        northing_stored_m=stored_n,
        rotation_angle_deg=_column(rows, "rotation_angle_degrees") if "rotation_angle_degrees" in columns else None,
        translation_easting_m=_column(rows, "translation_easting") if "translation_easting" in columns else None,
        translation_northing_m=_column(rows, "translation_northing") if "translation_northing" in columns else None,
        transformed_easting_m=_column(rows, "transformed_easting") if "transformed_easting" in columns else None,
        transformed_northing_m=_column(rows, "transformed_northing") if "transformed_northing" in columns else None,
        exported_error_m=_column(rows, "error") if "error" in columns else None,
        status=_column(rows, "status.status") if "status.status" in columns else None,
        service=_column(rows, "status.service") if "status.service" in columns else None,
        covariance_type=_column(rows, "position_covariance_type") if "position_covariance_type" in columns else None,
        covariance_diag_m2=cov,
        empty_counts={key: value for key, value in empty.items() if value},
        columns=columns,
    )


def load_poles(path: Path) -> np.ndarray:
    """Load mapped snow-pole positions as easting, northing in the file's UTM columns."""
    easting: list[float] = []
    northing: list[float] = []
    with Path(path).open(newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path} has no header")
        fields = {name.strip(): name for name in reader.fieldnames}
        north_key = fields.get("UTM33-Nord")
        east_key = fields.get("UTM33-Øst") or fields.get("UTM33-Ost")
        if north_key is None or east_key is None:
            raise ValueError(f"unexpected pole columns: {reader.fieldnames}")
        for row in reader:
            northing.append(float(row[north_key]))
            easting.append(float(row[east_key]))
    return np.column_stack([easting, northing])


def column_disagreement_m(trajectory: Trajectory) -> dict | None:
    """Horizontal distance between stored easting/northing and lat/lon projected to UTM 33N."""
    if trajectory.easting_stored_m is None:
        return None
    finite = np.isfinite(trajectory.easting_stored_m) & np.isfinite(trajectory.easting_wgs84_m)
    if not finite.any():
        return {"n": 0}
    delta = np.hypot(
        trajectory.easting_stored_m[finite] - trajectory.easting_wgs84_m[finite],
        trajectory.northing_stored_m[finite] - trajectory.northing_wgs84_m[finite],
    )
    return {
        "n": int(finite.sum()),
        "median_m": float(np.median(delta)),
        "mean_m": float(delta.mean()),
        "p95_m": float(np.percentile(delta, 95)),
        "max_m": float(delta.max()),
    }


def series_summary(values: np.ndarray) -> dict:
    finite = np.asarray(values, dtype=np.float64)
    finite = finite[np.isfinite(finite)]
    if len(finite) == 0:
        return {"n": 0}
    return {
        "n": int(len(finite)),
        "min": float(finite.min()),
        "max": float(finite.max()),
        "mean": float(finite.mean()),
        "std": float(finite.std()),
        "median": float(np.median(finite)),
    }


def time_summary(time_s: np.ndarray) -> dict:
    time_s = np.asarray(time_s, dtype=np.float64)
    dt = np.diff(time_s)
    finite = dt[np.isfinite(dt)]
    return {
        "n": int(len(time_s)),
        "t0": float(time_s[0]),
        "t1": float(time_s[-1]),
        "duration_s": float(time_s[-1] - time_s[0]),
        "duplicate_timestamps": int(len(time_s) - len(np.unique(time_s))),
        "nonmonotonic_steps": int(np.sum(finite <= 0)),
        "dt_s": {
            "median": float(np.median(finite)) if len(finite) else None,
            "mean": float(finite.mean()) if len(finite) else None,
            "min": float(finite.min()) if len(finite) else None,
            "max": float(finite.max()) if len(finite) else None,
            "p99": float(np.percentile(finite, 99)) if len(finite) else None,
            "n_above_0_5s": int(np.sum(finite > 0.5)),
        },
    }
