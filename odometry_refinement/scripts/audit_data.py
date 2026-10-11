"""Phase 0 audit. Reads CSV files and ROS bag indexes. Does not modify them."""

from __future__ import annotations

import json
import subprocess
from pathlib import Path

from _paths import install_src, repository_root

install_src()

from odom_refinement.data.csv_loader import (  # noqa: E402
    load_navsat_csv,
    load_poles,
    series_summary,
    time_summary,
)
from odom_refinement.data.sensor_reader import read_bag_index  # noqa: E402
from odom_refinement.data.synchronization import build_dual_gnss_reference  # noqa: E402
from odom_refinement.data.transforms import wgs84_to_utm33  # noqa: E402
from odom_refinement.evaluation.baseline import disagreement_record, nearest_pole_distances  # noqa: E402
from odom_refinement.evaluation.resource_metrics import host_description, package_versions  # noqa: E402
import numpy as np  # noqa: E402


FILES = {
    "fasterlio": "fasterlio.csv",
    "fastlio": "fastlio.csv",
    "fastreg": "fastreg.csv",
    "gps_left": "gps_left_position.csv",
    "gps_right": "gps_right_position.csv",
    "incremental_navigation": "incremental_navigation_results.csv",
}
BAGS = {
    "reduced": "2024-02-28-12-59-51_no_unwanted_topics.bag",
    "full": "2024-02-28-12-59-51.bag",
}


def _git_commit(root: Path) -> str | None:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    except Exception:
        return None


def _trajectory_record(trajectory) -> dict:
    record = {
        "path": trajectory.source_path,
        "sha256": trajectory.sha256,
        "n_rows": trajectory.n_rows,
        "columns": list(trajectory.columns),
        "empty_counts": trajectory.empty_counts,
        "time_basis": trajectory.time_basis,
        "time": time_summary(trajectory.time_s),
        "latitude_deg": series_summary(trajectory.latitude_deg),
        "longitude_deg": series_summary(trajectory.longitude_deg),
        "altitude_m": series_summary(trajectory.altitude_m),
        "header_seq_start": float(trajectory.header_seq[0]),
        "header_seq_end": float(trajectory.header_seq[-1]),
        "seq_steps_not_one": int(np.sum(np.diff(trajectory.header_seq) != 1)),
        "stored_versus_wgs84_utm33": disagreement_record(trajectory),
        "covariance_populated": bool(
            trajectory.covariance_diag_m2 is not None and np.isfinite(trajectory.covariance_diag_m2).any()
        ),
    }
    if trajectory.rotation_angle_deg is not None:
        record["rotation_angle_deg"] = series_summary(trajectory.rotation_angle_deg)
    if trajectory.status is not None:
        values, counts = np.unique(trajectory.status[np.isfinite(trajectory.status)], return_counts=True)
        record["status_counts"] = {str(int(v)): int(c) for v, c in zip(values, counts)}
    if trajectory.covariance_diag_m2 is not None:
        record["covariance_diag"] = {
            "east": series_summary(trajectory.covariance_diag_m2[:, 0]),
            "north": series_summary(trajectory.covariance_diag_m2[:, 1]),
            "up": series_summary(trajectory.covariance_diag_m2[:, 2]),
        }
        record["covariance_type"] = series_summary(trajectory.covariance_type) if trajectory.covariance_type is not None else None
    if trajectory.translation_easting_m is not None and trajectory.easting_stored_m is not None:
        delta_e = np.diff(trajectory.easting_stored_m)
        delta_n = np.diff(trajectory.northing_stored_m)
        te = trajectory.translation_easting_m[1:]
        tn = trajectory.translation_northing_m[1:]
        mask = np.isfinite(te) & np.isfinite(tn)
        if mask.any():
            residual = np.hypot(te[mask] - delta_e[mask], tn[mask] - delta_n[mask])
            record["translation_matches_position_step"] = {
                "n": int(mask.sum()),
                "median_abs_m": float(np.median(residual)),
                "max_abs_m": float(residual.max()),
            }
    return record


def main() -> None:
    root = repository_root()
    trajectories = {name: load_navsat_csv(root / filename, name) for name, filename in FILES.items()}
    reference = build_dual_gnss_reference(trajectories["gps_left"], trajectories["gps_right"])
    poles = load_poles(root / "Groundtruth_pole_location_at_test_site_E39_Hemnekjølen.csv")
    # Confirm the pyproj/fallback conversion against the incremental export, which is internally consistent.
    check_e, check_n = wgs84_to_utm33(
        trajectories["incremental_navigation"].latitude_deg[:1],
        trajectories["incremental_navigation"].longitude_deg[:1],
    )
    bags = {}
    for name, filename in BAGS.items():
        path = root / filename
        if not path.exists():
            bags[name] = {"present": False, "path": str(path)}
            continue
        index = read_bag_index(path)
        bags[name] = {
            "present": True,
            "path": index.path,
            "size_bytes": index.size_bytes,
            "connection_count": index.connection_count_header,
            "chunk_count": index.chunk_count_header,
            "n_chunk_infos": index.n_chunk_infos,
            "n_chunks_with_nonsane_time": index.n_chunks_with_nonsane_time,
            "sane_start_unix_s": index.sane_start_unix_s,
            "sane_end_unix_s": index.sane_end_unix_s,
            "topics": [
                {
                    "name": topic.name,
                    "type": topic.msg_type,
                    "count": topic.message_count,
                    "pure_chunks": topic.pure_chunk_count,
                }
                for topic in index.topics
            ],
            "decoded_payloads": False,
        }
    fastlio = trajectories["fastlio"]
    fastreg = trajectories["fastreg"]
    duplicate = {
        "latitude_max_abs_deg": float(np.max(np.abs(fastlio.latitude_deg - fastreg.latitude_deg))),
        "longitude_max_abs_deg": float(np.max(np.abs(fastlio.longitude_deg - fastreg.longitude_deg))),
        "altitude_max_abs_m": float(np.max(np.abs(fastlio.altitude_m - fastreg.altitude_m))),
        "rotation_max_abs_deg": float(np.max(np.abs(fastlio.rotation_angle_deg - fastreg.rotation_angle_deg))),
        "stored_easting_max_abs_m": float(np.max(np.abs(fastlio.easting_stored_m - fastreg.easting_stored_m))),
        "stored_northing_max_abs_m": float(np.max(np.abs(fastlio.northing_stored_m - fastreg.northing_stored_m))),
    }
    left = trajectories["gps_left"]
    report = {
        "git_commit": _git_commit(root),
        "host": host_description(),
        "packages": package_versions(),
        "crs": {
            "geographic": "EPSG:4326",
            "projected": "EPSG:32633",
            "reason": "The existing snow-pole pipeline and the pole map use UTM zone 33N.",
            "incremental_first_sample_projection_residual_m": float(
                np.hypot(
                    check_e[0] - trajectories["incremental_navigation"].easting_stored_m[0],
                    check_n[0] - trajectories["incremental_navigation"].northing_stored_m[0],
                )
            ),
        },
        "trajectories": {name: _trajectory_record(item) for name, item in trajectories.items()},
        "fastlio_fastreg_column_copy": duplicate,
        "dual_gnss": {
            "n": int(len(reference.time_s)),
            "duration_s": float(reference.time_s[-1] - reference.time_s[0]),
            "left_minus_right_median_time_s": reference.left_minus_right_time_s,
            "baseline_length_m": series_summary(reference.baseline_length_m),
            "altitude_m": series_summary(reference.altitude_m),
            "quality_weight": series_summary(reference.quality_weight),
            "fraction_weight_below_0_5": float(np.mean(reference.quality_weight < 0.5)),
            "path_length_m": float(np.sum(np.hypot(np.diff(reference.easting_m), np.diff(reference.northing_m)))),
            "start_to_end_m": float(
                np.hypot(reference.easting_m[-1] - reference.easting_m[0], reference.northing_m[-1] - reference.northing_m[0])
            ),
            "provenance": "midpoint of /gps_left_position and /gps_right_position CSV exports; not an independent survey trajectory",
        },
        "poles_versus_gnss_midpoint": nearest_pole_distances(reference.xy(), poles),
        "bags": bags,
        "fasterlio_minus_gps_left_start_s": float(trajectories["fasterlio"].time_s[0] - left.time_s[0]),
        "incremental_minus_gps_left_time_max_abs_s": float(
            np.max(np.abs(trajectories["incremental_navigation"].time_s - left.time_s))
        ),
        "limitations": [
            "No independently extracted survey ground-truth trajectory is in the repository.",
            "NavSat covariance type is UNKNOWN and the up-variance column is empty.",
            "fasterlio, fastlio and fastreg latitude/longitude disagree with their stored easting/northing.",
            "fastreg latitude, longitude, altitude and rotation match fastlio exactly; only stored easting/northing differ.",
            "Odometry altitude is not GNSS ellipsoidal height.",
            "rotation_angle_degrees is not validated as body yaw.",
            "IMU, LiDAR images, point clouds, gps_state, utm_position and vehicle_enu_heading were counted from the bag index and not decoded.",
            "The reduced bag has no /ouster/points topic. The full bag does, mixed into compressed chunks.",
        ],
    }
    out = root / "reports" / "dataset_statistics.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2))
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
