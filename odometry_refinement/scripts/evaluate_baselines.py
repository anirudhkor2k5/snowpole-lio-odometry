"""Baseline ATE/RPE evaluation. Writes plots and a JSON summary. Does not modify source CSVs."""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

from _paths import install_src, repository_root

install_src()

from odom_refinement.data.csv_loader import file_sha256, load_navsat_csv  # noqa: E402
from odom_refinement.data.synchronization import build_dual_gnss_reference  # noqa: E402
from odom_refinement.evaluation.baseline import (  # noqa: E402
    disagreement_record,
    evaluate_synchronized,
    prepare_case,
    public_summary,
    representation_motion_disagreement,
)
from odom_refinement.evaluation.plots import (  # noqa: E402
    plot_error_distance,
    plot_error_time,
    plot_histograms,
    plot_reference_quality,
    plot_rpe,
    plot_segments,
    plot_trajectories,
)
from odom_refinement.evaluation.trajectory_metrics import cumulative_distance  # noqa: E402
from odom_refinement.evaluation.resource_metrics import host_description, package_versions, peak_rss_mib  # noqa: E402

MAX_GAP_S = 0.15
CHORD_M = 30.0
RPE_HORIZONS_S = (0.5, 1.0, 2.0, 10.0)
RPE_TOLERANCE_S = 0.15
DISTANCE_HORIZONS_M = (50.0, 100.0, 500.0)
SEGMENT_M = 500.0
FAILURE_THRESHOLDS_M = (5.0, 10.0, 20.0, 50.0, 100.0)

EVAL_FILES = {
    "fasterlio": "fasterlio.csv",
    "fastlio": "fastlio.csv",
    "fastreg": "fastreg.csv",
    "incremental_navigation": "incremental_navigation_results.csv",
}


def _json_default(value):
    if isinstance(value, np.floating):
        return float(value)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.ndarray):
        raise TypeError("refusing to write an ndarray into the summary JSON")
    raise TypeError(type(value))


def main() -> None:
    started = time.perf_counter()
    root = repository_root()
    out_dir = root / "experiments" / "baseline"
    out_dir.mkdir(parents=True, exist_ok=True)
    hashes_before = {name: file_sha256(root / filename) for name, filename in {**EVAL_FILES, "gps_left": "gps_left_position.csv", "gps_right": "gps_right_position.csv"}.items()}
    left = load_navsat_csv(root / "gps_left_position.csv", "gps_left")
    right = load_navsat_csv(root / "gps_right_position.csv", "gps_right")
    reference = build_dual_gnss_reference(left, right)
    trajectories = {name: load_navsat_csv(root / filename, name) for name, filename in EVAL_FILES.items()}
    origins = {
        "reference_start": float(reference.time_s[0]),
        "fasterlio_start": float(trajectories["fasterlio"].time_s[0]),
    }
    cases = []
    plot_xy = {}
    plot_err = {}
    plot_seg = {}
    for name, trajectory in trajectories.items():
        for source in trajectory.available_sources():
            for origin_name, origin in origins.items():
                if trajectory.time_basis == "unix" and origin_name != "reference_start":
                    continue
                prepared = prepare_case(
                    trajectory,
                    reference,
                    source,
                    None if trajectory.time_basis == "unix" else origin,
                    MAX_GAP_S,
                )
                result = evaluate_synchronized(
                    name,
                    source,
                    prepared["xy"],
                    prepared["altitude_m"],
                    reference,
                    prepared["valid"],
                    time_info={**prepared["time_info"], "origin_name": origin_name if trajectory.time_basis == "relative" else "native_unix"},
                    chord_m=CHORD_M,
                    rpe_horizons_s=RPE_HORIZONS_S,
                    rpe_tolerance_s=RPE_TOLERANCE_S,
                    distance_horizons_m=DISTANCE_HORIZONS_M,
                    segment_m=SEGMENT_M,
                    failure_thresholds_m=FAILURE_THRESHOLDS_M,
                    rotation_deg=prepared["rotation_deg"],
                )
                summary = public_summary(result)
                summary["column_disagreement"] = disagreement_record(trajectory)
                cases.append(summary)
                primary_origin = trajectory.time_basis == "unix" or origin_name == "reference_start"
                if primary_origin:
                    label = f"{name}:{source}"
                    plot_xy[label] = {"xy": result["_series"]["xy"], "valid": result["_series"]["valid"]}
                    plot_err[label] = result["_series"]["error_m"]
                    plot_seg[label] = summary["segments_500m"]
        sources = trajectory.available_sources()
        if "stored_en" in sources and "wgs84_utm33" in sources:
            a = prepare_case(trajectory, reference, "stored_en", None if trajectory.time_basis == "unix" else origins["reference_start"], MAX_GAP_S)
            b = prepare_case(trajectory, reference, "wgs84_utm33", None if trajectory.time_basis == "unix" else origins["reference_start"], MAX_GAP_S)
            both = a["valid"] & b["valid"]
            cases.append(
                {
                    "name": name,
                    "position_source": "stored_en_versus_wgs84_utm33",
                    "time": {"origin_name": "reference_start" if trajectory.time_basis == "relative" else "native_unix"},
                    "representation_disagreement": representation_motion_disagreement(a["xy"], b["xy"], reference.time_s, both),
                }
            )
    latlon_duplicate = bool(
        np.max(np.abs(trajectories["fastlio"].latitude_deg - trajectories["fastreg"].latitude_deg)) == 0.0
        and np.max(np.abs(trajectories["fastlio"].longitude_deg - trajectories["fastreg"].longitude_deg)) == 0.0
    )
    plot_trajectories(out_dir / "trajectories_unaligned.png", reference.xy(), plot_xy)
    plot_error_time(out_dir / "error_vs_time.png", reference.time_s, plot_err)
    distance = cumulative_distance(reference.xy(), np.ones(len(reference.time_s), dtype=bool))
    plot_error_distance(out_dir / "error_vs_distance.png", distance, plot_err)
    plot_segments(out_dir / "segment_rmse.png", plot_seg)
    plot_histograms(out_dir / "error_histograms.png", plot_err)
    plot_reference_quality(
        out_dir / "reference_quality.png",
        reference.time_s,
        reference.baseline_length_m,
        reference.covariance_east_m2,
        reference.quality_weight,
    )
    rpe_rows = []
    for case in cases:
        if "rpe_time" not in case:
            continue
        if case["time"].get("origin_name") not in (None, "native_unix", "reference_start"):
            continue
        one = next(item for item in case["rpe_time"] if item["horizon_s"] == 1.0)
        median = one["translation_m"].get("median")
        if median is None:
            continue
        rpe_rows.append(
            {
                "name": f"{case['name']}:{case['position_source']}",
                "median_m": median,
                "title": "Median 1 s relative displacement error",
            }
        )
    plot_rpe(out_dir / "rpe_1s.png", rpe_rows)
    hashes_after = {name: file_sha256(root / filename) for name, filename in {**EVAL_FILES, "gps_left": "gps_left_position.csv", "gps_right": "gps_right_position.csv"}.items()}
    payload = {
        "elapsed_s": time.perf_counter() - started,
        "peak_rss_mib": peak_rss_mib(),
        "host": host_description(),
        "packages": package_versions(),
        "source_sha256_unchanged": hashes_before == hashes_after,
        "config": {
            "max_gap_s": MAX_GAP_S,
            "initial_chord_m": CHORD_M,
            "rpe_horizons_s": RPE_HORIZONS_S,
            "distance_horizons_m": DISTANCE_HORIZONS_M,
            "segment_m": SEGMENT_M,
            "failure_thresholds_m": FAILURE_THRESHOLDS_M,
            "relative_time_primary_origin": "gps_left start",
            "relative_time_sensitivity_origin": "fasterlio first stamp",
        },
        "fastreg_latlon_duplicates_fastlio": latlon_duplicate,
        "reference": "dual GNSS midpoint in EPSG:32633; not survey ground truth",
        "cases": cases,
    }
    # The botched distance call above may have referenced a missing helper. The real plot uses `distance`.
    destination = out_dir / "baseline_summary.json"
    destination.write_text(json.dumps(payload, indent=2, default=_json_default))
    print(f"wrote {destination}")


if __name__ == "__main__":
    main()
