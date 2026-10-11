"""Write causal features and, in a separate file, GNSS-referenced labels."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from _paths import install_src, repository_root

install_src()

from odom_refinement.data.csv_loader import load_navsat_csv  # noqa: E402
from odom_refinement.data.synchronization import build_dual_gnss_reference  # noqa: E402
from odom_refinement.evaluation.baseline import prepare_case  # noqa: E402
from odom_refinement.features.odometry import FEATURE_NAMES, extract_odometry_features  # noqa: E402
from odom_refinement.reliability.labels import label_relative_motion  # noqa: E402
from odom_refinement.reliability.models import chronological_split  # noqa: E402

CASES = (
    ("fasterlio", "fasterlio.csv", "stored_en"),
    ("fasterlio", "fasterlio.csv", "wgs84_utm33"),
    ("fastlio", "fastlio.csv", "stored_en"),
    ("fastlio", "fastlio.csv", "wgs84_utm33"),
    ("fastreg", "fastreg.csv", "stored_en"),
    ("incremental_navigation", "incremental_navigation_results.csv", "stored_en"),
)
HORIZONS_S = (0.5, 1.0, 2.0)
THRESHOLDS_M = (1.0, 2.0, 5.0)


def main() -> None:
    root = repository_root()
    out = root / "experiments" / "features"
    out.mkdir(parents=True, exist_ok=True)
    left = load_navsat_csv(root / "gps_left_position.csv", "gps_left")
    right = load_navsat_csv(root / "gps_right_position.csv", "gps_right")
    reference = build_dual_gnss_reference(left, right)
    cache: dict[tuple[str, str], object] = {}
    for name, filename, source in CASES:
        key = (name, filename)
        if key not in cache:
            cache[key] = load_navsat_csv(root / filename, name)
        trajectory = cache[key]
        prepared = prepare_case(
            trajectory,
            reference,
            source,
            None if trajectory.time_basis == "unix" else float(reference.time_s[0]),
            0.15,
        )
        features = extract_odometry_features(
            reference.time_s,
            prepared["xy"],
            prepared["altitude_m"],
        )
        split = chronological_split(reference.time_s)
        stem = f"{name}__{source}"
        np.savez_compressed(
            out / f"{stem}_features.npz",
            time_s=reference.time_s,
            features=features["features"],
            valid=features["valid"],
            names=np.array(FEATURE_NAMES),
            split=split,
            backend=np.array(name),
            position_source=np.array(source),
            quality_weight=reference.quality_weight,
        )
        label_payload = {
            "time_s": reference.time_s,
            "quality_weight": reference.quality_weight,
            "reference_note": np.array("dual GNSS midpoint, not survey ground truth"),
        }
        for horizon in HORIZONS_S:
            for threshold in THRESHOLDS_M:
                labeled = label_relative_motion(
                    prepared["xy"],
                    reference.xy(),
                    reference.time_s,
                    prepared["valid"],
                    reference.quality_weight,
                    horizon_s=horizon,
                    tolerance_s=0.15,
                    threshold_m=threshold,
                )
                prefix = f"h{horizon:.1f}_t{threshold:.1f}".replace(".", "p")
                label_payload[f"{prefix}_translation_error_m"] = labeled["translation_error_m"]
                label_payload[f"{prefix}_forward_error_m"] = labeled["forward_error_m"]
                label_payload[f"{prefix}_lateral_error_m"] = labeled["lateral_error_m"]
                label_payload[f"{prefix}_excessive"] = labeled["excessive"]
                label_payload[f"{prefix}_usable"] = labeled["usable"]
        np.savez_compressed(out / f"{stem}_labels.npz", **label_payload)
        print(f"wrote {stem}")


if __name__ == "__main__":
    main()
