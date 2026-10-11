"""Exploratory reliability models on chronological blocks of the single drive.

Labels are GNSS-referenced. This script does not treat them as survey ground truth.
"""

from __future__ import annotations

import json
import time

import numpy as np

from _paths import install_src, repository_root

install_src()

from odom_refinement.evaluation.resource_metrics import host_description, package_versions, peak_rss_mib  # noqa: E402
from odom_refinement.reliability.experiment import run_reliability_case  # noqa: E402

PRIMARY = ("h1p0_t2p0", 1.0, 2.0)
CASES = (
    "fasterlio__stored_en",
    "fasterlio__wgs84_utm33",
    "fastlio__stored_en",
    "fastlio__wgs84_utm33",
    "fastreg__stored_en",
    "incremental_navigation__stored_en",
)


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
    feature_dir = root / "experiments" / "features"
    out_dir = root / "experiments" / "reliability"
    out_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for stem in CASES:
        features = np.load(feature_dir / f"{stem}_features.npz", allow_pickle=False)
        labels = np.load(feature_dir / f"{stem}_labels.npz", allow_pickle=False)
        prefix = "h1p0_t2p0"
        case = run_reliability_case(
            features["time_s"],
            features["features"],
            features["valid"],
            labels[f"{prefix}_excessive"],
            labels[f"{prefix}_usable"],
            features["quality_weight"],
        )
        case["stem"] = stem
        case["label"] = {
            "horizon_s": PRIMARY[1],
            "threshold_m": PRIMARY[2],
            "definition": "1 s relative displacement error above 2 m versus the dual-GNSS midpoint",
        }
        results.append(case)
        decision = case.get("decision", {})
        print(stem, case.get("blocked") or decision.get("validation_pr_auc"))
    payload = {
        "elapsed_s": time.perf_counter() - started,
        "peak_rss_mib": peak_rss_mib(),
        "host": host_description(),
        "packages": package_versions(),
        "study": "exploratory within-journey chronological split; one recording; GNSS reference, not survey ground truth",
        "primary_label": {
            "horizon_s": 1.0,
            "threshold_m": 2.0,
            "declared_before_test_evaluation": True,
        },
        "neural_temporal_model": "not trained; a lag stack was the temporal comparison. A TCN/GRU was not adopted on one journey.",
        "cases": results,
    }
    destination = out_dir / "reliability_summary.json"
    destination.write_text(json.dumps(payload, indent=2, default=_json_default))
    print(f"wrote {destination}")


if __name__ == "__main__":
    main()
