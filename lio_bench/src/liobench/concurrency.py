"""Baseline and support running at the same time (same roscore, same bag stream).

For each estimator, solo run (tag 'full') vs concurrent run (tag
'concurrent_<baseline>+<support>'): per-scan latency, scans without a pose,
CPU and memory, and whether accuracy changed because of the shared machine.
"""
from __future__ import annotations

import json

import numpy as np
import pandas as pd

from .io import load_config, load_ground_truth, load_run, results_dir, run_dir, run_meta
from .metrics import evaluate


def _latency(algo: str, tag: str) -> dict:
    d = run_dir(algo, tag)
    if not (d / "odom_recv.csv").exists() or not (d / "cloud_recv.csv").exists():
        return {}
    o, c = pd.read_csv(d / "odom_recv.csv"), pd.read_csv(d / "cloud_recv.csv")
    if algo == "dlio":
        # DLIO publishes IMU-rate poses from a timer; only scan-aligned latency is meaningful
        return {"note": "DLIO publishes IMU-rate poses; per-scan latency not defined"}
    i = np.searchsorted(c["stamp"].to_numpy(), o["stamp"].to_numpy(), side="right") - 1
    ok = i >= 0
    lat = (o["wall"].to_numpy()[ok] - c["wall"].to_numpy()[i[ok]]) * 1e3
    return {"latency_ms_median": float(np.median(lat)), "latency_ms_p95": float(np.percentile(lat, 95)),
            "latency_ms_max": float(lat.max()), "poses": int(len(o)), "clouds": int(len(c)),
            "scans_without_pose": int(max(0, len(c) - len(o)))}


def _resources(algo: str, tag: str) -> dict:
    f = run_dir(algo, tag) / "resources.csv"
    if not f.exists():
        return {}
    r = pd.read_csv(f)
    r = r[r["algo"] == algo]
    return {"cpu_pct_mean": float(r["cpu_pct"].mean()), "cpu_pct_p95": float(r["cpu_pct"].quantile(0.95)),
            "rss_mb_max": float(r["rss_mb"].max())}


def run(baseline: str, support: str, cfg: dict | None = None) -> dict:
    cfg = cfg or load_config()
    gt = load_ground_truth(cfg)
    tag_c = f"concurrent_{baseline}+{support}"
    rep = {"baseline": baseline, "support": support, "machine": run_meta(baseline, tag_c).get("machine"), "per_algo": {}}
    for a in (baseline, support):
        row = {}
        # like-for-like solo run: GenZ's 'full' is the offline Python run, its ROS solo run is 'full_ros'
        solo = "full_ros" if (run_dir(a, "full_ros") / "odom_sensor_clock.tum").exists() else "full"
        for label, tag in (("solo", solo), ("concurrent", tag_c)):
            tr = load_run(a, tag, cfg)
            if tr is None:
                row[label] = "missing"
                continue
            s = evaluate(tr, gt, "se2_full", cfg["evaluation"]).summary
            row[label] = {"ate_rmse_m": s["ate_xy"].get("rmse"), "drift_pct": s["drift_pct_mean_kitti"],
                          **_latency(a, tag), **_resources(a, tag)}
        rep["per_algo"][a] = row
    out = results_dir("concurrency")
    (out / f"{baseline}+{support}.json").write_text(json.dumps(rep, indent=2, default=float))
    return rep
