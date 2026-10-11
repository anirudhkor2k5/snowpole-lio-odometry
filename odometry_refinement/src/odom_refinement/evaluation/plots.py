"""Plotting for baseline evaluation. Kept separate from the metric definitions."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def _local(xy: np.ndarray, origin: np.ndarray) -> np.ndarray:
    return xy - origin


def plot_trajectories(path: Path, reference_xy: np.ndarray, series: dict[str, dict]) -> None:
    origin = reference_xy[0]
    fig, ax = plt.subplots(figsize=(8.2, 8.2))
    ref = _local(reference_xy, origin)
    ax.plot(ref[:, 0], ref[:, 1], color="black", lw=1.5, label="dual-GNSS midpoint")
    for name, item in series.items():
        xy = _local(item["xy"], origin)
        valid = item["valid"]
        ax.plot(xy[valid, 0], xy[valid, 1], lw=1.0, label=name)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("East of reference start (m)")
    ax.set_ylabel("North of reference start (m)")
    ax.set_title("Trajectories in EPSG:32633, no alignment")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def plot_error_time(path: Path, time_s: np.ndarray, errors: dict[str, np.ndarray]) -> None:
    t0 = time_s[0]
    fig, ax = plt.subplots(figsize=(9, 4.2))
    for name, err in errors.items():
        ax.plot(time_s - t0, err, lw=0.8, label=name)
    ax.set_xlabel("Time from reference start (s)")
    ax.set_ylabel("Horizontal error (m)")
    ax.set_title("Horizontal error versus time, no alignment")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def plot_error_distance(path: Path, distance_m: np.ndarray, errors: dict[str, np.ndarray]) -> None:
    fig, ax = plt.subplots(figsize=(9, 4.2))
    for name, err in errors.items():
        ax.plot(distance_m, err, lw=0.8, label=name)
    ax.set_xlabel("Distance along dual-GNSS midpoint (m)")
    ax.set_ylabel("Horizontal error (m)")
    ax.set_title("Horizontal error versus reference distance")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def plot_segments(path: Path, segments: dict[str, list[dict]]) -> None:
    fig, ax = plt.subplots(figsize=(9, 4.2))
    for name, rows in segments.items():
        if not rows:
            continue
        x = [0.5 * (row["start_m"] + row["stop_m"]) / 1000.0 for row in rows]
        y = [row["rmse"] for row in rows]
        ax.plot(x, y, marker="o", ms=3, lw=1, label=name)
    ax.set_xlabel("Reference distance (km)")
    ax.set_ylabel("Segment RMSE (m)")
    ax.set_title("Horizontal RMSE in 500 m reference segments")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def plot_histograms(path: Path, errors: dict[str, np.ndarray]) -> None:
    fig, ax = plt.subplots(figsize=(8, 4.2))
    for name, err in errors.items():
        finite = err[np.isfinite(err)]
        if len(finite) == 0:
            continue
        ax.hist(finite, bins=40, histtype="step", density=True, label=name)
    ax.set_xlabel("Horizontal error (m)")
    ax.set_ylabel("Density")
    ax.set_title("Horizontal error distribution, no alignment")
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def plot_reference_quality(path: Path, time_s: np.ndarray, baseline_m: np.ndarray, covariance: np.ndarray, weight: np.ndarray) -> None:
    t = time_s - time_s[0]
    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(t, baseline_m, color="tab:blue", lw=0.8)
    axes[0].set_ylabel("Antenna separation (m)")
    axes[0].set_title("Dual-GNSS reference quality")
    axes[1].plot(t, covariance, color="tab:orange", lw=0.8)
    axes[1].set_ylabel("Left cov east (m²)")
    axes[2].plot(t, weight, color="tab:green", lw=0.8)
    axes[2].set_ylabel("Supervision weight")
    axes[2].set_xlabel("Time from reference start (s)")
    for ax in axes:
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def plot_rpe(path: Path, rows: list[dict]) -> None:
    labels = [row["name"] for row in rows]
    values = [row["median_m"] for row in rows]
    fig, ax = plt.subplots(figsize=(9, 4.2))
    ax.bar(range(len(values)), values, color="tab:purple")
    ax.set_xticks(range(len(labels)), labels, rotation=30, ha="right")
    ax.set_ylabel("Median relative displacement error (m)")
    ax.set_title(rows[0]["title"] if rows else "Relative motion")
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
