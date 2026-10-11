"""Matplotlib helpers (headless)."""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

# Fixed colour per method so every figure reads the same way.
COLORS = {
    "ground_truth": "#222222",
    "fast_lio": "#1f77b4", "faster_lio": "#ff7f0e", "dlio": "#2ca02c", "genz_lio": "#d62728",
    "fused": "#9467bd", "fusion": "#9467bd",
}


def color_for(name: str) -> str | None:
    for key, c in COLORS.items():
        if name.startswith(key):
            return c
    return None


def save(fig, path: Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def map_overlay(ref_xy, tracks: dict, path: Path, title: str = ""):
    fig, ax = plt.subplots(figsize=(9, 7))
    ax.plot(ref_xy[:, 0], ref_xy[:, 1], color="k", lw=2.5, label="ground truth", zorder=1)
    for name, xy in tracks.items():
        ls = "--" if "csv" in name else "-"
        ax.plot(xy[:, 0], xy[:, 1], lw=1.2, ls=ls, color=color_for(name), label=name)
    ax.set_aspect("equal")
    ax.set_xlabel("easting [m]")
    ax.set_ylabel("northing [m]")
    ax.set_title(title)
    ax.legend(fontsize=8)
    return save(fig, path)


def series(x, ys: dict, path: Path, xlabel: str, ylabel: str, title: str = "", logy: bool = False):
    fig, ax = plt.subplots(figsize=(11, 4))
    for name, y in ys.items():
        ax.plot(x, y, lw=1, label=name, ls="--" if "csv" in name else "-", color=color_for(name))
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    if logy:
        ax.set_yscale("log")
    ax.set_title(title)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    return save(fig, path)


def colored_map(xy, value, path: Path, label: str, title: str = "", vmax=None):
    fig, ax = plt.subplots(figsize=(9, 7))
    v = np.asarray(value, float)
    sc = ax.scatter(xy[:, 0], xy[:, 1], c=v, s=3, cmap="inferno_r", vmax=vmax or np.nanpercentile(v, 98))
    fig.colorbar(sc, ax=ax, label=label)
    ax.set_aspect("equal")
    ax.set_title(title)
    return save(fig, path)
