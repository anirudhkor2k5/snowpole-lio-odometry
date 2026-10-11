"""Readers and writers. Original repository files are only ever read."""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd
import yaml

from . import BENCH_ROOT, REPO_ROOT, RESULTS_ROOT


@dataclass
class Trajectory:
    """Timestamped poses. `q` is (x, y, z, w) and may be absent (position-only)."""

    name: str
    t: np.ndarray                     # (N,) seconds, Unix
    p: np.ndarray                     # (N, 3) metres
    q: Optional[np.ndarray] = None    # (N, 4) xyzw
    meta: dict = field(default_factory=dict)

    def __post_init__(self):
        self.t = np.asarray(self.t, dtype=float)
        self.p = np.asarray(self.p, dtype=float)
        if self.q is not None:
            self.q = np.asarray(self.q, dtype=float)
        order = np.argsort(self.t, kind="stable")
        keep = np.r_[True, np.diff(self.t[order]) > 0]
        idx = order[keep]
        self.t, self.p = self.t[idx], self.p[idx]
        if self.q is not None:
            self.q = self.q[idx]

    def __len__(self) -> int:
        return len(self.t)

    def slice_time(self, t0: float, t1: float) -> "Trajectory":
        m = (self.t >= t0) & (self.t <= t1)
        return Trajectory(self.name, self.t[m], self.p[m], None if self.q is None else self.q[m], dict(self.meta))

    def shifted(self, dt: float) -> "Trajectory":
        return Trajectory(self.name, self.t + dt, self.p, self.q, dict(self.meta, time_shift_s=dt))


# --------------------------------------------------------------------------- config

_DATA_ROOT: Optional[Path] = None


def load_config(path: Optional[Path] = None) -> dict:
    """Dataset config. LIOBENCH_DATA_ROOT (env) overrides `data_root`."""
    global _DATA_ROOT
    path = Path(path or os.environ.get("LIOBENCH_CONFIG", BENCH_ROOT / "configs" / "dataset.yaml"))
    with open(path) as f:
        cfg = yaml.safe_load(f)
    cfg["_path"] = str(path)
    root = Path(os.environ.get("LIOBENCH_DATA_ROOT", cfg.get("data_root", ".")))
    _DATA_ROOT = root if root.is_absolute() else (REPO_ROOT / root).resolve()
    cfg["_data_root"] = str(_DATA_ROOT)
    return cfg


def repo_path(name: str) -> Path:
    """Resolve a dataset file against the data root (see load_config)."""
    p = Path(name)
    if p.is_absolute():
        return p
    return (_DATA_ROOT or REPO_ROOT) / p


def results_dir(*parts: str) -> Path:
    d = RESULTS_ROOT.joinpath(*parts)
    d.mkdir(parents=True, exist_ok=True)
    return d


# --------------------------------------------------------------------------- projections

_TRANSFORMERS: dict = {}


def latlon_to_utm(lat: np.ndarray, lon: np.ndarray, epsg: int = 32633) -> tuple[np.ndarray, np.ndarray]:
    from pyproj import Transformer

    if epsg not in _TRANSFORMERS:
        _TRANSFORMERS[epsg] = Transformer.from_crs(4326, epsg, always_xy=True)
    e, n = _TRANSFORMERS[epsg].transform(np.asarray(lon, float), np.asarray(lat, float))
    return np.asarray(e), np.asarray(n)


# --------------------------------------------------------------------------- CSV inputs

def _stamp(df: pd.DataFrame) -> np.ndarray:
    return df["header.stamp.secs"].to_numpy(float) + df["header.stamp.nsecs"].to_numpy(float) * 1e-9


def load_gnss_altitude(cfg: dict) -> tuple[np.ndarray, np.ndarray]:
    """Mean of the two antenna altitudes on the left-antenna clock."""
    left = pd.read_csv(repo_path(cfg["gnss_left"]))
    right = pd.read_csv(repo_path(cfg["gnss_right"]))
    tl, tr = _stamp(left), _stamp(right)
    alt = 0.5 * (left["altitude"].to_numpy(float) + np.interp(tl, tr, right["altitude"].to_numpy(float)))
    return tl, alt


def load_ground_truth(cfg: dict) -> Trajectory:
    """`groundtruth_extracted.csv`: UTM 33N easting/northing at 10 Hz.

    The file has no altitude, so the dual-GNSS mean altitude is interpolated onto
    its clock. Horizontal metrics never use that z; SE(3) alignment does.
    """
    df = pd.read_csv(repo_path(cfg["ground_truth"]))
    t = _stamp(df)
    e, n = df["easting"].to_numpy(float), df["northing"].to_numpy(float)
    ta, alt = load_gnss_altitude(cfg)
    z = np.interp(t, ta, alt)
    return Trajectory("ground_truth", t, np.c_[e, n, z], None, {"source": cfg["ground_truth"]})


def load_legacy_export(cfg: dict, name: str, column: str = "latlon") -> Trajectory:
    """Legacy FAST-LIO / Faster-LIO CSV exports (bagpy NavSatFix layout).

    column = "latlon"    : latitude/longitude reprojected to UTM 33N
    column = "stored_en" : the `easting`/`northing` columns as stored
    Relative-clock files (fastlio.csv starts at t=0) are mapped onto the GNSS
    clock by putting their first row at the first left-GNSS header stamp.
    """
    spec = cfg["legacy_exports"][name]
    df = pd.read_csv(repo_path(spec["file"]))
    t = _stamp(df)
    if spec.get("clock", "unix") == "relative":
        gl = pd.read_csv(repo_path(cfg["gnss_left"]), usecols=["header.stamp.secs", "header.stamp.nsecs"])
        t = t - t[0] + _stamp(gl)[0]
    if column == "latlon":
        e, n = latlon_to_utm(df["latitude"].to_numpy(float), df["longitude"].to_numpy(float), cfg.get("utm_epsg", 32633))
    elif column == "stored_en":
        e, n = df["easting"].to_numpy(float), df["northing"].to_numpy(float)
    else:
        raise ValueError(column)
    z = df["altitude"].to_numpy(float) if "altitude" in df else np.zeros_like(e)
    z = np.nan_to_num(z)
    return Trajectory(f"{name}_csv_{column}", t, np.c_[e, n, z], None,
                      {"source": spec["file"], "column": column, "kind": "legacy_export"})


# --------------------------------------------------------------------------- TUM files

def load_tum(path: Path, name: Optional[str] = None) -> Trajectory:
    a = np.loadtxt(path, comments="#", ndmin=2)
    if a.shape[1] < 8:
        raise ValueError(f"{path}: expected 8 TUM columns, got {a.shape[1]}")
    return Trajectory(name or Path(path).stem, a[:, 0], a[:, 1:4], a[:, 4:8], {"source": str(path)})


def write_tum(path: Path, traj: Trajectory) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    q = traj.q if traj.q is not None else np.tile([0.0, 0.0, 0.0, 1.0], (len(traj), 1))
    np.savetxt(path, np.c_[traj.t, traj.p, q], fmt="%.9f", header="timestamp x y z qx qy qz qw")
    return path


def run_dir(algo: str, tag: str = "full") -> Path:
    return RESULTS_ROOT / "runs" / algo / tag


def run_meta(algo: str, tag: str = "full") -> dict:
    f = run_dir(algo, tag) / "meta.json"
    return json.loads(f.read_text()) if f.exists() else {}


def load_run(algo: str, tag: str = "full", cfg: Optional[dict] = None) -> Optional[Trajectory]:
    """Trajectory from one of our own runs (ros/scripts/run_ros.py or genz_runner).

    Runners write the estimator's stamps untouched (Ouster sensor clock) to
    odom_sensor_clock.tum. On first load they are mapped to Unix time with the
    fitted sensor clock and cached as trajectory_tum.txt. For estimators whose
    world frame is the tilted start-up IMU frame, meta['gravity_up_world'] is set
    from the start-up IMU mean so evaluation can level the trajectory.
    """
    from .bag import fit_sensor_clock, sensor_to_unix

    d = run_dir(algo, tag)
    raw, out = d / "odom_sensor_clock.tum", d / "trajectory_tum.txt"
    if not out.exists():
        if not raw.exists() or raw.stat().st_size == 0:
            return None
        clock = fit_sensor_clock(cfg or load_config())
        tr = load_tum(raw)
        keep = tr.t > 0  # DLIO publishes one zero-stamped pose before it initialises
        tr = Trajectory(tr.name, tr.t[keep], tr.p[keep], None if tr.q is None else tr.q[keep])
        if len(tr) == 0:
            return None
        write_tum(out, Trajectory(algo, sensor_to_unix(tr.t, clock), tr.p, tr.q))
    tr = load_tum(out, name=algo if tag == "full" else f"{algo}_{tag}")
    meta = run_meta(algo, tag)
    tr.meta.update(kind="rerun", algo=algo, tag=tag, run=meta)
    if meta.get("world_frame", "gravity") == "first_imu":
        if meta.get("start_offset_s", 0) > 0:
            from .bag import imu_mean_acc
            if "gravity_up_world" not in meta:
                meta["gravity_up_world"] = imu_mean_acc(cfg or load_config(), meta["start_offset_s"])
                (d / "meta.json").write_text(json.dumps(meta, indent=2))
            tr.meta["gravity_up_world"] = meta["gravity_up_world"]
        else:
            tr.meta["gravity_up_world"] = fit_sensor_clock(cfg or load_config())["imu_static_mean_acc"]
    return tr


def available_runs(tag: str = "full") -> list[str]:
    root = RESULTS_ROOT / "runs"
    if not root.exists():
        return []
    return sorted(d.name for d in root.iterdir()
                  if not d.name.startswith("_") and (d / tag / "odom_sensor_clock.tum").exists())


def run_tags(algo: str) -> list[str]:
    d = RESULTS_ROOT / "runs" / algo
    return sorted(t.name for t in d.iterdir() if (t / "odom_sensor_clock.tum").exists()) if d.exists() else []
