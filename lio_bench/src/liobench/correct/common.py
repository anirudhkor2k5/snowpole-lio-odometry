"""Pose tracks, frames, the chronological split as time intervals, run output."""
from __future__ import annotations

import json
import platform
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from ..bag import fit_sensor_clock, sensor_to_unix
from ..io import load_ground_truth, load_tum, run_dir, run_meta


def to_matrices(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    T = np.tile(np.eye(4), (len(p), 1, 1))
    T[:, :3, :3] = Rotation.from_quat(q).as_matrix()
    T[:, :3, 3] = p
    return T


def to_pq(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return T[:, :3, 3].copy(), Rotation.from_matrix(T[:, :3, :3]).as_quat()


def inv(T: np.ndarray) -> np.ndarray:
    """Inverse of one (4,4) or many (N,4,4) rigid transforms."""
    R = np.swapaxes(T[..., :3, :3], -1, -2)
    out = np.zeros_like(T)
    out[..., :3, :3] = R
    out[..., :3, 3] = -(R @ T[..., :3, 3, None])[..., 0]
    out[..., 3, 3] = 1.0
    return out


@dataclass
class PoseTrack:
    """Poses of the IMU body in an estimator's world, on the Ouster sensor clock."""
    t: np.ndarray
    T: np.ndarray

    def at(self, tq, max_gap: float = 0.25) -> tuple[np.ndarray, np.ndarray]:
        """Interpolated poses (linear position, geodesic rotation); ok=False outside or across gaps."""
        tq = np.atleast_1d(np.asarray(tq, float))
        i = np.clip(np.searchsorted(self.t, tq) - 1, 0, len(self.t) - 2)
        t0, t1 = self.t[i], self.t[i + 1]
        a = np.clip((tq - t0) / np.maximum(t1 - t0, 1e-9), 0.0, 1.0)
        ok = (tq >= self.t[0] - 1e-6) & (tq <= self.t[-1] + 1e-6) & (t1 - t0 <= max_gap)
        R0, R1 = self.T[i, :3, :3], self.T[i + 1, :3, :3]
        rv = Rotation.from_matrix(np.swapaxes(R0, 1, 2) @ R1).as_rotvec()
        out = np.tile(np.eye(4), (len(tq), 1, 1))
        out[:, :3, :3] = R0 @ Rotation.from_rotvec(rv * a[:, None]).as_matrix()
        out[:, :3, 3] = self.T[i, :3, 3] + a[:, None] * (self.T[i + 1, :3, 3] - self.T[i, :3, 3])
        return out, ok


def baseline_track(algo: str, tag: str = "full") -> PoseTrack:
    """The estimator's own stamps (sensor clock), so they line up with the scans in the bag."""
    tr = load_tum(run_dir(algo, tag) / "odom_sensor_clock.tum")
    keep = tr.t > 0
    t, p, q = tr.t[keep], tr.p[keep], tr.q[keep]
    t, u = np.unique(t, return_index=True)
    return PoseTrack(t, to_matrices(p[u], q[u]))


def lidar_in_imu(cfg: dict) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = np.asarray(cfg["sensor"]["lidar_in_imu_R"], float).reshape(3, 3)
    T[:3, 3] = cfg["sensor"]["lidar_in_imu_t"]
    return T


def split_intervals(cfg: dict) -> dict[str, tuple[float, float]]:
    """The fusion model's chronological split (fusion.chronological_split on the 10 Hz
    ground-truth rows) as Unix-time intervals, so every fitted quantity in phase 2
    sees exactly the same training block as the learned models."""
    from ..fusion import chronological_split
    gt = load_ground_truth(cfg)
    s = cfg["split"]
    tr, va, te = chronological_split(len(gt.t), gt.t, s["train"], s["val"], s["gap_s"])
    return {name: (float(gt.t[r[0]]), float(gt.t[r[-1]])) for name, r in (("train", tr), ("val", va), ("test", te))}


def unix_to_sensor(t_unix, clock: dict) -> np.ndarray:
    """Exact inverse of bag.sensor_to_unix: u = s + off + d (s - s0)  =>  s = (u - off + d s0) / (1 + d)."""
    t_unix = np.asarray(t_unix, float)
    d = clock["drift"]
    return (t_unix - clock["offset_s"] + d * clock["stamp0"]) / (1.0 + d)


def write_run(name: str, tag: str, t_sensor: np.ndarray, T: np.ndarray, meta: dict) -> Path:
    """Same layout as the ROS / GenZ runners, so load_run() and every report pick it up."""
    out = run_dir(name, tag)
    out.mkdir(parents=True, exist_ok=True)
    p, q = to_pq(T)
    with open(out / "odom_sensor_clock.tum", "w") as f:
        f.write("# timestamp x y z qx qy qz qw  (IMU body, sensor clock)\n")
        for ti, pi, qi in zip(t_sensor, p, q):
            f.write(f"{ti:.9f} {pi[0]:.6f} {pi[1]:.6f} {pi[2]:.6f} {qi[0]:.9f} {qi[1]:.9f} {qi[2]:.9f} {qi[3]:.9f}\n")
    (out / "trajectory_tum.txt").unlink(missing_ok=True)     # stale Unix-time cache
    meta = {"algo": name, "tag": tag, "start_offset_s": 0.0,
            "machine": {"arch": platform.machine()}, **meta}
    (out / "meta.json").write_text(json.dumps(meta, indent=2, default=float))
    return out


def inherit_frame(baseline: str, tag: str) -> dict:
    """A corrected trajectory lives in its baseline's world frame."""
    m = run_meta(baseline, tag)
    return {"world_frame": m.get("world_frame", "gravity"), "baseline": baseline, "baseline_tag": tag}


def sensor_times_to_unix(t_sensor, cfg: dict) -> np.ndarray:
    return sensor_to_unix(t_sensor, fit_sensor_clock(cfg))
