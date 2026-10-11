"""Tilt-invariant planar motion: the single biggest correction on this drive.

All four estimators slowly drift in roll/pitch on this highway (body tilt grows
to 35-50 deg over 10 km; the real road is within +-3 deg). On an open road the
scan barely constrains forward motion, so a pitch error that leaks gravity into
the forward axis is indistinguishable from forward acceleration; the local map
bends along with the drift and nothing corrects it. In the drifting world frame
the car then "climbs" kilometres and its horizontal track is ~9 % short, even
though each estimator's own step lengths are right to 0.2 %.

A car moves along its own forward axis and turns about its own vertical axis.
So we rebuild the horizontal track from each estimator's *body-frame* steps:
  forward / lateral = step projected on the body's level axes
  heading change    = relative rotation about the body's vertical axis
The vertical axis in the IMU body frame is the mean specific force over the
training block (the vehicle's average 'up'; ~1.9 deg from IMU z here), which a
car also has online. Nothing is fitted to ground truth.
"""
from __future__ import annotations

import json

import numpy as np
from scipy.spatial.transform import Rotation

from ..io import load_config, repo_path, results_dir
from .common import PoseTrack, baseline_track, inv, split_intervals, write_run


def body_up(cfg: dict) -> np.ndarray:
    """Mean specific force (IMU frame) over the training block, cached in results/bag/body_up.json."""
    f = results_dir("bag") / "body_up.json"
    if f.exists():
        return np.asarray(json.loads(f.read_text())["up"])
    from pathlib import Path
    from rosbags.highlevel import AnyReader
    from ..bag import fit_sensor_clock, sensor_to_unix
    lo, hi = split_intervals(cfg)["train"]
    clock = fit_sensor_clock(cfg)
    acc, n = np.zeros(3), 0
    with AnyReader([Path(repo_path(cfg["reduced_bag"]))]) as r:
        con = [c for c in r.connections if c.topic == cfg["imu_topic"]]
        for c, _, raw in r.messages(connections=con):
            m = r.deserialize(raw, c.msgtype)
            u = float(sensor_to_unix(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, clock))
            if lo <= u <= hi:
                a = m.linear_acceleration
                acc += (a.x, a.y, a.z)
                n += 1
    up = acc / np.linalg.norm(acc)
    f.write_text(json.dumps({"up": up.tolist(), "samples": n, "source": "mean IMU specific force, training block",
                             "tilt_from_imu_z_deg": float(np.degrees(np.arccos(up[2])))}, indent=2))
    return up


def level_axes(up: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Body forward (IMU x projected off `up`) and left (up x forward)."""
    fx = np.array([1.0, 0.0, 0.0]) - up[0] * up
    fx /= np.linalg.norm(fx)
    return fx, np.cross(up, fx)


def body_steps(Ta: np.ndarray, Tb: np.ndarray, up: np.ndarray) -> np.ndarray:
    """(N,3) [forward, lateral, dyaw] of the motion from poses Ta to Tb, in Ta's level body axes."""
    fx, ly = level_axes(up)
    D = inv(Ta) @ Tb
    d = D[:, :3, 3]
    dyaw = Rotation.from_matrix(D[:, :3, :3]).as_rotvec() @ up
    return np.c_[d @ fx, d @ ly, dyaw]


def planarize(track: PoseTrack, up: np.ndarray, x0=(0.0, 0.0), yaw0: float = 0.0) -> np.ndarray:
    """(N,4,4) planar poses rebuilt from the track's body-frame steps."""
    u = body_steps(track.T[:-1], track.T[1:], up)
    th = yaw0 + np.r_[0.0, np.cumsum(u[:, 2])]
    c, s = np.cos(th[:-1]), np.sin(th[:-1])      # the step (incl. its lateral part) is in the start pose's axes
    xy = np.asarray(x0) + np.r_[[[0.0, 0.0]], np.cumsum(np.c_[c * u[:, 0] - s * u[:, 1], s * u[:, 0] + c * u[:, 1]], 0)]
    T = np.tile(np.eye(4), (len(track.t), 1, 1))
    T[:, :2, 3] = xy
    T[:, :3, :3] = Rotation.from_euler("z", th[:, None]).as_matrix()
    return T


def run(cfg: dict | None, method: str, tag: str = "full") -> str:
    cfg = cfg or load_config()
    up = body_up(cfg)
    track = baseline_track(method, tag)
    name = f"{method}+planar"
    write_run(name, "full" if tag in ("full", "full_ros") else tag, track.t, planarize(track, up),
              {"runner": "correct.planar", "role": "corrector", "world_frame": "gravity",
               "baseline": method.split("+")[0], "source_method": method, "source_tag": tag,
               "body_up": up.tolist()})
    return name
