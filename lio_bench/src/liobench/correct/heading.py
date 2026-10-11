"""Phase v3: attitude and heading tools on top of any estimator's poses (no point clouds needed).

gravity_filter   online roll/pitch correction toward IMU gravity (exact replica of the scan-pass
                 'grav_only' variant, but fast enough to tune its time constant on train/val)
planar_steps     tilt-invariant [forward, lateral, dyaw] of a pose track (see planar.py)
integrate        rebuild a planar track from steps, with optional
                   - Earth-rate removal   (physics: the gyro measures Earth's rotation, no LIO models it)
                   - yaw-rate bias        (calibrated on the training block against GT heading)
                   - no-side-slip         (vehicle constraint: lateral step = 0)
                   - yaw from another source (hybrid: translation from A, heading from B)
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from ..bag import fit_sensor_clock, sensor_to_unix
from ..io import Trajectory, load_ground_truth, repo_path, results_dir
from .common import PoseTrack, inv, split_intervals
from .planar import body_up, level_axes
from .scanpass import gravity_up, tilt_correct

EARTH_RATE = 7.2921159e-5          # rad/s
SITE_LAT_DEG = 63.24               # E39 Hemnekjølen


def earth_rate_up(lat_deg: float = SITE_LAT_DEG) -> float:
    """Vertical component of Earth's rotation (rad/s), counter-clockwise seen from above."""
    return EARTH_RATE * np.sin(np.radians(lat_deg))


def imu_cache(cfg: dict) -> dict:
    """All IMU samples (sensor clock) from the reduced bag, cached as npz."""
    f = results_dir("bag") / "imu.npz"
    if f.exists():
        z = np.load(f)
        return {k: z[k] for k in z.files}
    from pathlib import Path
    from rosbags.highlevel import AnyReader
    rows = []
    with AnyReader([Path(repo_path(cfg["reduced_bag"]))]) as r:
        con = [c for c in r.connections if c.topic == cfg["imu_topic"]]
        for c, _, raw in r.messages(connections=con):
            m = r.deserialize(raw, c.msgtype)
            a, g = m.linear_acceleration, m.angular_velocity
            rows.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, a.x, a.y, a.z, g.x, g.y, g.z])
    A = np.asarray(rows)
    A = A[np.argsort(A[:, 0])]
    np.savez(f, imu=A)
    return {"imu": A}


def gravity_filter(track: PoseTrack, imu: np.ndarray, tau_s: float, compensate: bool = True,
                   window_s: float = 1.0) -> np.ndarray:
    """(N,4,4) poses: the track's own steps, roll/pitch pulled toward IMU gravity with time constant tau."""
    T = np.empty_like(track.T)
    T[0] = track.T[0]
    t = track.t
    lo = np.searchsorted(imu[:, 0], t - window_s)
    hi = np.searchsorted(imu[:, 0], t, side="right")
    speed_prev = 0.0
    first = True
    for k in range(len(t)):
        if k > 0:
            P = inv(track.T[k - 1]) @ track.T[k]
            T[k] = T[k - 1] @ P
            dt = max(t[k] - t[k - 1], 1e-3)
            speed = float(np.linalg.norm(P[:3, 3]) / dt)
            dvdt = float(np.clip((speed - speed_prev) / dt, -5, 5)) if compensate else 0.0
            speed_prev = speed
        else:
            speed, dvdt = 0.0, 0.0
        u = gravity_up(imu[lo[k]:hi[k]], speed if compensate else 0.0, dvdt)
        if u is None:
            continue
        if first:                                   # initial attitude from gravity, as in the scan pass
            from .scanpass import relevel
            T[k, :3, :3] = relevel(T[k, :3, :3], u)
            first = False
        else:
            alpha = min(1.0, (t[k] - t[k - 1]) / tau_s)
            T[k, :3, :3] = tilt_correct(T[k, :3, :3], u, alpha)
    return T


def planar_steps(track: PoseTrack, up: np.ndarray) -> np.ndarray:
    """(N,3) [fwd, lat, dyaw] between consecutive poses (row 0 = 0), in level body axes."""
    fx, ly = level_axes(up)
    D = inv(track.T[:-1]) @ track.T[1:]
    d = D[:, :3, 3]
    dyaw = Rotation.from_matrix(D[:, :3, :3]).as_rotvec() @ up
    return np.vstack([np.zeros(3), np.c_[d @ fx, d @ ly, dyaw]])


def integrate(t: np.ndarray, u: np.ndarray, x0=(0.0, 0.0), yaw0: float = 0.0, *, earth: bool = False,
              yaw_bias: float = 0.0, no_slip: bool = False, dyaw_override: np.ndarray | None = None) -> np.ndarray:
    """(N,4,4) planar poses from steps u (N,3) at times t (s)."""
    dt = np.r_[0.0, np.diff(t)]
    dyaw = (u[:, 2] if dyaw_override is None else dyaw_override).copy()
    if earth:
        dyaw = dyaw - earth_rate_up() * dt
    dyaw = dyaw - yaw_bias * dt
    lat = np.zeros(len(u)) if no_slip else u[:, 1]
    th = yaw0 + np.cumsum(dyaw)
    th_prev = np.r_[yaw0, th[:-1]]
    c, s = np.cos(th_prev), np.sin(th_prev)
    xy = np.asarray(x0) + np.cumsum(np.c_[c * u[:, 0] - s * lat, s * u[:, 0] + c * lat], 0)
    T = np.tile(np.eye(4), (len(t), 1, 1))
    T[:, :2, 3] = xy
    T[:, :3, :3] = Rotation.from_euler("z", th[:, None]).as_matrix()
    return T


def to_traj(name: str, t_sensor: np.ndarray, T: np.ndarray, cfg: dict) -> Trajectory:
    t = sensor_to_unix(t_sensor, fit_sensor_clock(cfg))
    return Trajectory(name, t, T[:, :3, 3].copy(), Rotation.from_matrix(T[:, :3, :3]).as_quat(), {"world_frame": "gravity"})


def yaw_bias_on_train(t_sensor: np.ndarray, dyaw: np.ndarray, cfg: dict, dt_clock: float) -> float:
    """Constant yaw-rate bias (rad/s) of a heading source vs GT heading, fitted on the TRAINING block:
    slope of (integrated source yaw − GT course) over time while moving. Robust to per-step GT noise."""
    from ..dataset import _gt_heading
    gt = load_ground_truth(cfg)
    lo, hi = split_intervals(cfg)["train"]
    tu = sensor_to_unix(t_sensor, fit_sensor_clock(cfg)) + dt_clock
    psi = np.cumsum(dyaw)
    g = _gt_heading(gt)
    sp = np.r_[0, np.linalg.norm(np.diff(gt.p[:, :2], axis=0), axis=1) / np.diff(gt.t)]
    sel = (gt.t >= lo) & (gt.t <= hi) & (sp > 3.0) & (gt.t >= tu[0]) & (gt.t <= tu[-1])
    e = np.interp(gt.t[sel], tu, psi) - g[sel]
    return float(np.polyfit(gt.t[sel] - gt.t[sel][0], e, 1)[0])


def resample_dyaw(t_src: np.ndarray, dyaw_src: np.ndarray, t_dst: np.ndarray) -> np.ndarray:
    """Heading increments of one source on another source's stamps (via its integrated heading)."""
    psi = np.cumsum(dyaw_src)
    p = np.interp(t_dst, t_src, psi)
    return np.r_[0.0, np.diff(p)]


def blend_heading(t: np.ndarray, dyaw_fast: np.ndarray, dyaw_slow: np.ndarray, tau_s: float) -> np.ndarray:
    """Complementary heading filter (causal): the fast source's increments drive the heading,
    the slow (low-bias) source's integrated heading pulls it back with time constant tau.
        psi_k = psi_{k-1} + d_fast_k + a_k (psi_slow_k - psi_{k-1} - d_fast_k),  a_k = dt/tau
    Returns the blended heading increments (row 0 = 0). tau -> 0: slow source; tau -> inf: fast."""
    psi_slow = np.cumsum(dyaw_slow)
    out = np.zeros(len(t))
    psi = 0.0
    for k in range(1, len(t)):
        pred = psi + dyaw_fast[k]
        a = min(1.0, (t[k] - t[k - 1]) / tau_s) if np.isfinite(tau_s) else 0.0
        new = pred + a * (psi_slow[k] - pred)
        out[k] = new - psi
        psi = new
    return out
