"""Per-scan context features computed from the raw sensor data (not from any estimator).

These describe the conditions an estimator faced, so they can explain its errors
(analysis) and be inputs to the reliability / fusion model. All are causal: they
use only the current scan and the IMU samples inside it.

Geometry: point-to-plane registration constrains translation along the surface
normals it sees. With H_t = mean(n n^T) over planar points, a small eigenvalue
means a direction that is barely observed (e.g. along an open road with only the
ground and distant banks visible). Same for rotation with (p x n).
"""
from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree

FEATURE_NAMES = [
    "n_points", "return_ratio", "range_p10", "range_p50", "range_p90", "frac_far30",
    "z_std", "planar_frac",
    "trans_eig_min", "trans_eig_mid", "trans_cond", "weak_dir_forward_cos", "weak_dir_up_cos",
    "rot_eig_min", "rot_cond",
    "imu_gyro_norm", "imu_yaw_rate", "imu_acc_norm_std", "imu_acc_horiz",
]


def _voxel_downsample(p: np.ndarray, leaf: float) -> np.ndarray:
    key = np.floor(p / leaf).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return p[idx]


def scan_geometry(points: np.ndarray, total_beams: int, blind: float = 2.0, leaf: float = 0.5,
                  k: int = 10, max_points: int = 12000, rng=np.random.default_rng(0)) -> dict:
    p = np.asarray(points, float)
    r = np.linalg.norm(p, axis=1)
    keep = np.isfinite(r) & (r > blind)
    p, r = p[keep], r[keep]
    f = {"n_points": float(len(p)), "return_ratio": float(len(p) / max(total_beams, 1))}
    if len(p) < 50:
        return f
    f.update(range_p10=float(np.percentile(r, 10)), range_p50=float(np.median(r)),
             range_p90=float(np.percentile(r, 90)), frac_far30=float(np.mean(r > 30)), z_std=float(p[:, 2].std()))

    q = _voxel_downsample(p, leaf)
    if len(q) > max_points:
        q = q[rng.choice(len(q), max_points, replace=False)]
    if len(q) < k + 1:
        return f
    _, nn = cKDTree(q).query(q, k=k)
    nb = q[nn]                                     # (N, k, 3)
    c = nb - nb.mean(1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", c, c) / k
    w, v = np.linalg.eigh(cov)                     # ascending
    normal = v[:, :, 0]
    planar = (w[:, 0] < 0.05 * w[:, 1]) & (w[:, 1] > 1e-4)
    f["planar_frac"] = float(planar.mean())
    if planar.sum() < 20:
        return f
    n, pts = normal[planar], q[planar]

    Ht = n.T @ n / len(n)
    et, vt = np.linalg.eigh(Ht)
    weak = vt[:, 0]
    f.update(trans_eig_min=float(et[0]), trans_eig_mid=float(et[1]), trans_cond=float(et[0] / max(et[2], 1e-12)),
             weak_dir_forward_cos=float(abs(weak[0])), weak_dir_up_cos=float(abs(weak[2])))

    rn = np.cross(pts, n) / np.linalg.norm(pts, axis=1, keepdims=True)
    Hr = rn.T @ rn / len(rn)
    er = np.linalg.eigvalsh(Hr)
    f.update(rot_eig_min=float(er[0]), rot_cond=float(er[0] / max(er[2], 1e-12)))
    return f


def imu_features(imu: np.ndarray) -> dict:
    """imu rows: t, ax, ay, az, gx, gy, gz (sensor frame)."""
    if imu is None or len(imu) == 0:
        return {}
    acc, gyr = imu[:, 1:4], imu[:, 4:7]
    an = np.linalg.norm(acc, axis=1)
    return {"imu_gyro_norm": float(np.linalg.norm(gyr, axis=1).mean()), "imu_yaw_rate": float(gyr[:, 2].mean()),
            "imu_acc_norm_std": float(an.std()), "imu_acc_horiz": float(np.linalg.norm(acc[:, :2].mean(0)))}
