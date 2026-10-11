"""Pose interpolation, alignment and body-frame increments."""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from .io import Trajectory


def wrap_pi(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def cumulative_distance(xy: np.ndarray) -> np.ndarray:
    d = np.linalg.norm(np.diff(xy[:, :2], axis=0), axis=1)
    d[~np.isfinite(d)] = 0.0
    return np.r_[0.0, np.cumsum(d)]


# --------------------------------------------------------------------------- interpolation

def interpolate(traj: Trajectory, t_query: np.ndarray, max_gap: float = 0.15):
    """Positions (linear) and orientations (slerp) at `t_query`.

    A query is valid only inside the trajectory span and when its two bracketing
    samples are at most `max_gap` apart. Nothing is extrapolated.
    Returns (p (M,3), q (M,4) or None, valid (M,)).
    """
    t_query = np.asarray(t_query, float)
    t = traj.t
    idx = np.searchsorted(t, t_query)
    inside = (idx > 0) & (idx < len(t))
    exact = np.zeros_like(inside)
    hit = np.searchsorted(t, t_query, side="left")
    ok = hit < len(t)
    exact[ok] = np.isclose(t[hit[ok]], t_query[ok], atol=1e-9, rtol=0)
    i1 = np.clip(idx, 1, len(t) - 1)
    gap = t[i1] - t[i1 - 1]
    valid = (inside & (gap <= max_gap)) | exact
    p = np.full((len(t_query), 3), np.nan)
    for k in range(3):
        p[:, k] = np.interp(t_query, t, traj.p[:, k])
    p[~valid] = np.nan
    q = None
    if traj.q is not None:
        q = np.full((len(t_query), 4), np.nan)
        if valid.any():
            slerp = Slerp(t, Rotation.from_quat(traj.q))
            q[valid] = slerp(np.clip(t_query[valid], t[0], t[-1])).as_quat()
    return p, q, valid


# --------------------------------------------------------------------------- alignment

def umeyama(src: np.ndarray, dst: np.ndarray, with_scale: bool = False):
    """Least-squares similarity dst ≈ s R src + t (Umeyama 1991). Works in 2D or 3D."""
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(src.shape[1])
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[-1, -1] = -1
    R = U @ S @ Vt
    s = (D * np.diag(S)).sum() / xs.var(0).sum() if with_scale else 1.0
    t = mu_d - s * R @ mu_s
    return R, t, s


def yaw_matrix(psi: float) -> np.ndarray:
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def leveling_rotation(gravity_up_world: np.ndarray) -> np.ndarray:
    """Rotation taking the estimator's measured 'up' (−gravity) direction to +z."""
    g = np.asarray(gravity_up_world, float)
    g = g / np.linalg.norm(g)
    r, _ = Rotation.align_vectors([[0, 0, 1.0]], [g])
    return r.as_matrix()


def transform(traj: Trajectory, R: np.ndarray, t: np.ndarray, s: float = 1.0, name=None) -> Trajectory:
    p = s * traj.p @ R.T + t
    q = None
    if traj.q is not None:
        q = (Rotation.from_matrix(R) * Rotation.from_quat(traj.q)).as_quat()
    return Trajectory(name or traj.name, traj.t, p, q, dict(traj.meta))


def level(traj: Trajectory) -> Trajectory:
    """Apply meta['gravity_up_world'] (if the estimator frame is not gravity-aligned)."""
    g = traj.meta.get("gravity_up_world")
    if g is None:
        return traj
    out = transform(traj, leveling_rotation(g), np.zeros(3))
    out.meta["leveled"] = True
    return out


def fit_alignment(est_p: np.ndarray, ref_p: np.ndarray, mode: str, ref_dist: np.ndarray,
                  head_m: float = 200.0, with_scale: bool = False):
    """Rigid alignment of associated, valid samples. Returns (R 3x3, t 3, s, info).

    se3_full  Umeyama on all samples, 3D (the usual benchmark protocol, evo -a)
    se2_full  yaw + planar translation on all samples (horizontal-only benchmark)
    se2_head  yaw + planar translation fit on the first `head_m` metres only;
              everything after is pure prediction (what a deployment anchored on
              an initial GNSS fix would see)
    origin    translate the first sample onto the reference, no rotation
    none      identity (estimate already in UTM, e.g. legacy exports)
    """
    m = np.isfinite(est_p).all(1) & np.isfinite(ref_p).all(1)
    info = {"mode": mode, "n_fit": int(m.sum())}
    R, t, s = np.eye(3), np.zeros(3), 1.0
    if mode == "none":
        return R, t, s, info
    if mode == "origin":
        k = np.flatnonzero(m)[0]
        t = ref_p[k] - est_p[k]
        t[2] = 0.0
        return R, t, s, info
    if mode.endswith("_head"):
        m = m & (ref_dist <= head_m)
        info["n_fit"] = int(m.sum())
        info["head_m"] = head_m
    if m.sum() < 3:
        raise ValueError(f"alignment {mode}: only {m.sum()} usable samples")
    if mode.startswith("se3"):
        R, t, s = umeyama(est_p[m], ref_p[m], with_scale)
    elif mode.startswith("se2"):
        R2, t2, s = umeyama(est_p[m, :2], ref_p[m, :2], with_scale)
        R = np.eye(3)
        R[:2, :2] = R2
        t = np.r_[t2, np.mean(ref_p[m, 2] - est_p[m, 2])]
    else:
        raise ValueError(mode)
    info["yaw_deg"] = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
    info["scale"] = float(s)
    return R, t, s, info


# --------------------------------------------------------------------------- heading and increments

def yaw_from_quat(q: np.ndarray) -> np.ndarray:
    """Heading of the body x-axis in the (levelled) world xy-plane, radians CCW from +x."""
    R = Rotation.from_quat(q).as_matrix()
    return np.arctan2(R[:, 1, 0], R[:, 0, 0])


def course_from_positions(t: np.ndarray, xy: np.ndarray, half_window_s: float = 0.3, min_speed: float = 1.0):
    """Course over ground (radians CCW from +x/east) by central differences over ±half_window.

    Samples slower than `min_speed` get NaN. Course equals body heading only up to
    side-slip and the sensor's mounting yaw.
    """
    t = np.asarray(t, float)
    lo = np.searchsorted(t, t - half_window_s)
    hi = np.clip(np.searchsorted(t, t + half_window_s, side="right") - 1, 0, len(t) - 1)
    d = xy[hi, :2] - xy[lo, :2]
    dt = t[hi] - t[lo]
    with np.errstate(invalid="ignore", divide="ignore"):
        speed = np.linalg.norm(d, axis=1) / dt
    course = np.arctan2(d[:, 1], d[:, 0])
    course[~(speed >= min_speed)] = np.nan
    return course, speed


def body_increments(p: np.ndarray, yaw: np.ndarray) -> np.ndarray:
    """Planar motion between consecutive samples in the previous body frame.

    Returns (N, 3): [forward, lateral, dyaw]; row 0 is NaN. Independent of the
    world frame the estimator uses, which is what lets increments from different
    estimators (and the ground truth) be compared and mixed directly.
    """
    n = len(p)
    out = np.full((n, 3), np.nan)
    d = np.diff(p[:, :2], axis=0)
    c, s = np.cos(yaw[:-1]), np.sin(yaw[:-1])
    out[1:, 0] = c * d[:, 0] + s * d[:, 1]
    out[1:, 1] = -s * d[:, 0] + c * d[:, 1]
    out[1:, 2] = wrap_pi(np.diff(yaw))
    return out


def integrate_increments(u: np.ndarray, p0: np.ndarray, yaw0: float):
    """Exact inverse of `body_increments`. NaN rows are treated as no motion."""
    u = np.nan_to_num(u)
    n = len(u)
    yaw = np.empty(n)
    xy = np.empty((n, 2))
    yaw[0], xy[0] = yaw0, p0[:2]
    for k in range(1, n):
        psi = yaw[k - 1]
        c, s = np.cos(psi), np.sin(psi)
        xy[k, 0] = xy[k - 1, 0] + c * u[k, 0] - s * u[k, 1]
        xy[k, 1] = xy[k - 1, 1] + s * u[k, 0] + c * u[k, 1]
        yaw[k] = psi + u[k, 2]
    return xy, yaw
