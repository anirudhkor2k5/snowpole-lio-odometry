"""Absolute position fixes from snow poles matched to the surveyed pole map.

Detection (per scan, in scanpass): thin, tall, isolated vertical clusters in the
deskewed scan. Snowbanks fail the isolation test, trees and bushes the width
test. Detections are stored in the IMU frame at the scan's end stamp, so any
trajectory can be corrected with them afterwards.

Correction (here): a causal EKF on the planar map<-odometry correction
    C(q) = c_k + t + R(theta) (q - c_k)
anchored at the current odometry position c_k (so theta does not couple with the
distance to the odometry origin). It starts from the same 200 m head alignment the
`se2_head` evaluation uses (stand-in for an initial GNSS fix), grows its
uncertainty with distance driven, and is updated by every detection that matches a
surveyed pole unambiguously and within a Mahalanobis gate.

Nothing here is fitted to the drive. The constants below are physical choices:
  DRIFT_FRAC   1 %/m translational odometry drift (the baselines measure 0.5-3 %)
  HEADING_RW   1e-4 rad/sqrt(m) heading random walk (0.2 deg after 1 km)
  SIGMA_POLE   0.25 m pole-centre measurement (beam footprint + survey)
"""
from __future__ import annotations

import json

import numpy as np
import pandas as pd
from scipy import ndimage
from scipy.spatial import cKDTree

from ..geometry import cumulative_distance, fit_alignment, interpolate, level, yaw_from_quat
from ..io import load_ground_truth, load_run, repo_path, results_dir
from ..metrics import estimate_time_offset, stats
from .common import inherit_frame, split_intervals, to_matrices, unix_to_sensor, write_run
from ..bag import fit_sensor_clock

DRIFT_FRAC = 0.01
HEADING_RW = 1e-4
SIGMA_POLE = 0.25
GATE_M = 2.0
GATE_CHI2 = 9.21          # 2 dof, 99 %
AMBIGUITY_M = 1.5         # second-nearest pole must be this much farther than the nearest

DETECTION_COLUMNS = ["stamp", "x", "y", "z", "h_min", "h_max", "width", "n", "intensity", "range"]


# --------------------------------------------------------------------------- detection

def detect(p: np.ndarray, intensity: np.ndarray | None = None, r_min: float = 2.0, r_max: float = 30.0,
           cell: float = 0.15, ground_cell: float = 0.5, h_lo: float = 0.3, h_hi: float = 2.5,
           max_width: float = 0.45, min_height: float = 0.6, min_points: int = 4,
           ring_in: float = 0.5, ring_out: float = 1.5, max_ring_points: int = 2) -> np.ndarray:
    """Pole candidates in a deskewed scan (N,3, IMU frame, z roughly up).

    Height is measured above the local *surface* (lowest return over the 3x3
    neighbourhood of 0.5 m cells), not above the road: snow poles stand in
    snowbanks, and measured from the road the bank itself fills the isolation
    ring and every pole is rejected.

    Returns (K, 9): x, y, z, h_min, h_max, width, n, mean intensity, range.
    """
    r = np.hypot(p[:, 0], p[:, 1])
    m = (r > r_min) & (r < r_max)
    p = p[m]
    inten = intensity[m] if intensity is not None else np.zeros(len(p))
    if len(p) < 50:
        return np.empty((0, 9))
    # local surface: lowest return per 0.5 m cell, then the minimum over its 3x3 neighbourhood
    n_g = int(np.ceil(2 * r_max / ground_cell)) + 3
    gij = np.floor((p[:, :2] + r_max) / ground_cell).astype(int) + 1
    zmin = np.full((n_g, n_g), np.inf)
    np.minimum.at(zmin, (gij[:, 0], gij[:, 1]), p[:, 2])
    surf = ndimage.minimum_filter(zmin, size=3, mode="nearest")
    h = p[:, 2] - surf[gij[:, 0], gij[:, 1]]
    band = (h > h_lo) & (h < h_hi)
    q, hq, iq = p[band], h[band], inten[band]
    if len(q) < min_points:
        return np.empty((0, 9))
    # connected components on an occupancy grid of the height band
    n_side = int(np.ceil(2 * r_max / cell)) + 2
    ij = np.floor((q[:, :2] + r_max) / cell).astype(int) + 1
    occ = np.zeros((n_side, n_side), bool)
    occ[ij[:, 0], ij[:, 1]] = True
    lab, nlab = ndimage.label(occ, structure=np.ones((3, 3)))
    if nlab == 0:
        return np.empty((0, 9))
    pl = lab[ij[:, 0], ij[:, 1]]
    tree = cKDTree(q[:, :2])
    out = []
    order = np.argsort(pl)
    bounds = np.searchsorted(pl[order], np.arange(1, nlab + 2))
    for c in range(nlab):
        idx = order[bounds[c]:bounds[c + 1]]
        if len(idx) < min_points:
            continue
        xy = q[idx, :2]
        width = float(np.max(xy.max(0) - xy.min(0)))
        if width > max_width:
            continue
        hmin, hmax = float(hq[idx].min()), float(hq[idx].max())
        if hmax - hmin < min_height:
            continue
        cen = xy.mean(0)
        near = len(tree.query_ball_point(cen, ring_out)) - len(tree.query_ball_point(cen, ring_in))
        if near > max_ring_points:
            continue
        out.append([cen[0], cen[1], float(q[idx, 2].mean()), hmin, hmax, width, len(idx),
                    float(iq[idx].mean()), float(np.hypot(*cen))])
    return np.asarray(out) if out else np.empty((0, 9))


# --------------------------------------------------------------------------- map

def load_pole_map(cfg: dict) -> np.ndarray:
    df = pd.read_csv(repo_path(cfg["pole_map"]), encoding="utf-8-sig")
    return df[["UTM33-Øst", "UTM33-Nord"]].to_numpy(float)


def _rot(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s], [s, c]])


def _drot(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[-s, -c], [c, -s]])


# --------------------------------------------------------------------------- filter

def correct(cfg: dict, method: str, tag: str = "full", detections: str | None = None, baseline: str | None = None,
            write: bool = True) -> dict:
    """Pole-corrected copy of `method`'s trajectory (run name '<method>+poles')."""
    baseline = baseline or method.split("+")[0]
    det_file = detections or results_dir("poles") / f"detections_{baseline}.csv"
    det = pd.read_csv(det_file)
    poles = load_pole_map(cfg)
    ptree = cKDTree(poles)
    gt = load_ground_truth(cfg)
    clock = fit_sensor_clock(cfg)
    blocks = split_intervals(cfg)

    tr = load_run(method, tag, cfg)
    if tr is None:
        raise FileNotFoundError(f"no run {method}/{tag}")
    tr = level(tr)
    # clock residual from the training block only
    lo, hi = blocks["train"]
    off = estimate_time_offset(tr, gt.slice_time(lo, hi))["dt_s"]
    tr = tr.shifted(off)

    # initial map<-odom alignment from the first 200 m (same as the se2_head protocol)
    p_gt, _, ok_gt = interpolate(tr, gt.t, cfg["evaluation"].get("max_assoc_dt", 0.15))
    dist = cumulative_distance(gt.p)
    R0, t0, _, ainfo = fit_alignment(p_gt, gt.p, "se2_head", dist, cfg["evaluation"].get("head_align_m", 200.0))
    th0 = float(np.arctan2(R0[1, 0], R0[0, 0]))
    head_end_t = float(gt.t[np.searchsorted(dist, cfg["evaluation"].get("head_align_m", 200.0))])

    # detections -> method's clock (sensor stamp -> unix -> + residual offset)
    from ..bag import sensor_to_unix
    det = det.assign(t=sensor_to_unix(det["stamp"].to_numpy(), clock) + off).sort_values("t")
    det_groups = {t: g for t, g in det.groupby("t")}

    t_odo = tr.t
    xy = tr.p[:, :2]
    Rq = to_matrices(tr.p, tr.q)
    # filter state: t (2), theta; anchored at c_k = xy[k]
    c0 = xy[0]
    x = np.r_[R0[:2, :2] @ c0 + t0[:2] - c0, th0]           # C(c0) = c0 + t
    P = np.diag([0.3 ** 2, 0.3 ** 2, np.radians(0.2) ** 2])
    out_xy = np.empty_like(xy)
    out_yaw = np.empty(len(t_odo))
    log = []
    det_times = np.array(sorted(det_groups))
    j = 0
    for k in range(len(t_odo)):
        if k > 0:
            dc = xy[k] - xy[k - 1]
            ds = float(np.linalg.norm(dc))
            F = np.eye(3)
            F[:2, 2] = _drot(x[2]) @ dc
            x[:2] = x[:2] + (_rot(x[2]) - np.eye(2)) @ dc
            P = F @ P @ F.T + np.diag([(DRIFT_FRAC * ds) ** 2] * 2 + [HEADING_RW ** 2 * ds])
        # detections stamped up to this pose (fixes are only used after the head alignment ends)
        while j < len(det_times) and det_times[j] <= t_odo[k]:
            td = det_times[j]
            j += 1
            if td <= head_end_t:
                continue
            Tk, okk = _pose_at(tr, td)
            if not okk:
                continue
            ck = Tk[:2, 3]
            # re-anchor the state at the detection pose (exact, see module doc)
            dc = ck - xy[k]
            F = np.eye(3)
            F[:2, 2] = _drot(x[2]) @ dc
            xa = np.r_[x[:2] + (_rot(x[2]) - np.eye(2)) @ dc, x[2]]
            Pa = F @ P @ F.T
            g = det_groups[td]
            body = g[["x", "y", "z"]].to_numpy()
            qw = (Tk[:3, :3] @ body.T).T[:, :2] + ck            # odom-frame (levelled) pole positions
            for qi in qw:
                Rr = _rot(xa[2])
                h = ck + xa[:2] + Rr @ (qi - ck)
                d2, idx = ptree.query(h, k=2)
                r = poles[idx[0]] - h
                H = np.zeros((2, 3))
                H[:, :2] = np.eye(2)
                H[:, 2] = _drot(xa[2]) @ (qi - ck)
                S = H @ Pa @ H.T + np.eye(2) * SIGMA_POLE ** 2
                m2 = float(r @ np.linalg.solve(S, r))
                accepted = d2[0] < GATE_M and m2 < GATE_CHI2 and d2[1] > d2[0] + AMBIGUITY_M
                log.append({"t": td, "pole": int(idx[0]), "resid_m": float(d2[0]), "mahal2": m2,
                            "accepted": bool(accepted)})
                if accepted:
                    K = Pa @ H.T @ np.linalg.inv(S)
                    xa = xa + K @ r
                    Pa = (np.eye(3) - K @ H) @ Pa
            # back to the anchor at xy[k]
            dc = xy[k] - ck
            F = np.eye(3)
            F[:2, 2] = _drot(xa[2]) @ dc
            x = np.r_[xa[:2] + (_rot(xa[2]) - np.eye(2)) @ dc, xa[2]]
            P = F @ Pa @ F.T
        out_xy[k] = xy[k] + x[:2]
        out_yaw[k] = x[2]

    # corrected poses: rotate each body orientation by theta about z, positions from C
    T_out = Rq.copy()
    for k in range(len(t_odo)):
        Rz = np.eye(3)
        Rz[:2, :2] = _rot(out_yaw[k])
        T_out[k, :3, :3] = Rz @ Rq[k, :3, :3]
        T_out[k, :2, 3] = out_xy[k]
    log = pd.DataFrame(log)
    report = _summary(gt, tr, R0, t0, T_out, log, blocks, poles, head_end_t)
    report.update(method=method, tag=tag, time_offset_s=off, head_alignment=ainfo,
                  constants={"DRIFT_FRAC": DRIFT_FRAC, "HEADING_RW": HEADING_RW, "SIGMA_POLE": SIGMA_POLE,
                             "GATE_M": GATE_M, "GATE_CHI2": GATE_CHI2, "AMBIGUITY_M": AMBIGUITY_M})
    if write:
        name = f"{method}+poles"
        out = write_run(name, tag, unix_to_sensor(t_odo - off, clock), T_out,
                        {"runner": "correct.poles", "role": "corrector", "world_frame": "utm",
                         "baseline": baseline, "source_method": method, "report": report})
        log.to_csv(out / "pole_fixes.csv", index=False)
        (results_dir("correct", baseline) / f"poles_{method}.json").write_text(json.dumps(report, indent=2, default=float))
    return report


def _pose_at(tr, t):
    p, q, ok = interpolate(tr, np.array([t]), 0.25)
    if not ok[0]:
        return None, False
    return to_matrices(p, q)[0], True


def _summary(gt, tr, R0, t0, T_out, log, blocks, poles, head_end_t) -> dict:
    """Absolute horizontal error (UTM, no re-alignment) with and without pole fixes."""
    p_odo, _, ok = interpolate(tr, gt.t, 0.15)
    base = p_odo @ R0.T + t0                                  # head-aligned, never corrected
    from ..io import Trajectory
    corr = Trajectory("c", tr.t, T_out[:, :3, 3])
    p_cor, _, ok2 = interpolate(corr, gt.t, 0.15)
    e_base = np.linalg.norm(base[:, :2] - gt.p[:, :2], axis=1)
    e_cor = np.linalg.norm(p_cor[:, :2] - gt.p[:, :2], axis=1)
    valid = ok & ok2 & (gt.t > head_end_t)
    near = cKDTree(poles).query(gt.p[:, :2])[0] < 15.0
    zone = np.zeros(len(gt.t), bool)
    if near.any():
        i = np.flatnonzero(near)
        zone[i[0]:i[-1] + 1] = True
    rep = {"fixes_accepted": int(log["accepted"].sum()) if len(log) else 0,
           "candidates_tested": int(len(log)),
           "accepted_residual_m": stats(log.loc[log["accepted"], "resid_m"]) if len(log) else {}}
    for name, (lo, hi) in [("all", (gt.t[0], gt.t[-1]))] + list(blocks.items()):
        sel = valid & (gt.t >= lo) & (gt.t <= hi)
        rep[name] = {}
        for zname, zmask in (("pole_zone", zone), ("outside_poles", ~zone), ("everywhere", np.ones_like(zone))):
            s = sel & zmask
            rep[name][zname] = {"head_aligned_only": stats(e_base[s]), "pole_corrected": stats(e_cor[s])}
    return rep
