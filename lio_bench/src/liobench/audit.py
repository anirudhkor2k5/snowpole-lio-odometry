"""Phase A audit: one canonical evaluation, error decomposition, planar ablation, GT provenance.

Canonical (deployable) protocol, used for every number in report_v2:
  - clock: each method's residual offset to ground truth is estimated on the TRAINING
    block only (speed-profile match) and then held fixed;
  - alignment: one SE(2) transform fitted on the first 200 m of the evaluated span
    (stand-in for an initial GNSS fix); everything after is prediction;
  - metrics: ATE RMSE, final error, drift % (KITTI-style mean over 100-800 m segments,
    frame-free) with a moving-block bootstrap 95 % CI, RPE over 1 s / 10 s, horizontal
    length ratio, heading error at the end of the span.
Diagnostic protocols (whole-drive rigid fits) are never used to state deployable accuracy.
"""
from __future__ import annotations

import json

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

from .bag import fit_sensor_clock, sensor_to_unix
from .geometry import (course_from_positions, cumulative_distance, interpolate, level,
                       leveling_rotation)
from .io import Trajectory, latlon_to_utm, load_config, load_ground_truth, load_run, repo_path, results_dir, run_meta
from .metrics import estimate_time_offset, evaluate, segment_drift, stats

HEAD_M = 200.0
DRIFT_L = (100, 200, 400, 800)


# --------------------------------------------------------------------------- canonical protocol

def spans(cfg: dict) -> dict[str, tuple[float, float]]:
    from .correct.common import split_intervals
    gt = load_ground_truth(cfg)
    return {"whole": (float(gt.t[0]), float(gt.t[-1])), **split_intervals(cfg)}


def train_offset(tr: Trajectory, cfg: dict) -> float:
    lo, hi = spans(cfg)["train"]
    return float(estimate_time_offset(level(tr), load_ground_truth(cfg).slice_time(lo, hi))["dt_s"])


def _block_bootstrap_mean(x: np.ndarray, block: int = 20, n: int = 2000, seed: int = 0) -> tuple[float, float]:
    x = x[np.isfinite(x)]
    if len(x) < 2 * block:
        return float("nan"), float("nan")
    rng = np.random.default_rng(seed)
    nb = int(np.ceil(len(x) / block))
    starts = rng.integers(0, len(x) - block + 1, size=(n, nb))
    means = np.array([np.concatenate([x[s:s + block] for s in row])[:len(x)].mean() for row in starts])
    return float(np.percentile(means, 2.5)), float(np.percentile(means, 97.5))


def canonical(tr: Trajectory, cfg: dict, span: tuple[float, float], dt: float) -> dict:
    """Deployable metrics of `tr` over `span` (Unix s). `dt` = training-block clock offset."""
    gt = load_ground_truth(cfg)
    lo, hi = span
    ref = gt.slice_time(lo, hi)
    est = tr.shifted(dt).slice_time(lo - 2.0, hi + 2.0)
    if len(est) < 20 or len(ref) < 20:
        return {}
    ecfg = dict(cfg["evaluation"], estimate_time_offset=False, head_align_m=HEAD_M, drift_lengths_m=list(DRIFT_L))
    ev = evaluate(est, ref, "se2_head", ecfg)
    s = ev.summary
    _, d200 = segment_drift(np.where(ev.valid[:, None], ev.est_xy, np.nan), ev.ref_xy, ev.dist, 200)
    lo_ci, hi_ci = _block_bootstrap_mean(np.asarray(d200))
    # heading error at the end: course of the last 50 m, estimate vs reference
    ok = np.flatnonzero(ev.valid & (ev.dist > ev.dist[-1] - 50))
    head_err = float("nan")
    if len(ok) > 5:
        de, dr = ev.est_xy[ok[-1]] - ev.est_xy[ok[0]], ev.ref_xy[ok[-1]] - ev.ref_xy[ok[0]]
        head_err = float(np.degrees(np.angle(np.exp(1j * (np.arctan2(de[1], de[0]) - np.arctan2(dr[1], dr[0]))))))
    return {"ate_m": s["ate_xy"].get("rmse"), "final_m": s["final_error_m"], "drift_pct": s["drift_pct_mean_kitti"],
            "drift200_pct": s["drift_pct"]["200m"].get("mean"), "drift200_ci95": [lo_ci, hi_ci],
            "rpe1s_m": s["rpe_time"]["1s"].get("median"), "rpe10s_m": s["rpe_time"]["10s"].get("median"),
            "length_ratio": s["path_length_m"]["ratio"], "heading_err_end_deg": head_err,
            "coverage": s["coverage"], "length_m": float(ev.dist[-1])}


def evaluate_all_spans(tr: Trajectory, cfg: dict, dt: float | None = None) -> dict:
    dt = train_offset(tr, cfg) if dt is None else dt
    return {"clock_offset_s": dt, **{name: canonical(tr, cfg, sp, dt) for name, sp in spans(cfg).items()}}


# --------------------------------------------------------------------------- trajectory variants

def _track_unix(algo: str, tag: str, cfg: dict):
    """Estimator poses (levelled world, IMU body), Unix time."""
    from .correct.common import baseline_track
    tk = baseline_track(algo, tag)
    meta = run_meta(algo, tag)
    clock = fit_sensor_clock(cfg)
    g = meta.get("gravity_up_world")
    if g is None and meta.get("world_frame") == "first_imu":
        g = clock["imu_static_mean_acc"]
    T = tk.T.copy()
    if g is not None:
        L = np.eye(4)
        L[:3, :3] = leveling_rotation(np.asarray(g, float))
        T = L[None] @ T
    return sensor_to_unix(tk.t, clock), T


def _traj(name: str, t: np.ndarray, T: np.ndarray) -> Trajectory:
    return Trajectory(name, t, T[:, :3, 3].copy(), Rotation.from_matrix(T[:, :3, :3]).as_quat(), {"world_frame": "gravity"})


def _rebuild(t, T, att_fn, name, drop_lateral=False):
    """Horizontal track from body-frame steps rotated by a modified attitude att_fn(R_world) at each step start."""
    D = np.linalg.inv(T[:-1]) @ T[1:]
    d = D[:, :3, 3]
    if drop_lateral:
        d = d * np.array([1.0, 0.0, 1.0])
    Ra = np.stack([att_fn(R) for R in T[:-1, :3, :3]])
    steps = np.einsum("nij,nj->ni", Ra, d)
    P = np.vstack([T[:1, :3, 3], T[0, :3, 3] + np.cumsum(steps, 0)])
    out = np.tile(np.eye(4), (len(t), 1, 1))
    out[:, :3, 3] = P
    out[:, :3, :3] = np.concatenate([Ra, Ra[-1:]], 0)
    return _traj(name, t, out)


def _zyx(R):
    return Rotation.from_matrix(R).as_euler("ZYX")


def _from_zyx(y, p, r):
    return Rotation.from_euler("ZYX", [y, p, r]).as_matrix()


def ablation_variants(algo: str, tag: str, cfg: dict) -> dict[str, Trajectory]:
    """A3: which part of 'planar' matters. All built from the estimator's own poses; nothing fitted."""
    from .correct.common import PoseTrack
    from .correct.planar import body_up, planarize
    t, T = _track_unix(algo, tag, cfg)
    up = body_up(cfg)
    v = {"as_is": _traj("as_is", t, T)}
    v["remove_pitch_only"] = _rebuild(t, T, lambda R: _from_zyx(_zyx(R)[0], 0.0, _zyx(R)[2]), "remove_pitch_only")
    v["remove_roll_only"] = _rebuild(t, T, lambda R: _from_zyx(_zyx(R)[0], _zyx(R)[1], 0.0), "remove_roll_only")
    v["level_world_yaw"] = _rebuild(t, T, lambda R: _from_zyx(_zyx(R)[0], 0.0, 0.0), "level_world_yaw")
    P = planarize(PoseTrack(t, T), up, x0=T[0, :2, 3], yaw0=float(_zyx(T[0, :3, :3])[0]))
    v["planar"] = _traj("planar", t, P)
    P0 = planarize(PoseTrack(t, T), np.array([0.0, 0.0, 1.0]), x0=T[0, :2, 3], yaw0=float(_zyx(T[0, :3, :3])[0]))
    v["planar_up_imu_z"] = _traj("planar_up_imu_z", t, P0)
    v["planar_no_lateral"] = _rebuild(_traj("p", t, P).t, P, lambda R: R, "planar_no_lateral", drop_lateral=True)
    return v


# --------------------------------------------------------------------------- error decomposition

def decompose(algo: str, tag: str, cfg: dict, span_name: str = "whole") -> dict:
    """Counterfactual re-integration on the ground-truth clock (diagnostic: uses GT).

    E_as_is      the estimator's world track (tilted)                 -> canonical metrics
    E_planar     its body-frame steps on the level                    -> removes tilt
    E_gt_yaw     planar steps, heading increments from GT             -> removes heading drift
    E_gt_fwd     planar steps, GT heading and GT forward length       -> removes along-track scale
    The remaining error after E_gt_fwd is lateral + step noise.
    """
    from .correct.planar import body_steps, body_up
    from .dataset import _gt_heading
    from .geometry import body_increments, integrate_increments
    gt = load_ground_truth(cfg)
    lo, hi = spans(cfg)[span_name]
    tr = load_run(algo, tag, cfg)
    dt = train_offset(tr, cfg)
    t_unix, T = _track_unix(algo, tag, cfg)
    up = body_up(cfg)
    # method steps resampled on the GT clock (body frame, level axes) via its own interpolated poses
    from .correct.common import to_matrices
    tw = _traj(algo, t_unix + dt, T)
    p, q, ok = interpolate(tw, gt.t, 0.15)
    Tg = np.tile(np.eye(4), (len(gt.t), 1, 1))
    Tg[ok] = to_matrices(p[ok], q[ok])
    u = np.full((len(gt.t), 3), np.nan)
    i = np.flatnonzero(ok & np.r_[False, ok[:-1]])
    u[i] = body_steps(Tg[i - 1], Tg[i], up)
    # mounting yaw (training block, as in dataset.build)
    ltr, htr = spans(cfg)["train"]
    sel = (gt.t >= ltr) & (gt.t <= htr) & np.isfinite(u).all(1) & (np.abs(u[:, 0]) > 0.3)
    m = float(np.arctan2(np.nansum(u[sel, 1]), np.nansum(u[sel, 0])))
    c, s = np.cos(-m), np.sin(-m)
    u[:, 0], u[:, 1] = c * u[:, 0] - s * u[:, 1], s * u[:, 0] + c * u[:, 1]
    g_yaw = _gt_heading(gt)
    u_gt = body_increments(gt.p, g_yaw)
    rows = np.flatnonzero((gt.t >= lo) & (gt.t <= hi))
    out = {}

    def integ(name, uu):
        uu = pd.DataFrame(uu[rows]).ffill().fillna(0.0).to_numpy()
        xy, yw = integrate_increments(uu, gt.p[rows[0], :2], g_yaw[rows[0]])
        tt = Trajectory(name, gt.t[rows], np.c_[xy, np.zeros(len(xy))], Rotation.from_euler("z", yw[:, None]).as_quat(),
                        {"world_frame": "gravity"})
        out[name] = canonical(tt, cfg, (lo, hi), 0.0)

    out["as_is"] = canonical(load_run(algo, tag, cfg), cfg, (lo, hi), dt)
    integ("planar", u)
    integ("planar+gt_heading", np.c_[u[:, :2], u_gt[:, 2]])
    integ("planar+gt_heading+gt_forward", np.c_[u_gt[:, 0], u[:, 1], u_gt[:, 2]])
    # heading drift rate: accumulated yaw-increment error vs distance
    e = np.nancumsum(np.where(np.isfinite(u[rows, 2]) & np.isfinite(u_gt[rows, 2]), u[rows, 2] - u_gt[rows, 2], 0.0))
    dist = cumulative_distance(gt.p[rows]) / 1000.0
    out["heading_drift"] = {"deg_at_end": float(np.degrees(e[-1])),
                            "deg_per_km_fit": float(np.degrees(np.polyfit(dist, e, 1)[0])) if dist[-1] > 0.5 else None,
                            "deg_abs_max": float(np.degrees(np.abs(e).max()))}
    out["scale"] = {"forward_sum_ratio": float(np.nansum(u[rows, 0]) / np.nansum(u_gt[rows, 0]))}
    out["mount_yaw_deg"] = float(np.degrees(m))
    return out


# --------------------------------------------------------------------------- ground-truth provenance

def gt_provenance(cfg: dict) -> dict:
    gt = load_ground_truth(cfg)
    rep = {"samples": len(gt), "rate_hz": float(1.0 / np.median(np.diff(gt.t)))}
    L, R = pd.read_csv(repo_path(cfg["gnss_left"])), pd.read_csv(repo_path(cfg["gnss_right"]))

    def stamp(df):
        return df["header.stamp.secs"].to_numpy(float) + df["header.stamp.nsecs"].to_numpy(float) * 1e-9
    tl, tr_ = stamp(L), stamp(R)
    el, nl = latlon_to_utm(L["latitude"].to_numpy(float), L["longitude"].to_numpy(float), cfg["utm_epsg"])
    er, nr = latlon_to_utm(R["latitude"].to_numpy(float), R["longitude"].to_numpy(float), cfg["utm_epsg"])
    er_i, nr_i = np.interp(tl, tr_, er), np.interp(tl, tr_, nr)
    base = np.hypot(el - er_i, nl - nr_i)
    zb = L["altitude"].to_numpy(float) - np.interp(tl, tr_, R["altitude"].to_numpy(float))
    rep["antenna_baseline_m"] = {"horizontal_median": float(np.median(base)), "horizontal_std": float(np.std(base)),
                                 "horizontal_p05_p95": [float(np.percentile(base, 5)), float(np.percentile(base, 95))],
                                 "vertical_diff_std": float(np.std(zb))}
    mid = np.c_[0.5 * (el + er_i), 0.5 * (nl + nr_i)]
    g_on_l = np.c_[np.interp(tl, gt.t, gt.p[:, 0]), np.interp(tl, gt.t, gt.p[:, 1])]
    inside = (tl > gt.t[0]) & (tl < gt.t[-1])
    rep["gt_minus_midpoint_m"] = stats(np.linalg.norm(g_on_l[inside] - mid[inside], axis=1))
    rep["gt_minus_left_m"] = stats(np.linalg.norm(g_on_l[inside] - np.c_[el, nl][inside], axis=1))
    # status flags
    rep["navsat_status"] = L["status.status"].value_counts().to_dict()
    st = pd.read_csv(results_dir("bag") / "gps_state.csv")
    rep["gnss_fix_status_counts"] = st["gnss_fix_status"].value_counts().to_dict()
    rep["lat_std_populated"] = bool((st["lat_std"] != 0).any())
    rep["receiver_minus_header_stamp_s"] = stats((L["Time"].to_numpy(float) - tl))
    # GT high-frequency jitter: residual from a 1 s moving fit (per-step label noise floor)
    sm = pd.DataFrame(gt.p[:, :2]).rolling(11, center=True, min_periods=11).mean().to_numpy()
    rep["gt_jitter_m"] = stats(np.linalg.norm(gt.p[:, :2] - sm, axis=1))
    steps = np.linalg.norm(np.diff(gt.p[:, :2], axis=0), axis=1)
    rep["gt_step_m"] = stats(steps)
    return rep


# --------------------------------------------------------------------------- driver

def run(cfg: dict | None = None, baselines=("faster_lio", "fast_lio")) -> dict:
    cfg = cfg or load_config()
    out = results_dir("audit")
    rep = {"gt_provenance": gt_provenance(cfg), "decomposition": {}, "ablation": {}, "gravity_locked": {}}
    for algo, tag in [("faster_lio", "full"), ("fast_lio", "full"), ("dlio", "full"), ("genz_lio", "full_ros")]:
        try:
            rep["decomposition"][algo] = {sp: decompose(algo, tag, cfg, sp) for sp in ("whole", "test")}
        except Exception as exc:  # noqa: BLE001
            rep["decomposition"][algo] = {"error": str(exc)}
    for b in baselines:
        tr0 = load_run(b, "full", cfg)
        dt = train_offset(tr0, cfg)
        rep["ablation"][b] = {k: evaluate_all_spans(v, cfg, dt) for k, v in ablation_variants(b, "full", cfg).items()}
        g = {}
        for name in (f"{b}+grav_only", f"{b}+vgicp_grav", f"{b}+vgicp", f"{b}+vgicp+planar", f"{b}+planar"):
            tr = load_run(name, "full", cfg)
            if tr is not None:
                g[name] = evaluate_all_spans(tr, cfg, dt)
        rep["gravity_locked"][b] = g
    rep["decomposition"]["kiss_icp"] = {sp: decompose("kiss_icp", "full", cfg, sp) for sp in ("whole", "test")}
    rep["fusion"] = {}
    for name in ("fast_lio", "faster_lio", "fast_lio_correctors", "faster_lio_correctors"):
        try:
            rep["fusion"][name] = fusion_canonical(cfg, name)
        except Exception as exc:  # noqa: BLE001
            rep["fusion"][name] = {"error": str(exc)}
    (out / "audit.json").write_text(json.dumps(rep, indent=2, default=float))
    return rep


def fusion_canonical(cfg: dict, name: str) -> dict:
    """Canonical test-block metrics of a saved fusion model (results/fusion/<name>/weights_holdout.csv):
    fused step = Σ w·step over its methods, integrated over the test block, then the same
    200 m start fit as every other trajectory."""
    from .dataset import build
    from .geometry import integrate_increments
    import torch
    fdir = results_dir("fusion", name)
    ck = torch.load(fdir / "gating_model.pt", weights_only=False)
    methods = ck["methods"]
    st = build(cfg, methods=methods)
    w = pd.read_csv(fdir / "weights_holdout.csv")
    U = np.stack([st.inc[m] for m in methods], 1)
    wt = w[[f"w_trans_{m}" for m in methods]].to_numpy()
    wy = w[[f"w_yaw_{m}" for m in methods]].to_numpy()
    u = np.c_[(wt[:, :, None] * np.nan_to_num(U[:, :, :2])).sum(1), (wy * np.nan_to_num(U[:, :, 2])).sum(1)]
    rows = np.flatnonzero(w["split"].to_numpy() == "test")
    res = {}
    lo, hi = float(st.t[rows[0]]), float(st.t[rows[-1]])

    def score(label, uu):
        xy, yw = integrate_increments(uu[rows], st.gt_xy[rows[0]], st.gt_yaw[rows[0]])
        tr = Trajectory(label, st.t[rows], np.c_[xy, np.zeros(len(xy))], Rotation.from_euler("z", yw[:, None]).as_quat(),
                        {"world_frame": "gravity"})
        res[label] = canonical(tr, cfg, (lo, hi), 0.0)
    score("fused:" + ck.get("chosen", "?"), u)
    for i, m in enumerate(methods):
        score(m, pd.DataFrame(U[:, i]).ffill().fillna(0).to_numpy())
    return res
