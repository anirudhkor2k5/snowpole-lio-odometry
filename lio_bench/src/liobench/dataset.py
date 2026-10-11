"""The shared per-step table every learned / analysis step uses.

One row per ground-truth sample (10 Hz). For each method m the planar body-frame
increment u_m = [forward, lateral, dyaw] between consecutive samples (see
geometry.body_increments), plus ground-truth increments built the same way from
ground-truth positions and heading, plus causal context features.

Ground-truth heading: course over ground of the ground-truth track (vehicle
heading up to side-slip). The constant yaw between each estimator's IMU body
x-axis and the vehicle's direction of travel is estimated on the TRAINING block
only and removed, so increments of all methods live in the same vehicle frame.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import pandas as pd

from .bag import fit_sensor_clock, load_aux, sensor_to_unix
from .geometry import body_increments, course_from_positions, interpolate, level, wrap_pi, yaw_from_quat
from .io import Trajectory, load_config, load_ground_truth, load_run, results_dir, run_dir
from .metrics import estimate_time_offset

INC = ["fwd", "lat", "dyaw"]


@dataclass
class StepTable:
    t: np.ndarray                         # GT clock
    gt: np.ndarray                        # (N, 3) GT increments
    gt_xy: np.ndarray                     # (N, 2) GT positions
    gt_yaw: np.ndarray                    # (N,) GT heading (rad, CCW from east)
    inc: dict[str, np.ndarray]            # method -> (N, 3) increments, NaN where unavailable
    ctx: pd.DataFrame                     # causal context features, one row per step
    valid: np.ndarray                     # rows usable for training/evaluation
    mount_yaw: dict[str, float] = field(default_factory=dict)
    meta: dict = field(default_factory=dict)

    @property
    def methods(self) -> list[str]:
        return list(self.inc)


def _gt_heading(gt: Trajectory) -> np.ndarray:
    course, speed = course_from_positions(gt.t, gt.p, half_window_s=0.5, min_speed=1.0)
    # Hold the last good heading through slow / stopped segments.
    s = pd.Series(course).ffill().bfill().to_numpy()
    return np.unwrap(s)


def _method_increments(tr: Trajectory, t: np.ndarray, max_gap: float, cfg: dict) -> np.ndarray:
    """[forward, lateral, dyaw] between consecutive query times, in the method's own level body axes.

    Taken from the method's relative pose, not from its world positions: every estimator
    here drifts tens of degrees in roll/pitch over the drive, and differencing positions in
    that tilted world would shorten the forward step by cos(tilt) (see correct/planar.py).
    """
    from .correct.common import to_matrices
    from .correct.planar import body_steps, body_up
    p, q, valid = interpolate(tr, t, max_gap)
    u = np.full((len(t), 3), np.nan)
    if q is None:
        return u
    ok = valid & np.r_[False, valid[:-1]]
    i = np.flatnonzero(ok)
    if len(i) == 0:
        return u
    runner = tr.meta.get("run", {}).get("runner", "")
    up = np.array([0.0, 0.0, 1.0]) if runner in ("correct.planar", "correct.poles") else body_up(cfg)
    T = np.tile(np.eye(4), (len(t), 1, 1))
    T[valid] = to_matrices(p[valid], q[valid])
    u[i] = body_steps(T[i - 1], T[i], up)
    return u


def _context(t: np.ndarray, cfg: dict, methods: list[str]) -> pd.DataFrame:
    """Causal context per step: scan geometry/IMU (sensor), GenZ internals, CAN speed."""
    clock = fit_sensor_clock(cfg)
    ctx = pd.DataFrame({"t": t})

    def attach(df: pd.DataFrame, tcol: str, prefix: str, cols=None):
        df = df.sort_values(tcol)
        cols = cols or [c for c in df.columns if c != tcol and df[c].dtype.kind in "fiub"]
        idx = np.searchsorted(df[tcol].to_numpy(), t, side="right") - 1   # last sample at or before t
        ok = idx >= 0
        for c in cols:
            v = np.full(len(t), np.nan)
            v[ok] = df[c].to_numpy(float)[idx[ok]]
            ctx[f"{prefix}{c}"] = v

    sc = results_dir("features") / "scan_context.csv"
    if sc.exists():
        df = pd.read_csv(sc)
        df["t_unix"] = sensor_to_unix(df["stamp"], clock)
        attach(df.drop(columns=["stamp"]), "t_unix", "scan_")
    if "genz_lio" in methods and (run_dir("genz_lio") / "diagnostics.csv").exists():
        df = pd.read_csv(run_dir("genz_lio") / "diagnostics.csv")
        df = df[df["valid"]].copy()
        df["t_unix"] = sensor_to_unix(df["stamp"], clock)
        keep = ["plane_matches", "point_matches", "scale_indicator", "leaf_size", "voxelized_points",
                "median_range", "cov_pos_trace", "cov_rot_trace", "processing_time_ms"]
        attach(df[["t_unix"] + [k for k in keep if k in df]], "t_unix", "genz_")
    st = load_aux("steering")
    if st is not None:
        attach(st[["t_record", "speed_mps", "steering_wheel_angle_rad"]], "t_record", "can_")
    return ctx


def build(cfg: dict | None = None, methods: list[str] | None = None, train_frac: float | None = None,
          tag: str = "full") -> StepTable:
    cfg = cfg or load_config()
    ecfg = cfg["evaluation"]
    gt = load_ground_truth(cfg)
    t = gt.t
    gt_yaw = _gt_heading(gt)
    u_gt = body_increments(gt.p, gt_yaw)

    methods = methods or [m for m in ("fast_lio", "faster_lio", "dlio", "genz_lio") if load_run(m, tag, cfg) is not None]
    inc, offsets = {}, {}
    for m in methods:
        tr = load_run(m, tag, cfg)
        off = estimate_time_offset(level(tr), gt)
        offsets[m] = off["dt_s"]
        inc[m] = _method_increments(tr.shifted(off["dt_s"]), t, ecfg.get("max_assoc_dt", 0.15), cfg)

    # Mounting yaw per method, from the training block only.
    split = cfg["split"]
    n_train = int(len(t) * (train_frac if train_frac is not None else split["train"]))
    speed = np.r_[0, np.linalg.norm(np.diff(gt.p[:, :2], axis=0), axis=1) / np.diff(t)]
    moving = speed > 3.0
    mount = {}
    for m, u in inc.items():
        sel = moving[:n_train] & np.isfinite(u[:n_train]).all(1)
        mount[m] = float(np.arctan2(np.nansum(u[:n_train][sel, 1]), np.nansum(u[:n_train][sel, 0]))) if sel.any() else 0.0
        c, s = np.cos(-mount[m]), np.sin(-mount[m])
        f, l = u[:, 0].copy(), u[:, 1].copy()
        u[:, 0], u[:, 1] = c * f - s * l, s * f + c * l

    ctx = _context(t, cfg, methods)
    ctx["gt_speed_for_split_only"] = speed     # NOT a feature; used for masks/plots
    valid = np.isfinite(u_gt).all(1)
    return StepTable(t, u_gt, gt.p[:, :2], gt_yaw, inc, ctx, valid, mount,
                     {"time_offsets_s": offsets, "n_train_for_mount": n_train})
