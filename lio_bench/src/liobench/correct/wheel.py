"""Forward motion from CAN wheel speed.

The baselines' main weakness on this road is along-track: the scene constrains
lateral and vertical motion well and forward motion poorly. Wheel speed measures
exactly that component. Three quantities are fitted, all on the training block:
  lag     CAN stamp -> true time (speed-profile match against ground truth)
  scale   ground-truth distance / CAN distance (tyre radius, speedometer bias)
  axis    the vehicle's forward direction in the IMU body frame (mean direction of
          the baseline's body-frame velocity while moving)
"""
from __future__ import annotations

import json

import numpy as np

from ..bag import fit_sensor_clock, load_aux, sensor_to_unix
from ..geometry import course_from_positions
from ..io import load_ground_truth, results_dir
from .common import baseline_track, inherit_frame, inv, split_intervals, write_run


def _can(cfg):
    can = load_aux("steering")
    if can is None:
        raise FileNotFoundError("results/bag/steering.csv missing - run `liobench prepare` first")
    can = can.sort_values("t_stamp").drop_duplicates("t_stamp")
    return can["t_stamp"].to_numpy(float), can["speed_mps"].to_numpy(float)


def calibrate(cfg: dict, baseline: str, tag: str = "full") -> dict:
    lo, hi = split_intervals(cfg)["train"]
    gt = load_ground_truth(cfg)
    _, v_gt = course_from_positions(gt.t, gt.p, 0.3, min_speed=0.0)
    t_can, v_can = _can(cfg)
    sel = (gt.t >= lo) & (gt.t <= hi) & np.isfinite(v_gt) & (v_gt > 1.0)
    tt, vv = gt.t[sel], v_gt[sel]

    lags = np.arange(-0.5, 0.5001, 0.01)
    cost = [np.mean(np.abs(np.interp(tt - lag, t_can, v_can) - vv)) for lag in lags]
    lag = float(lags[int(np.argmin(cost))])
    vc = np.interp(tt - lag, t_can, v_can)
    scale = float(vv.sum() / vc.sum())

    track = baseline_track(baseline, tag)
    clock = fit_sensor_clock(cfg)
    u = sensor_to_unix(track.t, clock)
    d = (inv(track.T[:-1]) @ track.T[1:])[:, :3, 3]          # body-frame step
    dt = np.diff(track.t)
    sp = np.linalg.norm(d, axis=1) / np.maximum(dt, 1e-6)
    m = (u[1:] >= lo) & (u[1:] <= hi) & (sp > 3.0) & (sp < 60.0)
    f = d[m].sum(0)
    f /= np.linalg.norm(f)

    # per-step wheel distance error on the training block (for the record and for priors)
    D = can_distance_fn(t_can, v_can, lag, scale)
    gt_step = np.linalg.norm(np.diff(gt.p[:, :2], axis=0), axis=1)
    wheel_step = D(gt.t[1:]) - D(gt.t[:-1])
    ms = sel[1:] & sel[:-1]
    err = wheel_step[ms] - gt_step[ms]
    cal = {"baseline": baseline, "train_interval_unix": [lo, hi], "lag_s": lag, "scale": scale,
           "forward_axis_body": f.tolist(),
           "forward_axis_yaw_deg": float(np.degrees(np.arctan2(f[1], f[0]))),
           "forward_axis_pitch_deg": float(np.degrees(np.arcsin(np.clip(f[2], -1, 1)))),
           "train_step_err_m": {"bias": float(err.mean()), "std": float(err.std()),
                                "rel_pct": float(100 * np.abs(err).sum() / gt_step[ms].sum())},
           "speed_mae_at_lag_mps": float(min(cost))}
    out = results_dir("correct", baseline)
    (out / "wheel_calibration.json").write_text(json.dumps(cal, indent=2))
    return cal


def can_distance_fn(t_can, v_can, lag, scale):
    """D(t_unix): CAN distance travelled since the first sample (trapezoid integral)."""
    tc = t_can + lag
    cum = np.r_[0.0, np.cumsum(0.5 * (v_can[1:] + v_can[:-1]) * np.diff(tc))] * scale
    return lambda t: np.interp(t, tc, cum)


def load_calibration(cfg: dict, baseline: str, tag: str = "full") -> dict:
    f = results_dir("correct", baseline) / "wheel_calibration.json"
    return json.loads(f.read_text()) if f.exists() else calibrate(cfg, baseline, tag)


def wheel_steps(cfg: dict, cal: dict, t_sensor: np.ndarray) -> np.ndarray:
    """CAN distance between consecutive sensor stamps (NaN where CAN does not cover)."""
    t_can, v_can = _can(cfg)
    D = can_distance_fn(t_can, v_can, cal["lag_s"], cal["scale"])
    u = sensor_to_unix(t_sensor, fit_sensor_clock(cfg))
    steps = np.r_[np.nan, D(u[1:]) - D(u[:-1])]
    covered = (u >= t_can[0] + cal["lag_s"]) & (u <= t_can[-1] + cal["lag_s"])
    steps[~covered] = np.nan
    return steps


def replace_forward(t_body: np.ndarray, f: np.ndarray, dist: float) -> np.ndarray:
    """Body-frame translation with its component along `f` replaced by `dist`."""
    return t_body - (t_body @ f) * f + dist * f


def run(cfg: dict, baseline: str, tag: str = "full") -> dict:
    cal = calibrate(cfg, baseline, tag)
    track = baseline_track(baseline, tag)
    f = np.asarray(cal["forward_axis_body"])
    w = wheel_steps(cfg, cal, track.t)
    Tn = track.T.copy()
    for k in range(1, len(track.t)):
        D = inv(track.T[k - 1]) @ track.T[k]
        if np.isfinite(w[k]):
            D = D.copy()
            D[:3, 3] = replace_forward(D[:3, 3], f, w[k])
        Tn[k] = Tn[k - 1] @ D
    write_run(f"{baseline}+wheel", tag, track.t, Tn,
              {"runner": "correct.wheel", "role": "corrector", **inherit_frame(baseline, tag), "calibration": cal})
    return cal
