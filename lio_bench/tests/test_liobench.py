"""Unit tests on synthetic data (no bag needed).  python -m pytest lio_bench/tests -q"""
import numpy as np
import pandas as pd
import pytest
from scipy.spatial.transform import Rotation

from liobench import fusion
from liobench.dataset import StepTable
from liobench.geometry import (body_increments, fit_alignment, integrate_increments, interpolate, leveling_rotation,
                               umeyama, cumulative_distance)
from liobench.io import Trajectory
from liobench.metrics import estimate_time_offset, segment_drift
from liobench.windows import adaptive, periodic


def _circle(n=2000, dt=0.1, v=15.0, r=300.0, t0=1.7e9):
    t = t0 + np.arange(n) * dt
    th = v * (t - t0) / r
    p = np.c_[r * np.sin(th), r * (1 - np.cos(th)), 0.01 * (t - t0)]
    yaw = th
    return t, p, yaw


def test_umeyama_recovers_rigid_transform():
    rng = np.random.default_rng(1)
    src = rng.normal(size=(200, 3)) * 50
    R = Rotation.from_euler("zyx", [30, 2, -3], degrees=True).as_matrix()
    dst = src @ R.T + [10, -5, 2]
    R2, t2, s = umeyama(src, dst)
    assert np.allclose(R2, R, atol=1e-9) and np.allclose(t2, [10, -5, 2], atol=1e-7) and s == 1.0


def test_increments_roundtrip_is_exact():
    t, p, yaw = _circle()
    u = body_increments(p, yaw)
    xy, yw = integrate_increments(u, p[0], yaw[0])
    assert np.abs(xy - p[:, :2]).max() < 1e-6
    # pure forward motion on a circle: lateral ~ chord sagitta, tiny
    assert np.nanmax(np.abs(u[1:, 1])) < 0.01


def test_increments_are_frame_independent():
    t, p, yaw = _circle()
    R = Rotation.from_euler("z", 77, degrees=True).as_matrix()
    u1 = body_increments(p, yaw)
    u2 = body_increments(p @ R.T + [1000, -2000, 5], yaw + np.radians(77))
    assert np.nanmax(np.abs(u1 - u2)) < 1e-9


def test_time_offset_estimation():
    t, p, _ = _circle(v=5.0)
    # vary the speed so the profile has structure
    s = np.cumsum(5 + 3 * np.sin(np.arange(len(t)) * 0.02)) * 0.1
    th = s / 300.0
    p = np.c_[300 * np.sin(th), 300 * (1 - np.cos(th)), np.zeros_like(th)]
    ref = Trajectory("ref", t, p)
    est = Trajectory("est", t + 0.237, p)                  # estimate stamped 237 ms late
    off = estimate_time_offset(est, ref)
    assert abs(off["dt_s"] + 0.237) < 0.01


def test_se2_head_uses_only_head():
    t, p, _ = _circle()
    d = cumulative_distance(p)
    est = p.copy()
    est[d > 200, 0] += 50.0                                # corrupt everything after the head
    R, tt, s, info = fit_alignment(est, p, "se2_head", d, head_m=200)
    assert abs(info["yaw_deg"]) < 1e-6 and np.allclose(tt[:2], 0, atol=1e-6)


def test_segment_drift_is_zero_for_rotated_copy():
    t, p, _ = _circle()
    d = cumulative_distance(p)
    R = Rotation.from_euler("z", 40, degrees=True).as_matrix()[:2, :2]
    _, dr = segment_drift(p[:, :2] @ R.T + 500, p[:, :2], d, 100)
    assert np.nanmax(dr) < 1e-6


def test_leveling_rotation():
    g = np.array([-0.17, 0.98, 10.0])
    R = leveling_rotation(g)
    assert np.allclose(R @ (g / np.linalg.norm(g)), [0, 0, 1], atol=1e-9)


def test_interpolate_refuses_gaps():
    tr = Trajectory("x", np.array([0, 0.1, 0.2, 1.0, 1.1]), np.arange(15.0).reshape(5, 3))
    _, _, ok = interpolate(tr, np.array([0.05, 0.6, 1.05]), max_gap=0.15)
    assert ok.tolist() == [True, False, True]


def test_window_schedules():
    t = np.arange(0, 100, 0.1)
    score = np.where((t > 20) & (t < 35), 2.0, 0.1)
    w = adaptive(t, score, 1.0, 0, 100, hold_s=2, min_len_s=5, max_len_s=60)
    assert len(w) == 1 and abs(w[0][0] - 20.1) < 0.11 and 36.5 < w[0][1] < 37.5
    assert len(periodic(t, 10, 30, 0, 100)) == 3


def _synthetic_steptable(n=3000, seed=0):
    """Two methods: A is good except in 'bad' regimes where it shrinks forward motion; B is noisy but unbiased."""
    rng = np.random.default_rng(seed)
    t = 1.7e9 + np.arange(n) * 0.1
    fwd = 2.0 + 0.3 * np.sin(np.arange(n) * 0.01)
    gt = np.c_[fwd, np.zeros(n), 0.002 * np.sin(np.arange(n) * 0.003)]
    bad = (np.sin(np.arange(n) * 0.02) > 0.4)          # ~314-step period: every split block has both regimes
    a = gt.copy(); a[bad, 0] *= 0.7; a += rng.normal(0, 0.005, a.shape)
    b = gt + rng.normal(0, 0.05, gt.shape)
    ctx = pd.DataFrame({"t": t, "scan_trans_cond": np.where(bad, 0.01, 0.06) + rng.normal(0, 0.003, n)})
    xy, yaw = integrate_increments(gt, np.zeros(2), 0.0)
    return StepTable(t, gt, xy, yaw, {"A": a, "B": b}, ctx, np.ones(n, bool))


def test_gating_learns_regime_switch():
    st = _synthetic_steptable()
    methods = ["A", "B"]
    U = np.stack([st.inc[m] for m in methods], 1)
    mask = np.isfinite(U).all(2)
    X, names = fusion.make_features(st, methods)
    tr, va, te = fusion.chronological_split(len(st.t), st.t, 0.6, 0.2, 5)
    fit = fusion.train_gating(X, U, mask, st.gt, tr, va, epochs=150, seed=0)
    wt, _ = fusion.predict_weights(fit, X, mask)
    bad = st.ctx["scan_trans_cond"].to_numpy() < 0.035
    # trusts A (precise) when its regime is good, shifts to B when A shrinks
    assert wt[te][~bad[te], 0].mean() > 0.6
    assert wt[te][bad[te], 0].mean() < 0.4
    u = np.c_[(wt[:, :, None] * U[:, :, :2]).sum(1), (wt * U[:, :, 2]).sum(1)]
    e_fused = np.abs(u[te, 0] - st.gt[te, 0]).mean()
    e_a = np.abs(U[te, 0, 0] - st.gt[te, 0]).mean()
    e_b = np.abs(U[te, 1, 0] - st.gt[te, 0]).mean()
    assert e_fused < min(e_a, e_b)


def test_masked_method_gets_zero_weight():
    st = _synthetic_steptable(n=800)
    methods = ["A", "B"]
    st.inc["B"][::3] = np.nan
    U = np.stack([st.inc[m] for m in methods], 1)
    mask = np.isfinite(U).all(2)
    X, _ = fusion.make_features(st, methods)
    tr, va, te = fusion.chronological_split(len(st.t), st.t, 0.6, 0.2, 2)
    fit = fusion.train_gating(X, U, mask, st.gt, tr, va, epochs=5)
    wt, wy = fusion.predict_weights(fit, X, mask)
    assert np.all(wt[~mask[:, 1], 1] == 0) and np.allclose(wt.sum(1), 1, atol=1e-5)


# --------------------------------------------------------------------------- phase 2 (correct/)

def test_posetrack_interpolation_and_clock_inverse():
    from liobench.correct.common import PoseTrack, to_matrices, unix_to_sensor
    from liobench.bag import sensor_to_unix
    t = np.array([0.0, 1.0])
    q = Rotation.from_euler("z", [[0], [90]], degrees=True).as_quat()
    tr = PoseTrack(t, to_matrices(np.array([[0, 0, 0], [2.0, 0, 0]]), q))
    T, ok = tr.at([0.5, 2.0], max_gap=2.0)
    assert ok.tolist() == [True, False]
    assert np.allclose(T[0, :3, 3], [1, 0, 0]) and abs(np.degrees(Rotation.from_matrix(T[0, :3, :3]).as_euler("zyx")[0]) - 45) < 1e-9
    clock = {"offset_s": 1.7e9, "drift": 1.05e-4, "stamp0": 13214.0}
    s = np.array([13214.0, 13500.0, 13756.3])
    assert np.abs(unix_to_sensor(sensor_to_unix(s, clock), clock) - s).max() < 1e-6


def test_wheel_replaces_only_forward_component():
    from liobench.correct.wheel import replace_forward
    f = np.array([np.cos(0.1), np.sin(0.1), 0.0])
    t = 1.9 * f + np.array([0.0, 0.0, 0.05]) + 0.02 * np.cross([0, 0, 1], f)
    t2 = replace_forward(t, f, 2.0)
    assert abs(t2 @ f - 2.0) < 1e-12 and np.allclose(t2 - (t2 @ f) * f, t - (t @ f) * f)


def test_pole_detector_finds_pole_and_rejects_wall():
    from liobench.correct.poles import detect
    rng = np.random.default_rng(0)
    gx, gy = np.meshgrid(np.arange(-35, 35, 0.4), np.arange(-35, 35, 0.4))
    ground = np.c_[gx.ravel(), gy.ravel(), -2.0 + 0.01 * rng.normal(size=gx.size)]
    pole = np.c_[12.0 + 0.02 * rng.normal(size=30), -6.0 + 0.02 * rng.normal(size=30), np.linspace(-1.6, -0.2, 30)]
    wall = np.c_[np.linspace(-10, 10, 400), np.full(400, 8.0), np.tile(np.linspace(-1.6, -0.2, 20), 20)]
    d = detect(np.r_[ground, pole, wall])
    assert len(d) == 1 and np.hypot(d[0, 0] - 12.0, d[0, 1] + 6.0) < 0.1


def test_pole_filter_reanchoring_is_exact():
    from liobench.correct.poles import _rot
    # C(q) = c + t + R(q - c); moving the anchor c -> c' with t' = t + (R - I)(c' - c) gives the same map
    th, t, c, c2 = 0.03, np.array([1.0, -2.0]), np.array([500.0, 300.0]), np.array([520.0, 310.0])
    q = np.array([600.0, 250.0])
    R = _rot(th)
    t2 = t + (R - np.eye(2)) @ (c2 - c)
    assert np.allclose(c + t + R @ (q - c), c2 + t2 + R @ (q - c2))


def test_degeneracy_aware_fusion_keeps_prior_along_corridor():
    sg = pytest.importorskip("small_gicp")
    from liobench.correct import scanpass as sp
    rng = np.random.default_rng(0)
    x = rng.uniform(-30, 30, 20000)
    pts = np.r_[np.c_[x[:8000], rng.uniform(-5, 5, 8000), np.zeros(8000)],
                np.c_[x[8000:14000], np.full(6000, 5.0), rng.uniform(0, 3, 6000)],
                np.c_[x[14000:], np.full(6000, -5.0), rng.uniform(0, 3, 6000)]]
    tgt, tree = sg.preprocess_points(pts, 0.3, 10, 2)
    src, _ = sg.preprocess_points(pts + rng.normal(0, 0.01, pts.shape), 0.3, 10, 2)
    T0 = np.eye(4); T0[:3, 3] = [0.3, 0.2, 0.0]                  # wrong by 0.3 m along, 0.2 m across
    r = sg.align(tgt, src, tree, T0, registration_type="PLANE_ICP", max_correspondence_distance=1.0)
    D = np.linalg.inv(T0) @ np.asarray(r.T_target_source)
    d = np.r_[Rotation.from_matrix(D[:3, :3]).as_rotvec(), D[:3, 3]]
    s2 = 2 * r.error / r.num_inliers
    I = np.asarray(r.H) / s2 / sp.N_CORR
    L = np.diag([sp.SIGMA_PRIOR_R ** -2] * 3 + [sp.SIGMA_PRIOR_T ** -2] * 3)
    d2 = np.linalg.solve(I + L, I @ d)
    assert abs(d2[4] + 0.2) < 0.02          # across the corridor: follows ICP back to 0
    assert abs(d2[3]) < 0.05                # along it: ICP has no information, the prior is kept


def test_linear_gating_and_static_checkpoint_shapes():
    m = fusion.GatingMLP(7, 3, hidden=0)
    wt, wy = m(torch_rand(5, 7), torch_ones_mask(5, 3))
    assert wt.shape == (5, 3) and np.allclose(wt.sum(1).detach().numpy(), 1, atol=1e-6)
    U = np.ones((4, 2, 3)); U[:, 1] *= 3
    mask = np.ones((4, 2), bool); mask[0, 1] = False
    u = fusion.apply_static(U, mask, np.array([[0.5] * 3, [0.5] * 3]))
    assert np.allclose(u[0], 1) and np.allclose(u[1:], 2)


def torch_rand(*shape):
    import torch
    return torch.rand(*shape)


def torch_ones_mask(*shape):
    import torch
    return torch.ones(*shape, dtype=torch.bool)


def test_planarize_removes_world_tilt_drift():
    from liobench.correct.common import PoseTrack
    from liobench.correct.planar import planarize
    t, p, yaw = _circle(n=600)
    up = np.array([0.0, 0.0, 1.0])
    T = np.tile(np.eye(4), (len(t), 1, 1))
    T[:, :3, :3] = Rotation.from_euler("z", yaw[:, None]).as_matrix()
    T[:, :2, 3] = p[:, :2]
    # the estimator's world slowly tips over: a growing pitch of the whole trajectory about the start
    tilt = Rotation.from_euler("y", np.linspace(0, 40, len(t))[:, None], degrees=True).as_matrix()
    Tw = T.copy()
    for k in range(1, len(t)):            # apply each step in a world that has tipped by tilt[k]
        D = np.linalg.inv(T[k - 1]) @ T[k]
        Tw[k] = Tw[k - 1] @ D
        Tw[k, :3, :3] = tilt[k] @ T[k, :3, :3]
        Tw[k, :3, 3] = Tw[k - 1, :3, 3] + tilt[k] @ (T[k, :3, 3] - T[k - 1, :3, 3])
    horiz = np.sum(np.linalg.norm(np.diff(Tw[:, :2, 3], axis=0), axis=1)) / np.sum(np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1))
    P = planarize(PoseTrack(t, Tw), up, x0=p[0, :2], yaw0=yaw[0])
    assert horiz < 0.97                                     # the tilted world shortens the horizontal track (mean cos 0-40 deg)
    assert np.abs(P[:, :2, 3] - p[:, :2]).max() < 0.05     # the planar rebuild recovers it


# --------------------------------------------------------------------------- v3 (heading / evaluation)

def test_integrate_removes_constant_yaw_bias_and_resample_is_consistent():
    from liobench.correct import heading as H
    t = np.arange(0, 100, 0.1)
    n = len(t)
    u_true = np.c_[np.full(n, 2.0), np.zeros(n), np.r_[0.0, np.full(n - 1, 0.001)]]
    bias = np.radians(0.05)                                   # 0.05 deg/s counter-clockwise
    u_biased = u_true.copy()
    u_biased[1:, 2] += bias * 0.1
    T_true = H.integrate(t, u_true)
    T_fix = H.integrate(t, u_biased, yaw_bias=bias)
    assert np.abs(T_fix[:, :2, 3] - T_true[:, :2, 3]).max() < 1e-6
    T_bad = H.integrate(t, u_biased)
    assert np.abs(T_bad[-1, :2, 3] - T_true[-1, :2, 3]).max() > 5.0
    # yaw increments resampled onto a coarser clock integrate to the same heading
    t2 = t[::3]
    dy2 = H.resample_dyaw(t, u_true[:, 2], t2)
    assert abs(dy2.sum() - u_true[:, 2][: len(t)][:(len(t2) - 1) * 3 + 1].sum()) < 1e-9


def test_no_slip_drops_lateral_motion_only():
    from liobench.correct import heading as H
    t = np.arange(0, 10, 0.1)
    u = np.c_[np.ones(len(t)), np.full(len(t), 0.1), np.zeros(len(t))]
    T = H.integrate(t, u, no_slip=True)
    assert np.allclose(T[:, 1, 3], 0) and abs(T[-1, 0, 3] - (len(t))) < 1e-9


def test_paired_differences_and_sign_counts():
    from liobench.v3 import paired
    per = {"ref": [{"ate_m": 10.0}, {"ate_m": 20.0}, {"ate_m": 30.0}],
           "new": [{"ate_m": 8.0}, {"ate_m": 18.0}, {"ate_m": 31.0}]}
    o = paired(per, "ref", keys=("ate_m",))["new"]["ate_m"]
    assert abs(o["diff_vs_ref"] - (-1.0)) < 1e-12 and o["blocks_better"] == 2


def test_gravity_filter_removes_slow_tilt_drift():
    from liobench.correct.common import PoseTrack
    from liobench.correct import heading as H
    n = 600
    t = np.arange(n) * 0.1
    # true motion: straight and level at 20 m/s; the estimator's attitude tips 0 -> 20 deg in pitch
    T = np.tile(np.eye(4), (n, 1, 1))
    pitch = np.radians(np.linspace(0, 20, n))
    for k in range(n):
        T[k, :3, :3] = Rotation.from_euler("y", -pitch[k]).as_matrix()
        if k:
            T[k, :3, 3] = T[k - 1, :3, 3] + T[k - 1, :3, :3] @ np.array([2.0, 0, 0])
    # IMU: level vehicle -> specific force straight up in the body, no rotation
    ti = np.arange(-1, t[-1] + 0.01, 0.01)
    imu = np.c_[ti, np.zeros((len(ti), 2)), np.full(len(ti), 9.81), np.zeros((len(ti), 3))]
    G = H.gravity_filter(PoseTrack(t, T), imu, tau_s=2.0)
    tilt_end = np.degrees(np.arccos(np.clip(G[-1, 2, 2], -1, 1)))
    raw_end = np.degrees(np.arccos(np.clip(T[-1, 2, 2], -1, 1)))
    assert raw_end > 19 and tilt_end < 1.0


def test_blend_heading_limits_and_bias_removal():
    from liobench.correct.heading import blend_heading
    t = np.arange(0, 200, 0.1)
    n = len(t)
    true = np.r_[0.0, np.full(n - 1, 0.002)]
    fast = true + np.r_[0.0, np.full(n - 1, np.radians(0.05) * 0.1)]          # biased but smooth
    rng = np.random.default_rng(0)
    slow = true + np.r_[0.0, rng.normal(0, 0.0003, n - 1)]                      # unbiased, mildly noisy (integrated RW ~0.013 rad)
    assert np.allclose(np.cumsum(blend_heading(t, fast, slow, np.inf)), np.cumsum(fast))
    assert np.allclose(np.cumsum(blend_heading(t, fast, slow, 1e-9)), np.cumsum(slow))
    b = np.cumsum(blend_heading(t, fast, slow, 10.0))
    err_b, err_f = np.abs(b - np.cumsum(true))[-1], np.abs(np.cumsum(fast) - np.cumsum(true))[-1]
    assert err_b < 0.2 * err_f                                                    # bias pulled out
