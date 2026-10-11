"""One pass over the bag: ICP variants seeded by a baseline, KISS-ICP, pole candidates.

Per scan:
  1. deskew with the baseline's own motion (its poses interpolated at each point's time)
  2. pole candidates (poles.detect) on a gravity-levelled copy of the deskewed scan
  3. every ICP variant registers the scan to its *own* local map, starting from
         T_init = T_prev_corrected * prediction
     where prediction = the baseline's increment, or for '*_wheel' the baseline's
     increment with its forward component replaced by CAN wheel distance
  4. degeneracy-aware variants ('*_da*') do not take the ICP pose as is. They fuse
     the ICP correction d (6-DoF, body frame) with the prediction as a prior:
         d' = (I_icp + L_prior)^-1 I_icp d,   I_icp = H / s^2 / N_CORR
     H is ICP's own Gauss-Newton information matrix, s^2 its residual scatter.
     Directions the scene does not constrain (small eigenvalues of H, typically
     along the road here) then keep the prediction; well-constrained ones follow ICP.
  5. KISS-ICP runs independently on the raw scans (LiDAR-only odometry, no prior)

Variants and their meaning:
  icp            point-to-point ICP             (raw: ICP pose replaces the prediction)
  plane_icp      point-to-plane ICP             (raw)
  gicp           generalized ICP                (raw)
  vgicp          voxelized GICP (NDT-like distribution-to-distribution matching) (raw)
  plane_icp_da   point-to-plane, degeneracy-aware, baseline prior
  gicp_da        GICP, degeneracy-aware, baseline prior
  gicp_da_wheel  GICP, degeneracy-aware, prior = baseline with wheel-speed forward motion

Fixed (not fitted) noise levels, per 0.1 s scan step:
  SIGMA_PRIOR_T  0.02 m   baseline translation increment (smoke runs: ~0.3 m RMSE / 30 s)
  SIGMA_PRIOR_R  0.1 deg  baseline rotation increment
  N_CORR         10       ICP residuals are spatially correlated; count every 10th as independent
Wheel forward sigma comes from the wheel calibration (training block).
"""
from __future__ import annotations

import json
import time

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

from ..bag import fit_sensor_clock, sensor_to_unix
from ..geometry import leveling_rotation
from ..io import BENCH_ROOT, load_config, repo_path, results_dir, run_meta
from . import poles as poles_mod
from .common import baseline_track, inherit_frame, inv, lidar_in_imu, write_run

VARIANTS = {
    "icp":           {"reg": "ICP"},
    "plane_icp":     {"reg": "PLANE_ICP"},
    "gicp":          {"reg": "GICP"},
    "vgicp":         {"reg": "VGICP"},
    "plane_icp_da":  {"reg": "PLANE_ICP", "prior": "baseline"},
    "gicp_da":       {"reg": "GICP", "prior": "baseline"},
    "gicp_da_wheel": {"reg": "GICP", "prior": "wheel"},
    # gravity-locked: roll/pitch replaced every scan by the IMU's gravity direction (motion-compensated),
    # so attitude cannot drift; x, y, z, yaw from ICP. 'grav_only' = no registration at all.
    "vgicp_grav":    {"reg": "VGICP", "gravity": True},
    "grav_only":     {"reg": "NONE", "gravity": True},
    # information-weighted (filter-style) vs direct: same constraint, fused with the prediction
    "vgicp_grav_da": {"reg": "VGICP", "gravity": True, "prior": "baseline"},
    # selective correction: register only every n-th scan; the map is updated every scan
    "vgicp_grav_every2":  {"reg": "VGICP", "gravity": True, "every": 2},
    "vgicp_grav_every5":  {"reg": "VGICP", "gravity": True, "every": 5},
    "vgicp_grav_every10": {"reg": "VGICP", "gravity": True, "every": 10},
    "vgicp_grav_every20": {"reg": "VGICP", "gravity": True, "every": 20},
    # leakage test: rotation of the prediction from a constant-velocity model of the corrector's
    # own last step (as KISS-ICP does) instead of the LIO's gyro-driven rotation; translation
    # of the prediction still from the LIO
    "vgicp_grav_cvrot": {"reg": "VGICP", "gravity": True, "cv_rotation": True},
    # and fully constant-velocity (no LIO in the prediction at all)
    "vgicp_grav_cv":    {"reg": "VGICP", "gravity": True, "cv_rotation": True, "cv_translation": True},
}
GRAVITY_WINDOW_S = 1.0  # accelerometer averaging window for the gravity direction
GRAVITY_TAU_S = 5.0     # complementary filter: roll/pitch pulled toward gravity with this time constant
SIGMA_PRIOR_T = 0.02
SIGMA_PRIOR_R = np.radians(0.1)
N_CORR = 10.0

SRC_LEAF = 0.5          # source downsampling (m)
MAP_LEAF = 0.5          # local map voxel (m)
VGICP_RES = 1.0         # VGICP voxel (m)
MAX_CORR = 1.0          # max correspondence distance (m); the prediction is cm-accurate
MAP_RADIUS = 80.0       # target cloud = map points within this radius of the vehicle
LRU_HORIZON = 100       # scans a map voxel survives without being refreshed
REBUILD_EVERY = 2       # rebuild the target cloud / kd-tree every n scans
MIN_INLIERS = 200
MAX_JUMP_T, MAX_JUMP_R = 1.0, np.radians(2.0)   # larger ICP corrections = failure -> prediction


def _so3_log(R):
    return Rotation.from_matrix(R).as_rotvec()


def relevel(R: np.ndarray, up_b: np.ndarray) -> np.ndarray:
    """Rotation with the body's measured 'up' mapped to world z and the body's heading kept."""
    z = np.array([0.0, 0.0, 1.0])
    ax = np.cross(up_b, z)
    s_, c_ = np.linalg.norm(ax), float(up_b @ z)
    R0 = np.eye(3) if s_ < 1e-12 else Rotation.from_rotvec(ax / s_ * np.arctan2(s_, c_)).as_matrix()
    fx = np.array([1.0, 0.0, 0.0]) - up_b[0] * up_b
    h, h0 = R @ fx, R0 @ fx
    dyaw = np.arctan2(h[1], h[0]) - np.arctan2(h0[1], h0[0])
    return Rotation.from_euler("z", dyaw).as_matrix() @ R0


def tilt_correct(R: np.ndarray, up_b: np.ndarray, alpha: float) -> np.ndarray:
    """Rotate the body a fraction `alpha` of the way from its current 'up' (R^T z) to the measured
    'up' (complementary filter). alpha = 1 snaps to gravity; small alpha removes slow drift without
    passing accelerometer noise into the attitude (and into the map)."""
    u = R.T @ np.array([0.0, 0.0, 1.0])
    ax = np.cross(u, up_b)
    sn = np.linalg.norm(ax)
    if sn < 1e-12:
        return R
    ang = np.arctan2(sn, float(u @ up_b))
    return R @ Rotation.from_rotvec(-ax / sn * alpha * ang).as_matrix()


def gravity_up(imu: np.ndarray, speed: float, dvdt: float) -> np.ndarray | None:
    """Body 'up' from accelerometer specific force minus the vehicle's own acceleration.

    f = a + g_up (specific force); a = dv/dt * x + omega x v with v = speed * x (no side-slip).
    Uses only the IMU and the baseline's speed: LiDAR + IMU, no CAN.
    """
    if imu is None or len(imu) < 10:
        return None
    f = imu[:, 1:4].mean(0)
    w = imu[:, 4:7].mean(0)
    x = np.array([1.0, 0.0, 0.0])
    a = dvdt * x + np.cross(w, speed * x)
    u = f - a
    n = np.linalg.norm(u)
    return u / n if n > 5.0 else None


def _exp(d):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(d[:3]).as_matrix()
    T[:3, 3] = d[3:]
    return T


class _Registrar:
    """One ICP variant with its own local map and its own corrected trajectory."""

    def __init__(self, name: str, spec: dict, threads: int, f_axis: np.ndarray, wheel_sigma: float):
        import small_gicp as sg
        self.sg, self.name, self.reg = sg, name, spec["reg"]
        self.prior = spec.get("prior")
        self.gravity = bool(spec.get("gravity"))
        self.every = int(spec.get("every", 1))
        self.cv_rot = bool(spec.get("cv_rotation"))
        self.cv_trans = bool(spec.get("cv_translation"))
        self.last_delta = None
        self.alpha = 0.1 / GRAVITY_TAU_S          # per 0.1 s scan
        self.threads = threads
        self.f = f_axis
        self.wheel_sigma = wheel_sigma
        if self.reg == "NONE":
            self.map = None
        else:
            self.map = sg.GaussianVoxelMap(VGICP_RES) if self.reg == "VGICP" else sg.IncrementalVoxelMap(MAP_LEAF)
            self.map.set_lru(horizon=LRU_HORIZON, clear_cycle=10)
        self.target = self.tree = None
        self.n = 0
        self.T = None
        self.poses, self.diag = [], []
        self.seconds = 0.0

    def _rebuild(self, center):
        pts = np.asarray(self.map.voxel_points())[:, :3]
        pts = pts[np.linalg.norm(pts[:, :2] - center[:2], axis=1) < MAP_RADIUS]
        if len(pts) < 100:
            self.target = self.tree = None
            return
        self.target, self.tree = self.sg.preprocess_points(pts, downsampling_resolution=MAP_LEAF,
                                                           num_neighbors=10, num_threads=self.threads)

    def step(self, src, stamp: float, pred: np.ndarray, wheel_used: bool, up_b: np.ndarray | None = None):
        t0 = time.perf_counter()
        sg = self.sg
        row = {"stamp": stamp, "fallback": 0, "inliers": 0, "s_m": np.nan, "corr_f": np.nan, "corr_l": np.nan,
               "corr_v": np.nan, "corr_yaw_deg": np.nan, "eig_min": np.nan, "eig_max": np.nan, "weak_fwd_cos": np.nan}
        T_prev = None if self.T is None else self.T.copy()
        if T_prev is not None and (self.cv_rot or self.cv_trans) and self.last_delta is not None:
            pred = pred.copy()                       # constant-velocity parts of the prediction
            if self.cv_rot:
                pred[:3, :3] = self.last_delta[:3, :3]
            if self.cv_trans:
                pred[:3, 3] = self.last_delta[:3, 3]
        if T_prev is None:                                   # first scan: start on the baseline
            self.T = pred.copy()
            if self.gravity and up_b is not None:
                self.T[:3, :3] = relevel(self.T[:3, :3], up_b)
        elif self.reg == "NONE" or (self.every > 1 and self.n % self.every != 0):
            self.T = self.T @ pred          # no registration this scan (map still updated below)
            row["fallback"] = 3 if self.reg != "NONE" else 0
            if self.gravity and up_b is not None:
                self.T[:3, :3] = tilt_correct(self.T[:3, :3], up_b, self.alpha)
        else:
            T_init = self.T @ pred
            res = None
            if self.reg == "VGICP":
                if self.map.size() > 0:
                    res = sg.align(self.map, src, T_init, max_correspondence_distance=MAX_CORR,
                                   num_threads=self.threads)
            else:
                if self.target is None or self.n % REBUILD_EVERY == 0:
                    self._rebuild(T_init[:3, 3])
                if self.target is not None:
                    res = sg.align(self.target, src, self.tree, T_init, registration_type=self.reg,
                                   max_correspondence_distance=MAX_CORR, num_threads=self.threads)
            T_new = T_init
            if res is not None and res.converged and res.num_inliers >= MIN_INLIERS:
                D = inv(T_init) @ np.asarray(res.T_target_source)
                d = np.r_[_so3_log(D[:3, :3]), D[:3, 3]]        # body frame, [rot, trans] like small_gicp
                H = np.asarray(res.H)
                dof = 1.0 if self.reg == "PLANE_ICP" else 3.0
                s2 = max(2.0 * res.error / (res.num_inliers * dof), 1e-6)
                I = H / s2 / N_CORR
                Htt = H[3:, 3:] - H[3:, :3] @ np.linalg.solve(H[:3, :3] + 1e-9 * np.eye(3), H[:3, 3:])
                w, V = np.linalg.eigh(Htt / s2 / N_CORR)
                row.update(inliers=int(res.num_inliers), s_m=float(np.sqrt(s2)), eig_min=float(w[0]),
                           eig_max=float(w[-1]), weak_fwd_cos=float(abs(V[:, 0] @ self.f)))
                if np.linalg.norm(d[3:]) > MAX_JUMP_T or np.linalg.norm(d[:3]) > MAX_JUMP_R:
                    row["fallback"] = 2
                else:
                    if self.prior:
                        L = np.diag([SIGMA_PRIOR_R ** -2] * 3 + [SIGMA_PRIOR_T ** -2] * 3)
                        if self.prior == "wheel" and wheel_used:
                            # wheel distance is tighter than the baseline along the forward axis
                            Lt = (np.eye(3) - np.outer(self.f, self.f)) * SIGMA_PRIOR_T ** -2 \
                                + np.outer(self.f, self.f) * self.wheel_sigma ** -2
                            L[3:, 3:] = Lt
                        d = np.linalg.solve(I + L, I @ d)
                    T_new = T_init @ _exp(d)
                    lat = np.cross([0, 0, 1.0], self.f)
                    lat /= max(np.linalg.norm(lat), 1e-9)
                    row.update(corr_f=float(d[3:] @ self.f), corr_l=float(d[3:] @ lat), corr_v=float(d[5]),
                               corr_yaw_deg=float(np.degrees(d[2])))
            else:
                row["fallback"] = 1
            if self.gravity and up_b is not None:
                T_new = T_new.copy()
                T_new[:3, :3] = tilt_correct(T_new[:3, :3], up_b, self.alpha)
            self.T = T_new
        # keep the pose rigid: re-project the rotation onto SO(3) every scan. Without this the
        # constant-velocity prediction (rotation built from the previous step) amplifies rounding
        # error ~2.4x per scan until the pose explodes.
        U_, _, Vt_ = np.linalg.svd(self.T[:3, :3])
        self.T = self.T.copy()
        self.T[:3, :3] = U_ @ Vt_
        if T_prev is not None:
            self.last_delta = inv(T_prev) @ self.T
        if self.map is not None:
            self.map.insert(src, self.T)
        self.n += 1
        self.poses.append(self.T.copy())
        self.diag.append(row)
        self.seconds += time.perf_counter() - t0


def run(cfg: dict | None = None, baseline: str = "fast_lio", baseline_tag: str = "full", tag: str = "full",
        variants: list[str] | None = None, kiss: bool = True, poles: bool = True, threads: int = 4,
        start_offset_s: float = 0.0, duration_s: float | None = None, progress_every: int = 250) -> dict:
    import small_gicp as sg
    from genz_lio.config import load_config as genz_config
    from ..genz_runner import iter_frames
    from .wheel import can_distance_fn, _can, load_calibration

    cfg = cfg or load_config()
    variants = variants if variants is not None else list(VARIANTS)
    track = baseline_track(baseline, baseline_tag)
    T_IL = lidar_in_imu(cfg)
    clock = fit_sensor_clock(cfg)
    cal = load_calibration(cfg, baseline, baseline_tag)
    f_axis = np.asarray(cal["forward_axis_body"])
    wheel_sigma = max(float(cal["train_step_err_m"]["std"]), 0.005)
    t_can, v_can = _can(cfg)
    D_can = can_distance_fn(t_can, v_can, cal["lag_s"], cal["scale"])

    meta_b = run_meta(baseline, baseline_tag)
    g_up = clock["imu_static_mean_acc"] if meta_b.get("world_frame") == "first_imu" else [0.0, 0.0, 1.0]
    R_level = leveling_rotation(np.asarray(g_up, float))

    regs = [_Registrar(v, VARIANTS[v], threads, f_axis, wheel_sigma) for v in variants]
    kiss_odo, kiss_poses, kiss_sec = None, [], 0.0
    if kiss:
        from kiss_icp.config import KISSConfig
        from kiss_icp.kiss_icp import KissICP
        kc = KISSConfig()
        kc.data.max_range, kc.data.min_range, kc.data.deskew = 100.0, 2.0, True
        kc.mapping.voxel_size = 1.0
        kc.registration.max_num_threads = threads
        kiss_odo = KissICP(kc)

    gcfg = genz_config(BENCH_ROOT / "configs" / "algorithms" / "genz_lio.yaml")
    stamps, dets = [], []
    prev_Tb, prev_u = None, None
    imu_buf = np.empty((0, 7))
    speed_prev, prev_end = 0.0, None
    need_gravity = any(r.gravity for r in regs)
    n, t_wall = 0, time.perf_counter()
    for points, begin, end, imu, times, inten, rings, total in iter_frames(
            repo_path(cfg["bag"]), cfg["lidar_topic"], cfg["imu_topic"], gcfg.preprocess, start_offset_s, duration_s):
        Tb, ok = track.at(end)
        if not ok[0] or len(points) < 1000:
            continue
        Tb = Tb[0]
        rel = np.asarray(times, float) if times is not None else np.zeros(len(points))
        tp = begin + rel if rel.max() < 1.0 else rel          # per-point absolute time (sensor clock)
        rng = np.linalg.norm(points, axis=1)
        keep = (rng > 2.0) & (rng < 150.0)
        P = np.asarray(points, float)[keep]
        tp = tp[keep]
        inten_k = np.asarray(inten, float)[keep] if inten is not None else None

        # 1. deskew into the IMU frame at scan end, with the baseline's motion (20 time bins)
        p_imu = P @ T_IL[:3, :3].T + T_IL[:3, 3]
        edges = np.linspace(tp.min(), tp.max() + 1e-9, 21)
        b = np.clip(np.searchsorted(edges, tp, side="right") - 1, 0, 19)
        Tbins, okb = track.at(0.5 * (edges[:-1] + edges[1:]))
        if not okb.all():
            Tbins = np.repeat(Tb[None], 20, axis=0)
        Tend_inv = inv(Tb)
        M = Tend_inv[None] @ Tbins                            # (20, 4, 4): bin pose in end frame
        p_end = np.einsum("nij,nj->ni", M[b, :3, :3], p_imu) + M[b, :3, 3]

        # 2. pole candidates on a levelled, yaw-free copy
        if poles:
            Rwb = R_level @ Tb[:3, :3]
            yaw = np.arctan2(Rwb[1, 0], Rwb[0, 0])
            c, s = np.cos(-yaw), np.sin(-yaw)
            R_tilt = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ Rwb
            cand = poles_mod.detect(p_end @ R_tilt.T, inten_k)
            for row in cand:
                pb = R_tilt.T @ row[:3]                       # back to the IMU body frame
                dets.append([end, pb[0], pb[1], pb[2], *row[3:]])

        # gravity direction for the gravity-locked variants (IMU + baseline speed only)
        up_b = None
        if need_gravity and imu is not None and len(imu):
            imu_buf = np.vstack([imu_buf, np.asarray(imu, float)])
            imu_buf = imu_buf[imu_buf[:, 0] > imu_buf[-1, 0] - GRAVITY_WINDOW_S]
            if prev_Tb is not None and prev_end is not None and end > prev_end:
                dt_s = end - prev_end
                speed = float(np.linalg.norm((inv(prev_Tb) @ Tb)[:3, 3]) / dt_s)
                dvdt = (speed - speed_prev) / dt_s
                speed_prev = speed
            else:
                speed, dvdt = 0.0, 0.0
            up_b = gravity_up(imu_buf, speed, float(np.clip(dvdt, -5, 5)))

        # 3./4. ICP variants
        if regs:
            src, _ = sg.preprocess_points(p_end, downsampling_resolution=SRC_LEAF, num_neighbors=10,
                                          num_threads=threads)
            pred = np.eye(4) if prev_Tb is None else inv(prev_Tb) @ Tb
            u = float(sensor_to_unix(end, clock))
            pred_w, wheel_used = pred.copy(), False
            if prev_u is not None and t_can[0] + cal["lag_s"] <= u <= t_can[-1] + cal["lag_s"]:
                dist = float(D_can(u) - D_can(prev_u))
                t_b = pred[:3, 3]
                pred_w[:3, 3] = t_b - (t_b @ f_axis) * f_axis + dist * f_axis
                wheel_used = True
            for r in regs:
                start_pose = Tb if r.T is None else None
                p_use = pred_w if r.prior == "wheel" else pred
                r.step(src, end, start_pose if start_pose is not None else p_use, wheel_used, up_b)
            prev_u = u

        # 5. KISS-ICP (LiDAR frame, its own deskew with per-point times normalised to [0, 1])
        if kiss_odo is not None:
            t0 = time.perf_counter()
            span = max(tp.max() - tp.min(), 1e-6)
            kiss_odo.register_frame(P, (tp - tp.min()) / span)
            kiss_poses.append(np.asarray(kiss_odo.last_pose).copy())
            kiss_sec += time.perf_counter() - t0

        stamps.append(end)
        prev_Tb, prev_end = Tb, end
        n += 1
        if progress_every and n % progress_every == 0:
            wall = time.perf_counter() - t_wall
            print(f"[scanpass {baseline}] {n} scans, {wall:.0f} s wall, {n * 0.1 / wall:.2f}x real time, "
                  + ", ".join(f"{r.name} {1e3 * r.seconds / n:.0f} ms" for r in regs)
                  + (f", kiss {1e3 * kiss_sec / n:.0f} ms" if kiss_odo else ""), flush=True)

    stamps = np.asarray(stamps)
    frame = inherit_frame(baseline, baseline_tag)
    summary = {"baseline": baseline, "scans": n, "wall_s": time.perf_counter() - t_wall, "variants": {}}
    for r in regs:
        name = f"{baseline}+{r.name}"
        d = pd.DataFrame(r.diag)
        fr = dict(frame, world_frame="gravity") if r.gravity else frame
        out = write_run(name, tag, stamps, np.asarray(r.poses),
                        {"runner": "correct.scanpass", "role": "corrector", **fr, "variant": r.name,
                         "spec": VARIANTS[r.name], "ms_per_scan": 1e3 * r.seconds / max(n, 1),
                         "constants": {"SIGMA_PRIOR_T": SIGMA_PRIOR_T, "SIGMA_PRIOR_R_deg": np.degrees(SIGMA_PRIOR_R),
                                       "N_CORR": N_CORR, "wheel_sigma": wheel_sigma, "SRC_LEAF": SRC_LEAF,
                                       "MAP_LEAF": MAP_LEAF, "MAX_CORR": MAX_CORR}})
        d.to_csv(out / "diagnostics.csv", index=False)
        summary["variants"][r.name] = {"ms_per_scan": 1e3 * r.seconds / max(n, 1),
                                       "fallback_rate": float((d["fallback"] > 0).mean()) if len(d) else None}
    if kiss_odo is not None and kiss_poses:
        # KISS world = first LiDAR frame; express as IMU poses in the first IMU frame (R_IL = I here)
        K = np.asarray(kiss_poses)
        T_imu = T_IL[None] @ K @ inv(T_IL)[None]
        write_run("kiss_icp", tag, stamps, T_imu,
                  {"runner": "correct.scanpass", "role": "support", "world_frame": "first_imu",
                   "ms_per_scan": 1e3 * kiss_sec / max(n, 1), "note": "KISS-ICP 1.x, LiDAR only, 1.0 m voxel"})
        summary["kiss_icp_ms_per_scan"] = 1e3 * kiss_sec / max(n, 1)
    if poles:
        df = pd.DataFrame(dets, columns=poles_mod.DETECTION_COLUMNS)
        f = results_dir("poles") / (f"detections_{baseline}.csv" if tag == "full" else f"detections_{baseline}_{tag}.csv")
        df.to_csv(f, index=False)
        summary["pole_candidates"] = int(len(df))
    (results_dir("correct", baseline) / f"scanpass_{tag}.json").write_text(json.dumps(summary, indent=2, default=float))
    return summary
