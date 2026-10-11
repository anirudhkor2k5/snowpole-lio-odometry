"""GenZ-LIO through its Python bindings, straight from the bag (no ROS).

Faster than real time, so it is also the vehicle for the sliding-window study:
any [start, start + duration] slice can be run, cold (estimator assumes rest at
start-up, as it would after a restart) or warm (filter velocity seeded from a
baseline's velocity once the IMU has initialised; see setup/install_genz_py.sh).

The same pass can compute the per-scan context features (scan_features.py),
which do not depend on the estimator.

Outputs (results/runs/genz_lio/<tag>/), same layout as the ROS runner:
  odom_sensor_clock.tum  IMU body pose, gravity-aligned world, sensor clock
  diagnostics.csv        GenZ internals per scan (matches, scale, covariance, ...)
  meta.json
and, with scan_features=True, results/features/scan_context.csv.
"""
from __future__ import annotations

import json
import time
import warnings
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

from .io import RESULTS_ROOT, BENCH_ROOT, load_config, repo_path, results_dir, run_dir
from .scan_features import imu_features, scan_geometry


def _stamp(h) -> float:
    return float(h.stamp.sec) + float(h.stamp.nanosec) * 1e-9


def iter_frames(bag: Path, lidar_topic: str, imu_topic: str, preprocess, start_offset_s: float = 0.0,
                duration_s: Optional[float] = None):
    """Yield (points, begin, end, imu, times, intensities, rings, total_beams) for one time slice.

    Mirrors genz_lio.datasets.rosbag.RosbagDataset (same scan/IMU pairing via
    the library's own _ScanBuffer), plus record-time start/stop.
    """
    from genz_lio.datasets.pointcloud2 import read_pointcloud2
    from genz_lio.genz_lio_pybind import _ScanBuffer
    from rosbags.highlevel import AnyReader

    with AnyReader([Path(bag)]) as reader:
        conns = [c for c in reader.connections if c.topic in (lidar_topic, imu_topic)]
        start = reader.start_time + int(start_offset_s * 1e9)
        stop = start + int(duration_s * 1e9) if duration_s else None
        buf = _ScanBuffer(preprocess)
        beams = {}
        for c, _, raw in reader.messages(connections=conns, start=start, stop=stop):
            msg = reader.deserialize(raw, c.msgtype)
            if c.topic == imu_topic:
                a, g = msg.linear_acceleration, msg.angular_velocity
                event = buf.push_imu(np.array([[_stamp(msg.header), a.x, a.y, a.z, g.x, g.y, g.z]]))
            else:
                pts, ts, inten, rings = read_pointcloud2(msg, preprocess.lidar_type)
                if len(pts) == 0:
                    continue
                st = _stamp(msg.header)
                beams[round(st, 6)] = msg.width * msg.height
                event = buf.push_scan(pts, st, ts, inten, rings)
            if event in (1, 2):
                raise ValueError("sensor timestamps went backwards")
            while (b := buf.pop()) is not None:
                points, begin, end, imu, times, inten, rings, _ = b
                total = beams.pop(min(beams, key=lambda s: abs(s - begin)), len(points)) if beams else len(points)
                yield points, begin, end, imu, times, inten, rings, total


def run(tag: str = "full", start_offset_s: float = 0.0, duration_s: Optional[float] = None,
        config_path: Optional[Path] = None, warm_start_from: Optional[str] = None,
        scan_features: bool = False, max_threads: Optional[int] = None, progress: bool = True,
        time_scale: float = 1.0) -> Path:
    """`time_scale` multiplies the per-point times inside each scan (deskew probe, v3):
    1 = as recorded, 0 = no motion compensation, 0.5 = half compensation."""
    from genz_lio.config import load_config as genz_config
    from genz_lio.genz_lio_pybind import GenZLIO

    cfg = load_config()
    gcfg = genz_config(config_path or BENCH_ROOT / "configs" / "algorithms" / "genz_lio.yaml")
    if max_threads is not None:
        gcfg.max_threads = max_threads
    odo = GenZLIO(gcfg)

    seed_v = None
    if warm_start_from:
        seed_v = _baseline_body_velocity(warm_start_from, start_offset_s, cfg)

    out = run_dir("genz_lio", tag)
    out.mkdir(parents=True, exist_ok=True)
    tum = open(out / "odom_sensor_clock.tum", "w")
    tum.write("# timestamp x y z qx qy qz qw  (IMU body in gravity-aligned world, sensor clock)\n")
    diag, ctx = [], []
    seeded = False
    t_wall = time.perf_counter()
    n = 0
    bag = repo_path(cfg["bag"])
    for points, begin, end, imu, times, inten, rings, total in iter_frames(
            bag, cfg["lidar_topic"], cfg["imu_topic"], gcfg.preprocess, start_offset_s, duration_s):
        n += 1
        if scan_features:
            f = {"stamp": end}
            f.update(scan_geometry(points, total, blind=gcfg.preprocess.blind_min))
            f.update(imu_features(imu))
            ctx.append(f)
        if len(imu) == 0:
            continue
        if time_scale != 1.0 and times is not None:
            times = np.asarray(times, float) * time_scale
        res = odo.register_scan(points, begin, end, imu, timestamps=times, intensities=inten, rings=rings,
                                timing_prepared=True)
        row = {"stamp": res.timestamp if res.valid else end, "valid": bool(res.valid)}
        if res.valid:
            if seed_v is not None and not seeded:
                odo.seed_velocity_body(seed_v)
                seeded = True
            A = np.asarray(odo.gravity_alignment)
            T = np.asarray(res.pose)
            R, p = A @ T[:3, :3], A @ T[:3, 3]
            q = Rotation.from_matrix(R).as_quat()
            tum.write(f"{res.timestamp:.9f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")
            P = np.asarray(odo.covariance)
            bg, ba = odo.imu_biases
            v = np.asarray(res.velocity)
            row.update(
                plane_matches=res.plane_matches, point_matches=res.point_matches, scale_indicator=res.scale_indicator,
                leaf_size=res.leaf_size, setpoint=res.setpoint, scan_points=res.scan_points,
                voxelized_points=res.voxelized_points, median_range=res.median_range,
                update_rejected=bool(res.update_rejected), processing_time_ms=res.processing_time_ms,
                cov_pos_trace=float(np.trace(P[0:3, 0:3])), cov_rot_trace=float(np.trace(P[3:6, 3:6])),
                cov_vel_trace=float(np.trace(P[12:15, 12:15])) if P.shape[0] >= 15 else np.nan,
                speed=float(np.linalg.norm(v)), bg_norm=float(np.linalg.norm(bg)), ba_norm=float(np.linalg.norm(ba)),
                        bg_x=float(bg[0]), bg_y=float(bg[1]), bg_z=float(bg[2]),
                        ba_x=float(ba[0]), ba_y=float(ba[1]), ba_z=float(ba[2]))
        diag.append(row)
        if progress and n % 500 == 0:
            el = time.perf_counter() - t_wall
            print(f"[genz {tag}] {n} scans, {el:.0f} s wall, {n * 0.1 / el:.2f}x real time", flush=True)
    tum.close()
    wall = time.perf_counter() - t_wall
    pd.DataFrame(diag).to_csv(out / "diagnostics.csv", index=False)
    if scan_features:
        f = results_dir("features") / "scan_context.csv"
        pd.DataFrame(ctx).to_csv(f, index=False)
    meta = {"algo": "genz_lio", "tag": tag, "runner": "python", "world_frame": "gravity", "role": "support",
            "start_offset_s": start_offset_s, "duration_s": duration_s, "warm_start_from": warm_start_from,
            "seed_velocity_body": None if seed_v is None else list(map(float, seed_v)),
            "scans": n, "valid": int(sum(d["valid"] for d in diag)), "wall_s": wall,
            "real_time_factor": n * 0.1 / wall if wall > 0 else None,
            "config": str(config_path or BENCH_ROOT / "configs" / "algorithms" / "genz_lio.yaml"),
            "time_scale": time_scale}
    (out / "meta.json").write_text(json.dumps(meta, indent=2))
    for stale in ("trajectory_tum.txt",):
        (out / stale).unlink(missing_ok=True)
    return out


def _baseline_body_velocity(algo: str, start_offset_s: float, cfg: dict) -> np.ndarray:
    """Velocity of a baseline's full run, in its body (IMU) frame, at the window start.

    In deployment this is the running baseline's current state; offline it is read
    from that baseline's recorded trajectory. Both estimators use the same IMU body frame.
    """
    from rosbags.highlevel import AnyReader

    from .io import load_run

    tr = load_run(algo, "full", cfg)          # stamps already mapped to Unix time
    if tr is None:
        raise FileNotFoundError(f"warm start needs a full run of {algo} first")
    with AnyReader([repo_path(cfg["reduced_bag"])]) as r:
        t_unix = r.start_time * 1e-9 + start_offset_s
    # backward difference over the last 0.6 s: only poses the baseline has already produced
    i = int(np.clip(np.searchsorted(tr.t, t_unix, side="right") - 1, 6, len(tr) - 1))
    v_world = (tr.p[i] - tr.p[i - 6]) / (tr.t[i] - tr.t[i - 6])
    R = Rotation.from_quat(tr.q[i]).as_matrix()
    return R.T @ v_world
