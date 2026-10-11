"""Where does LiDAR-only odometry's constant yaw drift come from? KISS-ICP under controlled changes.

KISS-ICP (no IMU) drifts ~+0.02 deg/s counter-clockwise on this drive, at a constant rate per
second. Each probe changes one thing in its input and the resulting yaw-rate bias is compared:
  nodeskew   per-point timestamps ignored (no motion compensation)
  revtime    per-point timestamps reversed (deskew applied in the wrong direction)
  vox05      0.5 m map voxels instead of 1.0 m
  shift5ms   per-point timestamps delayed by 5 ms of a 100 ms scan (timing offset)
A bias that flips with 'revtime' and vanishes with 'nodeskew' would point at deskew/timing;
one that is unchanged points at registration / scene geometry.
  noground   points > 1 m below the sensor removed (ground)
  minr10     points closer than 10 m removed
  upperbeams only rings 0-63 (the upper half; ring 0 is the top beam, rings >= 64 hold the ground)
"""
from __future__ import annotations

import time

import numpy as np

from ..io import BENCH_ROOT, load_config, repo_path
from .common import inv, lidar_in_imu, write_run

PROBES = {
    "default": {},                        # re-run of the default config, to time it the same way
    "nodeskew": {"deskew": False},
    "revtime": {"time": "reverse"},
    "vox05": {"voxel": 0.5},
    "shift5ms": {"time": "shift", "shift": 0.05},
    # geometry probes (which points carry the pitch / heading bias?)
    "noground": {"z_min": -1.0},          # drop points more than 1 m below the sensor (ground ~ -1.67 m)
    "minr10": {"r_min": 10.0},            # drop points closer than 10 m
    "upperbeams": {"rings_max": 64},      # upper half of the beams only (ring 0 = top; rings >= 64 see the ground)
    # config sweep (cost vs heading quality; KISS-ICP options only, no source change)
    "vox15": {"voxel": 1.5},
    "vox20": {"voxel": 2.0},
    "range50": {"max_range": 50.0},
    "mppv5": {"max_points_per_voxel": 5},
}


def run(probe: str, cfg: dict | None = None, threads: int = 4, duration_s: float | None = None) -> dict:
    from genz_lio.config import load_config as genz_config
    from kiss_icp.config import KISSConfig
    from kiss_icp.kiss_icp import KissICP

    from ..genz_runner import iter_frames
    cfg = cfg or load_config()
    spec = PROBES[probe]
    kc = KISSConfig()
    kc.data.max_range, kc.data.min_range = spec.get("max_range", 100.0), 2.0
    kc.data.deskew = spec.get("deskew", True)
    kc.mapping.voxel_size = spec.get("voxel", 1.0)
    kc.mapping.max_points_per_voxel = spec.get("max_points_per_voxel", 20)
    kc.registration.max_num_threads = threads
    odo = KissICP(kc)
    gcfg = genz_config(BENCH_ROOT / "configs" / "algorithms" / "genz_lio.yaml")
    stamps, poses = [], []
    kiss_s = 0.0
    t0 = time.perf_counter()
    for points, begin, end, imu, times, inten, rings, total in iter_frames(
            repo_path(cfg["bag"]), cfg["lidar_topic"], cfg["imu_topic"], gcfg.preprocess, 0.0, duration_s):
        P = np.asarray(points, float)
        r = np.linalg.norm(P, axis=1)
        keep = (r > spec.get("r_min", 2.0)) & (r < spec.get("max_range", 100.0))
        if "z_min" in spec:
            keep &= P[:, 2] > spec["z_min"]
        if "rings_max" in spec and rings is not None:
            keep &= np.asarray(rings) < spec["rings_max"]
        P = P[keep]
        if len(P) < 1000:
            continue
        rel = np.asarray(times, float)[keep] if times is not None else np.zeros(len(P))
        span = max(rel.max() - rel.min(), 1e-6)
        tn = (rel - rel.min()) / span
        if spec.get("time") == "reverse":
            tn = 1.0 - tn
        elif spec.get("time") == "shift":
            tn = np.clip(tn + spec["shift"], 0.0, 1.0)
        tk0 = time.perf_counter()
        odo.register_frame(P, tn)
        kiss_s += time.perf_counter() - tk0
        poses.append(np.asarray(odo.last_pose).copy())
        stamps.append(end)
    T_IL = lidar_in_imu(cfg)
    K = np.asarray(poses)
    T_imu = T_IL[None] @ K @ inv(T_IL)[None]
    name = f"kiss_icp_{probe}"
    write_run(name, "full", np.asarray(stamps), T_imu,
              {"runner": "correct.kiss_probe", "role": "probe", "world_frame": "first_imu", "probe": spec,
               "wall_s": time.perf_counter() - t0, "ms_per_scan": 1e3 * kiss_s / max(len(stamps), 1)})
    return {"probe": probe, "scans": len(stamps), "wall_s": time.perf_counter() - t0}
