"""KISS-ICP fed exactly like kiss_fusion_node.on_points, read from the bag offline (no real time).

  python lio_bench/verify/kiss_input_test.py <threads> <tag> [--clip100]   (from the repo root)

Used to find why the online node's heading differs from the offline KISS-ICP pass
(results/optionB_all/SCORES.md, "Why offline and online differ"). Writes results/runs/kiss_icp_nodeinput/<tag>.
--clip100 also drops points beyond 100 m before KISS-ICP (as the offline pass does).
"""
import sys, time
from pathlib import Path
import numpy as np
sys.path.insert(0, "lio_bench/src")
from rosbags.highlevel import AnyReader
from kiss_icp.config import KISSConfig
from kiss_icp.kiss_icp import KissICP
from liobench.io import load_config, repo_path
from liobench.correct.kiss_probe import write_run
threads = int(sys.argv[1]); tag = sys.argv[2]; clip = "--clip100" in sys.argv
cfg = load_config()
kc = KISSConfig(); kc.data.max_range = 100.0; kc.data.min_range = 2.0; kc.data.deskew = True
kc.mapping.voxel_size = 2.0; kc.mapping.max_points_per_voxel = 20; kc.registration.max_num_threads = threads
kiss = KissICP(kc)
t_il = np.array([-0.006253, 0.011775, -0.007645]); T_IL = np.eye(4); T_IL[:3, 3] = t_il
stamps, poses = [], []; t0 = time.perf_counter()
with AnyReader([repo_path(cfg["bag"])]) as r:
    conns = [c for c in r.connections if c.topic == cfg["lidar_topic"]]
    for c, _, raw in r.messages(connections=conns):
        msg = r.deserialize(raw, c.msgtype)
        names = {f.name: f.offset for f in msg.fields}
        b = np.asarray(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
        xyz = np.stack([b[:, names[k]:names[k] + 4].copy().view(np.float32)[:, 0] for k in "xyz"], 1).astype(float)
        tt = b[:, names["t"]:names["t"] + 4].copy().view(np.uint32)[:, 0].astype(float) * 1e-9
        rr = np.linalg.norm(xyz, axis=1); keep = (rr > 2.0) & np.isfinite(rr) & ((rr < 100.0) if clip else True)
        xyz, tt = xyz[keep], tt[keep]
        if len(xyz) < 1000: continue
        span = max(tt.max() - tt.min(), 1e-6)
        kiss.register_frame(xyz, (tt - tt.min()) / span)
        poses.append(T_IL @ np.asarray(kiss.last_pose) @ np.linalg.inv(T_IL))
        stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 + tt.max())
write_run("kiss_icp_nodeinput", tag, np.asarray(stamps), np.asarray(poses),
          {"runner": "verify/kiss_input_test.py", "role": "probe", "world_frame": "first_imu",
           "note": f"node's input path, offline, {threads} threads, 2.0 m", "wall_s": time.perf_counter() - t0})
print(len(stamps), "scans", time.perf_counter() - t0, "s")
