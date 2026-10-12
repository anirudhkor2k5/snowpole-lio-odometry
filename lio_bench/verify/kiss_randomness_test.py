"""Is the offline KISS-ICP pass really better than the node's, or did it land well by chance?

  python lio_bench/verify/kiss_randomness_test.py run <offline|node> <seed>     one KISS-ICP pass
  python lio_bench/verify/kiss_randomness_test.py score                         table of every pass

Each pass reads the whole bag offline (no real time) with KISS-ICP at 2.0 m, exactly as one of the two
readers does, and drops a random 0.1 % of the points of every scan (a different random set per seed).
That change is physically meaningless, so the spread over seeds shows how much the 10 km result moves
by chance:
  offline  the offline pass (correct/kiss_probe.py, vox20): GenZ-LIO scan reader, 2 < r < 100 m, 4 threads
  node     kiss_fusion_node.on_points: raw PointCloud2 decoding, r > 2 m (KISS-ICP clips at 100 m), 2 threads
Writes results/runs/kiss_icp_rand_<reader>/s<seed>; score writes results/optionB_all/kiss_randomness.json.
"""
import json
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "lio_bench" / "src"))
sys.path.insert(0, str(REPO / "lio_bench" / "pipeline"))
DROP = 0.001


def kiss(threads):
    from kiss_icp.config import KISSConfig
    from kiss_icp.kiss_icp import KissICP
    kc = KISSConfig()
    kc.data.max_range, kc.data.min_range, kc.data.deskew = 100.0, 2.0, True
    kc.mapping.voxel_size, kc.mapping.max_points_per_voxel = 2.0, 20
    kc.registration.max_num_threads = threads
    return KissICP(kc)


def scans_offline(cfg):
    from genz_lio.config import load_config as genz_config
    from liobench.correct.kiss_probe import BENCH_ROOT
    from liobench.genz_runner import iter_frames
    from liobench.io import repo_path
    g = genz_config(BENCH_ROOT / "configs" / "algorithms" / "genz_lio.yaml")
    for points, begin, end, imu, times, inten, rings, total in iter_frames(
            repo_path(cfg["bag"]), cfg["lidar_topic"], cfg["imu_topic"], g.preprocess, 0.0, None):
        P = np.asarray(points, float)
        r = np.linalg.norm(P, axis=1)
        keep = (r > 2.0) & (r < 100.0)
        tt = np.asarray(times, float)[keep] if times is not None else np.zeros(keep.sum())
        yield P[keep], tt, end


def scans_node(cfg):
    from rosbags.highlevel import AnyReader
    from liobench.io import repo_path
    with AnyReader([repo_path(cfg["bag"])]) as rd:
        conns = [c for c in rd.connections if c.topic == cfg["lidar_topic"]]
        for c, _, raw in rd.messages(connections=conns):
            msg = rd.deserialize(raw, c.msgtype)
            names = {f.name: f.offset for f in msg.fields}
            b = np.asarray(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
            xyz = np.stack([b[:, names[k]:names[k] + 4].copy().view(np.float32)[:, 0] for k in "xyz"], 1).astype(float)
            tt = b[:, names["t"]:names["t"] + 4].copy().view(np.uint32)[:, 0].astype(float) * 1e-9
            r = np.linalg.norm(xyz, axis=1)
            keep = (r > 2.0) & np.isfinite(r)
            yield xyz[keep], tt[keep], msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 + tt[keep].max()


def run(reader, seed):
    from liobench.correct.kiss_probe import lidar_in_imu, write_run
    from liobench.io import load_config
    cfg = load_config()
    rng = np.random.default_rng(seed)
    odo = kiss(4 if reader == "offline" else 2)
    src = scans_offline(cfg) if reader == "offline" else scans_node(cfg)
    stamps, poses, t0 = [], [], time.perf_counter()
    for P, tt, stamp in src:
        m = rng.random(len(P)) >= DROP
        P, tt = P[m], tt[m]
        if len(P) < 1000:
            continue
        span = max(tt.max() - tt.min(), 1e-6)
        odo.register_frame(P, (tt - tt.min()) / span)
        poses.append(np.asarray(odo.last_pose).copy())
        stamps.append(stamp)
    if reader == "offline":
        T_IL = lidar_in_imu(cfg)
    else:
        T_IL = np.eye(4)
        T_IL[:3, 3] = [-0.006253, 0.011775, -0.007645]
    T = T_IL[None] @ np.asarray(poses) @ np.linalg.inv(T_IL)[None]
    write_run(f"kiss_icp_rand_{reader}", f"s{seed}", np.asarray(stamps), T,
              {"runner": "verify/kiss_randomness_test.py", "role": "probe", "world_frame": "first_imu",
               "reader": reader, "seed": seed, "drop_fraction": DROP, "wall_s": time.perf_counter() - t0})
    print(f"{reader} seed {seed}: {len(stamps)} scans, {time.perf_counter() - t0:.0f} s", flush=True)


def score():
    import score_option_b as S
    from liobench.geometry import cumulative_distance
    from liobench.io import load_config, load_ground_truth, results_dir
    cfg = load_config()
    gt = load_ground_truth(cfg)
    dist = cumulative_distance(gt.p)
    yaw0 = S.E.antenna_heading_enu()
    out = {}
    for reader in ("offline", "node"):
        for d in sorted((results_dir("runs") / f"kiss_icp_rand_{reader}").glob("s*")):
            S.X.v3.SOURCES["kiss_icp"] = (f"kiss_icp_rand_{reader}", d.name)
            row = {}
            for key, (src, tag, pipe) in {"kiss_alone": (f"kiss_icp_rand_{reader}", d.name, "raw"),
                                          "faster_lio_gk": ("faster_lio", "rerun_b", "gravity+kiss_heading")}.items():
                tr, _ = S.X.trajectory(cfg, src, tag, pipe)
                r, _c = S.score(tr, yaw0, gt, dist)
                row[key] = {k: r[k] for k in ("final_m", "ate_rmse_m", "kitti_trans_pct", "kitti_rot_deg_per_100m",
                                              "heading_err_end_deg", "err_at_km0.72", "err_at_km5.19")}
            out[f"{reader}/{d.name}"] = row
            print(f"{reader:8s} {d.name}: KISS alone end {row['kiss_alone']['final_m']:5.0f} m, "
                  f"Faster-LIO+G+KISS end {row['faster_lio_gk']['final_m']:5.0f} m, "
                  f"KITTI {row['faster_lio_gk']['kitti_trans_pct']:.2f} %, heading at end "
                  f"{row['faster_lio_gk']['heading_err_end_deg']:+.1f} deg", flush=True)
    (results_dir("optionB_all") / "kiss_randomness.json").write_text(json.dumps(out, indent=2))


if __name__ == "__main__":
    if sys.argv[1] == "run":
        run(sys.argv[2], int(sys.argv[3]))
    else:
        score()
