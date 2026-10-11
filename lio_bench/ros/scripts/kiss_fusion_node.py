#!/usr/bin/env python3
"""Online post-processing of a black-box LIO: gravity-filtered attitude + heading from KISS-ICP.

Subscribes
  ~points (sensor_msgs/PointCloud2)   raw Ouster scans          -> KISS-ICP (LiDAR only)
  ~imu    (sensor_msgs/Imu)           IMU                       -> gravity direction
  ~lio    (nav_msgs/Odometry)         the LIO's odometry        -> translation + short-term rotation
Publishes
  ~Odometry (nav_msgs/Odometry)       corrected planar pose, stamped like the LIO pose it came from

Per LIO pose k (all causal):
  1. step  P = T_lio_{k-1}^-1 T_lio_k;  T_G <- T_G P;  roll/pitch of T_G pulled toward the IMU's
     gravity direction (complementary filter, time constant tau)        [REPORT_v2 §5, v3 §2]
  2. tilt-free step [fwd, lat] of T_G in its level body axes, heading change about 'up'
  3. heading change replaced by KISS-ICP's for the same scan (fallback: the LIO's, when KISS-ICP
     is missing or its yaw rate disagrees by more than guard_rate)       [REPORT_v3 §4]
  4. x <- x + Rot(psi_{k-1}) [fwd, lat];  psi <- psi + dpsi
Self-contained (numpy + kiss_icp + rospy), so it can be dropped into the vehicle's ROS graph.
"""
from __future__ import annotations

import json
import threading
import time
from collections import deque

import numpy as np
import rospy
import yaml
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2


# --------------------------------------------------------------------------- SO(3) helpers (no scipy)

def quat_to_R(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        return (R[2, 1] - R[1, 2]) * s, (R[0, 2] - R[2, 0]) * s, (R[1, 0] - R[0, 1]) * s, 0.25 / s
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def rotvec(R):
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    a = np.arccos(c)
    if a < 1e-9:
        return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / 2.0
    return a / (2.0 * np.sin(a)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def exp_so3(v):
    a = np.linalg.norm(v)
    if a < 1e-12:
        return np.eye(3)
    k = v / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


def project_so3(R):
    U, _, Vt = np.linalg.svd(R)
    return U @ Vt


def tilt_correct(R, up_b, alpha):
    """Pull the body's current 'up' (R^T z) a fraction alpha toward the measured up_b."""
    u = R.T @ np.array([0.0, 0.0, 1.0])
    ax = np.cross(u, up_b)
    sn = np.linalg.norm(ax)
    if sn < 1e-12:
        return R
    ang = np.arctan2(sn, float(u @ up_b))
    return R @ exp_so3(-ax / sn * alpha * ang)


def relevel(R, up_b):
    z = np.array([0.0, 0.0, 1.0])
    ax = np.cross(up_b, z)
    sn, c = np.linalg.norm(ax), float(up_b @ z)
    R0 = np.eye(3) if sn < 1e-12 else exp_so3(ax / sn * np.arctan2(sn, c))
    fx = np.array([1.0, 0.0, 0.0]) - up_b[0] * up_b
    h, h0 = R @ fx, R0 @ fx
    return exp_so3(np.array([0, 0, np.arctan2(h[1], h[0]) - np.arctan2(h0[1], h0[0])])) @ R0


# --------------------------------------------------------------------------- node

class KissFusion:
    def __init__(self, cfg: dict):
        self.tau = float(cfg.get("gravity_tau_s", 5.0))
        self.window = float(cfg.get("gravity_window_s", 1.0))
        self.up_b = np.asarray(cfg.get("body_up", [0.0, 0.0, 1.0]), float)
        self.up_b /= np.linalg.norm(self.up_b)
        self.t_il = np.asarray(cfg.get("lidar_in_imu_t", [0.0, 0.0, 0.0]), float)
        self.guard_rate = float(cfg.get("guard_rate_rad_s", 0.2))
        self.match_tol = float(cfg.get("match_tol_s", 0.02))
        fx = np.array([1.0, 0.0, 0.0]) - self.up_b[0] * self.up_b
        self.fx = fx / np.linalg.norm(fx)
        self.ly = np.cross(self.up_b, self.fx)

        from kiss_icp.config import KISSConfig
        from kiss_icp.kiss_icp import KissICP
        kc = KISSConfig()
        kc.data.max_range = float(cfg.get("kiss_max_range", 100.0))
        kc.data.min_range = float(cfg.get("kiss_min_range", 2.0))
        kc.data.deskew = True
        kc.mapping.voxel_size = float(cfg.get("kiss_voxel", 1.0))
        kc.mapping.max_points_per_voxel = int(cfg.get("kiss_max_points_per_voxel", 20))
        kc.registration.max_num_threads = int(cfg.get("kiss_threads", 2))
        self.kiss = KissICP(kc)

        self.lock = threading.Lock()
        self.imu = deque(maxlen=400)                  # (t, ax, ay, az, gx, gy, gz)
        self.kiss_poses = {}                          # scan-end stamp -> 4x4 IMU pose in KISS world
        self.lio = deque()                            # (stamp, 4x4) waiting for their KISS scan
        self.state = None
        self.stats = {"scans": 0, "kiss_ms": [], "fused": 0, "fallback_missing": 0, "fallback_guard": 0,
                      "fuse_us": [], "dropped_lio": 0}
        self.pub = rospy.Publisher("~Odometry", Odometry, queue_size=50)
        rospy.Subscriber("~imu", Imu, self.on_imu, queue_size=2000)
        rospy.Subscriber("~lio", Odometry, self.on_lio, queue_size=200)
        rospy.Subscriber("~points", PointCloud2, self.on_points, queue_size=2, buff_size=2 ** 26)
        rospy.on_shutdown(self.report)

    # -- inputs
    def on_imu(self, m):
        a, g = m.linear_acceleration, m.angular_velocity
        with self.lock:
            self.imu.append((m.header.stamp.to_sec(), a.x, a.y, a.z, g.x, g.y, g.z))

    def on_lio(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        T = np.eye(4)
        T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
        T[:3, 3] = (p.x, p.y, p.z)
        with self.lock:
            self.lio.append((m.header.stamp.to_sec(), T))
        self.try_fuse()

    def on_points(self, msg):
        t0 = time.perf_counter()
        names = {f.name: f.offset for f in msg.fields}
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
        xyz = np.stack([raw[:, names[c]:names[c] + 4].copy().view(np.float32)[:, 0] for c in "xyz"], 1).astype(float)
        tt = raw[:, names["t"]:names["t"] + 4].copy().view(np.uint32)[:, 0].astype(float) * 1e-9 if "t" in names else np.zeros(len(xyz))
        r = np.linalg.norm(xyz, axis=1)
        keep = (r > 2.0) & np.isfinite(r)
        xyz, tt = xyz[keep], tt[keep]
        if len(xyz) < 1000:
            return
        span = max(tt.max() - tt.min(), 1e-6)
        self.kiss.register_frame(xyz, (tt - tt.min()) / span)
        K = np.asarray(self.kiss.last_pose)
        T_IL = np.eye(4)
        T_IL[:3, 3] = self.t_il
        T_imu = T_IL @ K @ np.linalg.inv(T_IL)
        stamp = msg.header.stamp.to_sec() + tt.max()
        with self.lock:
            self.kiss_poses[round(stamp, 4)] = T_imu
            if len(self.kiss_poses) > 200:
                for k in sorted(self.kiss_poses)[:-200]:
                    self.kiss_poses.pop(k)
            self.stats["scans"] += 1
            self.stats["kiss_ms"].append(1e3 * (time.perf_counter() - t0))
        self.try_fuse()

    # -- fusion
    def kiss_at(self, stamp):
        if not self.kiss_poses:
            return None, None
        keys = np.fromiter(self.kiss_poses.keys(), float)
        i = int(np.argmin(np.abs(keys - stamp)))
        return (self.kiss_poses[keys[i]], keys[i]) if abs(keys[i] - stamp) <= self.match_tol else (None, None)

    def up_meas(self, stamp, speed, dvdt):
        s = [x for x in self.imu if stamp - self.window < x[0] <= stamp]
        if len(s) < 10:
            return None
        s = np.asarray(s)
        f, w = s[:, 1:4].mean(0), s[:, 4:7].mean(0)
        a = dvdt * np.array([1.0, 0, 0]) + np.cross(w, speed * np.array([1.0, 0, 0]))
        u = f - a
        n = np.linalg.norm(u)
        return u / n if n > 5.0 else None

    def try_fuse(self):
        with self.lock:
            while self.lio:
                stamp, T = self.lio[0]
                K, kstamp = self.kiss_at(stamp)
                newest_kiss = max(self.kiss_poses) if self.kiss_poses else -np.inf
                if K is None and newest_kiss < stamp + self.match_tol:
                    return                               # KISS-ICP for this scan not done yet: wait
                self.lio.popleft()
                self.fuse(stamp, T, K)

    def fuse(self, stamp, T, K):
        t0 = time.perf_counter()
        st = self.state
        if st is None:
            up = self.up_meas(stamp, 0.0, 0.0)
            G = T.copy()
            if up is not None:
                G[:3, :3] = relevel(G[:3, :3], up)
            h = G[:3, :3] @ self.fx
            self.state = {"stamp": stamp, "T": T, "G": G, "K": K, "xy": G[:2, 3].copy(),
                          "psi": float(np.arctan2(h[1], h[0])), "speed": 0.0}
            self.publish(stamp)
            return
        dt = max(stamp - st["stamp"], 1e-3)
        P = np.linalg.inv(st["T"]) @ T
        G = st["G"] @ P
        speed = float(np.linalg.norm(P[:3, 3]) / dt)
        dvdt = float(np.clip((speed - st["speed"]) / dt, -5, 5))
        up = self.up_meas(stamp, speed, dvdt)
        if up is not None:
            G[:3, :3] = tilt_correct(G[:3, :3], up, min(1.0, dt / self.tau))
        G[:3, :3] = project_so3(G[:3, :3])
        D = np.linalg.inv(st["G"]) @ G
        d = D[:3, 3]
        fwd, lat = float(d @ self.fx), float(d @ self.ly)
        dpsi_lio = float(rotvec(D[:3, :3]) @ self.up_b)
        dpsi = dpsi_lio
        if K is not None and st["K"] is not None:
            dpsi_kiss = float(rotvec(st["K"][:3, :3].T @ K[:3, :3]) @ self.up_b)
            if abs(dpsi_kiss - dpsi_lio) / dt <= self.guard_rate:
                dpsi = dpsi_kiss
            else:
                self.stats["fallback_guard"] += 1
        else:
            self.stats["fallback_missing"] += 1
        psi0 = st["psi"]
        c, s = np.cos(psi0), np.sin(psi0)
        st["xy"] = st["xy"] + np.array([c * fwd - s * lat, s * fwd + c * lat])
        st.update(stamp=stamp, T=T, G=G, K=K if K is not None else st["K"], psi=psi0 + dpsi, speed=speed)
        self.stats["fused"] += 1
        self.stats["fuse_us"].append(1e6 * (time.perf_counter() - t0))
        self.publish(stamp)

    def publish(self, stamp):
        st = self.state
        m = Odometry()
        m.header.stamp = rospy.Time.from_sec(stamp)
        m.header.frame_id = "corrected_world"
        m.pose.pose.position.x, m.pose.pose.position.y = float(st["xy"][0]), float(st["xy"][1])
        m.pose.pose.position.z = float(st["G"][2, 3])
        qx, qy, qz, qw = R_to_quat(exp_so3(np.array([0, 0, st["psi"]])))
        m.pose.pose.orientation.x, m.pose.pose.orientation.y = qx, qy
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = qz, qw
        self.pub.publish(m)

    def report(self):
        s = self.stats
        k = np.asarray(s["kiss_ms"]) if s["kiss_ms"] else np.zeros(1)
        f = np.asarray(s["fuse_us"]) if s["fuse_us"] else np.zeros(1)
        out = {"scans": s["scans"], "fused": s["fused"], "kiss_ms_median": float(np.median(k)),
               "kiss_ms_p95": float(np.percentile(k, 95)), "fuse_us_median": float(np.median(f)),
               "fallback_missing": s["fallback_missing"], "fallback_guard": s["fallback_guard"]}
        print("KISS_FUSION_STATS " + json.dumps(out), flush=True)


if __name__ == "__main__":
    import argparse
    rospy.init_node("kiss_fusion")
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True, help="yaml with the parameters read in KissFusion.__init__")
    a = ap.parse_args(rospy.myargv()[1:])
    with open(a.config) as f:
        KissFusion(yaml.safe_load(f) or {})
    rospy.spin()
