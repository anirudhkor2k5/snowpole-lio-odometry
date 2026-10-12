#!/usr/bin/env python3
"""kiss_fusion_node.py for a LIO that publishes faster than the LiDAR (DLIO: 100 Hz IMU-rate poses).

Same node, same parameters; only the heading rule changes. kiss_fusion_node.py replaces the LIO's
heading change of each pose step with KISS-ICP's change since the previous matched scan. That is
right when every LIO pose has its own scan (Faster-LIO, FAST-LIO2, GenZ-LIO: 10 Hz), but at 100 Hz
nine of ten poses have no scan: they turn with the LIO's yaw, and the tenth then adds KISS-ICP's turn
over the whole 100 ms on top (the turn is counted twice), and the guard compares a 100 ms KISS turn
with a 10 ms LIO turn.

Here, between scans the heading follows the LIO; at each scan the heading is reset to
heading(previous scan) + KISS-ICP's turn since that scan, with the guard comparing that turn to the
LIO's accumulated turn over the same interval. With a scan at every pose this is the original rule.
Positions still use the heading at the start of each step, as in the original.
Each scan is used once: with 100 Hz poses several poses fall inside the match window of one scan, so
the config for this node (kiss_fusion_100hz.yaml) narrows match_tol_s to 5 ms (half the pose spacing),
and a scan already used is treated as missing.
"""
from __future__ import annotations

import time

import numpy as np
import rospy
import yaml

import kiss_fusion_node as N


class KissFusionAnyRate(N.KissFusion):
    def __init__(self, cfg: dict):
        super().__init__(cfg)
        self.stats["between_scans"] = 0

    def fuse(self, stamp, T, K):
        t0 = time.perf_counter()
        st = self.state
        if K is not None and st is not None and K is st["Kref"]:
            K = None                                     # this scan was already used by an earlier pose
        if st is None:
            up = self.up_meas(stamp, 0.0, 0.0)
            G = T.copy()
            if up is not None:
                G[:3, :3] = N.relevel(G[:3, :3], up)
            h = G[:3, :3] @ self.fx
            psi = float(np.arctan2(h[1], h[0]))
            self.state = {"stamp": stamp, "T": T, "G": G, "xy": G[:2, 3].copy(), "psi": psi, "speed": 0.0,
                          "Kref": K, "psi_ref": psi, "t_ref": stamp, "lio_acc": 0.0}
            self.publish(stamp)
            return
        dt = max(stamp - st["stamp"], 1e-3)
        P = np.linalg.inv(st["T"]) @ T
        G = st["G"] @ P
        speed = float(np.linalg.norm(P[:3, 3]) / dt)
        dvdt = float(np.clip((speed - st["speed"]) / dt, -5, 5))
        up = self.up_meas(stamp, speed, dvdt)
        if up is not None:
            G[:3, :3] = N.tilt_correct(G[:3, :3], up, min(1.0, dt / self.tau))
        G[:3, :3] = N.project_so3(G[:3, :3])
        D = np.linalg.inv(st["G"]) @ G
        d = D[:3, 3]
        fwd, lat = float(d @ self.fx), float(d @ self.ly)
        dpsi_lio = float(N.rotvec(D[:3, :3]) @ self.up_b)
        psi0 = st["psi"]
        psi = psi0 + dpsi_lio
        if K is None:
            st["lio_acc"] += dpsi_lio                    # between scans: follow the LIO
            self.stats["between_scans"] += 1
        else:
            if st["Kref"] is not None:
                acc = st["lio_acc"] + dpsi_lio
                dk = float(N.rotvec(st["Kref"][:3, :3].T @ K[:3, :3]) @ self.up_b)
                if abs(dk - acc) / max(stamp - st["t_ref"], 1e-3) <= self.guard_rate:
                    psi = st["psi_ref"] + dk
                else:
                    self.stats["fallback_guard"] += 1
            else:
                self.stats["fallback_missing"] += 1
            st.update(Kref=K, psi_ref=psi, t_ref=stamp, lio_acc=0.0)
        c, s = np.cos(psi0), np.sin(psi0)
        st["xy"] = st["xy"] + np.array([c * fwd - s * lat, s * fwd + c * lat])
        st.update(stamp=stamp, T=T, G=G, psi=psi, speed=speed)
        self.stats["fused"] += 1
        self.stats["fuse_us"].append(1e6 * (time.perf_counter() - t0))
        self.publish(stamp)


if __name__ == "__main__":
    import argparse
    rospy.init_node("kiss_fusion")
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True, help="yaml with the parameters read in KissFusion.__init__")
    a = ap.parse_args(rospy.myargv()[1:])
    with open(a.config) as f:
        KissFusionAnyRate(yaml.safe_load(f) or {})
    rospy.spin()
