#!/usr/bin/env python3
"""Record nav_msgs/Odometry streams to TUM files, plus receive wall-times.

Per odometry topic, writes <out>/<name>/odom_sensor_clock.tum (stamps as the
estimator publishes them: the Ouster sensor clock) and odom_recv.csv
(stamp, wall time received). The point-cloud topic is subscribed as raw bytes
(rospy.AnyMsg, nothing deserialised) and only its header stamp is read, giving
cloud_recv.csv; latency per scan = odom wall - cloud wall for the same scan.

A status file (<out>/recorder_status.json) is rewritten every second so the
orchestrator can tell when the estimators have drained their queues.
"""
import argparse
import json
import os
import struct
import threading
import time

import rospy
from nav_msgs.msg import Odometry


class TopicWriter:
    def __init__(self, directory):
        os.makedirs(directory, exist_ok=True)
        self.tum = open(os.path.join(directory, "odom_sensor_clock.tum"), "w")
        self.tum.write("# timestamp x y z qx qy qz qw  (estimator stamps, sensor clock)\n")
        self.recv = open(os.path.join(directory, "odom_recv.csv"), "w")
        self.recv.write("stamp,wall\n")
        self.count = 0
        self.last_stamp = -1.0
        self.lock = threading.Lock()

    def __call__(self, msg):
        wall = time.time()
        s = msg.header.stamp.to_sec()
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        with self.lock:
            # Some estimators (DLIO) re-publish the latest state from a timer; keep only
            # poses whose stamp advances, and drop DLIO's zero-stamped pre-initialisation pose.
            if s <= self.last_stamp or s <= 0.0:
                return
            self.last_stamp = s
            self.tum.write(f"{s:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n")
            self.recv.write(f"{s:.9f},{wall:.6f}\n")
            self.count += 1

    def close(self):
        with self.lock:
            self.tum.close()
            self.recv.close()


class CloudStampWriter:
    """Header stamp of a serialized message: uint32 seq, uint32 secs, uint32 nsecs."""

    def __init__(self, path):
        self.f = open(path, "w")
        self.f.write("stamp,wall\n")
        self.count = 0
        self.lock = threading.Lock()

    def __call__(self, raw):
        wall = time.time()
        _, secs, nsecs = struct.unpack_from("<III", raw._buff, 0)
        with self.lock:
            self.f.write(f"{secs + nsecs * 1e-9:.9f},{wall:.6f}\n")
            self.count += 1

    def close(self):
        with self.lock:
            self.f.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topics", required=True, help="comma list name=/topic, e.g. fast_lio=/fast_lio/Odometry")
    ap.add_argument("--cloud-topic", default="/ouster/points")
    ap.add_argument("--out", required=True)
    args = ap.parse_args(rospy.myargv()[1:])

    rospy.init_node("lio_bench_recorder", anonymous=True, disable_signals=False)
    writers = {}
    subs = []
    for item in args.topics.split(","):
        name, topic = item.split("=", 1)
        writers[name] = TopicWriter(os.path.join(args.out, name))
        subs.append(rospy.Subscriber(topic, Odometry, writers[name], queue_size=1000, tcp_nodelay=True))
    cloud = None
    if args.cloud_topic:
        cloud = CloudStampWriter(os.path.join(args.out, "cloud_recv.csv"))
        subs.append(rospy.Subscriber(args.cloud_topic, rospy.AnyMsg, cloud, queue_size=50, tcp_nodelay=True))

    status_path = os.path.join(args.out, "recorder_status.json")

    def write_status():
        st = {"wall": time.time(), "odom": {k: w.count for k, w in writers.items()},
              "last_stamp": {k: w.last_stamp for k, w in writers.items()},
              "clouds": cloud.count if cloud else None}
        tmp = status_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(st, f)
        os.replace(tmp, status_path)

    rate = rospy.Rate(1.0)
    try:
        while not rospy.is_shutdown():
            write_status()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        for s in subs:
            s.unregister()
        for w in writers.values():
            w.close()
        if cloud:
            cloud.close()
        write_status()


if __name__ == "__main__":
    main()
