#!/usr/bin/env python3
"""Run one or more ROS LiDAR-inertial estimators on the bag, headless.

    run_ros.py --algos fast_lio                      # one baseline, whole drive
    run_ros.py --algos fast_lio,dlio --tag concurrent # baseline + support at once
    run_ros.py --algos dlio --start 120 --duration 40 --tag win_120_40

Each run gets its own roscore on a free port (so several runs can share a
machine), plays only the LiDAR and IMU topics, records every estimator's
odometry and receive times, samples CPU / RSS of each estimator process, and
waits for the estimators to drain before shutting down. Outputs land in
lio_bench/results/runs/<algo>/<tag>/. Converting sensor-clock stamps to Unix
time is done later by the Python side (liobench.io.load_run).

Must run inside the lio-ros env with the workspace sourced; use
lio_bench/scripts/run_ros.sh, which does that.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import psutil
import yaml

BENCH = Path(__file__).resolve().parents[2]
REPO = BENCH.parent


def log(msg: str) -> None:
    print(f"[run_ros {time.strftime('%H:%M:%S')}] {msg}", flush=True)


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def dataset_config() -> dict:
    cfg = yaml.safe_load(open(os.environ.get("LIOBENCH_CONFIG", BENCH / "configs" / "dataset.yaml")))
    root = Path(os.environ.get("LIOBENCH_DATA_ROOT", cfg.get("data_root", ".")))
    cfg["_root"] = root if root.is_absolute() else (REPO / root).resolve()
    return cfg


def wait_for_master(env: dict, timeout: float = 30) -> None:
    import rosgraph
    master = rosgraph.Master("/lio_bench", master_uri=env["ROS_MASTER_URI"])
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            master.getPid()
            return
        except Exception:
            time.sleep(0.3)
    raise RuntimeError("roscore did not come up")


def subscriber_count(env: dict, topic: str) -> int:
    import rosgraph
    master = rosgraph.Master("/lio_bench", master_uri=env["ROS_MASTER_URI"])
    _, subs, _ = master.getSystemState()
    for t, nodes in subs:
        if t == topic:
            return len(nodes)
    return 0


class ResourceMonitor(threading.Thread):
    """Samples CPU% and RSS of each estimator process (found by binary name)."""

    def __init__(self, launchers: dict[str, subprocess.Popen], binaries: dict[str, str], path: Path, period=0.5):
        super().__init__(daemon=True)
        self.launchers, self.binaries, self.path, self.period = launchers, binaries, path, period
        self.stop_flag = threading.Event()
        self.procs: dict[str, psutil.Process] = {}

    def _find(self):
        for algo, launcher in self.launchers.items():
            if algo in self.procs:
                continue
            try:
                me = psutil.Process(launcher.pid)
                if self.binaries[algo] in " ".join(me.cmdline()):      # script nodes run as the launcher itself
                    self.procs[algo] = me
                    me.cpu_percent(None)
                    continue
                for child in me.children(recursive=True):
                    if self.binaries[algo] in " ".join(child.cmdline()):
                        self.procs[algo] = child
                        child.cpu_percent(None)
            except psutil.Error:
                pass

    def run(self):
        with open(self.path, "w") as f:
            f.write("wall,algo,cpu_pct,rss_mb,threads\n")
            while not self.stop_flag.is_set():
                self._find()
                for algo, p in list(self.procs.items()):
                    try:
                        with p.oneshot():
                            f.write(f"{time.time():.3f},{algo},{p.cpu_percent(None):.1f},"
                                    f"{p.memory_info().rss / 2**20:.1f},{p.num_threads()}\n")
                    except psutil.Error:
                        self.procs.pop(algo, None)
                f.flush()
                self.stop_flag.wait(self.period)

    def stop(self):
        self.stop_flag.set()


def terminate(p: subprocess.Popen | None, name: str, timeout: float = 20) -> None:
    if p is None or p.poll() is not None:
        return
    p.send_signal(signal.SIGINT)
    try:
        p.wait(timeout)
    except subprocess.TimeoutExpired:
        log(f"{name} ignored SIGINT; terminating")
        p.terminate()
        try:
            p.wait(5)
        except subprocess.TimeoutExpired:
            p.kill()


REPOS = {"fast_lio": "FAST_LIO", "faster_lio": "faster-lio", "dlio": "direct_lidar_inertial_odometry",
         "genz_lio": "GenZ-LIO"}


def git_commit(path: Path) -> str | None:
    try:
        return subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()
    except Exception:
        return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--algos", required=True, help="comma list from configs/algorithms/registry.yaml")
    ap.add_argument("--tag", default=None, help="output sub-directory (default: full, or concurrent_<algos>)")
    ap.add_argument("--start", type=float, default=0.0, help="seconds from bag start")
    ap.add_argument("--duration", type=float, default=None, help="seconds to play (default: to the end)")
    ap.add_argument("--rate", type=float, default=1.0, help="bag playback rate (1.0 = real time)")
    ap.add_argument("--config-override", action="append", default=[],
                    help="algo=path.yaml to use instead of the registry config")
    ap.add_argument("--drain-timeout", type=float, default=300.0)
    ap.add_argument("--no-latency", action="store_true", help="do not subscribe to the raw cloud for latency")
    args = ap.parse_args()

    registry = yaml.safe_load(open(BENCH / "configs" / "algorithms" / "registry.yaml"))
    algos = [a.strip() for a in args.algos.split(",") if a.strip()]
    for a in algos:
        if a not in registry:
            sys.exit(f"unknown algorithm {a!r}; known: {', '.join(registry)}")
    overrides = dict(o.split("=", 1) for o in args.config_override)
    tag = args.tag or ("full" if len(algos) == 1 else "concurrent_" + "+".join(algos))
    cfg = dataset_config()
    bag = cfg["_root"] / cfg["bag"]
    if not bag.exists():
        sys.exit(f"bag not found: {bag} (set data_root in configs/dataset.yaml or LIOBENCH_DATA_ROOT)")

    work = BENCH / "results" / "runs" / "_work" / f"{tag}_{os.getpid()}"
    work.mkdir(parents=True, exist_ok=True)
    port = free_port()
    env = dict(os.environ, ROS_MASTER_URI=f"http://127.0.0.1:{port}", ROS_HOSTNAME="127.0.0.1",
               ROS_LOG_DIR=str(work / "ros_logs"), ROS_HOME=str(work / "ros_home"), ROSCONSOLE_STDOUT_LINE_BUFFERED="1")
    log(f"tag={tag} algos={algos} port={port} bag={bag.name} start={args.start} dur={args.duration} rate={args.rate}")

    procs: dict[str, subprocess.Popen] = {}
    logs = []

    def spawn(name: str, cmd: list[str]) -> subprocess.Popen:
        f = open(work / f"{name}.log", "w")
        logs.append(f)
        p = subprocess.Popen(cmd, env=env, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        procs[name] = p
        return p

    monitor = None
    t_wall = {}
    try:
        spawn("roscore", ["roscore", "-p", str(port)])
        wait_for_master(env)

        launchers = {}
        for a in algos:
            conf = Path(overrides.get(a, BENCH / "configs" / "algorithms" / registry[a]["config"])).resolve()
            if "script" in registry[a]:
                # plain rospy node outside any catkin package (e.g. the kiss_fusion post-processor)
                script = BENCH / "ros" / "scripts" / registry[a]["script"]
                remaps = [f"__name:={a}", f"~points:={cfg['lidar_topic']}", f"~imu:={cfg['imu_topic']}"]
                if registry[a].get("lio_input"):
                    remaps.append(f"~lio:=/{registry[a]['lio_input']}/Odometry")
                launchers[a] = spawn(f"{a}", [sys.executable, str(script), "--config", str(conf), *remaps])
            else:
                launch = BENCH / "ros" / "launch" / registry[a]["launch"]
                launchers[a] = spawn(f"{a}", ["roslaunch", "--wait", str(launch), f"ns:={a}", f"config:={conf}"])

        topics = ",".join(f"{a}=/{a}/Odometry" for a in algos)
        rec_cmd = [sys.executable, str(BENCH / "ros" / "scripts" / "odom_recorder.py"),
                   "--topics", topics, "--out", str(work / "rec"),
                   "--cloud-topic", "" if args.no_latency else cfg["lidar_topic"]]
        spawn("recorder", rec_cmd)

        need = len(algos) + (0 if args.no_latency else 1)
        t0 = time.time()
        while subscriber_count(env, cfg["lidar_topic"]) < need:
            for name, p in procs.items():
                if p.poll() is not None:
                    raise RuntimeError(f"{name} exited early (code {p.returncode}); see {work / (name + '.log')}")
            if time.time() - t0 > 90:
                raise RuntimeError(f"estimators did not subscribe to {cfg['lidar_topic']} within 90 s")
            time.sleep(0.5)
        time.sleep(2.0)

        monitor = ResourceMonitor(launchers, {a: registry[a]["node_binary"] for a in algos}, work / "resources.csv")
        monitor.start()

        play = ["rosbag", "play", "--quiet", str(bag), "--topics", cfg["lidar_topic"], cfg["imu_topic"],
                "-r", str(args.rate), "-s", str(args.start)]
        if args.duration:
            play += ["-u", str(args.duration)]
        t_wall["play_start"] = time.time()
        player = spawn("rosbag_play", play)
        while player.poll() is None:
            for a in algos:
                if launchers[a].poll() is not None:
                    raise RuntimeError(f"{a} died during playback; see {work / (a + '.log')}")
            time.sleep(1.0)
        t_wall["play_end"] = time.time()
        log(f"playback finished in {t_wall['play_end'] - t_wall['play_start']:.0f} s; draining")

        # Drain: wait until no estimator's latest pose stamp has advanced for 5 s.
        status = work / "rec" / "recorder_status.json"
        last, still_since = None, time.time()
        while time.time() - t_wall["play_end"] < args.drain_timeout:
            time.sleep(1.0)
            cur = json.loads(status.read_text())["last_stamp"] if status.exists() else None
            if cur != last:
                last, still_since = cur, time.time()
            elif time.time() - still_since > 5:
                break
        t_wall["drained"] = time.time()
        counts = json.loads(status.read_text())["odom"] if status.exists() else {}
        log(f"odometry poses recorded: {counts}")
    finally:
        if monitor:
            monitor.stop()
        for name in ["recorder"] + algos + ["rosbag_play", "roscore"]:
            terminate(procs.get(name), name)
        for f in logs:
            f.close()

    # File the outputs per algorithm.
    for a in algos:
        out = BENCH / "results" / "runs" / a / tag
        if out.exists():
            shutil.rmtree(out)
        out.mkdir(parents=True)
        src = work / "rec" / a
        for f in ("odom_sensor_clock.tum", "odom_recv.csv"):
            if (src / f).exists():
                shutil.copy(src / f, out / f)
        if (work / "rec" / "cloud_recv.csv").exists():
            shutil.copy(work / "rec" / "cloud_recv.csv", out / "cloud_recv.csv")
        if (work / "resources.csv").exists():
            shutil.copy(work / "resources.csv", out / "resources.csv")
        shutil.copy(work / f"{a}.log", out / "node.log")
        meta = {
            "algo": a, "tag": tag, "runner": "ros", "concurrent_with": [b for b in algos if b != a],
            "world_frame": registry[a]["world_frame"], "role": registry[a]["role"],
            "config": str(Path(overrides.get(a, BENCH / "configs" / "algorithms" / registry[a]["config"]))),
            "bag": str(bag), "start_offset_s": args.start, "duration_s": args.duration, "rate": args.rate,
            "wall": t_wall, "machine": {"cpus": os.cpu_count(), "arch": os.uname().machine,
                                        "mem_gb": round(psutil.virtual_memory().total / 2**30, 1)},
            "estimator_commit": (git_commit(BENCH / "third_party" / "ros_ws" / "src" / REPOS[a]) if a in REPOS
                                 else None),           # script nodes (e.g. kiss_fusion) live in this repo
        }
        (out / "meta.json").write_text(json.dumps(meta, indent=2))
        n = sum(1 for _ in open(out / "odom_sensor_clock.tum")) - 1 if (out / "odom_sensor_clock.tum").exists() else 0
        log(f"{a}: {n} poses -> {out}")
    shutil.rmtree(work, ignore_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
