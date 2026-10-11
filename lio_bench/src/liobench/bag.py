"""Facts read once from the bag: sensor clock -> Unix mapping, start-up gravity,
and small auxiliary topics (INS heading, CAN speed/steering, INS fix state).

Everything is cached under results/bag/ so later steps never touch the 41 GB file.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd

from .io import repo_path, results_dir


def _stamp(h) -> float:
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def _reader(path: Path):
    from rosbags.highlevel import AnyReader
    return AnyReader([Path(path)])


# --------------------------------------------------------------------------- clock

def fit_sensor_clock(cfg: dict) -> dict:
    """unix = stamp + offset + drift * (stamp - stamp0), from the IMU stream.

    The Ouster stamps come from the sensor's own oscillator (~13 000 s at the
    start of this drive), not Unix time. The bag record time is Unix time plus
    transport latency, so the lower envelope of (record - stamp) per second is a
    clean estimate of the clock offset. Linear in time to absorb oscillator drift.
    """
    out = results_dir("bag") / "sensor_clock.json"
    if out.exists():
        return json.loads(out.read_text())
    st, rec, acc = [], [], []
    with _reader(repo_path(cfg["reduced_bag"])) as r:
        conns = [c for c in r.connections if c.topic == cfg["imu_topic"]]
        for c, ts, raw in r.messages(connections=conns):
            m = r.deserialize(raw, c.msgtype)
            st.append(_stamp(m.header))
            rec.append(ts * 1e-9)
            a = m.linear_acceleration
            acc.append([a.x, a.y, a.z])
    st, rec, acc = np.array(st), np.array(rec), np.array(acc)
    s0 = float(st[0])
    d = rec - st
    bins = ((st - s0) // 1.0).astype(int)
    env = pd.Series(d).groupby(bins).min()
    tc = pd.Series(st - s0).groupby(bins).mean()
    drift, offset = np.polyfit(tc.values, env.values, 1)
    resid = env.values - np.polyval([drift, offset], tc.values)

    # Start-up gravity: IMU mean over the initial still period (GT speed ~0).
    still = st < s0 + cfg.get("static_start_s", 1.0)
    g = acc[still].mean(0)
    clock = {
        "stamp0": s0, "offset_s": float(offset), "drift": float(drift), "drift_ppm": float(drift * 1e6),
        "envelope_residual_std_s": float(resid.std()),
        "imu_rate_hz": float(1.0 / np.median(np.diff(st))),
        "imu_static_mean_acc": g.tolist(), "imu_static_acc_norm": float(np.linalg.norm(g)),
        "imu_frame_tilt_deg": float(np.degrees(np.arccos(g[2] / np.linalg.norm(g)))),
    }
    out.write_text(json.dumps(clock, indent=2))
    return clock


def imu_mean_acc(cfg: dict, start_offset_s: float, duration_s: float = 1.0) -> list[float]:
    """Mean IMU specific force over [bag start + offset, + duration] (record time).

    Used to level estimators whose world frame is the IMU frame at *their* start,
    for runs that begin mid-drive (sliding windows). At constant speed this is
    gravity; under acceleration it carries that error, as the estimator does.
    """
    acc = []
    with _reader(repo_path(cfg["reduced_bag"])) as r:
        conns = [c for c in r.connections if c.topic == cfg["imu_topic"]]
        t0 = r.start_time + int(start_offset_s * 1e9)
        for c, _, raw in r.messages(connections=conns, start=t0, stop=t0 + int(duration_s * 1e9)):
            a = r.deserialize(raw, c.msgtype).linear_acceleration
            acc.append([a.x, a.y, a.z])
    return np.mean(acc, axis=0).tolist()


def sensor_to_unix(stamp, clock: dict):
    stamp = np.asarray(stamp, float)
    return stamp + clock["offset_s"] + clock["drift"] * (stamp - clock["stamp0"])


# --------------------------------------------------------------------------- auxiliary topics

def extract_aux(cfg: dict, force: bool = False) -> dict[str, Path]:
    """INS heading, CAN speed/steering and INS state to CSV (record-time clock)."""
    outdir = results_dir("bag")
    files = {k: outdir / f"{k}.csv" for k in ("enu_heading", "steering", "utm_position", "gps_state")}
    if not force and all(f.exists() for f in files.values()):
        return files
    topics = {"/vehicle_enu_heading": "enu_heading", "/vehicle/steering_report": "steering",
              "/utm_position": "utm_position", "/gps_state": "gps_state"}
    rows: dict[str, list] = {v: [] for v in topics.values()}
    with _reader(repo_path(cfg["bag"])) as r:
        conns = [c for c in r.connections if c.topic in topics]
        for c, ts, raw in r.messages(connections=conns):
            m = r.deserialize(raw, c.msgtype)
            key = topics[c.topic]
            t = ts * 1e-9
            if key == "enu_heading":
                rows[key].append((t, m.data))
            elif key == "steering":
                rows[key].append((t, _stamp(m.header), m.speed, m.steering_wheel_angle))
            elif key == "utm_position":
                rows[key].append((t, _stamp(m.header), m.utm_easting, m.utm_northing, m.utm_height, m.utm_zone_number))
            elif key == "gps_state":
                f = m.filter_status
                rows[key].append((t, _stamp(m.header), f.GNSSFixStatus, int(f.DualAntennaHeadingActive),
                                  int(f.NavigationFilterInitialized), int(f.HeadingInitialized),
                                  m.sat_fix_std_dev.latitude, m.sat_fix_std_dev.longitude))
    cols = {
        "enu_heading": ["t_record", "heading_rad"],
        "steering": ["t_record", "t_stamp", "speed_mps", "steering_wheel_angle_rad"],
        "utm_position": ["t_record", "t_stamp", "easting", "northing", "height", "zone"],
        "gps_state": ["t_record", "t_stamp", "gnss_fix_status", "dual_antenna_heading", "nav_filter_init",
                      "heading_init", "lat_std", "lon_std"],
    }
    for key, f in files.items():
        pd.DataFrame(rows[key], columns=cols[key]).to_csv(f, index=False)
    return files


def load_aux(name: str) -> pd.DataFrame | None:
    f = results_dir("bag") / f"{name}.csv"
    return pd.read_csv(f) if f.exists() else None
