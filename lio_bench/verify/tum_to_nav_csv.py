"""Turn a Faster-LIO / FAST-LIO trajectory (TUM text) into the pipeline's CSV, option B.

  python lio_bench/verify/tum_to_nav_csv.py Log/traj.txt incremental_navigation_mine.csv   (from the repo root)

Input: TUM lines "t x y z qx qy qz qw". Faster-LIO writes this to Log/traj.txt at shutdown when
path_save_en: true. Stamps may be the Ouster sensor clock (~13 000 s) or Unix; sensor stamps are mapped
to Unix with the clock fit of this bag (results/bag/sensor_clock.json, constants below).

GNSS used: only what the pipeline itself starts from, plus ONE heading reading at t0:
  - start heading = direction of the left->right GNSS antenna baseline (+90 deg) in the first row of
    gps_left/right_position.csv (car parked; compass 84.01 deg), minus the odometry's own yaw at its
    first pose. One rotation, applied to the whole trajectory. (Not /vehicle_enu_heading: its
    heading_init flag is never set in this bag and it reads ~5 deg low while driving.)
  - nothing else; the start position is irrelevant because the pipeline uses only row-to-row steps
    and starts from GNSS frame 0 itself.
Rows: one per row of incremental_navigation_results.csv (the pipeline pairs rows with frames by index),
positions interpolated at each row's GNSS stamp, held constant before the first / after the last pose.
"""
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from pyproj import Transformer

REPO = Path(__file__).resolve().parents[2]
TEMPLATE = REPO / "incremental_navigation_results.csv"
# Ouster sensor clock -> Unix for 2024-02-28-12-59-51.bag (fitted on the IMU stream, residual 0.14 ms)
STAMP0, OFFSET, DRIFT = 13214.2395428, 1709108377.3551764, 0.00010493499328967526
CLOCK = 0.0                             # no clock shift (option B: the 0.06 s offset was measured against GNSS)


def antenna_yaw0():
    """ENU yaw (rad, from east, CCW) from the first left/right antenna row."""
    to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32633", always_xy=True)
    L = pd.read_csv(REPO / "gps_left_position.csv").iloc[0]
    R = pd.read_csv(REPO / "gps_right_position.csv").iloc[0]
    le, ln = to_utm.transform(L["longitude"], L["latitude"])
    re, rn = to_utm.transform(R["longitude"], R["latitude"])
    return float(np.arctan2(rn - ln, re - le) + np.pi / 2)


def main(tum, out):
    a = np.loadtxt(tum, comments="#")
    t = a[:, 0]
    if t[0] < 1e9:                                       # sensor clock -> Unix
        t = t + OFFSET + DRIFT * (t - STAMP0)
    t = t + CLOCK
    yaw_odo0 = np.arctan2(2 * (a[0, 7] * a[0, 6] + a[0, 4] * a[0, 5]), 1 - 2 * (a[0, 5] ** 2 + a[0, 6] ** 2))
    rot = antenna_yaw0() - yaw_odo0
    c, s = np.cos(rot), np.sin(rot)
    p = (a[:, 1:3] - a[0, 1:3]) @ np.array([[c, -s], [s, c]]).T
    tm = pd.read_csv(TEMPLATE)
    ts = tm["header.stamp.secs"].to_numpy(float) + tm["header.stamp.nsecs"].to_numpy(float) * 1e-9
    e = tm["easting"].iloc[0] + np.interp(ts, t, p[:, 0])
    n = tm["northing"].iloc[0] + np.interp(ts, t, p[:, 1])
    lon, lat = Transformer.from_crs("EPSG:32633", "EPSG:4326", always_xy=True).transform(e, n)
    d = tm.copy()
    for col in ("translation_easting", "translation_northing", "rotation_angle_degrees",
                "transformed_easting", "transformed_northing", "error"):
        d[col] = np.nan                                  # FastReg's own processing columns: not ours
    d["easting"], d["northing"], d["latitude"], d["longitude"] = e, n, lat, lon
    d.to_csv(out, index=False)
    print(f"{len(a)} poses -> {len(d)} rows; start heading rotation {np.degrees(rot):+.2f} deg; wrote {out}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
