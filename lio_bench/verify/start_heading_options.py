"""How much GNSS does the pipeline input really need? Compares ways of giving raw Faster-LIO a start heading.

  python lio_bench/verify/start_heading_options.py [friend_faster_lio_full.csv]      (from the repo root)

Raw odometry lives in its own start frame: it knows how far and which way it turned, but not where
north is. The pipeline already takes its start POSITION from GNSS (frame 0). The options here differ
only in where the start HEADING comes from:
  A. 200 m fit       rotation fitted to GNSS over the first 200 m (what our export does)
  B. one reading     the INS heading at the first frame only, a single number, as a compass or the
                     dual-antenna heading at standstill would give; no GNSS after t0
  C. none            friend's CSV as is (start heading as placed by their tooling), and the pipeline's
                     own pole-based heading reset ("reset" mode), which uses only poles and no GNSS
"""
import contextlib
import io
import sys
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd
from pyproj import Transformer

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "lio_bench" / "pipeline"))
import snowpole_stage3 as S  # noqa: E402

TUM = REPO / "lio_bench" / "results" / "runs" / "faster_lio" / "full" / "trajectory_tum.txt"
OURS = REPO / "lio_bench" / "results" / "export" / "incremental_navigation_faster_lio_full_raw.csv"
TEMPLATE = REPO / "incremental_navigation_results.csv"
CLOCK = 0.06                                         # estimator clock offset (training block), as in the export


def one_reading_csv(path):
    """Raw poses rotated by the INS heading at the first frame; anchored at the first GNSS row."""
    a = np.loadtxt(TUM, comments="#")
    t, xy = a[:, 0] + CLOCK, a[:, 1:3]
    h = pd.read_csv(REPO / "lio_bench" / "results" / "bag" / "enu_heading.csv")
    yaw0 = float(h["heading_rad"].iloc[0])           # ENU yaw of the vehicle (x axis) at the first record
    yaw_odo0 = 2 * np.arctan2(a[0, 6], a[0, 7])      # odometry's own yaw at its first pose (qz, qw)
    c, s = np.cos(yaw0 - yaw_odo0), np.sin(yaw0 - yaw_odo0)
    p = (xy - xy[0]) @ np.array([[c, -s], [s, c]]).T
    tm = pd.read_csv(TEMPLATE)
    ts = tm["header.stamp.secs"].to_numpy(float) + tm["header.stamp.nsecs"].to_numpy(float) * 1e-9
    e0, n0 = tm["easting"].iloc[0], tm["northing"].iloc[0]   # any anchor: the pipeline starts from GNSS frame 0
    e, n = e0 + np.interp(ts, t, p[:, 0]), n0 + np.interp(ts, t, p[:, 1])
    lon, lat = Transformer.from_crs("EPSG:32633", "EPSG:4326", always_xy=True).transform(e, n)
    d = tm.copy()
    d["easting"], d["northing"], d["latitude"], d["longitude"] = e, n, lat, lon
    d.to_csv(path, index=False)
    return np.degrees(yaw0 - yaw_odo0)


def run(csv, mode, tag):
    d = pd.read_csv(csv)
    if len(d) < 5422:
        d = pd.concat([d, d.iloc[[-1] * (5422 - len(d))]])
    p = Path(S.OUT) / f"{tag}_in.csv"
    d.to_csv(p, index=False)
    with contextlib.redirect_stdout(io.StringIO()):
        r = S.run(str(p), mode, tag)
    nb, ps = r["native_box_metric"], r["pole_span"]
    return f"in-box mean {nb['mean']:6.1f} m, median {nb['median']:5.1f} m | pole span mean {ps['mean']:6.1f} m, max {ps['max']:6.1f} m"


def main():
    friend = sys.argv[1] if len(sys.argv) > 1 else None
    with tempfile.TemporaryDirectory() as tmp:
        S.OUT = Path(tmp)
        rot = one_reading_csv(Path(tmp) / "one.csv")
        rows = [("A. ours, 200 m GNSS fit", OURS, "none"),
                ("A. ours, 200 m GNSS fit + pole heading reset", OURS, "reset"),
                (f"B. ours, one INS heading at t0 ({rot:+.1f} deg)", Path(tmp) / "one.csv", "none"),
                ("B. ours, one INS heading at t0 + pole heading reset", Path(tmp) / "one.csv", "reset")]
        if friend:
            rows += [("C. friend's CSV as is", friend, "none"),
                     ("C. friend's CSV as is + pole heading reset", friend, "reset")]
        for k, (name, csv, mode) in enumerate(rows):
            print(f"{name:52s} {run(csv, mode, f'r{k}')}", flush=True)


if __name__ == "__main__":
    main()
