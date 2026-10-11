"""Snow-pole localisation pipeline with stage-3 heading correction (new file; the original is untouched).

Re-implements the main loop of `snowpole_based_vehicle_localization.py` step for step, calling the
original `geoloc_utils` functions, and adds two optional stage-3 steps that use the pipeline's own
pole fixes:
  heading reset  : between pole fixes >= MIN_BASELINE_M apart, the direction of the fix-to-fix
                   vector vs the odometry's own displacement over the same stretch gives the heading
                   offset of the odometry; it is applied to the dead-reckoning heading
  drift rate     : the heading offset's drift rate (deg/s) is fitted over the resets so far and
                   extrapolated while no pole is seen (e.g. after the last pole)

Two phases, so the slow part runs once:
  detect  streams the bag (one frame in memory at a time instead of ~5.7 GB), runs the repo's YOLO
          model on every signal image exactly as the original does, and caches each detection's
          geometry (bbox, nearest range pixel, xyz). Detections do not depend on the odometry.
  run     the localisation loop for one incremental-navigation CSV and one heading mode, from the
          cache. Seconds per variant.

Metrics (vs the dual-GNSS midpoint kriged to the range-image stamps, as in the original):
  native      the original's metric: frames whose predicted position lies inside the pole map's
              bounding box (identical computation)
  whole       every frame of the 10.3 km drive
  pole_span   frames between where the reference track passes the first and the last surveyed pole

Usage (repo root, lio-py env):
  python lio_bench/pipeline/snowpole_stage3.py detect
  python lio_bench/pipeline/snowpole_stage3.py run --nav <csv> --heading {none,reset,reset+rate} --label NAME
"""
from __future__ import annotations

import argparse
import json
import os
import pickle
import sys
import time
import types
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[2]
OUT = REPO / "lio_bench" / "results" / "pipeline"
BAG = REPO / "2024-02-28-12-59-51_no_unwanted_topics.bag"
POLES = REPO / "Groundtruth_pole_location_at_test_site_E39_Hemnekjølen.csv"
MODEL = REPO / "model" / "pole_best_signal.pt"
CACHE = OUT / "detections_cache.pkl"

MIN_BASELINE_M = 15.0        # pole-fix pairs closer than this are too noisy for a heading estimate
MAX_HEADING_STEP_DEG = 15.0  # larger single measured offsets are treated as mis-associations
GAIN = 0.5                   # fraction of each measured heading offset applied
RATE_MIN_UPDATES = 5         # drift rate is used once this many offset updates span >= RATE_MIN_SPAN_S
RATE_MIN_SPAN_S = 60.0


def _import_geoloc_utils():
    """Import the repo's geoloc_utils unchanged. Its module-level imports include packages the
    functions used here never touch (ROS cv_bridge / sensor_msgs, bagpy, open3d); those are
    stubbed so the module imports on a machine without ROS."""
    for name in ("cv_bridge", "bagpy", "open3d"):
        if name not in sys.modules:
            try:
                __import__(name)
            except Exception:
                m = types.ModuleType(name)
                m.CvBridgeError = Exception
                m.bagreader = None
                sys.modules[name] = m
    for name in ("sensor_msgs", "sensor_msgs.point_cloud2"):
        if name not in sys.modules:
            try:
                __import__(name)
            except Exception:
                sys.modules[name] = types.ModuleType(name)
    sys.modules["sensor_msgs"].point_cloud2 = sys.modules["sensor_msgs.point_cloud2"]
    sys.path.insert(0, str(REPO))
    import geoloc_utils as gu
    return gu


# --------------------------------------------------------------------------- phase 1: detections

def _load_model():
    os.environ["YOLOv5_AUTOINSTALL"] = "False"
    import torch
    _load = torch.load
    torch.load = lambda *a, **k: _load(*a, **{**k, "weights_only": False})  # trusted repo checkpoint
    model = torch.hub.load("ultralytics/yolov5:v7.0", "custom", path=str(MODEL), trust_repo=True, verbose=False)
    torch.load = _load
    model.conf = 0.6                       # as load_custom_model(..., confidence_threshold=0.6)
    return model


def detect(max_frames: int | None = None) -> Path:
    from rosbags.highlevel import AnyReader
    from ouster.sdk import core as client       # SDK 1.0 renamed ouster.client -> ouster.sdk.core
    gu = _import_geoloc_utils()
    OUT.mkdir(parents=True, exist_ok=True)
    with open(REPO / "Trip068.json") as f:
        xyzlut = client.XYZLut(client.SensorInfo(f.read()))
    model = _load_model()
    topics = {"/ouster/signal_image", "/ouster/range_image", "/gps_left_position", "/gps_right_position"}
    sig, rng = {}, {}
    gl, gr, tl, trr, ts_sig, ts_rng = [], [], [], [], [], []
    frames = {}
    t0 = time.time()
    with AnyReader([BAG]) as r:
        con = [c for c in r.connections if c.topic in topics]
        for c, t, raw in r.messages(connections=con):
            topic = c.topic
            if topic in ("/gps_left_position", "/gps_right_position"):
                m = r.deserialize(raw, c.msgtype)
                (gl if topic == "/gps_left_position" else gr).append((m.latitude, m.longitude))
                (tl if topic == "/gps_left_position" else trr).append(t * 1e-9)
                continue
            m = r.deserialize(raw, c.msgtype)
            img = np.frombuffer(m.data, dtype=np.uint16).reshape((m.height, m.width)).copy()
            if topic == "/ouster/signal_image":
                sig[len(ts_sig)] = img
                ts_sig.append(t * 1e-9)
            else:
                rng[len(ts_rng)] = img
                ts_rng.append(t * 1e-9)
            # process frame i as soon as both its signal and range image are in (original pairs by index)
            for i in sorted(set(sig) & set(rng)):
                if i > 0:
                    frames[i] = _detect_frame(i, sig[i], rng[i], model, xyzlut, gu)
                sig.pop(i)
                rng.pop(i)
                if i % 100 == 0:
                    import psutil
                    print(f"[detect] frame {i}, {sum(len(v) for v in frames.values())} detections, "
                          f"{time.time() - t0:.0f} s, rss {psutil.Process().memory_info().rss / 2**20:.0f} MB, "
                          f"buffered sig/rng {len(sig)}/{len(rng)}", flush=True)
            if max_frames and len(ts_rng) > max_frames:
                break
    cache = {"frames": frames, "gnss_left": gl, "gnss_right": gr, "t_left": tl, "t_right": trr,
             "t_signal": ts_sig, "t_range": ts_rng, "model": str(MODEL), "conf": 0.6}
    with open(CACHE, "wb") as f:
        pickle.dump(cache, f)
    print(f"[detect] done: {len(ts_rng)} range images, {sum(len(v) for v in frames.values())} detections, "
          f"{time.time() - t0:.0f} s -> {CACHE}")
    return CACHE


def _detect_frame(i, signal, range_image, model, xyzlut, gu):
    """Exactly the original per-frame steps up to the per-bbox geometry (lines 201-230)."""
    xyz = xyzlut(range_image.astype(np.uint32)) * 4      # SDK 1.0 needs uint32; values unchanged
    rgb = np.stack((signal, signal, signal), axis=-1)
    rgb = ((rgb - rgb.min()) / (rgb.max() - rgb.min()) * 255).astype(np.uint8)
    results = model(rgb)
    out = []
    for bbox in results.xyxy[0]:
        x_min, y_min, x_max, y_max = map(int, bbox[:4])
        gx, gy, nd = gu.extract_nearest_point_in_bounding_box_region(x_min, x_max, y_min, y_max, rgb, {i: range_image}, i)
        if gx is None or gy is None or nd is None:
            out.append({"bbox": (x_min, y_min, x_max, y_max), "valid": False})
            continue
        out.append({"bbox": (x_min, y_min, x_max, y_max), "valid": True, "global_x": int(gx), "global_y": int(gy),
                    "nearest_distance": float(nd), "center_xyz": np.array(xyz[int(gy), int(gx), :], dtype=float).copy(),
                    "conf": float(bbox[4])})
    return out


# --------------------------------------------------------------------------- phase 2: localisation

class HeadingCorrector:
    """Stage-3 heading offset from the pipeline's own pole fixes (see module doc)."""

    def __init__(self, mode: str):
        self.mode = mode
        self.theta = 0.0                     # deg, added to the odometry's heading
        self.anchor = None                   # (easting, northing) of the fix the stretch starts at
        self.odo = None                      # odometry-only displacement since the anchor
        self.updates = []                    # (t, theta) after each accepted update
        self.t_last = None
        self.rate = 0.0
        self.log = []

    def heading(self, h_odo: float, t: float) -> float:
        if self.mode == "none" or not np.isfinite(h_odo):
            return h_odo
        th = self.theta
        if self.mode == "reset+rate" and self.t_last is not None:
            th = self.theta + self.rate * (t - self.t_last)
        return (h_odo + th) % 360.0

    def step(self, de: float, dn: float):
        """Odometry displacement of this frame (with the corrected heading)."""
        if self.odo is not None:
            self.odo += (de, dn)

    def fix(self, e: float, n: float, t: float, frame: int):
        if self.mode == "none":
            return
        if self.anchor is None:
            self.anchor, self.odo = np.array([e, n]), np.zeros(2)
            return
        v_fix = np.array([e, n]) - self.anchor
        if np.hypot(*v_fix) < MIN_BASELINE_M:
            return                          # keep the anchor until the stretch is long enough
        v_odo = self.odo
        if np.hypot(*v_odo) > 1.0:
            az = lambda v: np.degrees(np.arctan2(v[0], v[1]))     # azimuth: clockwise from north
            d = (az(v_fix) - az(v_odo) + 180.0) % 360.0 - 180.0
            ok = abs(d) <= MAX_HEADING_STEP_DEG
            if ok:
                if self.mode == "reset+rate" and self.t_last is not None:
                    self.theta += self.rate * (t - self.t_last)   # bring the prediction up to now first
                self.theta += GAIN * d
                self.t_last = t
                self.updates.append((t, self.theta))
                if self.mode == "reset+rate" and len(self.updates) >= RATE_MIN_UPDATES:
                    T = np.array(self.updates)
                    if T[-1, 0] - T[0, 0] >= RATE_MIN_SPAN_S:
                        self.rate = float(np.polyfit(T[:, 0] - T[0, 0], T[:, 1], 1)[0])
            self.log.append({"frame": frame, "t": t, "measured_deg": float(d), "accepted": bool(ok),
                             "theta_deg": self.theta, "rate_deg_s": self.rate, "baseline_m": float(np.hypot(*v_fix))})
        self.anchor, self.odo = np.array([e, n]), np.zeros(2)


def run(nav_csv: str, heading_mode: str, label: str) -> dict:
    gu = _import_geoloc_utils()
    from pyproj import CRS, Geod, Transformer
    with open(CACHE, "rb") as f:
        C = pickle.load(f)
    frames = C["frames"]
    n_frames = len(C["t_range"])
    data = pd.read_csv(nav_csv)
    fe, fn = data["easting"].values, data["northing"].values
    utm33n, wgs84 = CRS("EPSG:32633"), CRS("EPSG:4326")
    to_wgs, to_utm = Transformer.from_crs(utm33n, wgs84), Transformer.from_crs(wgs84, utm33n)
    lats, lons = to_wgs.transform(fe, fn)
    geod = Geod(ellps="WGS84")
    headings, translations = [np.nan], [np.nan]                     # lines 50-66
    for i in range(1, len(lats)):
        if np.isnan(lats[i]) or np.isnan(lats[i - 1]):
            headings.append(np.nan)
            translations.append(np.nan)
        else:
            fwd, _, dist = geod.inv(lons[i - 1], lats[i - 1], lons[i], lats[i])
            headings.append((fwd + 360) % 360)
            translations.append(dist)
    proj_latlon, proj_utm33 = "EPSG:4326", "EPSG:32633"
    gl, gr = C["gnss_left"], C["gnss_right"]
    vlat = [(gl[i + 1][0] + gr[i + 1][0]) / 2 for i in range(n_frames)]   # lines 109-118
    vlon = [(gl[i + 1][1] + gr[i + 1][1]) / 2 for i in range(n_frames)]
    gnss_offset, lidar_offset = np.array([-0.32, 0.0, 1.24]), np.array([0.7, 0.0, 1.8])
    g2l = gnss_offset - lidar_offset
    pl, plo = gu.kriging_interpolation(C["t_range"], C["t_left"][1:5421], vlat, vlon)   # line 129
    ref_e, ref_n = gu.transform_coordinates(proj_latlon, proj_utm33, plo, pl)    # (lon, lat), as line 130
    ref_e, ref_n = np.asarray(ref_e, float), np.asarray(ref_n, float)
    (dataRef_northings, dataRef_eastings, min_north, max_north, min_east, max_east,
     *_rest) = gu.load_gnss_data_and_calculate_utm_range(str(POLES))
    poles = list(zip(dataRef_eastings, dataRef_northings))

    hc = HeadingCorrector(heading_mode)
    pe, pn = [], []            # predicted position appended BEFORE any reset (original lines 189-190)
    post_e = np.full(n_frames, np.nan)
    post_n = np.full(n_frames, np.nan)   # position after this frame's resets (what line 325 measures)
    err_native, in_box = [], np.zeros(n_frames, bool)
    fixes = []
    t_r = np.asarray(C["t_range"])
    for i in range(n_frames):                                       # lines 171-341
        if i == 0:
            e, n = gu.transform_coordinates(proj_latlon, proj_utm33, plo[i], pl[i])
        else:
            h = hc.heading(headings[i], t_r[i])
            e0, n0 = e, n
            e, n = gu.calculate_new_positions(e, n, h, translations[i])
            if np.isfinite(e) and np.isfinite(e0):
                hc.step(e - e0, n - n0)
        pe.append(e)
        pn.append(n)
        if not (min_north <= n <= max_north and min_east <= e <= max_east):
            post_e[i], post_n[i] = e, n
            continue
        in_box[i] = True
        if i > 0:
            for det in frames.get(i, []):
                if not det["valid"]:
                    continue
                cxyz = det["center_xyz"]
                new_distance = gu.calculate_distance(cxyz, g2l)
                if new_distance > 5:
                    continue
                e1, n1 = pe[i - 1], pn[i - 1]
                heading = gu.calculate_vehicle_heading_from_two_utm(e1, n1, e, n)
                gu.calculate_vehicle_heading_direction_utm(e1, n1, heading)
                az_gnss, _ = gu.calculate_azimuth_elevation_from_gnss(cxyz[0], cxyz[1], cxyz[2], g2l)
                az_gnss = -az_gnss
                plat, plon = to_wgs.transform(e, n)
                te, tn, adj = gu.local_to_utm33_with_offset_proj(plat, plon, heading, az_gnss, new_distance)
                back_az = (180 + adj) % 360
                dmin, nearest = float("inf"), None
                for pe_, pn_ in poles:
                    d = np.sqrt((te - pe_) ** 2 + (tn - pn_) ** 2)
                    if d < dmin:
                        dmin, nearest = d, (pe_, pn_)
                glat, glon = to_wgs.transform(*nearest)
                vlon_, vlat_, _ = Geod(ellps="WGS84").fwd(glon, glat, back_az, new_distance)
                e, n = to_utm.transform(vlat_, vlon_)                # the pipeline's reset (line 289)
                fixes.append({"frame": i, "t": float(t_r[i]), "pole_dist_m": float(dmin), "range_m": float(new_distance)})
                hc.fix(e, n, float(t_r[i]), i)
        post_e[i], post_n[i] = e, n
        err_native.append(np.hypot(ref_e[i] - e, ref_n[i] - n))       # line 325
    pe, pn = post_e, post_n
    err = np.hypot(pe - ref_e, pn - ref_n)
    # first / last surveyed pole along the reference track
    from scipy.spatial import cKDTree
    s = np.r_[0.0, np.cumsum(np.hypot(np.diff(ref_e), np.diff(ref_n)))]
    dmin_pole, idx = cKDTree(np.c_[ref_e, ref_n]).query(np.asarray(poles))
    near = dmin_pole < 15.0
    s_first, s_last = s[idx[near]].min(), s[idx[near]].max()
    span = (s >= s_first) & (s <= s_last)

    def st(x):
        x = np.asarray(x, float)
        x = x[np.isfinite(x)]
        return {"mean": float(x.mean()), "median": float(np.median(x)), "p95": float(np.percentile(x, 95)),
                "max": float(x.max()), "n": int(len(x))} if len(x) else {"n": 0}
    res = {"label": label, "nav_csv": str(nav_csv), "heading_mode": heading_mode,
           "native_box_metric": st(err_native),
           "whole_drive": {**st(err), "final_m": float(err[-1]), "length_km": float(s[-1] / 1000)},
           "pole_span": {**st(err[span]), "from_km": float(s_first / 1000), "to_km": float(s_last / 1000),
                         "at_exit_m": float(err[np.flatnonzero(span)[-1]])},
           "after_last_pole": {**st(err[s > s_last]), "final_m": float(err[-1])},
           "pole_fixes": len(fixes), "frames_in_box": int(in_box.sum()),
           "heading_updates": len([u for u in hc.log if u["accepted"]]),
           "heading_rejected": len([u for u in hc.log if not u["accepted"]]),
           "final_theta_deg": hc.theta, "final_rate_deg_s": hc.rate}
    OUT.mkdir(parents=True, exist_ok=True)
    pd.DataFrame({"frame": np.arange(n_frames), "t": t_r, "pred_e": pe, "pred_n": pn, "ref_e": ref_e, "ref_n": ref_n,
                  "err_m": err, "in_box": in_box, "s_km": s / 1000}).to_csv(OUT / f"{label}_frames.csv", index=False)
    pd.DataFrame(hc.log).to_csv(OUT / f"{label}_heading_updates.csv", index=False)
    (OUT / f"{label}_summary.json").write_text(json.dumps(res, indent=2))
    return res


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("detect")
    p.add_argument("--max-frames", type=int, default=None)
    p = sub.add_parser("run")
    p.add_argument("--nav", required=True)
    p.add_argument("--heading", choices=["none", "reset", "reset+rate"], default="none")
    p.add_argument("--label", required=True)
    a = ap.parse_args()
    if a.cmd == "detect":
        detect(a.max_frames)
    else:
        r = run(a.nav, a.heading, a.label)
        print(json.dumps(r, indent=2))
