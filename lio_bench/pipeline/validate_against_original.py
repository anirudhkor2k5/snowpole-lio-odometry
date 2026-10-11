"""Run the ORIGINAL `snowpole_based_vehicle_localization.py` unmodified, to validate snowpole_stage3.py.

The original file is executed as-is (runpy). Only its environment is adapted, without editing it:
  - bag loading: geoloc_utils.process_ros_bag_data is replaced by a lazy, streaming equivalent
    (same topics, same order, same record-time stamps) so ~5.7 GB of images are not held in RAM;
  - YOLO loading: geoloc_utils.load_custom_model loads the same checkpoint via the pinned
    YOLOv5 v7.0 hub release (the original pulls the moving `master`), conf 0.6 as in the script;
  - display: headless matplotlib (show/pause are no-ops), no OpenStreetMap tile download
    (contextily stubbed if not installed),
    cv2 window calls no-ops;
  - `from ouster import client`: Ouster SDK 1.0 renamed it to `ouster.sdk.core` (range image
    passed as uint32, values unchanged).
Its stdout (the per-frame "Average Error ..." lines) is the result; the last value is compared
with snowpole_stage3.py run on the same CSV.

  python lio_bench/pipeline/validate_against_original.py > lio_bench/results/logs/original_pipeline.log
"""
from __future__ import annotations

import os
import runpy
import sys
import types
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import snowpole_stage3 as S  # noqa: E402

REPO = S.REPO


class _FrameStream:
    """Forward-only reader of (signal, range) image pairs, frame i on demand."""

    def __init__(self, bag):
        from rosbags.highlevel import AnyReader
        self.r = AnyReader([bag])
        self.r.open()
        con = [c for c in self.r.connections if c.topic in ("/ouster/signal_image", "/ouster/range_image")]
        self.it = self.r.messages(connections=con)
        self.sig, self.rng = {}, {}
        self.ns = self.nr = 0
        self.cur = -1

    def get(self, i):
        while not (i in self.sig and i in self.rng):
            c, t, raw = next(self.it)
            m = self.r.deserialize(raw, c.msgtype)
            img = np.frombuffer(m.data, dtype=np.uint16).reshape((m.height, m.width)).copy()
            if c.topic == "/ouster/signal_image":
                self.sig[self.ns] = img
                self.ns += 1
            else:
                self.rng[self.nr] = img
                self.nr += 1
        for k in [k for k in self.sig if k < i - 1]:
            self.sig.pop(k)
        for k in [k for k in self.rng if k < i - 1]:
            self.rng.pop(k)
        return self.sig[i], self.rng[i]


class _LazyImages:
    def __init__(self, stream, which, n):
        self.s, self.which, self.n = stream, which, n

    def __len__(self):
        return self.n

    def __getitem__(self, i):
        return self.s.get(i)[0 if self.which == "signal" else 1]

    def __iter__(self):
        for i in range(self.n):
            yield self[i]


def _lazy_process_ros_bag_data(bag_file_path):
    from rosbags.highlevel import AnyReader
    bag = REPO / bag_file_path
    gl, gr, tl, trr, ts, tr_ = [], [], [], [], [], []
    with AnyReader([bag]) as r:
        con = [c for c in r.connections if c.topic in ("/gps_left_position", "/gps_right_position",
                                                        "/ouster/signal_image", "/ouster/range_image")]
        for c, t, raw in r.messages(connections=con):
            if c.topic == "/ouster/signal_image":
                ts.append(t * 1e-9)
            elif c.topic == "/ouster/range_image":
                tr_.append(t * 1e-9)
            else:
                m = r.deserialize(raw, c.msgtype)
                (gl if c.topic == "/gps_left_position" else gr).append((m.latitude, m.longitude))
                (tl if c.topic == "/gps_left_position" else trr).append(t * 1e-9)
    stream = _FrameStream(bag)
    sig, rng = _LazyImages(stream, "signal", len(ts)), _LazyImages(stream, "range", len(tr_))
    return sig, [], [], rng, gl, gr, [], [], [], ts, [], [], tr_, tl, trr, []


def main():
    os.chdir(REPO)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.show = lambda *a, **k: None
    plt.pause = lambda *a, **k: None
    try:
        import contextily
    except ImportError:                              # only used for the OpenStreetMap background
        contextily = types.ModuleType("contextily")
        contextily.providers = types.SimpleNamespace(OpenStreetMap=types.SimpleNamespace(Mapnik=None))
        sys.modules["contextily"] = contextily
    contextily.add_basemap = lambda *a, **k: None
    import cv2
    for fn in ("imshow", "waitKey", "destroyAllWindows"):
        setattr(cv2, fn, lambda *a, **k: None)
    from ouster.sdk import core
    shim = types.ModuleType("ouster.client")
    shim.SensorInfo = core.SensorInfo
    shim.XYZLut = lambda info: (lambda lut: (lambda img: lut(np.asarray(img).astype(np.uint32))))(core.XYZLut(info))
    import ouster
    ouster.client = shim
    sys.modules["ouster.client"] = shim
    gu = S._import_geoloc_utils()
    gu.process_ros_bag_data = _lazy_process_ros_bag_data
    gu.load_custom_model = lambda path, confidence_threshold=0.6, backend=None: S._load_model()
    matplotlib.use = lambda *a, **k: None          # the script's TkAgg request stays headless
    runpy.run_path(str(REPO / "snowpole_based_vehicle_localization.py"), run_name="__main__")


if __name__ == "__main__":
    main()
