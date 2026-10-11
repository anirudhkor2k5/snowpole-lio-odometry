"""LiDAR-inertial odometry benchmark, weakness analysis and learned fusion for the
E39 Hemnekjølen snow-pole drive.

Modules
-------
io          CSV / TUM readers and writers, dataset config
geometry    pose interpolation, alignment (Umeyama), body-frame increments
metrics     ATE, RPE (time / distance / KITTI drift), segment errors
verify      step 1: check the legacy fastlio.csv / fasterlio.csv exports
analysis    step 3: where and why the baselines go wrong
fusion      step 4: small gating model that mixes method increments
windows     step 5: supporting method in (adaptive) sliding windows
"""

from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent
BENCH_ROOT = PACKAGE_ROOT.parents[1]          # lio_bench/
REPO_ROOT = BENCH_ROOT.parent                 # repository root
RESULTS_ROOT = BENCH_ROOT / "results"
