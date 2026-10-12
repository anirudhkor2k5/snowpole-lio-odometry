# Snow-pole localisation: odometry study (E39 Hemnekjølen, 2024-02-28)

Better odometry for the snow-pole localisation pipeline in
<https://github.com/bdps1989/Snow-pole-based-vehicle-localization>. That repo is not modified; these
files sit next to it.

**Read [`REPORT.md`](REPORT.md).** It is the single report on everything: what was run, in what
order, every result, every script, CSV and figure, and what holds or doesn't.

## Results in brief

**Final method:** Faster-LIO + gravity correction (IMU) + heading from KISS-ICP (LiDAR only), run
online as a ROS node next to Faster-LIO. It keeps real time on a laptop: 0 dropped scans, ~45 ms
latency, ~0.7 core and 150 MB for the node.

Pure odometry, with GNSS used only for one heading at frame 1:

| | raw Faster-LIO | + gravity | + gravity + KISS-ICP heading |
|---|---:|---:|---:|
| KITTI relative error (100–800 m) | 4.42 % | 1.66 % | **~1.2–1.3 %** |
| end error after 10.3 km | 3 306 m | 2 512 m | **~500 m typical, 130–960 m range** |

The same holds for FAST-LIO2, DLIO and GenZ-LIO: once corrected, all four are equal, so the cheapest
(Faster-LIO) is the choice.

In the professor's pipeline (tested with the earlier 200 m-fit CSVs), a heading reset at poles takes
the error inside the pole area from ~10 m to ~2 m. The ~10 m floor of the original pipeline comes
from snapping to the pole on the wrong side of the road.

## Where things are

| path | what |
|---|---|
| [`REPORT.md`](REPORT.md) | **the full report** (start here) |
| `lio_bench/results/export_optionB_all/` | **CSVs to feed the professor's pipeline** (17 files, GNSS at frame 1 only). Put a path on line 28 of `snowpole_based_vehicle_localization.py` |
| `lio_bench/results/optionB_all/` | scores of every method under that rule (`SCORES.md`), randomness test, figures |
| `lio_bench/results/export/` | the earlier CSVs with a 200 m GNSS start fit |
| `lio_bench/results/pipeline/` | the professor's pipeline run on the 200 m CSVs: 24 runs, tables, figures |
| `lio_bench/results/runs/` | every estimator run: trajectories (TUM), latency, CPU/RAM, logs |
| `lio_bench/results/` (other folders) | v1–v3 study outputs: verify, analysis, correct, fusion, windows, audit, v3 |
| `lio_bench/ros/scripts/kiss_fusion_node.py` | the final online node (`kiss_fusion_node_anyrate.py` for 100 Hz LIOs like DLIO) |
| `lio_bench/pipeline/` | stage 3 (the professor's pipeline re-implemented and validated), option B scoring |
| `lio_bench/verify/` | independent checks you can run by hand |
| `lio_bench/src/liobench/` | the `liobench` library and CLI |
| `lio_bench/scripts/` | run queues, RAM cap (`capped.sh`) |
| `lio_bench/configs/`, `lio_bench/ros/launch/` | estimator and node configs, launch files |
| `lio_bench/setup/`, `lio_bench/envs/`, `lio_bench/third_party_patches/` | setup scripts, conda envs, upstream pins and diffs (the estimator source itself is not in the repo) |
| `odometry_refinement/`, `experiments/`, `reports/` | preliminary study on the old CSV exports (report §3) |
| `groundtruth_extracted.csv`, `gps_left/right_position.csv` | ground truth (dual-antenna GNSS, UTM-33N) and the two antennas |
| `fasterlio.csv`, `fastlio.csv` | the old LIO exports that were in the folder before this work |

Not included: the rosbags (41 GB + 5.7 GB), the built estimators (`lio_bench/third_party/`, rebuilt
from the pins and diffs), and `fastreg.csv` (ignored on purpose).

## Use it with the original repo

The stage-3 scripts import `geoloc_utils.py` and the YOLO model from the original repo, and expect
`lio_bench/` and the CSVs at its root:

```bash
git clone https://github.com/bdps1989/Snow-pole-based-vehicle-localization.git
git clone https://github.com/anirudhkor2k5/snowpole-lio-odometry.git
# copy everything except this repo's .git; never overwrite the original's files
rsync -a --ignore-existing --exclude .git snowpole-lio-odometry/ Snow-pole-based-vehicle-localization/
cd Snow-pole-based-vehicle-localization
```

Put the bags at the repo root.

**Setup, once:**

```bash
bash lio_bench/setup/bootstrap.sh
```

This installs the conda envs `lio-py` and `lio-ros`, clones and builds the estimators at their
pinned commits, and installs the GenZ-LIO Python bindings.

**Common commands** (`conda activate lio-py`; wrap heavy jobs as `MEM=4G bash lio_bench/scripts/capped.sh <cmd>`):

```bash
bash lio_bench/scripts/run_ros.sh --algos faster_lio,kiss_fusion --tag mytag    # Faster-LIO + online node, real time (~10 min)
python lio_bench/pipeline/score_option_b.py                                      # option B scores, CSVs, figures (minutes)
python lio_bench/verify/diagnose_nav_csv.py some.csv                             # check a CSV before feeding it to the pipeline
python lio_bench/verify/tum_to_nav_csv.py traj.txt out.csv                       # any LIO trajectory -> pipeline CSV, option B
bash lio_bench/scripts/run_all.sh                                                # the whole v1 study (hours)
```

Every script is listed in `REPORT.md`, Appendix D.
