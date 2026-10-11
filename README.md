# Snow-pole localisation: odometry study (E39 Hemnekjølen, 2024-02-28)

This repo holds the odometry work done on top of the snow-pole localisation code in
<https://github.com/bdps1989/Snow-pole-based-vehicle-localization>. That repo is not modified. These
files sit next to it.

## Layout

| path | what |
|---|---|
| `lio_bench/` | The whole study: code, configs, ROS launch files, tests, results, reports. Start with `lio_bench/FULL_ANALYSIS.md`. |
| `lio_bench/FULL_ANALYSIS.md` | What was done, what every script does, the final numbers, limits. |
| `lio_bench/REPORT_v3.md`, `METHODS.md`, `STUDY_SUMMARY.md`, `REPORT_v2.md` | Detailed reports and methods. |
| `lio_bench/CSV_INDEX.md` | Where every CSV is and what its columns mean. |
| `lio_bench/results/export/` | Our odometry in the pipeline's `incremental_navigation_results.csv` format. |
| `lio_bench/results/pipeline/` | Snow-pole pipeline (stage 3) results and figures. |
| `lio_bench/results/runs/` | Raw estimator runs: trajectories (TUM), logs, configs, resource use. |
| `lio_bench/verify/` | Independent checks you can run by hand, e.g. `VERIFY_FASTER_LIO.md`. |
| `lio_bench/third_party_patches/` | The upstream commit (`*.pin`: URL + hash) and our local diff (`*.diff`) for every estimator. The estimator source code itself is not in this repo. |
| `groundtruth_extracted.csv`, `gps_*_position.csv` | Ground truth (GNSS/INS, UTM-33N) and the two GNSS antennas. |
| `fasterlio.csv`, `fastlio.csv` | The old FAST-LIO / Faster-LIO exports that were in the original repo folder before this work. |
| `docs/`, `experiments/`, `odometry_refinement/`, `reports/` | Earlier work: data audit, frames, a baseline and a reliability experiment. |

Not included:
- the rosbags (41 GB + 5.7 GB);
- the built estimators (`lio_bench/third_party/`), which are rebuilt from the pins and diffs;
- `fastreg.csv`, which is ignored on purpose.

## Using it with the original repo

The stage-3 scripts import `geoloc_utils.py` and the YOLO model from the original repo. They expect
`lio_bench/` and the CSVs to sit at that repo's root:

```bash
git clone https://github.com/bdps1989/Snow-pole-based-vehicle-localization.git
git clone https://github.com/anirudhkor2k5/snowpole-lio-odometry.git
# copy everything except this repo's .git; never overwrite the original's files
rsync -a --ignore-existing --exclude .git snowpole-lio-odometry/ Snow-pole-based-vehicle-localization/
cd Snow-pole-based-vehicle-localization
```

Python env: `lio_bench/envs/lio-py.yml`; ROS env: `lio_bench/envs/lio-ros.yml`. Build the
estimators with `lio_bench/setup/build_ros_ws.sh`.
