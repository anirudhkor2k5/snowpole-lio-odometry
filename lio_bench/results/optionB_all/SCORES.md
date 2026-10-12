# Option B for every method: pure odometry, GNSS heading at frame 1 only

*`lio_bench/pipeline/score_option_b.py`. Every trajectory: one rotation so its heading at the first pose equals the dual-antenna heading (row 0 of `gps_left/right_position.csv`), one shift to the GNSS position there. No 200 m fit, no clock offset, no yaw-bias calibration, nothing fitted afterwards. KISS-ICP at 2.0 m voxels everywhere. Reference: `groundtruth_extracted.csv` (dual-antenna midpoint). Pipeline CSVs: `results/export_optionB_all/`.*

## 1. Whole drive, as placed (nothing fitted after frame 1)

| method | variant | ATE RMSE m | mean m | final m | end-to-end drift % | heading err at end ° | length ratio | first pole m | last pole m |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Faster-LIO | raw | 1391 | 985 | 3306 | 32.1 | +35.1 | 0.963 | 32.0 | 784 |
| Faster-LIO | + gravity | 1064 | 753 | 2512 | 24.4 | +31.4 | 0.997 | 28.5 | 586 |
| Faster-LIO | + gravity + KISS heading (offline) | 115 | 103 | 132 | 1.3 | +2.6 | 0.997 | 20.6 | 146 |
| Faster-LIO | + gravity + KISS heading (online node) | 464 | 359 | 962 | 9.3 | +10.5 | 0.997 | 21.0 | 357 |
| FAST-LIO2 | raw | 1643 | 1166 | 3895 | 37.8 | +27.7 | 0.921 | 27.1 | 974 |
| FAST-LIO2 | + gravity | 912 | 657 | 2098 | 20.4 | +24.0 | 0.998 | 23.3 | 562 |
| FAST-LIO2 | + gravity + KISS heading (offline) | 114 | 102 | 132 | 1.3 | +2.4 | 0.998 | 19.8 | 144 |
| FAST-LIO2 | + gravity + KISS heading (online node) | 458 | 355 | 949 | 9.2 | +10.3 | 0.998 | 20.1 | 353 |
| DLIO | raw | 1212 | 843 | 2928 | 28.4 | +20.2 | 0.921 | 19.4 | 660 |
| DLIO | + gravity | 760 | 541 | 1833 | 17.8 | +25.6 | 1.001 | 21.8 | 432 |
| DLIO | + gravity + KISS heading (offline) | 119 | 106 | 154 | 1.5 | +3.0 | 1.001 | 19.2 | 146 |
| DLIO | + gravity + KISS heading (online node) | 278 | 220 | 589 | 5.7 | +8.0 | 1.001 | 19.5 | 217 |
| GenZ-LIO | raw | 1560 | 1122 | 3666 | 35.6 | +26.8 | 0.926 | 30.8 | 950 |
| GenZ-LIO | + gravity | 915 | 676 | 2063 | 20.0 | +23.8 | 0.999 | 26.5 | 602 |
| GenZ-LIO | + gravity + KISS heading (offline) | 112 | 100 | 134 | 1.3 | +2.5 | 0.999 | 20.0 | 141 |
| GenZ-LIO | + gravity + KISS heading (online node) | 274 | 218 | 580 | 5.6 | +7.6 | 0.999 | 20.3 | 213 |
| KISS-ICP | alone (LiDAR only, 2.0 m) | 164 | 144 | 283 | 2.7 | +2.6 | 0.986 | 23.1 | 187 |

## 2. Relative accuracy (KITTI-style, needs no placement) and paper-style ATE

| method | variant | KITTI translation % | KITTI rotation °/100 m | RPE 1 s median m | RPE 1 s RMSE m | paper-style ATE m (whole-drive best fit, evaluation only) |
|---|---|---:|---:|---:|---:|---:|
| Faster-LIO | raw | 4.42 | 0.591 | 0.43 | 1.28 | 355 |
| Faster-LIO | + gravity | 1.66 | 0.385 | 0.09 | 0.23 | 253 |
| Faster-LIO | + gravity + KISS heading (offline) | 1.26 | 0.273 | 0.09 | 0.23 | 45 |
| Faster-LIO | + gravity + KISS heading (online node) | 1.27 | 0.285 | 0.09 | 0.23 | 51 |
| FAST-LIO2 | raw | 8.28 | 0.955 | 0.83 | 2.77 | 453 |
| FAST-LIO2 | + gravity | 1.48 | 0.342 | 0.09 | 0.18 | 192 |
| FAST-LIO2 | + gravity + KISS heading (offline) | 1.25 | 0.280 | 0.09 | 0.18 | 45 |
| FAST-LIO2 | + gravity + KISS heading (online node) | 1.25 | 0.287 | 0.09 | 0.18 | 49 |
| DLIO | raw | 7.82 | 0.642 | 0.81 | 2.62 | 414 |
| DLIO | + gravity | 1.37 | 0.349 | 0.23 | 0.37 | 181 |
| DLIO | + gravity + KISS heading (offline) | 1.23 | 0.299 | 0.23 | 0.37 | 49 |
| DLIO | + gravity + KISS heading (online node) | 1.13 | 0.280 | 0.23 | 0.37 | 22 |
| GenZ-LIO | raw | 7.66 | 0.861 | 0.74 | 2.59 | 405 |
| GenZ-LIO | + gravity | 1.42 | 0.342 | 0.09 | 0.15 | 164 |
| GenZ-LIO | + gravity + KISS heading (offline) | 1.17 | 0.277 | 0.09 | 0.15 | 46 |
| GenZ-LIO | + gravity + KISS heading (online node) | 1.09 | 0.259 | 0.09 | 0.16 | 24 |
| KISS-ICP | alone (LiDAR only, 2.0 m) | 2.04 | 0.323 | 0.21 | 0.46 | 40 |

## 3. Start rotation applied and pipeline CSV

| method | variant | rotation ° | poses | CSV |
|---|---|---:|---:|---|
| Faster-LIO | raw | +5.98 | 5411 | `lio_bench/results/export_optionB_all/incremental_navigation_faster_lio_raw.csv` |
| Faster-LIO | + gravity | +5.89 | 5411 | `lio_bench/results/export_optionB_all/incremental_navigation_faster_lio_gravity.csv` |
| Faster-LIO | + gravity + KISS heading (offline) | +5.89 | 5411 | `lio_bench/results/export_optionB_all/incremental_navigation_faster_lio_gk.csv` |
| Faster-LIO | + gravity + KISS heading (online node) | +5.92 | 5414 | `lio_bench/results/export_optionB_all/incremental_navigation_faster_lio_node.csv` |
| FAST-LIO2 | raw | +5.97 | 5412 | `lio_bench/results/export_optionB_all/incremental_navigation_fast_lio_raw.csv` |
| FAST-LIO2 | + gravity | +5.88 | 5412 | `lio_bench/results/export_optionB_all/incremental_navigation_fast_lio_gravity.csv` |
| FAST-LIO2 | + gravity + KISS heading (offline) | +5.88 | 5412 | `lio_bench/results/export_optionB_all/incremental_navigation_fast_lio_gk.csv` |
| FAST-LIO2 | + gravity + KISS heading (online node) | +5.90 | 5412 | `lio_bench/results/export_optionB_all/incremental_navigation_fast_lio_node.csv` |
| DLIO | raw | +5.99 | 41165 | `lio_bench/results/export_optionB_all/incremental_navigation_dlio_raw.csv` |
| DLIO | + gravity | +5.90 | 41165 | `lio_bench/results/export_optionB_all/incremental_navigation_dlio_gravity.csv` |
| DLIO | + gravity + KISS heading (offline) | +5.90 | 41165 | `lio_bench/results/export_optionB_all/incremental_navigation_dlio_gk.csv` |
| DLIO | + gravity + KISS heading (online node) | +5.98 | 41160 | `lio_bench/results/export_optionB_all/incremental_navigation_dlio_node.csv` |
| GenZ-LIO | raw | +5.98 | 5413 | `lio_bench/results/export_optionB_all/incremental_navigation_genz_lio_raw.csv` |
| GenZ-LIO | + gravity | +5.89 | 5413 | `lio_bench/results/export_optionB_all/incremental_navigation_genz_lio_gravity.csv` |
| GenZ-LIO | + gravity + KISS heading (offline) | +5.89 | 5413 | `lio_bench/results/export_optionB_all/incremental_navigation_genz_lio_gk.csv` |
| GenZ-LIO | + gravity + KISS heading (online node) | +5.91 | 5413 | `lio_bench/results/export_optionB_all/incremental_navigation_genz_lio_node.csv` |
| KISS-ICP | alone (LiDAR only, 2.0 m) | +5.99 | 5419 | `lio_bench/results/export_optionB_all/incremental_navigation_kiss_icp_alone.csv` |

## Figures

- `figures/error_vs_distance.png`: position error along the drive, one panel per LIO
- `figures/maps.png`: tracks vs GNSS, one panel per LIO
- `figures/kitti_translation.png`: KITTI translation error per method
- `figures/end_to_end_drift.png`: end-to-end drift per method

## How these runs were made and how to read them

- **Runs:**
  - Faster-LIO: `faster_lio/rerun_b` and `kiss_fusion/rerun_b_fusion` (2026-10-12, reused).
  - FAST-LIO2, DLIO and GenZ-LIO: fresh real-time bag runs (`<lio>/optb`), each run together with its own
    online node (`kiss_fusion_<lio>/optb`). The concurrency study showed that running two at once doesn't
    change accuracy. In every node, KISS-ICP processed all 5 419 scans.
  - KISS-ICP alone at 2.0 m: fresh offline pass (`kiss_icp_vox20/optb`). It is also the heading
    source for every offline "+ KISS" row.
  - Queue: `scripts/run_option_b_all.sh`; log: `results/logs/run_option_b_all.log`.
- **DLIO's node** is `ros/scripts/kiss_fusion_node_anyrate.py` with `configs/algorithms/kiss_fusion_100hz.yaml`.
  DLIO publishes 100 Hz poses and the original node assumes one pose per scan. The copy applies each
  KISS-ICP turn once per scan; with one pose per scan it is the same rule. The first DLIO run with an
  earlier version of the copy reused scans (49 m at the first pole, 423 guard fallbacks). That run was
  discarded and re-run; the discarded copy is in `/tmp/dlio_optb_v1/`. The re-run had 1 missing-scan
  fallback and 0 guard fallbacks.
- **What each column means:**
  - **Section 1** is the honest "GNSS at frame 1 only" result.
  - **KITTI** is what KISS-ICP reports. Like KITTI, each segment is matched to the reference at its own
    start, so it measures local tracking, not position.
  - **Paper-style ATE** is what the LIO papers print. It fits the whole drive to GNSS and is evaluation
    only; no CSV uses it.
- **Observations:**
  - The order is the same for all four LIOs: raw → + gravity (KITTI ~8 % → ~1.4 %, length ratio → 1.00)
    → + KISS heading.
  - With both corrections the four LIOs end within 132–154 m of each other after 10.3 km (1.3–1.5 %).
  - **Offline beats the online node** (end heading error +2.4…3.0° vs +7.6…10.5°). Both use 2.0 m
    voxels and the same rule. They differ in how the scans reach KISS-ICP: the offline pass reads the
    bag through the GenZ-LIO preprocessing with 4 threads, while the node takes the live ROS cloud with
    2 threads. Why that makes the node's heading drift more has not been investigated.
  - The offline 2.0 m KISS pass repeated to the metre (132 m end error in both runs).
- **Caveat for the pole pipeline:** every file is 19–32 m off at the first pole, which is more than half
  the 18 m pole spacing, so the original stage 3 may lock onto the wrong pole.

## Runtime (real-time replay on the laptop, indicative only)

| LIO | run | LIO CPU % | LIO RAM MB | LIO latency median / p95 ms | node CPU % | node RAM MB | node latency median / p95 ms | KISS-ICP in node, ms/scan median / p95 | scans KISS-ICP processed |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Faster-LIO | solo run (raw / gravity / gk rows) | 147 | 409 | 36 / 48 | — | — | — | — | — |
| Faster-LIO | with its node (node row) | 112 | 432 | 32 / 45 | 68 | 150 | 45 / 64 | 39 / 57 | 5419 / 5419 |
| FAST-LIO2 | with its node | 96 | 625 | 43 / 59 | 68 | 180 | 49 / 65 | 36 / 53 | 5419 / 5419 |
| DLIO | with its node (100 Hz poses) | 216 | 3074 | n/a (100 Hz timer poses) | 74 | 164 | 41 / 114 | 25 / 46 | 5419 / 5419 |
| GenZ-LIO | with its node | 217 | 2557 | 80 / 100 | 69 | 149 | 82 / 111 | 30 / 50 | 5419 / 5419 |

KISS-ICP alone (offline pass, 2.0 m, 4 threads): 11 ms/scan, 1.9 min for the whole drive (the drive is 9.0 min, so faster than real time).
The offline gravity and gravity + KISS rows are post-processing of recorded runs and take seconds.
Every run replayed the bag at real-time speed (~9.1 min of playback per run). DLIO publishes poses from a 100 Hz timer, so its per-scan latency is not defined; its node's latency is measured the same way as the others (pose received after the latest scan).
