# lio_bench — LIO baselines, weakness analysis, learned fusion, sliding windows

Everything here is meant to run on a cloud Linux machine (x86_64 or aarch64), headless.

## 1. Machine and data

- ≥ 8 cores, ≥ 16 GB RAM (32 GB comfortable), ~80 GB free disk.
- Copy the repository plus these files into the repo root (or anywhere, then `export LIOBENCH_DATA_ROOT=/path`):
  `2024-02-28-12-59-51.bag` (41 GB, the only bag with `/ouster/points`),
  `2024-02-28-12-59-51_no_unwanted_topics.bag`, `groundtruth_extracted.csv`,
  `gps_left_position.csv`, `gps_right_position.csv`, `fastlio.csv`, `fasterlio.csv`.
  `lio_bench/` is not committed yet; copy it with rsync or commit it first.

```bash
rsync -avP Snow-pole-based-vehicle-localization/ cloud:~/snowpole/   # includes the bags
```

## 2. Setup (one command, idempotent)

```bash
bash lio_bench/setup/bootstrap.sh
```

Installs Miniforge if no conda/mamba exists, creates `lio-py` (analysis, GenZ-LIO Python, torch-cpu)
and `lio-ros` (RoboStack ROS Noetic + toolchain), clones the four estimators at pinned commits into
`lio_bench/third_party/`, applies the compatibility patches in `setup/build_ros_ws.sh` (each one is
commented), builds them, builds GenZ-LIO's Python bindings, and checks the data files exist.

## 3. Run everything

```bash
tmux new -s lio 'bash lio_bench/scripts/run_all.sh'
```

Resumable (`results/.done/<step>` markers, logs in `results/logs/`). Expect several hours: each ROS
run replays the 542 s drive in real time. Single steps:

```bash
conda activate lio-py
liobench prepare                                    # clock fit + INS heading / CAN speed from the bag
liobench run-genz --scan-features                   # GenZ-LIO (offline) + per-scan context features
bash lio_bench/scripts/run_ros.sh --algos fast_lio  # or faster_lio, dlio, genz_lio (--tag full_ros)
liobench verify                                     # step 1
liobench analyze                                    # step 3
bash lio_bench/scripts/run_ros.sh --algos fast_lio,dlio && liobench concurrency --baseline fast_lio --support dlio
liobench fusion --baseline fast_lio                 # step 4
liobench windows --baseline fast_lio --support genz_lio --schedule adaptive [--cold]   # step 5
liobench correct all --baseline fast_lio            # phase 2: wheel, ICP variants + KISS-ICP + pole candidates, pole fixes, report
liobench fusion --baseline fast_lio --supports correctors --out-name fast_lio_correctors
liobench quick --algo faster_lio                    # 2-number sanity check of every run of one algorithm
python -c "from liobench import audit; audit.run()"   # v2: canonical protocol, error budget, ablations (results/audit/)
python -c "from liobench import v3; v3.run()"         # v3: leave-one-block-out heading pipelines (results/v3/)
liobench kiss-probe nodeskew                          # v3: KISS-ICP distortion probes (nodeskew|revtime|vox05|shift5ms)
liobench run-genz --tag full_ts0 --time-scale 0       # v3: GenZ-LIO without motion compensation
```

`run_all.sh` runs each step in its own RAM-capped scope (`MEM_STEP`, default 4G on machines with
< 24 GB), so a step that runs out of memory fails alone and the rest continues.

## 4. What each step produces (`lio_bench/results/`)

| step | output | question answered |
|---|---|---|
| verify | `verify/verify_summary.md`, maps, error vs distance | Are `fastlio.csv`/`fasterlio.csv` accurate, and do they match a fresh run of the same algorithm? |
| runs | `runs/<algo>/<tag>/` trajectory, latency, CPU/RSS, logs | Raw outputs of FAST-LIO, Faster-LIO, DLIO, GenZ-LIO |
| analyze | `analysis/analysis_summary.md`, maps | Where (map) and under which conditions (scan geometry, speed, turning, returns) the baselines fail; which method wins per window |
| concurrency | `concurrency/<b>+<s>.json` | Can baseline + support run together in real time, and does that hurt either? |
| fusion | `fusion/<baseline>/fusion_summary.md`, weights over time / on the map | Learned per-step weights (“which model, what percent”) vs each method alone, uniform, static, and an oracle; blocked k-fold CV; trigger model |
| correct | `correct/<baseline>/correct_summary.md`, `drift_by_block.png`, `runs/<baseline>+<strategy>/` | Which correction strategy (wheel speed, ICP / plane ICP / GICP / VGICP, degeneracy-aware fusion, KISS-ICP, pole-map fixes) fixes the baseline's weak directions; chosen on validation, scored on test |
| windows | `windows/<baseline>/<schedule>/windows_summary.md` | Performance drop when the support only runs in adaptive / periodic windows (cold vs warm start), duty cycle, compute |

## 5. Method notes

- **Clocks.** Ouster stamps are the sensor's own clock (~13 000 s), not Unix. `liobench prepare` fits
  `unix = stamp + offset + drift·(stamp − stamp0)` from the IMU stream (105 ppm drift on this drive).
  Every trajectory also gets a residual offset from speed-profile matching against ground truth.
- **Frames.** FAST-LIO / Faster-LIO / GenZ-ROS output in the tilted start-up IMU frame (5.7° here); evaluation
  levels them with the start-up IMU gravity. Protocols: `se2_head` (anchored on the first 200 m, then pure
  prediction), `se2_full` / `se3_full` (standard whole-drive ATE), and frame-free segment drift (KITTI-style).
- **Fusion targets** are planar body-frame increments (forward, lateral, yaw) of each method vs ground truth;
  the model outputs masked-softmax weights, is trained with availability dropout so it also works when a
  support method is off, and is evaluated only on chronological held-out blocks.
- **Windows.** Adaptive windows open when the trigger model (baseline-only features) predicts the baseline
  is about to be unreliable. GenZ-LIO gets an optional warm start (filter velocity seeded from the baseline),
  via a small patch applied by `setup/install_genz_py.sh`.

- **Correction strategies (phase 2).** Each scan is deskewed with the baseline's motion and registered
  to a local map by each ICP variant, starting from the baseline's increment. Degeneracy-aware variants
  fuse that ICP correction with the prior (baseline, or baseline with CAN forward motion) through
  ICP's own information matrix, so directions the road does not constrain keep the prior. Pole fixes:
  thin isolated vertical clusters matched to the surveyed pole map in a causal EKF (no fitted constants).
- **Overfitting (one drive).** Every fitted quantity uses the training block only; correction
  strategies are selected on validation; the fusion model is picked from a ladder (static weights →
  linear gating → MLP-16 → MLP-64) by blocked CV with the one-standard-error rule, and the summary
  reports a shuffled-context control, the train/val/test gap, seed spread and the overlap between the
  test road and the training route. If the learned model does not beat static weights and its
  control on test, the summary says so.

Reports: `STUDY_SUMMARY.md` (v1), `REPORT_v2.md` (Phase A audit), `REPORT_v3.md` (heading, online
attitude, paper perspective); exact definitions in `METHODS.md`.

## 6. Known issues

- **`point_filter_num` must not be 4 on this sensor** (fixed in the configs). The OS2 has 1024 columns
  per ring; keeping every 4th point keeps the same azimuth columns in every ring, and those radial
  stripes pin the scan to the map along the road. Over the first 30 s: Faster-LIO length ratio 0.24 → 1.004,
  FAST-LIO 0.974 → 1.006 with 3. The legacy `fastlio.csv` matches a run with 4
  (`configs/algorithms/variants/fast_lio_pf4_legacy.yaml`, run as tag `full_pf4_legacy`).
- **GenZ-LIO map pruning**: the authors' configs never prune (`map_range: -1`); on this 10 km drive that
  exceeds 4 GB after ~500 scans. We prune beyond 300 m (2× sensor range).
- DLIO publishes IMU-rate poses from a timer, so per-scan latency is not reported for it; DLIO cold-start
  windows calibrate the IMU while moving, which is part of the measured cold-start cost.
