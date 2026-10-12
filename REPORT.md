# Odometry for snow-pole localisation: full report

*E39 Hemnekjølen, one drive on 2024-02-28 (10.3 km, 542 s). Work done 2026-10-10 → 2026-10-13. This is
the single consolidated report: it replaces `STUDY_SUMMARY.md`, `REPORT_v2.md`, `REPORT_v3.md`,
`METHODS.md`, `FULL_ANALYSIS.md`, `ALL_RUNS_ANALYSIS.md`, `CSV_INDEX.md`, `VERIFY_FASTER_LIO.md`,
`lio_bench/README.md` and `docs/01–05`, which are still in the git history (commit `230e416` and earlier).
Tables that scripts generate are kept next to their data and linked from here. All paths are
relative to the repo root. Code comments that cite the old files map as: REPORT_v2 → §6,
REPORT_v3 §1–§10 → §7, REPORT_v3 §11 → §7.5–§7.8, REPORT_v3 §12 → §8, METHODS → Appendix A.*

---

## Contents

0. [Summary](#0-summary)
1. [The task, the data and the rules](#1-the-task-the-data-and-the-rules)
2. [Timeline: how each step led to the next](#2-timeline-how-each-step-led-to-the-next)
3. [Preliminary study on the exported CSVs](#3-preliminary-study-on-the-exported-csvs-odometry_refinement)
4. [Getting four LIOs to run](#4-getting-four-lios-to-run)
5. [v1: baselines, the tilt discovery, correctors, learned fusion, windows](#5-v1-baselines-the-tilt-discovery-correctors-learned-fusion-windows)
6. [v2: audit: heading is the real error](#6-v2-audit-heading-is-the-real-error)
7. [v3: heading, mechanisms, the final system](#7-v3-heading-mechanisms-the-final-system)
8. [Stage 3: the professor's snow-pole pipeline (200 m CSVs)](#8-stage-3-the-professors-snow-pole-pipeline-200-m-csvs)
9. [Is our Faster-LIO real? The friend's 53 m. How much GNSS did we use?](#9-is-our-faster-lio-real-the-friends-53-m-how-much-gnss-did-we-use)
10. [Option B: GNSS for one heading at frame 1, nothing else](#10-option-b-gnss-for-one-heading-at-frame-1-nothing-else)
11. [Offline vs online, and the randomness test](#11-offline-vs-online-and-the-randomness-test)
12. [Conclusions, claim status, limitations, open work](#12-conclusions-claim-status-limitations-open-work)
- [Appendix A: methods and equations](#appendix-a-methods-and-equations)
- [Appendix B: where GNSS enters (leakage audit)](#appendix-b-where-gnss-enters-leakage-audit)
- [Appendix C: every CSV](#appendix-c-every-csv)
- [Appendix D: every script](#appendix-d-every-script)
- [Appendix E: every figure](#appendix-e-every-figure)
- [Appendix F: generated result files](#appendix-f-generated-result-files)
- [Appendix G: verify it yourself](#appendix-g-verify-it-yourself)
- [Appendix H: glossary](#appendix-h-glossary)

---

## 0. Summary

**Goal.** The professor's pipeline dead-reckons the car from an odometry CSV and snaps its position
to surveyed snow poles. Better odometry means less drift between poles and after the last one. The
task: improve the odometry around an off-the-shelf LiDAR-inertial odometry (LIO) system, without
changing that system's code. FastReg is ignored.

**Final method.** **Faster-LIO + gravity correction + KISS-ICP heading, run online** (a ROS node,
`lio_bench/ros/scripts/kiss_fusion_node.py`).
- Distance comes from Faster-LIO.
- Tilt comes from the IMU's gravity: a 5 s complementary filter.
- The turn of each scan comes from KISS-ICP, a LiDAR-only odometry.

It runs in real time on the laptop: 0 dropped scans, ~45 ms latency, ~0.7 core and 150 MB for the
node.

**Headline numbers, option B (GNSS gives one heading at frame 1, nothing else):**

| | raw Faster-LIO | + gravity | + gravity + KISS heading |
|---|---:|---:|---:|
| KITTI relative error (100–800 m pieces) | 4.42 % | 1.66 % | **1.26–1.27 %** |
| 10.3 km end error | 3 306 m | 2 512 m | **~500 m typical, 130–960 m range** (§11) |

The same pattern holds for all four LIOs (FAST-LIO2, Faster-LIO, DLIO, GenZ-LIO). Once corrected
they are equal, so the cheapest one, Faster-LIO, is the choice.

**The five findings that carry the work:**
1. **Every LIO fails the same way on this road: tilt.** They slowly pitch nose-up and "climb"
   2.4–3.4 km over 10 km, so the horizontal track comes out 5–9 % short. The scene causes it, not
   the code: the snowy open road barely constrains forward motion. Tilt from IMU gravity fixes it:
   KITTI ~8 % → ~1.4 %.
2. **What is left is heading drift.** All methods turn slowly counter-clockwise at a near-constant
   rate per second (a bias). LiDAR-only KISS-ICP drifts 2–3× less. Taking the heading from KISS-ICP
   is the second fix.
3. **After both fixes the LIO doesn't matter.** All four end within 132–154 m of each other
   (offline) at ~1.2 % KITTI. DLIO and GenZ-LIO cost 2× the CPU and 5–7× the RAM.
4. **In the professor's pipeline** (tested with the 200 m-fit CSVs), a heading reset at poles takes
   the pole-span error from ~10 m to ~2 m for every input. The original pipeline's ~10 m floor is
   **wrong-side pole association**: poles stand in pairs 8.8 m apart across the road.
5. **A single 10 km end error can't rank methods.** KISS-ICP's long-distance heading changes with
   trivial input details: the same method ends 130–960 m off. Report KITTI %, plus the end error as
   a range.

**What you feed the professor's code:** `lio_bench/results/export_optionB_all/` (17 CSVs, §10.4).
Put the path on line 28 (`file_path = ...`) of `snowpole_based_vehicle_localization.py`. Every file
is 19–32 m off at the first pole, more than half the 18 m pole spacing, so the original pipeline may
lock onto a wrong pole there.

---

## 1. The task, the data and the rules

### 1.1 Data

| item | facts |
|---|---|
| Drive | E39 Hemnekjølen, 2024-02-28, 541.75 s, 10.3 km. ~38 km/h for the first km, then ~75 km/h. GNSS altitude 203–387 m, the road descends 163 m overall. |
| Full bag `2024-02-28-12-59-51.bag` (41 GB) | The only bag with `/ouster/points` (5 419 scans, Ouster OS2-128, 1 024 columns, 10 Hz). Also `/ouster/imu` (~100 Hz), `/vehicle_enu_heading`, CAN (Dataspeed MKZ), cameras. Not in the repo. |
| Reduced bag (5.7 GB) | `/ouster/imu`, GNSS, INS, and four Ouster images (range, signal, reflec, nearir; 5 420 each). No point clouds. Used by the pole detector. Not in the repo. |
| Ground truth `groundtruth_extracted.csv` | UTM-33N easting/northing at 10 Hz. It is the vehicle's own **dual-antenna GNSS midpoint** (median 3.4 cm from it), RTK float (fix status 5 on 5 382 samples, 6 = dead reckoning on 40). Not an independent survey. |
| Antennas `gps_left/right_position.csv` | 5 422 rows each. Baseline 0.499 m ± 1.3 cm, perpendicular to travel. Their row 0 gives the start heading in option B. |
| Pole map (professor's repo) | 290 surveyed poles covering km 0.72–5.19, ~18 m apart, in pairs on both sides of the road (8.8 m apart across). |
| Legacy CSVs `fastlio.csv`, `fasterlio.csv` | Old FAST-LIO / Faster-LIO exports that were in the folder before this work (§3, §5.1). |
| Pipeline input `incremental_navigation_results.csv` (professor's repo) | 5 422 rows, one per left-GNSS message. The pipeline reads only `easting`/`northing`. It takes heading and step length from consecutive rows and pairs rows with image frames **by row index**. Used as the template for every CSV we write. |

**Sensor facts established along the way:**
- **Clock.** Ouster stamps are the sensor's own oscillator (~13 214 s), not Unix. Fitted mapping:
  `unix = s + 1709108377.355 + 105 ppm·(s − s0)`, residual 0.14 ms (`results/bag/sensor_clock.json`).
- **LiDAR–IMU extrinsic** (from the bag's `Trip068.json`):
  - `/ouster/points` is in `os_sensor`;
  - `lidar_to_sensor` = 180° yaw + 74.3 mm z (already applied in the published cloud);
  - `imu_to_sensor` = identity rotation + (6.253, −11.775, 7.645) mm.
  - So the LIO extrinsic is **R = I, T = [−0.006253, 0.011775, −0.007645] m**.
- **Vertical agreement.** IMU and LiDAR agree on "up" to ~0.3°: gyro turning axis 1.75°, LiDAR
  ground normal 1.50°, training-block mean specific force 1.71° from IMU z. The 5.6° accelerometer
  tilt at the start is the car parked on a slope.
- **The INS heading topic is unreliable.** `/vehicle_enu_heading` never sets its `heading_init` /
  `nav_filter_init` / `dual_antenna_heading` flags in this bag and reads 4–9° low while driving.
- **CAN wheel speed** reads 7 % high (scale 0.928) with a 0.35 s lag; per-step noise 3 %, unbiased.

### 1.2 Rules we worked under

- **The professor's repo is never modified.** Nothing was committed there; everything new lives in
  `lio_bench/` and this private repo.
- **Estimators are official upstream code at pinned commits** (§4). Only build fixes and the
  config changes listed in §4.
- **Every heavy job runs alone under a hard RAM cap** (`lio_bench/scripts/capped.sh`, 3–5 GB) on a
  15 GB laptop (Apple M-series, Asahi Linux).
- **Pure odometry means no GNSS correction after frame 1** (option B, §10). Earlier results used a
  200 m GNSS start fit; §9.3 lists exactly which results used how much GNSS.

---

## 2. Timeline: how each step led to the next

| # | when | question | answer | led to |
|---|---|---|---|---|
| 1 | before 10-10 | Can the exported CSVs be used as is? (§3) | The Faster-LIO export contradicts itself (lat/lon vs easting/northing 271 m apart); a learned reliability model is a no-go | re-run the estimators from the bag |
| 2 | 10-10 | Can four LIOs run on this bag on a laptop? (§4) | Yes, after `point_filter_num` 4 → 3 and a GenZ-LIO map limit | v1 study |
| 3 | 10-10 | How good are they, and where do they fail? (§5) | All drift km-level; tracks 5–9 % short because of **tilt**, not scale | correctors, learned fusion, windows |
| 4 | 10-10 | Do correctors, fusion or sliding windows help? (§5) | Planar re-integration is the big free win; fusion helps locally; windows keep ~30 % of the benefit | an audit of the protocol |
| 5 | 10-11 | With a deployable protocol, what is the error made of? (§6) | **~97 % heading**; tilt is pitch; gravity-locked attitude fixes tilt online | find the least-biased heading source |
| 6 | 10-11 | Where does heading come from best, and why do the LIOs drift? (§7) | KISS-ICP heading wins 9/10 blocks; motion distortion drives the bias; several causes rejected | final system: real-time node |
| 7 | 10-11 | Does it help the professor's pipeline? (§8) | Yes; the heading reset at poles is the big stage-3 lever; the ~10 m floor is wrong-side pole association | — |
| 8 | 10-11 | Is our Faster-LIO "too good"? Why did a friend get 53 m? (§9) | Ours is genuine (slightly worse than the old export); the friend's start heading was 4.5° off | the 200 m fit counts as GNSS help |
| 9 | 10-12 | Pure odometry with GNSS at frame 1 only: option B (§10) | Faster-LIO's four CSVs re-made from fresh bag runs; then every LIO (17 trajectories, KISS at 2.0 m everywhere) | offline beat online: why? |
| 10 | 10-12 | Why does offline beat online? (§11) | It doesn't; the 10 km end error swings 130–960 m with trivial input changes (randomness test) | report KITTI % and an end-error range |

---

## 3. Preliminary study on the exported CSVs (`odometry_refinement/`)

Before any estimator was run, the existing exports were audited and a black-box reliability model
was tried. Code: `odometry_refinement/` (scripts `audit_data.py`, `evaluate_baselines.py`,
`extract_features.py`, `train_reliability.py`; tests `tests/test_core.py`). Outputs:
`reports/dataset_statistics.json`, `experiments/baseline/`, `experiments/reliability/`.

### 3.1 Data audit (read-only; source SHA-256 unchanged)

| file | rows | role |
|---|---:|---|
| `fasterlio.csv` | 5 415 | Faster-LIO export (Unix clock; starts 0.345 s before the first GNSS row) |
| `fastlio.csv` | 5 416 | FAST-LIO export (relative clock starting at 0) |
| `fastreg.csv` | 5 416 | FastReg-labelled export (ignored) |
| `incremental_navigation_results.csv` | 5 422 | what the pipeline reads; stamps equal `gps_left` to 0.24 µs |
| `gps_left/right_position.csv` | 5 422 | GNSS antennas |
| pole map | 290 | landmark map |

**Findings:**
- **The pipeline uses only `easting`/`northing`.** It derives the heading as the geodesic azimuth
  between consecutive rows, and associates odometry with range images by index, not by time. GNSS
  is read at index i+1. With 5 420 images vs 5 422 rows, the index is off by design.
  `rotation_angle_degrees` is not read; it is not yaw (64–72° median difference from course).
- **The Faster-LIO export contradicts itself.** Its lat/lon and its easting/northing differ by a
  median of 270.6 m (max 592 m), and their 1 s motions differ by 1.59 m.
  - `fastlio.csv`: 10.1 m and 0.16 m.
  - `fastreg.csv`'s lat/lon is a copy of `fastlio.csv`'s.
  - The incremental file is self-consistent.
- **Export altitude is not a height.** It runs to 2 387 m (Faster-LIO) and 2 974 m (FAST-LIO),
  against 203–387 m for GNSS. This was the first sign of the tilt problem found in §5.3.
- **The pipeline's lever arms** `gnss_offset = [−0.32, 0, 1.24]` and `lidar_offset = [0.70, 0, 1.80]`
  are undocumented in axis order and were not applied.
- **UTM zone.** The site is at ~9.5° E (zone 32), but the repo and the pole map use zone 33; we
  follow zone 33. All 290 poles lie within 15 m of the GNSS track (median 5.53 m).

### 3.2 Baseline of the exports (identity frame, GNSS midpoint)

| export | column | ATE median | ATE RMSE | end error | 1 s RPE median |
|---|---|---:|---:|---:|---:|
| fasterlio | lat/lon → UTM | 234 m | 868 m | 2 233 m | 2.38 m |
| fasterlio | stored EN | 411 m | 1 135 m | 2 784 m | 4.39 m |
| fastlio | lat/lon → UTM | 438 m | 1 116 m | 2 850 m | 3.85 m |
| fastreg | stored EN | 554 m | 933 m | 2 327 m | 4.15 m |
| incremental (pipeline) | stored EN | 50.2 m | 77.6 m | 231 m | 4.04 m |

The LIO exports are small for about a minute and then drift to kilometres. Figures:
`experiments/baseline/` (`trajectories_unaligned.png`, `error_vs_time.png`, `error_vs_distance.png`,
`segment_rmse.png`, `rpe_1s.png`, `error_histograms.png`, `reference_quality.png`).

### 3.3 Reliability experiment: no-go

**Question.** Can causal motion features from the exported track predict which frames will have
more than 2 m of 1 s error later in the drive?

**Answer: no.**
- **The held-out part has no negatives.** For the LIO exports, validation and test contain only
  "bad" frames (positive rate 1.00), so ranking can't be tested at all.
- **On the incremental file** (test positive rate 0.955), every model sits at the base rate:

  | model | test PR-AUC | test ROC-AUC |
  |---|---:|---:|
  | analytical rule | 0.952 | 0.461 |
  | logistic regression | 0.951 | 0.463 |
  | XGBoost | 0.957 | 0.506 |
  | MLP | 0.948 | 0.444 |
  | lagged logistic | 0.961 | 0.539 |

**Decision:** no learned reliability model and no correction oracle on the exports. Re-run the
estimators from the bag instead (§4).

---

## 4. Getting four LIOs to run

| estimator | upstream commit | patches (`lio_bench/third_party_patches/*.diff`) |
|---|---|---|
| FAST-LIO2 | `7cc4175` | build fixes (glog 0.7, PCL C++17, message-generation races) |
| Faster-LIO | `8038fba` (upstream `main` when checked) | build fixes + `std::execution::par_unseq` → `seq` in 5 loops (same maths, deterministic) |
| DLIO | `fc8d183` | build fixes (`boost` → `std` pointers) |
| GenZ-LIO | `c6c464d` | build fixes; Python bindings (`setup/install_genz_py.sh`) with a `seed_velocity_body` hook and a `GENZ_INIT_IMU_SAMPLES` override (default unchanged) |
| KISS-ICP | `kiss-icp` 1.x from pip | none |

**Problems found and fixed:**

| problem | cause | fix |
|---|---|---|
| Faster-LIO tracked 24 % of the distance | `point_filter_num: 4`: 1 024 columns ÷ 4 keeps the same azimuth columns in every ring; the radial stripes pin the scan to the map | `point_filter_num: 3`. Length ratio over 30 s: 0.24 → 1.004 for Faster-LIO, 0.974 → 1.006 for FAST-LIO. Evidence: `results/runs/faster_lio/smoke30` (4) vs `smoke30_pf3` (3). The legacy `fastlio.csv` matches a run with 4 (`runs/fast_lio/full_pf4_legacy`) |
| GenZ-LIO killed at 4 GB after ~500 scans | the authors' configs never prune the map (`map_range: -1`) | `map_range: 300` (2× sensor range) → steady 2.6 GB |
| DLIO publishes a zero-stamped pose | pose sent before initialisation | dropped in the recorder and the loader |
| one out-of-memory error killed the whole study | one cap over everything | each step capped separately |
| the laptop froze (~6 GB) | uncapped parallel jobs | every heavy job capped, one at a time |

**Determinism.** Faster-LIO (this sequential build) and GenZ-LIO are deterministic. FAST-LIO2 and
DLIO vary ±1.5 % and ±4 % in ATE between runs (§7.4). KISS-ICP inside a node is not exactly
repeatable; offline with a fixed input it is (§11).

---

## 5. v1: baselines, the tilt discovery, correctors, learned fusion, windows

*Run unattended 2026-10-10 (`scripts/run_all.sh`, `run_unattended.sh`). Generated report:
`lio_bench/results/REPORT.md`.*

### 5.1 Are the legacy CSVs real? (`liobench verify`)

| | `fasterlio.csv` | `fastlio.csv` |
|---|---|---|
| vs a fresh run of the same algorithm | median 14 m apart over 10.3 km, length ratio 0.993: **matches** | median 48 m apart, CSV 1.6 % longer: close |
| drift vs GT (frame-free) | 5.7 % | 8.2 % |
| timestamp offset | **+0.5–0.6 s late** | **+0.4–0.5 s late** |

**Verdict:** both are genuine algorithm outputs. The late stamps are worth ~10 m of apparent error
at highway speed; the evaluation corrects them. Figures: `lio_bench/results/verify/`.

### 5.2 Full-drive runs

| method | ATE (whole-drive best fit) | ATE (200 m start fit) final | length ratio | CPU | RAM |
|---|---:|---:|---:|---:|---:|
| Faster-LIO | 382 m | 3 006 m | 0.953 | 124 % | 437 MB |
| DLIO | 396 m | 2 699 m | 0.923 | 234 % | 3.1 GB |
| GenZ-LIO (ROS) | 455 m | 3 379 m | 0.911 | 216 % | 2.6 GB |
| FAST-LIO2 | 500 m | 3 581 m | 0.905 | 117 % | 615 MB |

- GenZ-LIO's offline Python runner and its ROS node agree (452 vs 447 m).
- 100 % CPU = one core.

### 5.3 The main discovery: tilt, not scale

Every horizontal track is 5–9 % short, yet each method's own 3D step lengths add up to 10 278 m vs
10 295 m of ground truth (0.2 %). The estimates slowly pitch up:

| FAST-LIO2 | km 0–2 | 2–4 | 4–6 | 6–8 | 8–10 |
|---|---:|---:|---:|---:|---:|
| estimated body pitch | +5° | +14° | +19° | +21° | +34° |
| estimated climb angle | +11° | +14° | +20° | +17° | +35° |
| real road slope | +0.2° | 0.0° | −1.0° | −2.6° | −1.3° |

- **Height error at km 10:** FAST-LIO +3.4 km, DLIO +3.2 km, GenZ +3.3 km, Faster-LIO +2.4 km.
- **The scene's weakest direction is along the road** in every kilometre (cos ≈ 0.98–1.00).
- **What's left once tilt is removed is heading drift.** Scale error is −0.17…+0.13 %; yaw drifts
  0.26–0.48° per 10 s.
- **FAST-LIO, Faster-LIO and GenZ-LIO fail together:** their step errors correlate at ρ 0.95–0.97.
  DLIO is noisier per step but has less yaw drift.

Figures: `lio_bench/results/analysis/`.

### 5.4 Running a baseline and a support together (concurrency)

| pair | baseline ATE solo → together | support ATE solo → together | baseline latency median / p95 | scans dropped |
|---|---|---|---|---:|
| FAST-LIO + DLIO | 500 → 493 m | 396 → 411 m | 39 / 57 ms | 4 |
| FAST-LIO + GenZ | 500 → 496 m | 450 → 450 m | 37 / 54 ms | 7 |
| Faster-LIO + DLIO | 382 → 382 m | 396 → 401 m | 24 / 35 ms | 7–8 |
| Faster-LIO + GenZ | 382 → 382 m | 450 → 458 m | 26 / 38 ms | 7 |

- **All pairs keep real time.** GenZ-LIO runs near its limit (p95 latency 88–96 ms against the
  100 ms scan period).
- **Laptop timings mean "fits", not exact numbers.** Performance/efficiency-core scheduling made
  concurrent runs measure faster than solo ones.

Data: `lio_bench/results/concurrency/`.

### 5.5 Correction strategies (Phase 2)

Per block, anchored on the block's own first 200 m; drift % per 200 m; test = last 20 % of the drive.

| strategy (FAST-LIO baseline) | train | val | test | ms/scan |
|---|---:|---:|---:|---:|
| FAST-LIO as is | 4.23 | 9.30 | 25.59 | — |
| + CAN wheel distance (tilted frame) | 5.27 | 9.55 | 25.39 | ~0 |
| **+ planar re-integration** | 1.29 | 1.63 | **0.85** | ~0 |
| + wheel + planar | 2.66 | 1.61 | 0.87 | ~0 |
| + point-to-point ICP + planar | 1.47 | 1.12 | 2.19 | 52 |
| + point-to-plane ICP + planar | 1.32 | 1.45 | 1.54 | 44 |
| + point-to-plane, degeneracy-aware + planar (selected on val) | 1.31 | 0.83 | 1.27 | 36 |
| + GICP + planar | 1.12 | 1.04 | 0.67 | 36 |
| + GICP degeneracy-aware + planar | 1.13 | 1.05 | 0.64 | 34 |
| + VGICP + planar | 1.24 | 1.64 | 0.63 | **8** |
| KISS-ICP + planar | 1.45 | 0.58 | 1.65 | 34 |
| DLIO + planar / GenZ-LIO + planar | 1.45 / 1.38 | 1.62 / 1.02 | 0.91 / 0.88 | — |

- **Faster-LIO:** 13.41 % → 1.20 % (planar) → 0.65 % (GICP/VGICP + planar).
- **Planar re-integration is the big free win.** ICP correctors only help on top of it.
- **Degeneracy-aware fusion and the wheel prior add nothing measurable.**
- **Selection on one drive is noisy:** the validation pick lost on test.

Tables: `lio_bench/results/correct/<baseline>/correct_summary.md`.

**Geometric pole detection failed.** Only 6 of 290 poles were ever matched, even with the true
pose: the detector found objects 10–36 m to the side, while the poles stand 1.5–6 m away at the
snowbank. The filter's uncertainty also grew like √distance instead of linearly. This was replaced
by the professor's YOLO detector in stage 3 (§8).

### 5.6 Learned fusion ("which model, what percent")

- **Model:** per-step masked-softmax weights over the methods' tilt-invariant steps, with causal
  context (scan geometry, IMU statistics, CAN, GenZ internals, agreement between methods).
- **Guards against overfitting one drive:**
  - a chronological split with 10 s gaps;
  - a model ladder chosen by blocked cross-validation with a paired one-standard-error rule;
  - a shuffled-context control;
  - 3 seeds;
  - availability dropout.

Held-out test block, integrated error / drift per 200 m:

| | FAST-LIO setup | Faster-LIO setup |
|---|---:|---:|
| baseline alone | 21.0 m / 0.66 % | 31.9 m / 0.94 % |
| static weights | 19.4 m / 0.59 % | 23.2 m / 0.73 % |
| learned (DLIO + GenZ supports, MLP-16) | 17.2 m / 0.44 % | 20.1 m / 0.44 % |
| shuffled-context control | 28.3 m / 0.57 % | 34.0 m / 0.70 % |
| learned with correctors (linear) | 11.4 m / 0.41 % | 27.7 m / 0.47 % |
| baseline + CAN wheel distance alone | 3.6 m / 0.49 % | 14.0 m / 0.58 % |

- **The reliability trigger** (predicts the baseline's error per metre over the next second) ranks
  better than a speed-only control: 0.78 vs 0.27 for FAST-LIO, 0.54 vs 0.39 for Faster-LIO.
- **Re-scored canonically in v2 (§6.5),** fusion's advantage is local drift only. It is not used in
  any final CSV, and it is trained on GNSS labels.

Data: `lio_bench/results/fusion/`.

### 5.7 Sliding windows (support only when the trigger fires)

| baseline + support, schedule | duty | baseline alone | support always on | support in windows |
|---|---:|---:|---:|---:|
| FAST-LIO + DLIO, adaptive cold | 16 % | 21.0 m / 0.66 % | 12.7 / 0.46 | 18.4 / 0.55 |
| FAST-LIO + GenZ, adaptive warm | 17 % | 21.0 / 0.66 | 18.2 / 0.56 | 23.4 / 0.51 |
| FAST-LIO + GenZ, adaptive cold | 17 % | 21.0 / 0.66 | 18.2 / 0.56 | 24.7 / 0.50 |
| FAST-LIO + GenZ, periodic 20 s / 60 s | 41 % | 21.0 / 0.66 | 18.2 / 0.56 | 19.3 / 0.64 |
| Faster-LIO + DLIO, adaptive cold | 10 % | 31.9 / 0.94 | 16.5 / 0.63 | 31.2 / 0.89 |
| Faster-LIO + GenZ, adaptive warm / cold / periodic | 10 / 10 / 41 % | 31.9 / 0.94 | 25.8 / 0.63 | 32.3 / 34.7 / 29.7 m |

- **Windows keep ~30 % of the support's benefit at ~16 % duty.** GenZ-LIO windows can make things
  worse.
- **At the 90th percentile the trigger never fired** on test.
- **Not pursued further.**

Data: `lio_bench/results/windows/`.

---

## 6. v2: audit: heading is the real error

*2026-10-11. Code `lio_bench/src/liobench/audit.py`; numbers `lio_bench/results/audit/audit.json`.*

### 6.1 The canonical protocol (used for v2, v3 and stage 3)

1. Clock offset estimated on the training block only.
2. **One SE(2) fit on the first 200 m** of the evaluated span (a stand-in for an initial GNSS fix).
3. Pure prediction after that.

Drift is the KITTI-style 100–800 m segment drift. 95 % CIs come from a moving-block bootstrap.

### 6.2 Ground-truth provenance

- **What it is.** The dual-antenna midpoint (GT − midpoint: median 3.4 cm), RTK float, receiver
  timing consistent (median 0.7 ms).
- **Jitter** against a 1 s moving average: median 7 cm.
- **Valid for:** metre-level drift over hundreds of metres.
- **Not valid for:** centimetre claims or per-step yaw. At 2 m steps, GT yaw has ~2° of noise per
  step, which causes a spurious −31 % "scale" in every method's per-step yaw.

### 6.3 Error budget (whole drive, ATE in m, counterfactual re-integration)

| method | as is | tilt removed | + true heading | + true forward length | heading at km 10 |
|---|---:|---:|---:|---:|---:|
| Faster-LIO | 1 245 | 917 | **29.7** | 25.3 | +32.6° |
| FAST-LIO | 1 486 | 749 | **31.6** | 27.3 | +25.1° |
| DLIO | 1 103 | 573 | **25.4** | 25.8 | +20.1° |
| GenZ-LIO | 1 409 | 798 | **29.1** | 27.4 | +23.1° |
| KISS-ICP | 563 | 372 | **27.9** | 25.2 | +12.6° |

- **Tilt costs 30–50 %.**
- **Heading is almost all of the rest (~97 % in total).**
- **Forward scale is right to 0.1–0.4 %.**

### 6.4 Planar ablation and the gravity-locked mechanism test

- **Pitch is the whole length problem.** Removing pitch alone restores length (0.953 → 0.997);
  removing roll alone does nothing.
- **Integrating yaw about the body's vertical** gains another 20–45 % of ATE.
- **The measured "up"** (1.7° from IMU z) is worth 8–13 %.

| Faster-LIO baseline | whole ATE | whole drift | length | test drift 200 m |
|---|---:|---:|---:|---:|
| Faster-LIO | 1 245 | 5.05 % | 0.953 | 13.3 % |
| VGICP corrector, raw | 1 242 | **8.40 %** | 0.916 | 23.9 % |
| **VGICP, gravity-locked (5 s filter)** | **711** | **1.37 %** | 0.998 | 0.72 % |
| **gravity only (no ICP)** | 965 | 1.43 % | 0.996 | 0.92 % |
| planar (offline) | 948 | 1.39 % | 0.997 | 0.94 % |
| VGICP + planar (offline) | 655 | 1.15 % | 1.001 | 0.48 % |

- **ICP "harm" is attitude drift.** Locking roll/pitch to gravity removes it, online.
- **Snapping the attitude every scan** made ICP fail on 54 % of scans; the 5 s filter: 0.5 %.

### 6.5 Heading drift is a bias, and v1 re-scored

- **The rate is constant per second, not per km.** Faster-LIO: 0.054°/s at ~10 m/s and 0.062°/s at
  ~21 m/s.
- **Every method drifts counter-clockwise.**
- **It is not concentrated in degenerate scenes and not driven by turning.**
- **Earth's rotation explains only 0.0037°/s.**
- **Re-scored canonically on the test block:**
  - KISS-ICP (7.9 m), FAST-LIO + wheel (7.6 m) and DLIO (9.7 m) match or beat the learned fusion
    (8.5–14.3 m) on ATE.
  - Fusion is best only at local drift (0.41–0.47 %).
  - v1's "−46 %" overstated it.
- **Bugs fixed:**
  - GenZ warm start used a central difference, 0.3 s non-causal; now a backward difference.
  - Window compute read 0 s for ROS supports; now CPU-seconds from the resource logs.

---

## 7. v3: heading, mechanisms, the final system

*2026-10-11. Code `lio_bench/src/liobench/v3.py`, `correct/heading.py`, `correct/kiss_probe.py`;
numbers `lio_bench/results/v3/`, `v3_*.json`; figures `lio_bench/results/v3/figures/f0–f5`.*

### 7.1 Online gravity filter: time constant

Drift per 200 m, chosen on the validation block:

| τ (s) | 0.5 | 1 | 2 | **5** | 10 | 20 | 50 | 100 |
|---|---|---|---|---|---|---|---|---|
| Faster-LIO val | .843 | .843 | .843 | **.842** | .844 | .850 | .897 | 1.126 |
| Faster-LIO test | .934 | .922 | .917 | **.919** | .928 | .953 | 1.148 | 1.910 |

Flat for 0.5–10 s; τ = 5 s.

### 7.2 Heading diagnosis

**Yaw-rate bias per ~2 km block (°/s):**

| source | b0 | b1 | b2 | b3 | b4 | mean | std |
|---|---:|---:|---:|---:|---:|---:|---:|
| Faster-LIO | .027 | .056 | .066 | .101 | .046 | .059 | .028 |
| FAST-LIO | .017 | .065 | .025 | .101 | .025 | .047 | .036 |
| DLIO | .024 | .040 | .005 | .114 | .037 | .044 | .042 |
| GenZ-LIO | .034 | .060 | .015 | .082 | .030 | .044 | .027 |
| VGICP-grav | .038 | .018 | .037 | .056 | .070 | .044 | .020 |
| **KISS-ICP** | .015 | .021 | .022 | .035 | .028 | **.024** | **.008** |

**Hypotheses, each with a direct test:**

| hypothesis | result |
|---|---|
| Earth rotation | ❌ removing it helps the gyro-free KISS-ICP as much as the LIOs: indistinguishable from removing a generic counter-clockwise bias |
| constant gyro bias | ❌ the LIO biases vary 3–20× between blocks |
| scene-driven leakage | ⚠ weak correlations (Faster-LIO only); none survive multiple-testing correction |
| prediction leakage into ICP | ⚠ partial: constant-velocity rotation cuts whole-drive ATE 711 → 285 m, but the per-block bias is unchanged |
| motion distortion (deskew) | ✅ strong counter-clockwise bias: KISS-ICP with deskew off ×4.5, deskew reversed ×7.4 |
| distortion in an LIO too | ✅ GenZ-LIO per-point times ×1 / ×0.5 / ×0: 0.045 → 0.100 → 0.148°/s |
| short gyro-bias initialisation | ❌ the 0.2 s initial bias (−0.00065 rad/s) is correct |
| online gyro-bias wander carries the drift | ❌ correlated r = 0.87, but freezing the bias (×1e-4 random walk) leaves the heading bias unchanged (0.0449 → 0.0452°/s) |
| IMU-noise tuning artefact | ❌ gyro-bias walk ×100 / IMU noise ×10: bias and climb unchanged |

**Pitch mechanism.**
- **LiDAR-only KISS-ICP climbs too** (+2 155 m), so the climb is a property of LiDAR registration
  on this road.
- **It is carried by ground returns beyond ~10 m.** Removing the ground collapses the climb to
  +101 m, but doubles the yaw bias. Removing only points under 10 m changes nothing.
- **Less deskew means less climb,** so leftover distortion does not cause the climb.
- **Leading hypothesis:** beam-elevation calibration, or the road/snowbank shape.

**Open:** why the LIOs' heading is 2× more biased than KISS-ICP's.

### 7.3 Leave-one-block-out results (5 blocks of ~2 km; paired against the gravity filter G)

| Faster-LIO baseline | ATE | Δ vs G ± SE | better |
|---|---:|---:|---:|
| raw | 72.3 | +26.1 ± 15.5 | 1/5 |
| planar (offline) | 45.1 | −1.1 ± 1.5 | 2/5 |
| **G: gravity filter (online)** | 46.2 | — | — |
| G + own yaw bias (calibrated on GNSS) | 21.4 | −24.8 ± 9.1 | 4/5 |
| **G + yaw from the least-biased source** (KISS-ICP, chosen in 5/5 folds) | **17.9** | **−28.3 ± 9.8** | **5/5** |
| VGICP every scan + KISS yaw | 16.4 | −29.6 ± 9.9 | 5/5 |
| G + KISS yaw + CAN distance | 18.2 | −27.8 ± 10.5 | 4/5 |

- **Selective correction doesn't pay.** VGICP every 20/10/5 scans gives +0.6/+0.7/+0.9 m; every
  2nd −3.2 m; every scan −9.6 m (direct) or −11.5 m (information-weighted).
- **CAN wheel speed isn't needed** once heading is fixed.
- **Calibrating a constant yaw bias** (on GNSS) helps but is fragile: it helps FAST-LIO in only 2/5
  blocks.

### 7.4 Significance (10 blocks of ~1 km), learned vs rule, repeatability

| pipeline | ATE / km block | Δ vs G | better | sign test |
|---|---:|---:|---:|---:|
| Faster-LIO + G | 12.7 | — | — | — |
| G + KISS yaw | 7.7 | −5.0 ± 2.3 | 9/10 | p = 0.021 |
| FAST-LIO G + KISS yaw | 7.4 | −5.3 ± 2.2 | 9/10 | p = 0.021 |
| KISS-ICP alone + gravity | 7.9 | −4.7 ± 2.4 | 8/10 | p = 0.11 |

- **Learned fusion can't fix heading,** because per-step GT yaw is mostly noise. Its real role is
  translation mixing: −7…−11 % local drift in 5/5 blocks with KISS heading. Not adopted.
- **Repeatability:** run-to-run spread is ±1.5 % (FAST-LIO) and ±4 % (DLIO) in ATE, and ±0.002°/s
  in bias.

### 7.5 Final-system table (10 held-out ~1 km blocks, each with its own 200 m start fit)

| pipeline | ATE / km block | better than baseline | drift / 200 m | heading error |
|---|---:|---:|---:|---:|
| **FAST-LIO2 raw (baseline)** | **51.2 m** | — | 8.52 % | 8.0° |
| FAST-LIO2 + gravity | 9.6 m | 10/10 | 0.66 % | 2.0° |
| FAST-LIO2 + gravity + KISS heading | 7.4 m | 10/10 (p = 0.002) | 0.62 % | 1.6° |
| Faster-LIO raw | 28.9 m | 8/10 | 4.15 % | 4.3° |
| Faster-LIO + gravity | 12.7 m | 10/10 | 0.77 % | 2.7° |
| Faster-LIO + gravity + KISS heading | 7.7 m | 10/10 | 0.68 % | 1.6° |
| DLIO raw → corrected | 48.6 → 7.8 m | 9/10 | 0.68 % | 1.7° |
| GenZ-LIO raw → corrected | 47.7 → 7.2 m | 10/10 | 0.55 % | 1.6° |
| KISS-ICP + gravity (alone) | 7.9 m | 9/10 | 0.80 % | 1.6° |
| **online node (Faster-LIO + G + KISS 2.0 m)** | **6.1 m** | — | **0.67 %** | **1.3°** |

**Heading blend: tested, not adopted.** LIO yaw for fast changes plus the KISS trend was worse than
pure KISS heading (Faster-LIO 8.8 vs 7.7 m).

### 7.6 KISS-ICP voxel size

| config | ms/scan | val ATE | test ATE |
|---|---:|---:|---:|
| 1.0 m (default for 100 m range) | 28.5 | 38.6 m | 9.4 m |
| 1.5 m | 15.9 | 18.9 m | 12.2 m |
| **2.0 m (chosen on validation)** | **11.0** | **11.2 m** | **8.7 m** |
| max range 50 m | 18.1 | worse (12.0 m on 10 blocks) | |

- **Cutting range hurts heading:** distant structure holds yaw.
- **The node needs 2.0 m to keep real time.** At 1.0 m it dropped 185 scans and built a 2.2 s
  backlog (10.0 m per km block, vs 6.1 m at 2.0 m).
- **The offline scan pass hard-coded 1.0 m.** That's KISS-ICP's default, and it ran before the
  sweep. That mismatch carried into the first option B CSVs (§10.2).

### 7.7 The real-time node (`lio_bench/ros/scripts/kiss_fusion_node.py`)

How it works, per LIO pose (causal):
1. Apply the LIO's step to its own attitude estimate.
2. Pull roll/pitch toward IMU gravity (τ = 5 s, with vehicle acceleration removed).
3. Keep only the level step (forward, sideways).
4. Replace the turn with KISS-ICP's turn for the same scan. Fall back to the LIO's turn if no scan
   is matched within 20 ms, or if the two disagree by more than 0.2 rad/s.
5. Integrate and publish `/kiss_fusion/Odometry`.

| | KISS 1.0 m | **KISS 2.0 m** |
|---|---:|---:|
| scans processed | 5 235 (185 dropped) | **5 419 (0 dropped)** |
| KISS-ICP time / scan, median / p95 | 51 / 109 ms | 38 / 54 ms |
| fallbacks (missing / guard) | 229 / 6 | 45 / 0 |
| pose latency, median / p95 | backlog to 2 178 ms | **47 / 72 ms** |
| node CPU / RAM | 122 % / 197 MB | **71 % / 148 MB** |

### 7.8 The 200 m-fit export (`liobench export`, `lio_bench/results/export/`)

- **Format:** each export keeps the template's 5 422 rows and stamps and replaces the positions.
- **GNSS used:** the clock offset from the training block and one SE(2) fit on the first 200 m
  (GNSS-assisted, see §9.3).
- **End errors:** FAST-LIO raw 3 715 m, Faster-LIO raw 3 110 m, Faster-LIO + G 2 327 m, Faster-LIO
  + G + KISS 891 m, online node 777 m.

---

## 8. Stage 3: the professor's snow-pole pipeline (200 m CSVs)

*Code `lio_bench/pipeline/`; tables `lio_bench/results/pipeline/PIPELINE_RESULTS.md`,
`RATE_SENSITIVITY.md`; figures `lio_bench/results/pipeline/figures/p1–p7`. These runs used the
**200 m-fit CSVs** (§7.8). They have not been redone with option B; the user is testing those.*

### 8.1 The re-implementation equals the original

`pipeline/validate_against_original.py` runs the unmodified `snowpole_based_vehicle_localization.py`.
Only the I/O is adapted: a lazy bag reader, pinned YOLOv5 v7.0, headless plots, and the Ouster SDK
1.0 name.

| | final native metric | pole fixes |
|---|---:|---:|
| original | 13.632403 m | 358 |
| re-implementation (`snowpole_stage3.py`) | 13.632396 m | 358 |

Frame by frame over 2 158 frames the difference is at most 7.4 × 10⁻⁵ m.

**Stage-3 variants:**
- **original:** position reset at poles;
- **+ heading reset:** a heading offset estimated from consecutive pole fixes at least 15 m apart,
  with gain 0.5 and updates over 15° rejected;
- **+ drift rate:** the offset's trend is also extrapolated after the last pole.

### 8.2 Results (24 runs: 8 inputs × 3 variants)

**Inside the pole span** (first → last pole, km 0.72–5.19):

| odometry | stage 3 | mean | median | max |
|---|---|---:|---:|---:|
| FAST-LIO2 raw | original | 10.6 m | 10.9 m | 33.9 m |
| FAST-LIO2 raw | + heading reset | 2.1 m | 1.0 m | 16.4 m |
| Faster-LIO raw | original | 10.2 m | 10.5 m | 28.3 m |
| Faster-LIO + G + KISS (offline) | original | 5.3 m | 3.7 m | 17.8 m |
| online node | original | 7.4 m | 9.0 m | 18.7 m |
| online node | + heading reset | **2.4 m** | **1.2 m** | **11.0 m** |
| legacy `incremental_navigation_results.csv` | original | 13.4 m | 8.1 m | 64.6 m |

**Whole drive (10.3 km), final error:**

| odometry | original | + heading reset | + reset + rate |
|---|---:|---:|---:|
| FAST-LIO2 raw | 2 912 m | 1 333 m | 639 m |
| Faster-LIO raw | 2 484 m | 1 087 m | 469 m |
| Faster-LIO + G + KISS (offline) | 672 m | 249 m | 130 m |
| online node | 526 m | 151 m | 7.2 m (fragile: ~7–50 m) |
| legacy CSV | 173 m | 1 230 m | 1 323 m |

The professor's own metric (frames inside the pole bounding box) gives nearly the same pole-span
numbers: raw ~10.2–10.5 m, offline G + KISS 5.3 m, node 7.4 m; with the heading reset every input is
~2.0–2.5 m.

### 8.3 Why there is a ~10 m floor: wrong-side pole association

- **The plateau is cross-track:** median |cross-track| 8.6–10.2 m, against 0.8–1.4 m along-track.
- **Poles stand in pairs:** relative to the GNSS track they sit at a median +6.4 m (left) and
  −2.4 m (right), i.e. 8.8 m apart across the road.
- **How the lock happens:** accumulated heading error pushes the predicted position until the
  opposite-side pole is the nearer one. The fix snaps to it, and the state stays locked on the
  wrong side.
- **The heading reset cuts the time spent locked** from 63–77 % to 12 % of pole-span frames. km
  4.0–4.4 still locks.
- **Next step, not implemented:** side-consistent association.

### 8.4 Drift-rate sensitivity

- **The node's 7.2 m sits at the bottom of a sharp V.** A rate offset of ±0.002°/s (~1 SE) gives
  ~21 m; ±0.005°/s gives ~50 m.
- **The fit window matters too:** fitting only the second half of the updates gives over 100 m.
- **Report it as ~7–50 m.**
- **The raw LIOs barely respond to the rate** (±8 %). Their after-pole error is not mainly heading.

### 8.5 Robust vs fragile

- **Robust:**
  - the heading reset at poles gives ~2 m mean / ~1 m median inside the pole span, for every input;
  - corrected odometry halves the original pipeline's pole-span error;
  - over the whole drive, gravity + KISS heading cut the final error 4–5×.
- **Fragile:** the drift-rate extrapolation after the last pole.
- **Caveat:** the stage-3 constants (15 m, 0.5, 15°) were set once, while looking at this drive.

---

## 9. Is our Faster-LIO real? The friend's 53 m. How much GNSS did we use?

### 9.1 Our Faster-LIO baseline is genuine

`lio_bench/verify/verify_faster_lio.py` is an independent numpy-only check: a 200 m fit plus a ±1 s
clock search. Output: `verify_faster_lio_output.txt`.

| trajectory | length ratio | error at km 2 / 5 / 10 | final |
|---|---:|---|---:|
| old `fasterlio.csv` | 0.945 | 60 / 458 / 2 588 m | 2 696 m |
| **our Faster-LIO** | 0.962 | 102 / 628 / 2 983 m | **3 113 m** |
| our FAST-LIO2 | 0.921 | 113 / 813 / 3 558 m | 3 717 m |
| old `fastlio.csv` | 0.920 | 104 / 859 / 3 794 m | 3 964 m |

- **Ours is slightly worse than the independent old export.** The two are a median 36 m apart.
- **The good results come from the corrections and pole resets,** not from the baseline.

### 9.2 The friend's Faster-LIO: 53.7 m inside the pole area

- **The friend's odometry is fine; its start heading was 4.46° off.** With a 200 m fit their file
  goes 53.7 → 9.0 m (`lio_bench/verify/align_nav_csv.py`, output `verify/faster_lio_full_aligned.csv`).
- **Their yaml extrinsic was also wrong,** but that barely matters. The correct values are in §1.1.
- **Which export steps matter** (`verify/export_ablation.py`, in-box mean):
  - **Don't matter:** clock shifts, 50–1 000 m fits, row jitter, repeated rows (9.4–10.4 m).
  - **Heading matters:** ±4° gives 7.9–11.9 m.
  - **Distance scale matters most:** ×0.95 gives 50 m, ×0.9 gives 153 m.
- **Start-heading options** (`verify/start_heading_options.py`, in-box mean; with the pole heading
  reset in brackets):
  - A, 200 m fit: 10.2 m (2.0 m);
  - B with the INS heading at t0: 13.0 m (2.4 m). The INS reads ~5° low, so it was replaced by the
    antenna heading;
  - friend's file as is: 53.7 m (55.9 m).
- **For the friend:** `verify/tum_to_nav_csv.py` turns their `traj.txt` into a pipeline CSV with
  option B (antenna heading at frame 1, no clock shift).

### 9.3 How much GNSS each result used

The user's point: a 200 m GNSS fit is help that pure odometry wouldn't have. Audit:

| result | GNSS used to place or score it |
|---|---|
| **Option B CSVs and scores (§10, §11)** | **One dual-antenna heading at frame 1** (car parked) and the GNSS position there. Nothing after. |
| 200 m-fit CSVs (`results/export/`) and all stage-3 runs (§8) | One SE(2) fit on the first 200 m **plus** a clock offset fitted against GNSS on the training block |
| v2/v3 block numbers (51.2 → 6.1 m, drift %, heading error) | **Each** ~1–2 km block gets its own 200 m fit: 5–10 fits per drive |
| Corrector tables (§5.5) | Each block's own first 200 m |
| v1 tables | `se2_head` = 200 m fit; `se2_full` = best fit over the whole drive (evaluation only); per-run clock offset |
| `verify_faster_lio.py` | 200 m fit + ±1 s clock search |

**Shared by every result, option B included:**
- **Design choices made once by comparing against GNSS** on training/validation blocks: τ = 5 s,
  KISS-ICP as the heading source, the 2.0 m voxel, `point_filter_num: 3`. The learned fusion was
  trained on GNSS labels, but it is in no final CSV.
- **The professor's pipeline itself** starts from GNSS frame 0, snaps to GNSS-surveyed poles, and
  is scored against GNSS.

---

## 10. Option B: GNSS for one heading at frame 1, nothing else

### 10.1 Definition (`lio_bench/pipeline/export_option_b.py`, `score_option_b.py`)

- **Rotation.** Each trajectory is rotated once, about its first pose, so its heading there equals
  the dual-antenna heading. That is row 0 of `gps_left/right_position.csv`: the baseline left → right
  turned 90° CCW = compass 84.01°, ENU 5.99°. The car is parked there.
- **Shift.** The track is shifted so its first pose sits on the GNSS position at that time (or on
  the template's row 0, for the pipeline CSVs: the pipeline uses only row-to-row steps anyway).
- **Nothing else.** No 200 m fit, no clock offset (the measured 0.06 s is recorded but not
  applied), no yaw-bias calibration.
- **Sampling.** Each pipeline CSV is sampled at the template's 5 422 GNSS stamps, with the FastReg
  columns cleared.
- **Applied rotations:** +5.88…+5.99° for every method. Faster-LIO never sees GNSS; the rotation is
  post-processing.

### 10.2 Round 1: Faster-LIO's four CSVs, re-run from the bag (2026-10-12)

`lio_bench/scripts/rerun_option_b.sh`, ~25 min:
1. Faster-LIO, real time: 9.5 min.
2. KISS-ICP scan pass: ~5.5 min.
3. Faster-LIO + node, real time: 9.5 min.

| file | at first pole | at last pole | end |
|---|---:|---:|---:|
| raw | 32.3 m | 784 m | 3 307 m |
| + gravity | 28.8 m | 587 m | 2 513 m |
| + gravity + KISS (offline, **1.0 m** KISS) | 20.7 m | 274 m | 792 m |
| online node (2.0 m KISS) | 21.3 m | 358 m | 962 m |

- **Raw and gravity were identical to the earlier runs,** pose for pose: Faster-LIO is
  deterministic.
- **The KISS-based files changed:** KISS-ICP isn't exactly repeatable here, and this pass started
  2 scans later.
- **The user spotted the 1.0 m vs 2.0 m voxel mismatch** between the offline file and the node
  (§7.6). Rebuilt with 2.0 m, the offline file ended 132 m off.

Those round-1 folders were superseded and removed in the clean-up; they are in git history.

### 10.3 Round 2: every method (2026-10-12)

- **Runs:**
  - Faster-LIO: `faster_lio/rerun_b`, node `kiss_fusion/rerun_b_fusion` (reused from round 1).
  - FAST-LIO2, DLIO, GenZ-LIO: fresh real-time bag runs, each together with its own node
    (`<lio>/optb`, `kiss_fusion_<lio>/optb`).
  - KISS-ICP alone, 2.0 m: fresh offline pass (`kiss_icp_vox20/optb`). It is also the heading source
    of every offline "+ KISS" row.
- **Scripts:** `lio_bench/scripts/run_option_b_all.sh`, `lio_bench/pipeline/score_option_b.py`,
  `timing_option_b.py`.
- **DLIO needed its own node.** DLIO publishes poses at 100 Hz from a timer, while the node assumes
  one pose per scan. The first DLIO node run reused scans: 49 m at the first pole, 423 guard
  fallbacks. It was discarded and re-run with `kiss_fusion_node_anyrate.py` +
  `kiss_fusion_100hz.yaml`, which apply each KISS-ICP turn once per scan (5 ms match window). Result:
  19.5 m at the first pole, 1 missing / 0 guard fallbacks. With one pose per scan it is the original
  rule.

**Option B, as placed, nothing fitted after frame 1:**

| method | variant | ATE RMSE | final | end-to-end drift | heading at end | first pole | last pole |
|---|---|---:|---:|---:|---:|---:|---:|
| Faster-LIO | raw | 1 391 m | 3 306 m | 32.1 % | +35.1° | 32.0 m | 784 m |
| Faster-LIO | + gravity | 1 064 | 2 512 | 24.4 % | +31.4° | 28.5 | 586 |
| Faster-LIO | + G + KISS (offline) | 115 | 132 | 1.3 % | +2.6° | 20.6 | 146 |
| Faster-LIO | online node | 464 | 962 | 9.3 % | +10.5° | 21.0 | 357 |
| FAST-LIO2 | raw | 1 643 | 3 895 | 37.8 % | +27.7° | 27.1 | 974 |
| FAST-LIO2 | + gravity | 912 | 2 098 | 20.4 % | +24.0° | 23.3 | 562 |
| FAST-LIO2 | + G + KISS (offline) | 114 | 132 | 1.3 % | +2.4° | 19.8 | 144 |
| FAST-LIO2 | online node | 458 | 949 | 9.2 % | +10.3° | 20.1 | 353 |
| DLIO | raw | 1 212 | 2 928 | 28.4 % | +20.2° | 19.4 | 660 |
| DLIO | + gravity | 760 | 1 833 | 17.8 % | +25.6° | 21.8 | 432 |
| DLIO | + G + KISS (offline) | 119 | 154 | 1.5 % | +3.0° | 19.2 | 146 |
| DLIO | online node | 278 | 589 | 5.7 % | +8.0° | 19.5 | 217 |
| GenZ-LIO | raw | 1 560 | 3 666 | 35.6 % | +26.8° | 30.8 | 950 |
| GenZ-LIO | + gravity | 915 | 2 063 | 20.0 % | +23.8° | 26.5 | 602 |
| GenZ-LIO | + G + KISS (offline) | 112 | 134 | 1.3 % | +2.5° | 20.0 | 141 |
| GenZ-LIO | online node | 274 | 580 | 5.6 % | +7.6° | 20.3 | 213 |
| KISS-ICP | alone, 2.0 m | 164 | 283 | 2.7 % | +2.6° | 23.1 | 187 |

**Relative accuracy and paper-style ATE:**
- **KITTI** is what KISS-ICP's paper reports. Like KITTI, each segment is matched to the reference
  at its own start, so it measures local tracking, not position.
- **Paper ATE** is what the LIO papers print. It fits the whole drive to GNSS, so it is evaluation
  only and is in no CSV.

| method | variant | KITTI translation | KITTI rotation °/100 m | RPE 1 s median / RMSE | paper ATE |
|---|---|---:|---:|---:|---:|
| Faster-LIO | raw / + G / + G + KISS / node | 4.42 / 1.66 / 1.26 / 1.27 % | .591 / .385 / .273 / .285 | 0.43/1.28, 0.09/0.23 (G, KISS, node) | 355 / 253 / 45 / 51 m |
| FAST-LIO2 | same order | 8.28 / 1.48 / 1.25 / 1.25 % | .955 / .342 / .280 / .287 | 0.83/2.77, 0.09/0.18 | 453 / 192 / 45 / 49 m |
| DLIO | same order | 7.82 / 1.37 / 1.23 / 1.13 % | .642 / .349 / .299 / .280 | 0.81/2.62, 0.23/0.37 | 414 / 181 / 49 / 22 m |
| GenZ-LIO | same order | 7.66 / 1.42 / 1.17 / 1.09 % | .861 / .342 / .277 / .259 | 0.74/2.59, 0.09/0.15–0.16 | 405 / 164 / 46 / 24 m |
| KISS-ICP | alone | 2.04 % | .323 | 0.21 / 0.46 | 40 m |

**Runtime** (real-time replay, laptop; indicative only; `timing_option_b.py`):

| LIO | LIO CPU / RAM | LIO latency median / p95 | node CPU / RAM | node latency | KISS in node, ms/scan | scans processed |
|---|---:|---:|---:|---:|---:|---:|
| Faster-LIO (solo / with node) | 147 / 112 % · 409 / 432 MB | 36/48 · 32/45 ms | 68 % / 150 MB | 45 / 64 ms | 39 / 57 | 5 419 / 5 419 |
| FAST-LIO2 | 96 % / 625 MB | 43 / 59 ms | 68 % / 180 MB | 49 / 65 ms | 36 / 53 | 5 419 |
| DLIO | 216 % / 3 074 MB | n/a (100 Hz timer) | 74 % / 164 MB | 41 / 114 ms | 25 / 46 | 5 419 |
| GenZ-LIO | 217 % / 2 557 MB | 80 / 100 ms | 69 % / 149 MB | 82 / 111 ms | 30 / 50 | 5 419 |

KISS-ICP alone offline (2.0 m, 4 threads): 11 ms/scan, 1.9 min for the 9 min drive.

Full tables: `lio_bench/results/optionB_all/SCORES.md`; numbers `scores.json`; figures in §E.

### 10.4 The CSVs to feed the professor's code

`lio_bench/results/export_optionB_all/incremental_navigation_<lio>_<variant>.csv`, 5 422 rows each,
each with a `.json` note:
- `<lio>` ∈ `faster_lio`, `fast_lio`, `dlio`, `genz_lio`;
- `<variant>` ∈ `raw`, `gravity`, `gk` (+ gravity + KISS heading offline), `node` (online node);
- plus `incremental_navigation_kiss_icp_alone.csv`.

Point line 28 of `snowpole_based_vehicle_localization.py` at a file.

**Caveats:**
- **Every file is 19–32 m off at the first pole.** The 200 m versions were 7–15 m off. That is more
  than half the 18 m pole spacing, so the original pipeline may lock onto a wrong pole.
- **The `gk` files of Faster-LIO, FAST-LIO2 and GenZ-LIO are on the optimistic side.** All three
  were built on the same favourable KISS-ICP pass (§11).

---

## 11. Offline vs online, and the randomness test

**Observation.** With the same rule and the same 2.0 m KISS-ICP, the offline Faster-LIO + G + KISS
ended 132 m off and the online node 962 m off. Local accuracy was the same (KITTI 1.26 vs 1.27 %).

**How the two versions work.** The rule is identical:
- distance from the LIO step after the gravity filter;
- turn from KISS-ICP for the same scan;
- heading = previous + turn; position = previous + the step along that heading.

Offline does it after the drive from recorded runs; the node does it live.

**Isolating the cause:**

| test | Faster-LIO + G + KISS, end | KITTI |
|---|---:|---:|
| offline KISS pass (`kiss_icp_vox20/optb`; repeated: identical) | 132 m | 1.26 % |
| offline, but using the Faster-LIO run made next to the node | 134 m | — |
| KISS-ICP fed the node's way, offline, 1 / 2 / 4 threads (identical to each other) | 592 m | 1.17 % |
| node's way + points beyond 100 m dropped first | 712 m | 1.15 % |
| live node | 962 m | 1.27 % |

- **Same points, same order.** Neither the LIO run, the thread count nor real time is the cause.
  Tiny input differences make the two KISS-ICP runs separate after 134 scans, and the heading gap
  then grows over 10 km.
- **Script:** `lio_bench/verify/kiss_input_test.py`.

**Randomness test.** Each reader ran 5 times, each run dropping a different random 0.1 % of the
points of every scan (physically meaningless). Each run was scored as Faster-LIO + G + KISS.
Scripts: `lio_bench/verify/kiss_randomness_test.py`, `scripts/run_kiss_randomness.sh` (~22 min);
numbers `results/optionB_all/kiss_randomness.json`.

| reader | run 1 | 2 | 3 | 4 | 5 | mean | KITTI mean | heading at end |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| offline pass's | 594 m | 155 m | 759 m | 336 m | 648 m | **498 m** | 1.21 % | +1.0…+9.8° |
| node's | 546 m | 346 m | 263 m | 767 m | 483 m | **481 m** | 1.30 % | −5.2…+7.2° |

**Conclusion:**
- **Offline and online are equivalent.** 132 m was a favourable draw; 962 m an unfavourable one.
- **Honest figure for the method on this drive:** ~500 m end error after 10.3 km (~5 %), range
  ~130–960 m, with ~1.2–1.3 % KITTI error.
- **Some heading bias remains** (mostly +4…+10° counter-clockwise by the end).
- **Compare methods by KITTI %, or by several runs,** never by one end error.
- **This also weakens "2.0 m beats 1.0 m" based on end error** (132 vs 792 m). 2.0 m still stands on
  its validation result and on real-time speed.

---

## 12. Conclusions, claim status, limitations, open work

### 12.1 Claims

| claim | status |
|---|---|
| All LIOs fail the same way here: pitch drift → fake climb → short tracks | **holds** (§5.3, §6.4; KITTI raw 4–8 %) |
| The pitch-up drift exists in LiDAR-only registration too and is carried by far ground returns | **holds** (§7.2) |
| Gravity correction is the biggest single fix | **holds** (KITTI ~8 % → ~1.4 %, length ratio → 1.00) |
| Heading drift is ~97 % of what is left; it is a counter-clockwise bias | **holds** (§6.3, §6.5) |
| Motion distortion drives a counter-clockwise heading bias, in KISS-ICP and in GenZ-LIO | **holds** (controlled probes, §7.2) |
| KISS-ICP heading fixes most of the remaining error | **holds** (9/10 blocks, p = 0.021; option B end error ~2 000 m → ~500 m) |
| After both fixes, the choice of LIO doesn't matter; use Faster-LIO (cheapest) | **holds** |
| The online node keeps real time | **holds** (0 dropped scans, ~45 ms) |
| Heading reset at poles: ~10 m → ~2 m in the pole span; the floor is wrong-side association | **holds for the 200 m CSVs**; option B not tested yet |
| The online node beats the offline version | **dropped** (equivalent, §11) |
| 2.0 m voxels beat 1.0 m on end error | **weakened** (inside the spread); 2.0 m kept for its validation result and real time |
| Selective / learned / windowed correction pays off | **not supported** (§5.6, §5.7, §7.3) |
| Drift-rate extrapolation gives 7 m after the last pole | **fragile**: report ~7–50 m |
| Why the LIOs are 2× more heading-biased than KISS-ICP | **open** |

### 12.2 How to report it

- **Main number:** KITTI translation % (and rotation °/100 m) under option B.
- **10 km end error:** a range (~130–960 m; typical ~500 m), not one value.
- **Inside the pole span:** the stage-3 numbers.
- **Label every result** with how much GNSS it used (§9.3).

### 12.3 Limitations

- **One drive, one sensor.** Every claim is "on this drive".
- **Ground truth is the car's own GNSS** (RTK float): fine at metre level, not per-step yaw.
- **Laptop timings are indicative;** a paper's runtime table should come from the target computer.
- **Stage-3 constants** were chosen while looking at this drive.
- **The legacy FastReg CSV** beats ours over the whole drive with the original stage 3 (173 m
  final), though ours are better inside the pole span and everywhere once the heading reset is on.

### 12.4 Open work (ranked)

1. **Run the professor's pipeline on the option B CSVs.** Watch the first-pole lock.
2. **Side-consistent pole association** in stage 3. It targets the ~10 m floor.
3. **A second drive** (or public winter data) to show the findings generalise.
4. **Why the LIOs' heading is 2× more biased than KISS-ICP's.** Swap residual types or robust
   kernels.
5. **Beam-elevation calibration check** for the ground-carried pitch bias.
6. **A robust drift-rate estimator** for after the last pole.
7. **Runtime on the target computer;** optionally a C++ port of the node.

---

## Appendix A: methods and equations

Notation:
- `T_k ∈ SE(3)`: IMU body pose at scan k in an estimator's world; `R_k, p_k` its rotation and
  position;
- `D_k = T_{k-1}⁻¹ T_k`: the body-frame step;
- `u_b`: the vehicle's "up" in the IMU frame (training-block mean specific force, 1.71° from IMU z);
- level axes: `f = normalize(e_x − (e_x·u_b)u_b)`, `l = u_b × f`.

**A.1 Clocks.** `unix = s + offset + drift·(s − s0)` is fitted on the lower envelope of IMU
(record − stamp); 105 ppm drift, residual 0.14 ms (`bag.fit_sensor_clock`). The per-method clock
residual is found by matching speed profiles against GT (±1 s, parabolic refinement), on the
training block only (`metrics.estimate_time_offset`, `audit.train_offset`). Option B applies none.

**A.2 Planar re-integration (tilt-invariant steps)** (`correct/planar.py`):

```
d_k   = R_{k-1}^T (p_k − p_{k-1})          body-frame step
fwd_k = d_k·f,  lat_k = d_k·l              level components
ψ̇_k   = rotvec(R_{k-1}^T R_k)·u_b          heading change about the vehicle's vertical
x_k   = x_{k-1} + Rot(ψ_{k-1})[fwd_k, lat_k]
```

**A.3 Online gravity filter** (`correct/heading.py::gravity_filter`, `scanpass.tilt_correct`, node):

```
v      = |d_k|/Δt ;  a_b = (dv/dt) e_x + ω × (v e_x)
u_meas = normalize(mean(f_imu over 1 s) − a_b)        (if |·| > 5 m/s²)
R_k   ← R_k · Rot((R_k^T z) × u_meas, α·∠(R_k^T z, u_meas)),   α = Δt/τ, τ = 5 s
```

Heading is untouched by this filter.

**A.4 Heading from another source** (`correct/heading.py::integrate`, `v3.combo`):

```
ψ_k = ψ_0 + Σ ψ̇_src
x_k = x_{k-1} + Rot(ψ_{k-1})[fwd, lat]
```

- ψ̇_src is the KISS-ICP turn resampled onto the LIO stamps (`resample_dyaw`).
- Options, **off in all option B results:** yaw-bias removal (`b` = slope of source heading − GT
  course, from training blocks), Earth rate, no side-slip.

**A.5 Real-time node.** Same as A.3 + A.4 per LIO pose (§7.7). KISS-ICP poses are mapped to the IMU
frame with `t_IL`, and rotations are re-projected onto SO(3) every step.
`kiss_fusion_node_anyrate.py` (DLIO): between scans the heading follows the LIO; at each scan it is
set to heading(previous scan) + KISS turn since then, with the guard comparing against the LIO's
accumulated turn.

**A.6 Scan-pass correctors** (`correct/scanpass.py`):
- **Input:** points → IMU frame, deskewed to scan end with the baseline poses (20 time bins),
  downsampled 0.5 m.
- **Registration:** ICP / plane-ICP / GICP to a local map (0.5 m voxels, LRU 100 scans, ≤ 80 m), or
  VGICP to a Gaussian voxel map (1 m).
- **`_da` (degeneracy-aware):** `δ' = (I + Λ)⁻¹ I δ`.
- **`_wheel`:** the forward component comes from CAN.
- **`_grav`:** adds A.3.
- **`_everyN`:** register on every N-th scan only.
- **`_cv`, `_cvrot`:** constant-velocity prediction.
- **Fallback** to the prediction when not converged, < 200 inliers, or |δt| > 1 m / |δθ| > 2°.

**A.7 Wheel model** (`correct/wheel.py`). Lag, scale and forward axis are fitted on the training
block; `t' = t − (t·f)f + d_CAN f`.

**A.8 Fusion model** (`fusion.py`, `dataset.py`).
- **Targets:** GT steps `[fwd, lat, ψ̇]`.
- **Output:** two masked softmaxes over methods.
- **Loss:** Huber per step + 0.1·Huber over 2 s windows.
- **Model ladder:** best single → static inverse-variance → linear → MLP-16 → MLP-64, chosen by
  5-fold blocked CV with the paired one-SE rule.

**A.9 Evaluation.**
- **Canonical (§6.1):** 200 m SE(2) fit, then prediction.
- **Option B (§10.1):** one rotation + one shift at frame 1.
- **ATE:** RMSE of horizontal error.
- **End-to-end drift:** final error / GT path length.
- **KITTI translation:** for starts every 10 GT samples and L ∈ {100…800 m}, rotate the estimated
  segment so its first 10 m chord matches GT, then take the end-point error / L.
- **KITTI rotation:** |Δcourse_est − Δcourse_GT| / L.
- **RPE 1 s:** the same per-start matching over 1 s.
- **Paper ATE:** RMSE after the best rigid 2D fit over the whole drive (evaluation only).
- **Heading at end:** course over 20 m (50 m in §11), estimate vs GT.

**A.10 Leave-one-block-out** (`v3.py`). K = 5 or 10 equal-time blocks. Calibrations and selections
use the other blocks; the held-out block is scored canonically. Comparisons are paired: Δ ± SE,
number of blocks better, exact sign test.

**A.11 Stage 3** (`pipeline/snowpole_stage3.py`).
- **Detection:** YOLOv5 (repo checkpoint, conf 0.6) on the signal image; nearest LiDAR point in the
  box via `geoloc_utils`; cached.
- **Localisation:** the original loop: heading and step from consecutive rows; pole fixes inside the
  pole bounding box for detections ≤ 5 m; nearest surveyed pole; vehicle placed at the
  back-azimuth.
- **Heading reset:** `θ ← θ + 0.5·d` for fixes at least 15 m apart, |d| ≤ 15°.
- **Rate:** OLS slope of θ(t) after ≥ 5 updates spanning ≥ 60 s.
- **Reference:** GNSS midpoint kriged to the frame stamps.

---

## Appendix B: where GNSS enters (leakage audit)

| place | GNSS used? |
|---|---|
| estimators (FAST-LIO2, Faster-LIO, DLIO, GenZ-LIO, KISS-ICP), the nodes | **no**: only LiDAR + IMU topics are replayed |
| gravity filter, planar, ICP correctors | no |
| body "up" `u_b` | no (IMU only) |
| option B placement | **one antenna heading + position at frame 1** |
| canonical / 200 m-fit placement | first 200 m of each evaluated span + training-block clock offset |
| wheel lag/scale, mounting yaw | training block (calibration) |
| yaw-bias calibration (v3 only, never in option B) | training blocks |
| fusion labels, model selection | train / val |
| hyper-parameter choices (τ, voxel, heading source) | train / val, once |
| all metrics, decomposition, CIs | evaluation only |

---

## Appendix C: every CSV

Paths are relative to the repo root. The professor's repo files are marked (prof).

**Inputs:**

| file | what |
|---|---|
| `incremental_navigation_results.csv` (prof) | the pipeline's odometry CSV (FastReg era), "legacy CSV"; the template for all our exports |
| `Groundtruth_pole_location_at_test_site_E39_Hemnekjølen.csv` (prof) | 290 surveyed poles |
| `groundtruth_extracted.csv` | ground truth, UTM-33N, 10 Hz |
| `gps_left_position.csv`, `gps_right_position.csv` | the two antennas (row 0 → option B heading) |
| `fasterlio.csv`, `fastlio.csv` | legacy LIO exports (§3, §5.1). `fastreg.csv` is ignored and not included |

**Pipeline-format outputs** (drop-in for `incremental_navigation_results.csv`, 5 422 rows, each with
a `.json`):

| folder | what |
|---|---|
| **`lio_bench/results/export_optionB_all/`** | **option B, 17 files** (§10.4). Use these |
| `lio_bench/results/export/` | 200 m-fit versions (7 files: FAST-LIO / Faster-LIO raw, gravity, gravity + KISS (1.0 m), online node). Used by §8 |
| `lio_bench/verify/faster_lio_full_aligned.csv` | the friend's file after a 200 m fit (§9.2) |

**Stage-3 outputs** (`lio_bench/results/pipeline/`): `<input>__<mode>_frames.csv`.
- **Inputs:** `fastlio_raw`, `fasterlio_raw`, `fastlio_g`, `fasterlio_g`, `fastlio_gk`,
  `fasterlio_gk`, `final_online`, `legacy_csv`.
- **Modes:** `none`, `reset`, `reset_rate`.
- **Columns:** `t`, `pred_e`, `pred_n`, `ref_e`, `ref_n`, `err_m`, `in_box`, `s_km`.
- **Also:** `<run>_heading_updates.csv` (`measured_deg`, `accepted`, `theta_deg`, `rate_deg_s`,
  `baseline_m`), `<run>_summary.json`, `detections_cache.pkl`.

**Raw runs** (`lio_bench/results/runs/<algorithm>/<tag>/`):
- **Files:**
  - `odom_sensor_clock.tum` (sensor clock) and `trajectory_tum.txt` (Unix): the trajectory, as
    `t x y z qx qy qz qw`;
  - `odom_recv.csv`, `cloud_recv.csv`: latency and drops;
  - `resources.csv`: CPU/RAM;
  - `diagnostics.csv`;
  - `meta.json`, `node.log`.
- **Tags:**
  - `full` = whole drive; `full_r2/r3` = repeats; `concurrent_A+B`; `win_*` = sliding windows;
  - `full_bgyr100`, `full_imunoise10`, `full_bgyrfrozen`, `full_bias`, `full_ts0`, `full_ts0.5`,
    `full_pf4_legacy` = probes;
  - `smoke30` / `smoke30_pf3` = the `point_filter_num` evidence;
  - `rt_fusion` / `rt_fusion_v20` = node at 1.0 / 2.0 m;
  - `rerun_b`, `rerun_b_fusion`, `optb` = option B.
- **Algorithm folders:**
  - `<lio>+<corrector>`: correctors;
  - `kiss_icp_<probe>`: KISS-ICP probes and the voxel sweep;
  - `kiss_icp_nodeinput`: §11 input test;
  - `kiss_icp_rand_{offline,node}`: §11 randomness test;
  - `kiss_fusion[_<lio>]`: nodes.

**Other analysis CSVs** (`lio_bench/results/`):

| file | what |
|---|---|
| `bag/utm_position.csv` | GNSS/INS position from the bag |
| `bag/enu_heading.csv` | the INS heading topic (unreliable, §1.1) |
| `bag/gps_state.csv` | fix status, antenna heading |
| `bag/steering.csv` | CAN speed and steering |
| `features/scan_context.csv` | per-scan scene descriptors |
| `analysis/window_errors.csv` | 10 s window errors with scene features |
| `fusion/*/weights_holdout.csv`, `trigger_predictions.csv` | learned weights over time; the trigger's predictions |
| `poles/detections_*.csv` | geometric pole candidates (§5.5) |
| `v3/leak_scene_windows.csv` | yaw-bias leakage test |

---

## Appendix D: every script

Python runs in the `lio-py` conda env and ROS in `lio-ros`. Heavy jobs go through
`MEM=XG bash lio_bench/scripts/capped.sh <cmd>`. The CLI is `liobench <command>`
(`lio_bench/src/liobench/cli.py`).

**Setup:**

| script | what |
|---|---|
| `lio_bench/envs/lio-py.yml`, `lio-ros.yml` | conda environments |
| `lio_bench/setup/bootstrap.sh` | one-shot setup: Miniforge, envs, ROS workspace, GenZ Python bindings, `liobench` |
| `lio_bench/setup/build_ros_ws.sh` | clones the estimators at pinned commits, applies the commented patches, builds |
| `lio_bench/setup/install_genz_py.sh` | GenZ-LIO Python bindings (+ `seed_velocity_body`, `GENZ_INIT_IMU_SAMPLES`) |
| `lio_bench/setup/common.sh` | shared shell helpers |
| `lio_bench/third_party_patches/*.pin`, `*.diff` | upstream URL + commit, and our diff, per estimator |
| `lio_bench/configs/dataset.yaml` | bag, topics, ground truth, evaluation settings, split |
| `lio_bench/configs/algorithms/*.yaml` | estimator configs (`point_filter_num: 3`, GenZ `map_range: 300`, Trip068 extrinsics), node configs `kiss_fusion.yaml` (2.0 m) and `kiss_fusion_100hz.yaml` |
| `lio_bench/configs/algorithms/registry.yaml` | what `run_ros.py` can launch, incl. `kiss_fusion`, `kiss_fusion_{fast_lio,dlio,genz_lio}` |
| `lio_bench/configs/algorithms/variants/*.yaml` | probe configs (`pf4_legacy`, `bgyr100`, `imunoise10`, `bgyrfrozen`, `upstream_ouster64`) |

**Running estimators:**

| script | what |
|---|---|
| `lio_bench/scripts/run_ros.sh` → `ros/scripts/run_ros.py` | own roscore; replays only LiDAR + IMU in real time; launches estimators/nodes; monitors CPU/RAM; drains; writes `results/runs/<algo>/<tag>/` |
| `ros/scripts/odom_recorder.py` | records poses to TUM; logs arrival times |
| `ros/scripts/kiss_fusion_node.py` | **the final online node** |
| `ros/scripts/kiss_fusion_node_anyrate.py` | the node for 100 Hz LIOs (DLIO) |
| `ros/launch/*.launch` | launch files for the four LIOs |
| `src/liobench/genz_runner.py` (`liobench run-genz`) | GenZ-LIO offline from the bag; probes (`--time-scale`) |
| `src/liobench/bag.py` (`liobench prepare`) | clock fit; extracts INS heading, CAN, GNSS state |

**Library** (`lio_bench/src/liobench/`):
- `io.py`: readers/writers;
- `geometry.py`: alignment, increments;
- `metrics.py`: ATE, RPE, KITTI drift, clock offset;
- `plots.py`;
- `dataset.py`: step table for fusion;
- `scan_features.py`;
- `correct/common.py`.

**Study steps:**

| script | what | section |
|---|---|---|
| `verify.py` (`liobench verify`) | legacy CSVs vs GT and re-runs | §5.1 |
| `analysis.py` (`liobench analyze`) | 10 s window analysis, maps | §5.3 |
| `concurrency.py` (`liobench concurrency`) | solo vs concurrent | §5.4 |
| `correct/planar.py`, `wheel.py`, `scanpass.py`, `poles.py`, `report.py` (`liobench correct …`) | correctors, KISS-ICP pass, geometric poles, per-block report | §5.5 |
| `fusion.py` (`liobench fusion`) | learned fusion + trigger | §5.6 |
| `windows.py` (`liobench windows`) | sliding windows | §5.7 |
| `report_all.py` (`liobench summary`) | writes `results/REPORT.md` | §5 |
| `audit.py` | canonical protocol, budget, ablation, provenance | §6 |
| `correct/heading.py` | gravity filter, heading integration, blend | §7 |
| `v3.py` | leave-one-block-out, combos, learned vs rule | §7 |
| `correct/kiss_probe.py` (`liobench kiss-probe`) | KISS-ICP probes and voxel sweep | §7.2, §7.6 |
| `export.py` (`liobench export`) | 200 m-fit pipeline CSVs | §7.8 |
| `scripts/make_v3_figures.py` | v3 figures | §7 |
| `scripts/run_all.sh`, `run_unattended.sh` | v1, unattended | §5 |
| `scripts/run_phaseA_passes.sh` | v2 gravity-locked passes | §6 |
| `scripts/run_v3_queue*.sh` (1–7), `run_kiss_sweep.sh` | v3 heavy jobs: schedules, GenZ bias, leakage, deskew, IMU noise, KISS geometry, frozen bias; voxel sweep | §7 |

**Stage 3** (`lio_bench/pipeline/`):

| script | what |
|---|---|
| `snowpole_stage3.py` | `detect` (YOLO + LiDAR, cached) and `run <csv> <mode>` (original loop + heading corrector + metrics) |
| `validate_against_original.py` | runs the unmodified original script; compares |
| `run_all_variants.py` | 24 runs → `PIPELINE_RESULTS.md` |
| `rate_sensitivity.py` | drift-rate sensitivity → `RATE_SENSITIVITY.md` |
| `make_pipeline_figures.py` | figures p1–p7 |

**Option B and the checks** (`lio_bench/pipeline/`, `lio_bench/verify/`, `lio_bench/scripts/`):

| script | what |
|---|---|
| `verify/verify_faster_lio.py` | numpy-only check of our Faster-LIO vs the legacy exports (§9.1) |
| `verify/diagnose_nav_csv.py` | any pipeline CSV: distance at km 0.2/0.5/0.72/1.0, in-box metric, along/cross-track split |
| `verify/export_ablation.py` | which export step matters (§9.2) |
| `verify/align_nav_csv.py` | 200 m fit of any pipeline CSV (the friend's file) |
| `verify/start_heading_options.py` | 200 m fit vs one heading vs as is (§9.2) |
| `verify/tum_to_nav_csv.py` | TUM trajectory → pipeline CSV with option B (for the friend) |
| `scripts/rerun_option_b.sh` | fresh Faster-LIO, KISS-ICP and Faster-LIO + node runs (§10.2) |
| `pipeline/export_option_b.py` | option B helpers (antenna heading, yaw); first-round exporter |
| `scripts/run_option_b_all.sh` | KISS-ICP 2.0 m + FAST-LIO2/DLIO/GenZ-LIO with their nodes (§10.3) |
| `pipeline/score_option_b.py` | option B placement, all metrics, 17 CSVs, figures → `results/optionB_all/` |
| `pipeline/timing_option_b.py` | runtime table into `SCORES.md` |
| `verify/kiss_input_test.py` | KISS-ICP fed the node's way, offline (§11) |
| `verify/kiss_randomness_test.py`, `scripts/run_kiss_randomness.sh` | randomness test (§11) |

**Preliminary study** (`odometry_refinement/`): `scripts/audit_data.py`, `evaluate_baselines.py`,
`extract_features.py` (regenerates `experiments/features/`, not kept), `train_reliability.py`; tests
`tests/test_core.py`; package `src/odom_refinement/`; constants in `configs/`.

**Tests:** `lio_bench/tests/test_liobench.py` has 23 unit tests on synthetic data covering
geometry, metrics, increments and alignment (all pass).

---

## Appendix E: every figure

| figure | shows | section |
|---|---|---|
| `experiments/baseline/*.png` (7) | preliminary baseline of the exports | §3.2 |
| `lio_bench/results/verify/map_none.png`, `map_se2_head.png`, `map_se2_full.png`, `error_vs_distance_se2_head.png`, `error_vs_distance_se2_full.png` | legacy CSV check | §5.1 |
| `lio_bench/results/analysis/scale_error_vs_time.png`, `yaw_error_vs_time.png`, `map_{fast_lio,faster_lio,dlio,genz_lio}_step_error.png`, `map_best_method.png` | tilt discovery, where methods fail | §5.3 |
| `lio_bench/results/correct/{fast_lio,faster_lio}/drift_by_block.png` | correctors per block | §5.5 |
| `lio_bench/results/fusion/<setup>/test_block_trajectories.png`, `translation_weights_vs_time.png`, `map_baseline_weight.png` | learned fusion | §5.6 |
| `lio_bench/results/windows/<base>/<variant>/fused_trajectories.png`, `transient_after_window_start.png` | sliding windows | §5.7 |
| `lio_bench/results/v3/figures/f0_map.png` … `f5_bias_per_block.png` | map, height profile, heading error, KISS probes, duty cycle, bias per block | §7 |
| `lio_bench/results/pipeline/figures/p1_validation.png` … `p7_cross_track.png` | stage 3: validation, error vs distance, pole span, all runs, rate sensitivity, map, wrong-side lock | §8 |
| `lio_bench/results/pipeline/error_vs_distance__{none,reset,reset_rate}.png` | stage 3 per mode | §8 |
| `lio_bench/results/optionB_all/figures/error_vs_distance.png` | option B error along the drive, per LIO | §10.3 |
| `lio_bench/results/optionB_all/figures/maps.png` | option B tracks vs GNSS | §10.3 |
| `lio_bench/results/optionB_all/figures/kitti_translation.png`, `end_to_end_drift.png` | option B bar charts | §10.3 |

---

## Appendix F: generated result files

These are written by scripts and kept next to their data. This report summarises them.

| file | written by |
|---|---|
| `lio_bench/results/REPORT.md` | `liobench summary` (v1, everything) |
| `lio_bench/results/verify/verify_summary.md`, `analysis/analysis_summary.md` | `liobench verify`, `liobench analyze` |
| `lio_bench/results/correct/<base>/correct_summary.md` | `liobench correct report` |
| `lio_bench/results/fusion/<setup>/fusion_summary.md` | `liobench fusion` |
| `lio_bench/results/windows/<base>/<variant>/windows_summary.md` | `liobench windows` |
| `lio_bench/results/audit/audit.json`, `results/v3/*.json`, `results/v3_*.json` | `audit.run()`, `v3.run()` |
| `lio_bench/results/pipeline/PIPELINE_RESULTS.md`, `RATE_SENSITIVITY.md`, `all_results.json` | stage-3 scripts |
| `lio_bench/results/optionB_all/SCORES.md`, `scores.json`, `kiss_randomness.json` | option B scripts |
| `lio_bench/verify/*_output.txt` | the verify scripts |
| `lio_bench/results/logs/` | every run's log |
| `reports/dataset_statistics.json`, `experiments/baseline/baseline_summary.json`, `experiments/reliability/reliability_summary.json` | preliminary study |

---

## Appendix G: verify it yourself

1. **Estimator source.** `git -C lio_bench/third_party/ros_ws/src/faster-lio log -1` should show
   `8038fba`, and `git diff` there should equal `lio_bench/third_party_patches/faster-lio.diff`.
   (`third_party/` is rebuilt by `setup/build_ros_ws.sh`.)
2. **The parameter actually used.** `grep point_filter_num lio_bench/results/runs/faster_lio/full/node.log`.
3. **Independent accuracy check.** `python lio_bench/verify/verify_faster_lio.py` (numpy only).
4. **Replay Faster-LIO** (~10 min; deterministic, so it should match `full`):
   `MEM=4G bash lio_bench/scripts/capped.sh bash lio_bench/scripts/run_ros.sh --algos faster_lio --tag full_check`
5. **Option B scores:** `python lio_bench/pipeline/score_option_b.py`, which rewrites
   `results/optionB_all/` from the recorded runs in a few minutes.
6. **Pipeline re-implementation:** `python lio_bench/pipeline/validate_against_original.py` (needs
   the reduced bag).
7. **Check any CSV before feeding it to the pipeline:** `python lio_bench/verify/diagnose_nav_csv.py <file.csv>`.

---

## Appendix H: glossary

| term | meaning |
|---|---|
| LIO | LiDAR-inertial odometry (FAST-LIO2, Faster-LIO, DLIO, GenZ-LIO) |
| KISS-ICP | LiDAR-only odometry (no IMU); used here as the heading source |
| G, gravity correction | online complementary filter pulling roll/pitch toward IMU gravity (τ = 5 s) |
| gk | gravity + KISS-ICP heading, computed offline from recorded runs |
| node, online | the same rule running live in ROS (`kiss_fusion_node.py`) |
| option B | GNSS used only for one heading (+ position) at frame 1 |
| 200 m fit, canonical | one SE(2) fit on the first 200 m of the span, then prediction |
| ATE | RMSE of horizontal position error |
| end-to-end drift | final error / distance driven |
| KITTI translation / rotation | error over 100–800 m pieces, each matched at its own start |
| RPE 1 s | displacement error over 1 s |
| paper-style ATE | ATE after fitting the whole drive to GNSS (evaluation only) |
| pole span | km 0.72 → 5.19, first to last surveyed pole |
| in-box / native metric | the professor's pipeline's own number: frames inside the pole map's bounding box |
| heading reset | stage-3 correction of the heading from consecutive pole fixes |
| wrong-side lock | the pole fix snapping to the opposite pole of a pair, 8.8 m across the road |
