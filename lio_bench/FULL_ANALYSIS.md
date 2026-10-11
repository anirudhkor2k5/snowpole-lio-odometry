# Full analysis: what we did, what every script does, and how it led to the conclusions

*2026-10-11. One drive: E39 Hemnekjølen, 2024-02-28, 10.3 km, 542 s. Covers everything in
`lio_bench/`. Nothing has been committed to git, and no GitHub-tracked file was modified.
Detailed reports: `STUDY_SUMMARY.md` and `results/REPORT.md` (v1), `REPORT_v2.md` (audit),
`REPORT_v3.md` (heading + final system + §12 pipeline), `METHODS.md` (equations). Every CSV is
indexed in `CSV_INDEX.md`.*

---

## 0. The answer in one page

**Goal.** The professor's snow-pole localisation paper dead-reckons from an odometry CSV and resets
the position at surveyed snow poles. FastReg is dropped. The task: make the odometry better with a
pre- or post-processing step around an off-the-shelf LIO, without changing its source, then run it
through the pipeline.

**What we achieved:**

| | baseline (FAST-LIO2 raw) | ours (online node: Faster-LIO + gravity + KISS-ICP heading) |
|---|---:|---:|
| Odometry error, 10 held-out 1 km blocks (ATE per block) | 51.2 m | **6.1 m** (8× lower) |
| Drift per 200 m | 8.52 % | **0.67 %** |
| Heading error per block | 8.0° | **1.3°** |
| Real time on the laptop | yes | **yes**: 0 dropped scans, 47 ms latency, 71 % CPU, 148 MB |
| Pipeline, first → last pole, original stage 3 (mean / max) | 10.6 m / 33.9 m | **7.4 m / 18.7 m** |
| Pipeline, first → last pole, + heading reset (mean / median / max) | 2.1 / 1.0 / 16.4 m | **2.4 / 1.2 / 11.0 m** |
| Pipeline, whole 10.3 km, original stage 3 (final error) | 2 912 m | **526 m** |
| Pipeline, whole 10.3 km, + heading reset (final) | 1 333 m | **151 m** |
| Pipeline, whole 10.3 km, + heading reset + drift rate (final) | 639 m | 7.2 m (**fragile: honest range ~7–50 m**) |

**The five findings that carry the paper:**

1. **Every LIO on this road fails the same way, and the cause is the scene, not the code.** On an
   open snowy highway the scans barely constrain motion along the road. A small pitch error then
   leaks gravity into the forward axis, and the estimator "climbs" 2.4–3.4 km over 10 km. Fix:
   take roll/pitch from IMU gravity (a 5 s complementary filter), not from the LIO. This is the
   largest single gain: FAST-LIO2 51.2 → 9.6 m per km.
2. **What remains is heading bias.** All IMU-coupled LIOs drift counter-clockwise at a constant
   0.044–0.059 °/s. LiDAR-only KISS-ICP drifts at 0.024 °/s and is 3–5× more stable. Fix: take
   heading from KISS-ICP. Gain ~25 % more (9.6 → 7.4 m). The mechanism is uncompensated motion
   distortion. Earth rate, IMU-noise tuning, gyro-bias initialisation and gyro-bias wander were
   all tested and rejected.
3. **Once both fixes are applied, the choice of LIO no longer matters for accuracy.** All four land
   at 7.2–7.8 m per km. DLIO and GenZ-LIO cost 2× the CPU and 5–7× the RAM, so Faster-LIO is the
   one to run.
4. **In the pipeline, a heading reset from consecutive pole fixes takes the pole-span error from
   ~10 m to ~2 m.** It does this for every odometry input. The reason is new: the ~10 m floor of the
   original pipeline is the **wrong-side pole association** (poles stand in pairs 8.8 m apart across
   the road), and heading drift is what triggers it.
5. **After the last pole, better odometry is what matters:** 130–151 m vs 469–1 333 m for the raw
   LIOs. The drift-rate extrapolation's 7 m is partly luck (sensitivity test), so it must be reported
   as a range.

---

## 1. How we got there: the chain of evidence

Each step below lists the question, the scripts that answered it, what they showed, and the
decision it led to.

### Phase 0: get everything running (setup)

- **Question:** can FAST-LIO2, Faster-LIO, DLIO and GenZ-LIO run on this bag, on a RAM-limited
  laptop?
- **Scripts:**
  - `setup/bootstrap.sh`, `setup/build_ros_ws.sh`, `setup/install_genz_py.sh` and `envs/*.yml`
    set up the environments;
  - `ros/scripts/run_ros.py` and `odom_recorder.py` run the estimators;
  - `src/liobench/genz_runner.py` runs GenZ-LIO in Python;
  - `scripts/capped.sh` provides the RAM cap.
- **Found:**
  - Faster-LIO registered only 24 % of the distance. Cause: `point_filter_num: 4` aliases with the
    OS2's 1024 columns. Setting it to 3 fixed it, and also improved FAST-LIO.
  - GenZ-LIO ran out of memory. Fixed with `map_range: 300`.
  - The Ouster clock is the sensor oscillator. It was fitted to Unix time with 0.14 ms residual
    (`bag.py`).
- **Decision:** the official repos at pinned commits, with build-only patches. Every heavy job runs
  under a hard RAM cap, one at a time.

### Phase 1 (v1): verify the legacy CSVs and find the weakness

- **Step 1.** `verify.py` (`liobench verify`): are `fastlio.csv` and `fasterlio.csv` real?
  - Yes: they match fresh runs of the same algorithms.
  - Their timestamps are 0.4–0.6 s late, worth about 10 m of apparent error at highway speed. The
    evaluation code corrects this automatically.
- **Step 2.** Full-drive runs of all four methods (`run_ros.py`, `genz_runner.py`), scored with
  `metrics.py`:
  - ATE: Faster-LIO 380 m, DLIO 394 m, GenZ-LIO 447 m, FAST-LIO 497 m.
  - Every horizontal track is 5–9 % short.
- **Step 3.** `analysis.py` (`liobench analyze`), `scan_features.py`, `dataset.py`:
  - The tracks are short but the step lengths are right to 0.2 %. So it is **tilt**, not scale.
  - Pitch grows to +34° and the car "climbs" 2.4–3.4 km.
  - The weakest scan direction is along the road in every km (cos ≈ 0.98–1.00).
  - **Conclusion:** the failure comes from the scene: gravity leaks into an unobserved forward axis.
- **Step 3b.** Concurrency: `concurrency.py` with `run_ros.py --algos A,B`. All four
  baseline + support pairs run together in real time, with no accuracy loss.
- **Phase 2 corrections.**
  - **Scripts:**
    - `correct/planar.py`: tilt-invariant re-integration;
    - `correct/wheel.py`: CAN wheel distance;
    - `correct/scanpass.py`: ICP/GICP/VGICP correctors and KISS-ICP;
    - `correct/poles.py`: geometric pole fixes;
    - `correct/report.py`: per-block selection.
  - **Planar re-integration** is the big free win: test-block drift 25.6 % → 0.85 % for FAST-LIO.
  - **GICP/VGICP on top of planar** reach ~0.65 %.
  - **Geometric pole detection failed:** only 6 of 290 poles matched.
- **Step 4.** Learned fusion: `fusion.py` (`liobench fusion`).
  - A gating MLP mixes the methods' steps.
  - It helps FAST-LIO (test −46 %) but is not proven for Faster-LIO.
- **Step 5.** Sliding windows: `windows.py`, with the trigger model in `fusion.py`.
  - The support runs only when the trigger predicts trouble.
  - At 10–16 % duty it keeps only ~30 % of the support's benefit, and GenZ-LIO windows can hurt.
- **Report:** `report_all.py` (`liobench summary`) wrote `results/REPORT.md`; `STUDY_SUMMARY.md` is
  the hand-written summary.

### Phase A (v2): audit — make the numbers honest

- **Script:** `audit.py`. It defines the **canonical protocol** used everywhere after this:
  - the clock offset is fitted on the training block only;
  - one SE(2) fit on the first 200 m, a stand-in for an initial GNSS fix;
  - after that, pure prediction.
- **Found:**
  - About 97 % of the 10 km error is **heading**. Given the true heading, the same odometry stays
    within 30 m (Faster-LIO 1 245 → 30 m).
  - Heading drifts at a constant rate per second, counter-clockwise, for every method. That
    signature means a bias.
  - The tilt failure is **pitch**. Removing pitch restores the track length; removing roll does
    nothing.
  - A gravity-locked VGICP (`scanpass.py` variants `vgicp_grav`, `grav_only`; queue
    `scripts/run_phaseA_passes.sh`) turns 8.40 % drift into 1.37 %, online.
  - The ground truth is the car's own dual-antenna GNSS, RTK float. It is valid at metre level,
    but per-step yaw labels are noise-dominated.
  - Re-scored canonically, some v1 claims weaken. The learned fusion's advantage is local drift,
    not end position.
- **Decision:** pitch/roll must come from gravity; the next lever is the **lowest-bias heading
  source**.

### Phase v3: heading and the final system

- **Scripts:**
  - `correct/heading.py`: gravity filter, planar steps, integration, yaw-bias calibration, heading
    blend;
  - `v3.py`: leave-one-block-out evaluation with K = 5 and K = 10 blocks, paired Δ ± SE and sign
    tests;
  - `correct/kiss_probe.py`: KISS-ICP probes;
  - `scanpass.py`: schedule and update-form variants;
  - the queues `scripts/run_v3_queue*.sh` and `run_kiss_sweep.sh`;
  - the figures `scripts/make_v3_figures.py`.
- **Found:**
  - **Gravity filter, τ = 5 s.** Flat over 0.5–10 s and chosen on validation. It equals the
    offline planar fix (Δ −1.1 ± 1.5 m).
  - **Heading from KISS-ICP.** Chosen in 5/5 folds on the training blocks. It wins 9/10 blocks for
    both LIO baselines (exact sign test p = 0.021).
  - **Mechanism probes (`kiss_probe.py`, GenZ deskew probes):**
    - the heading bias scales with uncompensated motion distortion;
    - the pitch climb is carried by ground returns beyond ~10 m (ground removed: 2.2 km → 0.1 km).
  - **Rejected, each by a direct test:**
    - Earth rate;
    - IMU-noise tuning (×10 / ×100: no change; queue 5);
    - gyro-bias initialisation (the standstill bias is correct);
    - gyro-bias wander. Freezing the bias changes nothing (queue 7), although the bias correlates
      r = 0.87 with the drift.
  - **Selective correction does not pay** (VGICP every N-th scan). Always-on VGICP is −10 m.
  - **CAN wheel speed is not needed** once tilt and heading are handled.
  - **The heading blend is worse** than pure KISS heading (not adopted).
  - **KISS-ICP voxel sweep:** 2.0 m was chosen on the validation block. Test 8.7 vs 9.4 m for the
    default, at 61 % lower cost.
- **The real-time node** (`ros/scripts/kiss_fusion_node.py`, `configs/algorithms/kiss_fusion.yaml`,
  run by `run_ros.py --algos faster_lio,kiss_fusion`):
  - **6.1 m per km block**;
  - 0 dropped scans, 47 / 72 ms latency, 71 % CPU, 148 MB;
  - with 1.0 m voxels it fell behind (185 drops, 2.2 s backlog). That is why 2.0 m is used.
- **The 10-block summary table** (REPORT_v3 §11.1):

  | pipeline | FAST-LIO2 | Faster-LIO | DLIO | GenZ-LIO |
  |---|---:|---:|---:|---:|
  | raw | 51.2 m | 28.9 m | 48.6 m | 47.7 m |
  | + gravity | 9.6 m | 12.7 m | | |
  | + gravity + KISS heading | 7.4 m | 7.7 m | 7.8 m | 7.2 m |

  The online node gets **6.1 m**.
- **Export:** `export.py` (`liobench export`) writes each pipeline as an
  `incremental_navigation_*.csv` (§3.8).

### Stage 3: the snow-pole pipeline

- **Scripts:**
  - `pipeline/snowpole_stage3.py`: detection pass + localisation loop + heading corrector;
  - `pipeline/validate_against_original.py`: runs the original script unchanged;
  - `pipeline/run_all_variants.py`: 24 runs;
  - `pipeline/rate_sensitivity.py`: the luck check;
  - `pipeline/make_pipeline_figures.py`: P1–P7.
- **Found:**
  1. **The re-implementation is exact.** It matches the original frame by frame over 2 158
     frames: max difference 7.4 × 10⁻⁵ m, final 13.6324 m, 358 fixes. So every pipeline number is
     what the professor's code would produce.
  2. **Better odometry helps the original pipeline everywhere.**
     - Pole span: 10.6 → 7.4 m mean.
     - Whole-drive final error: 2 912 → 526 m.
  3. **The heading reset is the big stage-3 lever.** Pole-span mean ~10 → ~2 m for every input.
     Whole-drive final error 526 → 151 m for the online node.
  4. **Wrong-side pole association explains the original pipeline's floor.**
     - The ~10 m floor is cross-track. It equals the 8.8 m pole-pair spacing plus range.
     - Heading drift triggers the snap to the opposite-side pole, which is then self-sustaining.
     - The heading reset cuts the locked time from 63–77 % to 12 %.
  5. **The drift rate is sensitive.**
     - The online node's 7.2 m sits at the bottom of a V. ±0.002 °/s (≈ 1 SE) gives ~21 m;
       ±0.005 °/s gives ~50 m.
     - Fitting the second half of the updates only gives > 100 m.
     - The offline variant was unlucky the other way: 130 m, where its best offset would give 36 m.
- **Decision:**
  - Report the pole-span numbers and the "+ heading reset" whole-drive numbers as the robust
    results.
  - Report the drift-rate result as a range.
  - The next lever is side-consistent pole association. It is identified but not implemented.

---

## 2. Final results

### 2.1 Odometry (10 held-out ~1 km blocks, canonical protocol, paired vs FAST-LIO2 raw)

| pipeline | ATE / block | better than baseline | drift / 200 m | heading error |
|---|---:|---:|---:|---:|
| FAST-LIO2 raw (baseline) | 51.2 m | — | 8.52 % | 8.0° |
| FAST-LIO2 + gravity | 9.6 m | 10/10 | 0.66 % | 2.0° |
| FAST-LIO2 + gravity + KISS heading | 7.4 m | 10/10 (p = 0.002) | 0.62 % | 1.6° |
| Faster-LIO raw | 28.9 m | 8/10 | 4.15 % | 4.3° |
| Faster-LIO + gravity + KISS heading | 7.7 m | 10/10 | 0.68 % | 1.6° |
| DLIO raw → corrected | 48.6 → 7.8 m | 9/10 | 0.68 % | 1.7° |
| GenZ-LIO raw → corrected | 47.7 → 7.2 m | 10/10 | 0.55 % | 1.6° |
| **online node (real time)** | **6.1 m** | — | **0.67 %** | **1.3°** |

### 2.2 Pipeline, Result 1: whole drive (10.3 km)

| odometry | stage 3 | mean | median | final |
|---|---|---:|---:|---:|
| FAST-LIO2 raw | original | 584.5 m | 16.9 m | 2 912 m |
| FAST-LIO2 raw | + heading reset | 196.9 m | 5.5 m | 1 333 m |
| FAST-LIO2 + G + KISS | original | 147.5 m | 11.0 m | 675 m |
| Faster-LIO + G + KISS, offline | + heading reset | 45.6 m | 5.0 m | 249 m |
| **online node** | original | 118.0 m | 12.0 m | 526 m |
| **online node** | **+ heading reset** | **27.0 m** | **4.8 m** | **151 m** |
| online node | + heading reset + rate | 4.4 m | 3.1 m | 7.2 m (range ~7–50 m) |

### 2.3 Pipeline, Result 2: first pole → last pole (km 0.72–5.19)

| odometry | stage 3 | mean | median | p95 | max |
|---|---|---:|---:|---:|---:|
| FAST-LIO2 raw | original | 10.6 m | 10.9 m | 20.9 m | 33.9 m |
| FAST-LIO2 raw | + heading reset | 2.1 m | 1.0 m | 9.5 m | 16.4 m |
| FAST-LIO2 + G + KISS | original | 5.3 m | 3.6 m | 13.0 m | 17.8 m |
| **online node** | original | 7.4 m | 9.0 m | 13.9 m | 18.7 m |
| **online node** | **+ heading reset** | **2.4 m** | **1.2 m** | **9.1 m** | **11.0 m** |

All 24 runs are in `results/pipeline/PIPELINE_RESULTS.md`, with figures in `results/pipeline/figures/`.

---

## 3. Every script: what it does, what it outputs, which conclusion it fed

The CLI is `liobench <command>`, from `src/liobench/cli.py`. Python runs in the `lio-py` env and
ROS in `lio-ros`. Heavy jobs go through `MEM=XG bash scripts/capped.sh …`.

### 3.1 Setup and environments

| script | when you run it | outputs | used for |
|---|---|---|---|
| `envs/lio-py.yml`, `envs/lio-ros.yml` | conda env specs: Python analysis (numpy, torch, small_gicp, kiss-icp, rosbags, ouster-sdk…) and ROS Noetic (RoboStack) | the two envs | everything |
| `setup/bootstrap.sh` | One-shot and idempotent: Miniforge if needed → envs → ROS workspace → GenZ-LIO Python bindings → `liobench` package | a working install | Phase 0 |
| `setup/build_ros_ws.sh` | Clones FAST_LIO `7cc4175`, faster-lio `8038fba`, DLIO `fc8d183` and GenZ-LIO `c6c464d`, applies the commented build-only patches, builds with catkin | `third_party/ros_ws` | Phase 0 |
| `setup/install_genz_py.sh` | Builds GenZ-LIO's Python bindings from source, plus a `seed_velocity_body` hook for warm-started windows | GenZ-LIO Python module | sliding windows, probes |
| `setup/common.sh` | Shared shell helpers: repo root, conda activation (bash only) | — | all shell scripts |
| `configs/dataset.yaml` | Paths to the bag, topics and the ground truth | — | all |
| `configs/algorithms/*.yaml`, `registry.yaml`, `variants/*.yaml` | Each estimator's config (with the `point_filter_num: 3` fix and GenZ `map_range: 300`), the registry of what to launch, and the probe variants (bias frozen, IMU noise ×10, legacy pf4…) | — | runs and probes |

### 3.2 Running the estimators

| script | what it does when run | outputs | used for |
|---|---|---|---|
| `ros/scripts/run_ros.py` (via `scripts/run_ros.sh`) | Starts its own roscore on a free port, plays only the LiDAR + IMU topics in real time, launches the requested estimators (several at once for concurrency), samples their CPU/RAM, waits for them to drain, stops. Also starts script nodes such as `kiss_fusion`. | `results/runs/<algo>/<tag>/`: `odom_sensor_clock.tum`, `odom_recv.csv`, `cloud_recv.csv`, `resources.csv`, `meta.json`, `node.log` | every ROS number: accuracy, latency, CPU, concurrency |
| `ros/scripts/odom_recorder.py` | Started by `run_ros.py`. Records each odometry topic to TUM, logs when each pose and scan arrives, and writes a status file | the TUM + `*_recv.csv` files above | latency, drain detection |
| `ros/launch/*.launch` | ROS launch files for the four LIOs | — | `run_ros.py` |
| `src/liobench/genz_runner.py` (`liobench run-genz`) | Runs GenZ-LIO through Python straight from the bag, faster than real time. Any time slice, cold or warm start; optionally per-scan context features | `results/runs/genz_lio/<tag>/` TUM + `diagnostics.csv` (+ `results/features/scan_context.csv`) | GenZ results, sliding windows, bias logging (queue 2), deskew probes (queue 4) |
| `src/liobench/bag.py` (`liobench prepare`) | Reads the 41 GB bag once: fits the sensor-clock → Unix mapping, start-up gravity, and extracts INS heading, CAN speed/steering and fix state | `results/bag/*.csv`, clock fit json | every time conversion; wheel corrector |

### 3.3 Core library (no outputs of its own)

| module | role |
|---|---|
| `io.py` | Readers/writers: TUM, ground truth, legacy CSVs, run folders, lat/lon ↔ UTM. Original repo files are only ever read. |
| `geometry.py` | Pose interpolation, Umeyama/SE(2) alignment, levelling, body-frame increments `[forward, lateral, dyaw]`, integration |
| `metrics.py` | ATE, RPE, KITTI-style drift, time-offset estimation |
| `plots.py` | Headless matplotlib helpers |
| `dataset.py` | One table, one row per GT sample, with every method's increments + GT increments + causal context features; used by analysis and fusion |
| `scan_features.py` | Per-scan context from raw data: point counts, ranges, planarity, degeneracy eigenvalues, IMU vibration |
| `correct/common.py` | Pose tracks, chronological split intervals, writing a correction as an ordinary run |

### 3.4 v1 study steps

| script / command | what it does | outputs | conclusion it fed |
|---|---|---|---|
| `verify.py` (`liobench verify`) | Legacy `fastlio.csv`/`fasterlio.csv` vs ground truth and vs our re-runs; GT vs GNSS midpoint | `results/verify/verify_summary.md` + maps | the CSVs are genuine, 0.4–0.6 s late |
| `analysis.py` (`liobench analyze`) | 10 s windows: scale / lateral / yaw / step error per method; correlations with scene features; best method per window; maps | `results/analysis/` (incl. `window_errors.csv`) | **the tilt/pitch discovery**; methods fail together; heading is what is left |
| `concurrency.py` (`liobench concurrency`) | Solo vs concurrent runs: latency, dropped scans, CPU/RAM, accuracy change | `results/concurrency/*.json` | all pairs real time, no accuracy loss |
| `fusion.py` (`liobench fusion`) | Trains the gating model (best single → static → linear → MLP-16/64; blocked CV, paired 1-SE rule, shuffled-context control) and the trigger model | `results/fusion/<baseline>[_correctors]/`: summary md, `gating_model.pt`, `weights_holdout.csv`, `trigger_predictions.csv` | learned fusion helps FAST-LIO locally; real role = translation (v3) |
| `windows.py` (`liobench windows`) | Supports only inside periodic or adaptive windows (5 s warm-up, cold/warm); stitches increments | `results/windows/…/windows_summary.md`, `runs/*/win_*` | windows keep ~30 % of the benefit at 16 % duty |
| `report_all.py` (`liobench summary`) | Collects every step's summary into one file | `results/REPORT.md` | v1 report |

### 3.5 Corrections (Phase 2, v2, v3)

| script | what it does when run | outputs | conclusion it fed |
|---|---|---|---|
| `correct/planar.py` (`liobench correct planar`) | Rebuilds the horizontal path from tilt-invariant body steps | `runs/<base>+planar…/` | planar = big free win (v1); superseded by the online gravity filter |
| `correct/wheel.py` (`liobench correct wheel`) | Calibrates CAN lag/scale/axis on the training block, replaces forward motion with wheel distance | `runs/<base>+…wheel…/`, `wheel_calibration.json` | wheel useful only tilt-invariant; not needed in v3 |
| `correct/scanpass.py` (`liobench correct scan`) | One pass over the bag: deskews each scan with the baseline, registers it to a local map with ICP / point-to-plane / GICP / VGICP (and degeneracy-aware, gravity-locked, every-N-th, constant-velocity variants), runs KISS-ICP, finds pole candidates | `runs/<base>+<variant>/full/` (TUM + `diagnostics.csv`), `runs/kiss_icp/full`, `results/poles/detections_*.csv` | VGICP cheapest good corrector; **gravity-locked VGICP 8.40 % → 1.37 %** (v2); selective correction doesn't pay (v3) |
| `correct/poles.py` (`liobench correct poles`) | Geometric pole detection + EKF fix to the surveyed map | pole runs, detections | **did not work** (6/290 matched), replaced by the YOLO pipeline in stage 3 |
| `correct/report.py` (`liobench correct report`) | Every corrector vs its baseline per block; selected on val, reported on test | `results/correct/<base>/correct_summary.md`, `drift_by_block.png` | Phase 2 table |
| `audit.py` (Python: `audit.run()`) | Canonical protocol; error decomposition (heading / scale / lateral); planar ablation (roll vs pitch); GT provenance | `results/audit/audit.json` → REPORT_v2 | **heading is 97 % of the error**; pitch, not roll; GT validity |
| `correct/heading.py` | `gravity_filter` (online roll/pitch toward IMU gravity), `planar_steps`, `integrate` (heading from any source, yaw-bias / Earth-rate options), `yaw_bias_on_train`, `blend_heading` | used by `v3.py` | τ = 5 s gravity filter; heading-source swap |
| `v3.py` (Python: `v3.run()`, `v3.run_combos()`, `v3.learned_vs_rule()`) | K-block leave-one-out: everything calibrated on the other blocks; paired Δ ± SE, sign tests; combo pipelines (translation from A, heading from B); learned vs rule | `results/v3/*.json`, `results/v3_*.json` | **all v3 tables**: gravity + KISS heading 10/10; learned fusion vs rule |
| `correct/kiss_probe.py` (`liobench kiss-probe`) | KISS-ICP with one input change at a time (no deskew, reversed time, 5 ms shift, voxel sizes, no ground, min range, upper beams, max range…); records yaw bias, pitch climb, ms/scan | `runs/kiss_icp_<probe>/` | **heading bias = motion distortion; pitch carried by far ground**; voxel sweep → 2.0 m |
| `scripts/make_v3_figures.py` | v3 figures (map, height profile, heading error, probes, duty cycle, bias per block) | `results/v3/figures/f0–f5.png` | REPORT_v3 |

### 3.6 Real-time final system

| script | what it does when run | outputs | conclusion it fed |
|---|---|---|---|
| `ros/scripts/kiss_fusion_node.py` + `configs/algorithms/kiss_fusion.yaml` | ROS node next to Faster-LIO. For every LIO pose: applies the LIO's step, pulls roll/pitch toward IMU gravity (τ = 5 s), keeps only the level step, takes the heading change from KISS-ICP (2.0 m voxels) for the same scan (falls back to the LIO's if KISS-ICP is missing or disagrees by > 0.2 rad/s), integrates, publishes `/kiss_fusion/Odometry`. Causal. | `runs/kiss_fusion/rt_fusion_v20/` (TUM, latency, CPU/RAM) | **6.1 m per km block, real time** |

### 3.7 Export to the pipeline

| script | what it does when run | outputs |
|---|---|---|
| `export.py` (`liobench export --source … --pipeline …`) | Takes the pipeline's template CSV (5 422 rows, one per left-GNSS message) and replaces easting/northing with the chosen trajectory, sampled at each row's stamp (training-block clock offset; one SE(2) fit on the first 200 m; lat/lon recomputed) | `results/export/incremental_navigation_<source>_<tag>_<pipeline>.csv` + `.json` (alignment, row error vs GT) |

### 3.8 Stage 3: snow-pole pipeline (`lio_bench/pipeline/`, all new files)

| script | what it does when run | outputs | conclusion it fed |
|---|---|---|---|
| `snowpole_stage3.py detect` | Streams the 5.7 GB image bag; runs the repo's YOLOv5 pole model (pinned v7.0, conf 0.6) on each signal image; finds the nearest LiDAR point in each box with the repo's own `geoloc_utils` function | `results/pipeline/detections_cache.pkl` (once; ~586 MB RAM, flat) | detections shared by all runs |
| `snowpole_stage3.py run <csv> <mode>` | The original localisation loop, step for step. Optional `HeadingCorrector`: **reset** (heading offset from consecutive pole fixes ≥ 15 m apart; gain 0.5; reject > 15°) or **reset+rate** (also extrapolates the offset's trend). Computes metrics: whole drive, pole span, after the last pole, native | `<run>_frames.csv`, `<run>_heading_updates.csv`, `<run>_summary.json` | all pipeline numbers |
| `validate_against_original.py` | Runs the **unmodified** `snowpole_based_vehicle_localization.py` (only I/O shimmed: lazy bag reader, pinned YOLO, headless plots, Ouster SDK 1.0 name) | `results/logs/original_pipeline.log` | **re-implementation = original** (13.6324 m both, 358 fixes; frame by frame within 7.4e-5 m) |
| `run_all_variants.py` | 8 odometry CSVs × 3 modes = 24 runs; writes the tables | `PIPELINE_RESULTS.md`, `all_results.json`, `error_vs_distance__*.png` | Results 1 and 2 |
| `rate_sensitivity.py` | Re-runs reset+rate with the learned rate shifted (±0.002 / 0.005 / 0.010 °/s) or fitted on recent windows | `RATE_SENSITIVITY.md`, `rate_sensitivity.json` | **the 7 m is partly luck → report ~7–50 m** |
| `make_pipeline_figures.py` | P1 validation, P2 error vs distance, P3 pole span, P4 all-runs bars, P5 rate sensitivity, P6 map, P7 cross-track (pole-pair spacing measured from the map) | `results/pipeline/figures/p1–p7.png` | **wrong-side association finding** (P7) |

### 3.9 Orchestration (what ran unattended)

| script | what it ran |
|---|---|
| `scripts/capped.sh` | Wraps any command in `systemd-run --user --scope -p MemoryMax=… -p MemorySwapMax=0` with nice 10. An over-limit job is killed, never the machine. |
| `scripts/run_all.sh` | The whole v1 study, resumable (`results/.done/<step>` markers), each step separately capped |
| `scripts/run_unattended.sh` | Waits for `run_all.sh`, runs it twice (retrying failures), then `liobench summary` |
| `scripts/run_phaseA_passes.sh` | v2: gravity-locked VGICP + gravity-only passes, both baselines |
| `scripts/run_v3_queue.sh` … `run_v3_queue7.sh` | v3 heavy jobs in sequence: correction schedules (1), GenZ with bias logging (2), yaw-leakage / constant-velocity VGICP (3), GenZ deskew probes (4), IMU-noise ×10 / ×100 (5), KISS-ICP geometry probes (6), frozen gyro bias (7) |
| `scripts/run_kiss_sweep.sh` | KISS-ICP config sweep (voxel / range / points per voxel) |

### 3.10 Tests and docs

- `tests/test_liobench.py`: 23 unit tests on synthetic data (all pass, re-run 2026-10-11) covering geometry, metrics, increments
  and alignment.
- Documentation: `README.md`, `METHODS.md`, `STUDY_SUMMARY.md`, `REPORT_v2.md`, `REPORT_v3.md`,
  `CSV_INDEX.md`, this file.
- `odometry_refinement/`, `experiments/`, `docs/` and `reports/` at the repo root are outside
  `lio_bench` and are not covered here.

---

## 4. How far the results can be trusted

**Solid:**
- The estimators are official code at pinned commits. The only changes are build fixes plus the
  `point_filter_num` and `map_range` config changes.
- The pipeline re-implementation matches the original frame by frame.
- **Protocol:**
  - calibration on training blocks only;
  - one 200 m start fit;
  - paired block comparisons with sign tests;
  - choices (τ, voxel size, heading source) made on validation or training blocks, then scored
    once.
- **Mechanisms** are backed by direct interventions, not only correlations: ground removal kills
  the climb, the deskew probes move the bias, and the rejected hypotheses each had a test.

**Limits:**
- **One drive.** Block-level differences of 1–2 m are within noise: the KISS 1.5 m voxel setting
  won on 10 blocks and lost on test. Repeat runs vary ±1.5–4 % in ATE.
- **Ground truth** is the car's dual-antenna GNSS (RTK float). It is fine at metre level but not
  for centimetre or per-step yaw claims.
- **Laptop timings** (M-series cores) prove "fits in real time", not exact milliseconds. The paper
  runtime table should come from the target computer.
- **Stage-3 constants** (15 m, 0.5, 15°) were set once without tuning, but while looking at this
  drive.
- **The drift-rate result** is fragile: report the range.
- **The legacy CSV** (FastReg era) beats our inputs over the whole drive under the original
  stage 3 (173 m vs 526 m final). Ours is better inside the pole span and everywhere once the
  heading reset is on.

---

## 5. What is still open (ranked)

1. **Side-consistent pole association** in stage 3: only match poles on the side where the LiDAR
   sees them, or gate the jump a fix would cause. It directly attacks the ~10 m floor, and the
   remaining km 4.0–4.4 lock. Cheap, new files only. Not yet done.
2. **A robust drift-rate estimator**, e.g. weighted toward recent updates with a shrinkage prior, so
   the after-pole result no longer depends on luck. Evaluate it with the sensitivity test, not the
   single number.
3. **Runtime table on the target computer.** The C++ port of the node is optional.
4. **Why the LIOs' heading is 2× more biased than KISS-ICP's** is still unexplained. A paper can
   state this as an open observation.
5. **A second drive**, if one ever becomes available, is the only real cure for single-drive
   caveats.
