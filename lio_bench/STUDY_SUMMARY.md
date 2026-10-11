# LIO study on the 2024-02-28 E39 Hemnekjølen drive: what was done and what came out

*Status as of 2026-10-11 00:45. Every number below comes from `lio_bench/results/` (full details in
`results/REPORT.md`). Nothing has been committed to git.*

---

## 0. TL;DR

1. **All four LiDAR-inertial odometry (LIO) methods now run on this drive:** FAST-LIO, Faster-LIO,
   DLIO and GenZ-LIO, at pinned upstream commits and built with conda. Faster-LIO first failed
   (24 % of the true distance). The cause was one config value, `point_filter_num: 4`, which
   aliases with the OS2's 1024 columns. With `3` it matches the original `fasterlio.csv`.
2. **`fastlio.csv` and `fasterlio.csv` are genuine outputs** of the algorithms, but their timestamps
   are about 0.4–0.6 s late.
3. **The baselines' main weakness is tilt drift, not lost distance.** All four methods slowly drift
   in roll and pitch (35–50° by km 10). In that tilted frame the car "climbs" 2.4–3.4 km and the
   horizontal track looks 5–9 % short, although every method's own step lengths are right to 0.2 %.
   Cause: on an open road the scans barely constrain forward motion, so a pitch error looks the same
   as forward acceleration.
4. **Fixing it needs no learning.** Rebuilding the horizontal path from each method's body-frame
   steps on the level ("planar re-integration") cuts whole-drive error by 35–65 %
   (FAST-LIO 497 → 183 m) and test-block drift from 25.6 % to 0.85 %.
5. **On top of planar:** GICP/VGICP scan-to-map correctors reach about 0.65 % drift per 200 m.
   CAN wheel distance with the baseline's heading is the best single option on the test block.
   Learned fusion helps FAST-LIO (test error −46 %) but is not proven for Faster-LIO.
6. **Baseline + support run together in real time** on the laptop for all four pairs, with no
   accuracy loss. **Sliding windows** keep only part of the support's benefit at 10–16 % duty.
7. **Open:** pole fixes (the detector misses the snow poles), heading drift (now the main remaining
   error), putting the motion constraints inside the filter, clean timings on the cloud.

---

## 1. What was built (`lio_bench/`)

| part | what it is |
|---|---|
| `envs/lio-py.yml`, `envs/lio-ros.yml` | conda envs: Python analysis (numpy/torch/sklearn/small_gicp/kiss-icp) and ROS Noetic via RoboStack; works on x86_64 and aarch64 |
| `setup/bootstrap.sh` | one-shot idempotent setup: installs Miniforge if needed, creates envs, clones the 4 estimators at pinned commits, applies build-only patches, builds, installs GenZ-LIO's Python bindings |
| `setup/build_ros_ws.sh` | catkin workspace; each compatibility patch is commented (glog 0.7, PCL C++17, message-generation races, DLIO `boost`→`std` pointers) |
| `ros/` | launch files, `run_ros.py` (roscore, bag replay, resource monitor, drain), `odom_recorder.py` (poses, per-scan latency) |
| `src/liobench/` | the `liobench` CLI: clock fit, evaluation, verification, analysis, concurrency, fusion, windows, corrections, report |
| `scripts/run_all.sh` | the whole study, resumable; **each step in its own RAM-capped scope** (4 GB on this laptop) |
| `scripts/run_unattended.sh` | hands-off: waits, runs all steps, retries failed ones, writes `results/REPORT.md` |
| `scripts/capped.sh` | hard RAM cap via `systemd-run --user --scope -p MemoryMax=… -p MemorySwapMax=0` |
| `tests/` | 18 unit tests on synthetic data (all pass) |

**Trustworthiness of the estimators.** These are the official repositories at pinned commits
(FAST_LIO `7cc4175`, faster-lio `8038fba`, DLIO `fc8d183`, GenZ-LIO `c6c464d`). The only changes are
build fixes. Faster-LIO in this local build still has its parallel loops switched to sequential
from debugging, which changes speed, not results. The cloud build script resets the source.

### Problems found and fixed along the way

| problem | cause | fix |
|---|---|---|
| Faster-LIO registered 24 % of the distance | `point_filter_num: 4` keeps the same azimuth columns in every ring (1024 columns ÷ 4); the resulting radial stripes pin the scan to the map | `point_filter_num: 3` (1024 mod 3 = 1) → length ratio 1.004 over 30 s. Also improves FAST-LIO (0.974 → 1.006) |
| GenZ-LIO killed at 4 GB after ~500 scans | the authors' configs never prune the map (`map_range: -1`) | `map_range: 300` (2× sensor range) → steady 2.6 GB |
| One OOM killed the whole pipeline | one cap over the whole sequence | each step capped separately; an OOM fails one step only |
| DLIO stamp-0 pose | pose published before initialisation | dropped in recorder and loader |
| Laptop froze earlier (~6 GB) | uncapped parallel jobs | every heavy job under a hard cap, one at a time |
| Analysis crashed | 10 s windows with no data | handled; passed on the retry pass |
| Sliding windows reused the wrong runs | window run names didn't include the baseline | fixed; all window steps rerun |

---

## 2. Data facts established

- **Bags.** Only the 41 GB bag has `/ouster/points`. The 5.7 GB bag has `/ouster/imu`, GNSS,
  INS and 4 Ouster images, but no point clouds.
- **Clock.** Ouster stamps are the sensor's own oscillator (~13 214 s), not Unix. The fitted mapping
  is `unix = stamp + 1709108377.355 + 105 ppm · (stamp − stamp0)`, residual 0.14 ms.
- **Ground truth.** It matches the dual-GNSS midpoint to a median of 0.02 m.
  Route: 542 s, 10.3 km, 38 km/h for the first km, then ~75 km/h.
- **Sensors.** The LiDAR–IMU extrinsic (identity rotation) is correct. IMU and LiDAR agree on
  vertical to ~0.3° (gyro turning axis 1.75°, LiDAR ground normal 1.50°, training-block mean
  specific force 1.71°). The 5.6° accelerometer tilt at the start is the car parked on a slope.
- **CAN wheel speed** reads 7 % high (scale 0.928) with a 0.35 s lag, fitted on the training
  block. Per-step noise is 0.07 m (3 %), unbiased.
- **Pole map:** 290 surveyed poles, one per ~18 m, covering km 0.7–5.2. They stand 1.5–6 m to
  the side of the GNSS antenna midpoint.

---

## 3. Step 1: are `fastlio.csv` / `fasterlio.csv` accurate?

| | `fasterlio.csv` | `fastlio.csv` |
|---|---|---|
| vs. a fresh run of the same algorithm | median 14 m apart over 10.3 km, length ratio 0.993: **matches** | median 48 m apart, CSV 1.6 % longer: **close, slightly different run/config** |
| drift vs. ground truth (frame-free) | 5.7 % | 8.2 % |
| timestamp offset | **≈ +0.5–0.6 s late** | **≈ +0.4–0.5 s late** |

**Verdict:** both CSVs are real algorithm outputs. The ~0.5 s clock offset was introduced at export
and amounts to ~10 m of apparent error at highway speed. The evaluation code corrects it
automatically.

---

## 4. Continuous runs of the four methods (full drive)

Rigid planar fit over the whole drive (`se2_full`):

| method | role | ATE | horizontal length ratio | CPU mean | RAM max |
|---|---|---:|---:|---:|---:|
| Faster-LIO | baseline | **380 m** | 0.953 | 124 % | 437 MB |
| DLIO | support | 394 m | 0.923 | 234 % | 3.1 GB |
| GenZ-LIO (ROS) | support | 447 m | 0.911 | 216 % | 2.6 GB |
| FAST-LIO | baseline | 497 m | 0.906 | 117 % | 615 MB |

GenZ-LIO's offline Python runner and its ROS node agree (452 vs 447 m), so the fast offline runner
is a faithful stand-in. The FAST-LIO run with the legacy `point_filter_num: 4` gives about the same
full-drive numbers (520 m, ratio 0.908). The setting only matters early in the drive and, hugely,
for Faster-LIO.

---

## 5. Step 3: weakness analysis, including the main discovery

### 5.1 The real weakness: tilt drift

All four methods' horizontal tracks are 5–9 % short. That is **not** a scale error: their own 3D
step lengths add up to 10 278 m against 10 295 m of ground truth (0.2 %). What happens instead:

| FAST-LIO | km 0–2 | km 2–4 | km 4–6 | km 6–8 | km 8–10 |
|---|---:|---:|---:|---:|---:|
| estimated body pitch | +5° | +14° | +19° | +21° | +34° |
| estimated body tilt | 7° | 29° | 32° | 43° | 49° |
| estimated climb angle | +11° | +14° | +20° | +17° | +35° |
| real road slope (GNSS) | +0.2° | 0.0° | −1.0° | −2.6° | −1.3° |

Height error at km 10: FAST-LIO +3.4 km, DLIO +3.2 km, GenZ-LIO +3.3 km, Faster-LIO +2.4 km.
The real road descends 163 m.

**Why:** on an open snowy highway the scans pin the car sideways and vertically but barely along
the road. The scene's weakest direction is along the road in every kilometre (cos ≈ 0.98–1.00). A
small pitch error leaks gravity into the forward axis, which is indistinguishable from forward
acceleration when forward motion isn't observed. The local map bends with the drift, so nothing
corrects it. All four algorithms share it because it comes from the scene, not the code.

### 5.2 Once tilt is removed (tilt-invariant body-frame steps, 10 s windows)

| method | scale error median | yaw drift median per 10 s | step error median |
|---|---:|---:|---:|
| FAST-LIO | −0.13 % | 0.44° | 0.058 m |
| Faster-LIO | −0.17 % | 0.48° | 0.059 m |
| DLIO | +0.13 % | **0.26°** | 0.156 m (noisy per step, but least heading drift) |
| GenZ-LIO | −0.11 % | 0.47° | 0.057 m |

- **What's left after tilt is heading (yaw) drift.** Scale is fine.
- **FAST-LIO, Faster-LIO and GenZ-LIO fail together.** Their step errors correlate at
  ρ = 0.95–0.97, so one of them as a support for another adds little. DLIO behaves differently:
  noisier per step, but with less heading drift.
- **Errors grow with speed and with IMU vibration** (acceleration variability).
- **Best method per 10 s window:** Faster-LIO 45 %, GenZ-LIO 36 %, FAST-LIO 19 %.

Maps and plots: `results/analysis/`.

---

## 6. Baseline + support running at the same time

Same bag stream, both nodes live, laptop (Apple M-series, 10 cores):

| pair | baseline ATE solo → together | support ATE solo → together | latency (baseline, median / p95) | scans dropped |
|---|---|---|---|---|
| FAST-LIO + DLIO | 500 → 493 m | 396 → 411 m | 39 / 57 ms | 4 of ~5 400 |
| FAST-LIO + GenZ-LIO | 500 → 496 m | 450 → 450 m (GenZ p95 96 ms) | 37 / 54 ms | 7 |
| Faster-LIO + DLIO | 382 → 382 m | 396 → 401 m | 24 / 35 ms | 7–8 |
| Faster-LIO + GenZ-LIO | 382 → 382 m | 450 → 458 m (GenZ p95 88 ms) | 26 / 38 ms | 7 |

- **All four pairs run together in real time.** Accuracy changes stay within normal run-to-run
  variation.
- **GenZ-LIO runs close to its limit:** p95 latency is near the 100 ms scan period.
- **Laptop timings aren't reliable as numbers.** Concurrent runs consistently measured *lower* CPU
  and latency than solo runs, most likely because a single light job is scheduled on the M-series
  efficiency cores. Trust these timings as "it fits", not as exact figures. Clean numbers need the
  cloud or the vehicle computer.

---

## 7. Phase 2: correction strategies

Each strategy was evaluated per chronological block. Each block is anchored on its own first
200 m, so the numbers are drift within the block. Wheel calibration uses the training block only;
the strategy is selected on validation; the test block is reported once.

### 7.1 Results (drift % per 200 m; test = last 20 % of the drive, ~2 km)

| strategy (FAST-LIO baseline) | train | val | test | ms/scan |
|---|---:|---:|---:|---:|
| FAST-LIO as is | 4.23 | 9.30 | 25.59 | — |
| + CAN wheel distance (in the tilted frame) | 5.27 | 9.55 | 25.39 | ~0 |
| **+ planar re-integration** | 1.29 | 1.63 | **0.85** | ~0 |
| + wheel + planar | 2.66 | 1.61 | 0.87 | ~0 |
| + point-to-point ICP + planar | 1.47 | 1.12 | 2.19 | 52 |
| + point-to-plane ICP + planar | 1.32 | 1.45 | 1.54 | 44 |
| + point-to-plane, degeneracy-aware + planar (**selected on val**) | 1.31 | **0.83** | 1.27 | 36 |
| + GICP + planar | 1.12 | 1.04 | 0.67 | 36 |
| + GICP degeneracy-aware + planar | 1.13 | 1.05 | **0.64** | 34 |
| + GICP degeneracy-aware with wheel prior + planar | 1.13 | 1.05 | 0.66 | 34 |
| + VGICP (NDT-like) + planar | 1.24 | 1.64 | **0.63** | **8** |
| KISS-ICP + planar (independent) | 1.45 | 0.58 | 1.65 | 34 |
| DLIO + planar | 1.45 | 1.62 | 0.91 | — |
| GenZ-LIO + planar | 1.38 | 1.02 | 0.88 | — |

Faster-LIO baseline: 13.41 % as is → 1.20 % planar → 0.65 % with GICP/VGICP + planar. The selected
strategy (point-to-plane DA + planar) scored 1.94 % on test.

Whole-drive error when aligned only on the first 200 m (i.e. navigating 10 km from a start fix):
FAST-LIO 1 486 m → planar 782 m → GICP + planar 446 m → KISS-ICP + planar 372 m. Without absolute
fixes this is dominated by heading drift.

### 7.2 What this says

- **Planar re-integration is the big win.** It's free, has no fitted parameters, and works for all
  methods.
- **ICP correctors only help on top of planar.** Run alone they inherit the baseline's tilt.
  GICP and VGICP do best. **VGICP is the cheapest (8 ms/scan)** and among the best, a good candidate
  to run continuously. Point-to-point ICP does worst.
- **The degeneracy-aware fusion and the wheel prior add nothing measurable** on top of GICP.
- **Model selection is noisy on one drive.** The validation pick (1.27 %) lost on test to GICP/VGICP
  (0.63–0.67 %). Differences of ~0.2–0.5 points between the top strategies aren't reliable here.
- **Pole fixes did not work** (section 10).

---

## 8. Step 4: the learned fusion model ("which model, what percent")

**Model:** per-step weights over the methods' tilt-invariant body-frame steps, as two masked
softmaxes (translation, yaw). It's trained against ground-truth steps and uses causal context:
scan geometry, IMU statistics, CAN data, GenZ internals, and agreement between methods.

**Guards against overfitting a single drive:**
- chronological train / val / test split with 10 s gaps;
- the test road never overlaps the training road (0 % within 25 m);
- a model ladder (best single method → static weights → linear gating → MLP-16 → MLP-64), chosen
  by 5-fold blocked cross-validation with a **paired** one-standard-error rule (simplest model
  within one paired SE of the best);
- a **shuffled-context control** (context time-shifted so it no longer lines up with the errors);
- a train / val / test gap report and 3 random seeds;
- availability dropout, so the model also works when a support is switched off.

### Results on the held-out test block (~2 km)

Integrated position error over the block (started from the true pose at block start) / drift per 200 m:

| | FAST-LIO setup | Faster-LIO setup |
|---|---:|---:|
| baseline alone | 21.0 m / 0.66 % | 31.9 m / 0.94 % |
| best single method (chosen on train) | 21.4 m / 0.59 % | 31.9 m / 0.94 % |
| static weights | 19.4 m / 0.59 % | 23.2 m / 0.73 % |
| **learned (supports DLIO + GenZ)** | **17.2 m / 0.44 %** (MLP-16) | **20.1 m / 0.44 %** (MLP-16) |
| shuffled-context control | 28.3 m / 0.57 % | 34.0 m / 0.70 % |
| **learned with correctors added** (+ KISS-ICP, + wheel, + selected corrector) | **11.4 m / 0.41 %** (linear) | 27.7 m / 0.47 % (linear) |
| its shuffled-context control | 15.5 m / 0.45 % | 24.4 m / 0.50 % |
| *baseline + wheel distance alone* | ***3.6 m / 0.49 %*** | ***14.0 m / 0.58 %*** |

Train / val / test step errors are similar (0.072 / 0.072 / 0.096 m), so there's no large
generalisation gap.

**Verdict:**
- **FAST-LIO setup:** learned fusion is a real gain. It cuts test error 46 % and beats all simpler
  options and its control.
- **Faster-LIO setup:** with supports only it gains about 37 %. With correctors added it does
  **not** beat its control on position error, so that variant isn't supported.
- **The simplest thing is the strongest:** the baseline's heading plus CAN wheel distance.

### Reliability trigger (for the sliding windows)

The trigger predicts the baseline's **per-metre** error over the next second from baseline-only
data. Rank correlation on test: 0.78 vs 0.27 for a speed-only control (FAST-LIO), and 0.54 vs 0.39
(Faster-LIO). So it predicts reliability, not just speed. (The first version predicted absolute
error, which mostly tracked speed; that was fixed.)

---

## 9. Step 5: supports in sliding windows

Supports run only when the trigger says the baseline is about to be unreliable (adaptive), or on a
fixed schedule (periodic). Each window has a 5 s warm-up; "warm start" seeds GenZ-LIO's velocity
from the baseline.

Test block, integrated error / drift per 200 m:

| baseline + support, schedule | duty | baseline alone | support always on | **support in windows** |
|---|---:|---:|---:|---:|
| FAST-LIO + DLIO, adaptive cold | 16 % | 21.0 m / 0.66 % | 12.7 m / 0.46 % | **18.4 m / 0.55 %** |
| FAST-LIO + GenZ, adaptive warm | 17 % | 21.0 / 0.66 | 18.2 / 0.56 | 23.4 / 0.51 |
| FAST-LIO + GenZ, adaptive cold | 17 % | 21.0 / 0.66 | 18.2 / 0.56 | 24.7 / 0.50 |
| FAST-LIO + GenZ, periodic 20 s / 60 s | 41 % | 21.0 / 0.66 | 18.2 / 0.56 | 19.3 / 0.64 |
| Faster-LIO + DLIO, adaptive cold | 10 % | 31.9 / 0.94 | 16.5 / 0.63 | 31.2 / 0.89 |
| Faster-LIO + GenZ, adaptive warm | 10 % | 31.9 / 0.94 | 25.8 / 0.63 | 32.3 / 0.84 |
| Faster-LIO + GenZ, adaptive cold | 10 % | 31.9 / 0.94 | 25.8 / 0.63 | 34.7 / 1.12 |
| Faster-LIO + GenZ, periodic | 41 % | 31.9 / 0.94 | 25.8 / 0.63 | 29.7 / 0.93 |
| adaptive at the 90th percentile (both) | 0 % | trigger never fired on test | | |

**Performance drop from windowing:**
- **Most of the support's benefit is lost at 10–16 % duty.** The best case, DLIO in adaptive
  windows for FAST-LIO, keeps about 30 % of the always-on gain for about 16 % of the compute.
- **GenZ-LIO windows can make things worse.** A window's settling period costs more than it gains,
  especially with a cold start.
- **Warm start helps a little over cold** for GenZ-LIO.
- These come from one ~2 km test block: suggestive, not conclusive.

---

## 10. What did not work (yet)

- **Pole-map fixes.** The geometric detector (thin, tall, isolated vertical clusters) finds objects
  10–36 m to the side of the car, but the snow poles stand 1.5–6 m away, at the snowbank. Checked
  with the true pose, only 6 of 290 poles are ever matched. Measuring height above the local snow
  surface instead of the road didn't fix it. A secondary bug: the filter grew its uncertainty like
  random noise (√distance) instead of linearly, so even good matches would be gated out.
- **Wheel distance applied in the tilted world frame:** no effect, because length was never the
  problem. It only works in the tilt-invariant form.
- **The degeneracy-aware ICP fusion:** physically sound, but no measurable gain over plain GICP here.

---

## 11. What's left / recommended next

1. **Poles, the only absolute correction for heading drift.** Either use the poles' retro-reflective
   tape (very high laser intensity), or run your trained YOLO detector (`model/pole_best_signal.pt`)
   on the Ouster signal images in the 5.7 GB bag, which is the approach already proven in your
   pipeline. Then match windows of detections to the map as a whole, with linearly growing
   uncertainty.
2. **Fix the tilt inside the filter, not afterwards.** Add CAN wheel speed and the vehicle
   constraint (no sideways or vertical body velocity) as measurements in an LIO update. That makes
   pitch and roll observable and stops the drift at the source. GenZ-LIO's Python bindings are the
   easiest place to try it.
3. **Heading-drift reduction:** gyro-bias handling, DLIO's lower yaw drift as a heading source in
   fusion, and VGICP + planar as a cheap continuous corrector.
4. **Cloud runs:** clean latency/CPU numbers on uniform cores, and repeated runs to measure
   run-to-run variation of FAST-LIO/DLIO (they're not deterministic; GenZ-LIO and this Faster-LIO
   build are).
5. **One drive only:** every learned result here is from one 10 km drive. The guards above keep
   it honest, but differences under ~0.5 points of drift between top methods aren't reliable.

---

## 12. Where everything is

| what | where |
|---|---|
| everything in one file | `lio_bench/results/REPORT.md` |
| verification of the CSVs | `results/verify/verify_summary.md` + maps |
| weakness analysis | `results/analysis/` (maps of where each method fails) |
| concurrency | `results/concurrency/*.json` |
| fusion models | `results/fusion/<baseline>[_correctors]/fusion_summary.md`, `gating_model.pt`, weight plots |
| corrections | `results/correct/<baseline>/correct_summary.md`, `drift_by_block.png`, `wheel_calibration.json` |
| sliding windows | `results/windows/<baseline>/<schedule>/windows_summary.md` |
| every trajectory | `results/runs/<method>/<tag>/` (TUM, latency, CPU/RAM, meta) |
| logs | `results/logs/<step>.log` |

**Re-running everything** (laptop or cloud):

```bash
bash lio_bench/setup/bootstrap.sh            # once
setsid nohup bash lio_bench/scripts/run_unattended.sh > lio_bench/results/logs/unattended.out 2>&1 &
```
