# lio_bench — methods, equations and leakage audit

Exact definitions of everything report_v2 measures. Code paths are given so each line
can be checked. Notation: `T_k ∈ SE(3)` pose of the IMU body at scan k in an estimator's
world; `R_k, p_k` its rotation and position; `D_k = T_{k-1}^{-1} T_k` the body-frame step;
`u_b` the vehicle's 'up' direction in the IMU body frame.

---

## 1. Data and clocks

| item | definition | code |
|---|---|---|
| Sensor clock → Unix | `unix = s + offset + drift·(s − s0)`, fitted on the lower envelope of IMU (record − stamp) per second; drift 105 ppm, residual 0.14 ms | `bag.fit_sensor_clock` |
| Ground truth (GT) | `groundtruth_extracted.csv`: UTM-33 easting/northing at 10 Hz. It is the dual-antenna GNSS midpoint (median 3.4 cm from it), RTK-float quality (fix status 5 on 99.3 % of samples, 6 = dead reckoning on 40). Antenna baseline 0.499 m ± 1.3 cm. Not an independent survey. | `io.load_ground_truth`, `audit.gt_provenance` |
| GT heading | course over ground from GT positions over ±0.5 s | `dataset._gt_heading` |
| Per-method clock residual | speed-profile match against GT (±1 s search, parabolic refinement). **Canonical protocol: estimated on the training block only, then fixed** | `metrics.estimate_time_offset`, `audit.train_offset` |
| Chronological split | train 60 % / val 20 % / test 20 % of GT rows with 10 s gaps | `fusion.chronological_split`, `correct.common.split_intervals` |

## 2. Canonical evaluation (deployable)

For a trajectory over a span `[t0, t1]`:

1. shift by the training-block clock residual;
2. fit **one** SE(2) transform on the first 200 m of the span (stand-in for an initial GNSS fix); everything after is prediction;
3. ATE = RMSE of horizontal position error; final error; RPE over 1 s / 10 s;
   drift % = KITTI-style segment drift: for every start i, anchor both paths at i, rotate the
   estimate so its first 10 m chord matches GT, measure the end-point error after L m,
   divide by L, average over L ∈ {100, 200, 400, 800} m (frame-free);
   95 % CI of the 200 m drift by moving-block bootstrap (blocks of 20 starts);
   horizontal length ratio; heading error over the last 50 m.

Whole-drive rigid fits (`se2_full`, `se3_full`) are **diagnostic only**: they use the whole GT
track to align and are never reported as deployable accuracy. Code: `audit.canonical`.

## 3. Vehicle 'up' in the body frame

`u_b = normalize(mean specific force over the training block)` from the IMU in the reduced bag
(1.71° from IMU z; agrees with the gyro's turning axis, 1.75°, and the LiDAR ground normal,
1.50°). A vehicle can compute it online as a long running mean. Code: `correct.planar.body_up`.

Level body axes: `f = normalize(e_x − (e_x·u_b) u_b)`, `l = u_b × f`.

## 4. Planar re-integration (tilt-invariant motion)

From an estimator's own poses only:

```
d_k   = R_{k-1}^T (p_k − p_{k-1})                  body-frame step
fwd_k = d_k · f,   lat_k = d_k · l                 level components
ψ̇_k   = rotvec(R_{k-1}^T R_k) · u_b               heading change about the vehicle's vertical
ψ_k   = ψ_0 + Σ ψ̇                                 integrated heading
x_k   = x_{k-1} + Rot(ψ_{k-1}) [fwd_k, lat_k]^T      (rotated by the START heading of the step)
```

No parameter is fitted; nothing from GT. Code: `correct.planar.planarize`, `body_steps`.
Unit test: a path whose world frame tips 0→40° is recovered to < 5 cm.

## 5. Online gravity-locked attitude (A4)

Per scan k, from the last 1 s of IMU samples and the baseline's own speed (LiDAR+IMU only):

```
v      = |d_k| / Δt                                speed from the baseline step
a_b    = (dv/dt) e_x + ω × (v e_x)                 vehicle acceleration (no side-slip)
u_meas = normalize(mean(f_imu) − a_b)              measured 'up' (if |·| > 5 m/s²)
R_k   ← R_k · Rot(axis = (R_k^T z) × u_meas, angle = α·∠(R_k^T z, u_meas)),   α = 0.1 s / 5 s
```

i.e. a complementary filter that pulls roll/pitch toward gravity with a 5 s time constant
(snapping every scan, α = 1, made ICP diverge on 54 % of scans: accelerometer jitter blurs the
map). Heading is untouched. Code: `correct.scanpass.gravity_up`, `tilt_correct`.

## 6. Scan-pass correctors

For each scan: points (LiDAR frame) → IMU frame (`p_I = p_L + t_IL`, R_IL = I), deskewed to
scan end with the baseline's poses interpolated in 20 time bins, downsampled 0.5 m.
Prediction `P_k = T^b_{k-1}^{-1} T^b_k` (baseline step); `T_init = T_{k-1} P_k`.

| variant | registration (small_gicp) | update |
|---|---|---|
| icp / plane_icp / gicp | scan → local map (0.5 m voxels, LRU 100 scans, ≤ 80 m), max corr. 1 m | `T_k = T_icp` |
| vgicp | scan → Gaussian voxel map (1 m) | `T_k = T_icp` |
| `*_da` | as above | `δ' = (I_icp + Λ)^{-1} I_icp δ`, `I_icp = H/(s²·10)`, Λ = diag(0.1°⁻², 0.02 m⁻²); δ = log(T_init⁻¹ T_icp) |
| `*_da_wheel` | as `_da` | prediction's forward component = CAN distance; Λ forward = 1/σ²_wheel |
| `vgicp_grav` | vgicp | + §5 attitude correction |
| `grav_only` | none | `T_k = T_{k-1} P_k` + §5 |

Fallback to `T_init` when not converged, < 200 inliers, or |δt| > 1 m / |δθ| > 2°.
small_gicp's H is ordered [rotation, translation] (verified on a synthetic corridor).

## 7. Wheel (CAN) model

On the training block only: lag (speed-profile match, ±0.5 s), scale `Σ v_GT / Σ v_CAN`,
forward axis = mean body-frame velocity direction of the baseline. `+wheel` replaces the
forward component of each baseline step: `t' = t − (t·f) f + d_CAN f`. Code: `correct.wheel`.

## 8. Fusion model

Targets: GT steps `[fwd, lat, ψ̇]` (from GT positions and GT heading). Inputs per method:
tilt-invariant steps (§4 applied between consecutive GT times), mounting yaw removed (fitted
on train). Output: two masked softmaxes (translation, yaw) over methods; fused step =
weighted sum. Loss: Huber per step + 0.1·Huber of 2 s window sums. Ladder (best single →
static inverse-variance → linear → MLP-16 → MLP-64) chosen by 5-fold blocked CV, paired
one-SE rule on 200 m drift. Controls: shuffled context (context columns rolled by half the
drive), 3 seeds, train/val/test gap, test-road overlap. Code: `fusion.py`, `dataset.py`.

**Known label weakness:** per-step GT yaw comes from 10 Hz GNSS positions with ~7 cm jitter;
at 2 m steps that is ~2° of noise per step, so per-step yaw labels are noise-dominated
(regression of method yaw error on GT yaw shows a spurious −31 % 'scale' for every method).

## 9. Error decomposition (diagnostic, uses GT)

Counterfactual re-integration from the GT pose at span start, evaluated canonically:
`as_is` → `planar` (tilt removed) → `planar + GT heading` → `planar + GT heading + GT forward`.
The ATE drop at each stage is that component's contribution. Code: `audit.decompose`.

## 10. Leakage audit — where ground truth enters

| place | GT used? | allowed? |
|---|---|---|
| estimators (FAST-LIO, Faster-LIO, DLIO, GenZ-LIO, KISS-ICP) | no | — |
| planar (§4), gravity lock (§5), ICP correctors (§6) | no | — |
| body 'up' (§3) | no (IMU only, training block) | — |
| wheel lag / scale (§7) | **yes, training block** | yes (calibration) |
| mounting yaw, clock residual | **yes, training block** | yes (calibration) |
| fusion labels, early stopping, model selection | **yes, train / val** | yes |
| canonical alignment | **first 200 m of each evaluated span** | yes (initial fix) |
| decomposition (§9), CI, all metrics | yes | evaluation only |
| GenZ warm start | baseline poses only — **was non-causal by 0.3 s (central difference), fixed to a backward difference** | fixed |
| sliding-window compute accounting | — | **was 0 s for ROS supports (read a field only the GenZ runner writes), fixed to CPU-seconds** |

---

## 11. v3 additions

### 11.1 Offline gravity filter (`correct/heading.py::gravity_filter`)
Exact re-implementation of §5 on an estimator's recorded poses plus the IMU stream (cached in
`results/bag/imu.npz`): `T_k = T_{k-1} · (T^est_{k-1})⁻¹ T^est_k`, then the tilt correction of §5
with α = Δt/τ. It reproduces the scan-pass `grav_only` output (whole-drive ATE 966 vs 965 m).
τ = 5 s, chosen on the validation block from {0.5, 1, 2, 5, 10, 20, 50, 100} s (flat 0.5–10 s).

### 11.2 Heading treatments (`correct/heading.py::integrate`)
From tilt-free steps `[fwd, lat, ψ̇]` (§4, on the gravity-filtered poses):
`ψ_k = ψ_0 + Σ (ψ̇ − b·Δt − [Earth] Ω_E sin φ · Δt)`, `x_k = x_{k-1} + Rot(ψ_{k-1}) [fwd, lat (0 if no-slip)]`.
- **Yaw-rate bias b**: per block, slope of (source heading integrated − GT course) over time
  while moving; a pipeline uses the duration-weighted mean over its **training blocks only**.
- **Hybrid**: translation steps from source A; heading increments from source B, resampled on A's
  stamps via B's integrated heading (`resample_dyaw`).
- **Least-biased source**: the source with the smallest |b| on the training blocks.
- Earth rate Ω_E sin φ = 7.29e-5 · sin 63.24° rad/s (tested; not claimed, see REPORT_v3 §3.2).

### 11.3 Leave-one-block-out protocol (`v3.py`)
The drive is cut into K = 5 (or 10) equal-time blocks. For held-out block j: calibrations and
selections use blocks ≠ j; block j is scored with the canonical protocol (§2: 200 m start fit
inside the block, then prediction). Comparisons are **paired** per block: mean difference ± SE
over blocks, number of blocks improved, and an exact two-sided sign test (K = 10).

### 11.4 Learned fusion vs rule (`v3.py::learned_vs_rule`)
Per held-out block: static inverse-variance, linear gating and MLP-16 (fusion.py) trained on the
other blocks (minus 10 s gaps; the last 20 % of them for early stopping), on the tilt-invariant
step table with context features. The rule: translation from the baseline, ψ̇ from KISS-ICP.
Also "learned translation + KISS-ICP yaw".

### 11.5 Correction schedule and update form (`correct/scanpass.py`)
- `vgicp_grav_every{N}`: VGICP registration only on every N-th scan. The prediction (LIO step)
  is used otherwise, and **the map is updated every scan** with the current pose.
- `vgicp_grav_da`: information-weighted update `δ' = (I + Λ)⁻¹ I δ` (§6) + gravity lock:
  the filter-style counterpart of direct correction.
- `vgicp_grav_cvrot` / `vgicp_grav_cv`: the prediction's rotation (and translation) taken from
  the corrector's own previous step (constant-velocity model) instead of the LIO: the yaw-leakage test.

### 11.6 Distortion probes
- **KISS-ICP** (`correct/kiss_probe.py`): per-point times normalised to [0, 1] and then
  (nodeskew) deskew disabled, (revtime) t → 1 − t, (shift5ms) t → min(t + 0.05, 1);
  (vox05) 0.5 m voxels.
- **GenZ-LIO** (`genz_runner.run(time_scale=s)`): per-point times × s, s ∈ {0, 0.5}
  (0 = no motion compensation).

### 11.7 Noise-tuning probe
Faster-LIO re-run with `b_gyr_cov` 1e-4 → 1e-2 (`variants/faster_lio_bgyr100.yaml`) and with
`acc_cov`, `gyr_cov` 0.1 → 1.0 (`variants/faster_lio_imunoise10.yaml`).

### 11.8 Final system (REPORT_v3 §11)
- **Heading blend** (`correct/heading.py::blend_heading`, tested, not adopted):
  ψ_k = ψ_{k-1} + Δψ_LIO,k + (Δt/τ)(ψ_KISS,k − ψ_{k-1} − Δψ_LIO,k); τ chosen per fold on the
  training blocks from {1, 2, 5, 10, 20, 50, 100, 300} s.
- **KISS-ICP config selection:** {1.0, 1.5, 2.0} m voxels, chosen by ATE on the chronological
  validation block, scored once on test → 2.0 m.
- **Online node** (`ros/scripts/kiss_fusion_node.py`, config `configs/algorithms/kiss_fusion.yaml`):
  per LIO pose, T_G ← T_G·(T_LIO,k−1⁻¹ T_LIO,k), tilt correction (§5, τ = 5 s), level step
  [fwd, lat] of T_G, Δψ from KISS-ICP for the matched scan (|Δt| ≤ 20 ms), guard
  |Δψ̇_KISS − Δψ̇_LIO| ≤ 0.2 rad/s else Δψ_LIO, x ← x + Rot(ψ_{k-1})[fwd, lat]. Causal; KISS-ICP
  poses converted to the IMU frame with t_IL; rotations re-projected onto SO(3) every step.
- **Export** (`export.py`, `liobench export`): template rows kept; positions sampled at the
  rows' GNSS header stamps after the training-block clock offset; one SE(2) fit on the first
  200 m; UTM-33 → WGS84 for lat/lon.

## 12. Stage 3: snow-pole pipeline (REPORT_v3 §12)

- **Re-implementation** (`pipeline/snowpole_stage3.py`). `detect` phase:
  - streams signal/range images from the bag with rosbags;
  - runs the repo's YOLOv5 checkpoint via the pinned `ultralytics/yolov5:v7.0` hub release
    (conf 0.6);
  - takes the nearest LiDAR point in each box with the repo's
    `extract_nearest_point_in_bounding_box_region` (XYZLut on the uint32 range image × 4);
  - caches the detections.

  `run` phase: the original main loop step for step, using `geoloc_utils`:
  - heading and translation from consecutive CSV rows;
  - `calculate_new_positions`;
  - pole fixes only inside the pole map's bounding box, for detections ≤ 5 m;
  - nearest surveyed pole to the predicted pole position;
  - the vehicle placed at the back-azimuth from that pole.
- **Validation** (`pipeline/validate_against_original.py`): the original script is executed
  with runpy, unmodified. Only I/O is replaced (lazy bag reader with identical topics, order and
  stamps; pinned YOLO loader; headless matplotlib; contextily stub; `ouster.client` →
  `ouster.sdk.core` shim). Its per-frame running average is compared with the re-implementation's.
- **Reference:** dual-GNSS midpoint kriged (`pykrige`) to the range-image stamps, as in the original.
- **Heading reset** (`HeadingCorrector`):
  - between consecutive pole fixes with baseline ≥ 15 m, d = azimuth(fix→fix) −
    azimuth(odometry displacement over the same frames);
  - updates with |d| > 15° are rejected;
  - θ ← θ + 0.5·d, and θ is added to the odometry heading of every later step.
- **Drift rate:** after ≥ 5 accepted updates spanning ≥ 60 s, rate = OLS slope of θ(t) over all
  updates. θ is extrapolated as θ + rate·(t − t_last update), also after the last pole.
- **Metrics:**
  - **whole drive:** all 5 420 frames;
  - **pole span:** reference arc length between the first and last surveyed pole lying within
    15 m of the reference track;
  - **after last pole;**
  - **native:** the original's in-box running mean.
- **Sensitivity** (`pipeline/rate_sensitivity.py`): the rate is shifted by a fixed offset in
  {0, ±0.002, ±0.005, ±0.01} °/s, and fitted on windows of the last 300 / 600 s (void: the
  updates span 213 s).
- **Cross-track analysis** (`pipeline/make_pipeline_figures.py::p7_cross_track`):
  - the error is projected onto the reference track's tangent/normal;
  - pole lateral offsets are measured from the nearest reference sample;
  - "locked" means |cross-track| > 5 m.
