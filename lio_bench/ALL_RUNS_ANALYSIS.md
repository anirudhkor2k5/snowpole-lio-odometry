# Everything we ran: how much GNSS each result used, what came out, and where the figures are

*2026-10-12. One drive: E39 Hemnekjølen, 2024-02-28, 10.3 km, 542 s. All paths are relative to
`lio_bench/` unless they start with the repo root. Nothing in the professor's repo was changed or
committed. Older, longer write-ups: `FULL_ANALYSIS.md` (everything up to 2026-10-11),
`STUDY_SUMMARY.md` + `results/REPORT.md` (v1), `REPORT_v2.md` (audit), `REPORT_v3.md` (heading +
online node), `METHODS.md` (equations), `CSV_INDEX.md` (every CSV). This file adds the option B
work and answers three questions directly.*

---

## 0. Short answers

### 0.1 Is there a "200 m GNSS fit" anywhere else?

**Yes. Every result except the option B files used the 200 m fit, and some used more GNSS.**

| result | how GNSS was used to place / score it |
|---|---|
| **Option B CSVs** (`results/export_optionB/`, `results/export_optionB_rerun/`) | **One** dual-antenna heading at the first frame (car parked), to rotate the whole track once. No 200 m fit, no clock-offset correction, nothing after t0. |
| 200 m CSVs (`results/export/`) | One SE(2) fit (rotation + shift) on the first 200 m, **plus** a clock offset fitted against GNSS on the training block. |
| Professor's pipeline runs, all 24 (`results/pipeline/`, figures `p1–p7`, rate sensitivity) | Fed with the `results/export/` CSVs, so **200 m fit**. |
| Headline odometry numbers (51.2 → 6.1 m per km block, drift %, heading error; REPORT_v2/v3, `results/v3/`) | Each of the 10 held-out ~1 km blocks gets **its own 200 m start fit**: 10 fits per drive. These numbers measure "how far does it drift over ~1 km after a good start". |
| Corrector tables (`results/correct/*/correct_summary.md`) | Per block, the block's own first 200 m. |
| v1 tables (`results/REPORT.md`, `results/verify/`) | `se2_head` = 200 m fit; `se2_full` = best rigid fit to GNSS over the **whole drive** (evaluation only, uses all GNSS); clock offset fitted per run. |
| `verify/verify_faster_lio.py` | 200 m fit + a ±1 s clock search. |

Two more kinds of GNSS use apply to every result, option B included:

- **Design choices made once using GNSS on training/validation blocks**, then frozen: the gravity
  filter τ = 5 s, KISS-ICP as the heading source, the KISS voxel size 2.0 m (node), and
  `point_filter_num: 3`. These are tuning, not GNSS fed to the odometry while it runs, but they were
  picked by looking at GNSS. The learned fusion (v1) was trained on GNSS labels. It is not used in
  any CSV.
- **The professor's pipeline itself** starts from the GNSS position at frame 0, uses the
  GNSS-surveyed pole map, and is scored against GNSS. That is the same for every input file.

So for a fair "pure odometry" comparison, only option B matches the rule "GNSS for the first frame
only". The 200 m numbers are not wrong, but they get a better start heading than a real system would
have.

### 0.2 Voxel sizes: offline KISS-ICP 1.0 m, online fusion node 2.0 m. Why?

Correct. The difference is **historical, not a design decision**:

- **Offline (file 3), 1.0 m.** The scan pass (`src/liobench/correct/scanpass.py`, line 294) hard-codes
  1.0 m. That is KISS-ICP's own default for a 100 m range (range / 100). It ran in Phase 1, before
  anyone looked at voxel size. The offline export kept using that run (`runs/kiss_icp/full`, and
  now `runs/kiss_icp/rerun_b`).
- **Online node (file 4), 2.0 m.** Chosen later for two reasons:
  1. **Validation sweep** (REPORT_v3 §11.3, heading source for the Faster-LIO hybrid, 200 m protocol):

     | KISS config | ms / scan | val ATE | test ATE |
     |---|---:|---:|---:|
     | 1.0 m (default) | 28.5 | 38.6 m | 9.4 m |
     | 1.5 m | 15.9 | 18.9 m | 12.2 m |
     | **2.0 m** | **11.0** | **11.2 m** | **8.7 m** |

  2. **Real time.** At 1.0 m the node fell behind: 185 dropped scans, 2.2 s backlog, 10.0 m per
     block. At 2.0 m it had 0 dropped, 47 ms latency and 6.1 m per block. Dropped scans make KISS-ICP
     deskew across double gaps, which feeds the exact heading bias we are trying to remove.

### 0.3 Which is better?

**2.0 m, on every test we have.** After your question I checked it on option B directly. The
offline file 3 was rebuilt from the 2.0 m sweep run that already exists (`runs/kiss_icp_vox20/full`).
This was an export only, no new bag run, written to `/tmp/vox20/`:

| file 3 (Faster-LIO + gravity + KISS heading, offline), option B | KISS run | at first pole | at last pole | end of drive |
|---|---|---:|---:|---:|
| earlier (`export_optionB/`) | 1.0 m, `kiss_icp/full` | 20.9 m | 318 m | 1 051 m |
| fresh rerun (`export_optionB_rerun/`) | 1.0 m, `kiss_icp/rerun_b` | 20.7 m | 274 m | 792 m |
| 1.5 m | `kiss_icp_vox15/full` | 21.1 m | 333 m | 1 087 m |
| **2.0 m** | **`kiss_icp_vox20/full`** | **20.8 m** | **146 m** | **132 m** |
| (online node, file 4, 2.0 m, for comparison) | `kiss_fusion/rerun_b_fusion` | 21.3 m | 358 m | 962 m |

(Distance from GNSS at the given point of the drive; no pipeline run.)

- 2.0 m roughly halves the error at the last pole and cuts the end error ~6–8×.
- **Caveat:** each line is a single KISS-ICP run, and KISS-ICP is not exactly repeatable (the two
  1.0 m runs differ by 260 m at the end). The direction agrees with the validation sweep, though,
  so it's very likely not luck.
- The online node uses 2.0 m but ends worse (962 m) than the offline 2.0 m file (132 m). In real
  time, KISS-ICP still misses some scans (45 fallbacks to the LIO's heading), and it starts on a
  different scan. On the 200 m protocol it was the best (6.1 m per km block), so per-km quality
  and the 10 km end point don't rank the same way.
- **Recommendation:** make file 3 with 2.0 m. That is either an export from the existing 2.0 m run
  (seconds), or a fresh bag pass with the voxel changed (~5 min). Not done yet. Your four CSVs are
  unchanged.

---

## 1. The four option B CSVs (what you feed the professor's code)

**Folder:** `results/export_optionB_rerun/` (fresh runs on the bag, 2026-10-12). Each CSV has a
`.json` next to it with the source run, the rotation applied, and its error vs GNSS.

| # | file | built from | rotation applied | first pole | last pole | end |
|---|---|---|---:|---:|---:|---:|
| 1 | `incremental_navigation_faster_lio_full_raw.csv` | `runs/faster_lio/rerun_b` | +5.98° | 32.3 m | 784 m | 3 307 m |
| 2 | `incremental_navigation_faster_lio_full_gravity.csv` | same + gravity filter (τ 5 s) | +5.89° | 28.8 m | 587 m | 2 513 m |
| 3 | `incremental_navigation_faster_lio_full_gravity_kiss_heading.csv` | same + heading from `runs/kiss_icp/rerun_b` (1.0 m) | +5.89° | 20.7 m | 274 m | 792 m |
| 4 | `incremental_navigation_kiss_fusion_rt_fusion_v20_run.csv` | `runs/kiss_fusion/rerun_b_fusion` (online node, 2.0 m) | +5.92° | 21.3 m | 358 m | 962 m |

- **How they are made** (`pipeline/export_option_b.py`, run by `scripts/rerun_option_b.sh`):
  1. Take the trajectory.
  2. Rotate the whole thing once, so that its heading at the first pose equals the dual-antenna
     heading in row 0 of `gps_left/right_position.csv`. That reading is compass 84.01°, ENU 5.99°.
  3. Sample it at the 5 422 GNSS row stamps of the template CSV.
  4. Clear the FastReg columns.

  There is no clock offset (the 0.06 s measured offset is recorded in the json, not applied). The
  start position doesn't matter: the pipeline starts from GNSS frame 0 and uses only row-to-row
  steps.
- **Why not the INS heading:** `/vehicle_enu_heading` never sets its `heading_init` /
  `dual_antenna_heading` flags in this bag and reads 4–9° low while driving. With it, raw
  Faster-LIO got 13.0 m in-box instead of 10.2 m (`verify/start_heading_options_output.txt`).
- **Runtime of the rerun on this laptop:**

  | step | time |
  |---|---|
  | Faster-LIO, real time | 9.5 min |
  | KISS-ICP scan pass | ~5.5 min |
  | Faster-LIO + fusion node, real time | 9.5 min |
  | export | seconds |
  | **total** | **~25 min** |

  Log: `results/logs/rerun_option_b.log`, `rerun_option_b_resume.log`.
- **Repeatability:**
  - Files 1–2 are identical to the earlier option B set (`results/export_optionB/`). Faster-LIO
    (sequential build) is deterministic: all 5 411 poses match.
  - File 3 changed: 318/1 051 m → 274/792 m at the last pole / end.
  - File 4 changed a little: 350/942 m → 358/962 m.

  KISS-ICP is multi-threaded and not exactly repeatable. The fresh scan pass also started 2 scans
  later, and the gap grows over the drive (final yaw −10.2° vs −7.4°).
- **Risk at the first pole:** all four are 21–32 m off at the first pole (km 0.72). That is more
  than half the 18 m pole spacing. The original pipeline snaps to the *nearest* pole, so it may lock
  onto the wrong pole and stay there. The 200 m versions were 7–15 m off at that point (raw 14.6,
  gravity 12.5, offline KISS 7.7, node 7.4 m). Expect the
  option B in-box numbers to be worse than the 200 m ones (10.2 / 9.3 / 5.3 / 7.4 m). **The pipeline
  has not been run on the option B files:** you said you'd do that.
- **Figures:** none. Option B was CSV-only, with no pipeline runs.

---

## 2. Everything that was run, in order

Each phase lists what ran, the main result, and the figures, with paths.

### Phase 0: getting the estimators to run (2026-10-10)

- **What:**
  - FAST-LIO2 `7cc4175`, Faster-LIO `8038fba`, DLIO `fc8d183` and GenZ-LIO `c6c464d`, official
    code with build-only patches (`third_party_patches/`).
  - Real-time ROS replay of the bag (`scripts/run_ros.sh` → `ros/scripts/run_ros.py`), every heavy
    job under a RAM cap (`scripts/capped.sh`).
- **Fixes found:**
  - Faster-LIO registered only 24 % of the distance with `point_filter_num: 4`, because it aliases
    with the OS2's 1 024 columns. Setting 3 fixed it. Runs: `runs/faster_lio/smoke30` (4) vs
    `smoke30_pf3` (3).
  - GenZ-LIO ran out of memory until `map_range: 300`.
  - Ouster clock → Unix fit, residual 0.14 ms: `results/bag/sensor_clock.json`.
- **Figures:** none.

### Phase 1 (v1): baselines and the main failure (2026-10-10)

| step | result | figures |
|---|---|---|
| **Verify the legacy CSVs** (`liobench verify`) | `fastlio.csv` / `fasterlio.csv` are genuine; their stamps are 0.4–0.6 s late. | `results/verify/map_none.png`, `map_se2_head.png`, `map_se2_full.png`, `error_vs_distance_se2_head.png`, `error_vs_distance_se2_full.png`; text `results/verify/verify_summary.md` |
| **Four LIOs, full drive** | 200 m fit, final error: Faster-LIO 3.0 km, DLIO 2.7 km, GenZ 3.4 km, FAST-LIO 3.6 km. Best whole-drive fit ATE: 382 / 396 / 455 / 500 m. Every track 5–9 % short. Table: `results/REPORT.md`. | — |
| **Analysis** (`liobench analyze`) | Step lengths are right to 0.2 %, so the short tracks are **tilt (pitch), not scale**. The car "climbs" 2.4–3.4 km. The weak scan direction is along the road. | `results/analysis/scale_error_vs_time.png`, `yaw_error_vs_time.png`, `map_<algo>_step_error.png` (×4), `map_best_method.png` |
| **Concurrency** (2 LIOs at once) | All pairs run in real time with no accuracy loss. | none; `results/concurrency/*.json` |
| **Correctors** (`liobench correct …`): planar re-integration, CAN wheel, ICP/plane-ICP/GICP/VGICP (+ degeneracy-aware), geometric poles | Planar is the big free win (Faster-LIO test drift 13.4 % → 1.2 %); GICP/VGICP + planar ~0.65 %; wheel helps only after planar; geometric poles failed (6/290 matched). | `results/correct/faster_lio/drift_by_block.png`, `results/correct/fast_lio/drift_by_block.png`; tables `correct_summary.md` |
| **Learned fusion** (gating MLP mixing the methods' steps, trained on GNSS labels) | Helps FAST-LIO (test −46 % drift) but is not proven for Faster-LIO after the audit. Not used in any CSV. | `results/fusion/<fast_lio / faster_lio>[_correctors]/test_block_trajectories.png`, `translation_weights_vs_time.png`, `map_baseline_weight.png` |
| **Sliding windows** (support LIO only when a trigger predicts trouble) | At 10–16 % duty, keeps only ~30 % of the benefit; GenZ windows can hurt. | `results/windows/<base>/<variant>/fused_trajectories.png`, `transient_after_window_start.png` |

### Phase A (v2): audit, 2026-10-11 (`REPORT_v2.md`, `results/audit/audit.json`)

- **Canonical protocol** used from here on:
  - the clock offset is fitted on the training block;
  - **one 200 m start fit** per evaluated span;
  - then pure prediction.
- **~97 % of the 10 km error is heading.** Given the true heading, Faster-LIO's error drops from
  1 245 m to 30 m.
- **Every method drifts counter-clockwise at a constant rate per second**, i.e. a bias:
  Faster-LIO 0.06 °/s, KISS-ICP 0.02 °/s.
- **The tilt failure is pitch, not roll.** A gravity-locked VGICP cuts drift from 8.4 % to 1.37 %,
  online.
- **Ground truth** is the dual-antenna midpoint (RTK float): fine at metre level, too noisy for
  per-step yaw.
- **Figures:** none. Numbers are in `audit.json` and the REPORT_v2 tables.

### v3: heading, the final system, the online node (2026-10-11, `REPORT_v3.md`)

- **10 held-out ~1 km blocks, each with its own 200 m start fit.** ATE per block:

  | pipeline | FAST-LIO2 | Faster-LIO | DLIO | GenZ-LIO |
  |---|---:|---:|---:|---:|
  | raw | 51.2 m | 28.9 m | 48.6 m | 47.7 m |
  | + gravity (τ 5 s) | 9.6 m | 12.7 m | | |
  | + gravity + KISS heading (1.0 m) | 7.4 m | 7.7 m | 7.8 m | 7.2 m |
  | **online node (Faster-LIO, 2.0 m)** | | **6.1 m** | | |

- **Gravity filter.** τ = 5 s, chosen on validation; the result is flat for τ = 0.5–10 s.
- **KISS-ICP heading.** Wins 9/10 blocks for both LIO baselines.
- **Probes.** The heading bias scales with uncompensated motion distortion; the pitch climb is
  carried by ground returns beyond ~10 m.
- **Rejected, each by a direct test:** Earth rate, IMU-noise tuning, gyro-bias initialisation,
  gyro-bias wander, selective (every N-th scan) correction, CAN wheel speed, and a heading blend.
- **KISS voxel sweep:** see §0.2.
- **Online node.** 0 dropped scans, 47 / 72 ms latency, 71 % CPU, 148 MB; Faster-LIO next to it
  is unaffected.
- **Figures** (`results/v3/figures/`):

  | file | shows |
  |---|---|
  | `f0_map.png` | tracks |
  | `f1_height_profile.png` | the climb |
  | `f2_heading_error.png` | heading error over time |
  | `f3_kiss_distortion_probes.png` | KISS-ICP probes |
  | `f4_duty_cycle.png` | selective correction |
  | `f5_bias_per_block.png` | yaw bias per block |

### Export, 200 m version (2026-10-11, `results/export/`)

- **How it's built:** 7 CSVs. Clock offset from the training block, **200 m SE(2) fit**, sampled on
  the 5 422 GNSS rows.
- **Final error at km 10:**

  | input | final error |
  |---|---:|
  | Faster-LIO raw | 3 110 m |
  | Faster-LIO + gravity | 2 327 m |
  | Faster-LIO + gravity + KISS, offline | 891 m |
  | online node | 777 m |

  FAST-LIO versions also exist.
- **Figures:** none.

### Stage 3: the professor's pipeline on the 200 m CSVs (2026-10-11, `results/pipeline/`)

- **Re-implementation check.** `pipeline/snowpole_stage3.py` matches the unmodified
  `snowpole_based_vehicle_localization.py` frame by frame (max difference 7.4e-5 m; 13.6324 m both,
  358 fixes). Log: `results/logs/original_pipeline.log`.
- **24 runs:** 8 inputs × 3 stage-3 modes. The modes are:
  - original;
  - + heading reset from pole fixes;
  - + drift rate.

  Full tables: `results/pipeline/PIPELINE_RESULTS.md`.
- **Native in-box metric, original stage 3 (mean):**

  | input | in-box mean |
  |---|---:|
  | Faster-LIO raw | 10.2 m |
  | + gravity | 9.3 m |
  | + gravity + KISS offline | **5.3 m** |
  | online node | 7.4 m |
  | FAST-LIO2 raw | 10.5 m |
  | legacy FastReg CSV | 13.6 m |

  With the heading reset, every input gets ~2.0–2.5 m.
- **Whole drive, final error, original → + heading reset:**

  | input | original | + heading reset |
  |---|---:|---:|
  | Faster-LIO raw | 2 484 m | 1 087 m |
  | Faster-LIO + G + KISS offline | 672 m | 249 m |
  | online node | 526 m | 151 m |

  The online node with drift rate gets 7.2 m, but `RATE_SENSITIVITY.md` shows that is partly luck.
  The honest range is ~7–50 m.
- **Finding: the ~10 m floor of the original pipeline is wrong-side pole association.** Poles
  stand in pairs 8.8 m apart across the road.
- **Figures** (`results/pipeline/figures/`):

  | file | shows |
  |---|---|
  | `p1_validation.png` | ours vs original |
  | `p2_error_vs_distance.png` | error along the drive |
  | `p3_pole_span.png` | pole-span error |
  | `p4_summary_bars.png` | all 24 runs |
  | `p5_rate_sensitivity.png` | drift-rate sensitivity |
  | `p6_map.png` | map |
  | `p7_cross_track.png` | wrong-side lock |

  Also `results/pipeline/error_vs_distance__none.png`, `__reset.png`, `__reset_rate.png`.

### Is our Faster-LIO real? (2026-10-11, `verify/VERIFY_FASTER_LIO.md`)

- **Script:** `verify/verify_faster_lio.py`, numpy only, 200 m fit. Output:
  `verify/verify_faster_lio_output.txt`.
- **Final error:**

  | trajectory | final error |
  |---|---:|
  | old `fasterlio.csv` | 2 696 m |
  | **our Faster-LIO** | **3 113 m** |
  | our FAST-LIO2 | 3 717 m |
  | old `fastlio.csv` | 3 964 m |

- **Our Faster-LIO is not "too good".** It is slightly worse than the independent old export.
- **The raw baselines are all bad.** The good numbers come from gravity, KISS heading and pole
  resets.
- **Figures:** none.

### Your friend's 53.7 m (2026-10-11)

| script (`verify/`) | what it showed |
|---|---|
| `export_ablation.py` → `export_ablation_output.txt` | Our export's steps don't matter: no clock offset, other fit lengths and jitter all stay 9–10.5 m in-box. Only heading (±4° → 7.9–11.9 m) and distance scale (×0.95 → 50 m) move it. |
| `diagnose_nav_csv.py` | Per CSV: distance at km 0.2 / 0.5 / 0.72 / 1.0, in-box metric, along/cross-track split. |
| `align_nav_csv.py` | The friend's file was rotated **4.46°** wrong at the start. With a 200 m fit it goes 53.7 → **9.0 m** (`verify/faster_lio_full_aligned.csv`). |
| `start_heading_options.py` → `start_heading_options_output.txt` | A (200 m fit) 10.2 m; B with INS heading 13.0 m; friend as is 53.7 m. With heading reset: 2.0 / 2.4 / 55.9 m. |
| `tum_to_nav_csv.py` | Converter for the friend: their `traj.txt` → pipeline CSV, option B (antenna heading). |

- **Conclusion:** their Faster-LIO is fine; the start heading was off. Their yaml extrinsic was also
  wrong, but this barely matters. Correct values: identity rotation, T = [−0.006253, 0.011775,
  −0.007645] m (from the bag's `Trip068.json`).
- **Figures:** none.

### Option B (2026-10-11/12)

| set | made from | status |
|---|---|---|
| `results/export_optionB/` | the earlier runs (`faster_lio/full`, `kiss_icp/full`, `kiss_fusion/rt_fusion_v20`) | superseded, kept |
| **`results/export_optionB_rerun/`** | **fresh bag runs** (`rerun_b`, `rerun_b_fusion`) | **use these** (§1) |
| `/tmp/vox20/` | voxel check for file 3 (§0.3) | temporary, not a deliverable |

- `pipeline/run_option_b.py` (pipeline + sensitivity on option B) exists but was stopped on your
  request. Its output folder was deleted, and `results/pipeline/` was verified unchanged.

---

## 3. All figures in one list

| folder | files | phase |
|---|---|---|
| `results/verify/` | `map_none`, `map_se2_head`, `map_se2_full`, `error_vs_distance_se2_head`, `error_vs_distance_se2_full` | v1 legacy CSV check |
| `results/analysis/` | `scale_error_vs_time`, `yaw_error_vs_time`, `map_{fast_lio,faster_lio,dlio,genz_lio}_step_error`, `map_best_method` | v1 tilt discovery |
| `results/correct/{fast_lio,faster_lio}/` | `drift_by_block` | v1 correctors |
| `results/fusion/{fast_lio,faster_lio}[_correctors]/` | `test_block_trajectories`, `translation_weights_vs_time`, `map_baseline_weight` | v1 learned fusion |
| `results/windows/{fast_lio,faster_lio}/<variant>/` | `fused_trajectories`, `transient_after_window_start` | v1 sliding windows |
| `results/v3/figures/` | `f0_map` … `f5_bias_per_block` | v3 heading + node |
| `results/pipeline/figures/` | `p1_validation` … `p7_cross_track` | stage 3, 200 m CSVs |
| `results/pipeline/` | `error_vs_distance__{none,reset,reset_rate}` | stage 3, 200 m CSVs |

There are no figures for option B, the Faster-LIO verification, or the friend's-file
investigation; those are text outputs in `verify/`.

---

## 4. What I'd do next (not started)

1. **Make file 3 with KISS-ICP at 2.0 m** (§0.3): 132 m end error vs 792 m. Cheapest real gain.
2. **Run the professor's code on the option B files** (you're doing this). Watch for wrong-pole
   lock at the first pole: all four are 21–32 m off there.
3. **For the paper,** report option B as the honest "GNSS at t0 only" result. Keep the 200 m
   numbers as the "good initial heading" upper bound, and say which is which.
