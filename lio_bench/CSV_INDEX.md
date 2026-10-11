# Where every CSV is and what it means

All paths are relative to the repo root. Only two CSVs are tracked in GitHub (marked **[git]**); everything else is untracked.

## 1. Repo root: inputs

| file | what it is |
|---|---|
| `incremental_navigation_results.csv` **[git]** | The odometry CSV the professor's pipeline reads (FastReg-era). We call it the "legacy CSV" and use it as a reference only. Only `easting`/`northing` matter: the pipeline takes heading and step length from consecutive rows. 5,422 rows, one per range image. |
| `Groundtruth_pole_location_at_test_site_E39_Hemnekjølen.csv` **[git]** | Surveyed snow-pole positions (290 poles, pairs on both sides of the road). This is the map the pole fix snaps to. |
| `fastlio.csv`, `fasterlio.csv` | The old FAST-LIO / Faster-LIO exports, in the same format as the navigation CSV. Checked in step 1 (`liobench verify`). The relative clock starts at t = 0. |
| `fastreg.csv` | Old FastReg export. **Ignored** (its weights were tuned on this drive). |
| `gps_left_position.csv`, `gps_right_position.csv` | The two GNSS antennas from the bag (lat/lon). Their midpoint is the vehicle reference. |
| `groundtruth_extracted.csv` | Ground truth used for all odometry scoring: UTM-33N easting/northing at 10 Hz, from the vehicle's GNSS/INS. |

## 2. `lio_bench/results/export/`: our odometry, in the pipeline's format

These are drop-in replacements for `incremental_navigation_results.csv`: same rows, same timestamps, but the positions come from our odometry. Each has a `.json` note.

| file | odometry |
|---|---|
| `incremental_navigation_fast_lio_full_raw.csv` | FAST-LIO2 raw (**baseline**) |
| `incremental_navigation_faster_lio_full_raw.csv` | Faster-LIO raw |
| `incremental_navigation_fast_lio_full_gravity.csv` | FAST-LIO2 + gravity correction |
| `incremental_navigation_faster_lio_full_gravity.csv` | Faster-LIO + gravity correction |
| `incremental_navigation_fast_lio_full_gravity_kiss_heading.csv` | FAST-LIO2 + gravity + KISS-ICP heading |
| `incremental_navigation_faster_lio_full_gravity_kiss_heading.csv` | Faster-LIO + gravity + KISS-ICP heading (offline) |
| `incremental_navigation_kiss_fusion_rt_fusion_v20_run.csv` | **Final system**: the real-time node (Faster-LIO + gravity + KISS heading, 2 m voxels) |

## 3. `lio_bench/results/pipeline/`: snow-pole pipeline outputs (stage 3)

There are 24 runs: 8 inputs × 3 stage-3 modes. Each run is named `<input>__<mode>`.
- **Inputs:** `fastlio_raw`, `fasterlio_raw`, `fastlio_g`, `fasterlio_g`, `fastlio_gk`, `fasterlio_gk`, `final_online`, `legacy_csv`.
- **Modes:**
  - `none`: original pipeline;
  - `reset`: + heading reset at poles;
  - `reset_rate`: + heading reset + drift rate.

| file | columns / meaning |
|---|---|
| `<run>_frames.csv` | One row per range-image frame (5,420). See the column list below. |
| `<run>_heading_updates.csv` | One row per heading measurement between two pole fixes (stage-3 modes only). See the column list below. |
| `<run>_summary.json` | The numbers in the tables: whole drive, pole span, after the last pole, native metric. |
| `detections_cache.pkl` | YOLO pole detections + LiDAR range per frame, computed once and reused by every run. |

Columns of `<run>_frames.csv`:
- `t`: frame time;
- `pred_e`, `pred_n`: the pipeline's position;
- `ref_e`, `ref_n`: GNSS reference (antenna midpoint);
- `err_m`: distance between them;
- `in_box`: inside the pole map's bounding box (the original's metric uses only these frames);
- `s_km`: distance along the drive.

Columns of `<run>_heading_updates.csv`:
- `measured_deg`: heading error seen over that stretch;
- `accepted`: passed the 15° gate;
- `theta_deg`: the running correction;
- `rate_deg_s`: the learned drift rate;
- `baseline_m`: length of the stretch.

The aggregated tables are in `PIPELINE_RESULTS.md`, `RATE_SENSITIVITY.md` and `all_results.json`, not in CSVs.

## 4. `lio_bench/results/runs/<algorithm>/<run>/`: raw odometry runs

Folder names:
- **algorithm:** `fast_lio`, `faster_lio`, `dlio`, `genz_lio`, `kiss_fusion` (the real-time node), and post-correctors such as `faster_lio+vgicp_grav`.
- **run:** `full` is the whole drive; `full_r2` / `full_r3` are repeats; `smoke30` is a 30 s test; `concurrent_A+B` means two algorithms ran at once; `win_*` are sliding-window runs; `rt_fusion_v20` is the final real-time node.

| file | meaning |
|---|---|
| `trajectory_tum.txt`, `odom_sensor_clock.tum` | **The actual trajectory.** These are TUM text files, not CSVs: `t x y z qx qy qz qw`. |
| `odom_recv.csv` | `stamp, wall`: when each pose was published vs its scan time. Used for latency. |
| `cloud_recv.csv` | `stamp, wall`: when each scan arrived. Used for dropped-scan counts. |
| `resources.csv` | `wall, algo, cpu_pct, rss_mb, threads`: CPU/RAM over time. |
| `diagnostics.csv` | Per-scan internals. For the correctors: inliers, correction applied, eigenvalues. For GenZ-LIO: matches, leaf size, processing time. |
| `meta.json`, `node.log` | Config, commit and log. |

## 5. Analysis CSVs (inputs to the earlier reports)

| file | meaning |
|---|---|
| `lio_bench/results/bag/utm_position.csv` | Vehicle GNSS/INS position from the bag (UTM). |
| `lio_bench/results/bag/enu_heading.csv` | Heading from the bag. |
| `lio_bench/results/bag/gps_state.csv` | Fix status, antenna heading, std. devs. |
| `lio_bench/results/bag/steering.csv` | CAN speed + steering angle (the "wheel" data). |
| `lio_bench/results/features/scan_context.csv` | Per-scan scene descriptors: points, ranges, planarity, degeneracy. |
| `lio_bench/results/analysis/window_errors.csv` | Per-window errors + scene features (sliding-window analysis, REPORT v1). |
| `lio_bench/results/fusion/<lio>{,_correctors}/weights_holdout.csv` | Learned fusion weights per method over time (REPORT v1/v2). |
| `lio_bench/results/fusion/<lio>{,_correctors}/trigger_predictions.csv` | Predicted vs true step error for the "when to switch" trigger. |
| `lio_bench/results/poles/detections_*.csv` | LiDAR-only pole detections (geometric, not YOLO) from an early corrector experiment. |
| `lio_bench/results/v3/leak_scene_windows.csv` | Per-window yaw bias of every method vs scene features (the REPORT v3 leakage test). |
