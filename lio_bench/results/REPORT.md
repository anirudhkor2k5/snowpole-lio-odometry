# lio_bench — full report

Generated 2026-10-11 00:43. Every section below is also a separate file under results/ (paths given).

## Pipeline status

- finished steps (40): analyze, concurrency_report_fast_lio+dlio, concurrency_report_fast_lio+genz_lio, concurrency_report_faster_lio+dlio, concurrency_report_faster_lio+genz_lio, concurrent_fast_lio+dlio, concurrent_fast_lio+genz_lio, concurrent_faster_lio+dlio, concurrent_faster_lio+genz_lio, correct_poles_fast_lio, correct_poles_faster_lio, correct_report_fast_lio, correct_report_faster_lio, correct_scan_fast_lio, correct_scan_faster_lio, correct_wheel_fast_lio, correct_wheel_faster_lio, fusion_fast_lio, fusion_fast_lio_correctors, fusion_faster_lio, fusion_faster_lio_correctors, genz_python_full, prepare, ros_dlio, ros_fast_lio, ros_fast_lio_pf4_legacy, ros_faster_lio, ros_genz_lio, unit_tests, verify, windows_fast_lio_dlio_adaptive_cold, windows_fast_lio_genz_adaptive_cold, windows_fast_lio_genz_adaptive_q90, windows_fast_lio_genz_adaptive_warm, windows_fast_lio_genz_periodic_warm, windows_faster_lio_dlio_adaptive_cold, windows_faster_lio_genz_adaptive_cold, windows_faster_lio_genz_adaptive_q90, windows_faster_lio_genz_adaptive_warm, windows_faster_lio_genz_periodic_warm
- **not finished / failed** (0): none

## Every run (whole run span, rigid planar fit)

| run | tag | duration s | ATE se2_full m | length ratio | drift % | ms/scan |
|---|---|---:|---:|---:|---:|---:|
| dlio | concurrent_fast_lio+dlio | 542 | 408.2 | 0.922 | 7.64 |  |
| dlio | concurrent_faster_lio+dlio | 542 | 398.6 | 0.922 | 7.77 |  |
| dlio | full | 542 | 393.5 | 0.923 | 7.72 |  |
| dlio | smoke30 | 30 | 0.3 | 1.006 | nan |  |
| dlio | win_fast_lio_dlio_adaptive_cold_wu5_q0.75_000 | 10 | 5.4 | 0.854 | 25.95 |  |
| dlio | win_fast_lio_dlio_adaptive_cold_wu5_q0.75_001 | 11 | 5.3 | 0.863 | 23.21 |  |
| dlio | win_fast_lio_dlio_adaptive_cold_wu5_q0.75_002 | 10 | 5.8 | 0.844 | 13.52 |  |
| dlio | win_faster_lio_dlio_adaptive_cold_wu5_q0.75_000 | 10 | 5.4 | 0.854 | 22.21 |  |
| dlio | win_faster_lio_dlio_adaptive_cold_wu5_q0.75_001 | 10 | 5.8 | 0.845 | 13.46 |  |
| dlio+planar | full | 542 | 140.6 | 1.001 | 1.20 |  |
| fast_lio | concurrent_fast_lio+dlio | 541 | 489.3 | 0.905 | 9.40 |  |
| fast_lio | concurrent_fast_lio+genz_lio | 541 | 492.8 | 0.908 | 9.20 |  |
| fast_lio | full | 541 | 497.3 | 0.906 | 9.32 |  |
| fast_lio | full_pf4_legacy | 541 | 517.1 | 0.908 | 9.18 |  |
| fast_lio | smoke30 | 30 | 1.0 | 0.974 | nan |  |
| fast_lio | smoke30_pf3 | 29 | 0.3 | 1.006 | nan |  |
| fast_lio+gicp | full | 541 | 419.3 | 0.910 | 8.86 | 36 |
| fast_lio+gicp | smoke30 | 29 | 0.3 | 1.007 | nan | 28 |
| fast_lio+gicp+planar | full | 541 | 124.6 | 1.001 | 1.08 |  |
| fast_lio+gicp_da | full | 541 | 420.8 | 0.909 | 8.95 | 34 |
| fast_lio+gicp_da | smoke30 | 29 | 0.3 | 1.007 | nan | 22 |
| fast_lio+gicp_da+planar | full | 541 | 122.9 | 1.001 | 1.07 |  |
| fast_lio+gicp_da+planar+poles | full | 541 | 121.8 | 1.001 | 1.07 |  |
| fast_lio+gicp_da_wheel | full | 541 | 422.1 | 0.909 | 8.93 | 34 |
| fast_lio+gicp_da_wheel | smoke30 | 29 | 0.3 | 1.007 | nan | 22 |
| fast_lio+gicp_da_wheel+planar | full | 541 | 123.0 | 1.001 | 1.07 |  |
| fast_lio+gicp_da_wheel+planar+poles | full | 541 | 121.8 | 1.002 | 1.06 |  |
| fast_lio+icp | full | 541 | 323.2 | 0.917 | 8.06 | 52 |
| fast_lio+icp | smoke30 | 29 | 0.3 | 1.004 | nan | 48 |
| fast_lio+icp+planar | full | 541 | 86.6 | 0.994 | 1.46 |  |
| fast_lio+planar | full | 541 | 183.2 | 0.998 | 1.19 |  |
| fast_lio+planar+poles | full | 541 | 182.5 | 0.998 | 1.20 |  |
| fast_lio+plane_icp | full | 541 | 402.3 | 0.920 | 7.92 | 44 |
| fast_lio+plane_icp | smoke30 | 29 | 0.3 | 1.005 | nan | 38 |
| fast_lio+plane_icp+planar | full | 541 | 152.6 | 0.998 | 1.22 |  |
| fast_lio+plane_icp_da | full | 541 | 402.2 | 0.921 | 7.84 | 36 |
| fast_lio+plane_icp_da | smoke30 | 29 | 0.3 | 1.005 | nan | 25 |
| fast_lio+plane_icp_da+planar | full | 541 | 154.7 | 0.998 | 1.18 |  |
| fast_lio+vgicp | full | 541 | 419.4 | 0.916 | 8.33 | 8 |
| fast_lio+vgicp | smoke30 | 29 | 0.3 | 1.009 | nan | 7 |
| fast_lio+vgicp+planar | full | 541 | 146.5 | 1.001 | 1.14 |  |
| fast_lio+vgicp+planar+poles | full | 541 | 146.4 | 1.001 | 1.15 |  |
| fast_lio+wheel | full | 541 | 496.1 | 0.907 | 9.90 |  |
| fast_lio+wheel+planar | full | 541 | 179.6 | 1.000 | 2.10 |  |
| faster_lio | concurrent_faster_lio+dlio | 541 | 379.7 | 0.953 | 5.02 |  |
| faster_lio | concurrent_faster_lio+genz_lio | 541 | 380.1 | 0.953 | 5.04 |  |
| faster_lio | full | 541 | 379.7 | 0.953 | 5.02 |  |
| faster_lio | smoke30 | 30 | 23.5 | 0.239 | nan |  |
| faster_lio | smoke30_blind4 | 30 | 23.5 | 0.239 | nan |  |
| faster_lio | smoke30_extest | 30 | 23.5 | 0.237 | nan |  |
| faster_lio | smoke30_ivox1 | 30 | 23.4 | 0.242 | nan |  |
| faster_lio | smoke30_pf3 | 30 | 0.2 | 1.004 | nan |  |
| faster_lio | smoke30_seq | 29 | 23.4 | 0.240 | nan |  |
| faster_lio | smoke30_upstream | 29 | 0.3 | 1.003 | nan |  |
| faster_lio | smoke_t60 | 30 | 157.9 | 0.009 | 100.46 |  |
| faster_lio+gicp | full | 541 | 427.8 | 0.909 | 8.94 | 36 |
| faster_lio+gicp+planar | full | 541 | 130.0 | 1.001 | 1.12 |  |
| faster_lio+gicp_da | full | 541 | 428.8 | 0.909 | 8.90 | 34 |
| faster_lio+gicp_da+planar | full | 541 | 132.8 | 1.001 | 1.10 |  |
| faster_lio+gicp_da+planar+poles | full | 541 | 133.4 | 1.001 | 1.12 |  |
| faster_lio+gicp_da_wheel | full | 541 | 429.0 | 0.911 | 8.74 | 34 |
| faster_lio+gicp_da_wheel+planar | full | 541 | 138.4 | 1.001 | 1.11 |  |
| faster_lio+gicp_da_wheel+planar+poles | full | 541 | 138.6 | 1.002 | 1.13 |  |
| faster_lio+icp | full | 541 | 331.3 | 0.924 | 7.51 | 50 |
| faster_lio+icp+planar | full | 541 | 118.4 | 0.994 | 1.48 |  |
| faster_lio+planar | full | 541 | 247.4 | 0.997 | 1.39 |  |
| faster_lio+planar+poles | full | 541 | 247.1 | 0.997 | 1.41 |  |
| faster_lio+plane_icp | full | 541 | 407.5 | 0.921 | 7.78 | 43 |
| faster_lio+plane_icp+planar | full | 541 | 164.7 | 0.997 | 1.28 |  |
| faster_lio+plane_icp_da | full | 541 | 400.9 | 0.925 | 7.46 | 36 |
| faster_lio+plane_icp_da+planar | full | 541 | 169.9 | 0.997 | 1.26 |  |
| faster_lio+vgicp | full | 541 | 422.9 | 0.916 | 8.33 | 8 |
| faster_lio+vgicp+planar | full | 541 | 152.0 | 1.001 | 1.15 |  |
| faster_lio+vgicp+planar+poles | full | 541 | 152.0 | 1.001 | 1.17 |  |
| faster_lio+wheel | full | 541 | 376.1 | 0.955 | 5.64 |  |
| faster_lio+wheel+planar | full | 541 | 243.0 | 1.000 | 2.24 |  |
| genz_lio | concurrent_fast_lio+genz_lio | 541 | 447.5 | 0.911 | 8.78 |  |
| genz_lio | concurrent_faster_lio+genz_lio | 541 | 454.5 | 0.912 | 8.73 |  |
| genz_lio | full | 542 | 451.6 | 0.911 | 8.78 |  |
| genz_lio | full_ros | 541 | 447.5 | 0.911 | 8.78 |  |
| genz_lio | smoke30 | 30 | 0.3 | 1.009 | nan |  |
| genz_lio | win_fast_lio_genz_lio_adaptive_cold_wu5_q0.75_000 | 10 | 0.1 | 1.000 | 0.43 |  |
| genz_lio | win_fast_lio_genz_lio_adaptive_cold_wu5_q0.75_001 | 10 | 7.9 | 0.835 | 13.03 |  |
| genz_lio | win_fast_lio_genz_lio_adaptive_cold_wu5_q0.75_002 | 10 | 0.2 | 1.002 | 0.48 |  |
| genz_lio | win_fast_lio_genz_lio_adaptive_warm_wu5_q0.75_000 | 10 | 0.1 | 0.999 | 0.24 |  |
| genz_lio | win_fast_lio_genz_lio_adaptive_warm_wu5_q0.75_001 | 10 | 0.4 | 0.990 | 0.83 |  |
| genz_lio | win_fast_lio_genz_lio_adaptive_warm_wu5_q0.75_002 | 10 | 0.2 | 0.995 | 0.58 |  |
| genz_lio | win_fast_lio_genz_lio_periodic_warm_wu5_L20_P60_000 | 25 | 0.5 | 1.001 | 0.54 |  |
| genz_lio | win_fast_lio_genz_lio_periodic_warm_wu5_L20_P60_001 | 25 | 1.6 | 0.988 | 1.34 |  |
| genz_lio | win_faster_lio_genz_lio_adaptive_cold_wu5_q0.75_000 | 10 | 15.6 | 0.682 | 35.37 |  |
| genz_lio | win_faster_lio_genz_lio_adaptive_cold_wu5_q0.75_001 | 10 | 0.2 | 1.002 | 0.48 |  |
| genz_lio | win_faster_lio_genz_lio_adaptive_warm_wu5_q0.75_000 | 10 | 0.2 | 0.996 | 0.65 |  |
| genz_lio | win_faster_lio_genz_lio_adaptive_warm_wu5_q0.75_001 | 10 | 0.2 | 0.996 | 0.69 |  |
| genz_lio | win_faster_lio_genz_lio_periodic_warm_wu5_L20_P60_000 | 25 | 0.4 | 1.001 | 0.53 |  |
| genz_lio | win_faster_lio_genz_lio_periodic_warm_wu5_L20_P60_001 | 25 | 1.7 | 0.988 | 1.36 |  |
| genz_lio+planar | full | 542 | 158.8 | 0.999 | 1.21 |  |
| kiss_icp | full | 541 | 169.7 | 0.962 | 3.99 | 34 |
| kiss_icp | smoke30 | 29 | 0.3 | 1.008 | nan | 32 |
| kiss_icp+planar | full | 541 | 75.4 | 0.996 | 1.18 |  |

---

## Step 1 — verification of fastlio.csv / fasterlio.csv  (`results/verify/verify_summary.md`)

# Step 1 - verification of the legacy exports

Ground truth: 5420 samples, 542 s, 10295 m. Distance to the dual-GNSS midpoint: median 0.02 m, p95 0.04 m (clock offset -5 ms).

## A. Against ground truth (horizontal)

| trajectory | protocol | ATE RMSE m | ATE median m | final m | drift % (KITTI mean) | RPE 1 s median m | length ratio | clock dt ms |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| fastlio_csv_latlon | none | 1118.5 | 441.0 | 2853.8 | 8.22 | 4.03 | 0.921 | 510 |
| fastlio_csv_latlon | se2_head | 1684.7 | 742.2 | 4017.9 | 8.22 | 7.79 | 0.921 | 510 |
| fastlio_csv_latlon | se2_full | 482.1 | 296.8 | 1123.6 | 8.22 | 4.46 | 0.921 | 510 |
| fastlio_csv_latlon | se3_full | 366.2 | 261.2 | 794.6 | 3.91 | 3.80 | 0.968 | 510 |
| fastlio_csv_stored_en | none | 1114.1 | 439.9 | 2850.9 | 8.07 | 4.08 | 0.920 | 508 |
| fastlio_csv_stored_en | se2_head | 1653.5 | 713.7 | 3968.0 | 8.07 | 7.64 | 0.920 | 508 |
| fastlio_csv_stored_en | se2_full | 485.4 | 297.2 | 1133.0 | 8.07 | 4.54 | 0.920 | 508 |
| fastlio_csv_stored_en | se3_full | 369.3 | 263.0 | 804.6 | 3.80 | 3.98 | 0.968 | 508 |
| fasterlio_csv_latlon | none | 872.0 | 240.6 | 2244.9 | 5.66 | 2.93 | 0.946 | 595 |
| fasterlio_csv_latlon | se2_head | 1118.8 | 398.5 | 2751.5 | 5.66 | 4.61 | 0.946 | 595 |
| fasterlio_csv_latlon | se2_full | 373.2 | 262.9 | 846.9 | 5.66 | 3.78 | 0.946 | 595 |
| fasterlio_csv_latlon | se3_full | 308.9 | 242.1 | 655.5 | 3.16 | 3.47 | 0.975 | 595 |
| fasterlio_csv_stored_en | none | 1137.1 | 409.8 | 2796.4 | 5.55 | 5.06 | 0.945 | 593 |
| fasterlio_csv_stored_en | se2_head | 1086.4 | 369.2 | 2697.2 | 5.55 | 4.73 | 0.945 | 593 |
| fasterlio_csv_stored_en | se2_full | 376.1 | 262.9 | 855.8 | 5.55 | 3.93 | 0.945 | 593 |
| fasterlio_csv_stored_en | se3_full | 311.7 | 244.0 | 665.4 | 3.04 | 3.59 | 0.974 | 593 |
| dlio | se2_head | 1102.7 | 451.6 | 2699.1 | 7.77 | 4.49 | 0.923 | -78 |
| dlio | se2_full | 395.8 | 232.4 | 962.2 | 7.77 | 3.26 | 0.923 | -78 |
| dlio | se3_full | 261.4 | 199.3 | 602.9 | 3.24 | 2.59 | 0.976 | -78 |
| fast_lio | se2_head | 1485.6 | 621.5 | 3581.4 | 9.38 | 6.83 | 0.905 | 81 |
| fast_lio | se2_full | 500.4 | 279.9 | 1185.5 | 9.38 | 4.52 | 0.905 | 81 |
| fast_lio | se3_full | 336.9 | 242.6 | 748.2 | 3.85 | 3.46 | 0.969 | 81 |
| faster_lio | se2_head | 1245.5 | 483.3 | 3005.6 | 5.05 | 5.33 | 0.953 | 64 |
| faster_lio | se2_full | 381.7 | 263.8 | 825.3 | 5.05 | 3.72 | 0.953 | 64 |
| faster_lio | se3_full | 314.0 | 240.6 | 655.2 | 2.51 | 3.48 | 0.983 | 64 |
| genz_lio | se2_head | 1413.7 | 618.9 | 3379.2 | 8.86 | 6.27 | 0.911 | 75 |
| genz_lio | se2_full | 454.5 | 254.5 | 1099.4 | 8.86 | 3.89 | 0.911 | 75 |
| genz_lio | se3_full | 306.8 | 221.5 | 695.8 | 3.87 | 2.92 | 0.969 | 75 |
| fast_lio_full_pf4_legacy | se2_head | 1608.0 | 681.4 | 3851.2 | 9.28 | 7.65 | 0.908 | 84 |
| fast_lio_full_pf4_legacy | se2_full | 520.3 | 292.9 | 1213.9 | 9.28 | 4.65 | 0.908 | 84 |
| fast_lio_full_pf4_legacy | se3_full | 364.7 | 259.3 | 793.8 | 3.93 | 3.77 | 0.968 | 84 |

`none` = as stored in UTM 33N. `se2_head` = rigid fit on the first 200 m only, then pure prediction. `se2_full`/`se3_full` = rigid fit on the whole drive (standard benchmark ATE). Drift % is frame-free (each segment anchored at its own start).

## B. Legacy export vs our re-run of the same algorithm

- **fastlio_latlon**: residual after rigid planar fit median 50.5 m, max 174.6 m; per-step length difference median 0.040 m; length ratio legacy/rerun 1.017; clock offset -0.380 s.
- **fastlio_stored_en**: residual after rigid planar fit median 47.9 m, max 176.4 m; per-step length difference median 0.040 m; length ratio legacy/rerun 1.016; clock offset -0.376 s.
- **fastlio_vs_full_pf4_legacy_latlon**: residual after rigid planar fit median 47.4 m, max 159.8 m; per-step length difference median 0.036 m; length ratio legacy/rerun 1.014; clock offset -0.389 s.
- **fastlio_vs_full_pf4_legacy_stored_en**: residual after rigid planar fit median 44.6 m, max 159.0 m; per-step length difference median 0.036 m; length ratio legacy/rerun 1.014; clock offset -0.387 s.
- **fasterlio_latlon**: residual after rigid planar fit median 13.8 m, max 47.5 m; per-step length difference median 0.013 m; length ratio legacy/rerun 0.993; clock offset -0.511 s.
- **fasterlio_stored_en**: residual after rigid planar fit median 13.4 m, max 47.3 m; per-step length difference median 0.013 m; length ratio legacy/rerun 0.992; clock offset -0.509 s.

Small residual and length ratio ≈ 1: the CSV is a faithful record of what the algorithm does. Large residual: the CSV does not describe the algorithm's motion (export, clock or config problem).


---

## Step 3 — weakness analysis  (`results/analysis/analysis_summary.md`)

# Step 3 - weakness analysis

54 windows. Mounting yaw removed (deg): {"fast_lio": 1.11, "faster_lio": 1.13, "dlio": 1.07, "genz_lio": 1.08}

## Errors per method (10 s windows)

| method | scale err median | scale |p90| | lateral median | yaw drift median deg | step err median m |
|---|---:|---:|---:|---:|---:|
| fast_lio | -0.13% | 0.76% | -0.13% | 0.44 | 0.058 |
| faster_lio | -0.17% | 0.87% | -0.10% | 0.48 | 0.059 |
| dlio | 0.13% | 0.51% | -0.03% | 0.26 | 0.156 |
| genz_lio | -0.11% | 0.81% | -0.08% | 0.47 | 0.057 |

## What the errors go with (Spearman rho, top 6)

- **fast_lio__trans_err**: genz_lio__trans_err (+0.95), can_speed_mps (+0.40), genz_scale_indicator (+0.37), genz_median_range (+0.37), scan_imu_acc_norm_std (+0.37), gt_speed (+0.34)
- **fast_lio__scale_err**: scan_z_std (-0.46), genz_voxelized_points (-0.45), genz_point_matches (-0.44), genz_lio__scale_err (-0.44), genz_processing_time_ms (-0.35), scan_range_p50 (-0.35)
- **fast_lio__yaw_err_deg**: genz_lio__yaw_err_deg (+0.52), genz_lio__lat_err (+0.43), genz_plane_matches (+0.29), genz_cov_rot_trace (-0.26), scan_imu_yaw_rate (+0.25), genz_lio__trans_err (-0.24)
- **faster_lio__trans_err**: genz_lio__trans_err (+0.97), can_speed_mps (+0.38), scan_imu_acc_norm_std (+0.35), genz_scale_indicator (+0.34), genz_median_range (+0.34), gt_speed (+0.33)
- **faster_lio__scale_err**: genz_lio__scale_err (-0.59), scan_z_std (-0.52), genz_voxelized_points (-0.45), genz_processing_time_ms (-0.44), genz_median_range (-0.43), genz_scale_indicator (-0.42)
- **faster_lio__yaw_err_deg**: genz_lio__yaw_err_deg (+0.62), genz_lio__lat_err (+0.44), genz_point_matches (-0.30), genz_plane_matches (+0.29), scan_n_points (+0.27), scan_return_ratio (+0.27)
- **dlio__trans_err**: can_speed_mps (+0.70), gt_speed (+0.68), genz_scale_indicator (+0.65), genz_median_range (+0.64), scan_frac_far30 (+0.54), genz_cov_pos_trace (+0.54)
- **dlio__scale_err**: genz_point_matches (-0.38), genz_voxelized_points (-0.32), scan_planar_frac (+0.31), genz_lio__lat_err (-0.30), genz_processing_time_ms (-0.28), genz_leaf_size (-0.26)
- **dlio__yaw_err_deg**: can_steering_wheel_angle_rad (+0.38), genz_lio__yaw_err_deg (+0.35), scan_imu_yaw_rate (+0.32), genz_point_matches (-0.32), gt_turn_deg (+0.30), genz_lio__trans_err (-0.30)
- **genz_lio__trans_err**: genz_lio__trans_err (+1.00), scan_imu_acc_norm_std (+0.40), genz_scale_indicator (+0.39), genz_median_range (+0.39), can_speed_mps (+0.34), scan_range_p90 (+0.29)
- **genz_lio__scale_err**: genz_lio__scale_err (-0.49), genz_processing_time_ms (-0.45), scan_planar_frac (+0.37), genz_point_matches (-0.36), scan_imu_acc_norm_std (-0.31), genz_voxelized_points (-0.30)
- **genz_lio__yaw_err_deg**: genz_lio__yaw_err_deg (+0.69), scan_range_p90 (-0.37), genz_lio__lat_err (+0.36), genz_scale_indicator (-0.34), genz_median_range (-0.34), scan_n_points (+0.34)

## Median step error by regime (tertiles)

- **gt_speed** edges {'low': {'min': 0.417, 'max': 20.849}, 'mid': {'min': 20.867, 'max': 20.961}, 'high': {'min': 20.965, 'max': 21.08}}: {'fast_lio': {'low': 0.0528, 'mid': 0.0597, 'high': 0.0599}, 'faster_lio': {'low': 0.053, 'mid': 0.0578, 'high': 0.0594}, 'dlio': {'low': 0.1229, 'mid': 0.1599, 'high': 0.1608}, 'genz_lio': {'low': 0.0542, 'mid': 0.0574, 'high': 0.0586}}
- **gt_turn_deg** edges {'low': {'min': 0.034, 'max': 6.747}, 'mid': {'min': 7.006, 'max': 13.989}, 'high': {'min': 14.144, 'max': 101.685}}: {'fast_lio': {'low': 0.0599, 'mid': 0.0598, 'high': 0.0518}, 'faster_lio': {'low': 0.0587, 'mid': 0.0598, 'high': 0.0517}, 'dlio': {'low': 0.1467, 'mid': 0.1571, 'high': 0.1592}, 'genz_lio': {'low': 0.0577, 'mid': 0.0596, 'high': 0.0523}}
- **scan_trans_cond** edges {'low': {'min': 0.003, 'max': 0.022}, 'mid': {'min': 0.023, 'max': 0.041}, 'high': {'min': 0.041, 'max': 0.205}}: {'fast_lio': {'low': 0.0549, 'mid': 0.0597, 'high': 0.0619}, 'faster_lio': {'low': 0.0543, 'mid': 0.0592, 'high': 0.0614}, 'dlio': {'low': 0.167, 'mid': 0.1569, 'high': 0.138}, 'genz_lio': {'low': 0.0553, 'mid': 0.058, 'high': 0.0609}}
- **scan_return_ratio** edges {'low': {'min': 0.057, 'max': 0.31}, 'mid': {'min': 0.324, 'max': 0.468}, 'high': {'min': 0.5, 'max': 0.695}}: {'fast_lio': {'low': 0.0597, 'mid': 0.0587, 'high': 0.0528}, 'faster_lio': {'low': 0.0596, 'mid': 0.0579, 'high': 0.053}, 'dlio': {'low': 0.164, 'mid': 0.1544, 'high': 0.1315}, 'genz_lio': {'low': 0.0596, 'mid': 0.0571, 'high': 0.0542}}
- **scan_range_p50** edges {'low': {'min': 12.907, 'max': 15.732}, 'mid': {'min': 15.773, 'max': 22.552}, 'high': {'min': 22.941, 'max': 48.477}}: {'fast_lio': {'low': 0.0598, 'mid': 0.0537, 'high': 0.0597}, 'faster_lio': {'low': 0.0581, 'mid': 0.053, 'high': 0.0593}, 'dlio': {'low': 0.1453, 'mid': 0.1399, 'high': 0.1683}, 'genz_lio': {'low': 0.0569, 'mid': 0.0542, 'high': 0.0592}}
- **scan_planar_frac** edges {'low': {'min': 0.129, 'max': 0.357}, 'mid': {'min': 0.357, 'max': 0.526}, 'high': {'min': 0.539, 'max': 0.875}}: {'fast_lio': {'low': 0.0619, 'mid': 0.057, 'high': 0.0569}, 'faster_lio': {'low': 0.0614, 'mid': 0.0566, 'high': 0.0563}, 'dlio': {'low': 0.133, 'mid': 0.1574, 'high': 0.1592}, 'genz_lio': {'low': 0.0609, 'mid': 0.057, 'high': 0.056}}

## Best method share of windows

{'faster_lio': 0.453, 'genz_lio': 0.358, 'fast_lio': 0.189}



---

## Baseline + support running at the same time  (`results/concurrency/`)


**fast_lio+dlio**

| method | run | ATE m | drift % | latency median / p95 ms | scans w/o pose | CPU % | RAM MB |
|---|---|---:|---:|---|---:|---:|---:|
| fast_lio | solo | 500.4 | 9.38 | 49 / 67 | 6 | 117 | 615 |
| fast_lio | concurrent | 492.5 | 9.47 | 39 / 57 | 4 | 80 | 558 |
| dlio | solo | 395.8 | 7.77 | n/a |  | 234 | 3110 |
| dlio | concurrent | 410.8 | 7.69 | n/a |  | 191 | 3064 |

**fast_lio+genz_lio**

| method | run | ATE m | drift % | latency median / p95 ms | scans w/o pose | CPU % | RAM MB |
|---|---|---:|---:|---|---:|---:|---:|
| fast_lio | solo | 500.4 | 9.38 | 49 / 67 | 6 | 117 | 615 |
| fast_lio | concurrent | 495.9 | 9.24 | 37 / 54 | 7 | 83 | 607 |
| genz_lio | solo | 450.4 | 8.84 | 76 / 97 | 6 | 216 | 2556 |
| genz_lio | concurrent | 450.4 | 8.84 | 75 / 96 | 6 | 177 | 2556 |

**faster_lio+dlio**

| method | run | ATE m | drift % | latency median / p95 ms | scans w/o pose | CPU % | RAM MB |
|---|---|---:|---:|---|---:|---:|---:|
| faster_lio | solo | 381.7 | 5.05 | 31 / 47 | 8 | 124 | 437 |
| faster_lio | concurrent | 381.7 | 5.05 | 24 / 35 | 8 | 90 | 455 |
| dlio | solo | 395.8 | 7.77 | n/a |  | 234 | 3110 |
| dlio | concurrent | 401.4 | 7.74 | n/a |  | 183 | 3088 |

**faster_lio+genz_lio**

| method | run | ATE m | drift % | latency median / p95 ms | scans w/o pose | CPU % | RAM MB |
|---|---|---:|---:|---|---:|---:|---:|
| faster_lio | solo | 381.7 | 5.05 | 31 / 47 | 8 | 124 | 437 |
| faster_lio | concurrent | 382.1 | 5.07 | 25 / 38 | 7 | 90 | 452 |
| genz_lio | solo | 450.4 | 8.84 | 76 / 97 | 6 | 216 | 2556 |
| genz_lio | concurrent | 457.5 | 8.80 | 66 / 88 | 5 | 166 | 2554 |

---

## Step 4 — learned fusion


### `results/fusion/fast_lio/fusion_summary.md`

# Step 4 - learned fusion, baseline = fast_lio

Methods: ['fast_lio', 'dlio', 'genz_lio']. 72 causal features. Test rows [4436, 5419].

## Held-out (last block of the drive)

| method | step err RMSE m | yaw err median deg | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|---:|
| fast_lio | 0.0959 | 0.050 | 21.0 | 41.9 | 0.66 |
| dlio | 0.2496 | 0.056 | 16.6 | 37.4 | 0.68 |
| genz_lio | 0.0951 | 0.050 | 21.4 | 46.9 | 0.59 |
| uniform_mean | 0.1207 | 0.049 | 19.6 | 42.0 | 0.55 |
| oracle_best_per_step | 0.0811 | 0.049 | 23.8 | 51.6 | 0.59 |
| static_inv_var | 0.0965 | 0.049 | 19.4 | 41.0 | 0.59 |
| best_single | 0.0951 | 0.050 | 21.4 | 46.9 | 0.59 |
| chosen:mlp16 | 0.0957 | 0.050 | 17.2 | 37.3 | 0.44 |
| control:shuffled_context | 0.0977 | 0.050 | 28.3 | 60.3 | 0.57 |

**Model chosen by CV (one-SE rule): `mlp16`.**

## Overfitting checks

- Learned model beats best-single, static weights and its shuffled-context control on test.
- Step error RMSE train / val / test: {'train': 0.07197231404362507, 'val': 0.07231396670182344, 'test': 0.09574869610337186}
- Test drift 200 m % across seeds: [0.4358214288898312, 0.48103716474212815, 0.4645626597595345]
- Fraction of the test road within 25 m of the training route: 0.00

Mean test weights: {'w_trans_fast_lio': 0.23999999463558197, 'w_trans_dlio': 0.07400000095367432, 'w_trans_genz_lio': 0.6859999895095825, 'w_yaw_fast_lio': 0.2590000033378601, 'w_yaw_dlio': 0.32899999618530273, 'w_yaw_genz_lio': 0.41200000047683716}

Static inverse-variance weights (fwd, lat, yaw): {'fast_lio': [0.456, 0.387, 0.349], 'dlio': [0.077, 0.222, 0.322], 'genz_lio': [0.466, 0.391, 0.329]}

Oracle (best single method per step) share on test: {'fast_lio': 0.35772357723577236, 'dlio': 0.1991869918699187, 'genz_lio': 0.44308943089430897}

## Blocked 5-fold CV (mean over folds)

| method | step err RMSE m | integrated ATE RMSE m | drift 200 m % | ± SE |
|---|---:|---:|---:|---:|
| fast_lio | 0.0760 | 36.5 | 0.99 | 0.32 |
| dlio | 0.1947 | 36.0 | 1.00 | 0.38 |
| genz_lio | 0.0752 | 40.7 | 0.93 | 0.34 |
| uniform_mean | 0.0950 | 37.2 | 0.90 | 0.36 |
| oracle_best_per_step | 0.0627 | 39.9 | 0.89 | 0.30 |
| best_single | 0.0757 | 42.6 | 0.95 | 0.34 |
| static_inv_var | 0.0758 | 37.4 | 0.93 | 0.35 |
| linear_gating | 0.0808 | 36.6 | 0.97 | 0.41 |
| mlp16 | 0.0762 | 32.0 | 0.86 | 0.36 |
| mlp64 | 0.0766 | 32.7 | 0.93 | 0.38 |

Per-fold drift 200 m % (folds in time order):

- `fast_lio`: [2.27, 0.53, 0.69, 0.83, 0.62]
- `dlio`: [2.51, 0.45, 0.5, 0.86, 0.67]
- `genz_lio`: [2.29, 0.51, 0.62, 0.66, 0.58]
- `uniform_mean`: [2.32, 0.45, 0.48, 0.72, 0.54]
- `oracle_best_per_step`: [2.1, 0.54, 0.5, 0.72, 0.58]
- `best_single`: [2.29, 0.51, 0.69, 0.66, 0.58]
- `static_inv_var`: [2.33, 0.45, 0.58, 0.73, 0.57]
- `linear_gating`: [2.58, 0.46, 0.46, 0.78, 0.57]
- `mlp16`: [2.3, 0.37, 0.58, 0.67, 0.39]
- `mlp64`: [2.45, 0.43, 0.58, 0.67, 0.54]

Paired against the best by mean (`mlp16`), simplest first:

| model | mean diff (pp) | paired SE | folds better than best |
|---|---:|---:|---:|
| best_single | +0.08 | 0.04 | 2 |
| static_inv_var | +0.07 | 0.03 | 1 |
| linear_gating | +0.11 | 0.07 | 1 |
| mlp16 | +0.00 | 0.00 | 0 |
| mlp64 | +0.07 | 0.03 | 1 |

-> chosen `mlp16` (simplest within one paired SE of the best).

## Trigger model (baseline-only features -> baseline error next 1 s)

{
 "target": "next-1 s baseline step error per metre driven",
 "n_iter_chosen_on_val": 25,
 "val": {
  "spearman": 0.14597539790763367,
  "bad_threshold": 0.04494335669998658,
  "bad_rate": 0.13719512195121952,
  "roc_auc_bad": 0.5688304323168869,
  "speed_only_control": {
   "spearman": 0.05250453629299372,
   "roc_auc_bad": 0.5029577280460673
  }
 },
 "test": {
  "spearman": 0.37664088413972474,
  "bad_threshold": 0.04494335669998658,
  "bad_rate": 0.2299794661190965,
  "roc_auc_bad": 0.6531696428571429,
  "speed_only_control": {
   "spearman": 0.1612499260945171,
   "roc_auc_bad": 0.5498988095238095
  }
 }
}



### `results/fusion/fast_lio_correctors/fusion_summary.md`

# Step 4 - learned fusion, baseline = fast_lio

Methods: ['fast_lio', 'dlio', 'genz_lio', 'kiss_icp', 'fast_lio+wheel', 'fast_lio+plane_icp_da+planar']. 114 causal features. Test rows [4436, 5419].

## Held-out (last block of the drive)

| method | step err RMSE m | yaw err median deg | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|---:|
| fast_lio | 0.0959 | 0.050 | 21.0 | 41.9 | 0.66 |
| dlio | 0.2496 | 0.056 | 16.6 | 37.4 | 0.68 |
| genz_lio | 0.0951 | 0.050 | 21.4 | 46.9 | 0.59 |
| kiss_icp | 0.1056 | 0.055 | 9.1 | 22.4 | 0.93 |
| fast_lio+wheel | 0.0976 | 0.052 | 3.6 | 8.1 | 0.49 |
| fast_lio+plane_icp_da+planar | 0.1009 | 0.052 | 26.7 | 68.4 | 0.91 |
| uniform_mean | 0.1016 | 0.050 | 15.5 | 37.4 | 0.59 |
| oracle_best_per_step | 0.0756 | 0.047 | 24.8 | 58.1 | 0.53 |
| static_inv_var | 0.0952 | 0.050 | 15.5 | 36.8 | 0.61 |
| best_single | 0.0951 | 0.050 | 21.4 | 46.9 | 0.59 |
| chosen:linear_gating | 0.0946 | 0.052 | 11.4 | 27.8 | 0.41 |
| control:shuffled_context | 0.0952 | 0.051 | 15.5 | 37.2 | 0.45 |

**Model chosen by CV (one-SE rule): `linear_gating`.**

## Overfitting checks

- Learned model beats best-single, static weights and its shuffled-context control on test.
- Step error RMSE train / val / test: {'train': 0.07078228323017945, 'val': 0.07133967877385268, 'test': 0.09458955521926403}
- Test drift 200 m % across seeds: [0.4128608022276578, 0.4892151431088252, 0.41596527642277875]
- Fraction of the test road within 25 m of the training route: 0.00

Mean test weights: {'w_trans_fast_lio': 0.15399999916553497, 'w_trans_dlio': 0.04800000041723251, 'w_trans_genz_lio': 0.04899999871850014, 'w_trans_kiss_icp': 0.12300000339746475, 'w_trans_fast_lio+wheel': 0.3400000035762787, 'w_trans_fast_lio+plane_icp_da+planar': 0.28600001335144043, 'w_yaw_fast_lio': 0.30300000309944153, 'w_yaw_dlio': 0.12600000202655792, 'w_yaw_genz_lio': 0.09399999678134918, 'w_yaw_kiss_icp': 0.125, 'w_yaw_fast_lio+wheel': 0.22100000083446503, 'w_yaw_fast_lio+plane_icp_da+planar': 0.13099999725818634}

Static inverse-variance weights (fwd, lat, yaw): {'fast_lio': [0.199, 0.213, 0.177], 'dlio': [0.034, 0.122, 0.163], 'genz_lio': [0.204, 0.216, 0.167], 'kiss_icp': [0.166, 0.051, 0.164], 'fast_lio+wheel': [0.199, 0.208, 0.162], 'fast_lio+plane_icp_da+planar': [0.198, 0.189, 0.166]}

Oracle (best single method per step) share on test: {'fast_lio': 0.11077235772357724, 'dlio': 0.15040650406504066, 'genz_lio': 0.16260162601626016, 'kiss_icp': 0.20020325203252032, 'fast_lio+wheel': 0.18902439024390244, 'fast_lio+plane_icp_da+planar': 0.18699186991869918}

## Blocked 5-fold CV (mean over folds)

| method | step err RMSE m | integrated ATE RMSE m | drift 200 m % | ± SE |
|---|---:|---:|---:|---:|
| fast_lio | 0.0760 | 36.5 | 0.99 | 0.32 |
| dlio | 0.1947 | 36.0 | 1.00 | 0.38 |
| genz_lio | 0.0752 | 40.7 | 0.93 | 0.34 |
| kiss_icp | 0.0845 | 19.7 | 1.09 | 0.35 |
| fast_lio+wheel | 0.0766 | 43.7 | 1.87 | 1.05 |
| fast_lio+plane_icp_da+planar | 0.0772 | 38.6 | 0.97 | 0.32 |
| uniform_mean | 0.0801 | 34.2 | 1.01 | 0.45 |
| oracle_best_per_step | 0.0576 | 29.6 | 1.03 | 0.50 |
| best_single | 0.0757 | 42.6 | 0.95 | 0.34 |
| static_inv_var | 0.0752 | 34.4 | 1.03 | 0.45 |
| linear_gating | 0.0768 | 26.1 | 0.92 | 0.41 |
| mlp16 | 0.0755 | 34.8 | 0.90 | 0.34 |
| mlp64 | 0.0769 | 34.9 | 0.87 | 0.34 |

Per-fold drift 200 m % (folds in time order):

- `fast_lio`: [2.27, 0.53, 0.69, 0.83, 0.62]
- `dlio`: [2.51, 0.45, 0.5, 0.86, 0.67]
- `genz_lio`: [2.29, 0.51, 0.62, 0.66, 0.58]
- `kiss_icp`: [2.23, 0.38, 1.53, 0.47, 0.86]
- `fast_lio+wheel`: [6.05, 0.97, 0.75, 1.05, 0.53]
- `fast_lio+plane_icp_da+planar`: [2.23, 0.42, 0.75, 0.6, 0.84]
- `uniform_mean`: [2.81, 0.41, 0.63, 0.62, 0.56]
- `oracle_best_per_step`: [3.04, 0.46, 0.49, 0.64, 0.53]
- `best_single`: [2.29, 0.51, 0.69, 0.66, 0.58]
- `static_inv_var`: [2.83, 0.42, 0.68, 0.63, 0.58]
- `linear_gating`: [2.54, 0.42, 0.71, 0.53, 0.42]
- `mlp16`: [2.23, 0.47, 0.69, 0.7, 0.38]
- `mlp64`: [2.23, 0.42, 0.63, 0.67, 0.41]

Paired against the best by mean (`mlp64`), simplest first:

| model | mean diff (pp) | paired SE | folds better than best |
|---|---:|---:|---:|
| best_single | +0.07 | 0.03 | 1 |
| static_inv_var | +0.16 | 0.12 | 2 |
| linear_gating | +0.05 | 0.07 | 2 |
| mlp16 | +0.03 | 0.02 | 1 |
| mlp64 | +0.00 | 0.00 | 0 |

-> chosen `linear_gating` (simplest within one paired SE of the best).

## Trigger model (baseline-only features -> baseline error next 1 s)

{
 "target": "next-1 s baseline step error per metre driven",
 "n_iter_chosen_on_val": 25,
 "val": {
  "spearman": 0.14597539790763367,
  "bad_threshold": 0.04494335669998658,
  "bad_rate": 0.13719512195121952,
  "roc_auc_bad": 0.5688304323168869,
  "speed_only_control": {
   "spearman": 0.05250453629299372,
   "roc_auc_bad": 0.5029577280460673
  }
 },
 "test": {
  "spearman": 0.37664088413972474,
  "bad_threshold": 0.04494335669998658,
  "bad_rate": 0.2299794661190965,
  "roc_auc_bad": 0.6531696428571429,
  "speed_only_control": {
   "spearman": 0.1612499260945171,
   "roc_auc_bad": 0.5498988095238095
  }
 }
}



### `results/fusion/faster_lio/fusion_summary.md`

# Step 4 - learned fusion, baseline = faster_lio

Methods: ['faster_lio', 'dlio', 'genz_lio']. 72 causal features. Test rows [4436, 5419].

## Held-out (last block of the drive)

| method | step err RMSE m | yaw err median deg | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|---:|
| faster_lio | 0.1002 | 0.048 | 31.9 | 66.2 | 0.94 |
| dlio | 0.2496 | 0.056 | 16.6 | 37.4 | 0.68 |
| genz_lio | 0.0951 | 0.050 | 21.4 | 46.9 | 0.59 |
| uniform_mean | 0.1212 | 0.049 | 23.1 | 50.1 | 0.65 |
| oracle_best_per_step | 0.0815 | 0.049 | 23.2 | 49.4 | 0.59 |
| static_inv_var | 0.0978 | 0.048 | 23.2 | 49.4 | 0.73 |
| best_single | 0.1002 | 0.048 | 31.9 | 66.2 | 0.94 |
| chosen:mlp16 | 0.0978 | 0.048 | 20.1 | 43.5 | 0.44 |
| control:shuffled_context | 0.0974 | 0.049 | 34.0 | 72.1 | 0.70 |

**Model chosen by CV (one-SE rule): `mlp16`.**

## Overfitting checks

- Learned model beats best-single, static weights and its shuffled-context control on test.
- Step error RMSE train / val / test: {'train': 0.0722306008870393, 'val': 0.07478964298026532, 'test': 0.09782451997277411}
- Test drift 200 m % across seeds: [0.4431449717913091, 0.5444379436862742, 0.5231577148399641]
- Fraction of the test road within 25 m of the training route: 0.00

Mean test weights: {'w_trans_faster_lio': 0.5120000243186951, 'w_trans_dlio': 0.10899999737739563, 'w_trans_genz_lio': 0.3779999911785126, 'w_yaw_faster_lio': 0.37400001287460327, 'w_yaw_dlio': 0.2630000114440918, 'w_yaw_genz_lio': 0.3630000054836273}

Static inverse-variance weights (fwd, lat, yaw): {'faster_lio': [0.457, 0.409, 0.35], 'dlio': [0.077, 0.214, 0.321], 'genz_lio': [0.466, 0.377, 0.328]}

Oracle (best single method per step) share on test: {'faster_lio': 0.33028455284552843, 'dlio': 0.20223577235772358, 'genz_lio': 0.46747967479674796}

## Blocked 5-fold CV (mean over folds)

| method | step err RMSE m | integrated ATE RMSE m | drift 200 m % | ± SE |
|---|---:|---:|---:|---:|
| faster_lio | 0.0767 | 49.0 | 1.08 | 0.31 |
| dlio | 0.1947 | 36.0 | 1.00 | 0.38 |
| genz_lio | 0.0752 | 40.7 | 0.93 | 0.34 |
| uniform_mean | 0.0950 | 41.3 | 0.94 | 0.36 |
| oracle_best_per_step | 0.0627 | 42.2 | 0.91 | 0.32 |
| best_single | 0.0767 | 50.2 | 1.08 | 0.31 |
| static_inv_var | 0.0760 | 41.7 | 0.98 | 0.35 |
| linear_gating | 0.0772 | 42.6 | 1.00 | 0.38 |
| mlp16 | 0.0764 | 38.1 | 0.93 | 0.36 |
| mlp64 | 0.0772 | 27.8 | 0.91 | 0.39 |

Per-fold drift 200 m % (folds in time order):

- `faster_lio`: [2.3, 0.48, 0.87, 0.87, 0.88]
- `dlio`: [2.51, 0.45, 0.5, 0.86, 0.67]
- `genz_lio`: [2.29, 0.51, 0.62, 0.66, 0.58]
- `uniform_mean`: [2.35, 0.43, 0.53, 0.74, 0.63]
- `oracle_best_per_step`: [2.17, 0.48, 0.54, 0.75, 0.59]
- `best_single`: [2.29, 0.48, 0.87, 0.87, 0.88]
- `static_inv_var`: [2.36, 0.43, 0.64, 0.74, 0.7]
- `linear_gating`: [2.53, 0.52, 0.58, 0.82, 0.57]
- `mlp16`: [2.37, 0.46, 0.62, 0.74, 0.46]
- `mlp64`: [2.44, 0.34, 0.63, 0.65, 0.48]

Paired against the best by mean (`mlp64`), simplest first:

| model | mean diff (pp) | paired SE | folds better than best |
|---|---:|---:|---:|
| best_single | +0.17 | 0.09 | 1 |
| static_inv_var | +0.07 | 0.05 | 1 |
| linear_gating | +0.09 | 0.04 | 1 |
| mlp16 | +0.02 | 0.03 | 3 |
| mlp64 | +0.00 | 0.00 | 0 |

-> chosen `mlp16` (simplest within one paired SE of the best).

## Trigger model (baseline-only features -> baseline error next 1 s)

{
 "target": "next-1 s baseline step error per metre driven",
 "n_iter_chosen_on_val": 25,
 "val": {
  "spearman": 0.18088571587306476,
  "bad_threshold": 0.044348226527537955,
  "bad_rate": 0.14735772357723578,
  "roc_auc_bad": 0.603633225103777,
  "speed_only_control": {
   "spearman": 0.1191897927716252,
   "roc_auc_bad": 0.531823599523242
  }
 },
 "test": {
  "spearman": 0.28741337533080097,
  "bad_threshold": 0.044348226527537955,
  "bad_rate": 0.2782340862422998,
  "roc_auc_bad": 0.648811367203288,
  "speed_only_control": {
   "spearman": 0.2876817887931425,
   "roc_auc_bad": 0.6329725530541224
  }
 }
}



### `results/fusion/faster_lio_correctors/fusion_summary.md`

# Step 4 - learned fusion, baseline = faster_lio

Methods: ['faster_lio', 'dlio', 'genz_lio', 'kiss_icp', 'faster_lio+wheel', 'faster_lio+plane_icp_da+planar']. 114 causal features. Test rows [4436, 5419].

## Held-out (last block of the drive)

| method | step err RMSE m | yaw err median deg | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|---:|
| faster_lio | 0.1002 | 0.048 | 31.9 | 66.2 | 0.94 |
| dlio | 0.2496 | 0.056 | 16.6 | 37.4 | 0.68 |
| genz_lio | 0.0951 | 0.050 | 21.4 | 46.9 | 0.59 |
| kiss_icp | 0.1056 | 0.055 | 9.1 | 22.4 | 0.93 |
| faster_lio+wheel | 0.0974 | 0.052 | 14.0 | 35.9 | 0.58 |
| faster_lio+plane_icp_da+planar | 0.1048 | 0.051 | 31.1 | 78.4 | 1.17 |
| uniform_mean | 0.1023 | 0.050 | 20.0 | 47.8 | 0.70 |
| oracle_best_per_step | 0.0757 | 0.046 | 14.9 | 38.8 | 0.52 |
| static_inv_var | 0.0963 | 0.050 | 20.1 | 47.4 | 0.74 |
| best_single | 0.1002 | 0.048 | 31.9 | 66.2 | 0.94 |
| chosen:linear_gating | 0.0957 | 0.052 | 27.7 | 63.2 | 0.47 |
| control:shuffled_context | 0.0988 | 0.050 | 24.4 | 57.7 | 0.50 |

**Model chosen by CV (one-SE rule): `linear_gating`.**

## Overfitting checks

- Learned model beats best-single, static weights and its shuffled-context control on test.
- Step error RMSE train / val / test: {'train': 0.07130751251465245, 'val': 0.07204366534939911, 'test': 0.0956854244808242}
- Test drift 200 m % across seeds: [0.46767146957367695, 0.4525102138400873, 0.44501247343633415]
- Fraction of the test road within 25 m of the training route: 0.00

Mean test weights: {'w_trans_faster_lio': 0.22699999809265137, 'w_trans_dlio': 0.07599999755620956, 'w_trans_genz_lio': 0.06499999761581421, 'w_trans_kiss_icp': 0.15299999713897705, 'w_trans_faster_lio+wheel': 0.3089999854564667, 'w_trans_faster_lio+plane_icp_da+planar': 0.16899999976158142, 'w_yaw_faster_lio': 0.15700000524520874, 'w_yaw_dlio': 0.14399999380111694, 'w_yaw_genz_lio': 0.09600000083446503, 'w_yaw_kiss_icp': 0.18400000035762787, 'w_yaw_faster_lio+wheel': 0.2680000066757202, 'w_yaw_faster_lio+plane_icp_da+planar': 0.1509999930858612}

Static inverse-variance weights (fwd, lat, yaw): {'faster_lio': [0.2, 0.225, 0.178], 'dlio': [0.034, 0.118, 0.163], 'genz_lio': [0.203, 0.208, 0.167], 'kiss_icp': [0.165, 0.049, 0.164], 'faster_lio+wheel': [0.198, 0.21, 0.161], 'faster_lio+plane_icp_da+planar': [0.2, 0.189, 0.166]}

Oracle (best single method per step) share on test: {'faster_lio': 0.10060975609756098, 'dlio': 0.14939024390243902, 'genz_lio': 0.17378048780487804, 'kiss_icp': 0.20223577235772358, 'faster_lio+wheel': 0.2225609756097561, 'faster_lio+plane_icp_da+planar': 0.15142276422764228}

## Blocked 5-fold CV (mean over folds)

| method | step err RMSE m | integrated ATE RMSE m | drift 200 m % | ± SE |
|---|---:|---:|---:|---:|
| faster_lio | 0.0767 | 49.0 | 1.08 | 0.31 |
| dlio | 0.1947 | 36.0 | 1.00 | 0.38 |
| genz_lio | 0.0752 | 40.7 | 0.93 | 0.34 |
| kiss_icp | 0.0845 | 19.7 | 1.09 | 0.35 |
| faster_lio+wheel | 0.0765 | 56.3 | 1.95 | 1.04 |
| faster_lio+plane_icp_da+planar | 0.0776 | 41.1 | 1.03 | 0.33 |
| uniform_mean | 0.0802 | 38.7 | 1.05 | 0.45 |
| oracle_best_per_step | 0.0577 | 31.7 | 1.02 | 0.47 |
| best_single | 0.0767 | 50.2 | 1.08 | 0.31 |
| static_inv_var | 0.0753 | 38.9 | 1.07 | 0.45 |
| linear_gating | 0.0761 | 32.8 | 0.93 | 0.43 |
| mlp16 | 0.0762 | 37.3 | 1.02 | 0.41 |
| mlp64 | 0.0763 | 31.8 | 0.88 | 0.37 |

Per-fold drift 200 m % (folds in time order):

- `faster_lio`: [2.3, 0.48, 0.87, 0.87, 0.88]
- `dlio`: [2.51, 0.45, 0.5, 0.86, 0.67]
- `genz_lio`: [2.29, 0.51, 0.62, 0.66, 0.58]
- `kiss_icp`: [2.23, 0.38, 1.53, 0.47, 0.86]
- `faster_lio+wheel`: [6.11, 0.98, 0.93, 1.14, 0.62]
- `faster_lio+plane_icp_da+planar`: [2.25, 0.41, 0.78, 0.62, 1.08]
- `uniform_mean`: [2.83, 0.4, 0.69, 0.65, 0.67]
- `oracle_best_per_step`: [2.91, 0.44, 0.59, 0.67, 0.52]
- `best_single`: [2.29, 0.48, 0.87, 0.87, 0.88]
- `static_inv_var`: [2.85, 0.41, 0.73, 0.66, 0.71]
- `linear_gating`: [2.62, 0.35, 0.56, 0.65, 0.47]
- `mlp16`: [2.64, 0.44, 0.75, 0.72, 0.55]
- `mlp64`: [2.35, 0.42, 0.59, 0.58, 0.46]

Paired against the best by mean (`mlp64`), simplest first:

| model | mean diff (pp) | paired SE | folds better than best |
|---|---:|---:|---:|
| best_single | +0.20 | 0.09 | 1 |
| static_inv_var | +0.19 | 0.09 | 1 |
| linear_gating | +0.05 | 0.06 | 2 |
| mlp16 | +0.14 | 0.05 | 0 |
| mlp64 | +0.00 | 0.00 | 0 |

-> chosen `linear_gating` (simplest within one paired SE of the best).

## Trigger model (baseline-only features -> baseline error next 1 s)

{
 "target": "next-1 s baseline step error per metre driven",
 "n_iter_chosen_on_val": 25,
 "val": {
  "spearman": 0.18088571587306476,
  "bad_threshold": 0.044348226527537955,
  "bad_rate": 0.14735772357723578,
  "roc_auc_bad": 0.603633225103777,
  "speed_only_control": {
   "spearman": 0.1191897927716252,
   "roc_auc_bad": 0.531823599523242
  }
 },
 "test": {
  "spearman": 0.28741337533080097,
  "bad_threshold": 0.044348226527537955,
  "bad_rate": 0.2782340862422998,
  "roc_auc_bad": 0.648811367203288,
  "speed_only_control": {
   "spearman": 0.2876817887931425,
   "roc_auc_bad": 0.6329725530541224
  }
 }
}



---

## Phase 2 — correction strategies


### `results/correct/fast_lio/correct_summary.md`

# Correction strategies on top of fast_lio

Per block: anchored on the block's own first 200 m, then pure prediction. Drift % = KITTI-style mean over 100-800 m segments (frame-free). Fitted quantities use the training block only; the strategy is selected on validation.

| method | ms/scan | train drift % | val drift % | test drift % | test RPE 1 s m | test length ratio | whole-drive ATE m (se2_head) |
|---|---:|---:|---:|---:|---:|---:|---:|
| fast_lio | — | 4.23 | 9.30 | 25.59 | 6.498 | 0.751 | 1485.6 |
| fast_lio+gicp | 36 | 3.52 | 8.65 | 26.21 | 6.573 | 0.748 | 1066.5 |
| fast_lio+gicp+planar | — | 1.12 | 1.04 | 0.67 | 0.301 | 1.001 | 446.5 |
| fast_lio+gicp_da | 34 | 3.55 | 8.72 | 26.51 | 6.704 | 0.745 | 1071.9 |
| fast_lio+gicp_da+planar | — | 1.13 | 1.05 | 0.64 | 0.294 | 1.001 | 449.4 |
| fast_lio+gicp_da+planar+poles | — | 1.16 | 1.05 | 0.64 | 0.294 | 1.001 | 421.3 |
| fast_lio+gicp_da_wheel | 34 | 3.54 | 8.73 | 26.49 | 6.652 | 0.745 | 1088.4 |
| fast_lio+gicp_da_wheel+planar | — | 1.13 | 1.05 | 0.66 | 0.290 | 1.002 | 464.7 |
| fast_lio+gicp_da_wheel+planar+poles | — | 1.18 | 1.05 | 0.66 | 0.290 | 1.002 | 434.2 |
| fast_lio+icp | 52 | 3.90 | 7.61 | 22.32 | 6.068 | 0.780 | 906.8 |
| fast_lio+icp+planar | — | 1.47 | 1.12 | 2.19 | 2.191 | 0.993 | 346.8 |
| fast_lio+planar | — | 1.29 | 1.63 | 0.85 | 0.681 | 0.997 | 781.7 |
| fast_lio+planar+poles | — | 1.31 | 1.63 | 0.85 | 0.681 | 0.997 | 762.7 |
| fast_lio+plane_icp | 44 | 3.81 | 7.54 | 21.85 | 5.587 | 0.786 | 1208.4 |
| fast_lio+plane_icp+planar | — | 1.32 | 1.45 | 1.54 | 0.778 | 0.995 | 626.4 |
| fast_lio+plane_icp_da | 36 | 3.78 | 7.47 | 21.57 | 5.476 | 0.788 | 1223.3 |
| fast_lio+plane_icp_da+planar | — | 1.31 | 0.83 | 1.27 | 0.829 | 0.995 | 645.7 |
| fast_lio+vgicp | 8 | 3.58 | 8.05 | 24.04 | 6.200 | 0.768 | 1263.1 |
| fast_lio+vgicp+planar | — | 1.24 | 1.64 | 0.63 | 0.284 | 1.001 | 671.1 |
| fast_lio+vgicp+planar+poles | — | 1.27 | 1.64 | 0.63 | 0.284 | 1.001 | 665.6 |
| fast_lio+wheel | — | 5.27 | 9.55 | 25.39 | 6.453 | 0.753 | 1536.4 |
| fast_lio+wheel+planar | — | 2.66 | 1.61 | 0.87 | 0.936 | 0.999 | 845.6 |
| dlio | — | 3.24 | 7.15 | 22.42 | 5.616 | 0.786 | 1102.7 |
| genz_lio | — | 3.74 | 8.69 | 25.22 | 6.296 | 0.756 | 1413.7 |
| kiss_icp | 34 | 2.30 | 3.13 | 10.61 | 2.657 | 0.899 | 563.0 |
| dlio+planar | — | 1.45 | 1.62 | 0.91 | 0.644 | 1.001 | 571.7 |
| genz_lio+planar | — | 1.38 | 1.02 | 0.88 | 0.307 | 0.998 | 806.4 |
| kiss_icp+planar | — | 1.45 | 0.58 | 1.65 | 1.033 | 0.995 | 372.1 |

## Absolute (UTM, no re-alignment) for pole-corrected runs

| method | whole drive ATE m | train ATE m | val ATE m | test ATE m |
|---|---:|---:|---:|---:|
| fast_lio+gicp_da+planar+poles | 421.3 | 107.6 | 393.6 | 861.5 |
| fast_lio+gicp_da_wheel+planar+poles | 434.2 | 113.4 | 412.7 | 883.0 |
| fast_lio+planar+poles | 762.7 | 218.7 | 806.6 | 1499.8 |
| fast_lio+vgicp+planar+poles | 665.6 | 206.1 | 688.8 | 1302.1 |

**Selected on validation:** `fast_lio+plane_icp_da+planar`. Test drift: 1.27 % vs baseline 25.59 %. (Best on test, for reference only, not a selection: `fast_lio+vgicp+planar`.)


### `results/correct/faster_lio/correct_summary.md`

# Correction strategies on top of faster_lio

Per block: anchored on the block's own first 200 m, then pure prediction. Drift % = KITTI-style mean over 100-800 m segments (frame-free). Fitted quantities use the training block only; the strategy is selected on validation.

| method | ms/scan | train drift % | val drift % | test drift % | test RPE 1 s m | test length ratio | whole-drive ATE m (se2_head) |
|---|---:|---:|---:|---:|---:|---:|---:|
| faster_lio | — | 2.67 | 4.37 | 13.41 | 3.801 | 0.871 | 1245.5 |
| faster_lio+gicp | 36 | 3.56 | 8.73 | 26.31 | 6.627 | 0.747 | 1095.5 |
| faster_lio+gicp+planar | — | 1.15 | 1.07 | 0.65 | 0.296 | 1.001 | 472.0 |
| faster_lio+gicp_da | 34 | 3.57 | 8.70 | 26.12 | 6.588 | 0.749 | 1093.4 |
| faster_lio+gicp_da+planar | — | 1.12 | 1.08 | 0.68 | 0.333 | 1.001 | 469.6 |
| faster_lio+gicp_da+planar+poles | — | 1.16 | 1.08 | 0.68 | 0.333 | 1.001 | 474.3 |
| faster_lio+gicp_da_wheel | 34 | 3.51 | 8.50 | 25.73 | 6.524 | 0.752 | 1121.4 |
| faster_lio+gicp_da_wheel+planar | — | 1.13 | 1.09 | 0.72 | 0.317 | 1.002 | 508.8 |
| faster_lio+gicp_da_wheel+planar+poles | — | 1.20 | 1.09 | 0.72 | 0.317 | 1.002 | 495.8 |
| faster_lio+icp | 50 | 3.78 | 7.04 | 20.58 | 5.694 | 0.797 | 949.5 |
| faster_lio+icp+planar | — | 1.48 | 1.07 | 2.38 | 2.127 | 0.991 | 426.3 |
| faster_lio+planar | — | 1.44 | 1.34 | 1.20 | 0.507 | 0.994 | 948.3 |
| faster_lio+planar+poles | — | 1.50 | 1.34 | 1.20 | 0.507 | 0.994 | 951.1 |
| faster_lio+plane_icp | 43 | 3.81 | 7.31 | 21.29 | 5.410 | 0.790 | 1222.5 |
| faster_lio+plane_icp+planar | — | 1.35 | 1.08 | 2.07 | 1.683 | 0.992 | 648.8 |
| faster_lio+plane_icp_da | 36 | 3.72 | 7.00 | 20.32 | 5.184 | 0.800 | 1231.1 |
| faster_lio+plane_icp_da+planar | — | 1.34 | 0.83 | 1.94 | 1.628 | 0.993 | 678.6 |
| faster_lio+vgicp | 8 | 3.58 | 8.01 | 23.98 | 6.081 | 0.769 | 1242.3 |
| faster_lio+vgicp+planar | — | 1.22 | 1.07 | 0.65 | 0.299 | 1.001 | 655.4 |
| faster_lio+vgicp+planar+poles | — | 1.26 | 1.07 | 0.65 | 0.299 | 1.001 | 647.9 |
| faster_lio+wheel | — | 3.69 | 4.61 | 12.89 | 3.614 | 0.876 | 1301.2 |
| faster_lio+wheel+planar | — | 2.74 | 1.61 | 1.06 | 1.167 | 0.999 | 1009.8 |
| dlio | — | 3.24 | 7.15 | 22.42 | 5.616 | 0.786 | 1102.7 |
| genz_lio | — | 3.74 | 8.69 | 25.22 | 6.296 | 0.756 | 1413.7 |
| kiss_icp | 34 | 2.30 | 3.13 | 10.61 | 2.657 | 0.899 | 563.0 |
| dlio+planar | — | 1.45 | 1.62 | 0.91 | 0.644 | 1.001 | 571.7 |
| genz_lio+planar | — | 1.38 | 1.02 | 0.88 | 0.307 | 0.998 | 806.4 |
| kiss_icp+planar | — | 1.45 | 0.58 | 1.65 | 1.033 | 0.995 | 372.1 |

## Absolute (UTM, no re-alignment) for pole-corrected runs

| method | whole drive ATE m | train ATE m | val ATE m | test ATE m |
|---|---:|---:|---:|---:|
| faster_lio+gicp_da+planar+poles | 474.3 | 122.4 | 451.9 | 964.5 |
| faster_lio+gicp_da_wheel+planar+poles | 495.8 | 127.0 | 476.1 | 1006.3 |
| faster_lio+planar+poles | 951.2 | 246.8 | 988.4 | 1884.8 |
| faster_lio+vgicp+planar+poles | 647.9 | 188.2 | 675.2 | 1277.2 |

**Selected on validation:** `faster_lio+plane_icp_da+planar`. Test drift: 1.94 % vs baseline 13.41 %. (Best on test, for reference only, not a selection: `faster_lio+vgicp+planar`.)


---

## Step 5 — support in sliding windows


### `results/windows/fast_lio/dlio_adaptive_cold_wu5_q0.75/windows_summary.md`

# Step 5 - dlio in adaptive windows (baseline fast_lio)

3 windows, duty cycle 16.3%, warm-up 5.0 s, cold start, support compute 0 s.

Median step error inside windows: windowed 0.10111166414197492, continuous 0.14867682784155245, baseline 0.05736675443262232 (m).

| variant | step err RMSE m | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|
| baseline_only | 0.0959 | 21.0 | 41.9 | 0.66 |
| fused_continuous_support | 0.0969 | 12.7 | 27.1 | 0.46 |
| fused_windowed_support | 0.0955 | 18.4 | 36.9 | 0.55 |


### `results/windows/fast_lio/genz_lio_adaptive_cold_wu5_q0.75/windows_summary.md`

# Step 5 - genz_lio in adaptive windows (baseline fast_lio)

3 windows, duty cycle 16.5%, warm-up 5.0 s, cold start, support compute 19 s.

Median step error inside windows: windowed 0.0636712023010045, continuous 0.05427453414027127, baseline 0.05736675443262232 (m).

| variant | step err RMSE m | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|
| baseline_only | 0.0959 | 21.0 | 41.9 | 0.66 |
| fused_continuous_support | 0.0950 | 18.2 | 38.1 | 0.56 |
| fused_windowed_support | 0.0971 | 24.7 | 50.5 | 0.50 |


### `results/windows/fast_lio/genz_lio_adaptive_warm_wu5_q0.75/windows_summary.md`

# Step 5 - genz_lio in adaptive windows (baseline fast_lio)

3 windows, duty cycle 16.5%, warm-up 5.0 s, warm start, support compute 24 s.

Median step error inside windows: windowed 0.05404867334915525, continuous 0.05427453414027127, baseline 0.05736675443262232 (m).

| variant | step err RMSE m | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|
| baseline_only | 0.0959 | 21.0 | 41.9 | 0.66 |
| fused_continuous_support | 0.0950 | 18.2 | 38.1 | 0.56 |
| fused_windowed_support | 0.0947 | 23.4 | 48.1 | 0.51 |


### `results/windows/fast_lio/genz_lio_adaptive_warm_wu5_q0.9/windows_summary.md`

# Step 5 - genz_lio in adaptive windows (baseline fast_lio)

0 windows, duty cycle 0.0%, warm-up 5.0 s, warm start, support compute 0 s.

Median step error inside windows: windowed None, continuous None, baseline None (m).

| variant | step err RMSE m | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|
| baseline_only | 0.0959 | 21.0 | 41.9 | 0.66 |
| fused_continuous_support | 0.0950 | 18.2 | 38.1 | 0.56 |
| fused_windowed_support | 0.0959 | 21.0 | 41.9 | 0.66 |


### `results/windows/fast_lio/genz_lio_periodic_warm_wu5_L20_P60/windows_summary.md`

# Step 5 - genz_lio in periodic windows (baseline fast_lio)

2 windows, duty cycle 40.7%, warm-up 5.0 s, warm start, support compute 43 s.

Median step error inside windows: windowed 0.04923617729005216, continuous 0.04916770874535986, baseline 0.051263591038948016 (m).

| variant | step err RMSE m | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|
| baseline_only | 0.0959 | 21.0 | 41.9 | 0.66 |
| fused_continuous_support | 0.0950 | 18.2 | 38.1 | 0.56 |
| fused_windowed_support | 0.0959 | 19.3 | 40.7 | 0.64 |


### `results/windows/faster_lio/dlio_adaptive_cold_wu5_q0.75/windows_summary.md`

# Step 5 - dlio in adaptive windows (baseline faster_lio)

2 windows, duty cycle 10.0%, warm-up 5.0 s, cold start, support compute 0 s.

Median step error inside windows: windowed 0.08735919137627249, continuous 0.10596928299049868, baseline 0.07139746271926722 (m).

| variant | step err RMSE m | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|
| baseline_only | 0.1002 | 31.9 | 66.2 | 0.94 |
| fused_continuous_support | 0.0988 | 16.5 | 33.3 | 0.63 |
| fused_windowed_support | 0.0998 | 31.2 | 64.3 | 0.89 |


### `results/windows/faster_lio/genz_lio_adaptive_cold_wu5_q0.75/windows_summary.md`

# Step 5 - genz_lio in adaptive windows (baseline faster_lio)

2 windows, duty cycle 10.4%, warm-up 5.0 s, cold start, support compute 11 s.

Median step error inside windows: windowed 0.07723184769338612, continuous 0.0561078458660937, baseline 0.0709374886825921 (m).

| variant | step err RMSE m | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|
| baseline_only | 0.1002 | 31.9 | 66.2 | 0.94 |
| fused_continuous_support | 0.0955 | 25.8 | 55.7 | 0.63 |
| fused_windowed_support | 0.1104 | 34.7 | 70.5 | 1.12 |


### `results/windows/faster_lio/genz_lio_adaptive_warm_wu5_q0.75/windows_summary.md`

# Step 5 - genz_lio in adaptive windows (baseline faster_lio)

2 windows, duty cycle 10.4%, warm-up 5.0 s, warm start, support compute 12 s.

Median step error inside windows: windowed 0.05482111007160011, continuous 0.0561078458660937, baseline 0.0709374886825921 (m).

| variant | step err RMSE m | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|
| baseline_only | 0.1002 | 31.9 | 66.2 | 0.94 |
| fused_continuous_support | 0.0955 | 25.8 | 55.7 | 0.63 |
| fused_windowed_support | 0.0988 | 32.3 | 68.2 | 0.84 |


### `results/windows/faster_lio/genz_lio_adaptive_warm_wu5_q0.9/windows_summary.md`

# Step 5 - genz_lio in adaptive windows (baseline faster_lio)

0 windows, duty cycle 0.0%, warm-up 5.0 s, warm start, support compute 0 s.

Median step error inside windows: windowed None, continuous None, baseline None (m).

| variant | step err RMSE m | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|
| baseline_only | 0.1002 | 31.9 | 66.2 | 0.94 |
| fused_continuous_support | 0.0955 | 25.8 | 55.7 | 0.63 |
| fused_windowed_support | 0.1002 | 31.9 | 66.2 | 0.94 |


### `results/windows/faster_lio/genz_lio_periodic_warm_wu5_L20_P60/windows_summary.md`

# Step 5 - genz_lio in periodic windows (baseline faster_lio)

2 windows, duty cycle 40.7%, warm-up 5.0 s, warm start, support compute 35 s.

Median step error inside windows: windowed 0.0504514674755531, continuous 0.04916770874535986, baseline 0.048877371567955255 (m).

| variant | step err RMSE m | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|
| baseline_only | 0.1002 | 31.9 | 66.2 | 0.94 |
| fused_continuous_support | 0.0955 | 25.8 | 55.7 | 0.63 |
| fused_windowed_support | 0.1002 | 29.7 | 63.4 | 0.93 |

