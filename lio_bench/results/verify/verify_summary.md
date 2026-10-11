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
