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

