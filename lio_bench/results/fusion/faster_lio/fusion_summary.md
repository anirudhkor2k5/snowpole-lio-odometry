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

