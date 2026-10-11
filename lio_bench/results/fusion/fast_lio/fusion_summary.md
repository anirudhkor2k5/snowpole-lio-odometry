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

