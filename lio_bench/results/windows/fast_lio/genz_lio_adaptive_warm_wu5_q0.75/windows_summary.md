# Step 5 - genz_lio in adaptive windows (baseline fast_lio)

3 windows, duty cycle 16.5%, warm-up 5.0 s, warm start, support compute 24 s.

Median step error inside windows: windowed 0.05404867334915525, continuous 0.05427453414027127, baseline 0.05736675443262232 (m).

| variant | step err RMSE m | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|
| baseline_only | 0.0959 | 21.0 | 41.9 | 0.66 |
| fused_continuous_support | 0.0950 | 18.2 | 38.1 | 0.56 |
| fused_windowed_support | 0.0947 | 23.4 | 48.1 | 0.51 |
