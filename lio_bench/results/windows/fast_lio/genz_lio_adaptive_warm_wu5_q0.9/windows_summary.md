# Step 5 - genz_lio in adaptive windows (baseline fast_lio)

0 windows, duty cycle 0.0%, warm-up 5.0 s, warm start, support compute 0 s.

Median step error inside windows: windowed None, continuous None, baseline None (m).

| variant | step err RMSE m | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|
| baseline_only | 0.0959 | 21.0 | 41.9 | 0.66 |
| fused_continuous_support | 0.0950 | 18.2 | 38.1 | 0.56 |
| fused_windowed_support | 0.0959 | 21.0 | 41.9 | 0.66 |
