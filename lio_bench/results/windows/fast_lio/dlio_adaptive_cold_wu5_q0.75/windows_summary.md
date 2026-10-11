# Step 5 - dlio in adaptive windows (baseline fast_lio)

3 windows, duty cycle 16.3%, warm-up 5.0 s, cold start, support compute 0 s.

Median step error inside windows: windowed 0.10111166414197492, continuous 0.14867682784155245, baseline 0.05736675443262232 (m).

| variant | step err RMSE m | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|
| baseline_only | 0.0959 | 21.0 | 41.9 | 0.66 |
| fused_continuous_support | 0.0969 | 12.7 | 27.1 | 0.46 |
| fused_windowed_support | 0.0955 | 18.4 | 36.9 | 0.55 |
