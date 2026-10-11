# Step 5 - genz_lio in adaptive windows (baseline faster_lio)

0 windows, duty cycle 0.0%, warm-up 5.0 s, warm start, support compute 0 s.

Median step error inside windows: windowed None, continuous None, baseline None (m).

| variant | step err RMSE m | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|
| baseline_only | 0.1002 | 31.9 | 66.2 | 0.94 |
| fused_continuous_support | 0.0955 | 25.8 | 55.7 | 0.63 |
| fused_windowed_support | 0.1002 | 31.9 | 66.2 | 0.94 |
