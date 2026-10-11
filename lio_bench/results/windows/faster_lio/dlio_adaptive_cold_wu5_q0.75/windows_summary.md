# Step 5 - dlio in adaptive windows (baseline faster_lio)

2 windows, duty cycle 10.0%, warm-up 5.0 s, cold start, support compute 0 s.

Median step error inside windows: windowed 0.08735919137627249, continuous 0.10596928299049868, baseline 0.07139746271926722 (m).

| variant | step err RMSE m | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|
| baseline_only | 0.1002 | 31.9 | 66.2 | 0.94 |
| fused_continuous_support | 0.0988 | 16.5 | 33.3 | 0.63 |
| fused_windowed_support | 0.0998 | 31.2 | 64.3 | 0.89 |
