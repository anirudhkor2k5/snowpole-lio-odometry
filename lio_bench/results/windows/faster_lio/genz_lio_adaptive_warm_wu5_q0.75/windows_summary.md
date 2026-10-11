# Step 5 - genz_lio in adaptive windows (baseline faster_lio)

2 windows, duty cycle 10.4%, warm-up 5.0 s, warm start, support compute 12 s.

Median step error inside windows: windowed 0.05482111007160011, continuous 0.0561078458660937, baseline 0.0709374886825921 (m).

| variant | step err RMSE m | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|
| baseline_only | 0.1002 | 31.9 | 66.2 | 0.94 |
| fused_continuous_support | 0.0955 | 25.8 | 55.7 | 0.63 |
| fused_windowed_support | 0.0988 | 32.3 | 68.2 | 0.84 |
