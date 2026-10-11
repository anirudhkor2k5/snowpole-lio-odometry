# Step 5 - genz_lio in periodic windows (baseline faster_lio)

2 windows, duty cycle 40.7%, warm-up 5.0 s, warm start, support compute 35 s.

Median step error inside windows: windowed 0.0504514674755531, continuous 0.04916770874535986, baseline 0.048877371567955255 (m).

| variant | step err RMSE m | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|
| baseline_only | 0.1002 | 31.9 | 66.2 | 0.94 |
| fused_continuous_support | 0.0955 | 25.8 | 55.7 | 0.63 |
| fused_windowed_support | 0.1002 | 29.7 | 63.4 | 0.93 |
