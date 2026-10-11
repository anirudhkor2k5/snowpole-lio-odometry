# Step 5 - genz_lio in periodic windows (baseline fast_lio)

2 windows, duty cycle 40.7%, warm-up 5.0 s, warm start, support compute 43 s.

Median step error inside windows: windowed 0.04923617729005216, continuous 0.04916770874535986, baseline 0.051263591038948016 (m).

| variant | step err RMSE m | integrated ATE RMSE m | final m | drift 200 m % |
|---|---:|---:|---:|---:|
| baseline_only | 0.0959 | 21.0 | 41.9 | 0.66 |
| fused_continuous_support | 0.0950 | 18.2 | 38.1 | 0.56 |
| fused_windowed_support | 0.0959 | 19.3 | 40.7 | 0.64 |
