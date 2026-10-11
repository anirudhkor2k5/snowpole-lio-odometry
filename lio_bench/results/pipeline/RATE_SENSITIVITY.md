# Drift-rate sensitivity of the after-last-pole result

*`lio_bench/pipeline/rate_sensitivity.py`. Stage 3 = heading reset + drift rate. The learned rate is re-fitted at every accepted update; here it is shifted by a fixed offset, or fitted on only the last W seconds of updates. Cells: after-last-pole mean / final error [m] (≈5.1 km, no poles).*

## Rate offset (full-history fit)

| input | -0.010°/s | -0.005°/s | -0.002°/s | +0.000°/s | +0.002°/s | +0.005°/s | +0.010°/s |
|---|---:|---:|---:|---:|---:|---:|---:|
| fastlio_raw | 186 / 696 | 175 / 666 | 169 / 649 | 165 / 638 | 162 / 628 | 157 / 613 | 150 / 589 |
| fasterlio_raw | 180 / 551 | 164 / 509 | 154 / 485 | 148 / 469 | 142 / 453 | 132 / 429 | 117 / 392 |
| fastlio_gk | 95 / 234 | 77 / 184 | 67 / 155 | 60 / 135 | 53 / 115 | 42 / 86 | 25 / 38 |
| fasterlio_gk | 92 / 229 | 74 / 179 | 64 / 150 | 57 / 130 | 50 / 111 | 40 / 82 | 23 / 36 |
| final_online | 39 / 99 | 21 / 49 | 12 / 21 | 7 / 7 | 8 / 21 | 17 / 50 | 34 / 99 |

## Rate-fit window (no offset)

| input | all updates | last 600 s | last 300 s |
|---|---:|---:|---:|
| fastlio_raw | 165 / 638 | 165 / 638 | 165 / 638 |
| fasterlio_raw | 148 / 469 | 148 / 469 | 148 / 469 |
| fastlio_gk | 60 / 135 | 60 / 135 | 60 / 135 |
| fasterlio_gk | 57 / 130 | 57 / 130 | 57 / 130 |
| final_online | 7 / 7 | 7 / 7 | 7 / 7 |

**Window test is void:** accepted heading updates span only 213 s (the drive is 542 s; 244 s pass after the last update), so a 300 s / 600 s window keeps every update and the columns are identical.

## Uncertainty of the learned rate (OLS slope of the heading offset vs time, ± SE)

| input | all updates (used) | 2nd half of updates only |
|---|---:|---:|
| fastlio_raw | +0.0959 ± 0.0021 °/s | +0.0964 ± 0.0060 °/s |
| fasterlio_raw | +0.0704 ± 0.0014 °/s | +0.0801 ± 0.0042 °/s |
| fastlio_gk | +0.0122 ± 0.0017 °/s | +0.0021 ± 0.0024 °/s |
| fasterlio_gk | +0.0122 ± 0.0017 °/s | +0.0030 ± 0.0024 °/s |
| final_online | +0.0155 ± 0.0021 °/s | −0.0070 ± 0.0053 °/s |

## Reading

- The final online node's 7 m final after-pole error sits at the bottom of a V. The learned rate is the best possible value to within ~0.002 °/s, which is about 1 SE (the SE is itself optimistic, because the offsets are autocorrelated). At ±1 SE the error is ~21 m; at ±2.5 SE it is ~50 m. Fitting only the second half of the updates (an equally defensible choice) moves the rate by −0.022 °/s and the error to >100 m. **The 7 m is partly luck; quote it as a plausible range of ~7–50 m, not as 7 m.**
- On the same reading, the offline Faster-LIO + G + KISS result (130 m) is unlucky. Its best offset is beyond +0.01 °/s (36 m).
- For raw FAST-LIO2 and raw Faster-LIO the after-pole error barely responds to the rate (±0.01 °/s → ±8%). Their residual is not a heading-rate problem. It is most likely the along-track scale error that comes with their pitch drift.
- The pole-span results (first → last pole) do not depend on the rate: the reset+rate and reset rows are identical there.
