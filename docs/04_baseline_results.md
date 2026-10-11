# Baseline results

## 1. Research question

How large is the GNSS-referenced planar error of the exported Faster-LIO, FAST-LIO, FastReg, and pipeline trajectories, once the coordinate columns and the clocks are made explicit?

## 2. Hypothesis

If the exports are already in UTM 33N and were initialised on the GNSS position, a constant translation will not remove the error. Relative displacement error will then show whether the remaining error is slow drift or a disagreement of the motion itself.

This hypothesis is about the exports. It is not a claim that Faster-LIO, as an algorithm, has a particular drift rate. The two position columns disagree, so the algorithm’s own pose is not uniquely identified.

## 3. Dataset

One drive, 28 February 2024, E39 Hemnekjølen recordings already in the checkout. Reference: dual-GNSS midpoint, 5422 samples, 541.75 s, path length 10,298 m. Odometry files as listed in `docs/01_data_audit.md`. No second journey. DLIO and GenZ-LIO files are absent.

## 4. Coordinate frames and synchronization

`docs/02_coordinate_frames.md`. Primary absolute metric is identity in EPSG:32633 after the declared clock mapping. Relative clocks use the first left-GNSS stamp as the origin. A 0.345 s sensitivity shift was run and does not change the conclusion.

## 5. Algorithm configuration

`odometry_refinement/scripts/evaluate_baselines.py`

- Association gap: 0.15 s
- Initial chord: 30 m, opening segment only
- Time horizons: 0.5, 1, 2, 10 s, tolerance 0.15 s
- Distance horizons: 50, 100, 500 m along the reference
- Segments: 500 m
- Failure thresholds: 5, 10, 20, 50, 100 m of absolute horizontal error
- No scale, no time-varying warp, no use of the rotation column as yaw

## 6. Split

None. This is a descriptive evaluation of the whole drive. The reliability split is in `docs/05_reliability_experiment.md`.

## 7. Baselines

The methods are the exported trajectories, not re-runs of the estimators. Faster-LIO was not executed and its source was not modified.

## 8. Results

Primary clock only. Identity alignment. Horizontal metres against the dual-GNSS midpoint. `fastreg` lat/lon duplicates `fastlio` lat/lon and is not an independent result.

| Export | Column | ATE median | ATE RMSE | Error at end | 1 s RPE median | 1 s RPE RMSE | 10 s RPE median | 100 m RPE median | Linear growth |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| fasterlio | lat/lon → UTM | 234 m | 868 m | 2233 m | 2.38 m | 5.82 m | 23.5 m | 11.1 m | 187 m/km |
| fasterlio | stored EN | 411 m | 1135 m | 2784 m | 4.39 m | 7.00 m | 43.8 m | 21.0 m | 246 m/km |
| fastlio | lat/lon → UTM | 438 m | 1116 m | 2850 m | 3.85 m | 7.24 m | 37.8 m | 18.4 m | 225 m/km |
| fastlio | stored EN | 437 m | 1112 m | 2848 m | 3.84 m | 7.26 m | 37.6 m | 18.4 m | 225 m/km |
| fastreg | stored EN | 554 m | 933 m | 2327 m | 4.15 m | 6.43 m | 40.6 m | 23.1 m | 165 m/km |
| incremental (pipeline) | stored EN | 50.2 m | 77.6 m | 231 m | 4.04 m | 7.05 m | 12.0 m | 8.61 m | 13.3 m/km |

Start offsets are 0.08 m to 0.25 m. Translating the origin onto the reference does not change these figures. The initial-chord yaw is −4.9° for Faster-LIO lat/lon, +0.2° for Faster-LIO stored EN, about −10° for FAST-LIO, −14.7° for FastReg stored EN, and −1.0° for the pipeline file. Applying that yaw increases absolute error.

The whole-trajectory SE(2) diagnostic, which is not a reported accuracy, still leaves 369 m RMSE on Faster-LIO lat/lon and 54.5 m RMSE on the pipeline file.

Shifting the relative clock by 0.345 s changes the FAST-LIO lat/lon ATE median from 438 m to 433 m.

Fraction of synchronised frames whose absolute horizontal error exceeds 50 m: 0.84 for Faster-LIO lat/lon, 0.77 for Faster-LIO stored EN, about 0.89 for FAST-LIO, 0.90 for FastReg stored EN, and 0.50 for the pipeline file. The corresponding fractions above 100 m are 0.72, 0.71, about 0.86, 0.88, and 0.16.

## 9. Figures

Written by the evaluation script:

- `experiments/baseline/trajectories_unaligned.png`
- `experiments/baseline/error_vs_time.png`
- `experiments/baseline/error_vs_distance.png`
- `experiments/baseline/segment_rmse.png`
- `experiments/baseline/rpe_1s.png`
- `experiments/baseline/error_histograms.png`
- `experiments/baseline/reference_quality.png`

The map plot is the clearest statement. The GNSS track continues east. The LIO exports bend north and stop short. The pipeline incremental track stays on the GNSS track with a growing but much smaller offset. FAST-LIO lat/lon and FastReg lat/lon lie on top of each other, which is the copied-column result.

## 10. Error analysis

The LIO absolute error is small for roughly the first minute and then grows to kilometres. That shape is drift of the exported path, not a missed initial datum: the first point already matches GNSS.

Short-horizon motion is less wrong than the final position, and it is still large for lidar-inertial odometry. A median of 2.38 m over 1 s, on a vehicle moving at about 19 m/s, is a 2.38 m disagreement in displacement, not a 2.38 m absolute position. The pipeline file is worse at 1 s (median 4.04 m) and much better at 10 s (12.0 m versus 23.5–43.8 m) and in absolute error (median 50 m versus 234–554 m).

The two Faster-LIO columns do not support one drift number. Lat/lon is the less wrong of the two against GNSS and is still a 2.2 km end error.

## 11. Failure cases

Absolute error above 10 m occupies one long episode for most of the drive on every LIO column (frame fraction about 0.89 to 0.96). The pipeline file crosses 10 m for 79% of frames but returns below 100 m more often (23 separate episodes above 100 m, frame fraction 0.16).

Vertical failure of the LIO exports is separate: the altitude channel is not the road height.

## 12. Runtime and memory

The baseline script took 4.1 s and peaked at 122 MiB RSS on this workstation (`platform` recorded in `experiments/baseline/baseline_summary.json`). That is the cost of reading CSVs and computing metrics. It is not Faster-LIO runtime and it is not an edge-device measurement.

## 13. Interpretation

The exported LIO trajectories are not interchangeable with the pipeline’s incremental file. Substituting Faster-LIO’s stored easting/northing, or its lat/lon, into the snow-pole pipeline would feed that pipeline a path that is kilometres away from the GNSS track the pipeline currently uses as its reference.

The relative-motion numbers are the ones a reliability label can be built from. They are GNSS-referenced, they depend on which column is called the pose, and they grow along the drive.

## 14. Limitations

GNSS is not survey ground truth. One drive. LIO orientation is unusable. LIO altitude is unusable. The Faster-LIO file contradicts itself. Bag IMU and scans were not decoded, so this baseline says nothing about registration corrections.

## 15. Recommended next experiment

Do not train a correction policy on these labels yet. The reliability check in `docs/05_reliability_experiment.md` is that next experiment, and it does not clear the gate. Before any geometric corrector, the Faster-LIO pose column has to be re-exported from the estimator, with orientation, and checked against this same identity-frame protocol.

Full numbers: `experiments/baseline/baseline_summary.json`.
