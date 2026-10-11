# Reliability experiment

## 1. Research question

Can causal motion features from the exported trajectory rank, on a later part of this same drive, the frames whose 1 s GNSS-referenced displacement error exceeds 2 m?

## 2. Hypothesis

If odometry failures on this road are visible in speed, turn rate, constant-velocity residual, or the exported vertical rate before the GNSS error is known, a small classifier fit on the first half of the drive will rank the later half better than a fixed robust-z rule, with a calibration error that is small enough to act on.

## 3. Dataset and selection

The same single drive as the baseline. Features and labels were written for both Faster-LIO columns, both FAST-LIO columns, FastReg stored easting/northing, and the pipeline incremental file. FastReg lat/lon was skipped because it copies FAST-LIO.

Labels use the dual-GNSS midpoint. Samples whose reference weight is below 0.5 are unlabeled. That weight is a flag, not a variance: fix status other than 2, antenna separation more than 5 cm from the median, or reported east variance above 400 m². Under 1% of GNSS samples fall below 0.5.

IMU, LiDAR geometry, reflectivity, signal, and near-IR features are not in this experiment. Those measurements were not decoded.

## 4. Coordinate frames and synchronization

Same identity-frame synchronization as `docs/02_coordinate_frames.md`. Labels are relative displacement error, so a constant datum offset would not create a positive label. The column choice still changes the label, because the columns disagree in their motion.

## 5. Algorithm configuration

Primary label, declared before looking at test scores: horizon 1.0 s, threshold 2.0 m.

Split: first 50% of the clock is train, next 20% is validation, the rest is test, with a 20 s gap after train and after validation. Normalization mean and standard deviation are fit on train only. Logistic regression is scikit-learn `LogisticRegression` (L2, `C = 1`, balanced classes). XGBoost is depth 2, 80 trees, early stopping on the validation log loss. The MLP is one hidden layer of 16 ReLU units with L2 and early stopping on validation Brier score. A lag stack of 1, 2, 5, and 10 frames is the temporal comparison. A TCN or GRU was not trained.

The analytical rule converts acceleration, course rate, curvature, and constant-velocity residual to robust z-scores using the training block only, and ranks by the maximum z. A second version also flags exported vertical rate above 5 m/s. The 5 m/s gate is a physical absurdity for a road vehicle, applied to the exported z channel, which is not a verified vertical velocity.

Feature groups are in `odometry_refinement/src/odom_refinement/features/odometry.py`. They are rates, steps, and short-window residuals. Absolute easting, northing, latitude, longitude, and sample index are rejected by a unit test.

## 6. Training, validation, and test

One journey, so this is an exploratory within-journey study. It is not a route or weather generalization.

At the primary 2 m / 1 s label the later blocks are almost entirely positive:

| Case | Train positive rate | Validation | Test |
| --- | ---: | ---: | ---: |
| fasterlio lat/lon | 0.102 | 1.00 | 1.00 |
| fasterlio stored EN | 0.419 | 1.00 | 1.00 |
| fastlio, either column | ~0.47 | 1.00 | 1.00 |
| fastreg stored EN | 0.458 | 1.00 | 1.00 |
| incremental | 0.534 | 0.907 | 0.955 |

For the LIO exports, validation and test contain no negative frames. ROC-AUC is undefined. Precision-recall AUC is 1 because every frame is a positive, which is also what a constant “always fail” predictor scores. That AUC is not evidence of ranking.

The same pattern holds at 1 m and at 5 m for Faster-LIO lat/lon: the training block’s 1 s error has median 1.20 m and 90th percentile 2.02 m, while the test block’s median is 10.3 m. No fixed threshold puts both classes in the test block, because the error has already grown and stays large.

## 7. Baseline methods

The analytical motion rule, the same rule plus the vertical-rate flag, logistic regression, XGBoost, the small MLP, and logistic regression on the lag stack. Ablations drop one feature group at a time and are scored on validation only.

## 8. Results

On every LIO export the learned models do not get a usable discrimination score. Test recall at probability 0.5 is the fraction of an all-positive block that the model happens to mark positive. For Faster-LIO lat/lon that recall is 0.028 for logistic regression and 0.00 for XGBoost. The models fit on the early, mostly negative block do not fire on the later block, where every frame exceeds 2 m. The vertical-rate flag does fire on 82% of that later block, because the exported z channel is running away. With no negative frames there, that recall has no false-alarm rate.

The pipeline incremental file is the only export with negatives in the holdout, and they are scarce (test positive rate 0.955).

| Model | Validation PR-AUC | Validation ROC-AUC | Test PR-AUC | Test ROC-AUC | Test false alarms per minute |
| --- | ---: | ---: | ---: | ---: | ---: |
| Positive-rate baseline | 0.907 | — | 0.955 | — | — |
| Analytical motion | 0.900 | 0.455 | 0.952 | 0.461 | 20.3 |
| Logistic regression | 0.911 | 0.516 | 0.951 | 0.463 | 25.8 |
| XGBoost | 0.911 | 0.500 | 0.957 | 0.506 | 26.3 |
| MLP | 0.910 | 0.509 | 0.948 | 0.444 | 26.3 |
| Lagged logistic | 0.902 | 0.501 | 0.961 | 0.539 | 26.3 |

PR-AUC sits on the positive rate. ROC-AUC sits on 0.5. Nothing beats the analytical rule by a margin that survives the base rate. Expected calibration error on the incremental test block is 0.09 for logistic regression and 0.31 for the analytical score. Neither is a covariance. False alarms at a 0.5 threshold are about 20 to 26 per minute on the test block because the models mark almost every frame positive.

Validation ablations move PR-AUC between about 0.90 and 0.92. That is the base rate, not a feature effect. Dropping the consistency group raises validation PR-AUC slightly; that was not used to change the model.

The decision rule in the training script asks whether a learned model beats the analytical rule by more than 0.02 validation PR-AUC. It does not, on any export. The lag stack does not beat plain logistic regression on validation either.

## 9. Figures

Class balance and error growth are the error-versus-time figure in `experiments/baseline/error_vs_time.png`. A separate reliability plot would repeat that curve. The numerical summary is `experiments/reliability/reliability_summary.json`. Feature matrices and label files are separated under `experiments/features/`.

## 10. Error analysis

The quantity being predicted changes its magnitude along the drive. Motion features available at the start (ordinary speed and turn rate on a highway) are not the same event as a multi-kilometre accumulated drift later. A classifier cannot be shown to predict the later regime from the earlier one when the later regime has only one class.

The exported vertical rate is the one feature that becomes large later. It is a description of a broken altitude channel, not a snow or degeneracy measurement. Using it to “detect” an all-positive test block does not show that geometric failure is predictable.

## 11. Failure cases

The experiment fails in a specific way: the holdout for Faster-LIO is saturated. Lowering the threshold makes the training block more positive. Raising it to 5 m makes the Faster-LIO training block entirely negative (0 positives at 1 s) while the test block stays entirely positive. There is no threshold of this family that yields a real test of ranking for Faster-LIO on a chronological split.

## 12. Runtime and memory

Feature extraction and training together took a few seconds. The training script peaked at 157 MiB RSS. XGBoost 3.4.1, scikit-learn 1.9.1, numpy 2.5.3. This is not an edge-device budget. The models are small enough that inference cost is not the blocking issue. Validity is.

## 13. Interpretation

Motion-only features do not provide a reliability signal that is usable for scheduling corrections on this recording. The analytical rule is retained only as a transparent negative control. It is not deployed. No neural model is adopted. A TCN, GRU, or reinforcement-learning scheduler is not justified: the one-step labels are already too non-stationary, and the holdout does not show that a temporal model ranks them.

## 14. Limitations

One drive. GNSS reference. Two contradictory Faster-LIO position columns. No IMU or LiDAR features. The 2 m threshold was predeclared and then found to saturate the holdout; reporting that saturation is the result, not a search for a threshold that looks good on the test block.

## 15. Recommended next experiment

Re-export Faster-LIO poses, including orientation and a real height or an explicit local-z flag, and show that one position column matches the estimator. Then repeat the identity-frame baseline. Only if a chronological or multi-journey holdout still contains both reliable and unreliable motion, after IMU and scan features are computed from the bag, is a reliability model worth fitting again.

Geometric registration, an action oracle, and a scheduler were not built. The oracle would need scans and a pose column that is not self-contradictory, and the reliability gate that would justify a learned scheduler was not passed.

## Go / no-go

**No-go for a learned reliability model, and no-go for proceeding to a correction oracle on the current exports.**

The GNSS track is aligned with the pole map and is good enough to expose kilometre-scale export error. It is not good enough to support a supervised accuracy claim. The Faster-LIO file does not contain one trajectory. On the trajectory that is internally consistent, the pipeline incremental file, causal motion features do not rank short-horizon GNSS disagreement above chance.
