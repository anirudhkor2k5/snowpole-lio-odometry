"""Phase 2: strategies that correct a baseline's motion where it is weak.

Every strategy writes an ordinary run (results/runs/<baseline>+<strategy>/<tag>/),
so verify / analyze / fusion / the correction report treat it like any estimator.

  wheel     - baseline increments with the forward component taken from CAN wheel
              speed (scale, lag and forward axis fitted on the training block only)
  scanpass  - one pass over the bag: deskew each scan with the baseline, then
              register it to a local map with several ICP variants seeded by the
              baseline (point-to-point ICP, point-to-plane ICP, GICP, VGICP), plus
              degeneracy-aware variants that fuse the ICP update with the prior
              (baseline or wheel) using ICP's own information matrix; KISS-ICP as an
              independent LiDAR-only odometry; pole candidates for `poles`
  poles     - causal filter that corrects a trajectory with matches between
              detected poles and the surveyed pole map
  report    - every strategy against its baseline, per train / val / test block

Nothing here is fitted on the validation or test blocks. Noise levels that are
not fitted come from physics (stated where they are defined), not from tuning.
"""
