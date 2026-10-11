# Is the Faster-LIO baseline the real thing? How to check it yourself

## What was run

| item | value |
|---|---|
| source | <https://github.com/gaoxiang12/faster-lio>, commit `8038fbac95283708fe2aa30658b7e116a7ace6d8`. This was upstream `main` when checked (`git ls-remote origin main`). |
| local changes | `third_party_patches/faster-lio.diff`, which contains only: (1) build fixes (glog define, livox msg dependency, conda lib path, no `-g`), and (2) `std::execution::par_unseq` / `unseq` → `seq` in 5 loops. Sequential execution gives the same maths, just slower and deterministic. Upstream merged a note on exactly this non-determinism in the same commit. |
| config | `configs/algorithms/faster_lio.yaml`. This is the Ouster OS2-128 + internal IMU, with the same noise values as FAST-LIO2's config. |
| the one tuning choice | `point_filter_num: 3` instead of upstream's 4. With 4, Faster-LIO tracks only 24 % of the distance in the first 30 s (`results/runs/faster_lio/smoke30` ran with 4, `smoke30_pf3` with 3). The OS2 has 1024 columns; keeping every 4th point keeps the same columns in every ring, and these stripes pin the scan. FAST-LIO2 uses the same value of 3. |
| launch | `ros/launch/faster_lio.launch` via `scripts/run_ros.sh --algos faster_lio`: full bag, real time (rate 1.0), headless, poses recorded by `ros/scripts/odom_recorder.py`. |
| output | `results/runs/faster_lio/full/`. Its `meta.json` has the config path, commit and timing; `node.log` echoes every parameter the node loaded. |

## The check (no liobench code involved)

```bash
python lio_bench/verify/verify_faster_lio.py
```

This uses only numpy and pandas. Each trajectory gets a 2D rotation + translation fitted on its
first 200 m against `groundtruth_extracted.csv`, plus a ±1 s clock-offset search. Nothing is
fitted after that. Output from 2026-10-11 (`verify_faster_lio_output.txt`):

| trajectory | length ratio | error at km 2 / 5 / 10 | final |
|---|---:|---|---:|
| A. old `fasterlio.csv` (not made by us) | 0.945 | 60 / 458 / 2588 m | 2696 m |
| **B. our Faster-LIO run** | 0.962 | 102 / 628 / 2983 m | **3113 m** |
| C. our FAST-LIO2 run | 0.921 | 113 / 813 / 3558 m | 3717 m |
| D. old `fastlio.csv` (not made by us) | 0.920 | 104 / 859 / 3794 m | 3964 m |

A vs B directly: median 36 m apart over 10.3 km, length ratio 0.982.

**Reading:**
- Our Faster-LIO is *not* better than the independent old export. It is slightly worse (3.1 km vs
  2.7 km final error).
- The ranking Faster-LIO < FAST-LIO2 shows up in the old exports too (2.7 vs 4.0 km), so it is a
  property of the algorithms on this drive, not of our setup.
- The raw baselines are all bad, ~26–38 % of the distance. What looks good later comes from the
  corrections (gravity, KISS-ICP heading) and the pole resets, not from the baseline.

## Manual steps if you want to go further

1. `git -C lio_bench/third_party/ros_ws/src/faster-lio log -1` should show `8038fba`.
   `git diff` there should equal `third_party_patches/faster-lio.diff`.
2. `grep point_filter_num results/runs/faster_lio/full/node.log` should show the parameter the
   node actually used.
3. Replay it yourself: `MEM=4G bash lio_bench/scripts/capped.sh bash lio_bench/scripts/run_ros.sh --algos faster_lio --tag full_check`
   This takes about 10 min and writes `results/runs/faster_lio/full_check/`. Then point
   `verify_faster_lio.py` at it. It is deterministic, so it should match `full` closely.
4. For a fully clean build: reset the source (`git checkout .` in that folder), apply only the
   build-fix parts of the diff, rebuild, and replay. The parallel loops then give slightly
   different numbers each run.
