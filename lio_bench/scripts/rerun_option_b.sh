#!/usr/bin/env bash
# Fresh runs on the bag for the four option B CSVs. Each step is RAM-capped and runs alone.
# New tags only; the earlier runs (faster_lio/full, kiss_icp/full, kiss_fusion/rt_fusion_v20) are not touched.
#   1. Faster-LIO alone, real time                      -> results/runs/faster_lio/rerun_b
#   2. KISS-ICP (offline scan pass, LiDAR only, 1.0 m)   -> results/runs/kiss_icp/rerun_b
#   3. Faster-LIO + online fusion node, real time        -> results/runs/{faster_lio,kiss_fusion}/rerun_b_fusion
# The CSVs and scores are made afterwards by scripts/run_option_b_all.sh + pipeline/score_option_b.py
# (results/export_optionB_all/); the old step 4 (export_option_b.py -> export_optionB_rerun/) was retired.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
PY=/home/anirudhk/.conda/envs/lio-py/bin/python
LB=/home/anirudhk/.conda/envs/lio-py/bin/liobench
CAP="bash lio_bench/scripts/capped.sh"
log() { echo "[$(date +%H:%M:%S)] $*"; }
START=${START:-1}     # resume: START=2 skips step 1, etc.

if (( START <= 1 )); then log "1/4 Faster-LIO on the bag (real time, ~9.5 min)"
MEM=4G $CAP bash lio_bench/scripts/run_ros.sh --algos faster_lio --tag rerun_b; fi
if (( START <= 2 )); then log "2/4 KISS-ICP scan pass on the bag"
# --baseline only feeds the ICP-corrector variants, which are switched off here; KISS-ICP itself is LiDAR only
MEM=4G $CAP $LB correct scan --baseline faster_lio --baseline-tag full --tag rerun_b --variants "" --no-poles; fi
if (( START <= 3 )); then log "3/4 Faster-LIO + online fusion node on the bag (real time, ~9.5 min)"
MEM=4G $CAP bash lio_bench/scripts/run_ros.sh --algos faster_lio,kiss_fusion --tag rerun_b_fusion; fi
log "done"
