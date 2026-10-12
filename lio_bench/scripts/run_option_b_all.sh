#!/usr/bin/env bash
# Option B for every method: fresh bag runs (one at a time, RAM-capped), tag optb.
# Faster-LIO and its node are reused from rerun_option_b.sh (faster_lio/rerun_b, kiss_fusion/rerun_b_fusion).
#   1. KISS-ICP alone, 2.0 m voxels (offline pass)      -> results/runs/kiss_icp_vox20/optb
#   2-4. each LIO together with its online node (real time) -> results/runs/{fast_lio,dlio,genz_lio}/optb
#        and results/runs/kiss_fusion_{fast_lio,dlio,genz_lio}/optb
# Then: python lio_bench/pipeline/score_option_b.py
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
PY=/home/anirudhk/.conda/envs/lio-py/bin/python
CAP="bash lio_bench/scripts/capped.sh"
log() { echo "[$(date +%H:%M:%S)] $*"; }
START=${START:-1}
if (( START <= 1 )); then log "1/4 KISS-ICP alone, 2.0 m voxels"
MEM=3G $CAP $PY -W ignore -c "
import sys; sys.path.insert(0, 'lio_bench/src')
from liobench.correct import kiss_probe as K
w = K.write_run
K.write_run = lambda name, tag, *a, **k: w(name, 'optb', *a, **k)
print(K.run('vox20'))"; fi
if (( START <= 2 )); then log "2/4 FAST-LIO2 + node (real time)"
MEM=4G $CAP bash lio_bench/scripts/run_ros.sh --algos fast_lio,kiss_fusion_fast_lio --tag optb; fi
if (( START <= 3 )); then log "3/4 DLIO + node (real time)"
MEM=5G $CAP bash lio_bench/scripts/run_ros.sh --algos dlio,kiss_fusion_dlio --tag optb; fi
if (( START <= 4 )); then log "4/4 GenZ-LIO + node (real time)"
MEM=5G $CAP bash lio_bench/scripts/run_ros.sh --algos genz_lio,kiss_fusion_genz_lio --tag optb; fi
log "runs done"
