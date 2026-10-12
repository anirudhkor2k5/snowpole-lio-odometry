#!/usr/bin/env bash
# 5 seeds x 2 readers of verify/kiss_randomness_test.py, one at a time, RAM-capped, then the score table.
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"; cd "$ROOT"
PY=/home/anirudhk/.conda/envs/lio-py/bin/python
T=lio_bench/verify/kiss_randomness_test.py
for seed in 1 2 3 4 5; do for reader in offline node; do
  echo "[$(date +%H:%M:%S)] $reader seed $seed"
  MEM=3G bash lio_bench/scripts/capped.sh $PY -W ignore $T run $reader $seed
done; done
echo "[$(date +%H:%M:%S)] scoring"
MEM=4G bash lio_bench/scripts/capped.sh $PY -W ignore $T score
echo "[$(date +%H:%M:%S)] done"
