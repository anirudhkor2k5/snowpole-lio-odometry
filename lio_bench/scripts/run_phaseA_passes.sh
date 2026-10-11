#!/usr/bin/env bash
# Phase A4: gravity-locked VGICP (+ IMU-gravity-only) passes for both baselines, one at a time, RAM-capped.
source "$(dirname "$0")/../setup/common.sh"
set +e
for b in faster_lio fast_lio; do
  log "A4 scan pass: $b"
  MEM=3G bash "$LIOBENCH_ROOT/scripts/capped.sh" bash "$LIOBENCH_ROOT/scripts/run_all.sh" --exec py correct scan \
      --baseline "$b" --variants vgicp_grav,grav_only --no-kiss --no-poles --threads 4 \
      > "$LIOBENCH_ROOT/results/logs/phaseA_scan_$b.log" 2>&1
  log "A4 scan pass $b exit $?"
done
log "A4 passes done"
