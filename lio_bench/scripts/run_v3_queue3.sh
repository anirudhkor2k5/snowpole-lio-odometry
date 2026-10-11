#!/usr/bin/env bash
# v3 queue part 3 (after part 2): yaw-leakage test, VGICP with constant-velocity prediction.
source "$(dirname "$0")/../setup/common.sh"
set +e
until grep -q "v3 queue2 done" "$LIOBENCH_ROOT/results/logs/v3_queue2.out" 2>/dev/null; do sleep 30; done
log "v3 job: scan_cv_faster_lio"
MEM=3G bash "$LIOBENCH_ROOT/scripts/capped.sh" bash "$LIOBENCH_ROOT/scripts/run_all.sh" --exec py correct scan --baseline faster_lio \
    --variants vgicp_grav_cvrot,vgicp_grav_cv --no-kiss --no-poles --threads 4 > "$LIOBENCH_ROOT/results/logs/v3_scan_cv.log" 2>&1
log "v3 job scan_cv_faster_lio exit $?"
log "v3 queue3 done"
