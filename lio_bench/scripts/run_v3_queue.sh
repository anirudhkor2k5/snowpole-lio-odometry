#!/usr/bin/env bash
# v3 heavy jobs, strictly one at a time, each RAM-capped. Survives the session ending.
source "$(dirname "$0")/../setup/common.sh"
set +e
R="$LIOBENCH_ROOT/scripts/run_all.sh"
L="$LIOBENCH_ROOT/results/logs"
job() { local name="$1" mem="$2"; shift 2; log "v3 job: $name"; local t0=$SECONDS
  MEM="$mem" bash "$LIOBENCH_ROOT/scripts/capped.sh" bash "$R" --exec "$@" > "$L/v3_$name.log" 2>&1
  log "v3 job $name exit $? ($(( SECONDS - t0 )) s)"; }
job scan_selective_faster_lio 3G py correct scan --baseline faster_lio \
    --variants vgicp_grav_da,vgicp_grav_every2,vgicp_grav_every5,vgicp_grav_every10,vgicp_grav_every20 --no-kiss --no-poles --threads 4
for p in nodeskew revtime vox05 shift5ms; do job "kiss_$p" 3G py kiss-probe "$p"; done
job repeat_fast_lio_r2 4G ros_tagged fast_lio full_r2
job repeat_dlio_r2     4G ros_tagged dlio full_r2
job repeat_fast_lio_r3 4G ros_tagged fast_lio full_r3
job repeat_dlio_r3     4G ros_tagged dlio full_r3
log "v3 queue done"
