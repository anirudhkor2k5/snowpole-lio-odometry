#!/usr/bin/env bash
# v3 queue 7: gyro bias frozen at the (correct) standstill estimate. Is the LIO heading bias carried by bias wander?
source "$(dirname "$0")/../setup/common.sh"
set +e
R="$LIOBENCH_ROOT/scripts/run_all.sh"; L="$LIOBENCH_ROOT/results/logs"; V="$LIOBENCH_ROOT/configs/algorithms/variants"
log "v3 job: faster_lio_bgyrfrozen"
MEM=4G bash "$LIOBENCH_ROOT/scripts/capped.sh" bash "$R" --exec ros_tagged faster_lio full_bgyrfrozen \
    --config-override "faster_lio=$V/faster_lio_bgyrfrozen.yaml" > "$L/v3_faster_lio_bgyrfrozen.log" 2>&1
log "v3 job faster_lio_bgyrfrozen exit $?"
log "v3 job: genz_bgyrfrozen"
MEM=4G bash "$LIOBENCH_ROOT/scripts/capped.sh" bash "$R" --exec py run-genz --tag full_bgyrfrozen \
    --config "$V/genz_lio_bgyrfrozen.yaml" > "$L/v3_genz_bgyrfrozen.log" 2>&1
log "v3 job genz_bgyrfrozen exit $?"
log "v3 queue7 done"
