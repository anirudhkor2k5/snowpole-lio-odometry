#!/usr/bin/env bash
# v3 queue part 2 (waits for part 1): GenZ-LIO offline re-run with full IMU-bias logging.
source "$(dirname "$0")/../setup/common.sh"
set +e
until grep -q "v3 queue done" "$LIOBENCH_ROOT/results/logs/v3_queue.out" 2>/dev/null; do sleep 30; done
log "v3 job: genz_bias_logging"
MEM=4G bash "$LIOBENCH_ROOT/scripts/capped.sh" bash "$LIOBENCH_ROOT/scripts/run_all.sh" --exec py run-genz --tag full_bias \
    > "$LIOBENCH_ROOT/results/logs/v3_genz_bias.log" 2>&1
log "v3 job genz_bias_logging exit $?"
log "v3 queue2 done"
