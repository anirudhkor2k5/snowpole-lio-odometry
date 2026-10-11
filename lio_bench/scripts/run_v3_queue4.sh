#!/usr/bin/env bash
# v3 queue part 4 (after part 3): GenZ-LIO deskew probes (does residual motion distortion drive LIO yaw bias too?)
source "$(dirname "$0")/../setup/common.sh"
set +e
until grep -q "v3 queue3 done" "$LIOBENCH_ROOT/results/logs/v3_queue3.out" 2>/dev/null; do sleep 30; done
for ts in 0 0.5; do
  log "v3 job: genz_timescale_$ts"
  MEM=4G bash "$LIOBENCH_ROOT/scripts/capped.sh" bash "$LIOBENCH_ROOT/scripts/run_all.sh" --exec py run-genz \
      --tag "full_ts$ts" --time-scale "$ts" > "$LIOBENCH_ROOT/results/logs/v3_genz_ts$ts.log" 2>&1
  log "v3 job genz_timescale_$ts exit $?"
done
log "v3 queue4 done"
