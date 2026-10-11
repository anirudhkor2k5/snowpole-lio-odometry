#!/usr/bin/env bash
# KISS-ICP config sweep: cost vs heading quality, one at a time, RAM-capped.
source "$(dirname "$0")/../setup/common.sh"
set +e
for p in default vox15 vox20 range50 mppv5; do
  log "kiss sweep: $p"
  MEM=3G bash "$LIOBENCH_ROOT/scripts/capped.sh" bash "$LIOBENCH_ROOT/scripts/run_all.sh" --exec py kiss-probe "$p" \
      > "$LIOBENCH_ROOT/results/logs/kiss_sweep_$p.log" 2>&1
  log "kiss sweep $p exit $?"
done
log "kiss sweep done"
