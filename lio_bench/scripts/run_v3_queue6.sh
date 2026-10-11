#!/usr/bin/env bash
# v3 queue part 6 (after part 5): KISS-ICP geometry probes (where does the pitch-up bias come from?)
source "$(dirname "$0")/../setup/common.sh"
set +e
until grep -q "v3 queue5 done" "$LIOBENCH_ROOT/results/logs/v3_queue5.out" 2>/dev/null; do sleep 30; done
for p in noground minr10 upperbeams; do
  log "v3 job: kiss_$p"
  MEM=3G bash "$LIOBENCH_ROOT/scripts/capped.sh" bash "$LIOBENCH_ROOT/scripts/run_all.sh" --exec py kiss-probe "$p" \
      > "$LIOBENCH_ROOT/results/logs/v3_kiss_$p.log" 2>&1
  log "v3 job kiss_$p exit $?"
done
log "v3 queue6 done"
