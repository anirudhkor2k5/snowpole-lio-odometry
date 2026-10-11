#!/usr/bin/env bash
# Hands-off: finish the whole study and write results/REPORT.md, with nobody at the terminal.
#   1. wait for an already running run_all.sh (WAIT_PID=<pid>), if any
#   2. run_all.sh pass 1 (does every step not yet done)
#   3. run_all.sh pass 2 (retries only the steps that failed in pass 1, e.g. after a fix)
#   4. liobench summary -> results/REPORT.md
# Start it detached so it survives closing the terminal / SSH / the agent session:
#   setsid nohup bash lio_bench/scripts/run_unattended.sh > lio_bench/results/logs/unattended.out 2>&1 < /dev/null &
source "$(dirname "$0")/../setup/common.sh"
set +e
if [[ -n "${WAIT_PID:-}" ]]; then
  log "waiting for running pipeline (pid $WAIT_PID)"
  while kill -0 "$WAIT_PID" 2>/dev/null; do sleep 30; done
fi
for pass in 1 2; do
  log "run_all pass $pass"
  bash "$LIOBENCH_ROOT/scripts/run_all.sh" >> "$LIOBENCH_ROOT/results/logs/run_all.out" 2>&1
done
log "writing results/REPORT.md"
MEM="${MEM_STEP:-4G}" bash "$LIOBENCH_ROOT/scripts/capped.sh" bash "$LIOBENCH_ROOT/scripts/run_all.sh" --exec py summary
log "all done: $LIOBENCH_ROOT/results/REPORT.md"
