#!/usr/bin/env bash
# v3 queue part 5 (after part 4): is LIO pitch/heading drift an IMU-noise tuning artefact? (Faster-LIO, ROS, real time)
source "$(dirname "$0")/../setup/common.sh"
set +e
until grep -q "v3 queue4 done" "$LIOBENCH_ROOT/results/logs/v3_queue4.out" 2>/dev/null; do sleep 30; done
for v in bgyr100 imunoise10; do
  log "v3 job: faster_lio_$v"
  MEM=4G bash "$LIOBENCH_ROOT/scripts/capped.sh" bash "$LIOBENCH_ROOT/scripts/run_all.sh" --exec ros_tagged faster_lio "full_$v" \
      --config-override "faster_lio=$LIOBENCH_ROOT/configs/algorithms/variants/faster_lio_$v.yaml" \
      > "$LIOBENCH_ROOT/results/logs/v3_faster_lio_$v.log" 2>&1
  log "v3 job faster_lio_$v exit $?"
done
log "v3 queue5 done"
