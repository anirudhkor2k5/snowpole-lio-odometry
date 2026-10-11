#!/usr/bin/env bash
# Whole study, unattended and resumable. Run inside tmux on the cloud machine:
#   tmux new -s lio 'bash lio_bench/scripts/run_all.sh'
# A finished step leaves results/.done/<step>; re-running skips it. FORCE=1 redoes all,
# or delete one marker to redo that step. Logs: results/logs/<step>.log.
#
# Env knobs:
#   BASELINES="fast_lio faster_lio"   SUPPORTS_ROS="dlio genz_lio"
#   PARALLEL_SOLO=0   1 = run the solo ROS runs at the same time (needs >= 16 cores;
#                     leave 0 if you want clean latency/CPU numbers)
#   SKIP="windows_dlio ..."  step names to skip
#   MEM_STEP=4G       hard RAM cap per step (default: 4G below 24 GB of RAM, else 3/4 of RAM).
#                     A step that exceeds it is OOM-killed and marked FAILED; the rest continues.
source "$(dirname "$0")/../setup/common.sh"
set +e

# Internal: `run_all.sh --exec <function> args...` runs one step body inside its own capped scope.
if [[ "${1:-}" == --exec ]]; then EXEC_MODE=1; shift; fi

RES="$LIOBENCH_ROOT/results"
mkdir -p "$RES/.done" "$RES/logs"
BASELINES="${BASELINES:-fast_lio faster_lio}"
SUPPORTS_ROS="${SUPPORTS_ROS:-dlio genz_lio}"
FAILED=()

step() {  # step <name> <command...>
  local name="$1"; shift
  if [[ " ${SKIP:-} " == *" $name "* ]]; then log "skip $name (SKIP)"; return 0; fi
  if [[ -f "$RES/.done/$name" && "${FORCE:-0}" != 1 ]]; then log "done already: $name"; return 0; fi
  log "=== $name"
  local t0=$SECONDS
  if MEM="$MEM_STEP" bash "$LIOBENCH_ROOT/scripts/capped.sh" bash "$SELF" --exec "$@" > "$RES/logs/$name.log" 2>&1; then
    touch "$RES/.done/$name"; log "ok $name ($(( SECONDS - t0 )) s)"
  else
    FAILED+=("$name"); log "FAILED $name - see $RES/logs/$name.log (continuing)"
  fi
}

py() { activate_env "$ENV_PY"; liobench "$@"; }
ros() { bash "$LIOBENCH_ROOT/scripts/run_ros.sh" "$@"; }
ros_tagged() { local algo="$1" tag="$2"; shift 2; ros --algos "$algo" --tag "$tag" "$@"; }

SELF="$LIOBENCH_ROOT/scripts/run_all.sh"
if [[ -z "${MEM_STEP:-}" ]]; then
  total_gb=$(awk '/MemTotal/ {printf "%d", $2/1024/1024}' /proc/meminfo)
  if (( total_gb < 24 )); then MEM_STEP=4G; else MEM_STEP="$(( total_gb * 3 / 4 ))G"; fi
fi
if [[ "${EXEC_MODE:-0}" == 1 ]]; then "$@"; exit $?; fi
log "per-step RAM cap: $MEM_STEP"

# 0. one-off facts from the bag (clock fit, INS heading, CAN speed)
step prepare py prepare

# 1. continuous runs ------------------------------------------------------------
step genz_python_full py run-genz --scan-features            # also writes per-scan context features
if [[ "${PARALLEL_SOLO:-0}" == 1 ]]; then
  pids=()
  for a in $BASELINES $SUPPORTS_ROS; do
    tag=full; [[ $a == genz_lio ]] && tag=full_ros
    step "ros_${a}" ros_tagged "$a" "$tag" & pids+=($!)
    sleep 5
  done
  wait "${pids[@]}"
else
  for a in $BASELINES $SUPPORTS_ROS; do
    tag=full; [[ $a == genz_lio ]] && tag=full_ros   # GenZ 'full' is the Python run
    step "ros_${a}" ros_tagged "$a" "$tag"
  done
fi

# FAST-LIO with point_filter_num 4 (aliased column pattern): reproduces how the legacy fastlio.csv
# behaves, so verify can compare the CSV with an equivalent re-run as well as with the fixed config.
if [[ " $BASELINES " == *" fast_lio "* ]]; then
  step ros_fast_lio_pf4_legacy ros_tagged fast_lio full_pf4_legacy \
    --config-override "fast_lio=$LIOBENCH_ROOT/configs/algorithms/variants/fast_lio_pf4_legacy.yaml"
fi

# 2. step 1 of the plan: verify the legacy CSVs (and our re-runs) against ground truth
step verify py verify

# 3. step 3: weakness analysis of the baselines
step analyze py analyze

# 4. baseline + support running simultaneously
for b in $BASELINES; do
  for s in $SUPPORTS_ROS; do
    step "concurrent_${b}+${s}" ros --algos "${b},${s}"
    step "concurrency_report_${b}+${s}" py concurrency --baseline "$b" --support "$s"
  done
done

# 5. step 4: learned gating (+ trigger model) per baseline
for b in $BASELINES; do
  step "fusion_${b}" py fusion --baseline "$b"
done

# 5b. phase 2: correction strategies per baseline (wheel speed, ICP variants seeded by the
# baseline incl. degeneracy-aware fusion, KISS-ICP, pole-map fixes), then fusion over the
# supports plus the strategy selected on validation. KISS-ICP is baseline-independent: run once.
first=1
for b in $BASELINES; do
  kiss=""; (( first )) || kiss="--no-kiss"; first=0
  step "correct_wheel_${b}"  py correct wheel  --baseline "$b"
  step "correct_scan_${b}"   py correct scan   --baseline "$b" $kiss
  step "correct_poles_${b}"  py correct poles  --baseline "$b"
  step "correct_report_${b}" py correct report --baseline "$b"
  step "fusion_${b}_correctors" py fusion --baseline "$b" --supports correctors --out-name "${b}_correctors"
done

# 6. step 5: support in sliding windows (GenZ is offline and fast; DLIO replays in real time)
for b in $BASELINES; do
  step "windows_${b}_genz_adaptive_warm"  py windows --baseline "$b" --support genz_lio --schedule adaptive
  step "windows_${b}_genz_adaptive_cold"  py windows --baseline "$b" --support genz_lio --schedule adaptive --cold
  step "windows_${b}_genz_periodic_warm"  py windows --baseline "$b" --support genz_lio --schedule periodic
  step "windows_${b}_genz_adaptive_q90"   py windows --baseline "$b" --support genz_lio --schedule adaptive --threshold-q 0.9
  step "windows_${b}_dlio_adaptive_cold"  py windows --baseline "$b" --support dlio --schedule adaptive --cold
done

step unit_tests bash -c "source '$LIOBENCH_ROOT/setup/common.sh'; activate_env '$ENV_PY'; python -m pytest '$LIOBENCH_ROOT/tests' -q"

log "finished. Summaries:"
for f in verify/verify_summary.md analysis/analysis_summary.md fusion/*/fusion_summary.md correct/*/correct_summary.md windows/*/*/windows_summary.md concurrency/*.json; do
  ls "$RES"/$f 2>/dev/null
done
if (( ${#FAILED[@]} )); then log "failed steps: ${FAILED[*]}"; exit 1; fi
