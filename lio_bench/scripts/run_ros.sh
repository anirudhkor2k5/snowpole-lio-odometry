#!/usr/bin/env bash
# Run ROS estimators on the bag (headless). All arguments go to ros/scripts/run_ros.py:
#   bash lio_bench/scripts/run_ros.sh --algos fast_lio
#   bash lio_bench/scripts/run_ros.sh --algos fast_lio,dlio            # simultaneous
#   bash lio_bench/scripts/run_ros.sh --algos dlio --start 120 --duration 40 --tag win_120_40
source "$(dirname "$0")/../setup/common.sh"
activate_env "$ENV_ROS"
[[ -f "$ROS_WS/devel/setup.bash" ]] || die "workspace not built; run lio_bench/setup/build_ros_ws.sh"
set +u; source "$ROS_WS/devel/setup.bash"; set -u
exec python "$LIOBENCH_ROOT/ros/scripts/run_ros.py" "$@"
