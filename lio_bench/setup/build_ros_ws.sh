#!/usr/bin/env bash
# Clone (pinned) and build FAST-LIO, Faster-LIO, DLIO and the GenZ-LIO ROS1 node
# into lio_bench/third_party/ros_ws inside the lio-ros conda env.
#   bash lio_bench/setup/build_ros_ws.sh [--clean]
source "$(dirname "$0")/common.sh"

# name | url | commit
REPOS=(
  "FAST_LIO|https://github.com/hku-mars/FAST_LIO.git|7cc4175de6f8ba2edf34bab02a42195b141027e9"
  "faster-lio|https://github.com/gaoxiang12/faster-lio.git|8038fbac95283708fe2aa30658b7e116a7ace6d8"
  "direct_lidar_inertial_odometry|https://github.com/vectr-ucla/direct_lidar_inertial_odometry.git|fc8d183f18cdcfb9bb4fc754c6d373cedc4cbd04"
  "GenZ-LIO|https://github.com/cocel-postech/GenZ-LIO.git|c6c464de4a8288cbaae890a6390234486d2664d9"
)

[[ "${1:-}" == "--clean" ]] && { log "removing $ROS_WS/{build,devel,logs}"; rm -rf "$ROS_WS"/{build,devel,logs,.catkin_tools}; }

mkdir -p "$ROS_WS/src"
for entry in "${REPOS[@]}"; do
  IFS='|' read -r name url commit <<<"$entry"
  dir="$ROS_WS/src/$name"
  if [[ ! -d "$dir/.git" ]]; then
    log "cloning $name"
    git clone -q "$url" "$dir"
  fi
  git -C "$dir" fetch -q origin "$commit" 2>/dev/null || true
  git -C "$dir" checkout -q -f "$commit"
  git -C "$dir" submodule update -q --init --recursive
done

# --- source patches (idempotent; each is explained) ---------------------------
FL="$ROS_WS/src/FAST_LIO"
FR="$ROS_WS/src/faster-lio"
GZ="$ROS_WS/src/GenZ-LIO"

# FAST-LIO and Faster-LIO both need livox_ros_driver messages (never used with
# an Ouster, but compiled in). Faster-LIO vendors the package as a CMake
# subdirectory that catkin cannot see, so promote it to a standalone workspace
# package and make Faster-LIO depend on it like FAST-LIO does.
[[ -d "$ROS_WS/src/livox_ros_driver" ]] || cp -r "$FR/thirdparty/livox_ros_driver" "$ROS_WS/src/livox_ros_driver"
# As a subdirectory it never called catkin_package(), so it exported no config.
grep -q '^catkin_package(' "$ROS_WS/src/livox_ros_driver/CMakeLists.txt" || \
  printf '\ncatkin_package(CATKIN_DEPENDS message_runtime std_msgs)\n' >> "$ROS_WS/src/livox_ros_driver/CMakeLists.txt"
sed -i '/add_subdirectory(thirdparty\/livox_ros_driver)/d' "$FR/CMakeLists.txt"
grep -q '        livox_ros_driver' "$FR/cmake/packages.cmake" || \
  sed -i 's/^        eigen_conversions$/        eigen_conversions\n        livox_ros_driver/' "$FR/cmake/packages.cmake"
grep -q '<build_depend>livox_ros_driver</build_depend>' "$FR/package.xml" || \
  sed -i 's#</package>#  <build_depend>livox_ros_driver</build_depend>\n  <run_depend>livox_ros_driver</run_depend>\n</package>#' "$FR/package.xml"  # format-1 manifest

# DLIO includes <sys/vtimes.h>, which glibc >= 2.33 (Ubuntu 22.04+, conda sysroots)
# no longer ships. Nothing from it is used; times() comes from <sys/times.h>.
sed -i '/#include <sys\/vtimes.h>/d' "$ROS_WS/src/direct_lidar_inertial_odometry/include/dlio/dlio.h"
DL="$ROS_WS/src/direct_lidar_inertial_odometry"
# DLIO was written for PCL 1.10 (boost pointers). PCL >= 1.11, which every current
# distro and conda ship, uses std::shared_ptr. All 25 uses are PCL / nanoflann types.
grep -rlE 'boost::(make_shared|shared_ptr)' "$DL/src" "$DL/include" | \
  xargs -r sed -i -E 's/boost::(make_shared|shared_ptr)/std::\1/g'
# DLIO's nanoflann/nano_gicp libraries include dlio.h, which includes the generated
# save_pcd service header, but upstream never declares that dependency: a parallel
# build races. Declare it.
grep -q 'lio_bench: gencpp dependency' "$DL/CMakeLists.txt" || cat >> "$DL/CMakeLists.txt" <<'CMAKE'
# lio_bench: gencpp dependency for libraries that include generated headers
add_dependencies(nanoflann ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(nano_gicp ${PROJECT_NAME}_generate_messages_cpp)
CMAKE

# Faster-LIO's CMake hardcodes -g -ggdb; drop them (smaller, faster build, same code).
sed -i 's/ -g -ggdb//' "$FR/CMakeLists.txt"

# Faster-LIO links the bare name "tbb"; give the linker the env's lib dir.
grep -q 'CONDA_PREFIX' "$FR/src/CMakeLists.txt" || \
  sed -i '1i link_directories($ENV{CONDA_PREFIX}/lib)' "$FR/src/CMakeLists.txt"

# FAST-LIO: allow the number of OpenMP threads to follow the machine on any arch
# (upstream only enables MP_EN on x86 with >=3 cores; on ARM it silently falls to 1).
sed -i 's/if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )/if(TRUE)/' "$FL/CMakeLists.txt"

# FAST-LIO pins C++14, but PCL >= 1.13 refuses to compile below C++17.
sed -i -E 's/-std=c\+\+(0x|14)//g; s/CMAKE_CXX_STANDARD 14/CMAKE_CXX_STANDARD 17/' "$FL/CMakeLists.txt"
# ...and, like DLIO, it never declares that it includes its own generated Pose6D.h.
grep -q 'lio_bench: gencpp dependency' "$FL/CMakeLists.txt" || cat >> "$FL/CMakeLists.txt" <<'CMAKE'
# lio_bench: gencpp dependency for the generated Pose6D.h
add_dependencies(fastlio_mapping ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
CMAKE

# Faster-LIO depended on the in-tree target livox_ros_driver_gencpp; with the
# messages now in their own package, depend on the exported targets instead.
sed -i 's/ livox_ros_driver_gencpp)/ ${catkin_EXPORTED_TARGETS})/' "$FR/src/CMakeLists.txt"

# glog >= 0.7 must be consumed with GLOG_USE_GLOG_EXPORT defined (its CMake target
# does this; Faster-LIO's old FindGlog module does not). Harmless on older glog.
grep -q 'GLOG_USE_GLOG_EXPORT' "$FR/cmake/packages.cmake" || \
  sed -i 's/^find_package(Glog REQUIRED)$/find_package(Glog REQUIRED)\nadd_definitions(-DGLOG_USE_GLOG_EXPORT)/' "$FR/cmake/packages.cmake"

# GenZ-LIO is a monorepo whose only package.xml is ros/ (package `genz_lio`); it
# reaches ../cpp by relative path, so it stays in src/ unmodified. It reads
# ROS_VERSION, which RoboStack's activation sets to 1.

# --- build -------------------------------------------------------------------
activate_env "$ENV_ROS"
JOBS="${JOBS:-$(build_jobs)}"
log "building with $JOBS jobs in $ROS_WS (env $CONDA_PREFIX)"
cd "$ROS_WS"
catkin config --extend "$CONDA_PREFIX" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE="$CONDA_PREFIX/bin/python" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCATKIN_ENABLE_TESTING=OFF -DGENZ_LIO_BUILD_TESTS=OFF >/dev/null
catkin build -j"$JOBS" --no-status --summarize --continue-on-failure
log "done. source $ROS_WS/devel/setup.bash after activating $ENV_ROS"
