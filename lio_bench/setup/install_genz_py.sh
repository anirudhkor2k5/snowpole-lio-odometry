#!/usr/bin/env bash
# Build GenZ-LIO's Python bindings from the pinned source into the lio-py env.
#
# Always built from source (not the PyPI wheel) so every machine runs the same
# code, and so this small patch can be applied:
#   * GenZLIO::seedVelocityBody(v) and Python `seed_velocity_body` - overwrite the
#     filter velocity (body frame). Lets a sliding-window run warm-start from the
#     baseline's velocity instead of assuming the vehicle is at rest.
#   * Python `imu_biases` - (gyro bias, accel bias) of the current state.
# Nothing else in the estimator changes.
#   bash lio_bench/setup/install_genz_py.sh
source "$(dirname "$0")/common.sh"

COMMIT=c6c464de4a8288cbaae890a6390234486d2664d9
SRC="$THIRD_PARTY/genz_lio_py"
if [[ ! -d "$SRC/.git" ]]; then
  log "cloning GenZ-LIO"
  git clone -q https://github.com/cocel-postech/GenZ-LIO.git "$SRC"
fi
git -C "$SRC" fetch -q origin "$COMMIT" 2>/dev/null || true
git -C "$SRC" checkout -q -f "$COMMIT"

HPP="$SRC/cpp/genz_lio/core/GenZLIO.hpp"
PYB="$SRC/python/genz_lio/pybind/genz_lio_pybind.cpp"
if ! grep -q seedVelocityBody "$HPP"; then
  sed -i 's|^\(\s*\)void reset();|\1void reset();\n\1/// lio_bench: overwrite the filter velocity, given in the IMU body frame.\n\1void seedVelocityBody(const Vec3 \&v_body) {\n\1    auto x = kf_.get_x();\n\1    x.vel = x.rot * v_body;\n\1    kf_.change_x(x);\n\1}|' "$HPP"
fi
if ! grep -q seed_velocity_body "$PYB"; then
  sed -i 's|\.def("reset", &GenZLIO::reset)|.def("reset", \&GenZLIO::reset)\n        .def("seed_velocity_body", \&GenZLIO::seedVelocityBody, py::arg("v_body"))  // lio_bench\n        .def_property_readonly("imu_biases", [](const GenZLIO \&s) {  // lio_bench\n            const auto x = s.state();\n            return std::make_pair(Vec3(x.bg), Vec3(x.ba));\n        })|' "$PYB"
fi
grep -q seedVelocityBody "$HPP" && grep -q seed_velocity_body "$PYB" || die "GenZ-LIO patch did not apply"
# lio_bench (v3): IMU initialisation window overridable at run time (GENZ_INIT_IMU_SAMPLES); default unchanged.
IMUP="$SRC/cpp/genz_lio/core/ImuProcessor.cpp"
if ! grep -q GENZ_INIT_IMU_SAMPLES "$IMUP"; then
  python - "$IMUP" <<'PYEOF'
import sys
p = sys.argv[1]; s = open(p).read()
s = s.replace('#include <algorithm>', '#include <algorithm>\n#include <cstdlib>  // lio_bench: GENZ_INIT_IMU_SAMPLES', 1)
s = s.replace('void ImuProcessor::initializeImu(const MeasurementBundle &bundle, EsekfState &kf) {',
 'namespace {\nint initImuSamples(int fallback) {\n    static const int n = [fallback] {\n        const char *e = std::getenv("GENZ_INIT_IMU_SAMPLES");\n'
 '        return (e && std::atoi(e) > 0) ? std::atoi(e) : fallback;\n    }();\n    return n;\n}\n}  // namespace\n\n'
 'void ImuProcessor::initializeImu(const MeasurementBundle &bundle, EsekfState &kf) {', 1)
s = s.replace("if (init_iter_num_ > kInitScanCount) {", "if (init_iter_num_ > initImuSamples(kInitScanCount)) {")
open(p, "w").write(s)
PYEOF
fi
grep -q GENZ_INIT_IMU_SAMPLES "$IMUP" || die "GenZ-LIO init-window patch did not apply"

activate_env "$ENV_PY"
export CMAKE_BUILD_PARALLEL_LEVEL="${JOBS:-$(build_jobs)}"
export CMAKE_PREFIX_PATH="$CONDA_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
log "building GenZ-LIO bindings with $CMAKE_BUILD_PARALLEL_LEVEL jobs"
python -m pip install --no-build-isolation --no-deps -q "$SRC"
python -c "import genz_lio.genz_lio_pybind as g; assert hasattr(g.GenZLIO, 'seed_velocity_body'); print('genz_lio OK')"
