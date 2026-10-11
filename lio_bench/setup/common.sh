# Shared helpers for the lio_bench shell scripts. Source, don't execute.
#   source "$(dirname "$0")/../setup/common.sh"
# bash only: zsh has no BASH_SOURCE and would not find the repo (or conda) correctly.
if [ -z "${BASH_VERSION:-}" ]; then echo "common.sh must be sourced from bash (run: bash <script>)" >&2; return 1 2>/dev/null || exit 1; fi
set -euo pipefail

LIOBENCH_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ROOT="$(cd "$LIOBENCH_ROOT/.." && pwd)"
THIRD_PARTY="${LIOBENCH_THIRD_PARTY:-$LIOBENCH_ROOT/third_party}"
ROS_WS="$THIRD_PARTY/ros_ws"
ENV_PY="${LIOBENCH_ENV_PY:-lio-py}"
ENV_ROS="${LIOBENCH_ENV_ROS:-lio-ros}"
export _ZO_DOCTOR=0  # silence zoxide in non-interactive zsh-sourced shells

log() { printf '\033[1;34m[lio_bench]\033[0m %s\n' "$*" >&2; }
die() { printf '\033[1;31m[lio_bench] error:\033[0m %s\n' "$*" >&2; exit 1; }

# Pick a conda frontend: $CONDA_FRONTEND, then mamba, micromamba, conda.
# If none exists, install Miniforge into $HOME/miniforge3 (works on x86_64 and aarch64).
find_conda() {
  if [[ -n "${CONDA_FRONTEND:-}" ]]; then echo "$CONDA_FRONTEND"; return; fi
  for c in mamba micromamba conda "$HOME/miniforge3/bin/mamba" "$HOME/miniforge3/bin/conda"; do
    if command -v "$c" >/dev/null 2>&1 && "$c" --version >/dev/null 2>&1; then echo "$c"; return; fi
  done
  echo ""
}

install_miniforge() {
  local arch; arch="$(uname -m)"
  local url="https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-${arch}.sh"
  log "no conda found; installing Miniforge ($arch) into $HOME/miniforge3"
  curl -fsSL "$url" -o /tmp/miniforge.sh
  bash /tmp/miniforge.sh -b -p "$HOME/miniforge3"
  rm -f /tmp/miniforge.sh
}

ensure_conda() {
  CONDA="$(find_conda)"
  if [[ -z "$CONDA" ]]; then install_miniforge; CONDA="$HOME/miniforge3/bin/mamba"; fi
  export CONDA
}

# Activate an env inside a non-interactive bash script.
activate_env() {
  local name="$1"
  ensure_conda
  set +u
  case "$(basename "$CONDA")" in
    micromamba) eval "$("$CONDA" shell hook -s bash)"; micromamba activate "$name" ;;
    *)
      # base from `info --base`, else from the binary's location (<base>/bin/mamba); mamba 2 has
      # no `shell.bash hook`, so conda.sh is the portable way in
      local base; base="$("$CONDA" info --base 2>/dev/null || true)"
      [[ -f "$base/etc/profile.d/conda.sh" ]] || base="$(dirname "$(dirname "$(command -v "$CONDA")")")"
      if [[ -f "$base/etc/profile.d/conda.sh" ]]; then source "$base/etc/profile.d/conda.sh"
      else eval "$("$CONDA" shell hook --shell bash 2>/dev/null || "$CONDA" shell.bash hook)"; fi
      conda activate "$name" ;;
  esac
  set -u
  [[ "${CONDA_DEFAULT_ENV:-}" == "$name" || "${CONDA_PREFIX:-}" == */"$name" ]] || die "could not activate env $name"
}

env_exists() {
  ensure_conda
  "$CONDA" env list 2>/dev/null | awk '{print $1}' | grep -qx "$1"
}

# Parallel build jobs: leave headroom; C++ template-heavy LIO code needs ~1.5 GB/job.
build_jobs() {
  local cpus mem_gb by_mem
  cpus="$(nproc)"
  mem_gb="$(awk '/MemAvailable/ {printf "%d", $2/1024/1024}' /proc/meminfo)"
  by_mem=$(( mem_gb / 2 )); (( by_mem < 1 )) && by_mem=1
  echo $(( cpus < by_mem ? cpus : by_mem ))
}
