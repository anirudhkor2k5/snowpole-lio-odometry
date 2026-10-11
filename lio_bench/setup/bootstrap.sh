#!/usr/bin/env bash
# One-shot setup on the cloud machine (Linux x86_64 or aarch64). Idempotent: re-run after a failure.
#   bash lio_bench/setup/bootstrap.sh
# Steps: conda (installs Miniforge if none) -> envs lio-py + lio-ros -> ROS workspace
# (FAST-LIO, Faster-LIO, DLIO, GenZ-LIO) -> GenZ-LIO Python bindings -> liobench package.
source "$(dirname "$0")/common.sh"

ensure_conda
log "conda frontend: $CONDA"
command -v git >/dev/null || die "git is required"

for env in "$ENV_PY:lio-py.yml" "$ENV_ROS:lio-ros.yml"; do
  name="${env%%:*}"; file="$LIOBENCH_ROOT/envs/${env##*:}"
  if env_exists "$name"; then
    log "updating env $name"
    case "$(basename "$CONDA")" in
      micromamba) "$CONDA" install -y -n "$name" -f "$file" ;;
      *) "$CONDA" env update -n "$name" -f "$file" ;;
    esac
  else
    log "creating env $name (several minutes)"
    case "$(basename "$CONDA")" in
      micromamba) "$CONDA" create -y -n "$name" -f "$file" ;;
      *) "$CONDA" env create -y -n "$name" -f "$file" ;;
    esac
  fi
done

bash "$LIOBENCH_ROOT/setup/build_ros_ws.sh"
bash "$LIOBENCH_ROOT/setup/install_genz_py.sh"

activate_env "$ENV_PY"
python -m pip install -q --no-deps --no-build-isolation -e "$LIOBENCH_ROOT"
python -c "import liobench, genz_lio; print('liobench + genz_lio import OK')"

# Data check (does not copy anything).
python - <<'PY'
from liobench.io import load_config, repo_path
cfg = load_config()
need = [cfg["bag"], cfg["reduced_bag"], cfg["ground_truth"], cfg["gnss_left"], cfg["gnss_right"]] + \
       [v["file"] for v in cfg["legacy_exports"].values()]
missing = [n for n in need if not repo_path(n).exists()]
print("data root:", cfg["_data_root"])
print("MISSING:", missing) if missing else print("all data files present")
PY
log "bootstrap done. Next: tmux new -s lio 'bash lio_bench/scripts/run_all.sh'"
