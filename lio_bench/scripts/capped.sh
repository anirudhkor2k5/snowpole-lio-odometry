#!/usr/bin/env bash
# Run a command inside a hard memory limit with no swap, at low CPU priority, so it
# can never freeze the machine: if it exceeds the limit it is killed (exit 137).
#   MEM=4G bash lio_bench/scripts/capped.sh <command...>
# Needs systemd with user cgroup delegation (Fedora / Ubuntu 22.04+). Without it, runs uncapped.
if command -v systemd-run >/dev/null && systemd-run --user --scope -q -p MemoryMax=10M -- true 2>/dev/null; then
  exec systemd-run --user --scope -q -p MemoryMax="${MEM:-4G}" -p MemorySwapMax=0 -- nice -n 10 "$@"
fi
echo "[capped] no user cgroups; running without a memory cap" >&2
exec nice -n 10 "$@"
