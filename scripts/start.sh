#!/usr/bin/env bash
# Launch the commissioned arm_2 profile; both backends wait for public commands.
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
piper_backend="${1:-sim}"
if [[ $# -gt 0 ]]; then shift; fi
case "$piper_backend" in
  hardware)
    source scripts/env.sh
    piper_backend_args=(--backend hardware --arm arm_2 --launch-driver --ros --port 8090)
    ;;
  sim)
    piper_backend_args=(--backend sim --arm-config vendor/Piper_workspace/config/arms/arm_2.yaml --port 8088)
    ;;
  *) printf 'Usage: bash scripts/start.sh [sim|hardware] [additional service options]\n' >&2; exit 2 ;;
esac
exec "${PIPER_CONTROL_PYTHON:-$PWD/.venv/bin/python}" -m piper_control \
  "${piper_backend_args[@]}" --control-profile piper_control/assets/tcp_tracking.yaml \
  --max-velocity 3.141592653589793 --goal-tolerance .005 \
  --joint-goal-tolerances .0012 .0012 .0012 .005 .005 .01 \
  --tcp-position-tolerance .00045 --goal-settle-time 3 --goal-timeout 30 --goal-velocity-tolerance .01 --goal-velocity-window 1 "$@"
