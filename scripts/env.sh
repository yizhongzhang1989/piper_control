#!/usr/bin/env bash
# Usage: source scripts/env.sh
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    printf 'Use: source scripts/env.sh\n' >&2
    exit 1
fi
PIPER_CONTROL_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)" || return
if [[ -z "${PIPER_WORKSPACE:-}" ]]; then
    if [[ -f "$HOME/Documents/Piper_workspace/install/local_setup.bash" ]]; then
        export PIPER_WORKSPACE="$HOME/Documents/Piper_workspace"
    else
        export PIPER_WORKSPACE="$PIPER_CONTROL_ROOT/vendor/Piper_workspace"
    fi
fi
source /opt/ros/humble/setup.bash || return
if [[ -f "$PIPER_CONTROL_ROOT/.runtime/ros_deps/opt/ros/humble/share/control_msgs/local_setup.bash" ]]; then
    source "$PIPER_CONTROL_ROOT/.runtime/ros_deps/opt/ros/humble/share/control_msgs/local_setup.bash" || return
fi
if [[ -f "$PIPER_WORKSPACE/install/local_setup.bash" ]]; then
    source "$PIPER_WORKSPACE/install/local_setup.bash" || return
fi
if [[ -f "$PIPER_CONTROL_ROOT/.venv/bin/activate" ]]; then
    source "$PIPER_CONTROL_ROOT/.venv/bin/activate" || return
fi
export PIPER_CONFIG_DIR="$PIPER_WORKSPACE/config"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
