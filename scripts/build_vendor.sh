#!/usr/bin/env bash
set -eo pipefail
PIPER_BUILD_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
cd "$PIPER_BUILD_ROOT/vendor/Piper_workspace"
colcon build --base-paths src --packages-select piper_description piper_x_interfaces piper_x_driver piper_gravity piper_friction --symlink-install "$@" --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING="${PIPER_BUILD_TESTING:-OFF}"
