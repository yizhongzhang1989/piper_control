#!/usr/bin/env bash
# Optional user-local alternative to apt install ros-humble-control-msgs.
set -eo pipefail
PIPER_EXTRAS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$PIPER_EXTRAS_ROOT/.runtime/ros_deps"
cd "$PIPER_EXTRAS_ROOT/.runtime/ros_deps"
apt-get download ros-humble-control-msgs
for package in ros-humble-control-msgs_*.deb; do
    dpkg-deb -x "$package" .
done
printf 'Installed user-local ROS action messages. Source scripts/env.sh again.\n'
