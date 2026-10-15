#!/usr/bin/env bash
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f /ws/install/setup.bash ]]; then
  source /ws/install/setup.bash
fi
exec "$@"
