#!/usr/bin/env bash
set -euo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "/ws/install/setup.bash"

exec "$@"
