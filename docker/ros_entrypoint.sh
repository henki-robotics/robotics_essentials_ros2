#!/bin/bash
set -e

. "/opt/ros/${ROS_DISTRO}/setup.bash"
. "${ROS2_WS}/install/setup.bash"
. "${EXERCISES_WS}/install/setup.bash"

exec "$@"