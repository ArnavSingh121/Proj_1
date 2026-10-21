#!/usr/bin/env bash
# A ZED plugged into this laptop (no NVIDIA GPU needed) -> RTAB-Map (zed_usb/zed_usb_rtabmap.launch.py).
# Requires ros-humble-rtabmap-ros. The map database is ~/.ros/rtabmap.db.
# Usage: ./zed_usb/run.sh [flip:=true] [new_map:=false]
#        (ROS_OVERLAY=path/to/setup.bash also sources an extra workspace, as in sim/bridge.sh)
set -eo pipefail
source /opt/ros/humble/setup.bash
if [ -n "${ROS_OVERLAY:-}" ]; then source "$ROS_OVERLAY"; fi

ros2 launch "$(cd "$(dirname "$0")" && pwd)/zed_usb_rtabmap.launch.py" "$@"
