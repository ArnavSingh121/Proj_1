#!/usr/bin/env bash
# Terminal 3: RTAB-Map RGB-D SLAM on the bridged /uav topics. Requires ros-humble-rtabmap-ros.
# The map database is ~/.ros/rtabmap.db (recreated on every start unless new_map:=false).
# Usage: ./rtabmap.sh [odometry:=sim] [lidar:=true] [new_map:=false]
#        (ROS_OVERLAY=path/to/setup.bash also sources an extra workspace, as in bridge.sh)
set -eo pipefail
source /opt/ros/humble/setup.bash
if [ -n "${ROS_OVERLAY:-}" ]; then source "$ROS_OVERLAY"; fi
# Shared-memory transport for the camera images (see fastdds.xml); keeps a profile you set yourself.
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$(cd "$(dirname "$0")" && pwd)/fastdds.xml}"

ros2 launch "$(cd "$(dirname "$0")" && pwd)/rtabmap.launch.py" "$@"
