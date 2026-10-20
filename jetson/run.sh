#!/usr/bin/env bash
# On the drone's Jetson: ZED -> RTAB-Map, plus MAVROS to PX4 (jetson/zed_rtabmap.launch.py).
# Install first with ./jetson/install.sh. The map database is ~/.ros/rtabmap.db.
# Usage: ZED_MODEL=zed2i ./jetson/run.sh [launch args, e.g. fcu_url:=/dev/ttyACM0:57600 rtabmap_viz:=true]
set -eo pipefail
: "${ZED_MODEL:?set ZED_MODEL to the camera model: zed, zedm, zed2, zed2i, zedx, zedxm, ...}"
source /opt/ros/humble/setup.bash
source "${ZED_WS:-$HOME/zed_ws}/install/setup.bash"

ros2 launch "$(cd "$(dirname "$0")" && pwd)/zed_rtabmap.launch.py" camera_model:="$ZED_MODEL" "$@"
