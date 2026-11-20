#!/usr/bin/env bash
# A ZED plugged into this laptop (no NVIDIA GPU needed) -> RTAB-Map (zed_usb/zed_usb_rtabmap.launch.py).
# Requires ros-humble-rtabmap-ros. The map database is ~/.ros/rtabmap.db.
# Usage: ./zed_usb/run.sh [flip:=true] [new_map:=false]
#        (ROS_OVERLAY=path/to/setup.bash also sources an extra workspace, as in sim/bridge.sh)
set -eo pipefail
source /opt/ros/humble/setup.bash
if [ -n "${ROS_OVERLAY:-}" ]; then source "$ROS_OVERLAY"; fi
# Shared-memory transport for the camera images (see sim/fastdds.xml); keeps a profile you set yourself.
# Over UDP they reach RTAB-Map late and in bursts once the CPU is busy, and odometry loses track.
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$(cd "$(dirname "$0")/../sim" && pwd)/fastdds.xml}"

# On Ctrl+C RTAB-Map writes the map still in memory to its database, which takes more than launch's
# default 5 s once the map is large; then launch kills it mid-write. Give it a minute.
ros2 launch "$(cd "$(dirname "$0")" && pwd)/zed_usb_rtabmap.launch.py" sigterm_timeout:=60 "$@"
