#!/usr/bin/env bash
# A ZED plugged into this laptop (no NVIDIA GPU needed) -> RTAB-Map (zed_usb/zed_usb_rtabmap.launch.py).
# Requires ros-humble-rtabmap-ros. The map database is ~/.ros/rtabmap.db.
# Usage: ./zed_usb/run.sh [flip:=true] [new_map:=false]
#        (ROS_OVERLAY=path/to/setup.bash also sources an extra workspace, as in sim/bridge.sh)
set -eo pipefail
# Where the ZED SDK runs (the Jetson, or any machine with it installed), its depth is what we want.
# Replaying a recording (camera:=false) needs no camera, so it is fine anywhere.
if [ -z "${ZED_USB_FORCE:-}" ] && [[ " $* " != *" camera:=false "* ]] &&
  { [ -f /etc/nv_tegra_release ] || [ -f /usr/local/zed/include/sl/Camera.hpp ]; }; then
  echo "error: the ZED SDK can run here: use 'make jetson' for its depth instead of zed_usb's CPU stereo" >&2
  echo "       (ZED_USB_FORCE=1 to run zed_usb anyway)" >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
if [ -n "${ROS_OVERLAY:-}" ]; then source "$ROS_OVERLAY"; fi

ros2 launch "$(cd "$(dirname "$0")" && pwd)/zed_usb_rtabmap.launch.py" "$@"
