#!/usr/bin/env bash
# Shows an OctoMap .bt (make save-octomap) in RViz: octomap_server loads the file and publishes its
# occupied cells (colored by height: purple at the floor, then green, yellow, red at the top) and free
# cells; RViz draws them with tools/octomap.rviz.
# Closing RViz stops both.
# Usage: ./tools/view_octomap.sh [file.bt]   (default: ~/maps/map_octomap.bt)
# ROS_OVERLAY=path/to/setup.bash also sources an extra workspace, as in sim/bridge.sh.
set -eo pipefail  # not -u: the ROS setup script reads unset variables

BT="${1:-$HOME/maps/map_octomap.bt}"
if [ ! -f "$BT" ]; then
  echo "No $BT: run make save-octomap first, or pass the .bt to show" >&2
  exit 1
fi
HERE="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/humble/setup.bash
if [ -n "${ROS_OVERLAY:-}" ]; then source "$ROS_OVERLAY"; fi

# The nodes themselves rather than `ros2 run`, so that stopping them on exit leaves nothing behind.
trap 'jobs -p | xargs -r kill 2>/dev/null || true' EXIT
"$(ros2 pkg prefix octomap_server)/lib/octomap_server/octomap_server_node" --ros-args \
  -r __node:=octomap_viewer -p octomap_path:="$(realpath "$BT")" -p frame_id:=map \
  -p use_height_map:=true -p publish_free_space:=true &
# The cells are in frame 'map', RViz's fixed frame; nothing else publishes it here, which RViz flags.
"$(ros2 pkg prefix tf2_ros)/lib/tf2_ros/static_transform_publisher" --frame-id map --child-frame-id octomap \
  --ros-args -r __node:=octomap_viewer_frame >/dev/null &
rviz2 -d "$HERE/octomap.rviz"
