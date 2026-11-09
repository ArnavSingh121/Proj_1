#!/usr/bin/env bash
# Map with the ZED on this laptop: ./map.sh <name> [flip] [localize] [nobag]   e.g. ./map.sh halls
# Adds to Proj_1/maps/<name>.db (running it again continues the same map) and records the camera to a
# ros bag, Proj_1/maps/<name>_bag_<time>/, for ./replay_map.sh <name>. The laptop won't sleep while it
# runs, lid closed or not. Stop with Ctrl-C: RTAB-Map then saves and closes the map, and the bag closes.
#   flip      camera upside down
#   localize  only find where you are in the saved map, without changing it (no bag)
#   nobag     don't record a bag
# Save a copy + 2D grid + 3D cloud at any time from another terminal: ./save_map.sh <name>
set -eo pipefail
[ -n "${1:-}" ] || { echo "usage: $0 <map name> [flip] [localize] [nobag]"; exit 1; }
# The sim uses the same /rtabmap, /tf and /clock names and takes the CPU that mapping needs.
if pgrep -f 'px4_sitl_default/bin/[p]x4$' >/dev/null; then echo "error: the sim is running; ./stop_env.sh first"; exit 1; fi
export ROS_DOMAIN_ID=1
# Loopback only: Wi-Fi hand-offs while walking can't break ROS 2 discovery (see a running map from
# another terminal with: ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=1 ros2 topic list).
export ROS_LOCALHOST_ONLY=1
EXTRA="$HOME/.local/ros-extra/setup.bash"  # RTAB-Map unpacked without sudo; skipped if absent
if [ -f "$EXTRA" ]; then export ROS_OVERLAY="$EXTRA"; fi
FLIP=false LOCALIZE=false BAG=true
for opt in "${@:2}"; do
  case "$opt" in
    flip) FLIP=true ;;
    localize) LOCALIZE=true; BAG=false ;;
    nobag) BAG=false ;;
    *) echo "unknown option: $opt"; exit 1 ;;
  esac
done
HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"  # Proj_1/laptop, also when run through a symlink
PROJ="$(dirname "$HERE")"
LOGS="${PROJ1_LOGS:-$PROJ/logs}"
mkdir -p "$LOGS"
cd "$PROJ"
mkdir -p maps

BAG_PID=
close_bag() {
  [ -n "$BAG_PID" ] || return 0
  kill -INT "$BAG_PID" 2>/dev/null || true
  kill -CONT "$BAG_PID" 2>/dev/null || true
  wait "$BAG_PID" 2>/dev/null || true
  echo "Bag closed: $BAG_DIR ($(du -sh "$BAG_DIR" | cut -f1))"
}
trap close_bag EXIT
trap : INT  # Ctrl-C is for the launch below; this script stays to close the bag

if [ "$BAG" = true ]; then
  BAG_DIR="$PWD/maps/$1_bag_$(date +%Y%m%d-%H%M%S)"
  Z=/zed/zed_node
  source /opt/ros/humble/setup.bash
  # Job control gives the recorder its own process group with SIGINT working, so close_bag decides
  # when it stops: after the map has closed, not at the same moment.
  # </dev/null: its keyboard handler touches the terminal, which stops a background process (state T).
  set -m
  ros2 bag record -o "$BAG_DIR" "$Z/rgb/color/rect/image" "$Z/rgb/color/rect/camera_info" \
    "$Z/depth/depth_registered" "$Z/depth/camera_info" </dev/null >"$LOGS/bag_$1.log" 2>&1 &
  BAG_PID=$!
  set +m
  echo "Recording the camera to $BAG_DIR"
  # Say so loudly if it isn't: a walk can't be redone later.
  (sleep 15; ls "$BAG_DIR"/*.db3 >/dev/null 2>&1 || echo "

  ***** WARNING: NOT RECORDING A BAG. See $LOGS/bag_$1.log *****

") &
fi

systemd-inhibit --what=sleep:idle:handle-lid-switch --who=Proj_1 --why="Mapping $1" \
  ./zed_usb/run.sh flip:="$FLIP" new_map:=false localization:="$LOCALIZE" \
  database_path:="$PWD/maps/$1.db" || true
