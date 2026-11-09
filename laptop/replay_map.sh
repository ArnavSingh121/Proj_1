#!/usr/bin/env bash
# Replay a recorded walk: ./replay_map.sh <name> [speed] [bag folder]   e.g. ./replay_map.sh halls 2
# Plays the newest maps/<name>_bag_*/ through RTAB-Map, so the map builds up again on screen. It maps
# into maps/<name>_replay.db (fresh each time), never into maps/<name>.db. RTAB-Map keeps running after
# the bag ends so the map can be looked at. Stop with Ctrl-C. No camera needed.
set -eo pipefail
[ -n "${1:-}" ] || { echo "usage: $0 <map name> [speed] [bag folder]"; exit 1; }
# The sim publishes /clock too, which would scramble the bag's timestamps.
if pgrep -f 'px4_sitl_default/bin/[p]x4$' >/dev/null; then echo "error: the sim is running; ./stop_env.sh first"; exit 1; fi
SPEED="${2:-1.0}"
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=1
EXTRA="$HOME/.local/ros-extra/setup.bash"  # RTAB-Map unpacked without sudo; skipped if absent
if [ -f "$EXTRA" ]; then export ROS_OVERLAY="$EXTRA"; fi
cd "$(dirname "$(dirname "$(readlink -f "$0")")")"
BAG="${3:-$(ls -dt maps/"$1"_bag_*/ 2>/dev/null | head -1)}"
[ -d "$BAG" ] || { echo "no bag found for '$1' in maps/"; exit 1; }
source /opt/ros/humble/setup.bash
echo "Replaying $BAG at ${SPEED}x into maps/$1_replay.db"

PLAY_PID=
stop_play() { [ -z "$PLAY_PID" ] || kill -INT "$PLAY_PID" 2>/dev/null || true; }
trap stop_play EXIT
trap : INT
# Give RTAB-Map time to subscribe, or the first frames of the walk are lost.
set -m
(sleep 10; exec ros2 bag play "$BAG" --clock 100 --rate "$SPEED" --disable-keyboard-controls </dev/null) &
PLAY_PID=$!
set +m

systemd-inhibit --what=sleep:idle:handle-lid-switch --who=Proj_1 --why="Replaying $1" \
  ./zed_usb/run.sh camera:=false use_sim_time:=true new_map:=true \
  database_path:="$PWD/maps/$1_replay.db" || true
