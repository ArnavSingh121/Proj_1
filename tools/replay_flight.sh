#!/usr/bin/env bash
# Replays a recorded flight through RTAB-Map, so its map builds up again on screen. No sim, drone or camera.
# Usage: tools/replay_flight.sh [flight folder | folder name | latest] [speed]   (make replay-flight)
# Works for simulated flights (through sim/rtabmap.launch.py) and drone flights (the ZED topics, through
# zed_usb/zed_usb_rtabmap.launch.py without a camera). It maps into <flight>/replay.db, so the recorded map
# is never changed. RTAB-Map keeps running after the recording ends so the map can be looked at; Ctrl-C stops.
# Recordings are in $FLIGHT_RECORDINGS (default ~/flight_recordings). ROS_OVERLAY=path/to/setup.bash also
# sources an extra workspace, as in sim/bridge.sh.
set -eo pipefail
ROOT="${FLIGHT_RECORDINGS:-$HOME/flight_recordings}"
FLIGHT="${1:-latest}"
SPEED="${2:-1.0}"
[ -d "$FLIGHT" ] || FLIGHT="$ROOT/$FLIGHT"
FLIGHT="$(cd "$FLIGHT" 2>/dev/null && pwd -P)" || { echo "error: no flight '${1:-latest}' in $ROOT" >&2; exit 1; }
BAG="$FLIGHT/bag"
[ -f "$BAG/metadata.yaml" ] || { echo "error: no closed recording in $FLIGHT (no bag/metadata.yaml)" >&2; exit 1; }
# The sim publishes /clock and the same /rtabmap topics, which would scramble the replay.
if pgrep -f 'px4_sitl_default/bin/[p]x4$' >/dev/null; then
  echo "error: the simulation is running; stop it first" >&2
  exit 1
fi
HERE="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [ -n "${ROS_OVERLAY:-}" ]; then source "$ROS_OVERLAY"; fi

if grep -q '/uav/rgb/image' "$BAG/metadata.yaml"; then
  # Simulated flight: the recorded /clock drives sim time. The recorder stays off, or the replay would
  # record itself.
  TOPICS=(/uav/rgb/image /uav/rgb/camera_info /uav/depth/image /uav/depth/camera_info /uav/odom /clock)
  CLOCK=()
  LAUNCH=("$HERE/sim/rtabmap.sh" flight_recorder:=false)
else
  # Drone flight: the camera topics, timed by the bag (--clock).
  Z=/zed/zed_node
  TOPICS=("$Z/rgb/color/rect/image" "$Z/rgb/color/rect/camera_info" "$Z/depth/depth_registered" "$Z/depth/camera_info")
  CLOCK=(--clock 100)
  LAUNCH=("$HERE/zed_usb/run.sh" camera:=false use_sim_time:=true)
fi
echo "Replaying $FLIGHT at ${SPEED}x (map: $FLIGHT/replay.db)"

PLAY_PID=
stop_play() { [ -z "$PLAY_PID" ] || kill -INT "$PLAY_PID" 2>/dev/null || true; }
trap stop_play EXIT
trap : INT  # Ctrl-C is for RTAB-Map below; this script stays to stop the player
# Job control: the player gets its own process group, with SIGINT working. </dev/null and
# --disable-keyboard-controls: its keyboard handler would otherwise stop it (state T) on a terminal.
# The wait gives RTAB-Map time to subscribe, or the first frames of the flight are lost.
set -m
(sleep "${REPLAY_DELAY:-10}"; exec ros2 bag play "$BAG" --topics "${TOPICS[@]}" "${CLOCK[@]}" --rate "$SPEED" \
  --disable-keyboard-controls </dev/null) &
PLAY_PID=$!
set +m
"${LAUNCH[@]}" new_map:=true database_path:="$FLIGHT/replay.db" || true
