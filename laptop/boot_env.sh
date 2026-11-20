#!/usr/bin/env bash
# Boots the whole Proj_1 sim stack on this machine, each part in the background with a log:
#   QGroundControl -> PX4 + Gazebo -> Gazebo-to-ROS 2 bridge -> RTAB-Map -> MAVROS
# Usage: ./boot_env.sh [world] [--fly] [--headless]   world defaults to default_warehouse; --fly flies a demo
#        flight (take off, turn 360 deg, land), which the flight recorder saves to ~/flight_recordings; --headless runs without the Gazebo and RTAB-Map windows (frees CPU)
# Stop:  ./stop_env.sh        Logs: Proj_1/logs/ (or $PROJ1_LOGS)
# Made for a laptop with RTAB-Map, MAVROS, the Harmonic bridge and QGroundControl installed without sudo
# (laptop/README.md); with apt installs those overlays are simply skipped.
set -o pipefail   # no -u: /opt/ros/humble/setup.bash reads unset variables

WORLD=default_warehouse
FLY=0
VIZ=true
for arg in "$@"; do
  case "$arg" in
    --fly) FLY=1 ;;
    --headless) export HEADLESS=1; VIZ=false ;;  # HEADLESS: read by PX4's Gazebo start-up
    *) WORLD="$arg" ;;
  esac
done

HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"  # Proj_1/laptop, also when run through a symlink
PROJ="$(dirname "$HERE")"
LOGS="${PROJ1_LOGS:-$PROJ/logs}"
mkdir -p "$LOGS"
export ROS_DOMAIN_ID=1
GZ_OVERLAY="$HOME/ros_gz_harmonic_ws/install/setup.bash"   # Harmonic ros_gz_bridge built from source
EXTRA="$HOME/.local/ros-extra/setup.bash"                  # RTAB-Map, OctoMap, MAVROS unpacked without sudo
QGC="$HOME/.local/qgc/v5.0.8/squashfs-root/AppRun"         # v5.0.8: last release that runs on Ubuntu 22.04
QGC_LIBS="$HOME/.local/qgc-libs/root/usr/lib/x86_64-linux-gnu"
QGC_APPIMAGE="$HOME/QGroundControl-v5.0.8-x86_64.AppImage"  # what `make qgc` installs
PX4_BIN="$HOME/PX4-Autopilot/build/px4_sitl_default/bin"

source /opt/ros/humble/setup.bash
if [ -f "$EXTRA" ]; then source "$EXTRA"; else EXTRA=; fi
[ -f "$GZ_OVERLAY" ] || GZ_OVERLAY=

# wait_for <seconds> <description> <command...>
wait_for() {
  local t=$1 what=$2
  shift 2
  for ((i = 0; i < t; i++)); do
    if "$@" >/dev/null 2>&1; then echo "  ok: $what"; return 0; fi
    sleep 1
  done
  echo "  FAILED: $what (see $LOGS)"
  return 1
}

# turn_to <yaw>: MAV_CMD_DO_REPOSITION with only a new heading (radians, clockwise from north).
# Time limit and retries: under load the reply to a new service client can get lost, and
# `ros2 service call` then waits forever. Asking again for the same heading is harmless.
turn_to() {
  for _ in 1 2 3; do
    timeout 15 ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong \
      "{command: 192, param1: -1.0, param2: 1.0, param3: 0.0, param4: $1, param5: .nan, param6: .nan, param7: .nan}" |
      grep -q 'success=True' && return 0
  done
  return 1
}

# Straight from PX4, so never a stale reading: at least 2 m up and in Hold (nav_state 4).
at_altitude() {
  "$PX4_BIN/px4-listener" vehicle_local_position -n 1 | awk '$1 == "z:" { exit !($2 < -2.0) }' &&
    "$PX4_BIN/px4-listener" vehicle_status -n 1 | grep -qE '^ +nav_state: 4$'
}
disarmed() { "$PX4_BIN/px4-listener" vehicle_status -n 1 | grep -qE '^ +arming_state: 1$'; }

if pgrep -f "$PX4_BIN/px4\$" >/dev/null; then
  echo "The sim is already running; run $HERE/stop_env.sh first."
  exit 1
fi

echo "QGroundControl"
if pgrep -f 'usr/bin/[Q]GroundControl' >/dev/null; then
  echo "  ok: already running"
elif [ -x "$QGC" ]; then
  (cd "$(dirname "$QGC")/.." && LD_LIBRARY_PATH="$QGC_LIBS" setsid "$QGC" >"$LOGS/qgc.log" 2>&1 &)
  echo "  ok: started"
elif [ -x "$QGC_APPIMAGE" ]; then
  (setsid "$QGC_APPIMAGE" >"$LOGS/qgc.log" 2>&1 &)
  echo "  ok: started"
else
  echo "  not found ($QGC or $QGC_APPIMAGE): PX4 will wait for a ground station"
fi

echo "PX4 + Gazebo ($WORLD)"
setsid bash -c "cd '$PROJ' && tail -f /dev/null | WORLD='$WORLD' make sim" >"$LOGS/sim.log" 2>&1 &
wait_for 300 "PX4 started" grep -q 'Startup script returned' "$LOGS/sim.log" || exit 1
if grep -q 'Timed out waiting for Gazebo world' "$LOGS/sim.log"; then echo "  FAILED: world '$WORLD' did not load"; exit 1; fi

echo "Gazebo -> ROS 2 bridge"
setsid bash -c "cd '$PROJ' && WORLD='$WORLD' ROS_OVERLAY='$GZ_OVERLAY' make bridge" >"$LOGS/bridge.log" 2>&1 &
wait_for 60 "/uav topics have data" timeout 5 ros2 topic echo --once --no-arr /uav/depth/image || exit 1

echo "RTAB-Map"
setsid bash -c "cd '$PROJ' && ROS_OVERLAY='$EXTRA' ./sim/rtabmap.sh rtabmap_viz:=$VIZ" >"$LOGS/rtabmap.log" 2>&1 &
wait_for 90 "RTAB-Map is mapping" grep -q 'rtabmap (1)' "$LOGS/rtabmap.log" || exit 1

echo "MAVROS"
setsid ros2 launch "$PROJ/sim/mavros.launch.py" >"$LOGS/mavros.log" 2>&1 &  # sim time: see the launch file
wait_for 60 "MAVROS connected to PX4" bash -c 'ros2 topic echo --once /mavros/state | grep -q "connected: true"' || exit 1

wait_for 60 "PX4 ready for takeoff (QGroundControl connected)" grep -q 'Ready for takeoff' "$LOGS/sim.log" || exit 1

if [ "$FLY" = 1 ]; then
  echo "Demo flight: take off, turn 360 deg, land (the flight recorder saves it)"
  touch "$LOGS/.fly_start"
  "$PX4_BIN/px4-commander" takeoff
  # PX4 already reports Hold on the ground, and the sim runs below real time, so wait for real altitude.
  # Turn commands sent during the climb are dropped, or cancel the takeoff.
  wait_for 180 "at takeoff altitude, in Hold" at_altitude || exit 1
  # 45 deg steps.
  for yaw in 0.785 1.571 2.356 3.142 -2.356 -1.571 -0.785 0.0; do
    turn_to "$yaw" || { echo "  FAILED: PX4 did not accept the turn to $yaw rad (see $LOGS/sim.log)"; exit 1; }
    sleep 6
  done
  echo "  ok: sweep done"
  "$PX4_BIN/px4-commander" land
  wait_for 180 "landed and disarmed" disarmed || exit 1
  LATEST="${FLIGHT_RECORDINGS:-$HOME/flight_recordings}/latest"
  wait_for 180 "flight recording saved" bash -c "[ '$LATEST/flight.json' -nt '$LOGS/.fly_start' ] && grep -q '\"status\": \"saved\"' '$LATEST/flight.json'" || exit 1
  echo "  recording: $(readlink -f "$LATEST")"
  echo "  replay it: $HERE/replay_flight.sh latest   (after $HERE/stop_env.sh)"
fi

echo
echo "Everything is up. ROS_DOMAIN_ID=1. Map: /rtabmap/cloud_map, /rtabmap/map, /rtabmap/octomap_full"
echo "PX4 commands: $PX4_BIN/px4-commander takeoff | land"
