#!/usr/bin/env bash
# Puts the drone back upright at its spawn point without restarting the simulation, e.g. after a crash.
# (Gazebo's own reset deletes the drone: PX4 spawns it at startup, it is not part of the world file.)
# Usage: ./reset_drone.sh [path/to/PX4-Autopilot]   (default: ~/PX4-Autopilot)
#        WORLD=walls ./reset_drone.sh                (must match the world launch_sim.sh runs)
# ROS_OVERLAY=path/to/setup.bash also sources an extra workspace, as in bridge.sh.
set -eo pipefail  # not -u: the ROS setup script reads unset variables

PX4_DIR="${1:-$HOME/PX4-Autopilot}"
WORLD="${WORLD:-default_warehouse}"
# Talk to Gazebo over loopback, as PX4 starts it (GZ_IP=127.0.0.1): with the default interface, a
# Wi-Fi drop (suspend, roaming) cuts the link ("Exception sending a multicast message").
export GZ_IP="${GZ_IP:-127.0.0.1}"
MODEL=x500_depth_0
BIN="$PX4_DIR/build/px4_sitl_default/bin"
if [ ! -x "$BIN/px4-commander" ]; then
  echo "No PX4 SITL build in $PX4_DIR (run make px4)" >&2
  exit 1
fi

# 1. Motors off, whether flying, on the floor or upside down.
"$BIN/px4-commander" disarm -f

# 2. Upright at the spawn point: PX4 spawns the drone at the world origin. Dropped from 0.3 m so it
#    never starts inside the floor.
reply=$(gz service -s "/world/$WORLD/set_pose" --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 3000 --req "name: \"$MODEL\" position { x: 0 y: 0 z: 0.3 } orientation { w: 1 }" 2>&1 || true)
if [ "$reply" != "data: true" ]; then
  echo "Could not move $MODEL in world $WORLD (${reply:-no reply}). Is the sim running?" >&2
  echo "If Gazebo's reset removed the drone, restart the sim (make sim)." >&2
  exit 1
fi
sleep 2  # settle on the floor

# 3. Restart PX4's estimator. After a flip it keeps believing it is upside down and fails the
#    preflight checks; a fresh start aligns to the upright drone and the GPS position at once.
"$BIN/px4-ekf2" stop
"$BIN/px4-ekf2" start

# 4. RTAB-Map's visual odometry (if it runs): back to its start pose, which is where the drone is now,
#    so the map continues in the same frame. With odometry:=sim it follows Gazebo by itself.
source /opt/ros/humble/setup.bash
if [ -n "${ROS_OVERLAY:-}" ]; then source "$ROS_OVERLAY"; fi
if timeout 10 ros2 service list --no-daemon 2>/dev/null | grep -qx /rtabmap/rgbd_odometry/reset_odom; then
  timeout 10 ros2 service call /rtabmap/rgbd_odometry/reset_odom std_srvs/srv/Empty >/dev/null &&
    echo "RTAB-Map odometry reset to the start pose"
fi

# 5. Ready to arm again once the preflight checks pass.
for _ in $(seq 1 10); do
  if timeout 3 "$BIN/px4-listener" vehicle_status -n 1 2>/dev/null | grep -q 'pre_flight_checks_pass: True'; then
    echo "Drone reset at the spawn point, ready for takeoff"
    exit 0
  fi
  sleep 1
done
echo "Drone moved to the spawn point, but PX4's preflight checks have not passed yet (see the pxh> shell)" >&2
exit 1
