#!/usr/bin/env bash
# Terminal 2: Gazebo -> ROS 2 bridge. Requires ros-humble-ros-gzharmonic
# (the default ros-humble-ros-gz-bridge is built for Fortress: topics appear but no data flows).
# Usage: ./bridge.sh                 WORLD=walls ./bridge.sh   (must match the world launch_sim.sh runs)
# ROS_OVERLAY=path/to/install/setup.bash also sources a workspace with a Harmonic ros_gz_bridge built from source.
set -eo pipefail
source /opt/ros/humble/setup.bash
if [ -n "${ROS_OVERLAY:-}" ]; then source "$ROS_OVERLAY"; fi
# Shared-memory transport for the camera images (see fastdds.xml); keeps a profile you set yourself.
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$(cd "$(dirname "$0")" && pwd)/fastdds.xml}"

WORLD="${WORLD:-default_warehouse}"
# Talk to Gazebo over loopback, as PX4 starts it (GZ_IP=127.0.0.1): with the default interface, a
# Wi-Fi drop (suspend, roaming) cuts the link ("Exception sending a multicast message").
export GZ_IP="${GZ_IP:-127.0.0.1}"
MODEL=x500_depth_0
CAM=/world/$WORLD/model/$MODEL/link/camera_link/sensor/IMX214
IMU=/world/$WORLD/model/$MODEL/link/base_link/sensor/imu_sensor/imu
CONFIG="${XDG_RUNTIME_DIR:-/tmp}/proj1_bridge_$WORLD.yaml"

# Every topic is lazy: converted only while something subscribes to it. The bridge converts on a single
# thread, and converting the unused 640x480 /uav/points cloud (~10 MB a message) took most of a core.
# Lazy is only honored per topic in a config file (lazy:=true on the command line is ignored by this
# bridge build).
bridge() {  # gz_topic ros_topic ros_type gz_type
  printf -- '- gz_topic_name: "%s"\n  ros_topic_name: "%s"\n  ros_type_name: "%s"\n  gz_type_name: "%s"\n  direction: GZ_TO_ROS\n  lazy: true\n' "$@"
}
{
  bridge /depth_camera /uav/depth/image sensor_msgs/msg/Image gz.msgs.Image
  bridge /depth_camera/points /uav/points sensor_msgs/msg/PointCloud2 gz.msgs.PointCloudPacked
  bridge /camera_info /uav/depth/camera_info sensor_msgs/msg/CameraInfo gz.msgs.CameraInfo
  bridge "$CAM/image" /uav/rgb/image sensor_msgs/msg/Image gz.msgs.Image
  bridge "$CAM/camera_info" /uav/rgb/camera_info sensor_msgs/msg/CameraInfo gz.msgs.CameraInfo
  bridge /lidar_points/points /uav/lidar/points sensor_msgs/msg/PointCloud2 gz.msgs.PointCloudPacked
  bridge "/model/$MODEL/odometry_with_covariance" /uav/odom nav_msgs/msg/Odometry gz.msgs.OdometryWithCovariance
  bridge "$IMU" /uav/imu sensor_msgs/msg/Imu gz.msgs.IMU
  bridge "/world/$WORLD/clock" /clock rosgraph_msgs/msg/Clock gz.msgs.Clock
} > "$CONFIG"

ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$CONFIG"
