#!/usr/bin/env bash
# Terminal 2: Gazebo -> ROS 2 bridge. Requires ros-humble-ros-gzharmonic
# (the default ros-humble-ros-gz-bridge is built for Fortress: topics appear but no data flows).
set -eo pipefail
source /opt/ros/humble/setup.bash

WORLD=default_warehouse
MODEL=x500_depth_0
CAM=/world/$WORLD/model/$MODEL/link/camera_link/sensor/IMX214
IMU=/world/$WORLD/model/$MODEL/link/base_link/sensor/imu_sensor/imu

ros2 run ros_gz_bridge parameter_bridge \
  /depth_camera@sensor_msgs/msg/Image[gz.msgs.Image \
  /depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
  /camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  $CAM/image@sensor_msgs/msg/Image[gz.msgs.Image \
  $CAM/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /model/$MODEL/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance \
  $IMU@sensor_msgs/msg/Imu[gz.msgs.IMU \
  /world/$WORLD/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  --ros-args \
  -r /depth_camera:=/uav/depth/image \
  -r /depth_camera/points:=/uav/points \
  -r /camera_info:=/uav/depth/camera_info \
  -r $CAM/image:=/uav/rgb/image \
  -r $CAM/camera_info:=/uav/rgb/camera_info \
  -r /model/$MODEL/odometry_with_covariance:=/uav/odom \
  -r $IMU:=/uav/imu \
  -r /world/$WORLD/clock:=/clock
