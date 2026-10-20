# RTAB-Map RGB-D SLAM on the bridged /uav topics (start launch_sim.sh and bridge.sh first).
# RTAB-Map computes its own visual odometry; /uav/odom is ground truth, kept only for evaluation.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Camera mount: OakD-Lite pose in x500_depth (.12 .03 .242) + sensor pose in OakD-Lite (.01233 -.03 .01878).
CAMERA_XYZ = ['0.13233', '0.0', '0.26078']
# Ignore anything within 0.3 m of the camera, for the map and for odometry features, so the drone's
# own parts never count as obstacles (same values as jetson/zed_rtabmap.launch.py).
ODOM_SELF_FILTER = '--Vis/MinDepth 0.3'
SELF_FILTER = '--Grid/RangeMin 0.3 ' + ODOM_SELF_FILTER


def generate_launch_description():
    # base_link -> camera_optical_frame (z forward, x right, y down), the frame the bridged images carry.
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', CAMERA_XYZ[0], '--y', CAMERA_XYZ[1], '--z', CAMERA_XYZ[2],
                   '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_optical_frame'],
        parameters=[{'use_sim_time': True}],
    )

    # RGB and depth share pose, resolution and FOV (sim/models/OakD-Lite), so depth is already registered.
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start ' + SELF_FILTER,
            'odom_args': ODOM_SELF_FILTER,
            'rgb_topic': '/uav/rgb/image',
            'depth_topic': '/uav/depth/image',
            'camera_info_topic': '/uav/rgb/camera_info',
            'frame_id': 'base_link',
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.02',
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription([camera_tf, rtabmap])
