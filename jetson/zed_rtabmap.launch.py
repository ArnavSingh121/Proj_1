# On the drone's Jetson: ZED camera -> RTAB-Map RGB-D SLAM, plus MAVROS to the PX4 flight controller.
# Start it with jetson/run.sh (`make jetson`). The simulation counterpart is sim/rtabmap.launch.py.
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource, PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare

HERE = os.path.dirname(os.path.abspath(__file__))
ZED = '/zed/zed_node/'  # topic root of zed_wrapper v5 (camera_name 'zed', node_name 'zed_node')
# Ignore anything within 0.3 m of the camera, for the map and for odometry features, so the drone's
# own props and legs never count as obstacles (same values as sim/rtabmap.launch.py).
ODOM_SELF_FILTER = '--Vis/MinDepth 0.3'
SELF_FILTER = '--Grid/RangeMin 0.3 ' + ODOM_SELF_FILTER


def generate_launch_description():
    camera_model = LaunchConfiguration('camera_model')
    use_zed_odometry = LaunchConfiguration('use_zed_odometry')
    new_map = LaunchConfiguration('new_map')

    args = [
        DeclareLaunchArgument(
            'camera_model',
            description="[REQUIRED] ZED model: 'zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', ..."),
        DeclareLaunchArgument(
            'use_zed_odometry', default_value='true',
            description="'true': ZED visual-inertial odometry (lighter on the Jetson); "
                        "'false': RTAB-Map's own visual odometry, as in the simulation"),
        DeclareLaunchArgument(
            'new_map', default_value='true',
            description="'true' starts a fresh map; 'false' continues the map in database_path"),
        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/rtabmap.db',
            description='RTAB-Map map database'),
        DeclareLaunchArgument(
            'rtabmap_viz', default_value='false',
            description='RTAB-Map viewer; off by default because the Jetson usually runs headless'),
        DeclareLaunchArgument(
            'mavros', default_value='true', description='Start MAVROS to talk to PX4'),
        DeclareLaunchArgument(
            'fcu_url', default_value='/dev/ttyTHS1:921600',
            description='PX4 link: Jetson UART on TELEM2 by default; /dev/ttyACM0:57600 for USB'),
    ]

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])),
        launch_arguments={
            'camera_model': camera_model,
            'ros_params_override_path': os.path.join(HERE, 'zed_params.yaml'),
            # odom -> zed_camera_link comes from ZED tracking only when RTAB-Map isn't computing odometry.
            'publish_tf': use_zed_odometry,
            'publish_map_tf': 'false',  # RTAB-Map publishes map -> odom
            'publish_imu_tf': 'true',
        }.items(),
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py'])),
        launch_arguments={
            'rtabmap_args': [PythonExpression(
                ["'--delete_db_on_start' if '", new_map, "' == 'true' else ''"]), ' ' + SELF_FILTER],
            'odom_args': ODOM_SELF_FILTER,
            'database_path': LaunchConfiguration('database_path'),
            # Depth is registered to the rectified RGB image by the ZED SDK.
            'rgb_topic': ZED + 'rgb/color/rect/image',
            'depth_topic': ZED + 'depth/depth_registered',
            'camera_info_topic': ZED + 'rgb/color/rect/camera_info',
            'imu_topic': ZED + 'imu/data',
            'wait_imu_to_init': 'true',
            'frame_id': 'zed_camera_link',
            'approx_sync': 'false',  # RGB and depth come from the same ZED grab
            # Reliable: best-effort drops ~1 MB images whole once they exceed the socket buffer (212 KB).
            'qos': '1',
            'visual_odometry': PythonExpression(
                ["'false' if '", use_zed_odometry, "' == 'true' else 'true'"]),
            'odom_topic': PythonExpression(
                ["'", ZED, "odom' if '", use_zed_odometry, "' == 'true' else 'odom'"]),
            'use_sim_time': 'false',
            'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
        }.items(),
    )

    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('mavros'), 'launch', 'px4.launch'])),
        launch_arguments={'fcu_url': LaunchConfiguration('fcu_url')}.items(),
        condition=IfCondition(LaunchConfiguration('mavros')),
    )

    return LaunchDescription(args + [zed, rtabmap, mavros])
