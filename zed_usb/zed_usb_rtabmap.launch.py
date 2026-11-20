# A ZED plugged into a laptop without an NVIDIA GPU -> RTAB-Map, in the same RGB-D setup as the drone
# (jetson/zed_rtabmap.launch.py). zed_usb_publisher.py stands in for the ZED SDK wrapper: same topics,
# CPU stereo depth. No IMU, so RTAB-Map computes its own visual odometry. Start with zed_usb/run.sh.
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

HERE = os.path.dirname(os.path.abspath(__file__))
ZED = '/zed/zed_node/'
# Same 0.3 m near-range filter as sim/ and jetson/.
ODOM_SELF_FILTER = '--Vis/MinDepth 0.3'
SELF_FILTER = '--Grid/RangeMin 0.3 ' + ODOM_SELF_FILTER


def generate_launch_description():
    new_map = LaunchConfiguration('new_map')

    args = [
        DeclareLaunchArgument(
            'flip', default_value='false',
            description="'true' if the camera is upside down (images are turned and left/right swapped)"),
        DeclareLaunchArgument(
            'new_map', default_value='true',
            description="'true' starts a fresh map; 'false' continues the one in database_path"),
        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/rtabmap.db', description='RTAB-Map map database'),
        DeclareLaunchArgument(
            'localization', default_value='false',
            description="'true' only localizes in the saved map in database_path, without changing it"),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description="'false' runs RTAB-Map without the ZED, on topics replayed from a bag"),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description="'true' with a bag played with --clock"),
        DeclareLaunchArgument('rtabmap_viz', default_value='true', description='RTAB-Map viewer'),
    ]

    camera = ExecuteProcess(
        cmd=['python3', os.path.join(HERE, 'zed_usb_publisher.py'), '--ros-args',
             '-p', ['flip:=', LaunchConfiguration('flip')]],  # node 'zed_usb', topics under /zed/zed_node/
        # The system OpenCV is built for NumPy 1; a NumPy 2 in ~/.local would break it.
        additional_env={'PYTHONNOUSERSITE': '1'},
        output='screen',
        # The publisher waits out camera problems itself; this restarts it if it ever dies anyway.
        respawn=True, respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration('camera')),
    )

    # zed_camera_link -> left optical frame, as the ZED URDF places it (left lens 6 cm left of center).
    camera_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--y', '0.06', '--roll', '-1.5708', '--yaw', '-1.5708',
                   '--frame-id', 'zed_camera_link', '--child-frame-id', 'zed_left_camera_frame_optical'],
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py'])),
        launch_arguments={
            'rtabmap_args': [PythonExpression(
                ["'--delete_db_on_start' if '", new_map, "' == 'true' else ''"]), ' ' + SELF_FILTER],
            'database_path': LaunchConfiguration('database_path'),
            'localization': LaunchConfiguration('localization'),
            # Handheld camera: when tracking is lost, restart odometry right away instead of waiting
            # forever to re-match the last keyframe; RTAB-Map links the pieces by loop closure.
            'odom_args': ODOM_SELF_FILTER + ' --Odom/ResetCountdown 1',
            'rgb_topic': ZED + 'rgb/color/rect/image',
            'depth_topic': ZED + 'depth/depth_registered',
            'camera_info_topic': ZED + 'rgb/color/rect/camera_info',
            'frame_id': 'zed_camera_link',
            'approx_sync': 'false',  # color and depth carry the same timestamp
            # Reliable: best-effort drops ~1 MB images whole once they exceed the socket buffer (212 KB).
            'qos': '1',
            'visual_odometry': 'true',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
        }.items(),
    )

    return LaunchDescription(args + [camera, camera_tf, rtabmap])
