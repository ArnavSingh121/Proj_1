# RTAB-Map RGB-D SLAM on the bridged /uav topics (start launch_sim.sh and bridge.sh first).
# Inputs (all simulated): RGB + depth camera always; odometry from RTAB-Map's own visual odometry
# (odometry:=visual, default) or from Gazebo (odometry:=sim); the 3D lidar optionally (lidar:=true).
# odom_relay.py checks Gazebo's odometry in both modes: invalid samples are dropped, never passed on.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
# Camera mount: OakD-Lite pose in x500_depth (.12 .03 .242) + sensor pose in OakD-Lite (.01233 -.03 .01878).
CAMERA_XYZ = ['0.13233', '0.0', '0.26078']
# Lidar mount: lidar_3d pose in x500_depth (0 0 .31) + sensor pose in lidar_3d (0 0 .036). The points carry
# frame 'lidar_link' (gz_frame_id) but are measured from the sensor, so the TF includes the sensor offset.
LIDAR_XYZ = ['0.0', '0.0', '0.346']
LIDAR_TOPIC = '/uav/lidar/points'
# Ignore anything within 0.3 m of the camera, for the map and for odometry features, so the drone's
# own parts never count as obstacles (same values as jetson/zed_rtabmap.launch.py).
ODOM_SELF_FILTER = '--Vis/MinDepth 0.3'
SELF_FILTER = '--Grid/RangeMin 0.3 ' + ODOM_SELF_FILTER
# When visual odometry loses track (blank wall, depth dropout), restart it right away instead of waiting
# forever to re-match the last keyframe; RTAB-Map links the pieces back together by loop closure.
ODOM_RECOVERY = '--Odom/ResetCountdown 1'


def rtabmap_and_odometry(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)
    sim_odometry = arg('odometry') == 'sim'
    lidar = arg('lidar') == 'true'
    if arg('odometry') not in ('visual', 'sim'):
        raise ValueError(f"odometry:={arg('odometry')}: use 'visual' or 'sim'")

    rtabmap_args = SELF_FILTER
    rtabmap_args += ' --Rtabmap/DetectionRate ' + arg('detection_rate')
    if arg('new_map') == 'true':
        rtabmap_args = '--delete_db_on_start ' + rtabmap_args
    if lidar:
        rtabmap_args += ' --Grid/Sensor 2'  # occupancy grid from both lidar and depth camera

    launch_arguments = {
        'rtabmap_args': rtabmap_args,
        'odom_args': ODOM_SELF_FILTER + ' ' + ODOM_RECOVERY,
        'database_path': arg('database_path'),
        'rgb_topic': '/uav/rgb/image',
        'depth_topic': '/uav/depth/image',
        'camera_info_topic': '/uav/rgb/camera_info',
        'frame_id': 'base_link',
        'approx_sync': 'true',
        'approx_sync_max_interval': '0.02',
        'use_sim_time': 'true',
        'rtabmap_viz': arg('rtabmap_viz'),
        'visual_odometry': 'false' if sim_odometry else 'true',
        # visual: RTAB-Map's own odometry on /rtabmap/odom; sim: Gazebo's, checked by odom_relay.py
        'odom_topic': '/uav/odom_checked' if sim_odometry else 'odom',
    }
    if lidar:
        launch_arguments.update({'subscribe_scan_cloud': 'true', 'scan_cloud_topic': LIDAR_TOPIC})

    # RGB and depth share pose, resolution and FOV (sim/models/OakD-Lite), so depth is already registered.
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')),
        launch_arguments=launch_arguments.items(),
    )

    odom_relay = ExecuteProcess(
        cmd=['python3', os.path.join(HERE, 'odom_relay.py'), '--ros-args',
             '-p', 'use_sim_time:=true', '-p', f'publish_tf:={str(sim_odometry).lower()}'],
        output='screen',
        respawn=True, respawn_delay=1.0,
    )

    actions = [rtabmap, odom_relay]
    if lidar:
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['--x', LIDAR_XYZ[0], '--y', LIDAR_XYZ[1], '--z', LIDAR_XYZ[2],
                       '--frame-id', 'base_link', '--child-frame-id', 'lidar_link'],
            parameters=[{'use_sim_time': True}],
        ))
    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'odometry', default_value='visual',
            description="'visual': RTAB-Map's own visual odometry (the estimate under test); "
                        "'sim': Gazebo's simulated odometry from /uav/odom (ground truth)"),
        DeclareLaunchArgument(
            'lidar', default_value='false',
            description="'true' also feeds the simulated 3D lidar (/uav/lidar/points) to RTAB-Map"),
        DeclareLaunchArgument(
            'new_map', default_value='true',
            description="'true' starts a fresh map; 'false' reloads and continues the map in database_path"),
        DeclareLaunchArgument(
            'detection_rate', default_value='2',
            description="Map update rate in Hz (odometry runs at the camera rate regardless); RTAB-Map's own "
                        "default is 1. '0' adds every camera frame: the database grows ~3.6 GB in 20 min "
                        "and rtabmap_viz stalls"),
        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/rtabmap.db', description='RTAB-Map map database'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true', description='RTAB-Map viewer'),
    ]

    # base_link -> camera_optical_frame (z forward, x right, y down), the frame the bridged images carry.
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', CAMERA_XYZ[0], '--y', CAMERA_XYZ[1], '--z', CAMERA_XYZ[2],
                   '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_optical_frame'],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription(args + [camera_tf, OpaqueFunction(function=rtabmap_and_odometry)])
