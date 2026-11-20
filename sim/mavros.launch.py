# MAVROS for PX4 SITL. PX4 runs on Gazebo's clock, which runs slower than real time on a busy laptop,
# so MAVROS has to use sim time as well. Otherwise its time sync with PX4 keeps resetting
# ("TM: Time jump detected") and timestamped messages, like vision poses, reach PX4 at the wrong time.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='udp://:14540@127.0.0.1:14580',
                              description='PX4 SITL onboard MAVLink link'),
        GroupAction([
            # Applies to every node in the MAVROS process, including each plugin's node.
            SetParameter(name='use_sim_time', value=True),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(PathJoinSubstitution(
                    [FindPackageShare('mavros'), 'launch', 'px4.launch'])),
                launch_arguments={'fcu_url': LaunchConfiguration('fcu_url')}.items()),
        ]),
    ])
