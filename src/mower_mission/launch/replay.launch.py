import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    qos_overrides = os.path.join(
        get_package_share_directory('mower_mission'),
        'config', 'replay_qos.yaml',
    )

    bag = LaunchConfiguration('bag')
    declare_bag = DeclareLaunchArgument(
        'bag',
        default_value='bags/real_scene',
        description='Path to the recorded ros2 bag to replay (looped)',
    )

    # rosbridge + rosapi so the app can connect to the replayed scene.
    rosbridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mower_bringup'),
                'launch', 'rosbridge.launch.py',
            )
        )
    )

    # Loop the recorded REAL data. QoS overrides keep latched layers
    # transient_local so a late-joining app still gets them.
    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag, '--loop',
            '--qos-profile-overrides-path', qos_overrides,
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_bag,
        rosbridge_launch,
        bag_play,
    ])
