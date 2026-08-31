import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


SAFE_REPLAY_TOPICS = [
    '/adapter/robot_pose',
    '/adapter/map_datum',
    '/adapter/coverage_settings',
    '/adapter/zone_summaries',
    '/adapter/map_layers/map_grid',
    '/adapter/map_layers/free_space_inflated',
    '/adapter/map_layers/risk_map_inflated',
    '/adapter/map_layers/chennal_map_inflated',
    '/adapter/marker_layers/coverage_path',
    '/adapter/marker_layers/zones',
    '/adapter/marker_layers/channels',
    '/adapter/marker_layers/connectors',
    '/adapter/marker_layers/invalid_segments',
    '/adapter/marker_layers/risk_zones',
    '/robot/online',
    '/battery_state',
]


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
            # Never replay recorded /cmd_vel, TF, GPS, action status or other
            # engineering topics into a live robot domain. This launch is an
            # app-scene viewer even if the source bag contains actuator data.
            '--topics', *SAFE_REPLAY_TOPICS,
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_bag,
        rosbridge_launch,
        bag_play,
    ])
