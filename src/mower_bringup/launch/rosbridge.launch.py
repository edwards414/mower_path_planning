import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rosbridge_config = os.path.join(
        get_package_share_directory('mower_bringup'),
        'config',
        'rosbridge_params.yaml',
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='WebSocket port for rosbridge_websocket',
    )

    rosbridge_websocket = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[
            rosbridge_config,
            {'port': LaunchConfiguration('port')},
        ],
    )

    rosapi = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen',
    )

    return LaunchDescription([
        port_arg,
        rosbridge_websocket,
        rosapi,
    ])
