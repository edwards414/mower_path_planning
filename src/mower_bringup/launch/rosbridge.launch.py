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
    address_arg = DeclareLaunchArgument(
        'address',
        default_value='127.0.0.1',
        description=(
            'rosbridge listen address; expose 0.0.0.0 only on a trusted LAN'
        ),
    )

    rosbridge_websocket = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[
            rosbridge_config,
            {
                'port': LaunchConfiguration('port'),
                'address': LaunchConfiguration('address'),
            },
        ],
    )

    rosapi = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen',
        # Apply the same topics/services/params allowlists as websocket so
        # rosapi cannot bypass the bridge policy through parameter services.
        parameters=[rosbridge_config],
    )

    return LaunchDescription([
        port_arg,
        address_arg,
        rosbridge_websocket,
        rosapi,
    ])
