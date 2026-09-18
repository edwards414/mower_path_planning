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

    # The app's door is the pairing gate (mower_mission/rosbridge_auth_proxy):
    # it listens on `address:port`, checks the pairing headers of every
    # connection against ~/.mower/identity.json and pipes the frames to
    # rosbridge, which only ever listens on loopback. Without an identity
    # file (development, simulation) the gate passes everything through.
    auth_proxy = Node(
        package='mower_mission',
        executable='rosbridge_auth_proxy',
        name='rosbridge_auth_proxy',
        output='screen',
        arguments=[
            '--address', LaunchConfiguration('address'),
            '--port', LaunchConfiguration('port'),
            '--upstream', 'ws://127.0.0.1:9091',
        ],
    )

    # Outbound link to the fleet backend (docs/BACKEND_ARCHITECTURE.md):
    # registers the robot, sends heartbeats and relays phone sessions through
    # the gate above. Idle unless MOWER_BACKEND_URL is set (docker-compose
    # passes it from /opt/mower/.env).
    agent = Node(
        package='mower_mission',
        executable='mower_agent',
        name='mower_agent',
        output='screen',
        arguments=[
            # The gate binds `address` (the WireGuard/LAN IP on a real robot,
            # 127.0.0.1 in dev), so the agent must dial the same address.
            '--gate', ['ws://', LaunchConfiguration('address'), ':', LaunchConfiguration('port')],
            '--rosbridge', 'ws://127.0.0.1:9091',
        ],
    )

    rosbridge_websocket = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[
            rosbridge_config,
            {
                'port': 9091,
                'address': '127.0.0.1',
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
        auth_proxy,
        agent,
        rosbridge_websocket,
        rosapi,
    ])
