from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    blade_teleop_config = os.path.join(
        get_package_share_directory('mower_teleop'),
        'config',
        'blade_teleop.yaml'
    )

    # mower_controller launch
    mower_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mower_controller'),
                'launch',
                'controller_test.launch.py'
            )
        )
    )

    # twist_mux launch
    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_gps_waypoint_follower'),
                'launch',
                'twist_mux.launch.py'
            )
        )
    )

    # teleop keyboard
    teleop_keyboard = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'mower_teleop',
            'teleop_keyboard'
        ],
        output='screen'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
    )

    teleop_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[{
            'publish_stamped_twist': True,
            'require_enable_button': False,
            'enable_turbo_button': -1,
            'axis_linear.x': 1,
            'axis_angular.yaw': 3,
            'scale_linear.x': 0.6,
            'scale_linear_turbo.x': 0.6,
            'scale_angular.yaw': 0.8,
            'scale_angular_turbo.yaw': 0.8,
        }],
        remappings=[('/cmd_vel', '/joy_cmd')],
    )

    blade_teleop_joy = Node(
        package='mower_teleop',
        executable='blade_teleop_joy',
        name='blade_teleop_joy',
        output='screen',
        parameters=[blade_teleop_config],
    )

    return LaunchDescription([
        mower_controller_launch,
        twist_mux_launch,
        joy_node,
        teleop_joy,
        blade_teleop_joy,
        # teleop_keyboard
    ])
