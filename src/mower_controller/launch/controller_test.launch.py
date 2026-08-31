import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    publish_robot_state_publisher = LaunchConfiguration(
        'publish_robot_state_publisher'
    )

    # Get URDF via xacro
    robot_description_path = os.path.join(
        get_package_share_directory('mower_description'),
        'mower_robot',
        'real_robot.xacro')
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    test_controller = os.path.join(
        get_package_share_directory('mower_controller'),
        'controllers',
        'diff_drive_controller.yaml'
        )

    return LaunchDescription([
      DeclareLaunchArgument(
        'publish_robot_state_publisher',
        default_value='true',
        description='Launch robot_state_publisher together with ros2_control',
      ),
      Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        condition=IfCondition(publish_robot_state_publisher),
        parameters=[robot_description],
        output='screen'
        ),
      Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, test_controller],
        # Controllers are loaded into this process, so their private topics
        # must be remapped here (remapping a spawner has no effect).
        remappings=[
          (
            '/diff_controller/cmd_vel',
            '/drivetrain_guarded_cmd_vel',
          ),
          ('/diff_controller/odom', '/odom'),
        ],
        output={
          'stdout': 'screen',
          'stderr': 'screen',
          }
        ),
        # 启动 joint_state_broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        # 启动 diff_controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_controller'],
            output='screen'
        ),

    ])
