import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Get URDF via xacro
    robot_description_path = os.path.join(
        get_package_share_directory('mower_robot_description'),
        'urdf',
        'mower_robot.xacro')
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    test_controller = os.path.join(
        get_package_share_directory('mower_controller'),
        'controllers',
        'diff_drive_controller.yaml'
        )

    return LaunchDescription([
      Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
        ),
      Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, test_controller],
        output={
          'stdout': 'screen',
          'stderr': 'screen',
          }
        ),
        # 启动 joint_state_broadcaster
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['joint_state_broadcaster'],
        #     output='screen',
        # ),
        # 启动 diff_controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_controller'],
            output='screen',
        ),

    ])