import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ─────────────────────────────────────────────────────────
    # Xacro → 展開為 URDF 字串（在 launch 時處理，不交給 Gazebo）
    # ─────────────────────────────────────────────────────────
    xacro_file = os.path.join(
        get_package_share_directory('mower_description'),
        'mower_robot',
        'robot.xacro'
    )
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='X position to spawn the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Y position to spawn the robot')

    # ─────────────────────────────────────────────────────────
    # 1. robot_state_publisher：將 URDF 發布到 /robot_description
    # ─────────────────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_urdf,
            'use_sim_time': True,
        }]
    )

    # ─────────────────────────────────────────────────────────
    # 2. Gazebo Spawner：從 /robot_description topic 讀取 URDF
    #    （不再使用 -file，改用 -topic）
    # ─────────────────────────────────────────────────────────
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mower_robot',
            '-topic', 'robot_description',  # ← 從 topic 讀 URDF，不直接傳檔案
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.10',
        ],
        output='screen',
    )

    # ─────────────────────────────────────────────────────────
    # 3. ros_gz_bridge：橋接 Gazebo ↔ ROS2 topics
    # ─────────────────────────────────────────────────────────
    bridge_params = os.path.join(
        get_package_share_directory('mower_description'),
        'config',
        'mower_bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # ─────────────────────────────────────────────────────────
    # 組合 LaunchDescription
    # ─────────────────────────────────────────────────────────
    ld = LaunchDescription()

    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(robot_state_publisher_node)   # ← 新增
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)

    return ld
