import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    publish_robot_state_publisher = LaunchConfiguration(
        'publish_robot_state_publisher',
        default='true',
    )
    launch_controllers = LaunchConfiguration('launch_controllers', default='false')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='X position to spawn the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Y position to spawn the robot')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock if true')

    declare_publish_robot_state_publisher_cmd = DeclareLaunchArgument(
        'publish_robot_state_publisher',
        default_value='true',
        description='Launch robot_state_publisher before spawning the robot')

    declare_launch_controllers_cmd = DeclareLaunchArgument(
        'launch_controllers',
        default_value='false',
        description='Launch ros2_control controllers after robot spawn completes')

    # ─────────────────────────────────────────────────────────
    # 1. robot_state_publisher：將 URDF 發布到 /robot_description
    # ─────────────────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        condition=IfCondition(publish_robot_state_publisher),
        parameters=[{
            'robot_description': robot_urdf,
            'use_sim_time': use_sim_time,
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

    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mower_description'),
                'launch',
                'controllers.launch.py',
            )
        ),
        condition=IfCondition(launch_controllers),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    launch_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=start_gazebo_ros_spawner_cmd,
            on_exit=[controllers_launch],
        )
    )

    # ─────────────────────────────────────────────────────────
    # 組合 LaunchDescription
    # ─────────────────────────────────────────────────────────
    ld = LaunchDescription()

    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_publish_robot_state_publisher_cmd)
    ld.add_action(declare_launch_controllers_cmd)
    ld.add_action(robot_state_publisher_node)   # ← 新增
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(launch_controllers_after_spawn)

    return ld
