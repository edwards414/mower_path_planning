from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    mower_bringup_dir = get_package_share_directory('mower_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_localization = LaunchConfiguration('enable_localization')
    enable_navigation = LaunchConfiguration('enable_navigation')
    enable_apriltag_docking = LaunchConfiguration('enable_apriltag_docking')
    nav_autostart = LaunchConfiguration('nav_autostart')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true',
    )
    declare_enable_localization = DeclareLaunchArgument(
        'enable_localization',
        default_value='true',
        description='Launch dual EKF and navsat_transform',
    )
    declare_enable_navigation = DeclareLaunchArgument(
        'enable_navigation',
        default_value='true',
        description='Launch the Nav2 stack',
    )
    declare_enable_apriltag_docking = DeclareLaunchArgument(
        'enable_apriltag_docking',
        default_value='false',
        description='Launch rear-camera AprilTag docking detector',
    )
    declare_nav_autostart = DeclareLaunchArgument(
        'nav_autostart',
        default_value='false',
        description='Automatically activate Nav2 lifecycle nodes',
    )
    declare_nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(
            mower_bringup_dir,
            'config',
            'nav2_no_map_params.yaml',
        ),
        description='Full path to the Nav2 parameters file',
    )

    robot_description_path = os.path.join(
        get_package_share_directory('mower_description'),
        'mower_robot',
        'real_robot.xacro'
    )
    robot_description = {
        'robot_description': xacro.process_file(robot_description_path).toxml()
    }

    blade_teleop_config = os.path.join(
        get_package_share_directory('mower_teleop'),
        'config',
        'blade_teleop.yaml'
    )

    mower_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mower_controller'),
                'launch',
                'controller_test.launch.py'
            )
        ),
        launch_arguments={
            'publish_robot_state_publisher': 'false',
        }.items(),
    )

    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mower_bringup_dir,
                'launch',
                'twist_mux.launch.py'
            )
        )
    )

    robot_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mower_bringup_dir,
                'launch',
                'dual_ekf_navsat.launch.py',
            )
        ),
        condition=IfCondition(enable_localization),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mower_bringup_dir,
                'launch',
                'navigation.launch.py',
            )
        ),
        condition=IfCondition(enable_navigation),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': nav_autostart,
        }.items(),
    )

    apriltag_docking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mower_bringup_dir,
                'launch',
                'apriltag_docking.launch.py',
            )
        ),
        condition=IfCondition(enable_apriltag_docking),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    teleop_keyboard = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'mower_teleop',
            'teleop_keyboard'
        ],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    wit_ros2_imu_node = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu',
        output='screen',
        remappings=[('imu/data_raw', 'imu/data')],
    )

    imu_z_flip_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_z_flip_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '3.14159',
            '--frame-id', 'imu_link',
            '--child-frame-id', 'imu_link_corrected',
        ],
        output='screen',
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
        declare_use_sim_time,
        declare_enable_localization,
        declare_enable_navigation,
        declare_enable_apriltag_docking,
        declare_nav_autostart,
        declare_nav2_params_file,
        robot_state_publisher,
        wit_ros2_imu_node,
        imu_z_flip_tf,
        mower_controller_launch,
        twist_mux_launch,
        robot_localization_launch,
        navigation_launch,
        apriltag_docking_launch,
        joy_node,
        teleop_joy,
        blade_teleop_joy,
        # teleop_keyboard
    ])
