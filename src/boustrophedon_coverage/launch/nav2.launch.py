from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 聲明 gui 參數，默認為 false
    boustrophedon_coverage_dir = get_package_share_directory('boustrophedon_coverage')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_nav2_dir = get_package_share_directory('tb3_nav2_bringup')
    # turtlebot3_nav2_dir = get_package_share_directory('turtlebot3_navigation2')
    ROS_DISTRO = os.environ['ROS_DISTRO']
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    # 啟動 turtlebot3_gazebo 的 turtlebot3_world.launch.py  

    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    if ROS_DISTRO == 'humble':
        param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(
                get_package_share_directory('turtlebot3_navigation2'),
                'param',
                ROS_DISTRO,
                param_file_name))
    else:
        param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(
                get_package_share_directory('turtlebot3_navigation2'),
                'param',
                param_file_name))

    map_dir = LaunchConfiguration(
            'map',
            default=os.path.join(
                boustrophedon_coverage_dir, 
                'map',
                'map.yaml'))

    rviz_config_dir = os.path.join(
        boustrophedon_coverage_dir,
        'config',
        'config.rviz')
    print(rviz_config_dir)

    # 修正：不要在變量賦值時加逗號，否則會變成 tuple，導致 launch 解析錯誤
    param_declare_launch = DeclareLaunchArgument(
        'params_file',
        default_value=param_dir,
        description='Full path to param file to load'
    )

    map_declare_launch = DeclareLaunchArgument(
        'map',
        default_value=map_dir,
        description='Full path to map file to load'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        ), 
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_nav2_dir, 'launch', 'bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true', 
            'map': map_dir,
            'params_file': param_dir
        }.items()
    )

    rviz_config_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}],  # 布尔值
        output='screen')

    return LaunchDescription([
        map_declare_launch,
        param_declare_launch,
        rviz_config_node,
        # 暂时注释掉仿真和导航
        gazebo_launch,
        nav2_bringup_launch
    ])
