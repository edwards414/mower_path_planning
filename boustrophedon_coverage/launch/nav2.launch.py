from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 聲明 gui 參數，默認為 false
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo_custom')
    turtlebot3_nav2_dir = get_package_share_directory('turtlebot3_navigation2')


    # 啟動 turtlebot3_gazebo 的 turtlebot3_world.launch.py
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        ]), 
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot3_nav2_dir, 'launch', 'navigation2.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        gazebo_launch,   # 再啟動節點
        nav2_launch
    ])
