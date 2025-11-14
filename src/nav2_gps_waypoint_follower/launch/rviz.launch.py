from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_gps_waypoint_follower_dir = get_package_share_directory('nav2_gps_waypoint_follower')
    rviz_config_path = os.path.join(nav2_gps_waypoint_follower_dir, 'config', 'config.rviz')
    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )
    return LaunchDescription([
        rviz_node,
    ])

