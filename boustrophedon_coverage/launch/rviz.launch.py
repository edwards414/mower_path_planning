from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    boustrophedon_coverage_dir = get_package_share_directory('boustrophedon_coverage')
    rviz_config_path = os.path.join(boustrophedon_coverage_dir, 'config', 'config.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path])
    
    return LaunchDescription([
        rviz_node,
    ])

