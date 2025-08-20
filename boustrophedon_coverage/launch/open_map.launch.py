from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    boustrophedon_coverage_dir = get_package_share_directory('boustrophedon_coverage')
    map_path = os.path.join(boustrophedon_coverage_dir, 'map', 'map.yaml')
    rviz_config_path = os.path.join(boustrophedon_coverage_dir, 'config', 'config.rviz')
    
    # 先啟動 rviz2 節點
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # 其他節點
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_path}]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    return LaunchDescription([
        rviz_node,
        TimerAction(
            period=2.0,
            actions=[
                map_server_node,
                lifecycle_manager_node
            ]
        ),
    ])
