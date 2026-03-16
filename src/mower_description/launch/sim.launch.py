import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # ─────────────────────────────────────────────────────────────
    # 路徑設定
    # ─────────────────────────────────────────────────────────────
    mower_desc_share = get_package_share_directory('mower_description')
    nav_rviz_package_share = get_package_share_directory('nav2_gps_waypoint_follower')

    # rviz_config_file = os.path.join(mower_desc_share, 'rviz', 'display.rviz')
    rviz_config_file = os.path.join(nav_rviz_package_share, 'config', 'config.rviz')

    # ─────────────────────────────────────────────────────────────
    # 1. 啟動 Gazebo + spawn + controllers（完整仿真環境）
    #    gazebo.launch.py 內部已包含：
    #      - GZ_SIM_RESOURCE_PATH 設定
    #      - gz_sim（Gazebo Harmonic）
    #      - spawn_mowerbot（含 robot_state_publisher + ros_gz_bridge）
    #      - controllers（diff_drive + joint_state_broadcaster）
    # ─────────────────────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mower_desc_share, 'launch', 'gazebo.launch.py')
        )
    )

    # ─────────────────────────────────────────────────────────────
    # 2. 啟動 RViz2（延遲 5 秒，等待 robot_state_publisher 就緒）
    #    注意：不使用 display.launch.py，因為它會重複啟動
    #    robot_state_publisher，且其 robot.urdf 含有非標準 XML 標籤
    #    （<joint_properties>）會導致解析失敗。
    #    gazebo.launch.py 内的 spawn_mowerbot 已啟動 robot_state_publisher。
    # ─────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    delayed_rviz = TimerAction(
        period=5.0,   # 等 Gazebo(3s) + spawn(~2s) = 5s
        actions=[rviz_node]
    )

    # ─────────────────────────────────────────────────────────────
    # 組合
    # ─────────────────────────────────────────────────────────────
    return LaunchDescription([
        gazebo_launch,
        delayed_rviz,
    ])
