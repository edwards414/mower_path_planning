import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # ─────────────────────────────────────────────────────────────
    # 路徑設定
    # ─────────────────────────────────────────────────────────────
    mower_desc_share = get_package_share_directory('mower_description')
    mower_sim_share = get_package_share_directory('mower_sim')
    world_file        = os.path.join(mower_sim_share, 'worlds', 'mower_world.world')

    # ─────────────────────────────────────────────────────────────
    # GZ_SIM_RESOURCE_PATH：
    #   Gazebo 把 URDF 裡的 package://mower_description/...
    #   轉成 model://mower_description/... 後，
    #   會在這個路徑底下找 mower_description/ 資料夾
    #
    #   install/mower_description/share/
    #   └── mower_description/          ← Gazebo 找 "mower_description" 模型的地方
    #       └── mower_robot/
    #           └── assets/
    #               └── blade_hub.stl   ← 實際 STL 位置
    # ─────────────────────────────────────────────────────────────
    gz_resource_paths = [
        os.path.dirname(mower_desc_share),   # .../share （parent）
    ]
    existing_gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH')
    if existing_gz_resource_path:
        gz_resource_paths.append(existing_gz_resource_path)

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(gz_resource_paths),
    )

    # ─────────────────────────────────────────────────────────────
    # 1. 啟動 Gazebo Harmonic
    # ─────────────────────────────────────────────────────────────
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-v 4 -r {world_file}'
        }.items()
    )

    # ─────────────────────────────────────────────────────────────
    # 2. Spawn 機器人（延遲 3 秒，等 Gazebo 完全啟動）
    # ─────────────────────────────────────────────────────────────
    spawn_mowerbot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mower_desc_share,
                'launch',
                'spawn_mowerbot.launch.py'
            )
        ),
        launch_arguments={
            'x_pose': '0.0',
            'y_pose': '0.0',
        }.items()
    )

    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_mowerbot_launch]
    )

    # ─────────────────────────────────────────────────────────────
    # 3. 啟動 ros2_control controllers
    #    延遲 6 秒：等待 Gazebo 啟動(~3s) + spawn(~2s) + controller_manager 初始化(~1s)
    #    controller_manager 由 gz_ros2_control plugin 在 Gazebo 內啟動，
    #    必須在 spawn 完成後才能接受 spawner 請求。
    # ─────────────────────────────────────────────────────────────
    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mower_desc_share,
                'launch',
                'controllers.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
        }.items()
    )

    delayed_controllers = TimerAction(
        period=6.0,   # Gazebo(3s) + spawn(2s) + controller_manager init(1s)
        actions=[controllers_launch]
    )

    # ─────────────────────────────────────────────────────────────
    # 組合（resource path 必須在 gz_sim 之前 set）
    # ─────────────────────────────────────────────────────────────
    return LaunchDescription([
        set_gz_resource_path,
        gz_sim_launch,
        delayed_spawn,
        delayed_controllers,   # ← 新增：啟動 diff_drive_controller + joint_state_broadcaster
    ])
