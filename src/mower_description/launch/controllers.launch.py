import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    啟動 ros2_control controllers（Gazebo 模擬版本）

    ⚠️  架構說明：
        [實機/mock 測試用]：
            ros2_control_node（獨立節點）→ 建立 controller_manager → spawner
        [Gazebo 模擬用]（本檔案）：
            GazeboSimROS2ControlPlugin（Gazebo 內部）→ 建立 controller_manager → spawner

        在 Gazebo 模擬中，controller_manager 由 robot.gazebo.xacro 中的
        gz_ros2_control plugin 自動建立，parameters 由 controllers.yaml 提供。
        因此這裡「不需要」ros2_control_node，直接用 spawner 等候 controller_manager 即可。

    啟動順序：
      1. joint_state_broadcaster  → 發布 /joint_states
      2. diff_drive_controller    → 訂閱 /cmd_vel，發布 /odom, /tf
    """

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # ─────────────────────────────────────────────────────────────────
    # 1. joint_state_broadcaster
    #    負責把 hardware state interface 的 joint 角度發布到 /joint_states
    #
    #    --controller-manager-timeout 30：
    #      spawner 會主動等待 controller_manager 上線（最長30秒），
    #      這樣不需要精確計算 TimerAction 的延遲時間
    # ─────────────────────────────────────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',  # 等候 controller_manager 最長 30 秒
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ─────────────────────────────────────────────────────────────────
    # 2. diff_drive_controller
    #    在 joint_state_broadcaster spawn 完成後 2 秒再啟動，
    #    確保 joint_state 已可用
    #
    #    Topic remapping:
    #      /diff_drive_controller/cmd_vel   ← /cmd_vel  (nav2 發送)
    #      /diff_drive_controller/odom      → /odom     (EKF 接收)
    # ─────────────────────────────────────────────────────────────────
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='diff_drive_controller_spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        # NOTE: remappings 在這裡無效！
        # controller 跑在 controller_manager 進程裡，不在 spawner 進程裡。
        # 正確 remap 位置：robot.gazebo.xacro 的 <ros><remapping> 段。
        output='screen',
    )

    # joint_state_broadcaster 先啟動，2 秒後再啟動 diff_drive_controller
    delayed_diff_drive = TimerAction(
        period=2.0,
        actions=[diff_drive_controller_spawner]
    )

    return LaunchDescription([
        declare_use_sim_time,
        joint_state_broadcaster_spawner,
        delayed_diff_drive,
    ])

