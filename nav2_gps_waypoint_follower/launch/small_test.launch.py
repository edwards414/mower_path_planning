from subprocess import call
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # 2. 其他指令（每個間隔5秒執行）
    path_record = ExecuteProcess(
        cmd=['ros2', 'run', 'path_record', 'path_record'],
        output='screen'
    )
    map_manage = ExecuteProcess(
        cmd=['ros2', 'run', 'maphub', 'map_manage'],
        output='screen'
    )
    boustrophedon_coverage = ExecuteProcess(
        cmd=['ros2', 'run', 'boustrophedon_coverage', 'boustrophedon_coverage'],
        output='screen'
    )

    return LaunchDescription([
        path_record,
        map_manage,
        boustrophedon_coverage,
    ])
