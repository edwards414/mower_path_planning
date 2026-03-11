from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # mower_controller launch
    mower_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mower_controller'),
                'launch',
                'controller_test.launch.py'
            )
        )
    )

    # twist_mux launch
    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_gps_waypoint_follower'),
                'launch',
                'twist_mux.launch.py'
            )
        )
    )

    # teleop keyboard
    teleop_keyboard = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'mower_teleop',
            'teleop_keyboard'
        ],
        output='screen'
    )

    return LaunchDescription([
        mower_controller_launch,
        twist_mux_launch,
        # teleop_keyboard
    ])