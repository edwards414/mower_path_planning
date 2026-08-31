# Copyright 2024 fxrbindi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _require_sim_time(context):
    value = LaunchConfiguration('use_sim_time').perform(context).strip().lower()
    if value not in {'1', 'true', 'yes', 'on'}:
        raise RuntimeError(
            'system_test.launch.py requires use_sim_time:=true because it '
            'consumes Gazebo-stamped sensor and transform data'
        )
    launch_sim = LaunchConfiguration('launch_sim').perform(context)
    if launch_sim.strip().lower() not in {'1', 'true', 'yes', 'on'}:
        raise RuntimeError(
            'system_test.launch.py requires launch_sim:=true so its Nav2 '
            'output cannot accidentally run beside a standalone simulator '
            'that publishes directly to /cmd_vel'
        )
    return []


def generate_launch_description():
    launch_sim = LaunchConfiguration('launch_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    coverage_backend = LaunchConfiguration('coverage_backend')
    zigzag_angle_deg = LaunchConfiguration('zigzag_angle_deg')

    declare_launch_sim = DeclareLaunchArgument(
        'launch_sim',
        default_value='true',
        description='Must remain true; this launch owns the complete graph',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true',
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz when launch_sim is true',
    )
    declare_coverage_backend = DeclareLaunchArgument(
        'coverage_backend',
        default_value='rust',
        description='Coverage algorithm backend: "python" or "rust"',
    )
    declare_zigzag_angle_deg = DeclareLaunchArgument(
        'zigzag_angle_deg',
        default_value='0.0',
        description='Zigzag scan angle in degrees, from 0 to 180',
    )

    mower_bringup_dir = get_package_share_directory('mower_bringup')

    rosbridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mower_bringup_dir, 'launch', 'rosbridge.launch.py')
        ),
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mower_bringup_dir,
                'launch',
                'sim_with_nav.launch.py',
            )
        ),
        condition=IfCondition(launch_sim),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
            # This launch owns a coordinator-gated twist_mux, so Nav2 must not
            # publish directly to the simulated drivetrain.
            'cmd_vel_output_topic': '/nav_cmd_vel',
        }.items(),
    )

    path_record = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'mower_mission', 'path_record_node',
            '--ros-args',
            '-p', ['use_sim_time:=', use_sim_time],
        ],
        output='screen'
    )
    map_manage = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'mower_mission', 'map_manage_node',
            '--ros-args', '-p', ['use_sim_time:=', use_sim_time],
        ],
        output='screen'
    )
    boustrophedon_coverage = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'mower_mission', 'coverage_node',
            '--ros-args',
            '-p', ['coverage_backend:=', coverage_backend],
            '-p', ['zigzag_angle_deg:=', zigzag_angle_deg],
            '-p', 'allow_backend_fallback:=false',
            '-p', ['use_sim_time:=', use_sim_time],
        ],
        output='screen'
    )
    nav_action_server = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'mower_mission', 'nav_action_server',
            '--ros-args',
            '-p', ['use_sim_time:=', use_sim_time],
            '-p', 'require_navigation_health:=false',
        ],
        output='screen'
    )
    flutter_adapter = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'mower_mission', 'flutter_adapter_node',
            '--ros-args', '-p', ['use_sim_time:=', use_sim_time],
        ],
        output='screen'
    )
    # Robot liveness heartbeat -> /robot/online (the app's "online"). Needs
    # /odom flowing (i.e. the sim running) to report online=true.
    heartbeat = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'mower_mission', 'heartbeat_node',
            '--ros-args',
            # Liveness expiry must keep using wall time if Gazebo pauses.
            '-p', 'use_sim_time:=false',
            '-p', 'source_topic:=/odom',
        ],
        output='screen'
    )
    # This readiness-aware sequencer waits for each service and the latched
    # map products. Fixed one-shot CLI timers silently lost steps on slow CI or
    # first boot when a service had not appeared yet.
    auto_coverage = Node(
        package='mower_mission',
        executable='auto_coverage_node',
        name='auto_coverage',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    auto_coverage_required = RegisterEventHandler(
        OnProcessExit(
            target_action=auto_coverage,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason='auto_coverage exited before system shutdown'
                    )
                )
            ],
        )
    )

    twist_mux_config = os.path.join(
        mower_bringup_dir, 'config', 'twist_mux_topics.yaml'
    )
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/cmd_vel_guard_input')],
    )
    manual_velocity_guard = Node(
        package='mower_bringup',
        executable='velocity_command_guard',
        name='manual_velocity_guard',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'require_command_session': True,
        }],
        remappings=[
            ('cmd_vel_in', '/app_joy_cmd'),
            ('cmd_vel_out', '/joy_cmd'),
            ('command_clock', '/manual_command_clock'),
        ],
    )
    velocity_guard = Node(
        package='mower_bringup',
        executable='velocity_command_guard',
        name='velocity_command_guard',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel_in', '/cmd_vel_guard_input'),
            ('cmd_vel_out', '/cmd_vel'),
        ],
    )

    timer_path_record = TimerAction(
        period=5.0,
        actions=[path_record]
    )

    timer_map_manage = TimerAction(
        period=10.0,
        actions=[map_manage]
    )

    timer_boustrophedon_coverage = TimerAction(
        period=15.0,
        actions=[boustrophedon_coverage]
    )

    timer_flutter_adapter = TimerAction(
        period=19.0,
        actions=[flutter_adapter]
    )

    timer_heartbeat = TimerAction(
        period=19.0,
        actions=[heartbeat]
    )

    return LaunchDescription([
        declare_launch_sim,
        declare_use_sim_time,
        OpaqueFunction(function=_require_sim_time),
        declare_use_rviz,
        declare_coverage_backend,
        declare_zigzag_angle_deg,
        rosbridge_launch,
        manual_velocity_guard,
        twist_mux,
        velocity_guard,
        sim_launch,
        # Start the central lease/navigation coordinator before any mutation
        # service or the auto-coverage readiness sequencer.
        nav_action_server,
        timer_path_record,
        timer_map_manage,
        timer_boustrophedon_coverage,
        timer_flutter_adapter,
        timer_heartbeat,
        auto_coverage_required,
        auto_coverage,
    ])
