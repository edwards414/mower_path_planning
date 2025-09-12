import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import AppendEnvironmentVariable
def generate_launch_description():
    os.environ['TURTLEBOT3_MODEL'] = 'burger_cam_gps'
    # 取得 boustrophedon_coverage 套件的 share 目錄
    boustrophedon_coverage_dir = get_package_share_directory('boustrophedon_coverage')
    turtlebot3_gazebo_custom_dir = get_package_share_directory('turtlebot3_gazebo_custom')

    # 模型名稱
    model_name = 'turtlebot3_burger_cam_gps'

    # world 檔案
    world_file = os.path.join(
        turtlebot3_gazebo_custom_dir,
        'worlds',
        'mower_world.world'
    )
    print(world_file)

    # 橋接參數檔
    bridge_yaml = os.path.join(
        turtlebot3_gazebo_custom_dir,
        'params',
        'turtlebot3_burger_cam_gps_bridge.yaml'
    )

    # 載入 Gazebo world
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_custom_dir, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_custom_dir, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': '0.0',
            'y_pose': '0.0'
        }.items()
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # namespace='turtlebot3',
        parameters=[
            {
                'config_file': os.path.join(
                    turtlebot3_gazebo_custom_dir, 'params', 'turtlebot3_burger_cam_gps_bridge.yaml'
                ),
                'expand_gz_topic_names': True,
                'use_sim_time': True,
            }
        ],
        output='screen',
    )

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo_custom'),
                'models'))



    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gz_sim_launch)
    ld.add_action(bridge)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(robot_state_publisher_cmd)
    # ld.add_action(set_env_vars_resources)
    return ld