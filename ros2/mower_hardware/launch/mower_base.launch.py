from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    device = LaunchConfiguration("device")
    urdf = LaunchConfiguration("urdf")

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            urdf, " ", "device:=", device,
        ]),
        value_type=str,
    )
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("mower_hardware"), "config", "mower_controllers.yaml"])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controllers_yaml],
        output="screen",
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )
    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    ddc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        DeclareLaunchArgument("device", default_value="/dev/ttyS0",
                              description="UART device wired to STM32 PB6/PA10"),
        DeclareLaunchArgument(
            "urdf",
            default_value=PathJoinSubstitution(
                [FindPackageShare("mower_hardware"), "urdf", "mower_minimal.urdf.xacro"]),
            description="xacro file that includes mower_base.ros2_control.xacro"),
        control_node,
        rsp,
        jsb,
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[ddc])),
    ])
