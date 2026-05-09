import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    camera_frame = LaunchConfiguration('camera_frame')
    tag_frame = LaunchConfiguration('tag_frame')
    detected_pose_topic = LaunchConfiguration('detected_pose_topic')
    detections_topic = LaunchConfiguration('detections_topic')
    publish_camera_optical_tf = LaunchConfiguration('publish_camera_optical_tf')
    camera_link_frame = LaunchConfiguration('camera_link_frame')
    camera_optical_frame = LaunchConfiguration('camera_optical_frame')

    bringup_dir = get_package_share_directory('mower_bringup')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            bringup_dir,
            'config',
            'apriltag_docking.yaml',
        ),
        description='AprilTag detector parameters file',
    )
    declare_image_topic = DeclareLaunchArgument(
        'image_topic',
        default_value='/back_camera/image_raw',
        description='Rear camera image topic. Gazebo uses zero distortion.',
    )
    declare_camera_info_topic = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/back_camera/camera_info',
        description='Rear camera_info topic matching image timestamps',
    )
    declare_camera_frame = DeclareLaunchArgument(
        'camera_frame',
        default_value='back_camera_link',
        description='Frame used as parent for the detected dock pose',
    )
    declare_tag_frame = DeclareLaunchArgument(
        'tag_frame',
        default_value='home_dock_tag',
        description='AprilTag TF child frame configured in apriltag yaml',
    )
    declare_detected_pose_topic = DeclareLaunchArgument(
        'detected_pose_topic',
        default_value='/detected_dock_pose',
        description='PoseStamped topic consumed by Nav2 docking',
    )
    declare_detections_topic = DeclareLaunchArgument(
        'detections_topic',
        default_value='/rear_apriltag/detections',
        description='AprilTagDetectionArray debug topic',
    )
    declare_publish_camera_optical_tf = DeclareLaunchArgument(
        'publish_camera_optical_tf',
        default_value='false',
        description='Publish a static back_camera_link -> optical frame TF',
    )
    declare_camera_link_frame = DeclareLaunchArgument(
        'camera_link_frame',
        default_value='back_camera_link',
        description='Camera body frame for optional static optical TF',
    )
    declare_camera_optical_frame = DeclareLaunchArgument(
        'camera_optical_frame',
        default_value='back_camera_optical_frame',
        description='Camera optical frame for optional static optical TF',
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('image_rect', image_topic),
            ('camera_info', camera_info_topic),
            ('detections', detections_topic),
        ],
    )

    dock_pose_publisher = Node(
        package='mower_mission',
        executable='apriltag_dock_pose_publisher',
        name='apriltag_dock_pose_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'camera_frame': camera_frame,
            'tag_frame': tag_frame,
            'detected_pose_topic': detected_pose_topic,
        }],
    )

    camera_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='back_camera_optical_tf',
        output='screen',
        condition=IfCondition(publish_camera_optical_tf),
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '-1.57079632679',
            '--pitch', '0',
            '--yaw', '-1.57079632679',
            '--frame-id', camera_link_frame,
            '--child-frame-id', camera_optical_frame,
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_image_topic,
        declare_camera_info_topic,
        declare_camera_frame,
        declare_tag_frame,
        declare_detected_pose_topic,
        declare_detections_topic,
        declare_publish_camera_optical_tf,
        declare_camera_link_frame,
        declare_camera_optical_frame,
        camera_optical_tf,
        apriltag_node,
        dock_pose_publisher,
    ])
