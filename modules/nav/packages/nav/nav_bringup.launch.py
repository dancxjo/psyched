"""Launch Nav2 + RTAB-Map for 3D mapping using Kinect depth topics.

This launch file is intentionally minimal and uses placeholders for
parameters and namespace. Edit `kinect_rgb_topic`, `kinect_depth_topic`,
and frames as needed for your robot.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    kinect_rgb = LaunchConfiguration('kinect_rgb_topic', default='/camera/color/image_raw')
    kinect_depth = LaunchConfiguration('kinect_depth_topic', default='/camera/depth/image_raw')
    kinect_frame = LaunchConfiguration('camera_frame', default='camera_link')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('kinect_rgb_topic', default_value=kinect_rgb))
    ld.add_action(DeclareLaunchArgument('kinect_depth_topic', default_value=kinect_depth))
    ld.add_action(DeclareLaunchArgument('camera_frame', default_value=kinect_frame))

    # Include Nav2 bringup launch if available on the system
    nav2_share = FindPackageShare('nav2_bringup')
    nav2_launch = PathJoinSubstitution([nav2_share, 'launch', 'bringup_launch.py'])
    nav2_params = PathJoinSubstitution([FindPackageShare('nav'), 'params', 'nav2_params.yaml'])
    # Pass our nav2 params file into the nav2 bringup (bringup_launch.py accepts 'params_file')
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={'params_file': nav2_params}.items()
    ))

    # Start RTAB-Map node with parameter file in this package
    rtabmap_params = PathJoinSubstitution([FindPackageShare('nav'), 'params', 'rtabmap_params.yaml'])
    ld.add_action(Node(
        package='rtabmap_ros', executable='rtabmap', output='screen',
        parameters=[rtabmap_params],
        remappings=[
            ('/rgb/image', LaunchConfiguration('kinect_rgb_topic')),
            ('/depth/image', LaunchConfiguration('kinect_depth_topic')),
        ]
    ))

    # Publish static transform between base_link and camera if not provided by robot
    # Assumes camera is centered: x=0.0, y=0.0, z=0.3 (adjust as necessary)
    ld.add_action(Node(
        package='tf2_ros', executable='static_transform_publisher', output='screen',
        arguments=['0', '0', '0.3', '0', '0', '0', 'base_link', LaunchConfiguration('camera_frame')]
    ))

    return ld
