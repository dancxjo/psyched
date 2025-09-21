from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    frame_id = LaunchConfiguration('frame_id', default='gps_link')
    publish_rate = LaunchConfiguration('publish_rate', default='5.0')
    device = LaunchConfiguration('device', default='/dev/gps0')

    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='gps_link'),
        DeclareLaunchArgument('publish_rate', default_value='5.0'),
        DeclareLaunchArgument('device', default_value='/dev/gps0'),
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps',
            output='screen',
            parameters=[{
                'frame_id': frame_id,
                'publish_rate': publish_rate,
                'device': device,
            }],
            remappings=[
                # Standard topic names per REP-105, REP-145
                ('fix', '/gps/fix'),
                ('time_reference', '/gps/time_reference'),
            ]
        )
    ])
