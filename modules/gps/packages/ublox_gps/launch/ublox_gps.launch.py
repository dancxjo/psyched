
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    params_file = os.environ.get('PSH_MODULE_CONFIG', None)
    params = [params_file] if params_file else []
    node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps',
        output='screen',
        parameters=params,
        remappings=[
            ('fix', '/gps/fix'),
            ('time_reference', '/gps/time_reference'),
        ]
    )
    return LaunchDescription([node])
