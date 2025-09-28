
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    params_file = os.environ.get('PSH_MODULE_CONFIG', None)
    params = [params_file] if params_file else []
    node = Node(
        package='voice',
        executable='voice_node',
        name='voice',
        output='screen',
        parameters=params
    )
    return LaunchDescription([node])
