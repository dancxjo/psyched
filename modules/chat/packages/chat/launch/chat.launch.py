
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get params file from ROS args (passed by launch.sh)
    params_file = os.environ.get('PSH_MODULE_CONFIG', None)
    params = [params_file] if params_file else []
    node = Node(
        package='chat',
        executable='chat_node',
        name='chat_node',
        output='screen',
        parameters=params
    )
    return LaunchDescription([node])
