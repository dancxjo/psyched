from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch description for psyched framework nodes
    """
    return LaunchDescription([
        Node(
            package='psyched',
            executable='psyched_node.py',
            name='psyched_python_node',
            output='screen'
        ),
        Node(
            package='psyched',
            executable='psyched_cpp_node',
            name='psyched_cpp_node',
            output='screen'
        )
    ])