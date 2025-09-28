"""Launch configuration for the chat module."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_ARGUMENTS = {
    "system_prompt": (
        "You are an intelligent robot. You are hearing the user's voice and "
        "responding with your own voice. You are helpful, creative, clever, "
        "and very friendly. Keep your responses brief so that the user has "
        "the chance to speak again. You will be able to continue your "
        "thoughts in the next response."
    ),
    "conversation_topic": "/conversation",
    "voice_topic": "/voice",
    "transcript_topic": "/audio/transcription",
    "model": "gemma3",
    "ollama_host": "http://localhost:11434",
    "max_history": "20",
}


def _declare_launch_arguments():
    return [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in DEFAULT_ARGUMENTS.items()
    ]


def _chat_parameters():
    return {
        name: LaunchConfiguration(name)
        for name in DEFAULT_ARGUMENTS
    }


def generate_launch_description() -> LaunchDescription:
    """Return the launch description for the chat node."""

    node = Node(
        package="chat",
        executable="chat_node",
        name="chat_node",
        output="screen",
        parameters=[_chat_parameters()],
    )
    return LaunchDescription([*_declare_launch_arguments(), node])
