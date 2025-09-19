from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    system_prompt_arg = DeclareLaunchArgument(
        'system_prompt',
        default_value=EnvironmentVariable(name='CHAT_SYSTEM_PROMPT', default_value='You are a helpful assistant. Reply in one concise sentence.'),
        description='System prompt for the chat model'
    )
    conversation_topic_arg = DeclareLaunchArgument('conversation_topic', default_value='/conversation')
    voice_topic_arg = DeclareLaunchArgument('voice_topic', default_value='/voice')
    model_arg = DeclareLaunchArgument('model', default_value=EnvironmentVariable(name='CHAT_MODEL', default_value='gemma3'))
    max_history_arg = DeclareLaunchArgument('max_history', default_value='20')
    ollama_host_arg = DeclareLaunchArgument('ollama_host', default_value=EnvironmentVariable(name='OLLAMA_HOST', default_value='http://localhost:11434'))

    node = Node(
        package='chat',
        executable='chat_node',
        name='chat_node',
        output='screen',
        parameters=[{
            'system_prompt': LaunchConfiguration('system_prompt'),
            'conversation_topic': LaunchConfiguration('conversation_topic'),
            'voice_topic': LaunchConfiguration('voice_topic'),
            'model': LaunchConfiguration('model'),
            'ollama_host': LaunchConfiguration('ollama_host'),
            'max_history': LaunchConfiguration('max_history'),
        }]
    )

    return LaunchDescription([
        system_prompt_arg,
        conversation_topic_arg,
        voice_topic_arg,
        model_arg,
        max_history_arg,
        ollama_host_arg,
        node
    ])
